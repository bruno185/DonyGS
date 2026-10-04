
// --- Scanline Z-Buffer ---
// Alternative renderer, triggered by a dedicated key (e.g. Z/z) in the main
// viewer loop. Entirely additive: does not touch processModelFast,
// calculateFaceDepths, or the existing painter's-algorithm pipeline. Consumes
// the same already-computed vtx->x2d/y2d/zo and faces->vertex_indices_buffer.
//
// PIXEL PLOT: was done by MoveTo(x,y)+LineTo(x,y) (QuickDraw II, same point =
// single pixel), replaced with inline 65816
// assembly later (direct SHR nibble read-modify-write), once this scanline
// logic is validated. Marked below with "PLOT PIXEL HERE".
//
// DEPTH PRECISION: this Z-buffer is kept entirely in native float (NOT
// Fixed32) on purpose. Diagnosed with a real case (two perpendicular faces
// of the model, ids 13 and 18, whose observer-space depths interleave within
// less than 1% of each other in their screen overlap region) - a Fixed32
// round-trip for inv_z was not resolving such close depth conflicts
// correctly, causing consistently wrong occlusion between those faces
// regardless of rotation. Float32 has far more usable relative precision at
// this magnitude, so all inv_z math below stays in float from vertex
// computation through to the buffer comparison itself.
//
// Adapt SCREEN_WIDTH/SCREEN_HEIGHT and MAX_SPAN_INTERSECTIONS to your
// actual constants/limits.

typedef struct {
    int x;        // screen-space x of the intersection
    float inv_z;  // interpolated 1/z at that intersection (native float)
} ScanIntersection;

// Extracted verbatim from drawPolygons's fill/frame color logic, so the
// scanline Z-buffer renderer reproduces the exact same user choices
// (default colors, user overrides, orientation shading, palette cycling).

int getFaceFillColor(int face_id) {
    int fill_color;
    if (shaded_by_orientation) {
        fill_color = face_shade_color[face_id];
    } else if (user_fill_color == 16 && random_fill_colors != NULL && face_id < random_colors_capacity) {
        fill_color = random_fill_colors[face_id];
    } else if (user_fill_color >= 0) {
        fill_color = user_fill_color;
    } else {
        fill_color = COL_FILL_DEFAULT;
    }
    return fill_color;
}

int getFaceFrameColor(int face_id, int fill_color) {
    int frame_color;
    if (user_frame_color == 17) {
        frame_color = fill_color;
    } else if (user_frame_color == 16 && random_frame_colors != NULL && face_id < random_colors_capacity) {
        frame_color = random_frame_colors[face_id];
    } else if (user_frame_color >= 0) {
        frame_color = user_frame_color;
    } else {
        frame_color = COL_FRAME;
    }
    return frame_color;
}

void drawPixel(int x, int y, int color)
{
    int offset;
    unsigned char value;

    color &= 0x0F;

    /* Offset (0..31999) into SHR bank $E1 memory - computed in C,
       passed to assembly via the X register (16-bit, plenty of range) */
    offset = y * 160 + (x >> 1);

    asm {
        sep     #0x20        ; 8-bit accumulator

        ldx     offset       ; X = byte offset (index register stays 16-bit)
        lda     0xE12000,x    ; read the byte containing both pixels
                             ; (absolute LONG indexed addressing - bank $E1
                             ; is baked into the instruction itself, no
                             ; pointer variable involved at all)

        pha                  ; save original byte

        lda     x
        and     #0x01
        bne     pixel_odd

        /* x EVEN: pixel = high nibble */
        pla
        and     #0x0F
        sta     value

        lda     color
        asl     a
        asl     a
        asl     a
        asl     a
        and     #0xF0
        ora     value
        sta     0xE12000,x
        bra     done

pixel_odd:
        /* x ODD: pixel = low nibble */
        pla
        and     #0xF0
        sta     value

        lda     color
        and     #0x0F
        ora     value
        sta     0xE12000,x

done:
        rep     #0x20
    }
}

void renderModelScanlineZBuffer_old (Model3D* model) {
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;
    int fcount = faces->face_count;
    int y, f, i;

    // --- 1/z per vertex, recomputed locally once per call (not stored
    // elsewhere; existing pipeline untouched). Perspective-correct depth
    // interpolation needs 1/z, which is linear in screen space - z itself
    // is not. Kept in float (see header comment on precision). ---
    static float* inv_z = NULL;
    static int inv_z_capacity = 0;
    if (inv_z_capacity < vcount) {
        if (inv_z) free(inv_z);
        inv_z = (float*)malloc(vcount * sizeof(float));
        inv_z_capacity = vcount;
    }
    for (i = 0; i < vcount; i++) {
        float zo_f = FIXED_TO_FLOAT(vtx->zo[i]);
        inv_z[i] = (zo_f > 0.0f) ? (1.0f / zo_f) : 0.0f;
    }

    // --- One-scanline Z-buffer, reused every line (320 floats only,
    // not a full-screen buffer) ---
    static float zbuffer_line[SCREEN_WIDTH];

    ScanIntersection hits[MAX_SPAN_INTERSECTIONS];

    SetPenMode(0);
    applyPalette(palette);

    for (y = 0; y < SCREEN_HEIGHT; y++) {
        int screenY = y + pan_dy;
        if (screenY < 0 || screenY >= SCREEN_HEIGHT) continue;

        for (i = 0; i < SCREEN_WIDTH; i++) zbuffer_line[i] = -1.0f; // nothing drawn yet

        for (f = 0; f < fcount; f++) {
            int n, offt, k, hit_count;

            if (!faces->display_flag[f]) continue;
            n = faces->vertex_count[f];
            if (n < 3) continue;
            if (y < faces->miny[f] || y > faces->maxy[f]) continue;

            offt = faces->vertex_indices_ptr[f];
            hit_count = 0;

            // --- Gather all edge/scanline intersections for this face ---
            // Standard scan-conversion rule (y1 <= y < y2, taking edge
            // direction into account) avoids double-counting a vertex that
            // sits exactly on the scanline - this also makes concave faces
            // (e.g. the star) work correctly via pair-wise (even-odd) fill,
            // same principle as the Newell/shoelace robustness discussed
            // earlier for orientation.
            for (k = 0; k < n; k++) {
                int k2 = (k + 1 < n) ? (k + 1) : 0; // avoid modulo (no native op on 65816)
                int vid1 = faces->vertex_indices_buffer[offt + k] - 1;
                int vid2 = faces->vertex_indices_buffer[offt + k2] - 1;

                int y1 = vtx->y2d[vid1];
                int y2 = vtx->y2d[vid2];
                int x1 = vtx->x2d[vid1];
                int x2 = vtx->x2d[vid2];

                int ylo = (y1 < y2) ? y1 : y2;
                int yhi = (y1 < y2) ? y2 : y1;
                if (y < ylo || y >= yhi) continue; // half-open range, skips horizontal edges too

                // linear interpolation fraction along the edge in screen space
                float t = (float)(y - y1) / (float)(y2 - y1);
                int xi = x1 + (int)((x2 - x1) * t + 0.5f);

                // 1/z is linear in screen space along this edge (perspective-
                // correct interpolation) - plain float lerp, no Fixed32
                // round-trip (see header comment).
                float izf = inv_z[vid1] + (inv_z[vid2] - inv_z[vid1]) * t;

                if (hit_count < MAX_SPAN_INTERSECTIONS) {
                    hits[hit_count].x = xi;
                    hits[hit_count].inv_z = izf;
                    hit_count++;
                }
            }

            if (hit_count < 2) continue;

            // --- Sort intersections by x (simple insertion sort - hit_count
            // is small, typically <= number of face vertices) ---
            {
                int a, b;
                for (a = 1; a < hit_count; a++) {
                    ScanIntersection key = hits[a];
                    b = a - 1;
                    while (b >= 0 && hits[b].x > key.x) {
                        hits[b + 1] = hits[b];
                        b--;
                    }
                    hits[b + 1] = key;
                }
            }

            // --- Pair up consecutive intersections (even-odd rule) and
            // fill each span with a Z-test per pixel ---
            {
                int p;
                for (p = 0; p + 1 < hit_count; p += 2) {
                    int xa = hits[p].x;
                    int xb = hits[p + 1].x;
                    float iza = hits[p].inv_z;
                    float izb = hits[p + 1].inv_z;
                    int x;

                    if (xa == xb) continue; // degenerate span

                    // Color depends only on the face (f), never on x/y -
                    // compute once per span instead of once per pixel.
                    {
                        int fillColor = getFaceFillColor(f);
                        int frameColor = getFaceFrameColor(f, fillColor);

                        // iz_here varies linearly across the span - compute
                        // the per-pixel step ONCE (one division), then just
                        // add it each pixel instead of recomputing a full
                        // division + multiplication every pixel. This is
                        // the hottest loop in the function (executed once
                        // per pixel drawn, vs once per span/edge for the
                        // other optimizations), so this is where the real
                        // cost was.
                        float dIz = (izb - iza) / (float)(xb - xa);
                        float iz_here = iza;

                        for (x = xa; x <= xb; x++) {
                            int screenX = x + pan_dx;
                            if (screenX >= 0 && screenX < SCREEN_WIDTH) {
                                if (iz_here > zbuffer_line[x]) {
                                    zbuffer_line[x] = iz_here;

                                    // --- PLOT PIXEL HERE ---
                                    if (x == xa || x == xb) {
                                        drawPixel(screenX, screenY, frameColor);
                                    } else {
                                        drawPixel(screenX, screenY, fillColor);
                                    }
                                }
                            }
                            iz_here += dIz;
                        }
                    }
                }
            }
        }
    }
}

void renderModelScanlineZBuffer_fast(Model3D* model) {
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;
    int fcount = faces->face_count;
    int y, f, i;

    // --- 1/z per vertex (inchangé) ---
    static float* inv_z = NULL;
    static int inv_z_capacity = 0;
    if (inv_z_capacity < vcount) {
        if (inv_z) free(inv_z);
        inv_z = (float*)malloc(vcount * sizeof(float));
        inv_z_capacity = vcount;
    }
    for (i = 0; i < vcount; i++) {
        float zo_f = FIXED_TO_FLOAT(vtx->zo[i]);
        inv_z[i] = (zo_f > 0.0f) ? (1.0f / zo_f) : 0.0f;
    }

    static float zbuffer_line[SCREEN_WIDTH];
    ScanIntersection hits[MAX_SPAN_INTERSECTIONS];

    SetPenMode(0);
    applyPalette(palette);

    // Bounds de clipping en "model space" (constantes pour toute la frame)
    int clip_x_min = -pan_dx;
    int clip_x_max = SCREEN_WIDTH - 1 - pan_dx;

    for (y = 0; y < SCREEN_HEIGHT; y++) {
        int screenY = y + pan_dy;
        if (screenY < 0 || screenY >= SCREEN_HEIGHT) continue;

        // Reset Z-buffer (parcours pointeur, plus facile à optimiser pour le compilateur)
        float* zb_clear = zbuffer_line;
        float* zb_clear_end = zbuffer_line + SCREEN_WIDTH;
        while (zb_clear < zb_clear_end) *zb_clear++ = -1.0f;

        for (f = 0; f < fcount; f++) {
            int n, offt, k, hit_count;

            if (!faces->display_flag[f]) continue;
            n = faces->vertex_count[f];
            if (n < 3) continue;
            if (y < faces->miny[f] || y > faces->maxy[f]) continue;

            // OPTIM 1 : couleurs une seule fois par face
            int fillColor = getFaceFillColor(f);
            int frameColor = getFaceFrameColor(f, fillColor);

            offt = faces->vertex_indices_ptr[f];
            hit_count = 0;

            // --- Intersections edge/scanline ---
            for (k = 0; k < n; k++) {
                int k2 = (k + 1 < n) ? (k + 1) : 0;
                int vid1 = faces->vertex_indices_buffer[offt + k] - 1;
                int vid2 = faces->vertex_indices_buffer[offt + k2] - 1;

                int y1 = vtx->y2d[vid1];
                int y2 = vtx->y2d[vid2];
                int dy = y2 - y1;

                if (dy == 0) continue;                    // edge horizontal
                if (dy > 0) {
                    if (y < y1 || y >= y2) continue;      // half-open [y1, y2)
                } else {
                    if (y < y2 || y >= y1) continue;      // half-open [y2, y1)
                }

                int x1 = vtx->x2d[vid1];
                int x2 = vtx->x2d[vid2];

                float t = (float)(y - y1) / (float)dy;
                int xi = x1 + (int)((x2 - x1) * t + 0.5f);
                float izf = inv_z[vid1] + (inv_z[vid2] - inv_z[vid1]) * t;

                if (hit_count < MAX_SPAN_INTERSECTIONS) {
                    hits[hit_count].x = xi;
                    hits[hit_count].inv_z = izf;
                    hit_count++;
                }
            }

            if (hit_count < 2) continue;

            // OPTIM 2 : tri spécialisé (cas triangle = 2 hits, 90% du temps)
            if (hit_count == 2) {
                if (hits[0].x > hits[1].x) {
                    ScanIntersection tmp = hits[0];
                    hits[0] = hits[1];
                    hits[1] = tmp;
                }
            } else {
                int a, b;
                for (a = 1; a < hit_count; a++) {
                    ScanIntersection key = hits[a];
                    b = a - 1;
                    while (b >= 0 && hits[b].x > key.x) {
                        hits[b + 1] = hits[b];
                        b--;
                    }
                    hits[b + 1] = key;
                }
            }

            // --- Remplissage des spans ---
            int p;
            for (p = 0; p + 1 < hit_count; p += 2) {
                int xa = hits[p].x;
                int xb = hits[p + 1].x;
                float iza = hits[p].inv_z;
                float izb = hits[p + 1].inv_z;

                if (xa > xb) continue;

                // OPTIM 3 : clipping X avant la boucle, pas dedans
                if (xb < clip_x_min || xa > clip_x_max) continue;

                int x0 = xa;
                int x1 = xb;
                float iz0 = iza;

                // --- FIX : cas dégénéré (span d'1 pixel) traité à part,
                // AVANT toute division, pour éviter une division par
                // zéro sur (xb - xa) == 0. Le clipping peut aussi
                // réduire un span normal à 1 pixel après ajustement
                // (voir plus bas), donc ce cas est revérifié après clip. ---
                if (x0 == x1) {
                    int sx0 = x0 + pan_dx;
                    if (iz0 > zbuffer_line[x0]) {
                        zbuffer_line[x0] = iz0;
                        drawPixel(sx0, screenY, frameColor);
                    }
                    continue;
                }

                // dIz calculé seulement si xb != xa (span >= 2 pixels avant clip)
                float dIz = (izb - iza) / (float)(xb - xa);

                if (x0 < clip_x_min) {
                    iz0 += dIz * (clip_x_min - x0);  // pas de division, juste un fmul
                    x0 = clip_x_min;
                }
                if (x1 > clip_x_max) x1 = clip_x_max;
                if (x0 > x1) continue;

                int sx0 = x0 + pan_dx;
                int sx1 = x1 + pan_dx;

                // OPTIM 4 : pas de branche contour/fill dans la boucle interne
                if (x0 == x1) {
                    // Span réduit à 1 pixel par le clipping (pas par les
                    // coordonnées d'origine) - dIz est valide ici puisqu'il
                    // a été calculé avant clip, donc pas de division par 0.
                    if (iz0 > zbuffer_line[x0]) {
                        zbuffer_line[x0] = iz0;
                        drawPixel(sx0, screenY, frameColor);
                    }
                } else {
                    // Premier pixel = contour
                    if (iz0 > zbuffer_line[x0]) {
                        zbuffer_line[x0] = iz0;
                        drawPixel(sx0, screenY, frameColor);
                    }

                    // Milieu = fill (boucle la plus chaude : 0 branche, accès pointeur)
                    float iz = iz0 + dIz;
                    float* zb = &zbuffer_line[x0 + 1];
                    int x;
                    for (x = x0 + 1; x < x1; x++) {
                        if (iz > *zb) {
                            *zb = iz;
                            drawPixel(x + pan_dx, screenY, fillColor);
                        }
                        iz += dIz;
                        zb++;
                    }

                    // Dernier pixel = contour (iz accumulé = profondeur exacte au clip)
                    if (iz > *zb) {
                        *zb = iz;
                        drawPixel(sx1, screenY, frameColor);
                    }
                }
            }
        }
    }
}

/* Render a 3D model using a scanline Z-buffer algorithm with a small bias to reduce Z-fighting.
Slower than the non-biased version (renderModelScanlineZBuffer_fast) due to the additional bias calculations.
Called only when back-face culling is off.
*/
void renderModelScanlineZBuffer_biased(Model3D* model) {
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;
    int fcount = faces->face_count;
    int y, f, i;
    static const float Z_FIGHT_BIAS = 0.001f;  // or 0.0001f;

    // --- 1/z per vertex (inchangé) ---
    static float* inv_z = NULL;
    static int inv_z_capacity = 0;
    if (inv_z_capacity < vcount) {
        if (inv_z) free(inv_z);
        inv_z = (float*)malloc(vcount * sizeof(float));
        inv_z_capacity = vcount;
    }
    for (i = 0; i < vcount; i++) {
        float zo_f = FIXED_TO_FLOAT(vtx->zo[i]);
        inv_z[i] = (zo_f > 0.0f) ? (1.0f / zo_f) : 0.0f;
    }

    static float zbuffer_line[SCREEN_WIDTH];
    ScanIntersection hits[MAX_SPAN_INTERSECTIONS];

    SetPenMode(0);
    applyPalette(palette);

    // Bounds de clipping en "model space" (constantes pour toute la frame)
    int clip_x_min = -pan_dx;
    int clip_x_max = SCREEN_WIDTH - 1 - pan_dx;

    for (y = 0; y < SCREEN_HEIGHT; y++) {
        int screenY = y + pan_dy;
        if (screenY < 0 || screenY >= SCREEN_HEIGHT) continue;

        // Reset Z-buffer (parcours pointeur, plus facile à optimiser pour le compilateur)
        float* zb_clear = zbuffer_line;
        float* zb_clear_end = zbuffer_line + SCREEN_WIDTH;
        while (zb_clear < zb_clear_end) *zb_clear++ = -1.0f;

        for (f = 0; f < fcount; f++) {
            int n, offt, k, hit_count;

            if (!faces->display_flag[f]) continue;
            n = faces->vertex_count[f];
            if (n < 3) continue;
            if (y < faces->miny[f] || y > faces->maxy[f]) continue;

            // OPTIM 1 : couleurs une seule fois par face
            int fillColor = getFaceFillColor(f);
            int frameColor = getFaceFrameColor(f, fillColor);

            // Biais anti Z-fighting - cette fonction n'est appelee que quand
            // cull_back_faces est desactive (voir le dispatcher
            // renderModelScanlineZBuffer), donc pas besoin de re-tester
            // cull_back_faces ici : toujours pertinent d'appliquer le biais
            // sur les faces front.
            float depthBias = (faces->plane_d[f] > 0) ? Z_FIGHT_BIAS : 0.0f;

            offt = faces->vertex_indices_ptr[f];
            hit_count = 0;

            // --- Intersections edge/scanline ---
            for (k = 0; k < n; k++) {
                int k2 = (k + 1 < n) ? (k + 1) : 0;
                int vid1 = faces->vertex_indices_buffer[offt + k] - 1;
                int vid2 = faces->vertex_indices_buffer[offt + k2] - 1;

                int y1 = vtx->y2d[vid1];
                int y2 = vtx->y2d[vid2];
                int dy = y2 - y1;

                if (dy == 0) continue;                    // edge horizontal
                if (dy > 0) {
                    if (y < y1 || y >= y2) continue;      // half-open [y1, y2)
                } else {
                    if (y < y2 || y >= y1) continue;      // half-open [y2, y1)
                }

                int x1 = vtx->x2d[vid1];
                int x2 = vtx->x2d[vid2];

                float t = (float)(y - y1) / (float)dy;
                int xi = x1 + (int)((x2 - x1) * t + 0.5f);
                float izf = inv_z[vid1] + (inv_z[vid2] - inv_z[vid1]) * t;
                izf += depthBias; // biais applique une fois par intersection

                if (hit_count < MAX_SPAN_INTERSECTIONS) {
                    hits[hit_count].x = xi;
                    hits[hit_count].inv_z = izf;
                    hit_count++;
                }
            }

            if (hit_count < 2) continue;

            // OPTIM 2 : tri spécialisé (cas triangle = 2 hits, 90% du temps)
            if (hit_count == 2) {
                if (hits[0].x > hits[1].x) {
                    ScanIntersection tmp = hits[0];
                    hits[0] = hits[1];
                    hits[1] = tmp;
                }
            } else {
                int a, b;
                for (a = 1; a < hit_count; a++) {
                    ScanIntersection key = hits[a];
                    b = a - 1;
                    while (b >= 0 && hits[b].x > key.x) {
                        hits[b + 1] = hits[b];
                        b--;
                    }
                    hits[b + 1] = key;
                }
            }

            // --- Remplissage des spans ---
            int p;
            for (p = 0; p + 1 < hit_count; p += 2) {
                int xa = hits[p].x;
                int xb = hits[p + 1].x;
                float iza = hits[p].inv_z;
                float izb = hits[p + 1].inv_z;

                if (xa > xb) continue;

                // OPTIM 3 : clipping X avant la boucle, pas dedans
                if (xb < clip_x_min || xa > clip_x_max) continue;

                int x0 = xa;
                int x1 = xb;
                float iz0 = iza;

                // --- FIX : cas degenere (span d'1 pixel) traite a part,
                // AVANT toute division, pour eviter une division par zero
                // sur (xb - xa) == 0. ---
                if (x0 == x1) {
                    int sx0 = x0 + pan_dx;
                    if (iz0 > zbuffer_line[x0]) {
                        zbuffer_line[x0] = iz0;
                        drawPixel(sx0, screenY, frameColor);
                    }
                    continue;
                }

                // dIz calcule seulement si xb != xa (span >= 2 pixels avant clip)
                float dIz = (izb - iza) / (float)(xb - xa);

                if (x0 < clip_x_min) {
                    iz0 += dIz * (clip_x_min - x0);  // pas de division, juste un fmul
                    x0 = clip_x_min;
                }
                if (x1 > clip_x_max) x1 = clip_x_max;
                if (x0 > x1) continue;

                int sx0 = x0 + pan_dx;
                int sx1 = x1 + pan_dx;

                // OPTIM 4 : pas de branche contour/fill dans la boucle interne
                if (x0 == x1) {
                    // Span reduit a 1 pixel par le clipping (pas par les
                    // coordonnees d'origine) - dIz est valide ici puisqu'il
                    // a ete calcule avant clip, donc pas de division par 0.
                    if (iz0 > zbuffer_line[x0]) {
                        zbuffer_line[x0] = iz0;
                        drawPixel(sx0, screenY, frameColor);
                    }
                } else {
                    // Premier pixel = contour
                    if (iz0 > zbuffer_line[x0]) {
                        zbuffer_line[x0] = iz0;
                        drawPixel(sx0, screenY, frameColor);
                    }

                    // Milieu = fill (boucle la plus chaude : 0 branche, acces pointeur)
                    float iz = iz0 + dIz;
                    float* zb = &zbuffer_line[x0 + 1];
                    int x;
                    for (x = x0 + 1; x < x1; x++) {
                        if (iz > *zb) {
                            *zb = iz;
                            drawPixel(x + pan_dx, screenY, fillColor);
                        }
                        iz += dIz;
                        zb++;
                    }

                    // Dernier pixel = contour (iz accumule = profondeur exacte au clip)
                    if (iz > *zb) {
                        *zb = iz;
                        drawPixel(sx1, screenY, frameColor);
                    }
                }
            }
        }
    }
}


/* ====================================================================
 * Optimized scanline renderer
 *
 * The original fast/biased renderers above are kept for reference.  This
 * version moves work out of the per-scanline hot path, following the same
 * strategy as the optimized fullscreen Z-buffer renderer:
 *
 *   - 1/z and face colours are computed once per frame;
 *   - polygon edges are prepared once, including dx/dy and d(1/z)/dy;
 *   - a counting sort tells us which faces start on each scanline;
 *   - only active faces are visited on a scanline;
 *   - active edges are advanced by additions, with no division in the
 *     scanline/edge loop;
 *   - horizontal clipping is performed before the pixel loop;
 *   - the one-line float Z-buffer is retained, so this renderer keeps the
 *     depth representation and comparison rule of the original scanline code.
 * ==================================================================== */

typedef struct {
    float x;              /* Current x intersection. */
    float inv_z;          /* Current reciprocal depth. */
    float x_step;         /* dx / dy. */
    float inv_z_step;     /* d(1/z) / dy. */
    int ya, yb;           /* Active for ya <= y < yb. */
} ScanEdgeFast;

static void renderModelScanlineZBuffer_coreV2(Model3D* model, int biased)
{
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;
    int fcount = faces->face_count;
    int y, f, i, k, b, si, ai, keep, active_count;
    int y_lo, y_hi, clip_x_min, clip_x_max;
    int total_edges;
    static const float Z_FIGHT_BIAS = 0.001f;

    static float* inv_z = NULL;
    static int vert_capacity = 0;
    static int* face_start = NULL;
    static int* face_sorted = NULL;
    static int* face_active = NULL;
    static unsigned char* face_fill = NULL;
    static unsigned char* face_frame = NULL;
    static float* face_bias = NULL;
    static int face_capacity = 0;
    static ScanEdgeFast* edge_buf = NULL;
    static int edge_capacity = 0;
    static int bucket[SCREEN_HEIGHT + 1];
    static int bucket_cur[SCREEN_HEIGHT + 1];
    static float zbuffer_line[SCREEN_WIDTH];
    ScanIntersection hits[MAX_SPAN_INTERSECTIONS];

    if (vcount <= 0 || fcount <= 0) return;

    if (vert_capacity < vcount) {
        if (inv_z) free(inv_z);
        inv_z = (float*)malloc(vcount * sizeof(float));
        if (!inv_z) { vert_capacity = 0; return; }
        vert_capacity = vcount;
    }

    if (face_capacity < fcount) {
        if (face_start)  free(face_start);
        if (face_sorted) free(face_sorted);
        if (face_active) free(face_active);
        if (face_fill)   free(face_fill);
        if (face_frame)  free(face_frame);
        if (face_bias)   free(face_bias);
        face_start  = (int*)malloc(fcount * sizeof(int));
        face_sorted = (int*)malloc(fcount * sizeof(int));
        face_active = (int*)malloc(fcount * sizeof(int));
        face_fill   = (unsigned char*)malloc(fcount);
        face_frame  = (unsigned char*)malloc(fcount);
        face_bias   = (float*)malloc(fcount * sizeof(float));
        if (!face_start || !face_sorted || !face_active || !face_fill ||
            !face_frame || !face_bias) {
            face_capacity = 0;
            return;
        }
        face_capacity = fcount;
    }

    total_edges = 0;
    for (f = 0; f < fcount; f++) total_edges += faces->vertex_count[f];
    if (edge_capacity < total_edges) {
        if (edge_buf) free(edge_buf);
        edge_buf = (ScanEdgeFast*)malloc(total_edges * sizeof(ScanEdgeFast));
        if (!edge_buf) { edge_capacity = 0; return; }
        edge_capacity = total_edges;
    }

    /* Reciprocal depth is still float, exactly as in the original renderer. */
    for (i = 0; i < vcount; i++) {
        float zo_f = FIXED_TO_FLOAT(vtx->zo[i]);
        inv_z[i] = (zo_f > 0.0f) ? (1.0f / zo_f) : 0.0f;
    }

    clip_x_min = -pan_dx;
    clip_x_max = SCREEN_WIDTH - 1 - pan_dx;
    y_lo = -pan_dy;
    if (y_lo < 0) y_lo = 0;
    y_hi = SCREEN_HEIGHT - 1 - pan_dy;
    if (y_hi >= SCREEN_HEIGHT) y_hi = SCREEN_HEIGHT - 1;
    if (y_lo > y_hi) return;

    for (b = 0; b <= SCREEN_HEIGHT; b++) bucket[b] = 0;

    /* Prepare faces and edges once for the whole frame. */
    for (f = 0; f < fcount; f++) {
        int n = faces->vertex_count[f];
        int offt = faces->vertex_indices_ptr[f];
        int first_y, last_y;
        int fill;

        face_start[f] = -1;
        if (!faces->display_flag[f] || n < 3) continue;

        first_y = faces->miny[f];
        last_y  = faces->maxy[f];
        if (first_y < y_lo) first_y = y_lo;
        if (last_y > y_hi) last_y = y_hi;
        if (first_y > last_y) continue;

        face_start[f] = first_y - y_lo;
        bucket[face_start[f] + 1]++;
        fill = getFaceFillColor(f);
        face_fill[f] = (unsigned char)fill;
        face_frame[f] = (unsigned char)getFaceFrameColor(f, fill);
        face_bias[f] = (biased && faces->plane_d[f] > 0) ? Z_FIGHT_BIAS : 0.0f;

        for (k = 0; k < n; k++) {
            int k2 = (k + 1 < n) ? (k + 1) : 0;
            int vid1 = faces->vertex_indices_buffer[offt + k] - 1;
            int vid2 = faces->vertex_indices_buffer[offt + k2] - 1;
            int y1 = vtx->y2d[vid1];
            int y2 = vtx->y2d[vid2];
            int dy = y2 - y1;
            ScanEdgeFast* ed = &edge_buf[offt + k];

            if (dy == 0) {
                ed->ya = 32767;
                ed->yb = 32767;
                ed->x = ed->inv_z = ed->x_step = ed->inv_z_step = 0.0f;
                continue;
            }

            ed->x_step = (float)(vtx->x2d[vid2] - vtx->x2d[vid1]) / (float)dy;
            ed->inv_z_step = (inv_z[vid2] - inv_z[vid1]) / (float)dy;

            if (dy > 0) {
                ed->ya = y1;
                ed->yb = y2;
                ed->x = (float)vtx->x2d[vid1];
                ed->inv_z = inv_z[vid1];
            } else {
                ed->ya = y2;
                ed->yb = y1;
                ed->x = (float)vtx->x2d[vid2];
                ed->inv_z = inv_z[vid2];
            }

            /* Jump directly to the first scanline that can actually be visited. */
            if (ed->ya < y_lo && ed->yb > y_lo) {
                float skip = (float)(y_lo - ed->ya);
                ed->x += ed->x_step * skip;
                ed->inv_z += ed->inv_z_step * skip;
            }
        }
    }

    for (b = 0; b < SCREEN_HEIGHT; b++) bucket[b + 1] += bucket[b];
    for (b = 0; b <= SCREEN_HEIGHT; b++) bucket_cur[b] = bucket[b];
    for (f = 0; f < fcount; f++) {
        b = face_start[f];
        if (b >= 0) face_sorted[bucket_cur[b]++] = f;
    }

    SetPenMode(0);
    applyPalette(palette);

    active_count = 0;
    for (y = y_lo; y <= y_hi; y++) {
        int screenY = y + pan_dy;
        int row = y - y_lo;
        float* zp;
        float* zend;

        /* Add newly visible faces while retaining ascending face order. */
        for (si = bucket[row]; si < bucket[row + 1]; si++) {
            int nf = face_sorted[si];
            int pos = active_count;
            while (pos > 0 && face_active[pos - 1] > nf) {
                face_active[pos] = face_active[pos - 1];
                pos--;
            }
            face_active[pos] = nf;
            active_count++;
        }

        zp = zbuffer_line;
        zend = zbuffer_line + SCREEN_WIDTH;
        while (zp < zend) *zp++ = -1.0f;

        keep = 0;
        for (ai = 0; ai < active_count; ai++) {
            int n, offt, hit_count;
            int fillColor, frameColor;
            float depthBias;
            ScanEdgeFast* ed;

            f = face_active[ai];
            if (faces->maxy[f] < y) continue;
            face_active[keep++] = f;

            n = faces->vertex_count[f];
            offt = faces->vertex_indices_ptr[f];
            hit_count = 0;
            depthBias = face_bias[f];
            ed = edge_buf + offt;

            for (k = 0; k < n; k++, ed++) {
                if (y < ed->ya || y >= ed->yb) continue;
                if (hit_count < MAX_SPAN_INTERSECTIONS) {
                    hits[hit_count].x = (int)(ed->x + 0.5f);
                    hits[hit_count].inv_z = ed->inv_z + depthBias;
                    hit_count++;
                }
                ed->x += ed->x_step;
                ed->inv_z += ed->inv_z_step;
            }

            if (hit_count < 2) continue;
            if (hit_count == 2) {
                if (hits[0].x > hits[1].x) {
                    ScanIntersection tmp = hits[0];
                    hits[0] = hits[1];
                    hits[1] = tmp;
                }
            } else {
                int a, c;
                for (a = 1; a < hit_count; a++) {
                    ScanIntersection key = hits[a];
                    c = a - 1;
                    while (c >= 0 && hits[c].x > key.x) {
                        hits[c + 1] = hits[c];
                        c--;
                    }
                    hits[c + 1] = key;
                }
            }

            fillColor = face_fill[f];
            frameColor = face_frame[f];

            {
                int p;
                for (p = 0; p + 1 < hit_count; p += 2) {
                    int xa = hits[p].x;
                    int xb = hits[p + 1].x;
                    float iza = hits[p].inv_z;
                    float izb = hits[p + 1].inv_z;
                    int x0, x1, x;
                    float dIz, iz0, iz;
                    float* zb;

                    if (xa > xb) continue;
                    if (xb < clip_x_min || xa > clip_x_max) continue;

                    x0 = xa;
                    x1 = xb;
                    iz0 = iza;

                    if (x0 == x1) {
                        int sx = x0 + pan_dx;
                        if (iz0 > zbuffer_line[x0]) {
                            zbuffer_line[x0] = iz0;
                            drawPixel(sx, screenY, frameColor);
                        }
                        continue;
                    }

                    dIz = (izb - iza) / (float)(xb - xa);
                    if (x0 < clip_x_min) {
                        iz0 += dIz * (float)(clip_x_min - x0);
                        x0 = clip_x_min;
                    }
                    if (x1 > clip_x_max) x1 = clip_x_max;
                    if (x0 > x1) continue;

                    if (x0 == x1) {
                        if (iz0 > zbuffer_line[x0]) {
                            zbuffer_line[x0] = iz0;
                            drawPixel(x0 + pan_dx, screenY, frameColor);
                        }
                        continue;
                    }

                    if (iz0 > zbuffer_line[x0]) {
                        zbuffer_line[x0] = iz0;
                        drawPixel(x0 + pan_dx, screenY, frameColor);
                    }

                    iz = iz0 + dIz;
                    zb = &zbuffer_line[x0 + 1];
                    for (x = x0 + 1; x < x1; x++, zb++) {
                        if (iz > *zb) {
                            *zb = iz;
                            drawPixel(x + pan_dx, screenY, fillColor);
                        }
                        iz += dIz;
                    }

                    if (iz > *zb) {
                        *zb = iz;
                        drawPixel(x1 + pan_dx, screenY, frameColor);
                    }
                }
            }
        }
        active_count = keep;
    }
}



/* ====================================================================
 * V3: integer hot path for the one-scanline Z-buffer.
 *
 * Unlike V2, no floating-point operation and no C drawPixel() call occurs
 * for each covered pixel.  Reciprocal depth is quantized once per vertex;
 * edges are walked in 12.12 fixed point; a complete span is depth-tested and
 * painted by one 65816 routine.  The Z-buffer remains only one 320-pixel row.
 * ==================================================================== */
typedef unsigned int SLZWord;
#define SL_Z_FRAC 12
#define SL_Z_ROUND 0x800L
#define SL_Z_BIAS 0.0005f
#define SL_Z_FAR 0xFFFF
#define SL_SHR_BASE 0xE12000UL
#define SL_TARGET_MAX_CODE 65000.0f

static SLZWord sl_zline[SCREEN_WIDTH];
static float sl_zscale = 1.0f;


/* Clear the reusable 320-word scanline Z-buffer in native 65816 code. */
static void SL_ClearLine(void)
{
    asm {
        php
        rep #0x30
        ldx #0
        lda #0xFFFF
slcl_loop:
        sta >sl_zline,x
        inx
        inx
        cpx #SCREEN_WIDTH*2
        bne slcl_loop
        plp
    }
}

typedef struct { int x; SLZWord zq; } SLZHit;
typedef struct { long x,z,sx,sz; int ya,yb,xb; } SLZEdge;

static void SL_SetScale(float max_inv_z)
{
    if (max_inv_z > 0.0000001f) sl_zscale = SL_TARGET_MAX_CODE / max_inv_z;
}

static SLZWord SL_Quantize(float inv_z)
{
    float scaled;
    long q;
    if (inv_z < 0.0f) inv_z = 0.0f;
    scaled = inv_z * sl_zscale;
    q = (long)(scaled + 0.5f);
    if (q < 0) q = 0;
    if (q > 0xFFFFL) q = 0xFFFFL;
    return (SLZWord)(0xFFFFUL - (unsigned long)q);
}

static SLZWord SL_LerpZ(SLZWord za, SLZWord zb, int xa, int xb, int x)
{
    long num;
    if (xb == xa) return za;
    num = ((long)zb - (long)za) * (long)(x - xa);
    return (SLZWord)((long)za + num / (long)(xb - xa));
}

long sl_p_zacc, sl_p_zstep;
unsigned long sl_p_zptr, sl_p_pptr;
int sl_p_n, sl_p_odd, sl_p_mode, sl_p_cl, sl_p_ci, sl_p_cr;
unsigned int sl_p_zl, sl_p_zr;

static void SL_SpanRun(void)
{
    asm {
        /* Allocate a 32-byte direct-page frame on the stack and point D at it */
        /* (the C direct page is saved by phd and restored by pld at the end). */
        php
        phd
        rep #0x30
        tsc
        sec
        sbc #32
        tcs
        inc a
        tcd

        /* Load the parameters into the frame (see the layout above). */
        lda >sl_p_zacc
        sta 0
        lda >sl_p_zacc+2
        sta 2
        lda >sl_p_zstep
        sta 4
        lda >sl_p_zstep+2
        sta 6
        lda >sl_p_zptr
        sta 8
        lda >sl_p_zptr+2
        sta 10
        lda >sl_p_pptr
        sta 12
        lda >sl_p_pptr+2
        sta 14
        lda >sl_p_n
        sta 16
        /* Interior colour -> lo = 0x0N (low nibble) and hi = 0xN0 (high nibble). */
        lda >sl_p_ci
        and #0x000F
        sta 18
        asl a
        asl a
        asl a
        asl a
        sta 20
        /* Parity of the first interior pixel = parity of the left pixel xor 1. */
        lda >sl_p_odd
        eor #1
        sta 22
        /* Y = byte offset of the current pixel in the Z row (2 per pixel). */
        /*  */
        /* ---- LEFT PIXEL: exact zq (zl), colour cl ---- */
        /* A = parity -> mask (24) of the nibble to keep, value (26) to OR in. */
        ldy #0

        lda >sl_p_odd
        beq zbr_l_ev
        lda #0x00F0
        sta 24
        lda >sl_p_cl
        and #0x000F
        sta 26
        bra zbr_l_go
    zbr_l_ev:
        lda #0x000F
        sta 24
        lda >sl_p_cl
        and #0x000F
        asl a
        asl a
        asl a
        asl a
        sta 26
    zbr_l_go:
        /* Depth test: draw only if zq is STRICTLY smaller (closer) than the stored Z. */
        lda >sl_p_zl
        cmp [8],y
        bcs zbr_lsk
        sta [8],y
        sep #0x20
        lda [12]
        and 24
        ora 26
        sta [12]
        rep #0x20
    zbr_lsk:
        /* mode 0: left pixel only, we are done. */
        lda >sl_p_mode
        bne zbr_full
        brl zbr_done
        /* Step over the left pixel: Z index += 2, and the pixel pointer moves to the */
        /* next byte after an odd pixel (low nibble). */
    zbr_full:
        iny
        iny
        lda >sl_p_odd
        beq zbr_lev
        inc 12
    zbr_lev:

        /* ---- INTERIOR PIXELS (n may be 0) ---- */
        lda 16
        bne zbr_int
        brl zbr_right
        /* If the first interior pixel is odd, do it alone so that the rest runs as */
        /* (even, odd) pairs, one byte per pair. */
    zbr_int:
        lda 22
        beq zbp_even

        clc
        lda 0
        adc 4
        sta 0
        lda 2
        adc 6
        sta 2
        cmp [8],y
        bcs zbp_o1
        sta [8],y
        sep #0x20
        lda [12]
        and #0xF0
        ora 18
        sta [12]
        rep #0x20
    zbp_o1:
        iny
        iny
        inc 12
        dec 16

        /* Pairs: X = n / 2.  Each pass does an even pixel (high nibble) then an odd */
        /* pixel (low nibble) and then moves the pixel pointer to the next byte. */
        /* z += step is a 32-bit add; the high word is the pixel's zq. */
    zbp_even:
        lda 16
        lsr a
        tax
        beq zbp_tail

    zbp_pair:
        clc
        lda 0
        adc 4
        sta 0
        lda 2
        adc 6
        sta 2
        cmp [8],y
        bcs zbp_e1
        sta [8],y
        sep #0x20
        lda [12]
        and #0x0F
        ora 20
        sta [12]
        rep #0x20
    zbp_e1:
        iny
        iny
        clc
        lda 0
        adc 4
        sta 0
        lda 2
        adc 6
        sta 2
        cmp [8],y
        bcs zbp_o2
        sta [8],y
        sep #0x20
        lda [12]
        and #0xF0
        ora 18
        sta [12]
        rep #0x20
    zbp_o2:
        iny
        iny
        inc 12
        dex
        bne zbp_pair

        /* Tail: if n is odd, one last even pixel remains. */
    zbp_tail:
        lda 16
        and #1
        bne zbp_t1
        brl zbr_right
    zbp_t1:

        clc
        lda 0
        adc 4
        sta 0
        lda 2
        adc 6
        sta 2
        cmp [8],y
        bcs zbp_e2
        sta [8],y
        sep #0x20
        lda [12]
        and #0x0F
        ora 20
        sta [12]
        rep #0x20
    zbp_e2:
        iny
        iny

        /* ---- RIGHT PIXEL: exact zq (zr), colour cr ---- */
        /* Parity = (parity of first interior pixel + n) & 1. */
    zbr_right:
        lda >sl_p_n
        clc
        adc 22
        and #1
        beq zbr_r_ev
        lda #0x00F0
        sta 24
        lda >sl_p_cr
        and #0x000F
        sta 26
        bra zbr_r_go
    zbr_r_ev:
        lda #0x000F
        sta 24
        lda >sl_p_cr
        and #0x000F
        asl a
        asl a
        asl a
        asl a
        sta 26
    zbr_r_go:
        lda >sl_p_zr
        cmp [8],y
        bcs zbr_rsk
        sta [8],y
        sep #0x20
        lda [12]
        and 24
        ora 26
        sta [12]
        rep #0x20
    zbr_rsk:

        /* Free the frame and restore D and the processor flags. */
    zbr_done:
        rep #0x30
        tsc
        clc
        adc #32
        tcs
        pld
        plp
    }
}



static void SL_PaintSpan(int x0, int x1, int screenY,
                         SLZWord z0, SLZWord z1,
                         int fillColor, int frameColor, int nudge)
{
    int sx0, nsteps;
    long zstep, zacc;
    if (x0 > x1) return;
    if (nudge) {
        z0 = (z0 > (SLZWord)nudge) ? (SLZWord)(z0-nudge) : 0;
        z1 = (z1 > (SLZWord)nudge) ? (SLZWord)(z1-nudge) : 0;
    }
    sx0 = x0 + pan_dx;
    sl_p_zptr = (unsigned long)(sl_zline + sx0);
    sl_p_pptr = SL_SHR_BASE + (unsigned long)screenY * 160UL + (unsigned long)(sx0 >> 1);
    sl_p_odd = sx0 & 1;
    sl_p_zl = z0;
    sl_p_cl = frameColor;
    if (x0 == x1) {
        sl_p_mode = 0;
        SL_SpanRun();
        return;
    }
    nsteps = x1-x0;
    zstep = (((long)z1-(long)z0) << 16) / nsteps;
    zacc = ((long)z0 << 16) + 0x8000L;
    sl_p_zr = z1;
    sl_p_zacc = zacc;
    sl_p_zstep = zstep;
    sl_p_n = nsteps-1;
    sl_p_ci = fillColor;
    sl_p_cr = frameColor;
    sl_p_mode = 1;
    SL_SpanRun();
}

static void renderModelScanlineZBuffer_coreV3(Model3D* model, int biased)
{
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;
    int fcount = faces->face_count;
    int y, f, i, k, b, si, ai, keep, active_count;
    int total_edges, end;
    int clip_x_min, clip_x_max, y_lo, y_hi;
    float frame_max_inv_z;
    SLZEdge* ed;
    SLZHit hits[MAX_SPAN_INTERSECTIONS];

    static float* inv_z = NULL;
    static SLZWord* vzq = NULL;
    static SLZWord* vzq_b = NULL;
    static int vert_capacity = 0;

    static int* face_start = NULL;          /* first scanline of the face (relative to y_lo), -1 = skipped */
    static int* face_sorted = NULL;         /* faces sorted by first scanline (counting sort)               */
    static int* face_active = NULL;         /* faces crossing the current scanline, ascending face index    */
    static unsigned char* face_fill = NULL;
    static unsigned char* face_frame = NULL;
    static unsigned char* face_nudge = NULL;
    static int face_capacity = 0;

    static SLZEdge* edge_buf = NULL;
    static int edge_capacity = 0;

    /* counting-sort tables: bucket[r] .. bucket[r+1]-1 = faces starting at row r */
    static int bucket[SCREEN_HEIGHT + 1];
    static int bucket_cur[SCREEN_HEIGHT + 1];

    /* ---- scratch buffers: they only ever grow, so after the first frame there is no malloc ---- */
    if (face_capacity < fcount) {
        if (face_start)   free(face_start);
        if (face_sorted)  free(face_sorted);
        if (face_active)  free(face_active);
        if (face_fill)    free(face_fill);
        if (face_frame)   free(face_frame);
        if (face_nudge)   free(face_nudge);
        face_start  = (int*)malloc(fcount * sizeof(int));
        face_sorted = (int*)malloc(fcount * sizeof(int));
        face_active = (int*)malloc(fcount * sizeof(int));
        face_fill   = (unsigned char*)malloc(fcount);
        face_frame  = (unsigned char*)malloc(fcount);
        face_nudge  = (unsigned char*)malloc(fcount);
        face_capacity = (face_start && face_sorted && face_active &&
                         face_fill && face_frame && face_nudge) ? fcount : 0;
    }
    if (vert_capacity < vcount) {
        if (inv_z) free(inv_z);
        if (vzq)   free(vzq);
        if (vzq_b) free(vzq_b);
        inv_z = (float*)malloc(vcount * sizeof(float));
        vzq   = (SLZWord*)malloc(vcount * sizeof(SLZWord));
        vzq_b = (SLZWord*)malloc(vcount * sizeof(SLZWord));
        vert_capacity = (inv_z && vzq && vzq_b) ? vcount : 0;
    }
    total_edges = 0;
    for (f = 0; f < fcount; f++) {
        end = faces->vertex_indices_ptr[f] + faces->vertex_count[f];
        if (end > total_edges) total_edges = end;
    }
    if (edge_capacity < total_edges) {
        if (edge_buf) free(edge_buf);
        edge_buf = (SLZEdge*)malloc((size_t)total_edges * sizeof(SLZEdge));
        edge_capacity = edge_buf ? total_edges : 0;
    }
    if (face_capacity < fcount || vert_capacity < vcount || edge_capacity < total_edges)
        return;

    /* ---- PER VERTEX, once per frame -------------------------------------
     * inv_z = 1/z is linear in screen space.  The scale that maps inv_z to the
     * 16-bit range is chosen from the largest inv_z of this frame, then every
     * vertex is quantised ONCE (zq: smaller = closer).  Everything after this
     * point is integer: edges and spans only interpolate zq. */
    frame_max_inv_z = 0.0f;
    for (i = 0; i < vcount; i++) {
        float zo_f = FIXED_TO_FLOAT(vtx->zo[i]);
        inv_z[i] = (zo_f > 0.0f) ? (1.0f / zo_f) : 0.0f;
        if (inv_z[i] > frame_max_inv_z) frame_max_inv_z = inv_z[i];
    }
    SL_SetScale(frame_max_inv_z);

    for (i = 0; i < vcount; i++) {
        /* vzq_b is the "pulled closer" copy used by faces with plane_d > 0 (biased only). */
        vzq[i]   = SL_Quantize(inv_z[i]);
        vzq_b[i] = biased ? SL_Quantize(inv_z[i] + SL_Z_BIAS) : vzq[i];
    }

    SetPenMode(0);
    applyPalette(palette);

    /* Visible window in model coordinates: the pan offsets (pan_dx, pan_dy) are
     * added back when addressing the screen and the Z-buffer. */
    clip_x_min = -pan_dx;
    clip_x_max = SCREEN_WIDTH - 1 - pan_dx;
    y_lo = -pan_dy;
    y_hi = SCREEN_HEIGHT - 1 - pan_dy;


    /* ---- PER FACE, once per frame -----------------------------------------
     * - decide if the face can appear at all (shown, >= 3 vertices, overlaps the window)
     * - cache its colours and depth nudge
     * - set up one SLZEdge per polygon edge (see the SLZEdge comment) */
    for (b = 0; b <= SCREEN_HEIGHT; b++) bucket[b] = 0;

    for (f = 0; f < fcount; f++) {
        int n, offt, fmin, fmax, fill;
        SLZWord* zsrc;

        face_start[f] = -1;
        if (!faces->display_flag[f]) continue;
        n = faces->vertex_count[f];
        if (n < 3) continue;
        fmin = faces->miny[f];
        fmax = faces->maxy[f];
        if (fmax < y_lo || fmin > y_hi) continue;

        /* First scanline we will actually visit: faces that start above the
         * window are activated on its first row. */
        b = ((fmin > y_lo) ? fmin : y_lo) - y_lo;
        face_start[f] = b;
        bucket[b + 1]++;       /* histogram of start rows (shifted by one for the prefix sum) */

        fill = getFaceFillColor(f);
        face_fill[f]  = (unsigned char)fill;
        face_frame[f] = (unsigned char)getFaceFrameColor(f, fill);
        face_nudge[f] = (unsigned char)((biased && faces->plane_d[f] > 0) ? 2 : 0);

        offt = faces->vertex_indices_ptr[f];
        zsrc = (biased && faces->plane_d[f] > 0) ? vzq_b : vzq;
        ed = edge_buf + offt;

        for (k = 0; k < n; k++, ed++) {
            int k2 = (k + 1 < n) ? (k + 1) : 0;
            int vid1 = faces->vertex_indices_buffer[offt + k] - 1;
            int vid2 = faces->vertex_indices_buffer[offt + k2] - 1;
            int y1 = vtx->y2d[vid1];
            int y2 = vtx->y2d[vid2];
            int dy = y2 - y1;
            int ya, yb, xs;
            long zs, ex, ez, sx, sz, skip;

            if (dy == 0) {
                ed->ya = 32767; ed->yb = 32767;      /* horizontal edge: never active */
                ed->x = 0L; ed->z = 0L; ed->sx = 0L; ed->sz = 0L;
                continue;
            }

            /* Slopes per scanline in 12-bit fixed point (dx/dy and dzq/dy). */
            sx = (((long)vtx->x2d[vid2] - (long)vtx->x2d[vid1]) * 4096L) / (long)dy;
            sz = (((long)zsrc[vid2] - (long)zsrc[vid1]) * 4096L) / (long)dy;

            /* The edge is walked from its upper end downwards.  x is stored as an
             * offset from xb (= x of vertex 1), so for dy < 0 the starting offset
             * is x(v2) - x(v1) and the slopes stay valid (dx/dy changes sign with dy). */
            if (dy > 0) { ya = y1; yb = y2; xs = 0;                                        zs = (long)zsrc[vid1]; }
            else        { ya = y2; yb = y1; xs = vtx->x2d[vid2] - vtx->x2d[vid1];          zs = (long)zsrc[vid2]; }

            ex = (long)xs * 4096L;
            ez = zs * 4096L;

            /* Edge starts above the visible window: jump straight to the first visible row. */
            if (ya < y_lo && yb > y_lo) {
                skip = (long)(y_lo - ya);
                ex += sx * skip;
                ez += sz * skip;
            }

            ed->ya = ya;  ed->yb = yb;
            ed->xb = vtx->x2d[vid1];
            ed->x  = ex + SL_Z_ROUND;     /* the +0.5 rounding is baked in once, here */
            ed->z  = ez + SL_Z_ROUND;
            ed->sx = sx;  ed->sz = sz;
        }
    }

    /* ---- COUNTING SORT: faces ordered by their first visible scanline ----
     * (prefix sums turn the histogram into start offsets, then each face is placed). */
    for (b = 0; b < SCREEN_HEIGHT; b++) bucket[b + 1] += bucket[b];
    for (b = 0; b <= SCREEN_HEIGHT; b++) bucket_cur[b] = bucket[b];
    for (f = 0; f < fcount; f++) {
        b = face_start[f];
        if (b >= 0) face_sorted[bucket_cur[b]++] = f;
    }

    /* ---- MAIN LOOP: one scanline at a time ---------------------------------
     * Only faces crossing the scanline are visited (active list), instead of
     * testing every face on every row. */
    active_count = 0;
    for (y = y_lo; y <= y_hi; y++) {
        int screenY = y + pan_dy;
        int row = y - y_lo;
        /* One-line Z-buffer: clear only the row reused for this scanline. */
        SL_ClearLine();

        /* Faces that start on this row join the active list.  The list is kept in
         * ascending face index (insertion sort) so that depth ties are resolved in the
         * same order as a plain loop over all faces. */
        for (si = bucket[row]; si < bucket[row + 1]; si++) {
            int nf = face_sorted[si];
            int pos = active_count;
            while (pos > 0 && face_active[pos - 1] > nf) {
                face_active[pos] = face_active[pos - 1];
                pos--;
            }
            face_active[pos] = nf;
            active_count++;
        }

        keep = 0;
        for (ai = 0; ai < active_count; ai++) {
            int n, offt, hit_count;
            int fillColor, frameColor, nudge;

            f = face_active[ai];
            if (faces->maxy[f] < y) continue;      /* face finished: dropped from the list */
            face_active[keep++] = f;

            n = faces->vertex_count[f];
            offt = faces->vertex_indices_ptr[f];
            hit_count = 0;

            /* Intersect the scanline with the polygon: every edge active on row y gives
             * one hit (x, zq); the edge state is then advanced by one row. */
            ed = edge_buf + offt;
            for (k = 0; k < n; k++, ed++) {
                if (y < ed->ya || y >= ed->yb) continue;
                if (hit_count < MAX_SPAN_INTERSECTIONS) {
                    /* Fixed-point -> pixel: truncate toward zero (like the (int)(v + 0.5f)
                     * of the original float code), added to the edge's base x. */
                    long t = ed->x;
                    hits[hit_count].x  = ed->xb + ((t >= 0L) ? (int)(t >> SL_Z_FRAC) : -(int)((-t) >> SL_Z_FRAC));
                    hits[hit_count].zq = (SLZWord)(ed->z >> SL_Z_FRAC);
                    hit_count++;
                }
                /* advance even when the hit table is full, to keep the edge in step */
                ed->x += ed->sx;
                ed->z += ed->sz;
            }

            if (hit_count < 2) continue;

            /* Sort the hits by x (a swap for the usual 2 hits, insertion sort otherwise). */
            if (hit_count == 2) {
                if (hits[0].x > hits[1].x) {
                    SLZHit tmp = hits[0];
                    hits[0] = hits[1];
                    hits[1] = tmp;
                }
            } else {
                int a, c;
                for (a = 1; a < hit_count; a++) {
                    SLZHit key = hits[a];
                    c = a - 1;
                    while (c >= 0 && hits[c].x > key.x) {
                        hits[c + 1] = hits[c];
                        c--;
                    }
                    hits[c + 1] = key;
                }
            }

            fillColor  = face_fill[f];
            frameColor = face_frame[f];
            nudge      = face_nudge[f];

            /* Consecutive hit pairs are the inside spans of the polygon (even-odd rule).
             * Clip each span to the window; zq at a clipped end is interpolated. */
            {
                int p;
                for (p = 0; p + 1 < hit_count; p += 2) {
                    int xa = hits[p].x;
                    int xb = hits[p + 1].x;
                    SLZWord zqa = hits[p].zq;
                    SLZWord zqb = hits[p + 1].zq;
                    int x0, x1;
                    SLZWord zq0, zq1;

                    if (xa > xb) continue;
                    if (xb < clip_x_min || xa > clip_x_max) continue;

                    x0 = xa;  x1 = xb;
                    zq0 = zqa; zq1 = zqb;

                    if (x0 < clip_x_min) { zq0 = SL_LerpZ(zqa, zqb, xa, xb, clip_x_min); x0 = clip_x_min; }
                    if (x1 > clip_x_max) { zq1 = SL_LerpZ(zqa, zqb, xa, xb, clip_x_max); x1 = clip_x_max; }
                    if (x0 > x1) continue;

                    SL_PaintSpan(x0, x1, screenY, zq0, zq1,
                                 fillColor, frameColor, nudge);
                }
            }
        }
        active_count = keep;
    }
}



void renderModelScanlineZBuffer_fastV3(Model3D* model)
{
    renderModelScanlineZBuffer_coreV3(model, 0);
}

void renderModelScanlineZBuffer_biasedV3(Model3D* model)
{
    renderModelScanlineZBuffer_coreV3(model, 1);
}

void renderModelScanlineZBuffer_fastV2(Model3D* model)
{
    renderModelScanlineZBuffer_coreV2(model, 0);
}

void renderModelScanlineZBuffer_biasedV2(Model3D* model)
{
    renderModelScanlineZBuffer_coreV2(model, 1);
}

/* Dispatcher: the optimized V2 path replaces the former per-scanline/per-face
 * implementation.  The original functions remain above for A/B testing. */
void renderModelScanlineZBuffer(Model3D* model)
{
    if (cull_back_faces)
        renderModelScanlineZBuffer_fastV3(model);
    else
        renderModelScanlineZBuffer_biasedV3(model);
}
