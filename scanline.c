
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

// --- Dispatcher --------------------------------------------------------
// Choisit la version adaptee selon cull_back_faces :
//  - actif  -> pas de faces back rendues, donc pas de collision front/back
//              coplanaire possible -> version rapide, sans aucun surcout
//              lie au biais anti Z-fighting.
//  - inactif -> les deux faces d'une paire coplanaire peuvent etre visibles
//              -> version avec biais, seule capable de forcer le front a
//              gagner le Z-test de facon fiable.
void renderModelScanlineZBuffer(Model3D* model) {
    if (cull_back_faces) {
        renderModelScanlineZBuffer_fast(model);
    } else {
        renderModelScanlineZBuffer_biased(model);
    }
}
