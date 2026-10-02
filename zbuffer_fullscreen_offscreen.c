/* zbuffer_fullscreen_offscreen.c
 *
 * Offscreen variant of the fullscreen Z-buffer renderer.
 * Usage: include/compile after engine.c symbols are visible, then call
 *   Offscreen_Init() once at startup,
 *   renderModelFullscreenZBuffer_offscreen(model) to render into the offscreen
 *   buffer, and Offscreen_FlushToScreen() to copy the buffer to the visible SHR.
 */

segment "ZBUF";
#include "zbuffer_fullscreen.h"

/* Offscreen buffer (locked handle) */
#define OFF_PIXELS 32000   /* 200 lignes * 160 octets */

Handle offscreen_handle = NULL;
Byte   offscreen_bank = 0;
Word   offscreen_offset = 0;

int Offscreen_Init(void)
{
    Pointer p;
    offscreen_handle = NewHandle(OFF_PIXELS, userid(), attrLocked | attrNoCross, 0L);
    if (offscreen_handle == NULL || toolerror()) return 0;
    p = *offscreen_handle;
    offscreen_bank = (Byte)(((long)p >> 16) & 0xFF);
    offscreen_offset = (Word)((long)p & 0xFFFF);
    return 1;
}

void Offscreen_Shutdown(void)
{
    if (offscreen_handle != NULL) {
        DisposeHandle(offscreen_handle);
        offscreen_handle = NULL;
    }
}

/* Clear the offscreen pixel buffer to color 0 (both nibbles zeroed) */
void Offscreen_Clear(void)
{
    if (offscreen_handle == NULL) return;
    volatile unsigned char *buf = *offscreen_handle;
    int i;
    int clear_len = OFF_PIXELS;
    //if (clear_len > SCREEN_SIZE) clear_len = SCREEN_SIZE;
    for (i = 0; i < clear_len; ++i) buf[i] = 0x00;
}

// /* Diagnostic helpers: dump offscreen handle info and a small hex sample */
// void Offscreen_DumpInfo(void)
// {
//     if (offscreen_handle == NULL) {
//         printf("Offscreen: handle=NULL\n");
//         return;
//     }
//     Pointer p = *offscreen_handle;
//     printf("Offscreen: bank=0x%02X offset=0x%04X ptr=%p\n", offscreen_bank, offscreen_offset, p);
// }

// void Offscreen_DumpBytes(int start, int count)
// {
//     if (offscreen_handle == NULL) {
//         printf("Offscreen: handle=NULL\n");
//         return;
//     }
//     volatile unsigned char *buf = *offscreen_handle;
//     int i;
//     printf("Offscreen buffer sample starting 0x%04X (%d bytes):", start, count);
//     for (i = 0; i < count; ++i) {
//         if ((i & 0x0F) == 0) printf("\n%04X: ", start + i);
//         printf("%02X ", buf[start + i]);
//     }
//     printf("\n");
// }

/* Flush offscreen buffer to visible SHR bank $E1 using patched MVN */
void Offscreen_FlushToScreen(void)
{
    if (offscreen_handle == NULL) return;
    asm {
        php
        phb
        rep #0x10
        sep #0x20

        /* patch the MVN immediate operand bytes: operand1 = dest bank, operand2 = src bank
           We want to copy FROM offscreen_handle (src bank) TO visible SHR (dest bank 0xE1) */
        lda offscreen_bank
        sta >flush_mvn+2
        lda #0xE1
        sta >flush_mvn+1

        rep #0x30
        lda offscreen_offset
        tax
        ldy #0x2000
        lda #OFF_PIXELS-1
    flush_mvn:
        mvn 0x00,0xE1

        plb
        plp
    }
}

/* C fallback flush: direct CPU copy from offscreen buffer to SHR address (for debugging) */
// void Offscreen_FlushToScreen_c(void)
// {
//     unsigned int *src;
//     unsigned int *dst = (unsigned int *)0xE12000L;
//     unsigned int i;
//     if (offscreen_handle == NULL) return;
//     src = (unsigned int *)*offscreen_handle;
//     for (i = 0; i < OFF_PIXELS / 2; ++i) dst[i] = src[i];
// }

/* Offscreen pixel plot (C path): write into locked handle buffer packed 2 pixels/byte */
static inline void Offscreen_DrawPixel(int x, int y, int color)
{
    if (offscreen_handle == NULL) return;
    if ((unsigned)x >= (unsigned)SCREEN_WIDTH || (unsigned)y >= (unsigned)SCREEN_HEIGHT) return;
    int offset = y * 160 + (x >> 1);
    volatile unsigned char *buf = *offscreen_handle;
    unsigned char orig = buf[offset];
    color &= 0x0F;
    if ((x & 1) == 0) {
        unsigned char hi = (unsigned char)((color << 4) & 0xF0);
        unsigned char lo = orig & 0x0F;
        buf[offset] = (unsigned char)(hi | lo);
    } else {
        unsigned char lo = (unsigned char)(color & 0x0F);
        unsigned char hi = orig & 0xF0;
        buf[offset] = (unsigned char)(hi | lo);
    }
}

/* Local span-prev struct */
typedef struct {
    int valid;
    int y;
    int x0, x1;
    UWORD16 zq0, zq1;
} OffZBufSpanPrev;

/* Offscreen border/bridge/paint routines adapted from zbuffer_fullscreen_v2.c
   but calling Offscreen_DrawPixel instead of drawPixel. These operate on the
   global zbuf_row/ZBuffer_TestAndSet from the original ZBuffer implementation. */

static void Off_PlotBorder(int sx, int sy, UWORD16 zq, int color)
{
    FarWordPtr row;
    UWORD16 cur;
    if ((unsigned)sx >= (unsigned)ZBUF_WIDTH || (unsigned)sy >= (unsigned)ZBUF_HEIGHT) return;
    row = zbuf_row[sy];
    cur = row[sx];
    if (zq < cur) {
        row[sx] = zq;
        Offscreen_DrawPixel(sx, sy, color);
    }
}

static void Off_PlotBridge(int sx, int sy, UWORD16 zq, int color)
{
    FarWordPtr row;
    UWORD16 cur;
    if ((unsigned)sx >= (unsigned)ZBUF_WIDTH || (unsigned)sy >= (unsigned)ZBUF_HEIGHT) return;
    row = zbuf_row[sy];
    cur = row[sx];
    if (zq == cur)
        Offscreen_DrawPixel(sx, sy, color);
    else if (zq > cur && (int)zq - (int)cur <= 1)
        Offscreen_DrawPixel(sx, sy, color);
}

static void Off_BridgeBorderZQ(int x0, int y0, UWORD16 zq0,
                              int x1, int y1, UWORD16 zq1,
                              int color)
{
    int dx = x1 - x0;
    int dy = y1 - y0;
    int adx = (dx >= 0) ? dx : -dx;
    int ady = (dy >= 0) ? dy : -dy;
    int steps, i, x, y;
    long z_acc, z_step;
    if (dx == 0 && dy == 0) {
        Off_PlotBridge(x0 + pan_dx, y0 + pan_dy, zq0, color);
        return;
    }
    steps = (adx > ady) ? adx : ady;
    if (steps <= 0) return;
    z_acc  = ((long)zq0 << 16) + 0x8000L;
    z_step = (((long)zq1 - (long)zq0) << 16) / steps;
    if (adx >= ady) {
        int y_step = (dy >= 0) ? 1 : -1;
        int err = adx / 2;
        x = x0;
        y = y0;
        for (i = 0; i <= steps; i++) {
            Off_PlotBridge(x + pan_dx, y + pan_dy, (UWORD16)(z_acc >> 16), color);
            if (i == steps) break;
            z_acc += z_step;
            if (dx >= 0) x++; else x--;
            err -= ady;
            if (err < 0) { y += y_step; err += adx; }
        }
    } else {
        int x_step = (dx >= 0) ? 1 : -1;
        int err = ady / 2;
        x = x0;
        y = y0;
        for (i = 0; i <= steps; i++) {
            Off_PlotBridge(x + pan_dx, y + pan_dy, (UWORD16)(z_acc >> 16), color);
            if (i == steps) break;
            z_acc += z_step;
            if (dy >= 0) y++; else y--;
            err -= adx;
            if (err < 0) { x += x_step; err += ady; }
        }
    }
}

#define OFF_FRAC    12
#define OFF_ROUND   0x800L          /* 1 << (OFF_FRAC - 1) */
#define OFF_Z_BIAS  0.0005f         /* meme valeur que Z_FIGHT_BIAS */

typedef struct {
    int x;
    UWORD16 zq;
} OffScanIntersection;

/* zq a l'abscisse x, par interpolation lineaire entre (xa,za) et (xb,zb).
   Uniquement utilise pour les spans coupes par le clipping horizontal. */
static UWORD16 Off_LerpZ(UWORD16 za, UWORD16 zb, int xa, int xb, int x)
{
    long num;
    if (xb == xa) return za;
    num = ((long)zb - (long)za) * (long)(x - xa);
    return (UWORD16)((long)za + num / (long)(xb - xa));
}

/* Meme logique que Off_PaintSpan, mais prend directement les zq aux extremites */
static void Off_PaintSpanZ(int x0, int x1, int y, int screenY,
                           UWORD16 zq0, UWORD16 zq_end,
                           int fillColor, int frameColor,
                           int whole_span_is_border,
                           OffZBufSpanPrev *prev,
                           int z_nudge_lsbs)
{
    int nsteps, x, sx, sxr, silhouette_r;
    UWORD16 zq_cur, zr;
    long z_acc, z_step_16;

    if (x0 > x1) return;

    if (z_nudge_lsbs > 0) {
        if (zq0 > (UWORD16)z_nudge_lsbs) zq0 = (UWORD16)(zq0 - (UWORD16)z_nudge_lsbs);
        else zq0 = 0;
        if (zq_end > (UWORD16)z_nudge_lsbs) zq_end = (UWORD16)(zq_end - (UWORD16)z_nudge_lsbs);
        else zq_end = 0;
    }

    if (x0 == x1) {
        zq_end = zq0;
        Off_PlotBorder(x0 + pan_dx, screenY, zq0, frameColor);
    } else {
        nsteps    = x1 - x0;
        z_step_16 = (((long)zq_end - (long)zq0) << 16) / nsteps;
        z_acc     = ((long)zq0 << 16) + 0x8000L;

        Off_PlotBorder(x0 + pan_dx, screenY, zq0, frameColor);

        if (whole_span_is_border) {
            for (x = x0 + 1; x < x1; x++) {
                z_acc += z_step_16;
                zq_cur = (UWORD16)(z_acc >> 16);
                Off_PlotBorder(x + pan_dx, screenY, zq_cur, frameColor);
            }
        } else {
            for (x = x0 + 1; x < x1; x++) {
                z_acc += z_step_16;
                zq_cur = (UWORD16)(z_acc >> 16);
                sx = x + pan_dx;
                if (ZBuffer_TestAndSet(sx, screenY, zq_cur))
                    Offscreen_DrawPixel(sx, screenY, fillColor);
            }
        }

        sxr = x1 + pan_dx;
        silhouette_r = 1;
        if (sxr + 1 < ZBUF_WIDTH) {
            zr = zbuf_row[screenY][sxr + 1];
            if (zr != ZBUF_FAR_VALUE && (int)zq_end - (int)zr >= -1) silhouette_r = 0;
        }
        if (silhouette_r || whole_span_is_border)
            Off_PlotBorder(sxr, screenY, zq_end, frameColor);
        else if (ZBuffer_TestAndSet(sxr, screenY, zq_end))
            Offscreen_DrawPixel(sxr, screenY, fillColor);
    }

    if (prev != NULL && prev->valid && prev->y == y - 1) {
        int dxl = prev->x0 - x0;
        int dxr = prev->x1 - x1;
        if (dxl < 0) dxl = -dxl;
        if (dxr < 0) dxr = -dxr;
        if (dxl > 1) Off_BridgeBorderZQ(prev->x0, prev->y, prev->zq0, x0, y, zq0, frameColor);
        if (dxr > 1) Off_BridgeBorderZQ(prev->x1, prev->y, prev->zq1, x1, y, zq_end, frameColor);
    }

    if (prev != NULL) {
        prev->valid = 1;
        prev->y = y;
        prev->x0 = x0;
        prev->x1 = x1;
        prev->zq0 = zq0;
        prev->zq1 = zq_end;
    }
}

/* Rendu commun aux variantes fast (biased = 0) et biased (biased = 1) */
static void Off_RenderCore(Model3D* model, int biased)
{
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;
    int fcount = faces->face_count;
    int y, f, i, k, e;
    int total_edges, end;
    int clip_x_min, clip_x_max, y_lo, y_hi;
    float frame_max_inv_z;
    OffScanIntersection hits[MAX_SPAN_INTERSECTIONS];

    static float* inv_z = NULL;
    static UWORD16* vzq = NULL;
    static UWORD16* vzq_b = NULL;
    static int vert_capacity = 0;
    static OffZBufSpanPrev* span_prev = NULL;
    static int span_prev_capacity = 0;
    static long* e_x = NULL;     /* x courant (12 bits frac.)  */
    static long* e_z = NULL;     /* zq courant (12 bits frac.) */
    static long* e_sx = NULL;    /* pente x par ligne          */
    static long* e_sz = NULL;    /* pente zq par ligne         */
    static int edge_capacity = 0;

    /* ---- allocations (croissance seulement) ---- */
    if (span_prev_capacity < fcount) {
        if (span_prev) free(span_prev);
        span_prev = (OffZBufSpanPrev*)malloc(fcount * sizeof(OffZBufSpanPrev));
        span_prev_capacity = span_prev ? fcount : 0;
    }
    if (vert_capacity < vcount) {
        if (inv_z) free(inv_z);
        if (vzq)   free(vzq);
        if (vzq_b) free(vzq_b);
        inv_z = (float*)malloc(vcount * sizeof(float));
        vzq   = (UWORD16*)malloc(vcount * sizeof(UWORD16));
        vzq_b = (UWORD16*)malloc(vcount * sizeof(UWORD16));
        vert_capacity = (inv_z && vzq && vzq_b) ? vcount : 0;
    }
    total_edges = 0;
    for (f = 0; f < fcount; f++) {
        end = faces->vertex_indices_ptr[f] + faces->vertex_count[f];
        if (end > total_edges) total_edges = end;
    }
    if (edge_capacity < total_edges) {
        if (e_x)  free(e_x);
        if (e_z)  free(e_z);
        if (e_sx) free(e_sx);
        if (e_sz) free(e_sz);
        e_x  = (long*)malloc((size_t)total_edges * sizeof(long));
        e_z  = (long*)malloc((size_t)total_edges * sizeof(long));
        e_sx = (long*)malloc((size_t)total_edges * sizeof(long));
        e_sz = (long*)malloc((size_t)total_edges * sizeof(long));
        edge_capacity = (e_x && e_z && e_sx && e_sz) ? total_edges : 0;
    }
    if (!span_prev || !inv_z || !vzq || !vzq_b || !e_x || !e_z || !e_sx || !e_sz)
        return;

    for (f = 0; f < fcount; f++) span_prev[f].valid = 0;

    /* ---- par sommet, une fois par frame : 1/z, echelle, quantification ---- */
    frame_max_inv_z = 0.0f;
    for (i = 0; i < vcount; i++) {
        float zo_f = FIXED_TO_FLOAT(vtx->zo[i]);
        inv_z[i] = (zo_f > 0.0f) ? (1.0f / zo_f) : 0.0f;
        if (inv_z[i] > frame_max_inv_z) frame_max_inv_z = inv_z[i];
    }
    ZBuffer_SetScaleForFrame(frame_max_inv_z);
    for (i = 0; i < vcount; i++) {
        vzq[i]   = ZBuffer_QuantizeInvZ(inv_z[i]);
        vzq_b[i] = biased ? ZBuffer_QuantizeInvZ(inv_z[i] + OFF_Z_BIAS) : vzq[i];
    }

    SetPenMode(0);
    applyPalette(palette);

    clip_x_min = -pan_dx;
    clip_x_max = SCREEN_WIDTH - 1 - pan_dx;
    y_lo = -pan_dy;
    y_hi = SCREEN_HEIGHT - 1 - pan_dy;

    ZBuffer_Clear();

    /* ---- initialisation des aretes : pentes + valeur a la premiere ligne visible ---- */
    for (f = 0; f < fcount; f++) {
        int n, offt;
        UWORD16* zsrc;

        if (!faces->display_flag[f]) continue;
        n = faces->vertex_count[f];
        if (n < 3) continue;
        offt = faces->vertex_indices_ptr[f];
        zsrc = (biased && faces->plane_d[f] > 0) ? vzq_b : vzq;

        for (k = 0; k < n; k++) {
            int k2 = (k + 1 < n) ? (k + 1) : 0;
            int vid1 = faces->vertex_indices_buffer[offt + k] - 1;
            int vid2 = faces->vertex_indices_buffer[offt + k2] - 1;
            int y1 = vtx->y2d[vid1];
            int y2 = vtx->y2d[vid2];
            int dy = y2 - y1;
            int ya, yb, xs;
            long zs, ex, ez, sx, sz, skip;

            e = offt + k;
            if (dy == 0) {
                e_x[e] = 0L; e_z[e] = 0L; e_sx[e] = 0L; e_sz[e] = 0L;
                continue;
            }

            sx = (((long)vtx->x2d[vid2] - (long)vtx->x2d[vid1]) * 4096L) / (long)dy;
            sz = (((long)zsrc[vid2] - (long)zsrc[vid1]) * 4096L) / (long)dy;

            if (dy > 0) { ya = y1; yb = y2; xs = vtx->x2d[vid1]; zs = (long)zsrc[vid1]; }
            else        { ya = y2; yb = y1; xs = vtx->x2d[vid2]; zs = (long)zsrc[vid2]; }

            ex = (long)xs * 4096L;
            ez = zs * 4096L;

            /* arete qui commence au-dessus de la zone visible : avancer d'un coup */
            if (ya < y_lo && yb > y_lo) {
                skip = (long)(y_lo - ya);
                ex += sx * skip;
                ez += sz * skip;
            }

            e_x[e] = ex; e_z[e] = ez; e_sx[e] = sx; e_sz[e] = sz;
        }
    }

    /* ---- boucle principale ---- */
    for (y = y_lo; y <= y_hi; y++) {
        int screenY = y + pan_dy;

        for (f = 0; f < fcount; f++) {
            int n, offt, hit_count;
            int fillColor, frameColor;
            int on_top_or_bottom, nudge;

            if (!faces->display_flag[f]) continue;
            n = faces->vertex_count[f];
            if (n < 3) continue;
            if (y < faces->miny[f] || y > faces->maxy[f]) continue;

            fillColor  = getFaceFillColor(f);
            frameColor = getFaceFrameColor(f, fillColor);
            on_top_or_bottom = (y == faces->miny[f] || y == faces->maxy[f]);
            nudge = (biased && faces->plane_d[f] > 0) ? 2 : 0;

            offt = faces->vertex_indices_ptr[f];
            hit_count = 0;

            for (k = 0; k < n; k++) {
                int k2 = (k + 1 < n) ? (k + 1) : 0;
                int vid1 = faces->vertex_indices_buffer[offt + k] - 1;
                int vid2 = faces->vertex_indices_buffer[offt + k2] - 1;
                int y1 = vtx->y2d[vid1];
                int y2 = vtx->y2d[vid2];
                int dy = y2 - y1;

                if (dy == 0) continue;
                if (dy > 0) {
                    if (y < y1 || y >= y2) continue;
                } else {
                    if (y < y2 || y >= y1) continue;
                }

                e = offt + k;
                if (hit_count < MAX_SPAN_INTERSECTIONS) {
                    hits[hit_count].x  = (int)((e_x[e] + OFF_ROUND) >> OFF_FRAC);
                    hits[hit_count].zq = (UWORD16)((e_z[e] + OFF_ROUND) >> OFF_FRAC);
                    hit_count++;
                }
                /* avancer meme si la table d'intersections est pleine */
                e_x[e] += e_sx[e];
                e_z[e] += e_sz[e];
            }

            if (hit_count < 2) continue;

            if (hit_count == 2) {
                if (hits[0].x > hits[1].x) {
                    OffScanIntersection tmp = hits[0];
                    hits[0] = hits[1];
                    hits[1] = tmp;
                }
            } else {
                int a, b;
                for (a = 1; a < hit_count; a++) {
                    OffScanIntersection key = hits[a];
                    b = a - 1;
                    while (b >= 0 && hits[b].x > key.x) {
                        hits[b + 1] = hits[b];
                        b--;
                    }
                    hits[b + 1] = key;
                }
            }

            {
                int p;
                for (p = 0; p + 1 < hit_count; p += 2) {
                    int xa = hits[p].x;
                    int xb = hits[p + 1].x;
                    UWORD16 zqa = hits[p].zq;
                    UWORD16 zqb = hits[p + 1].zq;
                    int x0, x1;
                    UWORD16 zq0, zq1;

                    if (xa > xb) continue;
                    if (xb < clip_x_min || xa > clip_x_max) continue;

                    x0 = xa;  x1 = xb;
                    zq0 = zqa; zq1 = zqb;

                    if (x0 < clip_x_min) { zq0 = Off_LerpZ(zqa, zqb, xa, xb, clip_x_min); x0 = clip_x_min; }
                    if (x1 > clip_x_max) { zq1 = Off_LerpZ(zqa, zqb, xa, xb, clip_x_max); x1 = clip_x_max; }
                    if (x0 > x1) continue;

                    Off_PaintSpanZ(x0, x1, y, screenY, zq0, zq1,
                                   fillColor, frameColor, on_top_or_bottom,
                                   &span_prev[f], nudge);
                }
            }
        }
    }
}

void renderModelFullscreenZBuffer_offscreen_fastV2(Model3D* model)
{
    Off_RenderCore(model, 0);
}

void renderModelFullscreenZBuffer_offscreen_biasedV2(Model3D* model)
{
    Off_RenderCore(model, 1);
}

void renderModelFullscreenZBuffer_offscreen(Model3D* model)
{
    if (cull_back_faces) renderModelFullscreenZBuffer_offscreen_fastV2(model);
    else renderModelFullscreenZBuffer_offscreen_biasedV2(model);
}
