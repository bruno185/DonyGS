/* zbuffer_fullscreen.c
 *
 * Full-screen 16-bit Z-buffer + renderer — V2 (scanline borders).
 *
 * Border algorithm (scanline + gap bridge, no geometric edge pass):
 *   frameColor on span left/right endpoints;
 *   frameColor on the whole span when y == face miny/maxy;
 *   fillColor elsewhere.
 *   Between consecutive scanlines, left and right endpoints are
 *   connected (bridge) so shallow diagonals are solid, not dotted.
 *   Bridge never writes a closer Z (only reinforces on-surface pixels)
 *   — writing closer Z was a major source of occlusion leaks.
 *   Border/fill paint only when strictly closer.
 *
 * USAGE:
 *   #include "zbuffer_fullscreen_v2.c"  after engine.c symbols are visible.
 *   ZBuffer_Init() once at startup, ZBuffer_Shutdown() on exit.
 */


segment "ZBUF";
#include "zbuffer_fullscreen.h"


#ifdef ZBUF_DIAG_SCALE
#include <stdio.h>
#endif


FarWordPtr zbuf_row[ZBUF_HEIGHT];


static Handle  zbuf_handle = NULL;
static int     zbuf_bank_lo = 0;
static int     zbuf_bank_hi = 0;


#define ZBUF_USABLE_SIZE   (2UL * 65536UL)
#define ZBUF_ALLOC_SIZE    (ZBUF_USABLE_SIZE + 65536UL)


int ZBuffer_Init(void)
{
    ULONG32 raw;
    ULONG32 aligned;
    int     y;

    zbuf_handle = NewHandle(ZBUF_ALLOC_SIZE, userid(), attrFixed, 0L);
    if (toolerror() || zbuf_handle == NULL) {
        return 0;
    }

    raw     = (ULONG32) *zbuf_handle;
    aligned = (raw + 0xFFFFUL) & 0xFFFF0000UL;

    zbuf_bank_lo = (int)((aligned >> 16) & 0xFFUL);
    zbuf_bank_hi = zbuf_bank_lo + 1;

    for (y = 0; y < ZBUF_HEIGHT; y++) {
        zbuf_row[y] = (FarWordPtr)(aligned + (ULONG32)y * (ULONG32)ZBUF_ROW_BYTES);
    }

    ZBuffer_Clear();
    return 1;
}


void ZBuffer_Shutdown(void)
{
    if (zbuf_handle != NULL) {
        DisposeHandle(zbuf_handle);
        zbuf_handle = NULL;
    }
}


void ZBuffer_Clear(void)
{
    int y, x;
    for (y = 0; y < ZBUF_HEIGHT; y++) {
        FarWordPtr row = zbuf_row[y];
        for (x = 0; x < ZBUF_WIDTH; x++) {
            row[x] = ZBUF_FAR_VALUE;
        }
    }
}


static float zbuffer_scale = ZBUFFER_INV_Z_SCALE;


void ZBuffer_SetScaleForFrame(float max_inv_z)
{
    if (max_inv_z > 0.0000001f) {
        zbuffer_scale = ZBUFFER_TARGET_MAX_CODE / max_inv_z;
    }
}


UWORD16 ZBuffer_QuantizeInvZ(float inv_z)
{
    float   scaled;
    long    q;

    if (inv_z < 0.0f) inv_z = 0.0f;

    scaled = inv_z * zbuffer_scale;
    q = (long)(scaled + 0.5f);
    if (q < 0)      q = 0;
    if (q > 0xFFFF) q = 0xFFFF;

    return (UWORD16)(0xFFFFUL - (ULONG32)q);
}


int ZBuffer_TestAndSet(int x, int y, UWORD16 z)
{
    FarWordPtr row = zbuf_row[y];
    if (z < row[x]) {
        row[x] = z;
        return 1;
    }
    return 0;
}


/* ====================================================================
 * Inline assembler
 * ==================================================================== */


asm void ZBuffer_ClearFast(int bank_lo, int bank_hi)
    {
        php
        phb
        sep     #0x20
        lda     bank_lo
        pha
        plb
        rep     #0x30
        ldx     #0x0000
        lda     #0xFFFF
clear_bank_lo:
        sta     0x0000,x
        inx
        inx
        bne     clear_bank_lo
        sep     #0x20
        lda     bank_hi
        pha
        plb
        rep     #0x30
        ldx     #0x0000
        lda     #0xFFFF
clear_bank_hi:
        sta     0x0000,x
        inx
        inx
        bne     clear_bank_hi
        plb
        plp
        rtl
    }


asm void ZBuffer_TestSetRow(FarWordPtr zbuf_ptr, UWORD16 z_start,
                             SWORD16 dz_step, int count)
    {
        php
        rep     #0x30
        cld
        ldy     #0x0000
        lda     z_start
        ldx     count
        beq     zsr_done
zsr_loop:
        cmp     [zbuf_ptr],y
        bcs     zsr_skip
        sta     [zbuf_ptr],y
zsr_skip:
        clc
        adc     dz_step
        iny
        iny
        dex
        bne     zsr_loop
zsr_done:
        plp
        rtl
    }


/* ====================================================================
 * Span painter with integrated 1-px border + gap bridging (fast path)
 *
 * Hot path stays integer after the two endpoint quantisations — same
 * cost model as the original V1 span loop.  Bridge runs only when
 * |Δx| > 1 and also uses integer Z steps (no per-pixel float quantise).
 * ==================================================================== */

typedef struct {
    int     valid;
    int     y;
    int     x0, x1;
    UWORD16 zq0, zq1;
} ZBufSpanPrev;


/* Border plot: strictly closer only.
 * Equal-Z repaint was letting a later (often farther) face overwrite
 * colour when quantisation made depths look equal — small regions of
 * hidden faces then appeared "in front". */
static void ZBuffer_PlotBorder(int sx, int sy, UWORD16 zq, int color)
{
    FarWordPtr row;
    UWORD16 cur;

    if ((unsigned)sx >= (unsigned)ZBUF_WIDTH ||
        (unsigned)sy >= (unsigned)ZBUF_HEIGHT)
        return;

    row = zbuf_row[sy];
    cur = row[sx];
    if (zq < cur) {
        row[sx] = zq;
        drawPixel(sx, sy, color);
    }
}


/* Integer-Z bridge between two already-quantised endpoints. */
/* Bridge plot: NEVER write a closer Z (that punched through other
 * surfaces and caused occlusion leaks).  Only paint colour when the
 * buffer depth already matches ours (pixel is on this surface). */
static void ZBuffer_PlotBridge(int sx, int sy, UWORD16 zq, int color)
{
    FarWordPtr row;
    UWORD16 cur;

    if ((unsigned)sx >= (unsigned)ZBUF_WIDTH ||
        (unsigned)sy >= (unsigned)ZBUF_HEIGHT)
        return;

    row = zbuf_row[sy];
    cur = row[sx];
    /* identical depth → on-surface outline reinforce */
    if (zq == cur)
        drawPixel(sx, sy, color);
    /* allow 1 LSB quantisation tolerance on the FARTHER side only */
    else if (zq > cur && (int)zq - (int)cur <= 1)
        drawPixel(sx, sy, color);
    /* zq < cur would be closer → skip (would be a leak) */
}


static void ZBuffer_BridgeBorderZQ(int x0, int y0, UWORD16 zq0,
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
        ZBuffer_PlotBridge(x0 + pan_dx, y0 + pan_dy, zq0, color);
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
            ZBuffer_PlotBridge(x + pan_dx, y + pan_dy,
                               (UWORD16)(z_acc >> 16), color);
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
            ZBuffer_PlotBridge(x + pan_dx, y + pan_dy,
                               (UWORD16)(z_acc >> 16), color);
            if (i == steps) break;
            z_acc += z_step;
            if (dy >= 0) y++; else y--;
            err -= adx;
            if (err < 0) { x += x_step; err += ady; }
        }
    }
}


static void ZBuffer_PaintSpan(int x0, int x1, int y, int screenY,
                              float iz0, float dIz,
                              int fillColor, int frameColor,
                              int whole_span_is_border,
                              ZBufSpanPrev *prev,
                              int z_nudge_lsbs)
{
    int nsteps, x, sx;
    UWORD16 zq0, zq_end, zq_cur;
    float iz_end_f;
    long z_acc, z_step_16;   /* 16.16 fixed — avoids truncation drift */

    if (x0 > x1) return;

    if (x0 == x1) {
        zq0 = ZBuffer_QuantizeInvZ(iz0);
        if (z_nudge_lsbs > 0) {
            if (zq0 > (UWORD16)z_nudge_lsbs)
                zq0 = (UWORD16)(zq0 - (UWORD16)z_nudge_lsbs);
            else
                zq0 = 0;
        }
        zq_end = zq0;
        ZBuffer_PlotBorder(x0 + pan_dx, screenY, zq0, frameColor);
    } else {
        iz_end_f = iz0 + dIz * (float)(x1 - x0);
        zq0      = ZBuffer_QuantizeInvZ(iz0);
        zq_end   = ZBuffer_QuantizeInvZ(iz_end_f);
        if (z_nudge_lsbs > 0) {
            if (zq0 > (UWORD16)z_nudge_lsbs)
                zq0 = (UWORD16)(zq0 - (UWORD16)z_nudge_lsbs);
            else
                zq0 = 0;
            if (zq_end > (UWORD16)z_nudge_lsbs)
                zq_end = (UWORD16)(zq_end - (UWORD16)z_nudge_lsbs);
            else
                zq_end = 0;
        }
        nsteps   = x1 - x0;
        /* 16.16 step so intermediate depths stay faithful to endpoints
         * (plain int step was drifting and letting far faces leak). */
        z_step_16 = (((long)zq_end - (long)zq0) << 16) / nsteps;
        z_acc     = ((long)zq0 << 16) + 0x8000L;

        /* Left border */
        ZBuffer_PlotBorder(x0 + pan_dx, screenY, zq0, frameColor);

        /* Interior */
        if (whole_span_is_border) {
            for (x = x0 + 1; x < x1; x++) {
                z_acc += z_step_16;
                zq_cur = (UWORD16)(z_acc >> 16);
                ZBuffer_PlotBorder(x + pan_dx, screenY, zq_cur, frameColor);
            }
        } else {
            for (x = x0 + 1; x < x1; x++) {
                z_acc += z_step_16;
                zq_cur = (UWORD16)(z_acc >> 16);
                sx = x + pan_dx;
                if (ZBuffer_TestAndSet(sx, screenY, zq_cur))
                    drawPixel(sx, screenY, fillColor);
            }
        }

        /* Right edge: always cover the pixel (frame or fill).
         * Frame only on a true right silhouette; otherwise fillColor. */
        {
            int sxr = x1 + pan_dx;
            int silhouette_r = 1;
            if (sxr + 1 < ZBUF_WIDTH) {
                UWORD16 zr = zbuf_row[screenY][sxr + 1];
                if (zr != ZBUF_FAR_VALUE &&
                    (int)zq_end - (int)zr >= -1)
                    silhouette_r = 0;
            }
            if (silhouette_r || whole_span_is_border)
                ZBuffer_PlotBorder(sxr, screenY, zq_end, frameColor);
            else if (ZBuffer_TestAndSet(sxr, screenY, zq_end))
                drawPixel(sxr, screenY, fillColor);
        }
    }

    /* Bridge only real gaps; integer Z — cheap */
    if (prev != NULL && prev->valid && prev->y == y - 1) {
        int dxl = prev->x0 - x0;
        int dxr = prev->x1 - x1;
        if (dxl < 0) dxl = -dxl;
        if (dxr < 0) dxr = -dxr;
        if (dxl > 1)
            ZBuffer_BridgeBorderZQ(prev->x0, prev->y, prev->zq0,
                                   x0, y, zq0, frameColor);
        if (dxr > 1)
            ZBuffer_BridgeBorderZQ(prev->x1, prev->y, prev->zq1,
                                   x1, y, zq_end, frameColor);
    }

    if (prev != NULL) {
        prev->valid = 1;
        prev->y   = y;
        prev->x0  = x0;
        prev->x1  = x1;
        prev->zq0 = zq0;
        prev->zq1 = zq_end;
    }
}


/* ====================================================================
 * Rendering
 * ==================================================================== */


void renderModelFullscreenZBuffer_fastV2(Model3D* model)
{
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;
    int fcount = faces->face_count;
    int y, f, i;

    static float* inv_z = NULL;
    static int inv_z_capacity = 0;
    static ZBufSpanPrev* span_prev = NULL;
    static int span_prev_capacity = 0;

    if (inv_z_capacity < vcount) {
        if (inv_z) free(inv_z);
        inv_z = (float*)malloc(vcount * sizeof(float));
        inv_z_capacity = vcount;
    }
    if (span_prev_capacity < fcount) {
        if (span_prev) free(span_prev);
        span_prev = (ZBufSpanPrev*)malloc(fcount * sizeof(ZBufSpanPrev));
        span_prev_capacity = fcount;
    }
    for (f = 0; f < fcount; f++)
        span_prev[f].valid = 0;

    {
        float frame_max_inv_z = 0.0f;
        for (i = 0; i < vcount; i++) {
            float zo_f = FIXED_TO_FLOAT(vtx->zo[i]);
            inv_z[i] = (zo_f > 0.0f) ? (1.0f / zo_f) : 0.0f;
            if (inv_z[i] > frame_max_inv_z) frame_max_inv_z = inv_z[i];
        }
        ZBuffer_SetScaleForFrame(frame_max_inv_z);
    }

#ifdef ZBUF_DIAG_SCALE
    {
        static int zdiag_logged = 0;
        if (!zdiag_logged) {
            float dbg_min = 1e30f, dbg_max = -1e30f;
            FILE* zdiag_f;
            for (i = 0; i < vcount; i++) {
                if (inv_z[i] > 0.0f) {
                    if (inv_z[i] < dbg_min) dbg_min = inv_z[i];
                    if (inv_z[i] > dbg_max) dbg_max = inv_z[i];
                }
            }
            zdiag_f = fopen("zdiag.txt", "w");
            if (zdiag_f) {
                fprintf(zdiag_f, "inv_z min=%f max=%f vcount=%d\n",
                        dbg_min, dbg_max, vcount);
                fclose(zdiag_f);
            }
            zdiag_logged = 1;
        }
    }
#endif

    ScanIntersection hits[MAX_SPAN_INTERSECTIONS];

    SetPenMode(0);
    applyPalette(palette);

    {
        int clip_x_min = -pan_dx;
        int clip_x_max = SCREEN_WIDTH - 1 - pan_dx;
        int y_lo = -pan_dy;
        int y_hi = SCREEN_HEIGHT - 1 - pan_dy;

        ZBuffer_Clear();

        for (y = y_lo; y <= y_hi; y++) {
            int screenY = y + pan_dy;

            for (f = 0; f < fcount; f++) {
                int n, offt, k, hit_count;
                int fillColor, frameColor;
                int on_top_or_bottom;

                if (!faces->display_flag[f]) continue;
                n = faces->vertex_count[f];
                if (n < 3) continue;
                if (y < faces->miny[f] || y > faces->maxy[f]) continue;

                fillColor  = getFaceFillColor(f);
                frameColor = getFaceFrameColor(f, fillColor);
                on_top_or_bottom = (y == faces->miny[f] || y == faces->maxy[f]);

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

                    {
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
                }

                if (hit_count < 2) continue;

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

                {
                    int p;
                    for (p = 0; p + 1 < hit_count; p += 2) {
                        int xa = hits[p].x;
                        int xb = hits[p + 1].x;
                        float iza = hits[p].inv_z;
                        float izb = hits[p + 1].inv_z;
                        int x0, x1;
                        float iz0, dIz;

                        if (xa > xb) continue;
                        if (xb < clip_x_min || xa > clip_x_max) continue;

                        x0 = xa;
                        x1 = xb;
                        iz0 = iza;

                        if (x0 == x1) {
                            dIz = 0.0f;
                        } else {
                            dIz = (izb - iza) / (float)(xb - xa);
                        }

                        if (x0 < clip_x_min) {
                            iz0 += dIz * (clip_x_min - x0);
                            x0 = clip_x_min;
                        }
                        if (x1 > clip_x_max) x1 = clip_x_max;
                        if (x0 > x1) continue;

                        ZBuffer_PaintSpan(x0, x1, y, screenY, iz0, dIz,
                                          fillColor, frameColor,
                                          on_top_or_bottom,
                                          &span_prev[f],
                                          0);
                    }
                }
            }
        }
    }
}


void renderModelFullscreenZBuffer_biasedV2(Model3D* model)
{
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;
    int fcount = faces->face_count;
    int y, f, i;
    static const float Z_FIGHT_BIAS = 0.0005f; /* small; quantised nudge does the rest */

    static float* inv_z = NULL;
    static int inv_z_capacity = 0;
    static ZBufSpanPrev* span_prev = NULL;
    static int span_prev_capacity = 0;

    if (span_prev_capacity < fcount) {
        if (span_prev) free(span_prev);
        span_prev = (ZBufSpanPrev*)malloc(fcount * sizeof(ZBufSpanPrev));
        span_prev_capacity = fcount;
    }
    for (f = 0; f < fcount; f++)
        span_prev[f].valid = 0;

    if (inv_z_capacity < vcount) {
        if (inv_z) free(inv_z);
        inv_z = (float*)malloc(vcount * sizeof(float));
        inv_z_capacity = vcount;
    }
    {
        float frame_max_inv_z = 0.0f;
        for (i = 0; i < vcount; i++) {
            float zo_f = FIXED_TO_FLOAT(vtx->zo[i]);
            inv_z[i] = (zo_f > 0.0f) ? (1.0f / zo_f) : 0.0f;
            if (inv_z[i] > frame_max_inv_z) frame_max_inv_z = inv_z[i];
        }
        ZBuffer_SetScaleForFrame(frame_max_inv_z);
    }

    ScanIntersection hits[MAX_SPAN_INTERSECTIONS];

    SetPenMode(0);
    applyPalette(palette);

    {
        int clip_x_min = -pan_dx;
        int clip_x_max = SCREEN_WIDTH - 1 - pan_dx;
        int y_lo = -pan_dy;
        int y_hi = SCREEN_HEIGHT - 1 - pan_dy;

        ZBuffer_Clear();

        for (y = y_lo; y <= y_hi; y++) {
            int screenY = y + pan_dy;

            for (f = 0; f < fcount; f++) {
                int n, offt, k, hit_count;
                int fillColor, frameColor;
                int on_top_or_bottom;
                float depthBias;

                if (!faces->display_flag[f]) continue;
                n = faces->vertex_count[f];
                if (n < 3) continue;
                if (y < faces->miny[f] || y > faces->maxy[f]) continue;

                fillColor  = getFaceFillColor(f);
                frameColor = getFaceFrameColor(f, fillColor);
                on_top_or_bottom = (y == faces->miny[f] || y == faces->maxy[f]);
                depthBias = (faces->plane_d[f] > 0) ? Z_FIGHT_BIAS : 0.0f;

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

                    {
                        int x1 = vtx->x2d[vid1];
                        int x2 = vtx->x2d[vid2];
                        float t = (float)(y - y1) / (float)dy;
                        int xi = x1 + (int)((x2 - x1) * t + 0.5f);
                        float izf = inv_z[vid1] + (inv_z[vid2] - inv_z[vid1]) * t;
                        izf += depthBias;

                        if (hit_count < MAX_SPAN_INTERSECTIONS) {
                            hits[hit_count].x = xi;
                            hits[hit_count].inv_z = izf;
                            hit_count++;
                        }
                    }
                }

                if (hit_count < 2) continue;

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

                {
                    int p;
                    for (p = 0; p + 1 < hit_count; p += 2) {
                        int xa = hits[p].x;
                        int xb = hits[p + 1].x;
                        float iza = hits[p].inv_z;
                        float izb = hits[p + 1].inv_z;
                        int x0, x1;
                        float iz0, dIz;

                        if (xa > xb) continue;
                        if (xb < clip_x_min || xa > clip_x_max) continue;

                        x0 = xa;
                        x1 = xb;
                        iz0 = iza;

                        if (x0 == x1) {
                            dIz = 0.0f;
                        } else {
                            dIz = (izb - iza) / (float)(xb - xa);
                        }

                        if (x0 < clip_x_min) {
                            iz0 += dIz * (clip_x_min - x0);
                            x0 = clip_x_min;
                        }
                        if (x1 > clip_x_max) x1 = clip_x_max;
                        if (x0 > x1) continue;

                        ZBuffer_PaintSpan(x0, x1, y, screenY, iz0, dIz,
                                          fillColor, frameColor,
                                          on_top_or_bottom,
                                          &span_prev[f],
                                          (faces->plane_d[f] > 0) ? 2 : 0);
                    }
                }
            }
        }
    }
}


void renderModelFullscreenZBuffer(Model3D* model)
{
    if (cull_back_faces) {
        renderModelFullscreenZBuffer_fastV2(model);
    } else {
        renderModelFullscreenZBuffer_biasedV2(model);
    }
}
