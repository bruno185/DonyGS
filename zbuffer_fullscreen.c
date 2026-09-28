/* zbuffer_fullscreen.c
 *
 * Implementation of the full-screen 16-bit Z-buffer, AND rendering
 * (renderModelFullscreenZBuffer, port of
 * renderModelScanlineZBuffer_fast AND _biased, with the same
 * dispatcher according to cull_back_faces). See zbuffer_fullscreen.h for the
 * chosen value encoding (inverted distance, 0xFFFF = empty) and
 * the precision warning.
 *
 * USAGE IN engine.c :
 *   #include "zbuffer_fullscreen.c"
 * to be placed AFTER the definitions of Model3D, VertexArrays3D,
 * FaceArrays3D, ScanIntersection, MAX_SPAN_INTERSECTIONS,
 * SCREEN_WIDTH/SCREEN_HEIGHT, getFaceFillColor, getFaceFrameColor,
 * drawPixel, FIXED_TO_FLOAT, pan_dx/pan_dy, palette -- the rendering
 * part of this file (at the bottom) uses them directly without going
 * through a common header, so they must already be visible at the
 * moment the preprocessor pastes this file.
 *
 * Call ZBuffer_Init() once at program startup (before
 * the first call to renderModelFullscreenZBuffer) and
 * ZBuffer_Shutdown() on exit.
 */


segment "ZBUF";
#include "zbuffer_fullscreen.h"


#ifdef ZBUF_DIAG_SCALE
#include <stdio.h>   /* fopen/fprintf for the calibration diagnostic -- to be removed with ZBUF_DIAG_SCALE */
#endif


FarWordPtr zbuf_row[ZBUF_HEIGHT];


static Handle  zbuf_handle = NULL;
static int     zbuf_bank_lo = 0;
static int     zbuf_bank_hi = 0;


/* ------------------------------------------------------------------
 * Bank-aligned allocation.
 *
 * We keep the alignment on a bank boundary -- not to
 * prevent a line from straddling it (long indirect addressing
 * [ptr],y handles that correctly, see below), but so that
 * ZBuffer_ClearFast can simply blast two whole banks without
 * having to compute an alignment offset at clear time.
 * ------------------------------------------------------------------ */


#define ZBUF_USABLE_SIZE   (2UL * 65536UL)               /* 2 usable banks       */
#define ZBUF_ALLOC_SIZE    (ZBUF_USABLE_SIZE + 65536UL)  /* + alignment margin   */


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
    aligned = (raw + 0xFFFFUL) & 0xFFFF0000UL;   /* round up to upper bank boundary */


    zbuf_bank_lo = (int)((aligned >> 16) & 0xFFUL);
    zbuf_bank_hi = zbuf_bank_lo + 1;


    /* Simple linear addressing: no line needs special
     * treatment, even the one that straddles the bank boundary
     * (see ZBuffer_TestSetRow).
     *
     * WARNING 16-bit overflow (real bug encountered and fixed) :
     * (ULONG32)(y * ZBUF_ROW_BYTES) first computes "y * ZBUF_ROW_BYTES"
     * in "int" arithmetic (16 bits on ORCA/C), which overflows for
     * y >= 52 (52*640 = 33280 > 32767) BEFORE the cast to ULONG32
     * takes effect -- too late, the overflow has already occurred. You
     * must force 32 bits INSIDE the multiplication itself, not
     * after, by casting at least one of the two operands before
     * multiplying. */
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


/* Current scale, recomputed each frame by
 * ZBuffer_SetScaleForFrame() -- see zbuffer_fullscreen.h for
 * why a fixed value does not work as soon as the zoom changes. */
static float zbuffer_scale = ZBUFFER_INV_Z_SCALE;


void ZBuffer_SetScaleForFrame(float max_inv_z)
{
    if (max_inv_z > 0.0000001f) {
        zbuffer_scale = ZBUFFER_TARGET_MAX_CODE / max_inv_z;
    }
    /* otherwise: keep the previous scale rather than dividing by
     * zero or overwriting with an aberrant value for a
     * degenerate frame (no valid vertex). */
}


UWORD16 ZBuffer_QuantizeInvZ(float inv_z)
{
    float   scaled;
    long    q;


    if (inv_z < 0.0f) inv_z = 0.0f;   /* safeguard, should not happen */


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


/* ZBuffer_ClearFast -- bank_lo/bank_hi obtained directly from
 * ZBuffer_Init.
 *
 * WARNING accumulator width: the bank switch
 * (lda/pha/plb) must be done in 8 bits (PLB always pulls only ONE
 * byte, whatever the width of A) -- a previous version
 * of this routine did the lda/pha in 16 bits (REP #0x30), which
 * left a stray byte on the stack at each bank switch
 * and ended up shifting the return address at the final RTL:
 * arbitrary memory corruption on function return. Fixed
 * below by a localized SEP #0x20 around each lda/pha/plb,
 * on the same principle as the local SEP already used in
 * getkeypress_openA (asm.h) for a similar need.
 */
asm void ZBuffer_ClearFast(int bank_lo, int bank_hi)
    {
        php


        phb                     // save the caller's DBR (1 byte)


        sep     #0x20           // 8 bits, so that pha/plb stay balanced (1 byte each)
        lda     bank_lo
        pha
        plb                     // DBR = first bank of the Z-buffer


        rep     #0x30           // back to 16 bits for the fill loop
        ldx     #0x0000
        lda     #0xFFFF
clear_bank_lo:
        sta     0x0000,x
        inx
        inx
        bne     clear_bank_lo   // X returns to 0 after the 65536 bytes


        sep     #0x20
        lda     bank_hi
        pha
        plb                     // DBR = second bank of the Z-buffer


        rep     #0x30
        ldx     #0x0000
        lda     #0xFFFF
clear_bank_hi:
        sta     0x0000,x
        inx
        inx
        bne     clear_bank_hi


        plb                     // restore the caller's DBR (1 byte, balanced with the initial phb)
        plp
        rtl
    }


/* ZBuffer_TestSetRow
 *
 * ARCHITECTURE NOTE (correction relative to the previous version) :
 * the initial implementation fixed the Data Bank Register once
 * (PLB) for the whole span, then addressed in absolute indexed. This is
 * WRONG in a rare but real case: with 200 lines of 640 bytes on
 * 2 banks of 64 KB, line 102 exactly straddles the bank
 * boundary in its middle (at byte 65536, i.e. pixel 128 of this
 * line). A fixed DBR for the whole span would give a wrong address
 * for the pixels located after the boundary on this precise line.
 *
 * Corrected version: long indirect addressing [ptr],y. This form
 * of 65816 addressing does a TRUE 24-bit addition (carry into
 * the bank byte included) on each access -- therefore correct no matter
 * where the bank boundary falls, without needing
 * to know in advance whether the span straddles it. Cost: the far
 * pointer must reside in direct page (current $00 zone) or be a
 * normal ORCA/C pointer (already 24 bits, see zbuffer_fullscreen.h),
 * which the
 * zbuf_ptr parameter is assumed to satisfy (ORCA/C generally places
 * asm function parameters in a zone accessible in direct page
 * -- TO BE VERIFIED by compilation, as already noted in the .h).
 */
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
        cmp     [zbuf_ptr],y    ; A (current z, "inverted distance" code) vs stored
        bcs     zsr_skip        ; A >= stored -> farther/equal, skip
        sta     [zbuf_ptr],y    ; A < stored  -> closer, update
zsr_skip:
        clc
        adc     dz_step         ; advance the interpolated depth (2's complement)
        iny
        iny
        dex
        bne     zsr_loop


zsr_done:
        plp
        rtl
    }


/* --------------------------------------------------------------------
 * INTEGRATION NOTES
 *
 * 1) See renderModelFullscreenZBuffer (separate file) for the
 *    port of renderModelScanlineZBuffer_fast to this module:
 *    ZBuffer_Init() once at startup, ZBuffer_Clear() once
 *    per FRAME (no longer per line), ZBuffer_QuantizeInvZ() at both
 *    ends of each span to obtain z_start/dz_step, then
 *    ZBuffer_TestAndSet() + existing drawPixel() for each pixel --
 *    simple and correct version provided first; see note (3)
 *    for the follow-up.
 *
 * 2) The widened edges that previously overflowed onto the neighboring
 *    line (per-line Z-buffer, never resolved) can now
 *    be tested honestly: the depth of neighboring lines
 *    still exists in memory at the moment they are drawn.
 *
 * 3) ZBuffer_TestSetRow exists for a later, faster pass,
 *    when you want to merge depth test AND color
 *    write (nibble packing SHR, cf. drawPixel) into a single
 *    assembler loop. Real unresolved point of attention here: the Z-buffer
 *    naturally works in 16 bits (accumulator A in REP
 *    #0x30 mode) while the nibble write in the framebuffer is in 8
 *    bits (SEP #0x20) -- mixing the two in the same loop forces
 *    alternating SEP/REP on each pixel, which has a real cost and can
 *    cancel out a good part of the gain. Probably faster
 *    alternative to try: two separate passes per span -- a first
 *    100% 16-bit loop that does the depth test and sets a
 *    bit in a small visibility mask (1 bit/pixel, therefore 40
 *    bytes max for 320 pixels), then a second 100% 8-bit loop
 *    that writes the color pixels consulting only this mask. To
 *    be profiled on real hardware/emulator before choosing between the
 *    two approaches -- I have no way to measure the cycles here.
 * -------------------------------------------------------------------- */


/* ====================================================================
 * Rendering: port of renderModelScanlineZBuffer_fast/_biased (see
 * engine.c) to this full-screen Z-buffer. Pasted here (rather than
 * compiled separately) because it directly uses types/functions
 * defined in engine.c (Model3D, drawPixel, getFaceFillColor, etc.)
 * without going through a common header -- see the inclusion note at
 * the top of this file for where to paste it in engine.c.
 *
 * As in engine.c, two variants + a dispatcher:
 *  - _fast   : no anti Z-fighting bias, used when
 *              cull_back_faces is active (no coplanar front/back
 *              pair visible simultaneously in that case).
 *  - _biased : adds Z_FIGHT_BIAS to front faces (plane_d[f] > 0)
 *              so that they reliably win the Z-test against
 *              their coplanar back counterpart -- used when
 *              cull_back_faces is disabled. This was the lead
 *              left aside: without it, the two faces of a
 *              coplanar pair fight pixel by pixel according to
 *              quantization rounding, hence the color of the
 *              back face punching through in places through the
 *              front face.
 * ==================================================================== */


void renderModelFullscreenZBuffer_fast(Model3D* model) {
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;
    int fcount = faces->face_count;
    int y, f, i;


    /* --- 1/z per vertex (unchanged) --- */
    static float* inv_z = NULL;
    static int inv_z_capacity = 0;
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
        /* Recompute the quantization scale on the TRUE max of
         * this frame (hence of this zoom level), rather than on a
         * frozen constant -- see zbuffer_fullscreen.h. */
        ZBuffer_SetScaleForFrame(frame_max_inv_z);
    }


#ifdef ZBUF_DIAG_SCALE
    /* --- Temporary diagnostic to calibrate ZBUFFER_INV_Z_SCALE ---
     * Writes only ONCE (static zdiag_logged) so as not to
     * add disk I/O on every frame (already a sensitive point of the
     * project). Compile with ZBUF_DIAG_SCALE defined, run on
     * a few representative scenes/models (ideally the worst case
     * in terms of depth range), pick up "zdiag.txt" each
     * time (it is overwritten, not accumulated), then remove this block and
     * ZBUF_DIAG_SCALE once ZBUFFER_INV_Z_SCALE is recalibrated. Adapt
     * fopen/fprintf to your existing file log mechanism if
     * different (reminder: stdout redirection not available under
     * ORCA/C on this target). */
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


    /* Clipping bounds in "model space" (unchanged) */
    int clip_x_min = -pan_dx;
    int clip_x_max = SCREEN_WIDTH - 1 - pan_dx;


    /* FIXED BUG (inherited as-is from renderModelScanlineZBuffer_fast/
     * _biased): looping "y" over [0, SCREEN_HEIGHT) in model space
     * does NOT cover the whole screen as soon as pan_dy != 0. Concrete example:
     * pan_dy = -60 -> screen line 199 would need y=259, never
     * reached since the loop stops at y=199 -- which leaves a
     * band of |pan_dy| lines uncovered at the bottom (or top, depending
     * on the sign), even if the model does have content there.
     * The bounds must follow pan_dy so that screenY covers
     * exactly [0, SCREEN_HEIGHT-1] by construction -- to be reported
     * also in renderModelScanlineZBuffer_fast/_biased (same loop,
     * same defect, probably just less noticed until now). */
    int y_lo = -pan_dy;
    int y_hi = SCREEN_HEIGHT - 1 - pan_dy;


    /* Full-screen Z-buffer: a single clear for the WHOLE
     * frame, before scanning the lines -- no more reset per line. */
    ZBuffer_Clear();


    for (y = y_lo; y <= y_hi; y++) {
        int screenY = y + pan_dy;   /* always in [0, SCREEN_HEIGHT-1] by construction */


        for (f = 0; f < fcount; f++) {
            int n, offt, k, hit_count;


            if (!faces->display_flag[f]) continue;
            n = faces->vertex_count[f];
            if (n < 3) continue;
            if (y < faces->miny[f] || y > faces->maxy[f]) continue;


            int fillColor = getFaceFillColor(f);
            int frameColor = getFaceFrameColor(f, fillColor);


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


            int p;
            for (p = 0; p + 1 < hit_count; p += 2) {
                int xa = hits[p].x;
                int xb = hits[p + 1].x;
                float iza = hits[p].inv_z;
                float izb = hits[p + 1].inv_z;


                if (xa > xb) continue;
                if (xb < clip_x_min || xa > clip_x_max) continue;


                int x0 = xa;
                int x1 = xb;
                float iz0 = iza;


                if (x0 == x1) {
                    int sx0 = x0 + pan_dx;
                    UWORD16 zq = ZBuffer_QuantizeInvZ(iz0);
                    if (ZBuffer_TestAndSet(sx0, screenY, zq)) {
                        drawPixel(sx0, screenY, frameColor);
                    }
                    continue;
                }


                float dIz = (izb - iza) / (float)(xb - xa);


                if (x0 < clip_x_min) {
                    iz0 += dIz * (clip_x_min - x0);
                    x0 = clip_x_min;
                }
                if (x1 > clip_x_max) x1 = clip_x_max;
                if (x0 > x1) continue;


                int sx0 = x0 + pan_dx;
                int sx1 = x1 + pan_dx;


                if (x0 == x1) {
                    UWORD16 zq = ZBuffer_QuantizeInvZ(iz0);
                    if (ZBuffer_TestAndSet(sx0, screenY, zq)) {
                        drawPixel(sx0, screenY, frameColor);
                    }
                } else {
                    /* First pixel = outline */
                    UWORD16 zq0 = ZBuffer_QuantizeInvZ(iz0);
                    if (ZBuffer_TestAndSet(sx0, screenY, zq0)) {
                        drawPixel(sx0, screenY, frameColor);
                    }


                    /* Last pixel = outline -- exact depth at
                     * the clip, computed directly in float (as in
                     * the scanline version) rather than accumulated pixel
                     * by pixel. */
                    {
                        float   iz_end_f = iz0 + dIz * (float)(x1 - x0);
                        UWORD16 zq_end   = ZBuffer_QuantizeInvZ(iz_end_f);


                        /* Middle = fill. Optimization: the quantization
                         * (float->16 bits, costs a multiplication +
                         * rounding + clamp in SOFTWARE on 65816, no
                         * FPU) is no longer done ONLY AT THE TWO ENDS of
                         * the span, not per pixel. Inside the span, we
                         * advance by an INTEGER 16-bit step (a simple
                         * addition, native and fast) instead of
                         * recomputing float->16 bits on every pixel --
                         * this was the very likely cause of the measured
                         * 2-3x slowdown relative to the scanline version. */
                        int     nsteps = x1 - x0;
                        SWORD16 dz_step_int = (SWORD16)(((long)zq_end - (long)zq0) / nsteps);
                        UWORD16 zq_cur = zq0;
                        int     x;


                        for (x = x0 + 1; x < x1; x++) {
                            int screenX = x + pan_dx;
                            zq_cur = (UWORD16)(zq_cur + dz_step_int);
                            if (ZBuffer_TestAndSet(screenX, screenY, zq_cur)) {
                                drawPixel(screenX, screenY, fillColor);
                            }
                        }


                        if (ZBuffer_TestAndSet(sx1, screenY, zq_end)) {
                            drawPixel(sx1, screenY, frameColor);
                        }
                    }
                }
            }
        }
    }
}


/* renderModelFullscreenZBuffer_biased -- identical to _fast, only
 * difference: a bias (Z_FIGHT_BIAS, same value as in engine.c)
 * is added to the interpolated depth of FRONT faces
 * (faces->plane_d[f] > 0), computed once per face, before
 * quantization. This function is only called when
 * cull_back_faces is disabled (see the dispatcher below): it is
 * the only case where the two faces of a coplanar pair can be
 * visible at the same time and fight over the Z-test.
 */
void renderModelFullscreenZBuffer_biased(Model3D* model) {
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;
    int fcount = faces->face_count;
    int y, f, i;
    static const float Z_FIGHT_BIAS = 0.001f;  /* same value as engine.c */


    static float* inv_z = NULL;
    static int inv_z_capacity = 0;
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


    int clip_x_min = -pan_dx;
    int clip_x_max = SCREEN_WIDTH - 1 - pan_dx;


    int y_lo = -pan_dy;
    int y_hi = SCREEN_HEIGHT - 1 - pan_dy;


    ZBuffer_Clear();


    for (y = y_lo; y <= y_hi; y++) {
        int screenY = y + pan_dy;


        for (f = 0; f < fcount; f++) {
            int n, offt, k, hit_count;


            if (!faces->display_flag[f]) continue;
            n = faces->vertex_count[f];
            if (n < 3) continue;
            if (y < faces->miny[f] || y > faces->maxy[f]) continue;


            int fillColor = getFaceFillColor(f);
            int frameColor = getFaceFrameColor(f, fillColor);


            /* Anti Z-fighting bias -- this function is only called
             * when cull_back_faces is disabled (see the
             * dispatcher), so no need to re-test
             * cull_back_faces here: always relevant to apply the
             * bias on front faces. */
            float depthBias = (faces->plane_d[f] > 0) ? Z_FIGHT_BIAS : 0.0f;


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


                int x1 = vtx->x2d[vid1];
                int x2 = vtx->x2d[vid2];


                float t = (float)(y - y1) / (float)dy;
                int xi = x1 + (int)((x2 - x1) * t + 0.5f);
                float izf = inv_z[vid1] + (inv_z[vid2] - inv_z[vid1]) * t;
                izf += depthBias;  /* bias applied once per intersection */


                if (hit_count < MAX_SPAN_INTERSECTIONS) {
                    hits[hit_count].x = xi;
                    hits[hit_count].inv_z = izf;
                    hit_count++;
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


            int p;
            for (p = 0; p + 1 < hit_count; p += 2) {
                int xa = hits[p].x;
                int xb = hits[p + 1].x;
                float iza = hits[p].inv_z;
                float izb = hits[p + 1].inv_z;


                if (xa > xb) continue;
                if (xb < clip_x_min || xa > clip_x_max) continue;


                int x0 = xa;
                int x1 = xb;
                float iz0 = iza;


                if (x0 == x1) {
                    int sx0 = x0 + pan_dx;
                    UWORD16 zq = ZBuffer_QuantizeInvZ(iz0);
                    if (ZBuffer_TestAndSet(sx0, screenY, zq)) {
                        drawPixel(sx0, screenY, frameColor);
                    }
                    continue;
                }


                float dIz = (izb - iza) / (float)(xb - xa);


                if (x0 < clip_x_min) {
                    iz0 += dIz * (clip_x_min - x0);
                    x0 = clip_x_min;
                }
                if (x1 > clip_x_max) x1 = clip_x_max;
                if (x0 > x1) continue;


                int sx0 = x0 + pan_dx;
                int sx1 = x1 + pan_dx;


                if (x0 == x1) {
                    UWORD16 zq = ZBuffer_QuantizeInvZ(iz0);
                    if (ZBuffer_TestAndSet(sx0, screenY, zq)) {
                        drawPixel(sx0, screenY, frameColor);
                    }
                } else {
                    UWORD16 zq0 = ZBuffer_QuantizeInvZ(iz0);
                    if (ZBuffer_TestAndSet(sx0, screenY, zq0)) {
                        drawPixel(sx0, screenY, frameColor);
                    }


                    {
                        float   iz_end_f = iz0 + dIz * (float)(x1 - x0);
                        UWORD16 zq_end   = ZBuffer_QuantizeInvZ(iz_end_f);


                        int     nsteps = x1 - x0;
                        SWORD16 dz_step_int = (SWORD16)(((long)zq_end - (long)zq0) / nsteps);
                        UWORD16 zq_cur = zq0;
                        int     x;


                        for (x = x0 + 1; x < x1; x++) {
                            int screenX = x + pan_dx;
                            zq_cur = (UWORD16)(zq_cur + dz_step_int);
                            if (ZBuffer_TestAndSet(screenX, screenY, zq_cur)) {
                                drawPixel(screenX, screenY, fillColor);
                            }
                        }


                        if (ZBuffer_TestAndSet(sx1, screenY, zq_end)) {
                            drawPixel(sx1, screenY, frameColor);
                        }
                    }
                }
            }
        }
    }
}


/* --- Dispatcher, same logic as renderModelScanlineZBuffer in
 * engine.c --------------------------------------------------------- */
void renderModelFullscreenZBuffer(Model3D* model) {
    if (cull_back_faces) {
        renderModelFullscreenZBuffer_fast(model);
    } else {
        renderModelFullscreenZBuffer_biased(model);
    }
}