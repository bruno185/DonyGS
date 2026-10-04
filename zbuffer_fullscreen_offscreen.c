/* zbuffer_fullscreen_offscreen.c
 *
 * Off-screen variant of the full-screen Z-buffer renderer: the model is drawn into
 * a 32000-byte buffer in fast RAM and copied to the SHR screen ($E1:2000) in one
 * block move, so the slow screen memory is written only once per frame.
 *
 * USAGE
 *   Offscreen_Init()                       once at start-up
 *   Offscreen_Clear()                      before each frame
 *   renderModelFullscreenZBuffer_offscreen(model)   draw into the buffer
 *   Offscreen_FlushToScreen()              copy the buffer to the screen
 *   Offscreen_Shutdown()                   on exit
 * The Z-buffer itself (zbuf_row[], ZBuffer_Init, ZBuffer_Clear, quantisation of
 * inv_z, ZBuffer_SetScaleForFrame) lives in zbuffer_fullscreen.c.
 *
 * OFFSCREEN BUFFER
 *   200 rows x 160 bytes, two 4-bit pixels per byte (even x = high nibble), in a
 *   locked block that never crosses a bank boundary.  That guarantees that the
 *   block-move instruction (MVN) works and that a pixel pointer can advance with a
 *   plain 16-bit increment.
 *
 * DEPTH ENCODING
 *   Every vertex gets inv_z = 1/z.  inv_z is linear in screen space, so it can be
 *   interpolated along edges and spans with plain additions.  It is quantised to a
 *   16-bit code
 *       zq = 0xFFFF - round(inv_z * scale)
 *   SMALLER zq = CLOSER, and 0xFFFF means "empty".  The scale is recomputed every
 *   frame from the largest inv_z so the whole 16-bit range is used.
 *
 * FRAME PIPELINE  (Off_RenderCore)
 *   1. per vertex   1/z, scale, quantise            -> vzq[]
 *   2. per face     visibility, colours, edge set-up in 12-bit fixed point
 *   3. counting sort of the faces by first visible scanline
 *   4. per scanline activate the faces that start here, advance their edges,
 *                   sort the intersections, pair them up into spans
 *   5. per span     left pixel + interior + right pixel are written by
 *                   Off_SpanRun (assembler) with a strict "closer wins" test;
 *                   gap bridges join the span ends of consecutive scanlines
 *
 * BORDERS
 *   frameColor on the left end of a span and, on true silhouettes only, on the
 *   right end; frameColor on the whole span on the first and last scanline of a
 *   face; fillColor elsewhere.  Between two consecutive scanlines the span ends are
 *   joined by a bridge so that shallow diagonals are solid instead of dotted.  A
 *   bridge NEVER writes a closer depth (that punched holes through other surfaces);
 *   it only reinforces pixels that already belong to the same surface.
 */

segment "ZBUF";
#include "zbuffer_fullscreen.h"

#define OFF_PIXELS  32000           /* 200 rows * 160 bytes: the SHR pixel area      */
#define OFF_ROWS    200             /* must equal SCREEN_HEIGHT                      */
#define OFF_FRAC    12              /* fractional bits of the edge fixed point       */
#define OFF_ROUND   0x800L          /* 1 << (OFF_FRAC - 1): the +0.5 of a rounding   */
#define OFF_Z_BIAS  0.0005f         /* inv_z bias that pulls "biased" faces closer   */


/* Offscreen buffer: a locked handle that never crosses a bank boundary.  The bank
 * and the offset are kept in globals for the assembler routines. */
Handle offscreen_handle = NULL;
Byte   offscreen_bank = 0;
Word   offscreen_offset = 0;

/* Start of every row of the buffer (valid as long as the handle stays locked). */
static unsigned char *off_row[OFF_ROWS];
static int off_ready = 0;


/* Allocates the buffer and builds the row table.  Returns 1 on success, 0 on failure. */
int Offscreen_Init(void)
{
    Pointer p;
    unsigned char *base;
    int i;
    off_ready = 0;
    offscreen_handle = NewHandle(OFF_PIXELS, userid(), attrLocked | attrNoCross, 0L);
    if (offscreen_handle == NULL || toolerror()) return 0;
    p = *offscreen_handle;
    offscreen_bank = (Byte)(((long)p >> 16) & 0xFF);
    offscreen_offset = (Word)((long)p & 0xFFFF);
    base = (unsigned char *)p;
    for (i = 0; i < OFF_ROWS; ++i) off_row[i] = base + (long)i * 160L;
    off_ready = 1;
    return 1;
}


void Offscreen_Shutdown(void)
{
    off_ready = 0;
    if (offscreen_handle != NULL) {
        DisposeHandle(offscreen_handle);
        offscreen_handle = NULL;
    }
}


/* Clears the buffer to colour 0 (both nibbles zero).  The first byte is zeroed in
 * C, then an overlapping MVN (source = base, destination = base + 1) propagates it
 * over the whole buffer. */
void Offscreen_Clear(void)
{
    unsigned char *p;
    if (offscreen_handle == NULL) return;
    p = (unsigned char *)*offscreen_handle;
    *p = 0;
    asm {
        /* Fill by an overlapping block move: the first byte (already 0) is copied to the */
        /* second, the second to the third, and so on, which propagates the 0 over the whole */
        /* buffer.  Source and destination are in the same bank, so both operand bytes of the */
        /* MVN below are patched with that bank (self-modifying code). */
        php
        phb
        sep #0x20
        lda offscreen_bank
        sta >clear_mvn+1
        sta >clear_mvn+2
        /* X = source = start of the buffer, Y = destination = start + 1, */
        /* A = number of bytes to move - 1 (31999 bytes: every byte except the first). */
        rep #0x30
        lda offscreen_offset
        tax
        tay
        iny
        lda #OFF_PIXELS-2
    clear_mvn:
        mvn 0x00,0x00
        plb
        plp
    }
}


/* Copies the buffer to the visible SHR screen ($E1:2000) with one patched MVN. */
void Offscreen_FlushToScreen(void)
{
    if (offscreen_handle == NULL) return;
    asm {
        php
        phb
        rep #0x10
        sep #0x20

        /* Patch the MVN operands: first operand byte = destination bank ($E1, the visible */
        /* SHR screen), second = source bank (the offscreen buffer). */
        lda offscreen_bank
        sta >flush_mvn+2
        lda #0xE1
        sta >flush_mvn+1

        /* X = source offset (buffer), Y = destination offset ($2000 = start of the SHR */
        /* pixel area), A = number of bytes to copy - 1 (the 32000 pixel bytes only, so the */
        /* scan-line control bytes and palettes at $9D00 and above are left alone). */
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


/* Plots one pixel into the buffer (two pixels per byte, even x = high nibble).
 * Used by the gap bridges only; the span painter writes pixels itself. */
static inline void Offscreen_DrawPixel(int x, int y, int color)
{
    unsigned char *pp;
    if ((unsigned)x >= (unsigned)SCREEN_WIDTH || (unsigned)y >= (unsigned)SCREEN_HEIGHT) return;
    pp = off_row[y] + (x >> 1);
    if ((x & 1) == 0)
        *pp = (unsigned char)((*pp & 0x0F) | ((color & 0x0F) << 4));
    else
        *pp = (unsigned char)((*pp & 0xF0) | (color & 0x0F));
}


/* The span of the previous scanline for one face (used to bridge the gaps). */
typedef struct {
    int valid;
    int y;
    int x0, x1;
    UWORD16 zq0, zq1;
} OffZBufSpanPrev;


/* ====================================================================
 * Gap bridges between the span ends of consecutive scanlines
 *
 * A bridge runs only when a span end moved by more than one pixel between two
 * scanlines.  Everything here is integer.
 * ==================================================================== */

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


/* ====================================================================
 * Fixed-point edge walking and span painting
 * ==================================================================== */

/* One intersection of a scanline with the polygon: pixel x and its depth code. */
typedef struct {
    int x;
    UWORD16 zq;
} OffScanIntersection;


/* One polygon edge, walked from its upper end to its lower end, one row at a time.
 *
 *   x, z   current position, in 12-bit fixed point:
 *            x = offset from xb (the x of the edge's first vertex)
 *            z = zq code
 *          The +0.5 rounding is already included, so reading a value needs no
 *          extra add.
 *   sx, sz change per scanline (dx/dy and dzq/dy, 12-bit fixed point)
 *   ya, yb the edge is active on rows ya <= y < yb
 *   xb     x of the edge's first vertex
 *
 * Because the state is advanced once per visited row, an edge costs two long
 * additions per scanline and no division. */
typedef struct {
    long x, z;
    long sx, sz;
    int ya, yb;
    int xb;
} OffEdge;


/* zq at abscissa x by linear interpolation between (xa, za) and (xb, zb).
 * Only used for spans cut by the horizontal clipping. */
static UWORD16 Off_LerpZ(UWORD16 za, UWORD16 zb, int xa, int xb, int x)
{
    long num;
    if (xb == xa) return za;
    num = ((long)zb - (long)za) * (long)(x - xa);
    return (UWORD16)((long)za + num / (long)(xb - xa));
}


/* ------------------------------------------------------------------------
   Paints a whole span into the offscreen buffer in assembler: left pixel +
   interior pixels + right pixel in a single call.

   The parameters are passed in global variables (no calling convention to
   respect):
     off_p_zptr / off_p_pptr : 24-bit addresses of the Z word and of the buffer
                               byte of the LEFT pixel (x0 + pan_dx)
     off_p_odd               : parity of that pixel (1 = low nibble)
     off_p_zl / off_p_zr     : exact zq of the left / right pixel
     off_p_cl / off_p_ci / off_p_cr : colour (0..15) left / interior / right
     off_p_zacc / off_p_zstep : z accumulator and step, 16.16 (high word = zq)
     off_p_n                 : number of interior pixels (>= 0)
     off_p_mode              : 0 = left pixel only, 1 = whole span
   Each pixel:  if (zq < stored z) { stored z = zq; write the nibble }
   x is already clipped, so there is no bounds test here.

   A 32-byte direct-page frame is allocated on the stack:
     0 zacc(4)  4 zstep(4)  8 zptr(3)  12 pptr(3)  16 n  18 lo  20 hi
     22 parity of the first interior pixel  24 nibble mask  26 nibble value

   The Z row is indexed with Y through [8],y, which carries into the bank byte:
   a row that straddles two banks is handled.  The buffer pointer only needs a
   16-bit increment because the buffer never crosses a bank (attrNoCross).
   ------------------------------------------------------------------------ */
long off_p_zacc, off_p_zstep;
unsigned long off_p_zptr, off_p_pptr;
int off_p_n, off_p_odd, off_p_mode, off_p_cl, off_p_ci, off_p_cr;
unsigned int off_p_zl, off_p_zr;

static void Off_SpanRun(void)
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
        lda >off_p_zacc
        sta 0
        lda >off_p_zacc+2
        sta 2
        lda >off_p_zstep
        sta 4
        lda >off_p_zstep+2
        sta 6
        lda >off_p_zptr
        sta 8
        lda >off_p_zptr+2
        sta 10
        lda >off_p_pptr
        sta 12
        lda >off_p_pptr+2
        sta 14
        lda >off_p_n
        sta 16
        /* Interior colour -> lo = 0x0N (low nibble) and hi = 0xN0 (high nibble). */
        lda >off_p_ci
        and #0x000F
        sta 18
        asl a
        asl a
        asl a
        asl a
        sta 20
        /* Parity of the first interior pixel = parity of the left pixel xor 1. */
        lda >off_p_odd
        eor #1
        sta 22
        /* Y = byte offset of the current pixel in the Z row (2 per pixel). */

        /* ---- LEFT PIXEL: exact zq (zl), colour cl ---- */
        /* A = parity -> mask (24) of the nibble to keep, value (26) to OR in. */
        ldy #0

        lda >off_p_odd
        beq sr_l_ev
        lda #0x00F0
        sta 24
        lda >off_p_cl
        and #0x000F
        sta 26
        bra sr_l_go
    sr_l_ev:
        lda #0x000F
        sta 24
        lda >off_p_cl
        and #0x000F
        asl a
        asl a
        asl a
        asl a
        sta 26
    sr_l_go:
        /* Depth test: draw only if zq is STRICTLY smaller (closer) than the stored Z. */
        lda >off_p_zl
        cmp [8],y
        bcs sr_lsk
        sta [8],y
        sep #0x20
        lda [12]
        and 24
        ora 26
        sta [12]
        rep #0x20
    sr_lsk:
        /* mode 0: left pixel only, we are done. */
        lda >off_p_mode
        bne sr_full
        brl sr_done
        /* Step over the left pixel: Z index += 2, and the pixel pointer moves to the */
        /* next byte after an odd pixel (low nibble). */
    sr_full:
        iny
        iny
        lda >off_p_odd
        beq sr_lev
        inc 12
    sr_lev:

        /* ---- INTERIOR PIXELS (n may be 0) ---- */
        lda 16
        bne sr_int
        brl sr_right
        /* If the first interior pixel is odd, do it alone so that the rest runs as */
        /* (even, odd) pairs, one byte per pair. */
    sr_int:
        lda 22
        beq sp_even

        clc
        lda 0
        adc 4
        sta 0
        lda 2
        adc 6
        sta 2
        cmp [8],y
        bcs sp_o1
        sta [8],y
        sep #0x20
        lda [12]
        and #0xF0
        ora 18
        sta [12]
        rep #0x20
    sp_o1:
        iny
        iny
        inc 12
        dec 16

        /* Pairs: X = n / 2.  Each pass does an even pixel (high nibble) then an odd */
        /* pixel (low nibble) and then moves the pixel pointer to the next byte. */
        /* z += step is a 32-bit add; the high word is the pixel's zq. */
    sp_even:
        lda 16
        lsr a
        tax
        beq sp_tail

    sp_pair:
        clc
        lda 0
        adc 4
        sta 0
        lda 2
        adc 6
        sta 2
        cmp [8],y
        bcs sp_e1
        sta [8],y
        sep #0x20
        lda [12]
        and #0x0F
        ora 20
        sta [12]
        rep #0x20
    sp_e1:
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
        bcs sp_o2
        sta [8],y
        sep #0x20
        lda [12]
        and #0xF0
        ora 18
        sta [12]
        rep #0x20
    sp_o2:
        iny
        iny
        inc 12
        dex
        bne sp_pair

        /* Tail: if n is odd, one last even pixel remains. */
    sp_tail:
        lda 16
        and #1
        bne sp_t1
        brl sr_right
    sp_t1:

        clc
        lda 0
        adc 4
        sta 0
        lda 2
        adc 6
        sta 2
        cmp [8],y
        bcs sp_e2
        sta [8],y
        sep #0x20
        lda [12]
        and #0x0F
        ora 20
        sta [12]
        rep #0x20
    sp_e2:
        iny
        iny

        /* ---- RIGHT PIXEL: exact zq (zr), colour cr ---- */
        /* Parity = (parity of first interior pixel + n) & 1. */
    sr_right:
        lda >off_p_n
        clc
        adc 22
        and #1
        beq sr_r_ev
        lda #0x00F0
        sta 24
        lda >off_p_cr
        and #0x000F
        sta 26
        bra sr_r_go
    sr_r_ev:
        lda #0x000F
        sta 24
        lda >off_p_cr
        and #0x000F
        asl a
        asl a
        asl a
        asl a
        sta 26
    sr_r_go:
        lda >off_p_zr
        cmp [8],y
        bcs sr_rsk
        sta [8],y
        sep #0x20
        lda [12]
        and 24
        ora 26
        sta [12]
        rep #0x20
    sr_rsk:

        /* Free the frame and restore D and the processor flags. */
    sr_done:
        rep #0x30
        tsc
        clc
        adc #32
        tcs
        pld
        plp
    }
}


/* Paints the span x0..x1 (already clipped) of scanline y.
 *
 *   zq0, zq_end         depth codes at x0 and x1
 *   fillColor           colour of the interior
 *   frameColor          colour of the borders
 *   whole_span_is_border  1 on the first / last scanline of a face: the whole span
 *                       is drawn in frameColor
 *   prev                span of the previous scanline of the same face (bridges)
 *   z_nudge_lsbs        number of LSBs subtracted from zq (pulls a face closer) */
static void Off_PaintSpanZ(int x0, int x1, int y, int screenY,
                           UWORD16 zq0, UWORD16 zq_end,
                           int fillColor, int frameColor,
                           int whole_span_is_border,
                           OffZBufSpanPrev *prev,
                           int z_nudge_lsbs)
{
    int nsteps, sx0, sxr, silhouette_r;
    UWORD16 zr;
    long z_acc, z_step_16;

    if (x0 > x1) return;

    /* Depth nudge: a smaller zq is closer.  Saturates at 0. */
    if (z_nudge_lsbs > 0) {
        if (zq0 > (UWORD16)z_nudge_lsbs) zq0 = (UWORD16)(zq0 - (UWORD16)z_nudge_lsbs);
        else zq0 = 0;
        if (zq_end > (UWORD16)z_nudge_lsbs) zq_end = (UWORD16)(zq_end - (UWORD16)z_nudge_lsbs);
        else zq_end = 0;
    }

    sx0 = x0 + pan_dx;      /* screen column of the left pixel */

    if (x0 == x1) {
        /* One-pixel span: just the left pixel, in frameColor. */
        zq_end = zq0;
        off_p_zptr = (unsigned long)(zbuf_row[screenY] + sx0);
        off_p_pptr = (unsigned long)(off_row[screenY] + (sx0 >> 1));
        off_p_odd  = sx0 & 1;
        off_p_zl   = zq0;
        off_p_cl   = frameColor;
        off_p_mode = 0;
        Off_SpanRun();
    } else {
        nsteps    = x1 - x0;

        /* Depth step per pixel in 16.16, so intermediate depths stay faithful to the
         * two endpoints.  The accumulator starts with +0.5 so the high word is a
         * rounded value. */
        z_step_16 = (((long)zq_end - (long)zq0) << 16) / nsteps;
        z_acc     = ((long)zq0 << 16) + 0x8000L;

        /* Right end: it gets frameColor only on a true silhouette.  If the pixel to
         * the right already holds a depth that is not clearly farther than ours (within
         * 1 LSB, or closer), the surface continues there, so the end is just fill.  An
         * empty neighbour, or one clearly farther, means we are on the silhouette.  The
         * pixels drawn below only touch columns < sxr, so reading the neighbour first
         * gives the same answer as reading it afterwards. */
        sxr = x1 + pan_dx;
        silhouette_r = 1;
        if (sxr + 1 < ZBUF_WIDTH) {
            zr = zbuf_row[screenY][sxr + 1];
            if (zr != ZBUF_FAR_VALUE && (int)zq_end - (int)zr >= -1) silhouette_r = 0;
        }

        off_p_zptr  = (unsigned long)(zbuf_row[screenY] + sx0);
        off_p_pptr  = (unsigned long)(off_row[screenY] + (sx0 >> 1));
        off_p_odd   = sx0 & 1;
        off_p_zl    = zq0;
        off_p_zr    = zq_end;
        off_p_zacc  = z_acc;
        off_p_zstep = z_step_16;
        off_p_n     = nsteps - 1;       /* interior pixels */
        off_p_cl    = frameColor;
        off_p_ci    = whole_span_is_border ? frameColor : fillColor;
        off_p_cr    = (silhouette_r || whole_span_is_border) ? frameColor : fillColor;
        off_p_mode  = 1;
        Off_SpanRun();
    }

    /* Bridge only real gaps (span end moved by more than one pixel since the previous
     * scanline); integer Z - cheap. */
    if (prev != NULL && prev->valid && prev->y == y - 1) {
        int dxl = prev->x0 - x0;
        int dxr = prev->x1 - x1;
        if (dxl < 0) dxl = -dxl;
        if (dxr < 0) dxr = -dxr;
        if (dxl > 1) Off_BridgeBorderZQ(prev->x0, prev->y, prev->zq0, x0, y, zq0, frameColor);
        if (dxr > 1) Off_BridgeBorderZQ(prev->x1, prev->y, prev->zq1, x1, y, zq_end, frameColor);
    }

    /* Remember this span for the next scanline. */
    if (prev != NULL) {
        prev->valid = 1;
        prev->y = y;
        prev->x0 = x0;
        prev->x1 = x1;
        prev->zq0 = zq0;
        prev->zq1 = zq_end;
    }
}


/* ====================================================================
 * Rendering
 *
 *  - no float inside the scanline loop: edges are walked in fixed point and
 *    zq is quantised once per vertex and per frame;
 *  - only the faces crossing the current scanline are visited (active list built
 *    with a counting sort on the first visible row);
 *  - face colours are computed once per frame;
 *  - edges live in an array of structures with ya/yb precomputed;
 *  - spans are written by a single assembler call.
 * ==================================================================== */

/* Shared renderer for the "fast" (biased = 0) and "biased" (biased = 1) variants.
 *
 * biased = 1 is used when back faces are not culled: faces with plane_d > 0 are
 * pulled slightly closer (inv_z + OFF_Z_BIAS, then 2 more LSBs of zq in
 * Off_PaintSpanZ) so that they win depth ties against coplanar or nearly
 * coplanar faces instead of z-fighting. */
static void Off_RenderCore(Model3D* model, int biased)
{
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;
    int fcount = faces->face_count;
    int y, f, i, k, b, si, ai, keep, active_count;
    int total_edges, end;
    int clip_x_min, clip_x_max, y_lo, y_hi;
    float frame_max_inv_z;
    OffEdge* ed;
    OffScanIntersection hits[MAX_SPAN_INTERSECTIONS];

    static float* inv_z = NULL;
    static UWORD16* vzq = NULL;
    static UWORD16* vzq_b = NULL;
    static int vert_capacity = 0;

    static OffZBufSpanPrev* span_prev = NULL;
    static int* face_start = NULL;          /* first scanline of the face (relative to y_lo), -1 = skipped */
    static int* face_sorted = NULL;         /* faces sorted by first scanline (counting sort)               */
    static int* face_active = NULL;         /* faces crossing the current scanline, ascending face index    */
    static unsigned char* face_fill = NULL;
    static unsigned char* face_frame = NULL;
    static unsigned char* face_nudge = NULL;
    static int face_capacity = 0;

    static OffEdge* edge_buf = NULL;
    static int edge_capacity = 0;

    /* counting-sort tables: bucket[r] .. bucket[r+1]-1 = faces starting at row r */
    static int bucket[OFF_ROWS + 1];
    static int bucket_cur[OFF_ROWS + 1];

    /* Nothing to draw into until Offscreen_Init() has succeeded. */
    if (!off_ready) return;

    /* ---- scratch buffers: they only ever grow, so after the first frame there is no malloc ---- */
    if (face_capacity < fcount) {
        if (span_prev)    free(span_prev);
        if (face_start)   free(face_start);
        if (face_sorted)  free(face_sorted);
        if (face_active)  free(face_active);
        if (face_fill)    free(face_fill);
        if (face_frame)   free(face_frame);
        if (face_nudge)   free(face_nudge);
        span_prev   = (OffZBufSpanPrev*)malloc(fcount * sizeof(OffZBufSpanPrev));
        face_start  = (int*)malloc(fcount * sizeof(int));
        face_sorted = (int*)malloc(fcount * sizeof(int));
        face_active = (int*)malloc(fcount * sizeof(int));
        face_fill   = (unsigned char*)malloc(fcount);
        face_frame  = (unsigned char*)malloc(fcount);
        face_nudge  = (unsigned char*)malloc(fcount);
        face_capacity = (span_prev && face_start && face_sorted && face_active &&
                         face_fill && face_frame && face_nudge) ? fcount : 0;
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
        if (edge_buf) free(edge_buf);
        edge_buf = (OffEdge*)malloc((size_t)total_edges * sizeof(OffEdge));
        edge_capacity = edge_buf ? total_edges : 0;
    }
    if (face_capacity < fcount || vert_capacity < vcount || edge_capacity < total_edges)
        return;

    /* No previous span yet for any face (used for the gap bridges). */
    for (f = 0; f < fcount; f++) span_prev[f].valid = 0;

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
    ZBuffer_SetScaleForFrame(frame_max_inv_z);

    for (i = 0; i < vcount; i++) {
        /* vzq_b is the "pulled closer" copy used by faces with plane_d > 0 (biased only). */
        vzq[i]   = ZBuffer_QuantizeInvZ(inv_z[i]);
        vzq_b[i] = biased ? ZBuffer_QuantizeInvZ(inv_z[i] + OFF_Z_BIAS) : vzq[i];
    }

    SetPenMode(0);
    applyPalette(palette);

    /* Visible window in model coordinates: the pan offsets (pan_dx, pan_dy) are
     * added back when addressing the screen and the Z-buffer. */
    clip_x_min = -pan_dx;
    clip_x_max = SCREEN_WIDTH - 1 - pan_dx;
    y_lo = -pan_dy;
    y_hi = SCREEN_HEIGHT - 1 - pan_dy;

    ZBuffer_Clear();

    /* ---- PER FACE, once per frame -----------------------------------------
     * - decide if the face can appear at all (shown, >= 3 vertices, overlaps the window)
     * - cache its colours and depth nudge
     * - set up one OffEdge per polygon edge (see the OffEdge comment) */
    for (b = 0; b <= OFF_ROWS; b++) bucket[b] = 0;

    for (f = 0; f < fcount; f++) {
        int n, offt, fmin, fmax, fill;
        UWORD16* zsrc;

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
            ed->x  = ex + OFF_ROUND;     /* the +0.5 rounding is baked in once, here */
            ed->z  = ez + OFF_ROUND;
            ed->sx = sx;  ed->sz = sz;
        }
    }

    /* ---- COUNTING SORT: faces ordered by their first visible scanline ----
     * (prefix sums turn the histogram into start offsets, then each face is placed). */
    for (b = 0; b < OFF_ROWS; b++) bucket[b + 1] += bucket[b];
    for (b = 0; b <= OFF_ROWS; b++) bucket_cur[b] = bucket[b];
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
            int fillColor, frameColor, on_top_or_bottom, nudge;

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
                    hits[hit_count].x  = ed->xb + ((t >= 0L) ? (int)(t >> OFF_FRAC) : -(int)((-t) >> OFF_FRAC));
                    hits[hit_count].zq = (UWORD16)(ed->z >> OFF_FRAC);
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
                    OffScanIntersection tmp = hits[0];
                    hits[0] = hits[1];
                    hits[1] = tmp;
                }
            } else {
                int a, c;
                for (a = 1; a < hit_count; a++) {
                    OffScanIntersection key = hits[a];
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
            on_top_or_bottom = (y == faces->miny[f] || y == faces->maxy[f]);

            /* Consecutive hit pairs are the inside spans of the polygon (even-odd rule).
             * Clip each span to the window; zq at a clipped end is interpolated. */
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
        active_count = keep;
    }
}


/* Public entry points.  "fast" assumes back faces were culled; "biased" is for
 * models drawn with both sides, where coplanar faces need the depth bias. */
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
