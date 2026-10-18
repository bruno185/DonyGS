
/* Memory-lifetime correction: all renderer-owned allocations are released
 * by ZBuffer_Shutdown; repeated Init is idempotent; partial allocations and
 * target size_t overflow are handled. Header and rendering arithmetic unchanged.
 */
/* FULLSCREEN-ONLY COCKPIT GUARD, based on the validated no-bleed V5.
 * Only border visibility changes: when a face-plane sample is strictly closer
 * than the existing surface, the geometric edge depth must also pass Z.
 * Fill, span assembly, allocation and shared header are unchanged.
 * Adds integer interpolation along borders; no extra full-screen buffer.
 * This is a targeted improvement, not a general hidden-line guarantee.
 */
/* zbuffer_fullscreen.c -- V5 correction: inward fill + shared depth plane.
 * Apple IIGS / ORCA C / 65816, SHR 320x200.
 *
 * Coverage: keep subpixel edge intersections; ceil the left fill boundary and
 * floor the right boundary.  Nearest-pixel Bresenham outlines surround this
 * interior.  This removes the asymmetric negative-offset rounding of V4.
 *
 * Visibility: fill AND stroke evaluate one quantised depth plane at the actual
 * raster pixel.  No slope-dependent depth tolerance is needed.  Farther edges
 * are rejected by default, including edges just behind cockpit surfaces.
 * Planar faces are assumed.  Degenerate projected faces use constant depth.
 *
 * The assembler span writer, Z allocation, adaptive scale, optional side flags
 * and public render entry points are retained.  Per-face float plane setup
 * replaces per-edge Z slope setup and per-span Z divisions.  No extra full-screen
 * buffer is allocated.  FramePoly and Bresenham tie rules can still differ.
 */

segment "ZBUF";
#include "zbuffer_fullscreen.h"


/* Row pointers: zbuf_row[y] -> first depth word (UWORD16) of row y. */
FarWordPtr zbuf_row[ZBUF_HEIGHT];


static Handle  zbuf_handle = NULL;
static int     zbuf_bank_lo = 0;     /* the two banks holding the Z-buffer */
static void ZBFS_FreeScratch(void);
static int     zbuf_bank_hi = 0;


/* The buffer needs two full banks (128 KB).  One extra bank is requested so the
 * block can be moved up to the next bank boundary, whatever address the Memory
 * Manager returns. */
#define ZBUF_USABLE_SIZE   (2UL * 65536UL)
#define ZBUF_ALLOC_SIZE    (ZBUF_USABLE_SIZE + 65536UL)


/* Allocates the Z-buffer (non-moving block), aligns it on a bank boundary, builds
 * the row table and clears it.  Returns 1 on success, 0 on failure. */
int ZBuffer_Init(void)
{
    ULONG32 raw;
    ULONG32 aligned;
    int     y;

    /* Re-entering Init when loading another model must not lose the old handle. */
    if (zbuf_handle != NULL) return 1;
    zbuf_handle = NewHandle(ZBUF_ALLOC_SIZE, userid(), attrFixed, 0L);
    if (toolerror() || zbuf_handle == NULL) {
        if (zbuf_handle != NULL) DisposeHandle(zbuf_handle);
        zbuf_handle = NULL;
        return 0;
    }

    /* Round the block address up to the next multiple of 64 KB. */
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


/* Safe after partial initialization and safe to call more than once.
 * Edge flags are borrowed: forget the pointer, never free caller-owned memory.
 */
void ZBuffer_Shutdown(void)
{
    int y;
    ZBFS_FreeScratch();
    ZBuffer_SetEdgeFlags(NULL, 0UL);
    if (zbuf_handle != NULL) {
        DisposeHandle(zbuf_handle);
        zbuf_handle = NULL;
    }
    for (y = 0; y < ZBUF_HEIGHT; y++) zbuf_row[y] = NULL;
    zbuf_bank_lo = zbuf_bank_hi = 0;
}


/* Sets every depth word to ZBUF_FAR_VALUE (0xFFFF = empty), in assembler.
 *
 * For each row: DBR is set to the bank of the row, X to its offset in that bank,
 * and 16 stores of 0xFFFF are unrolled per pass (6 cycles per word).  The one row
 * that straddles two banks (row 102 with 640 bytes per row) is detected and cleared
 * by a slower [dp],y loop, which carries into the bank byte, so the result is
 * correct wherever the rows lie.
 *
 * Requirements: ZBUF_WIDTH is a multiple of 16, zbuf_row[] holds 4-byte pointers
 * (low word, then bank in the low byte of the high word), ZBUF_FAR_VALUE fits in
 * 16 bits.  The "sta |n,x" form forces absolute addressing (otherwise the
 * assembler would pick direct page for small offsets). */
void ZBuffer_Clear(void)
{
    asm {
        /* Frame: 1 word at D+0 = byte offset of the current row in the zbuf_row table */
        /* (4 bytes per pointer).  DBR is saved (phb) because we change it per row. */
        php
        phb
        phd
        rep #0x30
        tsc
        sec
        sbc #6
        tcs
        inc a
        tcd
        lda #0
        sta 0
        /* ---- one row per pass ---- */
        /* Copy the row pointer (low word at D+2, bank at D+4). */
    zc_row:
        ldx 0
        lda >zbuf_row,x
        sta 2
        lda >zbuf_row+2,x
        sta 4
        /* Does the row cross a bank boundary?  (start + last byte offset carries) */
        /* If so use the slow path below. */
        lda 2
        clc
        adc #ZBUF_WIDTH*2-1
        bcs zc_slow
        /* FAST PATH: DBR = bank of the row, X = offset of the row in that bank, then */
        /* 16 unrolled stores of 0xFFFF per pass (6 cycles per word). */
        lda 4
        sep #0x20
        pha
        plb
        rep #0x20
        ldx 2
        lda #ZBUF_FAR_VALUE
        ldy #ZBUF_WIDTH/16
    zc_loop:
        sta |0,x
        sta |2,x
        sta |4,x
        sta |6,x
        sta |8,x
        sta |10,x
        sta |12,x
        sta |14,x
        sta |16,x
        sta |18,x
        sta |20,x
        sta |22,x
        sta |24,x
        sta |26,x
        sta |28,x
        sta |30,x
        txa
        clc
        adc #32
        tax
        lda #ZBUF_FAR_VALUE
        dey
        bne zc_loop
        bra zc_next
        /* SLOW PATH (only the row that straddles two banks): [dp],y carries into the */
        /* bank byte, so this is correct wherever the row lies. */
    zc_slow:
        ldy #0
        ldx #ZBUF_WIDTH/16
        lda #ZBUF_FAR_VALUE
    zc_sl:
        sta [2],y
        iny
        iny
        sta [2],y
        iny
        iny
        sta [2],y
        iny
        iny
        sta [2],y
        iny
        iny
        sta [2],y
        iny
        iny
        sta [2],y
        iny
        iny
        sta [2],y
        iny
        iny
        sta [2],y
        iny
        iny
        sta [2],y
        iny
        iny
        sta [2],y
        iny
        iny
        sta [2],y
        iny
        iny
        sta [2],y
        iny
        iny
        sta [2],y
        iny
        iny
        sta [2],y
        iny
        iny
        sta [2],y
        iny
        iny
        sta [2],y
        iny
        iny
        dex
        bne zc_sl
        /* Next row: advance the table offset by 4 and loop until all rows are done. */
    zc_next:
        lda 0
        clc
        adc #4
        sta 0
        cmp #ZBUF_HEIGHT*4
        beq zc_end
        brl zc_row
    zc_end:
        tsc
        clc
        adc #6
        tcs
        pld
        plb
        plp
    }
}


/* Scale that maps inv_z to the 16-bit code range; recomputed every frame. */
static float zbuffer_scale = ZBUFFER_INV_Z_SCALE;


/* Chooses the quantisation scale for this frame: the largest inv_z (the nearest
 * vertex) is mapped to ZBUFFER_TARGET_MAX_CODE, which keeps the full precision of
 * the 16 bits whatever the distance of the model. */
void ZBuffer_SetScaleForFrame(float max_inv_z)
{
    if (max_inv_z > 0.0000001f) {
        zbuffer_scale = ZBUFFER_TARGET_MAX_CODE / max_inv_z;
    }
}


/* inv_z -> 16-bit depth code.
 *   q  = round(inv_z * scale), clamped to 0..0xFFFF   (larger q = closer)
 *   zq = 0xFFFF - q                                   (smaller zq = closer)
 * so a freshly cleared buffer (0xFFFF) is "infinitely far" and any quantised
 * depth is strictly closer than it, except q = 0 which maps to 0xFFFF itself. */
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


/* The depth test: writes z and returns 1 only if it is STRICTLY closer than the
 * stored value (equal depths never overwrite).  No bounds check: x and y must be
 * inside the buffer. */
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
 * Legacy assembler helpers
 *
 * Not used by the renderer in this file.  They are exported functions, so they
 * were kept in case another source file calls them; delete them (and
 * zbuf_bank_lo / zbuf_bank_hi) if nothing does.
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
 * Fixed-point edge walking and span painting
 * ==================================================================== */

#define ZB_Z_BIAS    0.0005f         /* inv_z bias that pulls "biased" faces closer */
#define ZB_SHR_BASE  0xE12000UL      /* SHR screen memory */


/* One intersection of a scanline with the polygon: pixel x and its depth code. */
typedef struct {
    long x;                 /* absolute model x, 12 fraction bits */
} ZbHit;


/* One polygon edge, walked top to bottom. x is a subpixel offset from xb,
 * sx is dx/dy, both with 12 fraction bits and NO baked-in rounding. The edge
 * participates on ya <= y < yb. Depth comes from the face plane, not this edge.
 */
typedef struct {
    long x;
    long sx;
    int  ya, yb;
    int  xb;
} ZbEdge;


/* ------------------------------------------------------------------------
   Paints a whole span straight into the SHR memory ($E1:2000) in assembler:
   left pixel + interior pixels + right pixel in a single call.

   drawPixel() must do exactly the same thing (320 mode, even pixel = high
   nibble, base $E12000, 160 bytes per line).

   The parameters are passed in global variables (no calling convention to
   respect):
     zb_p_zptr / zb_p_pptr : 24-bit addresses of the Z word and of the screen
                             byte of the LEFT pixel (x0 + pan_dx)
     zb_p_odd              : parity of that pixel (1 = low nibble)
     zb_p_zl / zb_p_zr     : exact zq of the left / right pixel
     zb_p_cl / zb_p_ci / zb_p_cr : colour (0..15) left / interior / right
     zb_p_zacc / zb_p_zstep : z accumulator and step, 16.16 (high word = zq)
     zb_p_n                : number of interior pixels (>= 0)
     zb_p_mode             : 0 = left pixel only, 1 = whole span
   Each pixel:  if (zq < stored z) { stored z = zq; write the nibble }
   x is already clipped, so there is no bounds test here.

   A 32-byte direct-page frame is allocated on the stack:
     0 zacc(4)  4 zstep(4)  8 zptr(3)  12 pptr(3)  16 n  18 lo  20 hi
     22 parity of the first interior pixel  24 nibble mask  26 nibble value

   The Z row is indexed with Y through [8],y, which carries into the bank byte:
   a row that straddles two banks is handled.  The screen pointer only needs a
   16-bit increment because $2000-$9CFF never crosses a bank.
   ------------------------------------------------------------------------ */
long zb_p_zacc, zb_p_zstep;
unsigned long zb_p_zptr, zb_p_pptr;
int zb_p_n, zb_p_odd, zb_p_mode, zb_p_cl, zb_p_ci, zb_p_cr;
unsigned int zb_p_zl, zb_p_zr;

static void ZBuffer_SpanRun(void)
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
        lda >zb_p_zacc
        sta 0
        lda >zb_p_zacc+2
        sta 2
        lda >zb_p_zstep
        sta 4
        lda >zb_p_zstep+2
        sta 6
        lda >zb_p_zptr
        sta 8
        lda >zb_p_zptr+2
        sta 10
        lda >zb_p_pptr
        sta 12
        lda >zb_p_pptr+2
        sta 14
        lda >zb_p_n
        sta 16
        /* Interior colour -> lo = 0x0N (low nibble) and hi = 0xN0 (high nibble). */
        lda >zb_p_ci
        and #0x000F
        sta 18
        asl a
        asl a
        asl a
        asl a
        sta 20
        /* Parity of the first interior pixel = parity of the left pixel xor 1. */
        lda >zb_p_odd
        eor #1
        sta 22
        /* Y = byte offset of the current pixel in the Z row (2 per pixel). */
        /*  */
        /* ---- LEFT PIXEL: exact zq (zl), colour cl ---- */
        /* A = parity -> mask (24) of the nibble to keep, value (26) to OR in. */
        ldy #0

        lda >zb_p_odd
        beq zbr_l_ev
        lda #0x00F0
        sta 24
        lda >zb_p_cl
        and #0x000F
        sta 26
        bra zbr_l_go
    zbr_l_ev:
        lda #0x000F
        sta 24
        lda >zb_p_cl
        and #0x000F
        asl a
        asl a
        asl a
        asl a
        sta 26
    zbr_l_go:
        /* Depth test: draw only if zq is STRICTLY smaller (closer) than the stored Z. */
        lda >zb_p_zl
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
        lda >zb_p_mode
        bne zbr_full
        brl zbr_done
        /* Step over the left pixel: Z index += 2, and the pixel pointer moves to the */
        /* next byte after an odd pixel (low nibble). */
    zbr_full:
        iny
        iny
        lda >zb_p_odd
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
        lda >zb_p_n
        clc
        adc 22
        and #1
        beq zbr_r_ev
        lda #0x00F0
        sta 24
        lda >zb_p_cr
        and #0x000F
        sta 26
        bra zbr_r_go
    zbr_r_ev:
        lda #0x000F
        sta 24
        lda >zb_p_cr
        and #0x000F
        asl a
        asl a
        asl a
        asl a
        sta 26
    zbr_r_go:
        lda >zb_p_zr
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


/* A single quantised depth plane is shared by a face's fill and outline.
 * Coordinates refer to SCREEN pixels, including pan.  Coefficients use 12
 * fraction bits; all per-pixel arithmetic is integer.  Unsigned accumulators
 * intentionally wrap at 32 bits, so cancellation in plane evaluation is safe.
 */
typedef struct {
    long gx, gy;
    long x, y;
    UWORD16 z;
} ZbPlane;

static float* zbfs_inv_z = NULL;
static UWORD16* zbfs_vzq = NULL;
static UWORD16* zbfs_vzq_b = NULL;
static int zbfs_vert_capacity = 0;

static int* zbfs_face_start = NULL;          /* first scanline of the face (relative to y_lo), -1 = skipped */
static int* zbfs_face_sorted = NULL;         /* faces sorted by first scanline (counting sort)               */
static int* zbfs_face_active = NULL;         /* faces crossing the current scanline, ascending face index    */
static unsigned char* zbfs_face_fill = NULL;
static unsigned char* zbfs_face_frame = NULL;
static unsigned char* zbfs_face_nudge = NULL;
static ZbPlane *zbfs_face_plane = NULL;
static int zbfs_face_capacity = 0;

static ZbEdge* zbfs_edge_buf = NULL;
static int zbfs_edge_capacity = 0;


static void ZBFS_FreeFaces(void)
{
    if (zbfs_face_start) free(zbfs_face_start);
    if (zbfs_face_sorted) free(zbfs_face_sorted);
    if (zbfs_face_active) free(zbfs_face_active);
    if (zbfs_face_fill) free(zbfs_face_fill);
    if (zbfs_face_frame) free(zbfs_face_frame);
    if (zbfs_face_nudge) free(zbfs_face_nudge);
    if (zbfs_face_plane) free(zbfs_face_plane);
    zbfs_face_start = zbfs_face_sorted = zbfs_face_active = NULL;
    zbfs_face_fill = zbfs_face_frame = zbfs_face_nudge = NULL;
    zbfs_face_plane = NULL;
    zbfs_face_capacity = 0;
}

static void ZBFS_FreeVertices(void)
{
    if (zbfs_inv_z) free(zbfs_inv_z);
    if (zbfs_vzq) free(zbfs_vzq);
    if (zbfs_vzq_b) free(zbfs_vzq_b);
    zbfs_inv_z = NULL;
    zbfs_vzq = zbfs_vzq_b = NULL;
    zbfs_vert_capacity = 0;
}

static void ZBFS_FreeEdges(void)
{
    if (zbfs_edge_buf) free(zbfs_edge_buf);
    zbfs_edge_buf = NULL;
    zbfs_edge_capacity = 0;
}

static void ZBFS_FreeScratch(void)
{
    ZBFS_FreeFaces();
    ZBFS_FreeVertices();
    ZBFS_FreeEdges();
}

/* Check BEFORE multiplying: size_t may be only 16 bits on the GS. */
static int ZBFS_SizeFits(int count, size_t element_size)
{
    return count > 0 && (unsigned long)count <=
           (unsigned long)((size_t)-1) / (unsigned long)element_size;
}

static void ZBuffer_SetPlane(ZbPlane *p, VertexArrays3D *vtx,
                             FaceArrays3D *faces, int f, UWORD16 *zsrc,
                             int nudge)
{
    int n, off, a, b, c, k;
    float ax, ay, az, bx, by, bz, det, gx, gy;
    off = faces->vertex_indices_ptr[f];
    n = faces->vertex_count[f];
    a = faces->vertex_indices_buffer[off] - 1;
    p->x = (long)vtx->x2d[a] + (long)pan_dx;
    p->y = (long)vtx->y2d[a] + (long)pan_dy;
    p->z = (zsrc[a] > (UWORD16)nudge) ? (UWORD16)(zsrc[a] - nudge) : 0;
    p->gx = p->gy = 0L;
    for (k = 1; k + 1 < n; k++) {
        b = faces->vertex_indices_buffer[off + k] - 1;
        c = faces->vertex_indices_buffer[off + k + 1] - 1;
        ax = (float)vtx->x2d[b] - (float)vtx->x2d[a];
        ay = (float)vtx->y2d[b] - (float)vtx->y2d[a];
        az = (float)zsrc[b] - (float)zsrc[a];
        bx = (float)vtx->x2d[c] - (float)vtx->x2d[a];
        by = (float)vtx->y2d[c] - (float)vtx->y2d[a];
        bz = (float)zsrc[c] - (float)zsrc[a];
        det = ax * by - ay * bx;
        if (det == 0.0f) continue;
        gx = ((az * by - ay * bz) / det) * 4096.0f;
        gy = ((ax * bz - az * bx) / det) * 4096.0f;
        /* Guard conversion for pathological projected planes. */
        if (gx > 2147483000.0f) gx = 2147483000.0f;
        if (gx < -2147483000.0f) gx = -2147483000.0f;
        if (gy > 2147483000.0f) gy = 2147483000.0f;
        if (gy < -2147483000.0f) gy = -2147483000.0f;
        p->gx = (long)((gx < 0.0f) ? gx - 0.5f : gx + 0.5f);
        p->gy = (long)((gy < 0.0f) ? gy - 0.5f : gy + 0.5f);
        return;
    }
}

static unsigned long ZBuffer_PlaneAt(ZbPlane *p, long x, long y)
{
    return (((unsigned long)p->z << 12) +
            (unsigned long)p->gx * (unsigned long)(x - p->x) +
            (unsigned long)p->gy * (unsigned long)(y - p->y)) & 0xFFFFFFFFUL;
}

static UWORD16 ZBuffer_PlaneCode(unsigned long z)
{
    z = (z + 0x800UL) & 0xFFFFFFFFUL;
    if (z & 0x80000000UL) return 0;
    z >>= 12;
    return (z > 65535UL) ? ZBUF_FAR_VALUE : (UWORD16)z;
}

/* Fill at the ACTUAL raster pixel positions, using the same depth plane as
 * the final stroke.  The assembler span writer is unchanged.  Its 16.16 step
 * is derived exactly from our 12-bit coefficient: no division per span and
 * no disagreement caused by interpolating between rounded edge positions.
 */
static void ZBuffer_PaintSpanZ(int x0, int x1, int screenY,
                               int fillColor, ZbPlane *plane)
{
    int sx0, nsteps;
    unsigned long raw;
    if (x0 > x1) return;
    sx0 = x0 + pan_dx;
    raw = ZBuffer_PlaneAt(plane, (long)sx0, (long)screenY);
    zb_p_zptr = (unsigned long)(zbuf_row[screenY] + sx0);
    zb_p_pptr = ZB_SHR_BASE + (unsigned long)screenY * 160UL +
                (unsigned long)(sx0 >> 1);
    zb_p_odd = sx0 & 1;
    zb_p_zl = ZBuffer_PlaneCode(raw);
    zb_p_cl = fillColor;
    zb_p_mode = 0;
    if (x0 != x1) {
        nsteps = x1 - x0;
        zb_p_zstep = (long)(((unsigned long)plane->gx << 4) & 0xFFFFFFFFUL);
        zb_p_zacc = (long)(((raw << 4) + 0x8000UL) & 0xFFFFFFFFUL);
        zb_p_zr = ZBuffer_PlaneCode(raw +
                      (unsigned long)plane->gx * (unsigned long)nsteps);
        zb_p_n = nsteps - 1;
        zb_p_ci = fillColor;
        zb_p_cr = fillColor;
        zb_p_mode = 1;
    }
    ZBuffer_SpanRun();
}


/* Optional flags use the same flat indexing as vertex_indices_buffer:
 * entry off+k controls vertex k -> vertex (k+1)%n of that face.
 * NULL means every original polygon edge.  The caller owns this array and
 * must reset it with (NULL, 0UL) before freeing it or changing model layout.
 * Entries beyond count default to enabled.
 */
static const unsigned char *zb_edge_flags = NULL;
static unsigned long zb_edge_flag_count = 0UL;

void ZBuffer_SetEdgeFlags(const unsigned char *flags, unsigned long count)
{
    zb_edge_flags = flags;
    zb_edge_flag_count = count;
}


/* No slope-dependent tolerance: fill and stroke now evaluate the SAME plane
 * at the SAME pixel.  Equal depth is visible; a farther edge is rejected.
 * A line pixel outside fill coverage updates depth only when strictly closer.
 */
static void ZBuffer_PlotEdge(int x, int y, UWORD16 zq,
                              UWORD16 edge_zq, int color)
{
    FarWordPtr row;
    UWORD16 cur;
    if ((unsigned)x >= (unsigned)ZBUF_WIDTH ||
        (unsigned)y >= (unsigned)ZBUF_HEIGHT || zq == ZBUF_FAR_VALUE)
        return;
    row = zbuf_row[y];
    cur = row[x];
    if (zq > cur) return;
    /* A plane extrapolated to an off-edge raster pixel can appear closer
     * than an unrelated surface. Require the geometric edge itself to pass
     * Z too. Equality with the face plane already stored by the fill is kept:
     * this preserves the face's own visible one-pixel outline.
     * No slope tolerance, neighbour sampling, or owner buffer is introduced.
     */
    if (cur != ZBUF_FAR_VALUE && zq < cur && edge_zq > cur) return;
    if (zq < cur) row[x] = zq;
    drawPixel(x, y, color);
}


/* Canonical Bresenham stroke.  Major-axis clipping retains the original
 * line phase.  Depth follows the face plane with additions for major/minor
 * coordinate steps, instead of interpolating depth along an ideal line that
 * does not pass through the actual raster pixel centres.
 */
static void ZBuffer_DrawEdge(int x0, int y0, UWORD16 edge_z0,
                             int x1, int y1, UWORD16 edge_z1,
                             int color, ZbPlane *plane)
{
    long dx, dy, major, minor, first, last, i;
    long u0, v0, v, ulimit, vstep, rem;
    unsigned long numerator, raw, zmajor, zminor, magnitude;
    long ez, erem, ewhole, epart, esign, delta;
    UWORD16 ztmp;
    int steep, tmp;
    dx = (long)x1 - (long)x0;
    dy = (long)y1 - (long)y0;
    if (dx < 0L) dx = -dx;
    if (dy < 0L) dy = -dy;
    steep = (dy > dx);
    if ((steep && y0 > y1) || (!steep && x0 > x1)) {
        tmp = x0; x0 = x1; x1 = tmp;
        tmp = y0; y0 = y1; y1 = tmp;
        ztmp = edge_z0; edge_z0 = edge_z1; edge_z1 = ztmp;
    }
    u0 = steep ? (long)y0 : (long)x0;
    v0 = steep ? (long)x0 : (long)y0;
    major = steep ? dy : dx;
    minor = steep ? dx : dy;
    ulimit = steep ? ZBUF_HEIGHT - 1L : ZBUF_WIDTH - 1L;
    vstep = ((steep ? x1 : y1) < v0) ? -1L : 1L;
    if (major == 0L) {
        ZBuffer_PlotEdge(x0, y0,
                         ZBuffer_PlaneCode(ZBuffer_PlaneAt(plane, x0, y0)),
                         (edge_z0 < edge_z1) ? edge_z0 : edge_z1, color);
        return;
    }
    first = (u0 < 0L) ? -u0 : 0L;
    last = major;
    if (u0 + last > ulimit) last = ulimit - u0;
    if (first > last) return;
    numerator = (unsigned long)first * (unsigned long)minor +
                (unsigned long)(major / 2L);
    v = v0 + vstep * (long)(numerator / (unsigned long)major);
    rem = (long)(numerator % (unsigned long)major);
    raw = steep ? ZBuffer_PlaneAt(plane, v, u0 + first) :
                  ZBuffer_PlaneAt(plane, u0 + first, v);
    zmajor = (unsigned long)(steep ? plane->gy : plane->gx);
    zminor = (unsigned long)(steep ? plane->gx : plane->gy);
    if (vstep < 0L) zminor = 0UL - zminor;
    /* Independently interpolate quantised 1/z along the original edge.
     * Quotient/remainder stepping is exact and fits 32-bit arithmetic.
     */
    delta = (long)edge_z1 - (long)edge_z0;
    esign = (delta < 0L) ? -1L : 1L;
    magnitude = (unsigned long)((delta < 0L) ? -delta : delta);
    ewhole = (long)(magnitude / (unsigned long)major);
    epart = (long)(magnitude % (unsigned long)major);
    numerator = (unsigned long)first * magnitude + (unsigned long)(major / 2L);
    ez = (long)edge_z0 + esign * (long)(numerator / (unsigned long)major);
    erem = (long)(numerator % (unsigned long)major);
    for (i = first; i <= last; i++) {
        if (steep)
            ZBuffer_PlotEdge((int)v, (int)(u0 + i), ZBuffer_PlaneCode(raw), (UWORD16)ez, color);
        else
            ZBuffer_PlotEdge((int)(u0 + i), (int)v, ZBuffer_PlaneCode(raw), (UWORD16)ez, color);
        ez += esign * ewhole;
        erem += epart;
        if (erem >= major) { erem -= major; ez += esign; }
        raw += zmajor;
        rem += minor;
        if (rem >= major) { rem -= major; v += vstep; raw += zminor; }
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

/* Clear SHR rows 0..11 (12 * 160 = 1920 bytes) to palette colour 0.
 * Absolute long stores target bank $E1 independently of DBR.
 * Preserve the accumulator, X and processor flags; D and Y are untouched.
 */
static void ZBFS_ClearTop12Lines(void)
{
    asm {
        php
        rep #0x30
        pha
        phx
        lda #0
        ldx #0x077E
    zbfs_ct12_loop:
        sta >0xE12000,x
        dex
        dex
        bpl zbfs_ct12_loop
        plx
        pla
        plp
    }
}


/* Shared renderer for the "fast" (biased = 0) and "biased" (biased = 1) variants.
 *
 * biased = 1 is used when back faces are not culled: faces with plane_d > 0 are
 * pulled slightly closer (inv_z + ZB_Z_BIAS, then 2 more LSBs in
 * the shared depth plane) so that they win depth ties against coplanar or nearly
 * coplanar faces instead of z-fighting. */
static void ZBuffer_RenderCore(Model3D* model, int biased)
{
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;
    int fcount = faces->face_count;
    int y, f, i, k, b, si, ai, keep, active_count;
    int total_edges, end;
    int clip_x_min, clip_x_max, y_lo, y_hi;
    float frame_max_inv_z;
    ZbEdge* ed;
    ZbHit hits[MAX_SPAN_INTERSECTIONS];

    /* counting-sort tables: bucket[r] .. bucket[r+1]-1 = faces starting at row r */
    static int bucket[ZBUF_HEIGHT + 1];
    static int bucket_cur[ZBUF_HEIGHT + 1];

    /* Scratch memory is reused between frames and freed by Shutdown.
     * A failed growth attempt frees every partial allocation of that group.
     */
    if (!ZBFS_SizeFits(vcount, sizeof(float)) ||
        !ZBFS_SizeFits(vcount, sizeof(UWORD16)) ||
        !ZBFS_SizeFits(fcount, sizeof(int)) ||
        !ZBFS_SizeFits(fcount, sizeof(ZbPlane))) return;
    total_edges = 0;
    for (f = 0; f < fcount; f++) {
        long edge_end = (long)faces->vertex_indices_ptr[f] +
                        (long)faces->vertex_count[f];
        if (faces->vertex_indices_ptr[f] < 0 || faces->vertex_count[f] < 0 ||
            edge_end > 32767L) return;
        end = (int)edge_end;
        if (end > total_edges) total_edges = end;
    }
    if (!ZBFS_SizeFits(total_edges, sizeof(ZbEdge))) return;
    if (zbfs_face_capacity < fcount) {
        ZBFS_FreeFaces();
        zbfs_face_start = (int*)malloc((size_t)fcount * sizeof(int));
        zbfs_face_sorted = (int*)malloc((size_t)fcount * sizeof(int));
        zbfs_face_active = (int*)malloc((size_t)fcount * sizeof(int));
        zbfs_face_fill = (unsigned char*)malloc((size_t)fcount);
        zbfs_face_frame = (unsigned char*)malloc((size_t)fcount);
        zbfs_face_nudge = (unsigned char*)malloc((size_t)fcount);
        zbfs_face_plane = (ZbPlane*)malloc((size_t)fcount * sizeof(ZbPlane));
        if (!zbfs_face_start || !zbfs_face_sorted || !zbfs_face_active ||
            !zbfs_face_fill || !zbfs_face_frame || !zbfs_face_nudge || !zbfs_face_plane) {
            ZBFS_FreeFaces(); return;
        }
        zbfs_face_capacity = fcount;
    }
    if (zbfs_vert_capacity < vcount) {
        ZBFS_FreeVertices();
        zbfs_inv_z = (float*)malloc((size_t)vcount * sizeof(float));
        zbfs_vzq = (UWORD16*)malloc((size_t)vcount * sizeof(UWORD16));
        zbfs_vzq_b = (UWORD16*)malloc((size_t)vcount * sizeof(UWORD16));
        if (!zbfs_inv_z || !zbfs_vzq || !zbfs_vzq_b) {
            ZBFS_FreeVertices(); return;
        }
        zbfs_vert_capacity = vcount;
    }
    if (zbfs_edge_capacity < total_edges) {
        ZBFS_FreeEdges();
        zbfs_edge_buf = (ZbEdge*)malloc((size_t)total_edges * sizeof(ZbEdge));
        if (!zbfs_edge_buf) return;
        zbfs_edge_capacity = total_edges;
    }

    /* ---- PER VERTEX, once per frame -------------------------------------
     * zbfs_inv_z = 1/z is linear in screen space.  The scale that maps zbfs_inv_z to the
     * 16-bit range is chosen from the largest zbfs_inv_z of this frame, then every
     * vertex is quantised ONCE (zq: smaller = closer).  Everything after this
     * point, each face receives a depth plane; raster loops use integers only. */
    frame_max_inv_z = 0.0f;
    for (i = 0; i < vcount; i++) {
        float zo_f = FIXED_TO_FLOAT(vtx->zo[i]);
        zbfs_inv_z[i] = (zo_f > 0.0f) ? (1.0f / zo_f) : 0.0f;
        if (zbfs_inv_z[i] > frame_max_inv_z) frame_max_inv_z = zbfs_inv_z[i];
    }
    ZBuffer_SetScaleForFrame(frame_max_inv_z);

    for (i = 0; i < vcount; i++) {
        /* zbfs_vzq_b is the "pulled closer" copy used by faces with plane_d > 0 (biased only). */
        zbfs_vzq[i]   = ZBuffer_QuantizeInvZ(zbfs_inv_z[i]);
        zbfs_vzq_b[i] = biased ? ZBuffer_QuantizeInvZ(zbfs_inv_z[i] + ZB_Z_BIAS) : zbfs_vzq[i];
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
     * - set up one ZbEdge per polygon edge (see the ZbEdge comment) */
    for (b = 0; b <= ZBUF_HEIGHT; b++) bucket[b] = 0;

    for (f = 0; f < fcount; f++) {
        int n, offt, fmin, fmax, fill;
        UWORD16* zsrc;

        zbfs_face_start[f] = -1;
        if (!faces->display_flag[f]) continue;
        n = faces->vertex_count[f];
        if (n < 3) continue;
        fmin = faces->miny[f];
        fmax = faces->maxy[f];
        if (fmax < y_lo || fmin > y_hi) continue;

        /* First scanline we will actually visit: faces that start above the
         * window are activated on its first row. */
        b = ((fmin > y_lo) ? fmin : y_lo) - y_lo;
        zbfs_face_start[f] = b;
        bucket[b + 1]++;       /* histogram of start rows (shifted by one for the prefix sum) */

        fill = getFaceFillColor(f);
        zbfs_face_fill[f]  = (unsigned char)fill;
        zbfs_face_frame[f] = (unsigned char)getFaceFrameColor(f, fill);
        zbfs_face_nudge[f] = (unsigned char)((biased && faces->plane_d[f] > 0) ? 2 : 0);

        offt = faces->vertex_indices_ptr[f];
        zsrc = (biased && faces->plane_d[f] > 0) ? zbfs_vzq_b : zbfs_vzq;
        ZBuffer_SetPlane(&zbfs_face_plane[f], vtx, faces, f, zsrc, zbfs_face_nudge[f]);
        ed = zbfs_edge_buf + offt;

        for (k = 0; k < n; k++, ed++) {
            int k2 = (k + 1 < n) ? (k + 1) : 0;
            int vid1 = faces->vertex_indices_buffer[offt + k] - 1;
            int vid2 = faces->vertex_indices_buffer[offt + k2] - 1;
            int y1 = vtx->y2d[vid1];
            int y2 = vtx->y2d[vid2];
            int dy = y2 - y1;
            int ya, yb, xs;
            long ex, sx, skip;

            if (dy == 0) {
                ed->ya = 32767; ed->yb = 32767;      /* horizontal edge: never active */
                ed->x = 0L; ed->sx = 0L;
                continue;
            }

            /* Subpixel x slope per scanline (12 fraction bits). */
            sx = (((long)vtx->x2d[vid2] - (long)vtx->x2d[vid1]) * 4096L) / (long)dy;

            /* The edge is walked from its upper end downwards.  x is stored as an
             * offset from xb (= x of vertex 1), so for dy < 0 the starting offset
             * is x(v2) - x(v1) and the slopes stay valid (dx/dy changes sign with dy). */
            if (dy > 0) { ya = y1; yb = y2; xs = 0; }
            else        { ya = y2; yb = y1; xs = vtx->x2d[vid2] - vtx->x2d[vid1]; }

            ex = (long)xs * 4096L;

            /* Edge starts above the visible window: jump straight to the first visible row. */
            if (ya < y_lo && yb > y_lo) {
                skip = (long)(y_lo - ya);
                ex += sx * skip;
            }

            ed->ya = ya;  ed->yb = yb;
            ed->xb = vtx->x2d[vid1];
            ed->x  = ex;               /* keep the subpixel intersection until span pairing */
            ed->sx = sx;
        }
    }

    /* ---- COUNTING SORT: faces ordered by their first visible scanline ----
     * (prefix sums turn the histogram into start offsets, then each face is placed). */
    for (b = 0; b < ZBUF_HEIGHT; b++) bucket[b + 1] += bucket[b];
    for (b = 0; b <= ZBUF_HEIGHT; b++) bucket_cur[b] = bucket[b];
    for (f = 0; f < fcount; f++) {
        b = zbfs_face_start[f];
        if (b >= 0) zbfs_face_sorted[bucket_cur[b]++] = f;
    }

    /* Keep the status text visible throughout allocation, depth clearing,
     * face setup and sorting. Erase it only when screen drawing is about to
     * start. This shared path covers both fast and biased fullscreen modes.
     */
    ZBFS_ClearTop12Lines();

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
            int nf = zbfs_face_sorted[si];
            int pos = active_count;
            while (pos > 0 && zbfs_face_active[pos - 1] > nf) {
                zbfs_face_active[pos] = zbfs_face_active[pos - 1];
                pos--;
            }
            zbfs_face_active[pos] = nf;
            active_count++;
        }

        keep = 0;
        for (ai = 0; ai < active_count; ai++) {
            int n, offt, hit_count;
            int fillColor;

            f = zbfs_face_active[ai];
            if (faces->maxy[f] < y) continue;      /* face finished: dropped from the list */
            zbfs_face_active[keep++] = f;

            n = faces->vertex_count[f];
            offt = faces->vertex_indices_ptr[f];
            hit_count = 0;

            /* Intersect the scanline with the polygon: every edge active on row y gives
             * one subpixel x hit; advance the edge state by one row. */
            ed = zbfs_edge_buf + offt;
            for (k = 0; k < n; k++, ed++) {
                if (y < ed->ya || y >= ed->yb) continue;
                if (hit_count < MAX_SPAN_INTERSECTIONS) {
                    hits[hit_count].x = (long)ed->xb * 4096L + ed->x;
                    hit_count++;
                }
                /* advance even when the hit table is full, to keep the edge in step */
                ed->x += ed->sx;
            }

            if (hit_count < 2) continue;

            /* Sort the hits by x (a swap for the usual 2 hits, insertion sort otherwise). */
            if (hit_count == 2) {
                if (hits[0].x > hits[1].x) {
                    ZbHit tmp = hits[0];
                    hits[0] = hits[1];
                    hits[1] = tmp;
                }
            } else {
                int a, c;
                for (a = 1; a < hit_count; a++) {
                    ZbHit key = hits[a];
                    c = a - 1;
                    while (c >= 0 && hits[c].x > key.x) {
                        hits[c + 1] = hits[c];
                        c--;
                    }
                    hits[c + 1] = key;
                }
            }

            fillColor  = zbfs_face_fill[f];

            /* Consecutive hit pairs are the inside spans of the polygon (even-odd rule).
             * Round inward, clip, and sample depth at the chosen raster pixels. */
            {
                int p;
                for (p = 0; p + 1 < hit_count; p += 2) {
                    long left = hits[p].x;
                    long right = hits[p + 1].x;
                    long xa, xb;
                    int x0, x1;
                    /* Mathematical ceil(left) and floor(right), including
                     * negative coordinates.  Fill never extends outside the
                     * continuous polygon boundary; the nearest-pixel outline
                     * surrounds this interior on either side.
                     */
                    xa = (left >= 0L) ? ((left + 4095L) >> 12) :
                                         -((-left) >> 12);
                    xb = (right >= 0L) ? (right >> 12) :
                                          -((-right + 4095L) >> 12);
                    if (xa > xb || xb < (long)clip_x_min ||
                        xa > (long)clip_x_max) continue;
                    x0 = (xa < (long)clip_x_min) ? clip_x_min : (int)xa;
                    x1 = (xb > (long)clip_x_max) ? clip_x_max : (int)xb;
                    ZBuffer_PaintSpanZ(x0, x1, screenY, fillColor, &zbfs_face_plane[f]);
                }
            }
        }
        active_count = keep;
    }

    /* All surfaces are now in the Z-buffer.  Stroke original polygon sides,
     * including horizontal sides and bottom sides omitted by half-open fills.
     * Use exactly the same biased vertex depths and nudge as the fill pass.
     */
    for (f = 0; f < fcount; f++) {
        int n, offt, color, nudge;
        UWORD16 *zsrc;
        if (zbfs_face_start[f] < 0) continue;
        n = faces->vertex_count[f];
        offt = faces->vertex_indices_ptr[f];
        color = zbfs_face_frame[f];
        nudge = zbfs_face_nudge[f];
        zsrc = (biased && faces->plane_d[f] > 0) ? zbfs_vzq_b : zbfs_vzq;
        for (k = 0; k < n; k++) {
            int k2, vid1, vid2;
            long sx0, sy0, sx1, sy1;
            unsigned long entry;
            UWORD16 z0, z1;
            entry = (unsigned long)offt + (unsigned long)k;
            if (zb_edge_flags != NULL && entry < zb_edge_flag_count &&
                zb_edge_flags[entry] == 0) continue;
            k2 = (k + 1 < n) ? k + 1 : 0;
            vid1 = faces->vertex_indices_buffer[offt + k] - 1;
            vid2 = faces->vertex_indices_buffer[offt + k2] - 1;
            sx0 = (long)vtx->x2d[vid1] + (long)pan_dx;
            sy0 = (long)vtx->y2d[vid1] + (long)pan_dy;
            sx1 = (long)vtx->x2d[vid2] + (long)pan_dx;
            sy1 = (long)vtx->y2d[vid2] + (long)pan_dy;
            /* The rest of the renderer uses signed 16-bit projected positions.
             * Reject unrepresentable panned edges before narrowing to int.
             */
            if (sx0 < -32768L || sx0 > 32767L ||
                sy0 < -32768L || sy0 > 32767L ||
                sx1 < -32768L || sx1 > 32767L ||
                sy1 < -32768L || sy1 > 32767L) continue;
            z0 = (zsrc[vid1] > (UWORD16)nudge) ? (UWORD16)(zsrc[vid1] - nudge) : 0;
            z1 = (zsrc[vid2] > (UWORD16)nudge) ? (UWORD16)(zsrc[vid2] - nudge) : 0;
            ZBuffer_DrawEdge((int)sx0, (int)sy0, z0,
                             (int)sx1, (int)sy1, z1, color, &zbfs_face_plane[f]);
        }
    }
}


void renderModelFullscreenZBuffer_fastV2(Model3D* model)
{
    ZBuffer_RenderCore(model, 0);
}


void renderModelFullscreenZBuffer_biasedV2(Model3D* model)
{
    ZBuffer_RenderCore(model, 1);
}


void renderModelFullscreenZBuffer(Model3D* model)
{
    if (cull_back_faces) {
        renderModelFullscreenZBuffer_fastV2(model);
    } else {
        renderModelFullscreenZBuffer_biasedV2(model);
    }
}
