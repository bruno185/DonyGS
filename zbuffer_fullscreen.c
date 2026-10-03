/* zbuffer_fullscreen.c
 *
 * Full-screen 16-bit Z-buffer + renderer — V3 (scanline borders, integer edges).
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


/* ZBuffer_Clear : trois variantes au choix.
 *   (defaut)          assembleur : 16 stores deroules par tour, DBR place sur la
 *                     banque de la ligne ; une ligne qui chevauche deux banques
 *                     (la ligne 102 pour 640 octets par ligne) passe par un chemin
 *                     plus lent, donc le resultat est toujours correct.
 *                     Necessite ZBUF_WIDTH multiple de 16.
 *   ZBUF_CLEAR_SAFE   assembleur, uniquement [dp],y (si "sta |n,x" est refuse
 *                     par ORCA/M) ; environ 2 fois plus lent que la precedente.
 *   ZBUF_CLEAR_C      version C d'origine (memset par ligne).
 */
#if defined(ZBUF_CLEAR_C)

void ZBuffer_Clear(void)
{
    int y;
    for (y = 0; y < ZBUF_HEIGHT; y++)
        memset((void*)zbuf_row[y], 0xFF, ZBUF_ROW_BYTES);
}

#elif defined(ZBUF_CLEAR_SAFE)

void ZBuffer_Clear(void)
{
    asm {
        php
        phd
        rep #0x30
        tsc
        sec
        sbc #8
        tcs
        inc a
        tcd
        lda #0
        sta 0
    zs_row:
        ldx 0
        lda >zbuf_row,x
        sta 2
        lda >zbuf_row+2,x
        sta 4
        ldy #0
        ldx #ZBUF_WIDTH/16
        lda #ZBUF_FAR_VALUE
    zs_loop:
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
        bne zs_loop
        lda 0
        clc
        adc #4
        sta 0
        cmp #ZBUF_HEIGHT*4
        bne zs_row
        tsc
        clc
        adc #8
        tcs
        pld
        plp
    }
}

#else

void ZBuffer_Clear(void)
{
    asm {
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
    zc_row:
        ldx 0
        lda >zbuf_row,x
        sta 2
        lda >zbuf_row+2,x
        sta 4
        lda 2
        clc
        adc #ZBUF_WIDTH*2-1
        bcs zc_slow
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

#endif

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


#define ZB_FRAC      12
#define ZB_ROUND     0x800L          /* 1 << (ZB_FRAC - 1) */
#define ZB_Z_BIAS    0.0005f         /* meme valeur que Z_FIGHT_BIAS */
#define ZB_SHR_BASE  0xE12000UL      /* ecran SHR, utilise seulement avec ZB_DIRECT_E1 */

/* #define ZB_DIRECT_E1 */          /* ecriture directe dans $E1 : voir ZBuffer_SpanRun */

typedef struct {
    int     x;
    UWORD16 zq;
} ZbHit;

/* Arete : x = decalage depuis xb (x du premier sommet) et z = zq courant, en
   virgule fixe 12 bits, arrondi (+0.5) deja inclus ; pentes par ligne ; lignes
   ou l'arete est active : [ya, yb). Le decalage est converti comme l'ancien code
   flottant : xi = xb + (int)(v + 0.5f), c'est-a-dire une troncature vers zero. */
typedef struct {
    long x, z;
    long sx, sz;
    int  ya, yb;
    int  xb;
} ZbEdge;


/* zq a l'abscisse x, par interpolation lineaire entre (xa,za) et (xb,zb).
   Uniquement utilise pour les spans coupes par le clipping horizontal. */
static UWORD16 ZBuffer_LerpZ(UWORD16 za, UWORD16 zb, int xa, int xb, int x)
{
    long num;
    if (xb == xa) return za;
    num = ((long)zb - (long)za) * (long)(x - xa);
    return (UWORD16)((long)za + num / (long)(xb - xa));
}

#ifdef ZB_DIRECT_E1
/* ------------------------------------------------------------------------
   Trace d'un span complet DIRECTEMENT dans la memoire SHR ($E1:2000), en
   assembleur : pixel gauche + pixels interieurs + pixel droit, en un appel.
   A n'activer (ZB_DIRECT_E1) que si drawPixel() fait exactement la meme chose
   (mode 320, pixel pair = nibble haut, base $E12000, 160 octets par ligne).
   Parametres passes par des variables globales (pas de convention d'appel) :
     zb_p_zptr / zb_p_pptr : adresses 24 bits du mot Z et de l'octet ecran du
                             pixel GAUCHE (x0 + pan_dx)
     zb_p_odd              : parite de ce pixel (1 = nibble bas)
     zb_p_zl / zb_p_zr     : zq exact du pixel gauche / du pixel droit
     zb_p_cl / zb_p_ci / zb_p_cr : couleur (0..15) gauche / interieur / droit
     zb_p_zacc / zb_p_zstep : accumulateur z 16.16 et pas (le mot haut = zq)
     zb_p_n                : nombre de pixels interieurs (>= 0)
     zb_p_mode             : 0 = pixel gauche seul, 1 = span complet
   Chaque pixel : si (zq < z courant) { z = zq; ecrire le nibble }.
   Les x sont deja clippes : aucun test de bornes ici.
   Le Z-buffer est indexe par Y avec [8],y : une ligne qui chevauche deux
   banques (la ligne 102 pour 640 octets par ligne) est geree.
   ------------------------------------------------------------------------ */
long zb_p_zacc, zb_p_zstep;
unsigned long zb_p_zptr, zb_p_pptr;
int zb_p_n, zb_p_odd, zb_p_mode, zb_p_cl, zb_p_ci, zb_p_cr;
unsigned int zb_p_zl, zb_p_zr;

static void ZBuffer_SpanRun(void)
{
    asm {
        php
        phd
        rep #0x30
        tsc
        sec
        sbc #32
        tcs
        inc a
        tcd

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
        lda >zb_p_ci
        and #0x000F
        sta 18
        asl a
        asl a
        asl a
        asl a
        sta 20
        lda >zb_p_odd
        eor #1
        sta 22
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
        lda >zb_p_mode
        bne zbr_full
        brl zbr_done
    zbr_full:
        iny
        iny
        lda >zb_p_odd
        beq zbr_lev
        inc 12
    zbr_lev:

        lda 16
        bne zbr_int
        brl zbr_right
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
#endif

/* Trace un span (x0..x1 deja clippes) avec zq interpole entre zq0 et zq_end */
static void ZBuffer_PaintSpanZ(int x0, int x1, int y, int screenY,
                               UWORD16 zq0, UWORD16 zq_end,
                               int fillColor, int frameColor,
                               int whole_span_is_border,
                               ZBufSpanPrev *prev,
                               int z_nudge_lsbs)
{
    int nsteps, sx, sx0, sxr, silhouette_r, n, interiorColor;
    UWORD16 zq_cur, zr;
    long z_acc, z_step_16;
    FarWordPtr zp;

    if (x0 > x1) return;

    if (z_nudge_lsbs > 0) {
        if (zq0 > (UWORD16)z_nudge_lsbs) zq0 = (UWORD16)(zq0 - (UWORD16)z_nudge_lsbs);
        else zq0 = 0;
        if (zq_end > (UWORD16)z_nudge_lsbs) zq_end = (UWORD16)(zq_end - (UWORD16)z_nudge_lsbs);
        else zq_end = 0;
    }

    sx0 = x0 + pan_dx;

    if (x0 == x1) {
        zq_end = zq0;
#ifdef ZB_DIRECT_E1
        zb_p_zptr = (unsigned long)(zbuf_row[screenY] + sx0);
        zb_p_pptr = ZB_SHR_BASE + (unsigned long)screenY * 160UL + (unsigned long)(sx0 >> 1);
        zb_p_odd  = sx0 & 1;
        zb_p_zl   = zq0;
        zb_p_cl   = frameColor;
        zb_p_mode = 0;
        ZBuffer_SpanRun();
#else
        ZBuffer_PlotBorder(sx0, screenY, zq0, frameColor);
#endif
    } else {
        nsteps    = x1 - x0;
        z_step_16 = (((long)zq_end - (long)zq0) << 16) / nsteps;
        z_acc     = ((long)zq0 << 16) + 0x8000L;
        n         = nsteps - 1;           /* pixels interieurs */

        /* Silhouette a droite : frame seulement si le voisin de droite n'est
           pas sur la meme surface. Les pixels traces plus bas ne modifient que
           des colonnes < sxr : l'ordre de lecture n'a pas d'importance. */
        sxr = x1 + pan_dx;
        silhouette_r = 1;
        if (sxr + 1 < ZBUF_WIDTH) {
            zr = zbuf_row[screenY][sxr + 1];
            if (zr != ZBUF_FAR_VALUE && (int)zq_end - (int)zr >= -1)
                silhouette_r = 0;
        }
        interiorColor = whole_span_is_border ? frameColor : fillColor;

#ifdef ZB_DIRECT_E1
        zb_p_zptr  = (unsigned long)(zbuf_row[screenY] + sx0);
        zb_p_pptr  = ZB_SHR_BASE + (unsigned long)screenY * 160UL + (unsigned long)(sx0 >> 1);
        zb_p_odd   = sx0 & 1;
        zb_p_zl    = zq0;
        zb_p_zr    = zq_end;
        zb_p_zacc  = z_acc;
        zb_p_zstep = z_step_16;
        zb_p_n     = n;
        zb_p_cl    = frameColor;
        zb_p_ci    = interiorColor;
        zb_p_cr    = (silhouette_r || whole_span_is_border) ? frameColor : fillColor;
        zb_p_mode  = 1;
        ZBuffer_SpanRun();
#else
        /* Left border */
        ZBuffer_PlotBorder(sx0, screenY, zq0, frameColor);

        /* Interior : test Z en ligne (plus d'appel a ZBuffer_TestAndSet) ;
           x est deja clippe, donc sx reste dans [0, SCREEN_WIDTH-1]. */
        if (n > 0) {
            sx = sx0 + 1;
            zp = zbuf_row[screenY] + sx;
            do {
                z_acc += z_step_16;
                zq_cur = (UWORD16)(z_acc >> 16);
                if (zq_cur < *zp) {
                    *zp = zq_cur;
                    drawPixel(sx, screenY, interiorColor);
                }
                zp++;
                sx++;
            } while (--n);
        }

        /* Right edge */
        if (silhouette_r || whole_span_is_border)
            ZBuffer_PlotBorder(sxr, screenY, zq_end, frameColor);
        else if (ZBuffer_TestAndSet(sxr, screenY, zq_end))
            drawPixel(sxr, screenY, fillColor);
#endif
    }

    /* Bridge only real gaps; integer Z - cheap */
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
 *
 * Optimisations par rapport a la version a scanline flottante :
 *  - pas de float dans la boucle de scanline : marche d'aretes en virgule
 *    fixe (12 bits), zq quantifie une seule fois par sommet et par frame ;
 *  - liste de faces actives (tri par comptage sur la premiere ligne visible)
 *    au lieu de tester toutes les faces a chaque ligne ;
 *  - couleurs de face calculees une seule fois par frame ;
 *  - aretes en tableau de structures, avec ya/yb pre-calcules ;
 *  - boucle des pixels interieurs sans appel de fonction.
 * ==================================================================== */

/* Rendu commun aux variantes fast (biased = 0) et biased (biased = 1) */
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

    static float* inv_z = NULL;
    static UWORD16* vzq = NULL;
    static UWORD16* vzq_b = NULL;
    static int vert_capacity = 0;

    static ZBufSpanPrev* span_prev = NULL;
    static int* face_start = NULL;          /* ligne de depart (relative a y_lo), -1 = ignoree */
    static int* face_sorted = NULL;         /* faces triees par ligne de depart                */
    static int* face_active = NULL;         /* faces actives, en ordre croissant d'indice      */
    static unsigned char* face_fill = NULL;
    static unsigned char* face_frame = NULL;
    static unsigned char* face_nudge = NULL;
    static int face_capacity = 0;

    static ZbEdge* edge_buf = NULL;
    static int edge_capacity = 0;

    static int bucket[ZBUF_HEIGHT + 1];
    static int bucket_cur[ZBUF_HEIGHT + 1];

    /* ---- allocations (croissance seulement) ---- */
    if (face_capacity < fcount) {
        if (span_prev)    free(span_prev);
        if (face_start)   free(face_start);
        if (face_sorted)  free(face_sorted);
        if (face_active)  free(face_active);
        if (face_fill)    free(face_fill);
        if (face_frame)   free(face_frame);
        if (face_nudge)   free(face_nudge);
        span_prev   = (ZBufSpanPrev*)malloc(fcount * sizeof(ZBufSpanPrev));
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
        edge_buf = (ZbEdge*)malloc((size_t)total_edges * sizeof(ZbEdge));
        edge_capacity = edge_buf ? total_edges : 0;
    }
    if (face_capacity < fcount || vert_capacity < vcount || edge_capacity < total_edges)
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

#ifdef ZBUF_DIAG_SCALE
    if (!biased) {
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
                fprintf(zdiag_f, "inv_z min=%f max=%f vcount=%d\n", dbg_min, dbg_max, vcount);
                fclose(zdiag_f);
            }
            zdiag_logged = 1;
        }
    }
#endif
    for (i = 0; i < vcount; i++) {
        vzq[i]   = ZBuffer_QuantizeInvZ(inv_z[i]);
        vzq_b[i] = biased ? ZBuffer_QuantizeInvZ(inv_z[i] + ZB_Z_BIAS) : vzq[i];
    }

    SetPenMode(0);
    applyPalette(palette);

    clip_x_min = -pan_dx;
    clip_x_max = SCREEN_WIDTH - 1 - pan_dx;
    y_lo = -pan_dy;
    y_hi = SCREEN_HEIGHT - 1 - pan_dy;

    ZBuffer_Clear();

    /* ---- par face, une fois par frame : eligibilite, couleurs, aretes ---- */
    for (b = 0; b <= ZBUF_HEIGHT; b++) bucket[b] = 0;

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

        b = ((fmin > y_lo) ? fmin : y_lo) - y_lo;
        face_start[f] = b;
        bucket[b + 1]++;

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
                ed->ya = 32767; ed->yb = 32767;      /* jamais active */
                ed->x = 0L; ed->z = 0L; ed->sx = 0L; ed->sz = 0L;
                continue;
            }

            sx = (((long)vtx->x2d[vid2] - (long)vtx->x2d[vid1]) * 4096L) / (long)dy;
            sz = (((long)zsrc[vid2] - (long)zsrc[vid1]) * 4096L) / (long)dy;

            if (dy > 0) { ya = y1; yb = y2; xs = 0;                                        zs = (long)zsrc[vid1]; }
            else        { ya = y2; yb = y1; xs = vtx->x2d[vid2] - vtx->x2d[vid1];          zs = (long)zsrc[vid2]; }

            ex = (long)xs * 4096L;
            ez = zs * 4096L;

            /* arete qui commence au-dessus de la zone visible : avancer d'un coup */
            if (ya < y_lo && yb > y_lo) {
                skip = (long)(y_lo - ya);
                ex += sx * skip;
                ez += sz * skip;
            }

            ed->ya = ya;  ed->yb = yb;
            ed->xb = vtx->x2d[vid1];
            ed->x  = ex + ZB_ROUND;     /* l'arrondi est integre une fois pour toutes */
            ed->z  = ez + ZB_ROUND;
            ed->sx = sx;  ed->sz = sz;
        }
    }

    /* ---- tri par comptage : faces classees par ligne de depart ---- */
    for (b = 0; b < ZBUF_HEIGHT; b++) bucket[b + 1] += bucket[b];
    for (b = 0; b <= ZBUF_HEIGHT; b++) bucket_cur[b] = bucket[b];
    for (f = 0; f < fcount; f++) {
        b = face_start[f];
        if (b >= 0) face_sorted[bucket_cur[b]++] = f;
    }

    /* ---- boucle principale ---- */
    active_count = 0;
    for (y = y_lo; y <= y_hi; y++) {
        int screenY = y + pan_dy;
        int row = y - y_lo;

        /* faces qui deviennent actives a cette ligne (insertion, ordre d'indice conserve) */
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
            if (faces->maxy[f] < y) continue;      /* face terminee : retiree de la liste */
            face_active[keep++] = f;

            n = faces->vertex_count[f];
            offt = faces->vertex_indices_ptr[f];
            hit_count = 0;

            ed = edge_buf + offt;
            for (k = 0; k < n; k++, ed++) {
                if (y < ed->ya || y >= ed->yb) continue;
                if (hit_count < MAX_SPAN_INTERSECTIONS) {
                    long t = ed->x;
                    hits[hit_count].x  = ed->xb + ((t >= 0L) ? (int)(t >> ZB_FRAC) : -(int)((-t) >> ZB_FRAC));
                    hits[hit_count].zq = (UWORD16)(ed->z >> ZB_FRAC);
                    hit_count++;
                }
                /* avancer meme si la table d'intersections est pleine */
                ed->x += ed->sx;
                ed->z += ed->sz;
            }

            if (hit_count < 2) continue;

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

            fillColor  = face_fill[f];
            frameColor = face_frame[f];
            nudge      = face_nudge[f];
            on_top_or_bottom = (y == faces->miny[f] || y == faces->maxy[f]);

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

                    if (x0 < clip_x_min) { zq0 = ZBuffer_LerpZ(zqa, zqb, xa, xb, clip_x_min); x0 = clip_x_min; }
                    if (x1 > clip_x_max) { zq1 = ZBuffer_LerpZ(zqa, zqb, xa, xb, clip_x_max); x1 = clip_x_max; }
                    if (x0 > x1) continue;

                    ZBuffer_PaintSpanZ(x0, x1, y, screenY, zq0, zq1,
                                   fillColor, frameColor, on_top_or_bottom,
                                   &span_prev[f], nudge);
                }
            }
        }
        active_count = keep;
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
