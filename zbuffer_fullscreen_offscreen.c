/* zbuffer_fullscreen_offscreen.c
 *
 * Offscreen variant of the fullscreen Z-buffer renderer.
 * Usage: include/compile after engine.c symbols are visible, then call
 *   Offscreen_Init() once at startup,
 *   renderModelFullscreenZBuffer_offscreen(model) to render into the offscreen
 *   buffer, and Offscreen_FlushToScreen() to copy the buffer to the visible SHR.
 *
 * Optimisations par rapport a la version precedente :
 *  - pas de float dans la boucle de scanline (marche d'aretes en virgule fixe)
 *  - liste de faces actives (tri par comptage sur la premiere ligne visible)
 *    au lieu de tester toutes les faces a chaque ligne
 *  - couleurs de face calculees une seule fois par frame
 *  - aretes en tableau de structures, avec ya/yb pre-calcules
 *  - boucle de pixels interieurs sans appel de fonction (pointeurs de ligne)
 *  - table des lignes du buffer offscreen (plus de y*160 ni de handle a deref)
 *  - Offscreen_Clear par MVN chevauchant
 *  - Off_SpanRun : pixel gauche + interieur + pixel droit d'un span en assembleur
 */

segment "ZBUF";
#include "zbuffer_fullscreen.h"

#define OFF_PIXELS  32000   /* 200 lignes * 160 octets */
#define OFF_ROWS    200     /* doit valoir SCREEN_HEIGHT */
#define OFF_FRAC    12
#define OFF_ROUND   0x800L          /* 1 << (OFF_FRAC - 1) */
#define OFF_Z_BIAS  0.0005f         /* meme valeur que Z_FIGHT_BIAS */
/* #define OFF_NO_ASM */            /* decommenter pour forcer la boucle C (comparaison) */

/* Offscreen buffer (locked handle) */
Handle offscreen_handle = NULL;
Byte   offscreen_bank = 0;
Word   offscreen_offset = 0;

/* Pointeur de debut de chaque ligne du buffer (valide tant que le handle est verrouille) */
static unsigned char *off_row[OFF_ROWS];
static int off_ready = 0;

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

/* Clear the offscreen pixel buffer to color 0 (both nibbles zeroed).
   Premier octet mis a 0 en C, puis MVN chevauchant (src = base, dst = base+1)
   qui propage ce 0 sur tout le buffer. */
void Offscreen_Clear(void)
{
    unsigned char *p;
    if (offscreen_handle == NULL) return;
    p = (unsigned char *)*offscreen_handle;
    *p = 0;
    asm {
        php
        phb
        sep #0x20
        lda offscreen_bank
        sta >clear_mvn+1
        sta >clear_mvn+2
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

/* Offscreen pixel plot: 2 pixels/octet, pixel pair x even = nibble haut */
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

typedef struct {
    int x;
    UWORD16 zq;
} OffScanIntersection;

/* Arete : x = decalage depuis xb (x du premier sommet) et z = zq courant, en
   virgule fixe 12 bits, arrondi (+0.5) deja inclus ; pentes par ligne ; lignes
   ou l'arete est active : [ya, yb). Le decalage est converti comme l'ancien code
   flottant : xi = xb + (int)(v + 0.5f), c'est-a-dire une troncature vers zero. */
typedef struct {
    long x, z;
    long sx, sz;
    int ya, yb;
    int xb;
} OffEdge;

/* zq a l'abscisse x, par interpolation lineaire entre (xa,za) et (xb,zb).
   Uniquement utilise pour les spans coupes par le clipping horizontal. */
static UWORD16 Off_LerpZ(UWORD16 za, UWORD16 zb, int xa, int xb, int x)
{
    long num;
    if (xb == xa) return za;
    num = ((long)zb - (long)za) * (long)(x - xa);
    return (UWORD16)((long)za + num / (long)(xb - xa));
}

/* ------------------------------------------------------------------------
   Trace d'un span complet en assembleur : pixel gauche + pixels interieurs +
   pixel droit, en un seul appel. Parametres passes par des variables globales
   (pas de convention d'appel a respecter) :
     off_p_zptr / off_p_pptr : adresses 24 bits du mot Z et de l'octet offscreen
                               du pixel GAUCHE (x0 + pan_dx)
     off_p_odd               : parite de ce pixel (1 = nibble bas)
     off_p_zl / off_p_zr     : zq exact du pixel gauche / du pixel droit
     off_p_cl / off_p_ci / off_p_cr : couleur (0..15) gauche / interieur / droit
     off_p_zacc / off_p_zstep : accumulateur z 16.16 et pas (le mot haut = zq)
     off_p_n                 : nombre de pixels interieurs (>= 0)
     off_p_mode              : 0 = pixel gauche seul, 1 = span complet
   Chaque pixel : si (zq < z courant) { z = zq; ecrire le nibble }.
   Les x sont deja clippes : aucun test de bornes ici.
   Cadre direct page de 32 octets alloue sur la pile :
     0 zacc(4) 4 zstep(4) 8 zptr(3) 12 pptr(3) 16 n 18 lo 20 hi 22 parite
     24 masque 26 valeur
   Le Z-buffer est indexe par Y avec [8],y : une ligne qui chevauche deux banques
   est geree. Le buffer offscreen ne chevauche jamais de banque (attrNoCross),
   donc le pointeur pixel avance par un simple inc 16 bits.
   ------------------------------------------------------------------------ */
long off_p_zacc, off_p_zstep;
unsigned long off_p_zptr, off_p_pptr;
int off_p_n, off_p_odd, off_p_mode, off_p_cl, off_p_ci, off_p_cr;
unsigned int off_p_zl, off_p_zr;

static void Off_SpanRun(void)
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
        lda >off_p_ci
        and #0x000F
        sta 18
        asl a
        asl a
        asl a
        asl a
        sta 20
        lda >off_p_odd
        eor #1
        sta 22
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
        lda >off_p_mode
        bne sr_full
        brl sr_done
    sr_full:
        iny
        iny
        lda >off_p_odd
        beq sr_lev
        inc 12
    sr_lev:

        lda 16
        bne sr_int
        brl sr_right
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

/* Trace un span (x0..x1 deja clippes) avec zq interpole entre zq0 et zq_end */
static void Off_PaintSpanZ(int x0, int x1, int y, int screenY,
                           UWORD16 zq0, UWORD16 zq_end,
                           int fillColor, int frameColor,
                           int whole_span_is_border,
                           OffZBufSpanPrev *prev,
                           int z_nudge_lsbs)
{
    int nsteps, x, sx, sx0, sxr, silhouette_r, n, odd;
    UWORD16 zq_cur, zr;
    long z_acc, z_step_16;
    FarWordPtr zp;
    unsigned char *pp;
    unsigned char hi, lo;

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
#ifndef OFF_NO_ASM
        off_p_zptr = (unsigned long)(zbuf_row[screenY] + sx0);
        off_p_pptr = (unsigned long)(off_row[screenY] + (sx0 >> 1));
        off_p_odd  = sx0 & 1;
        off_p_zl   = zq0;
        off_p_cl   = frameColor;
        off_p_mode = 0;
        Off_SpanRun();
#else
        Off_PlotBorder(sx0, screenY, zq0, frameColor);
#endif
    } else {
        nsteps    = x1 - x0;
        z_step_16 = (((long)zq_end - (long)zq0) << 16) / nsteps;
        z_acc     = ((long)zq0 << 16) + 0x8000L;

        /* silhouette a droite : lecture du voisin (les pixels traces ci-dessous
           ne modifient que des colonnes < sxr, l'ordre n'a donc pas d'importance) */
        sxr = x1 + pan_dx;
        silhouette_r = 1;
        if (sxr + 1 < ZBUF_WIDTH) {
            zr = zbuf_row[screenY][sxr + 1];
            if (zr != ZBUF_FAR_VALUE && (int)zq_end - (int)zr >= -1) silhouette_r = 0;
        }

#ifndef OFF_NO_ASM
        off_p_zptr  = (unsigned long)(zbuf_row[screenY] + sx0);
        off_p_pptr  = (unsigned long)(off_row[screenY] + (sx0 >> 1));
        off_p_odd   = sx0 & 1;
        off_p_zl    = zq0;
        off_p_zr    = zq_end;
        off_p_zacc  = z_acc;
        off_p_zstep = z_step_16;
        off_p_n     = nsteps - 1;
        off_p_cl    = frameColor;
        off_p_ci    = whole_span_is_border ? frameColor : fillColor;
        off_p_cr    = (silhouette_r || whole_span_is_border) ? frameColor : fillColor;
        off_p_mode  = 1;
        Off_SpanRun();
#else
        /* version C de reference */
        Off_PlotBorder(sx0, screenY, zq0, frameColor);

        n = nsteps - 1;
        if (n > 0) {
            sx  = sx0 + 1;
            zp  = zbuf_row[screenY] + sx;
            pp  = off_row[screenY] + (sx >> 1);
            odd = sx & 1;
            if (whole_span_is_border) { lo = (unsigned char)(frameColor & 0x0F); }
            else                      { lo = (unsigned char)(fillColor & 0x0F); }
            hi  = (unsigned char)(lo << 4);
            do {
                z_acc += z_step_16;
                zq_cur = (UWORD16)(z_acc >> 16);
                if (zq_cur < *zp) {
                    *zp = zq_cur;
                    if (odd) *pp = (unsigned char)((*pp & 0xF0) | lo);
                    else     *pp = (unsigned char)((*pp & 0x0F) | hi);
                }
                zp++;
                if (odd) { pp++; odd = 0; } else odd = 1;
            } while (--n);
        }

        if (silhouette_r || whole_span_is_border)
            Off_PlotBorder(sxr, screenY, zq_end, frameColor);
        else if (ZBuffer_TestAndSet(sxr, screenY, zq_end))
            Offscreen_DrawPixel(sxr, screenY, fillColor);
#endif
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
    static int* face_start = NULL;          /* ligne de depart (relative a y_lo), -1 = ignoree */
    static int* face_sorted = NULL;         /* faces triees par ligne de depart                */
    static int* face_active = NULL;         /* faces actives, en ordre croissant d'indice      */
    static unsigned char* face_fill = NULL;
    static unsigned char* face_frame = NULL;
    static unsigned char* face_nudge = NULL;
    static int face_capacity = 0;

    static OffEdge* edge_buf = NULL;
    static int edge_capacity = 0;

    static int bucket[OFF_ROWS + 1];
    static int bucket_cur[OFF_ROWS + 1];

    if (!off_ready) return;

    /* ---- allocations (croissance seulement) ---- */
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

    /* ---- par face, une fois par frame : eligibilite, couleurs, aretes ---- */
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
            ed->x  = ex + OFF_ROUND;     /* l'arrondi est integre une fois pour toutes */
            ed->z  = ez + OFF_ROUND;
            ed->sx = sx;  ed->sz = sz;
        }
    }

    /* ---- tri par comptage : faces classees par ligne de depart ---- */
    for (b = 0; b < OFF_ROWS; b++) bucket[b + 1] += bucket[b];
    for (b = 0; b <= OFF_ROWS; b++) bucket_cur[b] = bucket[b];
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
                    hits[hit_count].x  = ed->xb + ((t >= 0L) ? (int)(t >> OFF_FRAC) : -(int)((-t) >> OFF_FRAC));
                    hits[hit_count].zq = (UWORD16)(ed->z >> OFF_FRAC);
                    hit_count++;
                }
                /* avancer meme si la table d'intersections est pleine */
                ed->x += ed->sx;
                ed->z += ed->sz;
            }

            if (hit_count < 2) continue;

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
