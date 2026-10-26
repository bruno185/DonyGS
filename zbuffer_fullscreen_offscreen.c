/*
 * ============================================================================
 *  zbuffer_fullscreen_offscreen.c — Z-buffer plein écran — Framebuffer en
 *  RAM
 * ============================================================================
 *
 *  Rôle :
 *    Rendre le modèle dans un framebuffer RAM de 32000 octets, puis
 *    transférer les pixels vers SHR en une copie MVN, en utilisant le
 *    Z-buffer géré par zbuffer_fullscreen.c.
 *
 *  Organisation :
 *    Le cycle externe est Init, Clear, rendu, FlushToScreen, puis Shutdown.
 *    Le noyau prépare des arêtes et spans dont les codes Z sont interpolés.
 *    Les contours sont obtenus par extrémités et liaisons entre lignes
 *    consécutives ; ils ne reprennent pas le plan de contour du rendu
 *    direct.
 *
 *  Dépendances et contraintes :
 *    Inclus par engine.c après le module plein écran. Le handle verrouillé
 *    ne doit pas traverser de banque. Les MVN sont patchés à l'exécution,
 *    donc non réentrants et inscriptibles. Init doit être appelé une seule
 *    fois ; Shutdown ne libère pas les réserves statiques de préparation du
 *    rendu.
 *
 * ============================================================================
 */


segment "ZBUF";
#include "zbuffer_fullscreen.h"

#define OFF_PIXELS  32000
#define OFF_ROWS    200
#define OFF_FRAC    12
#define OFF_ROUND   0x800L
#define OFF_Z_BIAS  0.0005f


Handle offscreen_handle = NULL;
Byte   offscreen_bank = 0;
Word   offscreen_offset = 0;


static unsigned char *off_row[OFF_ROWS];
static int off_ready = 0;


/*
 * ----------------------------------------------------------------------------
 *  Offscreen_Init
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Allouer le framebuffer de 32000 octets en RAM sans franchissement de
 *    banque.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    1 en réussite, 0 sinon.
 *
 *  Sorties et effets de bord :
 *    handle verrouillé, banque, offset, table off_row et off_ready.
 *
 *  Fonctionnement :
 *    Appeler une fois avant utilisation. Contrairement à ZBuffer_Init,
 *    l'appel répété n'est pas idempotent : l'ancien handle n'est pas libéré
 *    avant remplacement. Le contenu n'est pas effacé ici.
 *
 * ----------------------------------------------------------------------------
 */
int Offscreen_Init(void)
{
    Pointer p;
    unsigned char *base;
    int i;
    /* Marquer indisponible jusqu'à la réussite du handle attrLocked|attrNoCross. */
    off_ready = 0;
    offscreen_handle = NewHandle(OFF_PIXELS, userid(), attrLocked | attrNoCross, 0L);
    if (offscreen_handle == NULL || toolerror()) return 0;
    /* Décomposer le pointeur far et construire les adresses des 200 lignes. */
    p = *offscreen_handle;
    offscreen_bank = (Byte)(((long)p >> 16) & 0xFF);
    offscreen_offset = (Word)((long)p & 0xFFFF);
    base = (unsigned char *)p;
    for (i = 0; i < OFF_ROWS; ++i) off_row[i] = base + (long)i * 160L;
    off_ready = 1;
    return 1;
}


/*
 * ----------------------------------------------------------------------------
 *  Offscreen_Shutdown
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Libérer le framebuffer offscreen détenu par le module.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Annule off_ready et le handle ; appel répété accepté. Ne libère pas les
 *    buffers statiques de préparation de Off_RenderCore ni le Z-buffer.
 *
 * ----------------------------------------------------------------------------
 */
void Offscreen_Shutdown(void)
{
    /* Empêcher un rendu ultérieur avant de disposer le handle. */
    off_ready = 0;
    if (offscreen_handle != NULL) {
        DisposeHandle(offscreen_handle);
        offscreen_handle = NULL;
    }
}


/*
 * ----------------------------------------------------------------------------
 *  Offscreen_Clear
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Effacer le framebuffer en propageant un octet zéro avec MVN.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Entrées et sorties implicites :
 *    [entrée] handle initialisé, sinon aucun effet.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Écrit le premier zéro en C puis copie OFF_PIXELS-1 octets depuis p vers
 *    p+1. Le chevauchement volontaire propage le zéro dans le sens
 *    croissant. Code auto-modifiant : patch des banques source et
 *    destination.
 *
 *  Registres et indicateurs :
 *    Préserve P/DBR, modifie A/X/Y ; le buffer doit rester dans une banque
 *    et le code être inscriptible.
 *
 * ----------------------------------------------------------------------------
 */
void Offscreen_Clear(void)
{
    unsigned char *p;
    if (offscreen_handle == NULL) return;
    p = (unsigned char *)*offscreen_handle;
    /* Amorcer le premier octet avant la propagation par copie chevauchante. */
    *p = 0;
    asm {


        php
        phb
        sep #0x20
        lda offscreen_bank
        /* Patcher les deux banques MVN avec celle du framebuffer. */
        sta >clear_mvn+1
        sta >clear_mvn+2


        rep #0x30
        lda offscreen_offset
        tax
        tay
        iny
        lda #OFF_PIXELS-2
    /* Transférer de p vers p+1 pour propager le zéro puis rétablir DBR/P. */
    clear_mvn:
        mvn 0x00,0x00
        plb
        plp
    }
}


/*
 * ----------------------------------------------------------------------------
 *  Offscreen_FlushToScreen
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Copier le framebuffer RAM vers les 32000 octets de pixels SHR.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Entrées et sorties implicites :
 *    [entrée] handle offscreen, sinon aucun effet.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    MVN patché : source RAM et destination $E1:2000, compteur OFF_PIXELS-1.
 *
 *    Ne copie ni les SCB ni les palettes situés après la zone des pixels.
 *
 *  Registres et indicateurs :
 *    Préserve P/DBR, modifie A/X/Y ; code auto-modifiant non réentrant.
 *
 * ----------------------------------------------------------------------------
 */
void Offscreen_FlushToScreen(void)
{
    if (offscreen_handle == NULL) return;
    asm {
        /* Préserver le contexte avant le patch des octets de banques. */
        php
        phb
        rep #0x10
        sep #0x20


        lda offscreen_bank
        sta >flush_mvn+2
        lda #0xE1
        sta >flush_mvn+1


        rep #0x30
        /* Préparer X source, Y destination et le compteur en nombre d'octets moins un. */
        lda offscreen_offset
        tax
        ldy #0x2000
        lda #OFF_PIXELS-1
    /* Copier le framebuffer puis restaurer la banque de données. */
    flush_mvn:
        mvn 0x00,0xE1

        plb
        plp
    }
}


/*
 * ----------------------------------------------------------------------------
 *  Offscreen_DrawPixel
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Écrire un pixel couleur dans le framebuffer RAM initialisé.
 *
 *  Paramètres :
 *    [entrée] x/y écran et color, quatre bits utiles
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Rejette hors viewport. x pair emploie le nibble haut, x impair le bas ;
 *    préserve le pixel voisin. Aucun test Z n'est réalisé par cette
 *    primitive.
 *
 * ----------------------------------------------------------------------------
 */
static inline void Offscreen_DrawPixel(int x, int y, int color)
{
    unsigned char *pp;
    if ((unsigned)x >= (unsigned)SCREEN_WIDTH || (unsigned)y >= (unsigned)SCREEN_HEIGHT) return;
    /* Localiser l'octet puis remplacer seulement le nibble correspondant à x. */
    pp = off_row[y] + (x >> 1);
    if ((x & 1) == 0)
        *pp = (unsigned char)((*pp & 0x0F) | ((color & 0x0F) << 4));
    else
        *pp = (unsigned char)((*pp & 0xF0) | (color & 0x0F));
}


typedef struct {
    int valid;
    int y;
    int x0, x1;
    UWORD16 zq0, zq1;
} OffZBufSpanPrev;


/*
 * ----------------------------------------------------------------------------
 *  Off_PlotBridge
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Renforcer un pixel de liaison de contour sans modifier la profondeur.
 *
 *  Paramètres :
 *    [entrée] sx/sy écran, zq interpolé et color.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Accepte une profondeur égale à Z ou plus lointaine d'au plus un code
 *    selon les différences int natives. N'introduit pas une surface plus
 *    proche dans Z. Le test de visibilité diffère de celui des contours du
 *    rendu direct.
 *
 * ----------------------------------------------------------------------------
 */
static void Off_PlotBridge(int sx, int sy, UWORD16 zq, int color)
{
    FarWordPtr row;
    UWORD16 cur;
    if ((unsigned)sx >= (unsigned)ZBUF_WIDTH || (unsigned)sy >= (unsigned)ZBUF_HEIGHT) return;
    /* Comparer à la surface existante sans écrire de profondeur. */
    row = zbuf_row[sy];
    cur = row[sx];
    /* Dessiner seulement pour égalité ou tolérance d'un code telle qu'implémentée. */
    if (zq == cur)
        Offscreen_DrawPixel(sx, sy, color);
    else if (zq > cur && (int)zq - (int)cur <= 1)
        Offscreen_DrawPixel(sx, sy, color);
}

/*
 * ----------------------------------------------------------------------------
 *  Off_BridgeBorderZQ
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Relier deux extrémités de spans par une ligne de contour testée contre
 *    Z.
 *
 *  Paramètres :
 *    [entrée] x0/y0,zq0 et x1/y1,zq1 sans pan ; color.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Bresenham sur l'axe majeur et interpolation Z en 16.16 ; applique pan
 *    lors du tracé par Off_PlotBridge. Ne remplit pas et ne remplace aucune
 *    profondeur.
 *
 * ----------------------------------------------------------------------------
 */
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
    /* Traiter le point unique avant le calcul du pas de profondeur. */
    if (dx == 0 && dy == 0) {
        Off_PlotBridge(x0 + pan_dx, y0 + pan_dy, zq0, color);
        return;
    }
    steps = (adx > ady) ? adx : ady;
    if (steps <= 0) return;
    /* Préparer l'interpolation puis choisir l'axe majeur du tracé. */
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


typedef struct {
    long x, z;
    long sx, sz;
    int ya, yb;
    int xb;
} OffEdge;


/*
 * ----------------------------------------------------------------------------
 *  Off_LerpZ
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Interpoler linéairement un code Z entre deux abscisses.
 *
 *  Paramètres :
 *    [entrée] za/zb, codes aux positions xa/xb ; x, position demandée.
 *
 *  Retour :
 *    Za si xa==xb, sinon za+(zb-za)*(x-xa)/(xb-xa), calcul intermédiaire
 *    long.
 *
 *  Fonctionnement :
 *    Aucune saturation ; le domaine interpolé et les produits doivent être
 *    valides.
 *
 * ----------------------------------------------------------------------------
 */
static UWORD16 Off_LerpZ(UWORD16 za, UWORD16 zb, int xa, int xb, int x)
{
    long num;
    /* Éviter la division nulle puis interpoler le code avec troncature entière. */
    if (xb == xa) return za;
    num = ((long)zb - (long)za) * (long)(x - xa);
    return (UWORD16)((long)za + num / (long)(xb - xa));
}


long off_p_zacc, off_p_zstep;
unsigned long off_p_zptr, off_p_pptr;
int off_p_n, off_p_odd, off_p_mode, off_p_cl, off_p_ci, off_p_cr;
unsigned int off_p_zl, off_p_zr;

/*
 * ----------------------------------------------------------------------------
 *  Off_SpanRun
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Dessiner un span préclippé avec test Z strict dans un noyau 65C816.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Entrées et sorties implicites :
 *    Entrée : paquet partagé off_p_* : pointeurs Z et pixels far, parité
 *    gauche, codes gauche/droite, couleurs gauche/intérieur/droite,
 *    accumulateur et pas 16.16, nombre de pixels intérieurs et mode 0
 *    (gauche seul) ou 1 (span complet).
 *
 *  Retour :
 *    Aucun ; écrit Z et les nibbles de couleur.
 *
 *  Registres et indicateurs :
 *    Préserve P et D ; A/X/Y modifiés, DBR inchangé. Mode décimal supposé
 *    désactivé. Cadre D de 32 octets : zacc à 0/2, zstep à 4/6, zptr à 8/10,
 *    pptr à 12/14, compte à 16, couleur basse/haute à 18/20, parité à 22,
 *    masque/valeur à 24/26. Les profondeurs [8],Y peuvent franchir une
 *    banque ; le pointeur de pixels n'est incrémenté que sur 16 bits et doit
 *    rester dans sa banque. Traite les extrémités exactement et l'intérieur
 *    par paires pair/impair. Les pixels de même code Z ne sont pas
 *    remplacés. Non réentrant.
 *
 * ----------------------------------------------------------------------------
 */
static void Off_SpanRun(void)
{
    asm {


        /* Sauvegarder le contexte et réserver 32 octets ; D est positionné
         * sur le premier octet utile.
         */
        php
        phd
        rep #0x30
        tsc
        sec
        sbc #32
        tcs
        inc a
        tcd


        /* Copier le paquet dans le cadre direct et préparer les deux nibbles
         * de couleur.
         */
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
    /* Tester la profondeur exacte du pixel gauche avant toute écriture du nibble. */
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


    /* Avancer après le pixel gauche ; l'adresse couleur progresse seulement
     * après un pixel impair.
     */
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


    /* Traiter seul un premier pixel impair pour aligner le reste sur des paires. */
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

    /* Avancer zacc en 32 bits pour chaque pixel ; propager C du mot bas au
     * mot haut, puis tester Z.
     */
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


    /* Traiter l'éventuel pixel pair résiduel après les paires. */
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


    /* Recalculer la parité et employer le code exact de l'extrémité droite. */
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


    /* Libérer exactement le cadre avant de restaurer D et P. */
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


/*
 * ----------------------------------------------------------------------------
 *  Off_PaintSpanZ
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Dessiner un span offscreen avec extrémités et liaisons de contour.
 *
 *  Paramètres :
 *    [entrée] x0/x1 inclus et y sans pan, screenY avec pan ; zq0/zq_end,
 *    codes ; fillColor/frameColor ; whole_span_is_border, ligne haute/basse
 *    ; z_nudge_lsbs.
 *
 *    [entrée-sortie] prev, span précédent facultatif de la face.
 *
 *  Retour :
 *    Aucun.
 *
 *  Préconditions :
 *    span préclippé et Z/framebuffer initialisés. Réduit les codes par nudge
 *    sans sous-dépassement, prépare le paquet et trace les liaisons entre
 *    spans consécutifs. Diffère du remplissage sur plan Z du rendu direct
 *    actuel.
 *
 * ----------------------------------------------------------------------------
 */
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


    /* Appliquer le petit rapprochement entier avant l'interpolation. */
    if (z_nudge_lsbs > 0) {
        if (zq0 > (UWORD16)z_nudge_lsbs) zq0 = (UWORD16)(zq0 - (UWORD16)z_nudge_lsbs);
        else zq0 = 0;
        if (zq_end > (UWORD16)z_nudge_lsbs) zq_end = (UWORD16)(zq_end - (UWORD16)z_nudge_lsbs);
        else zq_end = 0;
    }

    sx0 = x0 + pan_dx;

    /* Préparer séparément le cas d'un seul pixel. */
    if (x0 == x1) {

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


        z_step_16 = (((long)zq_end - (long)zq0) << 16) / nsteps;
        z_acc     = ((long)zq0 << 16) + 0x8000L;


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
        off_p_n     = nsteps - 1;
        off_p_cl    = frameColor;
        off_p_ci    = whole_span_is_border ? frameColor : fillColor;
        off_p_cr    = (silhouette_r || whole_span_is_border) ? frameColor : fillColor;
        off_p_mode  = 1;
        Off_SpanRun();
    }


    if (prev != NULL && prev->valid && prev->y == y - 1) {
        int dxl = prev->x0 - x0;
        int dxr = prev->x1 - x1;
        if (dxl < 0) dxl = -dxl;
        if (dxr < 0) dxr = -dxr;
        if (dxl > 1) Off_BridgeBorderZQ(prev->x0, prev->y, prev->zq0, x0, y, zq0, frameColor);
        if (dxr > 1) Off_BridgeBorderZQ(prev->x1, prev->y, prev->zq1, x1, y, zq_end, frameColor);
    }


    /* Mémoriser la ligne et les extrémités pour les liaisons de la ligne suivante. */
    if (prev != NULL) {
        prev->valid = 1;
        prev->y = y;
        prev->x0 = x0;
        prev->x1 = x1;
        prev->zq0 = zq0;
        prev->zq1 = zq_end;
    }
}


/*
 * ----------------------------------------------------------------------------
 *  Off_RenderCore
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Rendre les surfaces par balayage plein écran offscreen.
 *
 *  Paramètres :
 *    [entrée] model valide avec projection et métriques à jour ; biased,
 *    active le rapprochement des faces de plane_d>0 lorsque le culling est
 *    désactivé.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    couleurs, mémoire Z, écran ou framebuffer, buffers statiques non
 *    réentrants.
 *
 *  Fonctionnement :
 *    N’effectue ni transformation ni classement des faces ; les faces actives sont
 *    parcourues par identifiant pour définir les égalités de profondeur.
 *    Réserve/réutilise buffers par sommet/face/arête, calcule 1/z, prépare
 *    les arêtes, classe les débuts de face par histogramme puis balaie
 *    uniquement les faces actives. Profondeurs interpolées aux arêtes/spans
 *    ; contours par extrémités et liaisons des lignes successives. Ne
 *    reprend pas les contours sur plan du rendu direct.
 *
 *    Le clipping du viewport est appliqué aux spans ; pas de clipping 3D
 *    général. Les réserves et le comportement d'échec restent propres à
 *    cette version.
 *
 * ----------------------------------------------------------------------------
 */
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
    static int* face_start = NULL;
    static int* face_sorted = NULL;
    static int* face_active = NULL;
    static unsigned char* face_fill = NULL;
    static unsigned char* face_frame = NULL;
    static unsigned char* face_nudge = NULL;
    static int face_capacity = 0;

    static OffEdge* edge_buf = NULL;
    static int edge_capacity = 0;


    static int bucket[OFF_ROWS + 1];
    static int bucket_cur[OFF_ROWS + 1];


    if (!off_ready) return;


    /* Réserver les tables de faces, couleurs et biais. */
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
    /* Agrandir les buffers de sommets et conserver leur capacité entre les rendus. */
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


    frame_max_inv_z = 0.0f;
    /* Calculer les profondeurs inverses et les codes selon le format de
     * cette version.
     */
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


    /* Transformer l'histogramme en offsets cumulés pour classer les débuts
     * de faces.
     */
    /* Initialiser l'histogramme des premières lignes ; préparer visibilité,
     * couleurs et arêtes.
     */
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
                ed->ya = 32767; ed->yb = 32767;
                ed->x = 0L; ed->z = 0L; ed->sx = 0L; ed->sz = 0L;
                continue;
            }


            sx = (((long)vtx->x2d[vid2] - (long)vtx->x2d[vid1]) * 4096L) / (long)dy;
            sz = (((long)zsrc[vid2] - (long)zsrc[vid1]) * 4096L) / (long)dy;


            if (dy > 0) { ya = y1; yb = y2; xs = 0;                                        zs = (long)zsrc[vid1]; }
            else        { ya = y2; yb = y1; xs = vtx->x2d[vid2] - vtx->x2d[vid1];          zs = (long)zsrc[vid2]; }

            ex = (long)xs * 4096L;
            ez = zs * 4096L;


            if (ya < y_lo && yb > y_lo) {
                skip = (long)(y_lo - ya);
                ex += sx * skip;
                ez += sz * skip;
            }

            ed->ya = ya;  ed->yb = yb;
            ed->xb = vtx->x2d[vid1];
            ed->x  = ex + OFF_ROUND;
            ed->z  = ez + OFF_ROUND;
            ed->sx = sx;  ed->sz = sz;
        }
    }


    for (b = 0; b < OFF_ROWS; b++) bucket[b + 1] += bucket[b];
    for (b = 0; b <= OFF_ROWS; b++) bucket_cur[b] = bucket[b];
    for (f = 0; f < fcount; f++) {
        b = face_start[f];
        if (b >= 0) face_sorted[bucket_cur[b]++] = f;
    }


    /* Démarrer la liste des faces actives, puis balayer les lignes visibles. */
    active_count = 0;
    for (y = y_lo; y <= y_hi; y++) {
        int screenY = y + pan_dy;
        int row = y - y_lo;


        /* Insérer les faces commençant sur cette ligne en gardant l'ordre
         * d'identifiants.
         */
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
        /* Écarter les faces terminées et extraire les intersections des
         * arêtes actives.
         */
        for (ai = 0; ai < active_count; ai++) {
            int n, offt, hit_count;
            int fillColor, frameColor, on_top_or_bottom, nudge;

            f = face_active[ai];
            if (faces->maxy[f] < y) continue;
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

                ed->x += ed->sx;
                ed->z += ed->sz;
            }

            if (hit_count < 2) continue;


            /* Ordonner les intersections : échange simple pour deux, tri par
             * insertion au-delà.
             */
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
                /* Apparier les intersections par parité, clipper les spans
                 * et les transmettre au noyau de dessin.
                 */
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


/*
 * ----------------------------------------------------------------------------
 *  renderModelFullscreenZBuffer_offscreen_fastV2
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Appeler Off_RenderCore sans rapprochement des faces de plane_d>0.
 *
 *  Paramètres :
 *    [entrée] model valide avec données de vue préparées.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Reprend les préconditions, effets mémoire et règles de dessin du noyau
 *    appelé.
 *
 * ----------------------------------------------------------------------------
 */
void renderModelFullscreenZBuffer_offscreen_fastV2(Model3D* model)
{
    /* Transmettre explicitement le choix du biais au noyau commun. */
    Off_RenderCore(model, 0);
}

/*
 * ----------------------------------------------------------------------------
 *  renderModelFullscreenZBuffer_offscreen_biasedV2
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Appeler Off_RenderCore avec rapprochement des faces de plane_d>0.
 *
 *  Paramètres :
 *    [entrée] model valide avec données de vue préparées.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Reprend les préconditions, effets mémoire et règles de dessin du noyau
 *    appelé.
 *
 * ----------------------------------------------------------------------------
 */
void renderModelFullscreenZBuffer_offscreen_biasedV2(Model3D* model)
{
    /* Transmettre explicitement le choix du biais au noyau commun. */
    Off_RenderCore(model, 1);
}

/*
 * ----------------------------------------------------------------------------
 *  renderModelFullscreenZBuffer_offscreen
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Choisir la variante de rendu public selon le back-face culling.
 *
 *  Paramètres :
 *    [entrée] model valide et vue préparée.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Avec culling, chemin sans biais ; sans culling, chemin qui rapproche
 *    les faces plane_d>0. Les buffers nécessaires doivent être initialisés
 *    avant cet appel.
 *
 * ----------------------------------------------------------------------------
 */
void renderModelFullscreenZBuffer_offscreen(Model3D* model)
{
    /* Sélectionner le noyau avec la politique de biais correspondant à la
     * visibilité.
     */
    if (cull_back_faces) renderModelFullscreenZBuffer_offscreen_fastV2(model);
    else renderModelFullscreenZBuffer_offscreen_biasedV2(model);
}
