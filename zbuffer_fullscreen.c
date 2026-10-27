/*
 * ============================================================================
 *  zbuffer_fullscreen.c — Z-buffer plein écran — Rendu direct et contours
 *  sur plan
 * ============================================================================
 *
 *  Rôle :
 *    Gérer la table de profondeur plein écran et dessiner directement en SHGR
 *    les surfaces et leurs contours, avec un plan affine de codes Z commun
 *    aux deux passes.
 *
 *  Organisation :
 *    L'allocation aligne deux banques utilisables et construit les pointeurs
 *    de lignes. Le rendu prépare sommets, plans et arêtes, remplit les spans
 *    avec arrondi intérieur, puis trace les côtés originaux par Bresenham.
 *    Une garde compare aussi la profondeur géométrique des arêtes.
 *
 *  Dépendances et contraintes :
 *    Inclus par engine.c et utilise zbuffer_fullscreen.h. Le noyau de contours
 *    est autonome. Init est idempotent ; Shutdown libère les buffers propres à
 *    ce rendu. Les codes plus petits sont plus proches et $FFFF représente
 *    le vide. Les règles de contour diffèrent du rendu offscreen. Paquets
 *    assembleur partagés et non réentrants.
 *
 * ============================================================================
 */

segment "ZBUF";
#include "zbuffer_fullscreen.h"

FarWordPtr zbuf_row[ZBUF_HEIGHT];

static Handle  zbuf_handle = NULL;
static int     zbuf_bank_lo = 0;
static void ZBFS_FreeScratch(void);
static int     zbuf_bank_hi = 0;

#define ZBUF_USABLE_SIZE   (2UL * 65536UL)
#define ZBUF_ALLOC_SIZE    (ZBUF_USABLE_SIZE + 65536UL)

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_Init
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Réserver le Z-buffer plein écran aligné sur une frontière de banque.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    1 si déjà initialisé ou allocation réussie, 0 sinon.
 *
 *  Fonctionnement :
 *    Demande 192 Kio pour garantir deux banques utilisables après alignement
 *    64 Kio. Prépare zbuf_row pour 200 lignes de 320 mots puis efface les
 *    profondeurs. Le handle fixe reste possédé par le module jusqu'à
 *    ZBuffer_Shutdown.
 *
 * ----------------------------------------------------------------------------
 */
int ZBuffer_Init(void)
{
    ULONG32 raw;
    ULONG32 aligned;
    int     y;

    /* Réutiliser l'allocation existante ; nettoyer le handle partiel après
     * une erreur Toolbox.
     */
    if (zbuf_handle != NULL) return 1;
    zbuf_handle = NewHandle(ZBUF_ALLOC_SIZE, userid(), attrFixed, 0L);
    if (toolerror() || zbuf_handle == NULL) {
        if (zbuf_handle != NULL) DisposeHandle(zbuf_handle);
        zbuf_handle = NULL;
        return 0;
    }

    /* Arrondir l'adresse du bloc à la banque supérieure puis construire les
     * pointeurs de lignes.
     */
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

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_Shutdown
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Libérer les ressources du rendu plein écran direct.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Libère scratch et handle Z, annule lignes et banques. Oublie le masque
 *    d'arêtes emprunté sans le libérer. Appel répété accepté. Ne libère pas
 *    les allocations statiques propres aux autres moteurs de rendu.
 *
 * ----------------------------------------------------------------------------
 */
void ZBuffer_Shutdown(void)
{
    int y;
    /* Libérer les buffers auxiliaires avant le Z-buffer et invalider le
     * masque emprunté.
     */
    ZBFS_FreeScratch();
    ZBuffer_SetEdgeFlags(NULL, 0UL);
    if (zbuf_handle != NULL) {
        DisposeHandle(zbuf_handle);
        zbuf_handle = NULL;
    }
    for (y = 0; y < ZBUF_HEIGHT; y++) zbuf_row[y] = NULL;
    zbuf_bank_lo = zbuf_bank_hi = 0;
}

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_Clear
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Remplir de $FFFF les 200 lignes de profondeur.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Entrées et sorties implicites :
 *    Entrée : zbuf_row préparé.
 *
 *  Retour :
 *    Aucun.
 *
 *  Registres et indicateurs :
 *    Préserve P, DBR et D ; A/X/Y servent au transfert. Cadre D de six
 *    octets : offset de table à 0, pointeur de ligne à 2/4 ; D pointe S+1
 *    après réserve. Chemin rapide par blocs de seize mots avec DBR de la
 *    ligne ; chemin [dp],Y pour une ligne traversant une banque. Largeur
 *    multiple de 16 requise. Les accès |offset,X imposent le mode absolu,
 *    pas la page directe.
 *
 * ----------------------------------------------------------------------------
 */
void ZBuffer_Clear(void)
{
    asm {

        /* Sauvegarder P/DBR/D et réserver le cadre avant les accès multi-banques. */
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

    /* Charger le pointeur far de ligne et détecter un franchissement de banque. */
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
    /* Écrire seize profondeurs à la fois en adressage absolu indexé. */
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

    /* Pour une ligne à cheval, employer [dp],Y qui propage l'adresse dans la
     * banque.
     */
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

    /* Avancer de quatre octets dans la table de pointeurs. */
    zc_next:
        lda 0
        clc
        adc #4
        sta 0
        cmp #ZBUF_HEIGHT*4
        beq zc_end
        brl zc_row
    /* Libérer le cadre et rétablir D, DBR et P. */
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

static float zbuffer_scale = ZBUFFER_INV_Z_SCALE;

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_SetScaleForFrame
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Adapter l'échelle de quantification à la profondeur inverse maximale.
 *
 *  Paramètres :
 *    [entrée] max_inv_z.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    zbuffer_scale, cible/max_inv_z si max_inv_z > 1e-7 ; sinon conserve
 *    l'échelle précédente.
 *
 * ----------------------------------------------------------------------------
 */
void ZBuffer_SetScaleForFrame(float max_inv_z)
{
    /* Éviter une division par un maximum nul ou trop petit. */
    if (max_inv_z > 0.0000001f) {
        zbuffer_scale = ZBUFFER_TARGET_MAX_CODE / max_inv_z;
    }
}

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_QuantizeInvZ
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Quantifier une profondeur inverse en code 16 bits où petit signifie
 *    proche.
 *
 *  Paramètres :
 *    [entrée] inv_z, profondeur inverse flottante ; valeurs négatives
 *    ramenées à zéro.
 *
 *  Retour :
 *    65535 - round(inv_z*échelle), après saturation de la valeur quantifiée
 *    dans [0,65535]. $FFFF est la valeur vide/lointaine.
 *
 *  Fonctionnement :
 *    Aucun changement d'échelle.
 *
 * ----------------------------------------------------------------------------
 */
UWORD16 ZBuffer_QuantizeInvZ(float inv_z)
{
    /* Conserver l’échelle publique du plein écran ; partager les arrondis. */
    return ZBuffer_QuantizeAtScale(inv_z, zbuffer_scale);
}

#define ZB_Z_BIAS    0.0005f
#define ZB_SHGR_BASE  0xE12000UL

typedef struct {
    long x;
} ZbHit;

typedef struct {
    long x;
    long sx;
    int  ya, yb;
    int  xb;
} ZbEdge;


typedef struct {
    long gx, gy;
    long x, y;
    UWORD16 z;
} ZbPlane;

/* Inverse de projection commun à tous les plans de la vue courante. */
static float zbfs_inv_projection = 0.0f;

static ZBufferVertexWork zbfs_vertices;

static ZBufferFaceWork zbfs_faces;
static ZbPlane *zbfs_face_plane = NULL;

static ZbEdge* zbfs_edge_buf = NULL;
static int zbfs_edge_capacity = 0;

/*
 * ----------------------------------------------------------------------------
 *  ZBFS_FreeFaces
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Libérer les tables de faces, couleurs, biais et plans.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Remet les pointeurs et leur capacité à zéro. Utilise uniquement les
 *    allocations du rendu plein écran direct.
 *
 * ----------------------------------------------------------------------------
 */
static void ZBFS_FreeFaces(void)
{
    /* Libérer les tables communes, puis les plans propres au rendu direct. */
    ZBuffer_FreeFaces(&zbfs_faces);
    free(zbfs_face_plane);
    zbfs_face_plane = NULL;
}

/*
 * ----------------------------------------------------------------------------
 *  ZBFS_FreeVertices
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Libérer les profondeurs inverses et codes des sommets.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Remet les pointeurs et leur capacité à zéro. Utilise uniquement les
 *    allocations du rendu plein écran direct.
 *
 * ----------------------------------------------------------------------------
 */
static void ZBFS_FreeVertices(void)
{
    /* La réserve commune neutralise pointeurs et capacité après libération. */
    ZBuffer_FreeVertices(&zbfs_vertices);
}

/*
 * ----------------------------------------------------------------------------
 *  ZBFS_FreeEdges
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Libérer les arêtes préparées.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Remet les pointeurs et leur capacité à zéro. Utilise uniquement les
 *    allocations du rendu plein écran direct.
 *
 * ----------------------------------------------------------------------------
 */
static void ZBFS_FreeEdges(void)
{
    /* Libérer chaque buffer détenu puis invalider les pointeurs et la capacité. */
    if (zbfs_edge_buf) free(zbfs_edge_buf);
    zbfs_edge_buf = NULL;
    zbfs_edge_capacity = 0;
}

/*
 * ----------------------------------------------------------------------------
 *  ZBFS_FreeScratch
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Libérer les trois familles de buffers auxiliaires du plein écran
 *    direct.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Centralise faces, sommets et arêtes. Ne libère ni le handle Z ni le
 *    framebuffer offscreen.
 *
 * ----------------------------------------------------------------------------
 */
static void ZBFS_FreeScratch(void)
{
    /* Déléguer à chaque famille, dont les fonctions remettent les pointeurs
     * à NULL.
     */
    ZBFS_FreeFaces();
    ZBFS_FreeVertices();
    ZBFS_FreeEdges();
}



/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_SetPlane
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Préparer un plan affine de codes Z dans le repère écran.
 *
 *  Paramètres :
 *    [entrée] vtx/faces : coordonnées et plans observateur à jour ; f valide.
 *    [entrée] zsrc : codes des sommets pour le repli dégénéré ; nudge en codes.
 *    [entrée] code_bias : biais déjà converti en codes, nul sans rapprochement.
 *
 *  Retour :
 *    Aucun ; écrit le plan *p, avec pan et gradients à douze bits fractionnaires.
 *
 *  Fonctionnement :
 *    Réutilise le plan de Newell calculé par calculateFaceDepths. Pour le rayon
 *    écran (u,v,1), 1/z = -(a*u+b*v+c)/d ; cette fonction est affine avant
 *    quantification. Les coordonnées arrondies des sommets servent uniquement
 *    à choisir l'ancrage, jamais à reconstruire les gradients. Un plan nul ou
 *    passant par l'observateur conserve la profondeur constante de repli.
 *    Les quatre coefficients ont la même échelle : elle se simplifie dans
 *    leurs rapports. Évite donc les conversions intermédiaires en double et
 *    les divisions par 65536 ; l'inverse de projection est préparé par vue.
 *
 * ----------------------------------------------------------------------------
 */
static void ZBuffer_SetPlane(ZbPlane *p, VertexArrays3D *vtx,
                             FaceArrays3D *faces, int f, UWORD16 *zsrc,
                             int nudge, float code_bias)
{
    int off, vertex;
    float a, b, c, d, factor, gradient_factor, gx, gy, code;
    off = faces->vertex_indices_ptr[f];
    vertex = faces->vertex_indices_buffer[off] - 1;
    /* Préparer un repli constant pour les plans dégénérés, avec le même biais
     * que les codes des sommets.
     */
    p->x = (long)vtx->x2d[vertex] + (long)pan_dx;
    p->y = (long)vtx->y2d[vertex] + (long)pan_dy;
    p->z = (zsrc[vertex] > (UWORD16)nudge) ?
           (UWORD16)(zsrc[vertex] - nudge) : 0;
    p->gx = p->gy = 0L;
    d = (float)faces->plane_d[f];
    if (d == 0.0f || zbfs_inv_projection <= 0.0f) return;

    /* Convertir le plan observateur en gradients de codes, sans ajustement
     * sur un triangle projeté : un triangle presque aligné après arrondi peut
     * donner une pente erronée pour toute la surface.
     */
    a = (float)faces->plane_a[f];
    b = (float)faces->plane_b[f];
    c = (float)faces->plane_c[f];
    factor = zbuffer_scale / d;
    gradient_factor = factor * zbfs_inv_projection;
    gx = gradient_factor * a;
    gy = -gradient_factor * b;
    code = 65535.0f + factor * c +
           gx * ((float)vtx->x2d[vertex] - (float)CENTRE_X) +
           gy * ((float)vtx->y2d[vertex] - (float)CENTRE_Y) -
           code_bias - (float)nudge;
    /* Quantifier l'ancrage et les gradients seulement après le calcul du plan.
     * Le pan déplace l'ancrage sans changer les profondeurs de la vue.
     */
    if (code < 0.0f) code = 0.0f;
    if (code > 65535.0f) code = 65535.0f;
    p->z = (UWORD16)(code + 0.5f);
    gx *= 4096.0f;
    gy *= 4096.0f;
    if (gx > 2147483000.0f) gx = 2147483000.0f;
    if (gx < -2147483000.0f) gx = -2147483000.0f;
    if (gy > 2147483000.0f) gy = 2147483000.0f;
    if (gy < -2147483000.0f) gy = -2147483000.0f;
    p->gx = (long)((gx < 0.0f) ? gx - 0.5f : gx + 0.5f);
    p->gy = (long)((gy < 0.0f) ? gy - 0.5f : gy + 0.5f);
}

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_PlaneAt
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Évaluer le plan Z à un pixel écran sans encore réduire ses fractions.
 *
 *  Paramètres :
 *    [entrée] p, x/y en pixels incluant pan.
 *
 *  Retour :
 *    Accumulateur modulo 2^32, avec douze bits fractionnaires.
 *
 *  Fonctionnement :
 *    Aucun effet de bord ; saturation faite séparément.
 *
 * ----------------------------------------------------------------------------
 */
static unsigned long ZBuffer_PlaneAt(ZbPlane *p, long x, long y)
{
    /* Additionner les gradients par rapport à l'origine du plan avec
     * rebouclage explicite non signé.
     */
    return (((unsigned long)p->z << 12) +
            (unsigned long)p->gx * (unsigned long)(x - p->x) +
            (unsigned long)p->gy * (unsigned long)(y - p->y)) & 0xFFFFFFFFUL;
}

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_PlaneCode
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Arrondir et borner un accumulateur Z à douze bits fractionnaires.
 *
 *  Paramètres :
 *    [entrée] z, représentation modulo 2^32.
 *
 *  Retour :
 *    Code 16 bits ; valeur avec bit de signe après ajout du demi-pas donne 0
 *    ; débordement positif donne $FFFF.
 *
 *  Fonctionnement :
 *    Aucun effet de bord.
 *
 * ----------------------------------------------------------------------------
 */
static UWORD16 ZBuffer_PlaneCode(unsigned long z)
{
    /* Ajouter le demi-pas puis interpréter les bornes avant la réduction de
     * douze bits.
     */
    z = (z + 0x800UL) & 0xFFFFFFFFUL;
    if (z & 0x80000000UL) return 0;
    z >>= 12;
    return (z > 65535UL) ? ZBUF_FAR_VALUE : (UWORD16)z;
}

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_PaintSpanZ
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Préparer un span de surface évalué sur le plan Z commun de la face.
 *
 *  Paramètres :
 *    [entrée] x0/x1 inclus sans pan, screenY avec pan, fillColor et plane.
 *
 *  Retour :
 *    Aucun ; écrit profondeur et écran via ZBuffer_SpanRun.
 *
 *  Préconditions :
 *    coordonnées déjà clippées et lignes valides. Prépare codes exacts aux
 *    extrémités et pas 16.16 depuis le gradient à douze fractions.
 *
 * ----------------------------------------------------------------------------
 */
static void ZBuffer_PaintSpanZ(int x0, int x1, int screenY,
                               int fillColor, ZbPlane *plane)
{
    int sx0, nsteps;
    unsigned long raw;
    if (x0 > x1) return;
    /* Appliquer pan horizontal et calculer les adresses du premier pixel. */
    sx0 = x0 + pan_dx;
    raw = ZBuffer_PlaneAt(plane, (long)sx0, (long)screenY);
    zspan_p_zptr = (unsigned long)(zbuf_row[screenY] + sx0);
    zspan_p_pptr = ZB_SHGR_BASE + (unsigned long)screenY * 160UL +
                (unsigned long)(sx0 >> 1);
    zspan_p_odd = sx0 & 1;
    zspan_p_zl = ZBuffer_PlaneCode(raw);
    zspan_p_cl = fillColor;
    zspan_p_mode = 0;
    /* Distinguer un seul pixel d'un span avec intérieur et extrémité droite. */
    if (x0 != x1) {
        nsteps = x1 - x0;
        zspan_p_zstep = (long)(((unsigned long)plane->gx << 4) & 0xFFFFFFFFUL);
        zspan_p_zacc = (long)(((raw << 4) + 0x8000UL) & 0xFFFFFFFFUL);
        zspan_p_zr = ZBuffer_PlaneCode(raw +
                      (unsigned long)plane->gx * (unsigned long)nsteps);
        zspan_p_n = nsteps - 1;
        zspan_p_ci = fillColor;
        zspan_p_cr = fillColor;
        zspan_p_mode = 1;
    }
    /* Exécuter le noyau une seule fois pour le span complet. */
    ZBuffer_SpanRun();
}

static const unsigned char *zb_edge_flags = NULL;
static unsigned long zb_edge_flag_count = 0UL;

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_SetEdgeFlags
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Installer un masque facultatif des côtés de faces, sans prendre sa
 *    propriété.
 *
 *  Paramètres :
 *    [entrée] flags, un octet par entrée du buffer d'indices ; count,
 *    longueur.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Zéro masque un côté, non-zéro l'autorise ; NULL rétablit tous les
 *    côtés. Le buffer doit rester vivant pendant le rendu qui l'utilise.
 *
 * ----------------------------------------------------------------------------
 */
void ZBuffer_SetEdgeFlags(const unsigned char *flags, unsigned long count)
{
    /* Mémoriser le pointeur emprunté et sa borne ; aucune copie ni libération. */
    zb_edge_flags = flags;
    zb_edge_flag_count = count;
}

/* Paquet non réentrant copié en page directe : cinq valeurs 32 bits puis
 * quinze mots 16 bits. Les offsets du noyau suivent exactement cet ordre.
 */
typedef struct {
    unsigned long raw, zmajor, zminor, zptr, pptr;
    unsigned int x, y, major, minor, rem, ez, ezstep, epart, erem, esign;
    unsigned int count, steep, vstep, color_lo, color_hi;
} ZbEdgePacket;
/* Le cadre assembleur exige exactement les offsets documentés ci-dessous. */
typedef char ZbEdgePacketSizeCheck[(sizeof(ZbEdgePacket) == 50) ? 1 : -1];
static ZbEdgePacket zbfs_edge_packet;

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_EdgeRun
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Parcourir une arête préparée et écrire ses pixels visibles en SHGR.
 *
 *  Paramètres :
 *    Aucun paramètre explicite ; lit zbfs_edge_packet, préparé par DrawEdge.
 *
 *  Retour :
 *    Aucun ; met à jour les profondeurs et les nibbles des pixels acceptés.
 *
 *  Fonctionnement :
 *    Copie le paquet de 50 octets dans un cadre direct de 52 octets, puis
 *    avance Bresenham, les profondeurs et les pointeurs sans appel C par pixel.
 *    Les profondeurs égales restent dessinables ; un contour strictement plus
 *    proche doit aussi passer le test de profondeur géométrique de l'arête.
 *    Vérifie les coordonnées avant tout accès mémoire. Les sommes de restes
 *    propagent la retenue pour les arêtes dont l'axe majeur dépasse 32767.
 *
 *  Registres et contraintes :
 *    Préserve P et D ; DBR inchangé, A/X/Y volatils. Accumulateur 16 bits sauf
 *    pendant l'écriture du nibble. Offsets 0..49 : paquet ; 50 : code Z local.
 *    Pointeurs far avancés sur 32 bits pour franchir les banques du Z-buffer.
 *    Appel synchrone et non réentrant ; count positif, major/minor <= 65535.
 *
 * ----------------------------------------------------------------------------
 */
static void ZBuffer_EdgeRun(void)
{
    asm {
        /* Sauver le contexte, réserver le cadre puis recopier le paquet. */
        php
        phd
        rep #0x30
        tsc
        sec
        sbc #52
        tcs
        inc a
        tcd
        ldx #48
    zer_copy:
        lda >zbfs_edge_packet,x
        sta 0,x
        dex
        dex
        bpl zer_copy

    zer_pixel:
        /* Les pointeurs peuvent être hors viewport ; ne jamais les lire
         * avant les tests non signés des deux coordonnées.
         */
        lda 20
        cmp #320
        bcc zer_check_y
        brl zer_next
    zer_check_y:
        lda 22
        cmp #200
        bcc zer_code
        brl zer_next
    zer_code:
        /* Arrondir raw modulo 2^32, saturer et extraire ses seize bits utiles. */
        clc
        lda 0
        adc #0x0800
        tax
        lda 2
        adc #0
        bmi zer_zero
        cmp #0x1000
        bcc zer_extract
        brl zer_next
    zer_extract:
        asl a
        asl a
        asl a
        asl a
        sta 50
        txa
        xba
        and #0x00FF
        lsr a
        lsr a
        lsr a
        lsr a
        ora 50
        cmp #0xFFFF
        bne zer_depth
        brl zer_next
    zer_zero:
        lda #0
    zer_depth:
        sta 50
        ldy #0
        cmp [12],y
        bcc zer_closer
        beq zer_color
        brl zer_next
    zer_closer:
        /* Le plan seul ne doit pas faire ressortir une arête située derrière. */
        lda [12],y
        cmp #0xFFFF
        beq zer_store
        cmp 30
        bcs zer_store
        brl zer_next
    zer_store:
        lda 50
        sta [12],y
    zer_color:
        /* Modifier uniquement le nibble correspondant à la parité de x. */
        lda 20
        and #1
        bne zer_odd
        sep #0x20
        lda [16]
        and #0x0F
        ora 48
        sta [16]
        rep #0x20
        bra zer_next
    zer_odd:
        sep #0x20
        lda [16]
        and #0xF0
        ora 46
        sta [16]
        rep #0x20

    zer_next:
        dec 40
        bne zer_advance
        brl zer_done
    zer_advance:
        /* Avancer le plan Z sur l'axe majeur et la profondeur d'arête. */
        clc
        lda 0
        adc 4
        sta 0
        lda 2
        adc 6
        sta 2
        clc
        lda 30
        adc 32
        sta 30
        clc
        lda 36
        adc 34
        bcs zer_er_sub
        cmp 24
        bcc zer_er_save
    zer_er_sub:
        sec
        sbc 24
        sta 36
        clc
        lda 30
        adc 38
        sta 30
        bra zer_major
    zer_er_save:
        sta 36

    zer_major:
        /* L'axe majeur avance toujours de +1 ; propager les retenues des
         * pointeurs sans dépendre de la banque courante du processeur.
         */
        lda 42
        beq zer_major_x
        inc 22
        clc
        lda 12
        adc #640
        sta 12
        lda 14
        adc #0
        sta 14
        clc
        lda 16
        adc #160
        sta 16
        lda 18
        adc #0
        sta 18
        bra zer_remainder
    zer_major_x:
        inc 20
        clc
        lda 12
        adc #2
        sta 12
        lda 14
        adc #0
        sta 14
        lda 20
        and #1
        bne zer_remainder
        inc 16
        bne zer_remainder
        inc 18

    zer_remainder:
        /* Les restes sont non signés : un débordement 16 bits implique lui
         * aussi le franchissement du seuil major.
         */
        clc
        lda 28
        adc 26
        bcs zer_minor_sub
        cmp 24
        bcs zer_minor_sub
        sta 28
        brl zer_pixel
    zer_minor_sub:
        sec
        sbc 24
        sta 28
        clc
        lda 0
        adc 8
        sta 0
        lda 2
        adc 10
        sta 2
        lda 42
        beq zer_minor_y
        /* L'axe mineur est x : un octet écran pour deux pas horizontaux. */
        clc
        lda 20
        adc 44
        sta 20
        lda 44
        bmi zer_minor_x_neg
        clc
        lda 12
        adc #2
        sta 12
        lda 14
        adc #0
        sta 14
        lda 20
        and #1
        bne zer_xpos_done
        inc 16
        bne zer_xpos_done
        inc 18
    zer_xpos_done:
        brl zer_pixel
    zer_minor_x_neg:
        sec
        lda 12
        sbc #2
        sta 12
        lda 14
        sbc #0
        sta 14
        lda 20
        and #1
        beq zer_xneg_done
        lda 16
        bne zer_xneg_low
        dec 18
    zer_xneg_low:
        dec 16
    zer_xneg_done:
        brl zer_pixel

    zer_minor_y:
        /* L'axe mineur est y : déplacer les deux pointeurs d'une ligne. */
        clc
        lda 22
        adc 44
        sta 22
        lda 44
        bmi zer_minor_y_neg
        clc
        lda 12
        adc #640
        sta 12
        lda 14
        adc #0
        sta 14
        clc
        lda 16
        adc #160
        sta 16
        lda 18
        adc #0
        sta 18
        brl zer_pixel
    zer_minor_y_neg:
        sec
        lda 12
        sbc #640
        sta 12
        lda 14
        sbc #0
        sta 14
        sec
        lda 16
        sbc #160
        sta 16
        lda 18
        sbc #0
        sta 18
        brl zer_pixel

    zer_done:
        /* Libérer exactement le cadre avant de restaurer D et P. */
        tsc
        clc
        adc #52
        tcs
        pld
        plp
    }
}

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_DrawEdge
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Rasteriser un côté par Bresenham et tester sa visibilité après les
 *    surfaces.
 *
 *  Paramètres :
 *    [entrée] x0/y0, edge_z0 et x1/y1, edge_z1 en coordonnées écran et codes
 *    de sommets, color et plane commun.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Trie les extrémités selon l'axe majeur, clippe son parcours et avance à
 *    la fois le plan Z et la profondeur d'arête. Prépare un paquet par côté
 *    pour ZBuffer_EdgeRun ; aucun appel C par pixel ni allocation. Les règles
 *    de tie de Bresenham
 *    peuvent différer de FramePoly et du remplissage demi-ouvert.
 *
 * ----------------------------------------------------------------------------
 */
static void ZBuffer_DrawEdge(int x0, int y0, UWORD16 edge_z0,
                             int x1, int y1, UWORD16 edge_z1,
                             int color, ZbPlane *plane)
{
    long dx, dy, major, minor, first, last;
    long u0, v0, v, ulimit, vstep, rem;
    unsigned long numerator, raw, zmajor, zminor, magnitude;
    long ez, erem, ewhole, epart, esign, delta;
    int start_x, start_y;
    UWORD16 ztmp;
    int steep, tmp;
    /* Choisir l'axe majeur et normaliser le sens de parcours en échangeant
     * aussi les codes.
     */
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
    /* Calculer le domaine visible et les incréments entiers des coordonnées
     * et profondeurs.
     */
    u0 = steep ? (long)y0 : (long)x0;
    v0 = steep ? (long)x0 : (long)y0;
    major = steep ? dy : dx;
    minor = steep ? dx : dy;
    ulimit = steep ? ZBUF_HEIGHT - 1L : ZBUF_WIDTH - 1L;
    vstep = ((steep ? x1 : y1) < v0) ? -1L : 1L;
    if (major == 0L) {
        /* Un point unique emprunte le même noyau ; aucun incrément ne sera lu. */
        first = last = 0L;
        v = v0;
        raw = ZBuffer_PlaneAt(plane, x0, y0);
        ez = (edge_z0 < edge_z1) ? edge_z0 : edge_z1;
        zmajor = zminor = 0UL;
        rem = erem = ewhole = epart = 0L;
        esign = 1L;
        goto prepare_packet;
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

    delta = (long)edge_z1 - (long)edge_z0;
    esign = (delta < 0L) ? -1L : 1L;
    magnitude = (unsigned long)((delta < 0L) ? -delta : delta);
    ewhole = (long)(magnitude / (unsigned long)major);
    epart = (long)(magnitude % (unsigned long)major);
    numerator = (unsigned long)first * magnitude + (unsigned long)(major / 2L);
    ez = (long)edge_z0 + esign * (long)(numerator / (unsigned long)major);
    erem = (long)(numerator % (unsigned long)major);
prepare_packet:
    /* Les extrémités sont des int 16 bits ; major/minor/restes tiennent donc
     * dans un mot non signé. Le parcours visible compte au plus 320 pixels.
     * Même hors viewport, construire les adresses par arithmétique sans lire
     * de ligne invalide ; le noyau filtre les coordonnées avant tout accès.
     */
    start_x = steep ? (int)v : (int)(u0 + first);
    start_y = steep ? (int)(u0 + first) : (int)v;
    zbfs_edge_packet.raw = raw;
    zbfs_edge_packet.zmajor = zmajor;
    zbfs_edge_packet.zminor = zminor;
    zbfs_edge_packet.zptr = (unsigned long)zbuf_row[0] +
                           (unsigned long)((long)start_y * ZBUF_ROW_BYTES +
                                           (long)start_x * 2L);
    zbfs_edge_packet.pptr = ZB_SHGR_BASE +
                           (unsigned long)((long)start_y * 160L +
                                           ((long)start_x >> 1));
    zbfs_edge_packet.x = (unsigned int)start_x;
    zbfs_edge_packet.y = (unsigned int)start_y;
    zbfs_edge_packet.major = (unsigned int)major;
    zbfs_edge_packet.minor = (unsigned int)minor;
    zbfs_edge_packet.rem = (unsigned int)rem;
    zbfs_edge_packet.ez = (unsigned int)ez;
    zbfs_edge_packet.ezstep = (unsigned int)(esign * ewhole);
    zbfs_edge_packet.epart = (unsigned int)epart;
    zbfs_edge_packet.erem = (unsigned int)erem;
    zbfs_edge_packet.esign = (unsigned int)esign;
    zbfs_edge_packet.count = (unsigned int)(last - first + 1L);
    zbfs_edge_packet.steep = (unsigned int)steep;
    zbfs_edge_packet.vstep = (unsigned int)vstep;
    zbfs_edge_packet.color_lo = color & 15;
    zbfs_edge_packet.color_hi = (color & 15) << 4;
    ZBuffer_EdgeRun();
}

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_RenderCore
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Rendre les surfaces par balayage plein écran direct.
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
 *    uniquement les faces actives. Plan Z commun au remplissage et aux
 *    contours ; arrondi intérieur ceil/floor, puis côtés originaux par
 *    Bresenham et masque facultatif.
 *
 *    Le clipping du viewport est appliqué aux spans ; pas de clipping 3D
 *    général. Les réserves et le comportement d'échec restent propres à
 *    cette version.
 *
 * ----------------------------------------------------------------------------
 */
static void ZBuffer_RenderCore(Model3D* model, int biased)
{
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;
    int fcount = faces->face_count;
    int y, f, i, k, ai, keep, active_count;
    int total_edges, end;
    int clip_x_min, clip_x_max, y_lo, y_hi;
    float frame_max_inv_z, projection, frame_code_bias;
    ZbEdge* ed;
    ZbHit hits[MAX_SPAN_INTERSECTIONS];

    static int bucket[ZBUF_HEIGHT + 1];
    static int bucket_cur[ZBUF_HEIGHT + 1];

    if (!ZBuffer_SizeFits(vcount, sizeof(float)) ||
        !ZBuffer_SizeFits(vcount, sizeof(UWORD16)) ||
        !ZBuffer_SizeFits(fcount, sizeof(int)) ||
        !ZBuffer_SizeFits(fcount, sizeof(ZbPlane))) return;
    total_edges = 0;
    for (f = 0; f < fcount; f++) {
        long edge_end = (long)faces->vertex_indices_ptr[f] +
                        (long)faces->vertex_count[f];
        if (faces->vertex_indices_ptr[f] < 0 || faces->vertex_count[f] < 0 ||
            edge_end > 32767L) return;
        end = (int)edge_end;
        if (end > total_edges) total_edges = end;
    }
    if (!ZBuffer_SizeFits(total_edges, sizeof(ZbEdge))) return;
    /* Réserver les tables de faces et les plans Z du rendu direct. */
    if (zbfs_faces.capacity < fcount) {
        ZBFS_FreeFaces();
        if (!ZBuffer_ReserveFaces(&zbfs_faces, fcount)) return;
        zbfs_face_plane = (ZbPlane*)malloc((size_t)fcount * sizeof(ZbPlane));
        if (!zbfs_face_plane) { ZBFS_FreeFaces(); return; }
    }
    /* Agrandir les buffers de sommets du rendu direct. */
    if (!ZBuffer_ReserveVertices(&zbfs_vertices, vcount)) return;
    if (zbfs_edge_capacity < total_edges) {
        ZBFS_FreeEdges();
        zbfs_edge_buf = (ZbEdge*)malloc((size_t)total_edges * sizeof(ZbEdge));
        if (!zbfs_edge_buf) return;
        zbfs_edge_capacity = total_edges;
    }

    frame_max_inv_z = ZBuffer_PrepareInvZ(vtx, vcount, zbfs_vertices.inv_z);
    ZBuffer_SetScaleForFrame(frame_max_inv_z);
    /* Sortir des calculs par face les constantes de projection et de biais. */
    projection = FIXED_TO_FLOAT(s_global_proj_scale_fixed);
    zbfs_inv_projection = (projection > 0.0f) ? 1.0f / projection : 0.0f;
    frame_code_bias = biased ? zbuffer_scale * ZB_Z_BIAS : 0.0f;

    for (i = 0; i < vcount; i++) {

        zbfs_vertices.vzq[i]   = ZBuffer_QuantizeInvZ(zbfs_vertices.inv_z[i]);
        zbfs_vertices.vzq_b[i] = biased ? ZBuffer_QuantizeInvZ(zbfs_vertices.inv_z[i] + ZB_Z_BIAS) : zbfs_vertices.vzq[i];
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
    ZBuffer_PrepareFaces(faces, fcount, y_lo, y_hi, ZBUF_HEIGHT, biased,
                         zbfs_faces.face_start, zbfs_faces.face_fill, zbfs_faces.face_frame,
                         zbfs_faces.face_nudge, bucket);
    for (f = 0; f < fcount; ++f) {
        int n, offt;
        UWORD16 *zsrc;
        if (zbfs_faces.face_start[f] < 0) continue;
        n = faces->vertex_count[f];
        offt = faces->vertex_indices_ptr[f];
        zsrc = (biased && faces->plane_d[f] > 0) ? zbfs_vertices.vzq_b : zbfs_vertices.vzq;
        ZBuffer_SetPlane(&zbfs_face_plane[f], vtx, faces, f, zsrc,
                         zbfs_faces.face_nudge[f],
                         (biased && faces->plane_d[f] > 0) ? frame_code_bias : 0.0f);
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
                ed->ya = 32767; ed->yb = 32767;
                ed->x = 0L; ed->sx = 0L;
                continue;
            }

            sx = (((long)vtx->x2d[vid2] - (long)vtx->x2d[vid1]) * 4096L) / (long)dy;

            if (dy > 0) { ya = y1; yb = y2; xs = 0; }
            else        { ya = y2; yb = y1; xs = vtx->x2d[vid2] - vtx->x2d[vid1]; }

            ex = (long)xs * 4096L;

            if (ya < y_lo && yb > y_lo) {
                skip = (long)(y_lo - ya);
                ex += sx * skip;
            }

            ed->ya = ya;  ed->yb = yb;
            ed->xb = vtx->x2d[vid1];
            ed->x  = ex;
            ed->sx = sx;
        }
    }

    ZBuffer_SortFaceStarts(fcount, ZBUF_HEIGHT, zbfs_faces.face_start,
                           bucket, bucket_cur, zbfs_faces.face_sorted);

    screenClearTopLines(12);

    /* Démarrer la liste des faces actives, puis balayer les lignes visibles. */
    active_count = 0;
    for (y = y_lo; y <= y_hi; y++) {
        int screenY = y + pan_dy;
        int row = y - y_lo;

        /* Insérer les faces commençant sur cette ligne en gardant l'ordre
         * d'identifiants.
         */
        active_count = ZBuffer_ActivateFaces(row, bucket, zbfs_faces.face_sorted,
                                             zbfs_faces.face_active, active_count);

        keep = 0;
        /* Écarter les faces terminées et extraire les intersections des
         * arêtes actives.
         */
        for (ai = 0; ai < active_count; ai++) {
            int n, offt, hit_count;
            int fillColor;

            f = zbfs_faces.face_active[ai];
            if (faces->maxy[f] < y) continue;
            zbfs_faces.face_active[keep++] = f;

            n = faces->vertex_count[f];
            offt = faces->vertex_indices_ptr[f];
            hit_count = 0;

            ed = zbfs_edge_buf + offt;
            for (k = 0; k < n; k++, ed++) {
                if (y < ed->ya || y >= ed->yb) continue;
                if (hit_count < MAX_SPAN_INTERSECTIONS) {
                    hits[hit_count].x = (long)ed->xb * 4096L + ed->x;
                    hit_count++;
                }

                ed->x += ed->sx;
            }

            if (hit_count < 2) continue;

            /* Ordonner les intersections : échange simple pour deux, tri par
             * insertion au-delà.
             */
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

            fillColor  = zbfs_faces.face_fill[f];

            {
                int p;
                /* Apparier les intersections par parité, clipper les spans
                 * et les transmettre au noyau de dessin.
                 */
                for (p = 0; p + 1 < hit_count; p += 2) {
                    long left = hits[p].x;
                    long right = hits[p + 1].x;
                    long xa, xb;
                    int x0, x1;

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

    for (f = 0; f < fcount; f++) {
        int n, offt, color, nudge;
        UWORD16 *zsrc;
        if (zbfs_faces.face_start[f] < 0) continue;
        n = faces->vertex_count[f];
        offt = faces->vertex_indices_ptr[f];
        color = zbfs_faces.face_frame[f];
        nudge = zbfs_faces.face_nudge[f];
        zsrc = (biased && faces->plane_d[f] > 0) ? zbfs_vertices.vzq_b : zbfs_vertices.vzq;
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

/*
 * ----------------------------------------------------------------------------
 *  renderModelFullscreenZBuffer_fastV2
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Appeler ZBuffer_RenderCore sans rapprochement des faces de plane_d>0.
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
void renderModelFullscreenZBuffer_fastV2(Model3D* model)
{
    /* Transmettre explicitement le choix du biais au noyau commun. */
    ZBuffer_RenderCore(model, 0);
}

/*
 * ----------------------------------------------------------------------------
 *  renderModelFullscreenZBuffer_biasedV2
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Appeler ZBuffer_RenderCore avec rapprochement des faces de plane_d>0.
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
void renderModelFullscreenZBuffer_biasedV2(Model3D* model)
{
    /* Transmettre explicitement le choix du biais au noyau commun. */
    ZBuffer_RenderCore(model, 1);
}

/*
 * ----------------------------------------------------------------------------
 *  renderModelFullscreenZBuffer
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
void renderModelFullscreenZBuffer(Model3D* model)
{
    /* Sélectionner le noyau avec la politique de biais correspondant à la
     * visibilité.
     */
    if (cull_back_faces) {
        renderModelFullscreenZBuffer_fastV2(model);
    } else {
        renderModelFullscreenZBuffer_biasedV2(model);
    }
}
