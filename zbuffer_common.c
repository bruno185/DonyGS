/*
 * ============================================================================
 *  zbuffer_common.c — Primitives communes aux rendus Z-buffer
 * ============================================================================
 *
 *  Rôle :
 *    Fournir les réserves et préparations communes aux trois rendus Z-buffer,
 *    ainsi que leur noyau de spans 65C816.
 *
 *  Organisation :
 *    Regroupe les buffers typés privés de chaque moteur, la profondeur
 *    inverse, la quantification, les débuts de faces, les listes actives,
 *    les arêtes scanline/offscreen, leur tri et l'interpolation des spans.
 *    Le paquet zspan_p_* et le noyau assembleur servent aux trois rendus.
 *
 *  Dépendances et contraintes :
 *    Inclus par engine.c avant les moteurs Z-buffer. Sur ORCA/C, UWORD16
 *    et unsigned int sont non signés sur 16 bits ; long est signé sur 32 bits.
 *    Les produits intermédiaires doivent rester représentables. Les moteurs
 *    conservent leurs réserves, échelles, biais et règles de contour propres.
 *    Le paquet de spans reste partagé et non réentrant.
 *
 * ============================================================================
 */
#include "zbuffer_fullscreen.h"

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_LerpZ
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Interpoler un code de profondeur lors du clipping horizontal d'un span.
 *
 *  Paramètres :
 *    [entrée] za/zb : codes de profondeur aux abscisses xa/xb.
 *    [entrée] xa/xb : abscisses des extrémités ; x : abscisse demandée.
 *
 *  Retour :
 *    za si xa == xb ; sinon za + (zb-za)*(x-xa)/(xb-xa), converti en UWORD16.
 *
 *  Fonctionnement :
 *    Conserve les opérations en long et la troncature de la division entière
 *    des anciennes variantes scanline et offscreen. Aucun état modifié.
 *
 *  Limites :
 *    Aucune saturation ; les différences et produits doivent être valides
 *    dans les types intermédiaires. L'appelant choisit le domaine de x.
 *
 * ----------------------------------------------------------------------------
 */
static UWORD16 ZBuffer_LerpZ(UWORD16 za, UWORD16 zb, int xa, int xb, int x)
{
    long num;
    /* Traiter les extrémités confondues avant la division. */
    if (xb == xa) return za;
    /* Interpoler avec les mêmes conversions et le même ordre d'opérations. */
    num = ((long)zb - (long)za) * (long)(x - xa);
    return (UWORD16)((long)za + num / (long)(xb - xa));
}

/* Chaque moteur possède ses réserves ; seules leur gestion et leur forme
 * sont communes. Aucune mémoire de face ou de sommet n'est partagée entre rendus.
 */
typedef struct {
    float *inv_z;
    UWORD16 *vzq, *vzq_b;
    int capacity;
} ZBufferVertexWork;
typedef struct {
    int *face_start, *face_sorted, *face_active;
    unsigned char *face_fill, *face_frame, *face_nudge;
    int capacity;
} ZBufferFaceWork;

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_SizeFits
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Vérifier qu'un tableau de count éléments est représentable par size_t.
 *
 *  Paramètres :
 *    [entrée] count positif attendu et element_size non nul.
 *
 *  Retour :
 *    Booléen.
 *
 *  Fonctionnement :
 *    La division du maximum size_t évite de déborder lors du produit de
 *    taille.
 *
 * ----------------------------------------------------------------------------
 */
static int ZBuffer_SizeFits(int count, size_t element_size)
{
    /* Vérifier la borne avant toute multiplication destinée à malloc. */
    return count > 0 && (unsigned long)count <=
           (unsigned long)((size_t)-1) / (unsigned long)element_size;
}

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_FreeVertices
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Libérer une réserve de profondeurs de sommets.
 *
 *  Paramètres :
 *    [entrée-sortie] work : réserve possédée par un moteur.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Libère les trois buffers et neutralise tous les pointeurs et la
 *    capacité. Une réserve vide peut être libérée à nouveau.
 *
 * ----------------------------------------------------------------------------
 */
static void ZBuffer_FreeVertices(ZBufferVertexWork *work)
{
    /* Chaque pointeur est possédé exclusivement par cette réserve. */
    free(work->inv_z); free(work->vzq); free(work->vzq_b);
    memset(work, 0, sizeof(*work));
}

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_ReserveVertices
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Garantir la capacité des trois buffers de profondeur.
 *
 *  Paramètres :
 *    [entrée-sortie] work : réserve privée du moteur ; count : capacité
 *    requise.
 *
 *  Retour :
 *    1 si disponible, 0 si taille invalide ou échec d’allocation.
 *
 *  Fonctionnement :
 *    Réutilise la réserve suffisante. Sinon libère l’ancienne, vérifie les
 *    tailles puis alloue les trois buffers ; un échec laisse une réserve
 *    entièrement vide.
 *
 * ----------------------------------------------------------------------------
 */
static int ZBuffer_ReserveVertices(ZBufferVertexWork *work, int count)
{
    if (count <= work->capacity) return count >= 0;
    ZBuffer_FreeVertices(work);
    /* Vérifier les produits dans le size_t 16 bits de la cible. */
    if (!ZBuffer_SizeFits(count, sizeof(float)) || !ZBuffer_SizeFits(count, sizeof(UWORD16))) return 0;
    work->inv_z = (float*)malloc((size_t)count * sizeof(float));
    work->vzq = (UWORD16*)malloc((size_t)count * sizeof(UWORD16));
    work->vzq_b = (UWORD16*)malloc((size_t)count * sizeof(UWORD16));
    if (!work->inv_z || !work->vzq || !work->vzq_b) {
        ZBuffer_FreeVertices(work);
        return 0;
    }
    work->capacity = count;
    return 1;
}

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_FreeFaces
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Libérer les tables communes de préparation des faces.
 *
 *  Paramètres :
 *    [entrée-sortie] work : réserve privée du moteur.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Libère débuts, ordre, liste active, couleurs et nudges, puis neutralise
 *    la réserve. Les plans du direct et les liaisons offscreen restent
 *    possédés par leurs moteurs.
 *
 * ----------------------------------------------------------------------------
 */
static void ZBuffer_FreeFaces(ZBufferFaceWork *work)
{
    /* Les données spécifiques aux contours ne sont pas détenues ici. */
    free(work->face_start); free(work->face_sorted); free(work->face_active);
    free(work->face_fill); free(work->face_frame); free(work->face_nudge);
    memset(work, 0, sizeof(*work));
}

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_ReserveFaces
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Garantir les six tables communes de faces.
 *
 *  Paramètres :
 *    [entrée-sortie] work : réserve privée ; count : nombre de faces.
 *
 *  Retour :
 *    1 si les tables sont disponibles, 0 sinon.
 *
 *  Fonctionnement :
 *    Réutilise la capacité suffisante, vérifie les tailles puis alloue les
 *    six tables. Sur échec, libère le jeu incomplet et laisse une réserve
 *    vide pour le prochain essai.
 *
 * ----------------------------------------------------------------------------
 */
static int ZBuffer_ReserveFaces(ZBufferFaceWork *work, int count)
{
    if (count <= work->capacity) return count >= 0;
    ZBuffer_FreeFaces(work);
    /* La table d'entiers est la plus grande allocation par face de ce groupe. */
    if (!ZBuffer_SizeFits(count, sizeof(int))) return 0;
    work->face_start = (int*)malloc((size_t)count * sizeof(int));
    work->face_sorted = (int*)malloc((size_t)count * sizeof(int));
    work->face_active = (int*)malloc((size_t)count * sizeof(int));
    work->face_fill = (unsigned char*)malloc((size_t)count);
    work->face_frame = (unsigned char*)malloc((size_t)count);
    work->face_nudge = (unsigned char*)malloc((size_t)count);
    if (!work->face_start || !work->face_sorted || !work->face_active ||
        !work->face_fill || !work->face_frame || !work->face_nudge) {
        ZBuffer_FreeFaces(work);
        return 0;
    }
    work->capacity = count;
    return 1;
}

/* Types communs des intersections et arêtes quantifiées scanline/offscreen.
 * Le rendu direct garde ses arêtes X et ses plans Z spécifiques.
 */
typedef struct { int x; UWORD16 zq; } ZBufferScanHit;
typedef struct { long x,z,sx,sz; int ya,yb,xb; } ZBufferScanEdge;

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_QuantizeAtScale
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Quantifier une profondeur inverse avec une échelle explicite.
 *
 *  Paramètres :
 *    [entrée] inv_z : profondeur inverse ; scale : échelle du moteur
 *    appelant.
 *
 *  Retour :
 *    Code 16 bits inversé et saturé ; 65535 représente le fond.
 *
 *  Fonctionnement :
 *    Conserve les opérations float, l’arrondi par +0,5 puis la conversion en
 *    long des deux anciens quantificateurs. Ne modifie aucune échelle.
 *
 * ----------------------------------------------------------------------------
 */
static UWORD16 ZBuffer_QuantizeAtScale(float inv_z, float scale)
{
    float scaled;
    long q;
    /* Borner, quantifier puis inverser, sans changer les arrondis historiques. */
    if (inv_z < 0.0f) inv_z = 0.0f;
    scaled = inv_z * scale;
    q = (long)(scaled + 0.5f);
    if (q < 0) q = 0;
    if (q > 0xFFFFL) q = 0xFFFFL;
    return (UWORD16)(0xFFFFUL - (unsigned long)q);
}

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_PrepareInvZ
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Préparer les profondeurs inverses des sommets d’une vue.
 *
 *  Paramètres :
 *    [entrée] vtx : tableaux de sommets ; count : nombre de sommets.
 *    [sortie] inv_z : buffer de count flottants déjà alloué.
 *
 *  Retour :
 *    Maximum des profondeurs inverses, ou zéro.
 *
 *  Fonctionnement :
 *    Convertit zo par FIXED_TO_FLOAT, calcule 1/z pour les profondeurs
 *    positives et écrit zéro sinon. Le maximum sert à la politique d’échelle
 *    propre à chaque moteur.
 *
 * ----------------------------------------------------------------------------
 */
static float ZBuffer_PrepareInvZ(VertexArrays3D *vtx, int count, float *inv_z)
{
    float maximum = 0.0f;
    /* Garder exactement le format et les comparaisons des trois moteurs. */
    for (int i = 0; i < count; ++i) {
        float zo_f = FIXED_TO_FLOAT(vtx->zo[i]);
        inv_z[i] = (zo_f > 0.0f) ? (1.0f / zo_f) : 0.0f;
        if (inv_z[i] > maximum) maximum = inv_z[i];
    }
    return maximum;
}

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_PrepareFaces
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Filtrer les faces et préparer leurs premières lignes et couleurs.
 *
 *  Paramètres :
 *    [entrée] faces/count : faces de la vue ; y_lo/y_hi : bornes verticales ;
 *    height : hauteur écran ; biased : biais actif.
 *    [sortie] start, fill, frame, nudge : buffers par face ; bucket :
 *    histogramme de height+1 entiers.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Initialise l’histogramme, ignore les faces masquées, courtes ou hors
 *    écran, puis conserve les règles de couleurs et le nudge de deux unités
 *    pour les faces avant biaisées. Les arêtes restent préparées par les
 *    moteurs ou le helper dédié.
 *
 * ----------------------------------------------------------------------------
 */
static void ZBuffer_PrepareFaces(FaceArrays3D *faces, int count,
                                 int y_lo, int y_hi, int height, int biased,
                                 int *start, unsigned char *fill,
                                 unsigned char *frame, unsigned char *nudge,
                                 int *bucket)
{
    /* Réinitialiser les débuts de lignes avant le filtrage des faces. */
    for (int b = 0; b <= height; ++b) bucket[b] = 0;
    for (int f = 0; f < count; ++f) {
        int fmin, fmax, b, color;
        start[f] = -1;
        if (!faces->display_flag[f] || faces->vertex_count[f] < 3) continue;
        fmin = faces->miny[f]; fmax = faces->maxy[f];
        if (fmax < y_lo || fmin > y_hi) continue;
        b = ((fmin > y_lo) ? fmin : y_lo) - y_lo;
        start[f] = b;
        bucket[b + 1]++;
        /* Les couleurs restent résolues par les fonctions publiques existantes. */
        color = getFaceFillColor(f);
        fill[f] = (unsigned char)color;
        frame[f] = (unsigned char)getFaceFrameColor(f, color);
        nudge[f] = (unsigned char)((biased && faces->plane_d[f] > 0) ? 2 : 0);
    }
}

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_SortFaceStarts
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Transformer l’histogramme en liste stable des départs de faces.
 *
 *  Paramètres :
 *    [entrée] count, height et start : dimensions et première ligne par face.
 *    [entrée-sortie] bucket : histogramme devenant offsets ; cursor :
 *    workspace ; sorted : liste de sortie.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Effectue les sommes préfixes puis distribue les faces par identifiant
 *    croissant. Préserve les égalités de profondeur et l’ordre d’activation
 *    des anciens parcours.
 *
 * ----------------------------------------------------------------------------
 */
static void ZBuffer_SortFaceStarts(int count, int height, const int *start,
                                   int *bucket, int *cursor, int *sorted)
{
    /* Produire les intervalles de chaque ligne puis distribuer les faces. */
    for (int b = 0; b < height; ++b) bucket[b + 1] += bucket[b];
    for (int b = 0; b <= height; ++b) cursor[b] = bucket[b];
    for (int f = 0; f < count; ++f) {
        int b = start[f];
        if (b >= 0) sorted[cursor[b]++] = f;
    }
}

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_ActivateFaces
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Ajouter les faces d’une ligne à la liste active ordonnée.
 *
 *  Paramètres :
 *    [entrée] row, bucket et sorted : ligne et départs préparés.
 *    [entrée-sortie] active : buffer de faces actives ; count : longueur
 *    actuelle.
 *
 *  Retour :
 *    Nouvelle longueur de la liste active.
 *
 *  Fonctionnement :
 *    Insère chaque nouvelle face par identifiant croissant, sans modifier les
 *    bornes ni les règles de fin de vie des faces. Le buffer est dimensionné
 *    au nombre total de faces.
 *
 * ----------------------------------------------------------------------------
 */
static int ZBuffer_ActivateFaces(int row, const int *bucket, const int *sorted,
                                 int *active, int count)
{
    /* L’ordre d’identifiants conserve les mêmes priorités aux égalités Z. */
    for (int i = bucket[row]; i < bucket[row + 1]; ++i) {
        int face = sorted[i], pos = count;
        while (pos > 0 && active[pos - 1] > face) {
            active[pos] = active[pos - 1];
            --pos;
        }
        active[pos] = face;
        ++count;
    }
    return count;
}

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_PrepareScanEdges
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Préparer les arêtes quantifiées communes à scanline et offscreen.
 *
 *  Paramètres :
 *    [entrée] vtx, faces et face : contour projeté ; zsrc : codes Z par
 *    sommet ; y_lo : première ligne ; rounding : biais d’arrondi.
 *    [sortie] ed : buffer de vertex_count[face] arêtes.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Conserve le format 12 bits fractionnaires, les arêtes horizontales
 *    neutralisées, les divisions signées et le clipping vertical des anciens
 *    moteurs. Le rendu direct garde son calcul de plan et ses arêtes propres.
 *
 * ----------------------------------------------------------------------------
 */
static void ZBuffer_PrepareScanEdges(VertexArrays3D *vtx, FaceArrays3D *faces,
                                     int face, const UWORD16 *zsrc, int y_lo,
                                     long rounding, ZBufferScanEdge *ed)
{
    int offt = faces->vertex_indices_ptr[face];
    int n = faces->vertex_count[face], k;
    /* Appliquer le même noyau aux deux buffers possédés par leurs moteurs. */
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
        ed->x  = ex + rounding;
        ed->z  = ez + rounding;
        ed->sx = sx;  ed->sz = sz;
    }
}

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_SortScanHits
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Ordonner les intersections quantifiées d’un span.
 *
 *  Paramètres :
 *    [entrée-sortie] hits : tableau de count intersections ; count : longueur
 *    valide.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Échange les deux intersections si nécessaire, sinon utilise l’insertion
 *    stable selon x. Les codes Z restent attachés à leurs abscisses.
 *
 * ----------------------------------------------------------------------------
 */
static void ZBuffer_SortScanHits(ZBufferScanHit *hits, int hit_count)
{
    /* Préserver le tri stable et son chemin court à deux intersections. */
    if (hit_count == 2) {
        if (hits[0].x > hits[1].x) {
            ZBufferScanHit tmp = hits[0];
            hits[0] = hits[1];
            hits[1] = tmp;
        }
    } else {
        int a, c;
        for (a = 1; a < hit_count; a++) {
            ZBufferScanHit key = hits[a];
            c = a - 1;
            while (c >= 0 && hits[c].x > key.x) {
                hits[c + 1] = hits[c];
                c--;
            }
            hits[c + 1] = key;
        }
    }
}

/* Placer le noyau partagé dans le segment des deux rendus plein écran. */
segment "ZBUF";

/* Paquet d'entrée commun, préparé intégralement selon le mode par
 * l'appelant avant chaque span. Le noyau ne le modifie pas. Ces données
 * partagées imposent des appels synchrones, sans réentrance.
 */
long zspan_p_zacc, zspan_p_zstep;
unsigned long zspan_p_zptr, zspan_p_pptr;
int zspan_p_n, zspan_p_odd, zspan_p_mode, zspan_p_cl, zspan_p_ci, zspan_p_cr;
unsigned int zspan_p_zl, zspan_p_zr;

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_SpanRun
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Dessiner un span préclippé avec test Z strict dans un noyau 65C816.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Entrées et sorties implicites :
 *    Entrée : paquet partagé zspan_p_* : pointeurs Z et pixels far, parité
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
static void ZBuffer_SpanRun(void)
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
        lda >zspan_p_zacc
        sta 0
        lda >zspan_p_zacc+2
        sta 2
        lda >zspan_p_zstep
        sta 4
        lda >zspan_p_zstep+2
        sta 6
        lda >zspan_p_zptr
        sta 8
        lda >zspan_p_zptr+2
        sta 10
        lda >zspan_p_pptr
        sta 12
        lda >zspan_p_pptr+2
        sta 14
        lda >zspan_p_n
        sta 16

        lda >zspan_p_ci
        and #0x000F
        sta 18
        asl a
        asl a
        asl a
        asl a
        sta 20

        lda >zspan_p_odd
        eor #1
        sta 22

        ldy #0

        lda >zspan_p_odd
        beq zbr_l_ev
        lda #0x00F0
        sta 24
        lda >zspan_p_cl
        and #0x000F
        sta 26
        bra zbr_l_go
    zbr_l_ev:
        lda #0x000F
        sta 24
        lda >zspan_p_cl
        and #0x000F
        asl a
        asl a
        asl a
        asl a
        sta 26
    /* Tester la profondeur exacte du pixel gauche avant toute écriture du nibble. */
    zbr_l_go:

        lda >zspan_p_zl
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

        lda >zspan_p_mode
        bne zbr_full
        brl zbr_done

    /* Avancer après le pixel gauche ; l'adresse couleur progresse seulement
     * après un pixel impair.
     */
    zbr_full:
        iny
        iny
        lda >zspan_p_odd
        beq zbr_lev
        inc 12
    zbr_lev:

        lda 16
        bne zbr_int
        brl zbr_right

    /* Traiter seul un premier pixel impair pour aligner le reste sur des paires. */
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

    /* Avancer zacc en 32 bits pour chaque pixel ; propager C du mot bas au
     * mot haut, puis tester Z.
     */
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

    /* Traiter l'éventuel pixel pair résiduel après les paires. */
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

    /* Recalculer la parité et employer le code exact de l'extrémité droite. */
    zbr_right:
        lda >zspan_p_n
        clc
        adc 22
        and #1
        beq zbr_r_ev
        lda #0x00F0
        sta 24
        lda >zspan_p_cr
        and #0x000F
        sta 26
        bra zbr_r_go
    zbr_r_ev:
        lda #0x000F
        sta 24
        lda >zspan_p_cr
        and #0x000F
        asl a
        asl a
        asl a
        asl a
        sta 26
    zbr_r_go:
        lda >zspan_p_zr
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

    /* Libérer exactement le cadre avant de restaurer D et P. */
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

/* Rétablir le segment utilisé par le module scanline inclus ensuite. */
segment "data";
