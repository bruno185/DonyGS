/*
 * ============================================================================
 *  zbuffer_scanline.c — Z-buffer scanline à profondeur quantifiée
 * ============================================================================
 *
 *  Rôle :
 *    Résoudre la visibilité par pixel avec une seule ligne de profondeur
 *    réutilisée, à partir des coordonnées projetées et des métriques déjà
 *    préparées par le moteur.
 *
 *  Organisation :
 *    Le dispatch public sélectionne le moteur V3, qui prépare les codes
 *    16 bits de
 *    profondeur inverse, les arêtes et les faces actives, puis dessine
 *    chaque span par un noyau assembleur. Le fichier fournit aussi les
 *    couleurs effectives et drawPixel.
 *
 *  Dépendances et contraintes :
 *    Inclus par engine.c ; écran SHGR 320x200, deux pixels par octet.
 *    Sans culling, la variante active emploie un biais. Les
 *    buffers statiques et paquets de spans sont non réentrants ; la mémoire
 *    Z seule ne résume pas leur consommation totale.
 *
 * ============================================================================
 */

/*
 * ----------------------------------------------------------------------------
 *  getFaceFillColor
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Résoudre la couleur de remplissage d'une face.
 *
 *  Paramètres :
 *    [entrée] face_id valide base zéro
 *
 *  Retour :
 *    Indice selon priorité shading, aléatoire disponible, choix utilisateur,
 *    défaut.
 *
 *  Fonctionnement :
 *    Aucun dessin ni modification de l'état. Les sélections spéciales
 *    doivent être cohérentes avec les buffers.
 *
 * ----------------------------------------------------------------------------
 */
int getFaceFillColor(int face_id) {
    int fill_color;
    /* Appliquer la priorité d'orientation avant les autres sources de couleur. */
    if (shaded_by_orientation) {
        fill_color = face_shade_color[face_id];
    } else if (user_fill_color == 16 && random_fill_colors != NULL && face_id < random_colors_capacity) {
        fill_color = random_fill_colors[face_id];
    } else if (user_fill_color >= 0) {
        fill_color = user_fill_color;
    } else {
        fill_color = COL_FILL_DEFAULT;
    }
    return fill_color;
}

/*
 * ----------------------------------------------------------------------------
 *  getFaceFrameColor
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Résoudre la couleur de contour d'une face.
 *
 *  Paramètres :
 *    [entrée] face_id valide et fill_color effectif
 *
 *  Retour :
 *    Remplissage si choix 17, couleur aléatoire disponible si 16, choix
 *    explicite ou COL_FRAME.
 *
 *  Fonctionnement :
 *    Aucun dessin ; ne recalcule pas le remplissage.
 *
 * ----------------------------------------------------------------------------
 */
int getFaceFrameColor(int face_id, int fill_color) {
    int frame_color;
    /* Traiter le contour identique au remplissage avant les sélections
     * indépendantes.
     */
    if (user_frame_color == 17) {
        frame_color = fill_color;
    } else if (user_frame_color == 16 && random_frame_colors != NULL && face_id < random_colors_capacity) {
        frame_color = random_frame_colors[face_id];
    } else if (user_frame_color >= 0) {
        frame_color = user_frame_color;
    } else {
        frame_color = COL_FRAME;
    }
    return frame_color;
}

/*
 * ----------------------------------------------------------------------------
 *  drawPixel
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Écrire directement un pixel SHGR 320x200 en préservant son voisin.
 *
 *  Paramètres :
 *    [entrée] x dans [0,319], y dans [0,199], color masqué sur quatre bits.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Aucun clipping ni test Z. Adresse $E12000+y*160+x/2. L'assembleur passe
 *    A en 8 bits, laisse X sur 16 bits et termine A sur 16 bits. A/X et
 *    indicateurs modifiés ; DBR ignoré par l'adressage long. Pixel pair dans
 *    le nibble haut, impair dans le nibble bas.
 *
 * ----------------------------------------------------------------------------
 */
void drawPixel(int x, int y, int color)
{
    int offset;
    unsigned char value;

    color &= 0x0F;

    /* Calculer l'offset d'octet dans la banque SHGR. */
    offset = y * 160 + (x >> 1);

    asm {
        /* Lire l'octet en mode A 8 bits et sauver sa valeur avant le choix de parité. */
        sep     #0x20

        ldx     offset
        lda     0xE12000,x

        pha

        lda     x
        and     #0x01
        bne     pixel_odd

        pla
        and     #0x0F
        sta     value

        lda     color
        asl     a
        asl     a
        asl     a
        asl     a
        and     #0xF0
        ora     value
        sta     0xE12000,x
        bra     done

/* Modifier uniquement le nibble bas pour x impair. */
pixel_odd:

        pla
        and     #0xF0
        sta     value

        lda     color
        and     #0x0F
        ora     value
        sta     0xE12000,x

/* Revenir au format d'accumulateur 16 bits attendu par le C. */
done:
        rep     #0x20
    }
}

typedef UWORD16 SLZWord;
#define SL_Z_FRAC 12
#define SL_Z_ROUND 0x800L
#define SL_Z_BIAS 0.0005f

#define SL_SHGR_BASE 0xE12000UL
#define SL_TARGET_MAX_CODE 65000.0f

static SLZWord sl_zline[SCREEN_WIDTH];
static float sl_zscale = 1.0f;

/*
 * ----------------------------------------------------------------------------
 *  SL_ClearLine
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Réinitialiser la ligne Z réutilisée par le rendu scanline V3.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    Aucun ; écrit SCREEN_WIDTH mots $FFFF dans sl_zline.
 *
 *  Fonctionnement :
 *    Les accès globaux longs évitent la dépendance à DBR.
 *
 *  Registres et indicateurs :
 *    Préserve P, modifie A/X.
 *
 * ----------------------------------------------------------------------------
 */
static void SL_ClearLine(void)
{
    asm {
        php
        rep #0x30
        ldx #0
        lda #0xFFFF
/* Remplir chaque mot avant de rendre la ligne au parcours suivant. */
slcl_loop:
        sta >sl_zline,x
        inx
        inx
        cpx #SCREEN_WIDTH*2
        bne slcl_loop
        plp
    }
}

typedef struct { int x; SLZWord zq; } SLZHit;
typedef struct { long x,z,sx,sz; int ya,yb,xb; } SLZEdge;

/*
 * ----------------------------------------------------------------------------
 *  SL_SetScale
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Adapter la quantification scanline en réservant de la marge pour le
 *    biais.
 *
 *  Paramètres :
 *    [entrée] max_inv_z
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    sl_zscale=cible/(max_inv_z+SL_Z_BIAS) si le dénominateur dépasse 1e-7.
 *
 *  Fonctionnement :
 *    Sinon conserve l'échelle précédente.
 *
 * ----------------------------------------------------------------------------
 */
static void SL_SetScale(float max_inv_z)
{

    /* Inclure le biais maximal avant la division pour limiter la saturation. */
    float max_biased_inv_z = max_inv_z + SL_Z_BIAS;

    if (max_biased_inv_z > 0.0000001f)
        sl_zscale = SL_TARGET_MAX_CODE / max_biased_inv_z;
}

/*
 * ----------------------------------------------------------------------------
 *  SL_Quantize
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
static SLZWord SL_Quantize(float inv_z)
{
    float scaled;
    long q;
    /* Appliquer l'échelle scanline, arrondir, saturer puis inverser le code. */
    if (inv_z < 0.0f) inv_z = 0.0f;
    scaled = inv_z * sl_zscale;
    q = (long)(scaled + 0.5f);
    if (q < 0) q = 0;
    if (q > 0xFFFFL) q = 0xFFFFL;
    return (SLZWord)(0xFFFFUL - (unsigned long)q);
}

long sl_p_zacc, sl_p_zstep;
unsigned long sl_p_zptr, sl_p_pptr;
int sl_p_n, sl_p_odd, sl_p_mode, sl_p_cl, sl_p_ci, sl_p_cr;
unsigned int sl_p_zl, sl_p_zr;

/*
 * ----------------------------------------------------------------------------
 *  SL_SpanRun
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Dessiner un span préclippé avec test Z strict dans un noyau 65C816.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Entrées et sorties implicites :
 *    Entrée : paquet partagé sl_p_* : pointeurs Z et pixels far, parité
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
static void SL_SpanRun(void)
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
        lda >sl_p_zacc
        sta 0
        lda >sl_p_zacc+2
        sta 2
        lda >sl_p_zstep
        sta 4
        lda >sl_p_zstep+2
        sta 6
        lda >sl_p_zptr
        sta 8
        lda >sl_p_zptr+2
        sta 10
        lda >sl_p_pptr
        sta 12
        lda >sl_p_pptr+2
        sta 14
        lda >sl_p_n
        sta 16

        lda >sl_p_ci
        and #0x000F
        sta 18
        asl a
        asl a
        asl a
        asl a
        sta 20

        lda >sl_p_odd
        eor #1
        sta 22

        ldy #0

        lda >sl_p_odd
        beq zbr_l_ev
        lda #0x00F0
        sta 24
        lda >sl_p_cl
        and #0x000F
        sta 26
        bra zbr_l_go
    zbr_l_ev:
        lda #0x000F
        sta 24
        lda >sl_p_cl
        and #0x000F
        asl a
        asl a
        asl a
        asl a
        sta 26
    /* Tester la profondeur exacte du pixel gauche avant toute écriture du nibble. */
    zbr_l_go:

        lda >sl_p_zl
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

        lda >sl_p_mode
        bne zbr_full
        brl zbr_done

    /* Avancer après le pixel gauche ; l'adresse couleur progresse seulement
     * après un pixel impair.
     */
    zbr_full:
        iny
        iny
        lda >sl_p_odd
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
        lda >sl_p_n
        clc
        adc 22
        and #1
        beq zbr_r_ev
        lda #0x00F0
        sta 24
        lda >sl_p_cr
        and #0x000F
        sta 26
        bra zbr_r_go
    zbr_r_ev:
        lda #0x000F
        sta 24
        lda >sl_p_cr
        and #0x000F
        asl a
        asl a
        asl a
        asl a
        sta 26
    zbr_r_go:
        lda >sl_p_zr
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

/*
 * ----------------------------------------------------------------------------
 *  SL_PaintSpan
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Préparer un span V3 qui utilise la seule ligne de profondeur courante.
 *
 *  Paramètres :
 *    [entrée] x0/x1 inclus sans pan, screenY avec pan, z0/z1 aux extrémités,
 *    fillColor/frameColor et nudge en codes.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Span déjà clippé requis. Soustrait nudge avec borne zéro, prépare
 *    adresses et parité puis interpole Z en 16.16. Les deux extrémités
 *    utilisent la couleur de contour.
 *
 * ----------------------------------------------------------------------------
 */
static void SL_PaintSpan(int x0, int x1, int screenY,
                         SLZWord z0, SLZWord z1,
                         int fillColor, int frameColor, int nudge)
{
    int sx0, nsteps;
    long zstep, zacc;
    if (x0 > x1) return;
    /* Réduire les deux codes de profondeur sans passer sous zéro. */
    if (nudge) {
        z0 = (z0 > (SLZWord)nudge) ? (SLZWord)(z0-nudge) : 0;
        z1 = (z1 > (SLZWord)nudge) ? (SLZWord)(z1-nudge) : 0;
    }
    /* Préparer les pointeurs de la ligne Z et de l'écran au premier pixel. */
    sx0 = x0 + pan_dx;
    sl_p_zptr = (unsigned long)(sl_zline + sx0);
    sl_p_pptr = SL_SHGR_BASE + (unsigned long)screenY * 160UL + (unsigned long)(sx0 >> 1);
    sl_p_odd = sx0 & 1;
    sl_p_zl = z0;
    sl_p_cl = frameColor;
    /* Le span ponctuel se limite au pixel gauche ; sinon préparer intérieur
     * et bord droit.
     */
    if (x0 == x1) {
        sl_p_mode = 0;
        SL_SpanRun();
        return;
    }
    nsteps = x1-x0;
    zstep = (((long)z1-(long)z0) << 16) / nsteps;
    zacc = ((long)z0 << 16) + 0x8000L;
    sl_p_zr = z1;
    sl_p_zacc = zacc;
    sl_p_zstep = zstep;
    sl_p_n = nsteps-1;
    sl_p_ci = fillColor;
    sl_p_cr = frameColor;
    sl_p_mode = 1;
    SL_SpanRun();
}

/*
 * ----------------------------------------------------------------------------
 *  renderModelScanlineZBuffer_coreV3
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Rendre les surfaces par balayage scanline V3 quantifié.
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
 *    uniquement les faces actives. Ligne Z de 320 codes 16 bits ; arêtes à
 *    douze fractions, spans en 16.16 et petit nudge entier.
 *
 *    Le clipping du viewport est appliqué aux spans ; pas de clipping 3D
 *    général. Les réserves et le comportement d'échec restent propres à
 *    cette version.
 *
 * ----------------------------------------------------------------------------
 */
static void renderModelScanlineZBuffer_coreV3(Model3D* model, int biased)
{
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;
    int fcount = faces->face_count;
    int y, f, i, k, b, si, ai, keep, active_count;
    int total_edges, end;
    int clip_x_min, clip_x_max, y_lo, y_hi;
    float frame_max_inv_z;
    SLZEdge* ed;
    SLZHit hits[MAX_SPAN_INTERSECTIONS];

    static float* inv_z = NULL;
    static SLZWord* vzq = NULL;
    static SLZWord* vzq_b = NULL;
    static int vert_capacity = 0;

    static int* face_start = NULL;
    static int* face_sorted = NULL;
    static int* face_active = NULL;
    static unsigned char* face_fill = NULL;
    static unsigned char* face_frame = NULL;
    static unsigned char* face_nudge = NULL;
    static int face_capacity = 0;

    static SLZEdge* edge_buf = NULL;
    static int edge_capacity = 0;

    static int bucket[SCREEN_HEIGHT + 1];
    static int bucket_cur[SCREEN_HEIGHT + 1];

    /* Réserver les tables de faces, couleurs et biais. */
    if (face_capacity < fcount) {
        if (face_start)   free(face_start);
        if (face_sorted)  free(face_sorted);
        if (face_active)  free(face_active);
        if (face_fill)    free(face_fill);
        if (face_frame)   free(face_frame);
        if (face_nudge)   free(face_nudge);
        face_start  = (int*)malloc(fcount * sizeof(int));
        face_sorted = (int*)malloc(fcount * sizeof(int));
        face_active = (int*)malloc(fcount * sizeof(int));
        face_fill   = (unsigned char*)malloc(fcount);
        face_frame  = (unsigned char*)malloc(fcount);
        face_nudge  = (unsigned char*)malloc(fcount);
        face_capacity = (face_start && face_sorted && face_active &&
                         face_fill && face_frame && face_nudge) ? fcount : 0;
    }
    /* Agrandir les buffers de sommets et conserver leur capacité entre les rendus. */
    if (vert_capacity < vcount) {
        if (inv_z) free(inv_z);
        if (vzq)   free(vzq);
        if (vzq_b) free(vzq_b);
        inv_z = (float*)malloc(vcount * sizeof(float));
        vzq   = (SLZWord*)malloc(vcount * sizeof(SLZWord));
        vzq_b = (SLZWord*)malloc(vcount * sizeof(SLZWord));
        vert_capacity = (inv_z && vzq && vzq_b) ? vcount : 0;
    }
    total_edges = 0;
    for (f = 0; f < fcount; f++) {
        end = faces->vertex_indices_ptr[f] + faces->vertex_count[f];
        if (end > total_edges) total_edges = end;
    }
    if (edge_capacity < total_edges) {
        if (edge_buf) free(edge_buf);
        edge_buf = (SLZEdge*)malloc((size_t)total_edges * sizeof(SLZEdge));
        edge_capacity = edge_buf ? total_edges : 0;
    }
    if (face_capacity < fcount || vert_capacity < vcount || edge_capacity < total_edges)
        return;

    frame_max_inv_z = 0.0f;
    /* Calculer les profondeurs inverses et les codes selon le format de
     * cette version.
     */
    for (i = 0; i < vcount; i++) {
        float zo_f = FIXED_TO_FLOAT(vtx->zo[i]);
        inv_z[i] = (zo_f > 0.0f) ? (1.0f / zo_f) : 0.0f;
        if (inv_z[i] > frame_max_inv_z) frame_max_inv_z = inv_z[i];
    }
    SL_SetScale(frame_max_inv_z);

    for (i = 0; i < vcount; i++) {

        vzq[i]   = SL_Quantize(inv_z[i]);
        vzq_b[i] = biased ? SL_Quantize(inv_z[i] + SL_Z_BIAS) : vzq[i];
    }

    SetPenMode(0);
    applyPalette(palette);

    clip_x_min = -pan_dx;
    clip_x_max = SCREEN_WIDTH - 1 - pan_dx;
    y_lo = -pan_dy;
    y_hi = SCREEN_HEIGHT - 1 - pan_dy;

    /* Transformer l'histogramme en offsets cumulés pour classer les débuts
     * de faces.
     */
    /* Initialiser l'histogramme des premières lignes ; préparer visibilité,
     * couleurs et arêtes.
     */
    for (b = 0; b <= SCREEN_HEIGHT; b++) bucket[b] = 0;

    for (f = 0; f < fcount; f++) {
        int n, offt, fmin, fmax, fill;
        SLZWord* zsrc;

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
            ed->x  = ex + SL_Z_ROUND;
            ed->z  = ez + SL_Z_ROUND;
            ed->sx = sx;  ed->sz = sz;
        }
    }

    for (b = 0; b < SCREEN_HEIGHT; b++) bucket[b + 1] += bucket[b];
    for (b = 0; b <= SCREEN_HEIGHT; b++) bucket_cur[b] = bucket[b];
    for (f = 0; f < fcount; f++) {
        b = face_start[f];
        if (b >= 0) face_sorted[bucket_cur[b]++] = f;
    }

    /* Démarrer la liste des faces actives, puis balayer les lignes visibles. */
    active_count = 0;
    screenClearTopLines(12);
    for (y = y_lo; y <= y_hi; y++) {
        int screenY = y + pan_dy;
        int row = y - y_lo;

        SL_ClearLine();

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
            int fillColor, frameColor, nudge;

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
                    hits[hit_count].x  = ed->xb + ((t >= 0L) ? (int)(t >> SL_Z_FRAC) : -(int)((-t) >> SL_Z_FRAC));
                    hits[hit_count].zq = (SLZWord)(ed->z >> SL_Z_FRAC);
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
                    SLZHit tmp = hits[0];
                    hits[0] = hits[1];
                    hits[1] = tmp;
                }
            } else {
                int a, c;
                for (a = 1; a < hit_count; a++) {
                    SLZHit key = hits[a];
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

            {
                int p;
                /* Apparier les intersections par parité, clipper les spans
                 * et les transmettre au noyau de dessin.
                 */
                for (p = 0; p + 1 < hit_count; p += 2) {
                    int xa = hits[p].x;
                    int xb = hits[p + 1].x;
                    SLZWord zqa = hits[p].zq;
                    SLZWord zqb = hits[p + 1].zq;
                    int x0, x1;
                    SLZWord zq0, zq1;

                    if (xa > xb) continue;
                    if (xb < clip_x_min || xa > clip_x_max) continue;

                    x0 = xa;  x1 = xb;
                    zq0 = zqa; zq1 = zqb;

                    if (x0 < clip_x_min) { zq0 = ZBuffer_LerpZ(zqa, zqb, xa, xb, clip_x_min); x0 = clip_x_min; }
                    if (x1 > clip_x_max) { zq1 = ZBuffer_LerpZ(zqa, zqb, xa, xb, clip_x_max); x1 = clip_x_max; }
                    if (x0 > x1) continue;

                    SL_PaintSpan(x0, x1, screenY, zq0, zq1,
                                 fillColor, frameColor, nudge);
                }
            }
        }
        active_count = keep;
    }
}

/*
 * ----------------------------------------------------------------------------
 *  renderModelScanlineZBuffer_fastV3
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Appeler renderModelScanlineZBuffer_coreV3 sans rapprochement des faces
 *    de plane_d>0.
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
void renderModelScanlineZBuffer_fastV3(Model3D* model)
{
    /* Transmettre explicitement le choix du biais au noyau commun. */
    renderModelScanlineZBuffer_coreV3(model, 0);
}

/*
 * ----------------------------------------------------------------------------
 *  renderModelScanlineZBuffer_biasedV3
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Appeler renderModelScanlineZBuffer_coreV3 avec rapprochement des faces
 *    de plane_d>0.
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
void renderModelScanlineZBuffer_biasedV3(Model3D* model)
{
    /* Transmettre explicitement le choix du biais au noyau commun. */
    renderModelScanlineZBuffer_coreV3(model, 1);
}

/*
 * ----------------------------------------------------------------------------
 *  renderModelScanlineZBuffer
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
 *    les faces plane_d>0. Les deux chemins partagent le même moteur V3.
 *
 * ----------------------------------------------------------------------------
 */
void renderModelScanlineZBuffer(Model3D* model)
{
    /* Sélectionner le noyau avec la politique de biais correspondant à la
     * visibilité.
     */
    if (cull_back_faces)
        renderModelScanlineZBuffer_fastV3(model);
    else
        renderModelScanlineZBuffer_biasedV3(model);
}
