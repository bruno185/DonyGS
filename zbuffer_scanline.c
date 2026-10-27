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
 *    couleurs effectives.
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

typedef ZBufferScanHit SLZHit;
typedef ZBufferScanEdge SLZEdge;

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
    zspan_p_zptr = (unsigned long)(sl_zline + sx0);
    zspan_p_pptr = SL_SHGR_BASE + (unsigned long)screenY * 160UL + (unsigned long)(sx0 >> 1);
    zspan_p_odd = sx0 & 1;
    zspan_p_zl = z0;
    zspan_p_cl = frameColor;
    /* Le span ponctuel se limite au pixel gauche ; sinon préparer intérieur
     * et bord droit.
     */
    if (x0 == x1) {
        zspan_p_mode = 0;
        ZBuffer_SpanRun();
        return;
    }
    nsteps = x1-x0;
    zstep = (((long)z1-(long)z0) << 16) / nsteps;
    zacc = ((long)z0 << 16) + 0x8000L;
    zspan_p_zr = z1;
    zspan_p_zacc = zacc;
    zspan_p_zstep = zstep;
    zspan_p_n = nsteps-1;
    zspan_p_ci = fillColor;
    zspan_p_cr = frameColor;
    zspan_p_mode = 1;
    ZBuffer_SpanRun();
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
    int y, f, i, k, ai, keep, active_count;
    int total_edges, end;
    int clip_x_min, clip_x_max, y_lo, y_hi;
    float frame_max_inv_z;
    SLZEdge* ed;
    SLZHit hits[MAX_SPAN_INTERSECTIONS];

    static ZBufferVertexWork vertices;

    static ZBufferFaceWork face_work;

    static SLZEdge* edge_buf = NULL;
    static int edge_capacity = 0;

    static int bucket[SCREEN_HEIGHT + 1];
    static int bucket_cur[SCREEN_HEIGHT + 1];

    /* Réserver les tables de faces, couleurs et biais. */
    if (!ZBuffer_ReserveFaces(&face_work, fcount)) return;
    /* Agrandir les buffers de sommets et conserver leur capacité entre les rendus. */
    if (!ZBuffer_ReserveVertices(&vertices, vcount)) return;
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
    if (face_work.capacity < fcount || vertices.capacity < vcount || edge_capacity < total_edges)
        return;

    frame_max_inv_z = ZBuffer_PrepareInvZ(vtx, vcount, vertices.inv_z);
    SL_SetScale(frame_max_inv_z);

    for (i = 0; i < vcount; i++) {

        vertices.vzq[i]   = ZBuffer_QuantizeAtScale(vertices.inv_z[i], sl_zscale);
        vertices.vzq_b[i] = biased ? ZBuffer_QuantizeAtScale(vertices.inv_z[i] + SL_Z_BIAS, sl_zscale) : vertices.vzq[i];
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
    ZBuffer_PrepareFaces(faces, fcount, y_lo, y_hi, SCREEN_HEIGHT, biased,
                         face_work.face_start, face_work.face_fill, face_work.face_frame,
                         face_work.face_nudge, bucket);
    for (f = 0; f < fcount; ++f) {
        int offt;
        UWORD16 *zsrc;
        if (face_work.face_start[f] < 0) continue;
        offt = faces->vertex_indices_ptr[f];
        zsrc = (biased && faces->plane_d[f] > 0) ? vertices.vzq_b : vertices.vzq;
        ed = edge_buf + offt;

        ZBuffer_PrepareScanEdges(vtx, faces, f, zsrc, y_lo, SL_Z_ROUND, ed);
    }

    ZBuffer_SortFaceStarts(fcount, SCREEN_HEIGHT, face_work.face_start,
                           bucket, bucket_cur, face_work.face_sorted);

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
        active_count = ZBuffer_ActivateFaces(row, bucket, face_work.face_sorted,
                                             face_work.face_active, active_count);

        keep = 0;
        /* Écarter les faces terminées et extraire les intersections des
         * arêtes actives.
         */
        for (ai = 0; ai < active_count; ai++) {
            int n, offt, hit_count;
            int fillColor, frameColor, nudge;

            f = face_work.face_active[ai];
            if (faces->maxy[f] < y) continue;
            face_work.face_active[keep++] = f;

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
            ZBuffer_SortScanHits(hits, hit_count);

            fillColor  = face_work.face_fill[f];
            frameColor = face_work.face_frame[f];
            nudge      = face_work.face_nudge[f];

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
