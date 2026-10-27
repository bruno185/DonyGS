/*
 * ============================================================================
 *  zbuffer_fullscreen_offscreen.c — Z-buffer plein écran — Framebuffer en
 *  RAM
 * ============================================================================
 *
 *  Rôle :
 *    Rendre le modèle dans un framebuffer RAM de 32000 octets, puis
 *    transférer les pixels vers SHGR en une copie MVN, en utilisant le
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
 *    Copier le framebuffer RAM vers les 32000 octets de pixels SHGR.
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


typedef ZBufferScanHit OffScanIntersection;


typedef ZBufferScanEdge OffEdge;


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
        zspan_p_zptr = (unsigned long)(zbuf_row[screenY] + sx0);
        zspan_p_pptr = (unsigned long)(off_row[screenY] + (sx0 >> 1));
        zspan_p_odd  = sx0 & 1;
        zspan_p_zl   = zq0;
        zspan_p_cl   = frameColor;
        zspan_p_mode = 0;
        ZBuffer_SpanRun();
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

        zspan_p_zptr  = (unsigned long)(zbuf_row[screenY] + sx0);
        zspan_p_pptr  = (unsigned long)(off_row[screenY] + (sx0 >> 1));
        zspan_p_odd   = sx0 & 1;
        zspan_p_zl    = zq0;
        zspan_p_zr    = zq_end;
        zspan_p_zacc  = z_acc;
        zspan_p_zstep = z_step_16;
        zspan_p_n     = nsteps - 1;
        zspan_p_cl    = frameColor;
        zspan_p_ci    = whole_span_is_border ? frameColor : fillColor;
        zspan_p_cr    = (silhouette_r || whole_span_is_border) ? frameColor : fillColor;
        zspan_p_mode  = 1;
        ZBuffer_SpanRun();
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
    int y, f, i, k, ai, keep, active_count;
    int total_edges, end;
    int clip_x_min, clip_x_max, y_lo, y_hi;
    float frame_max_inv_z;
    OffEdge* ed;
    OffScanIntersection hits[MAX_SPAN_INTERSECTIONS];

    static ZBufferVertexWork vertices;

    static OffZBufSpanPrev* span_prev = NULL;
    static ZBufferFaceWork face_work;

    static OffEdge* edge_buf = NULL;
    static int edge_capacity = 0;


    static int bucket[OFF_ROWS + 1];
    static int bucket_cur[OFF_ROWS + 1];


    if (!off_ready) return;


    /* Réserver les tables de faces, couleurs et biais. */
    if (face_work.capacity < fcount) {
        if (!ZBuffer_SizeFits(fcount, sizeof(OffZBufSpanPrev))) return;
        free(span_prev); span_prev = NULL;
        if (!ZBuffer_ReserveFaces(&face_work, fcount)) return;
        span_prev = (OffZBufSpanPrev*)malloc((size_t)fcount * sizeof(OffZBufSpanPrev));
        if (!span_prev) { ZBuffer_FreeFaces(&face_work); return; }
    }
    /* Agrandir les buffers de sommets et conserver leur capacité entre les rendus. */
    if (!ZBuffer_ReserveVertices(&vertices, vcount)) return;
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
    if (face_work.capacity < fcount || vertices.capacity < vcount || edge_capacity < total_edges)
        return;


    for (f = 0; f < fcount; f++) span_prev[f].valid = 0;


    frame_max_inv_z = ZBuffer_PrepareInvZ(vtx, vcount, vertices.inv_z);
    ZBuffer_SetScaleForFrame(frame_max_inv_z);

    for (i = 0; i < vcount; i++) {

        vertices.vzq[i]   = ZBuffer_QuantizeInvZ(vertices.inv_z[i]);
        vertices.vzq_b[i] = biased ? ZBuffer_QuantizeInvZ(vertices.inv_z[i] + OFF_Z_BIAS) : vertices.vzq[i];
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
    ZBuffer_PrepareFaces(faces, fcount, y_lo, y_hi, OFF_ROWS, biased,
                         face_work.face_start, face_work.face_fill, face_work.face_frame,
                         face_work.face_nudge, bucket);
    for (f = 0; f < fcount; ++f) {
        int offt;
        UWORD16 *zsrc;
        if (face_work.face_start[f] < 0) continue;
        offt = faces->vertex_indices_ptr[f];
        zsrc = (biased && faces->plane_d[f] > 0) ? vertices.vzq_b : vertices.vzq;
        ed = edge_buf + offt;

        ZBuffer_PrepareScanEdges(vtx, faces, f, zsrc, y_lo, OFF_ROUND, ed);
    }


    ZBuffer_SortFaceStarts(fcount, OFF_ROWS, face_work.face_start,
                           bucket, bucket_cur, face_work.face_sorted);


    /* Démarrer la liste des faces actives, puis balayer les lignes visibles. */
    active_count = 0;
    for (y = y_lo; y <= y_hi; y++) {
        int screenY = y + pan_dy;
        int row = y - y_lo;


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
            int fillColor, frameColor, on_top_or_bottom, nudge;

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
            ZBuffer_SortScanHits(hits, hit_count);

            fillColor  = face_work.face_fill[f];
            frameColor = face_work.face_frame[f];
            nudge      = face_work.face_nudge[f];
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

                    if (x0 < clip_x_min) { zq0 = ZBuffer_LerpZ(zqa, zqb, xa, xb, clip_x_min); x0 = clip_x_min; }
                    if (x1 > clip_x_max) { zq1 = ZBuffer_LerpZ(zqa, zqb, xa, xb, clip_x_max); x1 = clip_x_max; }
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
