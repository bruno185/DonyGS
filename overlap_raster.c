/*
 * ============================================================================
 *  overlap_raster.c — Chevauchement projeté, clipping et projection subpixel
 * ============================================================================
 *
 *  Rôle :
 *    Fournir les méthodes de chevauchement des faces projetées et les points
 *    de comparaison employés par les outils d'ordre. Distinguer géométrie
 *    continue, heuristiques avec seuils et échantillonnage raster.
 *
 *  Organisation :
 *    Regroupe le cache de projection fractionnaire, les prédicats de points
 *    et segments, le clipping SH, les régions QuickDraw, les variantes
 *    Simple/V1/V2/Max/Fixed64/Raster. Le choix runtime est
 *    indépendant du wrapper V2 à sélection compilée.
 *
 *  Dépendances et contraintes :
 *    Inclus par engine.c ; utilise les types communs, les caches d'ordre et
 *    les routines de ray casting déclarées auparavant. Le clipping par
 *    demi-plans exige un clip convexe pour le cas général. Le chemin nommé
 *    centroïde QuickDraw rend un centre de bbox. Buffers et diagnostics
 *    partagés, non réentrants.
 *
 * ============================================================================
 */

#define FRACTIONAL_CACHE_BLOCK_SIZE 256
#define FRACTIONAL_CACHE_BLOCKS ((MAX_VERTICES + FRACTIONAL_CACHE_BLOCK_SIZE - 1) / FRACTIONAL_CACHE_BLOCK_SIZE)
typedef struct {
    double x[FRACTIONAL_CACHE_BLOCK_SIZE];
    double y[FRACTIONAL_CACHE_BLOCK_SIZE];
    unsigned int stamp[FRACTIONAL_CACHE_BLOCK_SIZE];
} FractionalProjectionBlock;
static FractionalProjectionBlock *fractional_cache[FRACTIONAL_CACHE_BLOCKS];
static unsigned int fractional_cache_failed[FRACTIONAL_CACHE_BLOCKS];
static unsigned int fractional_cache_generation = 0;
static Model3D *fractional_cache_owner = NULL;

/*
 * ----------------------------------------------------------------------------
 *  fractional_projection_cache_invalidate
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Invalider les rapports de projection et les caches d'ordre associés.
 *
 *  Paramètres :
 *    [entrée] model, nouveau propriétaire.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Incrémente la génération en conservant les allocations ; au rebouclage,
 *    efface stamps et échecs mémorisés.
 *
 * ----------------------------------------------------------------------------
 */
static void fractional_projection_cache_invalidate(Model3D *model)
{
    int i;
    /* Invalider aussi les métriques d'ordre dépendantes de cette transformation. */
    ordering_cache_invalidate(model);
    fractional_cache_owner = model;
    ++fractional_cache_generation;
    /* Effacer les anciennes générations avant de réutiliser le stamp 1. */
    if (fractional_cache_generation == 0) {
        for (i = 0; i < FRACTIONAL_CACHE_BLOCKS; ++i) {
            if (fractional_cache[i]) memset(fractional_cache[i]->stamp, 0,
                                           sizeof(fractional_cache[i]->stamp));
        }
        memset(fractional_cache_failed, 0, sizeof(fractional_cache_failed));
        fractional_cache_generation = 1;
    }
}

/*
 * ----------------------------------------------------------------------------
 *  fractional_projection_cache_release
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Libérer les caches d'ordre et de projection appartenant à model.
 *
 *  Paramètres :
 *    [entrée] model
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Le cache fractionnaire est libéré uniquement si son propriétaire
 *    correspond ; tous les pointeurs obtenus deviennent invalides.
 *
 * ----------------------------------------------------------------------------
 */
static void fractional_projection_cache_release(Model3D *model)
{
    int i;
    /* Libérer d'abord les caches d'ordre, puis vérifier le propriétaire du
     * cache fractionnaire.
     */
    ordering_cache_release(model);
    if (fractional_cache_owner != model) return;
    /* Libérer les blocs et remettre les métadonnées à zéro. */
    for (i = 0; i < FRACTIONAL_CACHE_BLOCKS; ++i) {
        free(fractional_cache[i]);
        fractional_cache[i] = NULL;
    }
    memset(fractional_cache_failed, 0, sizeof(fractional_cache_failed));
    fractional_cache_owner = NULL;
    fractional_cache_generation = 0;
}

/*
 * ----------------------------------------------------------------------------
 *  fractional_projection_cached
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Projeter un sommet en double en mémorisant xo/zo et yo/zo
 *    indépendamment du zoom.
 *
 *  Paramètres :
 *    [entrée] model ; v, indice base zéro valide ; scale, pixels par unité
 *    projetée.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    *outx/*outy, pixels fractionnaires sans pan
 *
 *  Préconditions :
 *    zo[v] non nul. Réserve des blocs de 256 sommets à la demande ; sans
 *    cache disponible, calcule directement les rapports. Ne pas confondre la
 *    borne du cache MAX_VERTICES avec une validation générale de v.
 *
 * ----------------------------------------------------------------------------
 */
static void fractional_projection_cached(Model3D *model, int v, double scale,
                                          double *outx, double *outy)
{
    VertexArrays3D *vtx = &model->vertices;
    FractionalProjectionBlock *block = NULL;
    double x, y;
    int block_index, slot;
    if (fractional_cache_owner != model) fractional_projection_cache_invalidate(model);
    /* Localiser le bloc ou le réserver sans répéter son échec dans la même
     * génération.
     */
    if (v >= 0 && v < MAX_VERTICES) {
        block_index = v / FRACTIONAL_CACHE_BLOCK_SIZE;
        slot = v % FRACTIONAL_CACHE_BLOCK_SIZE;
        block = fractional_cache[block_index];
        if (!block && fractional_cache_failed[block_index] != fractional_cache_generation) {
            block = (FractionalProjectionBlock*)calloc(1, sizeof(FractionalProjectionBlock));
            fractional_cache[block_index] = block;
            if (!block) fractional_cache_failed[block_index] = fractional_cache_generation;
        }
    }
    /* Réutiliser les rapports de cette vue ; à défaut, les recalculer sans
     * stockage.
     */
    if (block) {
        if (block->stamp[slot] != fractional_cache_generation) {
            block->x[slot] = (double)vtx->xo[v] / (double)vtx->zo[v];
            block->y[slot] = (double)vtx->yo[v] / (double)vtx->zo[v];
            block->stamp[slot] = fractional_cache_generation;
        }
        x = block->x[slot]; y = block->y[slot];
    } else {

        x = (double)vtx->xo[v] / (double)vtx->zo[v];
        y = (double)vtx->yo[v] / (double)vtx->zo[v];
    }
    /* Appliquer seulement l'échelle courante et le centrage écran aux
     * rapports normalisés.
     */
    *outx = (double)CENTRE_X + scale * x;
    *outy = (double)CENTRE_Y - scale * y;
}

/*
 * ----------------------------------------------------------------------------
 *  fractional_face_gather
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Préparer le contour subpixel antihoraire d'une face et ses vecteurs
 *    d'arêtes.
 *
 *  Paramètres :
 *    [entrée] model, face, n et scale
 *
 *  Retour :
 *    1 si contour exploitable, 0 si sommet derrière l'observateur ou aire
 *    quasi nulle.
 *
 *  Sorties et effets de bord :
 *    px/py de capacité n ; edge_x/edge_y reçoivent des pointeurs empruntés
 *    au cache ou NULL.
 *
 *  Préconditions :
 *    indices valides, n>=3 et scale cohérent avec l'échelle globale qui sert
 *    de clé. Au plus 16 sommets sont mémorisés ; un échec de cache n'empêche
 *    pas de retourner le contour dans les buffers fournis.
 *
 * ----------------------------------------------------------------------------
 */
static int fractional_face_gather(Model3D *model,int face,int n,double scale,
                                   double *px,double *py,
                                   double **edge_x,double **edge_y)
{
    FaceArrays3D *faces=&model->faces;
    VertexArrays3D *vtx=&model->vertices;
    OrderContour *entry;
    OrderWinding *winding;
    double area2=0.0;
    int i,prev,offset=faces->vertex_indices_ptr[face];
    *edge_x=*edge_y=NULL;
    if(fractional_cache_owner!=model)fractional_projection_cache_invalidate(model);
    /* Chercher le contour avec ses tags exacts de vue, face, taille et échelle. */
    entry=&order_contours[(unsigned int)face & (ORDER_CONTOUR_SLOTS-1)];
    if(n<=ORDER_CONTOUR_MAX_VERTICES && entry->generation==order_cache_generation &&
       entry->face==face && entry->count==n && entry->scale==s_global_proj_scale_fixed){
        memcpy(px,entry->data,n*sizeof(double));
        memcpy(py,entry->data+entry->capacity,n*sizeof(double));
        *edge_x=entry->data+2*entry->capacity;
        *edge_y=entry->data+3*entry->capacity;
        return 1;
    }
    /* Construire la projection et refuser une profondeur non positive. */
    for(i=0;i<n;++i){
        int v=faces->vertex_indices_buffer[offset+i]-1;
        if(vtx->zo[v]<=0)return 0;
        fractional_projection_cached(model,v,scale,&px[i],&py[i]);
    }
    /* Réutiliser ou calculer l'aire orientée, puis inverser les coordonnées
     * si nécessaire.
     */
    winding=ordering_winding_slot(model,face);
    if(ordering_winding_hit(winding,face,s_global_proj_scale_fixed))area2=winding->area2;
    else {
        prev=n-1;
        for(i=0;i<n;++i){
            area2+=(px[prev]-px[0])*(py[i]-py[0])
                  -(px[i]-px[0])*(py[prev]-py[0]);
            prev=i;
        }
        ordering_winding_store(winding,face,s_global_proj_scale_fixed,area2);
    }
    if(fabs(area2)<=1e-12)return 0;
    if(area2<0.0){
        for(i=0;i<n/2;++i){
            int j=n-1-i;
            double t=px[i];px[i]=px[j];px[j]=t;
            t=py[i];py[i]=py[j];py[j]=t;
        }
    }
    /* Limiter la mémorisation aux petits contours sans refuser les contours
     * plus grands.
     */
    if(n>ORDER_CONTOUR_MAX_VERTICES)return 1;
    if(entry->capacity<n){
        double *grown=NULL;
        if(entry->failed_generation!=order_cache_generation)
            grown=(double*)malloc((size_t)n*4*sizeof(double));
        if(!grown){entry->failed_generation=order_cache_generation;return 1;}
        free(entry->data);entry->data=grown;entry->capacity=n;
    }
    /* Mémoriser x/y et préparer les vecteurs du contour pour les découpages
     * suivants.
     */
    memcpy(entry->data,px,n*sizeof(double));
    memcpy(entry->data+entry->capacity,py,n*sizeof(double));
    *edge_x=entry->data+2*entry->capacity;
    *edge_y=entry->data+3*entry->capacity;
    for(i=0;i<n;++i){
        int next=i+1;if(next==n)next=0;
        (*edge_x)[i]=px[next]-px[i];
        (*edge_y)[i]=py[next]-py[i];
    }
    entry->face=face;entry->count=n;entry->scale=s_global_proj_scale_fixed;
    entry->generation=order_cache_generation;
    return 1;
}

/*
 * ----------------------------------------------------------------------------
 *  compute_bbox_intersection_center
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Retourner le centre entier du rectangle commun des boîtes de deux
 *    faces.
 *
 *  Paramètres :
 *    [entrée] model et f1/f2
 *
 *  Retour :
 *    1 en cas de rectangle commun, 0 sinon.
 *
 *  Sorties et effets de bord :
 *    outx/outy.
 *
 *  Fonctionnement :
 *    Ce point peut être hors des deux polygones.
 *
 * ----------------------------------------------------------------------------
 */
static int compute_bbox_intersection_center(Model3D* model, int f1, int f2, int *outx, int *outy) {
    if (!model || !outx || !outy) return 0;
    int ix0, iy0, ix1, iy1;
    /* Exiger un rectangle commun avant de moyenner ses bornes. */
    if (!compute_bbox_intersection(model, f1, f2, &ix0, &iy0, &ix1, &iy1)) return 0;
    *outx = (ix0 + ix1) / 2;
    *outy = (iy0 + iy1) / 2;
    return 1;
}
/*
 * ----------------------------------------------------------------------------
 *  compute_bbox_intersection
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Calculer le rectangle commun aux boîtes projetées de deux faces.
 *
 *  Paramètres :
 *    [entrée] model, f1/f2 base zéro
 *
 *  Retour :
 *    1 si les intervalles ont un point commun, 0 sinon ou arguments
 *    invalides.
 *
 *  Sorties et effets de bord :
 *    ix0/iy0/ix1/iy1.
 *
 *  Fonctionnement :
 *    Accepte les contacts d'aire nulle ; les sorties ne sont écrites qu'en
 *    réussite.
 *
 * ----------------------------------------------------------------------------
 */
static int compute_bbox_intersection(Model3D* model, int f1, int f2, int *ix0, int *iy0, int *ix1, int *iy1) {
    if (!model || !ix0 || !iy0 || !ix1 || !iy1) return 0;
    FaceArrays3D* faces = &model->faces;
    if (f1 < 0 || f2 < 0 || f1 >= faces->face_count || f2 >= faces->face_count) return 0;
    int minx1 = faces->minx[f1], maxx1 = faces->maxx[f1], miny1 = faces->miny[f1], maxy1 = faces->maxy[f1];
    int minx2 = faces->minx[f2], maxx2 = faces->maxx[f2], miny2 = faces->miny[f2], maxy2 = faces->maxy[f2];
    /* Prendre les maxima des minima et les minima des maxima. */
    int a0 = (minx1 > minx2) ? minx1 : minx2;
    int a1 = (maxx1 < maxx2) ? maxx1 : maxx2;
    int b0 = (miny1 > miny2) ? miny1 : miny2;
    int b1 = (maxy1 < maxy2) ? maxy1 : maxy2;
    /* Rejeter seulement une séparation stricte, puis publier les bornes. */
    if (a0 > a1 || b0 > b1) return 0;
    *ix0 = a0; *iy0 = b0; *ix1 = a1; *iy1 = b1;
    return 1;
}
/*
 * ----------------------------------------------------------------------------
 *  point_in_poly_arrays_int
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Tester un point entier par parité des traversées d'un rayon horizontal.
 *
 *  Paramètres :
 *    [entrée] px/py ; n ; vx/vy de longueur n.
 *
 *  Retour :
 *    0 ou 1 ; n<3 rend 0.
 *
 *  Fonctionnement :
 *    Un sommet exactement égal rend 1, mais les autres contacts de contour
 *    ne sont pas traités uniformément comme intérieur. Produits 64 bits
 *    après différences natives int : ne pas présenter cette routine comme
 *    exacte sur tout le domaine 16 bits.
 *
 * ----------------------------------------------------------------------------
 */
static int point_in_poly_arrays_int(int px, int py, int n, const int vx[], const int vy[]) {
    if (n < 3) return 0;
    int inside = 0;
    /* Compter les croisements en évitant la division par produit croisé ;
     * traiter les sommets égaux à part.
     */
    for (int i = 0, j = n - 1; i < n; j = i++) {
        int xi = vx[i], yi = vy[i];
        int xj = vx[j], yj = vy[j];

        if (px == xi && py == yi) return 1;

        int yi_gt_py = (yi > py);
        int yj_gt_py = (yj > py);
        if (yi_gt_py != yj_gt_py) {

            long long dx = (long long)(xj - xi);
            long long dy = (long long)(yj - yi);
            long long lhs = (long long)(px - xi) * dy;
            long long rhs = (long long)(py - yi) * dx;
            if ((dy > 0 && lhs < rhs) || (dy < 0 && lhs > rhs)) inside = !inside;
        }
    }
    return inside;
}

/*
 * ----------------------------------------------------------------------------
 *  orient_ll
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Calculer l'orientation signée de trois points entiers A, B et C.
 *
 *  Paramètres :
 *    [entrée] ax/ay, bx/by, cx/cy.
 *
 *  Retour :
 *    Déterminant 64 bits, nul si colinéaires.
 *
 *  Fonctionnement :
 *    Utilise le multiplicateur partagé de produits bruts ; signe positif
 *    pour C à gauche de AB dans le repère des coordonnées fournies.
 *
 * ----------------------------------------------------------------------------
 */
static long long orient_ll(int ax,int ay,int bx,int by,int cx,int cy) {
    /* Former les différences en long avant les deux produits 32x32. */
    return fixed_product32_fast((long)bx-ax,(long)cy-ay)
         - fixed_product32_fast((long)by-ay,(long)cx-ax);
}
/*
 * ----------------------------------------------------------------------------
 *  on_seg_ll
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Tester si C appartient à la boîte fermée du segment AB.
 *
 *  Paramètres :
 *    [entrée] ax/ay, bx/by, cx/cy.
 *
 *  Retour :
 *    Booléen, sans effet de bord.
 *
 *  Fonctionnement :
 *    Ne teste pas la colinéarité : l'appelant doit aussi vérifier
 *    l'orientation nulle.
 *
 * ----------------------------------------------------------------------------
 */
static int on_seg_ll(int ax,int ay,int bx,int by,int cx,int cy) {
    /* Comparer les coordonnées aux deux intervalles fermés indépendamment du
     * sens du segment.
     */
    return ((ax<=cx && cx<=bx)||(bx<=cx && cx<=ax)) &&
           ((ay<=cy && cy<=by)||(by<=cy && cy<=ay));
}

/*
 * ----------------------------------------------------------------------------
 *  segs_intersect_int_fixed64
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Tester une intersection propre de deux segments entiers.
 *
 *  Paramètres :
 *    [entrée] x1/y1 et x2/y2 pour AB, x3/y3 et x4/y4 pour CD.
 *
 *  Retour :
 *    1 si les extrémités sont strictement de côtés opposés dans les deux
 *    sens, 0 sinon.
 *
 *  Fonctionnement :
 *    Exclut contacts d'extrémité et recouvrements colinéaires. Les
 *    déterminants emploient les produits 64 bits partagés du moteur.
 *
 * ----------------------------------------------------------------------------
 */
static int segs_intersect_int_fixed64(
    int x1, int y1, int x2, int y2,
    int x3, int y3, int x4, int y4)
{
    /* Vérifier les deux changements stricts de signe ; une orientation nulle
     * ne passe pas le test.
     */
    long long o1 = orient_ll(x1,y1,x2,y2,x3,y3);
    long long o2 = orient_ll(x1,y1,x2,y2,x4,y4);
    long long o3,o4;

    if(!((o1>0 && o2<0)||(o1<0 && o2>0)))return 0;
    o3=orient_ll(x3,y3,x4,y4,x1,y1);
    o4=orient_ll(x3,y3,x4,y4,x2,y2);
    return (o3>0 && o4<0)||(o3<0 && o4>0);
}

/*
 * ----------------------------------------------------------------------------
 *  segs_intersect_int
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Tester une intersection propre de deux segments entiers.
 *
 *  Paramètres :
 *    [entrée] x1/y1 et x2/y2 pour AB, x3/y3 et x4/y4 pour CD.
 *
 *  Retour :
 *    1 si les extrémités sont strictement de côtés opposés dans les deux
 *    sens, 0 sinon.
 *
 *  Fonctionnement :
 *    Exclut contacts d'extrémité et recouvrements colinéaires. Les
 *    déterminants emploient les produits 64 bits partagés du moteur.
 *
 * ----------------------------------------------------------------------------
 */
static int segs_intersect_int(int x1,int y1,int x2,int y2,int x3,int y3,int x4,int y4) {
    /* Déléguer intégralement au prédicat 64 bits pour conserver cette
     * interface historique.
     */
    return segs_intersect_int_fixed64(x1,y1,x2,y2,x3,y3,x4,y4);
}
/*
 * ----------------------------------------------------------------------------
 *  point_in_poly_int
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Tester l'intérieur strict d'une face projetée à un point entier.
 *
 *  Paramètres :
 *    [entrée] px/py, faces/vtx, f à base zéro, n longueur valide de sa
 *    liste.
 *
 *  Retour :
 *    Parité 0/1, avec rejet explicite du contour ; n<=0 rend 0.
 *
 *  Fonctionnement :
 *    Utilise une intersection par division entière en long ; les différences
 *    restent des int natifs. N'est pas équivalent à la variante tableaux
 *    pour les bords.
 *
 * ----------------------------------------------------------------------------
 */
static int point_in_poly_int(int px,int py,FaceArrays3D* faces, VertexArrays3D* vtx, int f, int n) {
    int cnt=0,off=faces->vertex_indices_ptr[f];
    int i,vi,xi,yi;
    if(n<=0)return 0;
    vi=faces->vertex_indices_buffer[off]-1;
    xi=vtx->x2d[vi];yi=vtx->y2d[vi];
    /* Parcourir le cycle ; exclure le point sur une arête avant le test de parité. */
    for(i=0;i<n;++i){
        int next=i+1,vj,xj,yj;
        if(next==n)next=0;
        vj=faces->vertex_indices_buffer[off+next]-1;
        xj=vtx->x2d[vj];yj=vtx->y2d[vj];

        if(on_seg_ll(xi,yi,xj,yj,px,py) && orient_ll(xi,yi,xj,yj,px,py)==0)return 0;
        if((yi>py)!=(yj>py)){

            int dx=xj-xi,dpy=py-yi,dy=yj-yi;
            long crossing=(long)dx*(long)dpy/(long)dy+(long)xi;
            if((long)px<crossing)cnt^=1;
        }
        xi=xj;yi=yj;
    }
    return cnt;
}
/*
 * ----------------------------------------------------------------------------
 *  faces_vertices_equal
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Comparer deux contours projetés modulo rotation cyclique et inversion.
 *
 *  Paramètres :
 *    [entrée] faces/vtx et f1/f2.
 *
 *  Retour :
 *    1 si les suites de coordonnées entières sont égales dans un des deux
 *    sens, 0 sinon.
 *
 *  Fonctionnement :
 *    Ne compare pas la profondeur, ni l'identité OBJ des sommets ; une
 *    égalité projetée n'est pas une égalité 3D.
 *
 * ----------------------------------------------------------------------------
 */
static int faces_vertices_equal(FaceArrays3D* faces, VertexArrays3D* vtx, int f1, int f2) {
    int n1 = faces->vertex_count[f1];
    int n2 = faces->vertex_count[f2];
    if (n1 != n2 || n1 <= 0) return 0;
    int off1 = faces->vertex_indices_ptr[f1];
    int off2 = faces->vertex_indices_ptr[f2];

    /* Essayer toutes les origines cycliques dans le sens direct. */
    for (int shift = 0; shift < n1; ++shift) {
        int ok = 1;
        for (int k = 0; k < n1; ++k) {
            int idx1 = faces->vertex_indices_buffer[off1 + k] - 1;
            int idx2 = faces->vertex_indices_buffer[off2 + ((shift + k) % n1)] - 1;
            if (idx1 < 0 || idx2 < 0) { ok = 0; break; }
            if (vtx->x2d[idx1] != vtx->x2d[idx2] || vtx->y2d[idx1] != vtx->y2d[idx2]) { ok = 0; break; }
        }
        if (ok) return 1;
    }

    for (int shift = 0; shift < n1; ++shift) {
        int ok = 1;
        for (int k = 0; k < n1; ++k) {
            int idx1 = faces->vertex_indices_buffer[off1 + k] - 1;
            /* Comparer ensuite les coordonnées dans le sens inverse. */
            int idx2 = faces->vertex_indices_buffer[off2 + ((shift - k + n1) % n1)] - 1;
            if (idx1 < 0 || idx2 < 0) { ok = 0; break; }
            if (vtx->x2d[idx1] != vtx->x2d[idx2] || vtx->y2d[idx1] != vtx->y2d[idx2]) { ok = 0; break; }
        }
        if (ok) return 1;
    }
    return 0;
}

/*
 * ----------------------------------------------------------------------------
 *  compute_intersection_centroid_ordered_fixed_work
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Découper un sujet par les demi-plans du clip et calculer un point
 *    central.
 *
 *  Paramètres :
 *    [entrée] model, subj/clip à base zéro ; shared_work, espace emprunté
 *    assez grand ou NULL pour allocation locale.
 *
 *  Retour :
 *    1 si résultat produit, 0 sinon ; dans le chemin entier, 1 peut
 *    accompagner une aire nulle.
 *
 *  Sorties et effets de bord :
 *    outx/outy facultatifs et out_area2 obligatoire pour le chemin entier ;
 *    fractional_x, fractional_y et fractional_area2, tous trois non NULL,
 *    activent le chemin subpixel en double.
 *
 *    caches et diagnostics globaux ; ne libère jamais shared_work.
 *
 *  Fonctionnement :
 *    Malgré « fixed », le clipping intermédiaire utilise des doubles. Les
 *    entrées entières sont arrondies après clipping, puis l'aire et le
 *    centroïde utilisent des sommes entières ; le mode fractionnaire
 *    conserve la précision subpixel. Précondition géométrique : clip convexe
 *    pour l'intersection générale correcte par Sutherland-Hodgman. Limite de
 *    travail DEBUG_CLIP_MAX=512 : les écritures excédentaires sont
 *    tronquées, sans reconstruction générale d'un contour concave.
 *
 * ----------------------------------------------------------------------------
 */
static int compute_intersection_centroid_ordered_fixed_work(Model3D* model, int subj, int clip, int* outx, int* outy, long long* out_area2, double* shared_work, double* fractional_x, double* fractional_y, double* fractional_area2) {

    /* Vérifier les faces et les longueurs avant d'organiser les espaces de
     * travail.
     */
    if (!model || out_area2 == NULL) return 0;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    if (subj < 0 || clip < 0 || subj >= faces->face_count || clip >= faces->face_count) return 0;
    int sn = faces->vertex_count[subj];
    int cn = faces->vertex_count[clip];
    if (sn < 3 || cn < 3) {
        debug_clip_fixed_vcount = 0; debug_clip_raw_area2 = 0; debug_clip_fixed_area = 0.0; return 0;
    }

    double *work_storage;
    double *precise_sx, *precise_sy, *precise_cx, *precise_cy;
    double *clip_edge_x=NULL,*clip_edge_y=NULL;
    int fractional = (fractional_x && fractional_y && fractional_area2);
    int *input_storage, *output_storage;
    int *sx, *sy, *cx, *cy;
    double *buf_x1, *buf_y1, *buf_x2, *buf_y2;
    int *out_px, *out_py;
    long work_bytes = (long)sizeof(double) * (4L * DEBUG_CLIP_MAX
                    + (fractional ? 2L * cn : 0L));
    long integer_bytes = fractional ? 0L :
        (long)sizeof(int) * (2L * sn + 2L * cn + 2L * DEBUG_CLIP_MAX);
    /* Répartir un bloc entre les quatre buffers double et les listes
     * entières éventuelles.
     */
    work_storage = shared_work ? shared_work : (double*)malloc(work_bytes + integer_bytes);
    if (!work_storage) {
        debug_clip_fixed_vcount = 0; debug_clip_raw_area2 = 0;
        debug_clip_fixed_area = 0.0; *out_area2 = 0; return 0;
    }
    buf_x1 = work_storage;
    buf_y1 = buf_x1 + DEBUG_CLIP_MAX;
    buf_x2 = buf_y1 + DEBUG_CLIP_MAX;
    buf_y2 = buf_x2 + DEBUG_CLIP_MAX;
    /* Organiser les buffers selon que le chemin conserve les doubles ou
     * requiert des listes entières.
     */
    if (fractional) {
        sx = sy = cx = cy = out_px = out_py = NULL;

        precise_sx = buf_x1;precise_sy = buf_y1;
        precise_cx = work_storage + 4L * DEBUG_CLIP_MAX;
        precise_cy = precise_cx + cn;
    } else {
        input_storage = (int*)((char*)work_storage + work_bytes);
        output_storage = input_storage + 2 * sn + 2 * cn;
        sx = input_storage;sy = sx + sn;cx = sy + sn;cy = cx + cn;
        out_px = output_storage;out_py = out_px + DEBUG_CLIP_MAX;
        precise_sx = precise_sy = precise_cx = precise_cy = NULL;
    }
    /* Lire les projections entières existantes si le mode subpixel est désactivé. */
    if (!fractional) {
    for (int i = 0; i < sn; ++i) {
        int vi = faces->vertex_indices_buffer[faces->vertex_indices_ptr[subj] + i] - 1;
        sx[i] = vtx->x2d[vi]; sy[i] = vtx->y2d[vi];
    }
    for (int i = 0; i < cn; ++i) {
        int vi = faces->vertex_indices_buffer[faces->vertex_indices_ptr[clip] + i] - 1;
        cx[i] = vtx->x2d[vi]; cy[i] = vtx->y2d[vi];
    }
    }

    if (fractional) {
        /* En subpixel, construire les contours normalisés et réutiliser les
         * vecteurs du clip.
         */
        double scale = (double)s_global_proj_scale_fixed / (double)FIXED_SCALE;
        double *unused_edge_x,*unused_edge_y;
        if (scale <= 0.0 ||
            !fractional_face_gather(model,subj,sn,scale,precise_sx,precise_sy,
                                      &unused_edge_x,&unused_edge_y) ||
            !fractional_face_gather(model,clip,cn,scale,precise_cx,precise_cy,
                                      &clip_edge_x,&clip_edge_y)) {
            *fractional_area2 = 0.0;
            if (!shared_work) free(work_storage);
            return 0;
        }
    }

    if (!fractional) {
        /* Normaliser le winding des deux contours entiers pour conserver le
         * demi-plan gauche.
         */
        long long area2_subj = 0;
        int fast_subj = (sn <= 64);
        int wi;
        if (fast_subj) {
            for (wi = 0; wi < sn; ++wi) {
                if (sx[wi] < 0 || sx[wi] > 319 || sy[wi] < 0 || sy[wi] > 199) {
                    fast_subj = 0; break;
                }
            }
        }
        if (fast_subj) {
            long sum = 0;
            int prev = sn - 1;
            for (wi = 0; wi < sn; ++wi) {
                sum += (long)sx[prev] * sy[wi] - (long)sx[wi] * sy[prev];
                prev = wi;
            }
            area2_subj = (long long)sum;
        } else {
            int prev = sn - 1;
            for (wi = 0; wi < sn; ++wi) {
                area2_subj += (long long)sx[prev] * (long long)sy[wi]
                            - (long long)sx[wi] * (long long)sy[prev];
                prev = wi;
            }
        }

        if (area2_subj < 0) {

            for (int a = 0, b = sn - 1; a < b; ++a, --b) {
                int tx = sx[a]; sx[a] = sx[b]; sx[b] = tx;
                int ty = sy[a]; sy[a] = sy[b]; sy[b] = ty;
            }
        }
        /* Normaliser également le winding du clip pour que tous ses
         * demi-plans suivent la même convention.
         */
        long long area2_clip = 0;
        int fast_clip = (cn <= 64);
        if (fast_clip) {
            for (wi = 0; wi < cn; ++wi) {
                if (cx[wi] < 0 || cx[wi] > 319 || cy[wi] < 0 || cy[wi] > 199) {
                    fast_clip = 0; break;
                }
            }
        }
        if (fast_clip) {
            long sum = 0;
            int prev = cn - 1;
            for (wi = 0; wi < cn; ++wi) {
                sum += (long)cx[prev] * cy[wi] - (long)cx[wi] * cy[prev];
                prev = wi;
            }
            area2_clip = (long long)sum;
        } else {
            int prev = cn - 1;
            for (wi = 0; wi < cn; ++wi) {
                area2_clip += (long long)cx[prev] * (long long)cy[wi]
                            - (long long)cx[wi] * (long long)cy[prev];
                prev = wi;
            }
        }
        if (area2_clip < 0) {

            for (int a = 0, b = cn - 1; a < b; ++a, --b) {
                int tx = cx[a]; cx[a] = cx[b]; cx[b] = tx;
                int ty = cy[a]; cy[a] = cy[b]; cy[b] = ty;
            }
        }
    }

    /* Le pointeur de log reste NULL dans cette implémentation ; les
     * écritures conditionnelles sont inactives.
     */
    FILE* dlog = NULL;
    if (!fractional && ((debug_overlap_subj == subj && debug_overlap_clip == clip) || (subj == 12 && clip == 50))) {

        if (dlog) {
            fprintf(dlog, "--- compute_intersection_centroid_ordered_fixed debug (subj=%d clip=%d) ---\n", subj, clip);
            fprintf(dlog, "input subj verts (count=%d):\n", sn);
            for (int i = 0; i < sn; ++i) fprintf(dlog, "%d %d\n", sx[i], sy[i]);
            fprintf(dlog, "input clip verts (count=%d):\n", cn);
            for (int i = 0; i < cn; ++i) fprintf(dlog, "%d %d\n", cx[i], cy[i]);
            fflush(dlog);
        }
    }
    int cur_n = sn;
    if(!fractional)for (int i = 0; i < cur_n && i < DEBUG_CLIP_MAX; ++i) {
        buf_x1[i] = (double)sx[i];
        buf_y1[i] = (double)sy[i];
    }

    /* Découper successivement le contour courant par chaque arête du clip. */
    for (int ci = 0; ci < cn && cur_n > 0; ++ci) {
        int ci2 = ci + 1;
        double cx1d, cy1d, ex, ey;
        double sx1d, sy1d, side1,first_side;
        int inside1;
        int out_n_tmp = 0;
        int si;
        if (ci2 == cn) ci2 = 0;
        cx1d = fractional ? precise_cx[ci] : (double)cx[ci];
        cy1d = fractional ? precise_cy[ci] : (double)cy[ci];

        ex = clip_edge_x ? clip_edge_x[ci] :
             (fractional ? precise_cx[ci2] : (double)cx[ci2]) - cx1d;
        ey = clip_edge_y ? clip_edge_y[ci] :
             (fractional ? precise_cy[ci2] : (double)cy[ci2]) - cy1d;
        sx1d = buf_x1[0]; sy1d = buf_y1[0];
        side1 = ex * (sy1d - cy1d) - ey * (sx1d - cx1d);
        first_side=side1;
        inside1 = (side1 >= 0.0);
        for (si = 0; si < cur_n; ++si) {
            int sj = si + 1;
            double sx2d, sy2d, side2;
            int inside2;
            if (sj == cur_n) sj = 0;
            sx2d = buf_x1[sj]; sy2d = buf_y1[sj];

            side2 = (sj==0)?first_side:
                    ex * (sy2d - cy1d) - ey * (sx2d - cx1d);
            inside2 = (side2 >= 0.0);
            /* Émettre le sommet intérieur ou l'intersection lors d'un
             * changement de côté ; respecter la capacité du buffer.
             */
            if (inside1 && inside2) {
                if (out_n_tmp < DEBUG_CLIP_MAX) {
                    buf_x2[out_n_tmp] = sx2d;
                    buf_y2[out_n_tmp] = sy2d;
                    out_n_tmp++;
                }
            } else if (inside1 != inside2) {
                double dx = sx2d - sx1d, dy = sy2d - sy1d;

                double denom = fractional ? side1 - side2 : dx * ey - dy * ex;
                double t = 0.0;
                double ix, iy;
                if (fabs(denom) > 1e-12) {
                    if (fractional) t = side1 / denom;
                    else t = ((cx1d - sx1d) * ey - (cy1d - sy1d) * ex) / denom;
                }
                ix = sx1d + t * dx;
                iy = sy1d + t * dy;
                if (out_n_tmp < DEBUG_CLIP_MAX) {
                    buf_x2[out_n_tmp] = ix;
                    buf_y2[out_n_tmp] = iy;
                    out_n_tmp++;
                }
                if (inside2 && out_n_tmp < DEBUG_CLIP_MAX) {
                    buf_x2[out_n_tmp] = sx2d;
                    buf_y2[out_n_tmp] = sy2d;
                    out_n_tmp++;
                }
            }
            sx1d = sx2d;
            sy1d = sy2d;
            side1 = side2;
            inside1 = inside2;
        }
        cur_n = out_n_tmp;

        {
            double *tmp;
            tmp = buf_x1; buf_x1 = buf_x2; buf_x2 = tmp;
            tmp = buf_y1; buf_y1 = buf_y2; buf_y2 = tmp;
        }
        if (dlog) {
            fprintf(dlog, "after clip edge %d -> vertex_count=%d\n", ci, cur_n);
            for (int kk = 0; kk < cur_n; ++kk) fprintf(dlog, "  %.6f %.6f\n", buf_x1[kk], buf_y1[kk]);
            fflush(dlog);
        }
    }

    /* Calculer aire et centroïde subpixel autour d'une origine locale pour
     * réduire la perte numérique.
     */
    if (fractional_x && fractional_y && fractional_area2) {
        double area2 = 0.0, nx = 0.0, ny = 0.0;
        double ox, oy;
        int i, prev;
        *fractional_area2 = 0.0;
        if (cur_n < 3) {
            if (dlog) fclose(dlog);
            if (!shared_work) free(work_storage);
            return 0;
        }
        ox = buf_x1[0]; oy = buf_y1[0];
        prev = cur_n - 1;
        for (i = 0; i < cur_n; ++i) {
            double x0 = buf_x1[prev] - ox, y0 = buf_y1[prev] - oy;
            double x1 = buf_x1[i] - ox, y1 = buf_y1[i] - oy;
            double cross = x0 * y1 - x1 * y0;
            area2 += cross;
            nx += (x0 + x1) * cross;
            ny += (y0 + y1) * cross;
            prev = i;
        }
        if (fabs(area2) <= 1e-12) {
            if (dlog) fclose(dlog);
            if (!shared_work) free(work_storage);
            return 0;
        }
        *fractional_x = ox + nx / (3.0 * area2);
        *fractional_y = oy + ny / (3.0 * area2);
        *fractional_area2 = fabs(area2);
        if (dlog) fclose(dlog);
        if (!shared_work) free(work_storage);
        return 1;
    }

    /* Pour la voie entière, arrondir puis retirer doublons adjacents et
     * points colinéaires.
     */
    int final_n = 0;
    for (int i = 0; i < cur_n; ++i) {
        int px = (int)floor(buf_x1[i] + 0.5);
        int py = (int)floor(buf_y1[i] + 0.5);
        if (final_n > 0 && out_px[final_n-1] == px && out_py[final_n-1] == py) continue;
        if (final_n >= 2) {
            int x1 = out_px[final_n-2], y1 = out_py[final_n-2];
            int x2 = out_px[final_n-1], y2 = out_py[final_n-1];
            long long cross;
            if (x1 >= 0 && x1 <= 319 && x2 >= 0 && x2 <= 319 && px >= 0 && px <= 319 &&
                y1 >= 0 && y1 <= 199 && y2 >= 0 && y2 <= 199 && py >= 0 && py <= 199) {
                long cross32 = ((long)x2 - x1) * ((long)py - y1)
                             - ((long)y2 - y1) * ((long)px - x1);
                cross = (long long)cross32;
            } else {
                cross = (long long)(x2 - x1) * (long long)(py - y1)
                      - (long long)(y2 - y1) * (long long)(px - x1);
            }
            if (cross == 0) { out_px[final_n-1] = px; out_py[final_n-1] = py; continue; }
        }
        if (final_n < DEBUG_CLIP_MAX) { out_px[final_n] = px; out_py[final_n] = py; final_n++; }
    }
    if (final_n >= 2 && out_px[0] == out_px[final_n-1] && out_py[0] == out_py[final_n-1]) final_n--;
    if (final_n < 3) {
        debug_clip_fixed_vcount = 0; debug_clip_raw_area2 = 0; debug_clip_fixed_area = 0.0; *out_area2 = 0;
        if (dlog) { fprintf(dlog, "final_n < 3 -> no polygon\n"); fclose(dlog); }
        if (!shared_work) free(work_storage);
        return 0;
    }

    if (final_n > DEBUG_CLIP_MAX) final_n = DEBUG_CLIP_MAX;
    for (int i = 0; i < final_n; ++i) { debug_clip_fixed_vx[i] = out_px[i]; debug_clip_fixed_vy[i] = out_py[i]; }
    debug_clip_fixed_vcount = final_n;
    if (dlog) {
        fprintf(dlog, "rounded final polygon (count=%d):\n", final_n);
        for (int i = 0; i < final_n; ++i) fprintf(dlog, "%d %d\n", out_px[i], out_py[i]);
        fflush(dlog);
    }

    /* Calculer aire orientée et moments ; prendre les sommes 32 bits
     * seulement sous les bornes vérifiées.
     */
    long long raw_area2_signed = 0;
    long long cx_num = 0, cy_num = 0;
    {
        /* Prouver les bornes de taille et de coordonnées avant de réduire
         * les moments de surface à des sommes 32 bits.
         */
        int fast32 = (final_n <= 32);
        int i;
        if (fast32) {
            for (i = 0; i < final_n; ++i) {
                if (out_px[i] < 0 || out_px[i] > 319 ||
                    out_py[i] < 0 || out_py[i] > 199) {
                    fast32 = 0;
                    break;
                }
            }
        }
        if (fast32) {
            long area32 = 0, cx32 = 0, cy32 = 0;
            int j = final_n - 1;
            for (i = 0; i < final_n; ++i) {
                long cross32 = (long)out_px[j] * out_py[i]
                             - (long)out_py[j] * out_px[i];
                area32 += cross32;
                cx32 += ((long)out_px[j] + out_px[i]) * cross32;
                cy32 += ((long)out_py[j] + out_py[i]) * cross32;
                j = i;
            }
            raw_area2_signed = (long long)area32;
            cx_num = (long long)cx32;
            cy_num = (long long)cy32;
        } else {
            int j = final_n - 1;
            for (i = 0; i < final_n; ++i) {
                long long xi = (long long)out_px[j];
                long long yi = (long long)out_py[j];
                long long xj = (long long)out_px[i];
                long long yj = (long long)out_py[i];
                long long cross = xi * yj - yi * xj;
                raw_area2_signed += cross;
                cx_num += (xi + xj) * cross;
                cy_num += (yi + yj) * cross;
                j = i;
            }
        }
    }
    /* Rendre une aire nulle et un sommet comme point de repli pour un
     * contour dégénéré.
     */
    if (raw_area2_signed == 0) {
        debug_clip_raw_area2 = 0; debug_clip_fixed_area = 0.0; *out_area2 = 0;
        if (dlog) { fprintf(dlog, "raw_area2_signed == 0 -> degenerate polygon\n"); fflush(dlog); }
        if (outx) *outx = out_px[0]; if (outy) *outy = out_py[0];
        if (!shared_work) free(work_storage);
        if (dlog) fclose(dlog);
        return 1;
    }
    long long abs_raw = (raw_area2_signed >= 0) ? raw_area2_signed : -raw_area2_signed;
    debug_clip_raw_area2 = abs_raw;
    debug_clip_fixed_area = fabs((double)raw_area2_signed) * 0.5;

    /* Normaliser le signe du dénominateur et arrondir le centroïde avec une
     * division 32 bits si prouvée sûre.
     */
    long long numer_x = cx_num;
    long long numer_y = cy_num;
    long long denom = 3 * raw_area2_signed;
    if (denom < 0) { denom = -denom; numer_x = -numer_x; numer_y = -numer_y; }
    long long cx_i, cy_i;
    /* Choisir la division courte seulement si dénominateur, numérateurs et
     * ajout du demi-pas sont représentables.
     */
    if (denom <= 2147483647LL &&
        numer_x >= -2147483647LL && numer_x <= 2147483647LL &&
        numer_y >= -2147483647LL && numer_y <= 2147483647LL) {
        long d32 = (long)denom;
        long nx32 = (long)numer_x;
        long ny32 = (long)numer_y;
        long half32 = d32 / 2;

        if ((nx32 >= 0 ? nx32 : -nx32) <= 2147483647L - half32 &&
            (ny32 >= 0 ? ny32 : -ny32) <= 2147483647L - half32) {
            long x32 = (nx32 >= 0) ? (nx32 + half32) / d32
                                   : -((-nx32 + half32) / d32);
            long y32 = (ny32 >= 0) ? (ny32 + half32) / d32
                                   : -((-ny32 + half32) / d32);
            cx_i = (long long)x32;
            cy_i = (long long)y32;
        } else {
            cx_i = (numer_x >= 0) ? (numer_x + denom/2) / denom : -((-numer_x + denom/2) / denom);
            cy_i = (numer_y >= 0) ? (numer_y + denom/2) / denom : -((-numer_y + denom/2) / denom);
        }
    } else {
        cx_i = (numer_x >= 0) ? (numer_x + denom/2) / denom : -((-numer_x + denom/2) / denom);
        cy_i = (numer_y >= 0) ? (numer_y + denom/2) / denom : -((-numer_y + denom/2) / denom);
    }
    if (outx) *outx = (int)cx_i; if (outy) *outy = (int)cy_i;

    if (out_area2) *out_area2 = (long long)abs_raw;
    if (!shared_work) free(work_storage);
    return 1;
}
/*
 * ----------------------------------------------------------------------------
 *  compute_intersection_centroid_ordered_fixed
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Appeler le clipping ordonné entier avec un espace de travail réutilisé.
 *
 *  Paramètres :
 *    [entrée] model, subj/clip
 *
 *  Retour :
 *    Résultat du noyau, dont 1 peut accompagner une aire nulle.
 *
 *  Sorties et effets de bord :
 *    outx/outy facultatifs, out_area2 obligatoire.
 *
 *    cache de workspace et diagnostics globaux ; mémoire non transférée à
 *    l'appelant.
 *
 * ----------------------------------------------------------------------------
 */
static int compute_intersection_centroid_ordered_fixed(Model3D* model, int subj, int clip, int* outx, int* outy, long long* out_area2) {
    FaceArrays3D *faces;
    double *work;
    long bytes;
    int sn,cn;
    if (!model || !out_area2) return 0;
    faces=&model->faces;
    if (subj < 0 || clip < 0 || subj >= faces->face_count || clip >= faces->face_count) return 0;
    sn=faces->vertex_count[subj];cn=faces->vertex_count[clip];
    if (order_cache_owner != model) ordering_cache_invalidate(model);
    /* Dimensionner puis obtenir le workspace partagé. */
    bytes=(long)sizeof(double)*(4L*DEBUG_CLIP_MAX)
         +(long)sizeof(int)*(2L*sn+2L*cn+2L*DEBUG_CLIP_MAX);
    work=ordering_workspace(bytes);
    if (!work) {
        debug_clip_fixed_vcount=0;debug_clip_raw_area2=0;debug_clip_fixed_area=0.0;*out_area2=0;
        return 0;
    }
    /* Laisser au noyau le clipping, l'arrondi et les conventions de résultat. */
    return compute_intersection_centroid_ordered_fixed_work(model, subj, clip, outx, outy, out_area2, work, NULL, NULL, NULL);
}
/*
 * ----------------------------------------------------------------------------
 *  compute_intersection_centroid_fractional_tryboth
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Essayer le clipping subpixel dans les deux sens sans arrondir le
 *    centroïde.
 *
 *  Paramètres :
 *    [entrée] model, subj/clip
 *
 *  Retour :
 *    1 si un sens réussit, 0 sinon.
 *
 *  Sorties et effets de bord :
 *    outx/outy et out_area2, double de l'aire positive en pixels carrés.
 *
 *  Fonctionnement :
 *    Réutilise un workspace partagé ; l'inversion sujet/clip n'annule pas
 *    les limites de convexité du clipping. Aucune modification des
 *    coordonnées entières du modèle.
 *
 * ----------------------------------------------------------------------------
 */
static int compute_intersection_centroid_fractional_tryboth(Model3D *model,
        int subj, int clip, double *outx, double *outy, double *out_area2)
{
    FaceArrays3D *faces;
    double *work;
    long bytes;
    long long ignored_area2 = 0;
    int sn, cn, ok;
    if (!outx || !outy || !out_area2) return 0;
    *out_area2 = 0.0;
    if (!model) return 0;
    faces = &model->faces;
    if (subj < 0 || clip < 0 || subj >= faces->face_count || clip >= faces->face_count) return 0;
    sn = faces->vertex_count[subj]; cn = faces->vertex_count[clip];
    if (sn < 3 || cn < 3 || sn > DEBUG_CLIP_MAX || cn > DEBUG_CLIP_MAX) return 0;
    ORDER_COUNT(sh_calls);
    if (order_cache_owner != model) ordering_cache_invalidate(model);

    /* Réserver un workspace couvrant les deux longueurs possibles de clip. */
    bytes = (long)sizeof(double) * (4L * DEBUG_CLIP_MAX + 2L * (sn>cn?sn:cn));
    work = ordering_workspace(bytes);
    if (!work) return 0;
    /* Essayer le sens fourni puis son inverse uniquement si le premier échoue. */
    ok = compute_intersection_centroid_ordered_fixed_work(model, subj, clip,
             NULL, NULL, &ignored_area2, work, outx, outy, out_area2);
    if (!ok) ok = compute_intersection_centroid_ordered_fixed_work(model, clip, subj,
             NULL, NULL, &ignored_area2, work, outx, outy, out_area2);
    return ok;
}

/*
 * ----------------------------------------------------------------------------
 *  compute_intersection_centroid_ordered_fixed_tryboth
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Essayer le clipping entier puis chercher un point de repli commun.
 *
 *  Paramètres :
 *    [entrée] model, subj/clip
 *
 *  Retour :
 *    0 sur arguments/réserve invalides ; 1 peut désigner uniquement un repli
 *    avec out_area2=0 et out_fallback=1.
 *
 *  Sorties et effets de bord :
 *    outx/outy facultatifs, out_area2 obligatoire, out_fallback facultatif.
 *
 *  Fonctionnement :
 *    Après les deux sens SH, essaye le centre de bbox puis une grille 3x3 à
 *    l'intérieur strict des deux faces. Si rien ne convient, rend quand même
 *    le centre disponible. L'appelant doit tester l'aire ou le drapeau, pas
 *    seulement le booléen de retour.
 *
 * ----------------------------------------------------------------------------
 */
static int compute_intersection_centroid_ordered_fixed_tryboth(Model3D* model, int subj, int clip, int* outx, int* outy, long long* out_area2, int* out_fallback) {
    if (!model || out_area2 == NULL) return 0;
    /* Initialiser le marqueur puis réserver un seul espace pour les deux
     * essais SH.
     */
    if (out_fallback) *out_fallback = 0;

    {
        FaceArrays3D* f = &model->faces;
        int sn, cn;
        long bytes;
        double* shared_work;
        int res, res2;
        if (subj < 0 || clip < 0 || subj >= f->face_count || clip >= f->face_count) return 0;
        sn = f->vertex_count[subj];
        cn = f->vertex_count[clip];
        if (sn < 3 || cn < 3) {
            debug_clip_fixed_vcount = 0;
            debug_clip_raw_area2 = 0;
            debug_clip_fixed_area = 0.0;
            return 0;
        }
        bytes = (long)sizeof(double) * (4L * DEBUG_CLIP_MAX)
              + (long)sizeof(int) * (2L * sn + 2L * cn + 2L * DEBUG_CLIP_MAX);
        if (order_cache_owner != model) ordering_cache_invalidate(model);
        shared_work = ordering_workspace(bytes);
        if (!shared_work) {
            debug_clip_fixed_vcount = 0;
            debug_clip_raw_area2 = 0;
            debug_clip_fixed_area = 0.0;
            *out_area2 = 0;
            return 0;
        }

    /* Le nombre de sommets de diagnostic, et non le seul booléen, détermine
     * si l'essai ordonné est retenu.
     */
    res = compute_intersection_centroid_ordered_fixed_work(model, subj, clip, outx, outy, out_area2, shared_work, NULL, NULL, NULL);
    if (debug_clip_fixed_vcount >= 3) { if (out_fallback) *out_fallback = 0;  return res; }

    res2 = compute_intersection_centroid_ordered_fixed_work(model, clip, subj, outx, outy, out_area2, shared_work, NULL, NULL, NULL);
    if (debug_clip_fixed_vcount >= 3) { if (out_fallback) *out_fallback = 0;  return res2; }

    }

    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    /* Chercher un point intérieur commun dans le rectangle d'intersection. */
    int ix0=0, iy0=0, ix1=0, iy1=0;
    int found = 0;
    if (compute_bbox_intersection(model, subj, clip, &ix0, &iy0, &ix1, &iy1)) {
        int cx0 = (ix0 + ix1) / 2; int cy0 = (iy0 + iy1) / 2;
        if (point_in_poly_int(cx0, cy0, faces, vtx, subj, faces->vertex_count[subj]) && point_in_poly_int(cx0, cy0, faces, vtx, clip, faces->vertex_count[clip])) {
            if (outx) *outx = cx0; if (outy) *outy = cy0; found = 1;
        } else {
            int W = ix1 - ix0; int H = iy1 - iy0;
            for (int gx = 0; gx < 3 && !found; ++gx) for (int gy = 0; gy < 3 && !found; ++gy) {
                int tx = ix0 + (((2*gx + 1) * W + 3) / 6);
                int ty = iy0 + (((2*gy + 1) * H + 3) / 6);
                if (point_in_poly_int(tx, ty, faces, vtx, subj, faces->vertex_count[subj]) && point_in_poly_int(tx, ty, faces, vtx, clip, faces->vertex_count[clip])) { if (outx) *outx = tx; if (outy) *outy = ty; found = 1; break; }
            }
        }
    } else {

        ix0 = faces->minx[subj] > faces->minx[clip] ? faces->minx[subj] : faces->minx[clip];
        ix1 = faces->maxx[subj] < faces->maxx[clip] ? faces->maxx[subj] : faces->maxx[clip];
        iy0 = faces->miny[subj] > faces->miny[clip] ? faces->miny[subj] : faces->miny[clip];
        iy1 = faces->maxy[subj] < faces->maxy[clip] ? faces->maxy[subj] : faces->maxy[clip];
        int cx0 = (ix0 + ix1) / 2; int cy0 = (iy0 + iy1) / 2;
        if (point_in_poly_int(cx0, cy0, faces, vtx, subj, faces->vertex_count[subj]) && point_in_poly_int(cx0, cy0, faces, vtx, clip, faces->vertex_count[clip])) {
            if (outx) *outx = cx0; if (outy) *outy = cy0; found = 1;
        } else {
            int W = ix1 - ix0; int H = iy1 - iy0;
            for (int gx = 0; gx < 3 && !found; ++gx) for (int gy = 0; gy < 3 && !found; ++gy) {
                int tx = ix0 + (((2*gx + 1) * W + 3) / 6);
                int ty = iy0 + (((2*gy + 1) * H + 3) / 6);
                if (point_in_poly_int(tx, ty, faces, vtx, subj, faces->vertex_count[subj]) && point_in_poly_int(tx, ty, faces, vtx, clip, faces->vertex_count[clip])) { if (outx) *outx = tx; if (outy) *outy = ty; found = 1; break; }
            }
        }
    }
    /* Faute de point validé, conserver un centre de repli sans prétendre à
     * une aire positive.
     */
    if (!found) {
        if (outx) *outx = (ix0 + ix1) / 2;
        if (outy) *outy = (iy0 + iy1) / 2;
    }

    /* Signaler le repli et annuler toutes les valeurs d'aire du diagnostic. */
    if (out_fallback) *out_fallback = 1;
    debug_clip_fixed_vcount = 0; debug_clip_raw_area2 = 0; debug_clip_fixed_area = 0.0;
    debug_clip_centroid_x = outx ? *outx : 0; debug_clip_centroid_y = outy ? *outy : 0;
    if (out_area2) *out_area2 = 0;
    return 1;
}
/*
 * ----------------------------------------------------------------------------
 *  compute_intersection_region_bbox
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Construire l'intersection de deux régions QuickDraw et en lire la bbox.
 *
 *  Paramètres :
 *    [entrée] model, subj/clip
 *
 *  Retour :
 *    Région commune détectée, ou 0 si échec.
 *
 *  Sorties et effets de bord :
 *    ix0/iy0/ix1/iy1
 *
 *    out_area2.
 *
 *  Préconditions :
 *    QuickDraw initialisé. Alloue trois régions et deux polygones. Attention
 *    : out_area2 contient le produit largeur*hauteur de la bbox, pas le
 *    double de l'aire de la région. Les tableaux sont lus sans pan ; les
 *    ressources QuickDraw sont libérées avant retour sur les chemins pris en
 *    charge.
 *
 * ----------------------------------------------------------------------------
 */
static int compute_intersection_region_bbox(Model3D* model, int subj, int clip, int* ix0, int* iy0, int* ix1, int* iy1, long long* out_area2) {
    if (!model || !ix0 || !iy0 || !ix1 || !iy1) return 0;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    if (subj < 0 || clip < 0 || subj >= faces->face_count || clip >= faces->face_count) return 0;
    int n1 = faces->vertex_count[subj];
    int n2 = faces->vertex_count[clip];
    if (n1 <= 0 || n2 <= 0) return 0;

    int off1 = faces->vertex_indices_ptr[subj];
    int off2 = faces->vertex_indices_ptr[clip];

    /* Mesurer les boîtes projetées nécessaires aux replis de cette routine. */
    int minx1, maxx1, miny1, maxy1;
    {
        int vi = faces->vertex_indices_buffer[off1] - 1;
        minx1 = maxx1 = vtx->x2d[vi];
        miny1 = maxy1 = vtx->y2d[vi];
        for (int i = 1; i < n1; ++i) {
            vi = faces->vertex_indices_buffer[off1 + i] - 1;
            int vx = vtx->x2d[vi], vy = vtx->y2d[vi];
            if (vx < minx1) minx1 = vx; if (vx > maxx1) maxx1 = vx;
            if (vy < miny1) miny1 = vy; if (vy > maxy1) maxy1 = vy;
        }
    }
    int minx2, maxx2, miny2, maxy2;
    {
        int wi = faces->vertex_indices_buffer[off2] - 1;
        minx2 = maxx2 = vtx->x2d[wi];
        miny2 = maxy2 = vtx->y2d[wi];
        for (int i = 1; i < n2; ++i) {
            wi = faces->vertex_indices_buffer[off2 + i] - 1;
            int wx = vtx->x2d[wi], wy = vtx->y2d[wi];
            if (wx < minx2) minx2 = wx; if (wx > maxx2) maxx2 = wx;
            if (wy < miny2) miny2 = wy; if (wy > maxy2) maxy2 = wy;
        }
    }

    /* Réserver les trois régions et libérer celles obtenues si la réserve
     * est partielle.
     */
    RegionHndl r1 = NewRgn();
    RegionHndl r2 = NewRgn();
    RegionHndl r3 = NewRgn();
    if (!r1 || !r2 || !r3) {
        if (r1) DisposeRgn(r1);
        if (r2) DisposeRgn(r2);
        if (r3) DisposeRgn(r3);
        return 0;
    }

    {
        /* Enregistrer le premier contour dans une région en préservant
         * l'état du crayon.
         */
        Handle subj_poly = OpenPoly();
        off1 = faces->vertex_indices_ptr[subj];
        int v0 = faces->vertex_indices_buffer[off1] - 1;
        MoveTo(vtx->x2d[v0], vtx->y2d[v0]);
        for (int i = 1; i < n1; ++i) {
            int vi = faces->vertex_indices_buffer[off1 + i] - 1;
            LineTo(vtx->x2d[vi], vtx->y2d[vi]);
        }
        ClosePoly();
        PenState ps_subj; GetPenState(&ps_subj);
        OpenRgn(); FramePoly(subj_poly); CloseRgn(r1);
        SetPenState(&ps_subj);
        KillPoly(subj_poly);
    }

    {
        /* Construire la région du second contour avant l'intersection. */
        Handle clip_poly = OpenPoly();
        off2 = faces->vertex_indices_ptr[clip];
        int w0 = faces->vertex_indices_buffer[off2] - 1;
        MoveTo(vtx->x2d[w0], vtx->y2d[w0]);
        for (int i = 1; i < n2; ++i) {
            int wi = faces->vertex_indices_buffer[off2 + i] - 1;
            LineTo(vtx->x2d[wi], vtx->y2d[wi]);
        }
        ClosePoly();
        PenState ps_clip; GetPenState(&ps_clip);
        OpenRgn(); FramePoly(clip_poly); CloseRgn(r2);
        SetPenState(&ps_clip);
        KillPoly(clip_poly);
    }

    SectRgn(r1, r2, r3);

    int has = 0;
    if (!EmptyRgn(r3)) {

        HLock((Handle)r3);
        short* sp = (short*)(*r3);
        int w0 = (int)sp[0];
        int raw_top, raw_left, raw_bottom, raw_right;
        if (w0 >= 10 && (w0/2) >= 5) {
            raw_top = (int)sp[1]; raw_left = (int)sp[2]; raw_bottom = (int)sp[3]; raw_right = (int)sp[4];
        } else {

            raw_top = (int)sp[0]; raw_left = (int)sp[1]; raw_bottom = (int)sp[2]; raw_right = (int)sp[3];
        }

        int h_top = (raw_top < raw_bottom) ? raw_top : raw_bottom;
        int h_bottom = (raw_top < raw_bottom) ? raw_bottom : raw_top;
        int h_left = (raw_left < raw_right) ? raw_left : raw_right;
        int h_right = (raw_left < raw_right) ? raw_right : raw_left;

        *ix0 = h_left; *iy0 = h_top; *ix1 = h_right; *iy1 = h_bottom;
        if (*ix0 <= *ix1 && *iy0 <= *iy1) {
            /* Estimer la taille par l'aire de bbox ; ce champ ne représente
             * pas l'aire exacte de région.
             */
            if (out_area2) *out_area2 = (long long)((*ix1 - *ix0) * (*iy1 - *iy0));

            HUnlock((Handle)r3);
            DisposeRgn(r1); DisposeRgn(r2); DisposeRgn(r3);
            return 1;
        }
        HUnlock((Handle)r3);
    }
    DisposeRgn(r1); DisposeRgn(r2); DisposeRgn(r3);
    return has;
}

/*
 * ----------------------------------------------------------------------------
 *  compute_intersection_centroid_ordered_qd_fixed
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Retourner le centre de bbox de l'intersection QuickDraw de deux faces.
 *
 *  Paramètres :
 *    [entrée] model, subj/clip
 *
 *  Retour :
 *    1 si la région fournit un résultat, 0 sinon.
 *
 *  Sorties et effets de bord :
 *    outx/outy et out_area2.
 *
 *  Fonctionnement :
 *    Le nom historique est trompeur : ce n'est pas un centroïde de surface.
 *    out_area2 est l'aire de bbox, pas un double d'aire. QuickDraw doit être
 *    initialisé.
 *
 * ----------------------------------------------------------------------------
 */
static int compute_intersection_centroid_ordered_qd_fixed(Model3D* model, int subj, int clip, int* outx, int* outy, long long* out_area2) {
    int ix0,iy0,ix1,iy1; long long a2=0;
    /* Exiger la région avant de moyenner ses bornes entières. */
    if (!compute_intersection_region_bbox(model, subj, clip, &ix0, &iy0, &ix1, &iy1, &a2)) return 0;
    if (outx) *outx = (ix0 + ix1) / 2;
    if (outy) *outy = (iy0 + iy1) / 2;
    if (out_area2) *out_area2 = a2;
    return 1;
}

segment "ordering";

/*
 * ----------------------------------------------------------------------------
 *  projected_polygons_overlap_v1
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Tester le chevauchement avec les filtres et heuristiques historiques
 *    V1.
 *
 *  Paramètres :
 *    [entrée] model, f1/f2 valides et métriques à jour.
 *
 *  Retour :
 *    1 pour chevauchement retenu, 0 pour rejet, cas non concluants ou garde
 *    temporelle.
 *
 *  Sorties et effets de bord :
 *    compteurs, diagnostics, cache de clipping et lecture de GetTick.
 *
 *  Fonctionnement :
 *    Après boîtes et tests de segments/échantillons, recourt au clipping et
 *    vérifie ses aires avec seuils et bornes. N'est pas équivalent à un test
 *    géométrique exact.
 *
 *    Le watchdog nommé MS compare directement des ticks ; ne pas le décrire
 *    en millisecondes.
 *
 * ----------------------------------------------------------------------------
 */
static int projected_polygons_overlap_v1(Model3D* model, int f1, int f2)
{
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int n1 = faces->vertex_count[f1];
    int n2 = faces->vertex_count[f2];
    if (n1 < 3 || n2 < 3)
        return 0;

    /* Démarrer instrumentation et garde temporelle avant le rejet de boîtes. */
    overlapCheckCount++;

    long proj_start_tick = GetTick();
    const long PROJ_OVERLAP_WATCHDOG_MS = 1000;
    int minx1 = faces->minx[f1];
    int maxx1 = faces->maxx[f1];
    int miny1 = faces->miny[f1];
    int maxy1 = faces->maxy[f1];
    int minx2 = faces->minx[f2];
    int maxx2 = faces->maxx[f2];
    int miny2 = faces->miny[f2];
    int maxy2 = faces->maxy[f2];

    if (maxx1 <= minx2 ||
        maxx2 <= minx1 ||
        maxy1 <= miny2 ||
        maxy2 <= miny1)
    {
        return 0;
    }
    int off1 = faces->vertex_indices_ptr[f1];
    int off2 = faces->vertex_indices_ptr[f2];

    int f2_precomputed = (n2 <= MAX_FACE_VERTICES);
    int f2_cx[MAX_FACE_VERTICES];
    int f2_cy[MAX_FACE_VERTICES];
    int f2_dx[MAX_FACE_VERTICES];
    int f2_dy[MAX_FACE_VERTICES];
    int f2_minx[MAX_FACE_VERTICES];
    int f2_maxx[MAX_FACE_VERTICES];
    int f2_miny[MAX_FACE_VERTICES];
    int f2_maxy[MAX_FACE_VERTICES];
    /* Préparer les arêtes de la seconde face pour réutiliser leurs
     * coordonnées et bornes.
     */
    if (f2_precomputed) {
        int j;
        for (j = 0; j < n2; ++j) {
            int j2;
            int vc;
            int vd;
            int cx;
            int cy;
            int dx;
            int dy;
            j2 = (j + 1) % n2;
            vc =
                faces->vertex_indices_buffer[
                    off2 + j] - 1;
            vd =
                faces->vertex_indices_buffer[
                    off2 + j2] - 1;
            cx = vtx->x2d[vc];
            cy = vtx->y2d[vc];
            dx = vtx->x2d[vd];
            dy = vtx->y2d[vd];
            f2_cx[j] = cx;
            f2_cy[j] = cy;
            f2_dx[j] = dx;
            f2_dy[j] = dy;
            f2_minx[j] = (cx < dx) ? cx : dx;
            f2_maxx[j] = (cx > dx) ? cx : dx;
            f2_miny[j] = (cy < dy) ? cy : dy;
            f2_maxy[j] = (cy > dy) ? cy : dy;
        }
    }

    /* Chercher des indices de chevauchement sans encore valider une aire commune. */
    int candidate = 0;
    {
        int i;
        for (i = 0; i < n1; ++i) {
            int i2;
            int va;
            int vb;
            int ax;
            int ay;
            int bx;
            int by;
            int aminx;
            int amaxx;
            int aminy;
            int amaxy;
            if (GetTick() - proj_start_tick >
                PROJ_OVERLAP_WATCHDOG_MS)
            {
                return 0;
            }
            i2 = (i + 1) % n1;
            va =
                faces->vertex_indices_buffer[
                    off1 + i] - 1;
            vb =
                faces->vertex_indices_buffer[
                    off1 + i2] - 1;
            ax = vtx->x2d[va];
            ay = vtx->y2d[va];
            bx = vtx->x2d[vb];
            by = vtx->y2d[vb];
            aminx = (ax < bx) ? ax : bx;
            amaxx = (ax > bx) ? ax : bx;
            aminy = (ay < by) ? ay : by;
            amaxy = (ay > by) ? ay : by;
            {
                int j;
                for (j = 0; j < n2; ++j) {
                    int cx;
                    int cy;
                    int dx;
                    int dy;
                    int cminx;
                    int cmaxx;
                    int cminy;
                    int cmaxy;
                    if (f2_precomputed) {
                        cx = f2_cx[j];
                        cy = f2_cy[j];
                        dx = f2_dx[j];
                        dy = f2_dy[j];
                        cminx = f2_minx[j];
                        cmaxx = f2_maxx[j];
                        cminy = f2_miny[j];
                        cmaxy = f2_maxy[j];
                    } else {
                        int j2;
                        int vc;
                        int vd;
                        j2 = (j + 1) % n2;
                        vc =
                            faces->vertex_indices_buffer[
                                off2 + j] - 1;
                        vd =
                            faces->vertex_indices_buffer[
                                off2 + j2] - 1;
                        cx = vtx->x2d[vc];
                        cy = vtx->y2d[vc];
                        dx = vtx->x2d[vd];
                        dy = vtx->y2d[vd];
                        cminx = (cx < dx) ? cx : dx;
                        cmaxx = (cx > dx) ? cx : dx;
                        cminy = (cy < dy) ? cy : dy;
                        cmaxy = (cy > dy) ? cy : dy;
                    }

                    if (amaxx <= cminx ||
                        cmaxx <= aminx ||
                        amaxy <= cminy ||
                        cmaxy <= aminy)
                    {
                        continue;
                    }
                    if (segs_intersect_int(
                            ax, ay,
                            bx, by,
                            cx, cy,
                            dx, dy))
                    {
                        candidate = 1;
                        overlapCheckCount++;
                        overlapSegiAccept++;
                        break;
                    }
                }
            }
            if (candidate)
                return 1;
        }
    }

    /* Sans croisement propre, chercher un sommet strictement intérieur ; les
     * recherches suivantes affinent le cas resté indécis.
     */
    if (!candidate) {
        int ii;
        for (ii = 0; ii < n1; ++ii) {
            int vid;
            int px;
            int py;
            if (GetTick() - proj_start_tick >
                PROJ_OVERLAP_WATCHDOG_MS)
            {
                return 0;
            }
            vid =
                faces->vertex_indices_buffer[
                    off1 + ii] - 1;
            if (vid < 0 ||
                vid >= vtx->vertex_count)
            {
                continue;
            }
            px = vtx->x2d[vid];
            py = vtx->y2d[vid];
            if (px < minx2 ||
                px > maxx2 ||
                py < miny2 ||
                py > maxy2)
            {
                continue;
            }
            if (point_in_poly_int(
                    px, py,
                    faces, vtx,
                    f2, n2))
            {
                candidate = 1;
                break;
            }
        }
    }
    if (!candidate) {
        int jj;
        for (jj = 0; jj < n2; ++jj) {
            int vid;
            int px;
            int py;
            if (GetTick() - proj_start_tick >
                PROJ_OVERLAP_WATCHDOG_MS)
            {
                return 0;
            }
            vid =
                faces->vertex_indices_buffer[
                    off2 + jj] - 1;
            if (vid < 0 ||
                vid >= vtx->vertex_count)
            {
                continue;
            }
            px = vtx->x2d[vid];
            py = vtx->y2d[vid];
            if (px < minx1 ||
                px > maxx1 ||
                py < miny1 ||
                py > maxy1)
            {
                continue;
            }
            if (point_in_poly_int(
                    px, py,
                    faces, vtx,
                    f1, n1))
            {
                candidate = 1;
                break;
            }
        }
    }

    if (!candidate) {
        /* Reconnaître des contours projetés égaux, sans en déduire une
         * égalité des faces 3D.
         */
        if (faces_vertices_equal(
                faces,
                vtx,
                f1,
                f2))
        {
            return 1;
        }
    }

    {
        int oxmin;
        int oxmax;
        int oymin;
        int oymax;
        oxmin =
            (minx1 > minx2) ?
            minx1 : minx2;
        oxmax =
            (maxx1 < maxx2) ?
            maxx1 : maxx2;
        oymin =
            (miny1 > miny2) ?
            miny1 : miny2;
        oymax =
            (maxy1 < maxy2) ?
            maxy1 : maxy2;
        if (oxmin > oxmax ||
            oymin > oymax)
        {
            return 0;
        }

        {
            int cx;
            int cy;
            cx = (oxmin + oxmax) / 2;
            cy = (oymin + oymax) / 2;
            if (point_in_poly_int(
                    cx, cy,
                    faces, vtx,
                    f1, n1) &&
                point_in_poly_int(
                    cx, cy,
                    faces, vtx,
                    f2, n2))
            {
                overlapSampleAccept++;
                return 1;
            }
        }

        {
            int ixmin;
            int ixmax;
            int iymin;
            int iymax;
            int W;
            int H;
            /* Sonder la zone de boîtes commune pour confirmer un intérieur
             * partagé avant le clipping.
             */
            int sample_accept;
            int N;
            int sx;
            int sy;
            ixmin = oxmin;
            ixmax = oxmax;
            iymin = oymin;
            iymax = oymax;
            W = ixmax - ixmin;
            H = iymax - iymin;
            sample_accept = 0;
            N = 3;
            for (sx = 0;
                 sx < N;
                 ++sx)
            {
                for (sy = 0;
                     sy < N;
                     ++sy)
                {
                    int tx;
                    int ty;
                    tx =
                        ixmin +
                        (((2 * sx + 1) * W +
                          (2 * N - 1)) /
                         (2 * N));
                    ty =
                        iymin +
                        (((2 * sy + 1) * H +
                          (2 * N - 1)) /
                         (2 * N));
                    if (point_in_poly_int(
                            tx, ty,
                            faces, vtx,
                            f1, n1) &&
                        point_in_poly_int(
                            tx, ty,
                            faces, vtx,
                            f2, n2))
                    {
                        sample_accept = 1;
                        break;
                    }
                }
                if (sample_accept)
                    break;
            }
            if (sample_accept) {
                overlapSampleAccept++;
                return 1;
            }
        }

        overlapClipCalls++;
        {
            /* Préparer les seuils d'aire et une borne maximale liée à la
             * bbox pour filtrer les résultats numériques aberrants.
             */
            unsigned long long min_area2_fixed;
            unsigned long long bbox_limit;
            min_area2_fixed =
                (unsigned long long)
                (2.0 *
                 MIN_INTERSECTION_AREA_PIXELS *
                 (double)FIXED_SCALE *
                 (double)FIXED_SCALE +
                 0.5);
            bbox_limit = 0;
            if (!(oxmin > oxmax ||
                  oymin > oymax))
            {
                unsigned long long bbox_pixels;
                unsigned long long scale2;
                bbox_pixels =
                    (unsigned long long)
                    (oxmax - oxmin) *
                    (unsigned long long)
                    (oymax - oymin);
                scale2 =
                    (unsigned long long)
                    FIXED_SCALE *
                    (unsigned long long)
                    FIXED_SCALE;
                bbox_limit =
                    2ULL *
                    bbox_pixels *
                    scale2;
            }

            {
                int tx;
                int ty;
                long long a2;
                int ok1;
                unsigned long long area2_1;
                int valid1;
                tx = 0;
                ty = 0;
                a2 = 0;
                debug_overlap_subj = f1;
                debug_overlap_clip = f2;
                ok1 =
                    compute_intersection_centroid_ordered_fixed(
                        model,
                        f1,
                        f2,
                        &tx,
                        &ty,
                        &a2);
                debug_overlap_subj = -1;
                debug_overlap_clip = -1;
                area2_1 =
                    (unsigned long long)
                    ((a2 >= 0) ?
                     a2 : -a2);
                debug_clip_fixed_area =
                    (double)area2_1 * 0.5;
                valid1 =
                    (ok1 &&
                     debug_clip_fixed_vcount >= 3 &&
                     area2_1 >= min_area2_fixed &&
                     area2_1 <= bbox_limit);
                /* Accepter le premier clipping seulement si taille de
                 * contour et aire satisfont les seuils ; sinon essayer le
                 * sens inverse.
                 */
                if (valid1) {
                    overlapClipAccept++;
                    return 1;
                }
            }

            {
                int tx2;
                int ty2;
                long long a22;
                int ok2;
                unsigned long long area2_2;
                int valid2;
                tx2 = 0;
                ty2 = 0;
                a22 = 0;
                debug_overlap_subj = f2;
                debug_overlap_clip = f1;
                ok2 =
                    compute_intersection_centroid_ordered_fixed(
                        model,
                        f2,
                        f1,
                        &tx2,
                        &ty2,
                        &a22);
                debug_overlap_subj = -1;
                debug_overlap_clip = -1;
                area2_2 =
                    (unsigned long long)
                    ((a22 >= 0) ?
                     a22 : -a22);
                valid2 =
                    (ok2 &&
                     debug_clip_fixed_vcount >= 3 &&
                     area2_2 >= min_area2_fixed &&
                     area2_2 <= bbox_limit);
                /* Valider séparément l'aire obtenue par l'inversion
                 * sujet/clip.
                 */
                if (valid2) {
                    overlapClipAccept++;
                    return 1;
                }
            }
        }
    }
    /* Rendre un rejet si aucun chemin heuristique n'a validé le chevauchement. */
    return 0;
}
/*
 * ----------------------------------------------------------------------------
 *  projected_polygons_overlap_simple
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Tester rapidement croisements propres et inclusion stricte de sommets.
 *
 *  Paramètres :
 *    [entrée] model, f1/f2 valides
 *
 *  Retour :
 *    Booléen.
 *
 *  Fonctionnement :
 *    Aucun recours au clipping. Les contacts seuls ne suffisent pas. Peut
 *    manquer les contours coïncidents ou certains chevauchements dont tous
 *    les sommets sont sur le contour ; les filtres de boîtes d'arêtes et
 *    divisions entières définissent ses limites.
 *
 * ----------------------------------------------------------------------------
 */
static int projected_polygons_overlap_simple(Model3D* model, int f1, int f2) {
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int n1 = faces->vertex_count[f1];
    int n2 = faces->vertex_count[f2];
    if (n1 < 3 || n2 < 3) return 0;
    int minx1 = faces->minx[f1], maxx1 = faces->maxx[f1], miny1 = faces->miny[f1], maxy1 = faces->maxy[f1];
    int minx2 = faces->minx[f2], maxx2 = faces->maxx[f2], miny2 = faces->miny[f2], maxy2 = faces->maxy[f2];
    /* Écarter les boîtes séparées avant les tests d'arêtes. */
    if (maxx1 < minx2 || maxx2 < minx1 || maxy1 < miny2 || maxy2 < miny1) return 0;
    int off1 = faces->vertex_indices_ptr[f1];
    int off2 = faces->vertex_indices_ptr[f2];

    /* Chercher une intersection propre entre les deux contours. */
    for (int i = 0; i < n1; ++i) {
        int i2 = (i+1) % n1;
        int va = faces->vertex_indices_buffer[off1 + i] - 1;
        int vb = faces->vertex_indices_buffer[off1 + i2] - 1;
        int ax = vtx->x2d[va], ay = vtx->y2d[va];
        int bx = vtx->x2d[vb], by = vtx->y2d[vb];
        int aminx = ax < bx ? ax : bx; int amaxx = ax > bx ? ax : bx;
        int aminy = ay < by ? ay : by; int amaxy = ay > by ? ay : by;
        for (int j = 0; j < n2; ++j) {
            int j2 = (j+1) % n2;
            int vc = faces->vertex_indices_buffer[off2 + j] - 1;
            int vd = faces->vertex_indices_buffer[off2 + j2] - 1;
            int cx = vtx->x2d[vc], cy = vtx->y2d[vc];
            int dx = vtx->x2d[vd], dy = vtx->y2d[vd];
            int cminx = cx < dx ? cx : dx; int cmaxx = cx > dx ? cx : dx;
            int cminy = cy < dy ? cy : dy; int cmaxy = cy > dy ? cy : dy;

            if (amaxx <= cminx || cmaxx <= aminx || amaxy <= cminy || cmaxy <= aminy) continue;
            if (segs_intersect_int(ax,ay,bx,by,cx,cy,dx,dy)) return 1;
        }
    }

    /* Essayer ensuite les sommets de chaque face strictement à l'intérieur
     * de l'autre.
     */
    for (int ii = 0; ii < n1; ++ii) {
        int vid = faces->vertex_indices_buffer[off1 + ii] - 1;
        if (vid < 0 || vid >= vtx->vertex_count) continue;
        int px = vtx->x2d[vid], py = vtx->y2d[vid];
        if (px < minx2 || px > maxx2 || py < miny2 || py > maxy2) continue;
        if (point_in_poly_int(px, py, faces, vtx, f2, n2)) return 1;
    }
    for (int jj = 0; jj < n2; ++jj) {
        int vid = faces->vertex_indices_buffer[off2 + jj] - 1;
        if (vid < 0 || vid >= vtx->vertex_count) continue;
        int px = vtx->x2d[vid], py = vtx->y2d[vid];
        if (px < minx1 || px > maxx1 || py < miny1 || py > maxy1) continue;
        if (point_in_poly_int(px, py, faces, vtx, f1, n1)) return 1;
    }
    return 0;
}

segment "raster_overlap";

static int projected_polygons_overlap_raster(Model3D* model, int f1, int f2);
/*
 * ----------------------------------------------------------------------------
 *  projected_polygons_overlap_raster_x10
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Tester le chevauchement raster après un grossissement temporaire par
 *    dix.
 *
 *  Paramètres :
 *    [entrée] model, f1/f2
 *
 *  Retour :
 *    Booléen raster, pas une aire géométrique exacte.
 *
 *  Fonctionnement :
 *    Effets temporaires : échelle, coordonnées et boîtes de la paire,
 *    restaurées après le test.
 *
 *  Préconditions :
 *    profondeurs non nulles et indices valides.
 *
 * ----------------------------------------------------------------------------
 */
static int projected_polygons_overlap_raster_x10(
    Model3D* model,
    int f1,
    int f2)
{
    FaceArrays3D* faces;
    RCHZoomPatch patch;
    Fixed32 test_scale;
    int result;
    if (!model)
        return 0;
    faces = &model->faces;

    if (faces->maxx[f1] < faces->minx[f2] ||
        faces->maxx[f2] < faces->minx[f1] ||
        faces->maxy[f1] < faces->miny[f2] ||
        faces->maxy[f2] < faces->miny[f1])
    {
        return 0;
    }
    /* Préparer le grossissement après le premier rejet de boîtes. */
    test_scale =
        FIXED_MUL_64(
            s_global_proj_scale_fixed,
            FLOAT_TO_FIXED(10.0f));
    /* Sauvegarder et reprojeter la paire, tester puis restaurer les données. */
    rch_patch_pair_at_scale(
        model, f1, f2, test_scale, &patch);
    result =
        projected_polygons_overlap_raster(
            model, f1, f2);
    rch_unpatch_pair(
        model, &patch, f1, f2);
    return result;
}

#define RASTER_MAX_INTERSECTIONS MAX_FACE_VERTICES

/*
 * ----------------------------------------------------------------------------
 *  raster_sort_long
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Trier une courte liste d'intersections par insertion croissante.
 *
 *  Paramètres :
 *    [entrée-sortie] values de longueur count.
 *
 *  Retour :
 *    Aucun ; aucune allocation.
 *
 *  Fonctionnement :
 *    Les valeurs égales conservent leur ordre de rencontre.
 *
 * ----------------------------------------------------------------------------
 */
static void raster_sort_long(long* values, int count)
{
    int i;
    /* Insérer chaque valeur dans le préfixe déjà trié. */
    for (i = 1; i < count; ++i) {
        long v;
        int j;
        v = values[i];
        j = i - 1;
        while (j >= 0 && values[j] > v) {
            values[j + 1] = values[j];
            --j;
        }
        values[j + 1] = v;
    }
}

/*
 * ----------------------------------------------------------------------------
 *  raster_intervals_share_pixel
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Tester si deux intervalles ouverts contiennent un centre de pixel
 *    commun.
 *
 *  Paramètres :
 *    [entrée] a1/a2 et b1/b2 en coordonnées doublées ; ordre quelconque des
 *    bornes.
 *
 *  Retour :
 *    1 si leur intersection contient un entier impair strictement entre les
 *    deux limites, 0 sinon.
 *
 *  Fonctionnement :
 *    Exclut les contacts sur les limites.
 *
 * ----------------------------------------------------------------------------
 */
static int raster_intervals_share_pixel(
    long a1,
    long a2,
    long b1,
    long b2)
{
    long left;
    long right;
    long first;
    /* Normaliser l'ordre des deux intervalles. */
    if (a1 > a2) {
        long t = a1;
        a1 = a2;
        a2 = t;
    }
    if (b1 > b2) {
        long t = b1;
        b1 = b2;
        b2 = t;
    }
    left =
        (a1 > b1) ?
        a1 : b1;
    right =
        (a2 < b2) ?
        a2 : b2;

    if (left >= right)
        return 0;

    /* Chercher le premier entier impair strictement après la borne gauche. */
    first = left + 1;
    if ((first & 1L) == 0)
        ++first;
    return (first < right);
}

typedef struct {
    long x1;
    long ymin;
    long ymax;
    long divisor;
    long quotient;
    long remainder;
    long step_quotient;
    long step_remainder;
    long start_y;
} RasterPreparedEdge;
/*
 * ----------------------------------------------------------------------------
 *  raster_euclidean_divide
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Effectuer une division euclidienne avec reste positif ou nul.
 *
 *  Paramètres :
 *    [entrée] numerator signé 64 bits, divisor strictement positif
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    *quotient et *remainder en long.
 *
 *  Fonctionnement :
 *    Corrige le quotient tronqué de C quand le reste est négatif ; le
 *    quotient final doit tenir dans long.
 *
 * ----------------------------------------------------------------------------
 */
static void raster_euclidean_divide(
    long long numerator,
    long divisor,
    long* quotient,
    long* remainder)
{
    long long q;
    long long r;
    q = numerator / (long long)divisor;
    r = numerator % (long long)divisor;
    /* Transformer la division tronquée de C en division par défaut. */
    if (r < 0) {
        --q;
        r += divisor;
    }
    *quotient = (long)q;
    *remainder = (long)r;
}
/*
 * ----------------------------------------------------------------------------
 *  raster_prepare_edges_incremental
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Préparer les arêtes actives pour des lignes raster successives.
 *
 *  Paramètres :
 *    [entrée] model, face, scan_ymin inclus et scan_ymax exclu
 *
 *  Retour :
 *    Nombre préparé.
 *
 *  Sorties et effets de bord :
 *    edges, capacité au moins le nombre d'arêtes.
 *
 *  Fonctionnement :
 *    Travaille en coordonnées doublées, échantillonne aux centres y*2+1 et
 *    ignore les horizontales. Les quotients et restes permettent d'avancer
 *    sans redédivision.
 *
 * ----------------------------------------------------------------------------
 */
static int raster_prepare_edges_incremental(
    Model3D* model,
    int face,
    int scan_ymin,
    int scan_ymax,
    RasterPreparedEdge* edges)
{
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int n = faces->vertex_count[face];
    int off = faces->vertex_indices_ptr[face];
    long range_min = (long)scan_ymin * 2L + 1L;
    long range_max = (long)(scan_ymax - 1) * 2L + 1L;
    int count = 0;
    int i;
    /* Lire chaque arête, ignorer les horizontales et celles hors de la plage. */
    for (i = 0; i < n; ++i) {
        int i2 = (i + 1) % n;
        int va =
            faces->vertex_indices_buffer[off + i] - 1;
        int vb =
            faces->vertex_indices_buffer[off + i2] - 1;
        long x1 = (long)vtx->x2d[va] * 2L;
        long y1 = (long)vtx->y2d[va] * 2L;
        long x2 = (long)vtx->x2d[vb] * 2L;
        long y2 = (long)vtx->y2d[vb] * 2L;
        long dx = x2 - x1;
        long dy = y2 - y1;
        long ymin;
        long ymax;
        long first_y;
        long long numerator;
        long long step;
        RasterPreparedEdge* e;
        if (dy == 0)
            continue;
        ymin = (y1 < y2) ? y1 : y2;
        ymax = (y1 > y2) ? y1 : y2;

        if (ymax <= range_min ||
            ymin > range_max)
        {
            continue;
        }
        /* Normaliser le diviseur positif et préparer la première
         * intersection et son incrément.
         */
        e = &edges[count];
        e->x1 = x1;
        e->ymin = ymin;
        e->ymax = ymax;

        if (dy < 0) {
            dy = -dy;
            dx = -dx;
        }
        e->divisor = dy;

        first_y = range_min;
        if (first_y < ymin)
            first_y = ymin + 1L;
        e->start_y = first_y;

        numerator =
            (long long)(first_y - y1) *
            (long long)dx;
        raster_euclidean_divide(
            numerator,
            dy,
            &e->quotient,
            &e->remainder);

        step = (long long)dx * 2LL;
        raster_euclidean_divide(
            step,
            dy,
            &e->step_quotient,
            &e->step_remainder);
        ++count;
    }
    return count;
}
/*
 * ----------------------------------------------------------------------------
 *  raster_advance_edge
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Avancer une arête préparée d'une ligne raster.
 *
 *  Paramètres :
 *    [entrée-sortie] e.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Ajoute quotient et reste du pas, puis propage une unité au quotient si
 *    le reste atteint le diviseur positif.
 *
 * ----------------------------------------------------------------------------
 */
static void raster_advance_edge(
    RasterPreparedEdge* e)
{
    long remainder;
    /* Mettre à jour la fraction incrémentale avec une seule correction de retenue. */
    e->quotient += e->step_quotient;
    remainder =
        e->remainder + e->step_remainder;
    if (remainder >= e->divisor) {
        remainder -= e->divisor;
        ++e->quotient;
    }
    e->remainder = remainder;
}
/*
 * ----------------------------------------------------------------------------
 *  raster_edge_intersection
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Lire l'intersection entière tronquée vers zéro d'une arête préparée.
 *
 *  Paramètres :
 *    [entrée] e
 *
 *  Retour :
 *    X doublé.
 *
 *  Fonctionnement :
 *    Corrige le quotient euclidien négatif lorsqu'un reste subsiste, puis
 *    ajoute l'origine x1. Aucun effet de bord.
 *
 * ----------------------------------------------------------------------------
 */
static long raster_edge_intersection(
    const RasterPreparedEdge* e)
{
    long q = e->quotient;
    /* Convertir le quotient par défaut en quotient tronqué avant l'addition de x1. */
    if (q < 0 && e->remainder != 0)
        ++q;
    return e->x1 + q;
}
/*
 * ----------------------------------------------------------------------------
 *  raster_incremental_scanline
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Extraire et trier les intersections d'une ligne à partir d'arêtes
 *    préparées.
 *
 *  Paramètres :
 *    [entrée-sortie] edges
 *
 *    [entrée] edge_count et y entier
 *
 *  Retour :
 *    Nombre d'intersections.
 *
 *  Sorties et effets de bord :
 *    intersections, capacité edge_count.
 *
 *  Fonctionnement :
 *    Avance les arêtes actives d'un pas ; les appels doivent suivre la
 *    progression de lignes attendue.
 *
 * ----------------------------------------------------------------------------
 */
static int raster_incremental_scanline(
    RasterPreparedEdge* edges,
    int edge_count,
    int y,
    long* intersections)
{
    long scan_y = ((long)y * 2L) + 1L;
    int count = 0;
    int i;
    /* Activer les arêtes à leur première ligne et les exclure à ymax. */
    for (i = 0; i < edge_count; ++i) {
        RasterPreparedEdge* e = &edges[i];
        if (scan_y < e->start_y)
            continue;
        if (scan_y < e->ymax) {
            intersections[count++] =
                raster_edge_intersection(e);
        }

        if (scan_y < e->ymax)
            raster_advance_edge(e);
    }
    /* Trier les intersections pour l'appariement pair/impair des spans. */
    raster_sort_long(intersections, count);
    return count;
}
/*
 * ----------------------------------------------------------------------------
 *  projected_polygons_overlap_raster
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Chercher un centre de pixel strictement couvert par les deux faces.
 *
 *  Paramètres :
 *    [entrée] model, f1/f2 et boîtes à jour.
 *
 *  Retour :
 *    Booléen raster ; 0 aussi pour faces trop courtes ou dépassant
 *    RASTER_MAX_INTERSECTIONS.
 *
 *  Fonctionnement :
 *    Prépare les arêtes, parcourt les lignes de bbox communes et compare les
 *    spans pair/impair. Un chevauchement subpixel peut être absent de cet
 *    échantillonnage.
 *
 *  Limites :
 *    si la première face n'a pas deux intersections sur une ligne, la
 *    seconde n'est pas avancée sur cette ligne ; ne pas généraliser
 *    l'exactitude.
 *
 * ----------------------------------------------------------------------------
 */
static int projected_polygons_overlap_raster(
    Model3D* model,
    int f1,
    int f2)
{
    FaceArrays3D* faces;
    int n1;
    int n2;
    int xmin;
    int xmax;
    int ymin;
    int ymax;
    RasterPreparedEdge edges1[RASTER_MAX_INTERSECTIONS];
    RasterPreparedEdge edges2[RASTER_MAX_INTERSECTIONS];
    long intersections1[RASTER_MAX_INTERSECTIONS];
    long intersections2[RASTER_MAX_INTERSECTIONS];
    int edge_count1;
    int edge_count2;
    int y;
    if (!model)
        return 0;
    faces = &model->faces;
    n1 = faces->vertex_count[f1];
    n2 = faces->vertex_count[f2];
    if (n1 < 3 || n2 < 3)
        return 0;
    /* Contrôler la capacité locale des listes d'arêtes et d'intersections. */
    if (n1 > RASTER_MAX_INTERSECTIONS ||
        n2 > RASTER_MAX_INTERSECTIONS)
    {
        return 0;
    }
    /* Calculer la zone commune des boîtes sans la limiter au viewport matériel. */
    xmin =
        (faces->minx[f1] > faces->minx[f2]) ?
        faces->minx[f1] :
        faces->minx[f2];
    xmax =
        (faces->maxx[f1] < faces->maxx[f2]) ?
        faces->maxx[f1] :
        faces->maxx[f2];
    ymin =
        (faces->miny[f1] > faces->miny[f2]) ?
        faces->miny[f1] :
        faces->miny[f2];
    ymax =
        (faces->maxy[f1] < faces->maxy[f2]) ?
        faces->maxy[f1] :
        faces->maxy[f2];
    if (xmax <= xmin ||
        ymax <= ymin)
    {
        return 0;
    }
    /* Préparer les fractions incrémentales des deux contours. */
    edge_count1 =
        raster_prepare_edges_incremental(
            model,
            f1,
            ymin,
            ymax,
            edges1);
    if (edge_count1 < 2)
        return 0;
    edge_count2 =
        raster_prepare_edges_incremental(
            model,
            f2,
            ymin,
            ymax,
            edges2);
    if (edge_count2 < 2)
        return 0;
    /* Échantillonner les lignes et chercher un centre commun aux paires de spans. */
    for (y = ymin; y < ymax; ++y) {
        int count1;
        int count2;
        int i;
        int j;
        count1 =
            raster_incremental_scanline(
                edges1,
                edge_count1,
                y,
                intersections1);
        /* Le chemin saute cette ligne sans avancer les arêtes de la seconde
         * face si la première n'a pas assez d'intersections.
         */
        if (count1 < 2)
            continue;
        count2 =
            raster_incremental_scanline(
                edges2,
                edge_count2,
                y,
                intersections2);
        if (count2 < 2)
            continue;
        for (i = 0; i + 1 < count1; i += 2) {
            long a1 = intersections1[i];
            long a2 = intersections1[i + 1];
            for (j = 0; j + 1 < count2; j += 2) {
                long b1 = intersections2[j];
                long b2 = intersections2[j + 1];
                if (raster_intervals_share_pixel(
                        a1, a2, b1, b2))
                {
                    return 1;
                }
            }
        }
    }
    return 0;
}

/*
 * ----------------------------------------------------------------------------
 *  overlap_cross_double
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Calculer le déterminant orienté de AB et AP en double.
 *
 *  Paramètres :
 *    [entrée] ax/ay, bx/by et px/py.
 *
 *  Retour :
 *    (bx-ax)*(py-ay)-(by-ay)*(px-ax).
 *
 *  Fonctionnement :
 *    Aucun état modifié ; zéro signifie colinéaire selon l'arithmétique
 *    double.
 *
 * ----------------------------------------------------------------------------
 */
static double overlap_cross_double(double ax, double ay, double bx, double by,
                                  double px, double py)
{
    /* Conserver la même convention d'orientation dans les variantes flottantes. */
    return (bx-ax)*(py-ay) - (by-ay)*(px-ax);
}

/*
 * ----------------------------------------------------------------------------
 *  overlap_v2_point_in_face
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Classer un point double par rapport au contour projeté entier d'une
 *    face.
 *
 *  Paramètres :
 *    [entrée] model, face valide et px/py.
 *
 *  Retour :
 *    -1 sur le contour, 1 intérieur, 0 extérieur.
 *
 *  Fonctionnement :
 *    Parité du rayon horizontal et test explicite de colinéarité. La
 *    variante optimisée filtre le test de bord par la boîte de l'arête.
 *
 * ----------------------------------------------------------------------------
 */
static int overlap_v2_point_in_face(Model3D *model, int face,
                                    double px, double py)
{
    FaceArrays3D *faces;
    VertexArrays3D *vtx;
    int n, off, i, j, inside;
    double ax, ay, bx, by, c, minx, maxx, miny, maxy;
    double ix;

    faces = &model->faces;
    vtx = &model->vertices;
    n = faces->vertex_count[face];
    off = faces->vertex_indices_ptr[face];
    /* Parcourir les arêtes en distinguant bord et parité intérieure. */
    inside = 0;
    for (i = 0; i < n; ++i) {
        j = (i + 1 == n) ? 0 : i + 1;
        ax = (double)vtx->x2d[faces->vertex_indices_buffer[off+i]-1];
        ay = (double)vtx->y2d[faces->vertex_indices_buffer[off+i]-1];
        bx = (double)vtx->x2d[faces->vertex_indices_buffer[off+j]-1];
        by = (double)vtx->y2d[faces->vertex_indices_buffer[off+j]-1];
        c = overlap_cross_double(ax,ay,bx,by,px,py);
        minx = (ax < bx) ? ax : bx;
        maxx = (ax > bx) ? ax : bx;
        miny = (ay < by) ? ay : by;
        maxy = (ay > by) ? ay : by;
        if (c == 0.0 && px >= minx && px <= maxx &&
            py >= miny && py <= maxy) return -1;
        if ((ay > py) != (by > py)) {
            ix = ax + (py-ay)*(bx-ax)/(by-ay);
            if (ix > px) inside = !inside;
        }
    }
    return inside;
}

/*
 * ----------------------------------------------------------------------------
 *  overlap_v2_proper_cross
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Tester un croisement strict de segments doubles AB et CD.
 *
 *  Paramètres :
 *    [entrée] quatre couples ax/ay, bx/by, cx/cy, dx/dy.
 *
 *  Retour :
 *    1 si les deux paires de déterminants ont des signes strictement
 *    opposés, 0 sinon.
 *
 *  Fonctionnement :
 *    Exclut contacts et colinéarité ; aucun état modifié.
 *
 * ----------------------------------------------------------------------------
 */
static int overlap_v2_proper_cross(double ax, double ay, double bx, double by,
                                   double cx, double cy, double dx, double dy)
{
    double c1,c2,c3,c4;
    /* Tester les côtés dans les deux sens sans accepter de déterminant nul. */
    c1=overlap_cross_double(ax,ay,bx,by,cx,cy);
    c2=overlap_cross_double(ax,ay,bx,by,dx,dy);
    if (!((c1 > 0.0 && c2 < 0.0) ||
          (c1 < 0.0 && c2 > 0.0))) return 0;
    c3=overlap_cross_double(cx,cy,dx,dy,ax,ay);
    c4=overlap_cross_double(cx,cy,dx,dy,bx,by);
    return ((c3 > 0.0 && c4 < 0.0) ||
            (c3 < 0.0 && c4 > 0.0));
}

/*
 * ----------------------------------------------------------------------------
 *  overlap_v2_edge_probes
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Chercher un point intérieur commun près du milieu des arêtes des deux
 *    faces.
 *
 *  Paramètres :
 *    [entrée] model, f1/f2 valides.
 *
 *  Retour :
 *    1 si une sonde est strictement dans les deux faces, 0 sinon.
 *
 *  Fonctionnement :
 *    Deux côtés sont essayés à une distance normalisée par |dx|+|dy| et
 *    1024. Échantillonnage heuristique, pas une preuve d'absence.
 *
 * ----------------------------------------------------------------------------
 */
static int overlap_v2_edge_probes(Model3D *model, int f1, int f2)
{
    FaceArrays3D *faces;
    VertexArrays3D *vtx;
    int pass, f, n, off, i, j, va, vb;
    double ax,ay,bx,by,mx,my,dx,dy,scale,px,py;
    int side;
    faces=&model->faces;
    vtx=&model->vertices;
    /* Essayer chaque arête des deux contours ; ignorer celles de longueur nulle. */
    for (pass=0; pass<2; ++pass) {
        f = pass ? f2 : f1;
        n = faces->vertex_count[f];
        off = faces->vertex_indices_ptr[f];
        for (i=0; i<n; ++i) {
            j=(i+1==n)?0:i+1;
            va=faces->vertex_indices_buffer[off+i]-1;
            vb=faces->vertex_indices_buffer[off+j]-1;
            ax=(double)vtx->x2d[va]; ay=(double)vtx->y2d[va];
            bx=(double)vtx->x2d[vb]; by=(double)vtx->y2d[vb];
            dx=bx-ax; dy=by-ay;
            scale=((dx<0.0)?-dx:dx)+((dy<0.0)?-dy:dy);
            if (scale==0.0) continue;
            mx=(ax+bx)*0.5; my=(ay+by)*0.5;
            /* Sonder de part et d'autre du milieu avec la même convention
             * d'appartenance stricte.
             */
            for (side=-1; side<=1; side+=2) {
                px=mx+side*(-dy)/scale/1024.0;
                py=my+side*dx/scale/1024.0;
                if (overlap_v2_point_in_face(model,f1,px,py)==1 &&
                    overlap_v2_point_in_face(model,f2,px,py)==1)
                    return 1;
            }
        }
    }
    return 0;
}

/*
 * ----------------------------------------------------------------------------
 *  projected_polygons_overlap_v2_original
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Tester le chevauchement projeté par V2 original en double.
 *
 *  Paramètres :
 *    [entrée] model et f1/f2 valides
 *
 *  Retour :
 *    1 si un test trouve une zone commune, 0 sinon ou face trop courte.
 *
 *  Fonctionnement :
 *    Rejette d'abord les boîtes sans intérieur commun, puis essaye
 *    croisements propres, sommets strictement intérieurs et sondes près des
 *    milieux d'arêtes. Les seules touches de contours ne suffisent pas. Ne
 *    calcule pas l'aire d'intersection et ne modifie pas le modèle ; les
 *    variantes ne doivent pas être fusionnées sans comparaison de leurs
 *    limites numériques.
 *
 * ----------------------------------------------------------------------------
 */
static int projected_polygons_overlap_v2_original(Model3D *model, int f1, int f2)
{
    FaceArrays3D *faces;
    VertexArrays3D *vtx;
    int n1,n2,off1,off2,i,j,i2,j2,va,vb,vc,vd;
    double ax,ay,bx,by,cx,cy,dx,dy;
    /* Valider le modèle et les longueurs avant les filtres de boîtes. */
    if (!model) return 0;
    faces=&model->faces;
    vtx=&model->vertices;
    n1=faces->vertex_count[f1]; n2=faces->vertex_count[f2];
    if (n1<3 || n2<3) return 0;
    if (faces->maxx[f1]<=faces->minx[f2] ||
        faces->maxx[f2]<=faces->minx[f1] ||
        faces->maxy[f1]<=faces->miny[f2] ||
        faces->maxy[f2]<=faces->miny[f1]) return 0;
    off1=faces->vertex_indices_ptr[f1];
    off2=faces->vertex_indices_ptr[f2];
    /* Chercher les croisements propres entre les contours avant les
     * inclusions et les sondes.
     */
    for (i=0;i<n1;++i) {
        i2=(i+1==n1)?0:i+1;
        va=faces->vertex_indices_buffer[off1+i]-1;
        vb=faces->vertex_indices_buffer[off1+i2]-1;
        ax=(double)vtx->x2d[va]; ay=(double)vtx->y2d[va];
        bx=(double)vtx->x2d[vb]; by=(double)vtx->y2d[vb];
        for (j=0;j<n2;++j) {
            j2=(j+1==n2)?0:j+1;
            vc=faces->vertex_indices_buffer[off2+j]-1;
            vd=faces->vertex_indices_buffer[off2+j2]-1;
            cx=(double)vtx->x2d[vc]; cy=(double)vtx->y2d[vc];
            dx=(double)vtx->x2d[vd]; dy=(double)vtx->y2d[vd];
            if (overlap_v2_proper_cross(ax,ay,bx,by,cx,cy,dx,dy)) return 1;
        }
    }

    for (i=0;i<n1;++i) {
        va=faces->vertex_indices_buffer[off1+i]-1;
        if (overlap_v2_point_in_face(model,f2,
             (double)vtx->x2d[va],(double)vtx->y2d[va])==1) return 1;
    }
    for (j=0;j<n2;++j) {
        vc=faces->vertex_indices_buffer[off2+j]-1;
        if (overlap_v2_point_in_face(model,f1,
             (double)vtx->x2d[vc],(double)vtx->y2d[vc])==1) return 1;
    }
    return overlap_v2_edge_probes(model,f1,f2);
}

/*
 * ----------------------------------------------------------------------------
 *  ovmax_inside
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Classer un point double avec les tableaux projetés passés directement.
 *
 *  Paramètres :
 *    [entrée] ids à base un ; xs/ys ; off/n décrivant une face non vide ;
 *    px/py.
 *
 *  Retour :
 *    -1 bord, 1 intérieur, 0 extérieur.
 *
 *  Fonctionnement :
 *    Aucun état modifié. Conserve le sommet précédent pour limiter les
 *    lectures d'indices.
 *
 * ----------------------------------------------------------------------------
 */
static int ovmax_inside(const int *ids,const int *xs,const int *ys,
                        int off,int n,double px,double py)
{
    int i,k,prev,inside;
    double ax,ay,bx,by,ix;
    inside=0;
    /* Initialiser le cycle au dernier sommet, puis tester bord et parité. */
    prev=ids[off+n-1]-1;
    bx=(double)xs[prev]; by=(double)ys[prev];
    for(i=0;i<n;++i) {
        k=ids[off+i]-1;
        ax=bx; ay=by;
        bx=(double)xs[k]; by=(double)ys[k];

        if ((px>=ax && px<=bx || px>=bx && px<=ax) &&
            (py>=ay && py<=by || py>=by && py<=ay) &&
            overlap_cross_double(ax,ay,bx,by,px,py)==0.0) return -1;
        if ((ay>py)!=(by>py)) {
            ix=ax+(py-ay)*(bx-ax)/(by-ay);
            if(ix>px) inside=!inside;
        }
    }
    return inside;
}

/*
 * ----------------------------------------------------------------------------
 *  ovmax_probe
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Sonder les milieux d'arêtes dans la variante à tableaux directs.
 *
 *  Paramètres :
 *    [entrée] ids base un, xs/ys, off1/n1 et off2/n2.
 *
 *  Retour :
 *    Booléen de point strictement commun.
 *
 *  Fonctionnement :
 *    Même déplacement double normalisé de 1/1024 que V2 original. Ne modifie
 *    ni les tableaux ni les projections.
 *
 * ----------------------------------------------------------------------------
 */
static int ovmax_probe(const int *ids,const int *xs,const int *ys,
                       int off1,int n1,int off2,int n2)
{
    int pass,off,n,i,next,va,vb,side;
    double ax,ay,bx,by,dx,dy,scale,mx,my,px,py;
    /* Parcourir les deux contours et essayer les deux côtés de chaque milieu. */
    for(pass=0;pass<2;++pass) {
        off=pass?off2:off1;
        n=pass?n2:n1;
        for(i=0;i<n;++i) {
            next=(i+1==n)?0:i+1;
            va=ids[off+i]-1;
            vb=ids[off+next]-1;
            ax=(double)xs[va]; ay=(double)ys[va];
            bx=(double)xs[vb]; by=(double)ys[vb];
            dx=bx-ax; dy=by-ay;
            scale=((dx<0.0)?-dx:dx)+((dy<0.0)?-dy:dy);
            if(scale==0.0) continue;
            mx=(ax+bx)*0.5; my=(ay+by)*0.5;
            for(side=-1;side<=1;side+=2) {
                px=mx+side*(-dy)/scale/1024.0;
                py=my+side*dx/scale/1024.0;
                if(ovmax_inside(ids,xs,ys,off1,n1,px,py)==1 &&
                   ovmax_inside(ids,xs,ys,off2,n2,px,py)==1) return 1;
            }
        }
    }
    return 0;
}

/*
 * ----------------------------------------------------------------------------
 *  projected_polygons_overlap_v2_max
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Tester le chevauchement projeté par V2 Max avec pointeurs locaux et
 *    rejets par boîtes d'arêtes.
 *
 *  Paramètres :
 *    [entrée] model et f1/f2 valides
 *
 *  Retour :
 *    1 si un test trouve une zone commune, 0 sinon ou face trop courte.
 *
 *  Fonctionnement :
 *    Rejette d'abord les boîtes sans intérieur commun, puis essaye
 *    croisements propres, sommets strictement intérieurs et sondes près des
 *    milieux d'arêtes. Les seules touches de contours ne suffisent pas. Ne
 *    calcule pas l'aire d'intersection et ne modifie pas le modèle ; les
 *    variantes ne doivent pas être fusionnées sans comparaison de leurs
 *    limites numériques.
 *
 * ----------------------------------------------------------------------------
 */
static int projected_polygons_overlap_v2_max(Model3D *model,int f1,int f2)
{
    FaceArrays3D *faces;
    VertexArrays3D *vertices;
    int *ids,*xs,*ys;
    int n1,n2,off1,off2,i,j,va,vc;
    int iax,iay,ibx,iby,icx,icy,idx,idy;
    int minax,maxax,minay,maxay;
    double ax,ay,bx,by,cx,cy,dx,dy,c1,c2,c3,c4;
    /* Valider le modèle et les longueurs avant les filtres de boîtes. */
    if(!model) return 0;
    faces=&model->faces;
    n1=faces->vertex_count[f1]; n2=faces->vertex_count[f2];
    if(n1<3 || n2<3) return 0;
    if(faces->maxx[f1]<=faces->minx[f2] ||
       faces->maxx[f2]<=faces->minx[f1] ||
       faces->maxy[f1]<=faces->miny[f2] ||
       faces->maxy[f2]<=faces->miny[f1]) return 0;
    vertices=&model->vertices;
    ids=faces->vertex_indices_buffer;
    xs=vertices->x2d; ys=vertices->y2d;
    off1=faces->vertex_indices_ptr[f1];
    off2=faces->vertex_indices_ptr[f2];
    va=ids[off1+n1-1]-1;
    ibx=xs[va]; iby=ys[va];
    /* Chercher les croisements propres entre les contours avant les
     * inclusions et les sondes.
     */
    for(i=0;i<n1;++i) {
        iax=ibx; iay=iby;
        va=ids[off1+i]-1;
        ibx=xs[va]; iby=ys[va];
        minax=iax<ibx?iax:ibx; maxax=iax>ibx?iax:ibx;
        minay=iay<iby?iay:iby; maxay=iay>iby?iay:iby;
        ax=(double)iax; ay=(double)iay;
        bx=(double)ibx; by=(double)iby;
        vc=ids[off2+n2-1]-1;
        idx=xs[vc]; idy=ys[vc];
        for(j=0;j<n2;++j) {
            icx=idx; icy=idy;
            vc=ids[off2+j]-1;
            idx=xs[vc]; idy=ys[vc];
            if((icx>idx?icx:idx)<=minax ||
               (icx<idx?icx:idx)>=maxax ||
               (icy>idy?icy:idy)<=minay ||
               (icy<idy?icy:idy)>=maxay) continue;
            cx=(double)icx; cy=(double)icy;
            dx=(double)idx; dy=(double)idy;
            c1=overlap_cross_double(ax,ay,bx,by,cx,cy);
            c2=overlap_cross_double(ax,ay,bx,by,dx,dy);
            if(!((c1>0.0 && c2<0.0)||(c1<0.0 && c2>0.0))) continue;
            c3=overlap_cross_double(cx,cy,dx,dy,ax,ay);
            c4=overlap_cross_double(cx,cy,dx,dy,bx,by);
            if((c3>0.0 && c4<0.0)||(c3<0.0 && c4>0.0)) return 1;
        }
    }
    for(i=0;i<n1;++i) {
        va=ids[off1+i]-1;
        if(ovmax_inside(ids,xs,ys,off2,n2,
                        (double)xs[va],(double)ys[va])==1) return 1;
    }
    for(j=0;j<n2;++j) {
        vc=ids[off2+j]-1;
        if(ovmax_inside(ids,xs,ys,off1,n1,
                        (double)xs[vc],(double)ys[vc])==1) return 1;
    }
    return ovmax_probe(ids,xs,ys,off1,n1,off2,n2);
}

typedef signed long long ovf64;

/*
 * ----------------------------------------------------------------------------
 *  ovf_cross
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Calculer le déterminant de trois points entiers avec intermédiaires
 *    ovf64.
 *
 *  Paramètres :
 *    [entrée] ax/ay, bx/by, px/py.
 *
 *  Retour :
 *    Déterminant signé ; aucun état modifié.
 *
 *  Fonctionnement :
 *    Les conversions précèdent les différences pour éviter leur débordement
 *    en int.
 *
 * ----------------------------------------------------------------------------
 */
static ovf64 ovf_cross(int ax,int ay,int bx,int by,int px,int py)
{
    /* Élargir chaque coordonnée avant soustraction et multiplication. */
    return ((ovf64)bx-ax)*((ovf64)py-ay)
         - ((ovf64)by-ay)*((ovf64)px-ax);
}

/*
 * ----------------------------------------------------------------------------
 *  ovf_inside
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Classer un point rationnel par rapport à un contour projeté entier.
 *
 *  Paramètres :
 *    [entrée] ids base un, xs/ys, off/n ; nx/ny numérateurs et d
 *    dénominateur positif.
 *
 *  Retour :
 *    -1 bord, 1 intérieur, 0 extérieur.
 *
 *  Fonctionnement :
 *    Utilise des produits ovf64 sans division. L'appelant doit garantir que
 *    ces produits restent représentables.
 *
 * ----------------------------------------------------------------------------
 */
static int ovf_inside(const int *ids,const int *xs,const int *ys,
                      int off,int n,ovf64 nx,ovf64 ny,ovf64 d)
{
    int i,k,prev,inside;
    int ax,ay,bx,by;
    ovf64 c, lhs, rhs;
    inside=0;
    /* Parcourir le contour en comparant les coordonnées multipliées par le
     * dénominateur.
     */
    prev=ids[off+n-1]-1;
    bx=xs[prev]; by=ys[prev];
    for(i=0;i<n;++i) {
        k=ids[off+i]-1;
        ax=bx; ay=by;
        bx=xs[k]; by=ys[k];
        lhs=nx-(ovf64)ax*d;
        rhs=ny-(ovf64)ay*d;
        if ((nx>=(ovf64)ax*d && nx<=(ovf64)bx*d ||
             nx>=(ovf64)bx*d && nx<=(ovf64)ax*d) &&
            (ny>=(ovf64)ay*d && ny<=(ovf64)by*d ||
             ny>=(ovf64)by*d && ny<=(ovf64)ay*d)) {
            c=((ovf64)bx-ax)*rhs-((ovf64)by-ay)*lhs;
            if(c==0) return -1;
        }
        if (((ovf64)ay*d>ny)!=((ovf64)by*d>ny)) {

            c=((ovf64)bx-ax)*rhs-((ovf64)by-ay)*lhs;
            if (((by>ay) && c>0) || ((by<ay) && c<0)) inside=!inside;
        }
    }
    return inside;
}

/*
 * ----------------------------------------------------------------------------
 *  ovf_probe
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Chercher des sondes rationnelles communes près des milieux d'arêtes.
 *
 *  Paramètres :
 *    [entrée] ids base un, xs/ys, off1/n1 et off2/n2.
 *
 *  Retour :
 *    Booléen.
 *
 *  Fonctionnement :
 *    Encode les décalages normalisés avec d=2048*(|dx|+|dy|), puis appelle
 *    ovf_inside. Évite les divisions flottantes ; reste une stratégie de
 *    sondage fini.
 *
 * ----------------------------------------------------------------------------
 */
static int ovf_probe(const int *ids,const int *xs,const int *ys,
                     int off1,int n1,int off2,int n2)
{
    int pass,off,n,i,j,va,vb,side;
    int ax,ay,bx,by;
    ovf64 dx,dy,scale,d,nx,ny;
    /* Construire deux sondes rationnelles par arête non nulle et exiger leur
     * inclusion stricte dans les deux faces.
     */
    for(pass=0;pass<2;++pass) {
        off=pass?off2:off1;
        n=pass?n2:n1;
        for(i=0;i<n;++i) {
            j=(i+1==n)?0:i+1;
            va=ids[off+i]-1; vb=ids[off+j]-1;
            ax=xs[va]; ay=ys[va];
            bx=xs[vb]; by=ys[vb];
            dx=(ovf64)bx-ax; dy=(ovf64)by-ay;
            scale=(dx<0)?-(ovf64)dx:dx;
            scale+=(dy<0)?-(ovf64)dy:dy;
            if(scale==0) continue;

            d=2048*scale;
            for(side=-1;side<=1;side+=2) {
                nx=((ovf64)ax+bx)*1024*scale-side*(ovf64)dy*2;
                ny=((ovf64)ay+by)*1024*scale+side*(ovf64)dx*2;
                if(ovf_inside(ids,xs,ys,off1,n1,nx,ny,d)==1 &&
                   ovf_inside(ids,xs,ys,off2,n2,nx,ny,d)==1) return 1;
            }
        }
    }
    return 0;
}

/*
 * ----------------------------------------------------------------------------
 *  projected_polygons_overlap_v2_fixed64
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Tester le chevauchement projeté par Fixed64 avec déterminants et sondes
 *    rationnelles.
 *
 *  Paramètres :
 *    [entrée] model et f1/f2 valides
 *
 *  Retour :
 *    1 si un test trouve une zone commune, 0 sinon ou face trop courte.
 *
 *  Fonctionnement :
 *    Rejette d'abord les boîtes sans intérieur commun, puis essaye
 *    croisements propres, sommets strictement intérieurs et sondes près des
 *    milieux d'arêtes. Les seules touches de contours ne suffisent pas. Ne
 *    calcule pas l'aire d'intersection et ne modifie pas le modèle ; les
 *    variantes ne doivent pas être fusionnées sans comparaison de leurs
 *    limites numériques.
 *
 * ----------------------------------------------------------------------------
 */
static int projected_polygons_overlap_v2_fixed64(Model3D *model,int f1,int f2)
{
    FaceArrays3D *faces;
    VertexArrays3D *vertices;
    int *ids,*xs,*ys;
    int n1,n2,off1,off2,i,j,va,vc;
    int ax,ay,bx,by,cx,cy,dx,dy;
    int minax,maxax,minay,maxay;
    ovf64 c1,c2,c3,c4;
    /* Valider le modèle et les longueurs avant les filtres de boîtes. */
    if(!model) return 0;
    faces=&model->faces;
    n1=faces->vertex_count[f1]; n2=faces->vertex_count[f2];
    if(n1<3 || n2<3) return 0;
    if(faces->maxx[f1]<=faces->minx[f2] ||
       faces->maxx[f2]<=faces->minx[f1] ||
       faces->maxy[f1]<=faces->miny[f2] ||
       faces->maxy[f2]<=faces->miny[f1]) return 0;
    vertices=&model->vertices;
    ids=faces->vertex_indices_buffer;
    xs=vertices->x2d; ys=vertices->y2d;
    off1=faces->vertex_indices_ptr[f1];
    off2=faces->vertex_indices_ptr[f2];
    va=ids[off1+n1-1]-1;
    bx=xs[va]; by=ys[va];
    /* Chercher les croisements propres entre les contours avant les
     * inclusions et les sondes.
     */
    for(i=0;i<n1;++i) {
        ax=bx; ay=by;
        va=ids[off1+i]-1;
        bx=xs[va]; by=ys[va];
        minax=ax<bx?ax:bx; maxax=ax>bx?ax:bx;
        minay=ay<by?ay:by; maxay=ay>by?ay:by;
        vc=ids[off2+n2-1]-1;
        dx=xs[vc]; dy=ys[vc];
        for(j=0;j<n2;++j) {
            cx=dx; cy=dy;
            vc=ids[off2+j]-1;
            dx=xs[vc]; dy=ys[vc];
            if((cx>dx?cx:dx)<=minax ||
               (cx<dx?cx:dx)>=maxax ||
               (cy>dy?cy:dy)<=minay ||
               (cy<dy?cy:dy)>=maxay) continue;
            c1=ovf_cross(ax,ay,bx,by,cx,cy);
            c2=ovf_cross(ax,ay,bx,by,dx,dy);
            if(!((c1>0 && c2<0)||(c1<0 && c2>0))) continue;
            c3=ovf_cross(cx,cy,dx,dy,ax,ay);
            c4=ovf_cross(cx,cy,dx,dy,bx,by);
            if((c3>0 && c4<0)||(c3<0 && c4>0)) return 1;
        }
    }
    for(i=0;i<n1;++i) {
        va=ids[off1+i]-1;
        if(ovf_inside(ids,xs,ys,off2,n2,xs[va],ys[va],1)==1) return 1;
    }
    for(j=0;j<n2;++j) {
        vc=ids[off2+j]-1;
        if(ovf_inside(ids,xs,ys,off1,n1,xs[vc],ys[vc],1)==1) return 1;
    }
    return ovf_probe(ids,xs,ys,off1,n1,off2,n2);
}

#define OVERLAP_SIMPLE      0
#define OVERLAP_V1          1
#define OVERLAP_V2          2
#define OVERLAP_V2_MAX      3
#define OVERLAP_FIXED64     4
#define OVERLAP_RASTER      5
#define OVERLAP_RASTER_X10  6

static int overlap_method = OVERLAP_V1;

/*
 * ----------------------------------------------------------------------------
 *  projected_polygons_overlap
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Choisir la méthode de chevauchement configurée à l'exécution.
 *
 *  Paramètres :
 *    [entrée] model, f1/f2 ; overlap_method global.
 *
 *  Retour :
 *    Booléen de la méthode.
 *
 *  Fonctionnement :
 *    Sept choix : Simple, V1, V2 original, V2 Max, Fixed64, Raster, Raster
 *    x10. Une valeur inconnue prend V1. Les garanties diffèrent, notamment
 *    géométrie continue, seuils heuristiques et échantillonnage de centres
 *    de pixels.
 *
 * ----------------------------------------------------------------------------
 */
static int projected_polygons_overlap(Model3D *model, int f1, int f2)
{
    /* Répartir les méthodes sans convertir leurs résultats en une aire commune. */
    switch (overlap_method) {
    case OVERLAP_SIMPLE:
        return projected_polygons_overlap_simple(model, f1, f2);
    case OVERLAP_V1:
        return projected_polygons_overlap_v1(model, f1, f2);
    case OVERLAP_V2:
        return projected_polygons_overlap_v2_original(model, f1, f2);
    case OVERLAP_V2_MAX:
        return projected_polygons_overlap_v2_max(model, f1, f2);
    case OVERLAP_FIXED64:
        return projected_polygons_overlap_v2_fixed64(model, f1, f2);
    case OVERLAP_RASTER:
        return projected_polygons_overlap_raster(model, f1, f2);
    case OVERLAP_RASTER_X10:
        return projected_polygons_overlap_raster_x10(model, f1, f2);
    }
    return projected_polygons_overlap_v1(model, f1, f2);
}

/*
 * ----------------------------------------------------------------------------
 *  projected_polygons_overlap_cached
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Mémoriser uniquement les chevauchements positifs pour la méthode de classement.
 *
 *  Paramètres :
 *    [entrée] model, f1/f2 orientés
 *
 *  Retour :
 *    Booléen.
 *
 *  Sorties et effets de bord :
 *    cache et compteurs optionnels.
 *
 *  Fonctionnement :
 *    Clé : vue, paire, projection et méthode runtime. Un résultat nul peut
 *    être un échec de ressource ou de garde, donc n'est pas conservé.
 *
 * ----------------------------------------------------------------------------
 */
static int projected_polygons_overlap_cached(Model3D *model, int f1, int f2)
{
    OrderPairEntry *entry;
    int result;
    if (!model) return 0;
    ORDER_COUNT(overlap_calls);
    /* Chercher une entrée exacte pour le contexte courant. */
    entry=ordering_pair_slot(model, order_overlap_pairs, f1, f2);
    if (ordering_pair_hit(entry, f1, f2, s_global_proj_scale_fixed, (unsigned int)overlap_method)) {
        ORDER_COUNT(overlap_hits);
        return entry->result;
    }
    /* Calculer puis mémoriser seulement les résultats positifs. */
    result=projected_polygons_overlap(model,f1,f2);
    if (result) ordering_pair_store(entry,f1,f2,s_global_proj_scale_fixed,
                                   (unsigned int)overlap_method,result);
    return result;
}

/*
 * ----------------------------------------------------------------------------
 *  select_overlap_method
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Demander un choix runtime de méthode sans modifier directement l'état
 *    global.
 *
 *  Paramètres :
 *    [entrée] current_method, ramené à V1 si invalide.
 *
 *  Retour :
 *    Méthode 0..6 ; entrée vide, zéro ou EOF conserve le choix.
 *
 *  Sorties et effets de bord :
 *    terminal uniquement.
 *
 *  Fonctionnement :
 *    Vérifie les caractères résiduels après strtol et redemande après
 *    erreur.
 *
 * ----------------------------------------------------------------------------
 */
static int select_overlap_method(int current_method)
{
    static const char *names[7] = {
        "Simple", "V1", "V2 original", "V2 Max",
        "Fixed64", "Raster", "Raster x10"
    };
    char user_choice[40];
    char *end;
    long choice;
    int i;

    if (current_method < OVERLAP_SIMPLE ||
        current_method > OVERLAP_RASTER_X10)
        current_method = OVERLAP_V1;

    /* Présenter le menu jusqu'à une saisie valide ou une annulation. */
    for (;;) {
        printf("\nOVERLAP DETECTION METHOD\n\n");

        for (i = 0; i < 7; ++i)
            printf("%d. %-12s %s\n", i + 1, names[i],
                   (i == current_method) ? "<-- current" : "");

        printf("0. Cancel\n\nSelect (0-7, Return=cancel): ");
        fflush(stdout);

        if (fgets(user_choice, sizeof(user_choice), stdin) == NULL)
            return current_method;

        if (user_choice[0] == '\n' ||
            user_choice[0] == '\r' ||
            user_choice[0] == '\0')
            return current_method;

        /* Valider la conversion entière et le reste de la ligne avant
         * d'accepter le numéro.
         */
        choice = strtol(user_choice, &end, 10);

        if (end == user_choice) {
            printf("Invalid selection.\n");
            continue;
        }

        while (*end == ' ' || *end == '\t' || *end == '\r')
            ++end;

        if (*end != '\n' && *end != '\0') {
            printf("Invalid selection.\n");
            continue;
        }

        if (choice == 0)
            return current_method;

        if (choice >= 1 && choice <= 7)
            return (int)(choice - 1);

        printf("Invalid selection.\n");
    }
}
