#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
typedef int32_t Fixed32;typedef int64_t Fixed64;typedef void **Handle;
#define MAX_FACE_VERTICES 16
#define MAX_VERTICES 6000
#define MAX_FACES 6000
#define MAX_LINE_LENGTH 1024
#define FACE_PLANE_RAW_FIXED 0
#define FACE_PLANE_OBJECT_UNITS 1
#define CENTRE_X 160
#define CENTRE_Y 100
#define FIXED_SHIFT     16
#define FIXED_SCALE     (1L << FIXED_SHIFT)
#define FIXED_HALF      (FIXED_SCALE >> 1)
#define FIXED_ONE       FIXED_SCALE
#define INT_TO_FIXED(x)     ((Fixed32)(x) << FIXED_SHIFT)
#define FIXED_TO_INT(x)     ((int)((x) >> FIXED_SHIFT))
#define FLOAT_TO_FIXED(x)   ((Fixed32)((x) * FIXED_SCALE))
#define FIXED_TO_FLOAT(x)   ((float)(x) / (float)FIXED_SCALE)
#define FIXED_ADD(a, b)     ((a) + (b))
#define FIXED_SUB(a, b)     ((a) - (b))
#define FIXED_NEG(x)        (-(x))
#define FIXED_MUL_64(a, b)  ((Fixed32)(((Fixed64)(a) * (Fixed64)(b)) >> FIXED_SHIFT))
#define FIXED_DIV_64(a, b)  ((Fixed32)(((Fixed64)(a) << FIXED_SHIFT) / (Fixed64)(b)))
typedef struct {
    /* Tableaux parallèles : x/y/z objet, xo/yo/zo observateur en 16.16,
     * x2d/y2d en pixels ; handles et pointeurs associés.
     */
    Handle xHandle, yHandle, zHandle;
    Handle xoHandle, yoHandle, zoHandle;
    Handle x2dHandle, y2dHandle;
    Fixed32 *x, *y, *z;
    Fixed32 *xo, *yo, *zo;
    int *x2d, *y2d;
    int vertex_count;
    int saved_vertex_count;
} VertexArrays3D;
typedef struct {
    Handle vertex_countHandle;
    Handle vertex_indicesBufferHandle;
    Handle vertex_indicesPtrHandle;
    Handle z_maxHandle;
    Handle display_flagHandle;
    Handle sorted_face_indicesHandle;

    /* Faces à base zéro ; les valeurs du buffer d'indices restent à base un
     * comme dans OBJ. vertex_indices_ptr contient des offsets, pas des
     * pointeurs far.
     */
    int *vertex_count;
    int *saved_vertex_count;
    int *vertex_indices_buffer;
    int *vertex_indices_ptr;
    /* Métriques observateur et plans de Newell recalculés pour chaque vue ;
     * sorted_face_indices est une permutation de faces.
     */
    Fixed32 *z_min;
    Fixed32 *z_max;
    Fixed32 *z_mean;
    Fixed64 *plane_a, *plane_b, *plane_c, *plane_d;

    int *minx, *maxx, *miny, *maxy;

    /* Bornes dans le repère objet, distinctes des bornes projetées
     * minx/maxx/miny/maxy en pixels.
     */
    Fixed32 *minx3, *maxx3, *miny3, *maxy3, *minz3, *maxz3;

    int *display_flag;
    int *sorted_face_indices;
    int face_count;
    int total_indices;
} FaceArrays3D;
typedef struct {
    int vertex_count;
    /* Structure de face séparée des tableaux parallèles ; ne pas confondre
     * ce format avec FaceArrays3D.
     */
    int vertex_indices[MAX_FACE_VERTICES];
    Fixed32 z_max;
    int display_flag;
} Face3D;
typedef struct {
    /* Angles entiers en degrés ; distance de l'observateur en 16.16. */
    int angle_h;
    int angle_v;
    int angle_w;
    Fixed32 distance;
} ObserverParams;
typedef struct {
    VertexArrays3D vertices;
    FaceArrays3D   faces;

    /* Métadonnées du centrage et de l'ajustement de vue ; les drapeaux
     * indiquent les étapes déjà disponibles ou appliquées.
     */
    Fixed32 auto_scale;
    Fixed32 auto_center_x, auto_center_y, auto_center_z;
    int auto_scaled;
    int auto_centered;

    Fixed32 auto_suggested_distance;
    Fixed32 auto_suggested_proj_scale;
    Fixed32 auto_proj_scale;
    int auto_fit_ready;
    int auto_fit_applied;

    /* Copies des coordonnées d'origine et buffer temporaire de lecture,
     * possédés par le modèle.
     */
    Fixed32 *orig_x, *orig_y, *orig_z;

    float *coord_buf;
    int coord_buf_capacity;
} Model3D;
static const Fixed32 sin_table[360] = {
    0, 1144, 2287, 3430, 4572, 5712, 6850, 7987, 9121, 10252,
    11380, 12505, 13626, 14742, 15855, 16962, 18064, 19161, 20252, 21336,
    22415, 23486, 24550, 25607, 26656, 27697, 28729, 29753, 30767, 31772,
    32768, 33754, 34729, 35693, 36647, 37590, 38521, 39441, 40348, 41243,
    42126, 42995, 43852, 44695, 45525, 46341, 47143, 47930, 48703, 49461,
    50203, 50931, 51643, 52339, 53020, 53684, 54332, 54963, 55578, 56175,
    56756, 57319, 57865, 58393, 58903, 59396, 59870, 60326, 60764, 61183,
    61584, 61966, 62328, 62672, 62997, 63303, 63589, 63856, 64104, 64332,
    64540, 64729, 64898, 65048, 65177, 65287, 65376, 65446, 65496, 65526,
    65536, 65526, 65496, 65446, 65376, 65287, 65177, 65048, 64898, 64729,
    64540, 64332, 64104, 63856, 63589, 63303, 62997, 62672, 62328, 61966,
    61584, 61183, 60764, 60326, 59870, 59396, 58903, 58393, 57865, 57319,
    56756, 56175, 55578, 54963, 54332, 53684, 53020, 52339, 51643, 50931,
    50203, 49461, 48703, 47930, 47143, 46341, 45525, 44695, 43852, 42995,
    42126, 41243, 40348, 39441, 38521, 37590, 36647, 35693, 34729, 33754,
    32768, 31772, 30767, 29753, 28729, 27697, 26656, 25607, 24550, 23486,
    22415, 21336, 20252, 19161, 18064, 16962, 15855, 14742, 13626, 12505,
    11380, 10252, 9121, 7987, 6850, 5712, 4572, 3430, 2287, 1144,
    0, -1144, -2287, -3430, -4572, -5712, -6850, -7987, -9121, -10252,
    -11380, -12505, -13626, -14742, -15855, -16962, -18064, -19161, -20252, -21336,
    -22415, -23486, -24550, -25607, -26656, -27697, -28729, -29753, -30767, -31772,
    -32768, -33754, -34729, -35693, -36647, -37590, -38521, -39441, -40348, -41243,
    -42126, -42995, -43852, -44695, -45525, -46341, -47143, -47930, -48703, -49461,
    -50203, -50931, -51643, -52339, -53020, -53684, -54332, -54963, -55578, -56175,
    -56756, -57319, -57865, -58393, -58903, -59396, -59870, -60326, -60764, -61183,
    -61584, -61966, -62328, -62672, -62997, -63303, -63589, -63856, -64104, -64332,
    -64540, -64729, -64898, -65048, -65177, -65287, -65376, -65446, -65496, -65526,
    -65536, -65526, -65496, -65446, -65376, -65287, -65177, -65048, -64898, -64729,
    -64540, -64332, -64104, -63856, -63589, -63303, -62997, -62672, -62328, -61966,
    -61584, -61183, -60764, -60326, -59870, -59396, -58903, -58393, -57865, -57319,
    -56756, -56175, -55578, -54963, -54332, -53684, -53020, -52339, -51643, -50931,
    -50203, -49461, -48703, -47930, -47143, -46341, -45525, -44695, -43852, -42995,
    -42126, -41243, -40348, -39441, -38521, -37590, -36647, -35693, -34729, -33754,
    -32768, -31772, -30767, -29753, -28729, -27697, -26656, -25607, -24550, -23486,
    -22415, -21336, -20252, -19161, -18064, -16962, -15855, -14742, -13626, -12505,
    -11380, -10252, -9121, -7987, -6850, -5712, -4572, -3430, -2287, -1144
};
static const Fixed32 cos_table[360] = {
    65536, 65526, 65496, 65446, 65376, 65287, 65177, 65048, 64898, 64729,
    64540, 64332, 64104, 63856, 63589, 63303, 62997, 62672, 62328, 61966,
    61584, 61183, 60764, 60326, 59870, 59396, 58903, 58393, 57865, 57319,
    56756, 56175, 55578, 54963, 54332, 53684, 53020, 52339, 51643, 50931,
    50203, 49461, 48703, 47930, 47143, 46341, 45525, 44695, 43852, 42995,
    42126, 41243, 40348, 39441, 38521, 37590, 36647, 35693, 34729, 33754,
    32768, 31772, 30767, 29753, 28729, 27697, 26656, 25607, 24550, 23486,
    22415, 21336, 20252, 19161, 18064, 16962, 15855, 14742, 13626, 12505,
    11380, 10252, 9121, 7987, 6850, 5712, 4572, 3430, 2287, 1144,
    0, -1144, -2287, -3430, -4572, -5712, -6850, -7987, -9121, -10252,
    -11380, -12505, -13626, -14742, -15855, -16962, -18064, -19161, -20252, -21336,
    -22415, -23486, -24550, -25607, -26656, -27697, -28729, -29753, -30767, -31772,
    -32768, -33754, -34729, -35693, -36647, -37590, -38521, -39441, -40348, -41243,
    -42126, -42995, -43852, -44695, -45525, -46341, -47143, -47930, -48703, -49461,
    -50203, -50931, -51643, -52339, -53020, -53684, -54332, -54963, -55578, -56175,
    -56756, -57319, -57865, -58393, -58903, -59396, -59870, -60326, -60764, -61183,
    -61584, -61966, -62328, -62672, -62997, -63303, -63589, -63856, -64104, -64332,
    -64540, -64729, -64898, -65048, -65177, -65287, -65376, -65446, -65496, -65526,
    -65536, -65526, -65496, -65446, -65376, -65287, -65177, -65048, -64898, -64729,
    -64540, -64332, -64104, -63856, -63589, -63303, -62997, -62672, -62328, -61966,
    -61584, -61183, -60764, -60326, -59870, -59396, -58903, -58393, -57865, -57319,
    -56756, -56175, -55578, -54963, -54332, -53684, -53020, -52339, -51643, -50931,
    -50203, -49461, -48703, -47930, -47143, -46341, -45525, -44695, -43852, -42995,
    -42126, -41243, -40348, -39441, -38521, -37590, -36647, -35693, -34729, -33754,
    -32768, -31772, -30767, -29753, -28729, -27697, -26656, -25607, -24550, -23486,
    -22415, -21336, -20252, -19161, -18064, -16962, -15855, -14742, -13626, -12505,
    -11380, -10252, -9121, -7987, -6850, -5712, -4572, -3430, -2287, -1144,
    0, 1144, 2287, 3430, 4572, 5712, 6850, 7987, 9121, 10252,
    11380, 12505, 13626, 14742, 15855, 16962, 18064, 19161, 20252, 21336,
    22415, 23486, 24550, 25607, 26656, 27697, 28729, 29753, 30767, 31772,
    32768, 33754, 34729, 35693, 36647, 37590, 38521, 39441, 40348, 41243,
    42126, 42995, 43852, 44695, 45525, 46341, 47143, 47930, 48703, 49461,
    50203, 50931, 51643, 52339, 53020, 53684, 54332, 54963, 55578, 56175,
    56756, 57319, 57865, 58393, 58903, 59396, 59870, 60326, 60764, 61183,
    61584, 61966, 62328, 62672, 62997, 63303, 63589, 63856, 64104, 64332,
    64540, 64729, 64898, 65048, 65177, 65287, 65376, 65446, 65496, 65526
};
static int readVertices_last_count,cull_back_faces=0,shaded_by_orientation=0,framePolyOnly=1,painter_mode;static int face_shade_color[100];static Fixed32 s_global_proj_scale_fixed,current_observer_distance=150*65536;
#define COL_FILL_DEFAULT 14
static void keypress(void){}
static void fractional_projection_cache_release(Model3D*m){}
static void fractional_projection_cache_invalidate(Model3D*m);

static Fixed32 fixed_mul32_fast(Fixed32 a,Fixed32 b){return (Fixed32)((int64_t)a*b>>16);}
static Fixed64 fixed_product32_fast(Fixed32 a,Fixed32 b){return (int64_t)a*b;}
static Fixed64 fixed_product64x32_fast(Fixed64 a,Fixed32 b){return a*b;}
static void computeOrientationShading(Model3D*m){}
static inline int FIXED_ROUND_TO_INT(Fixed32 x);
static inline Fixed32 sin_deg_int(int deg);
static inline Fixed32 cos_deg_int(int deg);
static inline int normalize_deg(int deg);
Model3D* createModel3D(void);
void destroyModel3D(Model3D* model);
static void compute_auto_fit(VertexArrays3D *vtx, int vertex_count, Fixed32 max_dim, Model3D *owner);
int readVertices(const char* filename, VertexArrays3D* vtx, int max_vertices, Model3D* owner);
int readFaces_model(const char* filename, Model3D* model);
static void check_intersect(Model3D* model);
static int faces_share_vertex_or_edge(FaceArrays3D* faces, int i, int j);
static void compute_face_plane(Model3D* model, int fidx, double *a, double *b, double *c, double *d, int coordinate_mode);
static int strict_inside(int px, int py, int nfj_local, int *vx_local, int *vy_local);
static int evaluate_face_crossings(Model3D* model, int ei, int fj, int *interior);
static int split_face_by_plane(Model3D* model, int f1, int f2, int *out_newverts, int *out_n, int *constraint_verts, int constraint_n, int force_one_edge);
static int add_vertex(Model3D* model, Fixed32 x, Fixed32 y, Fixed32 z);
static void cleanup_list(int *arr, int *n);
static void remove_duplicates(int *arr, int *n);
static double poly_signed_area(Model3D* model, int *list, int n, double a, double b, double c);
static void reorder_poly(Model3D* model, int fidx, int *list, int *n);
static void compute_face_bboxes3D(Model3D* model);
void calculateFaceDepths(Model3D* model, Face3D* faces, int face_count);
void normalizeAutoFitDistanceTo150(Model3D* model);
static int point_in_poly_arrays_int(int px, int py, int n, const int vx[], const int vy[]);
#define FIXED64_TO_FLOAT(x) ((double)(x)/65536.0)
static int overlap_method=1;
#define MIN_INTERSECTION_AREA_PIXELS 2.0

#define GEOV3_DISABLE_ASM_SCAN 1
#define GEOV3_DISABLE_ASM_MUL 1
#define GEOV3_DISABLE_ASM_RELATION 1
static int debug_overlap_subj = -1;
static int debug_overlap_clip = -1;
#define DEBUG_CLIP_MAX 512
static long long debug_clip_raw_area2 = 0;
static int debug_clip_centroid_x = 0;
static int debug_clip_centroid_y = 0;
static double debug_clip_fixed_area = 0.0;
static int debug_clip_fixed_vcount = 0;
static int debug_clip_fixed_vx[DEBUG_CLIP_MAX];
static int debug_clip_fixed_vy[DEBUG_CLIP_MAX];
static unsigned long overlapCheckCount = 0;
static unsigned long overlapSegiAccept = 0;
static unsigned long overlapSampleAccept = 0;
static unsigned long overlapClipCalls = 0;
static unsigned long overlapClipAccept = 0;
enum{FACE_ORDER_BEFORE=-1,FACE_ORDER_UNDETERMINED=0,FACE_ORDER_AFTER=1,RAY_FIRST_NEARER=-1,RAY_ORDER_UNDETERMINED=0,RAY_FIRST_FARTHER=1};
#define GEOV3_EXCHANGE_PLANES 1
#define GEOV3_EXCHANGE_RAY 2
typedef int (*GeoV3ExchangeCallback)(Model3D*,int,int,int,int,int,void*);
static Fixed64 GeoV3_MulFixed64Fixed32Shift(Fixed64 a,Fixed32 b){return (a*b)>>16;}
static long GetTick(void){return 0;}
static int qd_calls=0;
/*
 * ============================================================================
 *  ordering_cache.c — Caches des calculs d’ordre géométrique
 * ============================================================================
 *
 *  Rôle :
 *    Regrouper les résultats réutilisables pendant le traitement d'une vue :
 *    relations entre faces, plans préparés, côtés des sommets et contours
 *    projetés. Ces caches évitent de répéter les calculs géométriques
 *    coûteux.
 *
 *  Organisation :
 *    Les entrées sont associées à un modèle et à une génération de données.
 *    Les clés complètes sont vérifiées avant réutilisation ; une collision
 *    peut remplacer une entrée, mais ne doit pas produire un faux résultat.
 *    Les allocations sont conservées entre invalidations.
 *
 *  Dépendances et contraintes :
 *    Inclus par engine.c après declare.c. Les changements de transformation,
 *    d'époque, de tolérance ou d'échelle invalident les entrées concernées.
 *    Les buffers et paquets assembleur sont partagés : leur utilisation est
 *    synchrone et non réentrante.
 *
 * ============================================================================
 */
#define ORDER_PAIR_SLOTS 256
#define ORDER_PLANE_BLOCK_SIZE 64
#define ORDER_PLANE_BLOCKS ((MAX_FACES + ORDER_PLANE_BLOCK_SIZE - 1) / ORDER_PLANE_BLOCK_SIZE)
/* Clé orientée : inverser f1 et f2 produit une autre requête, notamment pour
 * sujet et clip.
 */
typedef struct {
    int f1, f2;
    Fixed32 scale;
    unsigned int generation, context;
    int result;
} OrderPairEntry;
typedef struct {
    double plane[ORDER_PLANE_BLOCK_SIZE][4];
    unsigned int stamp[ORDER_PLANE_BLOCK_SIZE];
} OrderPlaneBlock;


#define ORDER_FIXED_PLANE_SLOTS 64
#define ORDER_WINDING_SLOTS 128
#define ORDER_VERTEX_SIDE_SLOTS 64
typedef struct {
    Fixed64 a,b,c,d,epsilon;
    Fixed32 a32,b32,c32;
    int face,observer,asm_ready;
    unsigned int generation,epoch;


    /* Côté exact par sommet : identifiant OBJ sur 13 bits et code sur deux
     * bits ; 0 vide, $2000 sur le plan, $4000 positif, $6000 négatif.
     */
    unsigned int vertex_sides[ORDER_VERTEX_SIDE_SLOTS];
} OrderFixedPlane;
typedef struct {
    double area2;
    Fixed32 scale;
    int face;
    unsigned int generation;
} OrderWinding;


#define ORDER_CONTOUR_SLOTS 64
#define ORDER_CONTOUR_MAX_VERTICES 16
typedef struct {
    /* Buffer de doubles : x puis y du contour antihoraire, suivis de dx puis
     * dy ; chaque groupe a capacity éléments.
     */
    double *data;
    Fixed32 scale;
    int face,count,capacity;
    unsigned int generation,failed_generation;
} OrderContour;


/* Codes denses sur deux bits : 0 inconnu, 1 avant, 2 indéterminé, 3 après ;
 * au plus 16 Kio pour 256 faces.
 */
#define ORDER_GEOMETRY_DENSE_LIMIT 256
static unsigned char *order_geometry_pairs=NULL;
static unsigned char *order_geometry_rows[ORDER_GEOMETRY_DENSE_LIMIT];
static unsigned int order_geometry_capacity=0,order_geometry_stride=0;
static unsigned int order_geometry_generation=0,order_geometry_epoch=0;
static unsigned int order_geometry_failed=0;
static int order_geometry_faces=0;
static Fixed64 order_geometry_epsilon=0;
static OrderContour order_contours[ORDER_CONTOUR_SLOTS];
static OrderFixedPlane order_fixed_planes[ORDER_FIXED_PLANE_SLOTS];
static OrderFixedPlane *order_asm_packet_plane=NULL;
static unsigned int order_fixed_plane_epoch=0;
static OrderWinding order_windings[ORDER_WINDING_SLOTS];
static OrderPairEntry order_ray_pairs[ORDER_PAIR_SLOTS];
static OrderPairEntry order_overlap_pairs[ORDER_PAIR_SLOTS];
static OrderPlaneBlock *order_planes[ORDER_PLANE_BLOCKS];
static unsigned int order_plane_failed[ORDER_PLANE_BLOCKS];
static Model3D *order_cache_owner = NULL;
static unsigned int order_cache_generation = 0;
static double *order_sh_workspace = NULL;
static long order_sh_capacity = 0;


/* Compteurs optionnels de réutilisation ; sans GEO_ORDER_PROFILE,
 * ORDER_COUNT est sans effet.
 */
#if defined(GEO_ORDER_PROFILE)
static struct {
    unsigned long ray_calls, ray_hits, overlap_calls, overlap_hits;
    unsigned long sh_calls, sh_allocations, plane_conversions;
} order_profile;
#define ORDER_COUNT(field) (++order_profile.field)
#else
#define ORDER_COUNT(field) ((void)0)
#endif


/*
 * ----------------------------------------------------------------------------
 *  ordering_cache_invalidate
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Changer la génération des caches et désigner leur modèle propriétaire.
 *
 *  Paramètres :
 *    [entrée] model, propriétaire de la nouvelle vue.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    propriétaire, génération et paquet assembleur invalidé.
 *
 *  Fonctionnement :
 *    Les allocations sont conservées. Lorsque le compteur revient à zéro,
 *    effacer les stamps avant de repartir à 1 pour ne pas réutiliser une
 *    ancienne vue.
 *
 * ----------------------------------------------------------------------------
 */
static void ordering_cache_invalidate(Model3D *model)
{
    int i;
    /* Changer de vue sans libérer les allocations ; invalider le paquet
     * assembleur.
     */
    order_cache_owner = model;
    order_asm_packet_plane=NULL;
    ++order_cache_generation;
    /* Éliminer tous les anciens stamps avant de réutiliser la génération 1. */
    if (order_cache_generation == 0) {
        order_geometry_generation=0;order_geometry_failed=0;
        for(i=0;i<ORDER_CONTOUR_SLOTS;++i){
            order_contours[i].generation=0;order_contours[i].failed_generation=0;
        }
        memset(order_fixed_planes,0,sizeof(order_fixed_planes));
        memset(order_windings,0,sizeof(order_windings));
        memset(order_ray_pairs, 0, sizeof(order_ray_pairs));
        memset(order_overlap_pairs, 0, sizeof(order_overlap_pairs));
        memset(order_plane_failed, 0, sizeof(order_plane_failed));
        for (i = 0; i < ORDER_PLANE_BLOCKS; ++i)
            if (order_planes[i]) memset(order_planes[i]->stamp, 0, sizeof(order_planes[i]->stamp));
        order_cache_generation = 1;
    }
}


/*
 * ----------------------------------------------------------------------------
 *  ordering_cache_release
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Libérer les caches appartenant au modèle détruit.
 *
 *  Paramètres :
 *    [entrée] model.
 *
 *  Retour :
 *    Aucun ; aucun effet si ce modèle n'est pas propriétaire.
 *
 *  Sorties et effets de bord :
 *    buffers libérés, tables et métadonnées remises à zéro.
 *
 *  Fonctionnement :
 *    Les pointeurs retournés auparavant par les fonctions de cache
 *    deviennent invalides.
 *
 * ----------------------------------------------------------------------------
 */
static void ordering_cache_release(Model3D *model)
{
    int i;
    /* Libérer uniquement les ressources du propriétaire pour éviter de
     * toucher à une autre vue.
     */
    if (order_cache_owner != model) return;
    order_asm_packet_plane=NULL;
    free(order_geometry_pairs);order_geometry_pairs=NULL;
    order_geometry_capacity=order_geometry_stride=order_geometry_generation=0;
    order_geometry_epoch=order_geometry_failed=0;order_geometry_faces=0;
    /* Libérer contours, blocs de plans et espace de clipping avant de
     * réinitialiser les métadonnées.
     */
    for(i=0;i<ORDER_CONTOUR_SLOTS;++i)free(order_contours[i].data);
    memset(order_contours,0,sizeof(order_contours));
    for (i = 0; i < ORDER_PLANE_BLOCKS; ++i) {
        free(order_planes[i]);order_planes[i] = NULL;
    }
    free(order_sh_workspace);order_sh_workspace = NULL;order_sh_capacity = 0;
    memset(order_ray_pairs, 0, sizeof(order_ray_pairs));
    memset(order_overlap_pairs, 0, sizeof(order_overlap_pairs));
    memset(order_plane_failed, 0, sizeof(order_plane_failed));
    memset(order_fixed_planes,0,sizeof(order_fixed_planes));
    memset(order_windings,0,sizeof(order_windings));
    order_cache_owner = NULL;order_cache_generation = 0;
}


/*
 * ----------------------------------------------------------------------------
 *  ordering_pair_slot
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Obtenir le slot de hash d'une paire orientée.
 *
 *  Paramètres :
 *    [entrée] model, propriétaire attendu ; table, cache de ORDER_PAIR_SLOTS
 *    entrées ; f1 et f2, identifiants à base zéro.
 *
 *  Retour :
 *    Adresse du slot, sans garantir un hit.
 *
 *  Sorties et effets de bord :
 *    invalide les caches si le modèle change.
 *
 *  Fonctionnement :
 *    Les collisions ne sont pas chaînées ici : ordering_pair_hit doit
 *    ensuite vérifier la clé entière.
 *
 * ----------------------------------------------------------------------------
 */
static OrderPairEntry *ordering_pair_slot(Model3D *model, OrderPairEntry *table,
                                         int f1, int f2)
{
    unsigned int slot;
    if (order_cache_owner != model) ordering_cache_invalidate(model);
    /* Calculer un hash orienté ; le slot seul ne valide jamais un résultat. */
    slot = (((unsigned int)f1 * 33U) ^ (unsigned int)f2) & (ORDER_PAIR_SLOTS - 1);
    return &table[slot];
}


/*
 * ----------------------------------------------------------------------------
 *  ordering_pair_hit
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Vérifier toutes les composantes de la clé d'une paire mémorisée.
 *
 *  Paramètres :
 *    [entrée] entry ; f1/f2 orientés ; scale, projection ; context, contexte
 *    du test.
 *
 *  Retour :
 *    Vrai si génération et clé correspondent, faux sinon.
 *
 *  Fonctionnement :
 *    Aucun effet de bord.
 *
 * ----------------------------------------------------------------------------
 */
static int ordering_pair_hit(OrderPairEntry *entry, int f1, int f2,
                              Fixed32 scale, unsigned int context)
{
    /* Contrôler ensemble la vue, la paire, l'échelle et le contexte. */
    return entry->generation == order_cache_generation &&
           entry->f1 == f1 && entry->f2 == f2 && entry->scale == scale &&
           entry->context == context;
}


/*
 * ----------------------------------------------------------------------------
 *  ordering_pair_store
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Remplacer un slot de paire par le résultat de la vue courante.
 *
 *  Paramètres :
 *    [entrée] f1/f2, scale, context et result.
 *
 *    [entrée-sortie] entry.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    La génération est publiée avec la clé complète ; une collision évince
 *    l'ancien résultat sans produire de faux hit.
 *
 * ----------------------------------------------------------------------------
 */
static void ordering_pair_store(OrderPairEntry *entry, int f1, int f2,
                                Fixed32 scale, unsigned int context, int result)
{
    /* Écrire le résultat avec sa clé complète et sa génération. */
    entry->f1=f1;entry->f2=f2;entry->scale=scale;entry->context=context;
    entry->result=result;entry->generation=order_cache_generation;
}


/*
 * ----------------------------------------------------------------------------
 *  ordering_workspace
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Obtenir un espace de travail partagé pour le clipping.
 *
 *  Paramètres :
 *    [entrée] bytes, taille demandée en octets.
 *
 *  Retour :
 *    Buffer existant ou agrandi, NULL si l'agrandissement est impossible.
 *
 *  Sorties et effets de bord :
 *    remplace l'ancien buffer uniquement après une allocation réussie ; le
 *    contenu précédent n'est pas conservé lors d'un agrandissement.
 *
 *  Fonctionnement :
 *    Ne pas libérer le pointeur retourné.
 *
 * ----------------------------------------------------------------------------
 */
static double *ordering_workspace(long bytes)
{
    /* Agrandir à la demande sans perdre l'ancien buffer si malloc échoue. */
    if (bytes > order_sh_capacity) {
        double *grown;
        if (bytes <= 0 || (unsigned long)bytes > (unsigned long)((size_t)-1)) return NULL;
        grown=(double*)malloc((size_t)bytes);
        if (!grown) return NULL;
        free(order_sh_workspace);order_sh_workspace=grown;order_sh_capacity=bytes;
        ORDER_COUNT(sh_allocations);
    }
    return order_sh_workspace;
}


/*
 * ----------------------------------------------------------------------------
 *  ordering_plane
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Convertir les quatre coefficients d'un plan fixe en double, avec cache.
 *
 *  Paramètres :
 *    [entrée] model ; face, identifiant valide ; fallback, tableau de quatre
 *    doubles fourni par l'appelant.
 *
 *  Retour :
 *    Coefficients en cache, ou fallback renseigné.
 *
 *  Sorties et effets de bord :
 *    allocation différée par blocs de 64 faces ; un échec d'allocation n'est
 *    pas retenté dans la même génération.
 *
 *  Fonctionnement :
 *    Le pointeur n'appartient pas à l'appelant. La borne MAX_FACES ne
 *    protège que le cache, pas l'accès aux données du modèle.
 *
 * ----------------------------------------------------------------------------
 */
static double *ordering_plane(Model3D *model, int face, double fallback[4])
{
    OrderPlaneBlock *block=NULL;
    double *out=fallback;
    int b,slot;
    if (order_cache_owner != model) ordering_cache_invalidate(model);
    /* Chercher un bloc de plans ; utiliser fallback si le stockage du cache
     * est indisponible.
     */
    if (face >= 0 && face < MAX_FACES) {
        b=face / ORDER_PLANE_BLOCK_SIZE;slot=face % ORDER_PLANE_BLOCK_SIZE;
        block=order_planes[b];
        if (!block && order_plane_failed[b] != order_cache_generation) {
            block=(OrderPlaneBlock*)calloc(1,sizeof(OrderPlaneBlock));order_planes[b]=block;
            if (!block) order_plane_failed[b]=order_cache_generation;
        }
        if (block) {
            out=block->plane[slot];
            if (block->stamp[slot] == order_cache_generation) return out;
            block->stamp[slot]=order_cache_generation;
        }
    }
    /* Convertir exactement les quatre coefficients de la face et compter
     * cette conversion si le profiling est activé.
     */
    out[0]=FIXED64_TO_FLOAT(model->faces.plane_a[face]);
    out[1]=FIXED64_TO_FLOAT(model->faces.plane_b[face]);
    out[2]=FIXED64_TO_FLOAT(model->faces.plane_c[face]);
    out[3]=FIXED64_TO_FLOAT(model->faces.plane_d[face]);
    ORDER_COUNT(plane_conversions);
    return out;
}


/*
 * ----------------------------------------------------------------------------
 *  ordering_fixed_plane_reset
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Invalider les plans préparés par changement d'époque.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    époque incrémentée et paquet assembleur invalidé ; en cas de
 *    rebouclage, efface les plans et invalide le cache dense.
 *
 *  Fonctionnement :
 *    Les autres caches de chevauchement et de ray casting sont conservés.
 *
 * ----------------------------------------------------------------------------
 */
static void ordering_fixed_plane_reset(void)
{
    order_asm_packet_plane=NULL;
    /* Distinguer une remise à zéro géométrique d'un changement complet de vue. */
    ++order_fixed_plane_epoch;
    if(order_fixed_plane_epoch==0){
        memset(order_fixed_planes,0,sizeof(order_fixed_planes));
        order_geometry_generation=0;
    }
}


/*
 * ----------------------------------------------------------------------------
 *  ordering_fixed_plane
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Préparer le plan d'une face et les conditions du chemin assembleur.
 *
 *  Paramètres :
 *    [entrée] model ; face, identifiant valide ; epsilon, tolérance du
 *    classement.
 *
 *  Retour :
 *    Slot partagé OrderFixedPlane, réutilisable seulement avec les mêmes
 *    face, génération, époque et epsilon.
 *
 *  Sorties et effets de bord :
 *    coefficients, côté de l'observateur, admissibilité assembleur et cache
 *    de côtés des sommets.
 *
 *  Fonctionnement :
 *    Une collision remplace le slot.
 *
 *    Le noyau n'est admissible que pour les coefficients 32 bits, |d| <=
 *    2^62 et epsilon positif ou nul.
 *
 * ----------------------------------------------------------------------------
 */
static OrderFixedPlane *ordering_fixed_plane(Model3D *model,int face,Fixed64 epsilon)
{
    OrderFixedPlane *p;
    FaceArrays3D *faces=&model->faces;
    if(order_cache_owner!=model)ordering_cache_invalidate(model);
    /* Vérifier la clé du plan avant de réutiliser coefficients et côtés des
     * sommets.
     */
    p=&order_fixed_planes[(unsigned int)face & (ORDER_FIXED_PLANE_SLOTS-1)];
    if(p->generation==order_cache_generation && p->epoch==order_fixed_plane_epoch &&
       p->face==face && p->epsilon==epsilon)return p;
    /* Réinitialiser les côtés et préparer la variante 32 bits seulement si
     * les bornes numériques le permettent.
     */
    memset(p->vertex_sides,0,sizeof(p->vertex_sides));
    p->a=faces->plane_a[face];p->b=faces->plane_b[face];
    p->c=faces->plane_c[face];p->d=faces->plane_d[face];
    p->a32=(Fixed32)p->a;p->b32=(Fixed32)p->b;p->c32=(Fixed32)p->c;
    p->observer=(p->d>epsilon)?1:((p->d < -epsilon)?-1:0);
    p->asm_ready=(p->a==(Fixed64)p->a32 && p->b==(Fixed64)p->b32 &&
                  p->c==(Fixed64)p->c32 && p->d>=-4611686018427387904LL &&
                  p->d<=4611686018427387904LL && epsilon>=0);
    p->face=face;p->epsilon=epsilon;p->generation=order_cache_generation;p->epoch=order_fixed_plane_epoch;
    return p;
}


/*
 * ----------------------------------------------------------------------------
 *  ordering_winding_slot
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Obtenir le slot de l'aire orientée d'une face.
 *
 *  Paramètres :
 *    [entrée] model et face.
 *
 *  Retour :
 *    Slot partagé, dont la validité reste à tester.
 *
 *  Sorties et effets de bord :
 *    changement de propriétaire si nécessaire ; collisions par masquage.
 *
 * ----------------------------------------------------------------------------
 */
static OrderWinding *ordering_winding_slot(Model3D *model,int face)
{
    if(order_cache_owner!=model)ordering_cache_invalidate(model);
    /* Sélectionner le slot ; une comparaison de clé reste nécessaire avant
     * lecture.
     */
    return &order_windings[(unsigned int)face & (ORDER_WINDING_SLOTS-1)];
}


/*
 * ----------------------------------------------------------------------------
 *  ordering_winding_hit
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Tester la validité d'une aire orientée mémorisée.
 *
 *  Paramètres :
 *    [entrée] entry, face et scale.
 *
 *  Retour :
 *    Vrai si génération, face et projection correspondent ; faux sinon.
 *
 *  Fonctionnement :
 *    Aucun effet de bord.
 *
 * ----------------------------------------------------------------------------
 */
static int ordering_winding_hit(OrderWinding *entry,int face,Fixed32 scale)
{
    /* Comparer également l'échelle : une autre projection peut inverser ou
     * modifier l'aire.
     */
    return entry->generation==order_cache_generation && entry->face==face && entry->scale==scale;
}


/*
 * ----------------------------------------------------------------------------
 *  ordering_winding_store
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Mémoriser le double de l'aire orientée d'une face projetée.
 *
 *  Paramètres :
 *    [entrée] face, scale et area2.
 *
 *    [entrée-sortie] entry
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    La valeur est marquée avec la génération courante.
 *
 * ----------------------------------------------------------------------------
 */
static void ordering_winding_store(OrderWinding *entry,int face,Fixed32 scale,double area2)
{
    /* Associer l'aire orientée à la face et à sa projection dans cette vue. */
    entry->face=face;entry->scale=scale;entry->area2=area2;
    entry->generation=order_cache_generation;
}


/*
 * ----------------------------------------------------------------------------
 *  ordering_geometry_slot
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Obtenir l'octet du cache dense qui contient une relation de plans
 *    orientée.
 *
 *  Paramètres :
 *    [entrée] model ; f1/f2 à base zéro ; epsilon.
 *
 *  Retour :
 *    Octet partagé à décoder sur deux bits, ou NULL pour le chemin de repli.
 *
 *  Fonctionnement :
 *    Aucun résultat n'est écrit ici. Le cache dense est limité à 256 faces.
 *    Génération, époque, nombre de faces et epsilon déclenchent sa remise à
 *    zéro ; chaque octet contient quatre codes. Un échec de réserve est
 *    mémorisé pour ne pas réallouer en boucle dans la vue.
 *
 * ----------------------------------------------------------------------------
 */
static unsigned char *ordering_geometry_slot(Model3D *model,int f1,int f2,Fixed64 epsilon)
{
    int nf=model->faces.face_count;
    unsigned int stride,bytes;
    /* Refuser les modèles trop grands et les indices hors limites pour le
     * cache dense.
     */
    if(nf<=0 || nf>ORDER_GEOMETRY_DENSE_LIMIT ||
       (unsigned int)f1>=(unsigned int)nf || (unsigned int)f2>=(unsigned int)nf)return NULL;
    if(order_cache_owner!=model)ordering_cache_invalidate(model);
    stride=((unsigned int)nf+3U)>>2;
    bytes=(unsigned int)nf*stride;
    /* Réserver le tableau compact, sans répéter un échec d'allocation dans
     * la même génération.
     */
    if(order_geometry_capacity<bytes){
        unsigned char *grown;
        if(order_geometry_failed==order_cache_generation)return NULL;
        grown=(unsigned char*)malloc(bytes);
        if(!grown){order_geometry_failed=order_cache_generation;return NULL;}
        free(order_geometry_pairs);order_geometry_pairs=grown;order_geometry_capacity=bytes;
        order_geometry_generation=0;
    }
    /* Effacer les codes après changement de clé et construire une fois les
     * pointeurs de lignes.
     */
    if(order_geometry_generation!=order_cache_generation ||
       order_geometry_epoch!=order_fixed_plane_epoch || order_geometry_faces!=nf ||
       order_geometry_epsilon!=epsilon){
        unsigned char *row=order_geometry_pairs;
        int i;
        memset(order_geometry_pairs,0,bytes);


        for(i=0;i<nf;++i){order_geometry_rows[i]=row;row+=stride;}
        order_geometry_stride=stride;order_geometry_faces=nf;
        order_geometry_epoch=order_fixed_plane_epoch;order_geometry_epsilon=epsilon;
        order_geometry_generation=order_cache_generation;
    }
    /* Retourner l'octet partagé ; le choix des deux bits de f2 incombe à
     * l'appelant.
     */
    return order_geometry_rows[f1]+((unsigned int)f2>>2);
}

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

#define RCH_ZOOM_MAX_VERTS (2 * MAX_FACE_VERTICES)

typedef struct {
    Fixed32 saved_scale;
    int idx[RCH_ZOOM_MAX_VERTS];
    int saved_x2d[RCH_ZOOM_MAX_VERTS];
    int saved_y2d[RCH_ZOOM_MAX_VERTS];
    int count;
    int f1_minx, f1_maxx, f1_miny, f1_maxy;
    int f2_minx, f2_maxx, f2_miny, f2_maxy;
} RCHZoomPatch;

#define DONY_RAY_PAIR_BUCKETS 256
typedef struct {
    unsigned int *first, *second, *next;
    unsigned int heads[DONY_RAY_PAIR_BUCKETS];
    unsigned int stamps[DONY_RAY_PAIR_BUCKETS];
    int capacity;
    unsigned int count;
} DonyRayPairs;



static int projected_polygons_overlap(Model3D*m,int a,int b);
static int compute_intersection_region_bbox(Model3D*m,int a,int b,int*x,int*y,int*z,int*w,long long*area){qd_calls++;return 0;}
static int compute_intersection_centroid_ordered_qd_fixed(Model3D*m,int a,int b,int*x,int*y,long long*area){qd_calls++;return 0;}
static void fractional_projection_cached(Model3D *model, int v, double scale, double *outx, double *outy);
static int compute_bbox_intersection(Model3D* model, int f1, int f2, int *ix0, int *iy0, int *ix1, int *iy1);
static int segs_intersect_int(int x1,int y1,int x2,int y2,int x3,int y3,int x4,int y4);
static int dony_ray_pairs_reserve(DonyRayPairs *cache, int nf);
static int compute_intersection_centroid_ordered_fixed_work(Model3D* model, int subj, int clip, int* outx, int* outy, long long* out_area2, double* shared_work, double* fractional_x, double* fractional_y, double* fractional_area2);
static int ray_cast_at(Model3D* model, int f1, int f2, int cx, int cy);
static int projected_polygons_overlap_cached(Model3D *model, int f1, int f2);
static int compute_intersection_centroid_ordered_fixed_tryboth(Model3D* model, int subj, int clip, int* outx, int* outy, long long* out_area2, int* out_fallback);
static int geov3_run(Model3D* model, int face_count, int preserve_order, GeoV3ExchangeCallback on_exchange, void *context);
static int faces_vertices_equal(FaceArrays3D* faces, VertexArrays3D* vtx, int f1, int f2);
static int projected_polygons_overlap_v1(Model3D* model, int f1, int f2);
static void dony_ray_pair_record(DonyRayPairs *cache, int p, int q, unsigned int generation);
static void rch_patch_pair_at_scale(Model3D* model, int f1, int f2, Fixed32 test_scale, RCHZoomPatch* p);
static void rch_unpatch_pair(Model3D* model, RCHZoomPatch* p, int f1, int f2);
static int on_seg_ll(int ax,int ay,int bx,int by,int cx,int cy);
static int ray_cast_hierarchical(Model3D* model, int f1, int f2);
static long long orient_ll(int ax,int ay,int bx,int by,int cx,int cy);
static void fractional_projection_cache_invalidate(Model3D *model);
static int compute_intersection_centroid_fractional_tryboth(Model3D *model, int subj, int clip, double *outx, double *outy, double *out_area2);
static int ray_cast_at_fractional(Model3D *model, int f1, int f2, double cx, double cy);
static int move_element_remove_and_insert(int *arr, int n, int from, int insert_idx);
static int ray_cast_hierarchical_impl(Model3D* model, int f1, int f2);
static void rch_recompute_face_bbox_cache(Model3D* model, int f);
static int ray_cast_distances(Model3D* model, int f1, int f2, int cx, int cy, float *out_tf1, float *out_tf2);
static void sort_by_zmax(int *sorted,Fixed32 *z_max,int nf);
static int dony_ray_pair_seen(DonyRayPairs *cache, int p, int q, unsigned int generation);
static int geometric_face_relation(Model3D* model, int f1, int f2);
static int fractional_face_gather(Model3D *model,int face,int n,double scale, double *px,double *py, double **edge_x,double **edge_y);
static int compute_intersection_centroid_ordered_fixed(Model3D* model, int subj, int clip, int* outx, int* outy, long long* out_area2);
static int rch_gather_pair_vertex_indices(FaceArrays3D* faces, int f1, int f2, int* out_idx);
static int point_in_poly_int(int px,int py,FaceArrays3D* faces, VertexArrays3D* vtx, int f, int n);
static int compute_bbox_intersection_center(Model3D* model, int f1, int f2, int *outx, int *outy);
static int segs_intersect_int_fixed64( int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4);
static inline int FIXED_ROUND_TO_INT(Fixed32 x) {
    /* Choisir le biais selon le signe ; le décalage négatif fait partie du
     * comportement existant.
     */
    if (x >= 0) return (int)(((x) + FIXED_HALF) >> FIXED_SHIFT);
    else        return (int)(((x) - FIXED_HALF) >> FIXED_SHIFT);
}
static inline Fixed32 sin_deg_int(int deg) {
    deg %= 360;
    if (deg < 0) deg += 360;
    /* Lire directement la valeur tabulée après réduction modulo un tour. */
    return sin_table[deg];
}
static inline Fixed32 cos_deg_int(int deg) {
    deg %= 360;
    if (deg < 0) deg += 360;
    /* Lire directement la valeur tabulée après réduction modulo un tour. */
    return cos_table[deg];
}
static inline int normalize_deg(int deg) {
    deg %= 360;
    if (deg < 0) deg += 360;
    /* Retourner l'angle normalisé sans modifier l'état de l'observateur. */
    return deg;
}
Model3D* createModel3D(void) {

    /* Réserver la structure propriétaire avant les tableaux ; sortir
     * immédiatement si elle manque.
     */
    Model3D* model = (Model3D*)malloc(sizeof(Model3D));
    if (model == NULL) {
        return NULL;
    }
    /* Initialiser uniquement la structure propriétaire : tous les pointeurs
     * non encore alloués restent NULL, donc le destructeur accepte chaque
     * étape de création. Les grands tableaux gardent leurs malloc d’origine. */
    memset(model, 0, sizeof(*model));
    int n = MAX_VERTICES;
    model->vertices.vertex_count = n;
    model->auto_scale = FIXED_ONE;

    /* Allouer les coordonnées objet, observateur et écran ; tout échec
     * rejoint la sortie de libération commune. */
    model->vertices.x = (Fixed32*)malloc(n * sizeof(Fixed32));
    model->vertices.y = (Fixed32*)malloc(n * sizeof(Fixed32));
    model->vertices.z = (Fixed32*)malloc(n * sizeof(Fixed32));
    model->vertices.xo = (Fixed32*)malloc(n * sizeof(Fixed32));
    model->vertices.yo = (Fixed32*)malloc(n * sizeof(Fixed32));
    model->vertices.zo = (Fixed32*)malloc(n * sizeof(Fixed32));
    model->vertices.x2d = (int*)malloc(n * sizeof(int));
    model->vertices.y2d = (int*)malloc(n * sizeof(int));

    if (!model->vertices.x || !model->vertices.y || !model->vertices.z ||
        !model->vertices.xo || !model->vertices.yo || !model->vertices.zo ||
        !model->vertices.x2d || !model->vertices.y2d) {
        printf("Error: Unable to allocate memory for vertex arrays\n");
        keypress();
        goto allocation_failed;
    }


    /* Réserver les listes et métriques des faces indépendamment des
     * coordonnées des sommets.
     */
    int nf = MAX_FACES;

    model->faces.vertex_count = (int*)malloc(nf * sizeof(int));
    if (!model->faces.vertex_count) {
        printf("Error: Unable to allocate memory for face vertex_count array\n");
        keypress();
        goto allocation_failed;
    }

    model->faces.saved_vertex_count = (int*)calloc(nf, sizeof(int));
    if (!model->faces.saved_vertex_count) {
        printf("Error: Unable to allocate memory for face saved_vertex_count array\n");
        keypress();
        goto allocation_failed;
    }

    /* Réserver le buffer compact pour une moyenne de cinq indices par face ;
     * aucune extension n'est effectuée ici.
     */
    int estimated_total_indices = nf * 5;
    model->faces.vertex_indices_buffer = (int*)malloc(estimated_total_indices * sizeof(int));
    if (!model->faces.vertex_indices_buffer) {
        printf("Error: Unable to allocate memory for vertex_indices_buffer\n");
        keypress();
        goto allocation_failed;
    }

    model->faces.vertex_indices_ptr = (int*)malloc(nf * sizeof(int));
    if (!model->faces.vertex_indices_ptr) {
        printf("Error: Unable to allocate memory for vertex_indices_ptr array\n");
        keypress();
        goto allocation_failed;
    }

    /* Allouer les intervalles de profondeur et la clé moyenne utilisée par FAST. */
    model->faces.z_min = (Fixed32*)malloc(nf * sizeof(Fixed32));
    model->faces.z_max = (Fixed32*)malloc(nf * sizeof(Fixed32));
    if (!model->faces.z_min || !model->faces.z_max) {
        printf("Error: Unable to allocate memory for face depth arrays\n");
        keypress();
        goto allocation_failed;
    }

    model->faces.z_mean = (Fixed32*)malloc(nf * sizeof(Fixed32));
    if (!model->faces.z_mean) {
        printf("Error: Unable to allocate memory for face z_mean array\n");
        keypress();
        goto allocation_failed;
    }

    /* Allouer les coefficients 64 bits des plans observateur. */
    model->faces.plane_a = (Fixed64*)malloc(nf * sizeof(Fixed64));
    model->faces.plane_b = (Fixed64*)malloc(nf * sizeof(Fixed64));
    model->faces.plane_c = (Fixed64*)malloc(nf * sizeof(Fixed64));
    model->faces.plane_d = (Fixed64*)malloc(nf * sizeof(Fixed64));
    if (!model->faces.plane_a || !model->faces.plane_b || !model->faces.plane_c || !model->faces.plane_d) {
        printf("Error: Unable to allocate memory for face plane arrays\n");
        keypress();
        goto allocation_failed;
    }

    /* Allouer les bornes projetées servant aux rejets rapides de paires. */
    model->faces.minx = (int*)malloc(nf * sizeof(int));
    model->faces.maxx = (int*)malloc(nf * sizeof(int));
    model->faces.miny = (int*)malloc(nf * sizeof(int));
    model->faces.maxy = (int*)malloc(nf * sizeof(int));
    if (!model->faces.minx || !model->faces.maxx || !model->faces.miny || !model->faces.maxy) {
        printf("Error: Unable to allocate memory for face bounding box arrays\n");
        keypress();
        goto allocation_failed;
    }

    /* Réserver les drapeaux de visibilité et la permutation de rendu. */
    model->faces.display_flag = (int*)malloc(nf * sizeof(int));
    if (!model->faces.display_flag) {
        printf("Error: Unable to allocate memory for face display_flag array\n");
        keypress();
        goto allocation_failed;
    }

    model->faces.sorted_face_indices = (int*)malloc(nf * sizeof(int));
    if (!model->faces.sorted_face_indices) {
        printf("Error: Unable to allocate memory for sorted_face_indices array\n");
        keypress();
        goto allocation_failed;
    }

    return model;

allocation_failed:
    /* Le même destructeur libère un modèle complet ou partiel, une seule
     * fois par pointeur. Les messages et l’attente clavier restent locaux. */
    destroyModel3D(model);
    return NULL;
}
void destroyModel3D(Model3D* model) {
    /* Ne rien faire pour NULL ; la structure doit avoir ses pointeurs
     * inutilisés à NULL, même pour une création interrompue.
     */
    if (model != NULL) {
        /* Libérer le cache fractionnaire avant les tableaux qu'il référence. */
        fractional_projection_cache_release(model);

        if (model->vertices.x) free(model->vertices.x);
        if (model->vertices.y) free(model->vertices.y);
        if (model->vertices.z) free(model->vertices.z);
        if (model->vertices.xo) free(model->vertices.xo);
        if (model->vertices.yo) free(model->vertices.yo);
        if (model->vertices.zo) free(model->vertices.zo);
        if (model->vertices.x2d) free(model->vertices.x2d);
        if (model->vertices.y2d) free(model->vertices.y2d);

        /* Libérer les listes et métriques de faces, puis les copies et
         * buffers supplémentaires.
         */
        if (model->faces.vertex_count) free(model->faces.vertex_count);
        if (model->faces.vertex_indices_buffer) free(model->faces.vertex_indices_buffer);
        if (model->faces.vertex_indices_ptr) free(model->faces.vertex_indices_ptr);
        if (model->faces.z_max) free(model->faces.z_max);
        if (model->faces.z_mean) free(model->faces.z_mean);
        if (model->faces.z_min) free(model->faces.z_min);
        if (model->faces.plane_a) free(model->faces.plane_a);
        if (model->faces.plane_b) free(model->faces.plane_b);
        if (model->faces.plane_c) free(model->faces.plane_c);
        if (model->faces.plane_d) free(model->faces.plane_d);
        if (model->faces.minx) free(model->faces.minx);
        if (model->faces.maxx) free(model->faces.maxx);
        if (model->faces.miny) free(model->faces.miny);
        if (model->faces.maxy) free(model->faces.maxy);
        if (model->faces.minx3) free(model->faces.minx3);
        if (model->faces.maxx3) free(model->faces.maxx3);
        if (model->faces.miny3) free(model->faces.miny3);
        if (model->faces.maxy3) free(model->faces.maxy3);
        if (model->faces.minz3) free(model->faces.minz3);
        if (model->faces.maxz3) free(model->faces.maxz3);
        if (model->faces.display_flag) free(model->faces.display_flag);
        if (model->faces.sorted_face_indices) free(model->faces.sorted_face_indices);
        if (model->faces.saved_vertex_count) free(model->faces.saved_vertex_count);

        if (model->orig_x) free(model->orig_x);
        if (model->orig_y) free(model->orig_y);
        if (model->orig_z) free(model->orig_z);
        if (model->coord_buf) free(model->coord_buf);

        /* Libérer la structure en dernier ; aucun de ses tableaux ne reste utilisable. */
        free(model);
    }
}
static void compute_auto_fit(VertexArrays3D *vtx, int vertex_count,
                             Fixed32 max_dim, Model3D *owner)
{
    owner->auto_suggested_distance = fixed_mul32_fast(max_dim, INT_TO_FIXED(3));

    /* Projeter provisoirement les sommets à la vue initiale pour mesurer
     * leur emprise. Les produits 16.16 réutilisent le noyau rapide existant,
     * avec le même décalage et la même troncature que FIXED_MUL_64.
     */
    int ah = 30, av = 20;
    Fixed32 cos_h = cos_deg_int(ah), sin_h = sin_deg_int(ah);
    Fixed32 cos_v = cos_deg_int(av), sin_v = sin_deg_int(av);
    const Fixed32 cos_h_cos_v = fixed_mul32_fast(cos_h, cos_v);
    const Fixed32 sin_h_cos_v = fixed_mul32_fast(sin_h, cos_v);
    const Fixed32 cos_h_sin_v = fixed_mul32_fast(cos_h, sin_v);
    const Fixed32 sin_h_sin_v = fixed_mul32_fast(sin_h, sin_v);
    float pxmin = 1e30f, pxmax = -1e30f, pymin = 1e30f, pymax = -1e30f;
    for (int i = 0; i < vertex_count; ++i) {
        Fixed32 x = vtx->x[i]; Fixed32 y = vtx->y[i]; Fixed32 z = vtx->z[i];
        Fixed32 term1 = fixed_mul32_fast(x, cos_h_cos_v);
        Fixed32 term2 = fixed_mul32_fast(y, sin_h_cos_v);
        Fixed32 term3 = fixed_mul32_fast(z, sin_v);
        Fixed32 zo = FIXED_ADD(FIXED_SUB(FIXED_SUB(FIXED_NEG(term1), term2), term3), owner->auto_suggested_distance);
        if (zo > 0) {
            Fixed32 xo = FIXED_ADD(FIXED_NEG(fixed_mul32_fast(x, sin_h)), fixed_mul32_fast(y, cos_h));
            Fixed32 yo = FIXED_ADD(FIXED_SUB(FIXED_NEG(fixed_mul32_fast(x, cos_h_sin_v)), fixed_mul32_fast(y, sin_h_sin_v)), fixed_mul32_fast(z, cos_v));
            /* L'échelle 16.16 s'annule dans xo/zo et yo/zo : ne pas
             * diviser les quatre conversions par 65536. Garder les deux
             * divisions finales, sans approximation par un inverse partagé.
             */
            float zo_float = (float)zo;
            float px = (float)xo / zo_float;
            float py = (float)yo / zo_float;
            if (px < pxmin) pxmin = px;
            if (px > pxmax) pxmax = px;
            if (py < pymin) pymin = py;
            if (py > pymax) pymax = py;
        }
    }

    /* Choisir l'échelle qui tient dans 90 % de la largeur et de la
     * hauteur disponibles.
     */
    float margin = 0.9f;
    float winw = (float)(CENTRE_X * 2);
    float winh = (float)(CENTRE_Y * 2);
    float s = 100.0f;
    if (pxmax > pxmin && pymax > pymin) {
        float s1 = (winw * margin) / (pxmax - pxmin);
        float s2 = (winh * margin) / (pymax - pymin);
        s = (s1 < s2) ? s1 : s2;
    }
    owner->auto_suggested_proj_scale = FLOAT_TO_FIXED(s);
    owner->auto_fit_ready = 1;
}
int readVertices(const char* filename, VertexArrays3D* vtx, int max_vertices, Model3D* owner) {
    FILE *file;
    char line[MAX_LINE_LENGTH];
    int line_number = 1;
    int vertex_count = 0;

    /* Ouvrir le fichier en texte ; les autres types de lignes sont ignorés. */
    file = fopen(filename, "r");
    if (file == NULL) {
        return -1;
    }

    printf("\nReading vertices from file '%s' ", filename);

    while (fgets(line, sizeof(line), file) != NULL) {
        /* Convertir trois coordonnées en 16.16 dans la capacité disponible,
         * en permutant y et z.
         */
        if (line[0] == 'v' && line[1] == ' ') {
            if (vertex_count < max_vertices) {
                float x, y, z;
                if (sscanf(line + 2, "%f %f %f", &x, &y, &z) == 3) {
                    vtx->x[vertex_count] = FLOAT_TO_FIXED(x);

                    vtx->y[vertex_count] = FLOAT_TO_FIXED(z);
                    vtx->z[vertex_count] = FLOAT_TO_FIXED(y);
                    vertex_count++;
                    if (vertex_count % 10 == 0) printf("..");
                } else {
                    printf("[\nDEBUG] readVertices: sscanf failed at line %d: %s\n", line_number, line);
                    keypress();
                }
            } else {
                printf("\n[DEBUG] readVertices: vertex limit reached (%d)\n", max_vertices);
                keypress();
            }
        }
        line_number++;
    }
    printf("\n");
    printf("Reading vertices finished : %d vertices read.\n", vertex_count);

    fclose(file);

    /* Calculer les bornes objet et soustraire leur centre aux sommets lus. */
    if (vertex_count > 0 && owner) {
        Fixed32 xmin = vtx->x[0], xmax = vtx->x[0];
        Fixed32 ymin = vtx->y[0], ymax = vtx->y[0];
        Fixed32 zmin = vtx->z[0], zmax = vtx->z[0];
        for (int i = 1; i < vertex_count; ++i) {
            if (vtx->x[i] < xmin) xmin = vtx->x[i];
            if (vtx->x[i] > xmax) xmax = vtx->x[i];
            if (vtx->y[i] < ymin) ymin = vtx->y[i];
            if (vtx->y[i] > ymax) ymax = vtx->y[i];
            if (vtx->z[i] < zmin) zmin = vtx->z[i];
            if (vtx->z[i] > zmax) zmax = vtx->z[i];
        }
        owner->auto_center_x = (Fixed32)((xmin + xmax) / 2);
        owner->auto_center_y = (Fixed32)((ymin + ymax) / 2);
        owner->auto_center_z = (Fixed32)((zmin + zmax) / 2);

        for (int i = 0; i < vertex_count; ++i) {
            vtx->x[i] = FIXED_SUB(vtx->x[i], owner->auto_center_x);
            vtx->y[i] = FIXED_SUB(vtx->y[i], owner->auto_center_y);
            vtx->z[i] = FIXED_SUB(vtx->z[i], owner->auto_center_z);
        }
        owner->auto_centered = 1;

        /* Établir une distance suggérée à partir de la plus grande dimension. */
        Fixed32 dx = FIXED_SUB(xmax, xmin);
        Fixed32 dy = FIXED_SUB(ymax, ymin);
        Fixed32 dz = FIXED_SUB(zmax, zmin);
        Fixed32 max_dim = dx;
        if (dy > max_dim) max_dim = dy;
        if (dz > max_dim) max_dim = dz;

        compute_auto_fit(vtx, vertex_count, max_dim, owner);
    }

    return vertex_count;
}
int readFaces_model(const char* filename, Model3D* model) {
    FILE *file;
    char line[MAX_LINE_LENGTH];
    int line_number = 1;
    int face_count = 0;
    int i;

    /* Valider la structure avant l'ouverture du fichier. */
    if (model == NULL || model->faces.vertex_count == NULL) {
        printf("Error: Invalid model structure for readFaces_model\n");
        return -1;
    }

    file = fopen(filename, "r");
    if (file == NULL) {
        printf("Error: Unable to open file '%s' to read faces\n", filename);
        return -1;
    }

    printf("\nReading faces from file '%s' ", filename);

    int buffer_pos = 0;

    while (fgets(line, sizeof(line), file) != NULL) {

        /* Initialiser la face à l'offset courant puis analyser les premiers
         * indices positifs.
         */
        if (line[0] == 'f' && line[1] == ' ') {
            if (face_count < MAX_FACES) {

                model->faces.vertex_count[face_count] = 0;
                model->faces.display_flag[face_count] = 1;
                model->faces.saved_vertex_count[face_count] = 0;
                model->faces.vertex_indices_ptr[face_count] = buffer_pos;

                char *ptr = line + 2;
                int temp_indices[MAX_FACE_VERTICES];
                int temp_vertex_count = 0;
                int invalid_index_found = 0;

                /* Parcourir les tokens, ignorer les suffixes et s'arrêter à
                 * la limite par face.
                 */
                while (*ptr != '\0' && *ptr != '\n' && temp_vertex_count < MAX_FACE_VERTICES) {

                    while (*ptr == ' ' || *ptr == '\t') ptr++;

                    if (*ptr == '\0' || *ptr == '\n') break;

                    int vertex_index = 0;
                    while (*ptr >= '0' && *ptr <= '9') {
                        vertex_index = vertex_index * 10 + (*ptr - '0');
                        ptr++;
                    }

                    while (*ptr != '\0' && *ptr != ' ' && *ptr != '\t' && *ptr != '\n') {
                        ptr++;
                    }

                    if (vertex_index >= 1) {
                        if (vertex_index > readVertices_last_count) {

                            invalid_index_found = 1;
                        }
                        temp_indices[temp_vertex_count] = vertex_index;
                        temp_vertex_count++;
                    }
                }

                /* Refuser la face entière si un indice dépasse la borne de validation. */
                if (invalid_index_found) {
                    printf("\nERROR: Face at line %d references vertex index > %d vertices\n",
                           line_number, readVertices_last_count);
                    fclose(file);
                    return -1;
                } else {

                    for (i = 0; i < temp_vertex_count; i++) {
                        model->faces.vertex_indices_buffer[buffer_pos++] = temp_indices[i];
                    }
                    model->faces.vertex_count[face_count] = temp_vertex_count;
                    model->faces.total_indices += temp_vertex_count;

                    if (model->faces.vertex_count[face_count] > 0) {
                        face_count++;
                        if (face_count % 10 == 0) {printf(".");}
                    } else {
                        printf("     -> WARNING: Face without valid vertices ignored\n");
                    }
                }
            } else {
                printf("     -> WARNING: Face limit reached (%d)\n", MAX_FACES);
            }
        }

        line_number++;
    }

    fclose(file);

    /* Publier le compteur lu et reconstruire une permutation canonique des faces. */
    model->faces.face_count = face_count;

    for (i = 0; i < face_count; i++) {
        model->faces.sorted_face_indices[i] = i;
    }

    printf("\nReading faces finished : %d faces read.\n", face_count);
    return face_count;
}
static void check_intersect(Model3D* model) {
    if (!model) return;
    /* Préparer les bornes objet utilisées comme premier filtre. */
    compute_face_bboxes3D(model);
    FaceArrays3D* faces = &model->faces;
    int fc = faces->face_count;

    if (fc <= 1) {

        keypress();
        return;
    }

    printf("check_intersect: scanning %d faces, please wait...\n", fc);

    const Fixed32 eps = FLOAT_TO_FIXED(0.02f);
    int count = 0;
    int ip_count = 0;

    int changed;
    /* Repartir du modèle modifié à chaque découpage accepté. */
    do {
        changed = 0;
        fc = faces->face_count;
        for (int i = 0; i < fc; ++i) {
            Fixed32 minx_i = faces->minx3[i];
            Fixed32 maxx_i = faces->maxx3[i];
            Fixed32 miny_i = faces->miny3[i];
            Fixed32 maxy_i = faces->maxy3[i];
            Fixed32 minz_i = faces->minz3[i];
            Fixed32 maxz_i = faces->maxz3[i];
            for (int j = i + 1; j < fc; ++j) {
                Fixed32 minx_j = faces->minx3[j];
                Fixed32 maxx_j = faces->maxx3[j];
                Fixed32 miny_j = faces->miny3[j];
                Fixed32 maxy_j = faces->maxy3[j];
                Fixed32 minz_j = faces->minz3[j];
                Fixed32 maxz_j = faces->maxz3[j];

                /* Limiter les tests coûteux aux boîtes proches selon epsilon. */
                if (minx_i <= maxx_j + eps && maxx_i >= minx_j - eps &&
                    miny_i <= maxy_j + eps && maxy_i >= miny_j - eps &&
                    minz_i <= maxz_j + eps && maxz_i >= minz_j - eps) {

                    Fixed32 ox = (minx_i < minx_j ? minx_j : minx_i);
                    Fixed32 ux = (maxx_i < maxx_j ? maxx_i : maxx_j);
                    Fixed32 oy = (miny_i < miny_j ? miny_j : miny_i);
                    Fixed32 uy = (maxy_i < maxy_j ? maxy_i : maxy_j);
                    Fixed32 oz = (minz_i < minz_j ? minz_j : minz_i);
                    Fixed32 uz = (maxz_i < maxz_j ? maxz_i : maxz_j);
                    float dx = FIXED_TO_FLOAT(ux - ox);
                    float dy = FIXED_TO_FLOAT(uy - oy);
                    float dz = FIXED_TO_FLOAT(uz - oz);

                    if (dx >= 0.0f && dy >= 0.0f) {

                        ++count;
                        /* Ignorer les faces ayant déjà un identifiant de sommet en commun. */
                        if (faces_share_vertex_or_edge(faces, i, j)) {

                        } else {
                            double a1,b1,c1,d1, a2,b2,c2,d2;
                            compute_face_plane(model, i, &a1, &b1, &c1, &d1, FACE_PLANE_RAW_FIXED);
                            compute_face_plane(model, j, &a2, &b2, &c2, &d2, FACE_PLANE_RAW_FIXED);
                            double norm1 = sqrt(a1*a1 + b1*b1 + c1*c1);
                            double norm2 = sqrt(a2*a2 + b2*b2 + c2*c2);
                            int coplanar = 0;
                            if (norm1 > 0 && norm2 > 0) {
                                double dot = a1*a2 + b1*b2 + c1*c2;
                                double cosang = dot / (norm1 * norm2);
                                if (fabs(fabs(cosang) - 1.0) < 0.01) {
                                    double dist = fabs(d1 - d2) / norm1;
                                    if (dist < FIXED_TO_FLOAT(eps)) {
                                        coplanar = 1;
                                    }
                                }
                            }
                            if (coplanar) {

                            }
                            /* Tester les arêtes seulement pour les paires non reconnues coplanaires. */
                            if (!coplanar) {

                                int interior_i_j, interior_j_i;
                                /* Les deux directions sont nécessaires au choix
                                 * du découpage ; chacune prépare son contour une fois. */
                                int cross_i_j = evaluate_face_crossings(model, i, j, &interior_i_j);
                                int cross_j_i = evaluate_face_crossings(model, j, i, &interior_j_i);
                                if (interior_i_j || interior_j_i) ++ip_count;

                            /* Choisir la face à découper d’après les nombres
                             * de traversées dans chaque sens.
                             */
                            if (cross_i_j == 2 && cross_j_i == 0) {

                                if (split_face_by_plane(model, i, j, NULL, NULL, NULL, 0, 0)) {
                                    changed = 1;
                                    break;
                                }
                            } else if (cross_j_i == 2 && cross_i_j == 0) {

                                if (split_face_by_plane(model, j, i, NULL, NULL, NULL, 0, 0)) {
                                    changed = 1;
                                    break;
                                }
                            } else {

                                if (cross_i_j > 0) {
                                    int force = (cross_i_j == 1);
                                    if (split_face_by_plane(model, j, i, NULL, NULL, NULL, 0, force)) {
                                        changed = 1;
                                        break;
                                    }
                                }
                                if (!changed && cross_j_i > 0) {
                                    int force = (cross_j_i == 1);
                                    if (split_face_by_plane(model, i, j, NULL, NULL, NULL, 0, force)) {
                                        changed = 1;
                                        break;
                                    }
                                }
                            }
                            }
                            }
                        }
                    }
                }
            }
            if (changed) break;
    } while (changed);

    /* Afficher les compteurs accumulés et le nombre final de faces. */
    printf("Total overlapping pairs: %d\n", count);
    printf("Total interpenetrating pairs: %d\n", ip_count);
    printf("Final face count: %d\n", faces->face_count);
}
static int faces_share_vertex_or_edge(FaceArrays3D* faces, int i, int j) {
    int cnti = faces->vertex_count[i];
    int offi = faces->vertex_indices_ptr[i];
    int cntj = faces->vertex_count[j];
    int offj = faces->vertex_indices_ptr[j];
    /* Comparer les deux listes d'identifiants OBJ, sans test de coordonnées. */
    for (int a = 0; a < cnti; ++a) {
        int vi = faces->vertex_indices_buffer[offi + a];
        for (int b = 0; b < cntj; ++b) {
            if (vi == faces->vertex_indices_buffer[offj + b]) {
                return 1;
            }
        }
    }
    return 0;
}
static void compute_face_plane(Model3D* model, int fidx,
                               double *a, double *b, double *c, double *d, int coordinate_mode) {
    /* Traiter les arguments invalides et les faces trop courtes avant les
     * accès aux coordonnées.
     */
    if (!model || fidx < 0 || fidx >= model->faces.face_count) {
        *a = *b = *c = *d = 0.0;
        return;
    }
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int cnt = faces->vertex_count[fidx];
    if (cnt < 3) {
        *a = *b = *c = *d = 0.0;
        return;
    }
    /* Lire le triangle initial de la face dans le repère objet. */
    int off = faces->vertex_indices_ptr[fidx];
    int vi0 = faces->vertex_indices_buffer[off] - 1;
    int vi1 = faces->vertex_indices_buffer[off + 1] - 1;
    int vi2 = faces->vertex_indices_buffer[off + 2] - 1;
    double x0, y0, z0, x1, y1, z1, x2, y2, z2;
    /* Conserver le passage par float du mode en unités objet ; le mode brut
     * conserve les entiers 16.16 convertis directement en double.
     */
    if (coordinate_mode == FACE_PLANE_OBJECT_UNITS) {
        x0 = FIXED_TO_FLOAT(vtx->x[vi0]);
        y0 = FIXED_TO_FLOAT(vtx->y[vi0]);
        z0 = FIXED_TO_FLOAT(vtx->z[vi0]);
        x1 = FIXED_TO_FLOAT(vtx->x[vi1]);
        y1 = FIXED_TO_FLOAT(vtx->y[vi1]);
        z1 = FIXED_TO_FLOAT(vtx->z[vi1]);
        x2 = FIXED_TO_FLOAT(vtx->x[vi2]);
        y2 = FIXED_TO_FLOAT(vtx->y[vi2]);
        z2 = FIXED_TO_FLOAT(vtx->z[vi2]);
    } else {
        x0 = (double)vtx->x[vi0];
        y0 = (double)vtx->y[vi0];
        z0 = (double)vtx->z[vi0];
        x1 = (double)vtx->x[vi1];
        y1 = (double)vtx->y[vi1];
        z1 = (double)vtx->z[vi1];
        x2 = (double)vtx->x[vi2];
        y2 = (double)vtx->y[vi2];
        z2 = (double)vtx->z[vi2];
    }
    /* Construire la normale par produit vectoriel puis choisir d pour que le
     * premier sommet appartienne au plan.
     */
    double ux = x1 - x0, uy = y1 - y0, uz = z1 - z0;
    double vx = x2 - x0, vy = y2 - y0, vz = z2 - z0;
    *a = uy * vz - uz * vy;
    *b = uz * vx - ux * vz;
    *c = ux * vy - uy * vx;
    *d = -(*a * x0 + *b * y0 + *c * z0);
}
static int strict_inside(int px, int py, int nfj_local, int *vx_local, int *vy_local) {
    int m;

    /* Écarter les points déjà hors du polygone. */
    if (!point_in_poly_arrays_int(px, py, nfj_local, vx_local, vy_local))
        return 0;

    /* Rejeter les points du contour, y compris ses extrémités. */
    for (m = 0; m < nfj_local; ++m) {
        int x1 = vx_local[m], y1 = vy_local[m];
        if (px == x1 && py == y1) return 0;
        {
            int m2 = (m + 1) % nfj_local;
            int x2 = vx_local[m2], y2 = vy_local[m2];
            long long dx = (long long)x2 - x1, dy = (long long)y2 - y1;
            long long dxp = (long long)px - x1, dyp = (long long)py - y1;
            if (dx * dyp == dy * dxp) {

                if ((px >= x1 && px <= x2) || (px >= x2 && px <= x1)) {
                    if ((py >= y1 && py <= y2) || (py >= y2 && py <= y1))
                        return 0;
                }
            }
        }
    }
    return 1;
}
static int evaluate_face_crossings(Model3D* model, int ei, int fj,
                                    int *interior) {
    int count = 0;
    *interior = 0;
    double a, b, c, d;
    /* Construire le plan et rejeter le cas dégénéré. */
    compute_face_plane(model, fj, &a, &b, &c, &d, FACE_PLANE_RAW_FIXED);
    if (a == 0.0 && b == 0.0 && c == 0.0) return 0;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int cnt = faces->vertex_count[ei];
    if (cnt < 2) return 0;
    double abs_a = fabs(a), abs_b = fabs(b), abs_c = fabs(c);
    int proj;
    if (abs_a >= abs_b && abs_a >= abs_c) proj = 0;
    else if (abs_b >= abs_a && abs_b >= abs_c) proj = 1;
    else proj = 2;
    int nfj = faces->vertex_count[fj];
    /* Réserver le contour projeté commun aux deux résultats, puis projeter
     * fj sur son axe dominant. Un échec de réserve laisse deux résultats nuls. */
    int *vx = (int*)malloc(sizeof(int) * nfj);
    int *vy = (int*)malloc(sizeof(int) * nfj);
    if (!vx || !vy) { free(vx); free(vy); return 0; }
    for (int m = 0; m < nfj; ++m) {
        int vi = faces->vertex_indices_buffer[faces->vertex_indices_ptr[fj] + m] - 1;
        Fixed32 X = vtx->x[vi], Y = vtx->y[vi], Z = vtx->z[vi];
        int ix, iy;
        if (proj == 0) { ix = FIXED_TO_INT(Y); iy = FIXED_TO_INT(Z); }
        else if (proj == 1) { ix = FIXED_TO_INT(X); iy = FIXED_TO_INT(Z); }
        else { ix = FIXED_TO_INT(X); iy = FIXED_TO_INT(Y); }
        vx[m] = ix;
        vy[m] = iy;
    }
    /* Parcourir les segments de ei, évaluer les côtés et projeter leurs
     * intersections avec le plan.
     */
    for (int k = 0; k < cnt; ++k) {
        int vi = faces->vertex_indices_buffer[faces->vertex_indices_ptr[ei] + k] - 1;
        int vj = faces->vertex_indices_buffer[faces->vertex_indices_ptr[ei] + ((k + 1) % cnt)] - 1;
        double Ax = (double)vtx->x[vi], Ay = (double)vtx->y[vi], Az = (double)vtx->z[vi];
        double Bx = (double)vtx->x[vj], By = (double)vtx->y[vj], Bz = (double)vtx->z[vj];
        const double plane_eps = 0.02;
        double sdA = a*Ax + b*Ay + c*Az + d;
        double sdB = a*Bx + b*By + c*Bz + d;
        if (fabs(sdA) < plane_eps) sdA = 0.0;
        if (fabs(sdB) < plane_eps) sdB = 0.0;
        if (sdA == 0.0 && sdB == 0.0) {
            int pax, pay, pbx, pby;
            if (proj == 0) { pax = FIXED_TO_INT((Fixed32)Ay); pay = FIXED_TO_INT((Fixed32)Az);
                             pbx = FIXED_TO_INT((Fixed32)By); pby = FIXED_TO_INT((Fixed32)Bz); }
            else if (proj == 1) { pax = FIXED_TO_INT((Fixed32)Ax); pay = FIXED_TO_INT((Fixed32)Az);
                                  pbx = FIXED_TO_INT((Fixed32)Bx); pby = FIXED_TO_INT((Fixed32)Bz); }
            else { pax = FIXED_TO_INT((Fixed32)Ax); pay = FIXED_TO_INT((Fixed32)Ay);
                   pbx = FIXED_TO_INT((Fixed32)Bx); pby = FIXED_TO_INT((Fixed32)By); }
            if (point_in_poly_arrays_int(pax, pay, nfj, vx, vy) ||
                point_in_poly_arrays_int(pbx, pby, nfj, vx, vy)) {
                count++;
            }
            continue;
        }
        if (sdA * sdB < 0.0) {
            double t = -sdA / (sdB - sdA);
            double Px = Ax + t*(Bx - Ax);
            double Py = Ay + t*(By - Ay);
            double Pz = Az + t*(Bz - Az);
            int px, py;
            if (proj == 0) { px = FIXED_TO_INT((Fixed32)Py); py = FIXED_TO_INT((Fixed32)Pz); }
            else if (proj == 1) { px = FIXED_TO_INT((Fixed32)Px); py = FIXED_TO_INT((Fixed32)Pz); }
            else { px = FIXED_TO_INT((Fixed32)Px); py = FIXED_TO_INT((Fixed32)Py); }
            /* Le compteur conserve les contacts de bord ; le verdict strict
             * exclut ces contacts et ne refait plus ce test après un succès. */
            if (!*interior && strict_inside(px, py, nfj, vx, vy)) *interior = 1;
            if (point_in_poly_arrays_int(px, py, nfj, vx, vy)) {
                count++;
            }
        }
    }
    /* Libérer les deux contours temporaires avant de rendre le résultat. */
    free(vx); free(vy);
    return count;
}
static int split_face_by_plane(Model3D* model, int f1, int f2,
                               int *out_newverts, int *out_n,
                               int *constraint_verts, int constraint_n,
                               int force_one_edge) {
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int cnt = faces->vertex_count[f1];
    if (cnt < 3) return 0;

    double a, b, c, d;
    /* Calculer le plan de coupe dans les unités objet, puis réserver les
     * classifications.
     */
    compute_face_plane(model, f2, &a, &b, &c, &d, FACE_PLANE_OBJECT_UNITS);
    if (a == 0.0 && b == 0.0 && c == 0.0) return 0;
    const double plane_eps = 0.02;

    double *dist = (double*)malloc(sizeof(double) * cnt);
    int *orig = (int*)malloc(sizeof(int) * cnt);
    if (!dist || !orig) {
        free(dist);
        free(orig);
        return 0;
    }
    /* Évaluer la distance signée non normalisée de chaque sommet au plan. */
    for (int i = 0; i < cnt; ++i) {
        int vi = faces->vertex_indices_buffer[faces->vertex_indices_ptr[f1] + i] - 1;
        orig[i] = vi;
        double x = FIXED_TO_FLOAT(vtx->x[vi]);
        double y = FIXED_TO_FLOAT(vtx->y[vi]);
        double z = FIXED_TO_FLOAT(vtx->z[vi]);
        double sd = a * x + b * y + c * z + d;
        if (fabs(sd) < plane_eps) sd = 0.0;
        dist[i] = sd;
    }
    /* Vérifier si les deux côtés sont présents, sauf demande de poursuite forcée. */
    int hasPos = 0, hasNeg = 0;
    for (int i = 0; i < cnt; ++i) {
        if (dist[i] > 0.0) hasPos = 1;
        if (dist[i] < 0.0) hasNeg = 1;
    }
    if (!hasPos || !hasNeg) {
        if (!force_one_edge) {
            free(dist);
            free(orig);
            return 0;
        } else {

        }
    }
    int pos_list[MAX_FACE_VERTICES * 2];
    int neg_list[MAX_FACE_VERTICES * 2];
    int cop_list[MAX_FACE_VERTICES];
    int pos_n = 0, neg_n = 0, cop_n = 0;

    int newbuf[MAX_FACE_VERTICES];
    int newcount = 0;

    double seg_min = 0, seg_max = 0;
    int have_constraint = 0;
    double dirx=0, diry=0, dirz=0;
    /* Construire une direction d'intersection et les limites du segment de
     * contrainte facultatif.
     */
    if (constraint_verts && constraint_n >= 2) {

        double a1,b1,c1,d1;
        compute_face_plane(model, f1, &a1, &b1, &c1, &d1, FACE_PLANE_RAW_FIXED);

        dirx = b * c1 - c * b1;
        diry = c * a1 - a * c1;
        dirz = a * b1 - b * a1;
        double len = sqrt(dirx*dirx + diry*diry + dirz*dirz);
        if (len > 0) {
            dirx /= len; diry /= len; dirz /= len;

            double p0x = FIXED_TO_FLOAT(model->vertices.x[constraint_verts[0]]);
            double p0y = FIXED_TO_FLOAT(model->vertices.y[constraint_verts[0]]);
            double p0z = FIXED_TO_FLOAT(model->vertices.z[constraint_verts[0]]);
            seg_min = seg_max = 0.0;
            for (int ci = 0; ci < constraint_n; ++ci) {
                int vi = constraint_verts[ci];
                double cx = FIXED_TO_FLOAT(model->vertices.x[vi]);
                double cy = FIXED_TO_FLOAT(model->vertices.y[vi]);
                double cz = FIXED_TO_FLOAT(model->vertices.z[vi]);
                double t = ( (cx - p0x) * dirx + (cy - p0y) * diry + (cz - p0z) * dirz );
                if (ci == 0 || t < seg_min) seg_min = t;
                if (ci == 0 || t > seg_max) seg_max = t;
            }
            have_constraint = 1;
            if (have_constraint) {
            }
        }
    }
    for (int i = 0; i < cnt; ++i) {
        int j = (i + 1) % cnt;
        double di = dist[i], dj = dist[j];
        int vi = orig[i], vj = orig[j];
        if (di == 0.0) cop_list[cop_n++] = vi;
        if (di >= 0.0) pos_list[pos_n++] = vi;
        if (di <= 0.0) neg_list[neg_n++] = vi;
        /* Créer un sommet pour chaque arête qui change strictement de côté,
         * sous réserve de la contrainte.
         */
        if (di * dj < 0.0) {
            double t = di / (di - dj);
            double ix = FIXED_TO_FLOAT(vtx->x[vi]) + t * (FIXED_TO_FLOAT(vtx->x[vj]) - FIXED_TO_FLOAT(vtx->x[vi]));
            double iy = FIXED_TO_FLOAT(vtx->y[vi]) + t * (FIXED_TO_FLOAT(vtx->y[vj]) - FIXED_TO_FLOAT(vtx->y[vi]));
            double iz = FIXED_TO_FLOAT(vtx->z[vi]) + t * (FIXED_TO_FLOAT(vtx->z[vj]) - FIXED_TO_FLOAT(vtx->z[vi]));

            if (have_constraint) {
                int refvi = constraint_verts[0];
                double rpx = FIXED_TO_FLOAT(model->vertices.x[refvi]);
                double rpy = FIXED_TO_FLOAT(model->vertices.y[refvi]);
                double rpz = FIXED_TO_FLOAT(model->vertices.z[refvi]);
                double pt = ((ix - rpx) * dirx + (iy - rpy) * diry + (iz - rpz) * dirz);
                if (pt < seg_min - 1e-6 || pt > seg_max + 1e-6) {
                    continue;
                }
            }
            int newvi = add_vertex(model, FLOAT_TO_FIXED((float)ix),
                                           FLOAT_TO_FIXED((float)iy),
                                           FLOAT_TO_FIXED((float)iz));
            if (newvi >= 0) {
                pos_list[pos_n++] = newvi;
                neg_list[neg_n++] = newvi;
                if (out_newverts && newcount < MAX_FACE_VERTICES) {
                    newbuf[newcount++] = newvi;
                }
            }
        }
    }
    /* Publier les indices nouveaux puis libérer les tableaux de classement. */
    if (out_newverts && out_n) {
        *out_n = newcount;
        for (int k = 0; k < newcount; ++k) out_newverts[k] = newbuf[k];
    }
    free(dist);
    free(orig);
    /* Nettoyer les listes et réordonner les contours ; les tests d'aire
     * suivants ne rejettent pas effectivement les petites aires.
     */
    cleanup_list(pos_list, &pos_n);
    cleanup_list(neg_list, &neg_n);
    cleanup_list(cop_list, &cop_n);
    remove_duplicates(pos_list, &pos_n);
    remove_duplicates(neg_list, &neg_n);
    if (cop_n >= 3) remove_duplicates(cop_list, &cop_n);

    if (!force_one_edge && (pos_n < 3 || neg_n < 3)) return 0;

    reorder_poly(model, f1, pos_list, &pos_n);
    reorder_poly(model, f1, neg_list, &neg_n);

    double a1, b1, c1, d1;
    compute_face_plane(model, f1, &a1, &b1, &c1, &d1, FACE_PLANE_RAW_FIXED);
    double area_pos = poly_signed_area(model, pos_list, pos_n, a1, b1, c1);
    double area_neg = poly_signed_area(model, neg_list, neg_n, a1, b1, c1);
    if (fabs(area_pos) < 0.001 || fabs(area_neg) < 0.001) {

    }
    reorder_poly(model, f1, pos_list, &pos_n);
    reorder_poly(model, f1, neg_list, &neg_n);
    int orig_cnt = faces->vertex_count[f1];
    int orig_off = faces->vertex_indices_ptr[f1];

    int *keep_list = NULL, keep_n = 0;
    int *other_list = NULL, other_n = 0;
    if (pos_n >= 3 && (neg_n < 3 || !force_one_edge)) {
        keep_list = pos_list; keep_n = pos_n;
        other_list = neg_list; other_n = neg_n;
    } else if (neg_n >= 3 && (pos_n < 3 || force_one_edge && pos_n < 3)) {
        keep_list = neg_list; keep_n = neg_n;
        other_list = pos_list; other_n = pos_n;
    } else {

        keep_list = pos_list; keep_n = pos_n;
        other_list = neg_list; other_n = neg_n;
    }
    /* Réutiliser l'ancien emplacement s'il suffit, sinon ajouter une
     * nouvelle liste au buffer compact.
     */
    if (keep_n <= orig_cnt) {
        for (int k = 0; k < keep_n; ++k)
            faces->vertex_indices_buffer[orig_off + k] = keep_list[k] + 1;
        faces->vertex_count[f1] = keep_n;
    } else {
        int new_off = faces->total_indices;
        for (int k = 0; k < keep_n; ++k)
            faces->vertex_indices_buffer[new_off + k] = keep_list[k] + 1;
        faces->vertex_indices_ptr[f1] = new_off;
        faces->vertex_count[f1] = keep_n;
        faces->total_indices += keep_n;
    }
    /* Créer les autres faces lorsqu'elles possèdent au moins trois indices ;
     * conserver le drapeau d'affichage d'origine.
     */
    int orig_disp = faces->display_flag[f1];

    if (other_n >= 3) {
        int fneg = faces->face_count++;
        faces->vertex_count[fneg] = other_n;
        faces->vertex_indices_ptr[fneg] = faces->total_indices;
        for (int k = 0; k < other_n; ++k)
            faces->vertex_indices_buffer[faces->total_indices + k] = other_list[k] + 1;
        faces->total_indices += other_n;
        faces->sorted_face_indices[fneg] = fneg;
        faces->display_flag[fneg] = orig_disp;
    }
    if (cop_n >= 3) {
        int fcop = faces->face_count++;
        faces->vertex_count[fcop] = cop_n;
        faces->vertex_indices_ptr[fcop] = faces->total_indices;
        for (int k = 0; k < cop_n; ++k)
            faces->vertex_indices_buffer[faces->total_indices + k] = cop_list[k] + 1;
        faces->total_indices += cop_n;
        faces->sorted_face_indices[fcop] = fcop;
        faces->display_flag[fcop] = orig_disp;
    }
    /* Rafraîchir les bornes objet ; les données observateur restent à recalculer. */
    compute_face_bboxes3D(model);
    return 1;
}
static int add_vertex(Model3D* model, Fixed32 x, Fixed32 y, Fixed32 z) {
    /* Valider le modèle et la capacité avant l'écriture. */
    if (!model) return -1;
    VertexArrays3D* vtx = &model->vertices;
    if (vtx->vertex_count >= MAX_VERTICES) return -1;
    int idx = vtx->vertex_count;
    vtx->x[idx]  = x;
    vtx->y[idx]  = y;
    vtx->z[idx]  = z;
    /* Invalider les projections et marquer les coordonnées dérivées comme à
     * recalculer.
     */
    fractional_projection_cache_invalidate(model);
    vtx->xo[idx] = 0;
    vtx->yo[idx] = 0;
    vtx->zo[idx] = 0;
    vtx->x2d[idx] = 0;
    vtx->y2d[idx] = 0;
    vtx->vertex_count++;
    return idx;
}
static void cleanup_list(int *arr, int *n) {
    if (*n <= 1) return;
    int w = 0;
    /* Conserver la première occurrence de chaque série adjacente. */
    for (int i = 0; i < *n; ++i) {
        if (i == 0 || arr[i] != arr[i-1]) {
            arr[w++] = arr[i];
        }
    }
    /* Retirer une fermeture explicite du cycle et publier la nouvelle longueur. */
    if (w > 1 && arr[0] == arr[w-1]) w--;
    *n = w;
}
static void remove_duplicates(int *arr, int *n) {
    int w = 0;
    /* Rechercher chaque valeur dans le préfixe déjà conservé avant de la recopier. */
    for (int i = 0; i < *n; ++i) {
        int found = 0;
        for (int j = 0; j < w; ++j) {
            if (arr[i] == arr[j]) { found = 1; break; }
        }
        if (!found) arr[w++] = arr[i];
    }
    *n = w;
}
static double poly_signed_area(Model3D* model, int *list, int n,
                               double a, double b, double c) {
    if (n < 3) return 0.0;
    /* Éliminer la composante de normale de plus grande amplitude pour
     * choisir la projection.
     */
    double abs_a = fabs(a), abs_b = fabs(b), abs_c = fabs(c);
    int proj;
    if (abs_a >= abs_b && abs_a >= abs_c) proj = 0;
    else if (abs_b >= abs_a && abs_b >= abs_c) proj = 1;
    else proj = 2;
    VertexArrays3D* vtx = &model->vertices;
    double area = 0;
    /* Appliquer la somme cyclique des déterminants puis diviser par deux. */
    for (int i = 0; i < n; ++i) {
        int vi = list[i];
        int vj = list[(i + 1) % n];
        double x1,y1,x2,y2;
        if (proj == 0) {
            x1 = FIXED_TO_FLOAT(vtx->y[vi]); y1 = FIXED_TO_FLOAT(vtx->z[vi]);
            x2 = FIXED_TO_FLOAT(vtx->y[vj]); y2 = FIXED_TO_FLOAT(vtx->z[vj]);
        } else if (proj == 1) {
            x1 = FIXED_TO_FLOAT(vtx->x[vi]); y1 = FIXED_TO_FLOAT(vtx->z[vi]);
            x2 = FIXED_TO_FLOAT(vtx->x[vj]); y2 = FIXED_TO_FLOAT(vtx->z[vj]);
        } else {
            x1 = FIXED_TO_FLOAT(vtx->x[vi]); y1 = FIXED_TO_FLOAT(vtx->y[vi]);
            x2 = FIXED_TO_FLOAT(vtx->x[vj]); y2 = FIXED_TO_FLOAT(vtx->y[vj]);
        }
        area += (x1 * y2 - x2 * y1);
    }
    return area * 0.5;
}
static void reorder_poly(Model3D* model, int fidx, int *list, int *n) {
    if (!model || *n < 3) return;

    double a, b, c, d;
    compute_face_plane(model, fidx, &a, &b, &c, &d, FACE_PLANE_RAW_FIXED);
    /* Choisir une projection 2D du plan de référence. */
    double abs_a = fabs(a), abs_b = fabs(b), abs_c = fabs(c);
    int proj;
    if (abs_a >= abs_b && abs_a >= abs_c) proj = 0;
    else if (abs_b >= abs_a && abs_b >= abs_c) proj = 1;
    else proj = 2;

    /* Réserver les coordonnées projetées ; abandonner sans changer la liste
     * si la réserve échoue.
     */
    double *px = (double*)malloc(sizeof(double) * (*n));
    double *py = (double*)malloc(sizeof(double) * (*n));
    if (!px || !py) {
        free(px); free(py);
        return;
    }
    VertexArrays3D* vtx = &model->vertices;
    for (int i = 0; i < *n; ++i) {
        int vi = list[i];
        double X = (double)vtx->x[vi];
        double Y = (double)vtx->y[vi];
        double Z = (double)vtx->z[vi];
        if (proj == 0) { px[i] = Y; py[i] = Z; }
        else if (proj == 1) { px[i] = X; py[i] = Z; }
        else { px[i] = X; py[i] = Y; }
    }

    /* Calculer le centre moyen puis l'angle de chaque sommet. */
    double cx = 0, cy = 0;
    for (int i = 0; i < *n; ++i) { cx += px[i]; cy += py[i]; }
    cx /= *n; cy /= *n;

    double *ang = (double*)malloc(sizeof(double) * (*n));
    if (!ang) { free(px); free(py); return; }
    for (int i = 0; i < *n; ++i) {
        ang[i] = atan2(py[i] - cy, px[i] - cx);
    }

    /* Trier les indices par angle croissant. */
    for (int i = 1; i < *n; ++i) {
        int idx = list[i];
        double a0 = ang[i];
        int j = i - 1;
        while (j >= 0 && ang[j] > a0) {
            ang[j+1] = ang[j];
            list[j+1] = list[j];
            j--;
        }
        ang[j+1] = a0;
        list[j+1] = idx;
    }

    /* Évaluer le signe du contour px/py d'origine et inverser la liste si ce
     * signe est négatif.
     */
    double area = 0;
    for (int i = 0; i < *n; ++i) {
        int k = (i + 1) % *n;
        double x1 = px[i], y1 = py[i];
        double x2 = px[k], y2 = py[k];
        area += (x1 * y2 - x2 * y1);
    }
    if (area < 0) {

        for (int i = 0; i < *n/2; ++i) {
            int t = list[i]; list[i] = list[*n-1-i]; list[*n-1-i] = t;
        }
    }
    /* Libérer les trois tableaux temporaires. */
    free(px); free(py); free(ang);
}
static void compute_face_bboxes3D(Model3D* model) {
    if (!model) return;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int fc = faces->face_count;
    if (fc <= 0) return;

    /* Libérer les anciennes bornes avant de réserver un jeu dimensionné au
     * nombre courant de faces.
     */
    if (faces->minx3) free(faces->minx3);
    if (faces->maxx3) free(faces->maxx3);
    if (faces->miny3) free(faces->miny3);
    if (faces->maxy3) free(faces->maxy3);
    if (faces->minz3) free(faces->minz3);
    if (faces->maxz3) free(faces->maxz3);
    faces->minx3 = (Fixed32*)malloc(fc * sizeof(Fixed32));
    faces->maxx3 = (Fixed32*)malloc(fc * sizeof(Fixed32));
    faces->miny3 = (Fixed32*)malloc(fc * sizeof(Fixed32));
    faces->maxy3 = (Fixed32*)malloc(fc * sizeof(Fixed32));
    faces->minz3 = (Fixed32*)malloc(fc * sizeof(Fixed32));
    faces->maxz3 = (Fixed32*)malloc(fc * sizeof(Fixed32));
    /* Ramener les six pointeurs à NULL si une allocation a échoué. */
    if (!faces->minx3||!faces->maxx3||!faces->miny3||!faces->maxy3||!faces->minz3||!faces->maxz3) {

        free(faces->minx3); free(faces->maxx3); free(faces->miny3);
        free(faces->maxy3); free(faces->minz3); free(faces->maxz3);
        faces->minx3 = faces->maxx3 = faces->miny3 = faces->maxy3 = faces->minz3 = faces->maxz3 = NULL;
        return;
    }
    /* Parcourir les sommets de chaque face ; une face vide conserve six
     * bornes nulles.
     */
    for (int fi = 0; fi < fc; ++fi) {
        int cnt = faces->vertex_count[fi];
        int off = faces->vertex_indices_ptr[fi];
        Fixed32 mnx=0,mxx=0,mny=0,mxy=0,mnz=0,mxz=0;
        for (int k = 0; k < cnt; ++k) {
            int vi = faces->vertex_indices_buffer[off + k] - 1;
            Fixed32 x = vtx->x[vi];
            Fixed32 y = vtx->y[vi];
            Fixed32 z = vtx->z[vi];
            if (k==0 || x < mnx) mnx = x;
            if (k==0 || x > mxx) mxx = x;
            if (k==0 || y < mny) mny = y;
            if (k==0 || y > mxy) mxy = y;
            if (k==0 || z < mnz) mnz = z;
            if (k==0 || z > mxz) mxz = z;
        }
        faces->minx3[fi] = mnx;
        faces->maxx3[fi] = mxx;
        faces->miny3[fi] = mny;
        faces->maxy3[fi] = mxy;
        faces->minz3[fi] = mnz;
        faces->maxz3[fi] = mxz;
    }
}
void calculateFaceDepths(Model3D* model, Face3D* faces, int face_count) {
    /* Changer la génération des résultats géométriques avant le parcours. */
    ordering_cache_invalidate(model);
    int i, j;
    int culled_count = 0;
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* face_arrays = &model->faces;
#if defined(FACE_DEPTH_USE_ASM)
    FaceDepth_ScanSetup(vtx);
#endif
    /* Parcourir chaque liste d'indices, avec des résultats de même format
     * pour C et assembleur.
     */
    for (i = 0; i < face_count; i++) {
        Fixed32 z_min,z_max,sum;
        int display_flag,minx,maxx,miny,maxy,valid_n;
        int n=face_arrays->vertex_count[i];
        int offset=face_arrays->vertex_indices_ptr[i];
        Fixed64 a64,b64,c64,d64,sumx64,sumy64,sumz64;
        Fixed32 xc,yc,zc;
/* Récupérer en bloc les métriques du noyau assembleur. */
#if defined(FACE_DEPTH_USE_ASM)
        FaceDepth_ScanFace(face_arrays,offset,n);
        valid_n=face_depth_result.valid_n;display_flag=face_depth_result.display_flag;
        z_min=face_depth_result.z_min;z_max=face_depth_result.z_max;sum=face_depth_result.sum;
        minx=face_depth_result.minx;maxx=face_depth_result.maxx;
        miny=face_depth_result.miny;maxy=face_depth_result.maxy;
        sumx64=face_depth_result.sumx;sumy64=face_depth_result.sumy;sumz64=face_depth_result.sumz;
        a64=face_depth_result.a;b64=face_depth_result.b;c64=face_depth_result.c;
#else
        Fixed32 x_prev=0,y_prev=0,z_prev=0;
        Fixed32 x_first=0,y_first=0,z_first=0;
        int have_prev=0;
        z_min=FLOAT_TO_FIXED(9999.0);z_max=FLOAT_TO_FIXED(-9999.0);
        display_flag=1;sum=0;valid_n=0;
        minx=miny=9999;maxx=maxy=-9999;
        a64=b64=c64=0;sumx64=sumy64=sumz64=0;
        /* Chemin C : ignorer les indices négatifs après passage à base zéro
         * et cumuler profondeur, bornes et coordonnées.
         */
        for (j = 0; j < n; j++) {
            int vertex_idx = face_arrays->vertex_indices_buffer[offset + j] - 1;
            if (vertex_idx >= 0) {
                Fixed32 zo = vtx->zo[vertex_idx];
                Fixed32 xo = vtx->xo[vertex_idx];
                Fixed32 yo = vtx->yo[vertex_idx];
                if (zo < 0) display_flag = 0;
                if (zo < z_min) z_min = zo;
                if (zo > z_max) z_max = zo;
                sum += zo;
                int x2d = vtx->x2d[vertex_idx];
                int y2d = vtx->y2d[vertex_idx];
                if (x2d < minx) minx = x2d;
                if (x2d > maxx) maxx = x2d;
                if (y2d < miny) miny = y2d;
                if (y2d > maxy) maxy = y2d;

                sumx64 += (Fixed64)xo;
                sumy64 += (Fixed64)yo;
                sumz64 += (Fixed64)zo;
                /* Initialiser le contour au premier sommet valide ; les
                 * suivants contribuent à Newell.
                 */
                if (!have_prev) {
                    x_first = xo; y_first = yo; z_first = zo;
                    x_prev  = xo; y_prev  = yo; z_prev  = zo;
                    have_prev = 1;
                } else {
                    Fixed32 dy = FIXED_SUB(y_prev, yo);
                    Fixed32 dz = FIXED_SUB(z_prev, zo);
                    Fixed32 dx = FIXED_SUB(x_prev, xo);
                    Fixed32 sy = y_prev + yo;
                    Fixed32 sz = z_prev + zo;
                    Fixed32 sx = x_prev + xo;
                    a64 += fixed_product32_fast(dy,sz);
                    b64 += fixed_product32_fast(dz,sx);
                    c64 += fixed_product32_fast(dx,sy);
                    x_prev = xo; y_prev = yo; z_prev = zo;
                }
                valid_n++;
            }
        }
#endif
        /* Neutraliser les plans des faces derrière la caméra ou
         * insuffisamment définies.
         */
        if (!display_flag || n < 3 || valid_n < 3) {

            face_arrays->plane_a[i] = 0;
            face_arrays->plane_b[i] = 0;
            face_arrays->plane_c[i] = 0;
            face_arrays->plane_d[i] = 0;
            if (shaded_by_orientation) {
                face_shade_color[i] = COL_FILL_DEFAULT;
            }
        } else {
/* Fermer le cycle de Newell et réduire la virgule fixe seulement après la
 * somme complète.
 */
#if !defined(FACE_DEPTH_USE_ASM)

            Fixed32 dy = FIXED_SUB(y_prev, y_first);
            Fixed32 dz = FIXED_SUB(z_prev, z_first);
            Fixed32 dx = FIXED_SUB(x_prev, x_first);
            Fixed32 sy = y_prev + y_first;
            Fixed32 sz = z_prev + z_first;
            Fixed32 sx = x_prev + x_first;
            a64 += fixed_product32_fast(dy,sz);
            b64 += fixed_product32_fast(dz,sx);
            c64 += fixed_product32_fast(dx,sy);
            a64 >>= FIXED_SHIFT;
            b64 >>= FIXED_SHIFT;
            c64 >>= FIXED_SHIFT;
#endif
#if defined(FACE_DEPTH_USE_ASM)
            xc=FaceDepth_CentroidCoordinate(sumx64,valid_n);
            yc=FaceDepth_CentroidCoordinate(sumy64,valid_n);
            zc=FaceDepth_CentroidCoordinate(sumz64,valid_n);
#else
            xc = (Fixed32)(sumx64 / valid_n);
            yc = (Fixed32)(sumy64 / valid_n);
            zc = (Fixed32)(sumz64 / valid_n);
#endif
            /* Calculer le terme d à partir du centroïde puis publier les coefficients. */
            d64 = -(fixed_product64x32_fast(a64,xc) + fixed_product64x32_fast(b64,yc)
                   + fixed_product64x32_fast(c64,zc));
            d64 >>= FIXED_SHIFT;
            face_arrays->plane_a[i] = a64;
            face_arrays->plane_b[i] = b64;
            face_arrays->plane_c[i] = c64;
            face_arrays->plane_d[i] = d64;

            /* Appliquer le culling observateur au signe de d, sans modifier la géométrie. */
            if (cull_back_faces && display_flag) {
                if (d64 <= 0) {
                    display_flag = 0;
                    ++culled_count;
                }
            }
        }
        /* Publier les profondeurs et la visibilité ; les faces vides
         * reçoivent des métriques nulles.
         */
        face_arrays->z_min[i] = z_min;
        face_arrays->z_max[i] = z_max;
        face_arrays->display_flag[i] = display_flag;
        if (n > 0) {
            if (n == 4)      face_arrays->z_mean[i] = sum >> 2;
            else if (n == 3) face_arrays->z_mean[i] = sum / 3;
            else             face_arrays->z_mean[i] = sum / n;
            face_arrays->minx[i] = minx;
            face_arrays->maxx[i] = maxx;
            face_arrays->miny[i] = miny;
            face_arrays->maxy[i] = maxy;
        } else {
            face_arrays->z_mean[i] = 0;
            face_arrays->z_min[i] = 0;
            face_arrays->z_max[i] = 0;
            face_arrays->minx[i] = 0;
            face_arrays->maxx[i] = 0;
            face_arrays->miny[i] = 0;
            face_arrays->maxy[i] = 0;
        }
    }
}
void normalizeAutoFitDistanceTo150(Model3D* model) {
    if (model == NULL) return;
    /* Vérifier les deux suggestions avant le calcul du facteur. */
    Fixed32 D = model->auto_suggested_distance;
    Fixed32 P = model->auto_suggested_proj_scale;
    if (D <= 0 || P <= 0) return;
    const Fixed32 targetD = INT_TO_FIXED(150);

    /* Calculer le facteur racine puis le convertir en 16.16. */
    float factor_f = sqrtf(FIXED_TO_FLOAT(targetD) / FIXED_TO_FLOAT(D));
    if (factor_f <= 0.0f) return;
    Fixed32 factor = FLOAT_TO_FIXED(factor_f);

    VertexArrays3D* vtx = &model->vertices;
    /* Appliquer le facteur aux trois coordonnées objet. */
    for (int i = 0; i < vtx->vertex_count; ++i) {
        vtx->x[i] = FIXED_MUL_64(vtx->x[i], factor);
        vtx->y[i] = FIXED_MUL_64(vtx->y[i], factor);
        vtx->z[i] = FIXED_MUL_64(vtx->z[i], factor);
    }

    /* Synchroniser distance suggérée, projection globale et distance de tolérance. */
    model->auto_suggested_distance = targetD;
    model->auto_suggested_proj_scale = FIXED_MUL_64(P, factor);
    model->auto_proj_scale = model->auto_suggested_proj_scale;
    s_global_proj_scale_fixed = model->auto_proj_scale;

    current_observer_distance = targetD;
    printf("Auto-fit normalized: D=%.4f P=%.2f factor=%.4f\n",
        FIXED_TO_FLOAT(model->auto_suggested_distance),
        FIXED_TO_FLOAT(model->auto_suggested_proj_scale),
        FIXED_TO_FLOAT(factor));
}
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
void processModelFast(Model3D* model, ObserverParams* params, const char* filename) {
    /* Invalider les rapports de projection avant de remplacer les
     * coordonnées observateur.
     */
    fractional_projection_cache_invalidate(model);
    int i;
    Fixed32 cos_h, sin_h, cos_v, sin_v, cos_w, sin_w;
    Fixed32 x, y, z, zo, xo, yo;
    Fixed32 inv_zo;

    /* Lire les six valeurs trigonométriques une seule fois pour la vue. */
    cos_h = cos_deg_int(params->angle_h);
    sin_h = sin_deg_int(params->angle_h);
    cos_v = cos_deg_int(params->angle_v);
    sin_v = sin_deg_int(params->angle_v);
    cos_w = cos_deg_int(params->angle_w);
    sin_w = sin_deg_int(params->angle_w);

    /* Précalculer les produits de la matrice en 16.16. */
    const Fixed32 cos_h_cos_v = fixed_mul32_fast(cos_h, cos_v);
    const Fixed32 sin_h_cos_v = fixed_mul32_fast(sin_h, cos_v);
    const Fixed32 cos_h_sin_v = fixed_mul32_fast(cos_h, sin_v);
    const Fixed32 sin_h_sin_v = fixed_mul32_fast(sin_h, sin_v);

    Fixed32 M00, M01, M02;
    Fixed32 M10, M11, M12;
    Fixed32 M20, M21, M22;

    Fixed32 scale = s_global_proj_scale_fixed;
    const Fixed32 centre_x_f = INT_TO_FIXED(CENTRE_X);
    const Fixed32 centre_y_f = INT_TO_FIXED(CENTRE_Y);
    Fixed32 distance = params->distance;

    VertexArrays3D* vtx = &model->vertices;
    Fixed32 *x_arr = vtx->x, *y_arr = vtx->y, *z_arr = vtx->z;
    Fixed32 *xo_arr = vtx->xo, *yo_arr = vtx->yo, *zo_arr = vtx->zo;
    int *x2d_arr = vtx->x2d, *y2d_arr = vtx->y2d;
    int vcount = vtx->vertex_count;

    /* Construire la matrice ; le cas angle_w nul évite les produits de
     * rotation écran.
     */
    M20 = FIXED_NEG(cos_h_cos_v);
    M21 = FIXED_NEG(sin_h_cos_v);
    M22 = FIXED_NEG(sin_v);
    if (params->angle_w == 0) {
        M00 = FIXED_NEG(sin_h);
        M01 = cos_h;
        M02 = 0;
        M10 = FIXED_NEG(cos_h_sin_v);
        M11 = FIXED_NEG(sin_h_sin_v);
        M12 = cos_v;
    } else {
        M00 = FIXED_ADD(fixed_mul32_fast(FIXED_NEG(cos_w), sin_h),
                        fixed_mul32_fast(sin_w, cos_h_sin_v));
        M01 = FIXED_SUB(fixed_mul32_fast(cos_w, cos_h),
                        fixed_mul32_fast(sin_w, FIXED_NEG(sin_h_sin_v)));
        M02 = fixed_mul32_fast(FIXED_NEG(sin_w), cos_v);
        M10 = FIXED_ADD(fixed_mul32_fast(FIXED_NEG(sin_w), sin_h),
                        fixed_mul32_fast(FIXED_NEG(cos_w), cos_h_sin_v));
        M11 = FIXED_ADD(fixed_mul32_fast(sin_w, cos_h),
                        fixed_mul32_fast(FIXED_NEG(cos_w), sin_h_sin_v));
        M12 = fixed_mul32_fast(cos_w, cos_v);
    }
    /* Transformer chaque sommet et appliquer la perspective seulement devant
     * l'observateur.
     */
    for (i = 0; i < vcount; i++) {
        x = x_arr[i];
        y = y_arr[i];
        z = z_arr[i];
        zo = FIXED_ADD(
                FIXED_ADD(
                    FIXED_ADD(fixed_mul32_fast(M20, x),
                            fixed_mul32_fast(M21, y)),
                    fixed_mul32_fast(M22, z)),
                distance);
        if (zo > 0) {
            xo = FIXED_ADD(FIXED_ADD(fixed_mul32_fast(M00, x),
                                    fixed_mul32_fast(M01, y)),
                        fixed_mul32_fast(M02, z));
            yo = FIXED_ADD(FIXED_ADD(fixed_mul32_fast(M10, x),
                                    fixed_mul32_fast(M11, y)),
                        fixed_mul32_fast(M12, z));
            zo_arr[i] = zo;
            xo_arr[i] = xo;
            yo_arr[i] = yo;
            inv_zo = FIXED_DIV_64(scale, zo);
            x2d_arr[i] = FIXED_ROUND_TO_INT(
                            FIXED_ADD(fixed_mul32_fast(xo, inv_zo), centre_x_f));
            y2d_arr[i] = FIXED_ROUND_TO_INT(
                            FIXED_SUB(centre_y_f, fixed_mul32_fast(yo, inv_zo)));
        } else {
            zo_arr[i] = zo;
            xo_arr[i] = 0;
            yo_arr[i] = 0;
            x2d_arr[i] = -1;
            y2d_arr[i] = -1;
        }
    }

    /* Recalculer plans et visibilité, puis les couleurs par orientation si
     * demandées.
     */
    calculateFaceDepths(model, NULL, model->faces.face_count);
    if (shaded_by_orientation) {
        computeOrientationShading(model);
    }
    /* Le filaire ne nécessite pas l'ordre des surfaces ; conserver le tri
     * pour les autres modes.
     */
    if (framePolyOnly) goto skip_calc;

    skip_calc:;
}static void fractional_projection_cached(Model3D *model, int v, double scale,
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
static int segs_intersect_int(int x1,int y1,int x2,int y2,int x3,int y3,int x4,int y4) {
    /* Déléguer intégralement au prédicat 64 bits pour conserver cette
     * interface historique.
     */
    return segs_intersect_int_fixed64(x1,y1,x2,y2,x3,y3,x4,y4);
}
static int dony_ray_pairs_reserve(DonyRayPairs *cache, int nf)
{
    unsigned int *a, *b, *next;
    /* Réutiliser la capacité existante ; sinon allouer un nouveau jeu
     * complet avant de remplacer l'ancien.
     */
    if (nf <= cache->capacity) return 1;
    a = (unsigned int*)malloc((size_t)nf * sizeof(unsigned int));
    b = (unsigned int*)malloc((size_t)nf * sizeof(unsigned int));
    next = (unsigned int*)malloc((size_t)nf * sizeof(unsigned int));
    /* Libérer seulement les nouvelles allocations en cas d'échec. */
    if (!a || !b || !next) {
        free(a); free(b); free(next);
        return 0;
    }
    free(cache->first); free(cache->second); free(cache->next);
    cache->first = a; cache->second = b; cache->next = next;
    cache->capacity = nf;
    cache->count = 0;
    memset(cache->stamps, 0, sizeof(cache->stamps));
    return 1;
}
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
static int ray_cast_at(Model3D* model, int f1, int f2, int cx, int cy) {
    float tf1 = 0.0f, tf2 = 0.0f;
    /* Exiger les deux intersections puis comparer leurs paramètres sans
     * tolérance additionnelle.
     */
    if (!ray_cast_distances(model, f1, f2, cx, cy, &tf1, &tf2)) return RAY_ORDER_UNDETERMINED;
    if (tf1 > tf2) return RAY_FIRST_FARTHER;
    else if (tf1 < tf2) return RAY_FIRST_NEARER;
    else return RAY_ORDER_UNDETERMINED;
}
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
static int geov3_run(Model3D* model, int face_count, int preserve_order,
                     GeoV3ExchangeCallback on_exchange, void *context)
{
    if (!model || face_count <= 0) return 0;
    FaceArrays3D* faces = &model->faces;
    int* sorted = faces->sorted_face_indices;

    static unsigned int* dd_flag = NULL;
    static DonyRayPairs ray_pairs;
    static int allocated = 0;
    static unsigned int generation = 0;

    int nf = face_count;
    int fs = 0;
    int exchanges = 0;


    /* Invalider explicitement les relations préparées avant le tri de cette vue. */
    (void)geometric_face_relation(NULL, 0, 0);

    /* Agrandir les buffers persistants de sélection et de prévention des cycles. */
    if (face_count > allocated) {
        free(dd_flag);

        dd_flag = (unsigned int*)calloc((size_t)face_count, sizeof(unsigned int));

        if (!dd_flag || !dony_ray_pairs_reserve(&ray_pairs, face_count)) {
            fprintf(stderr, "painter_geoV3: allocation failed\n");
            if (preserve_order) {
                free(dd_flag); dd_flag = NULL; allocated = 0;
                memset(ray_pairs.stamps, 0, sizeof(ray_pairs.stamps));
                return -1;
            }
            exit(1);
        }

        allocated = face_count;
        generation = 0;
    }


    /* Repartir de l'ordre canonique pour que les égalités ne dépendent pas
     * de la vue précédente.
     */
    if (!preserve_order) {
        for (int i = 0; i < nf; ++i) sorted[i] = i;
        sort_by_zmax(sorted, faces->z_max, nf);
    }
    /* Le filtrage parcourt la permutation choisie, sans supposer que le
     * repair a préalablement trié les profondeurs.
     */
#if !defined(GEOV3_DISABLE_ASM_SCAN)
    GeoV3_ScanSetup(faces, sorted, nf);
#endif

    /* Fixer successivement la prochaine face à dessiner au début de la
     * partie non traitée.
     */
    while (fs < nf - 1) {
        int p;
        int f2;
        int permutes_this_pass;


        /* Attribuer un stamp à la passe ; après rebouclage, effacer toute la
         * capacité avant de réutiliser 1.
         */
        ++generation;
        if (generation == 0) {
            memset(dd_flag, 0, (size_t)allocated * sizeof(unsigned int));
            memset(ray_pairs.stamps, 0, sizeof(ray_pairs.stamps));
            generation = 1;
        }

        ray_pairs.count = 0;
        p = sorted[fs];
        f2 = fs + 1;
        permutes_this_pass = 0;

        /* Comparer P aux candidats restants ; un échange impose de
         * recommencer la sélection.
         */
        while (f2 < nf) {
            int q;
#if !defined(GEOV3_DISABLE_ASM_SCAN)
            f2 = GeoV3_FindCandidate(faces, p, f2);
            if (f2 >= nf) break;
#endif
            q = sorted[f2];

#if defined(GEOV3_DISABLE_ASM_SCAN)


            /* Chemin C optionnel : éliminer les intervalles de profondeur séparés. */
            if (faces->z_max[q] <= faces->z_min[p])
                goto skip_q;


            {
                /* Rejeter les bornes X sans recouvrement strict, avec les
                 * soustractions natives 16 bits.
                 */
                int dx;

                dx = (faces->maxx[p] > faces->maxx[q])
                   ? faces->minx[p] - faces->maxx[q]
                   : faces->minx[q] - faces->maxx[p];

                if (dx >= 0)
                    goto skip_q;
            }


            {
                /* Appliquer le même rejet aux bornes Y. */
                int dy;

                dy = (faces->maxy[p] > faces->maxy[q])
                   ? faces->miny[p] - faces->maxy[q]
                   : faces->miny[q] - faces->maxy[p];

                if (dy >= 0)
                    goto skip_q;
            }

#endif


            {
                int geo_pq;
                int geo_qp;

                /* Tester d'abord P/Q ; le sens inverse est inutile lorsque
                 * le premier résultat valide déjà P avant Q.
                 */
                geo_pq = geometric_face_relation(model, p, q);


                if (geo_pq == -1)
                    goto skip_q;

                geo_qp = geometric_face_relation(model, q, p);


                if (geo_qp == 1)
                    goto skip_q;


                /* Si la géométrie impose Q avant P, échanger sauf si Q a
                 * déjà été écartée pendant cette passe.
                 */
                if (geo_pq == 1 || geo_qp == -1) {
                    if (dd_flag[q] != generation)
                        goto do_permute;

                    goto do_split_giveup;
                }


                goto do_split_giveup;
            }

/* Limiter les permutations pour garantir la terminaison et reprendre avec le
 * nouveau P.
 */
do_permute:
            /* Des fragments coplanaires peuvent être classés de part et
             * d’autre après quantification. Ne pas échanger des contours
             * sans aire commune : cet échange consommerait inutilement les
             * protections anti-cycle et pourrait bloquer une vraie correction.
             * Le repair applique désormais exactement le même filtre ici. */
            if (!projected_polygons_overlap_cached(model, p, q))
                goto skip_q;

            if (++permutes_this_pass > face_count)
                goto do_split_giveup;

            /* Proposer exactement l'échange décidé par les tests de plans. */
            if (on_exchange) {
                int decision = on_exchange(model, p, q, fs, f2,
                                           GEOV3_EXCHANGE_PLANES, context);
                if (decision < 0) return exchanges;
                if (!decision) goto skip_q;
            }
            sorted[fs]=q;
            sorted[f2]=p;
            ++exchanges;


            dd_flag[p] = generation;

            p = q;
            f2 = fs + 1;

            continue;

/* Remplacer le découpage absent par chevauchement et ray casting, sans
 * rééchanger une paire déjà mémorisée.
 */
do_split_giveup:


            if (!dony_ray_pair_seen(&ray_pairs, p, q, generation) &&
                projected_polygons_overlap_cached(model, p, q)) {
                int rc;

                rc = ray_cast_hierarchical(model, p, q);


                /* Un rayon impose Q avant P ; enregistrer la paire avant de
                 * redémarrer les comparaisons.
                 */
                if (rc < 0) {
                    if (++permutes_this_pass > face_count)
                        goto skip_q;

                    /* Même confirmation pour le repli par rayon hiérarchique. */
                    if (on_exchange) {
                        int decision = on_exchange(model, p, q, fs, f2,
                                                   GEOV3_EXCHANGE_RAY, context);
                        if (decision < 0) return exchanges;
                        if (!decision) goto skip_q;
                    }
                    dony_ray_pair_record(&ray_pairs, p, q, generation);

                    sorted[fs]=q;
                    sorted[f2]=p;
                    ++exchanges;

                    dd_flag[p] = generation;


                    p = q;
                    f2 = fs + 1;

                    continue;
                }


                if (rc > 0)
                    goto skip_q;
            }

/* Avancer après une paire acceptée ou sans solution ; terminer la passe à
 * épuisement des candidats.
 */
skip_q:
            ++f2;
        }

        ++fs;
    }
    return exchanges;
}
static int Previous_geov3_run(Model3D* model, int face_count, int preserve_order,

                     GeoV3ExchangeCallback on_exchange, void *context)

{

    if (!model || face_count <= 0) return 0;

    FaceArrays3D* faces = &model->faces;

    int* sorted = faces->sorted_face_indices;



    static unsigned int* dd_flag = NULL;

    static DonyRayPairs ray_pairs;

    static int allocated = 0;

    static unsigned int generation = 0;



    int nf = face_count;

    int fs = 0;

    int exchanges = 0;





    /* Invalider explicitement les relations préparées avant le tri de cette vue. */

    (void)geometric_face_relation(NULL, 0, 0);



    /* Agrandir les buffers persistants de sélection et de prévention des cycles. */

    if (face_count > allocated) {

        free(dd_flag);



        dd_flag = (unsigned int*)calloc((size_t)face_count, sizeof(unsigned int));



        if (!dd_flag || !dony_ray_pairs_reserve(&ray_pairs, face_count)) {

            fprintf(stderr, "painter_geoV3: allocation failed\n");

            if (preserve_order) {

                free(dd_flag); dd_flag = NULL; allocated = 0;

                memset(ray_pairs.stamps, 0, sizeof(ray_pairs.stamps));

                return -1;

            }

            exit(1);

        }



        allocated = face_count;

        generation = 0;

    }





    /* Repartir de l'ordre canonique pour que les égalités ne dépendent pas

     * de la vue précédente.

     */

    if (!preserve_order) {

        for (int i = 0; i < nf; ++i) sorted[i] = i;

        sort_by_zmax(sorted, faces->z_max, nf);

    }

    /* Le filtrage parcourt la permutation choisie, sans supposer que le

     * repair a préalablement trié les profondeurs.

     */

#if !defined(GEOV3_DISABLE_ASM_SCAN)

    GeoV3_ScanSetup(faces, sorted, nf);

#endif



    /* Fixer successivement la prochaine face à dessiner au début de la

     * partie non traitée.

     */

    while (fs < nf - 1) {

        int p;

        int f2;

        int permutes_this_pass;





        /* Attribuer un stamp à la passe ; après rebouclage, effacer toute la

         * capacité avant de réutiliser 1.

         */

        ++generation;

        if (generation == 0) {

            memset(dd_flag, 0, (size_t)allocated * sizeof(unsigned int));

            memset(ray_pairs.stamps, 0, sizeof(ray_pairs.stamps));

            generation = 1;

        }



        ray_pairs.count = 0;

        p = sorted[fs];

        f2 = fs + 1;

        permutes_this_pass = 0;



        /* Comparer P aux candidats restants ; un échange impose de

         * recommencer la sélection.

         */

        while (f2 < nf) {

            int q;

#if !defined(GEOV3_DISABLE_ASM_SCAN)

            f2 = GeoV3_FindCandidate(faces, p, f2);

            if (f2 >= nf) break;

#endif

            q = sorted[f2];



#if defined(GEOV3_DISABLE_ASM_SCAN)





            /* Chemin C optionnel : éliminer les intervalles de profondeur séparés. */

            if (faces->z_max[q] <= faces->z_min[p])

                goto skip_q;





            {

                /* Rejeter les bornes X sans recouvrement strict, avec les

                 * soustractions natives 16 bits.

                 */

                int dx;



                dx = (faces->maxx[p] > faces->maxx[q])

                   ? faces->minx[p] - faces->maxx[q]

                   : faces->minx[q] - faces->maxx[p];



                if (dx >= 0)

                    goto skip_q;

            }





            {

                /* Appliquer le même rejet aux bornes Y. */

                int dy;



                dy = (faces->maxy[p] > faces->maxy[q])

                   ? faces->miny[p] - faces->maxy[q]

                   : faces->miny[q] - faces->maxy[p];



                if (dy >= 0)

                    goto skip_q;

            }



#endif





            {

                int geo_pq;

                int geo_qp;



                /* Tester d'abord P/Q ; le sens inverse est inutile lorsque

                 * le premier résultat valide déjà P avant Q.

                 */

                geo_pq = geometric_face_relation(model, p, q);





                if (geo_pq == -1)

                    goto skip_q;



                geo_qp = geometric_face_relation(model, q, p);





                if (geo_qp == 1)

                    goto skip_q;





                /* Si la géométrie impose Q avant P, échanger sauf si Q a

                 * déjà été écartée pendant cette passe.

                 */

                if (geo_pq == 1 || geo_qp == -1) {

                    if (dd_flag[q] != generation)

                        goto do_permute;



                    goto do_split_giveup;

                }





                goto do_split_giveup;

            }



/* Limiter les permutations pour garantir la terminaison et reprendre avec le

 * nouveau P.

 */

do_permute:





            if (++permutes_this_pass > face_count)

                goto do_split_giveup;



            /* Proposer exactement l'échange décidé par les tests de plans. */

            if (on_exchange) {

                int decision = on_exchange(model, p, q, fs, f2,

                                           GEOV3_EXCHANGE_PLANES, context);

                if (decision < 0) return exchanges;

                if (!decision) goto skip_q;

            }

            sorted[fs]=q;

            sorted[f2]=p;

            ++exchanges;





            dd_flag[p] = generation;



            p = q;

            f2 = fs + 1;



            continue;



/* Remplacer le découpage absent par chevauchement et ray casting, sans

 * rééchanger une paire déjà mémorisée.

 */

do_split_giveup:





            if (!dony_ray_pair_seen(&ray_pairs, p, q, generation) &&

                projected_polygons_overlap_cached(model, p, q)) {

                int rc;



                rc = ray_cast_hierarchical(model, p, q);





                /* Un rayon impose Q avant P ; enregistrer la paire avant de

                 * redémarrer les comparaisons.

                 */

                if (rc < 0) {

                    if (++permutes_this_pass > face_count)

                        goto skip_q;



                    /* Même confirmation pour le repli par rayon hiérarchique. */

                    if (on_exchange) {

                        int decision = on_exchange(model, p, q, fs, f2,

                                                   GEOV3_EXCHANGE_RAY, context);

                        if (decision < 0) return exchanges;

                        if (!decision) goto skip_q;

                    }

                    dony_ray_pair_record(&ray_pairs, p, q, generation);



                    sorted[fs]=q;

                    sorted[f2]=p;

                    ++exchanges;



                    dd_flag[p] = generation;





                    p = q;

                    f2 = fs + 1;



                    continue;

                }





                if (rc > 0)

                    goto skip_q;

            }



/* Avancer après une paire acceptée ou sans solution ; terminer la passe à

 * épuisement des candidats.

 */

skip_q:

            ++f2;

        }



        ++fs;

    }

    return exchanges;

}
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
static void dony_ray_pair_record(DonyRayPairs *cache, int p, int q,
                                 unsigned int generation)
{
    unsigned int a=(unsigned int)p,b=(unsigned int)q,bucket;
    if(a>b){unsigned int swap=a;a=b;b=swap;}
    bucket=(((a << 5)+a)^b) & (DONY_RAY_PAIR_BUCKETS-1);
    /* Ajouter l'entrée en tête de chaîne ; la capacité est contrôlée par
     * l'appelant.
     */
    unsigned int i = cache->count++;

    cache->first[i] = a;
    cache->second[i] = b;
    cache->next[i] = (cache->stamps[bucket] == generation) ? cache->heads[bucket] : 0;
    cache->heads[bucket] = i + 1;
    cache->stamps[bucket] = generation;
}
static void rch_patch_pair_at_scale(Model3D* model, int f1, int f2, Fixed32 test_scale, RCHZoomPatch* p) {
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    const Fixed32 cx_f = INT_TO_FIXED(CENTRE_X);
    const Fixed32 cy_f = INT_TO_FIXED(CENTRE_Y);

    /* Sauvegarder échelle et bornes avant toute modification. */
    p->saved_scale = s_global_proj_scale_fixed;
    s_global_proj_scale_fixed = test_scale;

    p->f1_minx = faces->minx[f1]; p->f1_maxx = faces->maxx[f1];
    p->f1_miny = faces->miny[f1]; p->f1_maxy = faces->maxy[f1];
    p->f2_minx = faces->minx[f2]; p->f2_maxx = faces->maxx[f2];
    p->f2_miny = faces->miny[f2]; p->f2_maxy = faces->maxy[f2];

    /* Rassembler les sommets distincts et conserver leurs coordonnées projetées. */
    p->count = rch_gather_pair_vertex_indices(faces, f1, f2, p->idx);

    for (int i = 0; i < p->count; ++i) {
        p->saved_x2d[i] = vtx->x2d[p->idx[i]];
        p->saved_y2d[i] = vtx->y2d[p->idx[i]];
    }

    /* Reprojeter la paire à la nouvelle échelle puis recalculer uniquement
     * ses boîtes.
     */
    for (int i = 0; i < p->count; ++i) {
        int vidx = p->idx[i];
        Fixed32 inv_zo = FIXED_DIV_64(test_scale, vtx->zo[vidx]);
        Fixed32 xt = FIXED_ADD(FIXED_MUL_64(vtx->xo[vidx], inv_zo), cx_f);
        Fixed32 yt = FIXED_SUB(cy_f, FIXED_MUL_64(vtx->yo[vidx], inv_zo));
        vtx->x2d[vidx] = FIXED_ROUND_TO_INT(xt);
        vtx->y2d[vidx] = FIXED_ROUND_TO_INT(yt);
    }

    rch_recompute_face_bbox_cache(model, f1);
    rch_recompute_face_bbox_cache(model, f2);
}
static void rch_unpatch_pair(Model3D* model, RCHZoomPatch* p, int f1, int f2) {
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    /* Restituer d'abord les coordonnées, puis les deux boîtes et l'échelle. */
    for (int i = 0; i < p->count; ++i) {
        vtx->x2d[p->idx[i]] = p->saved_x2d[i];
        vtx->y2d[p->idx[i]] = p->saved_y2d[i];
    }
    faces->minx[f1] = p->f1_minx; faces->maxx[f1] = p->f1_maxx;
    faces->miny[f1] = p->f1_miny; faces->maxy[f1] = p->f1_maxy;
    faces->minx[f2] = p->f2_minx; faces->maxx[f2] = p->f2_maxx;
    faces->miny[f2] = p->f2_miny; faces->maxy[f2] = p->f2_maxy;
    s_global_proj_scale_fixed = p->saved_scale;
}
static int on_seg_ll(int ax,int ay,int bx,int by,int cx,int cy) {
    /* Comparer les coordonnées aux deux intervalles fermés indépendamment du
     * sens du segment.
     */
    return ((ax<=cx && cx<=bx)||(bx<=cx && cx<=ax)) &&
           ((ay<=cy && cy<=by)||(by<=cy && cy<=ay));
}
static int ray_cast_hierarchical(Model3D* model, int f1, int f2) {
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    if (f1 < 0 || f2 < 0 || f1 >= faces->face_count || f2 >= faces->face_count) return 0;
    RCHZoomPatch patch;
    Fixed32 original_scale = s_global_proj_scale_fixed;
    /* Chercher une relation pour la paire orientée à l'échelle d'origine. */
    OrderPairEntry *entry = ordering_pair_slot(model, order_ray_pairs, f1, f2);
    ORDER_COUNT(ray_calls);
    if (ordering_pair_hit(entry, f1, f2, original_scale, 0)) {
        ORDER_COUNT(ray_hits);
        return entry->result;
    }

    Fixed32 test_scale = FIXED_MUL_64(original_scale, FLOAT_TO_FIXED(10.0f));
    double fx, fy, fractional_area2;
    int rc = 0;
    /* Grossir le calcul fractionnaire sans modifier les coordonnées écran
     * entières.
     */
    s_global_proj_scale_fixed = test_scale;
    if (compute_intersection_centroid_fractional_tryboth(
            model, f1, f2, &fx, &fy, &fractional_area2)) {
        rc = ray_cast_at_fractional(model, f1, f2, fx, fy);
    }
    s_global_proj_scale_fixed = original_scale;
    /* Mémoriser uniquement le résultat fractionnaire concluant. */
    if (rc != 0) {
        ordering_pair_store(entry, f1, f2, original_scale, 0, rc);
        return rc;
    }

    /* En dernier recours, sauvegarder/reprojeter la paire puis restaurer
     * impérativement son état.
     */
    rch_patch_pair_at_scale(model, f1, f2, test_scale, &patch);
    rc = ray_cast_hierarchical_impl(model, f1, f2);
    rch_unpatch_pair(model, &patch, f1, f2);
    return rc;
}
static long long orient_ll(int ax,int ay,int bx,int by,int cx,int cy) {
    /* Former les différences en long avant les deux produits 32x32. */
    return fixed_product32_fast((long)bx-ax,(long)cy-ay)
         - fixed_product32_fast((long)by-ay,(long)cx-ax);
}
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
static int ray_cast_at_fractional(Model3D *model, int f1, int f2,
                                  double cx, double cy)
{
    FaceArrays3D *faces;
    double scale, dx, dy, denominator1, denominator2, t1, t2;
    double fallback1[4], fallback2[4];
    double *plane1, *plane2;
    /* Valider modèle, identifiants et projection avant la construction du rayon. */
    if (!model) return RAY_ORDER_UNDETERMINED;
    faces = &model->faces;
    if (f1 < 0 || f2 < 0 || f1 >= faces->face_count || f2 >= faces->face_count) return RAY_ORDER_UNDETERMINED;
    scale = (double)s_global_proj_scale_fixed / (double)FIXED_SCALE;
    if (scale <= 0.0) return RAY_ORDER_UNDETERMINED;
    dx = (cx - (double)CENTRE_X) / scale;
    dy = ((double)CENTRE_Y - cy) / scale;
    /* Évaluer les deux plans en double avec le même rayon. */
    plane1 = ordering_plane(model, f1, fallback1);
    denominator1 = plane1[0] * dx + plane1[1] * dy + plane1[2];
    if (fabs(denominator1) < 1e-6) return RAY_ORDER_UNDETERMINED;
    plane2 = ordering_plane(model, f2, fallback2);
    denominator2 = plane2[0] * dx + plane2[1] * dy + plane2[2];
    if (fabs(denominator2) < 1e-6) return RAY_ORDER_UNDETERMINED;
    /* Comparer les paramètres des intersections calculées. */
    t1 = -plane1[3] / denominator1;
    t2 = -plane2[3] / denominator2;
    if (t1 > t2) return RAY_FIRST_FARTHER;
    if (t1 < t2) return RAY_FIRST_NEARER;
    return RAY_ORDER_UNDETERMINED;
}
static int move_element_remove_and_insert(int *arr, int n, int from, int insert_idx) {
    /* Valider la source, borner la destination et éviter les déplacements nuls. */
    if (from < 0 || from >= n) return 0;
    if (insert_idx < 0) insert_idx = 0;
    if (insert_idx > n-1) insert_idx = n-1;
    if (from == insert_idx) return 0;
    int val = arr[from];
    /* Décaler le segment dans le sens approprié puis réinsérer la valeur
     * sauvegardée.
     */
    if (insert_idx < from) {

        memmove(&arr[insert_idx+1], &arr[insert_idx], (from - insert_idx) * sizeof(int));
        arr[insert_idx] = val;
    } else {

        memmove(&arr[from], &arr[from+1], (insert_idx - from) * sizeof(int));
        arr[insert_idx] = val;
    }
    return 1;
}
static int ray_cast_hierarchical_impl(Model3D* model, int f1, int f2)
{
    int cx = 0, cy = 0;
    long long area2 = 0;
    int rc = 0;
    int fallback = 0;

    if (!model) return 0;

    /* Essayer le centroïde du clipping numérique puis le rayon à son point entier. */
    if (compute_intersection_centroid_ordered_fixed_tryboth(
            model, f1, f2, &cx, &cy, &area2, &fallback) && area2 != 0) {
        rc = ray_cast_at(model, f1, f2, cx, cy);
        if (rc != 0) return rc;
    }

    area2 = 0;
    /* Replier sur la région QuickDraw si le premier essai ne tranche pas. */
    if (compute_intersection_centroid_ordered_qd_fixed(
            model, f1, f2, &cx, &cy, &area2) && area2 != 0) {
        rc = ray_cast_at(model, f1, f2, cx, cy);
        if (rc != 0) return rc;
    }

    /* Dernier repli : utiliser le centre des bornes communes, sans garantie
     * sur les contours.
     */
    if (compute_bbox_intersection_center(model, f1, f2, &cx, &cy)) {
        return ray_cast_at(model, f1, f2, cx, cy);
    }

    return 0;
}
static void rch_recompute_face_bbox_cache(Model3D* model, int f) {
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int base = faces->vertex_indices_ptr[f], vc = faces->vertex_count[f];
    /* Initialiser les quatre bornes avec le premier sommet, qui doit exister. */
    int v0 = faces->vertex_indices_buffer[base] - 1;
    int mnx = vtx->x2d[v0], mxx = vtx->x2d[v0];
    int mny = vtx->y2d[v0], mxy = vtx->y2d[v0];
    /* Étendre la boîte avec les autres sommets puis écrire les quatre valeurs. */
    for (int k = 1; k < vc; ++k) {
        int vidx = faces->vertex_indices_buffer[base + k] - 1;
        int x = vtx->x2d[vidx], y = vtx->y2d[vidx];
        if (x < mnx) mnx = x; if (x > mxx) mxx = x;
        if (y < mny) mny = y; if (y > mxy) mxy = y;
    }
    faces->minx[f] = mnx; faces->maxx[f] = mxx;
    faces->miny[f] = mny; faces->maxy[f] = mxy;
}
static int ray_cast_distances(Model3D* model, int f1, int f2, int cx, int cy, float *out_tf1, float *out_tf2) {
    if (!model || !out_tf1 || !out_tf2) return 0;
    FaceArrays3D* faces = &model->faces;
    if (f1 < 0 || f2 < 0 || f1 >= faces->face_count || f2 >= faces->face_count) return 0;
    double fallback1[4],fallback2[4];
    double *plane1,*plane2;
    /* Construire la direction écran avec la projection globale. */
    float proj_scale = FIXED_TO_FLOAT(s_global_proj_scale_fixed);
    float Dx = ((float)cx - (float)CENTRE_X) / proj_scale;
    float Dy = ((float)CENTRE_Y - (float)cy) / proj_scale;

    /* Récupérer les plans puis vérifier les dénominateurs avant les deux
     * divisions.
     */
    plane1=ordering_plane(model,f1,fallback1);
    float A1 = (float)plane1[0];
    float B1 = (float)plane1[1];
    float C1 = (float)plane1[2];
    float denom1 = A1 * Dx + B1 * Dy + C1;
    if (fabsf(denom1) < 1e-6f) return 0;

    plane2=ordering_plane(model,f2,fallback2);
    float A2 = (float)plane2[0];
    float B2 = (float)plane2[1];
    float C2 = (float)plane2[2];
    float denom2 = A2 * Dx + B2 * Dy + C2;
    if (fabsf(denom2) < 1e-6f) return 0;

    float D1 = (float)plane1[3];
    float D2 = (float)plane2[3];
    /* Publier les deux paramètres du rayon seulement si les calculs sont
     * possibles.
     */
    *out_tf1 = -D1 / denom1;
    *out_tf2 = -D2 / denom2;
    return 1;
}
static void sort_by_zmax(int *sorted,Fixed32 *z_max,int nf)
{
    int i;
    /* Étendre le préfixe trié sans déplacer les faces de profondeur égale. */
    for(i=1;i<nf;++i){
        int key=sorted[i];
        Fixed32 depth=z_max[key];
        int lo,hi;
        if(z_max[sorted[i-1]]>=depth)continue;
        lo=0;hi=i;
        /* Chercher la position après les clés égales afin de préserver la stabilité. */
        while(lo<hi){
            int mid=lo+(hi-lo)/2;

            if(z_max[sorted[mid]]>=depth)lo=mid+1;
            else hi=mid;
        }
        /* Déplacer la clé par la même primitive stable que les autres listes. */
        (void)move_element_remove_and_insert(sorted, nf, i, lo);
    }
}
static int dony_ray_pair_seen(DonyRayPairs *cache, int p, int q,
                              unsigned int generation)
{
    unsigned int a=(unsigned int)p,b=(unsigned int)q,bucket;
    /* Normaliser la paire pour que (p,q) et (q,p) partagent la même clé. */
    if(a>b){unsigned int swap=a;a=b;b=swap;}
    bucket=(((a << 5)+a)^b) & (DONY_RAY_PAIR_BUCKETS-1);
    unsigned int entry;
    /* Ignorer les buckets d'une autre passe, puis comparer les clés complètes. */
    if (cache->stamps[bucket] != generation) return 0;
    entry = cache->heads[bucket];
    while (entry) {
        unsigned int i = entry - 1;
        if (cache->first[i] == a && cache->second[i] == b) return 1;
        entry = cache->next[i];
    }
    return 0;
}
static int geometric_face_relation(Model3D* model, int f1, int f2)
{
    static int cache_f1[256];
    static int cache_f2[256];
    static signed char cache_result[256];
    static unsigned char cache_valid[256];
    static Fixed64 cached_epsilon = 0;
    static Fixed64 cached_distance = 0;
    static int epsilon_valid = 0;
    static Model3D *cache_model=NULL;
    static unsigned int cache_generation=0,cache_epoch=0;

    OrderFixedPlane *prepared;
    FaceArrays3D* faces;
    VertexArrays3D* vtx;
    Fixed64 epsilon;
    Fixed64 a1, b1, c1, d1;
    Fixed64 acc;
    unsigned slot;
    int obs;
    int n2, offset2;
    int all_same, all_opp;
    int k, v, side;
    int result;
    unsigned char *dense_entry;
    unsigned int dense_shift,dense_code;

    /* Traiter la demande explicite d'invalidation sans accéder au modèle. */
    if (model == NULL) {
        ordering_fixed_plane_reset();
        memset(cache_valid, 0, sizeof(cache_valid));
        return 0;
    }

    /* Éliminer les résultats d'un autre propriétaire, d'une autre vue ou
     * d'une autre époque.
     */
    if(order_cache_owner!=model)ordering_cache_invalidate(model);
    if(cache_model!=model || cache_generation!=order_cache_generation || cache_epoch!=order_fixed_plane_epoch){
        memset(cache_valid,0,sizeof(cache_valid));
        cache_model=model;cache_generation=order_cache_generation;cache_epoch=order_fixed_plane_epoch;
    }
    /* Recalculer epsilon uniquement si la distance change, et invalider les
     * relations locales.
     */
    if (!epsilon_valid ||
        current_observer_distance != cached_distance) {
        cached_epsilon = FIXED_MUL_64(
            current_observer_distance,
            FLOAT_TO_FIXED(0.001f));
        cached_distance = current_observer_distance;
        epsilon_valid = 1;
        memset(cache_valid, 0, sizeof(cache_valid));
    }

    /* Essayer le cache dense puis le hash à clé exacte. */
    dense_entry=ordering_geometry_slot(model,f1,f2,cached_epsilon);
    dense_shift=((unsigned int)f2&3U)<<1;
    if(dense_entry){
        dense_code=(*dense_entry>>dense_shift)&3U;
        if(dense_code)return (int)dense_code-2;
    }
    slot = (((unsigned)f1 * 33u) ^ (unsigned)f2) & 255u;
    if (cache_valid[slot] &&
        cache_f1[slot] == f1 &&
        cache_f2[slot] == f2) {
        return (int)cache_result[slot];
    }

    faces = &model->faces;
    vtx = &model->vertices;
    epsilon = cached_epsilon;

    /* Préparer les coefficients et le côté observateur ; un observateur dans
     * la tolérance ne tranche pas.
     */
    prepared=ordering_fixed_plane(model,f1,epsilon);
    result = FACE_ORDER_UNDETERMINED;
    obs=prepared->observer;
    if (!obs) goto store_result;

/* N'utiliser le noyau assembleur que si les bornes arithmétiques et le
 * nombre de sommets l'autorisent.
 */
#if !defined(GEOV3_DISABLE_ASM_MUL) && !defined(GEOV3_DISABLE_ASM_RELATION)
    if (prepared->asm_ready &&
        faces->vertex_count[f2] >= 0 &&
        faces->vertex_count[f2] <= MAX_FACE_VERTICES) {
        result = GeoRel_ClassifyPrepared(model,f2,prepared);
        goto store_result;
    }
#endif

    a1=prepared->a;b1=prepared->b;c1=prepared->c;d1=prepared->d;
    n2 = faces->vertex_count[f2];
    offset2 = faces->vertex_indices_ptr[f2];
    all_same = 1;
    all_opp = 1;

    /* Chemin général : évaluer les trois produits décalés puis éliminer les
     * hypothèses de côté.
     */
    for (k = 0; k < n2; ++k) {
        v = faces->vertex_indices_buffer[offset2 + k] - 1;

        acc = GeoV3_MulFixed64Fixed32Shift(a1, vtx->xo[v])
            + GeoV3_MulFixed64Fixed32Shift(b1, vtx->yo[v])
            + GeoV3_MulFixed64Fixed32Shift(c1, vtx->zo[v])
            + d1;

        if (acc > epsilon)
            side = 1;
        else if (acc < -epsilon)
            side = -1;
        else
            continue;

        if (side == obs)
            all_opp = 0;
        else
            all_same = 0;

        if (!all_same && !all_opp)
            goto store_result;
    }

    if (all_same)
        result = FACE_ORDER_BEFORE;
    else if (all_opp)
        result = FACE_ORDER_AFTER;

/* Publier le résultat dans les caches de la paire orientée. */
store_result:
    cache_f1[slot] = f1;
    cache_f2[slot] = f2;
    cache_result[slot] = (signed char)result;
    cache_valid[slot] = 1;
    if(dense_entry)*dense_entry|=(unsigned char)((unsigned int)(result+2)<<dense_shift);
    return result;
}
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
static int rch_gather_pair_vertex_indices(FaceArrays3D* faces, int f1, int f2, int* out_idx) {
    int n = 0;

    int base1 = faces->vertex_indices_ptr[f1], vc1 = faces->vertex_count[f1];
    /* Ajouter d'abord les sommets distincts de f1. */
    for (int k = 0; k < vc1; ++k) {
        int vidx = faces->vertex_indices_buffer[base1 + k] - 1;
        int dup = 0;
        for (int m = 0; m < n; ++m) if (out_idx[m] == vidx) { dup = 1; break; }
        if (!dup) out_idx[n++] = vidx;
    }

    /* Compléter avec ceux de f2 qui ne sont pas encore présents. */
    int base2 = faces->vertex_indices_ptr[f2], vc2 = faces->vertex_count[f2];
    for (int k = 0; k < vc2; ++k) {
        int vidx = faces->vertex_indices_buffer[base2 + k] - 1;
        int dup = 0;
        for (int m = 0; m < n; ++m) if (out_idx[m] == vidx) { dup = 1; break; }
        if (!dup) out_idx[n++] = vidx;
    }
    return n;
}
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
static int compute_bbox_intersection_center(Model3D* model, int f1, int f2, int *outx, int *outy) {
    if (!model || !outx || !outy) return 0;
    int ix0, iy0, ix1, iy1;
    /* Exiger un rectangle commun avant de moyenner ses bornes. */
    if (!compute_bbox_intersection(model, f1, f2, &ix0, &iy0, &ix1, &iy1)) return 0;
    *outx = (ix0 + ix1) / 2;
    *outy = (iy0 + iy1) / 2;
    return 1;
}
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
static int projected_polygons_overlap(Model3D*m,int a,int b){return projected_polygons_overlap_v1(m,a,b);}


#include <time.h>
static uint32_t rng=0x6581627u;
static uint32_t random_word(void){rng=rng*1664525u+1013904223u;return rng;}
static uint16_t image_old[64000],image_new[64000];
static void paint(Model3D*m,const int*order,uint16_t*image){
 memset(image,0,64000*sizeof(uint16_t));
 for(int oi=0;oi<m->faces.face_count;oi++){
  int f=order[oi],n=m->faces.vertex_count[f],off=m->faces.vertex_indices_ptr[f];
  if(!m->faces.display_flag[f]||n<3)continue;
  for(int y=0;y<200;y++){
   if(y<m->faces.miny[f]||y>m->faces.maxy[f])continue;
   double hits[32];int nh=0;
   for(int k=0;k<n;k++){
    int a=m->faces.vertex_indices_buffer[off+k]-1,b=m->faces.vertex_indices_buffer[off+(k+1)%n]-1;
    int ya=m->vertices.y2d[a],yb=m->vertices.y2d[b];double scan=y+0.5;
    if((ya>scan)==(yb>scan))continue;
    double x=m->vertices.x2d[a]+(scan-ya)*(m->vertices.x2d[b]-m->vertices.x2d[a])/(double)(yb-ya);
    int i=nh++;while(i&&hits[i-1]>x){hits[i]=hits[i-1];i--;}hits[i]=x;
   }
   for(int k=0;k+1<nh;k+=2){int a=(int)ceil(hits[k]-0.5),b=(int)ceil(hits[k+1]-0.5)-1;if(a<0)a=0;if(b>319)b=319;for(int x=a;x<=b;x++)image[y*320+x]=(uint16_t)(f+1);}
  }
 }
}
static double depth(Model3D*m,int face,int x,int y){
 double scale=(double)s_global_proj_scale_fixed/65536.0;
 double a=m->faces.plane_a[face],b=m->faces.plane_b[face],c=m->faces.plane_c[face],d=m->faces.plane_d[face];
 double den=a*(x+0.5-160)/scale+b*(100-y-0.5)/scale+c;
 if(fabs(den)<1e-15)return -1;return -d/den;
}
int main(int argc,char**argv){
 int views=argc>1?atoi(argv[1]):20;
 const char*models[]={"a","cube.obj","cylinder.obj","capsule.obj","jet.obj","star.obj","stool.obj","tore.obj","ring.obj","monkey.obj","other01.obj","other02.obj","other03.obj","gear2.obj","horn.obj","barbar.obj","charpente.obj","cross.obj","intersect2.obj","intersect3.obj","intersect.obj"};
 for(unsigned mi=0;mi<sizeof(models)/sizeof(models[0]);mi++){
  char path[256];snprintf(path,sizeof(path),"%s/%s",argc>2?argv[2]:"models",models[mi]);Model3D*m=createModel3D();m->vertices.vertex_count=readVertices(path,&m->vertices,6000,m);if(m->vertices.vertex_count<=0)return 2;readVertices_last_count=m->vertices.vertex_count;readFaces_model(path,m);
  if(mi>=18)check_intersect(m);
  s_global_proj_scale_fixed=m->auto_suggested_proj_scale;normalizeAutoFitDistanceTo150(m);
  int *before=malloc(sizeof(int)*m->faces.face_count),*after=malloc(sizeof(int)*m->faces.face_count);
  for(int vi=0;vi<views;vi++){
   ObserverParams p;
   p.angle_h=vi?random_word()%360:150;p.angle_v=vi?(int)(random_word()%151)-75:20;p.angle_w=vi?(int)(random_word()%361)-180:0;p.distance=150*65536;
   for(int cul=0;cul<2;cul++){
    cull_back_faces=cul;processModelFast(m,&p,"");
    qd_calls=0;clock_t start=clock();int no=Previous_geov3_run(m,m->faces.face_count,0,NULL,NULL);clock_t oldend=clock();int oqd=qd_calls;memcpy(before,m->faces.sorted_face_indices,sizeof(int)*m->faces.face_count);
    processModelFast(m,&p,"");qd_calls=0;clock_t newstart=clock();int nn=geov3_run(m,m->faces.face_count,0,NULL,NULL);clock_t newend=clock();int nqd=qd_calls;memcpy(after,m->faces.sorted_face_indices,sizeof(int)*m->faces.face_count);
    paint(m,before,image_old);paint(m,after,image_new);int pixels=0,better=0,worse=0,ties=0,invalid=0,orderdiff=0;
    for(int i=0;i<m->faces.face_count;i++)orderdiff+=before[i]!=after[i];
    for(int i=0;i<64000;i++)if(image_old[i]!=image_new[i]){
     pixels++;int a=image_old[i]-1,b=image_new[i]-1;
     if(a<0||b<0){invalid++;continue;}
     double da=depth(m,a,i%320,i/320),db=depth(m,b,i%320,i/320),tol=1e-5*fmax(1.0,fmax(fabs(da),fabs(db)));
     if(da<=0||db<=0){invalid++;continue;}
     if(db<da-tol)better++;else if(db>da+tol)worse++;else ties++;
    }
    printf("RESULT {\"model\":\"%s\",\"vertices\":%d,\"faces\":%d,\"h\":%d,\"v\":%d,\"w\":%d,\"culling\":%d,\"order_positions_changed\":%d,\"pixels_changed\":%d,\"closer\":%d,\"farther\":%d,\"near_equal\":%d,\"invalid_depth\":%d,\"old_qd_fallbacks\":%d,\"new_qd_fallbacks\":%d,\"old_exchanges\":%d,\"new_exchanges\":%d,\"old_host_seconds\":%.6f,\"new_host_seconds\":%.6f}\n",models[mi],m->vertices.vertex_count,m->faces.face_count,p.angle_h,p.angle_v,p.angle_w,cul,orderdiff,pixels,better,worse,ties,invalid,oqd,nqd,no,nn,(double)(oldend-start)/CLOCKS_PER_SEC,(double)(newend-newstart)/CLOCKS_PER_SEC);fflush(stdout);
   }
  }
  free(before);free(after);destroyModel3D(m);
 }
 return 0;
}
