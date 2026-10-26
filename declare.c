/*
 * ============================================================================
 *  declare.c — Déclarations communes et représentation des données
 * ============================================================================
 *
 *  Rôle :
 *    Définir les types, formats numériques, limites et états partagés de
 *    l'explorateur. Les coordonnées objet et observateur utilisent des
 *    valeurs signées 16.16 ; les produits et sommes étendus emploient
 *    Fixed64.
 *
 *  Organisation :
 *    Regroupe les inclusions Toolbox, les constantes et tables
 *    trigonométriques, les tableaux parallèles de sommets et de faces, les
 *    options globales, les utilitaires de palette et les prototypes.
 *
 *  Dépendances et contraintes :
 *    Inclus en tête de engine.c. Les tailles natives attendues sont celles
 *    d'ORCA/C : int sur 16 bits et long sur 32 bits. Les indices OBJ stockés
 *    sont à base un ; les faces et les permutations sont à base zéro. Les
 *    prototypes historiques ne prouvent pas qu'un chemin est appelé.
 *
 * ============================================================================
 */
#include <stdio.h>
#include <asm.h>
#include <string.h>
#include <misctool.h>
#include <stdlib.h>
#include <math.h>
#include <quickdraw.h>
#include <event.h>
#include <memory.h>
#include <window.h>
#include <orca.h>
#include <stdint.h>
#include <GSOS.h>
#include <texttool.h>
#include <misctool.h>

segment "data";

/* Format signé 16.16 sur long 32 bits pour ORCA/C ; Fixed64 sert aux
 * produits et accumulations 64 bits.
 */
typedef long        Fixed32;
typedef long long   Fixed64;

#define FIXED_SHIFT     16
#define FIXED_SCALE     (1L << FIXED_SHIFT)

#define FIXED_HALF      (FIXED_SCALE >> 1)

#define FIXED_ONE       FIXED_SCALE

#define INT_TO_FIXED(x)     ((Fixed32)(x) << FIXED_SHIFT)
#define FIXED_TO_INT(x)     ((int)((x) >> FIXED_SHIFT))
#define FLOAT_TO_FIXED(x)   ((Fixed32)((x) * FIXED_SCALE))
#define FIXED_TO_FLOAT(x)   ((float)(x) / (float)FIXED_SCALE)
#define FIXED64_TO_FLOAT(x) ((double)(x) / (double)FIXED_SCALE)

/*
 * ----------------------------------------------------------------------------
 *  FIXED_ROUND_TO_INT
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Convertir un Fixed32 en entier en appliquant le biais de demi-unité du
 *    moteur.
 *
 *  Paramètres :
 *    [entrée] x, valeur signée 16.16.
 *
 *  Retour :
 *    (x+32768)>>16 si x >= 0, (x-32768)>>16 sinon.
 *
 *  Fonctionnement :
 *    Aucun effet de bord. Le cas négatif suit le décalage signé du
 *    compilateur ; ne pas l'assimiler à un arrondi symétrique au plus proche
 *    ni à une troncature vers zéro.
 *
 * ----------------------------------------------------------------------------
 */
static inline int FIXED_ROUND_TO_INT(Fixed32 x) {
    /* Choisir le biais selon le signe ; le décalage négatif fait partie du
     * comportement existant.
     */
    if (x >= 0) return (int)(((x) + FIXED_HALF) >> FIXED_SHIFT);
    else        return (int)(((x) - FIXED_HALF) >> FIXED_SHIFT);
}

#define FIXED_ADD(a, b)     ((a) + (b))
#define FIXED_SUB(a, b)     ((a) - (b))
#define FIXED_NEG(x)        (-(x))

/* Ces macros utilisent seulement long : préférer les variantes 64 bits si le
 * produit ou le décalage peut dépasser 32 bits.
 */

#define FIXED_MUL_64(a, b)  ((Fixed32)(((Fixed64)(a) * (Fixed64)(b)) >> FIXED_SHIFT))
#define FIXED_DIV_64(a, b)  ((Fixed32)(((Fixed64)(a) << FIXED_SHIFT) / (Fixed64)(b)))

/* Échantillons trigonométriques d'un tour, un degré par entrée, amplitude
 * 65536.
 */
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

/*
 * ----------------------------------------------------------------------------
 *  sin_deg_int
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Lire le sinus d'un angle entier dans la table 16.16.
 *
 *  Paramètres :
 *    [entrée] deg, degrés éventuellement négatifs ou hors tour.
 *
 *  Retour :
 *    Sinus Fixed32.
 *
 *  Fonctionnement :
 *    Ramène l'angle dans [0,359] ; aucune interpolation et aucun effet de
 *    bord.
 *
 * ----------------------------------------------------------------------------
 */
static inline Fixed32 sin_deg_int(int deg) {
    deg %= 360;
    if (deg < 0) deg += 360;
    /* Lire directement la valeur tabulée après réduction modulo un tour. */
    return sin_table[deg];
}

/*
 * ----------------------------------------------------------------------------
 *  cos_deg_int
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Lire le cosinus d'un angle entier dans la table 16.16.
 *
 *  Paramètres :
 *    [entrée] deg, degrés éventuellement négatifs ou hors tour.
 *
 *  Retour :
 *    Cosinus Fixed32.
 *
 *  Fonctionnement :
 *    Ramène l'angle dans [0,359] ; aucune interpolation et aucun effet de
 *    bord.
 *
 * ----------------------------------------------------------------------------
 */
static inline Fixed32 cos_deg_int(int deg) {
    deg %= 360;
    if (deg < 0) deg += 360;
    /* Lire directement la valeur tabulée après réduction modulo un tour. */
    return cos_table[deg];
}

/*
 * ----------------------------------------------------------------------------
 *  normalize_deg
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Normaliser un angle entier en degrés.
 *
 *  Paramètres :
 *    [entrée] deg.
 *
 *  Retour :
 *    Représentant dans [0,359] par modulo corrigé si négatif.
 *
 *  Fonctionnement :
 *    Aucun effet de bord ; ne modifie pas l'argument de l'appelant.
 *
 * ----------------------------------------------------------------------------
 */
static inline int normalize_deg(int deg) {
    deg %= 360;
    if (deg < 0) deg += 360;
    /* Retourner l'angle normalisé sans modifier l'état de l'observateur. */
    return deg;
}

#define MAX_LINE_LENGTH         256
/* Limites de stockage : 6000 sommets, 6000 faces et 16 indices par face ; le
 * format OBJ complet n'est pas pris en charge.
 */
#define MAX_VERTICES            6000
#define MAX_FACES               6000
#define MAX_FACE_VERTICES       16
#define CENTRE_X                160
#define CENTRE_Y                100
#define mode                    320

#define PALETTE_BASE            ((unsigned int *)0xE19E00L)

/* Identifiants internes de modes ; ils ne correspondent pas aux caractères
 * 1..7 de l'interface.
 */
#define PAINTER_MODE_FAST       0
#define PAINTER_MODE_BUBBLE_SORT      1
#define PAINTER_MODE_CORRECT    3
#define PAINTER_MODE_GEO        5
#define PAINTER_MODE_CORRECTV2  6
#define PAINTER_MODE_GEOV3      7
#define PAINTER_MODE_NEWELL_SANCHA  8

#define COL_BLACK           0
#define COL_GREY            1

#define COL_BLUE            4

#define COL_ORANGE          6

#define COL_FRAME           7

#define COL_YELLOW          9
#define COL_LIGHT_GREEN    10

#define COL_PALE_PURPLE    12

#define COL_FILL_DEFAULT   14

#define COL_WHITE          15

/* Valeurs de sélection spéciales : 16 demande une couleur aléatoire, 17
 * reprend le remplissage pour le contour.
 */
#define COLOR_RANDOM       16
#define COLOR_SAME_AS_FILL 17

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 200
#define MAX_SPAN_INTERSECTIONS 16

typedef unsigned int GS_Color;

typedef struct {
    GS_Color color[16];
} GS_Palette;

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
    int polySize;
    Rect polyBBox;
    Point polyPoints[MAX_FACE_VERTICES];
} DynamicPolygon;

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

typedef struct {
    int face1;
    int face2;
} InconclusivePair;

typedef struct {
    int face1;
    int face2;
    /* Convention propre à PairCache : +1 signifie f1 avant f2 ; elle est
     * opposée à geometric_face_relation.
     */
    int8_t relation;
    int8_t occupied;
} PairCacheEntry;

typedef struct {
    PairCacheEntry* slots;
    int capacity;
} PairCache;

int readVertices_last_count = 0;

/* Handle QuickDraw réutilisé pour les polygones ; verrouillage suivi
 * explicitement.
 */
static Handle globalPolyHandle = NULL;
static int poly_handle_locked = 0;
static int framePolyOnly = 0;

int palette = 0;
GS_Palette palettes[16];
static int palette0_loaded = 0;

/* Options globales de visibilité et de rendu ; pan_dx/pan_dy sont des
 * déplacements écran en pixels.
 */
static int cull_back_faces = 1;
static int pan_dx = 0;
static int pan_dy = 0;
static int jitter = 0;
static int jitter_max = 7;

/* Diagnostic du clipping : dernière paire, arrêts de garde, contours et
 * centroïdes obtenus par les différentes méthodes.
 */
static int debug_overlap_subj = -1;
static int debug_overlap_clip = -1;

static int clip_last_bailout_subj = -1;
static int clip_last_bailout_clip = -1;

#define DEBUG_CLIP_MAX 512

static long long debug_clip_raw_area2 = 0;

static int debug_clip_centroid_x = 0;
static int debug_clip_centroid_y = 0;

static double debug_clip_fixed_area = 0.0;
static int debug_clip_fixed_vcount = 0;
static int debug_clip_fixed_vx[DEBUG_CLIP_MAX];
static int debug_clip_fixed_vy[DEBUG_CLIP_MAX];

/* Seuil d'aire projetée utilisé par les diagnostics d'intersection ; les
 * unités sont des pixels carrés.
 */
static const double MIN_INTERSECTION_AREA_PIXELS = 2.0;

/* Choix utilisateur : -1 par défaut ; 16 aléatoire ; 17 même couleur que le
 * remplissage pour le contour.
 */
static int user_fill_color = -1;
static int user_frame_color = -1;
static unsigned char *random_fill_colors = NULL;
static unsigned char *random_frame_colors = NULL;
static int random_colors_capacity = 0;

static int painter_mode = PAINTER_MODE_FAST;

/* Buffers de travail partagés des variantes géométriques et correctives ; ne
 * pas les assimiler à un mode utilisateur supplémentaire.
 */

/* Distance servant au calcul d'epsilon ; s_global_proj_scale_fixed est
 * l'échelle de projection en 16.16.
 */
static Fixed32 current_observer_distance = 65536;
static Fixed32 s_global_proj_scale_fixed;

/* Couleurs de remplissage calculées à partir de l'orientation des plans. */
static int shaded_by_orientation = 0;
static int face_shade_color[MAX_FACES];

/* Paires conservées pour l'inspection des ambiguïtés d'ordre ; le buffer est
 * agrandi selon les besoins.
 */
static int inconclusive_pairs_capacity = 0;
static InconclusivePair *inconclusive_pairs = NULL;
static int inconclusive_pairs_count = 0;

static unsigned long overlapCheckCount = 0;
static unsigned long overlapSegiAccept = 0;
static unsigned long overlapSampleAccept = 0;
static unsigned long overlapClipCalls = 0;
static unsigned long overlapClipAccept = 0;

/* Buffers de clipping partagés et réutilisés ; ils contribuent à la
 * non-réentrance du moteur.
 */

static FaceArrays3D* qsort_faces_ptr_for_cmp = NULL;

/*
 * ----------------------------------------------------------------------------
 *  encodePaletteColor
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Encoder une couleur RGB dans le mot $0RGB du matériel SHR.
 *
 *  Paramètres :
 *    [entrée] r, g et b, composantes entières bornées ici à [0,15].
 *
 *  Retour :
 *    GS_Color 12 bits.
 *
 *  Fonctionnement :
 *    Aucun accès matériel ni effet de bord.
 *
 * ----------------------------------------------------------------------------
 */
static GS_Color encodePaletteColor(int r, int g, int b)
{
    /* Saturer chaque composante sur quatre bits avant assemblage du mot RGB. */
    if (r < 0) r = 0; if (r > 15) r = 15;
    if (g < 0) g = 0; if (g > 15) g = 15;
    if (b < 0) b = 0; if (b > 15) b = 15;
    return ((GS_Color)r << 8) | ((GS_Color)g << 4) | (GS_Color)b;
}

/*
 * ----------------------------------------------------------------------------
 *  paletteGradientColor
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Construire une entrée d'une palette de dégradé.
 *
 *  Paramètres :
 *    [entrée] palette_num, 1 pour les gris, 2..15 pour une teinte cible ;
 *    index, position du dégradé.
 *
 *  Retour :
 *    Couleur $0RGB ; aucun effet de bord.
 *
 *  Fonctionnement :
 *    Pour les teintes, l'indice 8 porte la couleur cible, encadrée par le
 *    noir à 0 et le blanc à 15. Les interpolations entières utilisent un
 *    biais d'arrondi.
 *
 * ----------------------------------------------------------------------------
 */
static GS_Color paletteGradientColor(int palette_num, int index)
{
    int r = 0, g = 0, b = 0;

    /* La palette de gris affecte la même intensité aux trois composantes. */
    if (palette_num == 1) {

        return encodePaletteColor(index, index, index);
    }

    /* Choisir la teinte saturée du milieu de dégradé. */
    int target_r = 0, target_g = 0, target_b = 0;
    switch (palette_num) {
        case 2:  target_r = 15; target_g = 0;  target_b = 0;  break;
        case 3:  target_r = 0;  target_g = 15; target_b = 0;  break;
        case 4:  target_r = 0;  target_g = 0;  target_b = 15; break;
        case 5:  target_r = 15; target_g = 15; target_b = 0;  break;
        case 6:  target_r = 15; target_g = 0;  target_b = 15; break;
        case 7:  target_r = 0;  target_g = 15; target_b = 15; break;
        case 8:  target_r = 15; target_g = 8;  target_b = 0;  break;
        case 9:  target_r = 12; target_g = 0;  target_b = 15; break;
        case 10: target_r = 0;  target_g = 8;  target_b = 15; break;
        case 11: target_r = 8;  target_g = 15; target_b = 0;  break;
        case 12: target_r = 15; target_g = 6;  target_b = 11; break;
        case 13: target_r = 0;  target_g = 15; target_b = 8;  break;
        case 14: target_r = 12; target_g = 8;  target_b = 0;  break;
        case 15: target_r = 15; target_g = 0;  target_b = 8;  break;
        default: target_r = 15; target_g = 15; target_b = 15; break;
    }

    /* Imposer les extrémités noires et blanches des palettes colorées. */
    if (index <= 0) {
        return encodePaletteColor(0, 0, 0);
    }
    if (index >= 15) {
        return encodePaletteColor(15, 15, 15);
    }

    /* Interpoler du noir vers la teinte cible, puis de cette teinte vers le blanc. */
    const int midpoint = 8;
    if (index <= midpoint) {

        r = (target_r * index + midpoint / 2) / midpoint;
        g = (target_g * index + midpoint / 2) / midpoint;
        b = (target_b * index + midpoint / 2) / midpoint;
    } else {

        int step = index - midpoint;
        int range = 15 - midpoint;
        r = target_r + ((15 - target_r) * step + range / 2) / range;
        g = target_g + ((15 - target_g) * step + range / 2) / range;
        b = target_b + ((15 - target_b) * step + range / 2) / range;
    }
    return encodePaletteColor(r, g, b);
}

/* Déclarations partagées pour l'unité de compilation construite par
 * inclusions de fichiers C. Elles ne constituent pas à elles seules une
 * liste de fonctions actives.
 */
void initPalettes(void);
static GS_Color encodePaletteColor(int r, int g, int b);
static GS_Color paletteGradientColor(int palette_num, int index);

int readVertices(const char *filename, VertexArrays3D *vtx, int max_vertices, Model3D *owner);

void getObserverParams(ObserverParams *params, Model3D *model);

void drawPolygons(Model3D *model, int *vertex_count, int face_count, int vertex_count_total);
void drawPolygons_jitter(Model3D *model, int *vertex_count, int face_count, int vertex_count_total);
void calculateFaceDepths(Model3D *model, Face3D *faces, int face_count);

void painter_newell_sancha_fast(Model3D* model, int face_count);
void painter_bubble_sort(Model3D* model, int face_count);
void painter_newell_sancha(Model3D* model, int face_count);
void painter_geoV2(Model3D* model, int face_count);
static int painter_correct(Model3D* model, int face_count, int debug);
static int painter_correctV2(Model3D* model, int face_count, int debug);
void painter_geoV2(Model3D* model, int face_count);

static int geometric_face_relation(Model3D* model, int f1, int f2);
int projected_polygons_overlap(Model3D* model, int f1, int f2);
int projected_polygons_overlap_simple(Model3D* model, int f1, int f2);
int pair_plane_before(Model3D* model, int f1, int f2);
int pair_plane_after(Model3D* model, int f1, int f2);
int ray_cast_hierarchical(Model3D* model, int f1, int f2);

void drawFace(Model3D* model, int face_id, int fillPenPat, int show_index);
void drawFaceIndex(Model3D* model, int face_id);
void frameInconclusivePairs(Model3D* model);
void generate_random_colors(int face_count);

void compute2DFromObserver(Model3D* model, int angle_w);
static void updateFace2DBounds(Model3D* model);

void applyPalette(int palette_num);
GS_Palette ReadPalette(int palette_num);
int getFaceFillColor(int face_id);
int getFaceFrameColor(int face_id, int fill_color);

void renderModelScanlineZBuffer(Model3D* model);
void drawPixel(int x, int y, int color);

void DoColor(void);
void DoText(void);
static void show_help_pager(void);
void saveSHRAsRawPic(const char *filename);
void saveNextScreenshot(void);
void setProDOSFileType(const char* filename, int fileType, long auxType);

Model3D* createModel3D(void);
void     destroyModel3D(Model3D* model);
int      loadModel3D(Model3D* model, const char* filename);

void processModelFast(Model3D* model, ObserverParams* params, const char* filename);
void getObserverParams(ObserverParams* params, Model3D* model);
void normalizeAutoFitDistanceTo150(Model3D* model);

void inspect_faces_before(Model3D* model, ObserverParams* params, const char* filename);
void inspect_faces_after(Model3D* model, ObserverParams* params, const char* filename);
void showFace(Model3D* model, ObserverParams* params, const char* filename);
void display_model_face_ids(Model3D* model, ObserverParams* params, const char* filename);
void inspect_face_pair_ui(Model3D* model);

void dumpFaceEquationsCSV(Model3D* model, const char* filename, int use_semicolon);
void dumpFace2DCoordinates(Model3D* model, const char* txt_filename);
void dumpSortedFaceIndices(Model3D* model, const char* txt_filename);

void computeOrientationShading(Model3D* model);

static int compute_intersection_centroid_fractional_tryboth(Model3D *model,
        int subj, int clip, double *outx, double *outy, double *out_area2);
static int ray_cast_at_fractional(Model3D *model, int f1, int f2, double cx, double cy);

static void fractional_projection_cache_invalidate(Model3D *model);
static void fractional_projection_cache_release(Model3D *model);

static int projected_polygons_overlap_cached(Model3D *model, int f1, int f2);

static Fixed32 fixed_mul32_fast(Fixed32 a,Fixed32 b);
static Fixed64 fixed_product32_fast(Fixed32 a,Fixed32 b);
static Fixed64 fixed_product64x32_fast(Fixed64 a,Fixed32 b);
