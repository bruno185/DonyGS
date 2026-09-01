// ============================================================================
//  declare.c  –  Déclarations, structures, constantes et variables globales
//  Apple IIGS / ORCA-C – Moteur 3D (OBJ + peintre + clipping)
// ============================================================================

// ============================================================================
//  1. HEADER INCLUDES
// ============================================================================
#include <stdio.h>      // Standard input/output (printf, fgets, etc.)
#include <asm.h>        // ORCA specific assembler functions
#include <string.h>     // String manipulation (strlen, strcmp, etc.)
#include <misctool.h>   // ORCA misc tools
#include <stdlib.h>     // Standard functions (malloc, free, atof, etc.)
#include <math.h>       // Math functions (cos, sin, sqrt, etc.)
#include <quickdraw.h>  // Apple IIGS QuickDraw graphics API
#include <event.h>      // Apple IIGS event management
#include <memory.h>     // Apple IIGS memory management (NewHandle, etc.)
#include <window.h>     // Apple IIGS Window management
#include <orca.h>       // ORCA specific functions (startgraph, etc.)
#include <stdint.h>     // uint32_t, etc.
#include <GSOS.h>

segment "data";

// ============================================================================
//  2. FIXED-POINT ARITHMETIC (16.16)
// ============================================================================
/**
 * FIXED POINT ARITHMETIC - 64-bit safe version
 * Format: 16.16 (16 integer bits, 16 fractional bits)
 * Range : -32768.0 → +32767.99998  (précision 1/65536)
 */
typedef long        Fixed32;        // 32-bit fixed point
typedef long long   Fixed64;        // 64-bit intermediate calculations

#define FIXED_SHIFT     16
#define FIXED_SCALE     (1L << FIXED_SHIFT)   // 65536
#define FIXED_MASK      (FIXED_SCALE - 1)     // 0xFFFF
#define FIXED_HALF      (FIXED_SCALE >> 1)    // 32768 (pour arrondi)

// Constantes mathématiques
#define FIXED_PI        205887L               // π
#define FIXED_2PI       411775L               // 2π
#define FIXED_PI_2      102944L               // π/2
#define FIXED_ONE       FIXED_SCALE           // 1.0
#define FIXED_PI_180    1143LL                // π/180

// Conversion
#define INT_TO_FIXED(x)     ((Fixed32)(x) << FIXED_SHIFT)
#define FIXED_TO_INT(x)     ((int)((x) >> FIXED_SHIFT))
#define FLOAT_TO_FIXED(x)   ((Fixed32)((x) * FIXED_SCALE))
#define FIXED_TO_FLOAT(x)   ((float)(x) / (float)FIXED_SCALE)
#define FIXED64_TO_FLOAT(x) ((double)(x) / (double)FIXED_SCALE)

static inline int FIXED_ROUND_TO_INT(Fixed32 x) {
    if (x >= 0) return (int)(((x) + FIXED_HALF) >> FIXED_SHIFT);
    else        return (int)(((x) - FIXED_HALF) >> FIXED_SHIFT);
}

// Opérations arithmétiques
#define FIXED_ADD(a, b)     ((a) + (b))
#define FIXED_SUB(a, b)     ((a) - (b))
#define FIXED_NEG(x)        (-(x))
#define FIXED_ABS(x)        ((x) >= 0 ? (x) : -(x))
#define FIXED_FRAC(x)       ((x) & FIXED_MASK)

#define FIXED_MUL(a, b)     (((long)(a) * (long)(b)) >> FIXED_SHIFT)
#define FIXED_DIV(a, b)     (((long)(a) << FIXED_SHIFT) / (long)(b))

// Versions 64-bit sûres
#define FIXED_MUL_64(a, b)  ((Fixed32)(((Fixed64)(a) * (Fixed64)(b)) >> FIXED_SHIFT))
#define FIXED_DIV_64(a, b)  ((Fixed32)(((Fixed64)(a) << FIXED_SHIFT) / (Fixed64)(b)))
#define FIXED64_TO_32(x)    ((Fixed32)(x))

// Tables sin/cos (0..359°)
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

static inline Fixed32 sin_deg_int(int deg) {
    deg %= 360;
    if (deg < 0) deg += 360;
    return sin_table[deg];
}
static inline Fixed32 cos_deg_int(int deg) {
    deg %= 360;
    if (deg < 0) deg += 360;
    return cos_table[deg];
}
static inline int normalize_deg(int deg) {
    deg %= 360;
    if (deg < 0) deg += 360;
    return deg;
}

// ============================================================================
//  3. CONSTANTES GLOBALES & CONFIGURATION
// ============================================================================
#define PERFORMANCE_MODE        1       // 1 = pas de printf, 0 = mode debug
#define MAX_LINE_LENGTH         256
#define MAX_VERTICES            6000
#define MAX_FACES               6000
#define MAX_FACE_VERTICES       16
#define CENTRE_X                160     // 320/2
#define CENTRE_Y                100     // 200/2
#define mode                    320     // mode graphique 320×200

#define PALETTE_BASE            ((unsigned int *)0xE19E00L)

// Modes du peintre
#define PAINTER_MODE_FAST       0
#define PAINTER_MODE_FIXED      1
#define PAINTER_MODE_CORRECT    3
#define PAINTER_MODE_GEO        5
#define PAINTER_MODE_CORRECTV2  6

// Couleurs QuickDraw nommées
#define COL_BLACK           0
#define COL_GREY            1
#define COL_BROWN           2
#define COL_PURPLE          3
#define COL_BLUE            4
#define COL_GREEN           5
#define COL_ORANGE          6
#define COL_RED             7
#define COL_FRAME           7
#define COL_ROSE            8
#define COL_YELLOW          9
#define COL_LIGHT_GREEN    10
#define COL_AQUA           11
#define COL_PALE_PURPLE    12
#define COL_LIGHT_BLUE     13
#define COL_FILL_DEFAULT   14
#define COL_LIGHT_GREY     14
#define COL_WHITE          15

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 200
#define MAX_SPAN_INTERSECTIONS 16   // generous for concave faces (e.g. a star)


// ============================================================================
//  4. TYPES & STRUCTURES
// ============================================================================
typedef unsigned int GS_Color;

typedef struct {
    GS_Color color[16];
} GS_Palette;

// Parallel arrays pour les sommets (évite la limite 32K/64K des structs)
typedef struct {
    Handle xHandle, yHandle, zHandle;
    Handle xoHandle, yoHandle, zoHandle;
    Handle x2dHandle, y2dHandle;
    Fixed32 *x, *y, *z;
    Fixed32 *xo, *yo, *zo;
    int *x2d, *y2d;
    int vertex_count;
    int saved_vertex_count;
} VertexArrays3D;

// Parallel arrays pour les faces (compact + depth sorting)
typedef struct {
    Handle vertex_countHandle;
    Handle vertex_indicesBufferHandle;
    Handle vertex_indicesPtrHandle;
    Handle z_maxHandle;
    Handle display_flagHandle;
    Handle sorted_face_indicesHandle;

    int *vertex_count;
    int *saved_vertex_count;
    int *vertex_indices_buffer;
    int *vertex_indices_ptr;
    Fixed32 *z_min;
    Fixed32 *z_max;
    Fixed32 *z_mean;
    Fixed64 *plane_a, *plane_b, *plane_c, *plane_d;

    /* bbox 2D projetée */
    int *minx, *maxx, *miny, *maxy;
    /* bbox 3D object-space */
    Fixed32 *minx3, *maxx3, *miny3, *maxy3, *minz3, *maxz3;

    int *display_flag;
    int *sorted_face_indices;
    int face_count;
    int total_indices;
} FaceArrays3D;

// Structure legacy (conservée pour compatibilité)
typedef struct {
    int vertex_count;
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
    int angle_h;        // rotation horizontale (°)
    int angle_v;        // rotation verticale (°)
    int angle_w;        // rotation plan 2D (°)
    Fixed32 distance;   // distance observateur
} ObserverParams;

typedef struct {
    VertexArrays3D vertices;
    FaceArrays3D   faces;

    /* Auto-scale metadata */
    Fixed32 auto_scale;
    Fixed32 auto_center_x, auto_center_y, auto_center_z;
    int auto_scaled;
    int auto_centered;

    Fixed32 auto_suggested_distance;
    Fixed32 auto_suggested_proj_scale;
    Fixed32 auto_proj_scale;
    int auto_fit_ready;
    int auto_fit_applied;

    Fixed32 *orig_x, *orig_y, *orig_z;  // backup pour revert

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
    int8_t relation;  // 1 = f1 before f2, -1 = f2 before f1
    int8_t occupied;  // 1 = slot used, 0 = empty
} PairCacheEntry;

typedef struct {
    PairCacheEntry* slots;
    int capacity;
} PairCache;

// ============================================================================
//  5. VARIABLES GLOBALES
// ============================================================================

/* --- Vérification de cohérence OBJ --- */
int readVertices_last_count = 0;

/* --- Handle polygon persistant --- */
static Handle globalPolyHandle = NULL;
static int poly_handle_locked = 0;
static int framePolyOnly = 0;          // 1 = frame only, 0 = fill+frame

/* --- Palette --- */
int palette = 0;
GS_Palette palettes[16];
static int palette0_loaded = 0;

/* --- Culling & panning --- */
static int cull_back_faces = 1;        // back-face culling activé par défaut
static int pan_dx = 0;
static int pan_dy = 0;
static int jitter = 0;
static int jitter_max = 7;

/* --- Debug clipping --- */
static int debug_overlap_subj = -1;
static int debug_overlap_clip = -1;
static unsigned long clip_watchdog_bailouts = 0;
static unsigned long clip_iter_bailouts = 0;
static int clip_last_bailout_subj = -1;
static int clip_last_bailout_clip = -1;

#define DEBUG_CLIP_MAX 512
static int debug_clip_vcount = 0;
static int debug_clip_vx[DEBUG_CLIP_MAX];
static int debug_clip_vy[DEBUG_CLIP_MAX];
static long long debug_clip_raw_area2 = 0;
static double debug_clip_area = 0.0;
static int debug_clip_centroid_x = 0;
static int debug_clip_centroid_y = 0;

static double debug_clip_fixed_area = 0.0;
static int debug_clip_fixed_vcount = 0;
static int debug_clip_fixed_vx[DEBUG_CLIP_MAX];
static int debug_clip_fixed_vy[DEBUG_CLIP_MAX];

static double debug_clip_float_area = 0.0;
static int debug_clip_float_vcount = 0;
static int debug_clip_float_vx[DEBUG_CLIP_MAX];
static int debug_clip_float_vy[DEBUG_CLIP_MAX];
static int debug_clip_float_overridden = 0;

static const double MIN_INTERSECTION_AREA_PIXELS = 2.0;

/* --- Couleurs utilisateur --- */
static int user_fill_color = -1;
static int user_frame_color = -1;
static unsigned char *random_fill_colors = NULL;
static unsigned char *random_frame_colors = NULL;
static int random_colors_capacity = 0;

/* --- Mode peintre --- */
static int painter_mode = PAINTER_MODE_FAST;

/* --- Buffers float (pour modes correct / geo) --- */
static float *float_px = NULL, *float_py = NULL;
static int *float_px_int = NULL, *float_py_int = NULL;
static float *f_z_min_buf = NULL, *f_z_max_buf = NULL, *f_z_mean_buf = NULL;
static int *f_minx_buf = NULL, *f_maxx_buf = NULL, *f_miny_buf = NULL, *f_maxy_buf = NULL;
static int *f_display_buf = NULL;
static float *f_plane_a_buf = NULL, *f_plane_b_buf = NULL, *f_plane_c_buf = NULL, *f_plane_d_buf = NULL;
static int *f_plane_conv_buf = NULL;
static int *order_buf = NULL;
static int order_cap = 0;

/* --- Distance observateur (pour epsilon géométrique) --- */
static Fixed32 current_observer_distance = 65536; // 1.0
static Fixed32 s_global_proj_scale_fixed;

/* --- Shading par orientation --- */
static int shaded_by_orientation = 0;
static int face_shade_color[MAX_FACES];

/* --- Paires inconclusives (Newell-Sancha) --- */
static int inconclusive_pairs_capacity = 0;
static InconclusivePair *inconclusive_pairs = NULL;
static int inconclusive_pairs_count = 0;

/* --- Instrumentation overlap --- */
static unsigned long overlapCheckCount = 0;
static unsigned long overlapSegiAccept = 0;
static unsigned long overlapSampleAccept = 0;
static unsigned long overlapClipCalls = 0;
static unsigned long overlapClipAccept = 0;

/* --- Buffers de clipping Sutherland-Hodgman (réutilisés) --- */
static long long *clip_sx = NULL, *clip_sy = NULL;
static long long *clip_tx = NULL, *clip_ty = NULL;
static int clip_bufcap = 0;

static FaceArrays3D* qsort_faces_ptr_for_cmp = NULL;

// ============================================================================
//  Fonctions utilitaires palette (static)
// ============================================================================

static GS_Color encodePaletteColor(int r, int g, int b)
{
    if (r < 0) r = 0; if (r > 15) r = 15;
    if (g < 0) g = 0; if (g > 15) g = 15;
    if (b < 0) b = 0; if (b > 15) b = 15;
    return ((GS_Color)r << 8) | ((GS_Color)g << 4) | (GS_Color)b;
}

static GS_Color paletteGradientColor(int palette_num, int index)
{
    int r = 0, g = 0, b = 0;

    if (palette_num == 1) {
        /* Palette 1 = grayscale ramp from black to white */
        return encodePaletteColor(index, index, index);
    }

    /* Define a saturated target color for palettes 2..15. */
    int target_r = 0, target_g = 0, target_b = 0;
    switch (palette_num) {
        case 2:  target_r = 15; target_g = 0;  target_b = 0;  break; // red
        case 3:  target_r = 0;  target_g = 15; target_b = 0;  break; // green
        case 4:  target_r = 0;  target_g = 0;  target_b = 15; break; // blue
        case 5:  target_r = 15; target_g = 15; target_b = 0;  break; // yellow
        case 6:  target_r = 15; target_g = 0;  target_b = 15; break; // magenta
        case 7:  target_r = 0;  target_g = 15; target_b = 15; break; // cyan
        case 8:  target_r = 15; target_g = 8;  target_b = 0;  break; // orange
        case 9:  target_r = 12; target_g = 0;  target_b = 15; break; // purple
        case 10: target_r = 0;  target_g = 8;  target_b = 15; break; // sky blue
        case 11: target_r = 8;  target_g = 15; target_b = 0;  break; // lime
        case 12: target_r = 15; target_g = 6;  target_b = 11; break; // pink
        case 13: target_r = 0;  target_g = 15; target_b = 8;  break; // teal
        case 14: target_r = 12; target_g = 8;  target_b = 0;  break; // brown
        case 15: target_r = 15; target_g = 0;  target_b = 8;  break; // rose
        default: target_r = 15; target_g = 15; target_b = 15; break;
    }

    if (index <= 0) {
        return encodePaletteColor(0, 0, 0);
    }
    if (index >= 15) {
        return encodePaletteColor(15, 15, 15);
    }

    const int midpoint = 8;
    if (index <= midpoint) {
        /* Black -> saturated target at index 8 */
        r = (target_r * index + midpoint / 2) / midpoint;
        g = (target_g * index + midpoint / 2) / midpoint;
        b = (target_b * index + midpoint / 2) / midpoint;
    } else {
        /* Saturated target at index 8 -> white at index 15 */
        int step = index - midpoint;
        int range = 15 - midpoint; /* 7 */
        r = target_r + ((15 - target_r) * step + range / 2) / range;
        g = target_g + ((15 - target_g) * step + range / 2) / range;
        b = target_b + ((15 - target_b) * step + range / 2) / range;
    }
    return encodePaletteColor(r, g, b);
}


// ============================================================================
//  6. PROTOTYPES DE FONCTIONS
// ============================================================================

/* Palette */
void initPalettes(void);
static GS_Color encodePaletteColor(int r, int g, int b);
static GS_Color paletteGradientColor(int palette_num, int index);

/* Lecture OBJ */
int readVertices(const char *filename, VertexArrays3D *vtx, int max_vertices, Model3D *owner);
/* (readFaces est déclaré ailleurs ou à ajouter) */

/* Observateur */
void getObserverParams(ObserverParams *params, Model3D *model);

/* Rendu */
void drawPolygons(Model3D *model, int *vertex_count, int face_count, int vertex_count_total);
void drawPolygons_jitter(Model3D *model, int *vertex_count, int face_count, int vertex_count_total);
void calculateFaceDepths(Model3D *model, Face3D *faces, int face_count);

// Painter
void painter_newell_sancha_fast(Model3D* model, int face_count);
void painter_newell_sancha(Model3D* model, int face_count);
void painter_geoV2(Model3D* model, int face_count);
static int painter_correct(Model3D* model, int face_count, int debug);   // ou non-static selon usage
static int painter_correctV2(Model3D* model, int face_count, int debug);

// Helpers géométriques / ordre
static int geometric_face_relation(Model3D* model, int f1, int f2);
int projected_polygons_overlap(Model3D* model, int f1, int f2);          // (défini ailleurs ?)
int projected_polygons_overlap_simple(Model3D* model, int f1, int f2);
int pair_plane_before(Model3D* model, int f1, int f2);
int pair_plane_after(Model3D* model, int f1, int f2);
int ray_cast_hierarchical(Model3D* model, int f1, int f2);

// Dessin
void drawFace(Model3D* model, int face_id, int fillPenPat, int show_index);
void drawFaceIndex(Model3D* model, int face_id);
void frameInconclusivePairs(Model3D* model);
void generate_random_colors(int face_count);

// Projection / utilitaires
void compute2DFromObserver(Model3D* model, int angle_w);
static void updateFace2DBounds(Model3D* model);

// Palette / couleurs
void applyPalette(int palette_num);
void SetColor(int palette_num, int color_index, int r, int g, int b);
GS_Palette ReadPalette(int palette_num);
int getFaceFillColor(int face_id);
int getFaceFrameColor(int face_id, int fill_color);

// Rendu alternatif
void renderModelScanlineZBuffer(Model3D* model);
void drawPixel(int x, int y, int color);

// Divers
void debug_two_faces(Model3D* model, int f1, int f2);
void DoColor(void);
void DoText(void);
static void show_help_pager(void);
void saveSHRAsRawPic(const char *filename);
void saveNextScreenshot(void);
void setProDOSFileType(const char* filename, int fileType, long auxType);

// Création / destruction / chargement
Model3D* createModel3D(void);
void     destroyModel3D(Model3D* model);
int      loadModel3D(Model3D* model, const char* filename);

// Pipeline principal
void processModelFast(Model3D* model, ObserverParams* params, const char* filename);
void getObserverParams(ObserverParams* params, Model3D* model);
void normalizeAutoFitDistanceTo150(Model3D* model);

// Vérifications / réparation d’ordre
// void check_intersect(Model3D* model);
// void check_sort_repair(Model3D* model, int face_count);
// void check_sort_repair_fast(Model3D* model, int face_count);

// Inspection / debug
void inspect_faces_before(Model3D* model, ObserverParams* params, const char* filename);
void inspect_faces_after(Model3D* model, ObserverParams* params, const char* filename);
void showFace(Model3D* model, ObserverParams* params, const char* filename);
void display_model_face_ids(Model3D* model, ObserverParams* params, const char* filename);
void inspect_face_pair_ui(Model3D* model);
// void pair_plane_geometric_tests(Model3D* model, int f1, int f2);

// Dump
void dumpFaceEquationsCSV(Model3D* model, const char* filename, int use_semicolon);
void dumpFace2DCoordinates(Model3D* model, const char* txt_filename);   
void dumpSortedFaceIndices(Model3D* model, const char* txt_filename);   

// Shading
void computeOrientationShading(Model3D* model);

