
// ============================================================================
//                           HEADER INCLUDES
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
#include <stdint.h>      // uint32_t, etc.
#include <GSOS.h>

segment "data";

// ============================================================================
// --- Global variable for vertex index verification in readFaces ---
// (Added to ensure OBJ consistency)
int readVertices_last_count = 0;

// --- Global persistent polygon handle for drawing ---
// Allocated once, HLocked only during use, prevents repeated NewHandle/DisposeHandle
static Handle globalPolyHandle = NULL;
static int poly_handle_locked = 0;  // Track lock state
static int framePolyOnly = 0; // Toggle: 1 = frame-only, 0 = fill+frame (default: filled polygons)

int palette = 0; // Palette number to use
typedef unsigned int GS_Color;

/* palette = 16 colors */
typedef struct {
    GS_Color color[16];
} GS_Palette;

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

/* Global palette storage: 16 palettes indexed 0..15 */
GS_Palette palettes[16];
static int palette0_loaded = 0;

void initPalettes(void);

// Runtime back-face culling toggle
// - When enabled (1): an **observer-space** back-face test is performed per face during
//   `calculateFaceDepths()` using the face plane D term (observer-space d <= 0 => culled).
// - Faces culled by this test are marked with `display_flag = 0`. They are **excluded** from
//   the visible subset the painter sorts (to improve correctness and performance), but are
//   appended after visible faces in `sorted_face_indices` to preserve array stability.
// - Drawing still checks `display_flag` and will skip culled faces; toggling happens at runtime
//   via the `B` key in the UI when interacting with the program.
static int cull_back_faces = 1; // default: enabled
// 2D panning offsets (pixels) - used by drawing routines (frame-only, drawPolygons, drawFace)
static int pan_dx = 0;
static int pan_dy = 0;
static int jitter = 0; /* Toggle: 1 = use jittered drawPolygons_jitter, 0 = normal */
static int jitter_max = 7; /* maximum random pixel offset (0..10) */

/* Debug helper: when set to a pair (subj,clip) the clipping routine will print final vertices */
static int debug_overlap_subj = -1;
static int debug_overlap_clip = -1;

/* Clipping bail-out diagnostics (incremented when watchdog or iteration cap fires) */
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

/* Fixed-result snapshot diagnostics (before float fallback) */
static double debug_clip_fixed_area = 0.0;
static int debug_clip_fixed_vcount = 0;
static int debug_clip_fixed_vx[DEBUG_CLIP_MAX];
static int debug_clip_fixed_vy[DEBUG_CLIP_MAX];
/* double-precision fallback diagnostics (filled by compute_intersection) */
static double debug_clip_float_area = 0.0;
static int debug_clip_float_vcount = 0;
static int debug_clip_float_vx[DEBUG_CLIP_MAX];
static int debug_clip_float_vy[DEBUG_CLIP_MAX];
static int debug_clip_float_overridden = 0; /* 1 if float result overrode fixed result */

/* Thresholds for overlap/centroid decisions (pixels^2) */
static const double MIN_INTERSECTION_AREA_PIXELS = 2.0; /* strict overlap test threshold (set to 2 pixels) */

// User-selected colors: -1 means not set (use defaults), 0-15 are colors, 16=random

/* Named QuickDraw color indices (0..15) — use these instead of numeric literals */
#define COL_BLACK           0
#define COL_GREY            1
#define COL_BROWN           2
#define COL_PURPLE          3
#define COL_BLUE            4
#define COL_GREEN           5
#define COL_ORANGE          6   /* orange / brown */
#define COL_RED             7   /* commonly used frame color */
#define COL_FRAME           7   /* commonly used frame color */
#define COL_ROSE            8
#define COL_YELLOW          9
#define COL_LIGHT_GREEN    10
#define COL_AQUA           11
#define COL_PALE_PURPLE    12
#define COL_LIGHT_BLUE     13
#define COL_FILL_DEFAULT   14  /* default fill pen */
#define COL_LIGHT_GREY     14 
#define COL_WHITE          15  /* white */

static int user_fill_color = -1;  // Interior color
static int user_frame_color = -1; // Frame color
// Random color buffers (allocated per face when random mode is active)
static unsigned char* random_fill_colors = NULL;
static unsigned char* random_frame_colors = NULL;
static int random_colors_capacity = 0;

#define PAINTER_MODE_FAST 0
#define PAINTER_MODE_FIXED 1
// #define PAINTER_MODE_FLOAT 2
#define PAINTER_MODE_CORRECT 3
#define PAINTER_MODE_CORRECTV2 6
#define PAINTER_MODE_GEO 5


#define PALETTE_BASE  ((unsigned int *)0xE19E00L) /* QuickDraw palette base address (256 entries × 3 bytes each) */


static int painter_mode = PAINTER_MODE_FAST; // 0=fast,1=fixed,2=float,3=correct,5=geo,6=correctV2

static float *float_px = NULL, *float_py = NULL; static int *float_px_int = NULL, *float_py_int = NULL;static float *f_z_min_buf = NULL, *f_z_max_buf = NULL, *f_z_mean_buf = NULL;
static int *f_minx_buf = NULL, *f_maxx_buf = NULL, *f_miny_buf = NULL, *f_maxy_buf = NULL;
static int *f_display_buf = NULL;
static float *f_plane_a_buf = NULL, *f_plane_b_buf = NULL, *f_plane_c_buf = NULL, *f_plane_d_buf = NULL;
static int *f_plane_conv_buf = NULL; /* 0 = not converted from fixed, 1 = converted */
static int *order_buf = NULL; static int order_cap = 0;


// ============================================================================
//                            FIXED POINT DEFINITIONS
// ============================================================================

/**
 * FIXED POINT ARITHMETIC - 64-bit safe version
 * =============================================
 * 
 * This implementation uses 16.16 fixed-point arithmetic with
 * 64-bit intermediate calculations to avoid overflow.
 * 
 * Format: 16.16 (16 integer bits, 16 fractional bits)
 * Range: -32768.0 to +32767.99998 with 1/65536 precision
 */

// Basic fixed-point definitions
typedef long Fixed32;           // 32-bit fixed point number (16.16)
typedef long long Fixed64;      // 64-bit for intermediate calculations

// Cached observer distance (params.distance) used for epsilon scaling in geometric tests
static Fixed32 current_observer_distance = 65536; // 1.0 in 16.16 fixed point

static Fixed32 s_global_proj_scale_fixed; // Global projection scale (pixels per projected unit)

#define FIXED_SHIFT     16                    // Number of fractional bits
#define FIXED_SCALE     (1L << FIXED_SHIFT)  // 65536
#define FIXED_MASK      (FIXED_SCALE - 1)    // 0xFFFF
#define FIXED_HALF      (FIXED_SCALE >> 1)   // 32768 (for rounding)

// Mathematical constants in fixed point
#define FIXED_PI        205887L               // PI ≈ 3.14159265 in 16.16
#define FIXED_2PI       411775L               // 2*PI ≈ 6.28318530 in 16.16
#define FIXED_PI_2      102944L               // PI/2 ≈ 1.57079632 in 16.16
#define FIXED_ONE       FIXED_SCALE           // 1.0 in 16.16
#define FIXED_PI_180    1143LL                // PI/180 ≈ 0.017453293 in 16.16 (64-bit)

// Lookup tables: sin(0..359) and cos(0..359) in Fixed32 16.16
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
    -11380, -10252, -9121, -7987, -6850, -5712, -4572, -3430, -2287, -1144,
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
    64540, 64729, 64898, 65048, 65177, 65287, 65376, 65446, 65496, 65526,
};

// Integer-degree sin/cos helpers (direct table lookup)
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


// Conversion macros
#define INT_TO_FIXED(x)     ((Fixed32)(x) << FIXED_SHIFT)
#define FIXED_TO_INT(x)     ((int)((x) >> FIXED_SHIFT))
// Round fixed to int (nearest) handling negatives
static inline int FIXED_ROUND_TO_INT(Fixed32 x) {
    if (x >= 0) return (int)(((x) + FIXED_HALF) >> FIXED_SHIFT);
    else return (int)(((x) - FIXED_HALF) >> FIXED_SHIFT);
}
#define FLOAT_TO_FIXED(x)   ((Fixed32)((x) * FIXED_SCALE))
#define FIXED_TO_FLOAT(x)   ((float)(x) / (float)FIXED_SCALE)
#define FIXED64_TO_FLOAT(x)  ((double)(x) / (double)FIXED_SCALE)

// Arithmetic operations
#define FIXED_ADD(a, b)     ((a) + (b))
#define FIXED_SUB(a, b)     ((a) - (b))
#define FIXED_NEG(x)        (-(x))
#define FIXED_ABS(x)        ((x) >= 0 ? (x) : -(x))
#define FIXED_FRAC(x)       ((x) & FIXED_MASK)

// Simple multiplication and division for ORCA/C
#define FIXED_MUL(a, b)     (((long)(a) * (long)(b)) >> FIXED_SHIFT)
#define FIXED_DIV(a, b)     (((long)(a) << FIXED_SHIFT) / (long)(b))

// 64-bit safe multiplication and division for critical calculations
#define FIXED_MUL_64(a, b)  ((Fixed32)(((Fixed64)(a) * (Fixed64)(b)) >> FIXED_SHIFT))
#define FIXED_DIV_64(a, b)  ((Fixed32)(((Fixed64)(a) << FIXED_SHIFT) / (Fixed64)(b)))
#define FIXED64_TO_32(x)    ((Fixed32)(x))


// Integer degree normalization
static inline int normalize_deg(int deg) {
    deg %= 360;
    if (deg < 0) deg += 360;
    return deg;
}

// ============================================================================
//                            GLOBAL CONSTANTS
// ============================================================================

// Performance and debug configuration
//#define PERFORMANCE_MODE 0      // 1 = Optimized performance mode, 0 = Debug mode
// OPTIMIZATION: Performance mode - disable printf
#define PERFORMANCE_MODE 1      // 1 = no printf, 0 = normal printf

#define MAX_LINE_LENGTH 256     // Maximum file line size
#define MAX_VERTICES 6000       // Maximum vertices in a 3D model
#define MAX_FACES 6000          // Maximum faces in a 3D model (using parallel arrays)
#define MAX_FACE_VERTICES 16     // Maximum vertices per face (triangles/quads/hexagons)
#define CENTRE_X 160            // Screen center in X (320/2)
#define CENTRE_Y 100            // Screen center in Y (200/2)
//#define mode 640              // Graphics mode 640x200 pixels
#define mode 320                // Graphics mode 320x200 pixels

static int shaded_by_orientation = 0;        // Toggle orientation-based face shading via '!'
static int face_shade_color[MAX_FACES];

// ============================================================================
//                          3D DATA STRUCTURES
// ============================================================================

/**
 * Structure Vertex3D
 * 
 * DESCRIPTION:
 *   Represents a point in 3D space with its different representations
 *   throughout the 3D rendering pipeline.
 * 
 * FIELDS:
 *   x, y, z    : Original coordinates read from OBJ file
 *   xo, yo, zo : Transformed coordinates in the observer system
 *                (after applying rotations and translation)
 *   x2d, y2d   : Final projected coordinates on 2D screen
 * 
 * USAGE:
 *   This structure preserves all transformation steps to
 *   allow debugging and recalculations without rereading the file.
 */


// Parallel arrays for vertex data (to break 32K/64K struct limit)
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

/**
 * Structure FaceArrays3D - Compact dynamic face storage with depth-sorted rendering
 * Each face stores ONLY the vertices it needs:
 * - vertex_count: How many vertices this face has (3 for tri, 4 for quad, etc.)
 * - vertex_indices_buffer: ONE packed buffer with all indices (NO WASTED SLOTS!)
 * - vertex_indices_ptr: Offset array pointing to each face's slice in the buffer
 * - sorted_face_indices: Array of face indices SORTED by depth (for painter's algorithm)
 * - z_max: Depth for sorting
 * - display_flag: Culling flag
 * 
 * MEMORY LAYOUT:
 * Instead of 4 arrays of 6000 elements each, we use ONE packed buffer.
 * Triangles (1538 faces × 3 indices) + Quads (2504 faces × 4 indices) = packed linearly
 * Saves ~40-60% memory vs fixed 4 vertices/face
 * 
 * DEPTH SORTING STRATEGY:
 * Instead of moving data around (complex with variable-length indices), we maintain
 * sorted_face_indices[] which contains face numbers in depth order (farthest first).
 * Drawing loop: for(i=0; i<face_count; i++) { int face_id = sorted_face_indices[i]; ... }
 * This keeps the buffer untouched while providing correct rendering order.
 */
typedef struct {
    Handle vertex_countHandle;           // 1 array: face_count × 4 bytes
    Handle vertex_indicesBufferHandle;   // 1 buffer: all indices packed (NO WASTED SLOTS!)
    Handle vertex_indicesPtrHandle;      // 1 array: offset to each face's indices
    Handle z_maxHandle;                  // 1 array: face_count × 4 bytes
    Handle display_flagHandle;           // 1 array: face_count × 4 bytes
    Handle sorted_face_indicesHandle;    // 1 array: face numbers sorted by depth
    
    int *vertex_count;                   // Points to: [3, 3, 4, 3, 4, ...]
    int *saved_vertex_count;
    int *vertex_indices_buffer;          // Points to: [v1, v2, v3, v1, v2, v3, v4, v1, v2, v3, ...]
    int *vertex_indices_ptr;             // Points to: [offset0, offset3, offset6, offset10, ...]
    Fixed32 *z_min;
    Fixed32 *z_max;
    Fixed32 *z_mean;                     // Mean zo per face (computed in calculateFaceDepths)
    Fixed64 *plane_a;                     // per-face normalized normal X (a) stored as Fixed64 (16.16)
    Fixed64 *plane_b;                     // per-face normalized normal Y (b) stored as Fixed64 (16.16)
    Fixed64 *plane_c;                     // per-face normalized normal Z (c) stored as Fixed64 (16.16)
    Fixed64 *plane_d;                     // per-face D term for plane equation stored as Fixed64 (16.16)
    /* 2D cached projected bbox (x/y) */
    int *minx;
    int *maxx;
    int *miny;
    int *maxy;
    /* 3D object-space bbox, coordinates stored as Fixed32 */
    Fixed32 *minx3;
    Fixed32 *maxx3;
    Fixed32 *miny3;
    Fixed32 *maxy3;
    Fixed32 *minz3;
    Fixed32 *maxz3;
    int *display_flag;
    int *sorted_face_indices;            // Points to: [face_id1, face_id2, ...] sorted by z_max
    int face_count;                      // Actual number of loaded faces
    int total_indices;                   // Total indices across all faces (sum of all vertex_counts)
} FaceArrays3D;

/**
 * Structure Face3D
 * 
 * DESCRIPTION:
 *   Represents a face (polygon) of a 3D object. A face is defined
 *   by a list of indices pointing to vertices in the model's
 *   vertex array.
 * 
 * FIELDS:
 *   vertex_count    : Number of vertices composing this face (3+ for polygon)
 *   vertex_indices  : Array of vertex indices (1-based numbering as in OBJ format)
 * 
 * NOTES:
 *   - Indices are stored in base 1 (first vertex = index 1)
 *   - Conversion to base 0 needed to access the C array
 *   - Maximum MAX_FACE_VERTICES vertices per face (now 6 for triangles/quads/hexagons)
 *   - LEGACY STRUCTURE: Now replaced by FaceArrays3D for parallel array storage
 */
typedef struct {
    int vertex_count;                           // Number of vertices in the face
    int vertex_indices[MAX_FACE_VERTICES];     // Vertex indices (base 1, max 6 for polygons)
    Fixed32 z_max;                             // Maximum depth of the face (for sorting, Fixed Point)
    int display_flag;                          // 1 = display face, 0 = don't display (behind camera)
} Face3D;

/**
 * Structure DynamicPolygon
 * 
 * DESCRIPTION:
 *   Structure compatible with QuickDraw for drawing polygons.
 *   This structure must be dynamically allocated because its size
 *   varies according to the number of points in the polygon.
 * 
 * FIELDS:
 *   polySize    : Total size of the structure in bytes
 *   polyBBox    : Polygon bounding box rectangle
 *   polyPoints  : Array of polygon points in screen coordinates
 * 
 * QUICKDRAW FORMAT:
 *   QuickDraw expects a structure with header (size + bbox) followed
 *   by points. The size must include the header + all points.
 */
typedef struct {
    int polySize;                               // Total structure size (bytes)
    Rect polyBBox;                             // Bounding box rectangle
    Point polyPoints[MAX_FACE_VERTICES];       // Polygon points (screen coordinates)
} DynamicPolygon;

/**
 * Structure ObserverParams
 * 
 * DESCRIPTION:
 *   Contains all parameters defining the position and orientation
 *   of the observer (camera) in 3D space, as well as projection
 *   parameters.
 * 
 * FIELDS:
 *   angle_h  : Horizontal rotation angle of the observer (degrees)
 *              Rotation around Y-axis (left/right)
 *   angle_v  : Vertical rotation angle of the observer (degrees)
 *              Rotation around X-axis (up/down)
 *   angle_w  : Screen projection rotation angle (degrees)
 *              Rotation in the final 2D plane
 *   distance : Distance from observer to model center
 *              Larger = smaller object, smaller = larger object
 * 
 * MATHEMATICAL NOTES:
 *   - Angles are in degrees (converted to radians for calculations)
 *   - Distance affects perspective and apparent size
 *   - angle_w allows final rotation to adjust orientation
 */
typedef struct {
    int angle_h;   // Observer horizontal angle (degrees)
    int angle_v;   // Observer vertical angle (degrees)
    int angle_w;   // 2D projection rotation angle (degrees)
    Fixed32 distance;  // Observer-object distance (perspective, Fixed Point)
} ObserverParams;

/**
 * Structure Model3D
 * 
 * DESCRIPTION:
 *   Main structure containing all data of a 3D model.
 *   It groups vertices, faces, and associated counters.
 * 
 * FIELDS:
 *   vertices      : Pointer to dynamic vertex array
 *   faces         : Pointer to dynamic face array
 *   vertex_count  : Actual number of loaded vertices
 *   face_count    : Actual number of loaded faces
 * 
 * MEMORY MANAGEMENT:
 *   - Arrays are dynamically allocated (malloc)
 *   - Allows exceeding Apple IIGS stack limits
 *   - Mandatory cleanup with destroyModel3D()
 * 
 * USAGE:
 *   Model3D* model = createModel3D();
 *   loadModel3D(model, "file.obj");
 *   // ... usage ...
 *   destroyModel3D(model);
 */
typedef struct {
    VertexArrays3D vertices;          // Parallel arrays for all vertex data
    FaceArrays3D faces;               // Parallel arrays for all face data

    /* Auto-scale metadata (non-destructive) */
    Fixed32 auto_scale;               // Fixed32 scale factor applied (FIXED_ONE if none)
    Fixed32 auto_center_x, auto_center_y, auto_center_z; // center used during scaling
    int auto_scaled;                  // 1 if auto-scaling has been applied
    int auto_centered;                // 1 if auto-scale used centering

    /* Brute auto-fit suggestions (computed at vertex read) */
    Fixed32 auto_suggested_distance;      // Suggested observer distance (Fixed32) computed as k * max_dim
    Fixed32 auto_suggested_proj_scale;    // Suggested projection scale (Fixed32 pixels per projected unit)
    Fixed32 auto_proj_scale;               // Applied projection scale (Fixed32) when auto-fit is used
    int auto_fit_ready;                   // 1 if suggestions are ready
    int auto_fit_applied;                 // 1 if suggestions were applied to the current view

    /* Optional backup of original coordinates to allow exact revert and dynamic scale adjustments */
    Fixed32 *orig_x;                   // NULL if no backup
    Fixed32 *orig_y;
    Fixed32 *orig_z;

    float *coord_buf;               // scratch buffer for converted float coordinates (x,y,z interleaved)
    int coord_buf_capacity;         // capacity in number of vertices for coord_buf

    /* Bounding sphere (computed once at load) */

} Model3D;

// ============================================================================
//                       FUNCTION DECLARATIONS
// ============================================================================

/**
 * FIXED POINT MATHEMATICAL FUNCTIONS
 * ===================================
 */
/**
 * OBJ FILE READING FUNCTIONS
 * ===========================
/**
 * readVertices
 * 
 * DESCRIPTION:
 *   Reads vertices (3D points) from an OBJ format file.
 *   Searches for lines starting with "v " and extracts X,Y,Z coordinates.
 * 
 * PARAMETERS:
 *   filename     : OBJ filename to read
 *   vertices     : Destination array to store vertices
 *   max_vertices : Maximum array size (overflow protection)
 * 
 * RETURN:
 *   Number of successfully read vertices, or -1 on error
 * 
 * OBJ FORMAT:
 *   v 1.234 5.678 9.012
 *   v -2.5 0.0 3.14
 */
int readVertices(const char* filename, VertexArrays3D* vtx, int max_vertices, Model3D* owner);
// Note: readVertices now computes bbox center and stores it in owner->auto_center_{x,y,z} if owner!=NULL
// It does NOT modify vertex coordinates (no auto-translation).

/**
 * readFaces
 * 
 * DESCRIPTION:
 *   Reads faces (polygons) from an OBJ format file.
 *   Searches for lines starting with "f " and extracts vertex indices.
 * 
 * PARAMETERS:
 *   filename   : OBJ filename to read
 *   faces      : Destination array to store faces
 *   max_faces  : Maximum array size (overflow protection)
 * 
 * RETURN:
 *   Number of successfully read faces, or -1 on error
 * 
 * OBJ FORMAT:
 *   f 1 2 3        (triangle with vertices 1, 2, 3)
 *   f 4 5 6 7      (quadrilateral with vertices 4, 5, 6, 7)
 */

void getObserverParams(ObserverParams* params, Model3D* model);

/**
 * GRAPHIC RENDERING FUNCTIONS
 * ===========================
 */

/**
 * drawPolygons
 * 
 * DESCRIPTION:
 *   Draws all polygons (faces) of the 3D model on screen using
 *   Apple IIGS QuickDraw API. Each face is rendered with a different
 *   color for visualization.
 * 
 * PARAMETERS:
 *   vertices   : Array of vertices with calculated 2D coordinates
 *   faces      : Array of faces to draw
 *   face_count : Number of faces in the array
 * 
 * ALGORITHM:
 *   1. QuickDraw graphics mode initialization
 *   2. For each face:
 *      - Check vertex visibility
 *      - Create QuickDraw polygon structure
 *      - Calculate bounding box
 *      - Dynamic memory allocation
 *      - Drawing with PaintPoly()
 *      - Memory cleanup
 * 
 * COLOR MANAGEMENT:
 *   Cyclic colors based on face index (i % 15 + 1)
 * 
 * OPTIMIZATIONS:
 *   - Faces with less than 3 visible vertices ignored
 *   - Off-screen vertices handled correctly
 */
void drawPolygons(Model3D* model, int* vertex_count, int face_count, int vertex_count_total);
void drawPolygons_jitter(Model3D* model, int* vertex_count, int face_count, int vertex_count_total);
void calculateFaceDepths(Model3D* model, Face3D* faces, int face_count);

// Global container for *inconclusive* ordering pairs
// -------------------------------------------------------------------
// Purpose:
//   During the Newell/Sancha ordering pass, some face pairs cannot be conclusively
//   ordered by the available tests (depth, bbox, plane votes, sampling). Instead of
//   performing geometric splits (which would require mesh mutation), such pairs are
//   recorded here for diagnostic purposes and optional framing in the UI.
//
// Implementation notes:
//   - An `InconclusivePair` stores two face indices (face1, face2).
//   - The buffer is preallocated to `face_count * 4` entries (heuristic) for performance
//     and to avoid repeated reallocations during the sorting/correction pass.
//   - `inconclusive_pairs_count` tracks the number of recorded pairs this frame.
//   - Consumers: `frameInconclusivePairs()` draws these pairs; diagnostic logs print counts.
//
// Lifetime:
//   Allocated by `painter_newell_sancha()` at the start of a sort pass and freed when the
//   pass completes (or when resizing fails). The buffer is global to avoid per-call stack
//   pressure and to simplify diagnostics.
// -------------------------------------------------------------------
typedef struct {
    int face1;
    int face2;
} InconclusivePair;

// Global buffers; initialized per-call in painter_newell_sancha
static int inconclusive_pairs_capacity = 0;
static InconclusivePair* inconclusive_pairs = NULL;
static int inconclusive_pairs_count = 0;

// Lightweight instrumentation counters for projected_polygons_overlap (accumulated)
static unsigned long overlapCheckCount = 0;       // total calls
static unsigned long overlapSegiAccept = 0;      // accepted by segment intersection
static unsigned long overlapSampleAccept = 0;    // accepted by sampling
static unsigned long overlapClipCalls = 0;       // clipping fallback calls
static unsigned long overlapClipAccept = 0;      // clipping accepted (area >= threshold)


/* Shared buffers for Sutherland-Hodgman clipping (reused to avoid malloc/free) */
static long long *clip_sx = NULL, *clip_sy = NULL, *clip_tx = NULL, *clip_ty = NULL; /* Fixed16.16 stored as 64-bit */
static int clip_bufcap = 0;
