/*
 * ============================================================================
 *              DONYGS.cc - Fixed32 Optimized Implementation
 * ============================================================================
 *
 * Purpose:
 *   High-performance 3D viewer implementing the DONYGS painter algorithm with
 *   multiple painter modes (FAST / FIXED / CORRECT / CORRECTV2 / NEWELL_SANCHAV2). FLOAT mode is archived (see `chutier.txt`). Reads simplified OBJ files
 *   (vertices "v" and faces "f"), transforms them into observer space,
 *   projects to 2D screen coordinates and renders filled polygons.
 *
 * Highlights (2026):
 *   - Target: Apple IIGS (ORCA/C, QuickDraw)
 *   - Painter modes:
 *       * FAST  : z_mean + bbox tests (very fast, less robust)
 *       * FIXED : full fixed-point Newell/Sancha implementation with pairwise
 *                 tests and corrections (robust)
 *       * CORRECT : Advanced ordering correction with local face reordering (slower)
 *       * CORRECTV2 : Experimental local correction (painter_correctV2) for pathological cases
 *       * NEWELL_SANCHAV2 : Bubble-style variant for conservative local reordering
 *       * FLOAT : float-based painter (ARCHIVED: implementation moved to `chutier.txt`)
 *                 Calls in `DONYGS.cc` are commented out; to restore, move the implementation back from `chutier.txt`
 *   - Observer-space back-face culling toggle (`B` key) that marks faces as
 *     non-displayable and restricts sorting to visible faces for correctness
 *     and speed when enabled.
 *   - Diagnostic helpers: inconclusive pair recording, `frameInconclusivePairs()`
 *     for visual debugging, and timing instrumentation to measure stages.
 *   - Performance: heavy Fixed32 optimizations, precomputed trig tables, buffer
 *     reuse to avoid allocations and selective sorting when culling is active.
 *
 * Responsibilities (per-frame):
 *   - Transform vertices (Fixed32) to observer-space (xo/yo/zo).
 *   - Project to integer screen coordinates (x2d/y2d).
 *   - Compute per-face metrics: z_min/z_max/z_mean, bbox, plane coefficients.
 *   - Dispatch to the selected painter to populate `sorted_face_indices`.
 *   - Provide debug overlays (frame inconclusive pairs) and logging.
 *
 * Notes:
 *   - This file has evolved beyond the original Apple IIGS reference to include
 *     platform-specific frontends while preserving the algorithmic core.
 *   - Experimental: `painter_correctV2` and the in-memory face-splitting logic
 *     are work-in-progress and may be unstable on complex models; use with
 *     caution and enable only for debugging/analysis.
 *   - The code is optimized for interactive use; use the FAST painter for
 *     high frame-rate, or FIXED/CORRECT modes for correctness on tricky geometry.
 *     (FLOAT is archived in `chutier.txt`.)
 *
 * Author: Bruno
 * Date: 2026-01-23
 * ============================================================================
 */

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

#pragma memorymodel 1

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
static int user_fill_color = -1;  // Interior color
static int user_frame_color = -1; // Frame color
// Random color buffers (allocated per face when random mode is active)
static unsigned char* random_fill_colors = NULL;
static unsigned char* random_frame_colors = NULL;
static int random_colors_capacity = 0;

#define PAINTER_MODE_FAST 0
#define PAINTER_MODE_FIXED 1
#define PAINTER_MODE_FLOAT 2
#define PAINTER_MODE_CORRECT 3
#define PAINTER_MODE_CORRECTV2 4
#define PAINTER_MODE_NEWELL_SANCHAV2 5
#define PAINTER_MODE_NEWELL_SANCHAV3 6

static int painter_mode = PAINTER_MODE_FAST; // 0=fast,1=fixed,2=float,3=correct,4=correctV2,5=newell_sanchaV2,6=newell_sanchaV3

// Runtime toggle kept for compatibility; main() may set this and we propagate to painter_mode
static int use_float_painter = 0;

// --- Reusable scratch buffers for float painter (ARCHIVED: implementation moved to `chutier.txt`).
// Buffers retained for potential restoration; the FLOAT painter implementation is archived and calls are commented out in `DONYGS.cc`.
static float *float_xo = NULL, *float_yo = NULL, *float_zo = NULL; static int float_vcap = 0;
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
#define ENABLE_DEBUG_SAVE 0     // 1 = Enable debug save (SLOW!), 0 = Disable
//#define PERFORMANCE_MODE 0      // 1 = Optimized performance mode, 0 = Debug mode
// OPTIMIZATION: Performance mode - disable printf
#define PERFORMANCE_MODE 1      // 1 = no printf, 0 = normal printf

#define MAX_LINE_LENGTH 256     // Maximum file line size
#define MAX_VERTICES 6000       // Maximum vertices in a 3D model
#define MAX_FACES 6000          // Maximum faces in a 3D model (using parallel arrays)
#define MAX_FACE_VERTICES 6     // Maximum vertices per face (triangles/quads/hexagons)
#define CENTRE_X 160            // Screen center in X (320/2)
#define CENTRE_Y 100            // Screen center in Y (200/2)
//#define mode 640              // Graphics mode 640x200 pixels
#define mode 320                // Graphics mode 320x200 pixels

// ============================================================================
//                          DATA STRUCTURES
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
    int *vertex_indices_buffer;          // Points to: [v1, v2, v3, v1, v2, v3, v4, v1, v2, v3, ...]
    int *vertex_indices_ptr;             // Points to: [offset0, offset3, offset6, offset10, ...]
    Fixed32 *z_min;
    Fixed32 *z_max;
    Fixed32 *z_mean;                     // Mean zo per face (computed in calculateFaceDepths)
    Fixed64 *plane_a;                     // per-face normalized normal X (a) stored as Fixed64 (16.16)
    Fixed64 *plane_b;                     // per-face normalized normal Y (b) stored as Fixed64 (16.16)
    Fixed64 *plane_c;                     // per-face normalized normal Z (c) stored as Fixed64 (16.16)
    Fixed64 *plane_d;                     // per-face D term for plane equation stored as Fixed64 (16.16)
    int *minx;                            // cached 2D bounding box (projected x/y)
    int *maxx;
    int *miny;
    int *maxy;
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


/**
 * PAINTER'S ALGORITHM - NEWELL / SANCHA (detailed)
 * =================================================
 * Overview:
 *  - The painter builds an ordering of faces for correct filled-polygon rendering without
 *    performing geometric splits. The algorithm follows the Newell/Sancha approach:
 *      1) Compute per-face depth metrics (z_min/z_max/z_mean) and planar coefficients.
 *      2) Initially sort faces by z_mean (descending) to obtain a broadly correct depth order.
 *      3) Use axis-aligned bounding-box overlap tests (X and Y) to detect candidate overlapping faces.
 *      4) For overlapping pairs, apply robust pairwise ordering tests using plane/triangle votes and
 *         local sampling; if the order is ambiguous, record the pair in the `inconclusive_pairs`
 *         buffer for diagnostic framing and later inspection (no automatic splitting).
 *
 * Visibility & culling:
 *  - When `cull_back_faces` is enabled, the painter operates primarily on the subset of faces
 *    with `display_flag == 1` (visible). Culled faces are appended afterwards to preserve
 *    stable indices and deterministic behavior of `sorted_face_indices`.
 *
 * Modes:
 *  - FAST: performs only low-cost tests (depth + bbox) and skips plane calculations for speed.
 *  - FIXED: Full fixed-point Newell/Sancha implementation with all pairwise tests and corrections.
 *  - FLOAT: Same algorithm implemented with float intermediates (optimized for clarity/speed on
 *           systems with faster float math); plane coefficients and buffers are cached.
 *
 * Notes:
 *  - The algorithm is intentionally conservative: inconclusive pairs are stored rather than
 *    forcing a risky swap that may introduce visual artifacts.
 *  - Use `frameInconclusivePairs()` in diagnostics to highlight such pairs in the rendered view.
 *
 * Usage:
 *  - Call `painter_newell_sancha(model, face_count)` (or a mode-specific variant) after
 *    `calculateFaceDepths()` / `processModelFast()` has computed observer-space coordinates.
 */

// Comparator support for qsort in painter_newell_sancha
static FaceArrays3D* qsort_faces_ptr_for_cmp = NULL;
static int cmp_faces_by_zmean(const void* pa, const void* pb) {
    int a = *(const int*)pa;
    int b = *(const int*)pb;
    Fixed32 za = qsort_faces_ptr_for_cmp->z_mean[a];
    Fixed32 zb = qsort_faces_ptr_for_cmp->z_mean[b];
    if (za > zb) return -1;   // larger z_mean first (descending)
    if (za < zb) return 1;
    if (a < b) return -1;     // tie-breaker: smaller index first
    if (a > b) return 1;
    return 0;
}



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

/**
 * FAST PUBLISHABLE PASS (painter_newell_sancha_fast)
 * --------------------------------------------------
 * Purpose:
 *   Very fast ordering pass intended for interactive / high-frame-rate use. This mode:
 *    - Sorts visible faces by `z_mean` only (stable sort via qsort with tie-breaker on index).
 *    - Performs only inexpensive overlap tests (axis-aligned X and Y bounding boxes) to
 *      detect obvious separations. No plane coefficients, no Newell plane tests, and no
 *      ordering corrections are computed.
 *
 * Tradeoffs:
 *   - Much faster and suitable for large models, but may leave ordering ambiguities
 *     unresolved (which shows up as graphical overlap artifacts for complex geometry).
 *   - When `cull_back_faces` is set, this pass operates on the subset of faces with
 *     `display_flag == 1` and appends culled faces after the visible list to keep indices
 *     stable for downstream users.
 */
void painter_newell_sancha_fast(Model3D* model, int face_count) {
    FaceArrays3D* faces = &model->faces;
    if (!faces->z_mean) return; // safety

    // Build list of faces to sort: when culling is enabled, only visible faces are sorted
    int i;
    int visible_count = face_count;
    if (cull_back_faces) {
        visible_count = 0;
        // put visible faces first
        for (i = 0; i < face_count; ++i) {
            if (faces->display_flag[i]) faces->sorted_face_indices[visible_count++] = i;
        }
        // append culled faces to keep array content stable
        int tail = visible_count;
        for (i = 0; i < face_count; ++i) {
            if (!faces->display_flag[i]) faces->sorted_face_indices[tail++] = i;
        }
    } else {
        for (i = 0; i < face_count; ++i) faces->sorted_face_indices[i] = i;
    }

    // Initial stable sort by z_mean (descending) with tie-breaker on index, only on visible faces
    qsort_faces_ptr_for_cmp = faces;
    qsort(faces->sorted_face_indices, visible_count, sizeof(int), cmp_faces_by_zmean);
    qsort_faces_ptr_for_cmp = NULL;
}

void debug_two_faces(Model3D* model, int f1, int f2) {
    startgraph(mode);
    drawFace(model, f1, 10, 1);
    keypress();
    drawFace(model, f2, 10, 1);
    keypress();
    endgraph();
    DoText();
}


void painter_newell_sancha(Model3D* model, int face_count) {
    if (use_float_painter) { //painter_newell_sancha_float(model, face_count); return; 
        }
    // ...existing code...
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int i, j;
    
    Fixed32* face_zmean = faces->z_mean;
    if (!face_zmean) return; // safety


    // * * * * * 
    // Step 1: Stable descending Z sort on the mean, with tie-breaker on original index
    // * * * * *

    long t_start = GetTick();
    // Build list of faces to sort.
    //
    // When observer-space back-face culling is enabled (`cull_back_faces == 1`), we
    // only **sort the subset** of faces that are considered visible (`display_flag == 1`).
    // Raison:
    //  - Performance: qsort is O(n log n); sorting fewer faces reduces CPU time for complex meshes.
    //  - Correctness: some painter tests rely on per-face plane data; excluding culled faces
    //    from the primary sort prevents spurious re-ordering and reduces false corrections.
    //
    // Implementation detail (stability):
    //  - We place all visible faces first in `faces->sorted_face_indices[0..visible_count-1]`.
    //  - Then we append the culled faces at the end of the array (unchanged relative order).
    //    This keeps the `sorted_face_indices` array full and *stable* across frames and toggles.
    //    The drawing code still checks `display_flag` and will skip culled faces, so appending
    //    them simply preserves indices without affecting rendering.
    //
    // Note: `visible_count` is the number of entries that must be passed to the sorting
    // routine (qsort) so that only visible faces are reordered.
    int visible_count = face_count;
    if (cull_back_faces) {
        visible_count = 0;
        for (i = 0; i < face_count; ++i) {
            if (faces->display_flag[i]) faces->sorted_face_indices[visible_count++] = i;
        }
        // Append culled (non-visible) faces after visible ones to preserve array stability.
        int tail = visible_count;
        for (i = 0; i < face_count; ++i) {
            if (!faces->display_flag[i]) faces->sorted_face_indices[tail++] = i;
        }
    } else {
        // If culling is disabled, include all faces in the initial list (simple identity)
        for (i = 0; i < face_count; i++) faces->sorted_face_indices[i] = i;
    }
    // Use qsort for O(n log n) sorting while preserving the exact tie-breaker, applied only to visible faces
    qsort_faces_ptr_for_cmp = faces;
    qsort(faces->sorted_face_indices, visible_count, sizeof(int), cmp_faces_by_zmean);
    qsort_faces_ptr_for_cmp = NULL;
    long t_end = GetTick();
    

    int swap_count = 0;
    int swapped = 0; // flag used for the correction loop

    // Ordered pairs cache: store definitive pairwise relations to avoid re-testing.
    typedef struct {
        int face1;  // Face that must be drawn before (farther)
        int face2;  // Face that must be drawn after (closer)
    } OrderedPair;

    int ordered_pairs_capacity = face_count * 4; // heuristic capacity to reduce reallocs
    OrderedPair* ordered_pairs = NULL;
    if (ordered_pairs_capacity > 0) {
        ordered_pairs = (OrderedPair*)malloc(ordered_pairs_capacity * sizeof(OrderedPair));
        if (!ordered_pairs) {
            ordered_pairs_capacity = 0; // fall back to not caching ordered pairs
        }
    }
    int ordered_pairs_count = 0;


    // Prepare global inconclusive buffer for diagnostic recording. These pairs are
    // intentionally left unresolved in order to avoid mesh splits; they can be
    // inspected with `frameInconclusivePairs()` for debugging.
    if (inconclusive_pairs) {
        free(inconclusive_pairs);
        inconclusive_pairs = NULL;
    }
    inconclusive_pairs_capacity = face_count * 4;
    if (inconclusive_pairs_capacity > 0) {
        inconclusive_pairs = (InconclusivePair*)malloc(inconclusive_pairs_capacity * sizeof(InconclusivePair));
        if (!inconclusive_pairs) {
            inconclusive_pairs_capacity = 0;
        }
    }
    inconclusive_pairs_count = 0;

    // Bubble-like correction passes (iterate until stable)
    do {
        swapped = 0;
   
        int t1 = 0;
        int t2 = 0;
        int t3 = 0;
        int t4 = 0;
        int t5 = 0;
        int t6 = 0;
        int t7 = 0;


        // Iterate consecutive pairs (only over visible faces if culling is enabled)
        for (i = 0; i < visible_count-1; i++) {
            int f1 = faces->sorted_face_indices[i];
            int f2 = faces->sorted_face_indices[i+1];

            // show faces to be tested (only in debug mode)
            #if ENABLE_DEBUG_SAVE
            debug_two_faces(model, f1, f2);
            #endif


            // Skip pairs already declared ordered by previous swaps or tests.
            // ordered_pairs stores definitive relations discovered earlier in the pass to
            // avoid repeated work (e.g., if we previously determined f2 < f1 we won't re-evaluate).
            int already_ordered = 0;
            int p;
            for (p = 0; p < ordered_pairs_count; p++) {
                if (ordered_pairs[p].face1 == f1 && ordered_pairs[p].face2 == f2) {
                    already_ordered = 1; break; // f1 before f2
                }
                if (ordered_pairs[p].face1 == f2 && ordered_pairs[p].face2 == f1) {
                    already_ordered = 1; break; // f2 before f1 (inverse relation)
                }
            }
            if (already_ordered) continue;

            // Test 1: Depth overlap (cheap rejection/acceptance)
            // - If f2's farthest point is in front of f1's nearest point, the order f1->f2
            //   is respected (no swap). Conversely, if f1's farthest is in front of f2's nearest
            //   we *must* swap. This is a conservative quick test that filters many non-overlapping cases.
            t1++;
            // Test 1 : Depth overlap
            // Depth quick test: if farthest point of P2 is in front of nearest point of P1, order is respected
            if (faces->z_max[f2] <= faces->z_min[f1]) continue;
            if (faces->z_max[f1] <= faces->z_min[f2]) goto do_swap;
            
            // Bounding Box 2D overlap (split into X and Y parts)
            // Use cached bounding boxes (computed in calculateFaceDepths)

            t2++;
            // Test 2 : X overlap only (axis-aligned bbox separation test)
            // If bounding boxes do not intersect on X, faces cannot overlap on screen and
            // the current order is safe.
            int minx1 = faces->minx[f1], maxx1 = faces->maxx[f1], miny1 = faces->miny[f1], maxy1 = faces->maxy[f1];
            int minx2 = faces->minx[f2], maxx2 = faces->maxx[f2], miny2 = faces->miny[f2], maxy2 = faces->maxy[f2];

            if (maxx1 <= minx2 || maxx2 <= minx1) continue; // separated on X
            
            t3++;
            // Test 3 : Y overlap only (axis-aligned bbox separation test)
            // Similarly, if separated on Y, faces do not overlap and no swap is needed.
            if (maxy1 <= miny2 || maxy2 <= miny1) continue; // separated on Y


            if (!projected_polygons_overlap(model, f1, f2)) {
                // bounding boxes overlap but polygons do not overlap
                continue;
                
            }

            // Use cached plane normals and d terms computed in calculateFaceDepths
            int n1 = faces->vertex_count[f1];
            int n2 = faces->vertex_count[f2];
            int offset1 = faces->vertex_indices_ptr[f1];
            int offset2 = faces->vertex_indices_ptr[f2];
            int k;
            Fixed64 a1 = faces->plane_a[f1];
            Fixed64 b1 = faces->plane_b[f1];
            Fixed64 c1 = faces->plane_c[f1];
            Fixed64 d1 = faces->plane_d[f1];
            Fixed64 a2 = faces->plane_a[f2];
            Fixed64 b2 = faces->plane_b[f2];
            Fixed64 c2 = faces->plane_c[f2];
            Fixed64 d2 = faces->plane_d[f2];
            Fixed32 epsilon = FLOAT_TO_FIXED(0.01f);

            int obs_side1 = 0; // observer side relative to plane of f1: +1, -1, or 0 (inconclusive)
            int obs_side2 = 0; // observer side relative to plane of f2: +1, -1, or 0 (inconclusive)
            int side;           // vertex side.
            int all_same_side; // flag indicating whether all vertices are on the same side
            int all_opposite_side; // flag indicating whether all vertices are on the opposite side
            /* test_value replaced by Fixed64 accumulators inside loops to avoid Fixed32 overflow */
            Fixed64 test_value64 = 0;

            // ********************* TEST 4 *********************
            t4++;
            // Test 4: Is f2 entirely on the same observer-side of f1's plane?
            // - Evaluate face-plane sign for the observer (d1) and then test every vertex of f2
            //   against f1's plane (A1*x + B1*y + C1*z + D1). If all vertices are on the
            //   same side as the observer (within epsilon), then f2 is conclusively in front
            //   of f1 (no swap required). If any vertex is on the opposite side, the test fails
            //   and we continue to stronger tests.

            #if ENABLE_DEBUG_SAVE
            printf("\n**** Test 4 : Testing faces %d and %d\n", f1, f2);
            printf("Face coefs: a1=%f, b1=%f, c1=%f, d1=%f\n", FIXED64_TO_FLOAT(a1), FIXED64_TO_FLOAT(b1), FIXED64_TO_FLOAT(c1), FIXED64_TO_FLOAT(d1));
            #endif

            obs_side1 = 0; // sign of d1: +1, -1 or 0 (inconclusive)
            if (d1 > (Fixed64)epsilon) obs_side1 = 1; 
            else if (d1 < -(Fixed64)epsilon) obs_side1 = -1;
            else goto skipT4; // si l'observateur est sur le plan, on ne peut rien conclure, il faut faire d'autres tests
            all_same_side = 1;

            #if ENABLE_DEBUG_SAVE
                printf("FOR loop start\n");
                printf("obs_side1 = %d\n", obs_side1);
                printf("test_values for face %d must be of the same sign as obs_side1\n", f2);
            #endif
            for (k=0; k<n2; k++) {
                    int v = faces->vertex_indices_buffer[offset2+k]-1;
                    /* Accumulate in 64-bit to avoid overflow (each product is >> FIXED_SHIFT to keep Fixed32 scale) */
                    Fixed64 acc = 0;
                    acc  = (((Fixed64)a1 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
                    acc += (((Fixed64)b1 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
                    acc += (((Fixed64)c1 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
                    acc += (Fixed64)d1; // d1 already Fixed32 scale
                    #if ENABLE_DEBUG_SAVE
                        printf("k = %d, vertex index = %d, vtx = (%f, %f, %f)\n", k, v+1, FIXED_TO_FLOAT(vtx->xo[v]), FIXED_TO_FLOAT(vtx->yo[v]), FIXED_TO_FLOAT(vtx->zo[v]));
                        printf("test_value = %f\n", ((double)acc) / FIXED_SCALE);
                    #endif
                    if  (acc > (Fixed64)epsilon) side = 1;
                    else if (acc < -(Fixed64)epsilon) side = -1;
                    else continue; // si le vertex est sur le plan, on l'ignore et on passe au vertex suivant
                    if (obs_side1 != side) { 
                        // if a vertex is on the other side, break the loop
                        // and set the flag to 0 to indicate the test failed (move to next test)
                        all_same_side = 0; 
                        #if ENABLE_DEBUG_SAVE
                            printf("Test 4 failed for faces %d and %d\n", f1, f2);
                            keypress();
                        #endif  
                        break; 
                        }
            }
            #if ENABLE_DEBUG_SAVE
                printf("FOR loop stop\n");
            #endif

            // test 4 passed
            if (all_same_side) { 
                #if ENABLE_DEBUG_SAVE
                printf("Test 4 passed for Faces %d and %d\n", f1, f2);
                keypress();
                #endif

                continue; // faces are ordered correctly, move to next pair
            }

            skipT4:

            // ********************* TEST 5 ********************
            t5++;
            // Test 5: Is f1 entirely on the opposite side of f2's plane relative to the observer?
            // - This is symmetric to Test 4: if every vertex of f1 is strictly on the opposite
            //   side of f2's plane from the observer, then f1 is behind f2 and no swap is needed.
            // - Both Test 4 and Test 5 are relatively cheap and frequently decisive on planar geometry.

            #if ENABLE_DEBUG_SAVE
            printf("Test 5 : Testing faces %d and %d\n", f1, f2);
            printf("Face coefs: a2=%f, b2=%f, c2=%f, d2=%f\n", FIXED64_TO_FLOAT(a2), FIXED64_TO_FLOAT(b2), FIXED64_TO_FLOAT(c2), FIXED64_TO_FLOAT(d2) );
            #endif

            obs_side2 = 0; // sign of d1: +1, -1 or 0 (inconclusive)
            if (d2 > (Fixed64)epsilon) obs_side2 = 1; 
            else if (d2 < -(Fixed64)epsilon) obs_side2 = -1;
            else goto skipT5; // si l'observateur est sur le plan, on ne peut rien conclure, il faut faire d'autres tests
            all_opposite_side = 1;

            #if ENABLE_DEBUG_SAVE
                printf("FOR loop start\n");
                printf("obs_side2 = %d\n", obs_side2);
                printf("test_values for face %d must be of the opposite sign as obs_side2\n", f1);
            #endif

            for (k=0; k<n1; k++) {
                int v = faces->vertex_indices_buffer[offset1+k]-1;
                Fixed64 acc = 0;
                acc  = (((Fixed64)a2 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
                acc += (((Fixed64)b2 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
                acc += (((Fixed64)c2 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
                acc += (Fixed64)d2;
                if  (acc > (Fixed64)epsilon) side = 1;
                else if (acc < -(Fixed64)epsilon) side = -1;
                else continue; // si le vertex est sur le plan, on l'ignore et on passe au vertex suivant
                if (obs_side2 == side) {
                    // if a vertex is on the same side, break the loop
                    // and set the flag to 0 to indicate the test failed (move to next test)
                    all_opposite_side = 0; 
                    #if ENABLE_DEBUG_SAVE
                            printf("Test 5 failed for faces %d and %d\n", f1, f2);
                            keypress();
                        #endif  
                    break; }
                }

                // test 5 passed
                if (all_opposite_side) { // faces are ordered correctly, move to next pair
                    #if ENABLE_DEBUG_SAVE
                    printf("Test 5 passed for Faces %d and %d\n", f1, f2);
                    keypress();
                    #endif

                    continue; // faces are ordered correctly, move to next pair
                }
                
            skipT5:

            // ********************* TEST 6 *********************
            t6++;
            // Test 6: Is f2 entirely on the opposite side of f1's plane relative to the observer?
            // - If all vertices of f2 are strictly on the opposite side, then f2 is behind f1 and
            //   an adjacent swap is required (f2 should come after f1). This test often detects
            //   clear occlusion relations and triggers swaps.
            #if ENABLE_DEBUG_SAVE
            printf("Test 6 : Testing faces %d and %d\n", f1, f2);
            printf("Face coefs: a1=%f, b1=%f, c1=%f, d1=%f\n", FIXED64_TO_FLOAT(a1), FIXED64_TO_FLOAT(b1), FIXED64_TO_FLOAT(c1), FIXED64_TO_FLOAT(d1));
            #endif

            obs_side1 = 0; // sign of d1: +1, -1 or 0 (inconclusive)
            if (d1 > (Fixed64)epsilon) obs_side1 = 1; 
            else if (d1 < -(Fixed64)epsilon) obs_side1 = -1;
            else goto skipT6; // si l'observateur est sur le plan, on ne peut rien conclure, il faut faire d'autres tests

            all_opposite_side = 1;
            #if ENABLE_DEBUG_SAVE
                printf("FOR loop start\n");
                printf("obs_side1 = %d\n", obs_side1);
                printf("test_values for face %d must be of the opposite sign as obs_side1\n", f2);
            #endif
            for (k=0; k<n2; k++) {
                int v = faces->vertex_indices_buffer[offset2+k]-1;
                int side;
                Fixed64 acc = 0;
                acc  = (((Fixed64)a1 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
                acc += (((Fixed64)b1 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
                acc += (((Fixed64)c1 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
                acc += (Fixed64)d1;

                #if ENABLE_DEBUG_SAVE
                    printf("k = %d, vertex index = %d, vtx = (%f, %f, %f)\n", k, v+1, FIXED_TO_FLOAT(vtx->xo[v]), FIXED_TO_FLOAT(vtx->yo[v]), FIXED_TO_FLOAT(vtx->zo[v]));
                    printf("test_value = %f\n", ((double)acc) / FIXED_SCALE);
                #endif

                if  (acc > (Fixed64)epsilon) side = 1;
                else if  (acc < -(Fixed64)epsilon) side = -1;
                else continue; // si le vertex est sur le plan, on l'ignore et on passe au vertex suivant
                if (obs_side1 == side) { 
                    all_opposite_side = 0;
                    #if ENABLE_DEBUG_SAVE
                            printf("Test 6 failed for faces %d and %d\n", f1, f2);
                            keypress();
                        #endif  
                    break; 
                    }
                }

                if (all_opposite_side == 1) { // test 6 passed
                // f2 is on the opposite side of the observer, therefore f2 is behind f1 ==> swap required
                    #if ENABLE_DEBUG_SAVE
                    printf("Test 6 passed for faces %d and %d\n", f1, f2);
                    keypress();
                    #endif 

                    goto do_swap;
                }



            skipT6: ;

            // ********************* TEST 7 *********************
            t7++;
            // Test 7: Is f1 entirely on the same side of f2's plane as the observer?
            // - Symmetric to Test 6: if all vertices of f1 are on the observer side of f2's plane,
            //   then f1 is in front of f2 and f1 should be drawn after f2 (swap required).
            // - Passing Test 7 is a definitive reason to swap without further sampling.

            #if ENABLE_DEBUG_SAVE
            printf("Test 7 : Testing faces %d and %d\n", f1, f2);
            printf("Face coefs: a1=%f, b1=%f, c1=%f, d1=%f\n", FIXED64_TO_FLOAT(a2), FIXED64_TO_FLOAT(b2), FIXED64_TO_FLOAT(c2), FIXED64_TO_FLOAT(d2));
            #endif
            obs_side2 = 0; // sign of d1: +1, -1 or 0 (inconclusive)
            if (d2 > (Fixed64)epsilon) obs_side2 = 1; 
            else if (d2 < -(Fixed64)epsilon) obs_side2 = -1;
            else goto skipT7; // si l'observateur est sur le plan, on ne peut rien conclure, il faut faire d'autres tests

            all_same_side = 1;
            #if ENABLE_DEBUG_SAVE
                printf("FOR loop start\n");
                printf("obs_side1 = %d\n", obs_side1);
                printf("test_values for face %d must be of the opposite sign as obs_side1\n", f2);
            #endif

            for (k=0; k<n1; k++) {
                int v = faces->vertex_indices_buffer[offset1+k]-1;
                int side;
                Fixed64 acc = 0;
                acc  = (((Fixed64)a2 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
                acc += (((Fixed64)b2 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
                acc += (((Fixed64)c2 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
                acc += (Fixed64)d2;
                
                #if ENABLE_DEBUG_SAVE
                    printf("k = %d, vertex index = %d, vtx = (%f, %f, %f)\n", k, v+1, FIXED_TO_FLOAT(vtx->xo[v]), FIXED_TO_FLOAT(vtx->yo[v]), FIXED_TO_FLOAT(vtx->zo[v]));
                    printf("test_value = %f\n", ((double)acc) / FIXED_SCALE);
                #endif

                if  (acc > (Fixed64)epsilon) side = 1;
                else if  (acc < -(Fixed64)epsilon) side = -1;
                else continue; // si le vertex est sur le plan, on l'ignore et on passe au vertex suivant
                if (obs_side2 != side) { 
                    all_same_side = 0; 
                    #if ENABLE_DEBUG_SAVE
                            printf("Test 7 failed for faces %d and %d\n", f1, f2);
                            keypress();
                        #endif 
                    break; 
                    }
            }
                // f1 is not on the same side as the observer, so f1 is not in front of f2
                // on ne doit pas échanger l'ordre des faces
                // aucun test n'a permis de conclure : on signale non concluant
                if (all_same_side == 0)  goto skipT7;

                // f1 est devant f2, on doit échanger l'ordre
                else {
                    #if ENABLE_DEBUG_SAVE
                    printf("Test 7 passed for faces %d and %d\n", f1, f2);
                    keypress();
                    #endif 

                    goto do_swap;
                }




                
            do_swap: {

                #if ENABLE_DEBUG_SAVE
                    printf("Swapping faces %d and %d\n", f1, f2);
                #endif

                // Perform adjacent swap: this is an in-place stable operation and keeps
                // changes local (simple bubble logic). We record the definitive ordered
                // relation (f2 before f1 after swap) into `ordered_pairs` when possible
                // so subsequent passes skip redundant checks.
                int tmp = faces->sorted_face_indices[i];
                faces->sorted_face_indices[i] = faces->sorted_face_indices[i+1];
                faces->sorted_face_indices[i+1] = tmp;
                swapped = 1;
                swap_count++;

                // Record the pair as an established ordering (if capacity permits).
                // If we exceed capacity we silently drop the record — this only affects
                // performance (more re-evaluation), not correctness.
                if (ordered_pairs != NULL && ordered_pairs_count < ordered_pairs_capacity) {
                    ordered_pairs[ordered_pairs_count].face1 = f2; // now before
                    ordered_pairs[ordered_pairs_count].face2 = f1; // now after
                    ordered_pairs_count++;
                }

                // After a swap we skip ahead to the next pass iteration (goto ends current pair loop)
                goto endfor;
            }

        
        skipT7: 
        // Si on arrive ici, c'est que auncun test n'a pas permis de conclure
        // We should perform clipping of f1 by f2 (or vice versa), but it is not done now
        #if ENABLE_DEBUG_SAVE
                printf("NON CONCLUTANT POUR LES FACES %d ET %d\n", f1, f2);
                keypress();
        #endif    
       if (inconclusive_pairs != NULL && inconclusive_pairs_count < inconclusive_pairs_capacity) {
                inconclusive_pairs[inconclusive_pairs_count].face1 = f1;
                inconclusive_pairs[inconclusive_pairs_count].face2 = f2;
                inconclusive_pairs_count++;
            }
        // add them to the ordered-pairs list so they are not retested
        // if tests were inconclusive, preserve the current ordering
        if (ordered_pairs != NULL && ordered_pairs_count < ordered_pairs_capacity) {
                ordered_pairs[ordered_pairs_count].face1 = f2;
                ordered_pairs[ordered_pairs_count].face2 = f1;
                ordered_pairs_count++;
            }
        
        endfor: ;
        } // FIN de la boucle for int i=0; i<face_count-1; i++

        #if ENABLE_DEBUG_SAVE
            printf("Pass completed, swaps this pass: %d\n", swap_count);
            printf("t1=%d t2=%d t3=%d t4=%d t5=%d t6=%d t7=%d\n", t1, t2, t3, t4, t5, t6, t7);
            keypress();
        #endif

    } while (swapped);
    // Fin du tri à bulle

    
    #if ENABLE_DEBUG_SAVE
        printf("Total swaps: %d, Inconclusive pairs: %d, ordored pairs: %d\n", swap_count, inconclusive_pairs_count, ordered_pairs_count);
        keypress();
    #endif

    // Free the memory of the ordered pairs list
    if (ordered_pairs) {
        free(ordered_pairs);
    }  
}

/* Float-based painter: reproduces Windows numeric behaviour exactly
   Implemented as non-destructive function; enable via env var USE_FLOAT_PAINTER=1 */

/* painter_correct
 * ----------------
 * Deterministically adjust `faces->sorted_face_indices` in-place using the
 * exact geometric plane tests performed by `pair_plane_before` and `pair_plane_after`.
 *
 * Algorithm overview (safe & conservative):
 *  - First, partition faces into back-faces (plane_d <= 0) and front-faces (plane_d > 0)
 *    while preserving z_mean order within each group.
 *  - For each face id `target` within its partition, run two checks:
 *      1) Inspect faces *before* `target` in the current order. If one or more
 *         of these faces are *misplaced* (the pairwise plane tests indicate they
 *         should be after `target`) AND their projected polygons *overlap*
 *         (strict 2D overlap; touching-only is NOT overlap here), then move
 *         `target` to be *before* the earliest such misplaced face.
 *      2) Inspect faces *after* `target`. If one or more of these faces are
 *         misplaced (they should be before `target`) AND overlap with `target`,
 *         move `target` to be *after* the latest such misplaced face.
 *
 * Performance & correctness notes:
 *  - We first perform a fast axis-aligned bounding-box (AABB) rejection for
 *    each candidate pair. Only if bounding boxes intersect do we call the
 *    heavier `projected_polygons_overlap` (edge-intersection + containment)
 *    and the pairwise plane tests (`pair_plane_before`/`pair_plane_after`).
 *  - `projected_polygons_overlap` treats touching-only cases (shared edge or
 *    single vertex) as NON-overlap; this is intentional and consistent with
 *    the rest of the program.
 *  - The routine only performs local moves that are conservative with respect
 *    to the pair tests — it does not attempt global reordering heuristics.
 *  - Back-faces are corrected only if backface culling is OFF; otherwise they
 *    won't be rendered and correction would be wasted effort.
 *
 * Return value: number of element moves applied to `sorted_face_indices`.
 */
static int move_element_remove_and_insert(int *arr, int n, int from, int insert_idx) {
    if (from < 0 || from >= n) return 0;
    if (insert_idx < 0) insert_idx = 0;
    if (insert_idx > n-1) insert_idx = n-1;
    if (from == insert_idx) return 0;
    int val = arr[from];
    if (insert_idx < from) {
        /* shift left region [insert_idx..from-1] right by 1 */
        memmove(&arr[insert_idx+1], &arr[insert_idx], (from - insert_idx) * sizeof(int));
        arr[insert_idx] = val;
    } else {
        /* insert_idx > from: shift region [from+1..insert_idx] left by 1 */
        memmove(&arr[from], &arr[from+1], (insert_idx - from) * sizeof(int));
        arr[insert_idx] = val;
    }
    return 1;
}

/* Optimized variant: performs the same remove+insert but updates an optional
 * inverse map `pos_of_face` in a single pass to avoid a separate update loop.
 * If `pos_of_face` is NULL, this behaves like the classic version (but using
 * explicit loops instead of memmove so we can update positions inline).
 */
static int move_element_remove_and_insert_pos(int *arr, int n, int from, int insert_idx, int *pos_of_face) {
    if (from < 0 || from >= n) return 0;
    if (insert_idx < 0) insert_idx = 0;
    if (insert_idx > n-1) insert_idx = n-1;
    if (from == insert_idx) return 0;
    int val = arr[from];
    if (insert_idx < from) {
        /* shift right region [insert_idx..from-1] -> [insert_idx+1..from] */
        for (int i = from; i > insert_idx; --i) {
            arr[i] = arr[i-1];
            if (pos_of_face) pos_of_face[arr[i]] = i;
        }
        arr[insert_idx] = val;
        if (pos_of_face) pos_of_face[val] = insert_idx;
    } else {
        /* shift left region [from+1..insert_idx] -> [from..insert_idx-1] */
        for (int i = from; i < insert_idx; ++i) {
            arr[i] = arr[i+1];
            if (pos_of_face) pos_of_face[arr[i]] = i;
        }
        arr[insert_idx] = val;
        if (pos_of_face) pos_of_face[val] = insert_idx;
    }
    return 1;
}

static int painter_correct(Model3D* model, int face_count, int debug) {
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    if (face_count <= 0) return 0;

    int old_cull = cull_back_faces;
    cull_back_faces = 0;
    painter_newell_sancha_fast(model, face_count);

    int moves = 0;
    int n = face_count;

    /* Partition faces: back-faces (plane_d <= 0) go to beginning, front-faces after.
     * Maintain z_mean order within each partition using stable partitioning.
     * Both groups will be independently corrected (back-faces only if culling OFF).
     * This separation ensures plane equation tests remain valid within each group. */
    int *temp_indices = (int*)malloc(n * sizeof(int)); if (!temp_indices) { printf("Error: painter_correct malloc temp_indices failed\n"); return 0; }
    int back_idx = 0, front_idx = 0;
    /* First pass: count and separate */
    for (int i = 0; i < n; ++i) {
        int f = faces->sorted_face_indices[i];
        if (faces->plane_d[f] <= 0) {
            temp_indices[back_idx++] = f;  /* back-faces at beginning */
        }
    }
    int front_start_pos = back_idx;  /* Mark where front-faces start */
    for (int i = 0; i < n; ++i) {
        int f = faces->sorted_face_indices[i];
        if (faces->plane_d[f] > 0) {
            temp_indices[front_start_pos + front_idx++] = f;  /* front-faces after */
        }
    }
    /* Copy back to sorted_face_indices */
    for (int i = 0; i < n; ++i) faces->sorted_face_indices[i] = temp_indices[i];
    free(temp_indices);

    /* Inverse map for quick position lookup: face_id -> index in sorted_face_indices */
    int *pos_of_face = (int*)malloc(n * sizeof(int)); if (!pos_of_face) { printf("Error: painter_correct malloc pos_of_face failed\n"); return 0; }
    for (int i = 0; i < n; ++i) pos_of_face[faces->sorted_face_indices[i]] = i;
    /* Per-target minimal allowed index once moved: -1 means not moved yet. */
    int *min_allowed_pos = (int*)malloc(n * sizeof(int)); if (!min_allowed_pos) { free(pos_of_face); printf("Error: painter_correct malloc min_allowed_pos failed\n"); return 0; }
    for (int i = 0; i < n; ++i) min_allowed_pos[i] = -1;

    printf("Back faces:");

    /* Two independent passes: 
     * 1) Correct back-faces order within themselves (indices 0 to front_start_pos-1)
     * 2) Correct front-faces order within themselves (indices front_start_pos to n-1)
     * This ensures back-faces stay before front-faces while both groups benefit from corrections. */

    /* PASS 1: Correct back-faces (plane_d <= 0) - only if culling is OFF */
    if (old_cull == 0) { /* Skip this pass entirely if culling is ON (back-faces won't be displayed) */
        for (int target = 0; target < face_count; ++target) {
            if (faces->plane_d[target] > 0) continue; /* Skip front-faces in this pass */

        int pos = pos_of_face[target];
        if (pos < 0 || pos >= front_start_pos) continue; /* Must be in back-face section */

        /* 1) BEFORE test: check back-faces placed before target */
        int best_before = -1;
        for (int i = 0; i < pos; ++i) {
            int f = faces->sorted_face_indices[i];
            if (i >= front_start_pos) break; /* Don't cross into front-face section */

            /* depth (Z) quick rejection (cheap): if Z ranges do not overlap skip */
            if (faces->z_max[f] <= faces->z_min[target]) continue;
            if (faces->z_max[target] <= faces->z_min[f]) continue;

            /* bounding-box quick rejection (very cheap): if AABBs don't intersect we
             * skip the expensive overlap check entirely. Touching-only bbox cases
             * will pass here and be filtered by the overlap test (which treats
             * touching-only as NON-overlap).
             */
            if (faces->maxx[f] <= faces->minx[target]) continue;
            if (faces->maxx[target] <= faces->minx[f]) continue;
            if (faces->maxy[f] <= faces->miny[target]) continue;
            if (faces->maxy[target] <= faces->miny[f]) continue;    

            /* require actual projected polygon overlap before considering swap
             * (projected_polygons_overlap performs proper edge intersection and
             * containment tests; expensive but necessary for correctness).
             */
            if (!projected_polygons_overlap_simple(model, f, target)) continue;

            /* Use specialized plane-only 'after' test for BEFORE loop: quick check
             * to see if `f` is geometrically after (in front of) `target`.
             */
            if (pair_plane_after(model, f, target)) {
                if (best_before == -1 || i < best_before) best_before = i;
            }
        }
        if (best_before != -1) {
            /* move target to be BEFORE best_before (use optimized variant updating inverse map) */
            moves += move_element_remove_and_insert_pos(faces->sorted_face_indices, face_count, pos, best_before, pos_of_face);
            /* update pos to new location via inverse map */
            pos = pos_of_face[target];
            /* record minimal allowed index for this target (disallow later moves placing it at indices > pos) */
            if (min_allowed_pos[target] == -1 || pos < min_allowed_pos[target]) min_allowed_pos[target] = pos;
        }

        /* 2) AFTER test: check back-faces placed after target (within back-face section) */
        pos = pos_of_face[target];
        if (pos < 0 || pos >= front_start_pos) continue;
        int best_after = -1;
        for (int i = pos + 1; i < front_start_pos; ++i) { /* Stay within back-face section */
            int f = faces->sorted_face_indices[i];

            /* depth (Z) quick rejection (cheap) */
            if (faces->z_max[f] <= faces->z_min[target]) continue;
            if (faces->z_max[target] <= faces->z_min[f]) continue;

            /* bounding-box quick rejection (cheap) */
            if (faces->maxx[f] <= faces->minx[target]) continue;
            if (faces->maxx[target] <= faces->minx[f]) continue;
            if (faces->maxy[f] <= faces->miny[target]) continue;
            if (faces->maxy[target] <= faces->miny[f]) continue;

            if (!projected_polygons_overlap_simple(model, f, target)) continue;
            if (pair_plane_before(model, f, target)) { best_after = i; }
        }
        if (best_after != -1) {
            int insert_idx;
            if (best_after < pos) insert_idx = best_after + 1; else insert_idx = best_after;
            if (min_allowed_pos[target] != -1 && insert_idx > min_allowed_pos[target]) insert_idx = min_allowed_pos[target];
            /* Ensure we don't move into front-face section */
            if (insert_idx >= front_start_pos) insert_idx = front_start_pos - 1;
            moves += move_element_remove_and_insert_pos(faces->sorted_face_indices, face_count, pos, insert_idx, pos_of_face);
            pos = pos_of_face[target];
            if (min_allowed_pos[target] == -1 || pos < min_allowed_pos[target]) min_allowed_pos[target] = pos;
        }
        
        printf(" %d",target);
        }
    } /* End of PASS 1 (back-faces) */


    printf("\nFront faces:");
    /* PASS 2: Correct front-faces (plane_d > 0) */
    for (int target = 0; target < face_count; ++target) {
        if (faces->plane_d[target] <= 0) continue; /* Skip back-faces in this pass */

        int pos = pos_of_face[target];
        if (pos < 0 || pos < front_start_pos) continue; /* Must be in front-face section */

        /* 1) BEFORE test: check front-faces placed before target (don't test back-faces) */
        int best_before = -1;
        for (int i = front_start_pos; i < pos; ++i) { /* Start from front-face section */
            int f = faces->sorted_face_indices[i];

            /* depth (Z) quick rejection (cheap) */
            if (faces->z_max[f] <= faces->z_min[target]) continue;
            if (faces->z_max[target] <= faces->z_min[f]) continue;

            /* bounding-box quick rejection (cheap) */
            if (faces->maxx[f] <= faces->minx[target]) continue;
            if (faces->maxx[target] <= faces->minx[f]) continue;
            if (faces->maxy[f] <= faces->miny[target]) continue;
            if (faces->maxy[target] <= faces->miny[f]) continue;    

            if (!projected_polygons_overlap_simple(model, f, target)) continue;
            if (pair_plane_after(model, f, target)) {
                if (best_before == -1 || i < best_before) best_before = i;
            }
        }
        if (best_before != -1) {
            /* Ensure best_before is not in back-face section */
            if (best_before < front_start_pos) best_before = front_start_pos;
            moves += move_element_remove_and_insert_pos(faces->sorted_face_indices, face_count, pos, best_before, pos_of_face);
            pos = pos_of_face[target];
            if (min_allowed_pos[target] == -1 || pos < min_allowed_pos[target]) min_allowed_pos[target] = pos;
        }

        /* 2) AFTER test: check front-faces placed after target */
        pos = pos_of_face[target];
        if (pos < 0 || pos >= n) continue;
        int best_after = -1;
        for (int i = pos + 1; i < face_count; ++i) {
            int f = faces->sorted_face_indices[i];
            /* Note: loop goes to face_count which may include back-faces, but they
             * have plane_d <= 0 so they won't interfere with front-face tests */

            /* depth (Z) quick rejection (cheap) */
            if (faces->z_max[f] <= faces->z_min[target]) continue;
            if (faces->z_max[target] <= faces->z_min[f]) continue;

            /* bounding-box quick rejection (cheap) */
            // if (faces->maxx[f] <= faces->minx[target] || faces->maxx[target] <= faces->minx[f]
            //     || faces->maxy[f] <= faces->miny[target] || faces->maxy[target] <= faces->miny[f]) continue;
            if (faces->maxx[f] <= faces->minx[target]) continue;
            if (faces->maxx[target] <= faces->minx[f]) continue;
            if (faces->maxy[f] <= faces->miny[target]) continue;

            /* require actual projected polygon overlap before considering swap
             * (the overlap test is expensive; we avoid it whenever bbox rejects).
             */
            if (!projected_polygons_overlap_simple(model, f, target)) continue;
            /* Use specialized plane-only 'before' test for AFTER loop: quick check
             * to see if `f` is geometrically before `target`.
             */
            if (pair_plane_before(model, f, target)) { best_after = i; /* wants the largest index */ }

            
        }
        if (best_after != -1) {
            /* We want to move target AFTER best_after. Compute insertion index after removal: */
            int insert_idx;
            if (best_after < pos) insert_idx = best_after + 1; else insert_idx = best_after; /* as analyzed */
            /* If target was previously moved forward, clamp insertion so it cannot be placed at an index greater than its minimal reached index */
            if (min_allowed_pos[target] != -1 && insert_idx > min_allowed_pos[target]) insert_idx = min_allowed_pos[target];
            /* perform remove and insert (use optimized variant to update inverse map) */
            moves += move_element_remove_and_insert_pos(faces->sorted_face_indices, face_count, pos, insert_idx, pos_of_face);
            /* update recorded minimal position in case the move moved it even earlier */
            pos = pos_of_face[target];
            if (min_allowed_pos[target] == -1 || pos < min_allowed_pos[target]) min_allowed_pos[target] = pos;
        }
        
        printf(" %d",target);
    }

    cull_back_faces = old_cull;
    free(pos_of_face); free(min_allowed_pos);
    return moves;
}

/* painter_correctV2
 * ------------------
 * Version V2: Three-pass algorithm with partitioning and cross-section boundary test
 * 1. Partitions faces like painter_correct (back-faces / front-faces)
 * 2. PASS 1: Correct back-faces among themselves
 * 3. PASS 2: Correct front-faces among themselves  
 * 4. PASS 3 (only if culling OFF): Cross-section test between back/front boundary
 *    - For each back-face, test overlap with ALL front-faces
 *    - If overlap + back-face closer (z_mean) → move back-face after front-face
 */
static int painter_correctV2(Model3D* model, int face_count, int debug) {
    /*
     * painter_correctV2: experimental variation of painter_correct that keeps the
     * same two-pass local correction (back-faces then front-faces) but adds
     * diagnostics and limited corrective actions when culling is OFF.
     *
     * NOTE: The following comments aim to document intent and control flow only.
     *       No behavior is changed — this is strictly explanatory text.
     *
     * Summary of algorithmic steps (informative):
     *  1) Seed an initial ordering using the full Newell-Sancha sort (`painter_newell_sancha`) (more thorough than the fast variant)
     *  2) Partition faces into BACK (plane_d <= 0) then FRONT (plane_d > 0)
     *     and run local corrective passes inside each partition (Pass 1 for backs,
     *     Pass 2 for fronts). These passes perform only local swaps and rely on
     *     cheap quick-rejects (z-min/max, bbox on X/Y) before invoking
     *     expensive geometry tests (projected overlap, plane tests).
     *  3) If back-face culling was disabled: snapshot the result of passes 1+2,
     *     perform a Z-only global sort, and scan the nearest faces backward to
     *     locate BACK faces that actually appear near the front of the scene.
     *     For each such BACK face, we search for overlapping FRONT faces and
     *     attempt to resolve pair ordering using plane-based geometric tests;
     *     if geometric tests are inconclusive, z_mean is used as a deterministic
     *     fallback. Only local, minimal moves are applied to the snapshot.
     *
     * Rationale: the function is intentionally conservative — it avoids global
     *            reorders and prefers deterministic, local changes to preserve
     *            stability of the painter's output in most cases.
     */
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    if (face_count <= 0) return 0;

    /* Save and disable culling locally so cross-partition cases are considered. */
    int old_cull = cull_back_faces;
    cull_back_faces = 0;

    /* Seed initial ordering using the full Newell-Sancha sort (`painter_newell_sancha`).
     * Rationale: the full sort performs more extensive per-face computation (plane
     * conversions, bounding boxes and z_mean) which reduces inconclusive geometric
     * tests later during local corrections. This improves correctness at the cost
     * of increased CPU work compared to a lightweight 'fast' seed.
     */
    painter_newell_sancha(model, face_count);
    int moves = 0;
    int n = face_count;

    // Partition faces: back-faces (plane_d <= 0) go to beginning, front-faces after
    // This mirrors painter_correct's partition so that passes 1/2 operate within
    // homogenous groups (back-only then front-only) which simplifies plane tests.
    int *temp_indices = (int*)malloc(n * sizeof(int)); if (!temp_indices) { printf("Error: painter_correctV2 malloc temp_indices failed\n"); return 0; }
    int back_idx = 0, front_idx = 0;
    for (int i = 0; i < n; ++i) {
        int f = faces->sorted_face_indices[i];
        if (faces->plane_d[f] <= 0) {
            temp_indices[back_idx++] = f;
        }
    }
    int front_start_pos = back_idx;
    for (int i = 0; i < n; ++i) {
        int f = faces->sorted_face_indices[i];
        if (faces->plane_d[f] > 0) {
            temp_indices[front_start_pos + front_idx++] = f;
        }
    }
    for (int i = 0; i < n; ++i) faces->sorted_face_indices[i] = temp_indices[i];
    free(temp_indices);

    int *pos_of_face = (int*)malloc(n * sizeof(int)); if (!pos_of_face) { printf("Error: painter_correctV2 malloc pos_of_face failed\n"); return 0; }
    for (int i = 0; i < n; ++i) pos_of_face[faces->sorted_face_indices[i]] = i;
    int *min_allowed_pos = (int*)malloc(n * sizeof(int)); if (!min_allowed_pos) { free(pos_of_face); printf("Error: painter_correctV2 malloc min_allowed_pos failed\n"); return 0; }
    for (int i = 0; i < n; ++i) min_allowed_pos[i] = -1;

    printf("Back faces:");
    // PASS 1: Correct back-faces (plane_d <= 0) - only if culling is OFF
    // This pass attempts local moves within the back-face partition using
    // cheap quick-rejects followed by more expensive geometric tests.
    // Rationale: we iterate each BACK face and scan nearby faces inside the
    // partition. We only perform local swaps (to preserve stability) and we
    // always prefer the cheaper z/bbox tests before calling projected overlap
    // and plane-based tests which are heavier.
    if (old_cull == 0) {
        for (int target = 0; target < face_count; ++target) {
            printf(" %d",target);
            if (faces->plane_d[target] > 0) continue;
            int pos = pos_of_face[target];
            if (pos < 0 || pos >= front_start_pos) continue;
            int best_before = -1; // search for a preceding face 'f' that should be after target
            // Order of checks: cheap z/bbox quick-rejects first; only then call
            // projected_polygons_overlap() (expensive) and finally plane-based test.
            for (int i = 0; i < pos; ++i) {
                int f = faces->sorted_face_indices[i];
                if (i >= front_start_pos) break;
                if (faces->z_max[f] <= faces->z_min[target]) continue;
                if (faces->z_max[target] <= faces->z_min[f]) continue;
                if (faces->maxx[f] <= faces->minx[target]) continue;
                if (faces->maxx[target] <= faces->minx[f]) continue;
                if (faces->maxy[f] <= faces->miny[target]) continue;
                if (faces->maxy[target] <= faces->miny[f]) continue;
                // printf("here");
                if (!projected_polygons_overlap_simple(model, f, target)) continue;
                if (pair_plane_after(model, f, target)) {
                    if (best_before == -1 || i < best_before) best_before = i;
                }
            }
            if (best_before != -1) {
                moves += move_element_remove_and_insert_pos(faces->sorted_face_indices, face_count, pos, best_before, pos_of_face);
                pos = pos_of_face[target];
                if (min_allowed_pos[target] == -1 || pos < min_allowed_pos[target]) min_allowed_pos[target] = pos;
            }
            pos = pos_of_face[target];
            if (pos < 0 || pos >= front_start_pos) continue;
            int best_after = -1; // search for a succeeding face 'f' that should be before target
            // Same quick-reject ordering as above to avoid heavy geometry checks when possible.
            for (int i = pos + 1; i < front_start_pos; ++i) {
                int f = faces->sorted_face_indices[i];
                if (faces->z_max[f] <= faces->z_min[target]) continue;
                if (faces->z_max[target] <= faces->z_min[f]) continue;
                if (faces->maxx[f] <= faces->minx[target]) continue;
                if (faces->maxx[target] <= faces->minx[f]) continue;
                if (faces->maxy[f] <= faces->miny[target]) continue;
                if (faces->maxy[target] <= faces->miny[f]) continue;
                if (!projected_polygons_overlap_simple(model, f, target)) continue;
                if (pair_plane_before(model, f, target)) { best_after = i; }
            }
            if (best_after != -1) {
                int insert_idx;
                if (best_after < pos) insert_idx = best_after + 1; else insert_idx = best_after;
                if (min_allowed_pos[target] != -1 && insert_idx > min_allowed_pos[target]) insert_idx = min_allowed_pos[target];
                if (insert_idx >= front_start_pos) insert_idx = front_start_pos - 1;
                moves += move_element_remove_and_insert_pos(faces->sorted_face_indices, face_count, pos, insert_idx, pos_of_face);
                pos = pos_of_face[target];
                if (min_allowed_pos[target] == -1 || pos < min_allowed_pos[target]) min_allowed_pos[target] = pos;
            }
        }
    }

    // PASS 2: Correct front-faces (plane_d > 0)
    // Symmetric to PASS 1 but operating on front-face partition; same quick
    // rejection and plane-based logic is applied to refine local ordering.
    // Note: identical conservative checks are applied to avoid unexpected
    // global reorderings; we only accept local moves that are supported by
    // decisive geometrical or depth tests.
    printf("\n\nFront faces:");
    for (int target = 0; target < face_count; ++target) {
        printf(" %d",target);
        if (faces->plane_d[target] <= 0) continue;
        int pos = pos_of_face[target];
        if (pos < 0 || pos < front_start_pos) continue;
        int best_before = -1;
        for (int i = front_start_pos; i < pos; ++i) {
            int f = faces->sorted_face_indices[i];
            if (faces->z_max[f] <= faces->z_min[target]) continue;
            if (faces->z_max[target] <= faces->z_min[f]) continue;
            if (faces->maxx[f] <= faces->minx[target]) continue;
            if (faces->maxx[target] <= faces->minx[f]) continue;
            if (faces->maxy[f] <= faces->miny[target]) continue;
            if (faces->maxy[target] <= faces->miny[f]) continue;
            if (!projected_polygons_overlap_simple(model, f, target)) continue;
            if (pair_plane_after(model, f, target)) {
                if (best_before == -1 || i < best_before) best_before = i;
            }
        }
        if (best_before != -1) {
            if (best_before < front_start_pos) best_before = front_start_pos;
            moves += move_element_remove_and_insert_pos(faces->sorted_face_indices, face_count, pos, best_before, pos_of_face);
            pos = pos_of_face[target];
            if (min_allowed_pos[target] == -1 || pos < min_allowed_pos[target]) min_allowed_pos[target] = pos;
        }
        pos = pos_of_face[target];
        if (pos < 0 || pos < front_start_pos) continue;
        int best_after = -1;
        for (int i = pos + 1; i < face_count; ++i) {
            int f = faces->sorted_face_indices[i];
            if (faces->z_max[f] <= faces->z_min[target]) continue;
            if (faces->z_max[target] <= faces->z_min[f]) continue;
            if (faces->maxx[f] <= faces->minx[target]) continue;
            if (faces->maxx[target] <= faces->minx[f]) continue;
            if (faces->maxy[f] <= faces->miny[target]) continue;
            if (faces->maxy[target] <= faces->miny[f]) continue;
            if (!projected_polygons_overlap_simple(model, f, target)) continue;
            if (pair_plane_before(model, f, target)) { best_after = i; }
        }
        if (best_after != -1) {
            int insert_idx;
            if (best_after < pos) insert_idx = best_after + 1; else insert_idx = best_after;
            if (min_allowed_pos[target] != -1 && insert_idx > min_allowed_pos[target]) insert_idx = min_allowed_pos[target];
            moves += move_element_remove_and_insert_pos(faces->sorted_face_indices, face_count, pos, insert_idx, pos_of_face);
            pos = pos_of_face[target];
            if (min_allowed_pos[target] == -1 || pos < min_allowed_pos[target]) min_allowed_pos[target] = pos;
        }
    }

    // PASS 3: Cross-section test between back/front boundary (only if culling is OFF)

    /*
     * Rationale:
     *  - painter_correct separates backs and fronts; when culling is OFF a BACK
     *    face can legitimately be physically in front of the scene and overlap
     *    FRONT faces. Such cases require cross-partition handling.
     * Approach:
     *  - Save a snapshot of the ordering produced by passes 1+2 (so we can modify
     *    locally without losing the original two-pass result if we choose to).
     *  - Run a fast Z-only sort to get a global depth ordering, then scan from
     *    nearest faces back until a FRONT face is encountered.
     *  - For each BACK face encountered (i.e., back-face appearing near the front):
     *     * find overlapping FRONT faces (bbox quick reject + projected overlap)
     *     * try plane-based geometric tests between the BACK and each FRONT
     *       (preferential decision). If inconclusive, fallback to comparing
     *       z_mean as a deterministic tie-breaker.
     *     * upon a decisive result, perform a local move in the snapshot ordering
     *       so that the final ordering respects the concrete geometric decision.
     */
    if (old_cull == 0) {
        int *snapshot = (int*)malloc(n * sizeof(int));
        if (!snapshot) { printf("Error: painter_correctV2 malloc snapshot failed\n"); return 0; }
        for (int i = 0; i < n; ++i) snapshot[i] = faces->sorted_face_indices[i];

        /* Z-only sort (fast) */
        painter_newell_sancha(model, face_count);

        printf("\n\nCross-section back-face checks:");
        /* FILE *logf = fopen("incfaces.log", "a"); */
        FILE *logf = NULL; /* logging disabled */
        int total_scanned = 0, total_logged = 0, total_ov = 0, total_tch = 0, total_none = 0, total_moves = 0;
        /* Scan from highest index downwards */
        for (int idx = n - 1; idx >= 0; --idx) {
            int bf = faces->sorted_face_indices[idx];
            total_scanned++;
            /* If we hit a front-face, stop and restore original order */
            if (faces->plane_d[bf] > 0) {
                break;
            }

            // /* It's abreak-face: log detailed info */
            // printf(" %d",idx);
            // // XXXX
            // printf("\nface=%d", bf);

            total_logged++;
            int orig_pos = pos_of_face[bf];
            /*(logf) fprintf(logf, "BACK idx=%d pos=%d face=%d zmean=%.6f ", idx, orig_pos, bf, FIXED_TO_FLOAT(faces->z_mean[bf])); */
            /* Find front faces that overlap or touch this back-face */
            int found = 0;
            int per_ov = 0, per_tch = 0;
            for (int j = 0; j < n; ++j) {
                int ff = faces->sorted_face_indices[j];
                // printf("\n  => testing ff=%d", ff);

                if (faces->plane_d[ff] <= 0) continue; /* only front faces */
                /* Z quick reject */
                if (faces->z_max[ff] <= faces->z_min[bf]) continue;
                if (faces->z_max[bf] <= faces->z_min[ff]) continue;
                /* bbox tests (strict and non-strict to detect touching-only) */
                int bbox_strict = !(faces->maxx[ff] <= faces->minx[bf] || faces->maxx[bf] <= faces->minx[ff]
                                    || faces->maxy[ff] <= faces->miny[bf] || faces->maxy[bf] <= faces->miny[ff]);
                int bbox_touch = !(faces->maxx[ff] < faces->minx[bf] || faces->maxx[bf] < faces->minx[ff]
                                   || faces->maxy[ff] < faces->miny[bf] || faces->maxy[bf] < faces->miny[ff]);
                if (!bbox_touch) continue; /* no even-touch bbox => skip */
                int overlapping = 0;
                if (bbox_strict) {
                    if (projected_polygons_overlap_simple(model, ff, bf)) overlapping = 1;
                }
                if (overlapping || bbox_touch) {
                    int orig_pos_ff = pos_of_face[ff];
                    /* if (logf) fprintf(logf, "FF idx=%d pos=%d face=%d zmean=%.6f%s ", j, orig_pos_ff, ff, FIXED_TO_FLOAT(faces->z_mean[ff]), overlapping?"(ov)":"(tch)"); */
                    found++; if (overlapping) { per_ov++; total_ov++; } else { per_tch++; total_tch++; }

                    /* Decide ordering: try plane tests (bf vs ff) first. Policy:
                     * - If plane_after indicates bf is decisively after ff, prefer that
                     *   and move bf after ff.
                     * - If plane_before indicates bf is decisively before ff, prefer
                     *   that and keep bf before ff.
                     * - Only when plane tests are inconclusive or conflicting, we
                     *   use z_mean as a deterministic fallback. This keeps changes
                     *   local and reproducible across runs.
                     */
                    int plane_after = pair_plane_after(model, bf, ff); /* 1 if bf after ff */
                    int plane_before = pair_plane_before(model, bf, ff); /* 1 if bf before ff */

                    int decision = 0; /* 1 => move bf after ff, -1 => keep bf before ff */
                    const char *reason = NULL;
                    if (plane_after == 1 && plane_before == 0) { decision = 1; reason = "plane_after"; }
                    else if (plane_before == 1 && plane_after == 0) { decision = -1; reason = "plane_before"; }
                    else {
                        /* inconclusive: fallback to zmean */
                        float zbf = FIXED_TO_FLOAT(faces->z_mean[bf]);
                        float zff = FIXED_TO_FLOAT(faces->z_mean[ff]);
                        if (zbf < zff) { decision = 1; reason = "zmean"; } else { decision = -1; reason = "zmean"; }
                    }
                    // printf("\ndecision=%d (%s)", decision, reason);
                    // keypress();
                    // printf("\n");
                    // XXXX

                    /* Apply decision: if move needed, update snapshot and pos_of_face */
                    // Only move bf after ff if bf is currently before ff in the sorted order
                    if (decision == 1 && pos_of_face[bf] < pos_of_face[ff]) {
                        int from = pos_of_face[bf];
                        int to = pos_of_face[ff] + 1; /* insert after ff */
                        if (from < to) to--; /* account for removal shifting */
                        if (from != to) {
                            int tmp = snapshot[from];
                            if (from < to) {
                                for (int s = from; s < to; ++s) { snapshot[s] = snapshot[s+1]; pos_of_face[snapshot[s]] = s; }
                                snapshot[to] = tmp; pos_of_face[tmp] = to;
                            } else {
                                for (int s = from; s > to; --s) { snapshot[s] = snapshot[s-1]; pos_of_face[snapshot[s]] = s; }
                                snapshot[to] = tmp; pos_of_face[tmp] = to;
                            }
                            /* Log move */
                            /* if (logf) fprintf(logf, "  ACTION move bf->after ff (reason=%s) from=%d to=%d\n", reason, from, to); */
                            total_moves++;
                        } else {
                            /* if (logf) fprintf(logf, "  ACTION decision=%s but already at desired pos\n", reason); */
                        }
                        // Once a swap has been made (backface moved after a frontface),
                        // this backface should not be tested with other frontfaces.
                        // This prevents multiple swaps and ensures local ordering consistency.
                        //break; ==> NO : we nned to check all front faces to sort out the correct order
                    }
                }
            }
            if (!found) { /* if (logf) fprintf(logf, "none"); */ total_none++; }
            /* if (logf) fprintf(logf, " -- overlaps=%d touch=%d\n", per_ov, per_tch); */
        }
        /* if (logf) {
            fprintf(logf, "SUMMARY scanned=%d logged=%d total_ov=%d total_tch=%d total_none=%d total_moves=%d\n", total_scanned, total_logged, total_ov, total_tch, total_none, total_moves);
            fclose(logf);
        } */

        /* Restore original order from passes 1/2 */
        for (int i = 0; i < n; ++i) faces->sorted_face_indices[i] = snapshot[i];
        free(snapshot);
    }

    cull_back_faces = old_cull;
    free(pos_of_face);
    free(min_allowed_pos);
    // Affichage identique à painter_correct
    // printf("Faces:");
    // for (int i = 0; i < n; ++i) printf(" %d", faces->sorted_face_indices[i]);
    // printf("\n");
    return moves;
}





/* painter_newell_sanchaV3
 * -------------------------
 * Variant of V2 that, when culling is OFF, places back-faces at the beginning
 * of the sorted list and performs the local reordering pass only over the
 * FRONT partition (i.e., the double-loop iterates only over front faces).
 * This ensures back faces are drawn first while preserving conservative local
 * reordering behavior for front faces.
 */
void painter_newell_sanchaV3(Model3D* model, int face_count) {
    if (!model) return;
    FaceArrays3D* faces = &model->faces;
    if (face_count <= 0) return;

    int i;
    int n = face_count;
    int *pair_done = NULL;
    if (n > 0) {
        pair_done = (int*)malloc(n * n * sizeof(int));
        if (pair_done) {
            for (i = 0; i < n * n; ++i) pair_done[i] = 0;
        }
    }

    /* Seed with fast stable z-mean sort */
    painter_newell_sancha_fast(model, face_count);

    /* If culling is OFF, stable-partition sorted list so that back-faces (plane_d <= 0)
     * appear first. We'll then only process the front-face partition that follows. */
    int back_count = 0;
    if (!cull_back_faces) {
        int *back_list = (int*)malloc(sizeof(int) * n);
        int *front_list = (int*)malloc(sizeof(int) * n);
        if (!back_list || !front_list) { if (back_list) free(back_list); if (front_list) free(front_list); if (pair_done) free(pair_done); return; }
        int b = 0, f = 0;
        for (i = 0; i < n; ++i) {
            int fid = faces->sorted_face_indices[i];
            if (faces->plane_d[fid] <= 0) back_list[b++] = fid; else front_list[f++] = fid;
        }
        for (i = 0; i < b; ++i) faces->sorted_face_indices[i] = back_list[i];
        for (i = 0; i < f; ++i) faces->sorted_face_indices[b + i] = front_list[i];
        back_count = b;
        free(back_list); free(front_list);
    }

    /* Prepare helper arrays (position map and min-allowed positions) */
    int *pos_of_face = (int*)malloc(n * sizeof(int)); if (!pos_of_face) { if (pair_done) free(pair_done); return; }
    for (i = 0; i < n; ++i) pos_of_face[faces->sorted_face_indices[i]] = i;
    int *min_allowed_pos = (int*)malloc(n * sizeof(int)); if (!min_allowed_pos) { free(pos_of_face); if (pair_done) free(pair_done); return; }
    for (i = 0; i < n; ++i) min_allowed_pos[i] = -1;

    int *failed_pairs = (int*)malloc(2 * n * n * sizeof(int)); int failed_pairs_count = 0;

    /* Only process front faces: start index depends on whether culling is ON/OFF */
    int process_start = (!cull_back_faces) ? back_count : 0;

    for (int t_idx = process_start; t_idx < n; ++t_idx) {
        int target = faces->sorted_face_indices[t_idx];
        if (cull_back_faces && faces->plane_d[target] <= 0) continue; /* safety */
        int pos = pos_of_face[target];
        int best_move = -1, move_before = 0;
        for (int iidx = process_start; iidx < n; ++iidx) {
            if (iidx == pos) continue;
            int f = faces->sorted_face_indices[iidx];
            if (cull_back_faces && faces->plane_d[f] <= 0) continue; /* safety */
            int idx1 = target * n + f;
            int idx2 = f * n + target;
            if (pair_done && (pair_done[idx1] || pair_done[idx2])) continue;
            if (faces->z_max[f] <= faces->z_min[target]) { if (pair_done) pair_done[idx1] = pair_done[idx2] = 1; continue; }
            if (faces->z_max[target] <= faces->z_min[f]) { if (pair_done) pair_done[idx1] = pair_done[idx2] = 1; continue; }
            if (faces->maxx[f] <= faces->minx[target]) { if (pair_done) pair_done[idx1] = pair_done[idx2] = 1; continue; }
            if (faces->maxx[target] <= faces->minx[f]) { if (pair_done) pair_done[idx1] = pair_done[idx2] = 1; continue; }
            if (faces->maxy[f] <= faces->miny[target]) { if (pair_done) pair_done[idx1] = pair_done[idx2] = 1; continue; }
            if (faces->maxy[target] <= faces->miny[f]) { if (pair_done) pair_done[idx1] = pair_done[idx2] = 1; continue; }
            if (!projected_polygons_overlap(model, f, target)) { if (pair_done) pair_done[idx1] = pair_done[idx2] = 1; continue; }
            if (iidx < pos) {
                if (pair_plane_after(model, f, target)) { best_move = iidx; move_before = 1; if (pair_done) pair_done[idx1] = pair_done[idx2] = 1; break; }
            } else {
                if (pair_plane_before(model, f, target)) { best_move = iidx; move_before = 0; if (pair_done) pair_done[idx1] = pair_done[idx2] = 1; break; }
            }
            if (pair_done) pair_done[idx1] = pair_done[idx2] = 1;
        }
        if (best_move != -1) {
            int insert_idx;
            if (move_before) {
                insert_idx = best_move; if (insert_idx < process_start) insert_idx = process_start;
            } else {
                insert_idx = best_move + 1; if (insert_idx < process_start) insert_idx = process_start;
            }
            if (min_allowed_pos[target] != -1 && insert_idx > min_allowed_pos[target]) insert_idx = min_allowed_pos[target];
            if (insert_idx >= n) insert_idx = n - 1;
            move_element_remove_and_insert_pos(faces->sorted_face_indices, n, pos, insert_idx, pos_of_face);
            pos = pos_of_face[target];
            if (min_allowed_pos[target] == -1 || pos < min_allowed_pos[target]) min_allowed_pos[target] = pos;
        }
    }

    free(pos_of_face); free(min_allowed_pos);
    if (pair_done) free(pair_done);
    if (failed_pairs) free(failed_pairs);
}

/* painter_newell_sanchaV2
 * -------------------------
 * PASS-2 inspired conservative local reordering for FRONT faces.
 * - Seed with `painter_newell_sancha` to compute planes/zmeans
 * - Partition faces into BACK/FRONT and operate only on FRONT partition
 * - For each front face, scan faces before and after and apply cheap tests
 *   (z, bbox), projected overlap, then plane tests. On decisive decision,
 *   insert the target before/after the chosen face (no blind swaps).
 * - Limits applied to moves to keep behavior stable and interactive.
 */
void painter_newell_sanchaV2(Model3D* model, int face_count) {
    if (!model) return;
    FaceArrays3D* faces = &model->faces;
    if (face_count <= 0) return;

    int i, j, vi, a, b;
    int *indices_i;
    int *indices_j;
    int match;
    int found;

    // Allocate pair_done array to mark processed face pairs
    int n = face_count;
    int *pair_done = NULL;
    if (n > 0) {
        pair_done = (int*)malloc(n * n * sizeof(int));
        if (pair_done) {
            for (i = 0; i < n * n; ++i) pair_done[i] = 0;
        }
    }

    /* Nouvelle version sans partition back/front, double boucle sur toutes les faces selon culling */
    painter_newell_sancha_fast(model, face_count);
    int moves = 0;

    int *pos_of_face = (int*)malloc(n * sizeof(int)); 
    if (!pos_of_face) { if (pair_done) free(pair_done); return; }

    for (i = 0; i < n; ++i) pos_of_face[faces->sorted_face_indices[i]] = i;
    int *min_allowed_pos = (int*)malloc(n * sizeof(int)); 
    if (!min_allowed_pos) { free(pos_of_face); if (pair_done) free(pair_done); return; }
    for (i = 0; i < n; ++i) min_allowed_pos[i] = -1;

    int failed_tests_count = 0;
    // Buffer to store failed pairs (target, f)
    int max_failed_pairs = n * n;
    int *failed_pairs = (int*)malloc(2 * max_failed_pairs * sizeof(int));
    int failed_pairs_count = 0;
    for (int target = 0; target < n; ++target) {
        if (cull_back_faces && faces->plane_d[target] <= 0) continue;
        int pos = pos_of_face[target];
        int best_move = -1;
        int move_before = 0;
        for (int i = 0; i < n; ++i) {
            if (i == pos) continue;
            int f = faces->sorted_face_indices[i];
            if (cull_back_faces && faces->plane_d[f] <= 0) continue;
            int idx1 = target * n + f;
            int idx2 = f * n + target;
            if (pair_done && (pair_done[idx1] || pair_done[idx2])) continue;
            if (faces->z_max[f] <= faces->z_min[target]) { if (pair_done) pair_done[idx1] = pair_done[idx2] = 1; continue; }
            if (faces->z_max[target] <= faces->z_min[f]) { if (pair_done) pair_done[idx1] = pair_done[idx2] = 1; continue; }
            if (faces->maxx[f] <= faces->minx[target]) { if (pair_done) pair_done[idx1] = pair_done[idx2] = 1; continue; }
            if (faces->maxx[target] <= faces->minx[f]) { if (pair_done) pair_done[idx1] = pair_done[idx2] = 1; continue; }
            if (faces->maxy[f] <= faces->miny[target]) { if (pair_done) pair_done[idx1] = pair_done[idx2] = 1; continue; }
            if (faces->maxy[target] <= faces->miny[f]) { if (pair_done) pair_done[idx1] = pair_done[idx2] = 1; continue; }
            if (!projected_polygons_overlap(model, f, target)) { if (pair_done) pair_done[idx1] = pair_done[idx2] = 1; continue; }
            if (i < pos) {
                if (pair_plane_after(model, f, target)) { best_move = i; move_before = 1; if (pair_done) pair_done[idx1] = pair_done[idx2] = 1; break; }
            } else {
                if (pair_plane_before(model, f, target)) { best_move = i; move_before = 0; if (pair_done) pair_done[idx1] = pair_done[idx2] = 1; break; }
            }

            if (pair_done) pair_done[idx1] = pair_done[idx2] = 1;
        }
        if (best_move != -1) {
            if (move_before) {
                moves += move_element_remove_and_insert_pos(faces->sorted_face_indices, n, pos, best_move, pos_of_face);
                pos = pos_of_face[target];
                if (min_allowed_pos[target] == -1 || pos < min_allowed_pos[target]) min_allowed_pos[target] = pos;
            } else {
                int insert_idx = (best_move < pos) ? (best_move + 1) : best_move;
                if (min_allowed_pos[target] != -1 && insert_idx > min_allowed_pos[target]) insert_idx = min_allowed_pos[target];
                if (insert_idx >= n) insert_idx = n - 1;
                moves += move_element_remove_and_insert_pos(faces->sorted_face_indices, n, pos, insert_idx, pos_of_face);
                pos = pos_of_face[target];
                if (min_allowed_pos[target] == -1 || pos < min_allowed_pos[target]) min_allowed_pos[target] = pos;
            }
        }
    }
    // debug output of failed tests
    // {
    // printf("Number of cases where all tests failed: %d\n", failed_tests_count);
    // if (failed_pairs_count > 0) {
    //     printf("Pairs where all tests failed:\n");
    //     for (int k = 0; k < failed_pairs_count; ++k) {
    //         if (k % 3 == 0) printf("  ");
    //         printf("(face %d, face %d)", failed_pairs[2 * k], failed_pairs[2 * k + 1]);
    //         if ((k % 3 == 2) || (k == failed_pairs_count - 1)) {
    //             printf("\n");
    //         } else {
    //             printf(" ; ");
    //         }
    //     }
    //     fflush(stdout);
    // }
    // keypress();
    // }


    free(pos_of_face); free(min_allowed_pos);
    if (pair_done) free(pair_done);
    if (failed_pairs) free(failed_pairs);
}


/* Helper: pairwise ordering decision using tests 1..7 from painter_newell_sancha
 * Returns:
 *  -1 if f1 is (conclusively) before f2
 *   1 if f1 is (conclusively) after f2
 *   0 if inconclusive
 */

/* pair_order_relation
 * -------------------
 * Purpose:
 *  - Apply the same per-pair tests used by the painter (tests 1..7) to decide
 *    whether two faces are conclusively ordered.
 * Behavior / Return values:
 *  - returns -1 if f1 is conclusively BEFORE f2
 *  - returns  1 if f1 is conclusively AFTER f2
 *  - returns  0 if inconclusive
 * Notes:
 *  - Uses depth (z_min/z_max), axis-aligned bbox separation (X/Y) and plane-based
 *    vertex-side tests (observer-side checks) mirroring painter logic.
 */
segment "code01";
static int pair_order_relation(Model3D* model, int f1, int f2) {
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    Fixed32 epsilon = FLOAT_TO_FIXED(0.01f);

    // Test 1: depth separation
    if (faces->z_max[f2] <= faces->z_min[f1]) return -1; // f1 before f2
    if (faces->z_max[f1] <= faces->z_min[f2]) return 1;  // f1 after f2

    // Test 2/3: bbox separation (symmetric)
    int minx1 = faces->minx[f1], maxx1 = faces->maxx[f1], miny1 = faces->miny[f1], maxy1 = faces->maxy[f1];
    int minx2 = faces->minx[f2], maxx2 = faces->maxx[f2], miny2 = faces->miny[f2], maxy2 = faces->maxy[f2];
    if (maxx1 <= minx2) return -1; // f1 left of f2
    if (maxx2 <= minx1) return 1;  // f1 right of f2
    if (maxy1 <= miny2) return -1; // f1 below f2
    if (maxy2 <= miny1) return 1;  // f1 above f2

    // Plane-based tests 4..7 (copying logic from painter)
    int n1 = faces->vertex_count[f1];
    int n2 = faces->vertex_count[f2];
    int offset1 = faces->vertex_indices_ptr[f1];
    int offset2 = faces->vertex_indices_ptr[f2];
    int k;
    Fixed64 a1 = faces->plane_a[f1]; Fixed64 b1 = faces->plane_b[f1]; Fixed64 c1 = faces->plane_c[f1]; Fixed64 d1 = faces->plane_d[f1];
    Fixed64 a2 = faces->plane_a[f2]; Fixed64 b2 = faces->plane_b[f2]; Fixed64 c2 = faces->plane_c[f2]; Fixed64 d2 = faces->plane_d[f2];

    int obs_side1 = 0; int obs_side2 = 0; int side; int all_same_side; int all_opposite_side;

    // Test 4
    obs_side1 = 0; if (d1 > (Fixed64)epsilon) obs_side1 = 1; else if (d1 < -(Fixed64)epsilon) obs_side1 = -1; else goto skipT4;
    all_same_side = 1;
    for (k=0; k<n2; k++) {
        int v = faces->vertex_indices_buffer[offset2+k]-1;
        Fixed64 acc = 0;
        acc  = (((Fixed64)a1 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
        acc += (((Fixed64)b1 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
        acc += (((Fixed64)c1 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
        acc += (Fixed64)d1;
        if  (acc > (Fixed64)epsilon) side = 1; else if (acc < -(Fixed64)epsilon) side = -1; else continue;
        if (obs_side1 != side) { all_same_side = 0; break; }
    }
    if (all_same_side) return -1;
    skipT4: ;

    // Test 5
    obs_side2 = 0; if (d2 > (Fixed64)epsilon) obs_side2 = 1; else if (d2 < -(Fixed64)epsilon) obs_side2 = -1; else goto skipT5;
    all_opposite_side = 1;
    for (k=0; k<n1; k++) {
        int v = faces->vertex_indices_buffer[offset1+k]-1;
        Fixed64 acc = 0;
        acc  = (((Fixed64)a2 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
        acc += (((Fixed64)b2 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
        acc += (((Fixed64)c2 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
        acc += (Fixed64)d2;
        if  (acc > (Fixed64)epsilon) side = 1; else if (acc < -(Fixed64)epsilon) side = -1; else continue;
        if (obs_side2 == side) { all_opposite_side = 0; break; }
    }
    if (all_opposite_side) return -1;
    skipT5: ;

    // Test 6
    obs_side1 = 0; if (d1 > (Fixed64)epsilon) obs_side1 = 1; else if (d1 < -(Fixed64)epsilon) obs_side1 = -1; else goto skipT6;
    all_opposite_side = 1;
    for (k=0; k<n2; k++) {
        int v = faces->vertex_indices_buffer[offset2+k]-1;
        Fixed64 acc = 0;
        acc  = (((Fixed64)a1 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
        acc += (((Fixed64)b1 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
        acc += (((Fixed64)c1 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
        acc += (Fixed64)d1;
        if (acc > (Fixed64)epsilon) side = 1; else if (acc < -(Fixed64)epsilon) side = -1; else continue;
        if (obs_side1 == side) { all_opposite_side = 0; break; }
    }
    if (all_opposite_side) return 1; // swap -> f1 should be after f2
    skipT6: ;

    // Test 7
    obs_side2 = 0; if (d2 > (Fixed64)epsilon) obs_side2 = 1; else if (d2 < -(Fixed64)epsilon) obs_side2 = -1; else goto skipT7;
    all_same_side = 1;
    for (k=0; k<n1; k++) {
        int v = faces->vertex_indices_buffer[offset1+k]-1;
        Fixed64 acc = 0;
        acc  = (((Fixed64)a2 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
        acc += (((Fixed64)b2 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
        acc += (((Fixed64)c2 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
        acc += (Fixed64)d2;
        if (acc > (Fixed64)epsilon) side = 1; else if (acc < -(Fixed64)epsilon) side = -1; else continue;
        if (obs_side2 != side) { all_same_side = 0; break; }
    }
    if (all_same_side) return 1;
    skipT7: ;

    // Non-conclusive
    return 0;
}

/* check_sort
 * ----------
 * For every unordered pair of faces, if their 2D bounding boxes overlap,
 * call `ray_cast(model, f1, f2)` and check that the current ordering in
 * `faces->sorted_face_indices` matches the ray_cast result.
 * Prints mismatches and returns the number of mismatches detected.
 */
int check_sort(Model3D* model, int face_count) {
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    if (face_count <= 0) return 0;

    int n = face_count;
    int *pos_of_face = (int*)malloc(sizeof(int) * n);
    if (!pos_of_face) return 0;
    for (int i = 0; i < n; ++i) pos_of_face[faces->sorted_face_indices[i]] = i;

    int checked = 0;
    int mismatches = 0;
    int undetermined = 0;
    int skipped_by_cull = 0;

    for (int f1 = 0; f1 < n; ++f1) {
        if (faces->display_flag[f1] == 0) { ++skipped_by_cull; continue; }
        if (cull_back_faces && faces->plane_d[f1] <= 0) { ++skipped_by_cull; continue; }
        for (int f2 = f1 + 1; f2 < n; ++f2) {
            if (faces->display_flag[f2] == 0) { ++skipped_by_cull; continue; }
            if (cull_back_faces && faces->plane_d[f2] <= 0) { ++skipped_by_cull; continue; }

            int minx1 = faces->minx[f1], maxx1 = faces->maxx[f1], miny1 = faces->miny[f1], maxy1 = faces->maxy[f1];
            int minx2 = faces->minx[f2], maxx2 = faces->maxx[f2], miny2 = faces->miny[f2], maxy2 = faces->maxy[f2];
            // Quick reject: no bbox overlap
            if (maxx1 <= minx2 || maxx2 <= minx1 || maxy1 <= miny2 || maxy2 <= miny1) continue;
            
            // Use full projected polygon overlap test (touching-only treated as non-overlap)
            if (!projected_polygons_overlap(model, f1, f2)) continue;

            ++checked;
            int pos1 = pos_of_face[f1];
            int pos2 = pos_of_face[f2];
            int rc = ray_cast(model, f1, f2);
            if (rc == 0) { ++undetermined; continue; }
            // rc == -1 -> f1 is closer than f2 -> f1 should be after f2 (pos1 > pos2)
            if (rc == -1) {
                if (!(pos1 > pos2)) {
                    printf("check_sort: MISMATCH (ray says %d closer than %d): positions %d vs %d\n", f1, f2, pos1, pos2);
                    ++mismatches;
                }
            } else if (rc == 1) {
                // rc == 1 -> f1 is farther than f2 -> f1 should be before f2 (pos1 < pos2)
                if (!(pos1 < pos2)) {
                    printf("check_sort: MISMATCH (ray says %d farther than %d): positions %d vs %d\n", f1, f2, pos1, pos2);
                    ++mismatches;
                }
            }
        }
    }

    free(pos_of_face);
    printf("check_sort: checked=%d mismatches=%d undetermined=%d skipped_by_cull=%d\n", checked, mismatches, undetermined, skipped_by_cull);
    return mismatches;
}

/* check_sort_repair
 * -----------------
 * Minimal deterministic repair of face ordering. For each pair that
 * `check_sort` would report as a mismatch, move the offending face by
 * a single minimal step so the ordering matches `ray_cast`:
 *  - If ray_cast == -1 (f1 closer than f2), ensure f1 is immediately AFTER f2.
 *  - If ray_cast ==  1 (f1 farther  than f2), ensure f1 is immediately BEFORE f2.
 * Returns the number of repairs performed.
 */
int check_sort_repair(Model3D* model, int face_count) {
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    if (face_count <= 0) return 0;

    int n = face_count;
    int *pos_of_face = (int*)malloc(sizeof(int) * n);
    if (!pos_of_face) return 0;
    for (int i = 0; i < n; ++i) pos_of_face[faces->sorted_face_indices[i]] = i;

    int repairs = 0;

    for (int f1 = 0; f1 < n; ++f1) {
        if (faces->display_flag[f1] == 0) continue;
        if (cull_back_faces && faces->plane_d[f1] <= 0) continue;
        for (int f2 = f1 + 1; f2 < n; ++f2) {
            if (faces->display_flag[f2] == 0) continue;
            if (cull_back_faces && faces->plane_d[f2] <= 0) continue;

            int minx1 = faces->minx[f1], maxx1 = faces->maxx[f1], miny1 = faces->miny[f1], maxy1 = faces->maxy[f1];
            int minx2 = faces->minx[f2], maxx2 = faces->maxx[f2], miny2 = faces->miny[f2], maxy2 = faces->maxy[f2];
            // Quick reject: no bbox overlap
            if (maxx1 <= minx2 || maxx2 <= minx1 || maxy1 <= miny2 || maxy2 <= miny1) continue;

            // If projected polygons do not overlap (touching considered non-overlap), skip
            if (!projected_polygons_overlap(model, f1, f2)) continue;

            int pos1 = pos_of_face[f1];
            int pos2 = pos_of_face[f2];
            /* Try both intersection centroids (both orders) and prefer the one
             * with the larger positive clipped area. If the selected point yields
             * an indeterminate ray_cast (rc==0), try the other centroid, then
             * finally fall back to bbox-center as last resort. This mirrors the
             * inspector's behaviour but is slightly more tolerant (tries multiple
             * candidates) to avoid missing detectable inversions.
             */
            long long a1 = 0, a2 = 0;
            int cx1 = 0, cy1 = 0, cx2 = 0, cy2 = 0;
            int have1 = compute_intersection_centroid_ordered_fixed(model, f1, f2, &cx1, &cy1, &a1) && a1 > 0;
            int have2 = compute_intersection_centroid_ordered_fixed(model, f2, f1, &cx2, &cy2, &a2) && a2 > 0;

            int cx = 0, cy = 0;
            int point_source = 0; /* 1=cent1, 2=cent2, 3=bbox */
            int rc = 0;

            /* Prefer the larger centroid area when available */
            if (have1 || have2) {
                if (have1 && (!have2 || a1 >= a2)) { cx = cx1; cy = cy1; point_source = 1; }
                else { cx = cx2; cy = cy2; point_source = 2; }
                {
                    float _tf1 = 0.0f, _tf2 = 0.0f;
                    if (ray_cast_distances(model, f1, f2, cx, cy, &_tf1, &_tf2)) {
                        if (_tf1 < _tf2) rc = -1; else if (_tf1 > _tf2) rc = 1; else rc = 0;
                    } else rc = 0; /* indeterminate — ray_cast_at would also be indeterminate */
                    
                }
                /* If indeterminate, try the other centroid if present */
                if (rc == 0 && have1 && have2 && point_source == 1) {
                    cx = cx2; cy = cy2; point_source = 2;
                    {
                        float _tf1 = 0.0f, _tf2 = 0.0f;
                        if (ray_cast_distances(model, f1, f2, cx, cy, &_tf1, &_tf2)) {
                            if (_tf1 < _tf2) rc = -1; else if (_tf1 > _tf2) rc = 1; else rc = 0;
                        } else rc = 0;
                    }
                } else if (rc == 0 && have1 && have2 && point_source == 2) {
                    cx = cx1; cy = cy1; point_source = 1;
                    {
                        float _tf1 = 0.0f, _tf2 = 0.0f;
                        if (ray_cast_distances(model, f1, f2, cx, cy, &_tf1, &_tf2)) {
                            if (_tf1 < _tf2) rc = -1; else if (_tf1 > _tf2) rc = 1; else rc = 0;
                        } else rc = 0;
                    }
                }
            }

            /* Last resort: bbox center */
            if (rc == 0) {
                int ix0 = (minx1 > minx2) ? minx1 : minx2;
                int ix1 = (maxx1 < maxx2) ? maxx1 : maxx2;
                int iy0 = (miny1 > miny2) ? miny1 : miny2;
                int iy1 = (maxy1 < maxy2) ? maxy1 : maxy2;
                cx = (ix0 + ix1) / 2; cy = (iy0 + iy1) / 2; point_source = 3;
                {
                    float _tf1 = 0.0f, _tf2 = 0.0f;
                    if (ray_cast_distances(model, f1, f2, cx, cy, &_tf1, &_tf2)) {
                        if (_tf1 < _tf2) rc = -1; else if (_tf1 > _tf2) rc = 1; else rc = 0;
                    } else rc = 0;
                    
                }
            }

            if (debug_overlap_subj == f1 && debug_overlap_clip == f2) {
                if (point_source == 1) printf("check_sort_repair: using centroid f1->f2 (%d,%d) area1=%lld\n", cx, cy, a1);
                else if (point_source == 2) printf("check_sort_repair: using centroid f2->f1 (%d,%d) area2=%lld\n", cx, cy, a2);
                else printf("check_sort_repair: using bbox-center (%d,%d)\n", cx, cy);
            }
            if (rc == 0) continue; // undetermined

            if (rc == -1 || rc == 1) {
                /* Single-direction strategy: always move the *closer* face forward
                 * (towards higher indices) to be immediately AFTER the other face.
                 * This avoids oscillations caused by moving faces backward.
                 */
                int closer = (rc == -1) ? f1 : f2;
                int other  = (rc == -1) ? f2 : f1;
                int pclos = pos_of_face[closer];
                int pother = pos_of_face[other];
                if (pclos <= pother) {
                    /* Move `closer` to position `pother` (immediately after `other`). */
                    int tmp = faces->sorted_face_indices[pclos];
                    if (pclos < pother) {
                        memmove(&faces->sorted_face_indices[pclos], &faces->sorted_face_indices[pclos+1], sizeof(int) * (pother - pclos));
                        faces->sorted_face_indices[pother] = tmp;
                        for (int k = pclos; k <= pother; ++k) pos_of_face[faces->sorted_face_indices[k]] = k;
                        ++repairs;
                        printf("check_sort_repair: moved face %d FORWARD AFTER %d (pos %d -> %d)\n", closer, other, pclos, pother);
                    } else {
                        /* pclos == pother should not happen for distinct faces; ignore. */
                    }
                } else {
                    /* Already after `other` (pclos > pother): no change needed. */
                }
            }
        }
    }

    free(pos_of_face);
    printf("check_sort_repair: repairs=%d\n", repairs);
    return repairs;
}

segment "code02";
/* projected_polygons_overlap
 * --------------------------
 * Purpose:
 *  - Given a model and two face indices, determine whether their projected
 *    2D polygons overlap on screen.
 * Important behavior:
 *  - **Touching** cases (shared edge or single-vertex contact) are treated as
 *    **NON-overlap** (the function returns 0). This mirrors the graphical
 *    intent: touching does not mean the polygons have positive-area intersection.
 * Algorithm:
 *  1) Quick reject using screen-space axis-aligned bbox (min/max X/Y).
 *  2) If bboxes overlap, test whether any edge of poly1 intersects any edge of poly2
 *     (proper segment intersection only, colinear/on-segment is ignored here).
 *  3) If no edge intersects, test containment: whether any vertex of poly1 lies inside poly2
 *     or vice versa (ray-casting point-in-polygon). Points falling exactly on an edge
 *     are treated as outside (not contained).
 * Returns:
 *  - 1 if polygons overlap (proper intersection or one contains the other)
 *  - 0 if disjoint or touching-only
 */
static long long orient_ll(long long ax,long long ay,long long bx,long long by,long long cx,long long cy) {
    return (bx-ax)*(cy-ay) - (by-ay)*(cx-ax);
}
static int on_seg_ll(long long ax,long long ay,long long bx,long long by,long long cx,long long cy) {
    if (( (ax<=cx && cx<=bx) || (bx<=cx && cx<=ax) ) && ((ay<=cy && cy<=by) || (by<=cy && cy<=ay))) return 1;
    return 0;
}
/* tolerance (in pixels) below which an intersection is considered too close
 * to a vertex/edge and thus ignored. Default set by user request to 1.0 px. */
static double segs_intersect_tol_px = 1.0;

/*
 * point_seg_dist2
 * ----------------
 * Calcule la distance au carré entre un point P(px,py) et un segment AB (ax,ay)-(bx,by).
 *
 * Raison d'utiliser la distance au carré : évite l'appel coûteux à sqrt() pour des
 * comparaisons (nous comparons toujours à tol^2). Cela améliore la performance et
 * évite des erreurs d'arrondi supplémentaires.
 *
 * Méthode : projection orthogonale du point sur la droite AB.
 * - On calcule le paramètre de projection c = (AB · AP).
 * - Si c <= 0, le plus proche est A (distance P-A).
 * - Si c >= |AB|^2, le plus proche est B (distance P-B).
 * - Sinon, on projette P sur AB et on renvoie la distance P - projection.
 *
 * Propriétés et invariants :
 * - Retourne une valeur >= 0 (double) correspondant à (distance)^2.
 * - Comportement bien défini pour segments dégénérés (A==B) : la branche c>=d couvre ce cas.
 */
/* `point_seg_dist2` (double) removed: use `point_seg_dist2_fixed_int` (integer) for distance checks. */

/*
 * set_segs_intersect_tol_px
 * -------------------------
 * Setter runtime pour ajuster la tolérance (en pixels) utilisée par
 * `segs_intersect_int_dbl` pour ignorer les intersections jugées trop
 * proches des sommets/aretes.
 *
 * Comportement :
 * - Les valeurs négatives sont clampées à 0.0 (tolérance minimale nulle).
 * - La tolérance est interprétée en pixels d'écran.
 * - Modifier cette valeur influence directement les décisions d'intersection
 *   : augmenter tol réduit les faux positifs sur croisements sub‑pixel, mais
 *   peut masquer de petits recouvrements réels.
 *
 * Usage recommandé : fixer la tolérance entre 0.25 et 1.5 px selon vos
 * exigences visuelles ; 1.0 px est le réglage par défaut après discussion.
 */
static void set_segs_intersect_tol_px(double px) {
    if (px < 0.0) px = 0.0;
    segs_intersect_tol_px = px;
}

/*
 * segs_intersect_int_dbl
 * ----------------------
 * Double-precision core used by the integer wrapper and unit tests.
 *
 * Steps and rationale (detailed) :
 * 1) Orientation test (proper intersection):
 *    - We round coordinates to the nearest integer when computing orientations
 *      (via floor(x+0.5)) in order to preserve the original integer semantics
 *      used elsewhere in the codebase (avoids surprising behaviour due to tiny
 *      floating-point perturbations when deciding "proper" orientation).
 *    - A proper intersection requires strict sign differences for the pair of
 *      orientations on each segment (standard robust test).
 *
 * 2) Intersection point computation:
 *    - If orientation indicates a proper intersection, we compute the exact
 *      intersection point in double precision (t parameter and point coords).
 *    - Guard against near-parallel segments using a small epsilon on the
 *      determinant (fabs(denom) < 1e-12) to avoid numerical instability.
 *
 * 3) Proximity filtering (tolérance en pixels) :
 *    - We consider a global tolerance `segs_intersect_tol_px` (in pixels).
 *    - If the computed intersection point lies within `tol` of any of the 4
 *      segment endpoints, we treat the intersection as spurious and return 0.
 *      This removes sub-pixel / endpoint-touch cases that would otherwise be
 *      reported as proper intersections due to integer orientation tests.
 *
 * 4) Endpoint-segment proximity test :
 *    - Additionally, if any of the 4 segment endpoints is within `tol` of
 *      the other segment (distance point→segment < tol), we also ignore the
 *      intersection (returned 0). This handles cases where an endpoint lies
 *      very near the interior of the other segment but rounding/orientation
 *      might have flagged a proper intersection.
 *
 * 5) Final decision : return 1 only if the intersection is proper and passes
 *    both proximity filters. This yields a conservative behavior tuned to
 *    avoid visual artifacts from sub-pixel intersections while preserving
 *    real, well-separated crossings.
 */
/*
 * Compute squared distance (in fixed units) from point P(px,py) to segment AB (ax,ay)-(bx,by).
 * Returns an unsigned long long representing (distance_in_fixed_units)^2.
 * Implementation notes:
 *  - The function works entirely in integer arithmetic to avoid floating point.
 *  - We compute numsq = (wx*d - c*vx)^2 + (wy*d - c*vy)^2 and then divide by d^2,
 *    performing division before multiplying by FIXED_SCALE^2 to avoid overflow.
 */
static unsigned long long point_seg_dist2_fixed_int(int px,int py,int ax,int ay,int bx,int by) {
    long long vx = (long long)bx - (long long)ax, vy = (long long)by - (long long)ay;
    long long wx = (long long)px - (long long)ax, wy = (long long)py - (long long)ay;
    long long c = vx*wx + vy*wy;
    long long d = vx*vx + vy*vy;
    if (d == 0) {
        long long dx0 = px - ax, dy0 = py - ay;
        unsigned long long r = (unsigned long long)dx0*(unsigned long long)dx0 + (unsigned long long)dy0*(unsigned long long)dy0;
        return r * (unsigned long long)FIXED_SCALE * (unsigned long long)FIXED_SCALE;
    }
    if (c <= 0) {
        long long dx0 = px - ax, dy0 = py - ay;
        unsigned long long r = (unsigned long long)dx0*(unsigned long long)dx0 + (unsigned long long)dy0*(unsigned long long)dy0;
        return r * (unsigned long long)FIXED_SCALE * (unsigned long long)FIXED_SCALE;
    } else if (c >= d) {
        long long dx0 = px - bx, dy0 = py - by;
        unsigned long long r = (unsigned long long)dx0*(unsigned long long)dx0 + (unsigned long long)dy0*(unsigned long long)dy0;
        return r * (unsigned long long)FIXED_SCALE * (unsigned long long)FIXED_SCALE;
    } else {
        long long numx = wx * d - c * vx;
        long long numy = wy * d - c * vy;
        unsigned long long numsq = (unsigned long long)numx * (unsigned long long)numx + (unsigned long long)numy * (unsigned long long)numy;
        unsigned long long den = (unsigned long long)d * (unsigned long long)d;
        unsigned long long q = numsq / den;
        unsigned long long rem = numsq % den;
        unsigned long long scale2 = (unsigned long long)FIXED_SCALE * (unsigned long long)FIXED_SCALE;
        unsigned long long out = q * scale2 + (rem * scale2) / den;
        return out;
    }
}

static int segs_intersect_int_fixed64(int x1,int y1,int x2,int y2,int x3,int y3,int x4,int y4) {
    long long o1 = orient_ll(x1,y1,x2,y2,x3,y3);
    long long o2 = orient_ll(x1,y1,x2,y2,x4,y4);
    long long o3 = orient_ll(x3,y3,x4,y4,x1,y1);
    long long o4 = orient_ll(x3,y3,x4,y4,x2,y2);
    /* Proper intersection only: require strict orientation differences */
    if (!(((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) && ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0)))) return 0;

    long long dx1 = x2 - x1, dy1 = y2 - y1;
    long long dx2 = x4 - x3, dy2 = y4 - y3;
    long long denom = dx1 * (-dy2) - dy1 * (-dx2);
    if (denom == 0) return 0; /* numerically parallel */

    long long tnum = (long long)(x3 - x1) * (-dy2) - (long long)(y3 - y1) * (-dx2);

    /* intersection point in Fixed64 */
    Fixed64 t_fixed = ((Fixed64)tnum << FIXED_SHIFT) / (Fixed64)denom;
    Fixed64 ix_fixed = (((Fixed64)x1) << FIXED_SHIFT) + ((t_fixed * (((Fixed64)dx1) << FIXED_SHIFT)) >> FIXED_SHIFT);
    Fixed64 iy_fixed = (((Fixed64)y1) << FIXED_SHIFT) + ((t_fixed * (((Fixed64)dy1) << FIXED_SHIFT)) >> FIXED_SHIFT);

    /* tolerance in fixed units */
    Fixed64 tol_fixed = (Fixed64)(segs_intersect_tol_px * (double)FIXED_SCALE + 0.5);
    unsigned long long tol2_fixed = (unsigned long long)tol_fixed * (unsigned long long)tol_fixed; /* fixed^2 */

    /* distance to endpoints (in fixed units squared) */
    long long ddx, ddy; unsigned long long dist2u;
    ddx = (long long)ix_fixed - ((long long)x1 << FIXED_SHIFT); ddy = (long long)iy_fixed - ((long long)y1 << FIXED_SHIFT);
    dist2u = (unsigned long long)ddx * (unsigned long long)ddx + (unsigned long long)ddy * (unsigned long long)ddy; if (dist2u < tol2_fixed) return 0;
    ddx = (long long)ix_fixed - ((long long)x2 << FIXED_SHIFT); ddy = (long long)iy_fixed - ((long long)y2 << FIXED_SHIFT);
    dist2u = (unsigned long long)ddx * (unsigned long long)ddx + (unsigned long long)ddy * (unsigned long long)ddy; if (dist2u < tol2_fixed) return 0;
    ddx = (long long)ix_fixed - ((long long)x3 << FIXED_SHIFT); ddy = (long long)iy_fixed - ((long long)y3 << FIXED_SHIFT);
    dist2u = (unsigned long long)ddx * (unsigned long long)ddx + (unsigned long long)ddy * (unsigned long long)ddy; if (dist2u < tol2_fixed) return 0;
    ddx = (long long)ix_fixed - ((long long)x4 << FIXED_SHIFT); ddy = (long long)iy_fixed - ((long long)y4 << FIXED_SHIFT);
    dist2u = (unsigned long long)ddx * (unsigned long long)ddx + (unsigned long long)ddy * (unsigned long long)ddy; if (dist2u < tol2_fixed) return 0;



    if (point_seg_dist2_fixed_int(x1,y1,x3,y3,x4,y4) < tol2_fixed) return 0;
    if (point_seg_dist2_fixed_int(x2,y2,x3,y3,x4,y4) < tol2_fixed) return 0;
    if (point_seg_dist2_fixed_int(x3,y3,x1,y1,x2,y2) < tol2_fixed) return 0;
    if (point_seg_dist2_fixed_int(x4,y4,x1,y1,x2,y2) < tol2_fixed) return 0;

    return 1;
}

/* Double-based regression helpers removed: Fixed (integer) implementations are now authoritative.
 * The old `segs_intersect_int_dbl` and associated double point/segment helpers were deleted
 * to remove redundant floating-point paths and reduce maintenance surface.
 */

static int segs_intersect_int(int x1,int y1,int x2,int y2,int x3,int y3,int x4,int y4) {
    return segs_intersect_int_fixed64(x1,y1,x2,y2,x3,y3,x4,y4);
}
static int point_in_poly_int(int px,int py,FaceArrays3D* faces, VertexArrays3D* vtx, int f, int n) {
    int cnt = 0; int off = faces->vertex_indices_ptr[f];
    for (int i = 0; i < n; ++i) {
        int j = (i+1)%n;
        int vi = faces->vertex_indices_buffer[off + i] - 1;
        int vj = faces->vertex_indices_buffer[off + j] - 1;
        int xi = vtx->x2d[vi], yi = vtx->y2d[vi];
        int xj = vtx->x2d[vj], yj = vtx->y2d[vj];
        /* If the point lies exactly on the edge, consider it OUTSIDE (no overlap).
         * Use orient==0 + on-segment test to detect boundary points. */
        if (orient_ll(xi, yi, xj, yj, px, py) == 0 && on_seg_ll(xi, yi, xj, yj, px, py)) return 0;
        if (((yi > py) != (yj > py)) && (px < (long long)(xj - xi) * (py - yi) / (yj - yi) + xi)) cnt++;
    }
    return (cnt & 1);
}

/* Return 1 if faces f1 and f2 have identical 2D vertex sequences (allowing rotation
 * and reversed order). Comparison uses exact integer 2D coordinates. */
static int faces_vertices_equal(FaceArrays3D* faces, VertexArrays3D* vtx, int f1, int f2) {
    int n1 = faces->vertex_count[f1];
    int n2 = faces->vertex_count[f2];
    if (n1 != n2 || n1 <= 0) return 0;
    int off1 = faces->vertex_indices_ptr[f1];
    int off2 = faces->vertex_indices_ptr[f2];

    /* Forward rotation check */
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

    /* Reverse rotation check */
    for (int shift = 0; shift < n1; ++shift) {
        int ok = 1;
        for (int k = 0; k < n1; ++k) {
            int idx1 = faces->vertex_indices_buffer[off1 + k] - 1;
            int idx2 = faces->vertex_indices_buffer[off2 + ((shift - k + n1) % n1)] - 1;
            if (idx1 < 0 || idx2 < 0) { ok = 0; break; }
            if (vtx->x2d[idx1] != vtx->x2d[idx2] || vtx->y2d[idx1] != vtx->y2d[idx2]) { ok = 0; break; }
        }
        if (ok) return 1;
    }
    return 0;
}

/* Unit tests for segs_intersect: run with command-line flag --run-segs-test */
static int use_fixed_clipping = 1; /* enable Fixed64 clipping by default (non-invasive) */
static void set_use_fixed_clipping(int v) { use_fixed_clipping = v ? 1 : 0; }



static int segs_intersect_unit_tests(void) {
    struct {
        double x1,y1,x2,y2,x3,y3,x4,y4; int expect; const char *name;
    } tests[] = {
        {233.0,114.0, 98.0,88.0, 191.0,106.0, 212.0,101.0, 0, "case_7_vs_45"},
        {0.0,0.0, 10.0,0.0, 5.0,-5.0, 5.0,5.0, 1, "orthogonal_mid"},
        {0.0,0.0, 10.0,0.0, 9.1,-1.0, 9.1,1.0, 0, "near_endpoint"},
        {0.0,0.0, 10.0,0.0, 10.0,-1.0, 10.0,1.0, 0, "touch_endpoint"},
        {0.0,0.0, 10.0,0.0, 1.0,-1.0, 1.0,1.0, 1, "near_mid"}
    };
    int n = sizeof(tests)/sizeof(tests[0]);
    printf("Running segs_intersect unit tests (tol=%.3f px)\n", segs_intersect_tol_px);
    int failed = 0;
    for (int i = 0; i < n; ++i) {
        int res_fixed = segs_intersect_int((int)tests[i].x1,(int)tests[i].y1,(int)tests[i].x2,(int)tests[i].y2,(int)tests[i].x3,(int)tests[i].y3,(int)tests[i].x4,(int)tests[i].y4);
        printf(" test %s: expected=%d fixed=%d\n", tests[i].name, tests[i].expect, res_fixed);
        if (res_fixed != tests[i].expect) { printf("  -> FIXED FAIL\n"); failed++; } else printf("  -> FIXED PASS\n");
    }
    if (failed == 0) printf("All segs_intersect tests passed\n"); else printf("%d segs_intersect tests FAILED\n", failed);
    return failed;
}

/*
 * projected_polygons_overlap_simple (legacy / simple overlap test)
 * ---------------------------------------------------------------
 * Purpose:
 *  - Provide a fast, legacy-style screen-space overlap test used historically
 *    by the renderer. It implements a straightforward sequence of inexpensive
 *    checks and avoids the more costly sampling+clipping steps found in the
 *    stricter `projected_polygons_overlap` implementation.
 *
 * Algorithm summary (simple):
 *  1) Axis-aligned bbox quick-reject (if bboxes are disjoint -> NON-overlap)
 *  2) Edge-vs-edge *proper* intersection tests with per-edge bbox quick-reject
 *     (returns overlap on any proper intersection; colinear or endpoint-touching
 *     do not count as proper intersection)
 *  3) Containment checks via ray-casting for vertices that fall inside the
 *     other's bbox (points on edges count as OUTSIDE)
 *  - Returns 1 for detected intersection/containment, 0 otherwise.
 *
 * Performance & semantics:
 *  - This variant is faster and tends to accept borderline overlaps (including
 *    small/degenerate cases) because it does not enforce a minimum clipped area
 *    threshold. It is thus useful for compatibility and to avoid strict rejections
 *    that can cause ordering artifacts in some meshes.
 *
 * projected_polygons_overlap (strict / precise overlap test)
 * ----------------------------------------------------------
 * Purpose:
 *  - A more conservative and precise overlap test that reduces false positives
 *    for touching or sub-pixel intersections.
 *
 * Algorithm summary (strict):
 *  1) Axis-aligned bbox quick-reject (same as simple)
 *  2) Edge intersection + containment quick checks; if these indicate a candidate
 *     intersection, the function samples a 3×3 grid of interior pixel centers in
 *     the integer intersection bbox (fast acceptance if any sample lies inside
 *     both polygons).
 *  3) If sampling fails, the strict algorithm performs exact polygon clipping
 *     (Sutherland–Hodgman) and computes the clipped polygon area. The overlap
 *     is accepted only if the clipped area >= MIN_INTERSECTION_AREA_PIXELS (1.0).
 *
 * Rationale & guidance:
 *  - Use `projected_polygons_overlap_simple` when you need legacy/compat behavior
 *    (performance or to reproduce older rendering decisions).
 *  - Use `projected_polygons_overlap` when you require conservative detection
 *    that treats touching-only or tiny intersections as NON-overlap.
 *
 * Backwards compatibility:
 *  - Historically some callers used the name `projected_polygons_overlapV0` to
 *    refer to the legacy behavior. `projected_polygons_overlap_simple` preserves
 *    that semantics. An alias may be provided for compatibility.
 */
static int projected_polygons_overlap_simple(Model3D* model, int f1, int f2) {
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;

    /* Inform the user when an overlap check is performed (useful in interactive mode). */
    #if ENABLE_DEBUG_SAVE
    printf("Checking projected overlap for faces %d and %d (touching is considered NON-overlap)\n", f1, f2);
    #endif
    int n1 = faces->vertex_count[f1];
    int n2 = faces->vertex_count[f2];
    if (n1 < 3 || n2 < 3) return 0;

    int minx1 = faces->minx[f1], maxx1 = faces->maxx[f1], miny1 = faces->miny[f1], maxy1 = faces->maxy[f1];
    int minx2 = faces->minx[f2], maxx2 = faces->maxx[f2], miny2 = faces->miny[f2], maxy2 = faces->maxy[f2];
    if (maxx1 < minx2 || maxx2 < minx1 || maxy1 < miny2 || maxy2 < miny1) return 0;

    int off1 = faces->vertex_indices_ptr[f1];
    int off2 = faces->vertex_indices_ptr[f2];

    /* Edge-vs-edge proper intersection with per-edge bbox quick-reject.
     * This avoids expensive orientation tests for clearly separated edges.
     * We use <= in bbox checks so that touching-only edges are treated as
     * non-overlapping (consistent with the semantics). */
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
            /* quick reject if edge AABBs do not overlap (<= to consider touching as non-overlap) */
            if (amaxx <= cminx || cmaxx <= aminx || amaxy <= cminy || cmaxy <= aminy) continue;
            if (segs_intersect_int(ax,ay,bx,by,cx,cy,dx,dy)) return 1;
        }
    }

    // SUPPRESSION : Containment tests are disabled. 
    //Risk: case where a face is completely contained in another may be missed (Z test will sort it)
    /* Containment tests: only check if candidate point lies inside the other's bbox first
     * (cheap) before doing the full ray-cast in point_in_poly_int. This skips expensive
     * loops for points obviously outside the other polygon's bbox. */
    // Containment tests: check *all* vertices of poly1 against poly2, and vice versa.
    // This avoids missing a containment when the polygon's first vertex lies on a shared
    // boundary point (touching) which is treated as outside.
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

/*
 * projected_polygons_overlap(model, f1, f2)
 * -----------------------------------------
 * Détermine si les projections 2D des faces `f1` et `f2` se recouvrent (le simple contact = NON-recouvrement).
 * La décision est prise par une suite de tests de coût croissant afin d'être conservatif et rapide.
 *
 * Étapes (dans l'ordre) :
 *  1) Rejet rapide par AABB : si les boîtes entières sont disjointes ou seulement en contact -> NON-overlap.
 *  2) Règle d'acceptation précoce (heuristique) : si une arête d'un polygone présente **≥ 2** intersections propres
 *     avec l'autre polygone (entrée + sortie), on accepte immédiatement (recouvrement certain pour cette arête).
 *     Cette règle vise à éviter le clipping coûteux dans les cas évidents. Attention : le résultat dépend du
 *     comportement de `segs_intersect_int` (et de sa tolérance interne), donc la tolérance peut affecter ce test.
 *  3) Vérification arête‑contre‑arête (intersection propre) : pour chaque paire d'arêtes, AABB quick-reject puis
 *     test d'intersection entier ; une intersection propre marque la paire comme *candidate* (on n'accepte pas
 *     immédiatement mais on continue avec les tests d'échantillonnage et de clipping).
 *  4) Tests de contenance : vérifier si un sommet de l'un est strictement à l'intérieur de l'autre (points sur bord
 *     comptés comme extérieurs).
 *  5) Cas spécial : polygones identiques (mêmes séquences de sommets) sont considérés comme recouvrant.
 *  6) Gestion des candidats et échantillonnage : calculer la boîte d'intersection entière et tenter des échantillons
 *     rapides (centre puis grille 3×3) pour acceptation rapide.
 *  7) Si l'échantillonnage échoue, recours au clipping exact (Sutherland–Hodgman) dans les deux sens (f1→f2 et f2→f1).
 *  8) Test de validité sur l'aire de clipping : n'accepter que si aire >= MIN_INTERSECTION_AREA_PIXELS et <= aire bbox (+eps).
 *
 * Lorsqu'un résultat de clipping est valide dans les deux sens, on préfère (f1 clipped by f2) pour conserver la sémantique
 * Python. Les hooks de débogage impriment des détails sur le clipping et les centroïdes quand activés.
 */
static int projected_polygons_overlap(Model3D* model, int f1, int f2) {
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    /* Minimum area (pixels^2) considered as true overlap (use global constant) */
    /* (local override removed so global MIN_INTERSECTION_AREA_PIXELS controls behavior) */
    /* Inform the user when an overlap check is performed (useful in interactive mode). */
    #if ENABLE_DEBUG_SAVE
    printf("Checking projected overlap for faces %d and %d (touching is considered NON-overlap)\n", f1, f2);
    #endif
    int n1 = faces->vertex_count[f1];
    int n2 = faces->vertex_count[f2];
    if (n1 < 3 || n2 < 3) return 0;

    /* instrumentation */
    overlapCheckCount++;

    int minx1 = faces->minx[f1], maxx1 = faces->maxx[f1], miny1 = faces->miny[f1], maxy1 = faces->maxy[f1];
    int minx2 = faces->minx[f2], maxx2 = faces->maxx[f2], miny2 = faces->miny[f2], maxy2 = faces->maxy[f2];

    /* Step 1: Quick reject by AABB (axis-aligned bounding boxes)
     * - If integer bounding boxes are disjoint or only touch at an edge/point
     *   we treat the polygons as NON-overlapping (touching-only = non-overlap).
     * - This is a very cheap early filter to avoid running heavier tests.
     */
    if (maxx1 <= minx2 || maxx2 <= minx1 || maxy1 <= miny2 || maxy2 <= miny1) return 0;

    int off1 = faces->vertex_indices_ptr[f1];
    int off2 = faces->vertex_indices_ptr[f2];

    /* Step 2: Early-accept heuristic — per-edge double-intersection rule
     * - Heuristic: if any single edge of f1 (or f2) has >= 2 proper intersections
     *   with the other polygon, then that edge must enter and exit the polygon
     *   (clear crossing). We accept immediately (return 1) to avoid expensive
     *   sampling/clipping in these straightforward cases.
     * - Note: this depends on `segs_intersect_int`'s definition of "proper"
     *   intersection and on the configured tolerance; changing tolerance may
     *   affect which intersections are counted here.
     */
{
    int count;

    /* ---- Test segments of f1 against f2 ---- */
    for (int i = 0; i < n1; ++i) {
        int i2 = (i+1) % n1;
        int va = faces->vertex_indices_buffer[off1 + i]  - 1;
        int vb = faces->vertex_indices_buffer[off1 + i2] - 1;
        int ax = vtx->x2d[va], ay = vtx->y2d[va];
        int bx = vtx->x2d[vb], by = vtx->y2d[vb];

        count = 0;

        for (int j = 0; j < n2; ++j) {
            int j2 = (j+1) % n2;
            int vc = faces->vertex_indices_buffer[off2 + j]  - 1;
            int vd = faces->vertex_indices_buffer[off2 + j2] - 1;
            int cx = vtx->x2d[vc], cy = vtx->y2d[vc];
            int dx = vtx->x2d[vd], dy = vtx->y2d[vd];

            if (segs_intersect_int(ax,ay,bx,by,cx,cy,dx,dy)) {
                count++;
                if (count >= 2) return 1; /* YES */
            }
        }
    }

    /* ---- Test segments of f2 against f1 ---- */
    for (int j = 0; j < n2; ++j) {
        int j2 = (j+1) % n2;
        int vc = faces->vertex_indices_buffer[off2 + j]  - 1;
        int vd = faces->vertex_indices_buffer[off2 + j2] - 1;
        int cx = vtx->x2d[vc], cy = vtx->y2d[vc];
        int dx = vtx->x2d[vd], dy = vtx->y2d[vd];

        count = 0;

        for (int i = 0; i < n1; ++i) {
            int i2 = (i+1) % n1;
            int va = faces->vertex_indices_buffer[off1 + i]  - 1;
            int vb = faces->vertex_indices_buffer[off1 + i2] - 1;
            int ax = vtx->x2d[va], ay = vtx->y2d[va];
            int bx = vtx->x2d[vb], by = vtx->y2d[vb];

            if (segs_intersect_int(cx,cy,dx,dy,ax,ay,bx,by)) {
                count++;
                if (count >= 2) return 1; /* YES */
            }
        }
    }
}


    /* Debug: enable verbose output for a specific pair by setting OVERLAP_DEBUG_PAIR="f1,f2" in the environment */
    int dbg_pair = 0;


    /* Step 3: Edge-vs-edge proper intersection check.
     * For each edge in f1 and each edge in f2:
     *  - Quick AABB reject to skip clearly separated edges (<= treats touching as non-overlap).
     *  - If AABBs overlap, perform integer segment intersection test.
     * If a proper segment intersection is found we mark the pair as a candidate
     * (note: we do not immediately accept; further tests follow). */
    int candidate = 0;
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
            /* quick reject if edge AABBs do not overlap (<= to consider touching as non-overlap) */
            if (amaxx <= cminx || cmaxx <= aminx || amaxy <= cminy || cmaxy <= aminy) continue;
            if (segs_intersect_int(ax,ay,bx,by,cx,cy,dx,dy)) {
                /* Mark candidate on proper segment intersection (do not accept immediately) */
                candidate = 1;
                overlapCheckCount++; overlapSegiAccept++;
                if (dbg_pair) printf("OVERLAP_DEBUG: pair %d,%d -> seg-intersect -> candidate marked\n", f1, f2);
                break;
            }
        }
        if (candidate) break;
    }

    /* Step 4: Containment tests.
     * Check every vertex of poly1 against poly2 and vice versa, skipping vertices outside the other's bbox.
     * Points lying exactly on the other's boundary are considered outside (no overlap).
     * If a vertex is strictly inside the other polygon, mark the pair as a candidate. */
    if (!candidate) {
        for (int ii = 0; ii < n1; ++ii) {
            int vid = faces->vertex_indices_buffer[off1 + ii] - 1;
            if (vid < 0 || vid >= vtx->vertex_count) continue;
            int px = vtx->x2d[vid], py = vtx->y2d[vid];
            if (px < minx2 || px > maxx2 || py < miny2 || py > maxy2) continue;
            if (point_in_poly_int(px, py, faces, vtx, f2, n2)) { candidate = 1; break; }
        }
    }
    if (!candidate) {
        for (int jj = 0; jj < n2; ++jj) {
            int vid = faces->vertex_indices_buffer[off2 + jj] - 1;
            if (vid < 0 || vid >= vtx->vertex_count) continue;
            int px = vtx->x2d[vid], py = vtx->y2d[vid];
            if (px < minx1 || px > maxx1 || py < miny1 || py > maxy1) continue;
            if (point_in_poly_int(px, py, faces, vtx, f1, n1)) { candidate = 1; break; }
        }
    }

    if (!candidate) {
        /* Step 5: Identical polygons special-case.
         * If the 2D vertex sequences are identical (maybe reversed), treat the faces as overlapping.
         */
        if (faces_vertices_equal(faces, vtx, f1, f2)) return 1;
        return 0;
    }

    /* Step 6: Candidate handling & sampling fallback.
     * Compute the integer intersection bbox of the projected polygons.
     * Fast path: test the center pixel and a 3x3 grid of interior pixel centers.
     * If any sampled pixel is strictly inside both polygons, accept immediately.
     * Otherwise, proceed to exact clipping fallback and area thresholding. */
    int oxmin = minx1 > minx2 ? minx1 : minx2;
    int oxmax = maxx1 < maxx2 ? maxx1 : maxx2;
    int oymin = miny1 > miny2 ? miny1 : miny2;
    int oymax = maxy1 < maxy2 ? maxy1 : maxy2; /* FIX: use maxy2 not miny2 */
    if (oxmin > oxmax || oymin > oymax) return 0; /* integer bbox empty -> <1 pixel */

    /* Step 7: Quick center test (cheap) - check center of integer bbox */
    int cx = (oxmin + oxmax) / 2; int cy = (oymin + oymax) / 2;
    if (point_in_poly_int(cx, cy, faces, vtx, f1, n1) && point_in_poly_int(cx, cy, faces, vtx, f2, n2)) {
        overlapSampleAccept++;
        if (dbg_pair) printf("OVERLAP_DEBUG: pair %d,%d -> center_accept=(%d,%d)\n", f1, f2, cx, cy);
        return 1;
    }

    /* Step 8: Adaptive 3x3 sampling around bbox interior (center already tested above). */
    int ixmin = oxmin, ixmax = oxmax, iymin = oymin, iymax = oymax;
    int W = ixmax - ixmin; int H = iymax - iymin;
    int sample_accept = 0; int N = 3;
    for (int sx = 0; sx < N; ++sx) {
        for (int sy = 0; sy < N; ++sy) {
            int tx = ixmin + (((2*sx + 1) * W + (2*N - 1)) / (2*N));
            int ty = iymin + (((2*sy + 1) * H + (2*N - 1)) / (2*N));
            if (point_in_poly_int(tx, ty, faces, vtx, f1, n1) && point_in_poly_int(tx, ty, faces, vtx, f2, n2)) { sample_accept = 1; break; }
        }
        if (sample_accept) break;
    }

    if (sample_accept) {
        overlapSampleAccept++;
        if (dbg_pair) { printf("OVERLAP_DEBUG: pair %d,%d -> sample_accept=%d (N=%d)\n", f1, f2, sample_accept, N); }
        return 1;
    }

    /* Step 9: Exact clipping fallback using Sutherland–Hodgman.
     * Perform ordered clipping in both directions (f1 clipped by f2, and f2 clipped by f1).
     * For each clipping result compute centroid and absolute area in pixel^2.
     * A clipping result is valid only if:
     *  - clipping succeeded, and
     *  - area >= MIN_INTERSECTION_AREA_PIXELS, and
     *  - area <= bbox_area + eps (sanity: can't exceed integer bbox area)
     * If both orders produce valid results, prefer (f1 clipped by f2) to match Python semantics.
     */
    int icx1 = 0, icy1 = 0; double iarea1 = 0.0;
    int icx2 = 0, icy2 = 0; double iarea2 = 0.0;
    overlapClipCalls++;

    /* Non-invasive Fixed64 clipping option: if enabled, use integer-area comparison
     * to decide validity early; otherwise fall back to existing double-based logic.
     */
    unsigned long long area2_1 = 0, area2_2 = 0;
    int ok1 = 0, ok2 = 0;
    if (use_fixed_clipping) {
        /* compute fixed-area intersection (uses same internal buffers; sets debug_clip_raw_area2) */
        int tx=0, ty=0; long long a2 = 0; debug_overlap_subj = f1; debug_overlap_clip = f2; ok1 = compute_intersection_centroid_ordered_fixed(model, f1, f2, &tx, &ty, &a2); debug_overlap_subj = -1; debug_overlap_clip = -1; area2_1 = (unsigned long long)(a2 >= 0 ? a2 : -a2);
        debug_clip_fixed_area = (double)area2_1 * 0.5 / ((double)FIXED_SCALE * (double)FIXED_SCALE);
        int tx2=0, ty2=0; long long a22 = 0; debug_overlap_subj = f2; debug_overlap_clip = f1; ok2 = compute_intersection_centroid_ordered_fixed(model, f2, f1, &tx2, &ty2, &a22); debug_overlap_subj = -1; debug_overlap_clip = -1; area2_2 = (unsigned long long)(a22 >= 0 ? a22 : -a22);
        /* Also compute double areas for diagnostic parity */
        int dtx=0,dty=0; long long _tmpa2_1 = 0; /* diagnostic parity via QD bbox */
        compute_intersection_centroid_ordered_qd_fixed(model, f1, f2, &dtx, &dty, &_tmpa2_1);
        iarea1 = (double)_tmpa2_1;
        long long _tmpa2_2 = 0;
        compute_intersection_centroid_ordered_qd_fixed(model, f2, f1, &dtx, &dty, &_tmpa2_2);
        iarea2 = (double)_tmpa2_2; 
    } else {
        /* default behavior: double-based clipping (existing path) */
        debug_overlap_subj = f1; debug_overlap_clip = f2; long long _ia2_1 = 0; ok1 = compute_intersection_centroid_ordered_qd_fixed(model, f1, f2, &icx1, &icy1, &_ia2_1); iarea1 = ok1 ? (double)_ia2_1 : 0.0; debug_overlap_subj = -1; debug_overlap_clip = -1;
        debug_overlap_subj = f2; debug_overlap_clip = f1; long long _ia2_2 = 0; ok2 = compute_intersection_centroid_ordered_qd_fixed(model, f2, f1, &icx2, &icy2, &_ia2_2); iarea2 = ok2 ? (double)_ia2_2 : 0.0; debug_overlap_subj = -1; debug_overlap_clip = -1;
    }

    double bbox_area = 0.0; if (!(oxmin > oxmax || oymin > oymax)) bbox_area = (double)(oxmax - oxmin) * (double)(oymax - oymin);

    if (dbg_pair) {
        printf("OVERLAP_DEBUG: pair %d,%d -> ok1=%d iarea1=%.6f icx1=%d icy1=%d | ok2=%d iarea2=%.6f icx2=%d icy2=%d | bbox_area=%.6f\n",
               f1, f2, ok1, iarea1, icx1, icy1, ok2, iarea2, icx2, icy2, bbox_area);
        if (debug_clip_vcount > 0) {
            printf("OVERLAP_DEBUG: clipped verts (latest): ");
            for (int ii = 0; ii < debug_clip_vcount; ++ii) printf("(%d,%d) ", debug_clip_vx[ii], debug_clip_vy[ii]);
            printf("\n");
        }
        printf("PROJ_SCALE: %.4f px/unit\n", FIXED_TO_FLOAT(s_global_proj_scale_fixed));
    }

    const double eps = 1e-9;
    int valid1 = 0, valid2 = 0;
    if (use_fixed_clipping) {
        /* Compare using fixed-area thresholds (area2 units):
         * area2_fixed must be >= 2 * MIN_INTERSECTION_AREA_PIXELS * FIXED_SCALE^2
         * and <= 2 * bbox_area * FIXED_SCALE^2 (sanity bound).
         */
        unsigned long long min_area2_fixed = (unsigned long long)(2.0 * MIN_INTERSECTION_AREA_PIXELS * (double)FIXED_SCALE * (double)FIXED_SCALE + 0.5);
        unsigned long long bbox_limit = 0;
        if (!(oxmin > oxmax || oymin > oymax)) {
            unsigned long long bbox_pixels = (unsigned long long)(oxmax - oxmin) * (unsigned long long)(oymax - oymin);
            unsigned long long scale2 = (unsigned long long)FIXED_SCALE * (unsigned long long)FIXED_SCALE;
            bbox_limit = 2ULL * bbox_pixels * scale2;
        }
        valid1 = (ok1 && debug_clip_fixed_vcount >= 3 && area2_1 >= min_area2_fixed && area2_1 <= bbox_limit);
        valid2 = (ok2 && debug_clip_fixed_vcount >= 3 && area2_2 >= min_area2_fixed && area2_2 <= bbox_limit);
    } else {
        valid1 = (ok1 && iarea1 >= MIN_INTERSECTION_AREA_PIXELS && iarea1 <= bbox_area + eps);
        valid2 = (ok2 && iarea2 >= MIN_INTERSECTION_AREA_PIXELS && iarea2 <= bbox_area + eps);
    }

    if (valid1 || valid2) {
        /* Prefer f1->f2 when both valid (Python semantics) */
        if (valid1) {
            overlapClipAccept++; if (dbg_pair) printf("OVERLAP_DEBUG: pair %d,%d -> ACCEPT (f1 clipped by f2 valid)\n", f1, f2);
            return 1;
        } else {
            overlapClipAccept++; if (dbg_pair) printf("OVERLAP_DEBUG: pair %d,%d -> ACCEPT (f2 clipped by f1 valid)\n", f1, f2);
            return 1;
        }
    }
    if (dbg_pair) { printf("OVERLAP_DEBUG: pair %d,%d -> REJECT (no valid clipped area >= threshold)\n", f1, f2); }
    return 0;
}


/* compute_intersection_centroid_ordered
 * Signature (kept here as a comment for reference):
 * static int compute_intersection_centroid_ordered(Model3D* model, int subj, int clip, int* outx, int* outy, double* out_area);
 *
 * Reason for removal: centroid-from-clipped-polygon implementation is deprecated —
 * we now use runs-based region bbox or fixed-point variants. Keep a stub so
 * callers remain valid; preserve the original signature above for future
 * reference.
 */
static int compute_intersection_centroid_ordered(Model3D* model, int subj, int clip, int* outx, int* outy, double* out_area) {
    (void)model; (void)subj; (void)clip; (void)outx; (void)outy; (void)out_area;
    return 0; 
}

static int compute_intersection_centroid_ordered_fixed(Model3D* model, int subj, int clip, int* outx, int* outy, long long* out_area2) {
    /* Robust integer implementation for diagnostics and fixed-area comparisons.
     * - Clips `subj` by `clip` using Sutherland–Hodgman (double intermediates).
     * - Produces integer output vertices (rounded), computes signed raw area2
     *   (shoelace: 2 * pixel-area) and centroid (integer-rounded).
     * - Returns 1 if a clipped polygon (>=3 verts) was produced; 0 otherwise.
     * - Writes *out_area2 in FIXED units (raw_area2 * FIXED_SCALE^2).
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

    /* Gather integer input polygons (screen coords) */
    int *sx = (int*)malloc(sizeof(int) * sn);
    int *sy = (int*)malloc(sizeof(int) * sn);
    int *cx = (int*)malloc(sizeof(int) * cn);
    int *cy = (int*)malloc(sizeof(int) * cn);
    if (!sx || !sy || !cx || !cy) {
        if (sx) free(sx); if (sy) free(sy); if (cx) free(cx); if (cy) free(cy);
        debug_clip_fixed_vcount = 0; debug_clip_raw_area2 = 0; debug_clip_fixed_area = 0.0; *out_area2 = 0; return 0;
    }
    for (int i = 0; i < sn; ++i) {
        int vi = faces->vertex_indices_buffer[faces->vertex_indices_ptr[subj] + i] - 1;
        sx[i] = vtx->x2d[vi]; sy[i] = vtx->y2d[vi];
    }
    for (int i = 0; i < cn; ++i) {
        int vi = faces->vertex_indices_buffer[faces->vertex_indices_ptr[clip] + i] - 1;
        cx[i] = vtx->x2d[vi]; cy[i] = vtx->y2d[vi];
    }

    /* Optional detailed debug dump for failing cases (writes centroidlog.txt) */
    FILE* dlog = NULL;
    if ((debug_overlap_subj == subj && debug_overlap_clip == clip) || (subj == 12 && clip == 50)) {
        dlog = fopen("centroidlog.txt", "w");
        if (dlog) {
            fprintf(dlog, "--- compute_intersection_centroid_ordered_fixed debug (subj=%d clip=%d) ---\n", subj, clip);
            fprintf(dlog, "input subj verts (count=%d):\n", sn);
            for (int i = 0; i < sn; ++i) fprintf(dlog, "%d %d\n", sx[i], sy[i]);
            fprintf(dlog, "input clip verts (count=%d):\n", cn);
            for (int i = 0; i < cn; ++i) fprintf(dlog, "%d %d\n", cx[i], cy[i]);
            fflush(dlog);
        }
    }

    /* Working buffers on heap to avoid stack exhaustion for large polygons */
    double *buf_x1 = (double*)malloc(sizeof(double) * DEBUG_CLIP_MAX);
    double *buf_y1 = (double*)malloc(sizeof(double) * DEBUG_CLIP_MAX);
    double *buf_x2 = (double*)malloc(sizeof(double) * DEBUG_CLIP_MAX);
    double *buf_y2 = (double*)malloc(sizeof(double) * DEBUG_CLIP_MAX);
    int *out_px = (int*)malloc(sizeof(int) * DEBUG_CLIP_MAX);
    int *out_py = (int*)malloc(sizeof(int) * DEBUG_CLIP_MAX);
    if (!buf_x1 || !buf_y1 || !buf_x2 || !buf_y2 || !out_px || !out_py) {
        free(sx); free(sy); free(cx); free(cy);
        if (buf_x1) free(buf_x1); if (buf_y1) free(buf_y1); if (buf_x2) free(buf_x2); if (buf_y2) free(buf_y2);
        if (out_px) free(out_px); if (out_py) free(out_py);
        debug_clip_fixed_vcount = 0; debug_clip_raw_area2 = 0; debug_clip_fixed_area = 0.0; *out_area2 = 0; return 0;
    }

    int cur_n = sn;
    for (int i = 0; i < cur_n && i < DEBUG_CLIP_MAX; ++i) { buf_x1[i] = (double)sx[i]; buf_y1[i] = (double)sy[i]; }

    /* Sutherland–Hodgman: clip against each edge of `clip` */
    for (int ci = 0; ci < cn && cur_n > 0; ++ci) {
        int ci2 = (ci + 1) % cn;
        double cx1d = (double)cx[ci], cy1d = (double)cy[ci];
        double cx2d = (double)cx[ci2], cy2d = (double)cy[ci2];
        int out_n_tmp = 0;
        for (int si = 0; si < cur_n; ++si) {
            int sj = (si + 1) % cur_n;
            double sx1d = buf_x1[si], sy1d = buf_y1[si];
            double sx2d = buf_x1[sj], sy2d = buf_y1[sj];
            double side1 = (cx2d - cx1d) * (sy1d - cy1d) - (cy2d - cy1d) * (sx1d - cx1d);
            double side2 = (cx2d - cx1d) * (sy2d - cy1d) - (cy2d - cy1d) * (sx2d - cx1d);
            int inside1 = (side1 >= 0.0);
            int inside2 = (side2 >= 0.0);
            if (inside1 && inside2) {
                if (out_n_tmp < DEBUG_CLIP_MAX) { buf_x2[out_n_tmp] = sx2d; buf_y2[out_n_tmp] = sy2d; out_n_tmp++; }
            } else if (inside1 && !inside2) {
                double dx = sx2d - sx1d, dy = sy2d - sy1d;
                double ex = cx2d - cx1d, ey = cy2d - cy1d;
                /* denom = cross(segment, clipEdge) */
                double denom = dx * ey - dy * ex;
                double t = 0.0;
                if (fabs(denom) > 1e-12) t = ((cx1d - sx1d) * ey - (cy1d - sy1d) * ex) / denom;
                double ix = sx1d + t * dx; double iy = sy1d + t * dy;
                if (out_n_tmp < DEBUG_CLIP_MAX) { buf_x2[out_n_tmp] = ix; buf_y2[out_n_tmp] = iy; out_n_tmp++; }
            } else if (!inside1 && inside2) {
                double dx = sx2d - sx1d, dy = sy2d - sy1d;
                double ex = cx2d - cx1d, ey = cy2d - cy1d;
                /* denom = cross(segment, clipEdge) */
                double denom = dx * ey - dy * ex;
                double t = 0.0;
                if (fabs(denom) > 1e-12) t = ((cx1d - sx1d) * ey - (cy1d - sy1d) * ex) / denom;
                double ix = sx1d + t * dx; double iy = sy1d + t * dy;
                if (out_n_tmp < DEBUG_CLIP_MAX) { buf_x2[out_n_tmp] = ix; buf_y2[out_n_tmp] = iy; out_n_tmp++; }
                if (out_n_tmp < DEBUG_CLIP_MAX) { buf_x2[out_n_tmp] = sx2d; buf_y2[out_n_tmp] = sy2d; out_n_tmp++; }
            }
        }
        cur_n = out_n_tmp;
        for (int k = 0; k < cur_n; ++k) { buf_x1[k] = buf_x2[k]; buf_y1[k] = buf_y2[k]; }
        if (dlog) {
            fprintf(dlog, "after clip edge %d -> vertex_count=%d\n", ci, cur_n);
            for (int kk = 0; kk < cur_n; ++kk) fprintf(dlog, "  %.6f %.6f\n", buf_x1[kk], buf_y1[kk]);
            fflush(dlog);
        }
    }

    /* Round and remove duplicate/colinear vertices into out_px/out_py */
    int final_n = 0;
    for (int i = 0; i < cur_n; ++i) {
        int px = (int)floor(buf_x1[i] + 0.5);
        int py = (int)floor(buf_y1[i] + 0.5);
        if (final_n > 0 && out_px[final_n-1] == px && out_py[final_n-1] == py) continue;
        if (final_n >= 2) {
            int x1 = out_px[final_n-2], y1 = out_py[final_n-2];
            int x2 = out_px[final_n-1], y2 = out_py[final_n-1];
            long long cross = (long long)(x2 - x1) * (long long)(py - y1) - (long long)(y2 - y1) * (long long)(px - x1);
            if (cross == 0) { out_px[final_n-1] = px; out_py[final_n-1] = py; continue; }
        }
        if (final_n < DEBUG_CLIP_MAX) { out_px[final_n] = px; out_py[final_n] = py; final_n++; }
    }
    if (final_n >= 2 && out_px[0] == out_px[final_n-1] && out_py[0] == out_py[final_n-1]) final_n--;

    if (final_n < 3) {
        debug_clip_fixed_vcount = 0; debug_clip_raw_area2 = 0; debug_clip_fixed_area = 0.0; *out_area2 = 0;
        if (dlog) { fprintf(dlog, "final_n < 3 -> no polygon\n"); fclose(dlog); }
        free(sx); free(sy); free(cx); free(cy);
        free(buf_x1); free(buf_y1); free(buf_x2); free(buf_y2); free(out_px); free(out_py);
        return 0;
    }

    /* Copy to debug buffers */
    if (final_n > DEBUG_CLIP_MAX) final_n = DEBUG_CLIP_MAX;
    for (int i = 0; i < final_n; ++i) { debug_clip_fixed_vx[i] = out_px[i]; debug_clip_fixed_vy[i] = out_py[i]; }
    debug_clip_fixed_vcount = final_n;
    if (dlog) {
        fprintf(dlog, "rounded final polygon (count=%d):\n", final_n);
        for (int i = 0; i < final_n; ++i) fprintf(dlog, "%d %d\n", out_px[i], out_py[i]);
        fflush(dlog);
    }

    /* Compute signed raw area2 and centroid numerators */
    long long raw_area2_signed = 0;
    long long cx_num = 0, cy_num = 0;
    for (int i = 0; i < final_n; ++i) {
        int j = (i + 1) % final_n;
        long long xi = (long long)out_px[i], yi = (long long)out_py[i];
        long long xj = (long long)out_px[j], yj = (long long)out_py[j];
        long long cross = xi * yj - yi * xj;
        raw_area2_signed += cross;
        cx_num += (xi + xj) * cross;
        cy_num += (yi + yj) * cross;
    }

    if (raw_area2_signed == 0) {
        debug_clip_raw_area2 = 0; debug_clip_fixed_area = 0.0; *out_area2 = 0;
        if (dlog) { fprintf(dlog, "raw_area2_signed == 0 -> degenerate polygon\n"); fflush(dlog); }
        if (outx) *outx = out_px[0]; if (outy) *outy = out_py[0];
        free(sx); free(sy); free(cx); free(cy);
        free(buf_x1); free(buf_y1); free(buf_x2); free(buf_y2); free(out_px); free(out_py);
        if (dlog) fclose(dlog);
        return 1;
    }

    long long abs_raw = (raw_area2_signed >= 0) ? raw_area2_signed : -raw_area2_signed;
    debug_clip_raw_area2 = abs_raw;
    debug_clip_fixed_area = fabs((double)raw_area2_signed) * 0.5;

    /* Centroid formula: C = (1/(6*A)) * sum((xi+xj)*cross), with raw_area2_signed == 2*A
     * => Cx = (2 * cx_num) / (3 * raw_area2_signed)
     */
    long long numer_x = 2 * cx_num;
    long long numer_y = 2 * cy_num;
    long long denom = 3 * raw_area2_signed;
    if (denom < 0) { denom = -denom; numer_x = -numer_x; numer_y = -numer_y; }
    long long cx_i = (numer_x >= 0) ? (numer_x + denom/2) / denom : -(( -numer_x + denom/2) / denom);
    long long cy_i = (numer_y >= 0) ? (numer_y + denom/2) / denom : -(( -numer_y + denom/2) / denom);
    if (outx) *outx = (int)cx_i; if (outy) *outy = (int)cy_i;

    /* Return fixed-scaled raw area2 */
    unsigned long long a2_fixed = (unsigned long long)abs_raw * (unsigned long long)FIXED_SCALE * (unsigned long long)FIXED_SCALE;
    *out_area2 = (long long)a2_fixed;


    free(sx); free(sy); free(cx); free(cy);
    free(buf_x1); free(buf_y1); free(buf_x2); free(buf_y2); free(out_px); free(out_py);
    return 1;
}

/* Alternative implementation using QuickDraw regions (faster to reason about for
 * overlapping faces and robust to colinear/edge-sharing cases). This does NOT
 * compute the exact area-centroid, it returns the center of the region bbox.
 * Returns 1 and sets (*outx,*outy) and *out_area2 (bbox area) on success, 0 on no-overlap.
 */
static int compute_intersection_region_bbox(Model3D* model, int subj, int clip, int* ix0, int* iy0, int* ix1, int* iy1, long long* out_area2) {
    if (!model || !ix0 || !iy0 || !ix1 || !iy1) return 0;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    if (subj < 0 || clip < 0 || subj >= faces->face_count || clip >= faces->face_count) return 0;

    int n1 = faces->vertex_count[subj];
    int n2 = faces->vertex_count[clip];
    if (n1 <= 0 || n2 <= 0) return 0;

    /* Vertex index offsets used later */
    int off1 = faces->vertex_indices_ptr[subj];
    int off2 = faces->vertex_indices_ptr[clip];

    /* Compute per-face bounding boxes (used to clamp the region bbox) */
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

    /* Logging removed: region debug logs were deleted per request */

    RegionHndl r1 = NewRgn();
    RegionHndl r2 = NewRgn();
    RegionHndl r3 = NewRgn();
    if (!r1 || !r2 || !r3) {
        if (r1) DisposeRgn(r1);
        if (r2) DisposeRgn(r2);
        if (r3) DisposeRgn(r3);
        return 0;
    }

    /* Build region for subj */
    OpenRgn();
    off1 = faces->vertex_indices_ptr[subj];
    int v0 = faces->vertex_indices_buffer[off1] - 1;
    MoveTo(vtx->x2d[v0], vtx->y2d[v0]);
    for (int i = 1; i < n1; ++i) {
        int vi = faces->vertex_indices_buffer[off1 + i] - 1;
        LineTo(vtx->x2d[vi], vtx->y2d[vi]);
    }
    CloseRgn(r1);

    /* r1 debug dump removed */

    /* Build region for clip */
    OpenRgn();
    off2 = faces->vertex_indices_ptr[clip];
    int w0 = faces->vertex_indices_buffer[off2] - 1;
    MoveTo(vtx->x2d[w0], vtx->y2d[w0]);
    for (int i = 1; i < n2; ++i) {
        int wi = faces->vertex_indices_buffer[off2 + i] - 1;
        LineTo(vtx->x2d[wi], vtx->y2d[wi]);
    }
    CloseRgn(r2);

    /* r2 debug dump removed */

    /* Intersection */
    SectRgn(r1, r2, r3);

    /* debug Region drawing disabled */

    int r1_succ = 0;
    /* Compute runs-based intersection from r1 and r2 (avoid relying on r3 header) */
    {
        /* Parse r1 runs */
        int r1_succ = 0;
        HLock((Handle)r1);
        short* sp1 = (short*)(*r1);
        int s1 = (int)sp1[0]; int n1short = (s1>0)?(s1/2):0;
        int idx1 = 5;
        int cap1 = (n1short>8)?n1short:8; int cnt1 = 0;
        int (*run1)[3] = (int(*)[3])malloc(sizeof(int[3]) * cap1);
        while (idx1 < n1short) {
            int y = (int)sp1[idx1++]; if ((unsigned short)y == 0x3FFF) continue;
            while (idx1 < n1short && (unsigned short)sp1[idx1] != 0x3FFF) {
                int xs = (int)sp1[idx1++]; if (idx1 >= n1short) break; int xe = (int)sp1[idx1++];
                if (cnt1 >= cap1) { cap1 *= 2; run1 = (int(*)[3])realloc(run1, sizeof(int[3]) * cap1); }
                run1[cnt1][0] = y; run1[cnt1][1] = xs; run1[cnt1][2] = xe; cnt1++;
            }
            if (idx1 < n1short && (unsigned short)sp1[idx1] == 0x3FFF) idx1++;
        }
        HUnlock((Handle)r1);

        /* Parse r2 runs */
        HLock((Handle)r2);
        short* sp2 = (short*)(*r2);
        int s2 = (int)sp2[0]; int n2short = (s2>0)?(s2/2):0;
        int idx2 = 5;
        int cap2 = (n2short>8)?n2short:8; int cnt2 = 0;
        int (*run2)[3] = (int(*)[3])malloc(sizeof(int[3]) * cap2);
        while (idx2 < n2short) {
            int y = (int)sp2[idx2++]; if ((unsigned short)y == 0x3FFF) continue;
            while (idx2 < n2short && (unsigned short)sp2[idx2] != 0x3FFF) {
                int xs = (int)sp2[idx2++]; if (idx2 >= n2short) break; int xe = (int)sp2[idx2++];
                if (cnt2 >= cap2) { cap2 *= 2; run2 = (int(*)[3])realloc(run2, sizeof(int[3]) * cap2); }
                run2[cnt2][0] = y; run2[cnt2][1] = xs; run2[cnt2][2] = xe; cnt2++;
            }
            if (idx2 < n2short && (unsigned short)sp2[idx2] == 0x3FFF) idx2++;
        }
        HUnlock((Handle)r2);

        /* Two-pointer pass to compute overlaps by matching y values */
        int i = 0, j = 0;
        int runs_minx = 32767, runs_maxx = -32768, runs_miny = 32767, runs_maxy = -32768;
        while (i < cnt1 && j < cnt2) {
            int y1 = run1[i][0]; int y2v = run2[j][0];
            if (y1 < y2v) { i++; continue; }
            if (y1 > y2v) { j++; continue; }
            int i2 = i; while (i2 < cnt1 && run1[i2][0] == y1) i2++;
            int j2 = j; while (j2 < cnt2 && run2[j2][0] == y1) j2++;
            for (int a = i; a < i2; ++a) {
                for (int b = j; b < j2; ++b) {
                    int lo = (run1[a][1] > run2[b][1]) ? run1[a][1] : run2[b][1];
                    int hi = (run1[a][2] < run2[b][2]) ? run1[a][2] : run2[b][2];
                    if (lo <= hi) {
                        if (lo < runs_minx) runs_minx = lo;
                        if (hi > runs_maxx) runs_maxx = hi;
                        if (y1 < runs_miny) runs_miny = y1;
                        if (y1 > runs_maxy) runs_maxy = y1;
                    }
                }
            }
            i = i2; j = j2;
        }

        if (runs_minx <= runs_maxx && runs_miny <= runs_maxy) {
            *ix0 = runs_minx; *iy0 = runs_miny; *ix1 = runs_maxx; *iy1 = runs_maxy;
            r1_succ = 1;
        } else {
            r1_succ = 0;
        }

        free(run1); free(run2);
    }

    int has = 0;
    if (!EmptyRgn(r3)) {

        HLock((Handle)r3);
        short* sp = (short*)(*r3);
        int w0 = (int)sp[0];


        /* On classic QuickDraw the region record starts with a size (bytes) in sp[0],
         * followed by the bbox words [top,left,bottom,right] in sp[1..4]. Use that
         * convention when possible; otherwise fall back to the previous heuristic.
         */
        int raw_top, raw_left, raw_bottom, raw_right;
        if (w0 >= 10 && (w0/2) >= 5) {
            raw_top = (int)sp[1]; raw_left = (int)sp[2]; raw_bottom = (int)sp[3]; raw_right = (int)sp[4];
        } else {
            /* Fallback: use first four words as before */
            raw_top = (int)sp[0]; raw_left = (int)sp[1]; raw_bottom = (int)sp[2]; raw_right = (int)sp[3];
        }
        /* Normalize X and Y orientations into (left,top,right,bottom) from header */
        int h_top = (raw_top < raw_bottom) ? raw_top : raw_bottom;
        int h_bottom = (raw_top < raw_bottom) ? raw_bottom : raw_top;
        int h_left = (raw_left < raw_right) ? raw_left : raw_right;
        int h_right = (raw_left < raw_right) ? raw_right : raw_left;


        /* Parse the runs in the region record to compute a bbox based on actual runs.
         * Format after the 5 header words is a sequence of scanline descriptions:
         *   y, x1, x2, x3, x4, ..., 0x3FFF  (end of that scanline runs)
         * repeated. We'll walk the words and extract min/max X/Y seen in runs.
         */
        int nshorts = (w0 > 0) ? (w0 / 2) : 0;
        int idx = 5; /* start after size and header (sp[0..4]) */
        int runs_minx = 32767, runs_maxx = -32768, runs_miny = 32767, runs_maxy = -32768;
        while (idx < nshorts) {
            int y = (int)sp[idx++];
            if (y == 0x3FFF) continue; /* robustness */
            runs_miny = (y < runs_miny) ? y : runs_miny;
            runs_maxy = (y > runs_maxy) ? y : runs_maxy;
            /* Read until 0x3FFF sentinel */
            while (idx < nshorts && (unsigned short)sp[idx] != 0x3FFF) {
                int xs = (int)sp[idx++]; if (idx >= nshorts) break;
                int xe = (int)sp[idx++];
                if (xs < runs_minx) runs_minx = xs;
                if (xe > runs_maxx) runs_maxx = xe;
            }
            /* consume sentinel if present */
            if (idx < nshorts && (unsigned short)sp[idx] == 0x3FFF) idx++;
        }
        if (runs_minx <= runs_maxx && runs_miny <= runs_maxy) {
    
            /* Prefer runs bbox as authoritative, but do not override runs-based result if present */
            if (!r1_succ) {
                *ix0 = runs_minx; *iy0 = runs_miny; *ix1 = runs_maxx; *iy1 = runs_maxy;
            } else {
    
            }
        } else {
            /* Fall back to header bbox if runs parsing failed */

            if (!r1_succ) {
                *ix0 = h_left; *iy0 = h_top; *ix1 = h_right; *iy1 = h_bottom;
            } else {

            }
        }

        /* Clamp region bbox to intersection of face bboxes to avoid overshoot */
        int ibx0 = (minx1 > minx2) ? minx1 : minx2;
        int ibx1 = (maxx1 < maxx2) ? maxx1 : maxx2;
        int iby0 = (miny1 > miny2) ? miny1 : miny2;
        int iby1 = (maxy1 < maxy2) ? maxy1 : maxy2;

        if (*ix0 < ibx0) *ix0 = ibx0;
        if (*iy0 < iby0) *iy0 = iby0;
        if (*ix1 > ibx1) *ix1 = ibx1;
        if (*iy1 > iby1) *iy1 = iby1;

        if (*ix0 <= *ix1 && *iy0 <= *iy1) {
            if (out_area2) *out_area2 = (long long)((*ix1 - *ix0) * (*iy1 - *iy0));
            has = 1;

        } else {

        }
        HUnlock((Handle)r3);
    } else {

    }

    DisposeRgn(r1); DisposeRgn(r2); DisposeRgn(r3);

    return has;
}

static int compute_intersection_centroid_ordered_qd_fixed(Model3D* model, int subj, int clip, int* outx, int* outy, long long* out_area2) {
    int ix0,iy0,ix1,iy1; long long a2=0;
    if (!compute_intersection_region_bbox(model, subj, clip, &ix0, &iy0, &ix1, &iy1, &a2)) return 0;
    if (outx) *outx = (ix0 + ix1) / 2;
    if (outy) *outy = (iy0 + iy1) / 2;
    if (out_area2) *out_area2 = a2;
    return 1;
}

segment "code03";

/* Evaluate tests 1..7 individually for a pair (f1, f2).
 * out[0]..out[6] will be filled with:
 *   -1 => test concludes f1 is before f2
 *    1 => test concludes f1 is after f2
 *    0 => test inconclusive
 */
static void evaluate_pair_tests(Model3D* model, int f1, int f2, int out[7]) {
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    Fixed32 epsilon = FLOAT_TO_FIXED(0.01f);
    int k;

    for (k = 0; k < 7; ++k) out[k] = 0;

    // Test 1: depth separation
    if (faces->z_max[f2] <= faces->z_min[f1]) out[0] = -1;
    else if (faces->z_max[f1] <= faces->z_min[f2]) out[0] = 1;

    // Test 2: X bbox separation
    {
        int minx1 = faces->minx[f1], maxx1 = faces->maxx[f1], minx2 = faces->minx[f2], maxx2 = faces->maxx[f2];
        if (maxx1 <= minx2 || maxx2 <= minx1) out[1] = -1;
    }

    // Test 3: Y bbox separation
    {
        int miny1 = faces->miny[f1], maxy1 = faces->maxy[f1], miny2 = faces->miny[f2], maxy2 = faces->maxy[f2];
        if (maxy1 <= miny2 || maxy2 <= miny1) out[2] = -1;
    }

    // Plane-based tests 4..7
    int n1 = faces->vertex_count[f1];
    int n2 = faces->vertex_count[f2];
    int offset1 = faces->vertex_indices_ptr[f1];
    int offset2 = faces->vertex_indices_ptr[f2];
    Fixed64 a1 = faces->plane_a[f1]; Fixed64 b1 = faces->plane_b[f1]; Fixed64 c1 = faces->plane_c[f1]; Fixed64 d1 = faces->plane_d[f1];
    Fixed64 a2 = faces->plane_a[f2]; Fixed64 b2 = faces->plane_b[f2]; Fixed64 c2 = faces->plane_c[f2]; Fixed64 d2 = faces->plane_d[f2];
    int obs_side1 = 0; int obs_side2 = 0; int side; int all_same_side; int all_opposite_side;

    // Test 4
    obs_side1 = 0; if (d1 > (Fixed64)epsilon) obs_side1 = 1; else if (d1 < -(Fixed64)epsilon) obs_side1 = -1; else obs_side1 = 0;
    if (obs_side1 != 0) {
        all_same_side = 1;
        for (k = 0; k < n2; ++k) {
            int v = faces->vertex_indices_buffer[offset2+k]-1;
            Fixed64 acc = 0;
            acc  = (((Fixed64)a1 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)b1 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)c1 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
            acc += (Fixed64)d1;
            if (acc > (Fixed64)epsilon) side = 1; else if (acc < -(Fixed64)epsilon) side = -1; else continue;
            if (obs_side1 != side) { all_same_side = 0; break; }
        }
        if (all_same_side) out[3] = -1; else out[3] = 0;
    } else out[3] = 0;

    // Test 5
    obs_side2 = 0; if (d2 > (Fixed64)epsilon) obs_side2 = 1; else if (d2 < -(Fixed64)epsilon) obs_side2 = -1; else obs_side2 = 0;
    if (obs_side2 != 0) {
        all_opposite_side = 1;
        for (k = 0; k < n1; ++k) {
            int v = faces->vertex_indices_buffer[offset1+k]-1;
            Fixed64 acc = 0;
            acc  = (((Fixed64)a2 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)b2 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)c2 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
            acc += (Fixed64)d2;
            if (acc > (Fixed64)epsilon) side = 1; else if (acc < -(Fixed64)epsilon) side = -1; else continue;
            if (obs_side2 == side) { all_opposite_side = 0; break; }
        }
        if (all_opposite_side) out[4] = -1; else out[4] = 0;
    } else out[4] = 0;

    // Test 6
    obs_side1 = 0; if (d1 > (Fixed64)epsilon) obs_side1 = 1; else if (d1 < -(Fixed64)epsilon) obs_side1 = -1; else obs_side1 = 0;
    if (obs_side1 != 0) {
        all_opposite_side = 1;
        for (k = 0; k < n2; ++k) {
            int v = faces->vertex_indices_buffer[offset2+k]-1;
            Fixed64 acc = 0;
            acc  = (((Fixed64)a1 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)b1 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)c1 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
            acc += (Fixed64)d1;
            if (acc > (Fixed64)epsilon) side = 1; else if (acc < -(Fixed64)epsilon) side = -1; else continue;
            if (obs_side1 == side) { all_opposite_side = 0; break; }
        }
        if (all_opposite_side) out[5] = 1; else out[5] = 0;
    } else out[5] = 0;

    // Test 7
    obs_side2 = 0; if (d2 > (Fixed64)epsilon) obs_side2 = 1; else if (d2 < -(Fixed64)epsilon) obs_side2 = -1; else obs_side2 = 0;
    if (obs_side2 != 0) {
        all_same_side = 1;
        for (k = 0; k < n1; ++k) {
            int v = faces->vertex_indices_buffer[offset1+k]-1;
            Fixed64 acc = 0;
            acc  = (((Fixed64)a2 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)b2 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)c2 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
            acc += (Fixed64)d2;
            if (acc > (Fixed64)epsilon) side = 1; else if (acc < -(Fixed64)epsilon) side = -1; else continue;
            if (obs_side2 != side) { all_same_side = 0; break; }
        }
        if (all_same_side) out[6] = 1; else out[6] = 0;
    } else out[6] = 0;
}

/* Plane-based pair relation used by `painter_correct` fast path:
 * - Only evaluates geometric plane tests (equivalent to tests 4..7)
 * - Returns -1 if f1 is before f2, 1 if f1 is after f2, 0 if inconclusive
 *
 * This avoids depth/bbox checks which are unnecessary for the local
 * geometric decision in `painter_correct`.
 */


/* Specialized plane-only checks for painter_correct fast path
 * ------------------------------------------------------------
 * These functions implement the geometric depth tests from Newell-Sancha algorithm
 * using only plane equations (no bounding box checks - caller must do those first).
 *
 * - pair_plane_after(f1,f2): returns 1 if f1 is geometrically after (in front of) f2
 *   Implements Test6: all vertices of f2 on opposite side of f1's plane from observer
 *   Falls back to Test7: all vertices of f1 on same side of f2's plane as observer
 *
 * - pair_plane_before(f1,f2): returns 1 if f1 is geometrically before (behind) f2  
 *   Implements Test4: all vertices of f2 on same side of f1's plane as observer
 *   Falls back to Test5: all vertices of f1 on opposite side of f2's plane from observer
 *
 * Both functions:
 *   - Use Fixed64 arithmetic for plane equation evaluation
 *   - Apply epsilon tolerance (0.01) to handle near-coplanar cases
 *   - Return 0 if tests are inconclusive
 *   - Are only valid for front-faces (plane_d > 0); back-faces need special handling
 */

static int pair_plane_after(Model3D* model, int f1, int f2) {
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    
    Fixed32 epsilon = FLOAT_TO_FIXED(0.01f);
    int k;
    int n1 = faces->vertex_count[f1];
    int n2 = faces->vertex_count[f2];
    int offset1 = faces->vertex_indices_ptr[f1];
    int offset2 = faces->vertex_indices_ptr[f2];
    Fixed64 a1 = faces->plane_a[f1]; Fixed64 b1 = faces->plane_b[f1]; Fixed64 c1 = faces->plane_c[f1]; Fixed64 d1 = faces->plane_d[f1];

    /* Test6: Check if all vertices of f2 are on the opposite side of f1's plane from the observer.
     * If observer is in front of f1 (d1 > 0) and all f2 vertices are behind f1's plane,
     * then f1 occludes f2 -> f1 is after (should be drawn later). */
    int obs_side1 = 0; int side; int all_opposite_side;
    obs_side1 = 0; if (d1 > (Fixed64)epsilon) obs_side1 = 1; else if (d1 < -(Fixed64)epsilon) obs_side1 = -1; else obs_side1 = 0;
    if (obs_side1 != 0) {
        all_opposite_side = 1;
        for (k = 0; k < n2; ++k) {
            int v = faces->vertex_indices_buffer[offset2+k]-1;
            Fixed64 acc = 0;
            acc  = (((Fixed64)a1 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)b1 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)c1 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
            acc += (Fixed64)d1;
            if (acc > (Fixed64)epsilon) side = 1; else if (acc < -(Fixed64)epsilon) side = -1; else continue;
            if (obs_side1 == side) { all_opposite_side = 0; break; }
        }
        if (all_opposite_side) return 1;
    }
    /* Test7 fallback: Check if all vertices of f1 are on the same side of f2's plane as the observer.
     * If observer is in front of f2 and all f1 vertices are also in front, then f1 is after f2. */
    Fixed64 a2 = faces->plane_a[f2]; Fixed64 b2 = faces->plane_b[f2]; Fixed64 c2 = faces->plane_c[f2]; Fixed64 d2 = faces->plane_d[f2];
    int obs_side2 = 0; int all_same_side = 0;
    obs_side2 = 0; if (d2 > (Fixed64)epsilon) obs_side2 = 1; else if (d2 < -(Fixed64)epsilon) obs_side2 = -1; else obs_side2 = 0;
    if (obs_side2 != 0) {
        all_same_side = 1;
        for (k = 0; k < n1; ++k) {
            int v = faces->vertex_indices_buffer[offset1+k]-1;
            Fixed64 acc = 0;
            acc  = (((Fixed64)a2 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)b2 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)c2 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
            acc += (Fixed64)d2;
            if (acc > (Fixed64)epsilon) side = 1; else if (acc < -(Fixed64)epsilon) side = -1; else continue;
            if (obs_side2 != side) { all_same_side = 0; break; }
        }
        if (all_same_side) return 1;
    }
    return 0;
}

static int pair_plane_before(Model3D* model, int f1, int f2) {
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    
    Fixed32 epsilon = FLOAT_TO_FIXED(0.01f);
    int k;
    int n1 = faces->vertex_count[f1];
    int n2 = faces->vertex_count[f2];
    int offset1 = faces->vertex_indices_ptr[f1];
    int offset2 = faces->vertex_indices_ptr[f2];
    Fixed64 a1 = faces->plane_a[f1]; Fixed64 b1 = faces->plane_b[f1]; Fixed64 c1 = faces->plane_c[f1]; Fixed64 d1 = faces->plane_d[f1];

    /* Test4: Check if all vertices of f2 are on the same side of f1's plane as the observer.
     * If observer is in front of f1 and all f2 vertices are also in front,
     * then f2 occludes f1 -> f1 is before (should be drawn first). */
    int obs_side1 = 0; int side; int all_same_side;
    obs_side1 = 0; if (d1 > (Fixed64)epsilon) obs_side1 = 1; else if (d1 < -(Fixed64)epsilon) obs_side1 = -1; else obs_side1 = 0;
    if (obs_side1 != 0) {
        all_same_side = 1;
        for (k = 0; k < n2; ++k) {
            int v = faces->vertex_indices_buffer[offset2+k]-1;
            Fixed64 acc = 0;
            acc  = (((Fixed64)a1 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)b1 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)c1 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
            acc += (Fixed64)d1;
            if (acc > (Fixed64)epsilon) side = 1; else if (acc < -(Fixed64)epsilon) side = -1; else continue;
            if (obs_side1 != side) { all_same_side = 0; break; }
        }
        if (all_same_side) return 1;
    }
    /* Test5 fallback: Check if all vertices of f1 are on opposite side of f2's plane from observer.
     * If observer is in front of f2 and all f1 vertices are behind f2's plane, then f1 is before f2. */
    Fixed64 a2 = faces->plane_a[f2]; Fixed64 b2 = faces->plane_b[f2]; Fixed64 c2 = faces->plane_c[f2]; Fixed64 d2 = faces->plane_d[f2];
    int obs_side2 = 0; int all_opposite_side = 0;
    obs_side2 = 0; if (d2 > (Fixed64)epsilon) obs_side2 = 1; else if (d2 < -(Fixed64)epsilon) obs_side2 = -1; else obs_side2 = 0;
    if (obs_side2 != 0) {
        all_opposite_side = 1;
        for (k = 0; k < n1; ++k) {
            int v = faces->vertex_indices_buffer[offset1+k]-1;
            Fixed64 acc = 0;
            acc  = (((Fixed64)a2 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)b2 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)c2 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
            acc += (Fixed64)d2;
            if (acc > (Fixed64)epsilon) side = 1; else if (acc < -(Fixed64)epsilon) side = -1; else continue;
            if (obs_side2 == side) { all_opposite_side = 0; break; }
        }
        if (all_opposite_side) return 1;
    }
    return 0;
}

// ===================== RAY CAST & INSPECTOR PROTOTYPES =====================
int ray_cast(Model3D* model, int f1, int f2);
int check_sort(Model3D* model, int face_count);
int check_sort_repair(Model3D* model, int face_count);
void inspect_ray_cast(Model3D* model);
// Prototypes only at the top
// ===================== RAY CAST & INSPECTOR IMPLEMENTATION =====================
// Returns: 0 if error or undetermined, 1 if t1 < t2, -1 if t1 > t2,

// ===================== RAY CAST & INSPECTOR IMPLEMENTATION =====================
// Returns: 
// 0 if error or undetermined
// 1 if t1 farer than t2
// -1 if t1 closer than t2

/* Ray cast at centroid of bbox intersection (default) */
int ray_cast(Model3D* model, int f1, int f2) {
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    if (f1 < 0 || f2 < 0 || f1 >= faces->face_count || f2 >= faces->face_count) return 0;
    // Internal calculations moved here, no more pointers needed
    int minx1 = faces->minx[f1];
    int maxx1 = faces->maxx[f1];
    int miny1 = faces->miny[f1];
    int maxy1 = faces->maxy[f1];
    int minx2 = faces->minx[f2];
    int maxx2 = faces->maxx[f2];
    int miny2 = faces->miny[f2];
    int maxy2 = faces->maxy[f2];
    int ix0 = (minx1 > minx2) ? minx1 : minx2;
    int ix1 = (maxx1 < maxx2) ? maxx1 : maxx2;
    int iy0 = (miny1 > miny2) ? miny1 : miny2;
    int iy1 = (maxy1 < maxy2) ? maxy1 : maxy2;
    if (ix0 > ix1 || iy0 > iy1) return 0; // no intersection
    int cx = (ix0 + ix1) / 2;
    int cy = (iy0 + iy1) / 2;


    /* Simplified: don't attempt centroid-based decision. Use bbox center directly. */
    return ray_cast_at(model, f1, f2, cx, cy);
}

/* Compute ray cast distances (tf) for faces f1 and f2 at screen coordinate (cx,cy).
 * Returns 1 and fills *out_tf1/*out_tf2 on success, 0 if denom near zero (coplanar/parallel) or invalid.
 */
static int ray_cast_distances(Model3D* model, int f1, int f2, int cx, int cy, float *out_tf1, float *out_tf2) {
    if (!model || !out_tf1 || !out_tf2) return 0;
    FaceArrays3D* faces = &model->faces;
    if (f1 < 0 || f2 < 0 || f1 >= faces->face_count || f2 >= faces->face_count) return 0;
    float proj_scale = FIXED_TO_FLOAT(s_global_proj_scale_fixed);
    float Dx = ((float)cx - (float)CENTRE_X) / proj_scale;
    float Dy = ((float)CENTRE_Y - (float)cy) / proj_scale;
    float Dz = 1.0f;
    float A1 = (float)FIXED64_TO_FLOAT(faces->plane_a[f1]);
    float B1 = (float)FIXED64_TO_FLOAT(faces->plane_b[f1]);
    float C1 = (float)FIXED64_TO_FLOAT(faces->plane_c[f1]);
    float D1 = (float)FIXED64_TO_FLOAT(faces->plane_d[f1]);
    float A2 = (float)FIXED64_TO_FLOAT(faces->plane_a[f2]);
    float B2 = (float)FIXED64_TO_FLOAT(faces->plane_b[f2]);
    float C2 = (float)FIXED64_TO_FLOAT(faces->plane_c[f2]);
    float D2 = (float)FIXED64_TO_FLOAT(faces->plane_d[f2]);
    float denom1 = A1 * Dx + B1 * Dy + C1 * Dz;
    float denom2 = A2 * Dx + B2 * Dy + C2 * Dz;
    if (fabsf(denom1) < 1e-6f || fabsf(denom2) < 1e-6f) return 0; /* coplanar or parallel */
    *out_tf1 = -D1 / denom1;
    *out_tf2 = -D2 / denom2;
    return 1;
}

/* Ray cast at explicit screen coordinate (cx,cy) in projected 2D coords */
static int ray_cast_at(Model3D* model, int f1, int f2, int cx, int cy) {
    float tf1 = 0.0f, tf2 = 0.0f;
    if (!ray_cast_distances(model, f1, f2, cx, cy, &tf1, &tf2)) return 0;
    if (tf1 > tf2) return 1;  /* f1 is farther than f2 */
    else if (tf1 < tf2) return -1; /* f1 is closer than f2 */
    else return 0;
}

/* Compute center of intersection of two faces' projected axis-aligned bboxes.
 * Returns 1 and writes (*outx,*outy) on success, 0 if no intersection or invalid ids.
 */
static int compute_bbox_intersection_center(Model3D* model, int f1, int f2, int *outx, int *outy) {
    if (!model || !outx || !outy) return 0;
    FaceArrays3D* faces = &model->faces;
    if (f1 < 0 || f2 < 0 || f1 >= faces->face_count || f2 >= faces->face_count) return 0;
    int minx1 = faces->minx[f1], maxx1 = faces->maxx[f1], miny1 = faces->miny[f1], maxy1 = faces->maxy[f1];
    int minx2 = faces->minx[f2], maxx2 = faces->maxx[f2], miny2 = faces->miny[f2], maxy2 = faces->maxy[f2];
    int ix0 = (minx1 > minx2) ? minx1 : minx2;
    int ix1 = (maxx1 < maxx2) ? maxx1 : maxx2;
    int iy0 = (miny1 > miny2) ? miny1 : miny2;
    int iy1 = (maxy1 < maxy2) ? maxy1 : maxy2;
    if (ix0 > ix1 || iy0 > iy1) return 0; /* no intersection */
    *outx = (ix0 + ix1) / 2;
    *outy = (iy0 + iy1) / 2;
    return 1;
}

/* Compute axis-aligned integer intersection bbox of projected face bboxes.
 * Returns 1 and fills (*ix0,*iy0,*ix1,*iy1) on success, 0 if no intersection.
 */
static int compute_bbox_intersection(Model3D* model, int f1, int f2, int *ix0, int *iy0, int *ix1, int *iy1) {
    if (!model || !ix0 || !iy0 || !ix1 || !iy1) return 0;
    FaceArrays3D* faces = &model->faces;
    if (f1 < 0 || f2 < 0 || f1 >= faces->face_count || f2 >= faces->face_count) return 0;
    int minx1 = faces->minx[f1], maxx1 = faces->maxx[f1], miny1 = faces->miny[f1], maxy1 = faces->maxy[f1];
    int minx2 = faces->minx[f2], maxx2 = faces->maxx[f2], miny2 = faces->miny[f2], maxy2 = faces->maxy[f2];
    int a0 = (minx1 > minx2) ? minx1 : minx2;
    int a1 = (maxx1 < maxx2) ? maxx1 : maxx2;
    int b0 = (miny1 > miny2) ? miny1 : miny2;
    int b1 = (maxy1 < maxy2) ? maxy1 : maxy2;
    if (a0 > a1 || b0 > b1) return 0;
    *ix0 = a0; *iy0 = b0; *ix1 = a1; *iy1 = b1;
    return 1;
}

/* Integer point-in-polygon test using ray-crossing on integer vertex arrays.
 * Returns 1 if point (px,py) is inside or on edge, 0 otherwise.
 */
static int point_in_poly_arrays_int(int px, int py, int n, const int vx[], const int vy[]) {
    if (n < 3) return 0;
    int inside = 0;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        int xi = vx[i], yi = vy[i];
        int xj = vx[j], yj = vy[j];
        /* Check if point is exactly on a vertex */
        if (px == xi && py == yi) return 1;
        /* Ray crossing: test if edge (j->i) crosses horizontal line at py */
        int yi_gt_py = (yi > py);
        int yj_gt_py = (yj > py);
        if (yi_gt_py != yj_gt_py) {
            /* Compute intersection X coordinate in integer-safe way: xi + (py - yi) * (xj - xi) / (yj - yi) */
            long long dx = (long long)(xj - xi);
            long long dy = (long long)(yj - yi);
            long long lhs = (long long)(px - xi) * dy;
            long long rhs = (long long)(py - yi) * dx;
            if ((dy > 0 && lhs < rhs) || (dy < 0 && lhs > rhs)) inside = !inside;
        }
    }
    return inside;
}

/* Interactive helper: prompt for two face IDs, compute fixed-point intersection polygon
 * (via compute_intersection_centroid_ordered_fixed) and display wireframe model with the
 * clipped polygon overlaid. Mapped to the 'M' key by the caller.
 */
static void inspect_intersection_fixed_ui(Model3D* model) {
    if (!model) return;
    FaceArrays3D* faces = &model->faces;
    int face_count = faces->face_count;
    if (face_count <= 0) { printf("No faces in model\n"); return; }

    printf("Enter face id 1 (0..%d): ", face_count - 1);
    int f1 = -1;
    if (scanf("%d", &f1) != 1) { int ch; while ((ch = getchar()) != '\n' && ch != EOF); printf("Input cancelled\n"); return; }
    { int ch; while ((ch = getchar()) != '\n' && ch != EOF); }
    if (f1 < 0 || f1 >= face_count) { printf("Invalid face id 1\n"); return; }

    printf("Enter face id 2 (0..%d): ", face_count - 1);
    int f2 = -1;
    if (scanf("%d", &f2) != 1) { int ch; while ((ch = getchar()) != '\n' && ch != EOF); printf("Input cancelled\n"); return; }
    { int ch; while ((ch = getchar()) != '\n' && ch != EOF); }
    if (f2 < 0 || f2 >= face_count) { printf("Invalid face id 2\n"); return; }

    /* Clear previous debug polygon and compute fixed clipping (try both orders) */
    debug_clip_fixed_vcount = 0; debug_clip_raw_area2 = 0; debug_clip_fixed_area = 0.0;
    int cx = 0, cy = 0; long long a2 = 0;
    compute_intersection_centroid_ordered_fixed(model, f1, f2, &cx, &cy, &a2);
    if (debug_clip_fixed_vcount < 3) compute_intersection_centroid_ordered_fixed(model, f2, f1, &cx, &cy, &a2);

    /* Export debug file with 2D points for face1, face2 and computed intersection polygon */
    {
        FILE* wf = fopen("centroid.txt", "w");
        if (wf) {
            VertexArrays3D* vtx = &model->vertices;
            FaceArrays3D* faces = &model->faces;

            fprintf(wf, "# face1 id=%d\n", f1);
            int fn1 = faces->vertex_count[f1];
            for (int i = 0; i < fn1; ++i) {
                int vi = faces->vertex_indices_buffer[faces->vertex_indices_ptr[f1] + i] - 1;
                fprintf(wf, "%d %d\n", vtx->x2d[vi], vtx->y2d[vi]);
            }

            fprintf(wf, "# face2 id=%d\n", f2);
            int fn2 = faces->vertex_count[f2];
            for (int i = 0; i < fn2; ++i) {
                int vi = faces->vertex_indices_buffer[faces->vertex_indices_ptr[f2] + i] - 1;
                fprintf(wf, "%d %d\n", vtx->x2d[vi], vtx->y2d[vi]);
            }

            fprintf(wf, "# intersection_polygon count=%d area_fixed=%lld centroid=(%d,%d)\n", debug_clip_fixed_vcount, a2, cx, cy);
            for (int i = 0; i < debug_clip_fixed_vcount; ++i) fprintf(wf, "%d %d\n", debug_clip_fixed_vx[i], debug_clip_fixed_vy[i]);

            fclose(wf);
            printf("Wrote debug file: centroid.txt (face1=%d face2=%d vertices=%d)\n", f1, f2, debug_clip_fixed_vcount);
        } else {
            printf("Failed to open centroid.txt for writing\n");
        }
    }

    /* Start graphics and draw wireframe + overlay the debug polygon (if present) */
    startgraph(mode);
    int old_frame = framePolyOnly; framePolyOnly = 1; /* wireframe */
    VertexArrays3D* vtx = &model->vertices;
    for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = 1;
    if (jitter) drawPolygons_jitter(model, faces->vertex_count, faces->face_count, vtx->vertex_count);
    else drawPolygons(model, faces->vertex_count, faces->face_count, vtx->vertex_count);

    /* Draw intersection polygon populated by compute_intersection_centroid_ordered_fixed */
    if (debug_clip_fixed_vcount >= 3) {
        int screenScale = mode / 320;
        SetSolidPenPat(7);
        int first_sx = screenScale * (debug_clip_fixed_vx[0] + pan_dx);
        int first_sy = screenScale * (debug_clip_fixed_vy[0] + pan_dy);
        MoveTo(first_sx, first_sy);
        for (int ii = 1; ii < debug_clip_fixed_vcount; ++ii) {
            int sx = screenScale * (debug_clip_fixed_vx[ii] + pan_dx);
            int sy = screenScale * (debug_clip_fixed_vy[ii] + pan_dy);
            LineTo(sx, sy);
        }
        LineTo(first_sx, first_sy);

        SetSolidPenPat(15);
        for (int ii = 0; ii < debug_clip_fixed_vcount; ++ii) {
            int sx = screenScale * (debug_clip_fixed_vx[ii] + pan_dx);
            int sy = screenScale * (debug_clip_fixed_vy[ii] + pan_dy);
            Rect vr; SetRect(&vr, sx-1, sy-1, sx+1, sy+1); FrameRect(&vr);
        }
    } else {
        MoveTo(3, 10); printf("Intersection polygon: none (no clipped polygon)\n");
    }

    keypress();

    framePolyOnly = old_frame;
    endgraph();
    DoText();
}


/* Helper: show_graphical_inspect
 * --------------------------------
 * Extracted from the inline block inside `inspect_ray_cast` to improve
 * readability and maintenance.  All QuickDraw/graphics-only work is kept
 * here so callers can remain text-mode-safe.
 */
static void show_graphical_inspect(Model3D* model, int f1, int f2, int no_overlap,
                                  int saved_ix0, int saved_iy0, int saved_ix1, int saved_iy1,
                                  int has_bbox, int cx, int cy, int point_source, int cmp)
{
    if (!model) return;
    FaceArrays3D* faces = &model->faces;
    /* Local copies / locals that were previously in inspect_ray_cast */
    VertexArrays3D* vtx = &model->vertices;      /* required by drawPolygons/_jitter */
    int crx = 0, cry = 0;                         /* bbox center (fallback) */
    int ccx = 0, ccy = 0;                         /* QD centroid scratch */
    double iarea1 = 0.0;                          /* local area diagnostic */
    long long qd_area2 = 0;                      /* QD-area return value */
    int centroid_ok = 0;

    unsigned char* backup_flags = (unsigned char*)malloc(faces->face_count);
    if (backup_flags == NULL) { printf("Memory allocation failed\n"); DoText(); return; }
    for (int i = 0; i < faces->face_count; ++i) backup_flags[i] = faces->display_flag[i];
    int old_frame = framePolyOnly;

    startgraph(mode);

    framePolyOnly = 1; // wireframe
    for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = 1;
    if (jitter) drawPolygons_jitter(model, faces->vertex_count, faces->face_count, vtx->vertex_count);
    else drawPolygons(model, faces->vertex_count, faces->face_count, vtx->vertex_count);

    unsigned char saved_f1 = faces->display_flag[f1];
    unsigned char saved_f2 = faces->display_flag[f2];
    faces->display_flag[f1] = 1; faces->display_flag[f2] = 1;
    drawFace(model, f1, 10, 1);
    drawFace(model, f2, 6, 1);

    {
        int _old_frame = framePolyOnly; framePolyOnly = 1;
        drawFace(model, f1, -1, 0);
        framePolyOnly = _old_frame;
    }
    {
        int _old_frame = framePolyOnly; framePolyOnly = 1;
        drawFace(model, f2, -1, 0);
        framePolyOnly = _old_frame;
    }

    if (has_bbox) {
        Rect r;
        int screenScale = mode / 320;
        int sc_ix0 = screenScale * (saved_ix0 + pan_dx);
        int sc_ix1 = screenScale * (saved_ix1 + pan_dx);
        int sc_iy0 = screenScale * (saved_iy0 + pan_dy);
        int sc_iy1 = screenScale * (saved_iy1 + pan_dy);
        SetSolidPenPat(15);
        SetRect(&r, sc_ix0, sc_iy0, sc_ix1, sc_iy1);
        FrameRect(&r);

        if (debug_clip_fixed_vcount >= 3) {
            SetSolidPenPat(7);
            int first_sx = screenScale * (debug_clip_fixed_vx[0] + pan_dx);
            int first_sy = screenScale * (debug_clip_fixed_vy[0] + pan_dy);
            MoveTo(first_sx, first_sy);
            for (int ii = 1; ii < debug_clip_fixed_vcount; ++ii) {
                int sx = screenScale * (debug_clip_fixed_vx[ii] + pan_dx);
                int sy = screenScale * (debug_clip_fixed_vy[ii] + pan_dy);
                LineTo(sx, sy);
            }
            LineTo(first_sx, first_sy);

            SetSolidPenPat(15);
            for (int ii = 0; ii < debug_clip_fixed_vcount; ++ii) {
                int sx = screenScale * (debug_clip_fixed_vx[ii] + pan_dx);
                int sy = screenScale * (debug_clip_fixed_vy[ii] + pan_dy);
                Rect vr; SetRect(&vr, sx-1, sy-1, sx+1, sy+1); FrameRect(&vr);
            }

            double s_cross = 0.0, cx_sum = 0.0, cy_sum = 0.0;
            for (int ii = 0; ii < debug_clip_fixed_vcount; ++ii) {
                int jj = (ii + 1) % debug_clip_fixed_vcount;
                double xi = (double)debug_clip_fixed_vx[ii]; double yi = (double)debug_clip_fixed_vy[ii];
                double xj = (double)debug_clip_fixed_vx[jj]; double yj = (double)debug_clip_fixed_vy[jj];
                double cross = xi * yj - xj * yi;
                s_cross += cross;
                cx_sum += (xi + xj) * cross;
                cy_sum += (yi + yj) * cross;
            }
            if (s_cross != 0.0) {
                double centroid_fx = cx_sum / (3.0 * s_cross);
                double centroid_fy = cy_sum / (3.0 * s_cross);
                int sc_fx = screenScale * ((int)(centroid_fx + 0.5) + pan_dx);
                int sc_fy = screenScale * ((int)(centroid_fy + 0.5) + pan_dy);
                SetSolidPenPat(3);
                Rect fr; SetRect(&fr, sc_fx-2, sc_fy-2, sc_fx+2, sc_fy+2); FrameRect(&fr);
                (void)centroid_fx; (void)centroid_fy;
            }
        }
    }

    int screenScale = mode / 320;
    if (!no_overlap) {
        int tx0, ty0, tx1, ty1; long long tarea = 0;
        int region_ok = compute_intersection_region_bbox(model, f1, f2, &tx0, &ty0, &tx1, &ty1, &tarea);
        if (region_ok) {
            int sc_tx0 = screenScale * (tx0 + pan_dx);
            int sc_tx1 = screenScale * (tx1 + pan_dx);
            int sc_ty0 = (ty0 + pan_dy);
            int sc_ty1 = (ty1 + pan_dy);
            SetSolidPenPat(9);
            Rect rr; SetRect(&rr, sc_tx0, sc_ty0, sc_tx1, sc_ty1); FrameRect(&rr);

            int qdx = (tx0 + tx1) / 2;
            int qdy = (ty0 + ty1) / 2;
            int sc_cx = screenScale * (qdx + pan_dx);
            int sc_cy = (qdy + pan_dy);
            int d = 4 * screenScale;
            int th = screenScale > 0 ? screenScale : 1;
            Rect hr; SetRect(&hr, sc_cx - d, sc_cy - (th/2), sc_cx + d, sc_cy + (th/2) + 1); FrameRect(&hr);
            Rect vr; SetRect(&vr, sc_cx - (th/2), sc_cy - d, sc_cx + (th/2) + 1, sc_cy + d); FrameRect(&vr);

            int centroid_ok = 0;
            long long qd_area2 = 0;
            int qd_ok = compute_intersection_centroid_ordered_qd_fixed(model, f1, f2, &ccx, &ccy, &qd_area2);
            if (qd_ok) {
                MoveTo(1,10);
                printf("QD centroid: x=%d y=%d - area2: %.2f\n", ccx, ccy, (double)qd_area2);
                iarea1 = (double)qd_area2;
                float d1=0.0f,d2=0.0f;
                if (ray_cast_distances(model, f1, f2, ccx, ccy, &d1, &d2)) {
                    if (d1 < d2) cmp = -1; else if (d1 > d2) cmp = 1; else cmp = 0;
                } else {
                    cmp = ray_cast_at(model, f1, f2, ccx, ccy);
                }
                centroid_ok = 1; cx = ccx; cy = ccy; point_source = 1;
            } else {
                MoveTo(1,10); printf("QD centroid: KO\n");
                if (has_bbox) { cx = crx; cy = cry; point_source = 2; }
            }
        } else {
            if (has_bbox) {
                int sc_ix0 = screenScale * (saved_ix0 + pan_dx);
                int sc_ix1 = screenScale * (saved_ix1 + pan_dx);
                int sc_iy0 = (saved_iy0 + pan_dy);
                int sc_iy1 = (saved_iy1 + pan_dy);
                SetSolidPenPat(15);
                Rect r; SetRect(&r, sc_ix0, sc_iy0, sc_ix1, sc_iy1); FrameRect(&r);
                int fcx = (saved_ix0 + saved_ix1) / 2; int fcy = (saved_iy0 + saved_iy1) / 2;
                int sc_cx = screenScale * (fcx + pan_dx);
                int sc_cy = (fcy + pan_dy);
                int d = 4 * screenScale;
                int th = screenScale > 0 ? screenScale : 1;
                Rect hr; SetRect(&hr, sc_cx - d, sc_cy - (th/2), sc_cx + d, sc_cy + (th/2) + 1); FrameRect(&hr);
                Rect vr; SetRect(&vr, sc_cx - (th/2), sc_cy - d, sc_cx + (th/2) + 1, sc_cy + d); FrameRect(&vr);
            } else {
                MoveTo(3, 10);
                printf("Region bbox: undetermined\n");
            }
        }
    }

    MoveTo(3, 195);
    if (no_overlap) {
        printf("Face %d and face %d do not overlap\n", f1, f2);
    } else if (point_source == 1) {
        if (cmp == -1) printf("QD bbox center: %d,%d - f%d front\n", cx, cy, f1);
        else if (cmp == 1) printf("QD bbox center: %d,%d - f%d front\n", cx, cy, f2);
        else printf("QD bbox center: %d,%d - undetermined\n", cx, cy);
    } else if (point_source == 2) {
        if (cmp == -1) printf("bbox center: %d,%d - f%d front\n", cx, cy, f1);
        else if (cmp == 1) printf("bbox center: %d,%d - f%d front\n", cx, cy, f2);
        else printf("bbox center: %d,%d - undetermined\n", cx, cy);
    } else {
        printf("bbox center: none - undetermined\n");
    }
    keypress();

    for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = backup_flags[i];
    free(backup_flags);
    framePolyOnly = old_frame;
    endgraph();
    DoText();
}

/*
 * inspect_ray_cast
 */
void inspect_ray_cast(Model3D* model) {
    if (!model) return;
    FaceArrays3D* faces = &model->faces;
    int face_count = faces->face_count;
    if (face_count <= 0) { printf("No faces in model\n"); return; }
    printf("Inspector: ray_cast between two faces. Computes intersection center and plane tests.\n");
    printf("Enter face id 1 (0..%d): ", face_count - 1);
    int f1 = -1;

    if (scanf("%d", &f1) != 1) { int ch; while ((ch = getchar()) != '\n' && ch != EOF); printf("Input cancelled\n"); return; }
    { int ch; while ((ch = getchar()) != '\n' && ch != EOF); }
    if (f1 < 0 || f1 >= face_count) { printf("Invalid face id 1\n"); return; }
    
    printf("Enter face id 2 (0..%d): ", face_count - 1);
    int f2 = -1;
    if (scanf("%d", &f2) != 1) { int ch; while ((ch = getchar()) != '\n' && ch != EOF); printf("Input cancelled\n"); return; }
    { int ch; while ((ch = getchar()) != '\n' && ch != EOF); }
    if (f2 < 0 || f2 >= face_count) { printf("Invalid face id 2\n"); return; }

    /* Quick reject: if projected polygons do not overlap, note it and continue to graphical mode
     * (do not run the other tests). The graphics overlay will show a single-line message.
     */
    int no_overlap = 0;
    if (!projected_polygons_overlap(model, f1, f2)) {
        no_overlap = 1;
    }

    int cmp = 0;
    int icx1 = 0, icy1 = 0; double iarea1 = 0.0;
    int icx2 = 0, icy2 = 0; double iarea2 = 0.0;

    int cx = 0, cy = 0;
    int crx = 0, cry = 0;
    int ccx = 0, ccy = 0;
    /* point_source: 0=none, 1=centroid, 2=bbox-center */
    int point_source = 0;

    /* Precompute projected vertex arrays for bbox computation and reuse later */
    VertexArrays3D* vtx = &model->vertices;

    int saved_ix0 = 0, saved_ix1 = 0, saved_iy0 = 0, saved_iy1 = 0;
    int has_bbox = 0;

    if (!no_overlap) {

    if (compute_bbox_intersection(model, f1, f2, &saved_ix0, &saved_iy0, &saved_ix1, &saved_iy1)) {
        has_bbox = 1;
        crx = (saved_ix0 + saved_ix1) / 2;
        cry = (saved_iy0 + saved_iy1) / 2;
        printf("Bbox intersection center: x=%d y=%d\n", crx, cry);
        int bbox_w = saved_ix1 - saved_ix0;
        int bbox_h = saved_iy1 - saved_iy0;
        double bbox_area = (double)bbox_w * (double)bbox_h;
        printf("Intersection rectangle: width=%d height=%d area=%.2f (pixels^2)\n", bbox_w, bbox_h, bbox_area);
    } else {
        printf("Intersection rectangle: none\n");
    }

    {
        /* Non-graphical path: QuickDraw is not active here, so do NOT call QD-based functions.
         * Use bbox-center as a deterministic fallback for text-mode diagnostics.
         * The QD centroid (runs-based) will be computed later inside the graphical
         * inspection block where QuickDraw is initialized.
         */
        int centroid_ok = 0;
        iarea1 = 0.0;
        /* Prefer integer bbox center as a safe, always-available test point */
        if (has_bbox) { cx = crx; cy = cry; point_source = 2; }
        else if (compute_bbox_intersection_center(model, f1, f2, &crx, &cry)) { has_bbox = 1; cx = crx; cy = cry; point_source = 2; }
        else { cx = 0; cy = 0; point_source = 0; }
    }

    } else {
        /* No overlap: set defaults and skip tests (graphical mode will show single-line message). */
        has_bbox = 0; cx = 0; cy = 0; point_source = 0; iarea1 = 0.0; cmp = 0;
    }

    /* Compute and print ray_cast distances at the chosen test point (cx,cy) */
    {
        float tf1 = 0.0f, tf2 = 0.0f;
        if (!ray_cast_distances(model, f1, f2, cx, cy, &tf1, &tf2)) {
            printf("ray_cast distances at (%d,%d): denom very small (possible coplanar/parallel)\n", cx, cy);
        } else {
            printf("ray_cast distances at (%d,%d): tf(f%d)=%.6f tf(f%d)=%.6f (smaller=in front)\n", cx, cy, f1, tf1, f2, tf2);
        }
    }

    printf("Press any key to show graphical inspection...\n");
    keypress();
    show_graphical_inspect(model, f1, f2, no_overlap, saved_ix0, saved_iy0, saved_ix1, saved_iy1, has_bbox, cx, cy, point_source, cmp);
    return;


    // * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
    // --- Graphical inspection ---
    // * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
    //
    startgraph(mode);

    framePolyOnly = 1; // wireframe
    // Ensure all faces visible for wireframe backdrop
    for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = 1;
    if (jitter) drawPolygons_jitter(model, faces->vertex_count, faces->face_count, vtx->vertex_count); else drawPolygons(model, faces->vertex_count, faces->face_count, vtx->vertex_count);

    // Overlay filled faces: f1 = green (pen 10), f2 = orange (pen 6)
    unsigned char saved_f1 = faces->display_flag[f1];
    unsigned char saved_f2 = faces->display_flag[f2];
    faces->display_flag[f1] = 1; faces->display_flag[f2] = 1;
    drawFace(model, f1, 10, 1);
    drawFace(model, f2, 6, 1);

    // Draw red frame around both faces (outline) using FramePoly via drawFace (frame-only)
    {
        int old_frame = framePolyOnly; framePolyOnly = 1;
        drawFace(model, f1, -1, 0);
        framePolyOnly = old_frame;
    }
    {
        int old_frame = framePolyOnly; framePolyOnly = 1;
        drawFace(model, f2, -1, 0);
        framePolyOnly = old_frame;
    }

    /* The intersection rectangle is stored in `saved_ix0..saved_iy1` and drawn via FrameRect below. */
    if (has_bbox) {
        Rect r;
        int screenScale = mode / 320;
        int sc_ix0 = screenScale * (saved_ix0 + pan_dx);
        int sc_ix1 = screenScale * (saved_ix1 + pan_dx);
        int sc_iy0 = screenScale * (saved_iy0 + pan_dy);
        int sc_iy1 = screenScale * (saved_iy1 + pan_dy);

    /* Draw the clipped intersection polygon (if available) */
        if (debug_clip_fixed_vcount >= 3) {
            /* Outline in pen 7 (distinct) */
            SetSolidPenPat(7);
            int first_sx = screenScale * (debug_clip_fixed_vx[0] + pan_dx);
            int first_sy = screenScale * (debug_clip_fixed_vy[0] + pan_dy);
            MoveTo(first_sx, first_sy);
            for (int ii = 1; ii < debug_clip_fixed_vcount; ++ii) {
                int sx = screenScale * (debug_clip_fixed_vx[ii] + pan_dx);
                int sy = screenScale * (debug_clip_fixed_vy[ii] + pan_dy);
                LineTo(sx, sy);
            }
            LineTo(first_sx, first_sy); // close

            /* Draw small white boxes at each vertex */
            SetSolidPenPat(15);
            for (int ii = 0; ii < debug_clip_fixed_vcount; ++ii) {
                int sx = screenScale * (debug_clip_fixed_vx[ii] + pan_dx);
                int sy = screenScale * (debug_clip_fixed_vy[ii] + pan_dy);
                Rect vr; SetRect(&vr, sx-1, sy-1, sx+1, sy+1); FrameRect(&vr);
            }

            /* Compute and draw floating centroid (double) of the integer polygon for diagnostics */
            double s_cross = 0.0, cx_sum = 0.0, cy_sum = 0.0;
            for (int ii = 0; ii < debug_clip_fixed_vcount; ++ii) {
                int jj = (ii + 1) % debug_clip_fixed_vcount;
                double xi = (double)debug_clip_fixed_vx[ii]; double yi = (double)debug_clip_fixed_vy[ii];
                double xj = (double)debug_clip_fixed_vx[jj]; double yj = (double)debug_clip_fixed_vy[jj];
                double cross = xi * yj - xj * yi;
                s_cross += cross;
                cx_sum += (xi + xj) * cross;
                cy_sum += (yi + yj) * cross;
            }
            if (s_cross != 0.0) {
                double centroid_fx = cx_sum / (3.0 * s_cross);
                double centroid_fy = cy_sum / (3.0 * s_cross);
                int sc_fx = screenScale * ((int)(centroid_fx + 0.5) + pan_dx);
                int sc_fy = ((int)(centroid_fy + 0.5) + pan_dy);
                /* Red small filled rect for float centroid */
                SetSolidPenPat(3);
                Rect fr; SetRect(&fr, sc_fx-2, sc_fy-2, sc_fx+2, sc_fy+2); FrameRect(&fr);

                /* debug overlap suppressed in graphical text mode (minimal display) */
                (void)centroid_fx; (void)centroid_fy;
            }
        }

    }

        int screenScale = mode / 320;
        // If there was no projected polygon overlap, skip region/centroid computations and just show the faces and bbox (if any). The text-mode diagnostics will indicate no overlap, and the graphical overlay will show the faces without a highlighted intersection region (and a single-line message about no overlap).
        if (!no_overlap) {

        /* Compute region bbox and draw it (non-invasive but visible). Also compute QD centroid
         * here because QuickDraw is active in the graphical inspection block. If QD fails,
         * keep the integer bbox fallback drawn above.
         */
        {
            int tx0, ty0, tx1, ty1; long long tarea = 0;
            int region_ok = compute_intersection_region_bbox(model, f1, f2, &tx0, &ty0, &tx1, &ty1, &tarea);
            if (region_ok) {
                int sc_tx0 = screenScale * (tx0 + pan_dx);
                int sc_tx1 = screenScale * (tx1 + pan_dx);
                int sc_ty0 = (ty0 + pan_dy);
                int sc_ty1 = (ty1 + pan_dy);
                SetSolidPenPat(9); /* distinct pen for region bbox */
                Rect rr; SetRect(&rr, sc_tx0, sc_ty0, sc_tx1, sc_ty1); FrameRect(&rr);
                /* draw yellow cross at center of QD bbox */
                {
                    int qdx = (tx0 + tx1) / 2;
                    int qdy = (ty0 + ty1) / 2;
                    int sc_cx = screenScale * (qdx + pan_dx);
                    int sc_cy = (qdy + pan_dy);
                    int d = 4 * screenScale;
                    int th = screenScale > 0 ? screenScale : 1;
                    Rect hr; SetRect(&hr, sc_cx - d, sc_cy - (th/2), sc_cx + d, sc_cy + (th/2) + 1); FrameRect(&hr);
                    Rect vr; SetRect(&vr, sc_cx - (th/2), sc_cy - d, sc_cx + (th/2) + 1, sc_cy + d); FrameRect(&vr);
                }

                /* Now compute the runs-based/QD centroid (only valid with QD active) */
                int centroid_ok = 0;
                long long qd_area2 = 0;
                int qd_ok = compute_intersection_centroid_ordered_qd_fixed(model, f1, f2, &ccx, &ccy, &qd_area2);
                if (qd_ok) {
                    iarea1 = (double)qd_area2;
                    /* determine ordering via distances (no console output here) */
                    float d1=0.0f,d2=0.0f;
                    if (ray_cast_distances(model, f1, f2, ccx, ccy, &d1, &d2)) {
                        if (d1 < d2) cmp = -1; else if (d1 > d2) cmp = 1; else cmp = 0;
                    } else {
                        cmp = 0;
                    }
                    centroid_ok = 1; cx = ccx; cy = ccy; point_source = 1;
                } else {
                    /* QD centroid not available despite region_ok (rare). Leave fallback visuals as-is. */
                    MoveTo(1,10);printf("QD centroid: KO\n");
                    if (has_bbox) { cx = crx; cy = cry; point_source = 2; }
                }
            }
            else {
                /* QD not available — draw fallback bbox if present */
                if (has_bbox) {
                    int sc_ix0 = screenScale * (saved_ix0 + pan_dx);
                    int sc_ix1 = screenScale * (saved_ix1 + pan_dx);
                    int sc_iy0 = (saved_iy0 + pan_dy);
                    int sc_iy1 = (saved_iy1 + pan_dy);
                    SetSolidPenPat(15); /* white for fallback bbox (historic) */
                    Rect r; SetRect(&r, sc_ix0, sc_iy0, sc_ix1, sc_iy1); FrameRect(&r);
                    /* draw white cross at center of fallback bbox */
                    int fcx = (saved_ix0 + saved_ix1) / 2; int fcy = (saved_iy0 + saved_iy1) / 2;
                    int sc_cx = screenScale * (fcx + pan_dx);
                    int sc_cy = (fcy + pan_dy);
                    int d = 4 * screenScale;
                    int th = screenScale > 0 ? screenScale : 1;
                    Rect hr; SetRect(&hr, sc_cx - d, sc_cy - (th/2), sc_cx + d, sc_cy + (th/2) + 1); FrameRect(&hr);
                    Rect vr; SetRect(&vr, sc_cx - (th/2), sc_cy - d, sc_cx + (th/2) + 1, sc_cy + d); FrameRect(&vr);
                } else {
                    MoveTo(3, 10);
                    printf("Region bbox: undetermined\n");
                }
        } /* end if (!no_overlap) */
            }
        }

    /* Minimal textual status (single short line) shown in graphics mode */
    MoveTo(3, 190);
    if (no_overlap) {
        printf("Face %d and face %d do not overlap\n", f1, f2);
    } else if (point_source == 1) {
        if (cmp == -1) printf("QD bbox center: %d,%d; face %d front\n", cx, cy, f1);
        else if (cmp == 1) printf("QD bbox center: %d,%d; face %d front\n", cx, cy, f2);
        else printf("QD bbox center: %d,%d  undetermined\n", cx, cy);
    } else if (point_source == 2) {
        if (cmp == -1) printf("bbox center: %d,%d; face %d front\n", cx, cy, f1);
        else if (cmp == 1) printf("bbox center: %d,%d; face %d front\n", cx, cy, f2);
        else printf("bbox center: %d,%d; undetermined\n", cx, cy);
    } else {
        printf("bbox center: none; undetermined\n");
    }
    keypress();

}


segment "code04";
/* showFace
 * --------
 * Purpose:
 *  - Display a single face in filled mode (green) over the wireframe model.
 * Behavior:
 *  - Prompts user for a face id (0..face_count-1).
 *  - Draws the entire model in wireframe mode.
 *  - Overlays the selected face in filled green (pen 10).
 *  - Navigate: Left/Right arrows = prev/next face ID
 *              Up/Down arrows = next/prev position in sorted list
 *  - ESC to exit
 */
void showFace(Model3D* model, ObserverParams* params, const char* filename) {
    if (!model || !params) return;
    FaceArrays3D* faces = &model->faces;
    int face_count = faces->face_count;
    if (face_count <= 0) { printf("No faces in model\n"); return; }

    // Prompt user for face id
    printf("Enter face id (0..%d) to display: ", face_count - 1);
    int sel = -1;
    if (scanf("%d", &sel) != 1) {
        int ch; while ((ch = getchar()) != '\n' && ch != EOF) ;
        printf("Input cancelled\n");
        return;
    }
    // consume remaining chars on the line
    {
        int ch; while ((ch = getchar()) != '\n' && ch != EOF) ;
    }
    
    int target_face = sel;
    if (target_face < 0 || target_face >= face_count) {
        printf("Invalid face id\n");
        return;
    }

    printf("=> Face %d\n\n", target_face);
    printf("Use arrow keys to navigate (Left/Right: face ID, Up/Down: sorted list)\n");
    printf("Press SPACE to show detailed info about the selected face.\n");
    printf("Use any other key to exit.\n\n");

    printf("Press any key to show model...\n");
    keypress();
    
    // Backup display flags
    unsigned char* backup_flags = (unsigned char*)malloc(faces->face_count);
    for (int i = 0; i < faces->face_count; ++i) backup_flags[i] = faces->display_flag[i];

    int old_frame = framePolyOnly;
    framePolyOnly = 1; // wireframe mode
    
    // Navigation loop
    int quit = 0;
    while (!quit) {
        startgraph(mode);
        
        // Draw entire model in wireframe
        if (jitter) drawPolygons_jitter(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count); else drawPolygons(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count);

        // Overlay selected face in filled green (pen 10)
        faces->display_flag[target_face] = 1;
        drawFace(model, target_face, 10, 1);

        // Find position in sorted list for display
        int pos_in_sorted = -1;
        for (int i = 0; i < face_count; ++i) {
            if (faces->sorted_face_indices[i] == target_face) {
                pos_in_sorted = i;
                break;
            }
        }

        MoveTo(2, 195);
        // Orientation: front vs back (observer-space d > 0 => front)
        if (pos_in_sorted >= 0) {
            printf("Face %d (sorted pos %d) [%s]", target_face, pos_in_sorted, (faces->plane_d[target_face] > 0) ? "FRONT" : "BACK");
        } else {
            printf("Face %d [%s]", target_face, (faces->plane_d[target_face] > 0) ? "FRONT" : "BACK");
        }
        
        // Wait for key
        int key = 0;
        asm {
            sep #0x20
        waitkey:
            lda >0xC000
            bpl waitkey
            and #0x007f
            sta >0xC010
            sta key
            rep #0x30
        }
        
        endgraph();
        
        // Process key - only arrow keys continue, any other key exits
        if (key == 8) { // Left arrow - previous face ID
            target_face--;
            if (target_face < 0) target_face = face_count - 1;
        } else if (key == 21) { // Right arrow - next face ID
            target_face++;
            if (target_face >= face_count) target_face = 0;
        } else if (key == 11) { // Up arrow - next position in sorted list
            if (pos_in_sorted >= 0 && pos_in_sorted < face_count - 1) {
                target_face = faces->sorted_face_indices[pos_in_sorted + 1];
            }
        } else if (key == 10) { // Down arrow - previous position in sorted list
            if (pos_in_sorted > 0) {
                target_face = faces->sorted_face_indices[pos_in_sorted - 1];
            }
        } else if (key == 32) { // Space - show textual details about the face
            // Switch to text mode and print detailed info, then return to graphics on keypress
            DoText();
            int vn = faces->vertex_count[target_face];
            printf("\n=== Face detail (ID=%d) ===\n\n", target_face);
            printf("Orientation: %s\n", (faces->plane_d[target_face] > 0) ? "FRONT" : "BACK");
            // Plane equation (float)
            {
                float a = (float)FIXED64_TO_FLOAT(faces->plane_a[target_face]);
                float b = (float)FIXED64_TO_FLOAT(faces->plane_b[target_face]);
                float c = (float)FIXED64_TO_FLOAT(faces->plane_c[target_face]);
                float d = (float)FIXED64_TO_FLOAT(faces->plane_d[target_face]);
                printf("Plane equation: a=%f b=%f c=%f d=%f\n", a, b, c, d);
            }
            printf("Z min: %.6f   ;   ", FIXED_TO_FLOAT(faces->z_min[target_face]));
            printf("Z mean: %.6f   ;   ", FIXED_TO_FLOAT(faces->z_mean[target_face]));
            printf("Z max: %.6f\n\n", FIXED_TO_FLOAT(faces->z_max[target_face]));
            if (pos_in_sorted >= 0) printf("Position in sorted list: %d\n", pos_in_sorted);

            printf("Vertex count: %d\n\n", vn);
            int offt = faces->vertex_indices_ptr[target_face];

            for (int k = 0; k < vn; ++k) {
                int vid = faces->vertex_indices_buffer[offt + k] - 1;
                printf("vertex[%d] idx=%d model=(%f,%f,%f) obs=(%f,%f,%f) x2d=%d y2d=%d\n",
                       k, vid,
                       FIXED_TO_FLOAT(model->vertices.x[vid]), FIXED_TO_FLOAT(model->vertices.y[vid]), FIXED_TO_FLOAT(model->vertices.z[vid]),
                       FIXED_TO_FLOAT(model->vertices.xo[vid]), FIXED_TO_FLOAT(model->vertices.yo[vid]), FIXED_TO_FLOAT(model->vertices.zo[vid]),
                       model->vertices.x2d[vid], model->vertices.y2d[vid]);
            }

            printf("\nPress 'F' to save to file Face%d.txt, any other key to return to graphics...\n", target_face);
            fflush(stdout);
            int tkey = 0;
            asm {
                sep #0x20
            waitkey2:
                lda >0xC000
                bpl waitkey2
                and #0x007f
                sta >0xC010
                sta tkey
                rep #0x30
            }
            if (tkey == 'F' || tkey == 'f') {
                char fname[64]; sprintf(fname, "Face%d.txt", target_face);
                FILE *out = fopen(fname, "w");
                if (out) {
                    fprintf(out, "Face %d\n", target_face);
                    if (pos_in_sorted >= 0) fprintf(out, "Position in sorted list: %d\n", pos_in_sorted);
                    fprintf(out, "Vertex count: %d\n", vn);
                    for (int k = 0; k < vn; ++k) {
                        int vid = faces->vertex_indices_buffer[offt + k] - 1;
                        fprintf(out, "v[%d] idx=%d model=(%f,%f,%f) obs=(%f,%f,%f) x2d=%d y2d=%d\n",
                                k, vid,
                                FIXED_TO_FLOAT(model->vertices.x[vid]), FIXED_TO_FLOAT(model->vertices.y[vid]), FIXED_TO_FLOAT(model->vertices.z[vid]),
                                FIXED_TO_FLOAT(model->vertices.xo[vid]), FIXED_TO_FLOAT(model->vertices.yo[vid]), FIXED_TO_FLOAT(model->vertices.zo[vid]),
                                model->vertices.x2d[vid], model->vertices.y2d[vid]);
                    }
                    float a = (float)FIXED64_TO_FLOAT(faces->plane_a[target_face]);
                    float b = (float)FIXED64_TO_FLOAT(faces->plane_b[target_face]);
                    float c = (float)FIXED64_TO_FLOAT(faces->plane_c[target_face]);
                    float d = (float)FIXED64_TO_FLOAT(faces->plane_d[target_face]);
                    fprintf(out, "Plane equation: a=%f b=%f c=%f d=%f\n", a, b, c, d);
                    fprintf(out, "Orientation: %s\n", (faces->plane_d[target_face] > 0) ? "FRONT" : "BACK");
                    fprintf(out, "Z min: %.6f\n", FIXED_TO_FLOAT(faces->z_min[target_face]));
                    fprintf(out, "Z mean: %.6f\n", FIXED_TO_FLOAT(faces->z_mean[target_face]));
                    fprintf(out, "Z max: %.6f\n", FIXED_TO_FLOAT(faces->z_max[target_face]));
                    fclose(out);
                    printf("Saved to %s\n", fname); fflush(stdout);
                } else {
                    printf("Error: unable to open %s for writing\n", fname); fflush(stdout);
                }
                printf("Press any key to return to graphics...\n"); fflush(stdout);
                int tmpk = 0;
                asm {
                    sep #0x20
                waitkey3:
                    lda >0xC000
                    bpl waitkey3
                    and #0x007f
                    sta >0xC010
                    sta tmpk
                    rep #0x30
                }
                // Return to graphics (loop will redraw)
                continue;
            } else {
                // Any other key returns to graphics
                continue;
            }
        } else {
            // Any other key exits
            quit = 1;
        }
    }
    
    DoText();

    // Restore state
    framePolyOnly = old_frame;
    for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = backup_flags[i];
    free(backup_flags);
}

/* inspect_faces_before
 * --------------------
 * Purpose:
 *  - Interactively inspect faces that are placed BEFORE a selected face in the
 *    painter's ordered list but that the pairwise tests conclude should be AFTER.
 * Behavior:
 *  - Prompts the user for a face id, locates it in `sorted_face_indices` and checks
 *    faces earlier in the ordered list using `pair_order_relation` and `evaluate_pair_tests`.
 *  - Prints a compact per-face diagnostic and offers a wireframe preview with the
 *    selected face highlighted (green) and misplaced faces highlighted (orange).
 */
void inspect_faces_before(Model3D* model, ObserverParams* params, const char* filename) {
    if (!model || !params) return;
    FaceArrays3D* faces = &model->faces;
    int face_count = faces->face_count;
    if (face_count <= 0) { printf("No faces in model\n"); return; }

    // Ensure back-face culling on and recompute depths & ordering
    // helper: enable culling and run painter sort; returns previous culling state
    int old_cull = cull_back_faces;
    cull_back_faces = 1;

    // Prompt user for face id
    printf("Enter face id (0..%d) to inspect: ", face_count - 1);
    int sel = -1;
    if (scanf("%d", &sel) != 1) {
        int ch; while ((ch = getchar()) != '\n' && ch != EOF) ;
        printf("Input cancelled\n");
        cull_back_faces = old_cull; return;
    }
    // consume remaining chars on the line (newline) to avoid disturbing later fgets()
    {
        int ch; while ((ch = getchar()) != '\n' && ch != EOF) ;
    }
    // Use the exact number provided by the user (do not convert 1-based to 0-based)
    int target_face = sel;
    if (target_face < 0 || target_face >= face_count) { printf("Invalid face id\n"); cull_back_faces = old_cull; return; }

    // Find position of target_face in the sorted array
    int pos = -1;
    for (int i = 0; i < face_count; ++i) {
        if (faces->sorted_face_indices[i] == target_face) { pos = i; break; }
    }
    if (pos < 0) { printf("Selected face is not in sorted list\n"); cull_back_faces = old_cull; return; }

    // Inform user about target position
    printf("Selected face %d is at position %d in the ordered list (total faces = %d)\n", target_face, pos, face_count);

    // Ensure variables are visible throughout the function (used later for preview)
    int* misplaced = NULL;
    int misplaced_count = 0;

    // Check faces placed before the selected face (only consider visible faces)
    misplaced = (int*)malloc(face_count * sizeof(int));
    misplaced_count = 0;
    int checked = 0, rel_neg = 0, rel_pos = 0, rel_zero = 0, bbox_skipped = 0;
    for (int i = 0; i < pos; ++i) {
        int f = faces->sorted_face_indices[i];
        // Bounding-box quick rejection: if separated on X or Y, skip - no overlap
        if (faces->maxx[f] <= faces->minx[target_face] || faces->maxx[target_face] <= faces->minx[f]
            || faces->maxy[f] <= faces->miny[target_face] || faces->maxy[target_face] <= faces->miny[f]) {
            bbox_skipped++; continue;
        }
        checked++;
        int rel = pair_order_relation(model, f, target_face);
        if (rel == -1) rel_neg++; else if (rel == 1) rel_pos++; else rel_zero++;
        if (rel == 1) { // this face should be after target -> misplaced
            misplaced[misplaced_count++] = f;
        }
    }

    printf("%d \"before\" faces (total): checked=%d ; bbox_skipped=%d ; should_be_before(-1): %d ;  should_be_after(+1): %d (misplaced: %d) ; inconclusive: %d\n",
           pos, checked, bbox_skipped, rel_neg, rel_pos, misplaced_count, rel_zero);

    printf("%d misplaced faces relative to face %d : ", misplaced_count, target_face);
    for (int i = 0; i < misplaced_count; ++i) {
        int f = misplaced[i];
        // Find position of the misplaced face in the sorted list (1-based for readability)
        int pos_in_sorted = -1;
        for (int si = 0; si < faces->face_count; ++si) {
            if (faces->sorted_face_indices[si] == f) { pos_in_sorted = si; break; }
        }
        if (pos_in_sorted >= 0) printf("%d (pos=%d)", f, pos_in_sorted + 1);
        else printf("%d (pos=?)", f);
        if (i < misplaced_count - 1) printf(",");
    }
    /* Report overlapping subset among misplaced faces (global summary) */
    int overlap_count = 0;
    int *overlaps = NULL;
    if (misplaced_count > 0) {
        overlaps = (int*)malloc(sizeof(int) * misplaced_count);
        for (int i = 0; i < misplaced_count; ++i) {
            int f = misplaced[i];
            if (projected_polygons_overlap(model, f, target_face)) overlaps[overlap_count++] = f;
        }
    }
    printf("\n");
    if (overlap_count > 0) {
        printf("Overlap with target: %d face(s):", overlap_count);
        for (int i = 0; i < overlap_count; ++i) printf(" %d", overlaps[i]);
        printf("\n");
    } else {
        printf("Overlap with target: none\n");
    }
    // Note: overlaps will be freed at the end of the function

    printf("\n");
    keypress();

    // For each misplaced face, evaluate tests 1..7 and display which succeeded/failed/inconclusive
    for (int mi = 0; mi < misplaced_count; ++mi) {
        int f = misplaced[mi];
        int tests[7];
        evaluate_pair_tests(model, f, target_face, tests);

        // Build readable lists
        char passed[128]; passed[0] = '\0';
        char failed[128]; failed[0] = '\0';
        char incon[128]; incon[0] = '\0';
        int pcount = 0, fcount = 0, icount = 0;
        for (int t = 0; t < 7; ++t) {
            if (tests[t] == 1) {
                if (pcount) strncat(passed, ",", sizeof(passed)-strlen(passed)-1);
                char tmp[8]; snprintf(tmp, sizeof(tmp), "%d", t+1); strncat(passed, tmp, sizeof(passed)-strlen(passed)-1); pcount++;
            } else if (tests[t] == -1) {
                if (fcount) strncat(failed, ",", sizeof(failed)-strlen(failed)-1);
                char tmp[8]; snprintf(tmp, sizeof(tmp), "%d", t+1); strncat(failed, tmp, sizeof(failed)-strlen(failed)-1); fcount++;
            } else {
                if (icount) strncat(incon, ",", sizeof(incon)-strlen(incon)-1);
                char tmp[8]; snprintf(tmp, sizeof(tmp), "%d", t+1); strncat(incon, tmp, sizeof(incon)-strlen(incon)-1); icount++;
            }
        }
        if (pcount == 0) strncpy(passed, "(none)", sizeof(passed));
        if (fcount == 0) strncpy(failed, "(none)", sizeof(failed));
        if (icount == 0) strncpy(incon, "(none)", sizeof(incon));

        char after_list[64]; after_list[0] = '\0';
        char before_list[64]; before_list[0] = '\0';
        char incon_list[64]; incon_list[0] = '\0';
        int after_count = 0, before_count = 0, incon_count = 0;
        for (int t = 0; t < 7; ++t) {
            if (tests[t] == 1) {
                if (after_count) strncat(after_list, ",", sizeof(after_list)-strlen(after_list)-1);
                char tmp[8]; snprintf(tmp, sizeof(tmp), "%d", t+1); strncat(after_list, tmp, sizeof(after_list)-strlen(after_list)-1);
                after_count++; 
            } else if (tests[t] == -1) {
                if (before_count) strncat(before_list, ",", sizeof(before_list)-strlen(before_list)-1);
                char tmp[8]; snprintf(tmp, sizeof(tmp), "%d", t+1); strncat(before_list, tmp, sizeof(before_list)-strlen(before_list)-1);
                before_count++;
            } else {
                if (incon_count) strncat(incon_list, ",", sizeof(incon_list)-strlen(incon_list)-1);
                char tmp[8]; snprintf(tmp, sizeof(tmp), "%d", t+1); strncat(incon_list, tmp, sizeof(incon_list)-strlen(incon_list)-1);
                incon_count++;
            }
        }

        // Compact grouped output (single-line)
        if (before_count == 0) strncpy(before_list, "(none)", sizeof(before_list));
        if (after_count == 0) strncpy(after_list, "(none)", sizeof(after_list));
        if (incon_count == 0) strncpy(incon_list, "(none)", sizeof(incon_list));

        printf("Face %d: BEFORE tests: %s; AFTER tests: %s; inconclusive: %s\n", f, before_list, after_list, incon_list);
        
        // Conclusion lines (keep existing style)
        if (after_count == 0 && before_count == 0) {
            printf("  Overall: inconclusive (no decisive tests)\n");
        } else {
            if (after_count) printf("Concluded: Face %d (selected) should be AFTER face %d (tests: %s)\n\n", target_face, f, after_list);
            if (before_count) printf("Concluded: Face %d (selected) should be BEFORE face %d (tests: %s)\n\n", target_face, f, before_list);
        }
    }

    printf("\nPress a key to preview model with highlight selected and misplaced faces (if any).\n");
    keypress();
    startgraph(mode);

    // Backup display flags
    unsigned char* backup_flags = (unsigned char*)malloc(faces->face_count);
    for (int i = 0; i < faces->face_count; ++i) backup_flags[i] = faces->display_flag[i];

    // 1) Show entire model in wireframe
    int old_frame = framePolyOnly;
    framePolyOnly = 1; // wireframe
    // Wireframe preview: handled via the `framePolyOnly` flag and `drawPolygons()` (no separate `processModelWireframe()` function)
    if (jitter) drawPolygons_jitter(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count); else drawPolygons(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count);


    // 2) Overlay misplaced faces in orange (pen 6)
    for (int i = 0; i < misplaced_count; ++i) {
        int f = misplaced[i];
        faces->display_flag[f] = 1;
        drawFace(model, f, 6, 0);
    }

    // 3) Overlay: selected face in green (pen 10)
    // Ensure target face is visible for drawFace
    faces->display_flag[target_face] = 1;
    drawFace(model, target_face, 10, 1);

    MoveTo(2, 195);
    printf("%d face(s) should be after face %d, %d overlap%s\n", misplaced_count, target_face, overlap_count, overlap_count > 1 ? "s" : "");
    keypress();
    endgraph();
    DoText();

    // Restore state
    framePolyOnly = old_frame;
    for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = backup_flags[i];
    free(backup_flags);

    // If there are misplaced faces, offer to fix by moving target before the smallest index
    if (misplaced_count > 0) {
        printf("\nPress 'A' to move face %d considering ALL misplaced faces, 'O' for overlaps only, or any other key to skip: ", target_face);
        int key = 0;
        asm {
            sep #0x20
        waitkey:
            lda >0xC000
            bpl waitkey
            and #0x007f
            sta >0xC010
            sta key
            rep #0x30
        }
        
        if (key == 'A' || key == 'a' || key == 'O' || key == 'o') {
            int min_pos = face_count; // start with max possible
            int target_to_move_before = -1;
            int use_overlaps = (key == 'O' || key == 'o');
            
            // Determine which list to use and check if available
            if (use_overlaps && overlap_count == 0) {
                printf("\nNo overlaps to process.\n");
            } else {
                // Find the smallest position among selected faces
                int count_to_check = use_overlaps ? overlap_count : misplaced_count;
                int *faces_to_check = use_overlaps ? overlaps : misplaced;
                
                for (int i = 0; i < count_to_check; ++i) {
                    int f = faces_to_check[i];
                    // Find position in sorted list
                    for (int si = 0; si < face_count; ++si) {
                        if (faces->sorted_face_indices[si] == f) {
                            if (si < min_pos) {
                                min_pos = si;
                                target_to_move_before = f;
                            }
                            break;
                        }
                    }
                }
                
                if (target_to_move_before >= 0 && min_pos < face_count) {
                    printf("\nMoving face %d (currently at pos %d) to position %d (before face %d) - mode: %s...\n", 
                           target_face, pos, min_pos, target_to_move_before, use_overlaps ? "overlaps" : "all");
                    
                    // Move target_face to min_pos (before the misplaced face with smallest index)
                    move_element_remove_and_insert(faces->sorted_face_indices, face_count, pos, min_pos);
                    
                    printf("Done! Face %d has been moved.\n", target_face);
                    printf("Press any key to continue...\n");
                    keypress();
                } else {
                    printf("\nError: could not determine target position.\n");
                }
            }
        } else {
            printf("\nSkipped.\n");
        }
    }

    if (overlaps) free(overlaps);
    free(misplaced);
    cull_back_faces = old_cull;
}

segment "code05";
/* inspect_faces_after
 * -------------------
 * Purpose:
 *  - Interactively inspect faces that are placed AFTER a selected face in the
 *    painter's ordered list but that the pairwise tests conclude should be BEFORE.
 * Behavior:
 *  - Prompts the user for a face id, locates it in `sorted_face_indices` and checks
 *    visible faces later in the ordered list using `pair_order_relation` and `evaluate_pair_tests`.
 *  - Prints a compact per-face diagnostic and offers a wireframe preview with the
 *    selected face highlighted (green) and misplaced faces highlighted (pink).
 */
void inspect_faces_after(Model3D* model, ObserverParams* params, const char* filename) {
    if (!model || !params) return;
    FaceArrays3D* faces = &model->faces;
    int face_count = faces->face_count;
    if (face_count <= 0) { printf("No faces in model\n"); return; }

    int old_cull = cull_back_faces;
    cull_back_faces = 1;

    printf("Enter face id (0..%d) to inspect AFTER-list: ", face_count - 1);
    int sel = -1;

    if (scanf("%d", &sel) != 1) {
        int ch; while ((ch = getchar()) != '\n' && ch != EOF) ;
        printf("Input cancelled\n");
        cull_back_faces = old_cull; return;
    }
    // consume remaining chars on the line (newline) to avoid disturbing later fgets()
    {
        int ch; while ((ch = getchar()) != '\n' && ch != EOF) ;
    }
    int target_face = sel;
    if (target_face < 0 || target_face >= face_count) { printf("Invalid face id\n"); cull_back_faces = old_cull; return; }

    int pos = -1;
    for (int i = 0; i < face_count; ++i) if (faces->sorted_face_indices[i] == target_face) { pos = i; break; }
    if (pos < 0) { printf("Selected face is not in sorted list\n"); cull_back_faces = old_cull; return; }

    int* misplaced = (int*)malloc(face_count * sizeof(int)); int misplaced_count = 0;
    int checked = 0, rel_neg = 0, rel_pos = 0, rel_zero = 0, bbox_skipped = 0;
    for (int i = pos + 1; i < face_count; ++i) {
        int f = faces->sorted_face_indices[i];
        // Bounding-box quick rejection: if separated on X or Y, skip - no overlap
        if (faces->maxx[f] <= faces->minx[target_face] || faces->maxx[target_face] <= faces->minx[f]
            || faces->maxy[f] <= faces->miny[target_face] || faces->maxy[target_face] <= faces->miny[f]) {
            bbox_skipped++; continue;
        }
        int rel = pair_order_relation(model, f, target_face); // correct parameter order
        checked++;
        if (rel == -1) rel_neg++; else if (rel == 1) rel_pos++; else rel_zero++;
        if (rel == -1) { // f should be before target -> misplaced
            misplaced[misplaced_count++] = f;
        }
    }

    printf("%d \"before\" faces (total): checked=%d ; bbox_skipped=%d ; should_be_before(-1): %d ;  should_be_after(+1): %d (misplaced: %d) ; inconclusive: %d\n",
           pos, checked, bbox_skipped, rel_neg, rel_pos, misplaced_count, rel_zero);

    printf("%d misplaced faces relative to face %d : ", misplaced_count, target_face);
    for (int i = 0; i < misplaced_count; ++i) {
        int f = misplaced[i];
        int pos_in_sorted = -1;
        for (int si = 0; si < faces->face_count; ++si) {
            if (faces->sorted_face_indices[si] == f) { pos_in_sorted = si; break; }
        }
        if (pos_in_sorted >= 0) printf("%d (pos=%d)", f, pos_in_sorted + 1);
        else printf("%d (pos=?)", f);
        if (i < misplaced_count - 1) printf(",");
    }
    printf("\n");

    /* Report overlapping subset among the misplaced list (global summary) */
    int overlap_count = 0;
    int *overlaps = NULL;
    if (misplaced_count > 0) {
        overlaps = (int*)malloc(sizeof(int) * misplaced_count);
        for (int i = 0; i < misplaced_count; ++i) {
            int f = misplaced[i];
            if (projected_polygons_overlap(model, f, target_face)) overlaps[overlap_count++] = f;
        }
    }
    if (overlap_count > 0) {
        printf("Overlap with target: %d face(s):", overlap_count);
        for (int i = 0; i < overlap_count; ++i) printf(" %d", overlaps[i]);
        printf("\n");
    } else {
        printf("Overlap with target: none\n");
    }
    // Note: overlaps will be freed at the end of the function
    printf("\n");
    keypress();

    for (int ai = 0; ai < misplaced_count; ++ai) {
        int f = misplaced[ai];
        // Quick bbox rejection: if separated on X or Y, skip detailed tests
        if (faces->maxx[f] <= faces->minx[target_face] || faces->maxx[target_face] <= faces->minx[f]
            || faces->maxy[f] <= faces->miny[target_face] || faces->maxy[target_face] <= faces->miny[f]) {
            printf("Face %d: skipped (axis-aligned bbox separation)\n", f);
            continue;
        }

        int tests[7]; evaluate_pair_tests(model, f, target_face, tests);

        char passed_t[128]; passed_t[0] = '\0';
        char failed_t[128]; failed_t[0] = '\0';
        char incon_t[128]; incon_t[0] = '\0';
        int pcount_t = 0, fcount_t = 0, icount_t = 0;
        for (int t = 0; t < 7; ++t) {
            if (tests[t] == 1) { if (pcount_t) strncat(passed_t, ",", sizeof(passed_t)-strlen(passed_t)-1); char tmp[8]; snprintf(tmp, sizeof(tmp), "%d", t+1); strncat(passed_t, tmp, sizeof(passed_t)-strlen(passed_t)-1); pcount_t++; }
            else if (tests[t] == -1) { if (fcount_t) strncat(failed_t, ",", sizeof(failed_t)-strlen(failed_t)-1); char tmp[8]; snprintf(tmp, sizeof(tmp), "%d", t+1); strncat(failed_t, tmp, sizeof(failed_t)-strlen(failed_t)-1); fcount_t++; }
            else { if (icount_t) strncat(incon_t, ",", sizeof(incon_t)-strlen(incon_t)-1); char tmp[8]; snprintf(tmp, sizeof(tmp), "%d", t+1); strncat(incon_t, tmp, sizeof(incon_t)-strlen(incon_t)-1); icount_t++; }
        }
        if (pcount_t == 0) strncpy(passed_t, "(none)", sizeof(passed_t));
        if (fcount_t == 0) strncpy(failed_t, "(none)", sizeof(failed_t));
        if (icount_t == 0) strncpy(incon_t, "(none)", sizeof(incon_t));

        char after_list_str[64]; after_list_str[0] = '\0';
        char before_list_str[64]; before_list_str[0] = '\0';
        char incon_list_str[64]; incon_list_str[0] = '\0';
        int after_count_t = 0, before_count_t = 0, incon_count_t = 0;
        for (int t = 0; t < 7; ++t) {
            if (tests[t] == 1) { if (after_count_t) strncat(after_list_str, ",", sizeof(after_list_str)-strlen(after_list_str)-1); char tmp[8]; snprintf(tmp, sizeof(tmp), "%d", t+1); strncat(after_list_str, tmp, sizeof(after_list_str)-strlen(after_list_str)-1); after_count_t++; }
            else if (tests[t] == -1) { if (before_count_t) strncat(before_list_str, ",", sizeof(before_list_str)-strlen(before_list_str)-1); char tmp[8]; snprintf(tmp, sizeof(tmp), "%d", t+1); strncat(before_list_str, tmp, sizeof(before_list_str)-strlen(before_list_str)-1); before_count_t++; }
            else { if (incon_count_t) strncat(incon_list_str, ",", sizeof(incon_list_str)-strlen(incon_list_str)-1); char tmp[8]; snprintf(tmp, sizeof(tmp), "%d", t+1); strncat(incon_list_str, tmp, sizeof(incon_list_str)-strlen(incon_list_str)-1); incon_count_t++; }
        }
        if (before_count_t == 0) strncpy(before_list_str, "(none)", sizeof(before_list_str));
        if (after_count_t == 0) strncpy(after_list_str, "(none)", sizeof(after_list_str));
        if (incon_count_t == 0) strncpy(incon_list_str, "(none)", sizeof(incon_list_str));

        printf("Face %d: BEFORE tests: %s; AFTER tests: %s; inconclusive: %s\n", f, before_list_str, after_list_str, incon_list_str);
        if (after_count_t == 0 && before_count_t == 0) {
            printf("  Overall: inconclusive (no decisive tests)\n");
        } else {
            if (before_count_t) printf("Concluded: Face %d should be BEFORE face %d (tests: %s)\n\n", f, target_face, before_list_str);
            if (after_count_t) printf("Concluded: Face %d should be AFTER face %d (tests: %s)\n\n", f, target_face, after_list_str);
        }
    }

    // Offer a preview (always, as in inspect_faces_before)
    printf("\nPress a key to preview model with highlight selected and misplaced faces (if any).\n");
    keypress();
    startgraph(mode);

    unsigned char* backup_flags = (unsigned char*)malloc(faces->face_count);
    for (int i = 0; i < faces->face_count; ++i) backup_flags[i] = faces->display_flag[i];

    int old_frame = framePolyOnly;
    framePolyOnly = 1; // wireframe
    // Wireframe preview: handled via the `framePolyOnly` flag and `drawPolygons()` (no separate `processModelWireframe()` function)
    if (jitter) drawPolygons_jitter(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count); else drawPolygons(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count);

    for (int i = 0; i < misplaced_count; ++i) {
        int f = misplaced[i];
        faces->display_flag[f] = 1;
        drawFace(model, f, 12, 0);
    }
    faces->display_flag[target_face] = 1;
    drawFace(model, target_face, 10, 1);


    MoveTo(2, 195);
    printf("%d face(s) should be before face %d, %d overlap%s\n", misplaced_count, target_face, overlap_count, overlap_count > 1 ? "s" : "");
    keypress();
    endgraph();
    DoText();

    framePolyOnly = old_frame;
    for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = backup_flags[i];
    free(backup_flags);

    // If there are misplaced faces, offer to fix by moving target after the largest index
    if (misplaced_count > 0) {
        printf("\nPress 'A' to move face %d considering ALL misplaced faces, 'O' for overlaps only, or any other key to skip: ", target_face);
        int key = 0;
        asm {
            sep #0x20
        waitkey:
            lda >0xC000
            bpl waitkey
            and #0x007f
            sta >0xC010
            sta key
            rep #0x30
        }
        
        if (key == 'A' || key == 'a' || key == 'O' || key == 'o') {
            int max_pos = -1;
            int target_to_move_after = -1;
            int use_overlaps = (key == 'O' || key == 'o');
            
            // Determine which list to use and check if available
            if (use_overlaps && overlap_count == 0) {
                printf("\nNo overlaps to process.\n");
            } else {
                // Find the largest position among selected faces
                int count_to_check = use_overlaps ? overlap_count : misplaced_count;
                int *faces_to_check = use_overlaps ? overlaps : misplaced;
                
                for (int i = 0; i < count_to_check; ++i) {
                    int f = faces_to_check[i];
                    // Find position in sorted list
                    for (int si = 0; si < face_count; ++si) {
                        if (faces->sorted_face_indices[si] == f) {
                            if (si > max_pos) {
                                max_pos = si;
                                target_to_move_after = f;
                            }
                            break;
                        }
                    }
                }
                
                if (target_to_move_after >= 0 && max_pos >= 0) {
                    printf("\nMoving face %d (currently at pos %d) to position %d (after face %d) - mode: %s...\n", 
                           target_face, pos, max_pos, target_to_move_after, use_overlaps ? "overlaps" : "all");
                    
                    // Move target_face to max_pos (after the misplaced face with largest index)
                    move_element_remove_and_insert(faces->sorted_face_indices, face_count, pos, max_pos);
                    
                    printf("Done! Face %d has been moved.\n", target_face);
                    printf("Press any key to continue...\n");
                    keypress();
                } else {
                    printf("\nError: could not determine target position.\n");
                }
            }
        } else {
            printf("\nSkipped.\n");
        }
    }

    if (overlaps) free(overlaps);
    free(misplaced);
    cull_back_faces = old_cull;
}

segment "code23";
/* inspect_polygons_overlap
 * ------------------------
 * Interactive inspector for projected polygon overlap.
 * Behavior:
 *  - Prints a short explanation before prompting (touching = NON-overlap).
 *  - Prompts for face id 1 and face id 2 (0..face_count-1).
 *  - Calls `projected_polygons_overlap(model, f1, f2)` and prints YES/NO.
 *  - Prompts whether to show the faces on the model; **default** is YES when
 *    the user presses ENTER (empty line => show the model).
 *  - When showing, draws the entire model in wireframe, then overlays the two
 *    faces with colors and displays their indices (the overlay respects the
 *    painter's `sorted_face_indices` stacking order when possible).
 */
void inspect_polygons_overlap(Model3D* model, ObserverParams* params, const char* filename) {
    if (!model || !params) return;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int face_count = faces->face_count;
    if (face_count <= 0) { printf("No faces in model\n"); return; }

    // Inform the user what this inspector does before asking for input
    printf("Inspector: check whether two faces' 2D projections overlap (touching = NON-overlap).\n");

    // Prompt for face 1
    printf("Enter face id 1 (0..%d): ", face_count - 1);
    int f1 = -1;
    if (scanf("%d", &f1) != 1) { int ch; while ((ch = getchar()) != '\n' && ch != EOF); printf("Input cancelled\n"); return; }
    { int ch; while ((ch = getchar()) != '\n' && ch != EOF); }
    if (f1 < 0 || f1 >= face_count) { printf("Invalid face id 1\n"); return; }

    // Prompt for face 2
    printf("Enter face id 2 (0..%d): ", face_count - 1);
    int f2 = -1;
    if (scanf("%d", &f2) != 1) { int ch; while ((ch = getchar()) != '\n' && ch != EOF); printf("Input cancelled\n"); return; }
    { int ch; while ((ch = getchar()) != '\n' && ch != EOF); }
    if (f2 < 0 || f2 >= face_count) { printf("Invalid face id 2\n"); return; }

    // Report IDs and result
    int start_time = GetTick();
    int ov = projected_polygons_overlap(model, f1, f2);
    int end_time = GetTick();
    printf("Computed projected overlap in %d ticks\n", end_time - start_time);
    printf("Faces: %d and %d -> Projected overlap: %s\n", f1, f2, ov ? "YES" : "NO");

    /* Diagnostics and heavy tests removed for simplified inspector as requested. */
    (void)0;

    // Ask user whether to show faces on model (default = YES on empty input)
    char resp[16];
    printf("Show faces on model? (O/n) [ENTER = yes]: ");
    if (fgets(resp, sizeof(resp), stdin) == NULL) return;
    if (resp[0] == 'N' || resp[0] == 'n') return; // explicit no
    // anything else (including '\n' on empty line) -> show

    // Prepare view and display f1 (green) and f2 (orange)
    startgraph(mode);
    unsigned char* backup_flags = (unsigned char*)malloc(faces->face_count);
    for (int i = 0; i < faces->face_count; ++i) backup_flags[i] = faces->display_flag[i];

    int old_frame = framePolyOnly;
    framePolyOnly = 1; // not filled

    // Show the entire model in wireframe first, then overlay the two faces
    for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = 1; // ensure all faces visible for wireframe
    if (jitter) drawPolygons_jitter(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count); else drawPolygons(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count);

    // Overlay explicit colored faces (ensure visibility); force the two faces visible and draw indices
    unsigned char saved_f1 = faces->display_flag[f1]; unsigned char saved_f2 = faces->display_flag[f2];
    faces->display_flag[f1] = 1;
    faces->display_flag[f2] = 1;
    drawFace(model, f1, 10, 1); // green with index
    drawFace(model, f2, 6, 1);  // orange with index
    // restore (though we restore all flags later from backup)
    faces->display_flag[f1] = saved_f1; faces->display_flag[f2] = saved_f2;

    MoveTo(3, 185);
    printf("%d and %d overlap: %s\n", f1, f2, ov ? "YES" : "NO");
    printf("'F' to save overlap.csv, any key to return\n");

    /* Read hardware key directly (same technique used in main loop) */
    int inspector_key = 0;
    asm {
        sep #0x20
    readloop2:
        lda >0xC000
        bpl readloop2
        and #0x007f
        sta inspector_key
        sta >0xC010
        rep #0x30
    }

    if (inspector_key == 'F' || inspector_key == 'f') {
        FILE *of = fopen("overlap.csv", "w");
        if (of) {
            fprintf(of, "face1,%d,face2,%d,overlap,%s\n", f1, f2, ov ? "YES" : "NO");
            fprintf(of, "face_id,vertex_order,vertex_index,x2d,y2d\n");
            int off1 = faces->vertex_indices_ptr[f1];
            int n1 = faces->vertex_count[f1];
            for (int vi = 0; vi < n1; ++vi) {
                int idx = faces->vertex_indices_buffer[off1 + vi] - 1;
                if (idx >= 0 && idx < vtx->vertex_count) fprintf(of, "%d,%d,%d,%d,%d\n", f1, vi, idx, vtx->x2d[idx], vtx->y2d[idx]);
            }
            int off2 = faces->vertex_indices_ptr[f2];
            int n2 = faces->vertex_count[f2];
            for (int vi = 0; vi < n2; ++vi) {
                int idx = faces->vertex_indices_buffer[off2 + vi] - 1;
                if (idx >= 0 && idx < vtx->vertex_count) fprintf(of, "%d,%d,%d,%d,%d\n", f2, vi, idx, vtx->x2d[idx], vtx->y2d[idx]);
            }
            fclose(of);
            printf("Saved overlap.csv\n");
        } else {
            printf("Error: cannot open overlap.csv for writing\n");
        }
    }

    endgraph();
    DoText();

    // Restore state
    framePolyOnly = old_frame;
    for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = backup_flags[i];
    free(backup_flags);
}

/* End inspect_polygons_overlap */

/* Interactive tester: iterate all face pairs whose 2D bboxes intersect.
 * For each pair:
 *  - display the model in wireframe
 *  - highlight f1 in green, f2 in orange
 *  - print overlap status (YES/NO) with face numbers
 *  - wait for a key: any key -> next pair, ESC -> exit
 */
void test_all_overlap(Model3D* model, ObserverParams* params, const char* filename) {
    if (!model) { printf("No model loaded\n"); return; }
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;

    unsigned char* backup_flags = (unsigned char*)malloc(faces->face_count);
    for (int i = 0; i < faces->face_count; ++i) backup_flags[i] = faces->display_flag[i];

    int old_frame = framePolyOnly; /* keep global framePolyOnly unchanged; render wireframe explicitly when needed */
    framePolyOnly = 1; // wireframe

    /* Temporarily disable jitter during full overlap scan to ensure deterministic
     * behavior (calls to rand() in jitter rendering can change PRNG state and
     * affect repeated scans within the same session). Save and restore `jitter`. */
    int saved_jitter = jitter; 
    jitter = 0;

    /* Build list of candidate pairs (bbox intersection) */
    typedef struct { int a; int b; } Pair;
    int pair_count = 0;

    printf("Total overlapping pairs test\n");
    printf("Building pair list...\n");

    for (int i = 0; i < faces->face_count; ++i) for (int j = i+1; j < faces->face_count; ++j) {
        /* Respect back-face culling: when enabled, consider only pairs where both faces are front-facing */
        if (cull_back_faces) {
            if (faces->plane_d[i] <= 0 || faces->plane_d[j] <= 0) continue;
        }
        int minx1 = faces->minx[i], maxx1 = faces->maxx[i], miny1 = faces->miny[i], maxy1 = faces->maxy[i];
        int minx2 = faces->minx[j], maxx2 = faces->maxx[j], miny2 = faces->miny[j], maxy2 = faces->maxy[j];
        if (maxx1 <= minx2 || maxx2 <= minx1 || maxy1 <= miny2 || maxy2 <= miny1) continue;
        pair_count++;
    }
    if (pair_count == 0) { printf("No bbox-intersecting pairs found.\n"); 
        framePolyOnly = old_frame; free(backup_flags); return; }
    Pair* pairs = (Pair*)malloc(sizeof(Pair) * pair_count);
    int pi = 0;
    for (int i = 0; i < faces->face_count; ++i) for (int j = i+1; j < faces->face_count; ++j) {
        /* Respect back-face culling: when enabled, consider only pairs where both faces are front-facing */
        if (cull_back_faces) {
            if (faces->plane_d[i] <= 0 || faces->plane_d[j] <= 0) continue;
        }
        int minx1 = faces->minx[i], maxx1 = faces->maxx[i], miny1 = faces->miny[i], maxy1 = faces->maxy[i];
        int minx2 = faces->minx[j], maxx2 = faces->maxx[j], miny2 = faces->miny[j], maxy2 = faces->maxy[j];
        if (maxx1 <= minx2 || maxx2 <= minx1 || maxy1 <= miny2 || maxy2 <= miny1) continue;
        pairs[pi].a = i; pairs[pi].b = j; pi++;
    }

    /* Ask user for starting face id (instead of pair index). If the face appears
     * in any bbox-intersecting pair, start at the first such pair; otherwise start at first pair.
     */
    int start_idx = 0; /* default to first pair */
    printf("Start at face id (0..%d) or press Enter for first pair: ", faces->face_count - 1);
    char inbuf[64];
    if (fgets(inbuf, sizeof(inbuf), stdin) != NULL) {
        if (inbuf[0] != '\n') {
            int fid = -1;
            if (sscanf(inbuf, "%d", &fid) == 1 && fid >= 0 && fid < faces->face_count) {
                int found = -1;
                for (int k = 0; k < pair_count; ++k) {
                    if (pairs[k].a == fid || pairs[k].b == fid) { found = k; break; }
                }
                if (found >= 0) start_idx = found;
                else { printf("Face %d not part of any bbox-intersecting pair, starting at first pair\n", fid); start_idx = 0; }
            } else {
                printf("Invalid input, starting at first pair\n"); start_idx = 0;
            }
        }
    }

    printf("Arrows: navigate, space: cycle display mode (0: numbers+wireframe, 1: wireframe only, 2: neither, 3: numbers only), F: save, a: scan all\n");
    printf("Press any key to start...\n");
    keypress();

    /* display_state: 0 = numbers + wireframe (default)
     *                1 = wireframe only
     *                2 = neither numbers nor wireframe
     *                3 = numbers only
     */
    int display_state = 0;
    int idx = start_idx;
    while (1) {
        int f1 = pairs[idx].a; int f2 = pairs[idx].b;

        startgraph(mode);

        /* Set all faces visible for consistent highlights */
        for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = 1;

        /* Render model according to display_state:
         * - states 0 and 1: show wireframe outlines (drawFace with fillPenPat = -1)
         * - states 2 and 3: do NOT render the model at all (neither wireframe nor filled)
         */
        if (display_state == 0 || display_state == 1) {
            for (int fi = 0; fi < faces->face_count; ++fi) {
                if (!faces->display_flag[fi]) continue;
                drawFace(model, fi, -1, 0); /* wireframe outline */
            }
        } else {
            /* Intentionally skip drawing the model for states 2 and 3 */
        }

        /* Determine whether to show face ids for highlights */
        int show_face_ids = (display_state == 0 || display_state == 3) ? 1 : 0;

        /* Highlight faces */
        unsigned char saved_f1 = faces->display_flag[f1]; unsigned char saved_f2 = faces->display_flag[f2];
        faces->display_flag[f1] = 1; faces->display_flag[f2] = 1;
        drawFace(model, f1, 10, show_face_ids); // green
        drawFace(model, f2, 6, show_face_ids);  // orange
        faces->display_flag[f1] = saved_f1; faces->display_flag[f2] = saved_f2;

        /* Compute overlap using existing test */
        int ov = projected_polygons_overlap(model, f1, f2);

        MoveTo(3, 195);
        printf("Pair %d/%d: %d vs %d overlap: %s\n", idx+1, pair_count, f1, f2, ov ? "YES" : "NO");
        

        /* Read hardware key */
        int key = 0;
        asm {
            sep #0x20
        readkeyloop:
            lda >0xC000
            bpl readkeyloop
            and #0x007f
            sta key
            sta >0xC010
            rep #0x30
        }

        if (key == 8 || key == 11) { /* Left or Up -> previous */
            idx = (idx - 1 + pair_count) % pair_count;
            endgraph(); DoText();
            continue;
        } else if (key == 21 || key == 10) { /* Right or Down -> next */
            idx = (idx + 1) % pair_count;
            endgraph(); DoText();
            continue;
        } else if (key == ' ') { /* space -> cycle display_state 0..3 */
            display_state = (display_state + 1) & 3; /* faster than %4 */
            endgraph(); DoText();
            continue;
        } else if (key == 'F' || key == 'f') {
            FILE *of = fopen("overlap.csv", "w");
            if (of) {
                fprintf(of, "face1,%d,face2,%d,overlap,%s\n", f1, f2, ov ? "YES" : "NO");
                fprintf(of, "face_id,vertex_order,vertex_index,x2d,y2d\n");
                int off1 = faces->vertex_indices_ptr[f1]; int n1 = faces->vertex_count[f1];
                for (int vi = 0; vi < n1; ++vi) {
                    int idxv = faces->vertex_indices_buffer[off1 + vi] - 1;
                    if (idxv >= 0 && idxv < vtx->vertex_count) fprintf(of, "%d,%d,%d,%d,%d\n", f1, vi, idxv, vtx->x2d[idxv], vtx->y2d[idxv]);
                }
                int off2 = faces->vertex_indices_ptr[f2]; int n2 = faces->vertex_count[f2];
                for (int vi = 0; vi < n2; ++vi) {
                    int idxv = faces->vertex_indices_buffer[off2 + vi] - 1;
                    if (idxv >= 0 && idxv < vtx->vertex_count) fprintf(of, "%d,%d,%d,%d,%d\n", f2, vi, idxv, vtx->x2d[idxv], vtx->y2d[idxv]);
                }
                fclose(of);
                printf("Saved overlap.csv\n");
            } else printf("Error: cannot open overlap.csv for writing\n");
            endgraph(); DoText();
            continue;
        } else if (key == 'a' || key == 'A') {
            endgraph();
            DoText();
            printf("Starting full scan of all overlapping face pairs...\n");
            /* Scan all bbox-intersecting pairs and write each pair with YES/NO per projected_polygons_overlap.
             * Writes to a temporary file and atomically renames at the end to avoid partial files if interrupted. */
            const char *tmpname = "overlaptmp.csv";
            const char *finalname = "overlapall.csv";
            /* Remove stale files so the user won't see an old partial file while scan is running */
            remove(finalname);
            remove(tmpname);

            /* Accumulate CSV in memory first to avoid virtual-disk I/O errors during the scan. */
            char *buf = NULL; size_t blen = 0, bcap = 0;
            char tmp[512]; int tn = 0;
            int matches = 0;
            int processed = 0;
            int io_error = 0;
            for (int i = 0; i < faces->face_count && !io_error; ++i) {
                for (int j = i + 1; j < faces->face_count; ++j) {
                    int minx1 = faces->minx[i], maxx1 = faces->maxx[i], miny1 = faces->miny[i], maxy1 = faces->maxy[i];
                    int minx2 = faces->minx[j], maxx2 = faces->maxx[j], miny2 = faces->miny[j], maxy2 = faces->maxy[j];
                    if (maxx1 <= minx2 || maxx2 <= minx1 || maxy1 <= miny2 || maxy2 <= miny1) { processed++; continue; }
                    int ov = projected_polygons_overlap(model, i, j);
                    tn = snprintf(tmp, sizeof(tmp), "face1,%d,face2,%d,overlap,%s\n", i, j, ov ? "YES" : "NO");
                    if (tn < 0) { io_error = 1; break; }
                    if (blen + tn + 1 > bcap) {
                        size_t need = blen + tn + 1;
                        size_t newcap = bcap ? bcap * 2 : need + 1024;
                        while (newcap < need) newcap *= 2;
                        char *nb = (char*)realloc(buf, newcap);
                        if (!nb) { io_error = 1; break; }
                        buf = nb; bcap = newcap;
                    }
                    memcpy(buf + blen, tmp, tn); blen += tn; buf[blen] = '\0';

                    if ((tn = snprintf(tmp, sizeof(tmp), "face_id,vertex_order,vertex_index,x2d,y2d\n")) < 0) { io_error = 1; break; }
                    if (blen + tn + 1 > bcap) { size_t need = blen + tn + 1; size_t newcap = bcap ? bcap * 2 : need + 1024; while (newcap < need) newcap *= 2; char *nb = (char*)realloc(buf, newcap); if (!nb) { io_error = 1; break; } buf = nb; bcap = newcap; }
                    memcpy(buf + blen, tmp, tn); blen += tn; buf[blen] = '\0';

                    int off1 = faces->vertex_indices_ptr[i]; int n1 = faces->vertex_count[i];
                    for (int vi = 0; vi < n1; ++vi) {
                        int idxv = faces->vertex_indices_buffer[off1 + vi] - 1;
                        if (idxv >= 0 && idxv < vtx->vertex_count) {
                            tn = snprintf(tmp, sizeof(tmp), "%d,%d,%d,%d,%d\n", i, vi, idxv, vtx->x2d[idxv], vtx->y2d[idxv]);
                            if (tn < 0) { io_error = 1; break; }
                            if (blen + tn + 1 > bcap) { size_t need = blen + tn + 1; size_t newcap = bcap ? bcap * 2 : need + 1024; while (newcap < need) newcap *= 2; char *nb = (char*)realloc(buf, newcap); if (!nb) { io_error = 1; break; } buf = nb; bcap = newcap; }
                            memcpy(buf + blen, tmp, tn); blen += tn; buf[blen] = '\0';
                        }
                    }
                    if (io_error) break;
                    int off2 = faces->vertex_indices_ptr[j]; int n2 = faces->vertex_count[j];
                    for (int vi = 0; vi < n2; ++vi) {
                        int idxv = faces->vertex_indices_buffer[off2 + vi] - 1;
                        if (idxv >= 0 && idxv < vtx->vertex_count) {
                            tn = snprintf(tmp, sizeof(tmp), "%d,%d,%d,%d,%d\n", j, vi, idxv, vtx->x2d[idxv], vtx->y2d[idxv]);
                            if (tn < 0) { io_error = 1; break; }
                            if (blen + tn + 1 > bcap) { size_t need = blen + tn + 1; size_t newcap = bcap ? bcap * 2 : need + 1024; while (newcap < need) newcap *= 2; char *nb = (char*)realloc(buf, newcap); if (!nb) { io_error = 1; break; } buf = nb; bcap = newcap; }
                            memcpy(buf + blen, tmp, tn); blen += tn; buf[blen] = '\0';
                        }
                    }
                    if (io_error) break;
                    if ((tn = snprintf(tmp, sizeof(tmp), "\n")) < 0) { io_error = 1; break; }
                    if (blen + tn + 1 > bcap) { size_t need = blen + tn + 1; size_t newcap = bcap ? bcap * 2 : need + 1024; while (newcap < need) newcap *= 2; char *nb = (char*)realloc(buf, newcap); if (!nb) { io_error = 1; break; } buf = nb; bcap = newcap; }
                    memcpy(buf + blen, tmp, tn); blen += tn; buf[blen] = '\0';

                    /* Append debug entry to debug buffer */
                    {
                        /* lazy init debug buffer */
                        static char *dbuf = NULL; static size_t dblen = 0; static size_t dbcap = 0;
                        int sampled = 0;
                        int oxmin = minx1 > minx2 ? minx1 : minx2;
                        int oxmax = maxx1 < maxx2 ? maxx1 : maxx2;
                        int oymin = miny1 > miny2 ? miny1 : miny2;
                        int oymax = maxy1 < maxy2 ? maxy1 : maxy2;
                        if (!(oxmin > oxmax || oymin > oymax)) {
                            int W = oxmax - oxmin; int H = oymax - oymin;
                            for (int ssx = 0; ssx < 3 && !sampled; ++ssx) for (int ssy = 0; ssy < 3 && !sampled; ++ssy) {
                                int tx2 = oxmin + (((2*ssx + 1) * W + 3) / 6);
                                int ty2 = oymin + (((2*ssy + 1) * H + 3) / 6);
                                if (point_in_poly_int(tx2, ty2, faces, vtx, i, n1) && point_in_poly_int(tx2, ty2, faces, vtx, j, n2)) sampled = 1;
                            }
                        }
                        int icx = 0, icy = 0; double iarea = 0.0; long long _ia2 = 0; compute_intersection_centroid_ordered_qd_fixed(model, i, j, &icx, &icy, &_ia2); iarea = (double)_ia2;
                        int ident = faces_vertices_equal(faces, vtx, i, j);
                        int dtn = snprintf(tmp, sizeof(tmp), "%d,%d,%s,%d,%.6f,%d\n", i, j, ov ? "YES" : "NO", sampled, iarea, ident);
                        if (dtn >= 0) {
                            if (dblen + dtn + 1 > dbcap) {
                                size_t need = dblen + dtn + 1;
                                size_t newcap = dbcap ? dbcap * 2 : need + 1024;
                                while (newcap < need) newcap *= 2;
                                char *nb = (char*)realloc(dbuf, newcap);
                                if (nb) { dbuf = nb; dbcap = newcap; }
                            }
                            if (dblen + dtn + 1 <= dbcap) { memcpy(dbuf + dblen, tmp, dtn); dblen += dtn; dbuf[dblen] = '\0'; }
                        }
                        /* write debug buffer at the very end when building completes; store in static locals */
                    }

                    matches++;
                    processed++;
                    if ((processed & 10) == 0) {
                        printf(" .");
                        fflush(stdout);
                    }
                }
            }
            if (io_error) {
                if (buf) free(buf);
                printf("I/O error while building output buffer: processed %d, written %d\n", processed, matches);
                keypress();
                continue;
            }
            /* If we built a debug buffer, write it next to a debug CSV */
            {
                FILE *dbf = fopen("overlap.csv","w");
                if (dbf) {
                    fprintf(dbf, "face1,face2,reported,sampled,clipped_area,identical\n");
                    extern char *dbuf; extern size_t dblen; /* refer to static locals above */
                    /* We can't reference the static locals' names here; instead rebuild debug by recomputing quickly */
                    for (int ii = 0; ii < faces->face_count; ++ii) {
                        for (int jj = ii + 1; jj < faces->face_count; ++jj) {
                            int minx1b = faces->minx[ii], maxx1b = faces->maxx[ii], miny1b = faces->miny[ii], maxy1b = faces->maxy[ii];
                            int minx2b = faces->minx[jj], maxx2b = faces->maxx[jj], miny2b = faces->miny[jj], maxy2b = faces->maxy[jj];
                            if (maxx1b <= minx2b || maxx2b <= minx1b || maxy1b <= miny2b || maxy2b <= miny1b) continue;
                            int ov2 = projected_polygons_overlap(model, ii, jj);
                            int sampled2 = 0; int oxminb = minx1b > minx2b ? minx1b : minx2b; int oxmaxb = maxx1b < maxx2b ? maxx1b : maxx2b; int oyminb = miny1b > miny2b ? miny1b : miny2b; int oymaxb = maxy1b < maxy2b ? maxy1b : maxy2b;
                            if (!(oxminb > oxmaxb || oyminb > oymaxb)) {
                                int Wb = oxmaxb - oxminb; int Hb = oymaxb - oyminb;
                                for (int ssx = 0; ssx < 3 && !sampled2; ++ssx) for (int ssy = 0; ssy < 3 && !sampled2; ++ssy) {
                                    int tx2 = oxminb + (((2*ssx + 1) * Wb + 3) / 6);
                                    int ty2 = oyminb + (((2*ssy + 1) * Hb + 3) / 6);
                                    if (point_in_poly_int(tx2, ty2, faces, vtx, ii, faces->vertex_count[ii]) && point_in_poly_int(tx2, ty2, faces, vtx, jj, faces->vertex_count[jj])) sampled2 = 1;
                                }
                            }
                            int icx2 = 0, icy2 = 0; double iarea2 = 0.0; long long _ia22 = 0; compute_intersection_centroid_ordered_qd_fixed(model, ii, jj, &icx2, &icy2, &_ia22); iarea2 = (double)_ia22;
                            int ident2 = faces_vertices_equal(faces, vtx, ii, jj);
                            fprintf(dbf, "%d,%d,%s,%d,%.6f,%d\n", ii, jj, ov2 ? "YES" : "NO", sampled2, iarea2, ident2);
                        }
                    }
                    fclose(dbf);
                }
            }

            /* Attempt single-shot write to final file to avoid partials on virtual disk */
            FILE *of = fopen(finalname, "w");
            if (!of) {
                if (buf) free(buf);
                printf("Error: cannot open %s for writing\n", finalname);
                keypress();
                continue;
            }
            if (fwrite(buf ? buf : "", 1, blen, of) != blen) {
                printf("Write failed while saving %s\n", finalname);
                fclose(of);
                if (buf) free(buf);
                keypress();
                continue;
            }
            fclose(of);
            if (buf) free(buf);
            printf("\nSaved overlapall.csv (%d pairs written, processed %d pairs)\n", matches, processed);
            printf("Press any key to continue...\n");
            keypress(); /*  get user key  */
            continue;
        } else {
            break; /* any other key exits */
        }
    }

    free(pairs);
    endgraph(); 
    DoText();
    /* Restore */
    framePolyOnly = old_frame;
    /* Restore jitter state saved at the start of the scan */
    jitter = saved_jitter;
    for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = backup_flags[i];
    free(backup_flags);
}

/* Global non-interactive scanner (triggered by G/g) that writes ovscan.csv and ovscanrpt.txt */
void scan_all_overlaps(Model3D* model, ObserverParams* params) {
    if (!model) { printf("No model loaded\n"); return; }
    FaceArrays3D* faces = &model->faces; VertexArrays3D* vtx = &model->vertices;

    int saved_jitter = jitter; jitter = 0;
    Fixed32 saved_scale = s_global_proj_scale_fixed;

    /* gather bbox-intersecting pairs */
    typedef struct { int a; int b; } Pair;
    int pair_count = 0;
    for (int i = 0; i < faces->face_count; ++i) for (int j = i+1; j < faces->face_count; ++j) {
        if (cull_back_faces) { if (faces->plane_d[i] <= 0 || faces->plane_d[j] <= 0) continue; }
        int minx1 = faces->minx[i], maxx1 = faces->maxx[i], miny1 = faces->miny[i], maxy1 = faces->maxy[i];
        int minx2 = faces->minx[j], maxx2 = faces->maxx[j], miny2 = faces->miny[j], maxy2 = faces->maxy[j];
        if (maxx1 <= minx2 || maxx2 <= minx1 || maxy1 <= miny2 || maxy2 <= miny1) continue;
        pair_count++;
    }
    if (pair_count == 0) { printf("No bbox-intersecting pairs found.\n"); jitter = saved_jitter; return; }
    Pair* pairs = (Pair*)malloc(sizeof(Pair) * pair_count); int pi = 0;
    for (int i = 0; i < faces->face_count; ++i) for (int j = i+1; j < faces->face_count; ++j) {
        if (cull_back_faces) { if (faces->plane_d[i] <= 0 || faces->plane_d[j] <= 0) continue; }
        int minx1 = faces->minx[i], maxx1 = faces->maxx[i], miny1 = faces->miny[i], maxy1 = faces->maxy[i];
        int minx2 = faces->minx[j], maxx2 = faces->maxx[j], miny2 = faces->miny[j], maxy2 = faces->maxy[j];
        if (maxx1 <= minx2 || maxx2 <= minx1 || maxy1 <= miny2 || maxy2 <= miny1) continue;
        pairs[pi].a = i; pairs[pi].b = j; pi++;
    }

    FILE *of = fopen("ovscan.csv","w");
    if (!of) { printf("Error: cannot open ovscan.csv for writing\n"); free(pairs); jitter = saved_jitter; return; }
    fprintf(of, "f1,f2,ov_current,ov_fixed,ov_float,ia1_fixed,ia2_fixed,ia1_float,ia2_float,bbox_area,cf1_fixed_v,cf1_float_v,cf2_fixed_v,cf2_float_v,cf1_raw_area2,cf2_raw_area2,cf1_float_over,cf2_float_over,notes\n");

    int divergent_pairs = 0; int override_changed = 0;
    for (int k = 0; k < pair_count; ++k) {
        int f1 = pairs[k].a; int f2 = pairs[k].b;
        compute2DFromObserver(model, params->angle_w);
        int ov_current = projected_polygons_overlap(model, f1, f2);

        int icx1=0,icy1=0; double ia1=0.0; debug_overlap_subj = f1; debug_overlap_clip = f2; long long _ia1tmp = 0; compute_intersection_centroid_ordered_qd_fixed(model, f1, f2, &icx1, &icy1, &_ia1tmp); ia1 = (double)_ia1tmp; debug_overlap_subj = -1; debug_overlap_clip = -1;
        double ia1_fixed = debug_clip_fixed_area; double ia1_float = debug_clip_float_area; int cf1_fixed_v = debug_clip_fixed_vcount; int cf1_float_v = debug_clip_float_vcount; long long cf1_raw = debug_clip_raw_area2; int cf1_float_over = debug_clip_float_overridden;

        int icx2=0,icy2=0; double ia2=0.0; debug_overlap_subj = f2; debug_overlap_clip = f1; long long _ia2tmp = 0; compute_intersection_centroid_ordered_qd_fixed(model, f2, f1, &icx2, &icy2, &_ia2tmp); ia2 = (double)_ia2tmp; debug_overlap_subj = -1; debug_overlap_clip = -1;
        double ia2_fixed = debug_clip_fixed_area; double ia2_float = debug_clip_float_area; int cf2_fixed_v = debug_clip_fixed_vcount; int cf2_float_v = debug_clip_float_vcount; long long cf2_raw = debug_clip_raw_area2; int cf2_float_over = debug_clip_float_overridden;

        int bx0 = faces->minx[f1] > faces->minx[f2] ? faces->minx[f1] : faces->minx[f2];
        int bx1 = faces->maxx[f1] < faces->maxx[f2] ? faces->maxx[f1] : faces->maxx[f2];
        int by0 = faces->miny[f1] > faces->miny[f2] ? faces->miny[f1] : faces->miny[f2];
        int by1 = faces->maxy[f1] < faces->maxy[f2] ? faces->maxy[f1] : faces->maxy[f2];
        double bbox_area = 0.0; if (bx0 <= bx1 && by0 <= by1) bbox_area = (double)(bx1 - bx0) * (double)(by1 - by0);
        const double eps = 1e-9;
        int valid1_fixed = (cf1_fixed_v >= 3 && ia1_fixed >= MIN_INTERSECTION_AREA_PIXELS && ia1_fixed <= bbox_area + eps);
        int valid2_fixed = (cf2_fixed_v >= 3 && ia2_fixed >= MIN_INTERSECTION_AREA_PIXELS && ia2_fixed <= bbox_area + eps);
        int ov_fixed = (valid1_fixed || valid2_fixed) ? 1 : 0;
        int valid1_float = (cf1_float_v >= 3 && ia1_float >= MIN_INTERSECTION_AREA_PIXELS && ia1_float <= bbox_area + eps);
        int valid2_float = (cf2_float_v >= 3 && ia2_float >= MIN_INTERSECTION_AREA_PIXELS && ia2_float <= bbox_area + eps);
        int ov_float = (valid1_float || valid2_float) ? 1 : 0;
        char notes[128] = "";
        if (ov_fixed != ov_float) { divergent_pairs++; snprintf(notes, sizeof(notes), "ov_fixed!=ov_float"); }
        if (ov_current != ov_fixed) { override_changed++; if (notes[0]) strncat(notes, ";cur!=fixed", sizeof(notes)-strlen(notes)-1); else snprintf(notes, sizeof(notes), "cur!=fixed"); }
        fprintf(of, "%d,%d,%d,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%d,%d,%lld,%lld,%d,%d,%s\n",
                f1, f2, ov_current, ov_fixed, ov_float, ia1_fixed, ia2_fixed, ia1_float, ia2_float, bbox_area,
                cf1_fixed_v, cf1_float_v, cf2_fixed_v, cf2_float_v, cf1_raw, cf2_raw, cf1_float_over, cf2_float_over, notes);
    }
    fclose(of);
    FILE *rf = fopen("ovscanrpt.txt","w");
    if (rf) { fprintf(rf, "ovscan: total_pairs,%d,divergent_pairs,%d,override_changed,%d\n", pair_count, divergent_pairs, override_changed); fclose(rf); }
    printf("Scan complete: %d pairs scanned, %d divergent, %d where current differs from fixed\n", pair_count, divergent_pairs, override_changed);
    free(pairs);
    jitter = saved_jitter; s_global_proj_scale_fixed = saved_scale;
}

segment "code24";
/* display_model_face_ids
 * ----------------------
 * Draw the model in wireframe and overlay each face id centered on its polygon.
 * Implementation details:
 *  - Sets `framePolyOnly = 1` to render the wireframe for clarity,
 *  - Ensures all faces are visible, draws polygon frames, then calls `drawFace`
 *    with `show_index == 1` for each face to render the numeric ID in the center.
 */
void display_model_face_ids(Model3D* model, ObserverParams* params, const char* filename) {
    if (!model || !params) return;
    FaceArrays3D* faces = &model->faces;
    if (faces->face_count <= 0) { printf("No faces in model\n"); return; }

    startgraph(mode);

    // Backup display flags
    unsigned char* backup_flags = (unsigned char*)malloc(faces->face_count);
    for (int i = 0; i < faces->face_count; ++i) backup_flags[i] = faces->display_flag[i];

    int old_frame = framePolyOnly;
    framePolyOnly = 1; // wireframe for clear labels

    if (jitter) drawPolygons_jitter(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count); else drawPolygons(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count);

    int screenScale = mode / 320;
    // Draw each face in the current sorted order (use existing faces->sorted_face_indices)
    for (int si = 0; si < faces->face_count; ++si) {
        int f = faces->sorted_face_indices[si];
        if (!faces->display_flag[f]) continue;
        // Draw face filled with green (fillPenPat = 10)
        drawFace(model, f, 10, 0);

        // Compute integer bounding box center (use same logic as drawFace)
        int offset = faces->vertex_indices_ptr[f];
        int n = faces->vertex_count[f];
        int min_x = 999999, max_x = -999999, min_y = 999999, max_y = -999999;
        for (int k = 0; k < n; ++k) {
            int vi = faces->vertex_indices_buffer[offset + k] - 1;
            if (vi < 0 || vi >= model->vertices.vertex_count) continue;
            int x = model->vertices.x2d[vi];
            int y = model->vertices.y2d[vi];
            if (x < min_x) min_x = x; if (x > max_x) max_x = x; if (y < min_y) min_y = y; if (y > max_y) max_y = y;
        }
        if (min_x <= max_x && min_y <= max_y) {
            int center_x = (min_x + max_x) / 2;
            int center_y = (min_y + max_y) / 2;
            int screenCx = screenScale * (center_x + pan_dx);
            int screenCy = center_y + pan_dy;

            char tmp[32];
            int len = snprintf(tmp, sizeof(tmp), "%d", f);
            if (len > 15) len = 15;
            unsigned char pstr[16];
            pstr[0] = (unsigned char)len;
            memcpy(&pstr[1], tmp, len);

            // Draw index in green
            SetSolidPenPat(10);
            MoveTo(screenCx, screenCy);
            DrawString(pstr);
            SetSolidPenPat(14);
        }
    }

    MoveTo(5, 195);
    printf("Press any key to return\n");
    keypress();
    endgraph();
    DoText();

    // Restore
    framePolyOnly = old_frame;
    for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = backup_flags[i];
    free(backup_flags);
}

    // UTILITY FUNCTIONS...
// ============================================================================
//                    3D MODEL MANAGEMENT FUNCTIONS
// ============================================================================
/**
 * CREATING A NEW 3D MODEL
 * ========================
 * 
 * This function dynamically allocates all structures necessary
 * for a 3D model. Dynamic allocation is crucial on Apple IIGS
 * because the stack is limited and cannot contain large arrays.
 * 
 * ALLOCATION STRATEGY:
 * 1. Main Model3D structure allocation
 * 2. Vertex array allocation (MAX_VERTICES elements)
 * 3. Face array allocation (MAX_FACES elements)
 * 4. On failure: cleanup of previous allocations
 * 
 * ERROR HANDLING:
 * - Check each allocation
 * - Automatic cascade cleanup on partial failure
 * - Return NULL if unable to allocate
 */
Model3D* createModel3D(void) {
    // Step 1: Main structure allocation
    Model3D* model = (Model3D*)malloc(sizeof(Model3D));
    if (model == NULL) {
        return NULL;
    }
    int n = MAX_VERTICES;
    model->vertices.vertex_count = n;

    // Initialize auto-scale metadata
    model->auto_scale = FIXED_ONE;
    model->auto_center_x = 0;
    model->auto_center_y = 0;
    model->auto_center_z = 0;
    model->auto_scaled = 0;
    model->auto_centered = 0;
    model->orig_x = NULL;
    model->orig_y = NULL;
    model->orig_z = NULL;

    /* Auto-fit suggestion defaults */
    model->auto_suggested_distance = 0;
    model->auto_suggested_proj_scale = 0;
    model->auto_proj_scale = 0;
    model->auto_fit_ready = 0;
    model->auto_fit_applied = 0;

    model->coord_buf = NULL;
    model->coord_buf_capacity = 0;
    
    // Step 2: Allocate vertex arrays using malloc (handles bank crossing better)
    // Note: malloc() should handle bank boundaries better than NewHandle()
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
        // Allocation failed, cleanup
        if (model->vertices.x) free(model->vertices.x);
        if (model->vertices.y) free(model->vertices.y);
        if (model->vertices.z) free(model->vertices.z);
        if (model->vertices.xo) free(model->vertices.xo);
        if (model->vertices.yo) free(model->vertices.yo);
        if (model->vertices.zo) free(model->vertices.zo);
        if (model->vertices.x2d) free(model->vertices.x2d);
        if (model->vertices.y2d) free(model->vertices.y2d);
        free(model);
        return NULL;
    }
    
    // Set dummy handles to NULL (not used with malloc)
    model->vertices.xHandle = NULL;
    model->vertices.yHandle = NULL;
    model->vertices.zHandle = NULL;
    model->vertices.xoHandle = NULL;
    model->vertices.yoHandle = NULL;
    model->vertices.zoHandle = NULL;
    model->vertices.x2dHandle = NULL;
    model->vertices.y2dHandle = NULL;
    
    // Step 3: Face array allocation using parallel arrays (like vertices)
    // Each element stored separately to fit 32KB limit per allocation
    int nf = MAX_FACES;
    
    // Allocate vertex count array: nf * 4 bytes = 24KB
    model->faces.vertex_count = (int*)malloc(nf * sizeof(int));
    if (!model->faces.vertex_count) {
        printf("Error: Unable to allocate memory for face vertex_count array\n");
        keypress();
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model);
        return NULL;
    }
    
    // Allocate SINGLE packed buffer for all vertex indices
    // Estimate: average 3.5 indices per face (mix of triangles and quads)
    // For 6000 faces: ~21KB. We allocate conservatively at 5 per face = 120KB max
    int estimated_total_indices = nf * 5;
    model->faces.vertex_indices_buffer = (int*)malloc(estimated_total_indices * sizeof(int));
    if (!model->faces.vertex_indices_buffer) {
        printf("Error: Unable to allocate memory for vertex_indices_buffer\n");
        keypress();
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model->faces.vertex_count);
        free(model);
        return NULL;
    }
    
    // Allocate offset array: one offset per face into the packed buffer
    model->faces.vertex_indices_ptr = (int*)malloc(nf * sizeof(int));
    if (!model->faces.vertex_indices_ptr) {
        printf("Error: Unable to allocate memory for vertex_indices_ptr array\n");
        keypress();
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model->faces.vertex_count);
        free(model->faces.vertex_indices_buffer);
        free(model);
        return NULL;
    }
    
    // Allocate z_min and z_max arrays (min and max depth per face)
    model->faces.z_min = (Fixed32*)malloc(nf * sizeof(Fixed32));
    model->faces.z_max = (Fixed32*)malloc(nf * sizeof(Fixed32));
    if (!model->faces.z_min || !model->faces.z_max) {
        printf("Error: Unable to allocate memory for face depth arrays\n");
        keypress();
        if (model->faces.z_min) free(model->faces.z_min);
        if (model->faces.z_max) free(model->faces.z_max);
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model->faces.vertex_count);
        free(model->faces.vertex_indices_buffer);
        free(model->faces.vertex_indices_ptr);
        free(model);
        return NULL;
    }
    // Allocate z_mean array (mean depth per face) - used by painter_newell_sancha
    model->faces.z_mean = (Fixed32*)malloc(nf * sizeof(Fixed32));
    if (!model->faces.z_mean) {
        printf("Error: Unable to allocate memory for face z_mean array\n");
        keypress();
        if (model->faces.z_mean) free(model->faces.z_mean);
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model->faces.vertex_count);
        free(model->faces.vertex_indices_buffer);
        free(model->faces.vertex_indices_ptr);
        if (model->faces.z_min) free(model->faces.z_min);
        if (model->faces.z_max) free(model->faces.z_max);
        free(model);
        return NULL;
    }

    // Allocate plane coefficient arrays (normalized normals + d term)
    model->faces.plane_a = (Fixed64*)malloc(nf * sizeof(Fixed64));
    model->faces.plane_b = (Fixed64*)malloc(nf * sizeof(Fixed64));
    model->faces.plane_c = (Fixed64*)malloc(nf * sizeof(Fixed64));
    model->faces.plane_d = (Fixed64*)malloc(nf * sizeof(Fixed64));
    if (!model->faces.plane_a || !model->faces.plane_b || !model->faces.plane_c || !model->faces.plane_d) {
        printf("Error: Unable to allocate memory for face plane arrays\n");
        keypress();
        if (model->faces.plane_a) free(model->faces.plane_a);
        if (model->faces.plane_b) free(model->faces.plane_b);
        if (model->faces.plane_c) free(model->faces.plane_c);
        if (model->faces.plane_d) free(model->faces.plane_d);
        if (model->faces.z_mean) free(model->faces.z_mean);
        if (model->faces.z_min) free(model->faces.z_min);
        if (model->faces.z_max) free(model->faces.z_max);
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model->faces.vertex_count);
        free(model->faces.vertex_indices_buffer);
        free(model->faces.vertex_indices_ptr);
        free(model->faces.z_max);
        free(model);
        return NULL;
    }

    // Allocate cached 2D bounding boxes arrays
    model->faces.minx = (int*)malloc(nf * sizeof(int));
    model->faces.maxx = (int*)malloc(nf * sizeof(int));
    model->faces.miny = (int*)malloc(nf * sizeof(int));
    model->faces.maxy = (int*)malloc(nf * sizeof(int));
    if (!model->faces.minx || !model->faces.maxx || !model->faces.miny || !model->faces.maxy) {
        printf("Error: Unable to allocate memory for face bounding box arrays\n");
        keypress();
        if (model->faces.minx) free(model->faces.minx);
        if (model->faces.maxx) free(model->faces.maxx);
        if (model->faces.miny) free(model->faces.miny);
        if (model->faces.maxy) free(model->faces.maxy);
        if (model->faces.plane_a) free(model->faces.plane_a);
        if (model->faces.plane_b) free(model->faces.plane_b);
        if (model->faces.plane_c) free(model->faces.plane_c);
        if (model->faces.plane_d) free(model->faces.plane_d);
        if (model->faces.z_mean) free(model->faces.z_mean);
        if (model->faces.z_min) free(model->faces.z_min);
        if (model->faces.z_max) free(model->faces.z_max);
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model->faces.vertex_count);
        free(model->faces.vertex_indices_buffer);
        free(model->faces.vertex_indices_ptr);
        free(model->faces.z_max);
        free(model);
        return NULL;
    }
    
    // Allocate display_flag array: nf * 4 bytes = 24KB
    model->faces.display_flag = (int*)malloc(nf * sizeof(int));
    if (!model->faces.display_flag) {
        printf("Error: Unable to allocate memory for face display_flag array\n");
        keypress();
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model->faces.vertex_count);
        free(model->faces.vertex_indices_buffer);
        free(model->faces.vertex_indices_ptr);
        free(model->faces.z_max);
        if (model->faces.z_mean) free(model->faces.z_mean);
        free(model);
        return NULL;
    }
    
    // Initialize structure
    model->faces.vertex_countHandle = NULL;
    model->faces.vertex_indicesBufferHandle = NULL;
    model->faces.vertex_indicesPtrHandle = NULL;
    model->faces.z_maxHandle = NULL;
    model->faces.display_flagHandle = NULL;
    model->faces.sorted_face_indicesHandle = NULL;
    
    // Allocate sorted_face_indices array: nf * 4 bytes = 24KB max
    model->faces.sorted_face_indices = (int*)malloc(nf * sizeof(int));
    if (!model->faces.sorted_face_indices) {
        printf("Error: Unable to allocate memory for sorted_face_indices array\n");
        keypress();
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model->faces.vertex_count);
        free(model->faces.vertex_indices_buffer);
        free(model->faces.vertex_indices_ptr);
        if (model->faces.z_min) free(model->faces.z_min);
        if (model->faces.z_max) free(model->faces.z_max);
        if (model->faces.z_mean) free(model->faces.z_mean);
        free(model->faces.display_flag);
        free(model);
        return NULL;
    }
    
    // Initialize structure
    model->faces.vertex_countHandle = NULL;
    model->faces.vertex_indicesBufferHandle = NULL;
    model->faces.vertex_indicesPtrHandle = NULL;
    model->faces.z_maxHandle = NULL;
    model->faces.display_flagHandle = NULL;
    model->faces.sorted_face_indicesHandle = NULL;
    model->faces.total_indices = 0;
    
    return model;
}

/**
 * MEMORY CLEANUP - DESTROY MODEL 3D
 * ==================================
 * 
 * This function frees all memory allocated by createModel3D().
 * Must be called when the model is no longer needed to prevent memory leaks.
 * 
 * CLEANUP SEQUENCE:
 * 1. Free all vertex arrays (x, y, z, xo, yo, zo, x2d, y2d)
 * 2. Free all face arrays (vertex_count, vertex_indices_buffer, vertex_indices_ptr, z_max, display_flag, sorted_face_indices)
 * 3. Free main Model3D structure
 */
segment "code07";
void destroyModel3D(Model3D* model) {
    if (model != NULL) {
        // Free all vertex arrays
        if (model->vertices.x) free(model->vertices.x);
        if (model->vertices.y) free(model->vertices.y);
        if (model->vertices.z) free(model->vertices.z);
        if (model->vertices.xo) free(model->vertices.xo);
        if (model->vertices.yo) free(model->vertices.yo);
        if (model->vertices.zo) free(model->vertices.zo);
        if (model->vertices.x2d) free(model->vertices.x2d);
        if (model->vertices.y2d) free(model->vertices.y2d);
        
        // Free all face arrays (now simplified with packed buffer)
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
        if (model->faces.display_flag) free(model->faces.display_flag);
        if (model->faces.sorted_face_indices) free(model->faces.sorted_face_indices);

        // Free optional buffers and backups
        if (model->orig_x) free(model->orig_x);
        if (model->orig_y) free(model->orig_y);
        if (model->orig_z) free(model->orig_z);
        if (model->coord_buf) free(model->coord_buf);
        
        // Free main structure
        free(model);
    }
}

segment "code08";
/**
 * COMPLETE 3D MODEL LOADING
 * ==========================
 * 
 * This function coordinates the complete loading of an OBJ file
 * by successively calling the vertex and face reading functions.
 * 
 * LOADING PIPELINE:
 * 1. Input parameter validation
 * 2. Read vertices from file
 * 3. Read faces from file  
 * 4. Update counters in structure
 * 
 * ERROR HANDLING:
 * - Vertex reading failure: immediate stop
 * - Face reading failure: warning but continue
 *   (vertices-only model remains usable)
 */
int loadModel3D(Model3D* model, const char* filename) {
    // Input parameter validation
    if (model == NULL || filename == NULL) {
        return -1;  // Invalid parameters
    }
    
    // Step 1: Read vertices from OBJ file
    // --- Update global counter for face index verification ---
    readVertices_last_count = model->vertices.vertex_count;
    
    int vcount = readVertices(filename, &model->vertices, MAX_VERTICES, model);
    if (vcount < 0) {
        return -1;  // Critical failure: unable to read vertices
    }
    model->vertices.vertex_count = vcount;
    
    // Step 2: Read faces from OBJ file (using chunked allocation)
    // This function handles reading faces into 2 chunks transparently
    int fcount = readFaces_model(filename, model);
    if (fcount < 0) {
        // Critical failure: unable to read faces
        printf("\nWarning: Unable to read faces\n");
        model->faces.face_count = 0;  // No faces available
    } else {
        model->faces.face_count = fcount;
    }
    return 0;  // Success: model loaded (with or without faces)
}

// ============================================================================
//                    USER INTERFACE FUNCTIONS
// ============================================================================

/**
 * OBSERVER PARAMETER INPUT
 * ========================
 * 
 * This function presents a text interface to allow the
 * user to specify 3D visualization parameters.
 * 
 * REQUESTED PARAMETERS:
 * - Horizontal angle: rotation around Y-axis (left/right view)
 * - Vertical angle: rotation around X-axis (up/down view)
 * - Distance: observer distance (zoom)
 * - Screen rotation angle: final rotation in 2D plane
 * 
 * ERROR HANDLING:
 * - Default values if input failure
 * - Automatic string->Fixed32 conversion with atof() then FLOAT_TO_FIXED
 */
segment "code09";
void getObserverParams(ObserverParams* params, Model3D* model) {
    char input[50];  // Buffer for user input
    
    // Display section header
    printf("\nObserver parameters:\n");
    printf("============================\n");
    printf("(Press ENTER to use default values - apply suggested auto-fit if available)\n");
    
    // Input horizontal angle (rotation around Y)
    printf("Horizontal angle (degrees, default %d): ", params->angle_h);
    if (fgets(input, sizeof(input), stdin) != NULL) {
        // Remove newline
        input[strcspn(input, "\n")] = 0;
        if (strlen(input) != 0) {
            params->angle_h = atoi(input);  // Parse integer degrees from input
        }
    }
    
    // Input vertical angle (rotation around X)  
    printf("Vertical angle (degrees, default %d): ", params->angle_v);
    if (fgets(input, sizeof(input), stdin) != NULL) {
        // Remove newline
        input[strcspn(input, "\n")] = 0;
        if (strlen(input) != 0) {
            params->angle_v = atoi(input);  // Parse integer degrees from input
        }
    }
    

    // Input screen rotation angle (final 2D rotation)
    printf("Screen rotation angle (degrees, default %d): ", params->angle_w);
    if (fgets(input, sizeof(input), stdin) != NULL) {
        // Remove newline
        input[strcspn(input, "\n")] = 0;
        if (strlen(input) != 0) {
            params->angle_w = atoi(input);  // Parse integer degrees from input
        }
    }

    // Input observation distance (zoom/perspective).
    // Press ENTER = auto-scale + center using sphere-based fit (default target),
    // or enter a numeric value to use that distance directly (no scaling).
    printf("Distance (ENTER = auto-scale, or enter a value): ");

    if (fgets(input, sizeof(input), stdin) != NULL) {
        // Remove newline
        input[strcspn(input, "\n")] = 0;
        if (strlen(input) == 0) {
            // User pressed ENTER: if we have auto-fit suggestions from load, apply them; otherwise use default distance
            if (model != NULL && model->auto_fit_ready) {
                params->distance = model->auto_suggested_distance;
                // Apply suggested projection scale to model so subsequent processing uses it
                model->auto_proj_scale = model->auto_suggested_proj_scale;
                model->auto_fit_applied = 1;
                printf("Auto-fit applied (distance=%.4f proj_scale=%.2f). Use +/- to adjust.\n", FIXED_TO_FLOAT(model->auto_suggested_distance), FIXED_TO_FLOAT(model->auto_suggested_proj_scale));
            } else if (model != NULL) {
                // No precomputed suggestion: fallback to sensible default
                params->distance = FLOAT_TO_FIXED(30.0);
            } else {
                params->distance = FLOAT_TO_FIXED(30.0);
            }
        } else {
            // User provided a distance value -> use it directly (no auto-scale)
            params->distance = FLOAT_TO_FIXED(atof(input)); // String->Fixed32 conversion
        }
    } else {
        // fgets failed; default behavior: if we have a model, apply precomputed auto-fit suggestions, else fallback distance
        if (model != NULL && model->auto_fit_ready) {
            params->distance = model->auto_suggested_distance;
            model->auto_proj_scale = model->auto_suggested_proj_scale;
            model->auto_fit_applied = 1;
            printf("Auto-fit applied (distance=%.4f proj_scale=%.2f). Use +/- to adjust.\n", FIXED_TO_FLOAT(model->auto_suggested_distance), FIXED_TO_FLOAT(model->auto_suggested_proj_scale));
        } else if (model != NULL) {
            params->distance = FLOAT_TO_FIXED(30.0);
        } else {
            params->distance = FLOAT_TO_FIXED(30.0);        // Default distance: balanced view (Fixed Point)
        }
    }

    // Debug: show parsed observer angles in degrees
    // if (!PERFORMANCE_MODE)
    // {
    // printf("Observer angles (degrees) - H: %d, V: %d, W: %d\n", params->angle_h, params->angle_v, params->angle_w);
    // }
}

/**
 * processModelFast -- Combined transformation, projection, and painter invocation
 * ============================================================================
 * Responsibilities:
 *  - Transform model vertices into observer-space (xo/yo/zo) using Fixed32 arithmetic.
 *  - Project to integer screen coords (x2d/y2d) using configured projection scale.
 *  - If using the FLOAT painter mode, populate float caches (float_xo/float_yo/float_zo etc.)
 *    used by the float-based painter for efficient per-face computations.
 *  - Call `calculateFaceDepths()` to compute per-face depth metrics, bboxes and plane coeffs.
 *  - Dispatch to the selected painter implementation (FAST, FIXED, or FLOAT) to build
 *    the final `sorted_face_indices` for rendering.
 *
 * Notes:
 *  - Autoscaling is **not** performed here; any autoscale (previously implemented in an archived helper) must be applied
 *    before calling `processModelFast()` (e.g. at load time or via explicit user action). The archived helper's implementation is in `chutier.txt`.
 *  - The painter may only sort the visible faces when back-face culling is enabled. Culled
 *    faces are appended after visible faces to preserve index stability.
 *  - Timing instrumentation prints per-stage costs when not in PERFORMANCE_MODE.
 */
segment "code10";
void processModelFast(Model3D* model, ObserverParams* params, const char* filename) {
    int i;

    Fixed32 cos_h, sin_h, cos_v, sin_v, cos_w, sin_w;
    Fixed32 x, y, z, zo, xo, yo;
    Fixed32 inv_zo, x2d_temp, y2d_temp;
    
    // Direct table access - ultra-fast! (no function calls)
    cos_h = cos_deg_int(params->angle_h);
    sin_h = sin_deg_int(params->angle_h);
    cos_v = cos_deg_int(params->angle_v);
    sin_v = sin_deg_int(params->angle_v);
    cos_w = cos_deg_int(params->angle_w);
    sin_w = sin_deg_int(params->angle_w);

    // Pre-calculate all trigonometric products in Fixed32 - using 64-bit multiply
    const Fixed32 cos_h_cos_v = FIXED_MUL_64(cos_h, cos_v);
    const Fixed32 sin_h_cos_v = FIXED_MUL_64(sin_h, cos_v);
    const Fixed32 cos_h_sin_v = FIXED_MUL_64(cos_h, sin_v);
    const Fixed32 sin_h_sin_v = FIXED_MUL_64(sin_h, sin_v);
    // Use global projection scale, which is synced with auto-fit when applied
    Fixed32 scale = s_global_proj_scale_fixed;
    const Fixed32 centre_x_f = INT_TO_FIXED(CENTRE_X);
    const Fixed32 centre_y_f = INT_TO_FIXED(CENTRE_Y);
    Fixed32 distance = params->distance;
    // Apply auto-scale if any
    //distance = FIXED_MUL_64(distance, INT_TO_FIXED(4));
    
    // 100% Fixed32 loop - ZERO conversions, maximum speed!
    VertexArrays3D* vtx = &model->vertices;
    Fixed32 *x_arr = vtx->x, *y_arr = vtx->y, *z_arr = vtx->z;
    Fixed32 *xo_arr = vtx->xo, *yo_arr = vtx->yo, *zo_arr = vtx->zo;
    int *x2d_arr = vtx->x2d, *y2d_arr = vtx->y2d;
    int vcount = vtx->vertex_count;

    long t_loop_start = GetTick();
    for (i = 0; i < vcount; i++) {
        x = x_arr[i];
        y = y_arr[i];
        z = z_arr[i];
        // 3D transformation in pure Fixed32 (64-bit multiply)
        Fixed32 term1 = FIXED_MUL_64(x, cos_h_cos_v);
        Fixed32 term2 = FIXED_MUL_64(y, sin_h_cos_v);
        Fixed32 term3 = FIXED_MUL_64(z, sin_v);
        zo = FIXED_ADD(FIXED_SUB(FIXED_SUB(FIXED_NEG(term1), term2), term3), distance);
        if (zo > 0) {
            xo = FIXED_ADD(FIXED_NEG(FIXED_MUL_64(x, sin_h)), FIXED_MUL_64(y, cos_h));
            yo = FIXED_ADD(FIXED_SUB(FIXED_NEG(FIXED_MUL_64(x, cos_h_sin_v)), FIXED_MUL_64(y, sin_h_sin_v)), FIXED_MUL_64(z, cos_v));
            zo_arr[i] = zo;
            xo_arr[i] = xo;
            yo_arr[i] = yo;
            inv_zo = FIXED_DIV_64(scale, zo);
            x2d_temp = FIXED_ADD(FIXED_MUL_64(xo, inv_zo), centre_x_f);
            y2d_temp = FIXED_SUB(centre_y_f, FIXED_MUL_64(yo, inv_zo));
            x2d_arr[i] = FIXED_ROUND_TO_INT(FIXED_ADD(FIXED_SUB(FIXED_MUL_64(cos_w, FIXED_SUB(x2d_temp, centre_x_f)), FIXED_MUL_64(sin_w, FIXED_SUB(centre_y_f, y2d_temp))), centre_x_f));
            y2d_arr[i] = FIXED_ROUND_TO_INT(FIXED_SUB(centre_y_f, FIXED_ADD(FIXED_MUL_64(sin_w, FIXED_SUB(x2d_temp, centre_x_f)), FIXED_MUL_64(cos_w, FIXED_SUB(centre_y_f, y2d_temp)))));

        } else {
            zo_arr[i] = zo;
            xo_arr[i] = 0;
            yo_arr[i] = 0;
            x2d_arr[i] = -1;
            y2d_arr[i] = -1;
        }
    }
    long t_loop_end = GetTick();
    // if (!PERFORMANCE_MODE) {
    //     long elapsed_loop = t_loop_end - t_loop_start;
    //     double ms_loop = ((double)elapsed_loop * 1000.0) / 60.0; // 60 ticks per second
    //     printf("[TIMING] transform+project loop: %ld ticks (%.2f ms)\n", elapsed_loop, ms_loop);
    // }

    // Face sorting after transformation
    long t_start, t_end;
    t_start = GetTick();
    calculateFaceDepths(model, NULL, model->faces.face_count);
    t_end = GetTick();
    // if (!PERFORMANCE_MODE)
    // {
    //     long elapsed = t_end - t_start;
    //     double ms = ((double)elapsed * 1000.0) / 60.0; // 60 ticks per second
    //     printf("[TIMING] calculateFaceDepths: %ld ticks (%.2f ms)\n", elapsed, ms);
    // }

    if (framePolyOnly) goto skip_calc; // Skip face calculations for framed polygons only display

    // CRITICAL: Reset sorted_face_indices before each sort to prevent corruption
    for (i = 0; i < model->faces.face_count; i++) {
        model->faces.sorted_face_indices[i] = i;
    }
    // painter_newell_sancha 
    t_start = GetTick();
    if (painter_mode == PAINTER_MODE_FAST) {
        painter_newell_sancha_fast(model, model->faces.face_count);
    } else if (painter_mode == PAINTER_MODE_FIXED) {
        painter_newell_sancha(model, model->faces.face_count);
    } else if (painter_mode == PAINTER_MODE_CORRECT) {
        /* painter_correct acts as a sorting mode: it will adjust faces->sorted_face_indices in-place */
        painter_correct(model, model->faces.face_count, 0);
    } else if (painter_mode == PAINTER_MODE_CORRECTV2) {
        /* painter_correctV2: experimental face splitting version */
        painter_correctV2(model, model->faces.face_count, 0);
    } else if (painter_mode == PAINTER_MODE_NEWELL_SANCHAV3) {
        /* painter_newell_sanchaV3: back-faces-first variant when culling is OFF */
        painter_newell_sanchaV3(model, model->faces.face_count);
    } else if (painter_mode == PAINTER_MODE_NEWELL_SANCHAV2) {
        /* painter_newell_sanchaV2: conservative pass-2 inspired local reordering */
        painter_newell_sanchaV2(model, model->faces.face_count);
    } else {
        // PAINTER_MODE_FLOAT
        //painter_newell_sancha_float(model, model->faces.face_count);
    }
    t_end = GetTick();

    skip_calc:;
}

// ============================================================================
//                    BASIC FUNCTION IMPLEMENTATIONS
// ============================================================================

/**
 * VERTEX READING FROM OBJ FILE
 * ============================
 * 
 * This function parses an OBJ format file to extract
 * vertices (3D points). It searches for lines starting with "v "
 * and extracts X, Y, Z coordinates.
 * 
 * OBJ FORMAT FOR VERTICES:
 *   v 1.234 5.678 9.012
 *   v -2.5 0.0 3.14159
 * 
 * ALGORITHM:
 * 1. Open file in read mode
 * 2. Read line by line with fgets()
 * 3. Detect "v " lines with character verification
 * 4. Extract coordinates with sscanf()
 * 5. Store in array with bounds checking
 * 6. Progressive display for user feedback
 * 
 * ERROR HANDLING:
 * - File opening verification
 * - Array overflow protection
 * - Coordinate format validation
 */
segment "code12";
int readVertices(const char* filename, VertexArrays3D* vtx, int max_vertices, Model3D* owner) {
    FILE *file;
    char line[MAX_LINE_LENGTH];
    int line_number = 1;
    int vertex_count = 0;
    
    // Open file in read mode
    file = fopen(filename, "r");
    if (file == NULL) {
        printf("Error: Unable to open file '%s'\n", filename);
        printf("Check that the file exists.\n\n");
        return -1;  // Return -1 on error
    }
    
    printf("\nReading vertices from file '%s' ", filename);
    
    // Read file line by line
    while (fgets(line, sizeof(line), file) != NULL) {
        if (line[0] == 'v' && line[1] == ' ') {
            if (vertex_count < max_vertices) {
                float x, y, z;  // Temporary reading in float
                if (sscanf(line + 2, "%f %f %f", &x, &y, &z) == 3) {
                    vtx->x[vertex_count] = FLOAT_TO_FIXED(x);
                    // OBJ files often use Z as up; convert to viewer convention by swapping axes
                    vtx->y[vertex_count] = FLOAT_TO_FIXED(z);   // treat OBJ Z as up
                    vtx->z[vertex_count] = FLOAT_TO_FIXED(y);  // rotate -90° around X, corrected orientation
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

    // Close file
    fclose(file);

    // Compute bbox center and save into owner (do not modify vertex coords)
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
        // Apply centering to vertex coordinates (default behavior requested)
        for (int i = 0; i < vertex_count; ++i) {
            vtx->x[i] = FIXED_SUB(vtx->x[i], owner->auto_center_x);
            vtx->y[i] = FIXED_SUB(vtx->y[i], owner->auto_center_y);
            vtx->z[i] = FIXED_SUB(vtx->z[i], owner->auto_center_z);
        }
        owner->auto_centered = 1; // indicate coords were centered

        // --- Brute auto-fit suggestion (Fixed32) ---
        // Compute model-space max dimension (bbox metric) and suggest a distance = k * max_dim
        Fixed32 dx = FIXED_SUB(xmax, xmin);
        Fixed32 dy = FIXED_SUB(ymax, ymin);
        Fixed32 dz = FIXED_SUB(zmax, zmin);
        Fixed32 max_dim = dx;
        if (dy > max_dim) max_dim = dy;
        if (dz > max_dim) max_dim = dz;
        // k = 3 (user-specified)
        owner->auto_suggested_distance = FIXED_MUL_64(max_dim, INT_TO_FIXED(3));

        // Compute projected bounds using default observer angles (30,20,0) to suggest projection scale
        int ah = 30, av = 20, aw = 0;
        Fixed32 cos_h = cos_deg_int(ah), sin_h = sin_deg_int(ah);
        Fixed32 cos_v = cos_deg_int(av), sin_v = sin_deg_int(av);
        const Fixed32 cos_h_cos_v = FIXED_MUL_64(cos_h, cos_v);
        const Fixed32 sin_h_cos_v = FIXED_MUL_64(sin_h, cos_v);
        const Fixed32 cos_h_sin_v = FIXED_MUL_64(cos_h, sin_v);
        const Fixed32 sin_h_sin_v = FIXED_MUL_64(sin_h, sin_v);

        float pxmin = 1e30f, pxmax = -1e30f, pymin = 1e30f, pymax = -1e30f;
        for (int i = 0; i < vertex_count; ++i) {
            Fixed32 x = vtx->x[i]; Fixed32 y = vtx->y[i]; Fixed32 z = vtx->z[i];
            Fixed32 term1 = FIXED_MUL_64(x, cos_h_cos_v);
            Fixed32 term2 = FIXED_MUL_64(y, sin_h_cos_v);
            Fixed32 term3 = FIXED_MUL_64(z, sin_v);
            Fixed32 zo = FIXED_ADD(FIXED_SUB(FIXED_SUB(FIXED_NEG(term1), term2), term3), owner->auto_suggested_distance);
            if (zo > 0) {
                Fixed32 xo = FIXED_ADD(FIXED_NEG(FIXED_MUL_64(x, sin_h)), FIXED_MUL_64(y, cos_h));
                Fixed32 yo = FIXED_ADD(FIXED_SUB(FIXED_NEG(FIXED_MUL_64(x, cos_h_sin_v)), FIXED_MUL_64(y, sin_h_sin_v)), FIXED_MUL_64(z, cos_v));
                float px = FIXED_TO_FLOAT(xo) / FIXED_TO_FLOAT(zo);
                float py = FIXED_TO_FLOAT(yo) / FIXED_TO_FLOAT(zo);
                if (px < pxmin) pxmin = px;
                if (px > pxmax) pxmax = px;
                if (py < pymin) pymin = py;
                if (py > pymax) pymax = py;
            }
        }
        // Compute projection scale (pixels per projected unit) to fit viewport with margin m
        float margin = 0.9f;
        float winw = (float)(CENTRE_X * 2);
        float winh = (float)(CENTRE_Y * 2);
        float s = 100.0f; // fallback
        if (pxmax > pxmin && pymax > pymin) {
            float s1 = (winw * margin) / (pxmax - pxmin);
            float s2 = (winh * margin) / (pymax - pymin);
            s = (s1 < s2) ? s1 : s2;
        }
        owner->auto_suggested_proj_scale = FLOAT_TO_FIXED(s);
        owner->auto_fit_ready = 1;

    #if ENABLE_DEBUG_SAVE
        printf("[INFO] readVertices: applied bbox center cx=%.4f cy=%.4f cz=%.4f\n", FIXED_TO_FLOAT(owner->auto_center_x), FIXED_TO_FLOAT(owner->auto_center_y), FIXED_TO_FLOAT(owner->auto_center_z));
        printf("[INFO] readVertices: brute auto-fit suggestion: distance=%.4f proj_scale=%.2f\n", FIXED_TO_FLOAT(owner->auto_suggested_distance), s);
    #endif
    }

    // printf("\n\nAnalyse terminee. %d lignes lues.\n", line_number - 1);
    return vertex_count;  // Return the number of vertices read
}

segment "code13";
// Function to read faces into parallel arrays in FaceArrays3D structure
int readFaces_model(const char* filename, Model3D* model) {
    FILE *file;
    char line[MAX_LINE_LENGTH];
    int line_number = 1;
    int face_count = 0;
    int i;
    
    // Validate model structure
    if (model == NULL || model->faces.vertex_count == NULL) {
        printf("Error: Invalid model structure for readFaces_model\n");
        return -1;
    }
    
    // Open file in read mode
    file = fopen(filename, "r");
    if (file == NULL) {
        printf("Error: Unable to open file '%s' to read faces\n", filename);
        return -1;
    }
    
    printf("\nReading faces from file '%s' ", filename);
    
    int buffer_pos = 0;  // Current position in the packed buffer
    
    // Read file line by line
    while (fgets(line, sizeof(line), file) != NULL) {
        // Check if line starts with "f " (face)
        if (line[0] == 'f' && line[1] == ' ') {
            if (face_count < MAX_FACES) {
                // Initialize face data
                model->faces.vertex_count[face_count] = 0;
                model->faces.display_flag[face_count] = 1;  // Displayable by default
                model->faces.vertex_indices_ptr[face_count] = buffer_pos;  // Store offset to this face's indices
                
                // Parse vertices from this face
                char *ptr = line + 2;  // Start after "f "
                int temp_indices[MAX_FACE_VERTICES];
                int temp_vertex_count = 0;
                int invalid_index_found = 0;
                
                // Parse character by character
                while (*ptr != '\0' && *ptr != '\n' && temp_vertex_count < MAX_FACE_VERTICES) {
                    // Skip spaces and tabs
                    while (*ptr == ' ' || *ptr == '\t') ptr++;
                    
                    if (*ptr == '\0' || *ptr == '\n') break;
                    
                    // Read the number
                    int vertex_index = 0;
                    while (*ptr >= '0' && *ptr <= '9') {
                        vertex_index = vertex_index * 10 + (*ptr - '0');
                        ptr++;
                    }
                    
                    // Skip texture/normal data (after /)
                    while (*ptr != '\0' && *ptr != ' ' && *ptr != '\t' && *ptr != '\n') {
                        ptr++;
                    }
                    
                    // Validate vertex index
                    if (vertex_index >= 1) {
                        if (vertex_index > readVertices_last_count) {
                            // Index out of bounds
                            invalid_index_found = 1;
                        }
                        temp_indices[temp_vertex_count] = vertex_index;
                        temp_vertex_count++;
                    }
                }
                
                // Check for errors
                if (invalid_index_found) {
                    printf("\nERROR: Face at line %d references vertex index > %d vertices\n", 
                           line_number, readVertices_last_count);
                    fclose(file);
                    return -1;
                } else {
                    // Store valid indices into the packed buffer
                    for (i = 0; i < temp_vertex_count; i++) {
                        model->faces.vertex_indices_buffer[buffer_pos++] = temp_indices[i];
                    }
                    model->faces.vertex_count[face_count] = temp_vertex_count;
                    model->faces.total_indices += temp_vertex_count;
                    
                    // Skip faces with zero vertices
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
    
    // Close file
    fclose(file);
    
    model->faces.face_count = face_count;
    
    // Initialize sorted_face_indices with identity mapping (will be sorted later)
    for (i = 0; i < face_count; i++) {
        model->faces.sorted_face_indices[i] = i;
    }
    
    printf("\nReading faces finished : %d faces read.\n", face_count);
    return face_count;
}


/**
 * CALCULATING MINIMUM FACE DEPTHS AND VISIBILITY FLAGS
 * =====================================================
 * 
 * This function calculates for each face:
 * 1. The minimum depth (z_min) of all its vertices in the observer coordinate system
 * 2. The display visibility flag based on vertex positions relative to camera
 * 
 * The z_min value is used for face sorting during rendering (painter's algorithm).

 * We use minimum (closest point) for correct occlusion in the painter's algorithm.
 * The display_flag is used to cull faces that have vertices behind the camera.
 * 
 * PARAMETERS:
 *   vertices   : Array of vertices with coordinates in observer system
 *   faces      : Array of faces to process  
 *   face_count : Number of faces
 * 
 * PER-FACE DEPTH, BBOX, & PLANE COMPUTATION (calculateFaceDepths)
 * ================================================================
 * Purpose:
 *   Compute per-face metrics required by the painters and diagnostics:
 *     - z_min, z_max and z_mean (observer-space depths) used for coarse sorting
 *     - screen-space axis-aligned bounding box (min/max X/Y) used for cheap overlap tests
 *     - planar coefficients (Newell method) A/B/C/D for robust face-plane tests
 *     - `display_flag` indicating basic visibility (behind-camera or culled by plane test)
 *
 * Algorithm summary (per face):
 *   1) Iterate vertices in observer-space (xo/yo/zo). If any vertex has zo <= 0, mark
 *      `display_flag = 0` (non-displayable) to avoid projection artefacts.
 *   2) Accumulate z_min/z_max and compute z_mean (used by initial z-sorting in the painter).
 *   3) Compute the face's axis-aligned bbox from integer screen coords (x2d/y2d) for quick
 *      overlap tests (used by Newell/Sancha tests 2 and 3).
 *   4) Compute planar coefficients (A,B,C,D) using Newell's method; these are used to
 *      perform finer pairwise tests and the optional **observer-space back-face culling**
 *      if `cull_back_faces` is enabled (test D <= 0 indicates a back-face relative to observer).
 *
 * Implementation notes:
 *   - Plane coefficients are kept both in Fixed32 (for fixed-mode painter) and cached/converted
 *     to float buffers for the FLOAT painter mode. Conversion flags track whether a float copy
 *     is available to avoid repeated conversions.
 *   - `display_flag == 1` means the face is eligible for drawing; `0` indicates it is either
 *     behind the camera or culled by the observer-space back-face test when enabled.
 *   - This function must be called AFTER vertex transforms into observer-space (e.g. by
 *     `processModelFast()` or `transformToObserver()`), because it reads xo/yo/zo/x2d/y2d arrays.
 *
 * Performance & debug:
 *   - The function optionally logs culling counts when not in PERFORMANCE_MODE.
 *   - The function is intentionally designed to be O(n) over faces and linear in face vertex counts.
 */
segment "code14";
void calculateFaceDepths(Model3D* model, Face3D* faces, int face_count) {
    int i, j;
    int culled_count = 0; // diagnostic: number of faces culled by back-face test (observer-space)
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* face_arrays = &model->faces;
    
    for (i = 0; i < face_count; i++) {
        Fixed32 z_min = FLOAT_TO_FIXED(9999.0);  // Initialize to very large value (closest)
        Fixed32 z_max = FLOAT_TO_FIXED(-9999.0); // Initialize to very small value (farthest)
        int display_flag = 1;
        Fixed32 sum = 0;
        int n = face_arrays->vertex_count[i];
        int minx = 9999, maxx = -9999, miny = 9999, maxy = -9999;
        
        // Access indices from the packed buffer using the offset
        int offset = face_arrays->vertex_indices_ptr[i];
        for (j = 0; j < n; j++) {
            int vertex_idx = face_arrays->vertex_indices_buffer[offset + j] - 1;
            if (vertex_idx >= 0) {
                Fixed32 zo = vtx->zo[vertex_idx];
                if (zo < 0) display_flag = 0; // strictly behind camera
                if (zo < z_min) z_min = zo;  // Find minimum (closest)
                if (zo > z_max) z_max = zo;  // Find maximum (farthest)
                sum += zo;
                int x2d = vtx->x2d[vertex_idx];
                int y2d = vtx->y2d[vertex_idx];
                if (x2d < minx) minx = x2d;
                if (x2d > maxx) maxx = x2d;
                if (y2d < miny) miny = y2d;
                if (y2d > maxy) maxy = y2d;
            }
        }
        // Compute plane coefficients (a,b,c,d) using only the first 3 vertices (observer space)
        // Formules (Fixed16.16 arithmetic implemented in Fixed64 intermediates):
        // a := y1 * (z2 - z3) + y2 * (z3 - z1) + y3 * (z1 - z2);
        // b := -x1 * (z2 - z3) + x2 * (z1 - z3) - x3 * (z1 - z2);
        // c := x1 * (y2 - y3) - x2 * (y1 - y3) + x3 * (y1 - y2);
        // d := -x1 * (y2 * z3 - y3 * z2) + x2 * (y1 * z3 - y3 * z1) - x3 * (y1 * z2 - y2 * z1);
        Fixed64 a64 = 0, b64 = 0, c64 = 0, d64 = 0;
        if (!display_flag || n < 3) {
            // face is behind camera or degenerate: zero coefficients
            face_arrays->plane_a[i] = 0;
            face_arrays->plane_b[i] = 0;
            face_arrays->plane_c[i] = 0;
            face_arrays->plane_d[i] = 0;
        } else {
            int idx0 = face_arrays->vertex_indices_buffer[offset] - 1;
            int idx1 = face_arrays->vertex_indices_buffer[offset + 1] - 1;
            int idx2 = face_arrays->vertex_indices_buffer[offset + 2] - 1;
            if (idx0 < 0 || idx1 < 0 || idx2 < 0) {
                face_arrays->plane_a[i] = 0;
                face_arrays->plane_b[i] = 0;
                face_arrays->plane_c[i] = 0;
                face_arrays->plane_d[i] = 0;
            } else {
                Fixed32 x1 = vtx->xo[idx0], y1 = vtx->yo[idx0], z1 = vtx->zo[idx0];
                Fixed32 x2 = vtx->xo[idx1], y2 = vtx->yo[idx1], z2 = vtx->zo[idx1];
                Fixed32 x3 = vtx->xo[idx2], y3 = vtx->yo[idx2], z3 = vtx->zo[idx2];

                // Compute a = y1*(z2-z3) + y2*(z3-z1) + y3*(z1-z2) in Fixed64
                Fixed64 term_a1 = (((Fixed64)y1 * (Fixed64)FIXED_SUB(z2, z3)) >> FIXED_SHIFT);
                Fixed64 term_a2 = (((Fixed64)y2 * (Fixed64)FIXED_SUB(z3, z1)) >> FIXED_SHIFT);
                Fixed64 term_a3 = (((Fixed64)y3 * (Fixed64)FIXED_SUB(z1, z2)) >> FIXED_SHIFT);
                a64 = FIXED_ADD(FIXED_ADD(term_a1, term_a2), term_a3);

                // b = -x1*(z2-z3) + x2*(z1-z3) - x3*(z1-z2)
                Fixed64 term_b1 = (((Fixed64)x1 * (Fixed64)FIXED_SUB(z2, z3)) >> FIXED_SHIFT);
                Fixed64 term_b2 = (((Fixed64)x2 * (Fixed64)FIXED_SUB(z1, z3)) >> FIXED_SHIFT);
                Fixed64 term_b3 = (((Fixed64)x3 * (Fixed64)FIXED_SUB(z1, z2)) >> FIXED_SHIFT);
                // b = -term_b1 + term_b2 - term_b3
                b64 = FIXED_SUB(FIXED_ADD(FIXED_NEG(term_b1), term_b2), term_b3);

                // c = x1*(y2-y3) - x2*(y1-y3) + x3*(y1-y2)
                Fixed64 term_c1 = (((Fixed64)x1 * (Fixed64)FIXED_SUB(y2, y3)) >> FIXED_SHIFT);
                Fixed64 term_c2 = (((Fixed64)x2 * (Fixed64)FIXED_SUB(y1, y3)) >> FIXED_SHIFT);
                Fixed64 term_c3 = (((Fixed64)x3 * (Fixed64)FIXED_SUB(y1, y2)) >> FIXED_SHIFT);
                c64 = FIXED_ADD(FIXED_SUB(term_c1, term_c2), term_c3);

                // d = -x1*(y2*z3 - y3*z2) + x2*(y1*z3 - y3*z1) - x3*(y1*z2 - y2*z1)
                Fixed64 t1 = ((((Fixed64)y2 * (Fixed64)z3) - ((Fixed64)y3 * (Fixed64)z2)) >> FIXED_SHIFT);
                Fixed64 t2 = ((((Fixed64)y1 * (Fixed64)z3) - ((Fixed64)y3 * (Fixed64)z1)) >> FIXED_SHIFT);
                Fixed64 t3 = ((((Fixed64)y1 * (Fixed64)z2) - ((Fixed64)y2 * (Fixed64)z1)) >> FIXED_SHIFT);
                Fixed64 term_d1 = (((Fixed64)x1 * t1) >> FIXED_SHIFT);
                Fixed64 term_d2 = (((Fixed64)x2 * t2) >> FIXED_SHIFT);
                Fixed64 term_d3 = (((Fixed64)x3 * t3) >> FIXED_SHIFT);
                d64 = FIXED_SUB(FIXED_ADD(FIXED_NEG(term_d1), term_d2), term_d3);

                face_arrays->plane_a[i] = a64;
                face_arrays->plane_b[i] = b64;
                face_arrays->plane_c[i] = c64;
                face_arrays->plane_d[i] = d64;

                // Optional back-face culling in observer-space: if the plane D term is <= 0,
                // the plane faces away from the observer (origin), so cull the face when enabled.
                if (cull_back_faces && display_flag) {
                    // d64 is stored in face_arrays->plane_d[i]
                    if (d64 <= 0) {
                        display_flag = 0;
                        ++culled_count;
                        // if (!PERFORMANCE_MODE) {
                        //     printf("[DEBUG] CULL: Face %d culled (plane_d=%.6f)\n", i, FIXED64_TO_FLOAT(d64));
                        // }
                    }
                }
            }
        }

        // Diagnostic: report suspiciously negative z_max values to help debug
        // if (!PERFORMANCE_MODE) {
        //     float zmax_f = FIXED_TO_FLOAT(z_max);
        //     if (zmax_f < -100.0f) {
        //         printf("[DEBUG] Face %d: z_min=%.2f z_max=%.2f display_flag=%d n=%d\n", i, FIXED_TO_FLOAT(z_min), zmax_f, display_flag, n);
        //         // Print per-vertex observer-space zo values
        //         printf("[DEBUG]  vertex zo: ");
        //         for (int jj = 0; jj < n; ++jj) {
        //             int vidx = face_arrays->vertex_indices_buffer[offset + jj] - 1;
        //             if (vidx >= 0) printf("(%d: %.2f) ", vidx, FIXED_TO_FLOAT(vtx->zo[vidx]));
        //         }
        //         printf("\n");
        //     }
        // }

        face_arrays->z_min[i] = z_min;  // Store minimum depth for this face (closest)
        face_arrays->z_max[i] = z_max;  // Store maximum depth for this face (farthest)
        face_arrays->display_flag[i] = display_flag;
        if (n > 0) {
            face_arrays->z_mean[i] = sum / n;
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

    // if (cull_back_faces && !PERFORMANCE_MODE) {
    //     printf("[DEBUG] calculateFaceDepths: culled %d faces by back-face test\n", culled_count);
    // }
}



// Dump face plane coefficients and depth stats to CSV
// Columns: face;a;b;c;d;z_min;z_mean;z_max;vertex_indices (or CSV style depending on flag)
segment "code15";
// New signature: last parameter 'alt_format' == 0 -> default (commas and dot decimal)
//                                   == 1 -> use semicolon column separator and comma decimal separator
void dumpFaceEquationsCSV(Model3D* model, const char* csv_filename, int alt_format) {
    if (model == NULL || csv_filename == NULL) return;
    FILE* f = fopen(csv_filename, "w");
    if (!f) {
        printf("Error: cannot open '%s' for writing\n", csv_filename);
        return;
    }
    FaceArrays3D* faces = &model->faces;
    int face_count = faces->face_count;

    char col_sep = alt_format ? ';' : ',';
    int use_comma_decimal = alt_format ? 1 : 0;

    // Determine maximum number of vertices in a face so we can create fixed per-vertex columns
    int max_v = 0;
    for (int ii = 0; ii < face_count; ++ii) {
        if (faces->vertex_count[ii] > max_v) max_v = faces->vertex_count[ii];
    }

    // Header: base columns + per-vertex groups (vN_idx,vN_xo,vN_yo,vN_zo)
    // Use selected column separator
    fprintf(f, "face"); fprintf(f, "%c", col_sep);
    fprintf(f, "a"); fprintf(f, "%c", col_sep);
    fprintf(f, "b"); fprintf(f, "%c", col_sep);
    fprintf(f, "c"); fprintf(f, "%c", col_sep);
    fprintf(f, "d"); fprintf(f, "%c", col_sep);
    fprintf(f, "z_min"); fprintf(f, "%c", col_sep);
    fprintf(f, "z_mean"); fprintf(f, "%c", col_sep);
    fprintf(f, "z_max"); fprintf(f, "%c", col_sep);
    fprintf(f, "vertex_indices");
    for (int k = 0; k < max_v; ++k) {
        fprintf(f, "%c", col_sep);
        fprintf(f, "v%d_idx", k+1);
        fprintf(f, "%c", col_sep);
        fprintf(f, "v%d_xo", k+1);
        fprintf(f, "%c", col_sep);
        fprintf(f, "v%d_yo", k+1);
        fprintf(f, "%c", col_sep);
        fprintf(f, "v%d_zo", k+1);
    }
    fprintf(f, "\n");

    for (int i = 0; i < face_count; ++i) {
        float a = (float)FIXED64_TO_FLOAT(faces->plane_a[i]);
        float b = (float)FIXED64_TO_FLOAT(faces->plane_b[i]);
        float c = (float)FIXED64_TO_FLOAT(faces->plane_c[i]);
        float d = (float)FIXED64_TO_FLOAT(faces->plane_d[i]);
        float zmin = FIXED_TO_FLOAT(faces->z_min[i]);
        float zmean = FIXED_TO_FLOAT(faces->z_mean[i]);
        float zmax = FIXED_TO_FLOAT(faces->z_max[i]);

        char buf[64];
        // face index
        fprintf(f, "%d", i);
        fprintf(f, "%c", col_sep);

        // Helper: print float value respecting decimal separator
        #define PRINTF_FLOAT(val) do { snprintf(buf, sizeof(buf), "%.6f", (double)(val)); if (use_comma_decimal) { for (char *_p = buf; *_p; ++_p) if (*_p == '.') *_p = ','; } fprintf(f, "%s", buf); } while(0)

        PRINTF_FLOAT(a); fprintf(f, "%c", col_sep);
        PRINTF_FLOAT(b); fprintf(f, "%c", col_sep);
        PRINTF_FLOAT(c); fprintf(f, "%c", col_sep);
        PRINTF_FLOAT(d); fprintf(f, "%c", col_sep);
        PRINTF_FLOAT(zmin); fprintf(f, "%c", col_sep);
        PRINTF_FLOAT(zmean); fprintf(f, "%c", col_sep);
        PRINTF_FLOAT(zmax);

        // vertex_indices as quoted field (space-separated)
        fprintf(f, "%c\"", col_sep);
        int offset = faces->vertex_indices_ptr[i];
        int n = faces->vertex_count[i];
        for (int j = 0; j < n; ++j) {
            if (j) fputc(' ', f);
            int vid = faces->vertex_indices_buffer[offset + j]; // keep OBJ 1-based index
            fprintf(f, "%d", vid);
        }
        fprintf(f, "\""); // close indices field

        // Then print fixed per-vertex columns up to max_v: idx,xo,yo,zo (empty if face has fewer vertices)
        for (int k = 0; k < max_v; ++k) {
            if (k < n) {
                int vid = faces->vertex_indices_buffer[offset + k]; // OBJ 1-based
                int vidx = vid - 1;
                if (vidx >= 0) {
                    float xo = FIXED_TO_FLOAT(model->vertices.xo[vidx]);
                    float yo = FIXED_TO_FLOAT(model->vertices.yo[vidx]);
                    float zo = FIXED_TO_FLOAT(model->vertices.zo[vidx]);
                    // idx
                    fprintf(f, "%c%d", col_sep, vid);
                    // xo, yo, zo
                    snprintf(buf, sizeof(buf), "%.6f", (double)xo); if (use_comma_decimal) for (char *_p = buf; *_p; ++_p) if (*_p == '.') *_p = ','; fprintf(f, "%c%s", col_sep, buf);
                    snprintf(buf, sizeof(buf), "%.6f", (double)yo); if (use_comma_decimal) for (char *_p = buf; *_p; ++_p) if (*_p == '.') *_p = ','; fprintf(f, "%c%s", col_sep, buf);
                    snprintf(buf, sizeof(buf), "%.6f", (double)zo); if (use_comma_decimal) for (char *_p = buf; *_p; ++_p) if (*_p == '.') *_p = ','; fprintf(f, "%c%s", col_sep, buf);
                } else {
                    // index present but coordinates missing
                    fprintf(f, "%c%d%c%c%c", col_sep, vid, col_sep, col_sep, col_sep);
                }
            } else {
                // no vertex: write four empty CSV fields
                fprintf(f, "%c%c%c%c", col_sep, col_sep, col_sep, col_sep);
            }
        }
        fprintf(f, "\n");
    }
    fclose(f);
    printf("Wrote face equations to %s (%d faces)\n", csv_filename, face_count);
}

// Helper macro to swap face indices in the sorted_face_indices array
// (We swap indices, not the faces themselves, to keep the buffer intact)
#define SWAP_FACE(faces, i, j) \
    do { \
        int temp_idx = faces->sorted_face_indices[i]; \
        faces->sorted_face_indices[i] = faces->sorted_face_indices[j]; \
        faces->sorted_face_indices[j] = temp_idx; \
    } while (0)


// Draw a single face (by face index) using the same QuickDraw path as drawPolygons
// - respects face validity and display_flag
// - uses globalPolyHandle and poly_handle_locked like drawPolygons
segment "code16";
// - honors framePolyOnly for frame-only rendering
void drawFace(Model3D* model, int face_id, int fillPenPat, int show_index) {
    if (!model) return;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    if (face_id < 0 || face_id >= faces->face_count) return;
    if (faces->display_flag[face_id] == 0) return;
    int vcount_face = faces->vertex_count[face_id];
    if (vcount_face < 3) return;

    int offset = faces->vertex_indices_ptr[face_id];
    int *indices_base = &faces->vertex_indices_buffer[offset];

    // Quick validity pass
    for (int j = 0; j < vcount_face; ++j) {
        int vi = indices_base[j] - 1;
        if (vi < 0 || vi >= vtx->vertex_count) return; // invalid face -> nothing to draw
    }

    // Ensure global handle exists
    if (globalPolyHandle == NULL) {
        int max_polySize = 2 + 8 + (4 * 4);  // Max for quad (4 vertices)
        globalPolyHandle = NewHandle((long)max_polySize, userid(), 0xC014, 0L);
        if (globalPolyHandle == NULL) {
            printf("Error: Unable to allocate global polygon handle\n");
            return;
        }
    }

    Handle polyHandle = globalPolyHandle;
    if (poly_handle_locked) { HUnlock(polyHandle); poly_handle_locked = 0; }
    HLock(polyHandle); poly_handle_locked = 1;

    DynamicPolygon *poly = (DynamicPolygon *)*polyHandle;
    int polySize = 2 + 8 + (vcount_face * 4);
    poly->polySize = polySize;

    int *x2d = vtx->x2d;
    int *y2d = vtx->y2d;

    // Initialize first point
    int first_vi = indices_base[0] - 1;
    int px = x2d[first_vi];
    int py = y2d[first_vi];
    poly->polyPoints[0].h = mode / 320 * (px + pan_dx);
    poly->polyPoints[0].v = py + pan_dy;
    int min_x = px, max_x = px, min_y = py, max_y = py;

    for (int j = 1; j < vcount_face; ++j) {
        int vi = indices_base[j] - 1;
        px = x2d[vi];
        py = y2d[vi];
        poly->polyPoints[j].h = mode / 320 * (px + pan_dx);
        poly->polyPoints[j].v = py + pan_dy;
        if (px < min_x) min_x = px;
        if (px > max_x) max_x = px;
        if (py < min_y) min_y = py;
        if (py > max_y) max_y = py;
    }

    poly->polyBBox.h1 = min_x + pan_dx;
    poly->polyBBox.v1 = min_y + pan_dy;
    poly->polyBBox.h2 = max_x + pan_dx;
    poly->polyBBox.v2 = max_y + pan_dy;

    int screenScale = mode / 320;
    int screenW = screenScale * (CENTRE_X * 2);
    int screenH = screenScale * (CENTRE_Y * 2);

    int sc_min_x = screenScale * (min_x + pan_dx);
    int sc_max_x = screenScale * (max_x + pan_dx);
    int sc_min_y = (min_y + pan_dy);
    int sc_max_y = (max_y + pan_dy);
    if (sc_max_x < 0 || sc_min_x >= screenW || sc_max_y < 0 || sc_min_y >= screenH) {
        // Off-screen; nothing to draw
    } else {
        if (fillPenPat >= 0) {
            SetSolidPenPat(fillPenPat);
            Pattern pat;
            GetPenPat(pat);
            FillPoly(polyHandle, pat);
            SetSolidPenPat(7);
            FramePoly(polyHandle);
            SetSolidPenPat(14);
        } else if (framePolyOnly) {
            /* Draw explicit solid wireframe using MoveTo/LineTo to avoid dotted
             * outlines produced by FramePoly in certain pen/pattern states. */
            SetPenMode(0); // PenMode = Copy ==> avoid dotted lines
            SetSolidPenPat(7);
            FramePoly(polyHandle);
            // int first_h = poly->polyPoints[0].h;
            // int first_v = poly->polyPoints[0].v;
            // MoveTo(first_h, first_v);
            // for (int kk = 1; kk < vcount_face; ++kk) {
            //     LineTo(poly->polyPoints[kk].h, poly->polyPoints[kk].v);
            // }
            // LineTo(first_h, first_v);
            SetSolidPenPat(14);
        } else {
            Pattern pat;
            GetPenPat(pat);
            FillPoly(polyHandle, pat);
            SetSolidPenPat(7);
            FramePoly(polyHandle);
            SetSolidPenPat(14);
        }

        // Draw face index centered in the polygon using QuickDraw MoveTo + DrawString
        if (show_index) {
            int center_x = (min_x + max_x) / 2;
            int center_y = (min_y + max_y) / 2;
            // Use same transformation as polygon points: horizontal scaled, vertical and pan applied
            int screenCx = screenScale * (center_x + pan_dx);
            int screenCy = center_y + pan_dy;

            char tmp[32];
            int len = snprintf(tmp, sizeof(tmp), "%d", face_id);
            if (len > 15) len = 15; // fit into Pascal string buffer
            unsigned char pstr[16];
            pstr[0] = (unsigned char)len;
            memcpy(&pstr[1], tmp, len);

            MoveTo(screenCx, screenCy);
            DrawString(pstr);
        }
    }
}


segment "code17";
// Function to draw polygons with QuickDraw
void drawPolygons(Model3D* model, int* vertex_count, int face_count, int vertex_count_total) {
    int i, j;
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    Handle polyHandle;
    DynamicPolygon *poly;
    int min_x, max_x, min_y, max_y;
    int valid_faces_drawn = 0;
    int invalid_faces_skipped = 0;
    int triangle_count = 0;
    int quad_count = 0;
    Pattern pat;
    
    // Precompute screen scale and screen bounds for culling
    int screenScale = mode / 320;
    int screenW = screenScale * (CENTRE_X * 2);
    int screenH = screenScale * (CENTRE_Y * 2);

    // Use global persistent handle to avoid repeated NewHandle/DisposeHandle
    // Each call allocates fresh if needed, but reuses same handle block
    if (globalPolyHandle == NULL) {
        int max_polySize = 2 + 8 + (MAX_FACE_VERTICES * 4);  // Max for MAX_FACE_VERTICES vertices
        globalPolyHandle = NewHandle((long)max_polySize, userid(), 0xC014, 0L);
        if (globalPolyHandle == NULL) {
            printf("Error: Unable to allocate global polygon handle\n");
            return;
        }
    }
    
    polyHandle = globalPolyHandle;
    
    // Make sure handle is unlocked before locking
    if (poly_handle_locked) { 
        HUnlock(polyHandle); 
        poly_handle_locked = 0; 
    }
    HLock(polyHandle); 
    poly_handle_locked = 1;

    SetPenMode(0);
    // printf("\nDrawing polygons on screen:\n");
    // printf("\nDrawing polygons on screen:\n");

    // Set fill pen once per frame (reduces state changes)
    SetSolidPenPat(14);

    // Use sorted_face_indices to draw in correct depth order
    // Draw ALL faces - painter's algorithm handles occlusion
    int start_face = 0;
    int max_faces_to_draw = face_count;
    for (i = start_face; i < start_face + max_faces_to_draw; i++) {
        int face_id = faces->sorted_face_indices[i];
        if (faces->display_flag[face_id] == 0) continue;
        if (faces->vertex_count[face_id] >= 3) {
            int offset = faces->vertex_indices_ptr[face_id];
            int vcount_face = faces->vertex_count[face_id];
            int *indices_base = &faces->vertex_indices_buffer[offset];

            // Quick validity pass: ensure all indices are valid to avoid undefined points
            int all_valid = 1;
            for (j = 0; j < vcount_face; ++j) {
                int vi = indices_base[j] - 1;
                if (vi < 0 || vi >= vtx->vertex_count) { all_valid = 0; break; }
            }
            if (!all_valid) { invalid_faces_skipped++; continue; }

            // Calculate polySize for this specific face
            int polySize = 2 + 8 + (vcount_face * 4);
            poly = (DynamicPolygon *)*polyHandle;
            poly->polySize = polySize;

            // Cache arrays
            int *x2d = vtx->x2d;
            int *y2d = vtx->y2d;

            // Initialize min/max with the first vertex to avoid sentinel checks
            int first_vi = indices_base[0] - 1;
            int x = x2d[first_vi];
            int y = y2d[first_vi];
            poly->polyPoints[0].h = screenScale * (x + pan_dx);
            poly->polyPoints[0].v = y + pan_dy;
            min_x = max_x = x;
            min_y = max_y = y;

            // Fill remaining points using cached data
            for (j = 1; j < vcount_face; ++j) {
                int vi = indices_base[j] - 1;
                x = x2d[vi];
                y = y2d[vi];
                poly->polyPoints[j].h = screenScale * (x + pan_dx);
                poly->polyPoints[j].v = y + pan_dy;
                if (x < min_x) min_x = x;
                if (x > max_x) max_x = x;
                if (y < min_y) min_y = y;
                if (y > max_y) max_y = y;
            }

            poly->polyBBox.h1 = min_x + pan_dx;
            poly->polyBBox.v1 = min_y + pan_dy;
            poly->polyBBox.h2 = max_x + pan_dx;
            poly->polyBBox.v2 = max_y + pan_dy;

            // Bounding-box culling (convert to screen pixels, apply pan)
            int sc_min_x = screenScale * (min_x + pan_dx);
            int sc_max_x = screenScale * (max_x + pan_dx);
            int sc_min_y = (min_y + pan_dy);
            int sc_max_y = (max_y + pan_dy);
            if (sc_max_x < 0 || sc_min_x >= screenW || sc_max_y < 0 || sc_min_y >= screenH) {
                // Off-screen; skip drawing
            } else {
                if (framePolyOnly) {
                    // Frame-only rendering (no fill)
                    int frame_color;
                    if (user_frame_color == 17) {
                        // Same as fill color
                        if (user_fill_color == 16 && random_fill_colors != NULL && face_id < random_colors_capacity) {
                            frame_color = random_fill_colors[face_id];
                        } else if (user_fill_color >= 0) {
                            frame_color = user_fill_color;
                        } else {
                            frame_color = 14; // default fill color
                        }
                    } else if (user_frame_color == 16 && random_frame_colors != NULL && face_id < random_colors_capacity) {
                        frame_color = random_frame_colors[face_id];
                    } else if (user_frame_color >= 0) {
                        frame_color = user_frame_color;
                    } else {
                        frame_color = 7;
                    }
                    SetSolidPenPat(frame_color);
                    FramePoly(polyHandle);
                    SetSolidPenPat(14); // keep fill pen as default for next faces
                } else {
                    // Fill color
                    int fill_color;
                    if (user_fill_color == 16 && random_fill_colors != NULL && face_id < random_colors_capacity) {
                        fill_color = random_fill_colors[face_id];
                    } else if (user_fill_color >= 0) {
                        fill_color = user_fill_color;
                    } else {
                        fill_color = 14;
                    }
                    SetSolidPenPat(fill_color);
                    GetPenPat(pat);
                    FillPoly(polyHandle, pat);
                    
                    // Frame color
                    int frame_color;
                    if (user_frame_color == 17) {
                        // Same as fill color
                        frame_color = fill_color;
                    } else if (user_frame_color == 16 && random_frame_colors != NULL && face_id < random_colors_capacity) {
                        frame_color = random_frame_colors[face_id];
                    } else if (user_frame_color >= 0) {
                        frame_color = user_frame_color;
                    } else {
                        frame_color = 7;
                    }
                    SetSolidPenPat(frame_color);
                    FramePoly(polyHandle);
                    SetSolidPenPat(14); // restore fill pen
                }
                valid_faces_drawn++;
                if (vcount_face == 3) triangle_count++;
                else if (vcount_face == 4) quad_count++;
            }
        } else {
            invalid_faces_skipped++;
        }
    }
    // Print statistics after drawing
//     printf("Display statistics: %d valid faces drawn, %d invalid faces skipped\n", valid_faces_drawn, invalid_faces_skipped);
//     printf("Triangles: %d, Quads: %d\n", triangle_count, quad_count);
}

#if 0
// drawPolygons_jitter: same behavior as drawPolygons but applies a small random
// pixel offset (0..jitter_max) to each vertex 2D coordinate prior to drawing.
// This function is intentionally a copy of drawPolygons() with the per-vertex
// jitter applied when filling the QuickDraw polygon points. It keeps bbox
// culling decisions unchanged (bbox uses unjittered coords) to avoid spurious
// off-screen skips.
void drawPolygons_jitter(Model3D* model, int* vertex_count, int face_count, int vertex_count_total) {
    int i, j;
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    Handle polyHandle;
    DynamicPolygon *poly;
    int min_x, max_x, min_y, max_y;
    int valid_faces_drawn = 0;
    int invalid_faces_skipped = 0;
    int triangle_count = 0;
    int quad_count = 0;
    Pattern pat;
    
    // Precompute screen scale and screen bounds for culling
    int screenScale = mode / 320;
    int screenW = screenScale * (CENTRE_X * 2);
    int screenH = screenScale * (CENTRE_Y * 2);

    if (globalPolyHandle == NULL) {
        int max_polySize = 2 + 8 + (MAX_FACE_VERTICES * 4);
        globalPolyHandle = NewHandle((long)max_polySize, userid(), 0xC014, 0L);
        if (globalPolyHandle == NULL) {
            printf("Error: Unable to allocate global polygon handle\n");
            return;
        }
    }
    
    polyHandle = globalPolyHandle;
    
    if (poly_handle_locked) { 
        HUnlock(polyHandle); 
        poly_handle_locked = 0; 
    }
    HLock(polyHandle); 
    poly_handle_locked = 1;

    SetPenMode(0);
    SetSolidPenPat(14);

    int start_face = 0;
    int max_faces_to_draw = face_count;
    for (i = start_face; i < start_face + max_faces_to_draw; i++) {
        int face_id = faces->sorted_face_indices[i];
        if (faces->display_flag[face_id] == 0) continue;
        if (faces->vertex_count[face_id] >= 3) {
            int offset = faces->vertex_indices_ptr[face_id];
            int vcount_face = faces->vertex_count[face_id];
            int *indices_base = &faces->vertex_indices_buffer[offset];

            int all_valid = 1;
            for (j = 0; j < vcount_face; ++j) {
                int vi = indices_base[j] - 1;
                if (vi < 0 || vi >= vtx->vertex_count) { all_valid = 0; break; }
            }
            if (!all_valid) { invalid_faces_skipped++; continue; }

            int polySize = 2 + 8 + (vcount_face * 4);
            poly = (DynamicPolygon *)*polyHandle;
            poly->polySize = polySize;

            int *x2d = vtx->x2d;
            int *y2d = vtx->y2d;

            int first_vi = indices_base[0] - 1;
            int x = x2d[first_vi];
            int y = y2d[first_vi];
            int jx = rand() % (jitter_max + 1);
            int jy = rand() % (jitter_max + 1);
            poly->polyPoints[0].h = screenScale * (x + pan_dx) + jx;
            poly->polyPoints[0].v = y + pan_dy + jy;
            min_x = max_x = x;
            min_y = max_y = y;

            for (j = 1; j < vcount_face; ++j) {
                int vi = indices_base[j] - 1;
                x = x2d[vi];
                y = y2d[vi];
                jx = rand() % (jitter_max + 1);
                jy = rand() % (jitter_max + 1);
                poly->polyPoints[j].h = screenScale * (x + pan_dx) + jx;
                poly->polyPoints[j].v = y + pan_dy + jy;
                if (x < min_x) min_x = x;
                if (x > max_x) max_x = x;
                if (y < min_y) min_y = y;
                if (y > max_y) max_y = y;
            }

            poly->polyBBox.h1 = min_x + pan_dx;
            poly->polyBBox.v1 = min_y + pan_dy;
            poly->polyBBox.h2 = max_x + pan_dx;
            poly->polyBBox.v2 = max_y + pan_dy;

            int sc_min_x = screenScale * (min_x + pan_dx);
            int sc_max_x = screenScale * (max_x + pan_dx);
            int sc_min_y = (min_y + pan_dy);
            int sc_max_y = (max_y + pan_dy);
            if (sc_max_x < 0 || sc_min_x >= screenW || sc_max_y < 0 || sc_min_y >= screenH) {
                // Off-screen; skip drawing
            } else {
                if (framePolyOnly) {
                    int frame_color;
                    if (user_frame_color == 17) {
                        if (user_fill_color == 16 && random_fill_colors != NULL && face_id < random_colors_capacity) {
                            frame_color = random_fill_colors[face_id];
                        } else if (user_fill_color >= 0) {
                            frame_color = user_fill_color;
                        } else {
                            frame_color = 14; // default fill color
                        }
                    } else if (user_frame_color == 16 && random_frame_colors != NULL && face_id < random_colors_capacity) {
                        frame_color = random_frame_colors[face_id];
                    } else if (user_frame_color >= 0) {
                        frame_color = user_frame_color;
                    } else {
                        frame_color = 7;
                    }
                    SetSolidPenPat(frame_color);
                    FramePoly(polyHandle);
                    SetSolidPenPat(14); // keep fill pen as default for next faces
                } else {
                    int fill_color;
                    if (user_fill_color == 16 && random_fill_colors != NULL && face_id < random_colors_capacity) {
                        fill_color = random_fill_colors[face_id];
                    } else if (user_fill_color >= 0) {
                        fill_color = user_fill_color;
                    } else {
                        fill_color = 14;
                    }
                    SetSolidPenPat(fill_color);
                    GetPenPat(pat);
                    FillPoly(polyHandle, pat);
                    
                    int frame_color;
                    if (user_frame_color == 17) {
                        // Same as fill color
                        frame_color = fill_color;
                    } else if (user_frame_color == 16 && random_frame_colors != NULL && face_id < random_colors_capacity) {
                        frame_color = random_frame_colors[face_id];
                    } else if (user_frame_color >= 0) {
                        frame_color = user_frame_color;
                    } else {
                        frame_color = 7;
                    }
                    SetSolidPenPat(frame_color);
                    FramePoly(polyHandle);
                    SetSolidPenPat(14); // restore fill pen
                }
                valid_faces_drawn++;
                if (vcount_face == 3) triangle_count++;
                else if (vcount_face == 4) quad_count++;
            }
        } else {
            invalid_faces_skipped++;
        }
    }
    // Print statistics after drawing
//     printf("Display statistics: %d valid faces drawn, %d invalid faces skipped\n", valid_faces_drawn, invalid_faces_skipped);
//     printf("Triangles: %d, Quads: %d\n", triangle_count, quad_count);
#endif

segment "code18";

// Generate random colors for all faces
void generate_random_colors(int face_count) {
    if (face_count <= 0) return;
    
    // Ensure capacity
    if (random_colors_capacity < face_count) {
        random_fill_colors = (unsigned char*)realloc(random_fill_colors, face_count);
        random_frame_colors = (unsigned char*)realloc(random_frame_colors, face_count);
        random_colors_capacity = face_count;
    }
    
    // Generate random colors (1-15, skip 0 which is black background)
    for (int i = 0; i < face_count; i++) {
        random_fill_colors[i] = (rand() % 15) + 1;
        random_frame_colors[i] = (rand() % 15) + 1;
    }
}

// drawPolygons_jitter: same behavior as drawPolygons but applies a small random
// pixel offset (0..jitter_max) to each vertex 2D coordinate prior to drawing.
// This function mirrors `drawPolygons()` but applies per-vertex jitter at render time.
void drawPolygons_jitter(Model3D* model, int* vertex_count, int face_count, int vertex_count_total) {
    if (!model) return;
    VertexArrays3D* vtx = &model->vertices;

    int vcount = vertex_count_total;
    if (vcount <= 0) { drawPolygons(model, vertex_count, face_count, vertex_count_total); return; }

    // Backup original coordinates
    int *saved_x = (int*)malloc(vcount * sizeof(int));
    int *saved_y = (int*)malloc(vcount * sizeof(int));
    if (!saved_x || !saved_y) {
        if (saved_x) free(saved_x);
        if (saved_y) free(saved_y);
        // Fallback to normal draw
        drawPolygons(model, vertex_count, face_count, vertex_count_total);
        return;
    }

    for (int i = 0; i < vcount; ++i) {
        saved_x[i] = vtx->x2d[i];
        saved_y[i] = vtx->y2d[i];
        int dx = rand() % (jitter_max + 1); // 0..jitter_max
        int dy = rand() % (jitter_max + 1);
        vtx->x2d[i] = saved_x[i] + dx;
        vtx->y2d[i] = saved_y[i] + dy;
    }

    // Draw using modified coordinates
    drawPolygons(model, vertex_count, face_count, vertex_count_total);

    // Restore original coordinates
    for (int i = 0; i < vcount; ++i) {
        vtx->x2d[i] = saved_x[i];
        vtx->y2d[i] = saved_y[i];
    }

    free(saved_x);
    free(saved_y);
}// Diagnostic: Frame in white all polygons listed in `inconclusive_pairs`.
//
// This helper iterates the recorded ambiguous pairs and draws a white outline around both
// polygons in each pair so developers can visually inspect cases where the painter could
// not conclusively determine a stable order. It is a pure rendering aid (no state changes
// beyond drawing) and is invoked only in diagnostic/debug modes or on explicit user request.
void frameInconclusivePairs(Model3D* model) {
    if (!model) return;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    if (inconclusive_pairs == NULL || inconclusive_pairs_count == 0) return;

    // Ensure global handle exists (allocate based on maximum supported vertices)
    if (globalPolyHandle == NULL) {
        int max_polySize = 2 + 8 + (MAX_FACE_VERTICES * 4);
        globalPolyHandle = NewHandle((long)max_polySize, userid(), 0xC014, 0L);
        if (globalPolyHandle == NULL) {
            printf("Error: Unable to allocate global polygon handle\n");
            return;
        }
    }

    Handle polyHandle = globalPolyHandle;
    if (poly_handle_locked) { HUnlock(polyHandle); poly_handle_locked = 0; }
    HLock(polyHandle); poly_handle_locked = 1;

    DynamicPolygon *poly = (DynamicPolygon *)*polyHandle;

    // Iterate the inconclusive pairs and frame both faces (face1 and face2)
    for (int p = 0; p < inconclusive_pairs_count; ++p) {
        int pair_faces[2] = { inconclusive_pairs[p].face1, inconclusive_pairs[p].face2 };
        for (int pf = 0; pf < 2; ++pf) {
            int face_id = pair_faces[pf];
            if (face_id < 0 || face_id >= faces->face_count) continue;
            if (faces->display_flag[face_id] == 0) continue;
            int vcount_face = faces->vertex_count[face_id];
            if (vcount_face < 3 || vcount_face > MAX_FACE_VERTICES) continue;

            int offset = faces->vertex_indices_ptr[face_id];
            int *indices_base = &faces->vertex_indices_buffer[offset];

            // Quick validity pass
            int all_valid = 1;
            for (int j = 0; j < vcount_face; ++j) {
                int vi = indices_base[j] - 1;
                if (vi < 0 || vi >= vtx->vertex_count) { all_valid = 0; break; }
            }
            if (!all_valid) continue;

            // Build polygon using the same conventions as drawPolygons (screenScale applied to X)
            int screenScale = mode / 320;
            int polySize = 2 + 8 + (vcount_face * 4);
            poly->polySize = polySize;

            int first_vi = indices_base[0] - 1;
            int px = vtx->x2d[first_vi];
            int py = vtx->y2d[first_vi];
            poly->polyPoints[0].h = screenScale * (px + pan_dx);
            poly->polyPoints[0].v = py + pan_dy;
            int min_x = px, max_x = px, min_y = py, max_y = py;

            for (int j = 1; j < vcount_face; ++j) {
                int vi = indices_base[j] - 1;
                px = vtx->x2d[vi];
                py = vtx->y2d[vi];
                poly->polyPoints[j].h = screenScale * (px + pan_dx);
                poly->polyPoints[j].v = py + pan_dy;
                if (px < min_x) min_x = px;
                if (px > max_x) max_x = px;
                if (py < min_y) min_y = py;
                if (py > max_y) max_y = py;
            }

            poly->polyBBox.h1 = min_x;
            poly->polyBBox.v1 = min_y;
            poly->polyBBox.h2 = max_x;
            poly->polyBBox.v2 = max_y;

            // Bounding-box culling (convert to screen pixels)
            int screenW = screenScale * (CENTRE_X * 2);
            int screenH = screenScale * (CENTRE_Y * 2);
            int sc_min_x = screenScale * min_x;
            int sc_max_x = screenScale * max_x;
            int sc_min_y = screenScale * min_y;
            int sc_max_y = screenScale * max_y;
            if (sc_max_x < 0 || sc_min_x >= screenW || sc_max_y < 0 || sc_min_y >= screenH) continue;

            // Frame polygon in white (use same pen as drawPolygons framing)
            SetSolidPenPat(10);
            Pattern pat;
            GetPenPat(pat);
            FillPoly(polyHandle, pat);
            SetSolidPenPat(9);
            FramePoly(polyHandle);
        }
    }

    // Keep handle locked (consistent with drawing code)
}

// Simple helper: compute 2D projected coordinates from observer-space coords and a projection scale
segment "code19";
// Minimal version taking only Model3D* and angle_w (uses global projection scale)
// - model: model containing vertices and x2d/y2d output arrays
// - angle_w: final 2D rotation angle (degrees)
// This function performs no checks and is purposely minimal as requested.
void compute2DFromObserver(Model3D* model, int angle_w) {
    VertexArrays3D* vtx = &model->vertices;
    int vcount = vtx->vertex_count;
    Fixed32* xo = vtx->xo; Fixed32* yo = vtx->yo; Fixed32* zo = vtx->zo;
    int* x2d_out = vtx->x2d; int* y2d_out = vtx->y2d;
    Fixed32 scale = s_global_proj_scale_fixed;
    // Update model's stored projection scale to reflect the scale used
    model->auto_proj_scale = scale;

    const Fixed32 centre_x_f = INT_TO_FIXED(CENTRE_X);
    const Fixed32 centre_y_f = INT_TO_FIXED(CENTRE_Y);
    Fixed32 cos_w = cos_deg_int(angle_w);
    Fixed32 sin_w = sin_deg_int(angle_w);
    for (int i = 0; i < vcount; ++i) {
        Fixed32 xo_i = xo[i];
        Fixed32 yo_i = yo[i];
        Fixed32 zo_i = zo[i];
        Fixed32 inv_zo = FIXED_DIV_64(scale, zo_i);
        Fixed32 x2d_temp = FIXED_ADD(FIXED_MUL_64(xo_i, inv_zo), centre_x_f);
        Fixed32 y2d_temp = FIXED_SUB(centre_y_f, FIXED_MUL_64(yo_i, inv_zo));
        Fixed32 rx = FIXED_ADD(FIXED_SUB(FIXED_MUL_64(cos_w, FIXED_SUB(x2d_temp, centre_x_f)), FIXED_MUL_64(sin_w, FIXED_SUB(centre_y_f, y2d_temp))), centre_x_f);
        Fixed32 ry = FIXED_SUB(centre_y_f, FIXED_ADD(FIXED_MUL_64(sin_w, FIXED_SUB(x2d_temp, centre_x_f)), FIXED_MUL_64(cos_w, FIXED_SUB(centre_y_f, y2d_temp))));
        x2d_out[i] = FIXED_ROUND_TO_INT(rx);
        y2d_out[i] = FIXED_ROUND_TO_INT(ry);
    }
}

segment "code20";
void DoColor() {
        Rect r;
        unsigned char pstr[4];  // Pascal string: [length][characters...]]

        SetRect (&r, 0, 1, mode / 320 *10, 11);
        for (int i = 0; i < 16; i++) {
            SetSolidPenPat(i);
            PaintRect(&r);

            if (i == 0) {
                SetSolidPenPat(15); // White frame for black background
                FrameRect(&r);
            }

            MoveTo(r.h1, r.v2+10);
            // Create a Pascal string to display the number
            if (i < 10) {
                pstr[0] = 1;           // Length: 1 character
                pstr[1] = '0' + i;     // Digit 0-9
            } else {
                pstr[0] = 2;           // Length: 2 characters
                pstr[1] = '0' + (i / 10);      // Tens (1 for 10-15)
                pstr[2] = '0' + (i % 10);      // Units (0-5 for 10-15)
            }
            DrawString(pstr);
            OffsetRect(&r, 20, 0);
        }
}

segment "code21";
void DoText() {
        shroff();
        putchar((char) 12); // Clear screen    
}

/* show_help_pager
 * ----------------
 * Display the interactive help menu in pages limited to 20 lines each.
 * The header is repeated at the top of every page. The user can press
 * SPACE to continue to the next page or 'Q'/'q' to quit the help early.
 */
static void show_help_pager(void) {
    const char* header[] = {
        "===================================",
        "    HELP - Keyboard Controller",
        "===================================",
        "" /* blank line */
    };
    const int header_count = sizeof(header)/sizeof(header[0]);

    const char* lines[] = {
        "Space: Display model info",
        "A/Z: Increase/Decrease distance",
        "+/-: Increase/Decrease projection scale (pixels per projected unit)",
        "K: Edit angles/distance (ENTER will apply suggested auto-fit if available)",
        "Arrow Left/Right: Decrease/Increase horizontal angle",
        "Arrow Up/Down: Increase/Decrease vertical angle",
        "W/X: Increase/Decrease screen rotation angle",
        "C: Toggle color palette display",
        "J: Toggle jittered rendering (stylized per-vertex 0..7 px offset)",
        "G: Global overlap scan (writes ovscan.csv and ovscanrpt.txt)",
        "1: Set painter to FAST (simple face sorting only)",
        "2: Set painter to NORMAL (NEWELL_SANCHAV1, Fixed32/64)",
        "3: Set painter to NEWELL_SANCHAV2 (painter_newell_sanchaV2)",
        "U: Set painter to FLOAT (float-based)",
        "4: Set painter to CORRECT (painter_correct) - tries to fix ordering by local moves",
        "6: Set both colors to RANDOM mode",
        "7: Choose fill color (0-15, 16=random, -1=default)",
        "8: Choose frame color (0-15, 16=random, -1=default)",
        "9: Reset colors to defaults (fill=14, frame=7)",
        "P: Toggle frame-only polygons (default: OFF)",
        "B: Toggle back-face culling (observer-space D<=0)",
        "I: Toggle display of inconclusive face pairs",
        "A: Scan all overlapping pairs (writes overlapall.csv and overlap.csv; temporarily disables 'J' jitter for deterministic output)",
        "<: Check sort with ray_cast (verify ordering for overlapping bboxes)",
        "V: Show single face (arrows to navigate, any key to exit)",
        "D: Inspect faces BEFORE selected (orange) - Press 'A' for all or 'O' for overlaps",
        "S: Inspect faces AFTER selected (pink) - Press 'A' for all or 'O' for overlaps",
        "O: Check projected polygon overlap for two faces",
        "E/e: Pan left (2D screen offset, 5 px)",
        "R/r: Pan right (2D screen offset, 5 px)",
        "T/t: Pan up (2D screen offset, 5 px)",
        "Y/y: Pan down (2D screen offset, 5 px)",
        "0: Reset pan to (0,0)",
        "L: Show model with face IDs centered on each face (label mode)",
        "F: Dump face equations to equ.csv (debug)",
        "N: Load new model",
        "H: Display this help message",
        "ESC: Quit program"
    };
    int n = sizeof(lines)/sizeof(lines[0]);

    const int max_lines = 20; // page height limit
    int content_per_page = max_lines - header_count;
    if (content_per_page <= 0) content_per_page = 1;

    int pos = 0;
    while (pos < n) {
        // Print header
        for (int h = 0; h < header_count; ++h) printf("%s\n", header[h]);
        // Print a page of content
        int end = pos + content_per_page;
        for (int i = pos; i < end && i < n; ++i) printf("%s\n", lines[i]);

        // If we're at the end, wait for any key and return
        if (end >= n) {
            printf("\nPress any key to return...\n");
            keypress();
            return;
        }

        // Not the last page: prompt for next action
        printf("\nPress 'Q' pour quitter, any other key to continue...\n");
        char key;
        asm 
            {
            sep #0x20
        loop:
            lda >0xC000     // Read the keyboard status from memory address 0xC000
            bpl loop        // Wait until no key is pressed (= until bit 7 on)
            and #0x007f     // Clear the high bit
            sta >0xC010     // Clear the keypress by writing to 0xC010
            sta key         // Store the key code in variable 'key'
            rep #0x30
            }

        
        if (key == 'Q' || key == 'q') return;
        // any other key (including SPACE) continues
        pos = end;
        // clear a separating line between pages
        DoText(); // clear screen and home cursor... Don't know why.
        printf("\n");
    }
}


// ==============================================================
// THIS IS THE MAIN PROGRAM
// ==============================================================
//
segment "code22";
    int main(int argc, char** argv) {
        Model3D* model;
        ObserverParams params;
        char filename[100];
        char input[50];
        int colorpalette = 0; // default color palette
        int last_process_time_start = 0;
        int last_process_time_end = 0;
        int show_inconclusive = 0; // toggle: display inconclusive pair overlays (press 'i' to toggle)

        /* Command-line test hook: --run-segs-test */
        if (argc > 1 && strcmp(argv[1], "--run-segs-test") == 0) {
            int failed = segs_intersect_unit_tests();
            return (failed == 0) ? 0 : 1;
        }

    newmodel:
        printf("===================================\n");
        printf("       3D OBJ file explorer\n");
        printf("===================================\n\n");
        printf("A tribute to Robert DONY\n");
        printf("Author of \"Calcul des parties cachees\" (Masson, 1986)\n\n");

        // Creer le modele 3D
        model = createModel3D();
        if (model == NULL) {
            printf("Error: Unable to allocate memory for 3D model\n");
            printf("Press any key to quit...\n");
            keypress();
            return 1;
        }

        /* Initialize global projection scale to a sensible default (pixels per projected unit) */
        s_global_proj_scale_fixed = INT_TO_FIXED(100);

        // Initialize inconclusive pairs counter
        inconclusive_pairs_count = 0; // clear inconclusive pairs


        /* Optional: enable float painter to reproduce Windows numeric behaviour exactly via env var USE_FLOAT_PAINTER=1 */
        // {
        //     const char* tmp = getenv("USE_FLOAT_PAINTER");
        //     if (tmp && atoi(tmp) != 0) {
        //         use_float_painter = 1;
        //         painter_mode = PAINTER_MODE_FLOAT;
        //     }
        // }

        // Ask for filename (loop until a non-empty filename is entered and the model loads)
        while (1) {
            printf("Enter the filename to read (ENTER to exit): ");
            if (fgets(filename, sizeof(filename), stdin) != NULL) {
                printf("nom de fichier = %s\n", filename);
                size_t len = strlen(filename);
                if (len > 0 && filename[len-1] == '\n') {
                    filename[len-1] = '\0';
                }
            } else {
                // EOF or input error - exit gracefully
                printf("\nInput error or EOF. Exiting.\n");
                destroyModel3D(model);
                return 1;
            }

            if (filename[0] == '\0') {
                printf("No filename entered. Exiting.\n");
                destroyModel3D(model);
                return 0; // Exit program when user presses ENTER with empty filename
            }

            // Try to load the model; if it fails, inform the user, reset model state, and re-prompt
            if (loadModel3D(model, filename) < 0) {
                // printf("\nError loading file '%s'. Please try again.\n", filename);
                // Destroy and recreate model to ensure clean state for next attempt
                destroyModel3D(model);
                model = createModel3D();
                if (model == NULL) {
                    printf("Error: Unable to allocate memory for 3D model after failed load. Exiting.\n");
                    printf("Press any key to quit...\n");
                    keypress();
                    return 1;
                }
                continue;
            }
            // Successfully loaded
            break;
        }

        // Initialize observer params defaults and apply auto-fit at load when available
        params.angle_h = 30; params.angle_v = 20; params.angle_w = 0;
        if (model != NULL && model->auto_fit_ready) {
            params.distance = model->auto_suggested_distance;
            model->auto_proj_scale = model->auto_suggested_proj_scale;
            s_global_proj_scale_fixed = model->auto_suggested_proj_scale; // sync global scale
            model->auto_fit_applied = 1;
            printf("Auto-fit applied at load (distance=%.4f proj_scale=%.2f)\n", FIXED_TO_FLOAT(model->auto_suggested_distance), FIXED_TO_FLOAT(model->auto_suggested_proj_scale));
        } else {
            params.distance = FLOAT_TO_FIXED(30.0);
        }
        // Get observer parameters (user can override defaults by typing values)
        getObserverParams(&params, model);


    bigloop:
        // Process model with parameters - OPTIMIZED VERSION
            // Process model with parameters - OPTIMIZED VERSION
        last_process_time_start = GetTick();
        printf("Processing model...\n");
        // in processModelFast : if Frame-only mode is active, skip face sorting and use simple painter
        processModelFast(model, &params, filename);
        last_process_time_end = GetTick();


    loopReDraw:
        {
            int key = 0;
            char input[50];

            if (model->faces.face_count > 0) {
                // Initialize QuickDraw
                startgraph(mode);
                // Draw 3D object
                if (jitter) drawPolygons_jitter(model, model->faces.vertex_count, model->faces.face_count, model->vertices.vertex_count); else drawPolygons(model, model->faces.vertex_count, model->faces.face_count, model->vertices.vertex_count);
                // display available colors
                if (colorpalette == 1) { 
                    DoColor(); 
                }
                // If there are inconclusive pairs and display is enabled, underline them on screen
                if (show_inconclusive && inconclusive_pairs_count > 0) {
                    frameInconclusivePairs(model);  
                }
                
                // Wait for key press and get key code
        asm 
            {
            sep #0x20
        loop:
            lda >0xC000     // Read the keyboard status from memory address 0xC000
            bpl loop        // Wait until no key is pressed (= until bit 7 on)
            and #0x007f     // Clear the high bit
            sta >0xC010     // Clear the keypress by writing to 0xC010
            sta key         // Store the key code in variable 'key'
            rep #0x30
            }

        endgraph();        // Close QuickDraw
        }

        DoText();           // Show text screen

    #if ENABLE_DEBUG_SAVE
        sprintf(input, "You pressed key code: %d\n", key);
        printf("%s", input);
    #endif

        // Handle keyboard input with switch statement
        switch (key) {
            case 32:  // Space bar - display info and redraw
                printf("===================================\n");
                printf(" Model information and parameters\n");
                printf("===================================\n");
                printf("Model: %s\n", filename);
                printf("Vertices: %d, Faces: %d\n", model->vertices.vertex_count, model->faces.face_count);
                printf("Observer Parameters:\n");
                printf("    Distance: %.2f\n", FIXED_TO_FLOAT(params.distance));
                printf("    Horizontal Angle: %d deg\n", params.angle_h);
                printf("    Vertical Angle: %d deg\n", params.angle_v);
                printf("    Screen Rotation Angle: %d deg\n", params.angle_w);
                printf("    Projection scale: %.2f\n", FIXED_TO_FLOAT(s_global_proj_scale_fixed));
                if (painter_mode == PAINTER_MODE_FAST) printf("    Painter mode: FAST (simple face sorting only)\n");
                else if (painter_mode == PAINTER_MODE_FIXED) printf("    Painter mode: NORMAL (Fixed32/64)\n");
                else if (painter_mode == PAINTER_MODE_CORRECT) printf("    Painter mode: CORRECT (painter_correct)\n");
                else if (painter_mode == PAINTER_MODE_CORRECTV2) printf("    Painter mode: CORRECT V2 (painter_correctV2 with face splitting detection)\n");
                else if (painter_mode == PAINTER_MODE_NEWELL_SANCHAV3) printf("    Painter mode: NEWELL_SANCHAV3 (painter_newell_sanchaV3)\n");
                else if (painter_mode == PAINTER_MODE_NEWELL_SANCHAV2) printf("    Painter mode: NEWELL_SANCHAV2 (painter_newell_sanchaV2)\n");
                else printf("    Painter mode: FLOAT (float-based)\n\n");
                printf("    Back-face culling: %s\n", cull_back_faces ? "ON" : "OFF");
                printf("    Pan offset: (%d, %d)\n", pan_dx, pan_dy);
                if (user_fill_color == 16) printf("    Fill color: Random\n");
                else if (user_fill_color >= 0) printf("    Fill color: %d\n", user_fill_color);
                else printf("    Fill color: Default (14)\n");
                if (user_frame_color == 16) printf("    Frame color: Random\n");
                else if (user_frame_color >= 0) printf("    Frame color: %d\n", user_frame_color);
                else printf("    Frame color: Default (7)\n");
                printf ("Processing time: %d ticks (1/60 sec.)\n", last_process_time_end - last_process_time_start);
                printf("===================================\n");
                printf("\n");
                printf("Press any key to continue...\n");
                keypress();
                goto loopReDraw;

            case 82:  // 'R' - pan right (was revert; revert disabled)
            case 114: // 'r'
                pan_dx += 10; // move right by 10 pixels
                printf("Pan offset -> (%d,%d)\n", pan_dx, pan_dy);
                goto loopReDraw;

            case 59: // ';' - Run check_sort_repair (repair ordering) and wait for key so user can read results
                printf("Running check_sort_repair (ray_cast verification & minimal repair)...\n");
                check_sort_repair(model, model->faces.face_count);
                printf("Press any key to continue...\n");
                keypress();
                goto loopReDraw;

            case 77: // 'M' - interactive: select two faces -> compute fixed intersection polygon & display (wireframe + overlay)
            case 109: // 'm' (lowercase) - same as 'M'
                if (model == NULL) { printf("No model loaded\n"); goto loopReDraw; }
                inspect_intersection_fixed_ui(model);
                goto loopReDraw;

            case 60: // '<' - Run check_sort and wait for key so user can read results
                printf("Running check_sort (ray_cast verification)...\n");
                check_sort(model, model->faces.face_count);
                printf("Press any key to continue...\n");
                keypress();
                goto loopReDraw;

            case 43:  // '+' - increase projection scale by 10% (applies to current scale)
            case 61:  // '=' also acts as '+' on some keyboards
                if (model != NULL) {
                    // '+' now adjusts projection scale by +10%
                    // Compute new scale = current scale * 1.1 (fixed-point multiplication)
                    Fixed32 cur = s_global_proj_scale_fixed;
                    Fixed32 mul = FLOAT_TO_FIXED(1.1f);
                    Fixed32 new_scale = FIXED_MUL_64(cur, mul);
                    // clamp
                    Fixed32 min_scale = FLOAT_TO_FIXED(1.0f);
                    Fixed32 max_scale = FLOAT_TO_FIXED(10000.0f);
                    if (new_scale < min_scale) new_scale = min_scale;
                    if (new_scale > max_scale) new_scale = max_scale;
                    s_global_proj_scale_fixed = new_scale;
                    // sync to model field for visibility
                    model->auto_proj_scale = s_global_proj_scale_fixed;
                    model->auto_fit_applied = 1; // treat scale as explicitly set
                    printf("Projection scale increased -> %.2f\n", FIXED_TO_FLOAT(s_global_proj_scale_fixed));
                } else {
                    printf("No model loaded.\n");
                }
                compute2DFromObserver(model, params.angle_w);
                goto loopReDraw;

            case 45:  // '-' - decrease projection scale by 10% (applies to current scale)
                if (model != NULL) {
                    // '-' now adjusts projection scale by -10%
                    Fixed32 cur = s_global_proj_scale_fixed;
                    Fixed32 mul = FLOAT_TO_FIXED(0.9f);
                    Fixed32 new_scale = FIXED_MUL_64(cur, mul);
                    Fixed32 min_scale = FLOAT_TO_FIXED(1.0f);
                    Fixed32 max_scale = FLOAT_TO_FIXED(10000.0f);
                    if (new_scale < min_scale) new_scale = min_scale;
                    if (new_scale > max_scale) new_scale = max_scale;
                    s_global_proj_scale_fixed = new_scale;
                    model->auto_proj_scale = s_global_proj_scale_fixed;
                    model->auto_fit_applied = 1;
                    printf("Projection scale decreased -> %.2f\n", FIXED_TO_FLOAT(s_global_proj_scale_fixed));
                } else {
                    printf("No model loaded.\n");
                }
                compute2DFromObserver(model, params.angle_w);
                goto loopReDraw;

            case 65:  // 'A' - decrease distance
            case 97:  // 'a'
                params.distance = params.distance - (params.distance / 10);
                printf("Distance decreased -> %.2f\n", FIXED_TO_FLOAT(params.distance));
                goto bigloop;

            case 90:  // 'Z' - increase distance  
            case 122: // 'z'
                params.distance = params.distance + (params.distance / 10);
                printf("Distance increased -> %.2f\n", FIXED_TO_FLOAT(params.distance));
                goto bigloop;

            case 21:  // Right arrow - increase horizontal angle
                params.angle_h = normalize_deg(params.angle_h + 10);
                goto bigloop;

            case 8:   // Left arrow - decrease horizontal angle
                params.angle_h = normalize_deg(params.angle_h - 10);
                goto bigloop;

            case 10:  // Down arrow - decrease vertical angle
                params.angle_v = normalize_deg(params.angle_v - 10);
                goto bigloop;

            case 11:  // Up arrow - increase vertical angle
                params.angle_v = normalize_deg(params.angle_v + 10);
                goto bigloop;

            case 87:  // 'W' - increase screen rotation angle
            case 119: // 'w'
                params.angle_w = normalize_deg(params.angle_w + 10);
                goto bigloop;

            case 88:  // 'X' - decrease screen rotation angle
            case 120: // 'x'
                params.angle_w = normalize_deg(params.angle_w - 10);
                goto bigloop;

            /* 2D panning: E=left, R=right (R replaced revert), T=up, Y=down (and lowercase) */
            case 69: /* 'E' */
            case 101: /* 'e' */
                pan_dx -= 10; /* pan left */
                printf("Pan offset -> (%d,%d)\n", pan_dx, pan_dy);
                goto loopReDraw;

            case 84: /* 'T' */
            case 116: /* 't' */
                pan_dy -= 10; /* pan up = decrease Y */
                printf("Pan offset -> (%d,%d)\n", pan_dx, pan_dy);
                goto loopReDraw;

            case 89: /* 'Y' */
            case 121: /* 'y' */
                pan_dy += 10; /* pan down = increase Y */
                printf("Pan offset -> (%d,%d)\n", pan_dx, pan_dy);
                goto loopReDraw;

            case 48: /* '0' - reset pan */
                pan_dx = 0; pan_dy = 0;
                printf("Pan reset -> (%d,%d)\n", pan_dx, pan_dy);
                goto loopReDraw;
        
            case 67:  // 'C' - toggle color palette display
            case 99:  // 'c'
                colorpalette ^= 1; // Toggle between 0 and 1
                goto loopReDraw;

            case 73:  // 'I' - toggle display of inconclusive face pairs
            case 105: // 'i'
                show_inconclusive ^= 1;
                printf("Inconclusive pairs display: %s\n", show_inconclusive ? "ON" : "OFF");
                goto loopReDraw;

            case 74:  // 'J' - toggle jittered rendering
            case 106: // 'j'
                jitter ^= 1;
                printf("Jitter rendering: %s\n", jitter ? "ON" : "OFF");
                goto loopReDraw;

            case 68:  // 'D' - inspect face ordering and show misplaced faces
            case 100: // 'd'
                if (model == NULL) { printf("No model loaded\n"); goto loopReDraw; }
                inspect_faces_before(model, &params, filename);
                goto loopReDraw;

            case 86:  // 'V' - show a single face in filled mode
            case 118: // 'v'
                if (model == NULL) { printf("No model loaded\n"); goto loopReDraw; }
                showFace(model, &params, filename);
                goto loopReDraw;

            case 83: // 'S' - inspect faces that are AFTER target but should be BEFORE (new)
            case 115: // 's'
                if (model == NULL) { printf("No model loaded\n"); goto loopReDraw; }
                inspect_faces_after(model, &params, filename);
                goto loopReDraw;


            case 71: // 'G' - reorder using NEWELL_SANCHAV2
            case 103: // 'g'
                if (model == NULL) { printf("No model loaded\n"); goto loopReDraw; }
                printf("Running painter_newell_sanchaV2 reordering pass...\n");
                int tick_count_start = GetTick();
                painter_newell_sanchaV2(model, model->faces.face_count);
                int tick_count_end = GetTick();
                printf("Reordering completed in %d ticks (1/60 sec.)\n", tick_count_end - tick_count_start);
                keypress();
                goto loopReDraw;

            case 79: // 'O' - check projected polygon overlap
            case 111: // 'o'
                if (model == NULL) { printf("No model loaded\n"); goto loopReDraw; }
                inspect_polygons_overlap(model, &params, filename);
                goto loopReDraw;

            case 76: // 'L' - show model with face ID labels at polygon centers
            case 108: // 'l'
                if (model == NULL) { printf("No model loaded\n"); goto loopReDraw; }
                display_model_face_ids(model, &params, filename);
                goto loopReDraw;

            /* New: direct painter mode keys
             * '1' -> FAST, '2' -> NORMAL (Fixed), '3' -> NEWELL_SANCHAV2, 'U' -> FLOAT
             */
            case 49: // '1' - set FAST painter
                painter_mode = PAINTER_MODE_FAST;
                printf("Painter mode: FAST (simple face sorting only)\n");
                inconclusive_pairs_count = 0; // clear inconclusive pairs in fast mode
                if (model != NULL) { printf("Reprocessing model with current mode...\n"); goto bigloop; }
                

            case 50: // '2' - set NORMAL (Fixed32/64) painter
                painter_mode = PAINTER_MODE_FIXED;
                printf("Painter mode: NORMAL (full tests, Fixed32/64)\n");
                if (model != NULL) { printf("Reprocessing model with current mode...\n"); goto bigloop; }
                

            case 51: // '3' - set NEWELL_SANCHAV3 painter (back-faces-first variant)
                painter_mode = PAINTER_MODE_NEWELL_SANCHAV3;
                printf("Painter mode: NEWELL_SANCHAV3 (painter_newell_sanchaV3)\n");
                if (model != NULL) { printf("Reprocessing model with current mode...\n"); goto bigloop; }

            case 52: // '4' - set CORRECT painter (runs painter_correct)
                painter_mode = PAINTER_MODE_CORRECT;
                printf("Painter mode: CORRECT (painter_correct)\n");
                if (model != NULL) { printf("Reprocessing model with current mode...\n"); goto bigloop; }

            case 53: // '5' - set CORRECTV2 painter (runs painter_correctV2 with face splitting detection)
                painter_mode = PAINTER_MODE_CORRECTV2;
                printf("Painter mode: CORRECT V2 (painter_correctV2)\n");
                if (model != NULL) { printf("Reprocessing model with current mode...\n"); goto bigloop; }

            case 85: // 'U' - set FLOAT painter (moved from '3')
            case 117: // 'u'
                painter_mode = PAINTER_MODE_FLOAT;
                printf("Painter mode: FLOAT (float-based painter)\n");
                inconclusive_pairs_count = 0; // clear inconclusive pairs in float mode
                if (model != NULL) { printf("Reprocessing model with current mode...\n"); goto bigloop; }
            case 55: // '7' - choose fill color
                {
                    printf("\n=== Fill color selection ===\n");
                    printf("Available colors:\n");
                    printf(" 0 : black\n");
                    printf(" 1 : grey\n");
                    printf(" 2 : brown\n");
                    printf(" 3 : purple\n");
                    printf(" 4 : blue\n");
                    printf(" 5 : green\n");
                    printf(" 6 : orange\n");
                    printf(" 7 : red\n");
                    printf(" 8 : rose\n");
                    printf(" 9 : yellow\n");
                    printf("10 : light green\n");
                    printf("11 : aqua\n");
                    printf("12 : pale purple\n");
                    printf("13 : light blue\n");
                    printf("14 : light gray\n");
                    printf("15 : white\n");
                    printf("16 : random\n\n");
                    printf("Enter fill color: ");
                    int c = -1;
                    if (scanf("%d", &c) == 1) {
                        if (c >= -1 && c <= 16) {
                            user_fill_color = c;
                            if (c == 16) {
                                if (model != NULL) {
                                    generate_random_colors(model->faces.face_count);
                                }
                                printf("Fill color set to RANDOM (new colors generated)\n");
                            }
                            else if (c >= 0) printf("Fill color set to %d\n", c);
                            else printf("Fill color reset to DEFAULT (14)\n");
                        } else {
                            printf("Invalid color (must be -1 to 16)\n");
                        }
                    }
                    int ch; while ((ch = getchar()) != '\n' && ch != EOF);
                    goto loopReDraw;
                }

            case 56: // '8' - choose frame color
                {
                    printf("\n=== Frame color selection ===\n");
                    printf("Available colors:\n");
                    printf(" 0 : black\n");
                    printf(" 1 : grey\n");
                    printf(" 2 : brown\n");
                    printf(" 3 : purple\n");
                    printf(" 4 : blue\n");
                    printf(" 5 : green\n");
                    printf(" 6 : orange\n");
                    printf(" 7 : red\n");
                    printf(" 8 : rose\n");
                    printf(" 9 : yellow\n");
                    printf("10 : light green\n");
                    printf("11 : aqua\n");
                    printf("12 : pale purple\n");
                    printf("13 : light blue\n");
                    printf("14 : light gray\n");
                    printf("15 : white\n");
                    printf("16 : random\n");
                    printf("17 : same as fill\n\n");
                    printf("Enter frame color: ");
                    int c = -1;
                    if (scanf("%d", &c) == 1) {
                        if (c >= -1 && c <= 17) {
                            user_frame_color = c;
                            if (c == 16) {
                                if (model != NULL) {
                                    generate_random_colors(model->faces.face_count);
                                }
                                printf("Frame color set to RANDOM (new colors generated)\n");
                            }
                            else if (c == 17) printf("Frame color set to SAME AS FILL\n");
                            else if (c >= 0) printf("Frame color set to %d\n", c);
                            else printf("Frame color reset to DEFAULT (7)\n");
                        } else {
                            printf("Invalid color (must be -1 to 17)\n");
                        }
                    }
                    int ch; while ((ch = getchar()) != '\n' && ch != EOF);
                    goto loopReDraw;
                }

            case 57: // '9' - reset colors to default
                user_fill_color = -1;
                user_frame_color = -1;
                printf("Colors reset to defaults\n");
                goto loopReDraw;

            case 54: // '6' - quick random mode for both colors
                user_fill_color = 16;
                user_frame_color = 16;
                if (model != NULL) {
                    generate_random_colors(model->faces.face_count);
                }
                printf("Colors set to RANDOM mode (new colors generated)\n");
                goto loopReDraw;

            case 80:  // 'P' - toggle frame-only polygon rendering
            case 112: // 'p'
                framePolyOnly ^= 1;
                printf("Frame-only polygons: %s\n", framePolyOnly ? "ON" : "OFF");
                if (!framePolyOnly) {
                    // Switched back to filled polygons — re-run full processing to recompute depths & ordering
                    printf("Switching to filled mode: reprocessing model (sorting faces)...\n");
                    goto bigloop;
                }
                else {
                    // Just redraw in frame-only mode
                    goto loopReDraw;
                }

case 66:  // 'B' - toggle back-face culling (observer-space d<=0 test)
case 98:  // 'b'
                cull_back_faces ^= 1;
                printf("Back-face culling: %s\n", cull_back_faces ? "ON" : "OFF");
                if (model != NULL) {
                    printf("Reprocessing model with culling %s...\n", cull_back_faces ? "ON" : "OFF");
                    goto bigloop;
                }
                //goto loopReDraw;

            case 70:  // 'F' - dump face equations to equ.csv
            case 102: // 'f'
                if (model != NULL) {
                    // Use semicolon column separators and comma decimal separator
                    dumpFaceEquationsCSV(model, "equ.csv", 1);
                }
                goto loopReDraw;

            case 78:  // 'N' - load new model
            case 110: // 'n'
                // Reset all parameters to startup defaults when loading a new model
                painter_mode = PAINTER_MODE_FAST;
                pan_dx = 0; pan_dy = 0;
                cull_back_faces = 1;
                user_fill_color = -1;
                user_frame_color = -1;
                s_global_proj_scale_fixed = INT_TO_FIXED(100);
                jitter = 0; /* reset jitter when loading new model */
                destroyModel3D(model);
                goto newmodel;

            case 75:  // 'K' - edit angles/distance (no reload; ENTER may auto-fit)
            case 107: // 'k'
                getObserverParams(&params, model);
                printf("Observer parameters updated.\n");
                goto bigloop;
        
            // display help (paged)
            case 72:  // 'H'
            case 104: // 'h'
                show_help_pager();
                goto loopReDraw;

            case 62: // '>' - interactive ray_cast inspector
                if (model == NULL) { printf("No model loaded\n"); goto loopReDraw; }
                inspect_ray_cast(model);
                goto loopReDraw;

            case 44: // ',' - interactive test of all bbox-intersecting face pairs
                if (model == NULL) { printf("No model loaded\n"); goto loopReDraw; }
                test_all_overlap(model, &params, filename);
                goto loopReDraw;

            case 27:  // ESC - quit
                goto end;
            
            default:  // All other keys - redraw
                goto loopReDraw;
        }
        }  // End of loopReDraw block

        end:
        // Cleanup and exit
        // Dispose of the global polygon handle if it was allocated
        if (globalPolyHandle != NULL) {
            if (poly_handle_locked) {
                HUnlock(globalPolyHandle);
            }
            DisposeHandle(globalPolyHandle);
            globalPolyHandle = NULL;
        }
        destroyModel3D(model);
        return 0;
    }