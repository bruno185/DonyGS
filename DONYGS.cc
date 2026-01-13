/*
 * ============================================================================
 *              DONYGS.cc - Fixed32 Optimized Implementation
 * ============================================================================
 *
 * Purpose:
 *   High-performance 3D viewer implementing the DONYGS painter algorithm with
 *   multiple painter modes (FAST / FIXED / FLOAT). Reads simplified OBJ files
 *   (vertices "v" and faces "f"), transforms them into observer space,
 *   projects to 2D screen coordinates and renders filled polygons.
 *
 * Highlights (2026):
 *   - Multi-platform targets: Apple IIGS (ORCA/C, QuickDraw), Win32 native
 *     viewer (GDI) and an SDL2-based viewer.
 *   - Painter modes:
 *       * FAST  : z_mean + bbox tests (very fast, less robust)
 *       * FIXED : full fixed-point Newell/Sancha implementation with pairwise
 *                 tests and corrections (robust)
 *       * FLOAT : float-based painter that reuses cached float buffers for
 *                 higher throughput on float-capable platforms
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
 *   - The code is optimized for interactive use; use the FAST painter for
 *     high frame-rate, or FIXED/FLOAT modes for correctness on tricky geometry.
 *
 * Author: Bruno
 * Date: 2026-01-03
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
#include <event.h>      // System event management
#include <memory.h>     // Advanced memory management (NewHandle, etc.)
#include <window.h>     // Window management
#include <orca.h>       // ORCA specific functions (startgraph, etc.)
#include <stdint.h>      // uint32_t, etc.


#pragma memorymodel 1

segment "data";

// ============================================================================
// --- Variable globale pour la vérification des indices de sommet dans readFaces ---
// (Ajouté pour garantir la cohérence OBJ)
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
#define PAINTER_MODE_FAST 0
#define PAINTER_MODE_FIXED 1
#define PAINTER_MODE_FLOAT 2
#define PAINTER_MODE_CORRECT 3

static int painter_mode = PAINTER_MODE_FAST; // 0=fast,1=fixed,2=float,3=correct (set with '4')

// Runtime toggle kept for compatibility; main() may set this and we propagate to painter_mode
static int use_float_painter = 0;

// --- Reusable scratch buffers for float painter to avoid per-call malloc/free ---
static float *float_xo = NULL, *float_yo = NULL, *float_zo = NULL; static int float_vcap = 0;
static float *float_px = NULL, *float_py = NULL; static int *float_px_int = NULL, *float_py_int = NULL;static float *f_z_min_buf = NULL, *f_z_max_buf = NULL, *f_z_mean_buf = NULL;
static int *f_minx_buf = NULL, *f_maxx_buf = NULL, *f_miny_buf = NULL, *f_maxy_buf = NULL;
static int *f_display_buf = NULL;
static float *f_plane_a_buf = NULL, *f_plane_b_buf = NULL, *f_plane_c_buf = NULL, *f_plane_d_buf = NULL;
static int *f_plane_conv_buf = NULL; /* 0 = not converted from fixed, 1 = converted */
static int *order_buf = NULL; static int order_cap = 0;


static void ensure_vertex_capacity(int vcount) {
    if (float_vcap >= vcount) return;
    int newcap = (vcount + 15) & ~15; // align
    float_xo = (float*)realloc(float_xo, sizeof(float) * newcap);
    float_yo = (float*)realloc(float_yo, sizeof(float) * newcap);
    float_zo = (float*)realloc(float_zo, sizeof(float) * newcap);
    float_px = (float*)realloc(float_px, sizeof(float) * newcap);
    float_py = (float*)realloc(float_py, sizeof(float) * newcap);
    float_px_int = (int*)realloc(float_px_int, sizeof(int) * newcap);
    float_py_int = (int*)realloc(float_py_int, sizeof(int) * newcap);
    float_vcap = newcap;
}
static void ensure_face_capacity(int face_count) {
    // allocate if not present
    if (f_z_min_buf && f_z_max_buf && f_z_mean_buf && f_minx_buf) { if (order_cap >= face_count) return; }
    int newcap = (face_count + 7) & ~7;
    int oldcap = order_cap;
    f_z_min_buf = (float*)realloc(f_z_min_buf, sizeof(float)*newcap);
    f_z_max_buf = (float*)realloc(f_z_max_buf, sizeof(float)*newcap);
    f_z_mean_buf = (float*)realloc(f_z_mean_buf, sizeof(float)*newcap);
    f_minx_buf = (int*)realloc(f_minx_buf, sizeof(int)*newcap);
    f_maxx_buf = (int*)realloc(f_maxx_buf, sizeof(int)*newcap);
    f_miny_buf = (int*)realloc(f_miny_buf, sizeof(int)*newcap);
    f_maxy_buf = (int*)realloc(f_maxy_buf, sizeof(int)*newcap);
    f_display_buf = (int*)realloc(f_display_buf, sizeof(int)*newcap);
    f_plane_a_buf = (float*)realloc(f_plane_a_buf, sizeof(float)*newcap);
    f_plane_b_buf = (float*)realloc(f_plane_b_buf, sizeof(float)*newcap);
    f_plane_c_buf = (float*)realloc(f_plane_c_buf, sizeof(float)*newcap);
    f_plane_d_buf = (float*)realloc(f_plane_d_buf, sizeof(float)*newcap);
    f_plane_conv_buf = (int*)realloc(f_plane_conv_buf, sizeof(int)*newcap);
    // initialize newly allocated region's conversion flags to 0
    if (f_plane_conv_buf && newcap > oldcap) memset(f_plane_conv_buf + oldcap, 0, sizeof(int)*(newcap - oldcap));
}

static void ensure_order_capacity(int face_count) {
    if (order_cap >= face_count) return;
    int newcap = (face_count + 7) & ~7;
    order_buf = (int*)realloc(order_buf, sizeof(int)*newcap);
    order_cap = newcap;
}

// ============================================================================
//                            FIXED POINT DEFINITIONS
// ============================================================================

/**
 * FIXED POINT ARITHMETIC - 64-bit safe version
 * =============================================
 * 
 * Cette version utilise l'arithmétique en virgule fixe 16.16 avec
 * des calculs intermédiaires 64-bit pour éviter les débordements.
 * 
 * Format: 16.16 (16 bits entiers, 16 bits fractionnaires)
 * Plage: -32768.0 à +32767.99998 avec précision de 1/65536
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
    if (use_float_painter) { painter_newell_sancha_float(model, face_count); return; }
    // ...existing code...
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int i, j;
    
    Fixed32* face_zmean = faces->z_mean;
    if (!face_zmean) return; // sécurité


    // * * * * * 
    // Etape 1 : Tri Z décroissant STABLE sur la moyenne, avec tie-breaker sur l'indice d'origine
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
    if (!PERFORMANCE_MODE)
    {
        long elapsed = t_end - t_start;
        double ms = ((double)elapsed * 1000.0) / 60.0; // 60 ticks per second
        printf("[TIMING] initial sort (qsort): %ld ticks (%.2f ms)\n", elapsed, ms);
    }
    
   
    // * * * * *
    // Etape 2 : Boucle de correction d'ordre (adjacent-swap / bubble-like passes)
    // * * * * *
    // Algorithm rationale:
    //  - After the coarse z_mean sort, adjacent faces may still be in the wrong order
    //    due to overlap or interpenetration. We perform bubble-like passes over the
    //    visible subset to correct local inversions using progressively stronger tests.
    //  - Each adjacent pair (f1,f2) is subjected to a sequence of tests:
    //      Test1: quick depth overlap (z_min/z_max)
    //      Test2: X-axis bounding-box separation
    //      Test3: Y-axis bounding-box separation
    //      Test4..Test7: plane-based observer-side and vertex-plane consistency tests
    //  - If any test conclusively determines f1 should be after f2, we perform an
    //    adjacent swap and record the ordered pair to avoid re-testing the same relation.
    //  - Ambiguous pairs that cannot be concluded by these tests are recorded in
    //    `inconclusive_pairs` for diagnostic framing (no geometric splitting is performed).
    //
    // Implementation notes:
    //  - We use a preallocated `ordered_pairs` buffer (capacity = face_count * 4 heuristic)
    //    to remember definitive relations found during the passes. This prevents repeated
    //    re-evaluation and improves performance on complex meshes.
    //  - The bubble-like approach ensures stability of the result (only adjacent swaps)
    //    while being simple to implement; in practice a small number of passes suffices.
    //  - Complexity: O(P * visible_count) where P is number of passes (until no swaps).

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


        // Parcours des paires consécutives (seulement sur faces visibles si culling activé)
        for (i = 0; i < visible_count-1; i++) {
            int f1 = faces->sorted_face_indices[i];
            int f2 = faces->sorted_face_indices[i+1];

            // show faces to be tested (only in debug mode)
            if (ENABLE_DEBUG_SAVE) {
            debug_two_faces(model, f1, f2);
            }

            if ((f1 == 3 && f2 == 44) || (f1 == 44 && f2 == 3)) {
                printf("Debug breakpoint on faces 3 and 44\n");
                keypress();
            }

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

            int obs_side1 = 0; // côté de l'observateur par rapport au plan de f1 : +1, -1 ou 0 (inconclusive)
            int obs_side2 = 0; // côté de l'observateur par rapport au plan de f2 : +1, -1 ou 0 (inconclusive)
            int side;           // coté du vertex.
            int all_same_side; // flag pour indiquer si tous les vertex sont du même coté
            int all_opposite_side; // flag pour indiquer si tous les vertex sont du coté opposé
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

            if (ENABLE_DEBUG_SAVE) {
            printf("\n**** Test 4 : Testing faces %d and %d\n", f1, f2);
            printf("Face coefs: a1=%f, b1=%f, c1=%f, d1=%f\n", FIXED64_TO_FLOAT(a1), FIXED64_TO_FLOAT(b1), FIXED64_TO_FLOAT(c1), FIXED64_TO_FLOAT(d1));
            }

            obs_side1 = 0; // sign of d1: +1, -1 or 0 (inconclusive)
            if (d1 > (Fixed64)epsilon) obs_side1 = 1; 
            else if (d1 < -(Fixed64)epsilon) obs_side1 = -1;
            else goto skipT4; // si l'observateur est sur le plan, on ne peut rien conclure, il faut faire d'autres tests
            all_same_side = 1;

            if (ENABLE_DEBUG_SAVE) {
                printf("FOR loop start\n");
                printf("obs_side1 = %d\n", obs_side1);
                printf("test_values for face %d must be of the same sign as obs_side1\n", f2);
            }
            for (k=0; k<n2; k++) {
                    int v = faces->vertex_indices_buffer[offset2+k]-1;
                    /* Accumulate in 64-bit to avoid overflow (each product is >> FIXED_SHIFT to keep Fixed32 scale) */
                    Fixed64 acc = 0;
                    acc  = (((Fixed64)a1 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
                    acc += (((Fixed64)b1 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
                    acc += (((Fixed64)c1 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
                    acc += (Fixed64)d1; // d1 already Fixed32 scale
                    if (ENABLE_DEBUG_SAVE) {
                        printf("k = %d, vertex index = %d, vtx = (%f, %f, %f)\n", k, v+1, FIXED_TO_FLOAT(vtx->xo[v]), FIXED_TO_FLOAT(vtx->yo[v]), FIXED_TO_FLOAT(vtx->zo[v]));
                        printf("test_value = %f\n", ((double)acc) / FIXED_SCALE);
                    }
                    if  (acc > (Fixed64)epsilon) side = 1;
                    else if (acc < -(Fixed64)epsilon) side = -1;
                    else continue; // si le vertex est sur le plan, on l'ignore et on passe au vertex suivant
                    if (obs_side1 != side) { 
                        // si un vertex est de l'autre coté, on sort de la boucle
                        // et on met le flag à 0 pour indiquer que le test a échoué (et passer au test suivant)
                        all_same_side = 0; 
                        if (ENABLE_DEBUG_SAVE) {
                            printf("Test 4 failed for faces %d and %d\n", f1, f2);
                            keypress();
                        }  
                        break; 
                        }
            }
            if (ENABLE_DEBUG_SAVE) {
                printf("FOR loop stop\n");
            }

            // test 4 passed
            if (all_same_side) { 
                if (ENABLE_DEBUG_SAVE) {
                printf("Test 4 passed for Faces %d and %d\n", f1, f2);
                keypress();
                }

                continue; // faces are ordered correctly, move to next pair
            }

            skipT4:

            // ********************* TEST 5 ********************
            t5++;
            // Test 5: Is f1 entirely on the opposite side of f2's plane relative to the observer?
            // - This is symmetric to Test 4: if every vertex of f1 is strictly on the opposite
            //   side of f2's plane from the observer, then f1 is behind f2 and no swap is needed.
            // - Both Test 4 and Test 5 are relatively cheap and frequently decisive on planar geometry.

            if (ENABLE_DEBUG_SAVE) {
            printf("Test 5 : Testing faces %d and %d\n", f1, f2);
            printf("Face coefs: a2=%f, b2=%f, c2=%f, d2=%f\n", FIXED64_TO_FLOAT(a2), FIXED64_TO_FLOAT(b2), FIXED64_TO_FLOAT(c2), FIXED64_TO_FLOAT(d2) );
            }

            obs_side2 = 0; // sign of d1: +1, -1 or 0 (inconclusive)
            if (d2 > (Fixed64)epsilon) obs_side2 = 1; 
            else if (d2 < -(Fixed64)epsilon) obs_side2 = -1;
            else goto skipT5; // si l'observateur est sur le plan, on ne peut rien conclure, il faut faire d'autres tests
            all_opposite_side = 1;

            if (ENABLE_DEBUG_SAVE) {
                printf("FOR loop start\n");
                printf("obs_side2 = %d\n", obs_side2);
                printf("test_values for face %d must be of the opposite sign as obs_side2\n", f1);
            }

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
                    // si un vertex est du même coté, on sort de la boucle
                    // et on met le flag à 0 pour indiquer que le test a échoué (et passer au test suivant)
                    all_opposite_side = 0; 
                    if (ENABLE_DEBUG_SAVE) {
                            printf("Test 5 failed for faces %d and %d\n", f1, f2);
                            keypress();
                        }  
                    break; }
                }

                // test 5 passed
                if (all_opposite_side) { // faces are ordered correctly, move to next pair
                    if (ENABLE_DEBUG_SAVE) {
                    printf("Test 5 passed for Faces %d and %d\n", f1, f2);
                    keypress();
                    }

                    continue; // faces are ordered correctly, move to next pair
                }
                
            skipT5:

            // ********************* TEST 6 *********************
            t6++;
            // Test 6: Is f2 entirely on the opposite side of f1's plane relative to the observer?
            // - If all vertices of f2 are strictly on the opposite side, then f2 is behind f1 and
            //   an adjacent swap is required (f2 should come after f1). This test often detects
            //   clear occlusion relations and triggers swaps.
            if (ENABLE_DEBUG_SAVE) {
            printf("Test 6 : Testing faces %d and %d\n", f1, f2);
            printf("Face coefs: a1=%f, b1=%f, c1=%f, d1=%f\n", FIXED64_TO_FLOAT(a1), FIXED64_TO_FLOAT(b1), FIXED64_TO_FLOAT(c1), FIXED64_TO_FLOAT(d1));
            }

            obs_side1 = 0; // sign of d1: +1, -1 or 0 (inconclusive)
            if (d1 > (Fixed64)epsilon) obs_side1 = 1; 
            else if (d1 < -(Fixed64)epsilon) obs_side1 = -1;
            else goto skipT6; // si l'observateur est sur le plan, on ne peut rien conclure, il faut faire d'autres tests

            all_opposite_side = 1;
            if (ENABLE_DEBUG_SAVE) {
                printf("FOR loop start\n");
                printf("obs_side1 = %d\n", obs_side1);
                printf("test_values for face %d must be of the opposite sign as obs_side1\n", f2);
            }
            for (k=0; k<n2; k++) {
                int v = faces->vertex_indices_buffer[offset2+k]-1;
                int side;
                Fixed64 acc = 0;
                acc  = (((Fixed64)a1 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
                acc += (((Fixed64)b1 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
                acc += (((Fixed64)c1 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
                acc += (Fixed64)d1;

                if (ENABLE_DEBUG_SAVE) {
                    printf("k = %d, vertex index = %d, vtx = (%f, %f, %f)\n", k, v+1, FIXED_TO_FLOAT(vtx->xo[v]), FIXED_TO_FLOAT(vtx->yo[v]), FIXED_TO_FLOAT(vtx->zo[v]));
                    printf("test_value = %f\n", ((double)acc) / FIXED_SCALE);
                }

                if  (acc > (Fixed64)epsilon) side = 1;
                else if  (acc < -(Fixed64)epsilon) side = -1;
                else continue; // si le vertex est sur le plan, on l'ignore et on passe au vertex suivant
                if (obs_side1 == side) { 
                    all_opposite_side = 0;
                    if (ENABLE_DEBUG_SAVE) {
                            printf("Test 6 failed for faces %d and %d\n", f1, f2);
                            keypress();
                        }  
                    break; 
                    }
                }

                if (all_opposite_side == 1) { // test 6 passed
                // f2 est du coté opposé de l'observateur, donc f2 est  derrière f1 ==> échange nécessaire
                    if (ENABLE_DEBUG_SAVE) {
                    printf("Test 6 passed for faces %d and %d\n", f1, f2);
                    keypress();
                    } 

                    goto do_swap;
                }



            skipT6: ;

            // ********************* TEST 7 *********************
            t7++;
            // Test 7: Is f1 entirely on the same side of f2's plane as the observer?
            // - Symmetric to Test 6: if all vertices of f1 are on the observer side of f2's plane,
            //   then f1 is in front of f2 and f1 should be drawn after f2 (swap required).
            // - Passing Test 7 is a definitive reason to swap without further sampling.

            if (ENABLE_DEBUG_SAVE) {
            printf("Test 7 : Testing faces %d and %d\n", f1, f2);
            printf("Face coefs: a1=%f, b1=%f, c1=%f, d1=%f\n", FIXED64_TO_FLOAT(a2), FIXED64_TO_FLOAT(b2), FIXED64_TO_FLOAT(c2), FIXED64_TO_FLOAT(d2));
            }
            obs_side2 = 0; // sign of d1: +1, -1 or 0 (inconclusive)
            if (d2 > (Fixed64)epsilon) obs_side2 = 1; 
            else if (d2 < -(Fixed64)epsilon) obs_side2 = -1;
            else goto skipT7; // si l'observateur est sur le plan, on ne peut rien conclure, il faut faire d'autres tests

            all_same_side = 1;
            if (ENABLE_DEBUG_SAVE) {
                printf("FOR loop start\n");
                printf("obs_side1 = %d\n", obs_side1);
                printf("test_values for face %d must be of the opposite sign as obs_side1\n", f2);
            }

            for (k=0; k<n1; k++) {
                int v = faces->vertex_indices_buffer[offset1+k]-1;
                int side;
                Fixed64 acc = 0;
                acc  = (((Fixed64)a2 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
                acc += (((Fixed64)b2 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
                acc += (((Fixed64)c2 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
                acc += (Fixed64)d2;
                
                if (ENABLE_DEBUG_SAVE) {
                    printf("k = %d, vertex index = %d, vtx = (%f, %f, %f)\n", k, v+1, FIXED_TO_FLOAT(vtx->xo[v]), FIXED_TO_FLOAT(vtx->yo[v]), FIXED_TO_FLOAT(vtx->zo[v]));
                    printf("test_value = %f\n", ((double)acc) / FIXED_SCALE);
                }

                if  (acc > (Fixed64)epsilon) side = 1;
                else if  (acc < -(Fixed64)epsilon) side = -1;
                else continue; // si le vertex est sur le plan, on l'ignore et on passe au vertex suivant
                if (obs_side2 != side) { 
                    all_same_side = 0; 
                    if (ENABLE_DEBUG_SAVE) {
                            printf("Test 7 failed for faces %d and %d\n", f1, f2);
                            keypress();
                        } 
                    break; 
                    }
            }
                // f1 n'est pas du même côté de l'observateur, donc f1 n'est pas devant f2
                // on ne doit pas échanger l'ordre des faces
                // aucun test n'a permis de conclure : on signale non concluant
                if (all_same_side == 0)  goto skipT7;

                // f1 est devant f2, on doit échanger l'ordre
                else {
                    if (ENABLE_DEBUG_SAVE) {
                    printf("Test 7 passed for faces %d and %d\n", f1, f2);
                    keypress();
                    } 

                    goto do_swap;
                }




                
            do_swap: {

                if (ENABLE_DEBUG_SAVE) {
                    printf("Swapping faces %d and %d\n", f1, f2);
                }

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
        // 0n devrait découper f1 par f2 (ou inversement), mais on ne le fait pas pour l'instant
        if (ENABLE_DEBUG_SAVE){
                printf("NON CONCLUTANT POUR LES FACES %d ET %d\n", f1, f2);
                keypress();
        }    
       if (inconclusive_pairs != NULL && inconclusive_pairs_count < inconclusive_pairs_capacity) {
                inconclusive_pairs[inconclusive_pairs_count].face1 = f1;
                inconclusive_pairs[inconclusive_pairs_count].face2 = f2;
                inconclusive_pairs_count++;
            }
        // on les met dans la liste des paires ordonnées pour ne plus les tester
        // puisque les tests n'ont pas permis de conclure,l'ordre actuel est conservé
        if (ordered_pairs != NULL && ordered_pairs_count < ordered_pairs_capacity) {
                ordered_pairs[ordered_pairs_count].face1 = f2;
                ordered_pairs[ordered_pairs_count].face2 = f1;
                ordered_pairs_count++;
            }
        
        endfor: ;
        } // FIN de la boucle for int i=0; i<face_count-1; i++

        if (ENABLE_DEBUG_SAVE) {
            printf("Pass completed, swaps this pass: %d\n", swap_count);
            printf("t1=%d t2=%d t3=%d t4=%d t5=%d t6=%d t7=%d\n", t1, t2, t3, t4, t5, t6, t7);
            keypress();
        }

    } while (swapped);
    // Fin du tri à bulle

    
    if (ENABLE_DEBUG_SAVE) {
        printf("Total swaps: %d, Inconclusive pairs: %d, ordored pairs: %d\n", swap_count, inconclusive_pairs_count, ordered_pairs_count);
        keypress();
    }

    // Libérer la mémoire de la liste des paires ordonnées
    if (ordered_pairs) {
        free(ordered_pairs);
    }  
}


/* Float-based painter: reproduces Windows numeric behaviour exactly
   Implemented as non-destructive function; enable via env var USE_FLOAT_PAINTER=1 */

/* painter_correct
 * ----------------
 * Deterministically adjust `faces->sorted_face_indices` in-place using the
 * exact checks performed by `inspect_faces_before` and `inspect_faces_after`.
 *
 * Algorithm overview (safe & conservative):
 *  - For each face id `target`, run two checks:
 *      1) Inspect faces *before* `target` in the current order. If one or more
 *         of these faces are *misplaced* (the pairwise tests indicate they
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
 *    and the pairwise `pair_order_relation` tests.
 *  - `projected_polygons_overlap` treats touching-only cases (shared edge or
 *    single vertex) as NON-overlap; this is intentional and consistent with
 *    the rest of the program.
 *  - The implementation may use per-run caches (O(n^2) memory) to avoid
 *    recomputing overlap/ordering tests for the same pair multiple times.
 *  - The routine only performs local moves that are conservative with respect
 *    to the pair tests — it does not attempt global reordering heuristics.
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

    /* Prepare sorting state  */
    int old_cull = cull_back_faces;
    cull_back_faces = 0;
    painter_newell_sancha(model, face_count);

    int moves = 0;
    int n = face_count;
    /* Per-run caches to avoid recomputing expensive tests:
     * - rel_cache[f*n + t] stores the result of pair_order_relation(f, t):
     *     2 = unknown, -1/0/1 valid results
     * - ov_cache[f*n + t] stores projected overlap: 255 = unknown, 0 = no, 1 = yes
     *
     * These caches use O(n^2) memory but can drastically reduce runtime on
     * models where the same pair is queried often. We allocate them here once
     * per call and free at the end. If `n` is very large this may be disabled
     * or replaced by a smaller LRU cache to bound memory.
     */

    signed char *rel_cache = (signed char*)malloc(n * n * sizeof(signed char)); if (!rel_cache) { printf("Error: painter_correct malloc rel_cache failed\n"); return 0; }
    for (int i = 0; i < n * n; ++i) rel_cache[i] = 2; /* 2 = unknown, values -1,0,1 valid */
    unsigned char *ov_cache = (unsigned char*)malloc(n * n * sizeof(unsigned char)); if (!ov_cache) { free(rel_cache); printf("Error: painter_correct malloc ov_cache failed\n"); return 0; }
    for (int i = 0; i < n * n; ++i) ov_cache[i] = 255; /* 255 = unknown, 0=no-overlap, 1=overlap */

    /* Inverse map for quick position lookup: face_id -> index in sorted_face_indices */
    int *pos_of_face = (int*)malloc(n * sizeof(int)); if (!pos_of_face) { free(rel_cache); free(ov_cache); printf("Error: painter_correct malloc pos_of_face failed\n"); return 0; }
    for (int i = 0; i < n; ++i) pos_of_face[faces->sorted_face_indices[i]] = i;
    /* Per-target minimal allowed index once moved: -1 means not moved yet. */
    int *min_allowed_pos = (int*)malloc(n * sizeof(int)); if (!min_allowed_pos) { free(rel_cache); free(ov_cache); free(pos_of_face); printf("Error: painter_correct malloc min_allowed_pos failed\n"); return 0; }
    for (int i = 0; i < n; ++i) min_allowed_pos[i] = -1;


    /* For each face id (0..face_count-1) treat that as target */
    for (int target = 0; target < face_count; ++target) {
        /* Find current position of target via inverse map */
        int pos = pos_of_face[target];
        if (pos < 0) continue;
        if (pos >= n) continue; /* not present for some reason */

        /* 1) BEFORE test: check faces placed before target */
        int best_before = -1; /* smallest index of misplaced face */
        for (int i = 0; i < pos; ++i) {
            int f = faces->sorted_face_indices[i];

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
            if (!projected_polygons_overlap(model, f, target)) continue;
            /* pair_order_relation is relatively heavy; we cache its result to avoid
             * recalculating for the same pair multiple times during a run.
             */

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

        /* 2) AFTER test: check faces placed after target */
        /* recompute pos if necessary (ensure we have current location) */
        pos = pos_of_face[target];
        if (pos < 0 || pos >= n) continue;
        int best_after = -1; /* largest index of misplaced face */
        for (int i = pos + 1; i < face_count; ++i) {
            int f = faces->sorted_face_indices[i];

            /* depth (Z) quick rejection (cheap) */
            if (faces->z_max[f] <= faces->z_min[target]) continue;
            if (faces->z_max[target] <= faces->z_min[f]) continue;

            /* bounding-box quick rejection (cheap) */
            // if (faces->maxx[f] <= faces->minx[target] || faces->maxx[target] <= faces->minx[f]
            //     || faces->maxy[f] <= faces->miny[target] || faces->maxy[target] <= faces->miny[f]) continue;
            // XXX : ioptimisation 
            if (faces->maxx[f] <= faces->minx[target]) continue;
            if (faces->maxx[target] <= faces->minx[f]) continue;
            if (faces->maxy[f] <= faces->miny[target]) continue;
            if (faces->maxy[target] <= faces->miny[f]) continue;
            // XXX

            /* require actual projected polygon overlap before considering swap
             * (the overlap test is expensive; we avoid it whenever bbox rejects).
             */
            if (!projected_polygons_overlap(model, f, target)) continue;
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

    /* restore state */
    cull_back_faces = old_cull;
    free(rel_cache); free(ov_cache); free(pos_of_face); free(min_allowed_pos);
    if (debug) return moves; else return moves;
}

void painter_newell_sancha_float(Model3D* model, int face_count) {
    if (!model) return;
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;

    // Ensure reusable buffers are large enough and fill them
    ensure_vertex_capacity(vcount);
    ensure_face_capacity(face_count);
    ensure_order_capacity(face_count);


    typedef struct { int face1; int face2; } OrderedPair;
    int ordered_pairs_capacity = face_count * 4;
    OrderedPair* ordered_pairs = NULL;
    if (ordered_pairs_capacity > 0) {
        ordered_pairs = (OrderedPair*)malloc(ordered_pairs_capacity * sizeof(OrderedPair));
        if (!ordered_pairs) ordered_pairs_capacity = 0;
    }
    int ordered_pairs_count = 0;

    for (int i = 0; i < vcount; ++i) {
        float_xo[i] = FIXED_TO_FLOAT(vtx->xo[i]);
        float_yo[i] = FIXED_TO_FLOAT(vtx->yo[i]);
        float_zo[i] = FIXED_TO_FLOAT(vtx->zo[i]);
    }

    // Precompute projected px/py and integer screen coords per vertex (one division per vertex)
    float proj_scale = FIXED_TO_FLOAT(s_global_proj_scale_fixed);
    for (int i = 0; i < vcount; ++i) {
        float z = float_zo[i];
        float_px[i] = (z == 0.0f) ? float_xo[i] : (float_xo[i] / z);
        float_py[i] = (z == 0.0f) ? float_yo[i] : (float_yo[i] / z);
        float screenx = (0.0f - float_px[i]) * -proj_scale + (proj_scale * 0.5f);
        float screeny = (0.0f - float_py[i]) * -proj_scale + (proj_scale * 0.5f);
        float_px_int[i] = (int)(screenx + 0.5f);
        float_py_int[i] = (int)(screeny + 0.5f);
    }

    float *f_z_min = f_z_min_buf;
    float *f_z_max = f_z_max_buf;
    float *f_z_mean = f_z_mean_buf;
    int *f_minx = f_minx_buf;
    int *f_maxx = f_maxx_buf;
    int *f_miny = f_miny_buf;
    int *f_maxy = f_maxy_buf;
    int *f_display = f_display_buf;
    float *f_plane_a = f_plane_a_buf;
    float *f_plane_b = f_plane_b_buf;
    float *f_plane_c = f_plane_c_buf;
    float *f_plane_d = f_plane_d_buf;

    float proj_cx = 0.0f, proj_cy = 0.0f;
    for (int fi = 0; fi < face_count; ++fi) {
        int off = faces->vertex_indices_ptr[fi];
        int n = faces->vertex_count[fi];
        float zmin = 1e30f, zmaxf = -1e30f, sum = 0.0f;
        int minx = 999999, maxx = -999999, miny = 999999, maxy = -999999;
        int disp = 1;
        for (int k = 0; k < n; ++k) {
            int vid = faces->vertex_indices_buffer[off + k] - 1;
            if (vid < 0 || vid >= vcount) continue;
            float z = float_zo[vid]; if (z < 0.0f) disp = 0;
            if (z < zmin) zmin = z; if (z > zmaxf) zmaxf = z; sum += z;
            // use precomputed projected ints
            int sx = float_px_int[vid];
            int sy = float_py_int[vid];
            if (sx < minx) minx = sx; if (sx > maxx) maxx = sx; if (sy < miny) miny = sy; if (sy > maxy) maxy = sy;
        }
        if (!disp || n < 3) {
            f_plane_a[fi] = f_plane_b[fi] = f_plane_c[fi] = f_plane_d[fi] = 0.0f;
            f_plane_conv_buf[fi] = 1; // mark as converted (degenerate)
        }
        else {
            // Lazily convert Fixed32 plane coefficients to float only when needed.
            // Mark as not converted for now; conversion will happen in pair tests (T4/T5).
            f_plane_conv_buf[fi] = 0;
            f_plane_a[fi] = f_plane_b[fi] = f_plane_c[fi] = f_plane_d[fi] = 0.0f;
        }
        f_z_min[fi] = (n>0)?zmin:0.0f; f_z_max[fi] = (n>0)?zmaxf:0.0f; f_z_mean[fi] = (n>0)?(sum/n):0.0f;
        f_minx[fi] = (n>0)?minx:0; f_maxx[fi] = (n>0)?maxx:0; f_miny[fi] = (n>0)?miny:0; f_maxy[fi] = (n>0)?maxy:0; f_display[fi] = disp;
    }

    // initial order: reuse buffer
    int* order = order_buf;
    int visible_count = face_count;
    if (cull_back_faces) {
        visible_count = 0;
        for (int i = 0; i < face_count; ++i) {
            if (f_display[i]) order[visible_count++] = i;
        }
        // append culled faces to keep rest of array stable
        int tail = visible_count;
        for (int i = 0; i < face_count; ++i) {
            if (!f_display[i]) order[tail++] = i;
        }
    } else {
        for (int i = 0; i < face_count; ++i) order[i] = i;
    }

    // Sort faces by z_mean. For performance we use a simple bucket sort (linear time) when visible_count is large,
    // and insertion sort for small counts.
    if (visible_count <= 64) {
        // insertion sort (descending)
        for (int i = 1; i < visible_count; ++i) {
            int key = order[i];
            float kz = f_z_mean[key];
            int j = i - 1;
            while (j >= 0) {
                int ov = order[j];
                float oz = f_z_mean[ov];
                if (oz > kz || (oz == kz && ov < key)) break; // descending, tie-breaker: smaller index first
                order[j+1] = order[j]; j--;
            }
            order[j+1] = key;
        }
    } else {
        int buckets = (visible_count < 256) ? visible_count : 256;
        float zmin_all = 1e30f, zmax_all = -1e30f;
        for (int i = 0; i < visible_count; ++i) { int fi = order[i]; if (f_z_mean[fi] < zmin_all) zmin_all = f_z_mean[fi]; if (f_z_mean[fi] > zmax_all) zmax_all = f_z_mean[fi]; }
        if (zmax_all == zmin_all) {
            // all equal, keep identity order (stable tie-break is already in order[0..visible_count-1])
        } else {
            int *counts = (int*)calloc(buckets, sizeof(int));
            int *temp = (int*)malloc(sizeof(int) * visible_count);
            // bucket indices (counts)
            for (int i = 0; i < visible_count; ++i) {
                int fi = order[i];
                int idx = (int)((f_z_mean[fi] - zmin_all) / (zmax_all - zmin_all) * (buckets - 1));
                if (idx < 0) idx = 0; if (idx >= buckets) idx = buckets - 1;
                counts[idx]++;
            }
            // compute starts (prefix sums)
            int *starts = (int*)malloc(sizeof(int) * buckets);
            int acc = 0;
            for (int b = 0; b < buckets; ++b) { starts[b] = acc; acc += counts[b]; }
            // place items into temp according to starts (use face indices)
            int *pos_in_bucket = (int*)malloc(sizeof(int) * buckets);
            for (int b = 0; b < buckets; ++b) pos_in_bucket[b] = starts[b];
            for (int i = 0; i < visible_count; ++i) {
                int fi = order[i];
                int idx = (int)((f_z_mean[fi] - zmin_all) / (zmax_all - zmin_all) * (buckets - 1));
                if (idx < 0) idx = 0; if (idx >= buckets) idx = buckets - 1;
                temp[pos_in_bucket[idx]++] = fi;
            }
            // flatten buckets from high to low into order (descending z_mean)
            int pos = 0;
            for (int b = buckets - 1; b >= 0; --b) {
                int start = starts[b];
                int cnt = counts[b];
                for (int t = 0; t < cnt; ++t) {
                    order[pos++] = temp[start + t];
                }
            }
            free(pos_in_bucket); free(starts); free(temp); free(counts);
        }
    }

    int swapped_local = 0;
    do {
        swapped_local = 0;
        for (int i = 0; i < visible_count - 1; ++i) {
            int f1 = order[i], f2 = order[i+1];
            /* linear check against ordered_pairs array */
            int already_ordered = 0;
            int p;
            for (p = 0; p < ordered_pairs_count; ++p) {
                if (ordered_pairs[p].face1 == f1 && ordered_pairs[p].face2 == f2) { already_ordered = 1; break; }
            }
            if (already_ordered) continue; 

            // Test 1 : Depth overlap (float)
            if (f_z_max[f2] <= f_z_min[f1]) continue;
            if (f_z_max[f1] <= f_z_min[f2]) {
                int tmp = order[i]; order[i] = order[i+1]; order[i+1] = tmp;
                swapped_local = 1;
                // record pair in ordered_pairs array (f2 before f1)
                if (ordered_pairs != NULL && ordered_pairs_count < ordered_pairs_capacity) {
                    ordered_pairs[ordered_pairs_count].face1 = f2;
                    ordered_pairs[ordered_pairs_count].face2 = f1;
                    ordered_pairs_count++;
                }
                continue; 
            }

            // Test 2 : X overlap
            int minx1 = f_minx[f1], maxx1 = f_maxx[f1], miny1 = f_miny[f1], maxy1 = f_maxy[f1];
            int minx2 = f_minx[f2], maxx2 = f_maxx[f2], miny2 = f_miny[f2], maxy2 = f_maxy[f2];
            if (maxx1 <= minx2 || maxx2 <= minx1) continue;

            // Test 3 : Y overlap
            if (maxy1 <= miny2 || maxy2 <= miny1) continue;

            // Plane tests (4..7) using float plane coefficients
            int n1 = faces->vertex_count[f1];
            int n2 = faces->vertex_count[f2];
            int offset1 = faces->vertex_indices_ptr[f1];
            int offset2 = faces->vertex_indices_ptr[f2];
            float a1, b1, c1, d1;
            if (!f_plane_conv_buf[f1]) {
                a1 = (float)FIXED64_TO_FLOAT(faces->plane_a[f1]);
                b1 = (float)FIXED64_TO_FLOAT(faces->plane_b[f1]);
                c1 = (float)FIXED64_TO_FLOAT(faces->plane_c[f1]);
                d1 = (float)FIXED64_TO_FLOAT(faces->plane_d[f1]);
                f_plane_a[f1] = a1; f_plane_b[f1] = b1; f_plane_c[f1] = c1; f_plane_d[f1] = d1;
                f_plane_conv_buf[f1] = 1;
            } else { a1 = f_plane_a[f1]; b1 = f_plane_b[f1]; c1 = f_plane_c[f1]; d1 = f_plane_d[f1]; }

            float a2, b2, c2, d2;
            if (!f_plane_conv_buf[f2]) {
                a2 = (float)FIXED64_TO_FLOAT(faces->plane_a[f2]);
                b2 = (float)FIXED64_TO_FLOAT(faces->plane_b[f2]);
                c2 = (float)FIXED64_TO_FLOAT(faces->plane_c[f2]);
                d2 = (float)FIXED64_TO_FLOAT(faces->plane_d[f2]);
                f_plane_a[f2] = a2; f_plane_b[f2] = b2; f_plane_c[f2] = c2; f_plane_d[f2] = d2;
                f_plane_conv_buf[f2] = 1;
            } else { a2 = f_plane_a[f2]; b2 = f_plane_b[f2]; c2 = f_plane_c[f2]; d2 = f_plane_d[f2]; }

            float epsilon_f = 1e-6f;

            // Test 4
            int obs_side1 = 0;
            int k;
            float test_val;
            if (d1 > epsilon_f) obs_side1 = 1; else if (d1 < -epsilon_f) obs_side1 = -1; else goto skipT4_float;
            int all_same_side = 1;
            for (k = 0; k < n2; ++k) {
                int v = faces->vertex_indices_buffer[offset2 + k] - 1;
                test_val = a1 * float_xo[v] + b1 * float_yo[v] + c1 * float_zo[v] + d1;  // plane of f1
                int side = (test_val > epsilon_f) ? 1 : ((test_val < -epsilon_f) ? -1 : 0);
                if (side != obs_side1) { all_same_side = 0; break; }
            }
            if (all_same_side) continue;
            skipT4_float: ;

            // Test 5
            int obs_side2 = 0; int all_opposite_side = 1;
            if (d2 > epsilon_f) obs_side2 = 1; else if (d2 < -epsilon_f) obs_side2 = -1; else goto skipT5_float;
            for (k = 0; k < n1; ++k) {
                int v = faces->vertex_indices_buffer[offset1 + k] - 1;
                test_val = a2 * float_xo[v] + b2 * float_yo[v] + c2 * float_zo[v] + d2; // plane of f2
                int side = (test_val > epsilon_f) ? 1 : ((test_val < -epsilon_f) ? -1 : 0);
                if (side == obs_side2) { all_opposite_side = 0; break; }
            }
            if (all_opposite_side) continue;
            
            skipT5_float: ;
        
            // Test 6 : Test si f2 est du  côté opposé de l'observateur par rapport au plan de f1. 
            // Si oui, f2 est derrière f1, on doit échanger l'ordre
            obs_side1 = 0; // sign of d1: +1, -1 or 0 (inconclusive)
            if (d1 > epsilon_f) obs_side1 = 1; 
            else if (d1 < -epsilon_f) obs_side1 = -1;
            else goto skipT6_float; // si l'observateur est sur le plan, on ne peut rien conclure, il faut faire d'autres tests
            
            all_opposite_side = 1;
            for (k=0; k<n2; k++) {
                    int v = faces->vertex_indices_buffer[offset2+k]-1;
                    int side;
                    test_val = a1*vtx->xo[v] + b1*vtx->yo[v] + c1*vtx->zo[v] + d1;
                    if  (test_val > epsilon_f) side = 1;
                    else side = -1;
                    if (obs_side1 == side) { 
                    all_opposite_side = 0; 
                    break; 
                    }
            }
            if (all_opposite_side == 1) goto do_swap;
            // f2 est du coté opposé de l'observateur, donc f2 est derrière f1 ==> échange nécessaire

            skipT6_float: ;

            // Test 7 : Test si f1 est du même côté de l'observateur par rapport au plan de f2. 
            // Si oui, f1 est devant f2, on doit échanger l'ordre
            obs_side2 = 0; // sign of d1: +1, -1 or 0 (inconclusive)
            if (d2 > epsilon_f) obs_side2 = 1; 
            else if (d2 < -epsilon_f) obs_side2 = -1;
            else goto skipT7_float; // si l'observateur est sur le plan, on ne peut rien conclure, il faut faire d'autres tests
            all_same_side = 1;
            for (k=0; k<n1; k++) {
                int v = faces->vertex_indices_buffer[offset1+k]-1;
                int side;
                test_val = a2*vtx->xo[v] + b2*vtx->yo[v] + c2*vtx->zo[v] + d2;
                //test_value = a1*vtx->xo[v] + b1*vtx->yo[v] + c1*vtx->zo[v] + d1;
                if  (test_val > epsilon_f) side = 1;
                else side = -1;
                if (obs_side2 != side) { 
                    all_same_side = 0; 
                    break; 
                    }
            }
                if (all_same_side == 0) goto skipT7_float;
                // f1 n'est pas du même côté de l'observateur, donc f1 n'est pas devant f2
                // on ne doit pas échanger l'ordre des faces
                else {
                    goto do_swap;
                }

            // Si on arrive ici, les tests 1..5 n'ont pas conclu :
            // appliquer l'algorithme original de Newell/Newell/Sancha
            // => échanger les faces (swap) et enregistrer la paire
            do_swap:
            {
                int tmp = order[i]; order[i] = order[i+1]; order[i+1] = tmp;
                swapped_local = 1;
                // record pair in ordered_pairs array (f2 before f1)
                if (ordered_pairs != NULL && ordered_pairs_count < ordered_pairs_capacity) {
                    ordered_pairs[ordered_pairs_count].face1 = f2;
                    ordered_pairs[ordered_pairs_count].face2 = f1;
                    ordered_pairs_count++;
                }
            }

            skipT7_float: ;

        } // end for
    } while (swapped_local);

    // write back order to faces->sorted_face_indices (visible faces first)
    int tail = 0;
    for (int i = 0; i < visible_count; ++i) faces->sorted_face_indices[tail++] = order[i];
    // append culled faces to fill rest (if culling active)
    if (cull_back_faces) {
        for (int i = 0; i < face_count; ++i) if (!f_display[i]) faces->sorted_face_indices[tail++] = i;
    } else {
        for (int i = visible_count; i < face_count; ++i) faces->sorted_face_indices[tail++] = order[i];
    }

    if (ordered_pairs) free(ordered_pairs);

    // Note: buffers are reused across invocations to avoid malloc/free overhead

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
static int segs_intersect_int(int x1,int y1,int x2,int y2,int x3,int y3,int x4,int y4) {
    long long o1 = orient_ll(x1,y1,x2,y2,x3,y3);
    long long o2 = orient_ll(x1,y1,x2,y2,x4,y4);
    long long o3 = orient_ll(x3,y3,x4,y4,x1,y1);
    long long o4 = orient_ll(x3,y3,x4,y4,x2,y2);
    /* Proper intersection only: require strict orientation differences
     * This excludes colinear overlaps and endpoint-touching, which we treat
     * as NON-overlapping for the purposes of projected_polygons_overlap. */
    if (((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) && ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0))) return 1;
    /* Ignore colinear or on-segment cases (return 0) */
    return 0;
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
static int projected_polygons_overlap(Model3D* model, int f1, int f2) {
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    /* Inform the user when an overlap check is performed (useful in interactive mode). */
    if (ENABLE_DEBUG_SAVE) printf("Checking projected overlap for faces %d and %d (touching is considered NON-overlap)\n", f1, f2);
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
    // Risque : cas d'une face entièrement contenue dans l'autre non détecté (mais test Z aura trié)
    // /* Containment tests: only check if candidate point lies inside the other's bbox first
    //  * (cheap) before doing the full ray-cast in point_in_poly_int. This skips expensive
    //  * loops for points obviously outside the other polygon's bbox. */
    // // Containment tests: check *all* vertices of poly1 against poly2, and vice versa.
    // // This avoids missing a containment when the polygon's first vertex lies on a shared
    // // boundary point (touching) which is treated as outside.
    // for (int ii = 0; ii < n1; ++ii) {
    //     int vid = faces->vertex_indices_buffer[off1 + ii] - 1;
    //     if (vid < 0 || vid >= vtx->vertex_count) continue;
    //     int px = vtx->x2d[vid], py = vtx->y2d[vid];
    //     if (px < minx2 || px > maxx2 || py < miny2 || py > maxy2) continue;
    //     if (point_in_poly_int(px, py, faces, vtx, f2, n2)) return 1;
    // }

    // for (int jj = 0; jj < n2; ++jj) {
    //     int vid = faces->vertex_indices_buffer[off2 + jj] - 1;
    //     if (vid < 0 || vid >= vtx->vertex_count) continue;
    //     int px = vtx->x2d[vid], py = vtx->y2d[vid];
    //     if (px < minx1 || px > maxx1 || py < miny1 || py > maxy1) continue;
    //     if (point_in_poly_int(px, py, faces, vtx, f1, n1)) return 1;
    // }

    return 0;
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
 * - pair_plane_after(f1,f2): returns 1 if f1 is geometrically after f2 (Test6 or Test7)
 * - pair_plane_before(f1,f2): returns 1 if f1 is geometrically before f2 (Test4 or Test5)
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
    /* Test7 fallback (symmetric) */
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
    /* Test5 fallback */
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

segment "code04";
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
    if (overlaps) free(overlaps);

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
    drawPolygons(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count);


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

    MoveTo(5, 195);
    printf("%d face(s) should be after face %d\n", misplaced_count, target_face);
    keypress();
    endgraph();
    DoText();

    // Restore state
    framePolyOnly = old_frame;
    for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = backup_flags[i];
    free(backup_flags);
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
    if (overlaps) free(overlaps);
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
    drawPolygons(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count);

    for (int i = 0; i < misplaced_count; ++i) {
        int f = misplaced[i];
        faces->display_flag[f] = 1;
        drawFace(model, f, 12, 0);
    }
    faces->display_flag[target_face] = 1;
    drawFace(model, target_face, 10, 1);


    MoveTo(5, 195);
    printf("%d face(s) should be before face %d\n", misplaced_count, target_face);
    keypress();
    endgraph();
    DoText();

    framePolyOnly = old_frame;
    for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = backup_flags[i];
    free(backup_flags);
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
    int ov = projected_polygons_overlap(model, f1, f2);
    printf("Faces: %d and %d -> Projected overlap: %s\n", f1, f2, ov ? "YES" : "NO");

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
    drawPolygons(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count);

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

    drawPolygons(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count);

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
    // --- MAJ du compteur global pour la vérification des indices de faces ---
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
    if (!PERFORMANCE_MODE)
    {
    printf("Observer angles (degrees) - H: %d, V: %d, W: %d\n", params->angle_h, params->angle_v, params->angle_w);
    }
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
    if (!PERFORMANCE_MODE) {
        long elapsed_loop = t_loop_end - t_loop_start;
        double ms_loop = ((double)elapsed_loop * 1000.0) / 60.0; // 60 ticks per second
        printf("[TIMING] transform+project loop: %ld ticks (%.2f ms)\n", elapsed_loop, ms_loop);
    }

    // Face sorting after transformation
    long t_start, t_end;
    t_start = GetTick();
    calculateFaceDepths(model, NULL, model->faces.face_count);
    t_end = GetTick();
    if (!PERFORMANCE_MODE)
    {
        long elapsed = t_end - t_start;
        double ms = ((double)elapsed * 1000.0) / 60.0; // 60 ticks per second
        printf("[TIMING] calculateFaceDepths: %ld ticks (%.2f ms)\n", elapsed, ms);
    }

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
    } else {
        // PAINTER_MODE_FLOAT
        painter_newell_sancha_float(model, model->faces.face_count);
    }
    t_end = GetTick();

    if (!PERFORMANCE_MODE)
    {
        long elapsed = t_end - t_start;
        double ms = ((double)elapsed * 1000.0) / 60.0; // 60 ticks per second
        const char* pname = (painter_mode==PAINTER_MODE_FAST)?"painter_newell_sancha_fast":(painter_mode==PAINTER_MODE_FIXED?"painter_newell_sancha":(painter_mode==PAINTER_MODE_CORRECT?"painter_correct":"painter_newell_sancha_float"));
        printf("[TIMING] %s: %ld ticks (%.2f ms)\n", pname, elapsed, ms);
        keypress();
    }
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
    
    printf("\nReading vertices from file...'%s':\n", filename);
    
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

        printf("[INFO] readVertices: applied bbox center cx=%.4f cy=%.4f cz=%.4f\n", FIXED_TO_FLOAT(owner->auto_center_x), FIXED_TO_FLOAT(owner->auto_center_y), FIXED_TO_FLOAT(owner->auto_center_z));
        printf("[INFO] readVertices: brute auto-fit suggestion: distance=%.4f proj_scale=%.2f\n", FIXED_TO_FLOAT(owner->auto_suggested_distance), s);
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
    
    printf("\nReading faces from file '%s' :\n", filename);
    
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
                        if (!PERFORMANCE_MODE) {
                            printf("[DEBUG] CULL: Face %d culled (plane_d=%.6f)\n", i, FIXED64_TO_FLOAT(d64));
                        }
                    }
                }
            }
        }

        // Diagnostic: report suspiciously negative z_max values to help debug
        if (!PERFORMANCE_MODE) {
            float zmax_f = FIXED_TO_FLOAT(z_max);
            if (zmax_f < -100.0f) {
                printf("[DEBUG] Face %d: z_min=%.2f z_max=%.2f display_flag=%d n=%d\n", i, FIXED_TO_FLOAT(z_min), zmax_f, display_flag, n);
                // Print per-vertex observer-space zo values
                printf("[DEBUG]  vertex zo: ");
                for (int jj = 0; jj < n; ++jj) {
                    int vidx = face_arrays->vertex_indices_buffer[offset + jj] - 1;
                    if (vidx >= 0) printf("(%d: %.2f) ", vidx, FIXED_TO_FLOAT(vtx->zo[vidx]));
                }
                printf("\n");
            }
        }

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

    if (cull_back_faces && !PERFORMANCE_MODE) {
        printf("[DEBUG] calculateFaceDepths: culled %d faces by back-face test\n", culled_count);
    }
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
            SetSolidPenPat(7);
            FramePoly(polyHandle);
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
            int screenCx = screenScale * center_x; // horizontal is scaled
            int screenCy = center_y;              // vertical not scaled in this codebase

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
                    SetSolidPenPat(7);
                    FramePoly(polyHandle);
                    SetSolidPenPat(14); // keep fill pen as default for next faces
                } else {
                    GetPenPat(pat);
                    FillPoly(polyHandle, pat);
                    SetSolidPenPat(7);
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

segment "code18";
// Diagnostic: Frame in white all polygons listed in `inconclusive_pairs`.
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
        "1: Set painter to FAST (simple face sorting only)",
        "2: Set painter to NORMAL (Fixed32/64)",
        "3: Set painter to FLOAT (float-based)",
        "4: Set painter to CORRECT (painter_correct) - tries to fix ordering by local moves",
        "P: Toggle frame-only polygons (default: OFF)",
        "B: Toggle back-face culling (observer-space D<=0)",
        "I: Toggle display of inconclusive face pairs",
        "D: Inspect face ordering BEFORE selected face (misplaced faces shown in orange)",
        "S: Inspect faces AFTER selected face that should be BEFORE it (misplaced shown in pink)",
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


    newmodel:
        printf("===================================\n");
        printf("       3D OBJ file viewer\n");
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
        {
            const char* tmp = getenv("USE_FLOAT_PAINTER");
            if (tmp && atoi(tmp) != 0) {
                use_float_painter = 1;
                painter_mode = PAINTER_MODE_FLOAT;
            }
        }

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
                drawPolygons(model, model->faces.vertex_count, model->faces.face_count, model->vertices.vertex_count);
                // display available colors
                if (colorpalette == 1) { 
                    DoColor(); 
                }
                // S'il y a des paire inconclusive et si l'affichage est activé, on les souligne à l'affichage
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
                else printf("    Painter mode: FLOAT (float-based)\n\n");
                printf("    Back-face culling: %s\n", cull_back_faces ? "ON" : "OFF");
                printf("    Pan offset: (%d, %d)\n", pan_dx, pan_dy);
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

            case 68:  // 'D' - inspect face ordering and show misplaced faces
            case 100: // 'd'
                if (model == NULL) { printf("No model loaded\n"); goto loopReDraw; }
                inspect_faces_before(model, &params, filename);
                goto loopReDraw;

            case 83: // 'S' - inspect faces that are AFTER target but should be BEFORE (new)
            case 115: // 's'
                if (model == NULL) { printf("No model loaded\n"); goto loopReDraw; }
                inspect_faces_after(model, &params, filename);
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
             * '1' -> FAST, '2' -> NORMAL (Fixed), '3' -> FLOAT
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
                

            case 51: // '3' - set FLOAT painter
                painter_mode = PAINTER_MODE_FLOAT;
                printf("Painter mode: FLOAT (float-based painter)\n");
                inconclusive_pairs_count = 0; // clear inconclusive pairs in float mode
                if (model != NULL) { printf("Reprocessing model with current mode...\n"); goto bigloop; }

            case 52: // '4' - set CORRECT painter (runs painter_correct)
                painter_mode = PAINTER_MODE_CORRECT;
                printf("Painter mode: CORRECT (painter_correct)\n");
                if (model != NULL) { printf("Reprocessing model with current mode...\n"); goto bigloop; }


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
                // Reset painter mode to FAST when loading a new model
                painter_mode = PAINTER_MODE_FAST;
                // Reset 2D pan offsets when loading a new model
                pan_dx = 0; pan_dy = 0;
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