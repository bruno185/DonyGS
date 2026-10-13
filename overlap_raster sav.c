/* Raster-based projected polygon overlap implementation.
 * Included by engine.c after shared projection and geometry helpers.
 */

 segment "raster_overlap";

static int projected_polygons_overlap_raster(Model3D* model, int f1, int f2);
/*
 * projected_polygons_overlap_raster
 * =================================
 *
 * Raster-space overlap test independent of QuickDraw.
 *
 * The function returns 1 when at least one screen pixel has its center
 * strictly inside both projected polygons.
 *
 * Pixel centers are represented in doubled integer coordinates:
 *
 *     pixel (x,y) center = (2*x + 1, 2*y + 1)
 *
 * Polygon vertices are therefore also multiplied by 2. This avoids
 * floating-point and fixed-point arithmetic completely.
 *
 * Important semantics:
 *
 *   - vertex touching vertex        -> no overlap
 *   - vertex touching an edge       -> no overlap
 *   - shared edge only              -> no overlap
 *   - simple tangency               -> no overlap
 *   - positive common raster area   -> overlap
 *
 * This function does not require QuickDraw and therefore also works
 * while the program is in text mode.
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
    /*
     * Fast rejection before high-resolution reprojection.
     *
     * A positive gap is required to reject a pair.
     * Touching bounding boxes must still be tested at x10.
     */
    if (faces->maxx[f1] < faces->minx[f2] ||
        faces->maxx[f2] < faces->minx[f1] ||
        faces->maxy[f1] < faces->miny[f2] ||
        faces->maxy[f2] < faces->miny[f1])
    {
        return 0;
    }
    test_scale =
        FIXED_MUL_64(
            s_global_proj_scale_fixed,
            FLOAT_TO_FIXED(10.0f));
    rch_patch_pair_at_scale(
        model, f1, f2, test_scale, &patch);
    result =
        projected_polygons_overlap_raster(
            model, f1, f2);
    rch_unpatch_pair(
        model, &patch, f1, f2);
    return result;
}
/*
 * Return 1 if the two projected polygons share at least one
 * interior screen pixel.
 */
 /*
 * Maximum number of horizontal intersections for one polygon.
 *
 * A horizontal scanline cannot normally intersect a simple polygon
 * more times than the polygon has edges.
 */
#define RASTER_MAX_INTERSECTIONS MAX_FACE_VERTICES


/*
 * Sort a small array of long integers.
 *
 * Face vertex counts are normally small, so insertion sort is
 * sufficient and avoids any library dependency.
 */
static void raster_sort_long(long* values, int count)
{
    int i;
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
 * Return 1 if two open X intervals contain at least one common
 * pixel center.
 *
 * All coordinates are doubled.
 *
 * Pixel centers are odd coordinates:
 *
 *     ..., 2*x+1, ...
 *
 * Boundaries themselves are excluded.
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
    /*
     * Open intervals:
     *
     *     left < x < right
     */
    if (left >= right)
        return 0;
    /*
     * Find the first odd integer strictly greater than left.
     *
     * Odd doubled coordinates correspond exactly to pixel centers.
     */
    first = left + 1;
    if ((first & 1L) == 0)
        ++first;
    return (first < right);
}
/*
 * Exact incremental raster scanline intersection.
 *
 * The edge maintains the Euclidean quotient and remainder of:
 *
 *     N / D
 *
 * where:
 *
 *     N = (scan_y - y1) * dx
 *     D = abs(dy)
 *
 * Euclidean division guarantees:
 *
 *     N = quotient * D + remainder
 *     0 <= remainder < D
 *
 * The original C division truncates toward zero.
 * For negative N with a nonzero remainder, the result is
 * therefore quotient + 1 rather than quotient.
 *
 * Each scanline advances scan_y by 2, so N advances by
 * 2 * dx. No division is required during scanline traversal.
 */

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
 * Initialize Euclidean quotient and remainder.
 *
 * divisor must be positive.
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
    if (r < 0) {
        --q;
        r += divisor;
    }
    *quotient = (long)q;
    *remainder = (long)r;
}
/*
 * Prepare all non-horizontal edges.
 *
 * The incremental state is initialized at the first
 * scanline in the common bounding-box range.
 */
/*
 * Prepare polygon edges for exact incremental rasterization.
 *
 * Each edge is initialized with:
 *
 *   - its initial Euclidean quotient and remainder;
 *   - the Euclidean quotient and remainder of its
 *     per-scanline numerator increment.
 *
 * This allows constant-time scanline advancement while
 * preserving the original integer division results.
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
        /*
         * Reject edges that cannot intersect any scanline
         * within the common vertical bounding-box range.
         */
        if (ymax <= range_min ||
            ymin > range_max)
        {
            continue;
        }
        e = &edges[count];
        e->x1 = x1;
        e->ymin = ymin;
        e->ymax = ymax;
        /*
         * Normalize the divisor to a positive value.
         */
        if (dy < 0) {
            dy = -dy;
            dx = -dx;
        }
        e->divisor = dy;
        /*
         * First active pixel-center scanline.
         *
         * Polygon vertices have even doubled Y values,
         * while scanlines have odd doubled Y values.
         */
        first_y = range_min;
        if (first_y < ymin)
            first_y = ymin + 1L;
        e->start_y = first_y;
        /*
         * Initial quotient and remainder.
         */
        numerator =
            (long long)(first_y - y1) *
            (long long)dx;
        raster_euclidean_divide(
            numerator,
            dy,
            &e->quotient,
            &e->remainder);
        /*
         * Decompose the per-scanline increment:
         *
         *     step = 2 * dx
         *
         * The remainder is always nonnegative.
         */
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
 * Advance an edge by one pixel-center scanline.
 *
 * The Euclidean quotient and remainder are updated
 * without performing a division.
 *
 * For projected polygon edges, abs(dx) may be larger
 * than abs(dy), so multiple quotient corrections
 * can be necessary.
 */
/*
 * Advance an edge by one pixel-center scanline.
 *
 * The step has already been decomposed into a Euclidean
 * quotient and remainder during edge preparation.
 *
 * Exactly one remainder correction may be necessary.
 * No multiplication or division is performed here.
 */
static void raster_advance_edge(
    RasterPreparedEdge* e)
{
    long remainder;
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
 * Return the same integer intersection as the original
 * C expression:
 *
 *     x1 + (long)(num / dy)
 *
 * The stored quotient uses Euclidean division, so the
 * negative non-integral case requires correction.
 */
static long raster_edge_intersection(
    const RasterPreparedEdge* e)
{
    long q = e->quotient;
    if (q < 0 && e->remainder != 0)
        ++q;
    return e->x1 + q;
}
/*
 * Compute all intersections on the current scanline.
 *
 * Edge states are advanced only when the scanline is
 * at or beyond the edge's first active scanline.
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
    for (i = 0; i < edge_count; ++i) {
        RasterPreparedEdge* e = &edges[i];
        if (scan_y < e->start_y)
            continue;
        if (scan_y < e->ymax) {
            intersections[count++] =
                raster_edge_intersection(e);
        }
        /*
         * Advance after using the current intersection.
         */
        if (scan_y < e->ymax)
            raster_advance_edge(e);
    }
    raster_sort_long(intersections, count);
    return count;
}
/*
 * Optimized raster polygon overlap test.
 *
 * Uses exact incremental scanline intersections.
 *
 * No 64-bit division is performed inside the
 * scanline traversal.
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
    if (n1 > RASTER_MAX_INTERSECTIONS ||
        n2 > RASTER_MAX_INTERSECTIONS)
    {
        return 0;
    }
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

// XXX

/*
 * Compare original and raster x10 polygon overlap tests
 * for every unordered pair of faces.
 *
 * Display progress every 100 pairs.
 * Measure each algorithm separately.
 * Write detailed results to log.txt.
 */
static void test_all_projected_overlaps(Model3D* model)
{
    FaceArrays3D* faces;
    FILE* log;
    int face_count;
    int f1, f2;
    unsigned long total_pairs = 0;
    unsigned long same_results = 0;
    unsigned long different_results = 0;
    unsigned long normal_only = 0;
    unsigned long raster_only = 0;
    unsigned long progress_count = 0;
    long normal_ticks = 0;
    long raster_ticks = 0;
    long start_tick;
    long elapsed_ticks;
    long t;
    double normal_average;
    double raster_average;
    if (!model) {
        printf("Invalid model.\n");
        keypress();
        return;
    }
    faces = &model->faces;
    face_count = faces->face_count;
    log = fopen("log.txt", "w");
    if (!log) {
        printf("Cannot create log.txt\n");
        keypress();
        return;
    }
    printf("\n");
    printf("PROJECTED POLYGON OVERLAP COMPARISON\n");
    printf("====================================\n");
    printf("Faces: %d\n", face_count);
    printf("Pairs: %lu\n",
           (unsigned long)face_count *
           (unsigned long)(face_count - 1) / 2UL);
    printf("\nProgress (1 dot = 100 pairs):\n");
    fprintf(log, "PROJECTED POLYGON OVERLAP COMPARISON\n");
    fprintf(log, "====================================\n\n");
    fprintf(log, "Face count: %d\n", face_count);
    fprintf(log, "Normal: projected_polygons_overlap (V1)\n");
    fprintf(log, "Raster: projected_polygons_overlap_raster_x10\n\n");
    fprintf(log, "DIFFERENT RESULTS\n");
    fprintf(log, "-----------------\n");
    fprintf(log, "Face1  Face2  Normal  Raster\n");
    start_tick = GetTick();
    for (f1 = 0; f1 < face_count - 1; ++f1) {
        for (f2 = f1 + 1; f2 < face_count; ++f2) {
            int normal_result;
            int raster_result;
            /* Measure original overlap test. */
            t = GetTick();
            normal_result =
                projected_polygons_overlap(model, f1, f2);
            normal_ticks += GetTick() - t;
            /* Measure raster x10 overlap test. */
            t = GetTick();
            raster_result =
                projected_polygons_overlap_raster_x10(
                    model, f1, f2);
            raster_ticks += GetTick() - t;
            normal_result = (normal_result != 0);
            raster_result = (raster_result != 0);
            ++total_pairs;
            ++progress_count;
            if (normal_result == raster_result) {
                ++same_results;
            }
            else {
                ++different_results;
                if (normal_result)
                    ++normal_only;
                else
                    ++raster_only;
                fprintf(log,
                        "%5d  %5d  %6d  %6d\n",
                        f1, f2,
                        normal_result,
                        raster_result);
            }
            if (progress_count >= 100UL) {
                printf(".");
                fflush(stdout);
                progress_count = 0;
            }
        }
    }
    elapsed_ticks = GetTick() - start_tick;
    normal_average = 0.0;
    raster_average = 0.0;
    if (total_pairs > 0) {
        normal_average =
            (double)normal_ticks / (double)total_pairs;
        raster_average =
            (double)raster_ticks / (double)total_pairs;
    }
    fprintf(log, "\n");
    fprintf(log, "SUMMARY\n");
    fprintf(log, "=======\n");
    fprintf(log, "Faces:             %d\n", face_count);
    fprintf(log, "Pairs tested:      %lu\n", total_pairs);
    fprintf(log, "Same results:      %lu\n", same_results);
    fprintf(log, "Different results: %lu\n", different_results);
    fprintf(log, "Normal only:       %lu\n", normal_only);
    fprintf(log, "Raster only:       %lu\n", raster_only);
    fprintf(log, "\nPERFORMANCE\n");
    fprintf(log, "===========\n");
    fprintf(log, "Normal total ticks: %ld\n", normal_ticks);
    fprintf(log, "Raster total ticks: %ld\n", raster_ticks);
    fprintf(log, "Normal avg ticks:   %.4f\n", normal_average);
    fprintf(log, "Raster avg ticks:   %.4f\n", raster_average);
    fprintf(log, "Total elapsed ticks: %ld\n", elapsed_ticks);
    fclose(log);
    printf("\n\n");
    printf("RESULTS\n");
    printf("=======\n");
    printf("Faces:       %d\n", face_count);
    printf("Pairs:       %lu\n", total_pairs);
    printf("Same:        %lu\n", same_results);
    printf("Different:   %lu\n", different_results);
    printf("Normal only: %lu\n", normal_only);
    printf("Raster only: %lu\n", raster_only);
    printf("\nPERFORMANCE\n");
    printf("===========\n");
    printf("Normal total: %ld ticks\n", normal_ticks);
    printf("Raster total: %ld ticks\n", raster_ticks);
    printf("Normal avg:   %.4f ticks/pair\n", normal_average);
    printf("Raster avg:   %.4f ticks/pair\n", raster_average);
    printf("Total:        %ld ticks\n", elapsed_ticks);
    printf("\nResults saved to log.txt\n");
    keypress();
}

/* Overlap comparison benchmark; keep in overlap_raster.c. */
static void benchmark_projected_overlap(Model3D *model,
                                        ObserverParams *params,
                                        int angle_count)
{
    FILE *log;
    char angle_input_buffer[40];
    char *endptr;
    long requested;
    int a, i, f1, f2, n, nf;
    unsigned long agree_yes = 0, agree_no = 0;
    unsigned long original_only = 0, raster_only = 0;
    unsigned long tested = 0;
    long original_ticks = 0, raster_ticks = 0;
    int old_h, old_v;
    unsigned long seed = 12345UL;
    VertexArrays3D *vtx;
    FaceArrays3D *faces;
    Fixed32 scale, cx, cy;

    if (!model || !params) return;
    if (angle_count <= 0) angle_count = 10;
    printf("Number of model positions [%d]: ", angle_count);
    if (!fgets(angle_input_buffer, sizeof(angle_input_buffer), stdin)) return;
    if (angle_input_buffer[0] != '\n' && angle_input_buffer[0] != '\r' && angle_input_buffer[0] != '\0') {
        requested = strtol(angle_input_buffer, &endptr, 10);
        while (*endptr == ' ' || *endptr == '\t') ++endptr;
        if (requested < 1 || requested > 10000 ||
            (*endptr != '\0' && *endptr != '\n' && *endptr != '\r')) {
            printf("Invalid number of positions (1..10000).\n");
            return;
        }
        angle_count = (int)requested;
    }
    vtx = &model->vertices;
    faces = &model->faces;
    n = vtx->vertex_count;
    nf = faces->face_count;
    if (n <= 0 || nf < 2) return;
    log = fopen("overlap.tst.txt", "w");
    if (!log) { printf("Cannot open overlap.tst.txt\n"); return; }
    old_h = params->angle_h;
    old_v = params->angle_v;
    scale = s_global_proj_scale_fixed;
    cx = INT_TO_FIXED(CENTRE_X);
    cy = INT_TO_FIXED(CENTRE_Y);
    fprintf(log, "Overlap comparison: %d angles, %d faces, seed 12345\n", angle_count, nf);
    fprintf(log, "DIVERGENCES: angle h v face1 face2 original raster\n");
    fprintf(log, "Each divergence is followed by two POLY lines: face count x0 y0 ...\n");

    for (a = 0; a < angle_count; ++a) {
        Fixed32 ch, sh, cv, sv, m00, m01, m02;
        Fixed32 m10, m11, m12, m20, m21, m22;
        long angle_orig = 0, angle_rast = 0;
        unsigned long angle_diffs = 0;
        int h, v;
        seed = seed * 1664525UL + 1013904223UL;
        h = (int)((seed >> 8) % 360UL);
        seed = seed * 1664525UL + 1013904223UL;
        v = (int)((seed >> 8) % 181UL) - 90;
        params->angle_h = h;
        params->angle_v = v;
        ch = cos_deg_int(h); sh = sin_deg_int(h);
        cv = cos_deg_int(v); sv = sin_deg_int(v);
        /* Same rotation matrix as processModelFast when angle_w == 0. */
        m00 = FIXED_NEG(sh); m01 = ch; m02 = 0;
        m10 = FIXED_NEG(FIXED_MUL_64(ch, sv));
        m11 = FIXED_NEG(FIXED_MUL_64(sh, sv));
        m12 = cv;
        m20 = FIXED_NEG(FIXED_MUL_64(ch, cv));
        m21 = FIXED_NEG(FIXED_MUL_64(sh, cv));
        m22 = FIXED_NEG(sv);
        for (i = 0; i < n; ++i) {
            Fixed32 x = vtx->x[i], y = vtx->y[i], z = vtx->z[i];
            Fixed32 xo, yo, zo, inv;
            zo = FIXED_ADD(FIXED_ADD(FIXED_ADD(
                FIXED_MUL_64(m20,x), FIXED_MUL_64(m21,y)),
                FIXED_MUL_64(m22,z)), params->distance);
            vtx->zo[i] = zo;
            if (zo > 0) {
                xo = FIXED_ADD(FIXED_ADD(FIXED_MUL_64(m00,x),
                     FIXED_MUL_64(m01,y)), FIXED_MUL_64(m02,z));
                yo = FIXED_ADD(FIXED_ADD(FIXED_MUL_64(m10,x),
                     FIXED_MUL_64(m11,y)), FIXED_MUL_64(m12,z));
                vtx->xo[i] = xo; vtx->yo[i] = yo;
                inv = FIXED_DIV_64(scale, zo);
                vtx->x2d[i] = FIXED_ROUND_TO_INT(FIXED_ADD(
                     FIXED_MUL_64(xo,inv), cx));
                vtx->y2d[i] = FIXED_ROUND_TO_INT(FIXED_SUB(
                     cy, FIXED_MUL_64(yo,inv)));
            } else {
                vtx->xo[i] = 0; vtx->yo[i] = 0;
                vtx->x2d[i] = -1; vtx->y2d[i] = -1;
            }
        }
        calculateFaceDepths(model, NULL, nf);
        for (f1 = 0; f1 < nf - 1; ++f1) {
            for (f2 = f1 + 1; f2 < nf; ++f2) {
                int o, r;
                long t;
                t = GetTick();
                o = (projected_polygons_overlap_v2(model,f1,f2) != 0);
                angle_orig += GetTick() - t;
                t = GetTick();
                r = (projected_polygons_overlap_raster_x10(model,f1,f2) != 0);
                angle_rast += GetTick() - t;
                ++tested;
                if (o && r) ++agree_yes;
                else if (!o && !r) ++agree_no;
                else {
                    ++angle_diffs;
                    if (o) ++original_only;
                    else ++raster_only;
                    int face_id, k, count, offset, vertex_id;
                    fprintf(log,"%d %d %d %d %d %d %d\n",
                            a,h,v,f1,f2,o,r);
                    /* Export the exact projected integer coordinates used
                       by both overlap tests. OBJ vertex indices are 1-based. */
                    for (face_id = 0; face_id < 2; ++face_id) {
                        int face = (face_id == 0) ? f1 : f2;
                        count = faces->vertex_count[face];
                        offset = faces->vertex_indices_ptr[face];
                        fprintf(log,"POLY %d %d",face,count);
                        for (k = 0; k < count; ++k) {
                            vertex_id = faces->vertex_indices_buffer[offset+k]-1;
                            fprintf(log," %d %d",
                                    vtx->x2d[vertex_id],vtx->y2d[vertex_id]);
                        }
                        fprintf(log,"\n");
                    }
                }
            }
        }
        original_ticks += angle_orig;
        raster_ticks += angle_rast;
        fprintf(log,"ANGLE %d h=%d v=%d differences=%lu original_ticks=%ld raster_ticks=%ld\n",
                a,h,v,angle_diffs,angle_orig,angle_rast);
        printf("Angle %d/%d: %lu differences\n",a+1,angle_count,angle_diffs);
    }
    fprintf(log,"\nSUMMARY\nPairs tested: %lu\nBoth yes: %lu\nBoth no: %lu\nOriginal only: %lu\nRaster only: %lu\nOriginal ticks: %ld\nRaster ticks: %ld\n",
            tested,agree_yes,agree_no,original_only,raster_only,
            original_ticks,raster_ticks);
    fclose(log);
    params->angle_h = old_h;
    params->angle_v = old_v;
    /* The caller must refresh the current view before rendering again. */
    printf("Done. Results: overlap.tst.txt\n");
}


/* General simple-polygon overlap test, including concave faces.
 * Returns 1 only for intersection with positive area.
 * No triangulation, rasterization or polygon clipping.
 * Assumes simple polygon contours (no self-intersections).
 * Uses double intermediates to avoid 16-bit integer overflow.
 */

static double overlap_v2_cross(double ax, double ay, double bx, double by,
                               double px, double py)
{
    return (bx-ax)*(py-ay) - (by-ay)*(px-ax);
}

/* -1 = boundary, 0 = outside, 1 = strictly inside. */
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
    inside = 0;
    for (i = 0; i < n; ++i) {
        j = (i + 1 == n) ? 0 : i + 1;
        ax = (double)vtx->x2d[faces->vertex_indices_buffer[off+i]-1];
        ay = (double)vtx->y2d[faces->vertex_indices_buffer[off+i]-1];
        bx = (double)vtx->x2d[faces->vertex_indices_buffer[off+j]-1];
        by = (double)vtx->y2d[faces->vertex_indices_buffer[off+j]-1];
        c = overlap_v2_cross(ax,ay,bx,by,px,py);
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

static int overlap_v2_proper_cross(double ax, double ay, double bx, double by,
                                   double cx, double cy, double dx, double dy)
{
    double c1,c2,c3,c4;
    c1=overlap_v2_cross(ax,ay,bx,by,cx,cy);
    c2=overlap_v2_cross(ax,ay,bx,by,dx,dy);
    if (!((c1 > 0.0 && c2 < 0.0) ||
          (c1 < 0.0 && c2 > 0.0))) return 0;
    c3=overlap_v2_cross(cx,cy,dx,dy,ax,ay);
    c4=overlap_v2_cross(cx,cy,dx,dy,bx,by);
    return ((c3 > 0.0 && c4 < 0.0) ||
            (c3 < 0.0 && c4 > 0.0));
}

/* Test tiny offsets on both sides of edge midpoints. This handles
 * coincident polygons and overlapping interiors along common edges.
 * Coordinates are integer pixels; 1/1024 pixel is the probe offset.
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

static int projected_polygons_overlap_v2_original(Model3D *model, int f1, int f2)
{
    FaceArrays3D *faces;
    VertexArrays3D *vtx;
    int n1,n2,off1,off2,i,j,i2,j2,va,vb,vc,vd;
    double ax,ay,bx,by,cx,cy,dx,dy;
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
    /* Strict containment, including concave polygons. */
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


/* Optimized V2. Same geometric tests and floating-point expressions.
 * Intended for ORCA/C (C89, 16-bit int). No fixed vertex-count limit.
 */

static double overlap_v2_opt_cross(double ax, double ay, double bx, double by,
                                   double px, double py)
{
    return (bx-ax)*(py-ay) - (by-ay)*(px-ax);
}

/* -1: boundary, 0: outside, 1: strictly inside. */
static int overlap_v2_opt_point_in_face(Model3D *model, int face,
                                        double px, double py)
{
    FaceArrays3D *faces;
    VertexArrays3D *vtx;
    int *indices;
    int *x2d, *y2d;
    int n, off, i, j, inside;
    double ax, ay, bx, by, c, ix;
    double minx, maxx, miny, maxy;

    faces = &model->faces;
    vtx = &model->vertices;
    indices = faces->vertex_indices_buffer;
    x2d = vtx->x2d;
    y2d = vtx->y2d;
    n = faces->vertex_count[face];
    off = faces->vertex_indices_ptr[face];
    inside = 0;

    j = indices[off+n-1]-1;
    bx = (double)x2d[j];
    by = (double)y2d[j];
    for (i=0; i<n; ++i) {
        j = indices[off+i]-1;
        ax = bx; ay = by;
        bx = (double)x2d[j];
        by = (double)y2d[j];

        /* Only points within the edge bounds can lie on its boundary. */
        minx = (ax < bx) ? ax : bx;
        maxx = (ax > bx) ? ax : bx;
        miny = (ay < by) ? ay : by;
        maxy = (ay > by) ? ay : by;
        if (px >= minx && px <= maxx &&
            py >= miny && py <= maxy) {
            c = overlap_v2_opt_cross(ax,ay,bx,by,px,py);
            if (c == 0.0) return -1;
        }
        if ((ay > py) != (by > py)) {
            ix = ax + (py-ay)*(bx-ax)/(by-ay);
            if (ix > px) inside = !inside;
        }
    }
    return inside;
}

static int overlap_v2_opt_proper_cross(double ax, double ay,
                                       double bx, double by,
                                       double cx, double cy,
                                       double dx, double dy)
{
    double c1, c2, c3, c4;
    c1 = overlap_v2_opt_cross(ax,ay,bx,by,cx,cy);
    c2 = overlap_v2_opt_cross(ax,ay,bx,by,dx,dy);
    if (!((c1 > 0.0 && c2 < 0.0) ||
          (c1 < 0.0 && c2 > 0.0))) return 0;
    c3 = overlap_v2_opt_cross(cx,cy,dx,dy,ax,ay);
    c4 = overlap_v2_opt_cross(cx,cy,dx,dy,bx,by);
    return ((c3 > 0.0 && c4 < 0.0) ||
            (c3 < 0.0 && c4 > 0.0));
}

static int overlap_v2_opt_edge_probes(Model3D *model, int f1, int f2)
{
    FaceArrays3D *faces;
    VertexArrays3D *vtx;
    int *indices, *x2d, *y2d;
    int pass, f, n, off, i, j, va, vb, side;
    double ax, ay, bx, by, mx, my, dx, dy, scale, px, py;

    faces = &model->faces;
    vtx = &model->vertices;
    indices = faces->vertex_indices_buffer;
    x2d = vtx->x2d;
    y2d = vtx->y2d;
    for (pass=0; pass<2; ++pass) {
        f = pass ? f2 : f1;
        n = faces->vertex_count[f];
        off = faces->vertex_indices_ptr[f];
        for (i=0; i<n; ++i) {
            j = (i+1==n) ? 0 : i+1;
            va = indices[off+i]-1;
            vb = indices[off+j]-1;
            ax = (double)x2d[va]; ay = (double)y2d[va];
            bx = (double)x2d[vb]; by = (double)y2d[vb];
            dx = bx-ax; dy = by-ay;
            scale = ((dx<0.0)?-dx:dx)+((dy<0.0)?-dy:dy);
            if (scale==0.0) continue;
            mx = (ax+bx)*0.5; my = (ay+by)*0.5;
            for (side=-1; side<=1; side+=2) {
                px = mx+side*(-dy)/scale/1024.0;
                py = my+side*dx/scale/1024.0;
                if (overlap_v2_opt_point_in_face(model,f1,px,py)==1 &&
                    overlap_v2_opt_point_in_face(model,f2,px,py)==1)
                    return 1;
            }
        }
    }
    return 0;
}

/* ORCA/C C89, 16-bit int. No change to the V2 geometric predicates. */
/* Compile in the same translation unit as Model3D definitions. */
static double ovmax_cross(double ax,double ay,double bx,double by,double px,double py)
{
    return (bx-ax)*(py-ay)-(by-ay)*(px-ax);
}

/* All polygon data is passed once, avoiding repeated model/face lookups. */
static int ovmax_inside(const int *ids,const int *xs,const int *ys,
                        int off,int n,double px,double py)
{
    int i,k,prev,inside;
    double ax,ay,bx,by,ix;
    inside=0;
    prev=ids[off+n-1]-1;
    bx=(double)xs[prev]; by=(double)ys[prev];
    for(i=0;i<n;++i) {
        k=ids[off+i]-1;
        ax=bx; ay=by;
        bx=(double)xs[k]; by=(double)ys[k];
        /* Identical boundary predicate, without four min/max calculations. */
        if ((px>=ax && px<=bx || px>=bx && px<=ax) &&
            (py>=ay && py<=by || py>=by && py<=ay) &&
            ovmax_cross(ax,ay,bx,by,px,py)==0.0) return -1;
        if ((ay>py)!=(by>py)) {
            ix=ax+(py-ay)*(bx-ax)/(by-ay);
            if(ix>px) inside=!inside;
        }
    }
    return inside;
}

static int ovmax_probe(const int *ids,const int *xs,const int *ys,
                       int off1,int n1,int off2,int n2)
{
    int pass,off,n,i,next,va,vb,side;
    double ax,ay,bx,by,dx,dy,scale,mx,my,px,py;
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

static int projected_polygons_overlap_v2_max(Model3D *model,int f1,int f2)
{
    FaceArrays3D *faces;
    VertexArrays3D *vertices;
    int *ids,*xs,*ys;
    int n1,n2,off1,off2,i,j,va,vc;
    int iax,iay,ibx,iby,icx,icy,idx,idy;
    int minax,maxax,minay,maxay;
    double ax,ay,bx,by,cx,cy,dx,dy,c1,c2,c3,c4;
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
            c1=ovmax_cross(ax,ay,bx,by,cx,cy);
            c2=ovmax_cross(ax,ay,bx,by,dx,dy);
            if(!((c1>0.0 && c2<0.0)||(c1<0.0 && c2>0.0))) continue;
            c3=ovmax_cross(cx,cy,dx,dy,ax,ay);
            c4=ovmax_cross(cx,cy,dx,dy,bx,by);
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


/* Experimental fixed-point / integer-only V2 for compilers with 64-bit integers.
 * IMPORTANT: ORCA/C classic may not support 'long long'. This source is a
 * reference implementation, NOT verified as directly compilable in ORCA/C.
 * All geometric decisions are exact for signed 16-bit screen coordinates.
 * No floating point, no divisions in the polygon tests.
 * Include after Model3D definitions.
 */

typedef signed long long ovf64;

static ovf64 ovf_cross(int ax,int ay,int bx,int by,int px,int py)
{
    return ((ovf64)bx-ax)*((ovf64)py-ay)
         - ((ovf64)by-ay)*((ovf64)px-ax);
}

/* Point coordinates are rational nx/d,ny/d; d is strictly positive.
 * This avoids the rounding inherent in a 32.32 fixed-point division.
 * All intermediate operations fit signed 64 bits for 16-bit input coords
 * when the probe is constructed by ovf_probe().
 */
static int ovf_inside(const int *ids,const int *xs,const int *ys,
                      int off,int n,ovf64 nx,ovf64 ny,ovf64 d)
{
    int i,k,prev,inside;
    int ax,ay,bx,by;
    ovf64 c, lhs, rhs;
    inside=0;
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
            /* Equivalent to ray crossing, with no division.
             * Orientation of denominator determines inequality direction.
             */
            c=((ovf64)bx-ax)*rhs-((ovf64)by-ay)*lhs;
            if (((by>ay) && c>0) || ((by<ay) && c<0)) inside=!inside;
        }
    }
    return inside;
}

static int ovf_probe(const int *ids,const int *xs,const int *ys,
                     int off1,int n1,int off2,int n2)
{
    int pass,off,n,i,j,va,vb,side;
    int ax,ay,bx,by;
    ovf64 dx,dy,scale,d,nx,ny;
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
            /* midpoint +/- perpendicular/(1024*Manhattan length)
             * represented as exact rational, not rounded fixed point.
             */
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

static int projected_polygons_overlap_v2_fixed64(Model3D *model,int f1,int f2)
{
    FaceArrays3D *faces;
    VertexArrays3D *vertices;
    int *ids,*xs,*ys;
    int n1,n2,off1,off2,i,j,va,vc;
    int ax,ay,bx,by,cx,cy,dx,dy;
    int minax,maxax,minay,maxay;
    ovf64 c1,c2,c3,c4;
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



/* Select the implementation without changing existing callers.
 * 1: original floating-point V2 (reference)
 * 2: optimized floating-point V2 Max
 * 3: exact-rational Fixed64 (previous default)
 */

 // XXXXX
#define OVERLAP_V2_IMPLEMENTATION 1

static int projected_polygons_overlap_v2(Model3D *model,int f1,int f2)
{
#if OVERLAP_V2_IMPLEMENTATION == 1
    return projected_polygons_overlap_v2_original(model,f1,f2);
#elif OVERLAP_V2_IMPLEMENTATION == 2
    return projected_polygons_overlap_v2_max(model,f1,f2);
#elif OVERLAP_V2_IMPLEMENTATION == 3
    return projected_polygons_overlap_v2_fixed64(model,f1,f2);
#else
#error Invalid OVERLAP_V2_IMPLEMENTATION value
#endif
}


/* Overlap comparison benchmark; keep in overlap_raster.c. */
static void benchmark_projected_overlap_all(Model3D *model,
                                        ObserverParams *params,
                                        int angle_count)
{
    FILE *log;
    char angle_input_buf[40];
    char *endptr;
    long requested;
    int a, i, f1, f2, n, nf;
    unsigned long agree_all_yes = 0, agree_all_no = 0;
    unsigned long disagreements = 0;
    unsigned long tested = 0;
    long original_ticks = 0, v2_ticks = 0, raster_ticks = 0;
    int old_h, old_v;
    unsigned long seed = 12345UL;
    VertexArrays3D *vtx;
    FaceArrays3D *faces;
    Fixed32 scale, cx, cy;

    if (!model || !params) return;
    if (angle_count <= 0) angle_count = 10;
    printf("Number of model positions [%d]: ", angle_count);
    if (!fgets(angle_input_buf, sizeof(angle_input_buf), stdin)) return;
    if (angle_input_buf[0] != '\n' && angle_input_buf[0] != '\r' && angle_input_buf[0] != '\0') {
        requested = strtol(angle_input_buf, &endptr, 10);
        while (*endptr == ' ' || *endptr == '\t') ++endptr;
        if (requested < 1 || requested > 10000 ||
            (*endptr != '\0' && *endptr != '\n' && *endptr != '\r')) {
            printf("Invalid number of positions (1..10000).\n");
            return;
        }
        angle_count = (int)requested;
    }
    vtx = &model->vertices;
    faces = &model->faces;
    n = vtx->vertex_count;
    nf = faces->face_count;
    if (n <= 0 || nf < 2) return;
    log = fopen("overlap.all.txt", "w");
    if (!log) { printf("Cannot open overlap.all.txt\n"); return; }
    old_h = params->angle_h;
    old_v = params->angle_v;
    scale = s_global_proj_scale_fixed;
    cx = INT_TO_FIXED(CENTRE_X);
    cy = INT_TO_FIXED(CENTRE_Y);
    fprintf(log, "Overlap comparison: %d angles, %d faces, seed 12345\n", angle_count, nf);
    fprintf(log, "ALL: angle h v face1 face2 v1 selected_v2 raster\n");
    fprintf(log, "selected_v2 implementation: %d (1=original, 2=max, 3=fixed64)\n", OVERLAP_V2_IMPLEMENTATION);
    fprintf(log, "Each result is followed by two POLY lines: face count x0 y0 ...\n");

    for (a = 0; a < angle_count; ++a) {
        Fixed32 ch, sh, cv, sv, m00, m01, m02;
        Fixed32 m10, m11, m12, m20, m21, m22;
        long angle_orig = 0, angle_v2 = 0, angle_rast = 0;
        unsigned long angle_diffs = 0;
        int h, v;
        seed = seed * 1664525UL + 1013904223UL;
        h = (int)((seed >> 8) % 360UL);
        seed = seed * 1664525UL + 1013904223UL;
        v = (int)((seed >> 8) % 181UL) - 90;
        params->angle_h = h;
        params->angle_v = v;
        ch = cos_deg_int(h); sh = sin_deg_int(h);
        cv = cos_deg_int(v); sv = sin_deg_int(v);
        /* Same rotation matrix as processModelFast when angle_w == 0. */
        m00 = FIXED_NEG(sh); m01 = ch; m02 = 0;
        m10 = FIXED_NEG(FIXED_MUL_64(ch, sv));
        m11 = FIXED_NEG(FIXED_MUL_64(sh, sv));
        m12 = cv;
        m20 = FIXED_NEG(FIXED_MUL_64(ch, cv));
        m21 = FIXED_NEG(FIXED_MUL_64(sh, cv));
        m22 = FIXED_NEG(sv);
        for (i = 0; i < n; ++i) {
            Fixed32 x = vtx->x[i], y = vtx->y[i], z = vtx->z[i];
            Fixed32 xo, yo, zo, inv;
            zo = FIXED_ADD(FIXED_ADD(FIXED_ADD(
                FIXED_MUL_64(m20,x), FIXED_MUL_64(m21,y)),
                FIXED_MUL_64(m22,z)), params->distance);
            vtx->zo[i] = zo;
            if (zo > 0) {
                xo = FIXED_ADD(FIXED_ADD(FIXED_MUL_64(m00,x),
                     FIXED_MUL_64(m01,y)), FIXED_MUL_64(m02,z));
                yo = FIXED_ADD(FIXED_ADD(FIXED_MUL_64(m10,x),
                     FIXED_MUL_64(m11,y)), FIXED_MUL_64(m12,z));
                vtx->xo[i] = xo; vtx->yo[i] = yo;
                inv = FIXED_DIV_64(scale, zo);
                vtx->x2d[i] = FIXED_ROUND_TO_INT(FIXED_ADD(
                     FIXED_MUL_64(xo,inv), cx));
                vtx->y2d[i] = FIXED_ROUND_TO_INT(FIXED_SUB(
                     cy, FIXED_MUL_64(yo,inv)));
            } else {
                vtx->xo[i] = 0; vtx->yo[i] = 0;
                vtx->x2d[i] = -1; vtx->y2d[i] = -1;
            }
        }
        calculateFaceDepths(model, NULL, nf);
        for (f1 = 0; f1 < nf - 1; ++f1) {
            for (f2 = f1 + 1; f2 < nf; ++f2) {
                int o, v2, r;
                int face_id, k, count, offset, vertex_id, face;
                long t;
                t = GetTick();
                o = (projected_polygons_overlap(model,f1,f2) != 0);
                angle_orig += GetTick() - t;
                t = GetTick();
                v2 = (projected_polygons_overlap_v2(model,f1,f2) != 0);
                angle_v2 += GetTick() - t;
                t = GetTick();
                r = (projected_polygons_overlap_raster_x10(model,f1,f2) != 0);
                angle_rast += GetTick() - t;
                ++tested;
                if (o && v2 && r) ++agree_all_yes;
                else if (!o && !v2 && !r) ++agree_all_no;
                else { ++disagreements; ++angle_diffs; }
                fprintf(log,"%d %d %d %d %d %d %d %d\n",
                        a,h,v,f1,f2,o,v2,r);
                /* Export every pair, including agreements. */
                for (face_id = 0; face_id < 2; ++face_id) {
                    face = (face_id == 0) ? f1 : f2;
                    count = faces->vertex_count[face];
                    offset = faces->vertex_indices_ptr[face];
                    fprintf(log,"POLY %d %d",face,count);
                    for (k = 0; k < count; ++k) {
                        vertex_id = faces->vertex_indices_buffer[offset+k]-1;
                        fprintf(log," %d %d",
                                vtx->x2d[vertex_id],vtx->y2d[vertex_id]);
                    }
                    fprintf(log,"\n");
                }
            }
        }
        original_ticks += angle_orig;
        v2_ticks += angle_v2;
        raster_ticks += angle_rast;
        fprintf(log,"ANGLE %d h=%d v=%d differences=%lu v1_ticks=%ld v2_ticks=%ld raster_ticks=%ld\n",
                a,h,v,angle_diffs,angle_orig,angle_v2,angle_rast);
        printf("Angle %d/%d: %lu pairs exported\n",a+1,angle_count,
               (unsigned long)nf * (unsigned long)(nf-1) / 2UL);
    }
    fprintf(log,"\nSUMMARY\nPairs tested: %lu\nAll yes: %lu\nAll no: %lu\nDisagreements: %lu\nV1 ticks: %ld\nV2 ticks: %ld\nRaster ticks: %ld\n",
            tested,agree_all_yes,agree_all_no,disagreements,
            original_ticks,v2_ticks,raster_ticks);
    fclose(log);
    params->angle_h = old_h;
    params->angle_v = old_v;
    /* The caller must refresh the current view before rendering again. */
    printf("Done. Results: overlap.all.txt\n");
}
