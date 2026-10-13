
/*
 * PROJECTED POLYGON OVERLAP - APPLE IIGS
 * ======================================
 *
 * Purpose:
 *   Determine whether two projected 2D polygons overlap.
 *   An overlap requires a strictly positive intersection area.
 *   Contact along an edge or at a vertex is not sufficient.
 *
 * Implementations:
 *
 *   V1:
 *     projected_polygons_overlap_v1()
 *
 *   Simple overlap (used by painter sorting):
 *     projected_polygons_overlap_simple()
 *
 *   V2 original:
 *     projected_polygons_overlap_v2_original()
 *
 *   V2 Max:
 *     projected_polygons_overlap_v2_max()
 *
 *   Fixed64:
 *     projected_polygons_overlap_v2_fixed64()
 *
 *   Raster:
 *     projected_polygons_overlap_raster()
 *
 *   Raster x10:
 *     projected_polygons_overlap_raster_x10()
 *
 *   Runtime dispatcher (defaults to V1):
 *     projected_polygons_overlap()
 *
 *   Legacy V2 dispatcher:
 *     projected_polygons_overlap_v2()
 *
 * Benchmark:
 *     benchmark_projected_overlap_all()
 *
 * Validation:
 *   All six implementations were compared against an
 *   independent Python/Shapely geometric reference.
 *
 *   Test set:
 *     67 faces, 10 orientations, 22,110 polygon pairs.
 *
 *   Algorithm    Ticks    Errors vs Python
 *   --------------------------------------
 *   V1            3584          247
 *   V2 original  14331            0
 *   V2 Max        6841            0
 *   Fixed64       5988            0
 *   Raster         775          209
 *   Raster x10    3281          160
 *
 * Results:
 *   V2 original, V2 Max and Fixed64 produced identical
 *   results and agreed with Python on all 22,110 pairs.
 *
 *   Fixed64 was 2.39 times faster than V2 original
 *   and approximately 14% faster than V2 Max.
 *
 *   Raster was the fastest implementation, but missed
 *   209 positive-area intersections.
 *
 *   Raster x10 produced 81 false positives and
 *   79 false negatives.
 *
 * Recommendation:
 *   projected_polygons_overlap_v2_fixed64() offers
 *   the best measured performance among implementations
 *   matching the Python reference.
 *
 * Limitations:
 *   Validation applies only to the tested configurations.
 *   The Python reference uses floating-point geometry.
 *   Extremely small intersections have not been checked
 *   using exact rational arithmetic.
 *
 * Timing:
 *   Measured on Apple IIGS using GetTick() at 60 Hz.
 *   File I/O was excluded from timed sections.
 */


/* Geometric helpers relocated from engine.c without changing their bodies. */
/* Compute center of intersection of two faces' projected axis-aligned bboxes.
 * Returns 1 and writes (*outx,*outy) on success, 0 if no intersection or invalid ids.
 */
static int compute_bbox_intersection_center(Model3D* model, int f1, int f2, int *outx, int *outy) {
    if (!model || !outx || !outy) return 0;
    int ix0, iy0, ix1, iy1;
    if (!compute_bbox_intersection(model, f1, f2, &ix0, &iy0, &ix1, &iy1)) return 0;
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

static long long orient_ll(long long ax,long long ay,long long bx,long long by,long long cx,long long cy) {
    return (bx-ax)*(cy-ay) - (by-ay)*(cx-ax);
}
static int on_seg_ll(long long ax,long long ay,long long bx,long long by,long long cx,long long cy) {
    if (( (ax<=cx && cx<=bx) || (bx<=cx && cx<=ax) ) && ((ay<=cy && cy<=by) || (by<=cy && cy<=ay))) return 1;
    return 0;
}


/*
 * Return 1 only when the interiors of the two projected segments
 * strictly cross.
 *
 * The following cases are NOT considered intersections:
 *   - endpoint touching endpoint
 *   - endpoint touching the other segment
 *   - collinear segments
 *   - partially or fully overlapping collinear segments
 *
 * This intentionally uses no pixel tolerance. The decision is based
 * exclusively on the integer projected coordinates.
 *
 * A strict crossing occurs only when:
 *   - C and D are strictly on opposite sides of AB, and
 *   - A and B are strictly on opposite sides of CD.
 *
 * If any orientation is zero, the case is therefore a contact or
 * collinear case and is not considered an overlap here.
 */
static int segs_intersect_int_fixed64(
    int x1, int y1, int x2, int y2,
    int x3, int y3, int x4, int y4)
{
    long long o1 = orient_ll(x1, y1, x2, y2, x3, y3);
    long long o2 = orient_ll(x1, y1, x2, y2, x4, y4);
    long long o3 = orient_ll(x3, y3, x4, y4, x1, y1);
    long long o4 = orient_ll(x3, y3, x4, y4, x2, y2);
    if (!(((o1 > 0 && o2 < 0) ||
           (o1 < 0 && o2 > 0)) &&
          ((o3 > 0 && o4 < 0) ||
           (o3 < 0 && o4 > 0))))
    {
        return 0;
    }
    return 1;
}

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


static int compute_intersection_centroid_ordered_fixed(Model3D* model, int subj, int clip, int* outx, int* outy, long long* out_area2) {
    /* Robust integer implementation for diagnostics and fixed-area comparisons.
     * - Clips `subj` by `clip` using Sutherland–Hodgman (double intermediates).
     * - Produces integer output vertices (rounded), computes signed raw area2
     *   (shoelace: 2 * pixel-area) and centroid (integer-rounded).
     * - Returns 1 if a clipped polygon (>=3 verts) was produced; 0 otherwise.
     * - Writes *out_area2 in **pixel^2** units (raw_area2).  This matches
     *   compute_intersection_centroid_ordered_qd_fixed and simplifies callers.
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
    /* Watchdog and iteration-cap disabled for repro testing (temporary). */
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
    /* Normalize winding: ensure both polygons use CCW orientation so SH is deterministic
     * regardless of the input vertex order. If signed area < 0, reverse vertex order. */
    {
        long long area2_subj = 0;
        for (int i = 0; i < sn; ++i) {
            int j = (i + 1) % sn;
            area2_subj += (long long)sx[i] * (long long)sy[j] - (long long)sx[j] * (long long)sy[i];
        }
        /* record signed subject area for repro pair */
        if (area2_subj < 0) {
            /* reverse subj arrays in-place */
            for (int a = 0, b = sn - 1; a < b; ++a, --b) {
                int tx = sx[a]; sx[a] = sx[b]; sx[b] = tx;
                int ty = sy[a]; sy[a] = sy[b]; sy[b] = ty;
            }
        }
        long long area2_clip = 0;
        for (int i = 0; i < cn; ++i) {
            int j = (i + 1) % cn;
            area2_clip += (long long)cx[i] * (long long)cy[j] - (long long)cx[j] * (long long)cy[i];
        }
        if (area2_clip < 0) {
            /* reverse clip arrays in-place */
            for (int a = 0, b = cn - 1; a < b; ++a, --b) {
                int tx = cx[a]; cx[a] = cx[b]; cx[b] = tx;
                int ty = cy[a]; cy[a] = cy[b]; cy[b] = ty;
            }
        }
    }
    /* Optional detailed debug dump for failing cases (writes centroidlog.txt) */
    FILE* dlog = NULL;
    if ((debug_overlap_subj == subj && debug_overlap_clip == clip) || (subj == 12 && clip == 50)) {
        //dlog = fopen("centroidlog.txt", "w");
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
    for (int i = 0; i < cur_n && i < DEBUG_CLIP_MAX; ++i) {
        buf_x1[i] = (double)sx[i]; buf_y1[i] = (double)sy[i];
    }
    /* Sutherland–Hodgman: clip against each edge of `clip` */
    for (int ci = 0; ci < cn && cur_n > 0; ++ci) {
        /* SH outer-loop entry trace for repro pair */
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
     * => Cx = cx_num / (3 * raw_area2_signed)
     */
    long long numer_x = cx_num;
    long long numer_y = cy_num;
    long long denom = 3 * raw_area2_signed;
    if (denom < 0) { denom = -denom; numer_x = -numer_x; numer_y = -numer_y; }
    long long cx_i = (numer_x >= 0) ? (numer_x + denom/2) / denom : -(( -numer_x + denom/2) / denom);
    long long cy_i = (numer_y >= 0) ? (numer_y + denom/2) / denom : -(( -numer_y + denom/2) / denom);
    if (outx) *outx = (int)cx_i; if (outy) *outy = (int)cy_i;
    /* Return raw area2 in pixel^2 (absolute value of signed raw_area2). */
    if (out_area2) *out_area2 = (long long)abs_raw;
    free(sx); free(sy); free(cx); free(cy);
    free(buf_x1); free(buf_y1); free(buf_x2); free(buf_y2); free(out_px); free(out_py);
    return 1;
}
/* Try both clipping orders: call compute_intersection_centroid_ordered_fixed(subj,clip)
 * and if it produces fewer than 3 output vertices, try compute_intersection_centroid_ordered_fixed(clip,subj).
 * This keeps existing callers intact and provides a single-call fallback for UI/diagnostic use.
 */
static int compute_intersection_centroid_ordered_fixed_tryboth(Model3D* model, int subj, int clip, int* outx, int* outy, long long* out_area2, int* out_fallback) {
    if (!model || out_area2 == NULL) return 0;
    if (out_fallback) *out_fallback = 0;
    /* Try primary order first */
    int res = compute_intersection_centroid_ordered_fixed(model, subj, clip, outx, outy, out_area2);
    if (debug_clip_fixed_vcount >= 3) { if (out_fallback) *out_fallback = 0; return res; }
    /* Try reversed order */
    int res2 = compute_intersection_centroid_ordered_fixed(model, clip, subj, outx, outy, out_area2);
    if (debug_clip_fixed_vcount >= 3) { if (out_fallback) *out_fallback = 0; return res2; }
    /* No drawable clipped polygon found in either order — perform bbox/sample fallback
     * (mirror the inspector behavior): find a sample point inside the integer
     * intersection bbox that lies inside both polygons; otherwise use bbox center.
     * Report fallback via out_fallback (nullable).
     */
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
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
        /* Fallback to intersecting per-face bboxes if compute_bbox_intersection failed */
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
    if (!found) { /* final fallback: center of the intersection bbox */
        if (outx) *outx = (ix0 + ix1) / 2;
        if (outy) *outy = (iy0 + iy1) / 2;
    }
    /* Report fallback and clear clipped-polygon diagnostics */
    if (out_fallback) *out_fallback = 1;
    debug_clip_fixed_vcount = 0; debug_clip_raw_area2 = 0; debug_clip_fixed_area = 0.0;
    debug_clip_centroid_x = outx ? *outx : 0; debug_clip_centroid_y = outy ? *outy : 0;
    if (out_area2) *out_area2 = 0; /* no polygon area */
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
    /* Build region for subj — construct polygon then FramePoly into r1 */
    {
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
    /* Build region for clip — construct polygon then FramePoly into r2 */
    {
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
    /* Intersection */
    SectRgn(r1, r2, r3);
    /* runs-based parsing removed — QuickDraw region header bbox (r3->rgnBBox) is authoritative */
    int has = 0;
    if (!EmptyRgn(r3)) {
        /* Read region header (size + bbox words) from r3 */
        HLock((Handle)r3);
        short* sp = (short*)(*r3);
        int w0 = (int)sp[0];
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
        /* runs parsing removed — use region header bbox (r3->rgnBBox) directly */
        /* Use the region header bbox (r3->rgnBBox) as authoritative for the QD region rectangle.
         * Keep runs parsing only for diagnostics (do not override the region header bbox). */
        *ix0 = h_left; *iy0 = h_top; *ix1 = h_right; *iy1 = h_bottom;
        if (*ix0 <= *ix1 && *iy0 <= *iy1) {
            if (out_area2) *out_area2 = (long long)((*ix1 - *ix0) * (*iy1 - *iy0));
            /* Paint region for visual confirmation (apply current pan so visual matches overlays) */
            PenState ps_dbg; GetPenState(&ps_dbg); SetSolidPenPat(COL_YELLOW);
            /* shift the QuickDraw region by the global pan so PaintRgn draws at the
               same position as the other overlays which apply `pan_dx/pan_dy` at
               render time */
            OffsetRgn(r3, pan_dx, pan_dy);
            PaintRgn(r3); SetPenState(&ps_dbg);
            HUnlock((Handle)r3);
            DisposeRgn(r1); DisposeRgn(r2); DisposeRgn(r3);
            return 1;
        }
        HUnlock((Handle)r3);
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


 /* Original overlap implementations relocated from engine.c.
 * These functions remain static because overlap_raster.c is
 * textually included by engine.c, preserving one translation unit.
 * Keep the original arithmetic and decision logic unchanged.
 */
segment "ordering";

/* V1 overlap test. Retains the original arithmetic and decisions.
 * This implementation is used by the painter and the six-way benchmark.
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
    /* Instrumentation */
    overlapCheckCount++;
    /*
     * Watchdog: bail out conservatively if this pair takes
     * pathologically long.
     */
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
    /*
     * Step 1: whole-polygon AABB rejection.
     *
     * Touching only at an edge or point is not considered overlap.
     */
    if (maxx1 <= minx2 ||
        maxx2 <= minx1 ||
        maxy1 <= miny2 ||
        maxy2 <= miny1)
    {
        return 0;
    }
    int off1 = faces->vertex_indices_ptr[f1];
    int off2 = faces->vertex_indices_ptr[f2];
    /*
     * Precompute all edges of f2.
     */
    int f2_precomputed = (n2 <= MAX_FACE_VERTICES);
    int f2_cx[MAX_FACE_VERTICES];
    int f2_cy[MAX_FACE_VERTICES];
    int f2_dx[MAX_FACE_VERTICES];
    int f2_dy[MAX_FACE_VERTICES];
    int f2_minx[MAX_FACE_VERTICES];
    int f2_maxx[MAX_FACE_VERTICES];
    int f2_miny[MAX_FACE_VERTICES];
    int f2_maxy[MAX_FACE_VERTICES];
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
    /*
     * Step 2: edge-vs-edge strict intersection.
     *
     * segs_intersect_int() must detect only genuine crossings.
     * Vertex contacts, shared edges and tangencies do not count.
     */
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
                    /*
                     * Edge AABB quick rejection.
                     */
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
    /*
     * Step 3: vertex containment.
     *
     * Boundary points are outside according to point_in_poly_int().
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
    /*
     * Step 4: identical polygons.
     *
     * IMPORTANT:
     *
     * Do NOT return 0 here when no strict crossing or vertex
     * containment was found.
     *
     * Two projected polygons can have a positive common area
     * without either condition being detected, particularly when
     * they share vertices or edges.
     *
     * In that situation the sampling and exact clipping stages
     * below are allowed to make the final decision.
     */
    if (!candidate) {
        if (faces_vertices_equal(
                faces,
                vtx,
                f1,
                f2))
        {
            return 1;
        }
    }
    /*
     * Steps 5-7: sampling followed by exact clipping.
     */
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
        /*
         * Step 5: center of intersection bbox.
         */
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
        /*
         * Step 6: adaptive 3 x 3 sampling.
         */
        {
            int ixmin;
            int ixmax;
            int iymin;
            int iymax;
            int W;
            int H;
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
        /*
         * Step 7: exact Fixed64 Sutherland-Hodgman clipping.
         */
        overlapClipCalls++;
        {
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
            /*
             * First clipping order:
             * f1 clipped by f2.
             */
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
                if (valid1) {
                    overlapClipAccept++;
                    return 1;
                }
            }
            /*
             * Second clipping order:
             * f2 clipped by f1.
             */
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
                if (valid2) {
                    overlapClipAccept++;
                    return 1;
                }
            }
        }
    }
    return 0;
}
/*
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
    /* Containment tests: check *all* vertices of poly1 against poly2, and vice versa.
     * This avoids missing a containment when the polygon's first vertex lies on a shared
     * boundary point (touching) which is treated as outside. Only checks the candidate
     * point's bbox first (cheap) before the full ray-cast in point_in_poly_int, skipping
     * expensive loops for points obviously outside the other polygon's bbox. */
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
#define OVERLAP_V2_IMPLEMENTATION 3

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


/* Runtime overlap method selection. V1 is the default. */
#define OVERLAP_SIMPLE      0
#define OVERLAP_V1          1
#define OVERLAP_V2          2
#define OVERLAP_V2_MAX      3
#define OVERLAP_FIXED64     4
#define OVERLAP_RASTER      5
#define OVERLAP_RASTER_X10  6

static int overlap_method = OVERLAP_V1;

/* Unified entry point for all normal overlap tests. */
static int projected_polygons_overlap(Model3D *model, int f1, int f2)
{
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

/* Text-only UI. Returns the chosen method, or current_method on cancel. */
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

    for (;;) {
        printf("\nOVERLAP DETECTION METHOD\n\n");

        for (i = 0; i < 7; ++i)
            printf("%d. %-12s %s\n", i + 1, names[i],
                   (i == current_method) ? "<-- current" : "");

        printf("0. Cancel\n\nSelect (0-7, Return=cancel): ");
        fflush(stdout);

        if (fgets(user_choice, sizeof(user_choice), stdin) == NULL)
            return current_method;

        /* Return alone cancels immediately. */
        if (user_choice[0] == '\n' ||
            user_choice[0] == '\r' ||
            user_choice[0] == '\0')
            return current_method;

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
/* Six-way benchmark: timing excludes logging and projection.
 * Each algorithm is timed over all pairs as a single block.
 * The subsequent export pass is deliberately outside the timed blocks.
 */
#define OVERLAP_BENCH_ALGOS 6
#define OVERLAP_BENCH_REPEAT 1

static int overlap_bench_call(Model3D *model, int f1, int f2, int algorithm)
{
    switch (algorithm) {
    case 0: return projected_polygons_overlap_v1(model,f1,f2) != 0;
    case 1: return projected_polygons_overlap_v2_original(model,f1,f2) != 0;
    case 2: return projected_polygons_overlap_v2_max(model,f1,f2) != 0;
    case 3: return projected_polygons_overlap_v2_fixed64(model,f1,f2) != 0;
    case 4: return projected_polygons_overlap_raster(model,f1,f2) != 0;
    case 5: return projected_polygons_overlap_raster_x10(model,f1,f2) != 0;
    }
    return 0;
}

static void benchmark_projected_overlap_all(Model3D *model,
                                            ObserverParams *params,
                                            int angle_count)
{
    FILE *log;
    char angle_input_buf[40];
    char *endptr;
    long requested;
    int a, i, f1, f2, n, nf, alg, rep;
    int old_h, old_v;
    unsigned long seed = 12345UL;
    unsigned long tested = 0, disagreements = 0;
    unsigned long positives[OVERLAP_BENCH_ALGOS] = {0,0,0,0,0,0};
    long total_ticks[OVERLAP_BENCH_ALGOS] = {0,0,0,0,0,0};
    volatile unsigned long timing_sink = 0;
    VertexArrays3D *vtx;
    FaceArrays3D *faces;
    Fixed32 scale, cx, cy;

    if (!model || !params) return;
    if (angle_count <= 0) angle_count = 10;
    printf("Number of model positions [%d]: ", angle_count);
    if (!fgets(angle_input_buf, sizeof(angle_input_buf), stdin)) return;
    if (angle_input_buf[0] != '\n' && angle_input_buf[0] != '\r' &&
        angle_input_buf[0] != '\0') {
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
    fprintf(log,"Overlap benchmark six algorithms; angles=%d faces=%d seed=12345\n",
            angle_count,nf);
    fprintf(log,"ALL: angle h v face1 face2 v1 v2 v2_max fixed64 raster raster_x10\n");
    fprintf(log,"Each result is followed by two POLY lines: face count x0 y0 ...\n");
    fprintf(log,"Timing: block GetTick, repeats=%d, no disk I/O inside timed blocks\n",
            OVERLAP_BENCH_REPEAT);

    for (a = 0; a < angle_count; ++a) {
        Fixed32 ch, sh, cv, sv, m00, m01, m02;
        Fixed32 m10, m11, m12, m20, m21, m22;
        long angle_ticks[OVERLAP_BENCH_ALGOS] = {0,0,0,0,0,0};
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

        /* Time all pairs in one block per algorithm. */
        for (alg = 0; alg < OVERLAP_BENCH_ALGOS; ++alg) {
            long t = GetTick();
            unsigned long checksum = 0;
            for (rep = 0; rep < OVERLAP_BENCH_REPEAT; ++rep) {
                for (f1 = 0; f1 < nf-1; ++f1) {
                    for (f2 = f1+1; f2 < nf; ++f2) {
                        checksum += (unsigned long)
                            overlap_bench_call(model,f1,f2,alg);
                    }
                }
            }
            angle_ticks[alg] = GetTick() - t;
            total_ticks[alg] += angle_ticks[alg];
            timing_sink += checksum;
        }

        /* Export every pair and its coordinates, outside timing. */
        for (f1 = 0; f1 < nf-1; ++f1) {
            for (f2 = f1+1; f2 < nf; ++f2) {
                int values[OVERLAP_BENCH_ALGOS];
                int face_id, k, count, offset, vertex_id, face;
                int same = 1;
                for (alg = 0; alg < OVERLAP_BENCH_ALGOS; ++alg) {
                    values[alg] = overlap_bench_call(model,f1,f2,alg);
                    if (values[alg]) ++positives[alg];
                    if (alg && values[alg] != values[0]) same = 0;
                }
                ++tested;
                if (!same) { ++disagreements; ++angle_diffs; }
                fprintf(log,"%d %d %d %d %d %d %d %d %d %d %d\n",
                        a,h,v,f1,f2,
                        values[0],values[1],values[2],
                        values[3],values[4],values[5]);
                for (face_id = 0; face_id < 2; ++face_id) {
                    face = face_id ? f2 : f1;
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
        fprintf(log,
          "ANGLE %d h=%d v=%d differences=%lu v1_ticks=%ld v2_ticks=%ld v2_max_ticks=%ld fixed64_ticks=%ld raster_ticks=%ld raster_x10_ticks=%ld\n",
          a,h,v,angle_diffs,angle_ticks[0],angle_ticks[1],
          angle_ticks[2],angle_ticks[3],angle_ticks[4],angle_ticks[5]);
        printf("Angle %d/%d: %lu pairs exported\n",a+1,angle_count,
               (unsigned long)nf*(unsigned long)(nf-1)/2UL);
    }
    fprintf(log,"\nSUMMARY\nPairs tested: %lu\nDisagreements: %lu\n",
            tested,disagreements);
    fprintf(log,"V1 ticks: %ld; positives: %lu\n",total_ticks[0],positives[0]);
    fprintf(log,"V2 ticks: %ld; positives: %lu\n",total_ticks[1],positives[1]);
    fprintf(log,"V2_max ticks: %ld; positives: %lu\n",total_ticks[2],positives[2]);
    fprintf(log,"Fixed64 ticks: %ld; positives: %lu\n",total_ticks[3],positives[3]);
    fprintf(log,"Raster ticks: %ld; positives: %lu\n",total_ticks[4],positives[4]);
    fprintf(log,"Raster_x10 ticks: %ld; positives: %lu\n",total_ticks[5],positives[5]);
    fprintf(log,"Timing checksum: %lu\n",timing_sink);
    fclose(log);
    params->angle_h = old_h;
    params->angle_v = old_v;
    printf("Done. Results: overlap.all.txt\n");
}
