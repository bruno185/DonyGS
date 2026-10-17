/*
 * QD versus SH centroid/raycast comparison for every overlapping face pair.
 * Include this file in engine.c AFTER overlap_raster.c, where the static
 * geometry helpers are visible. Call after the model has been projected
 * and its face bounding boxes updated, while still in text mode.
 * ORCA/C, Apple IIGS.
 */
#include <stdio.h>
#include <stdlib.h>
#include <types.h>
#include <quickdraw.h>
#include <memory.h>
#include <misctool.h>

#define QDSH_DP_ATTRIBUTES 0xC015

/* Return 0=undetermined, 1=f1 closer, 2=f2 closer. */
static int qdsh_order(Model3D *model, int f1, int f2, int x, int y)
{
    int r = ray_cast_at(model, f1, f2, x, y);
    if (r == -1) return 1;
    if (r ==  1) return 2;
    return 0;
}

/* Diagnostic only: write projected vertices of one face. */
static void qdsh_log_vertices(FILE *log, Model3D *model, int f)
{
    FaceArrays3D *faces = &model->faces;
    VertexArrays3D *vtx = &model->vertices;
    int i, off, vi;
    off = faces->vertex_indices_ptr[f];
    fprintf(log, "FACE %d vertices=%d:", f, faces->vertex_count[f]);
    for (i = 0; i < faces->vertex_count[f]; ++i) {
        vi = faces->vertex_indices_buffer[off + i] - 1;
        fprintf(log, "\n    v%d idx=%d 2D=(%d,%d)",
                i, vi, vtx->x2d[vi], vtx->y2d[vi]);
        fprintf(log, " orig=(%.6f,%.6f,%.6f)",
                FIXED_TO_FLOAT(vtx->x[vi]),
                FIXED_TO_FLOAT(vtx->y[vi]),
                FIXED_TO_FLOAT(vtx->z[vi]));
        fprintf(log, " observer=(%.6f,%.6f,%.6f)",
                FIXED_TO_FLOAT(vtx->xo[vi]),
                FIXED_TO_FLOAT(vtx->yo[vi]),
                FIXED_TO_FLOAT(vtx->zo[vi]));
    }
    fprintf(log, "\n");
}

/* Log the two ray/plane distances without changing the raycaster. */
static void qdsh_log_distances(FILE *log, Model3D *model,
                               int f1, int f2, int x, int y,
                               const char *label, int valid)
{
    float t1, t2;
    if (!valid) {
        fprintf(log, "  %s: no point\n", label);
        return;
    }
    if (!ray_cast_distances(model, f1, f2, x, y, &t1, &t2)) {
        fprintf(log, "  %s (%d,%d): ray unavailable\n", label, x, y);
        return;
    }
    fprintf(log, "  %s (%d,%d): t1=%.9g t2=%.9g delta=%.9g\n",
            label, x, y, (double)t1, (double)t2,
            (double)t1 - (double)t2);
}

/* Strict interior check: points on edges count as outside. */
static int qdsh_inside_both(Model3D *model, int f1, int f2,
                            int x, int y)
{
    FaceArrays3D *faces = &model->faces;
    VertexArrays3D *vtx = &model->vertices;
    return point_in_poly_int(x, y, faces, vtx, f1,
                             faces->vertex_count[f1]) &&
           point_in_poly_int(x, y, faces, vtx, f2,
                             faces->vertex_count[f2]);
}

void test_qd_sh_all_pairs(Model3D *model)
{
    Handle dp = NULL;
    FILE *log = NULL;
    Word myID, err;
    int qd_started = 0;
    int n, f1, f2;
    long total_pairs;
    long tick_start, tick_end, tick_start_diag, tick_qd = 0L, tick_sh = 0L;
    long timed_qd_calls = 0L, timed_sh_calls = 0L;
    volatile long timing_sink = 0L;
    int percent;
    long pairs = 0, overlap = 0, both = 0;
    long same = 0, opposite = 0, uncertain = 0;
    long only_qd = 0, only_sh = 0, neither = 0;
    long qd_missing = 0, sh_missing = 0, sh_fallbacks = 0;
    int qx, qy, sx, sy, qok, sok, qo, so, fb;
    int qinside, sinside;
    long long qa, sa;

    if (model == NULL) return;
    n = model->faces.face_count;
    total_pairs = ((long)n * (n - 1)) / 2;
    printf("\nQD/SH pair test: %d faces\n", n);

    log = fopen("QD.SH.LOG", "w");
    if (log == NULL) {
        printf("Cannot open QD.SH.LOG\n");
        return;
    }
    fprintf(log, "QD/SH all overlapping face pairs; %d faces\n", n);
    fprintf(log, "Order: 0=unknown 1=first face closer 2=second face closer\n");
    fprintf(log, "QD point = intersection region bbox center (not area centroid)\n");
    fprintf(log, "f1 f2 QDx QDy QDord SHx SHy SHord SHfallback status\n");
    fprintf(log, "Inside means strictly inside BOTH projected polygons.\n");
    fprintf(log, "orig=x/y/z; observer=xo/yo/zo; 3D values as floats.\n");

    myID = userid();
    dp = NewHandle(0x300L, myID, QDSH_DP_ATTRIBUTES, NULL);
    err = toolerror();
    printf("NewHandle: $%04X\n", err);
    if (err || dp == NULL) goto cleanup;

    /* Same startup arguments as the previously successful text-mode test. */
    QDStartUp((Word)*dp, 0, mode320, myID);
    err = toolerror();
    printf("QDStartUp: $%04X\n", err);
    if (err) goto cleanup;
    qd_started = 1;

    tick_start_diag = GetTick();
    for (f1 = 0; f1 < n; ++f1) {
        /* Update one console line after each completed outer-loop row. */
        for (f2 = f1 + 1; f2 < n; ++f2) {
            ++pairs;
            if (!projected_polygons_overlap(model, f1, f2)) continue;
            ++overlap;
            qx = qy = sx = sy = 0;
            qa = sa = 0;
            fb = 0;
            qok = compute_intersection_centroid_ordered_qd_fixed(
                model, f1, f2, &qx, &qy, &qa) && qa != 0;
            sok = compute_intersection_centroid_ordered_fixed_tryboth(
                model, f1, f2, &sx, &sy, &sa, &fb) && sa != 0;
            if (fb) ++sh_fallbacks;
            if (!qok) ++qd_missing;
            if (!sok) ++sh_missing;
            qo = qok ? qdsh_order(model, f1, f2, qx, qy) : 0;
            so = sok ? qdsh_order(model, f1, f2, sx, sy) : 0;

            if (qo && so) {
                ++both;
                if (qo == so) ++same;
                else ++opposite;
            } else if (qo) ++only_qd;
            else if (so) ++only_sh;
            else ++neither;

            qinside = qok ? qdsh_inside_both(model, f1, f2, qx, qy) : 0;
            sinside = sok ? qdsh_inside_both(model, f1, f2, sx, sy) : 0;

            /* Log only disagreements and undetermined cases. */
            if (qo != so || qo == 0 || so == 0) {
                fprintf(log, "%d %d %d %d %d %d %d %d %d %s\n",
                    f1, f2, qx, qy, qo, sx, sy, so, fb,
                    (qo && so) ? "OPPOSITE" :
                    (qo ? "QD_ONLY" : (so ? "SH_ONLY" : "UNKNOWN")));

                /* Detailed diagnostic for opposing or one-sided orders.
                 * Raycast at both points uses the SAME ray_cast_at().
                 * This isolates point selection from raycast behavior.
                 */
                if (qo != so) {
                    fprintf(log, "  QD valid=%d inside=%d; SH valid=%d inside=%d\n",
                            qok, qinside, sok, sinside);
                    fprintf(log, "  RAY QDPOINT=%d SHPOINT=%d (0 unknown)\n",
                            qok ? qdsh_order(model, f1, f2, qx, qy) : 0,
                            sok ? qdsh_order(model, f1, f2, sx, sy) : 0);
                    fprintf(log, "  QD area2=%lld SH area2=%lld fallback=%d\n",
                            qa, sa, fb);
                    qdsh_log_distances(log, model, f1, f2,
                                       qx, qy, "QD", qok);
                    qdsh_log_distances(log, model, f1, f2,
                                       sx, sy, "SH", sok);
                    qdsh_log_vertices(log, model, f1);
                    qdsh_log_vertices(log, model, f2);
                }
            }
        }
        percent = total_pairs ? (int)((pairs * 100L) / total_pairs) : 100;
        printf("\rProgress: %3d%% (%ld/%ld pairs)   ",
               percent, pairs, total_pairs);
        fflush(stdout);
    }
    tick_end = GetTick();
    printf("\n");

    /* Timed passes: no console or file I/O within either timed loop.
     * Both passes apply the same overlap filter and call only one
     * intersection implementation.  Timing includes the shared filter.
     */
    printf("Timing QD pass...\n");
    tick_start = GetTick();
    for (f1 = 0; f1 < n; ++f1) {
        for (f2 = f1 + 1; f2 < n; ++f2) {
            if (!projected_polygons_overlap(model, f1, f2)) continue;
            qx = qy = 0;
            qa = 0;
            qok = compute_intersection_centroid_ordered_qd_fixed(
                model, f1, f2, &qx, &qy, &qa) && qa != 0;
            ++timed_qd_calls;
            timing_sink += qok;
        }
    }
    tick_qd = GetTick() - tick_start;

    printf("Timing SH pass...\n");
    tick_start = GetTick();
    for (f1 = 0; f1 < n; ++f1) {
        for (f2 = f1 + 1; f2 < n; ++f2) {
            if (!projected_polygons_overlap(model, f1, f2)) continue;
            sx = sy = fb = 0;
            sa = 0;
            sok = compute_intersection_centroid_ordered_fixed_tryboth(
                model, f1, f2, &sx, &sy, &sa, &fb) && sa != 0;
            ++timed_sh_calls;
            timing_sink += sok;
        }
    }
    tick_sh = GetTick() - tick_start;

    printf("Diagnostic: %ld ticks (%.2f s)\n",
           tick_end - tick_start_diag, (double)(tick_end - tick_start_diag) / 60.0);
    printf("QD pass: %ld ticks (%.3f s)\n",
           tick_qd, (double)tick_qd / 60.0);
    printf("SH pass: %ld ticks (%.3f s)\n",
           tick_sh, (double)tick_sh / 60.0);
    fprintf(log, "\nPERFORMANCE (60 ticks/second)\n");
    fprintf(log, "Diagnostic loop: %ld ticks (%.3f s)\n",
            tick_end - tick_start_diag, (double)(tick_end - tick_start_diag) / 60.0);
    fprintf(log, "QD timed pass: %ld ticks (%.3f s), %ld calls\n",
            tick_qd, (double)tick_qd / 60.0, timed_qd_calls);
    fprintf(log, "SH timed pass: %ld ticks (%.3f s), %ld calls\n",
            tick_sh, (double)tick_sh / 60.0, timed_sh_calls);
    fprintf(log, "Both timed passes include projected_polygons_overlap filtering.\n");
    fprintf(log, "No raycasts, diagnostics, or output inside timed passes.\n");
    fprintf(log, "Timing sink: %ld\n", (long)timing_sink);

    printf("Pairs: %ld  overlap: %ld\n", pairs, overlap);
    printf("Both decide: %ld\n", both);
    printf("Same: %ld  OPPOSITE: %ld\n", same, opposite);
    printf("QD only: %ld  SH only: %ld\n", only_qd, only_sh);
    printf("Neither: %ld\n", neither);
    printf("No QD point: %ld\n", qd_missing);
    printf("No SH point: %ld\n", sh_missing);
    printf("SH fallbacks: %ld\n", sh_fallbacks);

cleanup:
    if (qd_started) {
        QDShutDown();
        printf("QDShutDown: $%04X\n", toolerror());
    }
    if (dp != NULL) {
        DisposeHandle(dp);
        printf("DisposeHandle: $%04X\n", toolerror());
    }
    if (log != NULL) {
        if (fclose(log) != 0) printf("Log close error\n");
        else printf("Details: QD.SH.LOG\n");
    }
}
