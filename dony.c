/* =====================================================================
 * Optimized helpers for painter_geoV3 
 * ---------------------------------------------------------------------
 * - Precomputed per-face epsilon (avoids sqrt + double in the hot loop)
 * - Lightweight plane-side test
 * - Single initial sort by z_max
 * ===================================================================== */

static Fixed64* plane_eps = NULL;
static int      plane_eps_alloc = 0;

/* Precompute a base epsilon for every face (0.1% of plane normal length,
 * floored at 0.01). Called only when face_count changes. */
static void precompute_plane_epsilons(Model3D* model, int face_count)
{
    FaceArrays3D* faces = &model->faces;

    if (face_count > plane_eps_alloc) {
        free(plane_eps);
        plane_eps = (Fixed64*)malloc((size_t)face_count * sizeof(Fixed64));
        plane_eps_alloc = face_count;
    }

    for (int f = 0; f < face_count; ++f) {
        double fa = FIXED64_TO_FLOAT(faces->plane_a[f]);
        double fb = FIXED64_TO_FLOAT(faces->plane_b[f]);
        double fc = FIXED64_TO_FLOAT(faces->plane_c[f]);
        double n  = sqrt(fa*fa + fb*fb + fc*fc);
        if (n < 0.01) n = 0.01;
        plane_eps[f] = (Fixed64)FLOAT_TO_FIXED((float)(n * 0.001));
    }
}

/* Fast pair epsilon: max of the two precomputed face epsilons */
static Fixed64 pair_epsilon(int f1, int f2)
{
    Fixed64 e1 = plane_eps[f1];
    Fixed64 e2 = plane_eps[f2];
    return (e1 > e2) ? e1 : e2;
}

/* Plane-side test (faithful to Dony's original logic).
 * Returns 1 if no vertex of other_face lies clearly on the forbidden side
 * of plane_face. Vertices inside +/- epsilon are ignored. */
static int plane_side_test(Model3D* model, int plane_face, int other_face,
                           int sign_ref, Fixed64 epsilon)
{
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx  = &model->vertices;

    Fixed64 A = faces->plane_a[plane_face];
    Fixed64 B = faces->plane_b[plane_face];
    Fixed64 C = faces->plane_c[plane_face];
    Fixed64 D = faces->plane_d[plane_face];

    int d_sign   = (D > 0) - (D < 0);
    int ref_sign = sign_ref * d_sign;

    int n      = faces->vertex_count[other_face];
    int offset = faces->vertex_indices_ptr[other_face];

    for (int i = 0; i < n; ++i) {
        int v = faces->vertex_indices_buffer[offset + i] - 1;

        Fixed64 r = ((A * vtx->xo[v]) >> FIXED_SHIFT)
                  + ((B * vtx->yo[v]) >> FIXED_SHIFT)
                  + ((C * vtx->zo[v]) >> FIXED_SHIFT)
                  + D;

        if (r > -epsilon && r < epsilon) continue;

        int r_sign = (r > 0) - (r < 0);
        if (r_sign == ref_sign) return 0;
    }
    return 1;
}

/* Simple insertion sort by descending z_max */
static void sort_by_zmax(int* sorted, Fixed32* z_max, int nf)
{
    for (int i = 1; i < nf; ++i) {
        int key = sorted[i];
        Fixed32 kz = z_max[key];
        int j = i - 1;
        while (j >= 0 && z_max[sorted[j]] < kz) {
            sorted[j + 1] = sorted[j];
            --j;
        }
        sorted[j + 1] = key;
    }
}

/* =====================================================================
 * painter_geoV3
 * ---------------------------------------------------------------------
 * Selection-sort style painter's algorithm (Dony) with restart.
 * Face splitting disabled. Unresolved conflicts use ray_cast_hierarchical.
 *
 * Infinite-loop protection:
 *   - dd_flag        : a face that has been demoted cannot be promoted
 *                      again by plane tests in the same pass.
 *   - dd_flag_raycast: same protection for ray-cast triggered swaps.
 *   - permutes_this_pass counter as final safety net.
 * ===================================================================== */

 void painter_geoV3(Model3D* model, int face_count)
{
    FaceArrays3D* faces = &model->faces;

    /*
     * Keep frequently accessed arrays in local pointers.
     *
     * The inner selection loop can execute more than 100,000 times.
     * Avoid repeatedly dereferencing faces->xxx in that hot loop.
     */
    int* sorted       = faces->sorted_face_indices;

    Fixed32* z_min    = faces->z_min;
    Fixed32* z_max    = faces->z_max;

    int* minx         = faces->minx;
    int* maxx         = faces->maxx;
    int* miny         = faces->miny;
    int* maxy         = faces->maxy;

    static int* dd_flag = NULL;
    static int* dd_flag_raycast = NULL;
    static int  allocated = 0;

    int nf = face_count;
    int fs = 0;

    /*
     * Allocate the loop-protection arrays only when necessary.
     */
    if (face_count > allocated) {
        free(dd_flag);
        free(dd_flag_raycast);

        dd_flag =
            (int*)calloc((size_t)face_count, sizeof(int));

        dd_flag_raycast =
            (int*)calloc((size_t)face_count, sizeof(int));

        allocated = face_count;

        if (!dd_flag || !dd_flag_raycast) {
            fprintf(stderr,
                    "painter_geoV3: allocation failed\n");
            exit(1);
        }
    }
    else {
        memset(dd_flag,
               0,
               (size_t)face_count * sizeof(int));

        memset(dd_flag_raycast,
               0,
               (size_t)face_count * sizeof(int));
    }

    /*
     * Plane epsilon preprocessing unchanged.
     */
    {
        static int last_count = -1;

        if (face_count != last_count) {
            precompute_plane_epsilons(model, face_count);
            last_count = face_count;
        }
    }

    /*
     * Initial Z ordering.
     */
    sort_by_zmax(sorted, z_max, nf);

    /*
     * Dony selection-sort style painter algorithm.
     *
     * IMPORTANT:
     * p is compared with ALL remaining faces.
     *
     * When another face replaces p, the scan restarts from fs + 1.
     */
    while (fs < nf - 1) {

        int i;
        int p;
        int f2;
        int permutes_this_pass;

        /*
         * Reset protection flags for the unsorted part.
         */
        for (i = fs; i < nf; ++i) {
            int f = sorted[i];

            dd_flag[f] = 0;
            dd_flag_raycast[f] = 0;
        }

        p = sorted[fs];
        f2 = fs + 1;
        permutes_this_pass = 0;

        /*
         * Cache values belonging to the current candidate p.
         *
         * They remain valid until p changes following a permutation.
         */
        {
            Fixed32 p_zmin = z_min[p];

            int p_minx = minx[p];
            int p_maxx = maxx[p];
            int p_miny = miny[p];
            int p_maxy = maxy[p];

            while (f2 < nf) {

                int q = sorted[f2];

                /*
                 * --------------------------------------------------
                 * TEST 1: Z extent
                 * --------------------------------------------------
                 *
                 * Most pairs are rejected here.
                 *
                 * Use cached p_zmin instead of repeatedly evaluating
                 * z_min[p].
                 */
                if (z_max[q] <= p_zmin)
                    goto skip_q;

                /*
                 * --------------------------------------------------
                 * TEST 2: X extent
                 * --------------------------------------------------
                 *
                 * Original code computed dx.  Only its sign was
                 * subsequently used, so compare the bounds directly.
                 *
                 * Equivalent to:
                 *
                 *   if (dx >= 0) goto skip_q;
                 */
                {
                    int q_minx = minx[q];
                    int q_maxx = maxx[q];

                    if (p_maxx <= q_minx ||
                        q_maxx <= p_minx)
                        goto skip_q;
                }

                /*
                 * --------------------------------------------------
                 * TEST 3: Y extent
                 * --------------------------------------------------
                 *
                 * Same optimization as X.
                 */
                {
                    int q_miny = miny[q];
                    int q_maxy = maxy[q];

                    if (p_maxy <= q_miny ||
                        q_maxy <= p_miny)
                        goto skip_q;
                }

                /*
                 * Only a very small fraction of all pairs reaches
                 * this point.
                 */
                {
                    Fixed64 eps_pq = pair_epsilon(p, q);

                    /*
                     * TEST 4
                     */
                    if (plane_side_test(model,
                                        q,
                                        p,
                                        1,
                                        eps_pq))
                        goto skip_q;

                    /*
                     * TEST 5
                     */
                    if (plane_side_test(model,
                                        p,
                                        q,
                                        -1,
                                        eps_pq))
                        goto skip_q;

                    /*
                     * Dony ordering decision.
                     */
                    {
                        int t1 =
                            plane_side_test(model,
                                            p,
                                            q,
                                            1,
                                            eps_pq);

                        int t2 =
                            plane_side_test(model,
                                            q,
                                            p,
                                            -1,
                                            eps_pq);

                        /*
                         * A face which has already been demoted
                         * cannot be promoted again during this pass.
                         */
                        if (!dd_flag[q]) {
                            if (t1 != 0 || t2 != 0)
                                goto do_permute;
                        }

                        goto do_split_giveup;
                    }
                }


do_permute:

                /*
                 * Infinite-loop safety protection.
                 */
                if (++permutes_this_pass > face_count)
                    goto do_split_giveup;

                /*
                 * q replaces the current candidate p.
                 */
                {
                    int tmp = sorted[fs];

                    sorted[fs] = sorted[f2];
                    sorted[f2] = tmp;
                }

                /*
                 * The old candidate has been demoted.
                 */
                dd_flag[p] = 1;

                /*
                 * q is now the current candidate.
                 */
                p = sorted[fs];

                /*
                 * Update ALL cached values associated with p.
                 */
                p_zmin = z_min[p];

                p_minx = minx[p];
                p_maxx = maxx[p];

                p_miny = miny[p];
                p_maxy = maxy[p];

                /*
                 * Critical Dony behavior:
                 *
                 * restart the complete selection scan for the
                 * new candidate.
                 */
                f2 = fs + 1;

                continue;


do_split_giveup:

                /*
                 * No face splitting in this implementation.
                 *
                 * Keep the existing ray-cast fallback unchanged.
                 */
                if (!dd_flag_raycast[q] &&
                    projected_polygons_overlap(model, p, q)) {

                    int rc =
                        ray_cast_hierarchical(model, p, q);

                    if (rc < 0) {

                        if (++permutes_this_pass > face_count)
                            goto skip_q;

                        /*
                         * q replaces p following the ray-cast.
                         */
                        {
                            int tmp = sorted[fs];

                            sorted[fs] = sorted[f2];
                            sorted[f2] = tmp;
                        }

                        dd_flag[p] = 1;
                        dd_flag_raycast[p] = 1;

                        p = sorted[fs];

                        /*
                         * p changed: refresh the cached values.
                         */
                        p_zmin = z_min[p];

                        p_minx = minx[p];
                        p_maxx = maxx[p];

                        p_miny = miny[p];
                        p_maxy = maxy[p];

                        /*
                         * Restart selection from the beginning.
                         */
                        f2 = fs + 1;

                        continue;
                    }

                    if (rc > 0)
                        goto skip_q;
                }

                /*
                 * Relation remains unresolved.
                 *
                 * Since face splitting is deliberately disabled,
                 * preserve the current ordering.
                 */


skip_q:
                ++f2;
            }
        }

        ++fs;
    }
}


