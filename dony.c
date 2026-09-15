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
    int* sorted = faces->sorted_face_indices;

    static int* dd_flag = NULL;
    static int* dd_flag_raycast = NULL;
    static int  allocated = 0;

    if (face_count > allocated) {
        free(dd_flag);
        free(dd_flag_raycast);
        dd_flag         = (int*)calloc((size_t)face_count, sizeof(int));
        dd_flag_raycast = (int*)calloc((size_t)face_count, sizeof(int));
        allocated = face_count;
        if (!dd_flag || !dd_flag_raycast) {
            fprintf(stderr, "painter_geoV3: allocation failed\n");
            exit(1);
        }
    } else {
        memset(dd_flag, 0, (size_t)face_count * sizeof(int));
        memset(dd_flag_raycast, 0, (size_t)face_count * sizeof(int));
    }

    static int last_count = -1;
    if (face_count != last_count) {
        precompute_plane_epsilons(model, face_count);
        last_count = face_count;
    }

    int nf = face_count;
    int fs = 0;
    int split_giveup_count = 0;

    sort_by_zmax(sorted, faces->z_max, nf);

    while (fs < nf - 1) {

        for (int i = fs; i < nf; ++i) {
            dd_flag[sorted[i]] = 0;
            dd_flag_raycast[sorted[i]] = 0;
        }

        int p  = sorted[fs];
        int f2 = fs + 1;
        int permutes_this_pass = 0;   /* safety counter */

        while (f2 < nf) {
            int q = sorted[f2];

            /* ---- TEST 1: Z extent ---- */
            if (faces->z_max[q] <= faces->z_min[p]) goto skip_q;

            /* ---- TEST 2: X minimax ---- */
            {
                int dx = (faces->maxx[p] > faces->maxx[q])
                       ? (faces->minx[p] - faces->maxx[q])
                       : (faces->minx[q] - faces->maxx[p]);
                if (dx >= 0) goto skip_q;
            }

            /* ---- TEST 3: Y minimax ---- */
            {
                int dy = (faces->maxy[p] > faces->maxy[q])
                       ? (faces->miny[p] - faces->maxy[q])
                       : (faces->miny[q] - faces->maxy[p]);
                if (dy >= 0) goto skip_q;
            }

            Fixed64 eps_pq = pair_epsilon(p, q);

            /* ---- TEST 4 ---- */
            if (plane_side_test(model, q, p, 1, eps_pq)) goto skip_q;

            /* ---- TEST 5 ---- */
            if (plane_side_test(model, p, q, -1, eps_pq)) goto skip_q;

            /* ---- Decision (STRICTLY protected against already-demoted faces) ---- */
            {
                int t1 = plane_side_test(model, p, q, 1, eps_pq);
                int t2 = plane_side_test(model, q, p, -1, eps_pq);

                /* Only allow permutation if q has NEVER been demoted in this pass */
                if (!dd_flag[q]) {
                    if (t1 != 0 || t2 != 0) {
                        goto do_permute;
                    }
                }

                /* q is already demoted or tests are inconclusive → give up path */
                goto do_split_giveup;
            }

        do_permute:
            {
                /* Hard safety: never allow more permutations than faces */
                if (++permutes_this_pass > face_count) {
                    goto do_split_giveup;
                }

                int tmp = sorted[fs];
                sorted[fs] = sorted[f2];
                sorted[f2] = tmp;

                dd_flag[p] = 1;          /* mark the old candidate as demoted */
                p  = sorted[fs];
                f2 = fs + 1;
                continue;
            }

        do_split_giveup:
            if (!dd_flag_raycast[q] && projected_polygons_overlap(model, p, q)) {
                int rc = ray_cast_hierarchical(model, p, q);
                if (rc < 0) {
                    if (++permutes_this_pass > face_count) {
                        goto skip_q;
                    }
                    int tmp = sorted[fs];
                    sorted[fs] = sorted[f2];
                    sorted[f2] = tmp;
                    dd_flag[p] = 1;
                    dd_flag_raycast[p] = 1;
                    p  = sorted[fs];
                    f2 = fs + 1;
                    continue;
                }
                if (rc > 0) goto skip_q;
            }

            split_giveup_count++;
            goto skip_q;

        skip_q:
            f2++;
        }

        fs++;
    }

    /* printf("split_giveup_count = %d\n", split_giveup_count); */
}