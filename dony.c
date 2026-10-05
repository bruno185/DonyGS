/* GEOV3 painter - cleaned final version.
 * Geometric tests use geometric_face_relation() in both directions. 
 Inspired by Robert Dony's algorithm. */

/* Stable insertion sort by descending z_max. */
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

/* Dony-style selection/restart painter.
 * Face splitting is replaced by projected-overlap and ray-cast fallback. */

void painter_geoV3(Model3D* model, int face_count)
{
    FaceArrays3D* faces = &model->faces;
    int* sorted = faces->sorted_face_indices;

    static int* dd_flag = NULL;
    static int* dd_flag_raycast = NULL;
    static int allocated = 0;

    int nf = face_count;
    int fs = 0;

    if (face_count > allocated) {
        free(dd_flag);
        free(dd_flag_raycast);

        dd_flag = (int*)calloc((size_t)face_count, sizeof(int));
        dd_flag_raycast = (int*)calloc((size_t)face_count, sizeof(int));

        if (!dd_flag || !dd_flag_raycast) {
            fprintf(stderr, "painter_geoV3: allocation failed\n");
            exit(1);
        }

        allocated = face_count;
    } else {
        memset(dd_flag, 0, (size_t)face_count * sizeof(int));
        memset(dd_flag_raycast, 0, (size_t)face_count * sizeof(int));
    }

    /*
     * Initial ordering by maximum depth.
     */
    sort_by_zmax(sorted, faces->z_max, nf);

    while (fs < nf - 1) {
        int i;
        int p;
        int f2;
        int permutes_this_pass;

        /*
         * Dony's DD flags are local to the current selection pass.
         */
        for (i = fs; i < nf; ++i) {
            dd_flag[sorted[i]] = 0;
            dd_flag_raycast[sorted[i]] = 0;
        }

        p = sorted[fs];
        f2 = fs + 1;
        permutes_this_pass = 0;

        while (f2 < nf) {
            int q = sorted[f2];

            /*
             * Test 1: depth intervals do not overlap.
             */
            if (faces->z_max[q] <= faces->z_min[p])
                goto skip_q;

            /*
             * Test 2: projected X intervals do not overlap.
             */
            {
                int dx;

                dx = (faces->maxx[p] > faces->maxx[q])
                   ? faces->minx[p] - faces->maxx[q]
                   : faces->minx[q] - faces->maxx[p];

                if (dx >= 0)
                    goto skip_q;
            }

            /*
             * Test 3: projected Y intervals do not overlap.
             */
            {
                int dy;

                dy = (faces->maxy[p] > faces->maxy[q])
                   ? faces->miny[p] - faces->maxy[q]
                   : faces->miny[q] - faces->maxy[p];

                if (dy >= 0)
                    goto skip_q;
            }

            /*
             * Tests 4-7: geometric plane relations.
             *
             * Both directions are always evaluated. Each call tests
             * the two possible relations to one face plane, so the
             * four geometric tests are performed.
             */
            {
                int geo_pq;
                int geo_qp;

                geo_pq = geometric_face_relation(model, p, q);
                geo_qp = geometric_face_relation(model, q, p);

                /*
                 * At least one geometric test validates the current
                 * P-before-Q ordering.
                 */
                if (geo_pq == -1 || geo_qp == 1)
                    goto skip_q;

                /*
                 * Geometry requires Q before P.
                 */
                if (geo_pq == 1 || geo_qp == -1) {
                    if (!dd_flag[q])
                        goto do_permute;

                    goto do_split_giveup;
                }

                /*
                 * None of the four geometric tests can establish
                 * an ordering.
                 */
                goto do_split_giveup;
            }

do_permute:
            /*
             * Protect against a permutation cycle.
             */
            if (++permutes_this_pass > face_count)
                goto do_split_giveup;

            {
                int tmp = sorted[fs];
                sorted[fs] = sorted[f2];
                sorted[f2] = tmp;
            }

            /*
             * P has been demoted. Restart the comparisons with the
             * new candidate at FS, as in Dony's selection algorithm.
             */
            dd_flag[p] = 1;

            p = sorted[fs];
            f2 = fs + 1;

            continue;

do_split_giveup:
            /*
             * Face splitting is replaced by a projected-overlap test
             * followed, when necessary, by hierarchical ray casting.
             */
            if (!dd_flag_raycast[q] &&
                projected_polygons_overlap(model, p, q)) {
                int rc;

                rc = ray_cast_hierarchical(model, p, q);

                /*
                 * Q must precede P.
                 */
                if (rc < 0) {
                    if (++permutes_this_pass > face_count)
                        goto skip_q;

                    {
                        int tmp = sorted[fs];
                        sorted[fs] = sorted[f2];
                        sorted[f2] = tmp;
                    }

                    dd_flag[p] = 1;
                    dd_flag_raycast[p] = 1;

                    p = sorted[fs];
                    f2 = fs + 1;

                    continue;
                }

                /*
                 * Current P-before-Q ordering is valid.
                 */
                if (rc > 0)
                    goto skip_q;
            }

skip_q:
            ++f2;
        }

        ++fs;
    }
}
