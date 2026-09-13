/* =====================================================================
 * Précalcul des normes de plans (une seule fois)
 * ===================================================================== */
static Fixed64* plane_norm = NULL;   /* à allouer une fois pour le modèle */

static void precompute_plane_norms(Model3D* model, int face_count)
{
    FaceArrays3D* faces = &model->faces;
    if (!plane_norm) {
        plane_norm = (Fixed64*)malloc((size_t)face_count * sizeof(Fixed64));
        /* gestion d'erreur si besoin */
    }

    for (int f = 0; f < face_count; ++f) {
        double fa = FIXED64_TO_FLOAT(faces->plane_a[f]);
        double fb = FIXED64_TO_FLOAT(faces->plane_b[f]);
        double fc = FIXED64_TO_FLOAT(faces->plane_c[f]);
        double n = sqrt(fa*fa + fb*fb + fc*fc);
        if (n < 0.01) n = 0.01;
        plane_norm[f] = (Fixed64)FLOAT_TO_FIXED((float)(n * 0.001));
        /* on stocke déjà l'epsilon de base (0.1 % de la norme) */
    }
}

/* Version ultra-rapide de l'epsilon (plus de sqrt ni de double) */
static Fixed64 dony_pair_epsilon_fast(int f1, int f2)
{
    Fixed64 e1 = plane_norm[f1];
    Fixed64 e2 = plane_norm[f2];
    return (e1 > e2) ? e1 : e2;
}

/* =====================================================================
 * Test de côté optimisé (moins de calculs, early-out)
 * ===================================================================== */
static int dony_plane_side_test_fast(Model3D* model, int plane_face, int other_face,
                                     int sign_ref, Fixed64 epsilon)
{
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;

    Fixed64 A = faces->plane_a[plane_face];
    Fixed64 B = faces->plane_b[plane_face];
    Fixed64 C = faces->plane_c[plane_face];
    Fixed64 D = faces->plane_d[plane_face];

    int d_sign = (D > 0) - (D < 0);          /* branchless */
    int ref_sign = sign_ref * d_sign;

    int n      = faces->vertex_count[other_face];
    int offset = faces->vertex_indices_ptr[other_face];

    for (int i = 0; i < n; ++i) {
        int v = faces->vertex_indices_buffer[offset + i] - 1;

        /* Calcul Fixed64 soigneux (évite les overflows intermédiaires) */
        Fixed64 r = ((A * vtx->xo[v]) >> FIXED_SHIFT)
                  + ((B * vtx->yo[v]) >> FIXED_SHIFT)
                  + ((C * vtx->zo[v]) >> FIXED_SHIFT)
                  + D;

        if (r > -epsilon && r < epsilon) continue;   /* ambigu → ignore */

        int r_sign = (r > 0) - (r < 0);
        if (r_sign == ref_sign) return 0;            /* trouvé du mauvais côté */
    }
    return 1;
}

/* =====================================================================
 * Tri initial unique + insertion légère (remplace le bubble à chaque fs)
 * ===================================================================== */
static void sort_faces_by_zmax(int* sorted, Fixed32* z_max, int nf)
{
    /* Tri simple et efficace pour n typiquement < 200 sur IIGS */
    /* On peut utiliser un shell sort ou un insertion sort selon n */
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
 * painter_geoV3 optimisé
 * ===================================================================== */
void painter_geoV3(Model3D* model, int face_count)
{
    FaceArrays3D* faces = &model->faces;
    int* sorted = faces->sorted_face_indices;
    long long start_time = 0;
    start_time = GetTick();
    long long end_time = 0;

    /* Buffers statiques (évite calloc/free à chaque frame) */
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
            fprintf(stderr, "painter_geoV3: alloc failed\n");
            exit(1);
        }
    } else {
        memset(dd_flag, 0, (size_t)face_count * sizeof(int));
        memset(dd_flag_raycast, 0, (size_t)face_count * sizeof(int));
    }

    /* Précalcul des epsilons (une seule fois par modèle, ou quand les plans changent) */
    static int last_face_count = -1;
    if (face_count != last_face_count) {
        precompute_plane_norms(model, face_count);
        last_face_count = face_count;
    }

    int nf = face_count;
    int fs = 0;
    int split_giveup_count = 0;

    /* Un seul tri initial par z_max décroissant */
    sort_faces_by_zmax(sorted, faces->z_max, nf);

    while (fs < nf - 1) {
        /* On ne re-trie plus à chaque fs (gros gain) */

        /* Reset des flags uniquement sur la partie non finalisée */
        for (int i = fs; i < nf; ++i) {
            dd_flag[sorted[i]] = 0;
            dd_flag_raycast[sorted[i]] = 0;
        }

        int p = sorted[fs];
        int f2 = fs + 1;

        while (f2 < nf) {
            int q = sorted[f2];

            /* ---- TEST 1 : Z ---- */
            if (faces->z_max[q] <= faces->z_min[p]) goto skip_q;

            /* ---- TEST 2 : X ---- */
            {
                int dx = (faces->maxx[p] > faces->maxx[q])
                       ? (faces->minx[p] - faces->maxx[q])
                       : (faces->minx[q] - faces->maxx[p]);
                if (dx >= 0) goto skip_q;
            }

            /* ---- TEST 3 : Y ---- */
            {
                int dy = (faces->maxy[p] > faces->maxy[q])
                       ? (faces->miny[p] - faces->maxy[q])
                       : (faces->miny[q] - faces->maxy[p]);
                if (dy >= 0) goto skip_q;
            }

            /* Epsilon ultra-rapide */
            Fixed64 eps_pq = dony_pair_epsilon_fast(p, q);

            /* ---- TEST 4 & 5 (ordre important pour early-out) ---- */
            if (dony_plane_side_test_fast(model, q, p, 1, eps_pq)) goto skip_q;
            if (dony_plane_side_test_fast(model, p, q, -1, eps_pq)) goto skip_q;

            /* ---- Décision de permutation (version sécurisée + rapide) ---- */
            {
                int t1 = dony_plane_side_test_fast(model, p, q, 1, eps_pq);
                int t2 = dony_plane_side_test_fast(model, q, p, -1, eps_pq);

                if (!dd_flag[q] && (t1 != 0 || t2 != 0)) {
                    goto do_permute;
                }
                goto do_split_giveup;
            }

        do_permute:
            {
                int tmp = sorted[fs];
                sorted[fs] = sorted[f2];
                sorted[f2] = tmp;
                dd_flag[p] = 1;
                p = sorted[fs];
                f2 = fs + 1;
                continue;
            }

        do_split_giveup:
            if (!dd_flag_raycast[q] && projected_polygons_overlap(model, p, q)) {
                int rc = ray_cast_hierarchical(model, p, q);
                if (rc < 0) {
                    int tmp = sorted[fs];
                    sorted[fs] = sorted[f2];
                    sorted[f2] = tmp;
                    dd_flag[p] = 1;
                    dd_flag_raycast[p] = 1;
                    p = sorted[fs];
                    f2 = fs + 1;
                    continue;
                }
                if (rc > 0) goto skip_q;
            }
            split_giveup_count++;
            /* printf optionnel en debug seulement */
            // printf("p = %d, q = %d\n", p, q);
            goto skip_q;

        skip_q:
            f2++;
        }

        fs++;
    }

    printf("split_giveup_count = %d\n", split_giveup_count);
    /* Pas de free ici → buffers réutilisés */
    /* printf("split_giveup_count = %d\n", split_giveup_count); */
}

