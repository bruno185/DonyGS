/* GEOV3 painter - cleaned final version.
 * Geometric tests use geometric_face_relation() in both directions. 
 Inspired by Robert Dony's algorithm. */

/* Uncomment to use the original C depth/bounds candidate scan. */
// #define GEOV3_DISABLE_ASM_SCAN
#if !defined(GEOV3_DISABLE_ASM_SCAN)
#include "dony_scan_asm.c"
#endif

/* Ray-cast cycle guard: track unordered face pairs, not individual faces.
 * Each selection pass can perform at most nf permutations, so nf entries
 * suffice for all ray-cast swaps. Hash collisions use exact-key chaining;
 * they never suppress an unrelated pair. Separate arrays stay small on IIGS.
 */
#define DONY_RAY_PAIR_BUCKETS 256
typedef struct {
    unsigned int *first, *second, *next;
    unsigned int heads[DONY_RAY_PAIR_BUCKETS];
    unsigned int stamps[DONY_RAY_PAIR_BUCKETS];
    int capacity;
    unsigned int count;
} DonyRayPairs;

static int dony_ray_pairs_reserve(DonyRayPairs *cache, int nf)
{
    unsigned int *a, *b, *next;
    if (nf <= cache->capacity) return 1;
    a = (unsigned int*)malloc((size_t)nf * sizeof(unsigned int));
    b = (unsigned int*)malloc((size_t)nf * sizeof(unsigned int));
    next = (unsigned int*)malloc((size_t)nf * sizeof(unsigned int));
    if (!a || !b || !next) {
        free(a); free(b); free(next);
        return 0;
    }
    free(cache->first); free(cache->second); free(cache->next);
    cache->first = a; cache->second = b; cache->next = next;
    cache->capacity = nf;
    cache->count = 0;
    memset(cache->stamps, 0, sizeof(cache->stamps));
    return 1;
}

static int dony_ray_pair_seen(DonyRayPairs *cache, int p, int q,
                              unsigned int generation)
{
    unsigned int a=(unsigned int)p,b=(unsigned int)q,bucket;
    if(a>b){unsigned int swap=a;a=b;b=swap;}
    bucket=(((a << 5)+a)^b) & (DONY_RAY_PAIR_BUCKETS-1);
    unsigned int entry;
    if (cache->stamps[bucket] != generation) return 0;
    entry = cache->heads[bucket];
    while (entry) {
        unsigned int i = entry - 1;
        if (cache->first[i] == a && cache->second[i] == b) return 1;
        entry = cache->next[i];
    }
    return 0;
}

static void dony_ray_pair_record(DonyRayPairs *cache, int p, int q,
                                 unsigned int generation)
{
    unsigned int a=(unsigned int)p,b=(unsigned int)q,bucket;
    if(a>b){unsigned int swap=a;a=b;b=swap;}
    bucket=(((a << 5)+a)^b) & (DONY_RAY_PAIR_BUCKETS-1);
    unsigned int i = cache->count++;
    /* Caller checks the nf permutation limit before recording a new swap. */
    cache->first[i] = a;
    cache->second[i] = b;
    cache->next[i] = (cache->stamps[bucket] == generation) ? cache->heads[bucket] : 0;
    cache->heads[bucket] = i + 1;
    cache->stamps[bucket] = generation;
}

/* Stable binary insertion sort. Keep the linear fast path for an already
 * ordered prefix, locate an out-of-order key in logarithmic comparisons, then
 * shift its contiguous index run with memmove. No recursion or extra buffer.
 */
static void sort_by_zmax(int *sorted,Fixed32 *z_max,int nf)
{
    int i;
    for(i=1;i<nf;++i){
        int key=sorted[i];
        Fixed32 depth=z_max[key];
        int lo,hi;
        if(z_max[sorted[i-1]]>=depth)continue;
        lo=0;hi=i;
        while(lo<hi){
            int mid=lo+(hi-lo)/2;
            /* Insert after existing equal keys to preserve stability. */
            if(z_max[sorted[mid]]>=depth)lo=mid+1;
            else hi=mid;
        }
        memmove(&sorted[lo+1],&sorted[lo],(size_t)(i-lo)*sizeof(int));
        sorted[lo]=key;
    }
}

/* Dony-style selection/restart painter.
 * Face splitting is replaced by projected-overlap and ray-cast fallback. */

void painter_geoV3(Model3D* model, int face_count)
{
    FaceArrays3D* faces = &model->faces;
    int* sorted = faces->sorted_face_indices;

    static unsigned int* dd_flag = NULL;
    static DonyRayPairs ray_pairs;
    static int allocated = 0;
    static unsigned int generation = 0;

    int nf = face_count;
    int fs = 0;

    /*
     * The cached plane relation depends on the transformed face/vertex data,
     * which changes every frame. Its cache is keyed only by face indices and
     * observer distance, so clear it before sorting this frame. Passing NULL
     * uses geometric_face_relation()'s existing cache-clear path.
     */
    (void)geometric_face_relation(NULL, 0, 0);

    if (face_count > allocated) {
        free(dd_flag);

        dd_flag = (unsigned int*)calloc((size_t)face_count, sizeof(unsigned int));

        if (!dd_flag || !dony_ray_pairs_reserve(&ray_pairs, face_count)) {
            fprintf(stderr, "painter_geoV3: allocation failed\n");
            exit(1);
        }

        allocated = face_count;
        generation = 0;
    }

    /*
     * Start every frame from the model's canonical face order. The caller's
     * sorted_face_indices buffer is reused between frames and GeoV3 mutates it
     * with swaps; using that previous result as the input makes equal-z_max
     * faces depend on the direction/history of model rotation.
     */
    for (int i = 0; i < nf; ++i)
        sorted[i] = i;

    /* Initial ordering by maximum depth (stable for equal z_max). */
    sort_by_zmax(sorted, faces->z_max, nf);
#if !defined(GEOV3_DISABLE_ASM_SCAN)
    GeoV3_ScanSetup(faces, sorted, nf);
#endif

    while (fs < nf - 1) {
        int p;
        int f2;
        int permutes_this_pass;

        /* A flag is set only when its stamp matches this selection pass.
         * Keep the generation across frames to avoid routine array clearing.
         * Unsigned wrap is defined: clear the entire allocation before reusing
         * stamp 1, including entries outside a temporarily smaller model.
         * Do not advance the generation on a permutation/restart of this pass.
         */
        ++generation;
        if (generation == 0) {
            memset(dd_flag, 0, (size_t)allocated * sizeof(unsigned int));
            memset(ray_pairs.stamps, 0, sizeof(ray_pairs.stamps));
            generation = 1;
        }

        ray_pairs.count = 0; /* Previous pass entries are hidden by generation. */
        p = sorted[fs];
        f2 = fs + 1;
        permutes_this_pass = 0;

        while (f2 < nf) {
            int q;
#if !defined(GEOV3_DISABLE_ASM_SCAN)
            f2 = GeoV3_FindCandidate(faces, p, f2);
            if (f2 >= nf) break;
#endif
            q = sorted[f2];

#if defined(GEOV3_DISABLE_ASM_SCAN)

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

#endif /* GEOV3_DISABLE_ASM_SCAN */

            /*
             * Tests 4-7: geometric plane relations.
             *
             * Each call tests both relations to one face plane. Evaluate
             * the reverse direction only if the first call has not already
             * validated the current ordering.
             */
            {
                int geo_pq;
                int geo_qp;

                geo_pq = geometric_face_relation(model, p, q);
                /* This result validates P-before-Q regardless of the reverse
                 * relation. Avoid computing that relation when it cannot
                 * change the decision. Remaining tests keep their priority.
                 */
                if (geo_pq == -1)
                    goto skip_q;

                geo_qp = geometric_face_relation(model, q, p);

                /*
                 * At least one geometric test validates the current
                 * P-before-Q ordering.
                 */
                if (geo_qp == 1)
                    goto skip_q;

                /*
                 * Geometry requires Q before P.
                 */
                if (geo_pq == 1 || geo_qp == -1) {
                    if (dd_flag[q] != generation)
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

            sorted[fs]=q;
            sorted[f2]=p;

            /*
             * P has been demoted. Restart the comparisons with the
             * new candidate at FS, as in Dony's selection algorithm.
             */
            dd_flag[p] = generation;

            p = q;
            f2 = fs + 1;

            continue;

do_split_giveup:
            /*
             * Face splitting is replaced by a projected-overlap test
             * followed, when necessary, by hierarchical ray casting.
             */
            if (!dony_ray_pair_seen(&ray_pairs, p, q, generation) &&
                projected_polygons_overlap_cached(model, p, q)) {
                int rc;

                rc = ray_cast_hierarchical(model, p, q);

                /*
                 * Q must precede P.
                 */
                if (rc < 0) {
                    if (++permutes_this_pass > face_count)
                        goto skip_q;

                    dony_ray_pair_record(&ray_pairs, p, q, generation);

                    sorted[fs]=q;
                    sorted[f2]=p;

                    dd_flag[p] = generation;


                    p = q;
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
