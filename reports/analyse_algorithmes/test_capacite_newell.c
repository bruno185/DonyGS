
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
typedef int32_t Fixed32;
typedef struct {int *sorted_face_indices,*display_flag;Fixed32*z_min,*z_max,*z_mean;int*minx,*maxx,*miny,*maxy;}FaceArrays3D;
typedef struct {FaceArrays3D faces;}Model3D;
typedef struct{int face1,face2;}InconclusivePair;
static int cull_back_faces=0,inconclusive_pairs_count=0,inconclusive_pairs_capacity=0;static InconclusivePair *inconclusive_pairs=NULL;
static int fast_calls,measured_candidates,geo_calls;static uint32_t rejected;
static void *audit_realloc(void*p,size_t n){if(n>1000000){rejected=(uint32_t)n;return NULL;}return realloc(p,n);}
#define realloc audit_realloc
static FaceArrays3D *current;
static int cmp(const void*a,const void*b){int x=*(const int*)a,y=*(const int*)b;return current->z_mean[x]>current->z_mean[y]?-1:current->z_mean[x]<current->z_mean[y]?1:x-y;}
static void painter_newell_sancha_fast(Model3D*m,int n){int i;fast_calls++;for(i=0;i<n;i++)m->faces.sorted_face_indices[i]=i;current=&m->faces;qsort(m->faces.sorted_face_indices,n,sizeof(int),cmp);}
static int geometric_face_relation(Model3D*m,int a,int b){(void)a;(void)b;if(m)geo_calls++;return -1;}
static int projected_polygons_overlap(Model3D*m,int a,int b){(void)m;(void)a;(void)b;return 0;}
static int ray_cast_hierarchical(Model3D*m,int a,int b){(void)m;(void)a;(void)b;return 0;}
static int move_element_remove_and_insert_pos(int*a,int n,int j,int i,int*p){(void)a;(void)n;(void)j;(void)i;(void)p;abort();}
void painter_newell_sancha(Model3D* model, int face_count)
{
    FaceArrays3D* faces;
    int* sorted;
    int visible_count;
    typedef struct {
        int face1;
        int face2;
        int next;
    } OrderedPair;
    typedef struct {
        int16_t a;
        int16_t b;
    } CandidatePair;
#define HASH_SIZE_NS       2048
#define HASH_MASK_NS       (HASH_SIZE_NS - 1)
#define ORDERED_MAX_NS     4096

#define PAIR_HASH_NS(a,b) \
    ((((unsigned)(a) * 73856093u) ^ \
      ((unsigned)(b) * 19349663u)) & HASH_MASK_NS)

    static int* s_hash = NULL;
    static OrderedPair* s_ordered = NULL;
    static int* s_touched = NULL;
    static int* s_pos = NULL;
    static int s_pos_capacity = 0;
    static int* s_by_zmin = NULL;
    static int s_by_zmin_capacity = 0;
    static int* s_active = NULL;
    static int s_active_capacity = 0;
    static CandidatePair* s_candidates = NULL;
    static int s_candidates_capacity = 0;

    static unsigned char* s_moved = NULL;
    static int s_moved_capacity = 0;
    int ordered_count = 0;
    int touched_count = 0;
    int* pos;
    int* by_zmin;
    int* active;
    int active_count;
    int candidate_count;
    int changed;
    Fixed32* z_min;
    Fixed32* z_max;
    int* minx;
    int* maxx;
    int* miny;
    int* maxy;

    /* Réinitialiser les relations de plans et établir le préfixe visible de FAST. */
    (void)geometric_face_relation(NULL, 0, 0);

    if (face_count <= 1) {
        painter_newell_sancha_fast(model, face_count);
        return;
    }
    faces = &model->faces;
    painter_newell_sancha_fast(model, face_count);

    visible_count = face_count;
    if (cull_back_faces) {
        int i;
        visible_count = 0;
        for (i = 0; i < face_count; ++i) {
            if (faces->display_flag[i])
                ++visible_count;
        }
    }
    if (visible_count < 2)
        return;
    sorted = faces->sorted_face_indices;

    /* Réserver les tables persistantes de hash et de relations. */
    if (!s_hash) {
        int i;
        s_hash = (int*)malloc(HASH_SIZE_NS * sizeof(int));
        if (!s_hash) {
            painter_newell_sancha_fast(model, face_count);
            return;
        }
        for (i = 0; i < HASH_SIZE_NS; ++i)
            s_hash[i] = -1;
    }
    if (!s_ordered) {
        s_ordered =
            (OrderedPair*)malloc(
                ORDERED_MAX_NS * sizeof(OrderedPair));
        if (!s_ordered) {
            painter_newell_sancha_fast(model, face_count);
            return;
        }
    }
    if (!s_touched) {
        s_touched =
            (int*)malloc(
                ORDERED_MAX_NS * sizeof(int));
        if (!s_touched) {
            painter_newell_sancha_fast(model, face_count);
            return;
        }
    }
    /* Agrandir les buffers de positions et de balayage selon le nombre de faces. */
    if (s_pos_capacity < face_count) {
        int* tmp;
        tmp = (int*)realloc(
            s_pos,
            (size_t)face_count * sizeof(int));
        if (!tmp) {
            painter_newell_sancha_fast(model, face_count);
            return;
        }
        s_pos = tmp;
        s_pos_capacity = face_count;
    }
    if (s_by_zmin_capacity < visible_count) {
        int* tmp;
        tmp = (int*)realloc(
            s_by_zmin,
            (size_t)visible_count * sizeof(int));
        if (!tmp) {
            painter_newell_sancha_fast(model, face_count);
            return;
        }
        s_by_zmin = tmp;
        s_by_zmin_capacity = visible_count;
    }
    if (s_active_capacity < visible_count) {
        int* tmp;
        tmp = (int*)realloc(
            s_active,
            (size_t)visible_count * sizeof(int));
        if (!tmp) {
            painter_newell_sancha_fast(model, face_count);
            return;
        }
        s_active = tmp;
        s_active_capacity = visible_count;
    }
    /* Associer les buffers et tableaux de métriques à la vue courante. */
    pos = s_pos;
    by_zmin = s_by_zmin;
    active = s_active;
    z_min = faces->z_min;
    z_max = faces->z_max;
    minx = faces->minx;
    maxx = faces->maxx;
    miny = faces->miny;
    maxy = faces->maxy;

    {
        int i;
        for (i = 0; i < visible_count; ++i)
            pos[sorted[i]] = i;
    }

    {
        int i;
        int gap;
        for (i = 0; i < visible_count; ++i)
            by_zmin[i] = sorted[i];
        /* Classer les faces visibles par zmin avec un tri de Shell pour
         * préparer le balayage.
         */
        for (gap = visible_count / 2;
             gap > 0;
             gap /= 2) {
            for (i = gap; i < visible_count; ++i) {
                int f;
                Fixed32 z;
                int j;
                f = by_zmin[i];
                z = z_min[f];
                j = i;
                while (j >= gap &&
                       z_min[by_zmin[j - gap]] > z) {
                    by_zmin[j] =
                        by_zmin[j - gap];
                    j -= gap;
                }
                by_zmin[j] = f;
            }
        }
    }

    /* Construire les paires candidates en conservant les intervalles Z
     * encore actifs.
     */
    active_count = 0;
    candidate_count = 0;
    {
        int n;
        for (n = 0; n < visible_count; ++n) {
            int f;
            Fixed32 fmin;
            int dst;
            int a;
            f = by_zmin[n];
            fmin = z_min[f];

            dst = 0;
            for (a = 0; a < active_count; ++a) {
                int q = active[a];
                if (z_max[q] > fmin)
                    active[dst++] = q;
            }
            active_count = dst;

            for (a = 0; a < active_count; ++a) {
                /* Agrandir les listes de candidats et leurs drapeaux de
                 * déplacement ; replier sur FAST en cas d'échec.
                 */
                if (candidate_count >=
                    s_candidates_capacity) {
                    int16_t newcap;
                    CandidatePair* tmp;
                    unsigned char* tmp_moved;
                    if (s_candidates_capacity == 0)
                        newcap = 64;
                    else
                        newcap =
                            s_candidates_capacity * 2;
                    tmp = (CandidatePair*)realloc(
                        s_candidates,
                        (size_t)newcap *
                        sizeof(CandidatePair));
                    if (!tmp) {
                        painter_newell_sancha_fast(
                            model, face_count);
                        return;
                    }
                    s_candidates = tmp;
                    s_candidates_capacity = newcap;

                    if (s_moved_capacity < newcap) {
                        tmp_moved =
                            (unsigned char*)realloc(
                                s_moved,
                                (size_t)newcap *
                                sizeof(unsigned char));
                        if (!tmp_moved) {
                            painter_newell_sancha_fast(
                                model, face_count);
                            return;
                        }
                        s_moved = tmp_moved;
                        s_moved_capacity = newcap;
                    }
                }
                s_candidates[candidate_count].a =
                    active[a];
                s_candidates[candidate_count].b =
                    f;

                s_moved[candidate_count] = 0;
                ++candidate_count;
            }
            active[active_count++] = f;
        }
    }

    measured_candidates=candidate_count;
    /* Reconsidérer les candidats après les insertions en utilisant la table
     * inverse des positions.
     */
    do {
        int c;
        changed = 0;
        for (c = 0; c < candidate_count; ++c) {
            int A;
            int B;
            int i;
            int j;
            int P;
            int Q;
            unsigned h;
            int idx;
            int known;
            A = s_candidates[c].a;
            B = s_candidates[c].b;

            if (pos[A] < pos[B]) {
                i = pos[A];
                j = pos[B];
                P = A;
                Q = B;
            }
            else {
                i = pos[B];
                j = pos[A];
                P = B;
                Q = A;
            }

            h = PAIR_HASH_NS(P, Q);
            idx = s_hash[h];
            known = 0;
            while (idx >= 0) {
                if (s_ordered[idx].face1 == P &&
                    s_ordered[idx].face2 == Q) {
                    known = 1;
                    break;
                }
                idx = s_ordered[idx].next;
            }
            if (known)
                continue;

            /* Essayer les rejets projetés puis les relations de plans avant
             * le test de chevauchement.
             */
            if (maxx[P] <= minx[Q] ||
                maxx[Q] <= minx[P]) {
                goto order_valid;
            }

            if (maxy[P] <= miny[Q] ||
                maxy[Q] <= miny[P]) {
                goto order_valid;
            }

            {
                int geo_pq;
                geo_pq =
                    geometric_face_relation(
                        model, P, Q);
                if (geo_pq == -1)
                    goto order_valid;
            }

            {
                int geo_qp;
                geo_qp =
                    geometric_face_relation(
                        model, Q, P);
                if (geo_qp == 1)
                    goto order_valid;
            }

            if (!projected_polygons_overlap(
                    model, P, Q)) {
                goto order_valid;
            }

            {
                int rc;
                rc = ray_cast_hierarchical(
                    model, P, Q);
                if (rc > 0) {
                    goto order_valid;
                }
                if (rc < 0) {

                    /* Refuser une deuxième insertion causée par la même
                     * paire pour limiter les cycles.
                     */
                    if (s_moved[c]) {
                        goto order_unresolved;
                    }

                    s_moved[c] = 1;
                    goto do_move;
                }

                goto order_unresolved;
            }

/* Avancer Q à la position de P par insertion et mettre à jour toutes les
 * positions déplacées.
 */
do_move:
            /* Insertion stable avec mise à jour de toutes les positions inverses. */
            (void)move_element_remove_and_insert_pos(sorted, face_count, j, i, pos);

            changed = 1;
            continue;

/* Mémoriser une relation résolue dans la table à chaînes. */
order_valid:
            if (ordered_count < ORDERED_MAX_NS) {
                unsigned hh;
                hh = PAIR_HASH_NS(P, Q);
                s_ordered[ordered_count].face1 = P;
                s_ordered[ordered_count].face2 = Q;
                s_ordered[ordered_count].next =
                    s_hash[hh];

                if (s_hash[hh] == -1 &&
                    touched_count < ORDERED_MAX_NS) {
                    s_touched[touched_count++] =
                        (int)hh;
                }
                s_hash[hh] = ordered_count;
                ++ordered_count;
            }
            continue;

/* Consigner l'ambiguïté lorsque le buffer de diagnostic le permet, puis
 * éviter son retraitement.
 */
order_unresolved:
            if (inconclusive_pairs != NULL &&
                inconclusive_pairs_count <
                inconclusive_pairs_capacity) {
                inconclusive_pairs[
                    inconclusive_pairs_count].face1 = P;
                inconclusive_pairs[
                    inconclusive_pairs_count].face2 = Q;
                ++inconclusive_pairs_count;
            }

            if (ordered_count < ORDERED_MAX_NS) {
                unsigned hh;
                hh = PAIR_HASH_NS(P, Q);
                s_ordered[ordered_count].face1 = P;
                s_ordered[ordered_count].face2 = Q;
                s_ordered[ordered_count].next =
                    s_hash[hh];
                if (s_hash[hh] == -1 &&
                    touched_count < ORDERED_MAX_NS) {
                    s_touched[touched_count++] =
                        (int)hh;
                }
                s_hash[hh] = ordered_count;
                ++ordered_count;
            }
        }
    } while (changed);

    {
        int i;
        /* Effacer seulement les buckets utilisés pour préparer l'appel suivant. */
        for (i = 0; i < touched_count; ++i)
            s_hash[s_touched[i]] = -1;
    }
#undef PAIR_HASH_NS
#undef HASH_SIZE_NS
#undef HASH_MASK_NS
#undef ORDERED_MAX_NS
}
int main(void){Model3D m;FaceArrays3D*f=&m.faces;int n,i;scanf("%d",&n);
f->sorted_face_indices=calloc(n,sizeof(int));f->display_flag=calloc(n,sizeof(int));f->z_min=calloc(n,sizeof(Fixed32));f->z_max=calloc(n,sizeof(Fixed32));f->z_mean=calloc(n,sizeof(Fixed32));f->minx=calloc(n,sizeof(int));f->maxx=calloc(n,sizeof(int));f->miny=calloc(n,sizeof(int));f->maxy=calloc(n,sizeof(int));
for(i=0;i<n;i++)scanf("%d %d %d %d %d %d %d",&f->z_min[i],&f->z_max[i],&f->z_mean[i],&f->minx[i],&f->maxx[i],&f->miny[i],&f->maxy[i]);
painter_newell_sancha(&m,n);printf("%d %d %d %u\n",fast_calls,measured_candidates,geo_calls,rejected);return 0;}
