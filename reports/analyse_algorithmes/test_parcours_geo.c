
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>
typedef int32_t Fixed32;
#define FLOAT_TO_FIXED(x) ((Fixed32)((x)*65536))
typedef struct{int*sorted_face_indices,*display_flag,*vertex_count,*vertex_indices_ptr,*vertex_indices_buffer,*minx,*maxx,*miny,*maxy;Fixed32*z_mean,*z_min,*z_max;int64_t*pa,*pb,*pc,*pd;}FaceArrays3D;
typedef struct{FaceArrays3D faces;Fixed32*xo,*yo,*zo;}Model3D;
typedef struct{int face1,face2;int8_t relation,occupied;}PairCacheEntry;
typedef struct{PairCacheEntry*slots;int capacity;}PairCache;
typedef struct{int face1,face2;}InconclusivePair;
static int cull_back_faces=0,inconclusive_pairs_count,inconclusive_pairs_capacity;static InconclusivePair*inconclusive_pairs;
static long long find_calls,find_probes,insert_probes,full_misses,dropped,passes,visits,swaps,no_bbox_swaps,plane_calls;
static FaceArrays3D*current;
static int cmp(const void*a,const void*b){int x=*(const int*)a,y=*(const int*)b;return current->z_mean[x]>current->z_mean[y]?-1:current->z_mean[x]<current->z_mean[y]?1:x-y;}
static void painter_newell_sancha_fast(Model3D*m,int n){int i;for(i=0;i<n;i++)m->faces.sorted_face_indices[i]=i;current=&m->faces;qsort(m->faces.sorted_face_indices,n,sizeof(int),cmp);}
static int geometric_face_relation(Model3D*m,int a,int b){int obs,same=1,opp=1,i;int64_t eps=9750,acc;if(!m)return 0;plane_calls++;FaceArrays3D*f=&m->faces;obs=f->pd[a]>eps?1:f->pd[a]<-eps?-1:0;if(!obs)return 0;for(i=0;i<f->vertex_count[b];i++){int v=f->vertex_indices_buffer[f->vertex_indices_ptr[b]+i];acc=(f->pa[a]*m->xo[v]>>16)+(f->pb[a]*m->yo[v]>>16)+(f->pc[a]*m->zo[v]>>16)+f->pd[a];int side=acc>eps?1:acc<-eps?-1:0;if(!side)continue;if(side==obs)opp=0;else same=0;if(!same&&!opp)return 0;}return same?-1:opp?1:0;}
static int projected_polygons_overlap(Model3D*m,int a,int b){(void)m;(void)a;(void)b;return 0;}
static int ray_cast_hierarchical(Model3D*m,int a,int b){(void)m;(void)a;(void)b;return 0;}
static int bbox(Model3D*m,int a,int b){FaceArrays3D*f=&m->faces;return !(f->maxx[a]<=f->minx[b]||f->maxx[b]<=f->minx[a]||f->maxy[a]<=f->miny[b]||f->maxy[b]<=f->miny[a]);}
static PairCache* pair_cache_create(int capacity) {
    /* Allouer la structure puis ses slots remis à zéro ; libérer la
     * structure si la seconde allocation échoue.
     */
    PairCache* c = (PairCache*)malloc(sizeof(PairCache));
    if (!c) return NULL;
    c->capacity = capacity;
    c->slots = (PairCacheEntry*)calloc(capacity, sizeof(PairCacheEntry));
    if (!c->slots) { free(c); return NULL; }
    return c;
}
static int pair_cache_slot(PairCache* c, int f1, int f2) {

    /* Normaliser les identifiants et réduire le hash modulo la capacité. */
    int a = (f1 < f2) ? f1 : f2;
    int b = (f1 < f2) ? f2 : f1;
    unsigned int h = (unsigned int)(a * 2654435761u ^ b * 2246822519u);
    return (int)(h % (unsigned int)c->capacity);
}
static int pair_cache_find(PairCache* c, int f1, int f2) {
    int slot, i;
    if (!c) return 0;
    find_calls++;
    slot = pair_cache_slot(c, f1, f2);
    /* Sonder huit cases au maximum, comparer les clés exactes et réorienter
     * le résultat.
     */
    for (i = 0; i < 8; i++) {
        find_probes++;
        int pos = (slot + i) % c->capacity;
        PairCacheEntry* e = &c->slots[pos];
        if (!e->occupied) return 0;
        if ((e->face1 == f1 && e->face2 == f2)) return  e->relation;
        if ((e->face1 == f2 && e->face2 == f1)) return -e->relation;
    }
    full_misses++;
    return 0;
}
static void pair_cache_insert(PairCache* c, int f1, int f2, int8_t relation) {
    int slot, i;
    if (!c) return;
    slot = pair_cache_slot(c, f1, f2);
    /* Réutiliser le premier slot vide ou la paire déjà présente, dans l'un
     * ou l'autre sens.
     */
    for (i = 0; i < 8; i++) {
        insert_probes++;
        int pos = (slot + i) % c->capacity;
        PairCacheEntry* e = &c->slots[pos];
        if (!e->occupied ||
            (e->face1 == f1 && e->face2 == f2) ||
            (e->face1 == f2 && e->face2 == f1)) {
            e->face1    = f1;
            e->face2    = f2;
            e->relation = relation;
            e->occupied = 1;
            return;
        }
    }
    dropped++;
}
static void pair_cache_destroy(PairCache* c) {
    /* Libérer les slots avant la structure propriétaire. */
    if (c) { free(c->slots); free(c); }
}
void painter_geoV2(Model3D* model, int face_count) {
    FaceArrays3D* faces = &model->faces;
    int i;
    Fixed32* face_zmean = faces->z_mean;
    if (!face_zmean) return;
    Fixed32* face_zmin = faces->z_min;
    Fixed32* face_zmax = faces->z_max;

    /* Réinitialiser les relations et construire l'ordre initial FAST. */
    (void)geometric_face_relation(NULL, 0, 0);

    painter_newell_sancha_fast(model, face_count);
    int visible_count = face_count;
    if (cull_back_faces) {
        visible_count = 0;
        for (i = 0; i < face_count; ++i) {
            if (faces->display_flag[i]) ++visible_count;
        }
    }

    int swap_count = 0;
    int swapped;

    /* Chercher une réserve de cache adaptée ; aucun passage géométrique si
     * toutes échouent.
     */
    PairCache* cache = pair_cache_create(4096);
    if (!cache) {

        cache = pair_cache_create(2048);
        if (!cache)
            cache = pair_cache_create(1024);
    }
    if (!cache) {
        printf("painter_geoV2: warning - no pair cache, geometric pass disabled\n");
        return;
    }

    /* Recréer le diagnostic avec une capacité de quatre paires par face. */
    if (inconclusive_pairs) {
        free(inconclusive_pairs);
        inconclusive_pairs = NULL;
    }
    inconclusive_pairs_capacity = face_count * 4;
    inconclusive_pairs_count = 0;
    if (inconclusive_pairs_capacity > 0) {
        inconclusive_pairs = (InconclusivePair*)malloc(
            inconclusive_pairs_capacity * sizeof(InconclusivePair));
        if (!inconclusive_pairs)
            inconclusive_pairs_capacity = 0;
    }
    int limit = visible_count - 1;
    int new_limit;
    const int MAX_PASSES = visible_count * 2;
    int pass = 0;
    /* Parcourir les voisins non mémorisés jusqu'à stabilisation ou limite de
     * sécurité.
     */
    do {
        swapped = 0;
        new_limit = 0;
        ++pass;
        passes++;
        for (i = 0; i < limit; ++i) {
            visits++;
            int f1 = faces->sorted_face_indices[i];
            int f2 = faces->sorted_face_indices[i + 1];
            if (pair_cache_find(cache, f1, f2) != 0)
                continue;

            /* Essayer les plans dans les deux sens avant les replis projetés. */
            int geo = geometric_face_relation(model, f1, f2);
            if (geo == -1) {
                pair_cache_insert(cache, f1, f2, 1);
                continue;
            }
            if (geo == 1)
                goto do_swap;
            int geo2 = geometric_face_relation(model, f2, f1);
            if (geo2 == -1)
                goto do_swap;
            if (geo2 == 1) {
                pair_cache_insert(cache, f1, f2, 1);
                continue;
            }

            /* Éviter le rayon pour les intervalles Z disjoints ; tester
             * sinon le chevauchement.
             */
            int z_disjoint = 0;
            if (face_zmin && face_zmax) {
                if (face_zmax[f1] <= face_zmin[f2] ||
                    face_zmax[f2] <= face_zmin[f1])
                    z_disjoint = 1;
            } else {
                Fixed32 dz = face_zmean[f1] - face_zmean[f2];
                if (dz > FLOAT_TO_FIXED(1.0f) || dz < FLOAT_TO_FIXED(-1.0f))
                    z_disjoint = 1;
            }
            if (!z_disjoint && projected_polygons_overlap(model, f1, f2)) {
                int rc = ray_cast_hierarchical(model, f1, f2);
                if (rc < 0)
                    goto do_swap;
                if (rc > 0) {
                    pair_cache_insert(cache, f1, f2, 1);
                    continue;
                }
            }

            if (inconclusive_pairs &&
                inconclusive_pairs_count < inconclusive_pairs_capacity) {
                inconclusive_pairs[inconclusive_pairs_count].face1 = f1;
                inconclusive_pairs[inconclusive_pairs_count].face2 = f2;
                inconclusive_pairs_count++;
            }
            pair_cache_insert(cache, f1, f2, 1);
            continue;
        /* Échanger les voisins, mémoriser le sens obtenu et réduire la
         * prochaine passe.
         */
        do_swap:
            {
                int tmp = faces->sorted_face_indices[i];
                faces->sorted_face_indices[i]     = faces->sorted_face_indices[i + 1];
                faces->sorted_face_indices[i + 1] = tmp;
                swapped = 1;
                swap_count++;
                swaps++;
                if(!bbox(model,f1,f2)) no_bbox_swaps++;
                pair_cache_insert(cache, f2, f1, 1);
                new_limit = i;
            }
        }
        limit = new_limit;
    } while (swapped && limit > 0 && pass < MAX_PASSES);
    if (pass >= MAX_PASSES) {
        printf("painter_geoV2: safety limit reached (%d passes)\n", pass);
    }
    /* Libérer le cache propre à cet appel. */
    pair_cache_destroy(cache);

}
int main(void){int n,v,i,j,offset=0;Model3D m;memset(&m,0,sizeof m);FaceArrays3D*f=&m.faces;scanf("%d %d",&v,&n);
#define ALLOC(field,type,count) f->field=calloc(count,sizeof(type))
ALLOC(sorted_face_indices,int,n);ALLOC(display_flag,int,n);ALLOC(vertex_count,int,n);ALLOC(vertex_indices_ptr,int,n);ALLOC(vertex_indices_buffer,int,n*16);ALLOC(minx,int,n);ALLOC(maxx,int,n);ALLOC(miny,int,n);ALLOC(maxy,int,n);ALLOC(z_mean,Fixed32,n);ALLOC(z_min,Fixed32,n);ALLOC(z_max,Fixed32,n);ALLOC(pa,int64_t,n);ALLOC(pb,int64_t,n);ALLOC(pc,int64_t,n);ALLOC(pd,int64_t,n);
m.xo=calloc(v,sizeof(Fixed32));m.yo=calloc(v,sizeof(Fixed32));m.zo=calloc(v,sizeof(Fixed32));for(i=0;i<v;i++)scanf("%d %d %d",&m.xo[i],&m.yo[i],&m.zo[i]);
for(i=0;i<n;i++){scanf("%d %d %d %d %d %d %d %"SCNd64" %"SCNd64" %"SCNd64" %"SCNd64" %d",&f->z_min[i],&f->z_max[i],&f->z_mean[i],&f->minx[i],&f->maxx[i],&f->miny[i],&f->maxy[i],&f->pa[i],&f->pb[i],&f->pc[i],&f->pd[i],&f->vertex_count[i]);f->vertex_indices_ptr[i]=offset;for(j=0;j<f->vertex_count[i];j++)scanf("%d",&f->vertex_indices_buffer[offset++]);}
painter_geoV2(&m,n);printf("%lld %lld %lld %lld %lld %lld %lld %lld %lld %lld\n",passes,visits,find_calls,find_probes,insert_probes,full_misses,dropped,swaps,no_bbox_swaps,plane_calls);return 0;}
