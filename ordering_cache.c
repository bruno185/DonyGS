/* Per-transformation memoization for geometric ordering.
 * Directed pair keys preserve SH's subject/clip evaluation order exactly.
 * Full keys are checked on every hit; collisions only evict entries.
 * Resource-failure results and watchdog rejections are never memoized.
 */
#define ORDER_PAIR_SLOTS 256
#define ORDER_PLANE_BLOCK_SIZE 64
#define ORDER_PLANE_BLOCKS ((MAX_FACES + ORDER_PLANE_BLOCK_SIZE - 1) / ORDER_PLANE_BLOCK_SIZE)
typedef struct {
    int f1, f2;
    Fixed32 scale;
    unsigned int generation, context;
    int result;
} OrderPairEntry;
typedef struct {
    double plane[ORDER_PLANE_BLOCK_SIZE][4];
    unsigned int stamp[ORDER_PLANE_BLOCK_SIZE];
} OrderPlaneBlock;
/* Fixed-plane preparation is invariant for an entire transformed frame.
 * Exact tags include epsilon: observer tolerance can change independently.
 */
#define ORDER_FIXED_PLANE_SLOTS 64
#define ORDER_WINDING_SLOTS 128
#define ORDER_VERTEX_SIDE_SLOTS 64
typedef struct {
    Fixed64 a,b,c,d,epsilon;
    Fixed32 a32,b32,c32;
    int face,observer,asm_ready;
    unsigned int generation,epoch;
    /* Packed one-based vertex ID (13 bits) and exact side (2 bits).
     * 0 = empty, 0x2000 = on-plane, 0x4000 = positive, 0x6000 = negative.
     * Per-plane direct mapping uses 128 bytes; full IDs check collisions.
     */
    unsigned int vertex_sides[ORDER_VERTEX_SIDE_SLOTS];
} OrderFixedPlane;
typedef struct {
    double area2;
    Fixed32 scale;
    int face;
    unsigned int generation;
} OrderWinding;
/* Bounded contour cache: at most 64 * 16 * 4 doubles of payload.
 * Larger polygons use the existing workspace path. Entries retain allocations
 * across frames, but exact generation/scale/count tags govern every reuse.
 */
#define ORDER_CONTOUR_SLOTS 64
#define ORDER_CONTOUR_MAX_VERTICES 16
typedef struct {
    double *data; /* CCW x/y, then edge dx/dy, each capacity doubles. */
    Fixed32 scale;
    int face,count,capacity;
    unsigned int generation,failed_generation;
} OrderContour;
/* Small/medium models retain every directed plane relation in two bits.
 * Codes: 0 unknown, 1 before, 2 indeterminate, 3 after. At 256 faces this
 * costs 16 KB; larger models retain the bounded hash-cache fallback.
 */
#define ORDER_GEOMETRY_DENSE_LIMIT 256
static unsigned char *order_geometry_pairs=NULL;
static unsigned char *order_geometry_rows[ORDER_GEOMETRY_DENSE_LIMIT];
static unsigned int order_geometry_capacity=0,order_geometry_stride=0;
static unsigned int order_geometry_generation=0,order_geometry_epoch=0;
static unsigned int order_geometry_failed=0;
static int order_geometry_faces=0;
static Fixed64 order_geometry_epsilon=0;
static OrderContour order_contours[ORDER_CONTOUR_SLOTS];
static OrderFixedPlane order_fixed_planes[ORDER_FIXED_PLANE_SLOTS];
static OrderFixedPlane *order_asm_packet_plane=NULL;
static unsigned int order_fixed_plane_epoch=0;
static OrderWinding order_windings[ORDER_WINDING_SLOTS];
static OrderPairEntry order_ray_pairs[ORDER_PAIR_SLOTS];
static OrderPairEntry order_overlap_pairs[ORDER_PAIR_SLOTS];
static OrderPlaneBlock *order_planes[ORDER_PLANE_BLOCKS];
static unsigned int order_plane_failed[ORDER_PLANE_BLOCKS];
static Model3D *order_cache_owner = NULL;
static unsigned int order_cache_generation = 0;
static double *order_sh_workspace = NULL;
static long order_sh_capacity = 0;

/* Optional counters: enable in the compilation unit before engine.c. */
#if defined(GEO_ORDER_PROFILE)
static struct {
    unsigned long ray_calls, ray_hits, overlap_calls, overlap_hits;
    unsigned long sh_calls, sh_allocations, plane_conversions;
} order_profile;
#define ORDER_COUNT(field) (++order_profile.field)
#else
#define ORDER_COUNT(field) ((void)0)
#endif

static void ordering_cache_invalidate(Model3D *model)
{
    int i;
    order_cache_owner = model;
    order_asm_packet_plane=NULL;
    ++order_cache_generation;
    if (order_cache_generation == 0) {
        order_geometry_generation=0;order_geometry_failed=0;
        for(i=0;i<ORDER_CONTOUR_SLOTS;++i){
            order_contours[i].generation=0;order_contours[i].failed_generation=0;
        }
        memset(order_fixed_planes,0,sizeof(order_fixed_planes));
        memset(order_windings,0,sizeof(order_windings));
        memset(order_ray_pairs, 0, sizeof(order_ray_pairs));
        memset(order_overlap_pairs, 0, sizeof(order_overlap_pairs));
        memset(order_plane_failed, 0, sizeof(order_plane_failed));
        for (i = 0; i < ORDER_PLANE_BLOCKS; ++i)
            if (order_planes[i]) memset(order_planes[i]->stamp, 0, sizeof(order_planes[i]->stamp));
        order_cache_generation = 1;
    }
}

static void ordering_cache_release(Model3D *model)
{
    int i;
    if (order_cache_owner != model) return;
    order_asm_packet_plane=NULL;
    free(order_geometry_pairs);order_geometry_pairs=NULL;
    order_geometry_capacity=order_geometry_stride=order_geometry_generation=0;
    order_geometry_epoch=order_geometry_failed=0;order_geometry_faces=0;
    for(i=0;i<ORDER_CONTOUR_SLOTS;++i)free(order_contours[i].data);
    memset(order_contours,0,sizeof(order_contours));
    for (i = 0; i < ORDER_PLANE_BLOCKS; ++i) {
        free(order_planes[i]);order_planes[i] = NULL;
    }
    free(order_sh_workspace);order_sh_workspace = NULL;order_sh_capacity = 0;
    memset(order_ray_pairs, 0, sizeof(order_ray_pairs));
    memset(order_overlap_pairs, 0, sizeof(order_overlap_pairs));
    memset(order_plane_failed, 0, sizeof(order_plane_failed));
    memset(order_fixed_planes,0,sizeof(order_fixed_planes));
    memset(order_windings,0,sizeof(order_windings));
    order_cache_owner = NULL;order_cache_generation = 0;
}

static OrderPairEntry *ordering_pair_slot(Model3D *model, OrderPairEntry *table,
                                         int f1, int f2)
{
    unsigned int slot;
    if (order_cache_owner != model) ordering_cache_invalidate(model);
    slot = (((unsigned int)f1 * 33U) ^ (unsigned int)f2) & (ORDER_PAIR_SLOTS - 1);
    return &table[slot];
}

static int ordering_pair_hit(OrderPairEntry *entry, int f1, int f2,
                              Fixed32 scale, unsigned int context)
{
    return entry->generation == order_cache_generation &&
           entry->f1 == f1 && entry->f2 == f2 && entry->scale == scale &&
           entry->context == context;
}

static void ordering_pair_store(OrderPairEntry *entry, int f1, int f2,
                                Fixed32 scale, unsigned int context, int result)
{
    entry->f1=f1;entry->f2=f2;entry->scale=scale;entry->context=context;
    entry->result=result;entry->generation=order_cache_generation;
}

static double *ordering_workspace(long bytes)
{
    if (bytes > order_sh_capacity) {
        double *grown;
        if (bytes <= 0 || (unsigned long)bytes > (unsigned long)((size_t)-1)) return NULL;
        grown=(double*)malloc((size_t)bytes);
        if (!grown) return NULL;
        free(order_sh_workspace);order_sh_workspace=grown;order_sh_capacity=bytes;
        ORDER_COUNT(sh_allocations);
    }
    return order_sh_workspace;
}

static double *ordering_plane(Model3D *model, int face, double fallback[4])
{
    OrderPlaneBlock *block=NULL;
    double *out=fallback;
    int b,slot;
    if (order_cache_owner != model) ordering_cache_invalidate(model);
    if (face >= 0 && face < MAX_FACES) {
        b=face / ORDER_PLANE_BLOCK_SIZE;slot=face % ORDER_PLANE_BLOCK_SIZE;
        block=order_planes[b];
        if (!block && order_plane_failed[b] != order_cache_generation) {
            block=(OrderPlaneBlock*)calloc(1,sizeof(OrderPlaneBlock));order_planes[b]=block;
            if (!block) order_plane_failed[b]=order_cache_generation;
        }
        if (block) {
            out=block->plane[slot];
            if (block->stamp[slot] == order_cache_generation) return out;
            block->stamp[slot]=order_cache_generation;
        }
    }
    out[0]=FIXED64_TO_FLOAT(model->faces.plane_a[face]);
    out[1]=FIXED64_TO_FLOAT(model->faces.plane_b[face]);
    out[2]=FIXED64_TO_FLOAT(model->faces.plane_c[face]);
    out[3]=FIXED64_TO_FLOAT(model->faces.plane_d[face]);
    ORDER_COUNT(plane_conversions);
    return out;
}

/* Respect the legacy explicit geometry-cache reset without flushing unrelated
 * overlap/ray caches. Epoch wrap is rare and clears all prepared entries.
 */
static void ordering_fixed_plane_reset(void)
{
    order_asm_packet_plane=NULL;
    ++order_fixed_plane_epoch;
    if(order_fixed_plane_epoch==0){
        memset(order_fixed_planes,0,sizeof(order_fixed_planes));
        order_geometry_generation=0;
    }
}

static OrderFixedPlane *ordering_fixed_plane(Model3D *model,int face,Fixed64 epsilon)
{
    OrderFixedPlane *p;
    FaceArrays3D *faces=&model->faces;
    if(order_cache_owner!=model)ordering_cache_invalidate(model);
    p=&order_fixed_planes[(unsigned int)face & (ORDER_FIXED_PLANE_SLOTS-1)];
    if(p->generation==order_cache_generation && p->epoch==order_fixed_plane_epoch &&
       p->face==face && p->epsilon==epsilon)return p;
    memset(p->vertex_sides,0,sizeof(p->vertex_sides));
    p->a=faces->plane_a[face];p->b=faces->plane_b[face];
    p->c=faces->plane_c[face];p->d=faces->plane_d[face];
    p->a32=(Fixed32)p->a;p->b32=(Fixed32)p->b;p->c32=(Fixed32)p->c;
    p->observer=(p->d>epsilon)?1:((p->d < -epsilon)?-1:0);
    p->asm_ready=(p->a==(Fixed64)p->a32 && p->b==(Fixed64)p->b32 &&
                  p->c==(Fixed64)p->c32 && p->d>=-4611686018427387904LL &&
                  p->d<=4611686018427387904LL && epsilon>=0);
    p->face=face;p->epsilon=epsilon;p->generation=order_cache_generation;p->epoch=order_fixed_plane_epoch;
    return p;
}

static OrderWinding *ordering_winding_slot(Model3D *model,int face)
{
    if(order_cache_owner!=model)ordering_cache_invalidate(model);
    return &order_windings[(unsigned int)face & (ORDER_WINDING_SLOTS-1)];
}

static int ordering_winding_hit(OrderWinding *entry,int face,Fixed32 scale)
{
    return entry->generation==order_cache_generation && entry->face==face && entry->scale==scale;
}

static void ordering_winding_store(OrderWinding *entry,int face,Fixed32 scale,double area2)
{
    entry->face=face;entry->scale=scale;entry->area2=area2;
    entry->generation=order_cache_generation;
}

/* Return the packed byte for one directed relation, or NULL on the bounded
 * fallback path. Generation, explicit reset, epsilon and model size are tags.
 */
static unsigned char *ordering_geometry_slot(Model3D *model,int f1,int f2,Fixed64 epsilon)
{
    int nf=model->faces.face_count;
    unsigned int stride,bytes;
    if(nf<=0 || nf>ORDER_GEOMETRY_DENSE_LIMIT ||
       (unsigned int)f1>=(unsigned int)nf || (unsigned int)f2>=(unsigned int)nf)return NULL;
    if(order_cache_owner!=model)ordering_cache_invalidate(model);
    stride=((unsigned int)nf+3U)>>2;
    bytes=(unsigned int)nf*stride;
    if(order_geometry_capacity<bytes){
        unsigned char *grown;
        if(order_geometry_failed==order_cache_generation)return NULL;
        grown=(unsigned char*)malloc(bytes);
        if(!grown){order_geometry_failed=order_cache_generation;return NULL;}
        free(order_geometry_pairs);order_geometry_pairs=grown;order_geometry_capacity=bytes;
        order_geometry_generation=0;
    }
    if(order_geometry_generation!=order_cache_generation ||
       order_geometry_epoch!=order_fixed_plane_epoch || order_geometry_faces!=nf ||
       order_geometry_epsilon!=epsilon){
        unsigned char *row=order_geometry_pairs;
        int i;
        memset(order_geometry_pairs,0,bytes);
        /* Build row pointers once, avoiding a general 16-bit multiplication
         * in every packed-cache lookup on the 65816.
         */
        for(i=0;i<nf;++i){order_geometry_rows[i]=row;row+=stride;}
        order_geometry_stride=stride;order_geometry_faces=nf;
        order_geometry_epoch=order_fixed_plane_epoch;order_geometry_epsilon=epsilon;
        order_geometry_generation=order_cache_generation;
    }
    return order_geometry_rows[f1]+((unsigned int)f2>>2);
}
