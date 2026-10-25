/* Exact regressions for the consolidated GEO V3 optimization.
 * Compile/link/run from the project directory with ORCA/C and Golden Gate.
 * References deliberately retain the previous general 64-bit arithmetic.
 */
#pragma memorymodel 1
#pragma optimize 1
#include "engine.c"
static long long reference_orient_ll(long long ax,long long ay,long long bx,long long by,long long cx,long long cy) {
    return (bx-ax)*(cy-ay) - (by-ay)*(cx-ax);
}
static int reference_on_seg_ll(long long ax,long long ay,long long bx,long long by,long long cx,long long cy) {
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
static int reference_reference_segs_intersect_int_fixed64(
    int x1, int y1, int x2, int y2,
    int x3, int y3, int x4, int y4)
{
    long long o1 = reference_orient_ll(x1, y1, x2, y2, x3, y3);
    long long o2 = reference_orient_ll(x1, y1, x2, y2, x4, y4);
    long long o3 = reference_orient_ll(x3, y3, x4, y4, x1, y1);
    long long o4 = reference_orient_ll(x3, y3, x4, y4, x2, y2);
    if (!(((o1 > 0 && o2 < 0) ||
           (o1 < 0 && o2 > 0)) &&
          ((o3 > 0 && o4 < 0) ||
           (o3 < 0 && o4 > 0))))
    {
        return 0;
    }
    return 1;
}

static int reference_segs_intersect_int(int x1,int y1,int x2,int y2,int x3,int y3,int x4,int y4) {
    return reference_reference_segs_intersect_int_fixed64(x1,y1,x2,y2,x3,y3,x4,y4);
}
static int reference_point_in_poly_int(int px,int py,FaceArrays3D* faces, VertexArrays3D* vtx, int f, int n) {
    int cnt = 0; int off = faces->vertex_indices_ptr[f];
    for (int i = 0; i < n; ++i) {
        int j = (i+1)%n;
        int vi = faces->vertex_indices_buffer[off + i] - 1;
        int vj = faces->vertex_indices_buffer[off + j] - 1;
        int xi = vtx->x2d[vi], yi = vtx->y2d[vi];
        int xj = vtx->x2d[vj], yj = vtx->y2d[vj];
        /* If the point lies exactly on the edge, consider it OUTSIDE (no overlap).
         * Use orient==0 + on-segment test to detect boundary points. */
        if (reference_orient_ll(xi, yi, xj, yj, px, py) == 0 && reference_on_seg_ll(xi, yi, xj, yj, px, py)) return 0;
        if (((yi > py) != (yj > py)) && (px < (long long)(xj - xi) * (py - yi) / (yj - yi) + xi)) cnt++;
    }
    return (cnt & 1);
}

static unsigned long test_bits=0x2468ACE1UL;
static unsigned long next_bits(void){test_bits=test_bits*1664525UL+1013904223UL;return test_bits;}
static Fixed32 random_fixed(void){unsigned long bits=next_bits();Fixed32 value;memcpy(&value,&bits,4);return value;}
static int reference_plane(Model3D *m,int face,Fixed32 a,Fixed32 b,Fixed32 c,Fixed64 d,Fixed64 eps,int obs)
{
 int i,side,same=1,opposite=1;
 static Fixed64 acc;
 for(i=0;i<m->faces.vertex_count[face];++i){
  int v=m->faces.vertex_indices_buffer[m->faces.vertex_indices_ptr[face]+i]-1;
  acc=(((Fixed64)a*m->vertices.xo[v])>>16)+(((Fixed64)b*m->vertices.yo[v])>>16)+(((Fixed64)c*m->vertices.zo[v])>>16)+d;
  if(acc>eps)side=1;else if(acc < -eps)side=-1;else continue;
  if(side==obs)opposite=0;else same=0;
  if(!same&&!opposite)return 0;
 }
 return same?-1:(opposite?1:0);
}
static void check_geometry(Model3D *m)
{
 static Fixed32 a,b,c;static Fixed64 d,eps;
 unsigned int *sides;int t,j,obs,wanted,actual;
 OrderFixedPlane *p;
 m->vertices.vertex_count=MAX_VERTICES;m->faces.face_count=2;
 m->faces.vertex_indices_ptr[1]=0;
 for(t=0;t<2048;++t){
  a=random_fixed();b=random_fixed();c=random_fixed();
  if(t%3==0)a=0;if(t%5==0)b=0;if(t%7==0)c=0;
  eps=(Fixed64)(next_bits()&65535UL);
  if(t%4==0)d=4611686018427387904LL;
  else if(t%4==1)d=-4611686018427387904LL;
  else d=(Fixed64)random_fixed();
  if(t%17==0){a=b=c=0;d=eps;}
  if(t%19==0){a=b=c=0;d=-eps;}
  m->faces.plane_a[0]=a;m->faces.plane_b[0]=b;m->faces.plane_c[0]=c;m->faces.plane_d[0]=d;
  m->faces.vertex_count[1]=t%17;
  for(j=0;j<16;++j){int v=(j&1)?j+64:j;
   if(j==15)v=MAX_VERTICES-1;
   m->vertices.xo[v]=random_fixed();m->vertices.yo[v]=random_fixed();m->vertices.zo[v]=random_fixed();
   m->faces.vertex_indices_buffer[j]=v+1;
  }
  ordering_cache_invalidate(m);p=ordering_fixed_plane(m,0,eps);sides=p->vertex_sides;
  for(obs=-1;obs<=1;obs+=2){
   wanted=reference_plane(m,1,a,b,c,d,eps,obs);
   actual=GeoRel_Classify32(m,1,a,b,c,d,eps,obs,sides);
   if(actual!=wanted||GeoRel_Classify32(m,1,a,b,c,d,eps,obs,sides)!=wanted){printf("FAIL vertex-side cache fixture %d\n",t);exit(1);}
  }
  if(p->observer){
   wanted=reference_plane(m,1,a,b,c,d,eps,p->observer);
   if(GeoRel_ClassifyPrepared(m,1,p)!=wanted || GeoRel_ClassifyPrepared(m,1,p)!=wanted){
    puts("FAIL prepared assembly packet");exit(1);
   }
  }
  p=ordering_fixed_plane(m,0,eps+1);
  for(j=0;j<ORDER_VERTEX_SIDE_SLOTS;++j)if(p->vertex_sides[j]){puts("FAIL epsilon side reset");exit(1);}
 }
 puts("PASS 2048 exact plane fixtures, cached vertices, on-plane cases and collisions");
 /* Packed relations retain all three results, directed keys and row boundaries. */
 for(t=1;t<=256;t=t<5?t+1:(t==256?257:(t>128?256:t*2))){
  unsigned char *byte;unsigned shift;
  m->faces.face_count=t;
  for(j=0;j<t;++j){
   byte=ordering_geometry_slot(m,t-1,j,0);shift=(j&3)<<1;
   if(!byte||((*byte>>shift)&3)){puts("FAIL packed relation reset");exit(1);}
   *byte|=(unsigned char)((j%3+1)<<shift);
  }
  for(j=t-1;j>=0;--j){byte=ordering_geometry_slot(m,t-1,j,0);shift=(j&3)<<1;
   if(((*byte>>shift)&3)!=(unsigned)(j%3+1)){puts("FAIL packed relation key");exit(1);}
  }
  ordering_fixed_plane_reset();byte=ordering_geometry_slot(m,t-1,0,0);
  if(!byte||*byte){puts("FAIL packed epoch reset");exit(1);}
 }
 m->faces.face_count=257;if(ordering_geometry_slot(m,0,0,0)){puts("FAIL bounded pair table");exit(1);}
 m->faces.face_count=2;
 ordering_fixed_plane_reset();
 {unsigned char *byte=ordering_geometry_slot(m,0,0,0);
  *byte=255;
  if(*ordering_geometry_slot(m,0,0,1)){puts("FAIL dense epsilon reset");exit(1);}
  order_cache_generation=65535U;ordering_cache_invalidate(m);
  if(*ordering_geometry_slot(m,0,0,1)){puts("FAIL dense generation wrap");exit(1);}
  free(order_geometry_pairs);order_geometry_pairs=NULL;order_geometry_capacity=0;
  order_geometry_failed=order_cache_generation;
  if(ordering_geometry_slot(m,0,0,1)){puts("FAIL dense allocation fallback");exit(1);}
 }

 m->faces.face_count=2;current_observer_distance=INT_TO_FIXED(150);
 m->faces.vertex_count[1]=1;m->faces.vertex_indices_buffer[0]=1;
 m->vertices.xo[0]=INT_TO_FIXED(1);m->vertices.yo[0]=m->vertices.zo[0]=0;
 m->faces.plane_a[0]=INT_TO_FIXED(1);m->faces.plane_b[0]=m->faces.plane_c[0]=0;m->faces.plane_d[0]=INT_TO_FIXED(1);
 ordering_cache_invalidate(m);
 if(geometric_face_relation(m,0,1)!=-1){puts("FAIL geometry integration");exit(1);}
 m->vertices.xo[0]=-INT_TO_FIXED(2);ordering_cache_invalidate(m);
 if(geometric_face_relation(m,0,1)!=1){puts("FAIL transformed geometry reset");exit(1);}
 puts("PASS packed directed relations, size limit, explicit resets and transform invalidation");
}
static void check_products(void)
{
 static Fixed32 a,b,swapped;static Fixed64 expected;int t;
 for(t=0;t<8192;++t){
  a=random_fixed();b=(Fixed32)(next_bits()&32767UL)*65536L;
  if(t&1)b=-b;if(t&2){swapped=a;a=b;b=swapped;}
  if(t%5==0)b=(t&1)?-1L:1L;
  expected=(Fixed64)a*(Fixed64)b;
  if(fixed_product32_fast(a,b)!=expected||fixed_mul32_fast(a,b)!=(Fixed32)(expected>>16)){printf("FAIL factored product %d\n",t);exit(1);}
 }
 puts("PASS 8192 factored/unity products, both signs and operand orders");
}
static void check_overlap(Model3D *m)
{
 int t,i,n,px,py;static int v[8];
 m->faces.face_count=1;m->vertices.vertex_count=16;m->faces.vertex_indices_ptr[0]=0;
 for(t=0;t<4096;++t){
  n=t%14+3;m->faces.vertex_count[0]=n;
  for(i=0;i<n;++i){
   m->faces.vertex_indices_buffer[i]=i+1;
   m->vertices.x2d[i]=(int)next_bits();m->vertices.y2d[i]=(int)next_bits();
   if(t%3==0){m->vertices.x2d[i]%=320;m->vertices.y2d[i]%=200;}
  }
  for(i=0;i<8;++i)v[i]=(int)next_bits();
  if(t%3==0)for(i=0;i<8;++i)v[i]%=320;
  if(t%7==0){v[4]=v[0];v[5]=v[1];}
  if(segs_intersect_int(v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7])!=
     reference_segs_intersect_int(v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7])){printf("FAIL strict segment %d\n",t);exit(1);}
  px=(int)next_bits();py=(int)next_bits();
  if(t%2==0){px=m->vertices.x2d[0];py=m->vertices.y2d[0];}
  if(point_in_poly_int(px,py,&m->faces,&m->vertices,0,n)!=reference_point_in_poly_int(px,py,&m->faces,&m->vertices,0,n)){
   printf("FAIL point containment %d\n",t);exit(1);
  }
 }
 puts("PASS 4096 strict segment and containment fixtures, contacts and 16-bit extremes");
}

static void check_scanner(Model3D *m)
{
 int t,n,i,p,start,expected,actual;
 for(t=0;t<512;++t){
  n=t%4==0?MAX_FACES:(t%127+1);m->faces.face_count=n;
  for(i=0;i<n;++i){
   m->faces.sorted_face_indices[i]=n-1-i;
   m->faces.z_min[i]=random_fixed();m->faces.z_max[i]=random_fixed();
   m->faces.minx[i]=(int)next_bits();m->faces.maxx[i]=(int)next_bits();
   m->faces.miny[i]=(int)next_bits();m->faces.maxy[i]=(int)next_bits();
  }
  p=(int)(next_bits()%(unsigned)n);start=(int)(next_bits()%(unsigned)(n+1));
  expected=start;
  while(expected<n){
   int q=m->faces.sorted_face_indices[expected],dx,dy;
   if(m->faces.z_max[q]<=m->faces.z_min[p]){++expected;continue;}
   dx=m->faces.maxx[p]>m->faces.maxx[q]?m->faces.minx[p]-m->faces.maxx[q]:m->faces.minx[q]-m->faces.maxx[p];
   if(dx>=0){++expected;continue;}
   dy=m->faces.maxy[p]>m->faces.maxy[q]?m->faces.miny[p]-m->faces.maxy[q]:m->faces.miny[q]-m->faces.maxy[p];
   if(dy>=0){++expected;continue;}
   break;
  }
  GeoV3_ScanSetup(&m->faces,m->faces.sorted_face_indices,n);
  actual=GeoV3_FindCandidate(&m->faces,p,start);
  if(actual!=expected || GeoV3_FindCandidate(&m->faces,p,start)!=expected){puts("FAIL assembly candidate scanner");exit(1);}
 }
 puts("PASS 512 exact assembly candidate scans, signed bounds and 6000-face buffers");
}
int main(void){Model3D *m=createModel3D();if(!m)exit(1);check_products();check_geometry(m);check_overlap(m);check_scanner(m);destroyModel3D(m);if(order_geometry_pairs){puts("FAIL dense cache release");exit(1);}exit(0);return 0;}

