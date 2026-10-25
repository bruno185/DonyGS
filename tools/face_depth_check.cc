/* C/65816 equivalence checks for calculateFaceDepths().
 * Compile/link/run from the project directory through Golden Gate:
 *   iix compile tools/face_depth_check.cc
 *   iix -DKeepType=S16 link face_depth_check keep=face_depth_check
 *   iix face_depth_check
 * The reference retains the pre-assembly vertex loop and rounding order.
 */
#pragma memorymodel 1
#pragma optimize 1
#include "engine.c"
static void reference_calculateFaceDepths(Model3D* model, Face3D* faces, int face_count) {
    ordering_cache_invalidate(model);
    int i, j;
    int culled_count = 0; // diagnostic: number of faces culled by back-face test (observer-space)
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* face_arrays = &model->faces;
    for (i = 0; i < face_count; i++) {
        Fixed32 z_min = FLOAT_TO_FIXED(9999.0);  // Initialize to very large value (closest)
        Fixed32 z_max = FLOAT_TO_FIXED(-9999.0); // Initialize to very small value (farthest)
        int display_flag = 1;
        Fixed32 sum = 0;
        int n = face_arrays->vertex_count[i];
        int minx = 9999, maxx = -9999, miny = 9999, maxy = -9999;
        // Access indices from the packed buffer using the offset
        int offset = face_arrays->vertex_indices_ptr[i];
        // --- Newell accumulators (plane normal via ALL vertices, robust to concave faces) ---
        Fixed64 a64 = 0, b64 = 0, c64 = 0, d64 = 0;
        Fixed64 sumx64 = 0, sumy64 = 0, sumz64 = 0;
        Fixed32 x_prev = 0, y_prev = 0, z_prev = 0;
        Fixed32 x_first = 0, y_first = 0, z_first = 0;
        Fixed32 xc = 0, yc = 0, zc = 0;
        int valid_n = 0;
        int have_prev = 0;
        for (j = 0; j < n; j++) {
            int vertex_idx = face_arrays->vertex_indices_buffer[offset + j] - 1;
            if (vertex_idx >= 0) {
                Fixed32 zo = vtx->zo[vertex_idx];
                Fixed32 xo = vtx->xo[vertex_idx];
                Fixed32 yo = vtx->yo[vertex_idx];
                if (zo < 0) display_flag = 0;
                if (zo < z_min) z_min = zo;
                if (zo > z_max) z_max = zo;
                sum += zo;
                int x2d = vtx->x2d[vertex_idx];
                int y2d = vtx->y2d[vertex_idx];
                if (x2d < minx) minx = x2d;
                if (x2d > maxx) maxx = x2d;
                if (y2d < miny) miny = y2d;
                if (y2d > maxy) maxy = y2d;
                // centroid accumulation (Fixed64 to avoid overflow with many vertices)
                sumx64 += (Fixed64)xo;
                sumy64 += (Fixed64)yo;
                sumz64 += (Fixed64)zo;
                if (!have_prev) {
                    x_first = xo; y_first = yo; z_first = zo;
                    x_prev  = xo; y_prev  = yo; z_prev  = zo;
                    have_prev = 1;
                } else {
                    Fixed32 dy = FIXED_SUB(y_prev, yo);
                    Fixed32 dz = FIXED_SUB(z_prev, zo);
                    Fixed32 dx = FIXED_SUB(x_prev, xo);
                    Fixed32 sy = y_prev + yo;
                    Fixed32 sz = z_prev + zo;
                    Fixed32 sx = x_prev + xo;
                    a64 += fixed_product32_fast(dy,sz);
                    b64 += fixed_product32_fast(dz,sx);
                    c64 += fixed_product32_fast(dx,sy);
                    x_prev = xo; y_prev = yo; z_prev = zo;
                }
                valid_n++;
            }
        }
        if (!display_flag || n < 3 || valid_n < 3) {
            // face is behind camera or degenerate: zero coefficients
            face_arrays->plane_a[i] = 0;
            face_arrays->plane_b[i] = 0;
            face_arrays->plane_c[i] = 0;
            face_arrays->plane_d[i] = 0;
            if (shaded_by_orientation) {
                face_shade_color[i] = COL_FILL_DEFAULT;
            }
        } else {
            // close the Newell cycle: last valid vertex -> first valid vertex
            Fixed32 dy = FIXED_SUB(y_prev, y_first);
            Fixed32 dz = FIXED_SUB(z_prev, z_first);
            Fixed32 dx = FIXED_SUB(x_prev, x_first);
            Fixed32 sy = y_prev + y_first;
            Fixed32 sz = z_prev + z_first;
            Fixed32 sx = x_prev + x_first;
            a64 += fixed_product32_fast(dy,sz);
            b64 += fixed_product32_fast(dz,sx);
            c64 += fixed_product32_fast(dx,sy);
            a64 >>= FIXED_SHIFT;
            b64 >>= FIXED_SHIFT;
            c64 >>= FIXED_SHIFT;
            xc = (Fixed32)(sumx64 / valid_n);
            yc = (Fixed32)(sumy64 / valid_n);
            zc = (Fixed32)(sumz64 / valid_n);
            d64 = -(fixed_product64x32_fast(a64,xc) + fixed_product64x32_fast(b64,yc)
                   + fixed_product64x32_fast(c64,zc));
            d64 >>= FIXED_SHIFT;
            face_arrays->plane_a[i] = a64;
            face_arrays->plane_b[i] = b64;
            face_arrays->plane_c[i] = c64;
            face_arrays->plane_d[i] = d64;
            // Optional back-face culling in observer-space: if the plane D term is <= 0,
            // the plane faces away from the observer (origin), so cull the face when enabled.
            if (cull_back_faces && display_flag) {
                if (d64 <= 0) {
                    display_flag = 0;
                    ++culled_count;
                }
            }
        }
        face_arrays->z_min[i] = z_min;  // Store minimum depth for this face (closest)
        face_arrays->z_max[i] = z_max;  // Store maximum depth for this face (farthest)
        face_arrays->display_flag[i] = display_flag;
        if (n > 0) {
            if (n == 4)      face_arrays->z_mean[i] = sum >> 2;
            else if (n == 3) face_arrays->z_mean[i] = sum / 3;
            else             face_arrays->z_mean[i] = sum / n;
            face_arrays->minx[i] = minx;
            face_arrays->maxx[i] = maxx;
            face_arrays->miny[i] = miny;
            face_arrays->maxy[i] = maxy;
        } else {
            face_arrays->z_mean[i] = 0;
            face_arrays->z_min[i] = 0;
            face_arrays->z_max[i] = 0;
            face_arrays->minx[i] = 0;
            face_arrays->maxx[i] = 0;
            face_arrays->miny[i] = 0;
            face_arrays->maxy[i] = 0;
        }
    }
}

typedef struct {
 Fixed64 a,b,c,d;
 Fixed32 zmin,zmax,zmean;
 int minx,maxx,miny,maxy,display,shade;
} SavedDepth;
static SavedDepth *saved=NULL;
static int saved_capacity=0;
static unsigned long rng=0x13579BDFUL;
static unsigned long next_bits(void){rng=rng*1664525UL+1013904223UL;return rng;}
static Fixed32 random_fixed(void){unsigned long bits=next_bits();Fixed32 value;memcpy(&value,&bits,4);return value;}
static void capture(Model3D *m,int n)
{
 int i;
 for(i=0;i<n;++i){
  saved[i].a=m->faces.plane_a[i];saved[i].b=m->faces.plane_b[i];saved[i].c=m->faces.plane_c[i];saved[i].d=m->faces.plane_d[i];
  saved[i].zmin=m->faces.z_min[i];saved[i].zmax=m->faces.z_max[i];saved[i].zmean=m->faces.z_mean[i];
  saved[i].minx=m->faces.minx[i];saved[i].maxx=m->faces.maxx[i];saved[i].miny=m->faces.miny[i];saved[i].maxy=m->faces.maxy[i];
  saved[i].display=m->faces.display_flag[i];saved[i].shade=face_shade_color[i];
 }
}
static void compare(Model3D *m,int n,int fixture)
{
 int i;
 for(i=0;i<n;++i){
  if(saved[i].a!=m->faces.plane_a[i]||saved[i].b!=m->faces.plane_b[i]||saved[i].c!=m->faces.plane_c[i]||saved[i].d!=m->faces.plane_d[i]||
     saved[i].zmin!=m->faces.z_min[i]||saved[i].zmax!=m->faces.z_max[i]||saved[i].zmean!=m->faces.z_mean[i]||
     saved[i].minx!=m->faces.minx[i]||saved[i].maxx!=m->faces.maxx[i]||saved[i].miny!=m->faces.miny[i]||saved[i].maxy!=m->faces.maxy[i]||
     saved[i].display!=m->faces.display_flag[i]||saved[i].shade!=face_shade_color[i]){
    printf("FAIL fixture %d face %d\n",fixture,i);
    printf("plane expected %.0f %.0f %.0f %.0f\n",(double)saved[i].a,(double)saved[i].b,(double)saved[i].c,(double)saved[i].d);
    printf("plane actual   %.0f %.0f %.0f %.0f\n",(double)m->faces.plane_a[i],(double)m->faces.plane_b[i],(double)m->faces.plane_c[i],(double)m->faces.plane_d[i]);
    printf("depth expected %.0f %.0f %.0f actual %.0f %.0f %.0f\n",(double)saved[i].zmin,(double)saved[i].zmax,(double)saved[i].zmean,(double)m->faces.z_min[i],(double)m->faces.z_max[i],(double)m->faces.z_mean[i]);
    printf("bounds %d %d %d %d vs %d %d %d %d flags %d/%d colors %d/%d\n",saved[i].minx,saved[i].maxx,saved[i].miny,saved[i].maxy,m->faces.minx[i],m->faces.maxx[i],m->faces.miny[i],m->faces.maxy[i],saved[i].display,m->faces.display_flag[i],saved[i].shade,face_shade_color[i]);exit(1);
  }
 }
}
static void run_pair(Model3D *m,int n,int fixture)
{
 int i;
 if(n>saved_capacity){
  free(saved);saved=(SavedDepth*)malloc((size_t)n*sizeof(SavedDepth));
  if(!saved)exit(1);saved_capacity=n;
 }
 for(i=0;i<n;++i)face_shade_color[i]=7;
 reference_calculateFaceDepths(m,NULL,n);capture(m,n);
 for(i=0;i<n;++i)face_shade_color[i]=7;
 calculateFaceDepths(m,NULL,n);compare(m,n,fixture);
}
int main(void)
{
 static const int counts[]={-1,0,1,2,3,4,5,7,8,15,16,17,63,257,6000};
 Model3D *m=createModel3D();ObserverParams params;
 int t,n,j,nf,c,a;
 static const int angles[7][3]={{30,20,0},{207,20,0},{216,15,0},{0,0,0},{90,20,0},{270,180,90},{359,359,359}};
 static const char *models[3]={"models/a","models/avion2.obj","models/gear2.obj"};
 if(!m)exit(1);
#if defined(FACE_DEPTH_USE_ASM)
 if(sizeof(FaceDepthScanResult)!=72)exit(1);
 /* Exercise the truncation-preserving narrow and four-vertex average paths. */
 for(t=0;t<4096;++t){
  static Fixed64 total;
  n=t%6000+1;total=(Fixed64)random_fixed()*n;
  if(t%3==0)n=4;
  if(t%7==0)total=(Fixed64)(t%9-4);
  if(FaceDepth_CentroidCoordinate(total,n)!=(Fixed32)(total/n)){
   puts("FAIL exact centroid average");exit(1);
  }
 }
 puts("PASS 4096 exact centroid divisions, negative truncation and wide fallback");
#endif
 m->vertices.vertex_count=MAX_VERTICES;m->faces.face_count=1;m->faces.vertex_indices_ptr[0]=0;
 for(t=0;t<1024;++t){
  n=counts[t%15];m->faces.vertex_count[0]=n;
  cull_back_faces=t&1;shaded_by_orientation=(t>>1)&1;
  for(j=0;j<n;++j){
   Fixed32 x=random_fixed(),y=random_fixed(),z=random_fixed();
   if(t%4==0){x>>=8;y>>=8;z>>=8;}
   if(t%4==1)z=(Fixed32)((unsigned long)z&0x7FFFFFFFUL);
   if(t%11==0)z=0;
   if(t%13==0)z=INT_TO_FIXED(10000);
   if(t%17==0)z=-INT_TO_FIXED(10000);
   m->vertices.xo[j]=x;m->vertices.yo[j]=y;m->vertices.zo[j]=z;
   m->vertices.x2d[j]=(int)next_bits();m->vertices.y2d[j]=(int)next_bits();
   m->faces.vertex_indices_buffer[j]=(t%5==0 && j%3==0)?0:j+1;
   if(t%19==0)m->faces.vertex_indices_buffer[j]=-2;
  }
  run_pair(m,1,t);
 }
 puts("PASS 1024 C/assembly depth scans, 64-bit sums, sentinels, invalid indices and 6000 vertices");
 destroyModel3D(m);
 for(j=0;j<3;++j){
  m=createModel3D();if(!m||loadModel3D(m,models[j])<0)exit(1);
  s_global_proj_scale_fixed=m->auto_suggested_proj_scale;normalizeAutoFitDistanceTo150(m);
  nf=m->faces.face_count;
  for(a=0;a<7;++a)for(c=0;c<2;++c){
   params.angle_h=angles[a][0];params.angle_v=angles[a][1];params.angle_w=angles[a][2];params.distance=INT_TO_FIXED(150);
   framePolyOnly=1;cull_back_faces=c;shaded_by_orientation=1;
   processModelFast(m,&params,models[j]);run_pair(m,nf,1024+j*14+a*2+c);
  }
  printf("PASS 14 C/assembly depth configurations: %s (%d faces)\n",models[j],nf);destroyModel3D(m);
 }
 free(saved);puts("PASS all face depth outputs, culling and invalid-face shading");exit(0);return 0;
}
