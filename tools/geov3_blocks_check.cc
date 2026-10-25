/* GEO V3 block regressions: exact fixed products, stable sorting, prepared
 * planes, winding invalidation and unordered pair keys. No timing checks.
 * Run from the project directory with ORCA/C through Golden Gate:
 *   iix compile tools/geov3_blocks_check.cc
 *   iix -DKeepType=S16 link geov3_blocks_check keep=geov3_blocks_check
 *   iix geov3_blocks_check
 */
#pragma memorymodel 1
#pragma optimize 1
#include "engine.c"
/* Stable insertion sort by descending z_max. */
static void reference_sort_by_zmax(int* sorted, Fixed32* z_max, int nf)
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


static unsigned long test_state=0x12345678UL;
static unsigned long test_random(void){test_state=test_state*1664525UL+1013904223UL;return test_state;}
int main(void){
 Model3D *m=createModel3D();int *ids,*copy,*rank;Fixed32 *z;
 static const int lengths[]={0,1,2,3,16,17,31,32,33,63,64,65,127,128,129,257,511,1023,4096,6000};
 static DonyRayPairs pairs;
 int t,n,i,j,tmp,kind,expected_obs,expected_ready;unsigned long bits;static Fixed64 epsilon;
 OrderFixedPlane *prepared;OrderWinding *w;
 if(!m)exit(1);
 ids=m->faces.sorted_face_indices;z=m->faces.z_max;
 copy=(int*)malloc(6000U*sizeof(int));rank=(int*)malloc(6000U*sizeof(int));if(!copy||!rank)exit(1);
 for(t=0;t<100;++t){n=lengths[t%20];kind=t/20;
 for(i=0;i<n;++i){ids[i]=i;if(kind==0)z[i]=0;else if(kind==1)z[i]=i;else if(kind==2)z[i]=-i;else if(kind==3)z[i]=(i&1)?2147483647L:(-2147483647L-1L);else{bits=test_random();memcpy(&z[i],&bits,4);}}
 for(i=n-1;i>0;--i){j=(int)(test_random()%(unsigned)(i+1));tmp=ids[i];ids[i]=ids[j];ids[j]=tmp;}
 memcpy(copy,ids,n*sizeof(int));for(i=0;i<n;++i)rank[ids[i]]=i;
 sort_by_zmax(ids,z,n);
 for(i=1;i<n;++i)if(z[ids[i-1]]<z[ids[i]]||(z[ids[i-1]]==z[ids[i]]&&rank[ids[i-1]]>rank[ids[i]])){puts("FAIL binary insertion order/stability");exit(1);}
 memset(rank,0,n*sizeof(int));for(i=0;i<n;++i)if(ids[i]<0||ids[i]>=n||rank[ids[i]]++){puts("FAIL permutation");exit(1);}
 if(n<=257){reference_sort_by_zmax(copy,z,n);if(memcmp(ids,copy,n*sizeof(int))){puts("FAIL insertion reference");exit(1);}}
 }
 puts("PASS 100 stable sort cases through 6000 faces");
 {
  static const Fixed32 boundary[]={0L,1L,-1L,65535L,65536L,-65536L,2147483647L,(-2147483647L-1L)};
  static Fixed32 a,b,wanted,result;
  static Fixed64 product,wanted_product,wide;
  for(t=0;t<2112;++t){
   if(t<64){a=boundary[t/8];b=boundary[t%8];}
   else{bits=test_random();memcpy(&a,&bits,4);bits=test_random();memcpy(&b,&bits,4);}
   wanted_product=(Fixed64)a*(Fixed64)b;
   wanted=(Fixed32)(wanted_product>>16);
   product=fixed_product32_fast(a,b);result=fixed_mul32_fast(a,b);
   if(product!=wanted_product||result!=wanted){puts("FAIL assembly fixed arithmetic");exit(1);}
  }
  for(t=0;t<128;++t){wide=(Fixed64)(t-64)*1099511627776LL;a=(Fixed32)(t-32);
   if(fixed_product64x32_fast(wide,a)!=wide*(Fixed64)a){puts("FAIL wide product fallback");exit(1);}}
  puts("PASS 2112 exact fixed products and 128 wide fallback products");
 }

 for(t=0;t<2048;++t){
 static Fixed64 a,b,c,d;static Fixed32 a32,b32,c32;
 a=(Fixed64)(long)test_random();b=(Fixed64)(long)test_random();c=(Fixed64)(long)test_random();
 if(t%7==0)a=1099511627776LL;
 if(t%3==0)d=4611686018427387904LL;else if(t%3==1)d=-4611686018427387904LL;else d=(Fixed64)(long)test_random();
 if(t%11==0)d=4611686018427387905LL;
 epsilon=(Fixed64)(t%17);
 m->faces.plane_a[0]=a;m->faces.plane_b[0]=b;m->faces.plane_c[0]=c;m->faces.plane_d[0]=d;
 ordering_cache_invalidate(m);prepared=ordering_fixed_plane(m,0,epsilon);
 a32=(Fixed32)a;b32=(Fixed32)b;c32=(Fixed32)c;
 expected_obs=d>epsilon?1:(d < -epsilon?-1:0);
 expected_ready=a==(Fixed64)a32&&b==(Fixed64)b32&&c==(Fixed64)c32&&d>=-4611686018427387904LL&&d<=4611686018427387904LL&&epsilon>=0;
 if(prepared->a!=a||prepared->b!=b||prepared->c!=c||prepared->d!=d||prepared->observer!=expected_obs||prepared->asm_ready!=expected_ready){printf("FAIL plane t=%d obs %d/%d ready %d/%d a %.0f/%.0f b %.0f/%.0f c %.0f/%.0f d %.0f/%.0f\n",t,prepared->observer,expected_obs,prepared->asm_ready,expected_ready,(double)prepared->a,(double)a,(double)prepared->b,(double)b,(double)prepared->c,(double)c,(double)prepared->d,(double)d);exit(1);}
 }
 m->faces.plane_d[0]=10;ordering_fixed_plane_reset();prepared=ordering_fixed_plane(m,0,9);
 if(prepared->observer!=1||ordering_fixed_plane(m,0,10)->observer!=0){puts("FAIL epsilon key");exit(1);}
 m->faces.plane_d[0]=-11;(void)geometric_face_relation(NULL,0,0);
 if(ordering_fixed_plane(m,0,9)->observer!=-1){puts("FAIL explicit geometry reset");exit(1);}
 order_fixed_plane_epoch=65535U;ordering_fixed_plane_reset();
 if(ordering_fixed_plane(m,0,9)->observer!=-1){puts("FAIL plane epoch wrap");exit(1);}
 w=ordering_winding_slot(m,3);ordering_winding_store(w,3,65536,1.25);
 if(!ordering_winding_hit(w,3,65536)||ordering_winding_hit(w,3,65537)||ordering_winding_hit(w,131,65536)){puts("FAIL winding tags");exit(1);}
 ordering_cache_invalidate(m);if(ordering_winding_hit(w,3,65536)){puts("FAIL winding invalidation");exit(1);}
 puts("PASS 2048 fixed-plane cases, epsilon/reset/wrap and winding keys");
 if(!dony_ray_pairs_reserve(&pairs,6000))exit(1);
 for(i=0;i<6000;++i)dony_ray_pair_record(&pairs,i,(i+1)%6000,1);
 for(i=0;i<6000;++i)if(!dony_ray_pair_seen(&pairs,(i+1)%6000,i,1)){puts("FAIL unordered pair keys");exit(1);}
 if(dony_ray_pair_seen(&pairs,0,3000,1)||dony_ray_pair_seen(&pairs,0,1,2)){puts("FAIL pair isolation");exit(1);}
 puts("PASS 6000 pair keys, collisions and generations");
 free(pairs.first);free(pairs.second);free(pairs.next);free(copy);free(rank);destroyModel3D(m);exit(0);return 0;
}
