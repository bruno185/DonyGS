/* Ordering-cache lifecycle and key-isolation regression checks.
 * Compile from the project directory: iix compile tools/ordering_cache_check.cc
 */
#pragma memorymodel 1
#pragma optimize 1
#include "engine.c"
int main(void){
 Model3D *m=createModel3D();OrderPairEntry *entry;double fallback[4],*plane,*work;
 int i;
 if(!m)exit(1);
 m->faces.plane_a[0]=65536;m->faces.plane_b[0]=-65536;m->faces.plane_c[0]=32768;m->faces.plane_d[0]=131072;
 ordering_cache_invalidate(m);
 plane=ordering_plane(m,0,fallback);
 if(plane[0]!=1.0||plane[1]!=-1.0||plane[2]!=0.5||plane[3]!=2.0){puts("FAIL plane conversion");exit(1);}
 entry=ordering_pair_slot(m,order_ray_pairs,1,2);
 ordering_pair_store(entry,1,2,65536,0,1);
 if(!ordering_pair_hit(entry,1,2,65536,0)||ordering_pair_hit(entry,2,1,65536,0)||ordering_pair_hit(entry,1,2,131072,0)||ordering_pair_hit(entry,1,2,65536,1)){puts("FAIL exact pair keys");exit(1);}
 if(ordering_pair_slot(m,order_ray_pairs,0,35)!=entry||ordering_pair_hit(entry,0,35,65536,0)){puts("FAIL collision isolation");exit(1);}
 m->faces.plane_a[0]=196608;ordering_cache_invalidate(m);
 if(ordering_pair_hit(entry,1,2,65536,0)||ordering_plane(m,0,fallback)[0]!=3.0){puts("FAIL invalidation");exit(1);}
 work=ordering_workspace(1024);
 if(!work||ordering_workspace(512)!=work){puts("FAIL workspace reuse");exit(1);}
 work[0]=123.5;
 if(sizeof(size_t)==2 && (ordering_workspace(65536L)!=NULL||order_sh_workspace!=work||work[0]!=123.5)){puts("FAIL rejected workspace growth");exit(1);}
 order_cache_generation=1;
 ordering_pair_store(entry,1,2,65536,0,1);
 plane=ordering_plane(m,0,fallback);
 order_cache_generation=65535U;m->faces.plane_a[0]=262144;
 ordering_cache_invalidate(m);
 if(ordering_pair_hit(entry,1,2,65536,0)||ordering_plane(m,0,fallback)[0]!=4.0){puts("FAIL generation wrap");exit(1);}
 /* Hide/restore must invalidate pair results even without a new transform. */
 m->faces.face_count=1;m->faces.vertex_count[0]=3;m->faces.saved_vertex_count[0]=0;
 m->faces.vertex_indices_ptr[0]=0;m->faces.vertex_indices_buffer[0]=1;
 m->faces.vertex_indices_buffer[1]=2;m->faces.vertex_indices_buffer[2]=3;
 m->faces.z_min[0]=0;
 reverseFaceVertexOrder(m,0);
 if(ordering_plane(m,0,fallback)[0]!=-4.0){puts("FAIL winding invalidation");exit(1);}
 entry=ordering_pair_slot(m,order_ray_pairs,1,2);ordering_pair_store(entry,1,2,65536,0,1);
 hideFace(m,0);
 if(ordering_pair_hit(entry,1,2,65536,0)){puts("FAIL hide invalidation");exit(1);}
 restoreFace(m,0);
 destroyModel3D(m);
 if(order_cache_owner||order_sh_workspace){puts("FAIL cache release");exit(1);}
 for(i=0;i<ORDER_PLANE_BLOCKS;++i)if(order_planes[i]){puts("FAIL plane block release");exit(1);}
 puts("PASS ordering cache: keys, collisions, scale/method isolation, plane conversion, invalidation, workspace, wrap, release");
 exit(0);return 0;
}
