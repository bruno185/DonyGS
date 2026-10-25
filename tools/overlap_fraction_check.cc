/* Regression checks for the fractional SH ordering path. */
#pragma memorymodel 1
#pragma optimize 1
#include "engine.c"
int main(void) {
    Model3D *m=createModel3D();
    static const double rects[7][5]={
      {20,10,30,20,0}, {20,20,30,30,0}, {21,10,30,20,0},
      {12,12,18,18,36}, {15,15,25,25,25}, {19.75,10,30,20,2.5}, {10,10,20,20,100}
    };
    int t,w,i,f,ok,ok_reverse;
    double cx,cy,area,cx2,cy2,area2;
    unsigned long checks=0;
    if(!m)exit(1);
    m->faces.face_count=2;m->vertices.vertex_count=8;
    s_global_proj_scale_fixed=INT_TO_FIXED(1);
    for(t=0;t<7;++t)for(w=0;w<4;++w){
      for(f=0;f<2;++f){
        double x0=f?rects[t][0]:10, y0=f?rects[t][1]:10;
        double x1=f?rects[t][2]:20, y1=f?rects[t][3]:20;
        m->faces.vertex_count[f]=4;m->faces.vertex_indices_ptr[f]=f*4;
        for(i=0;i<4;++i){int k=f*4+i;double x=(i==0||i==3)?x0:x1;double y=i<2?y0:y1;
          m->vertices.xo[k]=FLOAT_TO_FIXED(x-160);m->vertices.yo[k]=FLOAT_TO_FIXED(100-y);m->vertices.zo[k]=INT_TO_FIXED(1);
          m->vertices.x2d[k]=(int)x;m->vertices.y2d[k]=(int)y;
          m->faces.vertex_indices_buffer[f*4+i]=f*4+((w&(1<<f))?3-i:i)+1;
        }
      }
      fractional_projection_cache_invalidate(m);
      ok=compute_intersection_centroid_fractional_tryboth(m,0,1,&cx,&cy,&area);
      ok_reverse=compute_intersection_centroid_fractional_tryboth(m,1,0,&cx2,&cy2,&area2);
      if(rects[t][4]==0){if(ok||ok_reverse){printf("FAIL contact %d winding %d\n",t,w);exit(1);}}
      else if(!ok||!ok_reverse||fabs(area*.5-rects[t][4])>1e-8||fabs(area-area2)>1e-8||fabs(cx-cx2)>1e-8||fabs(cy-cy2)>1e-8){printf("FAIL intersection %d winding %d\n",t,w);exit(1);}
      ++checks;
    }
    printf("PASS %lu contact/containment/thin-overlap cases, both orders and windings\n",checks);
    /* Cached ratios survive scale-only zooms, but never observer updates. */
    {
      double x,y,expected_x,expected_y;
      int ids[3]={0,256,MAX_VERTICES-1};
      int j;
      for(j=0;j<3;++j){int v=ids[j];
        m->vertices.xo[v]=FLOAT_TO_FIXED(2.25);
        m->vertices.yo[v]=FLOAT_TO_FIXED(-3.5);
        m->vertices.zo[v]=INT_TO_FIXED(7);
      }
      fractional_projection_cache_invalidate(m);
      for(j=0;j<3;++j){int v=ids[j];
        fractional_projection_cached(m,v,100.0,&x,&y);
        fractional_projection_cached(m,v,1000.0,&x,&y);
        expected_x=160.0+1000.0*2.25/7.0;
        expected_y=100.0+1000.0*3.5/7.0;
        if(fabs(x-expected_x)>1e-9||fabs(y-expected_y)>1e-9){puts("FAIL cached scale");exit(1);}
      }
      m->vertices.xo[0]=INT_TO_FIXED(7);
      fractional_cache_generation=65535U;
      fractional_projection_cache_invalidate(m); /* Force 16-bit wrap. */
      fractional_projection_cached(m,0,100.0,&x,&y);
      if(fabs(x-260.0)>1e-9){puts("FAIL cache wrap/invalidation");exit(1);}
      fractional_projection_cache_release(m);
      fractional_projection_cache_invalidate(m);
      fractional_cache_failed[0]=fractional_cache_generation; /* Uncached fallback. */
      fractional_projection_cached(m,0,100.0,&x,&y);
      if(fabs(x-260.0)>1e-9||fractional_cache[0]){puts("FAIL uncached fallback");exit(1);}
      puts("PASS cache zoom, observer invalidation, block boundaries, wrap and fallback");
    }
    destroyModel3D(m);
    if(fractional_cache_owner!=NULL){puts("FAIL cache release");exit(1);}
    exit(0);return 0;
}
