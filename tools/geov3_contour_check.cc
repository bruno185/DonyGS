/* Exact arithmetic and normalized-contour cache regressions.
 * Compile/link/run from the project directory with ORCA/C and Golden Gate.
 */
#pragma memorymodel 1
#pragma optimize 1
#include "engine.c"

static unsigned long contour_test_random=0x13579BDFUL;
static unsigned long contour_random(void)
{
    contour_test_random=contour_test_random*1664525UL+1013904223UL;
    return contour_test_random;
}

static void check_short_products(void)
{
    static const Fixed32 edges[]={0L,1L,-1L,255L,256L,32767L,32768L,
        65535L,-65535L,65536L,-65536L,2147483647L,(-2147483647L-1L)};
    static Fixed32 a,b,swapped;
    static Fixed64 expected,actual;
    unsigned long bits;
    int i;
    for(i=0;i<12314;++i){
        if(i<169){a=edges[i/13];b=edges[i%13];}
        else if(i<338){a=edges[(i-169)/13];b=edges[(i-169)%13];swapped=a;a=b;b=swapped;}
        else {
            bits=contour_random();memcpy(&a,&bits,4);
            b=(Fixed32)(contour_random()&65535UL);
            if(i&1)b=-b;
            if(i&2){swapped=a;a=b;b=swapped;}
        }
        expected=(Fixed64)a*(Fixed64)b;
        actual=fixed_product32_fast(a,b);
        if(actual!=expected||fixed_mul32_fast(a,b)!=(Fixed32)(expected>>16)){
            printf("FAIL short multiply %d\n",i);exit(1);
        }
    }
    puts("PASS 12314 exact short-path products, both signs and operand orders");
}

static void check_contour(Model3D *m,int face,int n,int reverse,int fail_alloc)
{
    static double x[17],y[17],expected_x[17],expected_y[17];
    double *ex,*ey,scale,area=0.0;
    OrderContour *entry;
    int i,j,prev=n-1;
    m->faces.vertex_count[face]=n;m->faces.vertex_indices_ptr[face]=0;
    for(i=0;i<n;++i){
        double a=6.283185307179586*(double)i/(double)n;
        m->vertices.xo[i]=FLOAT_TO_FIXED(3.0*cos(a)+1.0);
        m->vertices.yo[i]=FLOAT_TO_FIXED(2.0*sin(a)-1.0);
        m->vertices.zo[i]=INT_TO_FIXED(7);
        m->faces.vertex_indices_buffer[i]=(reverse?n-i:i+1);
    }
    fractional_projection_cache_invalidate(m);
    entry=&order_contours[(unsigned int)face&(ORDER_CONTOUR_SLOTS-1)];
    if(fail_alloc){
        free(entry->data);memset(entry,0,sizeof(*entry));
        entry->failed_generation=order_cache_generation;
    }
    scale=(double)s_global_proj_scale_fixed/(double)FIXED_SCALE;
    for(i=0;i<n;++i){
        int v=m->faces.vertex_indices_buffer[i]-1;
        expected_x[i]=(double)CENTRE_X+scale*((double)m->vertices.xo[v]/(double)m->vertices.zo[v]);
        expected_y[i]=(double)CENTRE_Y-scale*((double)m->vertices.yo[v]/(double)m->vertices.zo[v]);
    }
    for(i=0;i<n;++i){
        area+=(expected_x[prev]-expected_x[0])*(expected_y[i]-expected_y[0])
             -(expected_x[i]-expected_x[0])*(expected_y[prev]-expected_y[0]);
        prev=i;
    }
    if(area<0.0)for(i=0;i<n/2;++i){
        double tmp; j=n-1-i;
        tmp=expected_x[i];expected_x[i]=expected_x[j];expected_x[j]=tmp;
        tmp=expected_y[i];expected_y[i]=expected_y[j];expected_y[j]=tmp;
    }
    /* Second call exercises a cache hit, or the same precision-preserving fallback. */
    for(j=0;j<2;++j){
        if(!fractional_face_gather(m,face,n,scale,x,y,&ex,&ey)||
           memcmp(x,expected_x,n*sizeof(double))||memcmp(y,expected_y,n*sizeof(double))){
            puts("FAIL normalized contour");exit(1);
        }
        if(fail_alloc||n>ORDER_CONTOUR_MAX_VERTICES){
            if(ex||ey){puts("FAIL contour fallback");exit(1);}
        }else{
            if(!ex||!ey){puts("FAIL contour cache");exit(1);}
            for(i=0;i<n;++i){
                int next=i+1;if(next==n)next=0;
                if(ex[i]!=expected_x[next]-expected_x[i]||ey[i]!=expected_y[next]-expected_y[i]){
                    puts("FAIL cached edge");exit(1);
                }
            }
        }
    }
}

int main(void)
{
    Model3D *m=createModel3D();
    int n,reverse;
    double *ex,*ey;
    static double x[17],y[17];
    if(!m)exit(1);
    m->faces.face_count=65;m->vertices.vertex_count=17;
    check_short_products();
    for(n=3;n<=17;++n)for(reverse=0;reverse<2;++reverse){
        s_global_proj_scale_fixed=INT_TO_FIXED(100);
        check_contour(m,0,n,reverse,0);
        s_global_proj_scale_fixed=INT_TO_FIXED(1000);
        check_contour(m,64,n,reverse,0);
    }
    check_contour(m,0,4,0,1);
    /* Scale alone invalidates the contour, while normalized vertex ratios survive. */
    check_contour(m,0,4,0,0);
    s_global_proj_scale_fixed=INT_TO_FIXED(100);
    if(!fractional_face_gather(m,0,4,100.0,x,y,&ex,&ey)||order_contours[0].scale!=s_global_proj_scale_fixed){
        puts("FAIL contour scale key");exit(1);
    }
    /* Two distinct faces in the same slot must not share a contour tag. */
    m->faces.vertex_count[64]=4;m->faces.vertex_indices_ptr[64]=0;
    if(!fractional_face_gather(m,64,4,100.0,x,y,&ex,&ey)||order_contours[0].face!=64 ||
       !fractional_face_gather(m,0,4,100.0,x,y,&ex,&ey)||order_contours[0].face!=0){
        puts("FAIL contour collision key");exit(1);
    }
    /* A different owner must invalidate retained data even at the same scale. */
    {
        Model3D *other=createModel3D();
        if(!other)exit(1);
        fractional_projection_cache_invalidate(other);
        if(!fractional_face_gather(m,0,4,100.0,x,y,&ex,&ey)||order_cache_owner!=m){
            puts("FAIL contour owner");exit(1);
        }
        destroyModel3D(other);
    }
    order_cache_generation=65535U;
    fractional_projection_cache_invalidate(m);
    if(order_contours[0].generation){puts("FAIL contour generation wrap");exit(1);}
    m->vertices.zo[0]=0;
    if(fractional_face_gather(m,0,4,100.0,x,y,&ex,&ey)){puts("FAIL behind camera");exit(1);}
    fractional_projection_cache_release(m);
    for(n=0;n<ORDER_CONTOUR_SLOTS;++n)if(order_contours[n].data){puts("FAIL contour release");exit(1);}
    destroyModel3D(m);
    puts("PASS 60 contours, winding, zoom, collisions, allocation fallback, wrap and release");
    exit(0);return 0;
}
