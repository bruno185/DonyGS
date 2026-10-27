
#include <stdio.h>
#include <stdint.h>
#define ZBUF_WIDTH 320
#define ZBUF_HEIGHT 200
typedef uint16_t UWORD16;
typedef uint16_t *FarWordPtr;
typedef struct{int*x2d,*y2d;}VertexArrays3D;
typedef struct{int*vertex_indices_ptr,*vertex_count,*vertex_indices_buffer;}FaceArrays3D;
typedef struct{long x,z,sx,sz;int ya,yb,xb;}ZBufferScanEdge;


static void ZBuffer_PrepareScanEdges(VertexArrays3D *vtx, FaceArrays3D *faces,
                                     int face, const UWORD16 *zsrc, int y_lo,
                                     long rounding, ZBufferScanEdge *ed)
{
    int offt = faces->vertex_indices_ptr[face];
    int n = faces->vertex_count[face], k;
    /* Appliquer le même noyau aux deux buffers possédés par leurs moteurs. */
    for (k = 0; k < n; k++, ed++) {
        int k2 = (k + 1 < n) ? (k + 1) : 0;
        int vid1 = faces->vertex_indices_buffer[offt + k] - 1;
        int vid2 = faces->vertex_indices_buffer[offt + k2] - 1;
        int y1 = vtx->y2d[vid1];
        int y2 = vtx->y2d[vid2];
        int dy = y2 - y1;
        int ya, yb, xs;
        long zs, ex, ez, sx, sz, skip;

        if (dy == 0) {
            ed->ya = 32767; ed->yb = 32767;
            ed->x = 0L; ed->z = 0L; ed->sx = 0L; ed->sz = 0L;
            continue;
        }

        sx = (((long)vtx->x2d[vid2] - (long)vtx->x2d[vid1]) * 4096L) / (long)dy;
        sz = (((long)zsrc[vid2] - (long)zsrc[vid1]) * 4096L) / (long)dy;

        if (dy > 0) { ya = y1; yb = y2; xs = 0;                                        zs = (long)zsrc[vid1]; }
        else        { ya = y2; yb = y1; xs = vtx->x2d[vid2] - vtx->x2d[vid1];          zs = (long)zsrc[vid2]; }

        ex = (long)xs * 4096L;
        ez = zs * 4096L;

        if (ya < y_lo && yb > y_lo) {
            skip = (long)(y_lo - ya);
            ex += sx * skip;
            ez += sz * skip;
        }

        ed->ya = ya;  ed->yb = yb;
        ed->xb = vtx->x2d[vid1];
        ed->x  = ex + rounding;
        ed->z  = ez + rounding;
        ed->sx = sx;  ed->sz = sz;
    }
}static void PrepareAbsolute(VertexArrays3D *vtx, FaceArrays3D *faces,
                                     int face, const UWORD16 *zsrc, int y_lo,
                                     long rounding, ZBufferScanEdge *ed)
{
    int offt = faces->vertex_indices_ptr[face];
    int n = faces->vertex_count[face], k;
    /* Appliquer le même noyau aux deux buffers possédés par leurs moteurs. */
    for (k = 0; k < n; k++, ed++) {
        int k2 = (k + 1 < n) ? (k + 1) : 0;
        int vid1 = faces->vertex_indices_buffer[offt + k] - 1;
        int vid2 = faces->vertex_indices_buffer[offt + k2] - 1;
        int y1 = vtx->y2d[vid1];
        int y2 = vtx->y2d[vid2];
        int dy = y2 - y1;
        int ya, yb, xs;
        long zs, ex, ez, sx, sz, skip;

        if (dy == 0) {
            ed->ya = 32767; ed->yb = 32767;
            ed->x = 0L; ed->z = 0L; ed->sx = 0L; ed->sz = 0L;
            continue;
        }

        sx = (((long)vtx->x2d[vid2] - (long)vtx->x2d[vid1]) * 4096L) / (long)dy;
        sz = (((long)zsrc[vid2] - (long)zsrc[vid1]) * 4096L) / (long)dy;

        if (dy > 0) { ya = y1; yb = y2; xs = vtx->x2d[vid1];                                        zs = (long)zsrc[vid1]; }
        else        { ya = y2; yb = y1; xs = vtx->x2d[vid2];          zs = (long)zsrc[vid2]; }

        ex = (long)xs * 4096L;
        ez = zs * 4096L;

        if (ya < y_lo && yb > y_lo) {
            skip = (long)(y_lo - ya);
            ex += sx * skip;
            ez += sz * skip;
        }

        ed->ya = ya;  ed->yb = yb;
        ed->xb = 0;
        ed->x  = ex + rounding;
        ed->z  = ez + rounding;
        ed->sx = sx;  ed->sz = sz;
    }
}
static uint32_t seed=4096;static int rnd(void){seed=seed*1664525u+1013904223u;return (int)(seed>>16);}
static int oldhit(ZBufferScanEdge*e,int y){long t=e->x+(long)(y-e->ya)*e->sx;return e->xb+(t>=0?t>>12:-((-t)>>12));}
static int newhit(ZBufferScanEdge*e,int y){long t=e->x+(long)(y-e->ya)*e->sx;return t>=0?t>>12:-(((-t)+4095)>>12);}
int main(void){int t,y,oldbad=0,newbad=0,pixels=0;for(t=0;t<5000;t++){int xs[]={rnd()%501-100,rnd()%501-100,30,-20},ys[]={rnd()%401-100,rnd()%401-100,3,0};int off[]={0,3},count[]={3,3},ids[]={1,2,3,2,1,4};UWORD16 zs[]={10000,35000,20000,30000};VertexArrays3D v={xs,ys};FaceArrays3D f={off,count,ids};ZBufferScanEdge a[3],b[3],c[3],d[3];ZBuffer_PrepareScanEdges(&v,&f,0,zs,-1000,2048,a);ZBuffer_PrepareScanEdges(&v,&f,1,zs,-1000,2048,b);PrepareAbsolute(&v,&f,0,zs,-1000,2048,c);PrepareAbsolute(&v,&f,1,zs,-1000,2048,d);
for(y=0;y<200;y++)if(y>=a[0].ya&&y<a[0].yb){if(oldhit(a,y)<0||oldhit(a,y)>=320||oldhit(b,y)<0||oldhit(b,y)>=320||newhit(c,y)<0||newhit(c,y)>=320)continue;pixels++;if(oldhit(a,y)!=oldhit(b,y))oldbad++;if(newhit(c,y)!=newhit(d,y))newbad++;if(a[0].z!=c[0].z||a[0].sz!=c[0].sz)return 2;}}
printf("5000 reversed edges, %d visible scanline samples: old disagreements=%d; absolute-coordinate experiment=%d. Depth preparation unchanged.\n",pixels,oldbad,newbad);return newbad?1:0;}
