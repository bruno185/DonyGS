
#include <stdio.h>
#include <stdint.h>
#define ZBUF_WIDTH 320
#define ZBUF_HEIGHT 200
typedef uint16_t UWORD16;
typedef uint16_t *FarWordPtr;
typedef struct{int*x2d,*y2d;}VertexArrays3D;
typedef struct{int*vertex_indices_ptr,*vertex_count,*vertex_indices_buffer;}FaceArrays3D;
typedef struct{long x,z,sx,sz;int ya,yb,xb;}ZBufferScanEdge;
static FarWordPtr zbuf_row[200];static UWORD16 row[320];static int drawn;
static void Offscreen_DrawPixel(int x,int y,int color){(void)x;(void)y;(void)color;drawn++;}
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
}static void Off_PlotBridge(int sx, int sy, UWORD16 zq, int color)
{
    FarWordPtr row;
    UWORD16 cur;
    if ((unsigned)sx >= (unsigned)ZBUF_WIDTH || (unsigned)sy >= (unsigned)ZBUF_HEIGHT) return;
    /* Comparer à la surface existante sans écrire de profondeur. */
    row = zbuf_row[sy];
    cur = row[sx];
    /* Dessiner seulement pour égalité ou tolérance d'un code telle qu'implémentée. */
    if (zq == cur)
        Offscreen_DrawPixel(sx, sy, color);
    else if (zq > cur && (int)zq - (int)cur <= 1)
        Offscreen_DrawPixel(sx, sy, color);
}
int main(void){int xs[]={10,0,30,-20},ys[]={0,3,3,0};int off[]={0,3},count[]={3,3},ids[]={1,2,3,2,1,4};UWORD16 zs[]={30000,30000,30000,30000};VertexArrays3D v={xs,ys};FaceArrays3D f={off,count,ids};ZBufferScanEdge a[3],b[3];int y;UWORD16 tests[]={65535,30001,30000,29999,29998};
ZBuffer_PrepareScanEdges(&v,&f,0,zs,0,2048,a);ZBuffer_PrepareScanEdges(&v,&f,1,zs,0,2048,b);
puts("reversed_shared_edge");for(y=0;y<3;y++){long ta=a[0].x+y*a[0].sx,tb=b[0].x+y*b[0].sx;int xa=a[0].xb+(ta>=0?ta>>12:-((-ta)>>12));int xb=b[0].xb+(tb>=0?tb>>12:-((-tb)>>12));printf("y=%d forward=%d reverse=%d\n",y,xa,xb);}
zbuf_row[0]=row;puts("bridge_visibility");for(y=0;y<5;y++){row[0]=tests[y];drawn=0;Off_PlotBridge(0,0,30000,7);printf("stored=%u edge=30000 drawn=%d\n",tests[y],drawn);}
return 0;}
