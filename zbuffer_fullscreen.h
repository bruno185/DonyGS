/* Full-screen Z-buffer V5: coherent fill + depth-tested Bresenham borders. */

#ifndef ZBUFFER_FULLSCREEN_H
#define ZBUFFER_FULLSCREEN_H


/* Fill and border use the same depth plane.  No slope tolerance is needed.
 * Default 0 rejects every strictly farther edge.  Keep 0 for hidden-line tests.
 */
#ifndef ZBUFFER_BORDER_TOLERANCE
#define ZBUFFER_BORDER_TOLERANCE 0
#endif

/* Optional borrowed per-side mask, indexed like vertex_indices_buffer.
 * 0 hides a side; nonzero draws it. NULL restores all sides.
 */
void ZBuffer_SetEdgeFlags(const unsigned char *flags, unsigned long count);

#define ZBUF_WIDTH       320
#define ZBUF_HEIGHT      200
#define ZBUF_ROW_WORDS   ZBUF_WIDTH
#define ZBUF_ROW_BYTES   (ZBUF_WIDTH * 2)
#define ZBUF_FAR_VALUE   0xFFFFU


#define ZBUFFER_INV_Z_SCALE  7979000.0f
#define ZBUFFER_TARGET_MAX_CODE  60000.0f


void ZBuffer_SetScaleForFrame(float max_inv_z);


typedef unsigned short  UWORD16;
typedef unsigned long   ULONG32;
typedef short           SWORD16;


typedef UWORD16 * FarWordPtr;


extern FarWordPtr zbuf_row[ZBUF_HEIGHT];


int  ZBuffer_Init(void);
void ZBuffer_Shutdown(void);
void ZBuffer_Clear(void);

#define ZBuffer_RowPtr(y)  (zbuf_row[y])

UWORD16 ZBuffer_QuantizeInvZ(float inv_z);
int ZBuffer_TestAndSet(int x, int y, UWORD16 z);

asm void ZBuffer_ClearFast(int bank_lo, int bank_hi);
asm void ZBuffer_TestSetRow(FarWordPtr zbuf_ptr, UWORD16 z_start,
                             SWORD16 dz_step, int count);


#endif /* ZBUFFER_FULLSCREEN_H */
