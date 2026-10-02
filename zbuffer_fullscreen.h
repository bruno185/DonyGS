/* zbuffer_fullscreen_v2.h
 *
 * Full-screen 16-bit Z-buffer for Apple IIGS (320x200, SHR mode) — V2.
 *
 * Border strategy (V2, revised):
 *   Borders are produced *during* the fill scan conversion, not by a
 *   second geometric edge-stroke pass (that approach caused thick /
 *   missing / dotted lines depending on diagonal direction).
 *
 *   For each horizontal span of a face:
 *     - leftmost  pixel  → frameColor
 *     - rightmost pixel  → frameColor
 *     - if the span lies on the face's miny or maxy scanline,
 *       every pixel of the span → frameColor  (top / bottom edges)
 *     - all other pixels → fillColor
 *
 *   Depth test is the same for fill and border pixels, so hidden
 *   borders stay hidden and visible borders are exactly 1 pixel thick
 *   and aligned with the rasterised interior.
 *
 * Precision warning, encoding and scale policy unchanged from V1.
 */


#ifndef ZBUFFER_FULLSCREEN_H
#define ZBUFFER_FULLSCREEN_H


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
