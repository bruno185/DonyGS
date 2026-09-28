/* zbuffer_fullscreen.h
 *
 * Full-screen 16-bit Z-buffer for Apple IIGS (320x200, SHR mode).
 * Replaces the scanline Z-buffer (a single line, reinitialized on
 * every line, in native float) with a PERSISTENT depth buffer across
 * the whole frame, in 16-bit integers, spread over 2 banks of 64 KB.
 *
 * PRECISION WARNING (explicit decision, assumed risk):
 * the existing scanline renderer deliberately keeps depth in
 * float32 (see the header of renderModelScanlineZBuffer_old) because
 * of a real case already diagnosed: two faces (13 and 18) whose
 * interobserver depths interleave with less than 1% difference in
 * their on-screen overlap region -- a Fixed32 (16.16) had proved
 * insufficient for that case. A 16-bit integer offers LESS useful
 * precision than a Fixed32 for the same value range, so this
 * full-screen Z-buffer can reproduce the same occlusion defect on
 * similar close faces. Kept nonetheless by choice.
 * ZBUFFER_INV_Z_SCALE below must be calibrated empirically on the
 * real 1/z range of your models to best exploit the available
 * 16 bits (see ZBuffer_QuantizeInvZ).
 *
 * Stored encoding: 16-bit "inverted distance", NOT 1/z directly:
 *   stored_value = 0xFFFF - clamp(round(inv_z * ZBUFFER_INV_Z_SCALE), 0, 0xFFFF)
 * This choice keeps the same convention "smaller = closer, 0xFFFF
 * = empty/never drawn" as the first draft of this module, which
 * allows a single-branch test (BCS) in the assembly loop rather
 * than two. inv_z = 0 (point at infinity) naturally maps to
 * 0xFFFF, so "empty" and "infinitely far" are the same value --
 * no special case to handle for the initial state.
 */


#ifndef ZBUFFER_FULLSCREEN_H
#define ZBUFFER_FULLSCREEN_H


#define ZBUF_WIDTH       320
#define ZBUF_HEIGHT      200
#define ZBUF_ROW_WORDS   ZBUF_WIDTH               /* 320 16-bit words / line */
#define ZBUF_ROW_BYTES   (ZBUF_WIDTH * 2)          /* 640 bytes / line       */
#define ZBUF_FAR_VALUE   0xFFFFU                   /* "empty" = infinitely far */


/* TO BE CALIBRATED on the real inv_z (1/zo) range of your models.
 * Too small -> everything packs near 0xFFFF (loss of resolution
 * near the camera, where depth conflicts are most troublesome).
 * Too large -> saturation (clamp) for nearby points, making them
 * all "equal" in depth. Simple method: temporarily log min/max of
 * inv_z on a few typical scenes and choose ZBUFFER_INV_Z_SCALE so
 * that the observed max inv_z is close to 65535 / ZBUFFER_INV_Z_SCALE.
 */
#define ZBUFFER_INV_Z_SCALE  7979000.0f
/* Fallback value only, used until ZBuffer_SetScaleForFrame() has
 * been called at least once (e.g. very first frame). In normal
 * operation the real scale is recomputed EVERY FRAME by
 * ZBuffer_SetScaleForFrame() from the current scene's max inv_z --
 * see below. A fixed scale was tested and proved incorrect as soon
 * as the zoom level changes (max inv_z changes with zoom; a scale
 * calibrated on one scene saturates at 0xFFFF on a more zoomed-in
 * one, making several faces equal in depth -> random order). */


#define ZBUFFER_TARGET_MAX_CODE  60000.0f  /* margin under 65535 */


/* Recomputes the quantization scale for the current frame, from
 * the largest inv_z actually observed on this frame (not a fixed
 * constant). Call ONCE PER FRAME, after computing inv_z[] for all
 * visible vertices and before the first call to ZBuffer_QuantizeInvZ
 * of this frame. Does nothing (keeps the previous scale) if
 * max_inv_z is zero or negative (empty/degenerate scene), to avoid
 * a division by zero.
 */
void ZBuffer_SetScaleForFrame(float max_inv_z);


typedef unsigned short  UWORD16;
typedef unsigned long   ULONG32;
typedef short           SWORD16;   /* for signed dz_step */


/* 24-bit pointer (bank:offset). On ORCA/C for the IIGS, a "normal"
 * pointer is already a 24-bit pointer (no near/far distinction to
 * declare) -- no special keyword needed, unlike what was assumed
 * in a previous version of this file (the "far" keyword does not
 * exist in this compiler: compilation error confirmed).
 */
typedef UWORD16 * FarWordPtr;


/* Table of 200 far pointers, one per screen line, precomputed once
 * by ZBuffer_Init(). Simple linear addressing
 * (base + y*ZBUF_ROW_BYTES): NO special splitting is required to
 * prevent a line from straddling the boundary between the two
 * banks, because the 65816 long indirect addressing ([ptr],y)
 * performs a true 24-bit addition with carry into the bank byte --
 * see the architecture note at the top of ZBuffer_TestSetRow.
 */
extern FarWordPtr zbuf_row[ZBUF_HEIGHT];


/* Allocates 2 banks (128 KB usable, aligned on a bank boundary)
 * via the GS/OS memory manager, builds zbuf_row[], then calls
 * ZBuffer_Clear(). Returns 0 if allocation fails, 1 otherwise.
 */
int  ZBuffer_Init(void);


/* Frees the block allocated by ZBuffer_Init. */
void ZBuffer_Shutdown(void);


/* Resets the whole frame to ZBUF_FAR_VALUE. C version, readable, for
 * debug -- prefer ZBuffer_ClearFast (asm) for real rendering.
 */
void ZBuffer_Clear(void);


/* Far pointer to the start of line y (0..199). */
#define ZBuffer_RowPtr(y)  (zbuf_row[y])


/* Converts a native 1/z (float) into the 16-bit representation
 * stored in the Z-buffer (see the encoding explained at the top of
 * this file). Use once per span endpoint (2 calls), not per pixel --
 * per-pixel interpolation is then done in integer via dz_step,
 * cf. ZBuffer_TestSetRow.
 */
UWORD16 ZBuffer_QuantizeInvZ(float inv_z);


/* High-level test-and-set, one pixel at a time (debug / non-critical
 * paths). Returns 1 if z is closer (0xFFFF - inv_z code) than what
 * was already stored, 0 otherwise.
 */
int ZBuffer_TestAndSet(int x, int y, UWORD16 z);


/* ---- inline assembly routines (see zbuffer_fullscreen.c) ---- */


/* Fills BOTH banks of the Z-buffer with ZBUF_FAR_VALUE. Call once
 * per frame. bank_lo/bank_hi = bank numbers of the Z-buffer
 * (bank_hi = bank_lo + 1), obtained at init from the high byte of
 * the allocated base pointer.
 */
asm void ZBuffer_ClearFast(int bank_lo, int bank_hi);


/* Tests and updates "count" consecutive pixels of a Z-buffer line,
 * with linearly interpolated depth (z_start, + signed dz_step per
 * pixel). zbuf_ptr = ZBuffer_RowPtr(y) offset by x_start words
 * (see integration in the renderer). Uses long indirect addressing
 * [ptr],y: correct even if the span straddles the boundary between
 * the two banks (see architecture note in zbuffer_fullscreen.c).
 *
 * Does ONLY the depth test/write -- color write (nibble pack/unpack
 * in the SHR framebuffer, cf. drawPixel) remains to be merged
 * separately; see the end-of-file note in the .c on the cost of the
 * accumulator width change (SEP/REP) that implies in a tight loop.
 */
asm void ZBuffer_TestSetRow(FarWordPtr zbuf_ptr, UWORD16 z_start,
                             SWORD16 dz_step, int count);


#endif /* ZBUFFER_FULLSCREEN_H */