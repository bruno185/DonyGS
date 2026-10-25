/* GEO V3 candidate scanner, ORCA/C / 65816.
 * Skip consecutive Q faces rejected by depth/X/Y tests in one assembly call.
 * No sorting or geometric decision is changed; return nf when exhausted.
 * Non-reentrant staging is private to the synchronous painter invocation.
 */
static unsigned int geo_scan_words[22];
static int geo_scan_result;
static int geo_scan_cached_p = -1;

/* D-relative words:
 * 1/5/9/13/17/21: far pointers sorted/z_max/minx/maxx/miny/maxy
 * 25/27: P's z_min, low word / sign-biased high word
 * 29/31/33/35: P's minx/maxx/miny/maxy
 * 37/39: current/end sorted positions; 41: scratch; 43: Q*2.
 * Preserve P,A,X,Y,D; globals explicitly use long addressing.
 */
asm void GeoV3_ScanCandidates(void)
{
    php
    rep #0x30
    pha
    phy
    phd
    cld
    tsc
    sec
    sbc #44
    tcs
    tcd
    /* Copy the invariant-size packet without loop/index overhead. Scratch
     * words at 41/43 are written before use and need no input copy.
     */
    lda >geo_scan_words
    sta 1
    lda >geo_scan_words+2
    sta 3
    lda >geo_scan_words+4
    sta 5
    lda >geo_scan_words+6
    sta 7
    lda >geo_scan_words+8
    sta 9
    lda >geo_scan_words+10
    sta 11
    lda >geo_scan_words+12
    sta 13
    lda >geo_scan_words+14
    sta 15
    lda >geo_scan_words+16
    sta 17
    lda >geo_scan_words+18
    sta 19
    lda >geo_scan_words+20
    sta 21
    lda >geo_scan_words+22
    sta 23
    lda >geo_scan_words+24
    sta 25
    lda >geo_scan_words+26
    sta 27
    lda >geo_scan_words+28
    sta 29
    lda >geo_scan_words+30
    sta 31
    lda >geo_scan_words+32
    sta 33
    lda >geo_scan_words+34
    sta 35
    lda >geo_scan_words+36
    sta 37
    lda >geo_scan_words+38
    sta 39
geo_scan_loop:
    lda 37
    cmp 39
    bcc geo_scan_have_q
    brl geo_scan_done
geo_scan_have_q:
    asl a
    tay
    lda [1],y           // Q = sorted[position], a zero-based face ID.
    asl a
    sta 43              // Q*2 addresses 16-bit bounds arrays.
    asl a
    tay                 // Q*4 addresses the 32-bit depth array.
    iny
    iny
    lda [5],y
    eor #0x8000         // Bias sign bits to use an unsigned word comparison.
    cmp 27
    bcc geo_scan_depth_reject
    bne geo_scan_depth_pass
    dey
    dey
    lda [5],y
    cmp 25              // Equal high words: low words compare unsigned.
    bcc geo_scan_depth_reject
    beq geo_scan_depth_reject
    bra geo_scan_depth_pass
geo_scan_depth_reject:
    brl geo_scan_reject
geo_scan_depth_pass:
    ldy 43

    /* Preserve the original interval formula, including 16-bit differences:
     * maxP > maxQ ? minP-maxQ : minQ-maxP; reject if nonnegative.
     * Only the choice of branch uses a signed comparison of the maxima.
     */
    lda [13],y
    eor #0x8000
    sta 41
    lda 31
    eor #0x8000
    cmp 41
    bcc geo_scan_x_qmax
    beq geo_scan_x_qmax
    lda 29
    sec
    sbc [13],y
    bmi geo_scan_x_pass
    brl geo_scan_reject
geo_scan_x_qmax:
    lda [9],y
    sec
    sbc 31
    bmi geo_scan_x_pass
    brl geo_scan_reject
geo_scan_x_pass:

    /* Preserve the original interval formula, including 16-bit differences:
     * maxP > maxQ ? minP-maxQ : minQ-maxP; reject if nonnegative.
     * Only the choice of branch uses a signed comparison of the maxima.
     */
    lda [21],y
    eor #0x8000
    sta 41
    lda 35
    eor #0x8000
    cmp 41
    bcc geo_scan_y_qmax
    beq geo_scan_y_qmax
    lda 33
    sec
    sbc [21],y
    bmi geo_scan_y_pass
    brl geo_scan_reject
geo_scan_y_qmax:
    lda [17],y
    sec
    sbc 35
    bmi geo_scan_y_pass
    brl geo_scan_reject
geo_scan_y_pass:

    /* All quick rejects passed: let C evaluate the geometric relations. */
    bra geo_scan_done
geo_scan_reject:
    inc 37
    brl geo_scan_loop
geo_scan_done:
    lda 37
    sta >geo_scan_result
    tsc
    clc
    adc #44
    tcs
    pld
    ply
    pla
    plp
    rtl
}

/* Set up stable array pointers once per painter invocation. */
static void GeoV3_ScanSetup(FaceArrays3D *faces, int *sorted, int nf)
{
    memcpy(&geo_scan_words[0], &sorted, 4);
    memcpy(&geo_scan_words[2], &faces->z_max, 4);
    memcpy(&geo_scan_words[4], &faces->minx, 4);
    memcpy(&geo_scan_words[6], &faces->maxx, 4);
    memcpy(&geo_scan_words[8], &faces->miny, 4);
    memcpy(&geo_scan_words[10], &faces->maxy, 4);
    geo_scan_words[19] = (unsigned int)nf;
    geo_scan_cached_p = -1;
}

static int GeoV3_FindCandidate(FaceArrays3D *faces, int p, int start)
{
    /* P is unchanged while walking candidates; reload its bounds only after
     * advancing the selection or swapping in a new P. Geometry is read-only.
     */
    if (geo_scan_cached_p != p) {
        memcpy(&geo_scan_words[12], &faces->z_min[p], 4);
        geo_scan_words[13] ^= 0x8000U;
        geo_scan_words[14] = (unsigned int)faces->minx[p];
        geo_scan_words[15] = (unsigned int)faces->maxx[p];
        geo_scan_words[16] = (unsigned int)faces->miny[p];
        geo_scan_words[17] = (unsigned int)faces->maxy[p];
        geo_scan_cached_p = p;
    }
    geo_scan_words[18] = (unsigned int)start;
    GeoV3_ScanCandidates();
    return geo_scan_result;
}
