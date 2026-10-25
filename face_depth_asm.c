/* Per-face depth/bounds/Newell scan, ORCA/C / 65816.
 * The C caller retains centroid division, plane D, culling and shading.
 * Products are raw signed 32x32 -> 64; normal accumulators shift only once,
 * after closing the polygon, exactly as in calculateFaceDepths().
 * Global packets are non-reentrant, like the shared multiplication helper.
 */
typedef struct {
    int valid_n,display_flag;
    Fixed32 z_min,z_max,sum;
    int minx,maxx,miny,maxy;
    Fixed64 sumx,sumy,sumz,a,b,c;
} FaceDepthScanResult;
static FaceDepthScanResult face_depth_result;
static unsigned int face_depth_inputs[13];

/* D-relative byte layout, using 148 bounded stack bytes:
 * 1/5/9/13/17/21: far pointers indices/xo/yo/zo/x2d/y2d; 25: remaining.
 * 27/29/31/33: index offset, valid count, display flag, previous-valid flag.
 * 35/39/43: z_min/z_max/wrapped 32-bit z sum; 47/49/51/53: screen bounds.
 * 55/63/71: signed 64-bit coordinate sums; 79/87/95: raw Newell sums.
 * 103..113: previous xyz; 115..125: first xyz; 127..137: current xyz.
 * 139/141/143/145: coordinate offset, comparison scratch, screen offset, n.
 * JSL's return and the multiplier's workspace live BELOW this frame. The
 * multiplier preserves D/X/Y and returns its four words in geo_mul_p.
 */
asm void FaceDepth_ScanKernel(void)
{
    php
    rep #0x30
    pha
    phx
    phy
    phd
    cld
    tsc
    sec
    sbc #148
    tcs
    tcd
    /* Only the 26 input bytes are copied; output/scratch is initialized here. */
    lda >face_depth_inputs
    sta 1
    lda >face_depth_inputs+2
    sta 3
    lda >face_depth_inputs+4
    sta 5
    lda >face_depth_inputs+6
    sta 7
    lda >face_depth_inputs+8
    sta 9
    lda >face_depth_inputs+10
    sta 11
    lda >face_depth_inputs+12
    sta 13
    lda >face_depth_inputs+14
    sta 15
    lda >face_depth_inputs+16
    sta 17
    lda >face_depth_inputs+18
    sta 19
    lda >face_depth_inputs+20
    sta 21
    lda >face_depth_inputs+22
    sta 23
    lda >face_depth_inputs+24
    sta 25
    lda 25
    sta 145
    stz 27
    stz 29
    stz 33
    lda #1
    sta 31
    /* Retain the original +/-9999 sentinels, even for unusual coordinates. */
    stz 35
    stz 39
    lda #0x270F
    sta 37
    sta 47
    sta 51
    lda #0xD8F1
    sta 41
    sta 49
    sta 53
    stz 43
    stz 45
    stz 55
    stz 57
    stz 59
    stz 61
    stz 63
    stz 65
    stz 67
    stz 69
    stz 71
    stz 73
    stz 75
    stz 77
    stz 79
    stz 81
    stz 83
    stz 85
    stz 87
    stz 89
    stz 91
    stz 93
    stz 95
    stz 97
    stz 99
    stz 101
depth_scan_vertex:
    lda 25
    beq depth_scan_end_vertices
    bmi depth_scan_end_vertices
    ldy 27
    lda [1],y
    dec a               // Match the original signed 16-bit index-minus-one.
    bpl depth_scan_valid_index
    brl depth_scan_next
depth_scan_end_vertices:
    brl depth_scan_finish
depth_scan_valid_index:
    asl a
    sta 143             // Two bytes per screen coordinate.
    asl a
    sta 139             // Four bytes per observer coordinate.
    tay
    ldy 139
    lda [5],y
    sta 127
    iny
    iny
    lda [5],y
    sta 129
    ldy 139
    lda [9],y
    sta 131
    iny
    iny
    lda [9],y
    sta 133
    ldy 139
    lda [13],y
    sta 135
    iny
    iny
    lda [13],y
    sta 137
    lda 137
    bpl depth_scan_z_nonnegative
    stz 31              // Any negative observer depth hides the entire face.
depth_scan_z_nonnegative:
    /* Signed 32-bit comparisons: bias only the high word, then compare
     * equal-high low words unsigned. No subtraction overflow is possible.
     */
    lda 37
    eor #0x8000
    sta 141
    lda 137
    eor #0x8000
    cmp 141
    bcc depth_scan_zmin_less
    bne depth_scan_zmin_greater
    lda 135
    cmp 35
    bcc depth_scan_zmin_less
    bra depth_scan_zmin_greater
depth_scan_zmin_less:
    bra depth_scan_zmin_store
depth_scan_zmin_greater:
    bra depth_scan_zmin_done
depth_scan_zmin_store:
    lda 135
    sta 35
    lda 137
    sta 37
depth_scan_zmin_done:
    lda 41
    eor #0x8000
    sta 141
    lda 137
    eor #0x8000
    cmp 141
    bcc depth_scan_zmax_less
    bne depth_scan_zmax_greater
    lda 135
    cmp 39
    bcc depth_scan_zmax_less
    bra depth_scan_zmax_greater
depth_scan_zmax_less:
    bra depth_scan_zmax_done
depth_scan_zmax_greater:
depth_scan_zmax_store:
    lda 135
    sta 39
    lda 137
    sta 41
depth_scan_zmax_done:
    /* z_mean intentionally uses the old wrapping 32-bit sum; centroid
     * uses separate sign-extended 64-bit sums. These are not interchangeable.
     */
    clc
    lda 43
    adc 135
    sta 43
    lda 45
    adc 137
    sta 45
    ldy 143
    lda [17],y
    eor #0x8000
    sta 141
    lda 47
    eor #0x8000
    cmp 141
    bcc depth_scan_xmin_done
    ldy 143
    lda [17],y
    sta 47
depth_scan_xmin_done:
    lda 49
    eor #0x8000
    cmp 141
    bcs depth_scan_xmax_done
    ldy 143
    lda [17],y
    sta 49
depth_scan_xmax_done:
    ldy 143
    lda [21],y
    eor #0x8000
    sta 141
    lda 51
    eor #0x8000
    cmp 141
    bcc depth_scan_ymin_done
    ldy 143
    lda [21],y
    sta 51
depth_scan_ymin_done:
    lda 53
    eor #0x8000
    cmp 141
    bcs depth_scan_ymax_done
    ldy 143
    lda [21],y
    sta 53
depth_scan_ymax_done:
    /* Sign extension before accumulation retains all 64 centroid bits. */
    ldx #0
    lda 129
    bpl depth_scan_sumx_positive
    dex
depth_scan_sumx_positive:
    clc
    lda 55
    adc 127
    sta 55
    lda 57
    adc 129
    sta 57
    txa                 // TXA preserves carry into the sign-extension words.
    adc 59
    sta 59
    txa
    adc 61
    sta 61
    /* Sign extension before accumulation retains all 64 centroid bits. */
    ldx #0
    lda 133
    bpl depth_scan_sumy_positive
    dex
depth_scan_sumy_positive:
    clc
    lda 63
    adc 131
    sta 63
    lda 65
    adc 133
    sta 65
    txa                 // TXA preserves carry into the sign-extension words.
    adc 67
    sta 67
    txa
    adc 69
    sta 69
    /* Sign extension before accumulation retains all 64 centroid bits. */
    ldx #0
    lda 137
    bpl depth_scan_sumz_positive
    dex
depth_scan_sumz_positive:
    clc
    lda 71
    adc 135
    sta 71
    lda 73
    adc 137
    sta 73
    txa                 // TXA preserves carry into the sign-extension words.
    adc 75
    sta 75
    txa
    adc 77
    sta 77
    inc 29
    lda 33
    bne depth_scan_have_previous
    inc 33
    lda 127
    sta 103
    sta 115
    lda 129
    sta 105
    sta 117
    lda 131
    sta 107
    sta 119
    lda 133
    sta 109
    sta 121
    lda 135
    sta 111
    sta 123
    lda 137
    sta 113
    sta 125
    brl depth_scan_next
depth_scan_have_previous:
    jsl depth_scan_edge
    lda 127
    sta 103
    lda 129
    sta 105
    lda 131
    sta 107
    lda 133
    sta 109
    lda 135
    sta 111
    lda 137
    sta 113
depth_scan_next:
    inc 27
    inc 27
    dec 25
    brl depth_scan_vertex
depth_scan_finish:
    /* The C reference closes/shifts only a visible nondegenerate face.
     * Invalid indices are skipped; the cycle joins successive VALID vertices.
     */
    lda 31
    beq depth_scan_export
    lda 145
    cmp #3
    bcc depth_scan_export
    lda 29
    cmp #3
    bcc depth_scan_export
    lda 115
    sta 127
    lda 117
    sta 129
    lda 119
    sta 131
    lda 121
    sta 133
    lda 123
    sta 135
    lda 125
    sta 137
    jsl depth_scan_edge
    /* Arithmetic shift by exactly one word AFTER the full Newell sum.
     * Discard bits 0..15 and sign-extend bits 48..63 into the new top word.
     */
    lda 81
    sta 79
    lda 83
    sta 81
    lda 85
    sta 83
    lda 85
    bpl depth_scan_a_extend_positive
    lda #0xFFFF
    bra depth_scan_a_extend
depth_scan_a_extend_positive:
    lda #0
depth_scan_a_extend:
    sta 85
    lda 89
    sta 87
    lda 91
    sta 89
    lda 93
    sta 91
    lda 93
    bpl depth_scan_b_extend_positive
    lda #0xFFFF
    bra depth_scan_b_extend
depth_scan_b_extend_positive:
    lda #0
depth_scan_b_extend:
    sta 93
    lda 97
    sta 95
    lda 99
    sta 97
    lda 101
    sta 99
    lda 101
    bpl depth_scan_c_extend_positive
    lda #0xFFFF
    bra depth_scan_c_extend
depth_scan_c_extend_positive:
    lda #0
depth_scan_c_extend:
    sta 101
depth_scan_export:
    /* Export in the native C struct's 16-bit-aligned field order. */
    lda 29
    sta >face_depth_result
    lda 31
    sta >face_depth_result+2
    lda 35
    sta >face_depth_result+4
    lda 37
    sta >face_depth_result+6
    lda 39
    sta >face_depth_result+8
    lda 41
    sta >face_depth_result+10
    lda 43
    sta >face_depth_result+12
    lda 45
    sta >face_depth_result+14
    lda 47
    sta >face_depth_result+16
    lda 49
    sta >face_depth_result+18
    lda 51
    sta >face_depth_result+20
    lda 53
    sta >face_depth_result+22
    lda 55
    sta >face_depth_result+24
    lda 57
    sta >face_depth_result+26
    lda 59
    sta >face_depth_result+28
    lda 61
    sta >face_depth_result+30
    lda 63
    sta >face_depth_result+32
    lda 65
    sta >face_depth_result+34
    lda 67
    sta >face_depth_result+36
    lda 69
    sta >face_depth_result+38
    lda 71
    sta >face_depth_result+40
    lda 73
    sta >face_depth_result+42
    lda 75
    sta >face_depth_result+44
    lda 77
    sta >face_depth_result+46
    lda 79
    sta >face_depth_result+48
    lda 81
    sta >face_depth_result+50
    lda 83
    sta >face_depth_result+52
    lda 85
    sta >face_depth_result+54
    lda 87
    sta >face_depth_result+56
    lda 89
    sta >face_depth_result+58
    lda 91
    sta >face_depth_result+60
    lda 93
    sta >face_depth_result+62
    lda 95
    sta >face_depth_result+64
    lda 97
    sta >face_depth_result+66
    lda 99
    sta >face_depth_result+68
    lda 101
    sta >face_depth_result+70
    tsc
    clc
    adc #148
    tcs
    pld
    ply
    plx
    pla
    plp
    rtl

/* Local subroutine: add the edge previous -> current to all three raw
 * Newell sums. Coordinate sums/differences wrap at 32 bits as in Fixed32 C;
 * multiplication and accumulation retain the complete signed 64-bit product.
 * Each axis uses the same shared multiplier, without a C call or 64-bit copy.
 */
depth_scan_edge:
    /* a += (previous_y - current_y) * (previous_z + current_z). */
    sec
    lda 107
    sbc 131
    sta >geo_mul_inputs
    lda 109
    sbc 133
    sta >geo_mul_inputs+2
    clc
    lda 111
    adc 135
    sta >geo_mul_inputs+4
    lda 113
    adc 137
    sta >geo_mul_inputs+6
    jsl GeoV3_Mul32x32
    clc
    lda 79
    adc >geo_mul_p
    sta 79
    lda 81
    adc >geo_mul_p+2
    sta 81
    lda 83
    adc >geo_mul_p+4
    sta 83
    lda 85
    adc >geo_mul_p+6
    sta 85
    /* b += (previous_z - current_z) * (previous_x + current_x). */
    sec
    lda 111
    sbc 135
    sta >geo_mul_inputs
    lda 113
    sbc 137
    sta >geo_mul_inputs+2
    clc
    lda 103
    adc 127
    sta >geo_mul_inputs+4
    lda 105
    adc 129
    sta >geo_mul_inputs+6
    jsl GeoV3_Mul32x32
    clc
    lda 87
    adc >geo_mul_p
    sta 87
    lda 89
    adc >geo_mul_p+2
    sta 89
    lda 91
    adc >geo_mul_p+4
    sta 91
    lda 93
    adc >geo_mul_p+6
    sta 93
    /* c += (previous_x - current_x) * (previous_y + current_y). */
    sec
    lda 103
    sbc 127
    sta >geo_mul_inputs
    lda 105
    sbc 129
    sta >geo_mul_inputs+2
    clc
    lda 107
    adc 131
    sta >geo_mul_inputs+4
    lda 109
    adc 133
    sta >geo_mul_inputs+6
    jsl GeoV3_Mul32x32
    clc
    lda 95
    adc >geo_mul_p
    sta 95
    lda 97
    adc >geo_mul_p+2
    sta 97
    lda 99
    adc >geo_mul_p+4
    sta 99
    lda 101
    adc >geo_mul_p+6
    sta 101
    rtl
}

/* Stable observer-array pointers are staged once per calculateFaceDepths(). */
static void FaceDepth_ScanSetup(VertexArrays3D *vtx)
{
    memcpy(&face_depth_inputs[2],&vtx->xo,4);
    memcpy(&face_depth_inputs[4],&vtx->yo,4);
    memcpy(&face_depth_inputs[6],&vtx->zo,4);
    memcpy(&face_depth_inputs[8],&vtx->x2d,4);
    memcpy(&face_depth_inputs[10],&vtx->y2d,4);
}
static void FaceDepth_ScanFace(FaceArrays3D *faces,int offset,int n)
{
    int *indices=faces->vertex_indices_buffer+offset;
    memcpy(face_depth_inputs,&indices,4);
    face_depth_inputs[12]=(unsigned int)n;
    FaceDepth_ScanKernel();
}

/* Averages still truncate toward zero. Most coordinate sums fit signed 32
 * bits: prove that before using a cheaper 32-bit division. Four-vertex faces
 * use a shift with a negative-value bias; unproven wide sums retain C division.
 * Coordinate sums cannot overflow when adding 3 (at most MAX_VERTICES int32s).
 */
static Fixed32 FaceDepth_CentroidCoordinate(Fixed64 sum,int count)
{
    Fixed32 narrow=(Fixed32)sum;
    if(sum==(Fixed64)narrow){
        if(count==4)return (narrow+(narrow<0?3L:0L))>>2;
        return narrow/count;
    }
    if(count==4)return (Fixed32)((sum+(sum<0?3:0))>>2);
    return (Fixed32)(sum/count);
}
