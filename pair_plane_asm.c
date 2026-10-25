/* Assembly predicates for pair_plane_before()/pair_plane_after().
 * Independent entry point preserves the V6 GEO classifier's timing.
 * Shared multiplication: signed 32x32 -> 64; each product shifts separately.
 * Non-reentrant global staging, like the existing GEO assembly routines.
 *
 * Direct-page layout (byte offsets):
 * 1: index pointer; 5/9/13: x/y/z pointers (four bytes each)
 * 17: remaining vertices; 19: observer side; 21..24: unused
 * 25..31: signed accumulator; 33..39: epsilon; 41..47: -epsilon
 * 49..55: d; 57..67: a/b/c (two words each)
 * 69: coordinate offset / comparison scratch; 71: index-list offset
 * 73: requested predicate (1 same side, 0 opposite side).
 * The multiplier preserves D, X and Y; no C calls with temporary D.
 */
static unsigned int pair_plane_words[37];
static int pair_plane_result;

asm void PairPlane_Match32Kernel(void)
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
    sbc #74
    tcs
    tcd

    /* Copy the packet using long global addressing and direct-page stores. */
    ldx #72
pair_plane_copy:
    lda >pair_plane_words,x
    sta 1,x
    dex
    dex
    bpl pair_plane_copy
    stz 71

pair_plane_vertex:
    lda 17
    bne pair_plane_have_vertex
    brl pair_plane_finish
pair_plane_have_vertex:
    ldy 71
    lda [1],y           // OBJ indices are one-based, native 16-bit integers.
    dec a
    asl a
    asl a               // Four bytes per Fixed32 coordinate.
    sta 69
    inc 71
    inc 71
    lda 49
    sta 25
    lda 51
    sta 27
    lda 53
    sta 29
    lda 55
    sta 31              // Start from d; bounded sums cannot overflow.

    /* Add (coefficient_x * coordinate_x) >> 16, in signed 64 bits. */
    lda 57
    sta >geo_mul_a0
    lda 59
    sta >geo_mul_a1
    ldy 69
    lda [5],y
    sta >geo_mul_b0
    iny
    iny
    lda [5],y
    sta >geo_mul_b1
    jsl GeoV3_Mul32x32
    /* Discard the low word, then sign-extend the shifted 48-bit product. */
    lda >geo_mul_p+6
    bpl pair_plane_x_positive
    lda #0xFFFF
    bra pair_plane_x_extend
pair_plane_x_positive:
    lda #0
pair_plane_x_extend:
    tax                 // X supplies the sign-extension word, not a pointer.
    clc
    lda 25
    adc >geo_mul_p+2
    sta 25
    lda 27
    adc >geo_mul_p+4
    sta 27
    lda 29
    adc >geo_mul_p+6
    sta 29
    txa                 // TXA does not alter carry from the third ADC.
    adc 31
    sta 31

    /* Add (coefficient_y * coordinate_y) >> 16, in signed 64 bits. */
    lda 61
    sta >geo_mul_a0
    lda 63
    sta >geo_mul_a1
    ldy 69
    lda [9],y
    sta >geo_mul_b0
    iny
    iny
    lda [9],y
    sta >geo_mul_b1
    jsl GeoV3_Mul32x32
    /* Discard the low word, then sign-extend the shifted 48-bit product. */
    lda >geo_mul_p+6
    bpl pair_plane_y_positive
    lda #0xFFFF
    bra pair_plane_y_extend
pair_plane_y_positive:
    lda #0
pair_plane_y_extend:
    tax                 // X supplies the sign-extension word, not a pointer.
    clc
    lda 25
    adc >geo_mul_p+2
    sta 25
    lda 27
    adc >geo_mul_p+4
    sta 27
    lda 29
    adc >geo_mul_p+6
    sta 29
    txa                 // TXA does not alter carry from the third ADC.
    adc 31
    sta 31

    /* Add (coefficient_z * coordinate_z) >> 16, in signed 64 bits. */
    lda 65
    sta >geo_mul_a0
    lda 67
    sta >geo_mul_a1
    ldy 69
    lda [13],y
    sta >geo_mul_b0
    iny
    iny
    lda [13],y
    sta >geo_mul_b1
    jsl GeoV3_Mul32x32
    /* Discard the low word, then sign-extend the shifted 48-bit product. */
    lda >geo_mul_p+6
    bpl pair_plane_z_positive
    lda #0xFFFF
    bra pair_plane_z_extend
pair_plane_z_positive:
    lda #0
pair_plane_z_extend:
    tax                 // X supplies the sign-extension word, not a pointer.
    clc
    lda 25
    adc >geo_mul_p+2
    sta 25
    lda 27
    adc >geo_mul_p+4
    sta 27
    lda 29
    adc >geo_mul_p+6
    sta 29
    txa                 // TXA does not alter carry from the third ADC.
    adc 31
    sta 31

    /* Compare acc to epsilon without subtraction overflow.
     * Bias the signed top words; compare remaining words as unsigned. */
    lda 39
    eor #0x8000
    sta 69              // Coordinate offset is no longer needed for this vertex.
    lda 31
    eor #0x8000
    cmp 69
    bcc pair_plane_upper_less
    bne pair_plane_upper_greater
    lda 29
    cmp 37
    bcc pair_plane_upper_less
    bne pair_plane_upper_greater
    lda 27
    cmp 35
    bcc pair_plane_upper_less
    bne pair_plane_upper_greater
    lda 25
    cmp 33
    bcc pair_plane_upper_less
    bne pair_plane_upper_greater
    brl pair_plane_check_negative
pair_plane_upper_less:
    brl pair_plane_check_negative
pair_plane_upper_greater:
    brl pair_plane_positive

pair_plane_check_negative:

    /* Compare acc to -epsilon without subtraction overflow.
     * Bias the signed top words; compare remaining words as unsigned. */
    lda 47
    eor #0x8000
    sta 69              // Coordinate offset is no longer needed for this vertex.
    lda 31
    eor #0x8000
    cmp 69
    bcc pair_plane_lower_less
    bne pair_plane_lower_greater
    lda 29
    cmp 45
    bcc pair_plane_lower_less
    bne pair_plane_lower_greater
    lda 27
    cmp 43
    bcc pair_plane_lower_less
    bne pair_plane_lower_greater
    lda 25
    cmp 41
    bcc pair_plane_lower_less
    bne pair_plane_lower_greater
    brl pair_plane_next
pair_plane_lower_less:
    brl pair_plane_negative
pair_plane_lower_greater:
    brl pair_plane_next

pair_plane_positive:
    lda #1
    bra pair_plane_side
pair_plane_negative:
    lda #0xFFFF
pair_plane_side:
    /* Offset 73: 1 requires the observer side, 0 the opposite side.
     * Reject the first violating vertex; neutral vertices jump to next.
     */
    cmp 19
    beq pair_plane_same
    lda 73
    beq pair_plane_next
    bra pair_plane_fail
pair_plane_same:
    lda 73
    bne pair_plane_next
pair_plane_fail:
    lda #0
    bra pair_plane_store
pair_plane_next:
    dec 17
    brl pair_plane_vertex
pair_plane_finish:
    lda #1              // No violation, including empty/all-neutral faces.
pair_plane_store:
    sta >pair_plane_result
    tsc
    clc
    adc #74
    tcs
    pld
    ply
    plx
    pla
    plp
    rtl
}


/* Stage parameters once per single-plane predicate. */
static int PairPlane_Match32(Model3D *model, int face,
                            Fixed32 a, Fixed32 b, Fixed32 c,
                            Fixed64 d, Fixed64 epsilon, int obs, int want_same)
{
    int *indices = model->faces.vertex_indices_buffer
                 + model->faces.vertex_indices_ptr[face];
    Fixed64 negative_epsilon = -epsilon;
    memcpy(&pair_plane_words[0], &indices, 4);
    memcpy(&pair_plane_words[2], &model->vertices.xo, 4);
    memcpy(&pair_plane_words[4], &model->vertices.yo, 4);
    memcpy(&pair_plane_words[6], &model->vertices.zo, 4);
    pair_plane_words[8] = model->faces.vertex_count[face];
    pair_plane_words[9] = (unsigned int)obs;
    memcpy(&pair_plane_words[16], &epsilon, 8);
    memcpy(&pair_plane_words[20], &negative_epsilon, 8);
    memcpy(&pair_plane_words[24], &d, 8);
    memcpy(&pair_plane_words[28], &a, 4);
    memcpy(&pair_plane_words[30], &b, 4);
    memcpy(&pair_plane_words[32], &c, 4);
    pair_plane_words[36] = (unsigned int)want_same;
    PairPlane_Match32Kernel();
    return pair_plane_result;
}
