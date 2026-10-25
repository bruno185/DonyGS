/* GEO plane classification, signed 32-bit coefficients with 64-bit sums.
 * Included by engine.c after the V5 multiplication helper.
 * Non-reentrant, like GeoV3_Mul32x32: global input/output staging is shared.
 * Each product is shifted separately, preserving the C rounding behavior.
 */
static unsigned int geo_rel_words[40];
static int geo_rel_result;

/* Direct-page layout (byte offsets):
 * 1: index pointer; 5/9/13: x/y/z pointers (four bytes per pointer)
 * 17: remaining vertices; 19: observer side; 21/23: all_same/all_opp
 * 25..31: accumulator; 33..39: epsilon; 41..47: -epsilon
 * 49..55: d; 57..67: a/b/c (two words each)
 * 69: coordinate byte offset; 71: index-list byte offset.
 * 73: far pointer to the prepared plane's packed vertex-side cache.
 * 77: one-based vertex ID; 79: cache slot byte offset.
 * Cache entries are valid for exactly this plane/epsilon/transformation.
 * Even on-plane vertices are cached; no geometric tolerance is changed.
 * The existing multiplier preserves D, X and Y across its long call.
 */
asm void GeoRel_Classify32Kernel(void)
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
    sbc #80
    tcs
    tcd

    /* Copy only live packet fields. Unrolled long loads/direct-page stores
     * avoid loop/index overhead on every relation, including vertex-cache hits.
     * Flags, accumulator and per-vertex offsets are initialized below instead.
     */
    lda >geo_rel_words
    sta 1
    lda >geo_rel_words+2
    sta 3
    lda >geo_rel_words+4
    sta 5
    lda >geo_rel_words+6
    sta 7
    lda >geo_rel_words+8
    sta 9
    lda >geo_rel_words+10
    sta 11
    lda >geo_rel_words+12
    sta 13
    lda >geo_rel_words+14
    sta 15
    lda >geo_rel_words+16
    sta 17
    lda >geo_rel_words+18
    sta 19
    lda >geo_rel_words+32
    sta 33
    lda >geo_rel_words+34
    sta 35
    lda >geo_rel_words+36
    sta 37
    lda >geo_rel_words+38
    sta 39
    lda >geo_rel_words+40
    sta 41
    lda >geo_rel_words+42
    sta 43
    lda >geo_rel_words+44
    sta 45
    lda >geo_rel_words+46
    sta 47
    lda >geo_rel_words+48
    sta 49
    lda >geo_rel_words+50
    sta 51
    lda >geo_rel_words+52
    sta 53
    lda >geo_rel_words+54
    sta 55
    lda >geo_rel_words+56
    sta 57
    lda >geo_rel_words+58
    sta 59
    lda >geo_rel_words+60
    sta 61
    lda >geo_rel_words+62
    sta 63
    lda >geo_rel_words+64
    sta 65
    lda >geo_rel_words+66
    sta 67
    lda >geo_rel_words+72
    sta 73
    lda >geo_rel_words+74
    sta 75
    stz 71
    lda #1
    sta 21
    sta 23

geo_rel_vertex:
    lda 17
    bne geo_rel_have_vertex
    brl geo_rel_finish
geo_rel_have_vertex:
    ldy 71
    lda [1],y           // OBJ indices are one-based, native 16-bit integers.
    sta 77
    inc 71
    inc 71
    and #63
    asl a
    sta 79
    tay
    lda [73],y
    and #0x1FFF
    cmp 77
    bne geo_rel_vertex_uncached
    lda [73],y
    and #0x6000
    cmp #0x2000
    bne geo_rel_cached_off_plane
    brl geo_rel_next
geo_rel_cached_off_plane:
    cmp #0x4000
    bne geo_rel_cached_negative
    lda #1
    brl geo_rel_side
geo_rel_cached_negative:
    lda #0xFFFF
    brl geo_rel_side
geo_rel_vertex_uncached:
    lda 77
    dec a
    asl a
    asl a               // Four bytes per Fixed32 coordinate.
    sta 69
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
    ora 59
    bne geo_rel_x_multiply
    brl geo_rel_x_done
geo_rel_x_multiply:
    lda 57
    sta >geo_mul_inputs
    lda 59
    sta >geo_mul_inputs+2
    ldy 69
    lda [5],y
    sta >geo_mul_inputs+4
    iny
    iny
    lda [5],y
    sta >geo_mul_inputs+6
    jsl GeoV3_Mul32x32
    /* Discard the low word, then sign-extend the shifted 48-bit product. */
    lda >geo_mul_p+6
    bpl geo_rel_x_positive
    lda #0xFFFF
    bra geo_rel_x_extend
geo_rel_x_positive:
    lda #0
geo_rel_x_extend:
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
geo_rel_x_done:

    /* Add (coefficient_y * coordinate_y) >> 16, in signed 64 bits. */
    lda 61
    ora 63
    bne geo_rel_y_multiply
    brl geo_rel_y_done
geo_rel_y_multiply:
    lda 61
    sta >geo_mul_inputs
    lda 63
    sta >geo_mul_inputs+2
    ldy 69
    lda [9],y
    sta >geo_mul_inputs+4
    iny
    iny
    lda [9],y
    sta >geo_mul_inputs+6
    jsl GeoV3_Mul32x32
    /* Discard the low word, then sign-extend the shifted 48-bit product. */
    lda >geo_mul_p+6
    bpl geo_rel_y_positive
    lda #0xFFFF
    bra geo_rel_y_extend
geo_rel_y_positive:
    lda #0
geo_rel_y_extend:
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
geo_rel_y_done:

    /* Add (coefficient_z * coordinate_z) >> 16, in signed 64 bits. */
    lda 65
    ora 67
    bne geo_rel_z_multiply
    brl geo_rel_z_done
geo_rel_z_multiply:
    lda 65
    sta >geo_mul_inputs
    lda 67
    sta >geo_mul_inputs+2
    ldy 69
    lda [13],y
    sta >geo_mul_inputs+4
    iny
    iny
    lda [13],y
    sta >geo_mul_inputs+6
    jsl GeoV3_Mul32x32
    /* Discard the low word, then sign-extend the shifted 48-bit product. */
    lda >geo_mul_p+6
    bpl geo_rel_z_positive
    lda #0xFFFF
    bra geo_rel_z_extend
geo_rel_z_positive:
    lda #0
geo_rel_z_extend:
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
geo_rel_z_done:

    /* Compare acc to epsilon without subtraction overflow.
     * Bias the signed top words; compare remaining words as unsigned. */
    lda 39
    eor #0x8000
    sta 69              // Coordinate offset is no longer needed for this vertex.
    lda 31
    eor #0x8000
    cmp 69
    bcc geo_rel_upper_less
    bne geo_rel_upper_greater
    lda 29
    cmp 37
    bcc geo_rel_upper_less
    bne geo_rel_upper_greater
    lda 27
    cmp 35
    bcc geo_rel_upper_less
    bne geo_rel_upper_greater
    lda 25
    cmp 33
    bcc geo_rel_upper_less
    bne geo_rel_upper_greater
    brl geo_rel_check_negative
geo_rel_upper_less:
    brl geo_rel_check_negative
geo_rel_upper_greater:
    brl geo_rel_positive

geo_rel_check_negative:

    /* Compare acc to -epsilon without subtraction overflow.
     * Bias the signed top words; compare remaining words as unsigned. */
    lda 47
    eor #0x8000
    sta 69              // Coordinate offset is no longer needed for this vertex.
    lda 31
    eor #0x8000
    cmp 69
    bcc geo_rel_lower_less
    bne geo_rel_lower_greater
    lda 29
    cmp 45
    bcc geo_rel_lower_less
    bne geo_rel_lower_greater
    lda 27
    cmp 43
    bcc geo_rel_lower_less
    bne geo_rel_lower_greater
    lda 25
    cmp 41
    bcc geo_rel_lower_less
    bne geo_rel_lower_greater
    brl geo_rel_on_plane
geo_rel_lower_less:
    brl geo_rel_negative
geo_rel_lower_greater:
    brl geo_rel_on_plane

geo_rel_on_plane:
    lda 77
    ora #0x2000
    ldy 79
    sta [73],y
    brl geo_rel_next
geo_rel_positive:
    lda 77
    ora #0x4000
    ldy 79
    sta [73],y
    lda #1
    bra geo_rel_side
geo_rel_negative:
    lda 77
    ora #0x6000
    ldy 79
    sta [73],y
    lda #0xFFFF
geo_rel_side:
    cmp 19
    bne geo_rel_opposite
    stz 23              // A vertex on the observer side rules out all_opp.
    bra geo_rel_flags
geo_rel_opposite:
    stz 21              // A vertex on the other side rules out all_same.
geo_rel_flags:
    lda 21
    ora 23
    bne geo_rel_next
    lda #0              // Mixed sides: early return, relation undetermined.
    bra geo_rel_store
geo_rel_next:
    dec 17
    brl geo_rel_vertex
geo_rel_finish:
    lda 21
    beq geo_rel_not_same
    lda #0xFFFF         // Preserve C priority: all_same wins, even if no vertices.
    bra geo_rel_store
geo_rel_not_same:
    lda 23
    beq geo_rel_store
    lda #1
geo_rel_store:
    sta >geo_rel_result
    tsc
    clc
    adc #80
    tcs
    pld
    ply
    plx
    pla
    plp
    rtl
}

/* Called once per uncached relation, only after the coefficient/d bounds test.
 * Packet copying avoids dependence on the ORCA/C assembly parameter ABI.
 */
static int GeoRel_Classify32(Model3D *model, int face,
                            Fixed32 a, Fixed32 b, Fixed32 c,
                            Fixed64 d, Fixed64 epsilon, int obs,
                            unsigned int *vertex_sides)
{
    int *indices = model->faces.vertex_indices_buffer
                 + model->faces.vertex_indices_ptr[face];
    Fixed64 negative_epsilon = -epsilon;
    order_asm_packet_plane=NULL; /* Raw diagnostic calls replace the packet. */
    memcpy(&geo_rel_words[0], &indices, 4);
    memcpy(&geo_rel_words[2], &model->vertices.xo, 4);
    memcpy(&geo_rel_words[4], &model->vertices.yo, 4);
    memcpy(&geo_rel_words[6], &model->vertices.zo, 4);
    geo_rel_words[8] = model->faces.vertex_count[face];
    geo_rel_words[9] = (unsigned int)obs;
    memcpy(&geo_rel_words[16], &epsilon, 8);
    memcpy(&geo_rel_words[20], &negative_epsilon, 8);
    memcpy(&geo_rel_words[24], &d, 8);
    memcpy(&geo_rel_words[28], &a, 4);
    memcpy(&geo_rel_words[30], &b, 4);
    memcpy(&geo_rel_words[32], &c, 4);
    memcpy(&geo_rel_words[36], &vertex_sides, 4);
    GeoRel_Classify32Kernel();
    return geo_rel_result;
}

/* Reuse invariant plane inputs while classifying successive candidate faces.
 * Only the index-list pointer and vertex count vary. This also reduces the C
 * call packet to three arguments instead of repeatedly passing 64-bit values.
 */
static int GeoRel_ClassifyPrepared(Model3D *model,int face,OrderFixedPlane *plane)
{
    static int packet_face=-1;
    static Fixed64 packet_epsilon=0;
    int *indices,result;
    if(order_asm_packet_plane!=plane || packet_face!=plane->face || packet_epsilon!=plane->epsilon){
        result=GeoRel_Classify32(model,face,plane->a32,plane->b32,plane->c32,
                                 plane->d,plane->epsilon,plane->observer,plane->vertex_sides);
        order_asm_packet_plane=plane;packet_face=plane->face;packet_epsilon=plane->epsilon;
        return result;
    }
    indices=model->faces.vertex_indices_buffer+model->faces.vertex_indices_ptr[face];
    memcpy(&geo_rel_words[0],&indices,4);
    geo_rel_words[8]=model->faces.vertex_count[face];
    GeoRel_Classify32Kernel();
    return geo_rel_result;
}
