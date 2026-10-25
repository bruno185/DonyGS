/* V4 reference for comparison only; do not include in engine.c. */
unsigned int geo_mul_a0, geo_mul_a1, geo_mul_b0, geo_mul_b1;
unsigned int geo_mul_p[4];

/*
 * Exact signed Fixed32 x Fixed32 -> Fixed64 multiply.
 * The product uses 16 or 32 shift/add steps; zero inputs return directly.
 * geo_mul_p is written as four little-endian words.
 * Direct-page offsets: sign=1, swap=3..4, M=5..11, Q=13..15, P=17..23.
 * Global transfers explicitly use long addressing, independent of D/DBR.
 */
asm void GeoV3_Mul32x32(void)
{
    /* Calling convention: save A and X as 16-bit values; restore D and P.
     * Y and DBR remain unchanged. Do not call C with the temporary D.
     * Global inputs/output make this routine non-reentrant.
     * This is a raw product: two 16.16 inputs produce a 32.32 result.
     * The C wrapper performs the subsequent 16-bit right shift.
     */
    php                 // Save P before changing register widths and decimal mode.
    rep     #0x30       // Use 16-bit A and X/Y for word operations and the counter.
    pha                 // Save A (2 bytes).
    phx                 // Save X, used as the iteration counter.
    phd                 // Save the caller's direct page.
    cld                 // Use binary arithmetic for ADC/SBC.
    /* Reserve 24 bytes below the saved registers.
     * The stack grows downward; S points just below the first usable byte.
     * Little-endian words (offset -> bits):
     *   1: sign (0 positive, 1 negative); 3..4: swap temporary
     *   5,7,9,11: M, bits 0..15,16..31,32..47,48..63
     *   13,15: Q, bits 0..15,16..31
     *   17,19,21,23: P, bits 0..15,16..31,32..47,48..63
     * Short numeric operands address memory relative to D.
     * The > prefix forces long addressing for globals.
     */
    tsc
    sec
    sbc     #24
    tcs
    tcd                 // D = S: reserved bytes are D+1 through D+24.

    /* M = |a|: load both words of a and clear the upper 32 bits.
     * Bit 15 of the high input word determines the sign of a.
     */
    lda     >geo_mul_a0
    sta     5
    lda     >geo_mul_a1
    sta     7
    lda     #0
    sta     9
    sta     11
    sta     1

    lda     7
    bpl     geo_mul_a_positive

    /* 32-bit two's complement: invert the bits, then add 1.
     * ADC #0 propagates the carry from the low word to the high word.
     * |INT32_MIN| = 0x80000000 fits in an unsigned magnitude.
     */
    lda     5
    eor     #0xFFFF
    clc
    adc     #1
    sta     5
    lda     7
    eor     #0xFFFF
    adc     #0
    sta     7
    lda     #1
    sta     1

geo_mul_a_positive:
    /* Q = |b|, an unsigned 32-bit multiplier.
     * For negative b, take two's complement and toggle the final sign:
     * sign = sign(a) XOR sign(b).
     */
    lda     >geo_mul_b0
    sta     13
    lda     >geo_mul_b1
    sta     15
    lda     15
    bpl     geo_mul_b_positive

    lda     13
    eor     #0xFFFF
    clc
    adc     #1
    sta     13
    lda     15
    eor     #0xFFFF
    adc     #0
    sta     15
    lda     1
    eor     #1
    sta     1

geo_mul_b_positive:
    /* P = 0 before any early return or accumulation.
     */
    lda     #0
    sta     17
    sta     19
    sta     21
    sta     23

    /* A zero magnitude needs no shift/add iterations.
     */
    lda     5
    ora     7
    beq     geo_mul_store
    lda     13
    ora     15
    beq     geo_mul_store

    /* Q < 65536: all its significant bits fit in 16 iterations.
     */
    ldx     #16
    lda     15
    beq     geo_mul_loop

    /* Q needs 32 bits. If M fits in 16 bits, exchange magnitudes so
     * the smaller operand becomes Q and only 16 iterations are needed.
     * If both need 32 bits, their order does not affect the loop count.
     * The final sign is already recorded and multiplication is commutative.
     * M's upper words (9,11) are still zero at this point.
     */
    lda     7
    bne     geo_mul_use_32
    lda     5
    sta     3           // Temporarily save the low word of the old M.
    lda     13
    sta     5           // New M low word = old Q low word.
    lda     15
    sta     7           // New M high word = old Q high word.
    lda     3
    sta     13          // New Q low word = old M low word.
    stz     15          // Old M high word was zero; new Q fits in 16 bits.
    bra     geo_mul_loop

geo_mul_use_32:
    ldx     #32

geo_mul_loop:
    /* Q >>= 1 across two words: LSR transfers bit 0 of the high word
     * into bit 15 of the low word through ROR. After ROR, C holds
     * the previous bit 0 of Q, selecting whether to add the current M.
     */
    lsr     15
    ror     13
    bcc     geo_mul_skip_add // Extracted bit is zero: skip adding M.

    /* P += M across four words. Clear C so the extracted Q bit does
     * not introduce an extra +1. LDA/STA leave C unchanged, allowing
     * the four ADC instructions to propagate carry from low to high.
     */
    clc
    lda     17
    adc     5
    sta     17
    lda     19
    adc     7
    sta     19
    lda     21
    adc     9
    sta     21
    lda     23
    adc     11
    sta     23

geo_mul_skip_add:
    /* M <<= 1 across 64 bits for the next bit of Q.
     * ASL sets C; each ROL inserts the carry from the previous word.
     * Invariant: after i iterations, P contains the contributions of the
     * lowest i original multiplier bits, and M is |a| << i (64 bits).
     */
    asl     5
    rol     7
    rol     9
    rol     11

    dex                 // One more bit of Q has been processed.
    bne     geo_mul_loop // DEX sets Z; previous carry values are irrelevant.

    /* The magnitude is complete: if the final sign is 1, form -P
     * using two's complement across four words. Clear C once,
     * add 1 to the low word, then propagate carry with ADC #0.
     */
    lda     1
    beq     geo_mul_store

    lda     17
    eor     #0xFFFF
    clc
    adc     #1
    sta     17
    lda     19
    eor     #0xFFFF
    adc     #0
    sta     19
    lda     21
    eor     #0xFFFF
    adc     #0
    sta     21
    lda     23
    eor     #0xFFFF
    adc     #0
    sta     23

geo_mul_store:
    /* Export P before releasing the workspace: four words, least
     * significant first. Long addressing is independent of D and DBR.
     */
    lda     17
    sta     >geo_mul_p
    lda     19
    sta     >geo_mul_p+2
    lda     21
    sta     >geo_mul_p+4
    lda     23
    sta     >geo_mul_p+6
    /* Release exactly the 24 reserved bytes. The saved D, X, A and P
     * are then accessible to the matching pull instructions.
     */
    tsc
    clc
    adc     #24
    tcs
    pld                 // Restore the caller's direct page.
    plx                 // Restore X while index registers are still 16-bit.
    pla                 // Restore A while the accumulator is still 16-bit.
    plp                 // Restore original flags and register widths.
    rtl                 // Long return; S is back to its entry value.
}

static unsigned int GeoV3_LowWord(Fixed32 value)
{
    return (unsigned int)((unsigned long)value & 0xFFFFUL);
}

static unsigned int GeoV3_HighWord(Fixed32 value)
{
    return (unsigned int)(((unsigned long)value >> 16) & 0xFFFFUL);
}

static void GeoV3_Mul32x32SetInputs(Fixed32 a, Fixed32 b)
{
    geo_mul_a0 = GeoV3_LowWord(a);
    geo_mul_a1 = GeoV3_HighWord(a);
    geo_mul_b0 = GeoV3_LowWord(b);
    geo_mul_b1 = GeoV3_HighWord(b);
    GeoV3_Mul32x32();
}

/*
 * Diagnostic only: compare the assembly 32x32 multiply with the C reference.
 * Returns 0 if every case matches, 1 on the first mismatch.
 * On mismatch, the five output pointers receive the failing test index,
 * operands, and both 64-bit products. The first 100 cases are boundary pairs;
 * the following 2048 cases are deterministic pseudo-random bit patterns.
 */
int GeoV3_Mul32x32SelfTest(unsigned long *test_index,
                           Fixed32 *bad_a, Fixed32 *bad_b,
                           Fixed64 *asm_product_out,
                           Fixed64 *c_product_out)
{
    static const Fixed32 boundary[] = {
        0L, 1L, -1L, 2L, -2L,
        65535L, 65536L, -65536L,
        2147483647L, (-2147483647L - 1L)
    };
    unsigned long state;
    unsigned long index;
    unsigned long bits;
    unsigned int i, j;
    Fixed32 a, b;
    Fixed64 asm_product, c_product;

    index = 0;
    for (i = 0; i < (unsigned int)(sizeof(boundary) / sizeof(boundary[0])); ++i) {
        for (j = 0; j < (unsigned int)(sizeof(boundary) / sizeof(boundary[0])); ++j) {
            a = boundary[i];
            b = boundary[j];
            GeoV3_Mul32x32SetInputs(a, b);
            memcpy(&asm_product, geo_mul_p, sizeof(asm_product));
            c_product = (Fixed64)a * (Fixed64)b;
            if (asm_product != c_product) {
                *test_index = index;
                *bad_a = a;
                *bad_b = b;
                *asm_product_out = asm_product;
                *c_product_out = c_product;
                return 1;
            }
            ++index;
        }
    }

    state = 0x13579BDFUL;
    for (i = 0; i < 2048; ++i) {
        state = state * 1664525UL + 1013904223UL;
        bits = state;
        memcpy(&a, &bits, sizeof(a));
        state = state * 1664525UL + 1013904223UL;
        bits = state;
        memcpy(&b, &bits, sizeof(b));

        GeoV3_Mul32x32SetInputs(a, b);
        memcpy(&asm_product, geo_mul_p, sizeof(asm_product));
        c_product = (Fixed64)a * (Fixed64)b;
        if (asm_product != c_product) {
            *test_index = index;
            *bad_a = a;
            *bad_b = b;
            *asm_product_out = asm_product;
            *c_product_out = c_product;
            return 1;
        }
        ++index;
    }

    *test_index = index;
    return 0;
}

