/*
 * ============================================================================
 *  zbuffer_common.c — Primitives communes aux rendus Z-buffer
 * ============================================================================
 *
 *  Rôle :
 *    Partager les calculs de profondeur et le noyau de spans des rendus
 *    plein écran direct et offscreen.
 *
 *  Organisation :
 *    Fournit l'interpolation entière utilisée pour le clipping des spans
 *    scanline et offscreen, puis le noyau assembleur de test Z et dessin
 *    des spans plein écran. Le paquet zspan_p_* est commun aux deux rendus.
 *
 *  Dépendances et contraintes :
 *    Inclus par engine.c avant les moteurs Z-buffer. Sur ORCA/C, UWORD16
 *    et unsigned int sont des types non signés de 16 bits ; long est signé
 *    sur 32 bits. Les produits intermédiaires doivent rester représentables.
 *
 * ============================================================================
 */
#include "zbuffer_fullscreen.h"

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_LerpZ
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Interpoler un code de profondeur lors du clipping horizontal d'un span.
 *
 *  Paramètres :
 *    [entrée] za/zb : codes de profondeur aux abscisses xa/xb.
 *    [entrée] xa/xb : abscisses des extrémités ; x : abscisse demandée.
 *
 *  Retour :
 *    za si xa == xb ; sinon za + (zb-za)*(x-xa)/(xb-xa), converti en UWORD16.
 *
 *  Fonctionnement :
 *    Conserve les opérations en long et la troncature de la division entière
 *    des anciennes variantes scanline et offscreen. Aucun état modifié.
 *
 *  Limites :
 *    Aucune saturation ; les différences et produits doivent être valides
 *    dans les types intermédiaires. L'appelant choisit le domaine de x.
 *
 * ----------------------------------------------------------------------------
 */
static UWORD16 ZBuffer_LerpZ(UWORD16 za, UWORD16 zb, int xa, int xb, int x)
{
    long num;
    /* Traiter les extrémités confondues avant la division. */
    if (xb == xa) return za;
    /* Interpoler avec les mêmes conversions et le même ordre d'opérations. */
    num = ((long)zb - (long)za) * (long)(x - xa);
    return (UWORD16)((long)za + num / (long)(xb - xa));
}

/* Placer le noyau partagé dans le segment des deux rendus plein écran. */
segment "ZBUF";

/* Paquet d'entrée commun, préparé intégralement selon le mode par
 * l'appelant avant chaque span. Le noyau ne le modifie pas. Ces données
 * partagées imposent des appels synchrones, sans réentrance.
 */
long zspan_p_zacc, zspan_p_zstep;
unsigned long zspan_p_zptr, zspan_p_pptr;
int zspan_p_n, zspan_p_odd, zspan_p_mode, zspan_p_cl, zspan_p_ci, zspan_p_cr;
unsigned int zspan_p_zl, zspan_p_zr;

/*
 * ----------------------------------------------------------------------------
 *  ZBuffer_SpanRun
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Dessiner un span préclippé avec test Z strict dans un noyau 65C816.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Entrées et sorties implicites :
 *    Entrée : paquet partagé zspan_p_* : pointeurs Z et pixels far, parité
 *    gauche, codes gauche/droite, couleurs gauche/intérieur/droite,
 *    accumulateur et pas 16.16, nombre de pixels intérieurs et mode 0
 *    (gauche seul) ou 1 (span complet).
 *
 *  Retour :
 *    Aucun ; écrit Z et les nibbles de couleur.
 *
 *  Registres et indicateurs :
 *    Préserve P et D ; A/X/Y modifiés, DBR inchangé. Mode décimal supposé
 *    désactivé. Cadre D de 32 octets : zacc à 0/2, zstep à 4/6, zptr à 8/10,
 *    pptr à 12/14, compte à 16, couleur basse/haute à 18/20, parité à 22,
 *    masque/valeur à 24/26. Les profondeurs [8],Y peuvent franchir une
 *    banque ; le pointeur de pixels n'est incrémenté que sur 16 bits et doit
 *    rester dans sa banque. Traite les extrémités exactement et l'intérieur
 *    par paires pair/impair. Les pixels de même code Z ne sont pas
 *    remplacés. Non réentrant.
 *
 * ----------------------------------------------------------------------------
 */
static void ZBuffer_SpanRun(void)
{
    asm {

        /* Sauvegarder le contexte et réserver 32 octets ; D est positionné
         * sur le premier octet utile.
         */
        php
        phd
        rep #0x30
        tsc
        sec
        sbc #32
        tcs
        inc a
        tcd

        /* Copier le paquet dans le cadre direct et préparer les deux nibbles
         * de couleur.
         */
        lda >zspan_p_zacc
        sta 0
        lda >zspan_p_zacc+2
        sta 2
        lda >zspan_p_zstep
        sta 4
        lda >zspan_p_zstep+2
        sta 6
        lda >zspan_p_zptr
        sta 8
        lda >zspan_p_zptr+2
        sta 10
        lda >zspan_p_pptr
        sta 12
        lda >zspan_p_pptr+2
        sta 14
        lda >zspan_p_n
        sta 16

        lda >zspan_p_ci
        and #0x000F
        sta 18
        asl a
        asl a
        asl a
        asl a
        sta 20

        lda >zspan_p_odd
        eor #1
        sta 22

        ldy #0

        lda >zspan_p_odd
        beq zbr_l_ev
        lda #0x00F0
        sta 24
        lda >zspan_p_cl
        and #0x000F
        sta 26
        bra zbr_l_go
    zbr_l_ev:
        lda #0x000F
        sta 24
        lda >zspan_p_cl
        and #0x000F
        asl a
        asl a
        asl a
        asl a
        sta 26
    /* Tester la profondeur exacte du pixel gauche avant toute écriture du nibble. */
    zbr_l_go:

        lda >zspan_p_zl
        cmp [8],y
        bcs zbr_lsk
        sta [8],y
        sep #0x20
        lda [12]
        and 24
        ora 26
        sta [12]
        rep #0x20
    zbr_lsk:

        lda >zspan_p_mode
        bne zbr_full
        brl zbr_done

    /* Avancer après le pixel gauche ; l'adresse couleur progresse seulement
     * après un pixel impair.
     */
    zbr_full:
        iny
        iny
        lda >zspan_p_odd
        beq zbr_lev
        inc 12
    zbr_lev:

        lda 16
        bne zbr_int
        brl zbr_right

    /* Traiter seul un premier pixel impair pour aligner le reste sur des paires. */
    zbr_int:
        lda 22
        beq zbp_even

        clc
        lda 0
        adc 4
        sta 0
        lda 2
        adc 6
        sta 2
        cmp [8],y
        bcs zbp_o1
        sta [8],y
        sep #0x20
        lda [12]
        and #0xF0
        ora 18
        sta [12]
        rep #0x20
    zbp_o1:
        iny
        iny
        inc 12
        dec 16

    zbp_even:
        lda 16
        lsr a
        tax
        beq zbp_tail

    /* Avancer zacc en 32 bits pour chaque pixel ; propager C du mot bas au
     * mot haut, puis tester Z.
     */
    zbp_pair:
        clc
        lda 0
        adc 4
        sta 0
        lda 2
        adc 6
        sta 2
        cmp [8],y
        bcs zbp_e1
        sta [8],y
        sep #0x20
        lda [12]
        and #0x0F
        ora 20
        sta [12]
        rep #0x20
    zbp_e1:
        iny
        iny
        clc
        lda 0
        adc 4
        sta 0
        lda 2
        adc 6
        sta 2
        cmp [8],y
        bcs zbp_o2
        sta [8],y
        sep #0x20
        lda [12]
        and #0xF0
        ora 18
        sta [12]
        rep #0x20
    zbp_o2:
        iny
        iny
        inc 12
        dex
        bne zbp_pair

    /* Traiter l'éventuel pixel pair résiduel après les paires. */
    zbp_tail:
        lda 16
        and #1
        bne zbp_t1
        brl zbr_right
    zbp_t1:

        clc
        lda 0
        adc 4
        sta 0
        lda 2
        adc 6
        sta 2
        cmp [8],y
        bcs zbp_e2
        sta [8],y
        sep #0x20
        lda [12]
        and #0x0F
        ora 20
        sta [12]
        rep #0x20
    zbp_e2:
        iny
        iny

    /* Recalculer la parité et employer le code exact de l'extrémité droite. */
    zbr_right:
        lda >zspan_p_n
        clc
        adc 22
        and #1
        beq zbr_r_ev
        lda #0x00F0
        sta 24
        lda >zspan_p_cr
        and #0x000F
        sta 26
        bra zbr_r_go
    zbr_r_ev:
        lda #0x000F
        sta 24
        lda >zspan_p_cr
        and #0x000F
        asl a
        asl a
        asl a
        asl a
        sta 26
    zbr_r_go:
        lda >zspan_p_zr
        cmp [8],y
        bcs zbr_rsk
        sta [8],y
        sep #0x20
        lda [12]
        and 24
        ora 26
        sta [12]
        rep #0x20
    zbr_rsk:

    /* Libérer exactement le cadre avant de restaurer D et P. */
    zbr_done:
        rep #0x30
        tsc
        clc
        adc #32
        tcs
        pld
        plp
    }
}

/* Rétablir le segment utilisé par le module scanline inclus ensuite. */
segment "data";
