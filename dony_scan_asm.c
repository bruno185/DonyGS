/*
 * ============================================================================
 *  dony_scan_asm.c — Filtrage assembleur des candidats GEO V3
 * ============================================================================
 *
 *  Rôle :
 *    Sauter en un appel les candidates dont les intervalles de profondeur ou
 *    les boîtes projetées excluent le recouvrement. Rendre au C la première
 *    position nécessitant les tests géométriques.
 *
 *  Organisation :
 *    Deux wrappers préparent les pointeurs stables, les bornes de la face
 *    sélectionnée et la position de départ. Le noyau 65C816 copie le paquet
 *    dans un cadre de page directe sur la pile, puis parcourt la permutation
 *    sans la modifier.
 *
 *  Dépendances et contraintes :
 *    Inclus par dony.c lorsque le filtrage assembleur est activé. Pointeurs
 *    far ORCA/C sur quatre octets, bornes sur 16 bits et profondeur 16.16.
 *    Les soustractions d'intervalles conservent les règles du C natif.
 *    Paquet global non réentrant ; disposition du cadre et registres décrits
 *    au niveau du noyau.
 *
 * ============================================================================
 */
static unsigned int geo_scan_words[22];
static int geo_scan_result;
static int geo_scan_cached_p = -1;


/*
 * ----------------------------------------------------------------------------
 *  GeoV3_ScanCandidates
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Trouver le prochain candidat non rejeté par les tests rapides de la
 *    méthode de classement.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Entrées et sorties implicites :
 *    Entrée : geo_scan_words, préparé par les deux wrappers C.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    geo_scan_result, position dans la liste triée, ou nf à épuisement.
 *
 *  Fonctionnement :
 *    Ne calcule aucune relation de plan et ne modifie pas l'ordre des faces.
 *
 *    Calculs binaires 16 bits.
 *
 *    Les accès globaux > ignorent DBR.
 *
 *  Registres et indicateurs :
 *    Préserve P, A, Y et D ; X et DBR ne sont pas utilisés.
 *
 *  Organisation mémoire :
 *    Cadre D de 44 octets : pointeurs far tri/zmax/minx/maxx/miny/maxy à
 *    1/5/9/13/17/21 ; zmin de P à 25/27, bornes de P à 29/31/33/35,
 *    position/fin à 37/39, temporaire à 41 et offset Q*2 à 43.
 *
 * ----------------------------------------------------------------------------
 */
asm void GeoV3_ScanCandidates(void)
{
    /* Sauvegarder le contexte et réserver 44 octets de page directe sur la pile. */
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


    /* Copier le paquet invariant ; les temporaires 41 et 43 seront écrits
     * avant lecture.
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
/* Arrêter au-delà de la dernière position, sinon charger l'identifiant Q. */
geo_scan_loop:
    lda 37
    cmp 39
    bcc geo_scan_have_q
    brl geo_scan_done
/* Convertir Q en offsets : deux octets pour les bornes, quatre pour les
 * profondeurs.
 */
geo_scan_have_q:
    asl a
    tay
    lda [1],y
    asl a
    sta 43
    asl a
    tay
    iny
    iny
    lda [5],y
    /* Comparer les profondeurs signées par mots hauts biaisés puis mots bas
     * non signés ; rejeter zmax(Q) <= zmin(P).
     */
    eor #0x8000
    cmp 27
    bcc geo_scan_depth_reject
    bne geo_scan_depth_pass
    dey
    dey
    lda [5],y
    cmp 25
    bcc geo_scan_depth_reject
    beq geo_scan_depth_reject
    bra geo_scan_depth_pass
geo_scan_depth_reject:
    brl geo_scan_reject
/* Tester X avec la même soustraction 16 bits que le chemin C ; un écart nul
 * rejette aussi le candidat.
 */
geo_scan_depth_pass:
    ldy 43


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
/* Appliquer à Y le même test d'intervalle, sans modifier ses règles de
 * débordement 16 bits.
 */
geo_scan_x_pass:


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
/* Rendre la position courante au C, qui décidera de la relation géométrique. */
geo_scan_y_pass:


    bra geo_scan_done
/* Passer au candidat suivant après un rejet rapide. */
geo_scan_reject:
    inc 37
    brl geo_scan_loop
/* Publier la position et libérer exactement le cadre avant de restaurer le
 * contexte.
 */
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


/*
 * ----------------------------------------------------------------------------
 *  GeoV3_ScanSetup
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Préparer les pointeurs invariants du filtrage pour un appel à la méthode de classement.
 *
 *  Paramètres :
 *    [entrée] faces, tableaux de métriques ; sorted, indices de faces à base
 *    zéro ; nf, nombre de positions valides.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    paquet global, limite de recherche et invalidation du P mémorisé.
 *
 *  Préconditions :
 *    pointeurs ORCA/C de quatre octets et tableaux stables pendant le tri.
 *
 * ----------------------------------------------------------------------------
 */
static void GeoV3_ScanSetup(FaceArrays3D *faces, int *sorted, int nf)
{
    /* Installer une fois les six pointeurs far et la limite de recherche. */
    memcpy(&geo_scan_words[0], &sorted, 4);
    memcpy(&geo_scan_words[2], &faces->z_max, 4);
    memcpy(&geo_scan_words[4], &faces->minx, 4);
    memcpy(&geo_scan_words[6], &faces->maxx, 4);
    memcpy(&geo_scan_words[8], &faces->miny, 4);
    memcpy(&geo_scan_words[10], &faces->maxy, 4);
    geo_scan_words[19] = (unsigned int)nf;
    geo_scan_cached_p = -1;
}


/*
 * ----------------------------------------------------------------------------
 *  GeoV3_FindCandidate
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Rechercher une position candidate en réutilisant les bornes de la face
 *    P.
 *
 *  Paramètres :
 *    [entrée] faces, métriques immuables pendant le tri ; p, face à base
 *    zéro ; start, première position à examiner dans sorted.
 *
 *  Retour :
 *    Première position non rejetée, ou nf.
 *
 *  Sorties et effets de bord :
 *    paquet et résultat globaux.
 *
 *  Fonctionnement :
 *    Recharge les bornes uniquement si p change ; le mot haut de zmin est
 *    biaisé par XOR $8000 pour permettre une comparaison signée à partir de
 *    CMP.
 *
 * ----------------------------------------------------------------------------
 */
static int GeoV3_FindCandidate(FaceArrays3D *faces, int p, int start)
{


    /* Actualiser uniquement les invariants qui dépendent de la face sélectionnée. */
    if (geo_scan_cached_p != p) {
        memcpy(&geo_scan_words[12], &faces->z_min[p], 4);
        geo_scan_words[13] ^= 0x8000U;
        geo_scan_words[14] = (unsigned int)faces->minx[p];
        geo_scan_words[15] = (unsigned int)faces->maxx[p];
        geo_scan_words[16] = (unsigned int)faces->miny[p];
        geo_scan_words[17] = (unsigned int)faces->maxy[p];
        geo_scan_cached_p = p;
    }
    /* Transmettre la position de départ et exécuter le noyau synchrone. */
    geo_scan_words[18] = (unsigned int)start;
    GeoV3_ScanCandidates();
    return geo_scan_result;
}
