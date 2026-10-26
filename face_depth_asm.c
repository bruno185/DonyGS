/*
 * ============================================================================
 *  face_depth_asm.c — Calcul assembleur des métriques des faces
 * ============================================================================
 *
 *  Rôle :
 *    Parcourir les sommets des faces pour calculer profondeurs, bornes
 *    projetées, sommes de coordonnées et normale de Newell, en conservant le
 *    comportement numérique du parcours C.
 *
 *  Organisation :
 *    Les pointeurs invariants sont préparés une fois ; chaque appel de face
 *    fournit sa liste d'indices. Le noyau ferme le cycle des sommets valides
 *    et exporte une structure de résultats. La division du centroïde, le
 *    terme d du plan et le culling final restent dans calculateFaceDepths.
 *
 *  Dépendances et contraintes :
 *    Inclus conditionnellement par engine.c et dépend de GeoV3_Mul32x32. La
 *    somme servant à z_mean reboucle sur 32 bits ; les sommes de centroïde
 *    et de Newell sont sur 64 bits. Le décalage des normales intervient
 *    après accumulation complète. Les paquets sont non réentrants.
 *
 * ============================================================================
 */
typedef struct {
    int valid_n,display_flag;
    Fixed32 z_min,z_max,sum;
    int minx,maxx,miny,maxy;
    Fixed64 sumx,sumy,sumz,a,b,c;
} FaceDepthScanResult;
static FaceDepthScanResult face_depth_result;
static unsigned int face_depth_inputs[13];


/*
 * ----------------------------------------------------------------------------
 *  FaceDepth_ScanKernel
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Parcourir une face pour calculer profondeurs, bornes, centroïde et
 *    normale.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Entrées et sorties implicites :
 *    [entrée] face_depth_inputs, pointeurs far puis longueur sur 16 bits.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    face_depth_result
 *
 *  Fonctionnement :
 *    Noyau et multiplicateur non réentrants.
 *
 *    Pointeurs ORCA/C sur quatre octets, accès globaux longs.
 *
 *    La somme de profondeur conserve son rebouclage 32 bits ; les sommes du
 *    centroïde sont 64 bits. Les normales ne sont décalées qu'après
 *    fermeture du contour.
 *
 *  Registres et indicateurs :
 *    Préserve P, A, X, Y et D.
 *
 *  Organisation mémoire :
 *    Cadre D de 148 octets : indices/xo/yo/zo/x2d/y2d à 1/5/9/13/17/21 ;
 *    longueur à 25 ; offset/valides/affichage/précédent à 27/29/31/33 ;
 *    zmin/zmax/somme32 à 35/39/43 ; bbox à 47..53 ; sommes xyz 64 bits à
 *    55/63/71 ; Newell brut à 79/87/95 ; xyz précédent/premier/courant à
 *    103..113/115..125/127..137 ; temporaires à 139..145.
 *
 * ----------------------------------------------------------------------------
 */
asm void FaceDepth_ScanKernel(void)
{
    /* Sauvegarder le contexte, désactiver le mode décimal et réserver les
     * 148 octets du cadre D.
     */
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

    /* Copier les 26 octets d'entrées ; initialiser séparément les compteurs,
     * bornes et sommes.
     */
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

    stz 35
    stz 39
    /* Les sentinelles de bornes valent +/-9999 ; conserver ces valeurs même
     * pour une géométrie inhabituelle.
     */
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
/* Lire le prochain indice 16 bits ; ignorer les valeurs dont indice-1 est
 * négatif, sans contrôle de borne supérieure.
 */
depth_scan_vertex:
    lda 25
    beq depth_scan_end_vertices
    bmi depth_scan_end_vertices
    ldy 27
    lda [1],y
    dec a
    bpl depth_scan_valid_index
    brl depth_scan_next
depth_scan_end_vertices:
    brl depth_scan_finish
/* Calculer les offsets de deux octets pour l'écran et quatre pour les
 * coordonnées observateur.
 */
depth_scan_valid_index:
    asl a
    sta 143
    asl a
    sta 139
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
    stz 31
/* Après exclusion des profondeurs négatives, comparer zmin avec mot haut
 * signé biaisé puis mot bas non signé.
 */
depth_scan_z_nonnegative:


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
/* Mettre à jour zmax avec la même comparaison multi-mot. */
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
/* Cumuler z pour la moyenne en 32 bits avec rebouclage, puis étendre les
 * bornes écran X/Y.
 */
depth_scan_zmax_done:


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
/* Accumuler les coordonnées des sommets pour le centroïde, avec extension du
 * signe sur 64 bits.
 */
depth_scan_ymax_done:

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
    txa
    adc 59
    sta 59
    txa
    adc 61
    sta 61

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
    txa
    adc 67
    sta 67
    txa
    adc 69
    sta 69

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
    txa
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
/* Ajouter l'arête depuis le sommet valide précédent aux sommes de Newell ;
 * mémoriser ensuite le sommet courant.
 */
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
/* Avancer dans la liste d'indices sans relier une entrée invalide à la
 * géométrie.
 */
depth_scan_next:
    inc 27
    inc 27
    dec 25
    brl depth_scan_vertex
/* Fermer le cycle avec le premier sommet valide uniquement si la face est
 * affichable et possède au moins trois sommets valides.
 */
depth_scan_finish:


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


    /* Décaler la somme de Newell a de 16 bits après accumulation complète et
     * étendre son signe.
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
    /* Décaler la somme de Newell b de 16 bits après accumulation complète et
     * étendre son signe.
     */
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
    /* Décaler la somme de Newell c de 16 bits après accumulation complète et
     * étendre son signe.
     */
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
/* Copier les résultats dans l'ordre des champs C alignés sur deux octets,
 * puis restaurer le contexte.
 */
depth_scan_export:

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


/* Sous-routine locale appelée par JSL : ajouter les trois produits de Newell
 * bruts sur 64 bits ; RTL revient dans le noyau.
 */
depth_scan_edge:

    /* a += (yp-yc) * (zp+zc) ; différences et sommes 32 bits, produit et
     * cumul 64 bits avec retenues.
     */
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

    /* b += (zp-zc) * (xp+xc) ; différences et sommes 32 bits, produit et
     * cumul 64 bits avec retenues.
     */
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

    /* c += (xp-xc) * (yp+yc) ; différences et sommes 32 bits, produit et
     * cumul 64 bits avec retenues.
     */
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


/*
 * ----------------------------------------------------------------------------
 *  FaceDepth_ScanSetup
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Préparer les pointeurs de coordonnées pour un parcours complet des
 *    faces.
 *
 *  Paramètres :
 *    [entrée] vtx, tableaux observateur et projetés déjà calculés.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    paquet global ; les pointeurs doivent rester stables jusqu'à la fin du
 *    parcours.
 *
 * ----------------------------------------------------------------------------
 */
static void FaceDepth_ScanSetup(VertexArrays3D *vtx)
{
    /* Installer une fois les cinq pointeurs de coordonnées invariants pour
     * le parcours des faces.
     */
    memcpy(&face_depth_inputs[2],&vtx->xo,4);
    memcpy(&face_depth_inputs[4],&vtx->yo,4);
    memcpy(&face_depth_inputs[6],&vtx->zo,4);
    memcpy(&face_depth_inputs[8],&vtx->x2d,4);
    memcpy(&face_depth_inputs[10],&vtx->y2d,4);
}

/*
 * ----------------------------------------------------------------------------
 *  FaceDepth_ScanFace
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Calculer les métriques d'une liste d'indices de face.
 *
 *  Paramètres :
 *    [entrée] faces ; offset dans vertex_indices_buffer ; n, nombre
 *    d'indices.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    face_depth_result ; utilise les coordonnées préparées par
 *    FaceDepth_ScanSetup.
 *
 *  Fonctionnement :
 *    L'appelant doit garantir la validité de la liste.
 *
 * ----------------------------------------------------------------------------
 */
static void FaceDepth_ScanFace(FaceArrays3D *faces,int offset,int n)
{
    /* Décrire la liste de la face puis exécuter le noyau ; le résultat reste
     * dans face_depth_result.
     */
    int *indices=faces->vertex_indices_buffer+offset;
    memcpy(face_depth_inputs,&indices,4);
    face_depth_inputs[12]=(unsigned int)n;
    FaceDepth_ScanKernel();
}


/*
 * ----------------------------------------------------------------------------
 *  FaceDepth_CentroidCoordinate
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Diviser une somme de coordonnées par le nombre de sommets valides.
 *
 *  Paramètres :
 *    [entrée] sum, somme 64 bits ; count, diviseur strictement positif.
 *
 *  Retour :
 *    Moyenne Fixed32 tronquée vers zéro.
 *
 *  Fonctionnement :
 *    Aucun effet de bord. Réduit à une division 32 bits si possible ; pour
 *    count=4, ajoute 3 aux sommes négatives avant le décalage de deux bits
 *    afin de conserver la troncature C.
 *
 * ----------------------------------------------------------------------------
 */
static Fixed32 FaceDepth_CentroidCoordinate(Fixed64 sum,int count)
{
    /* Prouver que la somme tient sur 32 bits avant de prendre la division
     * plus courte.
     */
    Fixed32 narrow=(Fixed32)sum;
    if(sum==(Fixed64)narrow){
        if(count==4)return (narrow+(narrow<0?3L:0L))>>2;
        return narrow/count;
    }
    /* Pour quatre sommets, corriger les négatifs avant le décalage afin de
     * tronquer vers zéro.
     */
    if(count==4)return (Fixed32)((sum+(sum<0?3:0))>>2);
    return (Fixed32)(sum/count);
}
