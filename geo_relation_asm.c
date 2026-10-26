/*
 * ============================================================================
 *  geo_relation_asm.c — Classification assembleur des sommets par rapport à
 *  un plan
 * ============================================================================
 *
 *  Rôle :
 *    Déterminer si les sommets d'une face se trouvent tous du même côté
 *    qu'un observateur, tous du côté opposé, ou de côtés incompatibles par
 *    rapport à un plan de référence.
 *
 *  Organisation :
 *    Les wrappers construisent ou réutilisent un paquet de plan préparé. Le
 *    noyau évalue les produits signés 32x32 vers 64 bits, décale chaque
 *    terme séparément et compare la somme à la bande de tolérance. Le côté
 *    exact des sommets est mémorisé par plan.
 *
 *  Dépendances et contraintes :
 *    Inclus par engine.c après GeoV3_Mul32x32. Le noyau requiert des
 *    coefficients a/b/c représentables sur 32 bits et les bornes de d
 *    validées par ordering_fixed_plane. Le paquet et le multiplicateur sont
 *    partagés ; les accès globaux longs évitent une dépendance à DBR.
 *
 * ============================================================================
 */
static unsigned int geo_rel_words[40];
static int geo_rel_result;


/*
 * ----------------------------------------------------------------------------
 *  GeoRel_Classify32Kernel
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Classer une face candidate par rapport au plan et au côté de
 *    l'observateur.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Entrées et sorties implicites :
 *    [entrée] geo_rel_words
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    geo_rel_result, -1 avant, +1 après, 0 indéterminé.
 *
 *  Registres et indicateurs :
 *    Préserve P, A, X, Y et D ; accès globaux longs indépendants de DBR.
 *
 *  Organisation mémoire :
 *    Cadre D, 80 octets : indices à 1 ; xo/yo/zo à 5/9/13 ; reste/obs à
 *    17/19 ; all_same/all_opp à 21/23 ; acc à 25..31 ; epsilon à 33..39 ;
 *    -epsilon à 41..47 ; d à 49..55 ; a/b/c à 57..67 ; offsets à 69/71 ;
 *    cache far à 73 ; ID/slot à 77/79.
 *
 *  Préconditions :
 *    coefficients a/b/c signés 32 bits, |d| <= 2^62, epsilon >= 0, indices
 *    et cache de côtés valides. Décale chaque produit de 16 bits avant
 *    somme.
 *
 *    Cache partagé valide seulement pour le plan, la tolérance et la vue
 *    préparés.
 *
 * ----------------------------------------------------------------------------
 */
asm void GeoRel_Classify32Kernel(void)
{
    /* Sauvegarder P/A/X/Y/D, imposer les tailles 16 bits et réserver 80
     * octets en page directe sur la pile.
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
    sbc #80
    tcs
    tcd


    /* Copier seulement les entrées du paquet ; initialise ensuite offset et
     * hypothèses all_same/all_opp.
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

/* Parcourir les indices OBJ à base un ; terminer dès que tous les sommets
 * sont classés.
 */
geo_rel_vertex:
    lda 17
    bne geo_rel_have_vertex
    brl geo_rel_finish
/* Chercher le sommet dans le cache du plan : vérifier les 13 bits
 * d'identifiant avant d'interpréter le code de côté.
 */
geo_rel_have_vertex:
    ldy 71
    lda [1],y
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
/* Réutiliser le côté positif ou négatif ; un sommet dans la tolérance
 * n'élimine aucune hypothèse.
 */
geo_rel_cached_off_plane:
    cmp #0x4000
    bne geo_rel_cached_negative
    lda #1
    brl geo_rel_side
geo_rel_cached_negative:
    lda #0xFFFF
    brl geo_rel_side
/* Passer à un offset de coordonnées de quatre octets et initialiser
 * l'accumulateur 64 bits avec d.
 */
geo_rel_vertex_uncached:
    lda 77
    dec a
    asl a
    asl a
    sta 69
    lda 49
    sta 25
    lda 51
    sta 27
    lda 53
    sta 29
    lda 55
    sta 31


    lda 57
    ora 59
    bne geo_rel_x_multiply
    brl geo_rel_x_done
/* Ajouter (x * coefficient_x) >> 16. Le multiplicateur préserve D/X/Y ; le
 * produit est partagé dans geo_mul_p.
 */
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

    lda >geo_mul_p+6
    bpl geo_rel_x_positive
    lda #0xFFFF
    bra geo_rel_x_extend
geo_rel_x_positive:
    lda #0
/* Étendre le signe du produit décalé sur 64 bits ; propager C entre les
 * quatre additions sans le perdre lors de TXA.
 */
geo_rel_x_extend:
    tax
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
    txa
    adc 31
    sta 31
geo_rel_x_done:


    lda 61
    ora 63
    bne geo_rel_y_multiply
    brl geo_rel_y_done
/* Ajouter (y * coefficient_y) >> 16. Le multiplicateur préserve D/X/Y ; le
 * produit est partagé dans geo_mul_p.
 */
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

    lda >geo_mul_p+6
    bpl geo_rel_y_positive
    lda #0xFFFF
    bra geo_rel_y_extend
geo_rel_y_positive:
    lda #0
/* Étendre le signe du produit décalé sur 64 bits ; propager C entre les
 * quatre additions sans le perdre lors de TXA.
 */
geo_rel_y_extend:
    tax
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
    txa
    adc 31
    sta 31
geo_rel_y_done:


    lda 65
    ora 67
    bne geo_rel_z_multiply
    brl geo_rel_z_done
/* Ajouter (z * coefficient_z) >> 16. Le multiplicateur préserve D/X/Y ; le
 * produit est partagé dans geo_mul_p.
 */
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

    lda >geo_mul_p+6
    bpl geo_rel_z_positive
    lda #0xFFFF
    bra geo_rel_z_extend
geo_rel_z_positive:
    lda #0
/* Étendre le signe du produit décalé sur 64 bits ; propager C entre les
 * quatre additions sans le perdre lors de TXA.
 */
geo_rel_z_extend:
    tax
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
    txa
    adc 31
    sta 31
/* Comparer acc à +epsilon : biaiser uniquement le mot de poids fort par XOR
 * $8000, puis comparer les mots bas sans signe.
 */
geo_rel_z_done:


    lda 39
    eor #0x8000
    sta 69
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

/* Comparer acc à -epsilon par mots hauts signés biaisés, puis mots
 * inférieurs non signés ; éviter une soustraction débordante.
 */
geo_rel_check_negative:


    lda 47
    eor #0x8000
    sta 69
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

/* Mémoriser un sommet dans la bande de tolérance, sans départager les faces. */
geo_rel_on_plane:
    lda 77
    ora #0x2000
    ldy 79
    sta [73],y
    brl geo_rel_next
/* Mémoriser le côté positif et transmettre +1 au test par rapport à
 * l'observateur.
 */
geo_rel_positive:
    lda 77
    ora #0x4000
    ldy 79
    sta [73],y
    lda #1
    bra geo_rel_side
/* Mémoriser le côté négatif et transmettre -1 au test par rapport à
 * l'observateur.
 */
geo_rel_negative:
    lda 77
    ora #0x6000
    ldy 79
    sta [73],y
    lda #0xFFFF
/* Un sommet du côté observateur exclut all_opp ; un sommet opposé exclut
 * all_same.
 */
geo_rel_side:
    cmp 19
    bne geo_rel_opposite
    stz 23
    bra geo_rel_flags
geo_rel_opposite:
    stz 21
/* Si les deux hypothèses sont éliminées, rendre immédiatement une relation
 * indéterminée.
 */
geo_rel_flags:
    lda 21
    ora 23
    bne geo_rel_next
    lda #0
    bra geo_rel_store
/* Décrémenter le nombre de sommets restant à classer. */
geo_rel_next:
    dec 17
    brl geo_rel_vertex
/* Priorité à all_same, même pour une liste vide, conformément au chemin C. */
geo_rel_finish:
    lda 21
    beq geo_rel_not_same
    lda #0xFFFF
    bra geo_rel_store
geo_rel_not_same:
    lda 23
    beq geo_rel_store
    lda #1
/* Publier le résultat puis libérer les 80 octets et restaurer les registres
 * et P.
 */
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


/*
 * ----------------------------------------------------------------------------
 *  GeoRel_Classify32
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Construire le paquet complet et appeler le noyau de classement.
 *
 *  Paramètres :
 *    [entrée] model, face candidate ; a/b/c en 16.16 sur 32 bits ; d et
 *    epsilon sur 64 bits ; obs, côté de l'observateur ; vertex_sides, cache
 *    de 64 mots.
 *
 *  Retour :
 *    -1 avant, +1 après, 0 indéterminé selon le plan fourni.
 *
 *  Sorties et effets de bord :
 *    paquet global, cache de côtés et résultat ; invalide le paquet préparé.
 *
 *  Fonctionnement :
 *    L'appelant garantit les bornes arithmétiques requises par le noyau.
 *
 * ----------------------------------------------------------------------------
 */
static int GeoRel_Classify32(Model3D *model, int face,
                            Fixed32 a, Fixed32 b, Fixed32 c,
                            Fixed64 d, Fixed64 epsilon, int obs,
                            unsigned int *vertex_sides)
{
    int *indices = model->faces.vertex_indices_buffer
                 + model->faces.vertex_indices_ptr[face];
    Fixed64 negative_epsilon = -epsilon;
    /* Un appel brut remplace le paquet ; invalider le raccourci de
     * réutilisation du plan préparé.
     */
    order_asm_packet_plane=NULL;
    /* Marshaler les pointeurs far, coefficients et tolérances par copies de
     * mots natifs little-endian.
     */
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


/*
 * ----------------------------------------------------------------------------
 *  GeoRel_ClassifyPrepared
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Classer une face en réutilisant le paquet d'un plan préparé.
 *
 *  Paramètres :
 *    [entrée] model, face candidate et plane retourné par
 *    ordering_fixed_plane.
 *
 *  Retour :
 *    Relation -1/0/+1 du noyau.
 *
 *  Sorties et effets de bord :
 *    paquet et cache de côtés du plan.
 *
 *  Fonctionnement :
 *    Reconstruit le paquet complet au changement de plan ; sinon remplace
 *    uniquement la liste d'indices et sa longueur. Non réentrant : les
 *    globals sont partagés.
 *
 * ----------------------------------------------------------------------------
 */
static int GeoRel_ClassifyPrepared(Model3D *model,int face,OrderFixedPlane *plane)
{
    static int packet_face=-1;
    static Fixed64 packet_epsilon=0;
    int *indices,result;
    /* Reconstruire le paquet si le plan, la face de référence ou epsilon change. */
    if(order_asm_packet_plane!=plane || packet_face!=plane->face || packet_epsilon!=plane->epsilon){
        result=GeoRel_Classify32(model,face,plane->a32,plane->b32,plane->c32,
                                 plane->d,plane->epsilon,plane->observer,plane->vertex_sides);
        order_asm_packet_plane=plane;packet_face=plane->face;packet_epsilon=plane->epsilon;
        return result;
    }
    /* Pour le même plan, seuls le pointeur d'indices et le nombre de sommets
     * de la face candidate changent.
     */
    indices=model->faces.vertex_indices_buffer+model->faces.vertex_indices_ptr[face];
    memcpy(&geo_rel_words[0],&indices,4);
    geo_rel_words[8]=model->faces.vertex_count[face];
    GeoRel_Classify32Kernel();
    return geo_rel_result;
}
