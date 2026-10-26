/*
 * ============================================================================
 *  asm.h — Clavier et commutation SHR — Primitives 65C816
 * ============================================================================
 *
 *  Rôle :
 *    Accéder directement au clavier Apple IIgs, à l'état Open-Apple et aux
 *    soft switches d'affichage SHR. Fournir aussi une signature de registres
 *    utilisable par un breakpoint d'émulateur.
 *
 *  Organisation :
 *    Les lectures clavier attendent activement le strobe, l'acquittent et
 *    rendent soit le masque, soit le caractère, soit un mot associant
 *    caractère et modificateur. Les routines d'affichage écrivent le
 *    registre $00C029.
 *
 *  Dépendances et contraintes :
 *    Définitions assembleur ORCA/C destinées à l'unité de compilation du
 *    projet. Les contrats de taille et de préservation diffèrent entre
 *    routines ; ne pas supposer que P ou tous les registres sont restaurés.
 *    Les accès matériels longs ne dépendent pas de DBR. Aucun timeout
 *    clavier.
 *
 * ============================================================================
 */
/*
 * ----------------------------------------------------------------------------
 *  keypress
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Attendre une frappe et acquitter le strobe clavier.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    A contient 0x0080 après le masquage, pas le caractère.
 *
 *    lecture de $00C000, écriture de $00C010 ; attente active sans timeout.
 *
 *  Préconditions :
 *    accumulateur 16 bits conformément aux constantes employées.
 *
 * ----------------------------------------------------------------------------
 */
asm int keypress ()
        {
loop:
        /* Attendre le bit 7 du registre clavier ; le masquage détruit ici le
         * code de touche.
         */
        lda >0xC000
        and #0x0080
        beq loop
        sta >0xC010
        rtl
        }

/*
 * ----------------------------------------------------------------------------
 *  getkeypress
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Attendre une frappe et retourner son code sur sept bits.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    Caractère dans A, sans le bit de strobe.
 *
 *  Sorties et effets de bord :
 *    acquitte $00C010 ; attente active.
 *
 *  Fonctionnement :
 *    Accumulateur attendu sur 16 bits.
 *
 * ----------------------------------------------------------------------------
 */
asm char getkeypress ()
        {
loop:
        lda >0xC000
        /* Tester le strobe sans détruire A, puis conserver le caractère
         * après acquittement.
         */
        bit #0x0080
        beq loop
        sta >0xC010
        and #0x007F
        rtl
        }

#define KBD       0xC000
#define KBDSTRB   0xC010
#define OASK      0xC061

/*
 * ----------------------------------------------------------------------------
 *  getkeypress_openA
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Lire une frappe en conservant l'état de la touche Open-Apple.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    Code clavier dans l'octet bas, Open-Apple dans le bit 8.
 *
 *  Sorties et effets de bord :
 *    strobe acquitté ; A est rendu sur 16 bits, P n'est pas restauré.
 *
 *  Algorithme :
 *    accès matériels en 8 bits, sauvegarde du caractère sur la pile,
 *    transfert du bit 7 de $00C061 vers B, puis reconstitution de B:A.
 *
 * ----------------------------------------------------------------------------
 */
asm int getkeypress_openA()
        {
        /* Lire les soft switches par octets pour ne pas accéder au registre voisin. */
        sep   #0x20
loop:
        lda   >KBD
        bit   #0x80
        beq   loop
        sta   >KBDSTRB
        and   #0x7F
        pha

        /* Convertir le bit Open-Apple en 0 ou 1 et le placer dans l'octet haut. */
        lda   >OASK
        asl   a
        lda   #0x00
        rol   a
        xba

        /* Assembler le caractère et le modificateur en repassant A sur 16 bits. */
        pla
        rep   #0x20
        rtl
        }

/*
 * ----------------------------------------------------------------------------
 *  shroff
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Désactiver l'affichage SHR en écrivant $41 dans $00C029.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    A et les indicateurs sont modifiés ; la routine termine avec A, X et Y
 *    configurés sur 16 bits.
 *
 * ----------------------------------------------------------------------------
 */
asm shroff ()
        {
        sep #0x20
        /* Écriture matérielle sur un octet ; REP impose ensuite les tailles de sortie. */
        lda #0x41
        sta >0xC029
        rep #0x30
        rtl
        }
