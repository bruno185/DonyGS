/*
 * ============================================================================
 *  screen_save_restore.c — Sauvegarde, restauration et effacement SHR
 * ============================================================================
 *
 *  Rôle :
 *    Conserver une image de 32768 octets à partir de $E1:2000 pour revenir
 *    rapidement aux vues graphiques après une inspection textuelle, sans
 *    recalculer les polygones. Effacer aussi un nombre choisi de lignes
 *    depuis le haut de la zone de pixels.
 *
 *  Organisation :
 *    L'initialisation réserve un handle verrouillé et en décompose
 *    l'adresse. Les fonctions saveScreen et restoreScreen patchent les
 *    banques d'une instruction MVN, puis transfèrent la zone en un bloc.
 *    graph_changed suit l'invalidation de la sauvegarde. screenClearTopLines
 *    partage l'effacement du bandeau avant les rendus direct et scanline.
 *
 *  Dépendances et contraintes :
 *    Inclus par engine.c et dépend des outils mémoire Apple IIgs. Appeler
 *    l'initialisation une seule fois. Le transfert suppose une implantation
 *    compatible avec une banque et un offset 16 bits. Code auto-modifiant
 *    inscriptible et non réentrant ; P et DBR sont préservés, contrairement
 *    aux registres de transfert.
 *
 * ============================================================================
 */
#include <types.h>
#include <memory.h>

#define SCREEN_SIZE         32768

static Handle gSaveHandle    = NULL;
static Byte   gSaveBank;
static Word   gSaveOffset;

/* Invalidation de l'image sauvegardée : 1 impose un nouveau rendu ;
 * saveScreen remet ce drapeau à 0.
 */
int graph_changed = 1;

/*
 * ----------------------------------------------------------------------------
 *  initScreenSaveBuffer
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Allouer le buffer fixe utilisé par les transferts MVN de l'écran.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    1 si NewHandle réussit sans erreur Toolbox, 0 sinon.
 *
 *  Sorties et effets de bord :
 *    gSaveHandle, gSaveBank et gSaveOffset décrivent le bloc verrouillé.
 *
 *  Limites :
 *    les transferts suivants utilisent une banque unique et un offset 16
 *    bits ; ils supposent une implantation du bloc compatible avec ces
 *    transferts.
 *
 * ----------------------------------------------------------------------------
 */
int initScreenSaveBuffer(void)
{
    Pointer p;

    /* Verrouiller l'adresse du buffer pour garder valides les opérandes du
     * transfert.
     */
    gSaveHandle = NewHandle(SCREEN_SIZE, userid(), attrLocked, 0L);
    if (gSaveHandle == NULL || toolerror())
        return 0;

    /* Décomposer le pointeur far en banque et offset 16 bits. */
    p = *gSaveHandle;
    gSaveBank   = (Byte)(((long)p >> 16) & 0xFF);
    gSaveOffset = (Word)((long)p & 0xFFFF);

    return 1;
}

/*
 * ----------------------------------------------------------------------------
 *  saveScreen
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Copier la zone SHR vers le buffer préalablement alloué.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Entrées et sorties implicites :
 *    Entrée : banque et offset définis par initScreenSaveBuffer.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    buffer mis à jour et graph_changed remis à zéro.
 *
 *  Algorithme :
 *    patcher les octets de banque du MVN puis transférer 32768 octets. P et
 *    DBR sont restaurés ; A, X et Y sont utilisés. Code auto-modifiant, non
 *    réentrant et nécessitant un segment exécutable accessible en écriture.
 *
 * ----------------------------------------------------------------------------
 */
void saveScreen(void)

{
    asm {
        php
        phb
        rep #0x10
        sep #0x20

        lda gSaveBank
        /* Patcher la banque destination à +1 et la banque source à +2 du MVN. */
        sta save_mvn+1
        lda #0xE1
        sta save_mvn+2

        rep #0x30
        /* X est l'offset source, Y l'offset destination et A le nombre
         * d'octets moins un.
         */
        ldx #0x2000
        /* Préparer les offsets inverses et le compteur MVN sur 16 bits. */
        lda gSaveOffset
        tay

        lda #SCREEN_SIZE-1
    save_mvn:
        /* Transférer le bloc puis restaurer DBR, que MVN a remplacé par la
         * banque destination.
         */
        mvn 0x00,0x00
        plb
        plp
    }
    /* Marquer la sauvegarde comme représentative du dernier rendu. */
    graph_changed = 0;
}

/*
 * ----------------------------------------------------------------------------
 *  restoreScreen
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Restaurer la zone SHR depuis la dernière sauvegarde.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Entrées et sorties implicites :
 *    Entrée : buffer initialisé et contenant une image valide.
 *
 *  Retour :
 *    Aucun ; graph_changed n'est pas modifié.
 *
 *  Algorithme :
 *    MVN du buffer vers $E1:2000, avec banques patchées à l'exécution. P et
 *    DBR sont restaurés ; A, X et Y sont utilisés. Même contrainte de code
 *    auto-modifiant et de banque unique que saveScreen.
 *
 * ----------------------------------------------------------------------------
 */
void restoreScreen(void)
{
    asm {
        php
        phb
        rep #0x10
        sep #0x20
        lda gSaveBank
        /* Inverser les banques du transfert pour copier depuis le buffer vers l'écran. */
        sta restore_mvn+2
        lda #0xE1
        sta restore_mvn+1

        rep #0x30
        lda gSaveOffset
        tax
        ldy #0x2000

        lda #SCREEN_SIZE-1
    restore_mvn:
        /* Restaurer les 32768 octets, y compris la zone SHR située après les pixels. */
        mvn 0x00,0xE1

        plb
        plp
    }
}

/* Offset assembleur partagé ; la routine d'effacement n'est pas réentrante. */
static unsigned int screen_clear_last_offset;

/*
 * ----------------------------------------------------------------------------
 *  screenClearTopLines
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Effacer les premières lignes de pixels de l'écran SHR 320x200.
 *
 *  Paramètres :
 *    [entrée] line_count : nombre de lignes depuis le haut ; une valeur
 *    négative ou nulle ne fait rien, une valeur supérieure à 200 vaut 200.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    Met à zéro 160 octets par ligne depuis $E1:2000. Les tables SCB et
 *    palettes ne sont pas modifiées. Ne gère pas graph_changed : l'appelant
 *    conserve la responsabilité du cycle de rendu et de sauvegarde SHR.
 *
 *  Fonctionnement :
 *    Borne la hauteur puis calcule l'offset du dernier mot. Le noyau 16 bits
 *    descend de deux en deux jusqu'au mot d'offset zéro inclus. L'offset
 *    maximal, 31998, reste positif pour le test BPL. Les accès longs ne
 *    dépendent pas de DBR. L'offset statique rend la routine non réentrante.
 *
 *  Registres et indicateurs :
 *    Le bloc assembleur préserve P, A et X ; Y et DBR ne sont pas utilisés.
 *    Le code C de préparation reste soumis à la convention d'appel ORCA/C.
 *
 * ----------------------------------------------------------------------------
 */
static void screenClearTopLines(int line_count)
{
    /* Limiter l'écriture à la zone de pixels, sans toucher aux tables SHR. */
    if (line_count <= 0) return;
    if (line_count > 200) line_count = 200;
    screen_clear_last_offset = (unsigned int)(line_count * 160 - 2);
    asm {
        /* Sauvegarder le contexte et charger le dernier offset de mot. */
        php
        rep #0x30
        pha
        phx
        lda >screen_clear_last_offset
        tax
        lda #0
    screen_clear_top_loop:
        /* Écrire un mot puis reculer ; BPL inclut le mot d'offset zéro. */
        sta >0xE12000,x
        dex
        dex
        bpl screen_clear_top_loop
        /* Restaurer le contexte avant de rendre la main au code C. */
        plx
        pla
        plp
    }
}
