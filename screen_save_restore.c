/*
 * ============================================================================
 *  screen_save_restore.c — Sauvegarde et restauration de la zone SHR
 * ============================================================================
 *
 *  Rôle :
 *    Conserver une image de 32768 octets à partir de $E1:2000 pour revenir
 *    rapidement aux vues graphiques après une inspection textuelle, sans
 *    recalculer les polygones.
 *
 *  Organisation :
 *    L'initialisation réserve un handle verrouillé et en décompose
 *    l'adresse. Les fonctions saveScreen et restoreScreen patchent les
 *    banques d'une instruction MVN, puis transfèrent la zone en un bloc.
 *    graph_changed suit l'invalidation de la sauvegarde.
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
#define SCREEN_SRC_BANK     0xE1
#define SCREEN_SRC_OFFSET   0x2000

static Handle gSaveHandle    = NULL;
static Byte   gSaveBank;
static Word   gSaveOffset;
static Byte   gSavePatched    = 0;
static Byte   gRestorePatched = 0;


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
