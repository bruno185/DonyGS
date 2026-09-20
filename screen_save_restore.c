/* ============================================================
   screen_save_restore.c
   Sauvegarde / restauration rapide de l'ecran SHR complet
   (32000 octets) via MVN, pour eviter un rendu complet des
   polygones sur les touches qui ne modifient pas le graphique.

   Les deux operandes de banque de MVN sont des valeurs
   IMMEDIATES figees a l'assemblage : impossible de les charger
   depuis une variable au moment de l'execution. Solution
   retenue : code auto-modifiant. L'adresse du buffer (obtenue
   dynamiquement via NewHandle, adresse libre choisie par l'OS)
   est inconnue a la compilation, donc on patche une seule fois,
   au premier appel de chaque fonction, l'octet de banque
   concerne directement dans l'instruction MVN compilee.

   Seul l'octet de banque qu'on ne connait pas a l'avance doit
   etre patche :
     - saveScreen()    : banque destination (= banque du buffer)
     - restoreScreen() : banque source      (= banque du buffer)
   L'autre octet de banque (celle de l'ecran SHR, $E1, fixe et
   connue a la compilation) reste tel quel dans le code assemble.
   ============================================================ */

#include <types.h>
#include <memory.h>

#define SCREEN_SIZE         32512L   /* bitmap SHR : 200 lignes x 160 octets */
#define SCREEN_SRC_BANK     0xE1
#define SCREEN_SRC_OFFSET   0x2000   /* debut du bitmap SHR dans la banque $E1 */

static Handle gSaveHandle    = NULL;
static Byte   gSaveBank;             /* banque du buffer, connue seulement a l'execution */
static Word   gSaveOffset;           /* offset du buffer, connu seulement a l'execution */
static Byte   gSavePatched    = 0;   /* 1 des que saveScreen() a patche son MVN */
static Byte   gRestorePatched = 0;   /* 1 des que restoreScreen() a patche son MVN */

/* Flag d'invalidation : mis a 1 des qu'un rendu complet devient
   necessaire (rotation, zoom, changement de modele, couleurs...),
   remis a 0 juste apres l'appel a saveScreen(). */
int graph_changed = 1;

/* ------------------------------------------------------------
   A appeler UNE SEULE FOIS, au tout debut du programme.
   Alloue le buffer de sauvegarde (32000 octets, verrouille pour
   qu'il ne bouge pas en memoire) et note sa banque/offset reels.
   Renvoie 1 en cas de succes, 0 en cas d'echec (memoire insuffisante).
   ------------------------------------------------------------ */
int initScreenSaveBuffer(void)
{
    Pointer p;

    gSaveHandle = NewHandle(SCREEN_SIZE, userid(), attrLocked, 0L);
    if (gSaveHandle == NULL || toolerror())
        return 0;

    p = *gSaveHandle;                          /* pointeur far vers le bloc verrouille */
    gSaveBank   = (Byte)(((long)p >> 16) & 0xFF);
    gSaveOffset = (Word)((long)p & 0xFFFF);

    return 1;
}

/* ------------------------------------------------------------
   Sauvegarde l'ecran SHR complet dans le buffer.
   A appeler juste apres un rendu complet des polygones,
   puis remettre graph_changed a 0.
   ------------------------------------------------------------ */
void saveScreen(void)

{
    asm {
        php
        phb
        rep #0x10                 // X, Y en 16 bits (registre CPU)
        sep #0x20                 // A en 8 bits (registre CPU), pour le patch d'un seul octet

        lda gSaveBank
        sta save_mvn+1             // patch l'octet "banque destination" (immediat) 
        lda #0xE1
        sta save_mvn+2             // patch l'octet "banque source" (immediat) 

        rep #0x30                 // A repasse en 16 bits (registre CPU)
        ldx #0x2000                // = SCREEN_SRC_OFFSET
        lda gSaveOffset            // valeur du buffer (pas son adresse : pas de #)
        tay                        // Y <- valeur du buffer (contourne LDY var, qui pose probleme)
        // lda #0x7CFF                // = SCREEN_SIZE-1 (32000-1 octets)
        lda #SCREEN_SIZE-1
    save_mvn:
        mvn 0x00,0x00             // 0xE1 = SCREEN_SRC_BANK // 0x00 patche ci-dessus
        plb
        plp
    }
    graph_changed = 0;
}

/* ------------------------------------------------------------
   Restaure l'ecran SHR complet depuis le buffer.
   A appeler a la place d'un rendu complet quand graph_changed == 0
   (ex: touche espace qui n'affecte pas le graphique 3D).
   ------------------------------------------------------------ */
void restoreScreen(void)
{

    asm {
        php
        phb
        rep #0x10                 // X, Y en 16 bits (registre CPU)
        sep #0x20                 // A en 8 bits (registre CPU), pour le patch d'un seul octet
        lda gSaveBank
        sta restore_mvn+2          // patch l'octet "banque source" (immediat) -- test ordre inverse
        lda #0xE1
        sta restore_mvn+1          // patch l'octet "banque destination" (immediat) -- test ordre inverse

        rep #0x30                 // A repasse en 16 bits (registre CPU)
        lda gSaveOffset            // valeur du buffer (pas son adresse : pas de #)
        tax                        // X <- valeur du buffer (contourne LDX var, qui pose probleme)
        ldy #0x2000                // = SCREEN_SRC_OFFSET
        // lda #0x7CFF                // = SCREEN_SIZE-1 (32000-1 octets)
        lda #SCREEN_SIZE-1
    restore_mvn:
        mvn 0x00,0xE1             // 0x00 patche ci-dessus ; 0xE1 = SCREEN_SRC_BANK

        plb
        plp
    }
}

// /* ------------------------------------------------------------
//    Debug helpers (C versions) : to diagnose save/restore issues.
//    - saveScreen_c(): pure-C copy from SHR to buffer
//    - restoreScreen_c(): pure-C copy from buffer to SHR
//    - verifySavedData(): compares buffer vs SHR and prints first mismatch
//    These are for debugging only; keep them available to call from tests.
//    ------------------------------------------------------------ */
// void saveScreen_c(void)
// {
//     if (gSaveHandle == NULL) return;
//     volatile Pointer dest = *gSaveHandle;
//     volatile Pointer src = (Pointer)(((long)SCREEN_SRC_BANK << 16) | (long)SCREEN_SRC_OFFSET);
//     long i;
//     for (i = 0; i < SCREEN_SIZE; ++i) dest[i] = src[i];
//     graph_changed = 0;
// }

// void restoreScreen_c(void)
// {
//     if (gSaveHandle == NULL) return;
//     volatile Pointer src = *gSaveHandle;
//     volatile Pointer dest = (Pointer)(((long)SCREEN_SRC_BANK << 16) | (long)SCREEN_SRC_OFFSET);
//     long i;
//     for (i = 0; i < SCREEN_SIZE; ++i) dest[i] = src[i];
// }

// int verifySavedData(void)
// {
//     if (gSaveHandle == NULL) return -1;
//     volatile Pointer src = (Pointer)(((long)SCREEN_SRC_BANK << 16) | (long)SCREEN_SRC_OFFSET);
//     volatile Pointer buf = *gSaveHandle;
//     long i;
//     for (i = 0; i < SCREEN_SIZE; ++i) {
//         if (src[i] != buf[i]) {
//             printf("verifySavedData: mismatch at %ld: screen=0x%02X buf=0x%02X\n", i, src[i] & 0xFF, buf[i] & 0xFF);
//             return (int)i;
//         }
//     }
//     printf("verifySavedData: OK, all %ld bytes match\n", SCREEN_SIZE);
//     keypress();
//     return 0;
// }
