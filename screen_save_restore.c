/* ============================================================
   screen_save_restore.c
    Quick backup/restore of the entire SHR screen
   (32,768 bytes) via MVN, to avoid a full render of the
   polygons on keys that do not modify the graphic.
   ============================================================ */

#include <types.h>
#include <memory.h>

#define SCREEN_SIZE         32768   /* bitmap SHR : 200 lignes x 160 octets */
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
   Call this function ONLY ONCE, at the very beginning of the program.
   Allocates the save buffer (32,768 bytes, locks it in place
   so it doesn't move in memory) and records its actual bank and offset.
   Returns 1 on success, 0 on failure (insufficient memory).
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
   Saves the entire SHR screen to the buffer.
   Call this immediately after a full polygon render,
   then reset `graph_changed` to 0.
   ------------------------------------------------------------ */
void saveScreen(void)

{
    asm {
        php
        phb
        rep #0x10                 // X, Y in 16-bit (CPU registers)
        sep #0x20                 // A in 8-bit (CPU register), for patching a single byte

        lda gSaveBank
        sta save_mvn+1             // patch the "destination bank" byte (immediate)
        lda #0xE1
        sta save_mvn+2             // patch the "source bank" byte (immediate)

        rep #0x30                 // A back to 16-bit (CPU register)
        ldx #0x2000                // = SCREEN_SRC_OFFSET
        lda gSaveOffset            // value of the buffer (not its address: no #)
        tay                        // Y <- buffer value (avoids LDY var, which causes problems)
        // lda #0x7CFF                // = SCREEN_SIZE-1 (32000-1 bytes)
        lda #SCREEN_SIZE-1
    save_mvn:
        mvn 0x00,0x00             // 0xE1 = SCREEN_SRC_BANK // 0x00 patched above
        plb
        plp
    }
    graph_changed = 0;
}

/* ------------------------------------------------------------
        Restores the entire SHR screen from the buffer.
        Call this instead of a full render when graph_changed == 0
        (e.g., the spacebar, which does not affect the 3D graph).
   ------------------------------------------------------------ */
void restoreScreen(void)
{
    asm {
        php
        phb
        rep #0x10                 // X, Y in 16-bit (CPU registers)
        sep #0x20                 // A in 8-bit (CPU register), for patching a single byte
        lda gSaveBank
        sta restore_mvn+2          // patch the "source bank" byte (immediate) -- test reverse order
        lda #0xE1
        sta restore_mvn+1          // patch the "destination bank" byte (immediate) -- test reverse order

        rep #0x30                 // A back to 16-bit (CPU register)
        lda gSaveOffset            // value of the buffer (not its address: no #)
        tax                        // X <- buffer value (avoids LDX var, which causes problems)
        ldy #0x2000                // = SCREEN_SRC_OFFSET
        // lda #0x7CFF                // = SCREEN_SIZE-1 (32000-1 bytes)
        lda #SCREEN_SIZE-1
    restore_mvn:
        mvn 0x00,0xE1             // 0x00 patched above; 0xE1 = SCREEN_SRC_BANK

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
