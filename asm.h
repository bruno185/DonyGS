// keypress() will get a keypress from the user
// This function will wait until a key is pressed and then return.
asm int keypress ()
        {
loop:
        lda >0xC000     // Read the keyboard status from memory address 0xC000
        and #0x0080     // Check if the key is pressed (bit 7)
        beq loop        // If not pressed, loop until a key is pressed
        sta >0xC010     // Clear the keypress by writing back to 0xC010
        rtl
        }

asm char getkeypress ()
        {
loop:
        lda >0xC000     // Read the keyboard status from memory address 0xC000
        bit #0x0080     // Test only the pressed flag without destroying A
        beq loop        // If not pressed, loop until a key is pressed
        sta >0xC010     // Clear the keypress by writing back to 0xC010
        and #0x007F     // Mask off the pressed flag, keep the character code
        rtl
        }




#define KBD       0xC000   /* Registre clavier (donnée + strobe) */
#define KBDSTRB   0xC010   /* Reset du strobe clavier */
#define OASK      0xC061   /* État du bouton Open-Apple */

asm int getkeypress_openA()
        {
        sep   #0x20         // Passe A en 8 bits pour taper le hardware
loop:
        lda   >KBD          // Lit le registre clavier
        bit   #0x80         // Teste le bit "touche pressée" sans détruire A
        beq   loop          // Boucle tant qu'aucune touche n'est pressée
        sta   >KBDSTRB      // Acquitte le strobe clavier
        and   #0x7F         // Ne garde que le code caractère (7 bits)
        pha                 // Sauve le caractère sur la pile (push 8 bits)

        lda   >OASK         // Lit l'état d'Open-Apple
        asl   a             // Le bit 7 (pressé) part dans le carry
        lda   #0x00
        rol   a             // A = 1 si Open-Apple est pressée, sinon 0
        xba                 // Place le flag dans B (octet haut de C)

        pla                 // Récupère le caractère dans A (octet bas)
        rep   #0x20         // Passe A en 16 bits : C = B:A = (flag<<8) | char
        rtl
        }


// To use with an emulator
// With Crossrunner, you can set a breakpoint to break when 
// register A has the value 0xAAAA and register X has the value 0xBBBB.
asm  debug ()
        {
        pha 
        phx
        lda #0xAAAA
        ldx #0xBBBB     // will break after this instruction if you set a breakpoint
        plx             // restore X
        pla             // restore A
        rtl             // return from subroutine
        }

asm shroff ()           // turn off superhires mode
        {
        sep #0x20
        lda #0x41
        sta >0xC029
        rep #0x30
        rtl
        }

asm shron ()            // turn on superhires mode
        {
        sep #0x20
        lda #0xC1
        sta >0xC029
        rep #0x30
        rtl
        }