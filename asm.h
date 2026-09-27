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


#define KBD       0xC000   /* Keyboard register (data + strobe) */
#define KBDSTRB   0xC010   /* Reset keyboard strobe */
#define OASK      0xC061   /* Open-Apple button state */

asm int getkeypress_openA()
        {
        sep   #0x20         // Switch A to 8 bits to access hardware registers
loop:
        lda   >KBD          // Read the keyboard register
        bit   #0x80         // Test the "key pressed" bit without destroying A
        beq   loop          // Loop until a key is pressed
        sta   >KBDSTRB      // Acknowledge the keyboard strobe
        and   #0x7F         // Keep only the character code (7 bits)
        pha                 // Save the character on the stack (8-bit push)

        lda   >OASK         // Read the Open-Apple state
        asl   a             // The pressed bit (bit 7) goes into the carry
        lda   #0x00
        rol   a             // A = 1 if Open-Apple is pressed, 0 otherwise
        xba                 // Move the flag into B (high byte of C)

        pla                 // Retrieve the character into A (low byte)
        rep   #0x20         // Switch A to 16 bits: C = B:A = (flag<<8) | char
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