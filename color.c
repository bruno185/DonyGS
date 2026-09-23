/* ---- Constantes du chooser couleur ---- */

#define SCREEN_WIDTH     320
#define SCREEN_HEIGHT    200

#define GRID_COLS        8
#define GRID_ROWS        2
#define SQUARE_SIZE      26
#define SQUARE_GAP       4

#define LABEL_GAP        2    /* espace entre bas du carré et le texte du numéro */
#define LABEL_TEXT_H     8    /* hauteur approx. d'une ligne de texte (police système) */
#define ROW_PITCH        (SQUARE_SIZE + LABEL_GAP + LABEL_TEXT_H + SQUARE_GAP)

#define GRID_ORIGIN_Y    22
#define SPECIAL_GAP      40   /* écart entre les cases "Random" et "Same as fill", plus large
                                  que SQUARE_GAP pour laisser de la place à leurs libellés */

#define CHAR_WIDTH       8    /* largeur d'un caractère, utilisée pour centrer le texte --
                                  ajuste si ta police graphique n'est pas de 8px de large */

/* NB: COLOR_RANDOM et COLOR_SAME_AS_FILL sont définis par toi (16 et 17) et servent
   maintenant à la fois de valeur de retour ET d'index de case dans la grille */

#define UI_BLACK          0
#define UI_WHITE          15

#define KEY_LEFT    0x08
#define KEY_RIGHT   0x15
#define KEY_UP      0x0B
#define KEY_DOWN    0x0A


typedef struct {
    int x1, y1, x2, y2;
} ChooserRect;

/* Calcule le rectangle écran d'une case (0-15 = grille, COLOR_RANDOM/COLOR_SAME_AS_FILL
   = ligne spéciale en dessous) */
static void getSquareRect(int index, int isFrameChooser, ChooserRect *r)
{
    int gridWidth = GRID_COLS * SQUARE_SIZE + (GRID_COLS - 1) * SQUARE_GAP;
    int gridOriginX = (SCREEN_WIDTH - gridWidth) / 2;

    if (index < 16) {
        int col = index % GRID_COLS;
        int row = index / GRID_COLS;
        r->x1 = gridOriginX + col * (SQUARE_SIZE + SQUARE_GAP);
        r->y1 = GRID_ORIGIN_Y + row * ROW_PITCH;
    } else {
        int specialCount   = isFrameChooser ? 2 : 1;
        int specialWidth   = specialCount * SQUARE_SIZE + (specialCount - 1) * SPECIAL_GAP;
        int specialOriginX = (SCREEN_WIDTH - specialWidth) / 2;
        int specialCol      = (index == COLOR_RANDOM) ? 0 : 1;

        r->x1 = specialOriginX + specialCol * (SQUARE_SIZE + SPECIAL_GAP);
        r->y1 = GRID_ORIGIN_Y + GRID_ROWS * ROW_PITCH + SQUARE_GAP;
    }
    r->x2 = r->x1 + SQUARE_SIZE;
    r->y2 = r->y1 + SQUARE_SIZE;
}

/* Dessine le carré rempli de sa couleur (ou noir pour random/same-as-fill) */
static void drawSquare(int index, int isFrameChooser)
{
    ChooserRect r;
    Rect qdRect;

    getSquareRect(index, isFrameChooser, &r);
    SetRect(&qdRect, r.x1, r.y1, r.x2, r.y2);

    if (index < 16) {
        SetSolidPenPat(index);
    } else {
        SetSolidPenPat(UI_BLACK);
    }
    PaintRect(&qdRect);
    SetSolidPenPat(UI_BLACK);
    FrameRect(&qdRect);
}

/* Ajout : écrit le numéro (0-15) ou le libellé (Random / Same as fill) sous la case.
   N'a besoin d'être appelé qu'une fois à l'initialisation : un changement de palette
   ne repeint que l'intérieur des carrés (drawSquare), jamais cette zone en dessous. */
static void drawSquareLabel(int index, int isFrameChooser)
{
    ChooserRect r;
    char buf[16];
    int textWidth;

    getSquareRect(index, isFrameChooser, &r);

    if (index < 16) {
        sprintf(buf, "%d", index);
    } else if (index == COLOR_RANDOM) {
        strcpy(buf, "Random");
    } else {
        strcpy(buf, "Same as fill");
    }

    textWidth = strlen(buf) * CHAR_WIDTH;
    /* centré sous le carré -- pour "Same as fill", le texte est plus large que le
       carré et débordera symétriquement de part et d'autre, ce qui est voulu ici */
    MoveTo(r.x1 + (SQUARE_SIZE - textWidth) / 2, r.y2 + LABEL_GAP + LABEL_TEXT_H);
    printf("%s", buf);
}

/* Bordure de sélection : blanche quand active, noire (= fond) quand effacée */
static void drawSelectionBorder(int index, int isFrameChooser, int erase)
{
    ChooserRect r;
    Rect qdRect;

    getSquareRect(index, isFrameChooser, &r);
    SetRect(&qdRect, r.x1 - 3, r.y1 - 3, r.x2 + 3, r.y2 + 3);

    SetSolidPenPat(erase ? UI_BLACK : UI_WHITE);
    SetPenSize(2, 2);
    FrameRect(&qdRect);
    SetPenSize(1, 1);
}

/* Titre "Palette: x" centré en haut de l'écran */
static void drawPaletteNumber(int palette)
{
    char buf[24];
    int textWidth;

    sprintf(buf, "Palette: %2d", palette);
    textWidth = strlen(buf) * CHAR_WIDTH;

    MoveTo((SCREEN_WIDTH - textWidth) / 2, 12);
    printf("%s", buf);
}

/* Ajout : instructions centrées en bas de l'écran, écrites une seule fois */
static void drawInstructions(void)
{
    char *line1 = "P key: palette   Arrow keys: color";
    char *line2 = "Any key: accept";
    int w1 = strlen(line1) * CHAR_WIDTH;
    int w2 = strlen(line2) * CHAR_WIDTH;

    MoveTo((SCREEN_WIDTH - w1) / 2, SCREEN_HEIGHT - 28);
    printf("%s", line1);
    MoveTo((SCREEN_WIDTH - w2) / 2, SCREEN_HEIGHT - 16);
    printf("%s", line2);
}

/*
 * colorChooser
 * isFrameChooser : 0 = chooser de fill (17 cases : 0-15 + Random)
 *                  1 = chooser de frame (18 cases : 0-15 + Random + Same as fill)
 * initialPalette    : palette SHGR de départ (0-15)
 * initialSelection  : case initialement sélectionnée (0-15, COLOR_RANDOM ou
 *                      COLOR_SAME_AS_FILL)
 *
 * Retour : 0-15 = index de couleur choisi, ou COLOR_RANDOM / COLOR_SAME_AS_FILL.
 * N'importe quelle touche autre que P/p et les flèches valide immédiatement
 * la sélection courante (plus de touche d'annulation distincte).
 */
int colorChooser(int isFrameChooser, int initialPalette, int initialSelection)
{
    int palette   = initialPalette;
    int selection = initialSelection;
    int total     = isFrameChooser ? 18 : 17;
    int i, key, running = 1;

    applyPalette(palette);

    drawPaletteNumber(palette);
    for (i = 0; i < total; i++) {
        drawSquare(i, isFrameChooser);
        drawSquareLabel(i, isFrameChooser);   /* numéro / libellé, une seule fois */
    }
    drawInstructions();
    drawSelectionBorder(selection, isFrameChooser, 0);

    while (running) {
        key = getkeypress();

        switch (key) {

            case 'P':
            case 'p':
                palette = (palette + 1) % 16;
                applyPalette(palette);
                drawPaletteNumber(palette);
                for (i = 0; i < 16; i++) {
                    drawSquare(i, isFrameChooser);   /* seules les couleurs changent */
                }
                drawSelectionBorder(selection, isFrameChooser, 0);
                break;

            case KEY_RIGHT:
            case KEY_DOWN:
                drawSelectionBorder(selection, isFrameChooser, 1);
                selection = (selection + 1) % total;
                drawSelectionBorder(selection, isFrameChooser, 0);
                break;

            case KEY_LEFT:
            case KEY_UP:
                drawSelectionBorder(selection, isFrameChooser, 1);
                selection = (selection - 1 + total) % total;
                drawSelectionBorder(selection, isFrameChooser, 0);
                break;

            default:
                running = 0;   /* n'importe quelle autre touche valide */
                break;
        }
    }

    return selection;
}