/* ---- Constantes du chooser couleur ---- */

#define SCREEN_WIDTH     320
#define SCREEN_HEIGHT    200

#define GRID_COLS        8
#define GRID_ROWS        2
#define SQUARE_SIZE      26
#define SQUARE_GAP       4

#define LABEL_GAP        2
#define LABEL_TEXT_H     8
#define ROW_PITCH        (SQUARE_SIZE + LABEL_GAP + LABEL_TEXT_H + SQUARE_GAP)

#define GRID_ORIGIN_Y    22
#define SPECIAL_GAP      40

#define CHAR_WIDTH       8

#define UI_BLACK          0
#define UI_WHITE          15

#define KEY_LEFT    0x08
#define KEY_RIGHT   0x15
#define KEY_UP      0x0B
#define KEY_DOWN    0x0A


typedef struct {
    int x1, y1, x2, y2;
} ChooserRect;

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
    MoveTo(r.x1 + (SQUARE_SIZE - textWidth) / 2, r.y2 + LABEL_GAP + LABEL_TEXT_H);
    printf("%s", buf);
}

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

static void drawPaletteNumber(int palette)
{
    char buf[24];
    int textWidth;

    sprintf(buf, "Palette: %2d", palette);
    textWidth = strlen(buf) * CHAR_WIDTH;

    MoveTo((SCREEN_WIDTH - textWidth) / 2, 12);
    printf("%s", buf);
}

/* Instructions mises à jour : Up/Down = palette, Left/Right = color */
static void drawInstructions(void)
{
    char *line1 = "Up/Down: change palette";
    char *line2 = "Left/Right: change color   Any key: confirm";
    int w1 = strlen(line1) * CHAR_WIDTH;
    int w2 = strlen(line2) * CHAR_WIDTH;

#define LINE2_X_OFFSET 12   /* compense le débordement gauche du au texte plus large que l'écran */

    MoveTo((SCREEN_WIDTH - w1) / 2, SCREEN_HEIGHT - 28);
    printf("%s", line1);
    MoveTo((SCREEN_WIDTH - w2) / 2 + LINE2_X_OFFSET, SCREEN_HEIGHT - 16);
    printf("%s", line2);
}

int colorChooser(int isFrameChooser, int *palette, int initialSelection)
{
    int selection = initialSelection;
    int total     = isFrameChooser ? 18 : 17;
    int i, key, running = 1;

    applyPalette(*palette);

    drawPaletteNumber(*palette);
    for (i = 0; i < total; i++) {
        drawSquare(i, isFrameChooser);
        drawSquareLabel(i, isFrameChooser);
    }
    drawInstructions();
    drawSelectionBorder(selection, isFrameChooser, 0);

    while (running) {
        key = getkeypress();

        switch (key) {

            case 'P':
            case 'p':
                /* ne fait plus rien, désormais géré par Up/Down */
                break;

            case KEY_UP:
            case KEY_DOWN:
                *palette = (*palette + 1) % 16;
                applyPalette(*palette);
                drawPaletteNumber(*palette);
                for (i = 0; i < 16; i++) {
                    drawSquare(i, isFrameChooser);
                }
                drawSelectionBorder(selection, isFrameChooser, 0);
                break;

            case KEY_RIGHT:
                drawSelectionBorder(selection, isFrameChooser, 1);
                selection = (selection + 1) % total;
                drawSelectionBorder(selection, isFrameChooser, 0);
                break;

            case KEY_LEFT:
                drawSelectionBorder(selection, isFrameChooser, 1);
                selection = (selection - 1 + total) % total;
                drawSelectionBorder(selection, isFrameChooser, 0);
                break;

            default:
                running = 0;
                break;
        }
    }

    return selection;
}