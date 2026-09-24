/* ============================================================
 * Color chooser module
 * ------------------------------------------------------------
 * Interactive on-screen picker used to select the fill or frame
 * color for polygon rendering. Works in SHGR (Super Hi-Res)
 * graphics mode, on top of the current 16-color palette.
 *
 * Layout: an 8x2 grid of the 16 palette colors (indices 0-15),
 * plus a row of "special" squares below it: a Random color
 * square, and (frame chooser only) a "Same as fill" square.
 *
 * Controls:
 *   Up/Down    -> cycle through the 16 SHR palettes
 *   Left/Right -> move the selection cursor between squares
 *   Any other key -> confirm the current selection and return
 * ============================================================ */

/* ---- Chooser layout constants ---- */

#define SCREEN_WIDTH     320
#define SCREEN_HEIGHT    200

#define GRID_COLS        8    /* 8 columns x 2 rows = 16 palette color squares */
#define GRID_ROWS        2
#define SQUARE_SIZE      26   /* width/height of one color square, in pixels */
#define SQUARE_GAP       4    /* gap between adjacent squares within the grid */

#define LABEL_GAP        2    /* gap between the bottom of a square and its label */
#define LABEL_TEXT_H     8    /* approximate height of one line of graphics text */
#define ROW_PITCH        (SQUARE_SIZE + LABEL_GAP + LABEL_TEXT_H + SQUARE_GAP)
                              /* vertical distance from one grid row to the next,
                                 including room for the label under each square */

#define GRID_ORIGIN_Y    22   /* Y position of the top-left color square */
#define SPECIAL_GAP      70   /* horizontal gap between the "Random" and
                                 "Same as fill" squares, wide enough for their
                                 (longer) text labels not to overlap */

#define CHAR_WIDTH       8    /* fallback character width; superseded by
                                 CStringWidth() where used */

/* Colors reserved for the chooser's own UI (fixed across every SHR palette,
   so the UI stays visible and legible no matter which palette is active) */
#define UI_BLACK          0
#define UI_WHITE          15

/* Key codes for arrow keys, as returned by getkeypress() on this platform */
#define KEY_LEFT    0x08
#define KEY_RIGHT   0x15
#define KEY_UP      0x0B
#define KEY_DOWN    0x0A


/* Screen rectangle (in pixels) occupied by one chooser square */
typedef struct {
    int x1, y1, x2, y2;
} ChooserRect;

/* Computes the screen rectangle for a given square index.
 * index 0-15        : one of the 16 palette color squares, laid out in
 *                      an 8x2 grid centered horizontally on the screen.
 * index COLOR_RANDOM / COLOR_SAME_AS_FILL : one of the special squares on
 *                      the row below the grid, also centered horizontally
 *                      (1 square for the fill chooser, 2 for the frame
 *                      chooser).
 */
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

/* Fills a square with its color (index 0-15), or black for the special
 * squares (Random / Same as fill), and outlines every square in black. */
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

/* Draws the text label under a square: the color number (0-15), or
 * "Random" / "Same as fill" for the special squares. Only needs to be
 * called once per square at init time -- a palette change repaints the
 * square's fill color but never touches this label area. */
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

/* Draws (erase=0) or removes (erase=1) the selection cursor around a
 * square: a 2px border, white when shown, painted over in black to
 * erase it. Called every time the selection moves, once to clear the
 * previous square and once to highlight the new one. */
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

/* Draws the title line at the top of the screen: which color is being
 * edited (Fill/Frame) and the current palette number. Centered using
 * QuickDraw II's CStringWidth() for accurate text measurement. */
static void drawPaletteNumber(int palette, int isFrameChooser)
{
    char buf[32];
    int textWidth;

    sprintf(buf, "%s color - Palette: %2d  ", isFrameChooser ? "Frame" : "Fill", palette);
    textWidth = CStringWidth(buf);

    MoveTo((SCREEN_WIDTH - textWidth) / 2, 12);
    printf("%s", buf);
}


#define BLACK_MARKER_INSET 0   /* inset from the square's edges for the permanent
                                   white marker border on always-black squares */

/* Draws a fixed white border on a square that is always black (color 0,
 * Random, Same as fill), so it stays visible against the chooser's black
 * background. White (UI_WHITE) is guaranteed to stay fixed across every
 * SHR palette, so this marker is unaffected by palette changes. */
static void drawBlackSquareMarker(int index, int isFrameChooser)
{
    ChooserRect r;
    Rect qdRect;

    getSquareRect(index, isFrameChooser, &r);
    SetRect(&qdRect, r.x1 + BLACK_MARKER_INSET, r.y1 + BLACK_MARKER_INSET,
                      r.x2 - BLACK_MARKER_INSET, r.y2 - BLACK_MARKER_INSET);

    SetSolidPenPat(UI_WHITE);
    FrameRect(&qdRect);
}

/* Draws the two lines of on-screen instructions at the bottom of the
 * screen, centered using CStringWidth() for accurate text measurement. */
static void drawInstructions(void)
{
    char line1[] = "Up/Down: change palette";
    char line2[] = "Left/Right: change color | Any key: confirm  ";
    // int w1 = strlen(line1) * CHAR_WIDTH;
    // int w2 = strlen(line2) * CHAR_WIDTH;
    int w1 = CStringWidth(line1);
    int w2 = CStringWidth(line2);

    MoveTo((SCREEN_WIDTH - w1) / 2, SCREEN_HEIGHT - 18);
    printf("%s", line1);
    MoveTo((SCREEN_WIDTH - w2) / 2, SCREEN_HEIGHT - 8);
    printf("%s", line2);
}

/* colorChooser
 * ------------
 * Runs the interactive color picker until the user confirms a choice.
 *
 * isFrameChooser   : 0 = fill color chooser (17 squares: 0-15 + Random)
 *                    1 = frame color chooser (18 squares: 0-15 + Random
 *                        + Same as fill)
 * palette          : pointer to the caller's current palette index (0-15);
 *                    updated in place whenever the user changes palette
 *                    with Up/Down, so the caller's global palette state
 *                    stays in sync.
 * initialSelection : square initially selected (0-15, COLOR_RANDOM, or
 *                    COLOR_SAME_AS_FILL).
 *
 * Returns: 0-15 for a palette color, or COLOR_RANDOM / COLOR_SAME_AS_FILL.
 * Any key other than the arrow keys confirms the current selection and
 * exits the picker.
 */
int colorChooser(int isFrameChooser, int *palette, int initialSelection)
{
    int selection = initialSelection;
    int total     = isFrameChooser ? 18 : 17;
    int i, key, running = 1;

    applyPalette(*palette);

    /* Initial draw: title, all squares + black-square markers + labels,
       instructions, and the selection cursor on the starting square. */
    drawPaletteNumber(*palette, isFrameChooser);
    for (i = 0; i < total; i++) {
        drawSquare(i, isFrameChooser);
        if (i == 0 || i == COLOR_RANDOM || i == COLOR_SAME_AS_FILL) {
            drawBlackSquareMarker(i, isFrameChooser);
        }
        drawSquareLabel(i, isFrameChooser);
    }
    drawInstructions();
    drawSelectionBorder(selection, isFrameChooser, 0);

    while (running) {
        key = getkeypress();

        switch (key) {

            case KEY_UP:
                /* Cycle to the next palette (wraps from 15 back to 0) */
                *palette = (*palette + 1) % 16;
                 if (*palette > 15) *palette = 0;
                applyPalette(*palette);
                applyPalette(*palette);
                drawPaletteNumber(*palette, isFrameChooser);
                /* Only the 16 color squares need repainting -- the special
                   squares (Random / Same as fill) are always black. */
                for (i = 0; i < 16; i++) {
                    drawSquare(i, isFrameChooser);
                }
                drawSelectionBorder(selection, isFrameChooser, 0);
                break;
            case KEY_DOWN:
                /* Cycle to the previous palette (wraps from 0 back to 15) */
                *palette = (*palette - 1) % 16;
                if (*palette < 0) *palette = 15;
                applyPalette(*palette);
                drawPaletteNumber(*palette, isFrameChooser);
                for (i = 0; i < 16; i++) {
                    drawSquare(i, isFrameChooser);
                }
                drawSelectionBorder(selection, isFrameChooser, 0);
                break;

            case KEY_RIGHT:
                /* Move selection to the next square in the cycle */
                drawSelectionBorder(selection, isFrameChooser, 1);
                selection = (selection + 1) % total;
                drawSelectionBorder(selection, isFrameChooser, 0);
                break;

            case KEY_LEFT:
                /* Move selection to the previous square in the cycle */
                drawSelectionBorder(selection, isFrameChooser, 1);
                selection = (selection - 1 + total) % total;
                drawSelectionBorder(selection, isFrameChooser, 0);
                break;

            default:
                /* Any other key confirms the current selection */
                running = 0;
                break;
        }
    }

    return selection;
}