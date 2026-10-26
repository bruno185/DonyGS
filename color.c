/*
 * ============================================================================
 *  color.c — Sélecteur interactif de couleurs QuickDraw
 * ============================================================================
 *
 *  Rôle :
 *    Permettre de choisir la couleur de remplissage ou de contour dans les
 *    palettes logicielles de l'explorateur, avec les choix spéciaux
 *    aléatoire et contour identique au remplissage.
 *
 *  Organisation :
 *    Une grille 8x2 présente les seize indices de couleur ; une ligne
 *    séparée contient les choix spéciaux. Les fonctions de dessin calculent
 *    les rectangles, libellés et cadres ; colorChooser traite les flèches et
 *    la confirmation.
 *
 *  Dépendances et contraintes :
 *    Inclus par engine.c. Requiert un port QuickDraw actif, les palettes
 *    préparées et les fonctions clavier du projet. Le sélecteur modifie
 *    l'écran et la palette matérielle ; l'appelant redessine ensuite la
 *    scène. Les libellés visibles ne sont pas traduits ici.
 *
 * ============================================================================
 */
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
#define SPECIAL_GAP      70


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


/*
 * ----------------------------------------------------------------------------
 *  getSquareRect
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Calculer le rectangle en pixels d'une case du sélecteur.
 *
 *  Paramètres :
 *    [entrée] index, 0..15 ou choix spécial ; isFrameChooser, présence du
 *    choix même couleur que le remplissage.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    *r
 *
 *  Fonctionnement :
 *    Centre une grille 8x2 et une ligne séparée de une ou deux cases
 *    spéciales.
 *
 * ----------------------------------------------------------------------------
 */
static void getSquareRect(int index, int isFrameChooser, ChooserRect *r)
{
    int gridWidth = GRID_COLS * SQUARE_SIZE + (GRID_COLS - 1) * SQUARE_GAP;
    int gridOriginX = (SCREEN_WIDTH - gridWidth) / 2;

    /* Séparer la grille des couleurs et la ligne des choix spéciaux, toutes
     * deux centrées.
     */
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
    /* Fermer le rectangle avec une largeur et une hauteur de SQUARE_SIZE pixels. */
    r->x2 = r->x1 + SQUARE_SIZE;
    r->y2 = r->y1 + SQUARE_SIZE;
}


/*
 * ----------------------------------------------------------------------------
 *  drawSquare
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Dessiner le fond et le contour noir d'une case.
 *
 *  Paramètres :
 *    [entrée] index et isFrameChooser, transmis au calcul du rectangle.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    dessin QuickDraw et modification du motif de crayon.
 *
 *  Fonctionnement :
 *    Les choix spéciaux utilisent un fond noir plutôt qu'un indice de
 *    couleur.
 *
 * ----------------------------------------------------------------------------
 */
static void drawSquare(int index, int isFrameChooser)
{
    ChooserRect r;
    Rect qdRect;

    getSquareRect(index, isFrameChooser, &r);
    /* Convertir les bornes en rectangle QuickDraw puis remplir et encadrer
     * la case.
     */
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


/*
 * ----------------------------------------------------------------------------
 *  drawSquareLabel
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Écrire le libellé d'une case sous son rectangle.
 *
 *  Paramètres :
 *    [entrée] index et isFrameChooser.
 *
 *  Retour :
 *    Aucun
 *
 *  Sorties et effets de bord :
 *    texte sur le port actif.
 *
 *  Fonctionnement :
 *    Les indices 0..15 sont numériques ; les choix spéciaux gardent leurs
 *    libellés de l'interface. Le centrage emploie CHAR_WIDTH, pas la mesure
 *    QuickDraw.
 *
 * ----------------------------------------------------------------------------
 */
static void drawSquareLabel(int index, int isFrameChooser)
{
    ChooserRect r;
    char buf[16];
    int textWidth;

    getSquareRect(index, isFrameChooser, &r);

    if (index < 16) {
        /* Choisir le libellé numérique ou spécial ; les chaînes visibles
         * restent inchangées.
         */
        sprintf(buf, "%d", index);
    } else if (index == COLOR_RANDOM) {
        strcpy(buf, "Random");
    } else {
        strcpy(buf, "Same as fill");
    }

    /* Centrer le libellé avec la largeur fixe utilisée par ce dessin. */
    textWidth = strlen(buf) * CHAR_WIDTH;
    MoveTo(r.x1 + (SQUARE_SIZE - textWidth) / 2, r.y2 + LABEL_GAP + LABEL_TEXT_H);
    printf("%s", buf);
}


/*
 * ----------------------------------------------------------------------------
 *  drawSelectionBorder
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Dessiner ou effacer le cadre de sélection d'une case.
 *
 *  Paramètres :
 *    [entrée] index, isFrameChooser ; erase, vrai pour effacer en noir.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    cadre de deux pixels puis taille de crayon remise à 1x1.
 *
 *  Fonctionnement :
 *    Le motif courant reste celui utilisé pour le cadre.
 *
 * ----------------------------------------------------------------------------
 */
static void drawSelectionBorder(int index, int isFrameChooser, int erase)
{
    ChooserRect r;
    Rect qdRect;

    getSquareRect(index, isFrameChooser, &r);
    /* Déborder de trois pixels autour de la case pour garder lisible sa couleur. */
    SetRect(&qdRect, r.x1 - 3, r.y1 - 3, r.x2 + 3, r.y2 + 3);

    SetSolidPenPat(erase ? UI_BLACK : UI_WHITE);
    SetPenSize(2, 2);
    FrameRect(&qdRect);
    SetPenSize(1, 1);
}


/*
 * ----------------------------------------------------------------------------
 *  drawPaletteNumber
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Afficher le type de couleur modifié et le numéro de palette.
 *
 *  Paramètres :
 *    [entrée] palette, numéro affiché ; isFrameChooser, contour ou
 *    remplissage.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    texte centré avec CStringWidth sur le port actif.
 *
 * ----------------------------------------------------------------------------
 */
static void drawPaletteNumber(int palette, int isFrameChooser)
{
    char buf[32];
    int textWidth;

    /* Mesurer le titre complet avant son placement horizontal. */
    sprintf(buf, "%s color - Palette: %2d  ", isFrameChooser ? "Frame" : "Fill", palette);
    textWidth = CStringWidth(buf);

    MoveTo((SCREEN_WIDTH - textWidth) / 2, 12);
    printf("%s", buf);
}


#define BLACK_MARKER_INSET 0


/*
 * ----------------------------------------------------------------------------
 *  drawBlackSquareMarker
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Rendre visible une case noire avec un cadre blanc permanent.
 *
 *  Paramètres :
 *    [entrée] index et isFrameChooser.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    modifie le motif de crayon et dessine un cadre sur le port actif.
 *
 * ----------------------------------------------------------------------------
 */
static void drawBlackSquareMarker(int index, int isFrameChooser)
{
    ChooserRect r;
    Rect qdRect;

    getSquareRect(index, isFrameChooser, &r);
    /* Encadrer les cases noires sans modifier leur remplissage. */
    SetRect(&qdRect, r.x1 + BLACK_MARKER_INSET, r.y1 + BLACK_MARKER_INSET,
                      r.x2 - BLACK_MARKER_INSET, r.y2 - BLACK_MARKER_INSET);

    SetSolidPenPat(UI_WHITE);
    FrameRect(&qdRect);
}


/*
 * ----------------------------------------------------------------------------
 *  drawInstructions
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Afficher les deux lignes de commandes du sélecteur.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    texte centré au bas du port graphique.
 *
 *  Fonctionnement :
 *    La largeur de chaque ligne est mesurée par CStringWidth.
 *
 * ----------------------------------------------------------------------------
 */
static void drawInstructions(void)
{
    char line1[] = "Up/Down: change palette";
    char line2[] = "Left/Right: change color | Any key: confirm  ";


    /* Mesurer puis placer séparément les deux lignes de commandes. */
    int w1 = CStringWidth(line1);
    int w2 = CStringWidth(line2);

    MoveTo((SCREEN_WIDTH - w1) / 2, SCREEN_HEIGHT - 18);
    printf("%s", line1);
    MoveTo((SCREEN_WIDTH - w2) / 2, SCREEN_HEIGHT - 8);
    printf("%s", line2);
}


/*
 * ----------------------------------------------------------------------------
 *  colorChooser
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Exécuter la sélection interactive de couleur jusqu'à confirmation.
 *
 *  Paramètres :
 *    [entrée] isFrameChooser, 0 remplissage ou 1 contour ; initialSelection,
 *    choix initial valide.
 *
 *    [entrée-sortie] *palette, numéro 0..15.
 *
 *  Retour :
 *    Indice 0..15, COLOR_RANDOM, ou COLOR_SAME_AS_FILL pour le contour.
 *
 *  Sorties et effets de bord :
 *    écran redessiné, palette matérielle et crayon modifiés.
 *
 *  Fonctionnement :
 *    Haut/bas changent de palette, gauche/droite parcourent les choix ;
 *    toute autre touche confirme.
 *
 *    La scène précédente doit être redessinée ou restaurée par l'appelant.
 *
 * ----------------------------------------------------------------------------
 */
int colorChooser(int isFrameChooser, int *palette, int initialSelection)
{
    int selection = initialSelection;
    int total     = isFrameChooser ? 18 : 17;
    int i, key, running = 1;

    /* Appliquer la palette initiale puis dessiner cases, repères noirs,
     * libellés et sélection.
     */
    applyPalette(*palette);


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

    /* Attendre les commandes ; les flèches modifient la sélection sans
     * quitter le sélecteur.
     */
    while (running) {
        key = getkeypress();

        switch (key) {

            /* Passer à la palette suivante et repeindre uniquement les seize
             * cases colorées.
             */
            case KEY_UP:

                *palette = (*palette + 1) % 16;
                 if (*palette > 15) *palette = 0;
                applyPalette(*palette);
                applyPalette(*palette);
                drawPaletteNumber(*palette, isFrameChooser);


                for (i = 0; i < 16; i++) {
                    drawSquare(i, isFrameChooser);
                }
                drawSelectionBorder(selection, isFrameChooser, 0);
                break;
            /* Passer à la palette précédente en corrigeant le modulo négatif. */
            case KEY_DOWN:

                *palette = (*palette - 1) % 16;
                if (*palette < 0) *palette = 15;
                applyPalette(*palette);
                drawPaletteNumber(*palette, isFrameChooser);
                for (i = 0; i < 16; i++) {
                    drawSquare(i, isFrameChooser);
                }
                drawSelectionBorder(selection, isFrameChooser, 0);
                break;

            /* Effacer le cadre précédent puis avancer cycliquement parmi tous les choix. */
            case KEY_RIGHT:

                drawSelectionBorder(selection, isFrameChooser, 1);
                selection = (selection + 1) % total;
                drawSelectionBorder(selection, isFrameChooser, 0);
                break;

            /* Reculer cycliquement dans la même liste de choix. */
            case KEY_LEFT:

                drawSelectionBorder(selection, isFrameChooser, 1);
                selection = (selection - 1 + total) % total;
                drawSelectionBorder(selection, isFrameChooser, 0);
                break;

            /* Confirmer la sélection courante pour toute autre touche. */
            default:

                running = 0;
                break;
        }
    }

    return selection;
}