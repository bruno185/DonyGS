/*
 * ============================================================================
 *  animation.c — Rotation précalculée et transitions compilées 65C816
 * ============================================================================
 *
 *  Rôle :
 *    Préparer un tour horizontal du modèle, puis jouer les images en boucle
 *    jusqu’à une frappe, sans recalcul géométrique pendant la lecture.
 *
 *  Organisation :
 *    Les extrêmes projetés calent un rectangle adapté aux proportions du modèle,
 *    avec une surface plafonnée et au plus 164 lignes pour préserver la VBL
 *    et réserver une consigne graphique en bas d'écran.
 *    Le rendu Z-buffer direct prépare les images pendant l’affichage texte.
 *    Les différences deviennent des LDA immédiats / STA longs, regroupés en
 *    petits blocs terminés par RTL et lancés au début d’une VBL.
 *
 *  Dépendances et contraintes :
 *    Inclus à la fin de engine.c. Les handles exécutables sont verrouillés,
 *    sans franchissement de banque ni mémoire spéciale. Aucun détournement
 *    de pile ni shadowing. Les images et le code sont temporaires ; la vue
 *    initiale est recalculée sans tri, puis ses drapeaux sont restaurés.
 *    Plusieurs blocs rendent une transition progressive, pas atomique.
 *
 * ============================================================================
 */
segment "ANIM";

#define ANIM_MAX_CAPTURE_PIXELS 28160UL
#define ANIM_MAX_HEIGHT        164
#define ANIM_LEFT              ((320 - animation_width) / 2)
#define ANIM_TOP               ((200 - animation_height) / 2)
#define ANIM_ROW_BYTES         (animation_width / 2)
#define ANIM_FRAME_BYTES       (animation_height * ANIM_ROW_BYTES)
#define ANIM_FRAME_WORDS       (ANIM_FRAME_BYTES / 2)
#define ANIM_MAX_FRAMES        60
#define ANIM_MIN_FRAMES        4
#define ANIM_WORDS_PER_BLOCK   96
#define ANIM_BLOCK_BYTES       (ANIM_WORDS_PER_BLOCK * 7 + 1)
#define ANIM_CODE_BYTES        (ANIM_FRAME_WORDS * 7UL + (ANIM_FRAME_WORDS + ANIM_WORDS_PER_BLOCK - 1) / ANIM_WORDS_PER_BLOCK)
#define ANIM_MEMORY_RESERVE    131072UL
#define ANIM_HOLD_VBLS         2
/* Délai initial par vue, réglable au clavier et indépendant du nombre d'images. */
#define ANIM_VIEW_TICKS        12UL
#define ANIM_MIN_VIEW_TICKS    1UL
#define ANIM_MAX_VIEW_TICKS    120UL
/* Huit blocs de 96 mots au plus par rafraîchissement, au lieu d'un seul. */
#define ANIM_BLOCKS_PER_VBL    8

typedef struct {
    Handle pixels, code;
    unsigned int blocks, bytes;
} AnimationFrame;

static AnimationFrame animation_frames[ANIM_MAX_FRAMES];
static AnimationFrame animation_initial;
static int animation_width = 168, animation_height = 164;
static int animation_stop = 0;
static unsigned long animation_code_address;
static unsigned long animation_view_ticks = ANIM_VIEW_TICKS;
static int animation_playing = 0;
static int animation_key = 0;

/*
 * ----------------------------------------------------------------------------
 *  animation_handle_key
 * ----------------------------------------------------------------------------
 *  Rôle : régler le délai entre vues ou reconnaître une demande d'arrêt.
 *  Paramètres : [entrée] key, code clavier sur sept bits pendant la lecture.
 *  Retour : 0 pour une flèche traitée, 1 pour toute autre touche (arrêt).
 *  Fonctionnement : droite/haut diminuent le délai d'un tick, gauche/bas
 *    l'augmentent ; saturer entre ANIM_MIN_VIEW_TICKS et ANIM_MAX_VIEW_TICKS.
 *    La modification prend effet pendant l'attente courante, sans recalcul 3D.
 * ----------------------------------------------------------------------------
 */
static int animation_handle_key(int key)
{
    switch (key) {
        case 21: /* Flèche droite. */
        case 11: /* Flèche haut. */
            if (animation_view_ticks > ANIM_MIN_VIEW_TICKS) --animation_view_ticks;
            return 0;
        case 8:  /* Flèche gauche. */
        case 10: /* Flèche bas. */
            if (animation_view_ticks < ANIM_MAX_VIEW_TICKS) ++animation_view_ticks;
            return 0;
        default:
            return 1;
    }
}

/*
 * ----------------------------------------------------------------------------
 *  animation_stop_requested
 * ----------------------------------------------------------------------------
 *  Rôle : lire le clavier sans attente et acquitter une demande d’arrêt.
 *  Paramètres : aucun. Retour : 1 sur frappe, 0 sinon.
 *  Fonctionnement : accès matériels en 8 bits ; P est préservé, A volatil.
 * ----------------------------------------------------------------------------
 */
static int animation_stop_requested(void)
{
    animation_stop = 0;
    asm {
        php
        sep #0x20
        lda >0x00C000
        and #0x80
        beq anim_key_done
        lda >0x00C010
        lda #1
        sta >animation_stop
    anim_key_done:
        /* Remettre aussi l'état de l'assembleur en 16 bits avant le code C.
         * PLP restaure P à l'exécution, mais ne renseigne pas l'assembleur ORCA.
         */
        rep #0x30
        plp
    }
    return animation_stop;
}

/*
 * ----------------------------------------------------------------------------
 *  animation_wait_vbl
 * ----------------------------------------------------------------------------
 *  Rôle : attendre une nouvelle VBL en surveillant une demande d’arrêt.
 *  Paramètres : aucun. Retour : 1 si arrêt, 0 au début de la VBL suivante.
 *  Fonctionnement : en lecture, traiter les flèches puis reprendre l'attente ;
 *    toute autre touche arrête. En préparation, toute frappe annule.
 *    Sur GS, $C019 bit 7 vaut 1 à partir de la ligne 192.
 *    La zone animée se termine au plus à 181, avant le texte et ce signal. Attendre
 *    la fin d’une VBL déjà en cours ; ne pas compter plusieurs fois la même.
 *    P préservé, A volatil ; les interruptions restent autorisées en attente.
 * ----------------------------------------------------------------------------
 */
static int animation_wait_vbl(void)
{
    for (;;) {
    animation_stop = 0;
    animation_key = 0;
    asm {
        php
        sep #0x20
    anim_vbl_out:
        lda >0x00C000
        and #0x80
        bne anim_vbl_abort
        lda >0xE0C019
        bmi anim_vbl_out
    anim_vbl_in:
        lda >0x00C000
        and #0x80
        bne anim_vbl_abort
        lda >0xE0C019
        bpl anim_vbl_in
        bra anim_vbl_done
    anim_vbl_abort:
        lda >0x00C000
        and #0x7F
        sta >animation_key
        lda >0x00C010
        lda #1
        sta >animation_stop
    anim_vbl_done:
        /* ORCA doit encoder l'épilogue C en 16 bits, comme le P restauré. */
        rep #0x30
        plp
    }
    if (!animation_stop) return 0;
    if (!animation_playing || animation_handle_key(animation_key)) return 1;
    }
}

/*
 * ----------------------------------------------------------------------------
 *  animation_execute_block
 * ----------------------------------------------------------------------------
 *  Rôle : exécuter un bloc de transition compilé dans un handle verrouillé.
 *  Paramètres : [entrée] address, adresse far du début d’un bloc valide.
 *  Retour : aucun ; écrit uniquement les mots vidéo encodés dans le bloc.
 *  Fonctionnement : préparer un JSL auto-modifiant, imposer A/X/Y 16 bits,
 *    exécuter au plus ANIM_WORDS_PER_BLOCK écritures, puis restaurer le contexte.
 *    P, D et DBR préservés ; A/X/Y volatils. Interruptions masquées uniquement
 *    pendant ce court bloc. Le code généré termine par RTL et ne change pas S.
 * ----------------------------------------------------------------------------
 */
static void animation_execute_block(unsigned long address)
{
    animation_code_address = address;
    asm {
        php
        sei
        phb
        phd
        rep #0x30
        lda >animation_code_address
        sta >anim_code_call+1
        sep #0x20
        lda >animation_code_address+2
        sta >anim_code_call+3
        rep #0x30
    anim_code_call:
        jsl 0x000000
        pld
        plb
        plp
    }
}

/*
 * ----------------------------------------------------------------------------
 *  animation_compile_transition
 * ----------------------------------------------------------------------------
 *  Rôle : compiler les mots différents entre deux images centrales.
 *  Paramètres : [entrée] previous, ANIM_FRAME_BYTES octets, ou NULL pour un fond
 *    noir déjà effacé ; next, nouvelle image ; [entrée-sortie] frame, code réservé.
 *  Retour : aucun ; renseigne blocks et bytes dans frame.
 *  Fonctionnement : émettre A9 lo hi / 8F adresse24, soit LDA #mot / STA long.
 *    Conserver aussi les mots devenus noirs pour effacer l’ancienne silhouette.
 *    Comparer les mots 16 bits en un seul parcours, avec pointeurs et adresse
 *    vidéo incrémentaux. Chaque bloc plein occupe ANIM_BLOCK_BYTES ; zéro différence ne
 *    produit aucun bloc. Les réserves ANIM_CODE_BYTES couvrent le pire cas.
 * ----------------------------------------------------------------------------
 */
static void animation_compile_transition(const unsigned char *previous,
                                         const unsigned char *next,
                                         AnimationFrame *frame)
{
    unsigned char *code = (unsigned char *)*frame->code, *out = code;
    const UWORD16 *old = (const UWORD16 *)previous;
    const UWORD16 *pixels = (const UWORD16 *)next;
    unsigned int in_block = 0, blocks = 0;
    unsigned int address = 0x2000U + ANIM_TOP * 160U + ANIM_LEFT / 2;
    int y, x;
    /* La recherche préalable de bbox relisait toute l'image ; un seul passage
     * produit le même code, en ignorant toujours tous les mots inchangés.
     */
    for (y = 0; y < animation_height; ++y) {
        for (x = 0; x < ANIM_ROW_BYTES; x += 2, address += 2) {
            UWORD16 value = *pixels++;
            UWORD16 before = old ? *old++ : 0;
            if (value == before) continue;
            if (in_block == ANIM_WORDS_PER_BLOCK) {
                *out++ = 0x6B;
                ++blocks;
                in_block = 0;
            }
            *out++ = 0xA9;
            *out++ = (unsigned char)value;
            *out++ = (unsigned char)(value >> 8);
            *out++ = 0x8F;
            *out++ = (unsigned char)address;
            *out++ = (unsigned char)(address >> 8);
            *out++ = 0xE1;
            ++in_block;
        }
        address += 160 - ANIM_ROW_BYTES;
    }
    if (in_block) { *out++ = 0x6B; ++blocks; }
    frame->bytes = (unsigned int)(out - code);
    frame->blocks = blocks;
}

/*
 * ----------------------------------------------------------------------------
 *  animation_play_transition
 * ----------------------------------------------------------------------------
 *  Rôle : afficher une transition compilée et respecter la cadence minimale.
 *  Paramètres : [entrée] frame, code et nombre de blocs valides.
 *  Retour : 0 sur arrêt clavier, 1 après la transition.
 *  Fonctionnement : jusqu'à ANIM_BLOCKS_PER_VBL blocs par nouvelle VBL, puis
 *    attente pour atteindre ANIM_HOLD_VBLS. La borne limite les écritures à
 *    768 mots par rafraîchissement ; les grandes transitions restent progressives.
 * ----------------------------------------------------------------------------
 */
static int animation_play_transition(const AnimationFrame *frame)
{
    unsigned int i, j, block = 0;
    unsigned int count = (frame->blocks + ANIM_BLOCKS_PER_VBL - 1) / ANIM_BLOCKS_PER_VBL;
    if (count < ANIM_HOLD_VBLS) count = ANIM_HOLD_VBLS;
    for (i = 0; i < count; ++i) {
        if (animation_wait_vbl()) return 0;
        for (j = 0; j < ANIM_BLOCKS_PER_VBL && block < frame->blocks; ++j, ++block)
            animation_execute_block((unsigned long)*frame->code +
                                    (unsigned long)block * ANIM_BLOCK_BYTES);
    }
    return 1;
}

/*
 * ----------------------------------------------------------------------------
 *  animation_hold_view
 * ----------------------------------------------------------------------------
 *  Rôle : laisser la vue complète visible pendant sa durée minimale en ticks.
 *  Paramètres : aucun ; animation_view_ticks définit le délai courant par vue.
 *  Retour : 0 sur touche d'arrêt, 1 lorsque le délai est écoulé.
 *  Fonctionnement : mesurer les ticks réels, indépendamment de la fréquence CPU.
 *    Attendre les VBL en scrutant le clavier ; soustraction non signée pour
 *    accepter le rebouclage du compteur. Aucun temps de transfert n'est déduit
 *    de la durée pendant laquelle la vue terminée doit rester visible.
 * ----------------------------------------------------------------------------
 */
static int animation_hold_view(void)
{
    unsigned long start = (unsigned long)GetTick();
    while ((unsigned long)((unsigned long)GetTick() - start) < animation_view_ticks) {
        if (animation_wait_vbl()) return 0;
    }
    return 1;
}

/*
 * ----------------------------------------------------------------------------
 *  animation_free_frames
 * ----------------------------------------------------------------------------
 *  Rôle : libérer toutes les images et tous les blocs exécutables temporaires.
 *  Paramètres : aucun. Retour : aucun.
 *  Fonctionnement : accepte les réserves partielles et remet chaque entrée à zéro.
 * ----------------------------------------------------------------------------
 */
static void animation_free_frames(void)
{
    int i;
    for (i = 0; i < ANIM_MAX_FRAMES; ++i) {
        if (animation_frames[i].pixels) DisposeHandle(animation_frames[i].pixels);
        if (animation_frames[i].code) DisposeHandle(animation_frames[i].code);
    }
    if (animation_initial.code) DisposeHandle(animation_initial.code);
    memset(animation_frames, 0, sizeof(animation_frames));
    memset(&animation_initial, 0, sizeof(animation_initial));
}

/*
 * ----------------------------------------------------------------------------
 *  animation_reserve_frames
 * ----------------------------------------------------------------------------
 *  Rôle : réserver un nombre pair d’images adapté à la mémoire réellement allouable.
 *  Paramètres : aucun. Retour : nombre pair de 4 à 60, ou 0 sur réserve insuffisante.
 *  Fonctionnement : conserver ANIM_MEMORY_RESERVE octets libres, prévoir le code
 *    maximal de chaque transition et réduire le nombre prévu si un handle échoue.
 *    Les réserves partielles restent possédées jusqu’à animation_free_frames.
 * ----------------------------------------------------------------------------
 */
static int animation_reserve_frames(void)
{
    unsigned long available = FreeMem();
    /* Promouvoir dès le premier terme : pixels + code dépassent un int 16 bits. */
    unsigned long per_frame = (unsigned long)ANIM_FRAME_BYTES + ANIM_CODE_BYTES + 128UL;
    int count, i;
    if (available <= ANIM_MEMORY_RESERVE) return 0;
    available = (available - ANIM_MEMORY_RESERVE) / per_frame;
    count = (available > ANIM_MAX_FRAMES) ? ANIM_MAX_FRAMES : (int)available;
    count &= ~1;
    if (count < ANIM_MIN_FRAMES) return 0;
    /* Ne jamais exécuter un bloc traversant une banque ni situé en mémoire spéciale. */
    for (i = 0; i < count; ++i) {
        animation_frames[i].pixels = NewHandle(ANIM_FRAME_BYTES, userid(),
                                  attrLocked | attrNoCross | attrNoSpec, 0L);
        if (toolerror() || !animation_frames[i].pixels) break;
        animation_frames[i].code = NewHandle(ANIM_CODE_BYTES, userid(),
                                  attrLocked | attrNoCross | attrNoSpec, 0L);
        if (toolerror() || !animation_frames[i].code) break;
    }
    /* Libérer la dernière entrée incomplète et l’éventuelle image impaire. */
    count = i & ~1;
    for (; i >= count; --i) {
        if (i >= ANIM_MAX_FRAMES) continue;
        if (animation_frames[i].pixels) DisposeHandle(animation_frames[i].pixels);
        if (animation_frames[i].code) DisposeHandle(animation_frames[i].code);
        memset(&animation_frames[i], 0, sizeof(AnimationFrame));
    }
    return (count >= ANIM_MIN_FRAMES) ? count : 0;
}

/*
 * ----------------------------------------------------------------------------
 *  animation_select_frames
 * ----------------------------------------------------------------------------
 *  Rôle : choisir en mode texte le nombre pair de vues réellement réservable.
 *  Paramètres : [entrée] maximum, nombre pair de handles déjà alloués, au moins 4.
 *  Retour : nombre choisi entre 4 et maximum ; 0 pour annulation (0 ou Échap).
 *  Fonctionnement : proposer le maximum réellement réservé, accepter Entrée
 *    pour ce maximum, 0 pour annuler ; rejeter les nombres impairs et les
 *    saisies hors bornes ou contenant du texte résiduel. Effacer l'écran avant
 *    saisie ; séparer les lignes Text Tools par CR/LF. Lire getkeypress et
 *    afficher explicitement les caractères : ne pas dépendre de l'écho stdin
 *    pendant le contexte graphique. Retour arrière corrige, Échap annule.
 *    Ne possède aucun handle.
 * ----------------------------------------------------------------------------
 */
static int animation_select_frames(int maximum)
{
    char line[32], prompt[80], *end;
    long chosen;
    /* Effacer le texte et séparer les lignes par CR/LF, pas par CR seul. */
    WriteChar(12);
    WriteCString("Rotation animation\r\n\r\n");
    sprintf(prompt, "Available views: %d to %d (even).\r\n", ANIM_MIN_FRAMES, maximum);
    WriteCString(prompt);
    WriteCString("Enter accepts default; Esc/0 cancels.\r\n");
    for (;;) {
        int length = 0, overflow = 0;
        sprintf(prompt, "Number of views [%d]: ", maximum);
        WriteCString(prompt);
        /* L'entrée matérielle n'a pas d'écho implicite : utiliser le même
         * périphérique Text Tools que le prompt pour chaque caractère saisi.
         */
        for (;;) {
            int key = getkeypress();
            if (key == 27) { WriteCString("\r\n"); return 0; }
            if (key == 13 || key == 10) break;
            if (key == 8 || key == 127) {
                if (length > 0) {
                    --length;
                    WriteCString("\b \b");
                }
            } else if (key >= 32 && key <= 126) {
                if (length < sizeof(line) - 1) {
                    line[length++] = (char)key;
                    WriteChar(key);
                } else overflow = 1;
            }
        }
        line[length] = 0;
        WriteCString("\r\n");
        if (overflow) {
            WriteCString("Invalid number.\r\n");
            continue;
        }
        end = line;
        while (*end == ' ' || *end == '\t') ++end;
        if (*end == '\r' || *end == '\n' || !*end) return maximum;
        chosen = strtol(end, &end, 10);
        while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n') ++end;
        if (!*end && chosen == 0 && strchr(line, '0')) return 0;
        if (!*end && chosen >= ANIM_MIN_FRAMES && chosen <= maximum && !(chosen & 1))
            return (int)chosen;
        WriteCString("Enter an even number in range.\r\n");
    }
}

/*
 * ----------------------------------------------------------------------------
 *  animation_axis_extent
 * ----------------------------------------------------------------------------
 *  Rôle : borner exactement un axe projeté pendant une rotation horizontale.
 *  Paramètres : [entrée] radial2=x*x+y*y ; depth=distance-sin(V)*z ; a, b, c
 *    définissent le numérateur a*u+b*v+c ; g définit le dénominateur depth+g*u.
 *  Retour : maximum de la valeur absolue du quotient pour u*u+v*v=radial2,
 *    ou -1 si la trajectoire peut atteindre le plan de l'observateur.
 *  Fonctionnement : les tangences du quotient au cercle donnent une équation
 *    quadratique. Avec A=depth²-radial2*g², la plus grande valeur absolue vaut
 *    (|depth*c-radial2*a*g| + sqrt(radial2*((depth*a-c*g)²+b²*A)))/A.
 *    Aucun échantillonnage angulaire ni traitement complet du modèle.
 * ----------------------------------------------------------------------------
 */
static float animation_axis_extent(float radial2, float depth,
                                   float a, float b, float c, float g)
{
    float denominator = depth*depth - radial2*g*g;
    float center, tangent;
    if (denominator <= 0.0f) return -1.0f;
    center = depth*c - radial2*a*g;
    tangent = depth*a - c*g;
    return (fabsf(center) + sqrtf(radial2 *
            (tangent*tangent + b*b*denominator))) / denominator;
}

/*
 * ----------------------------------------------------------------------------
 *  animation_fit
 * ----------------------------------------------------------------------------
 *  Rôle : adapter cadrage et proportions de capture sans tronquer la rotation.
 *  Paramètres : [entrée] model ; [entrée-sortie] view, copie locale des paramètres.
 *  Retour : échelle 16.16, ou 0 pour géométrie nulle ou distance non représentable.
 *  Fonctionnement : le rayon maximal garantit une distance sûre. Pour chaque
 *    sommet, borner ensuite analytiquement x/z et y/z sur le tour complet avec
 *    les angles V/W courants. Parmi les rectangles de largeur multiple de huit,
 *    hauteur paire ≤164 et surface ≤28160 pixels, choisir la plus grande échelle
 *    uniforme avec deux pixels de marge. Ne pas déformer la géométrie objet.
 * ----------------------------------------------------------------------------
 */
static Fixed32 animation_fit(Model3D *model, ObserverParams *view)
{
    float radius2 = 0.0f, radius, distance, projection;
    float sv, cv, sw, cw, extent_x = 0.0f, extent_y = 0.0f;
    int i, width;
    for (i = 0; i < model->vertices.vertex_count; ++i) {
        float x = FIXED_TO_FLOAT(model->vertices.x[i]);
        float y = FIXED_TO_FLOAT(model->vertices.y[i]);
        float z = FIXED_TO_FLOAT(model->vertices.z[i]);
        float r2 = x*x + y*y + z*z;
        if (r2 > radius2) radius2 = r2;
    }
    radius = sqrtf(radius2);
    if (radius <= 0.0f || radius > 16000.0f) return 0;
    distance = FIXED_TO_FLOAT(view->distance);
    if (distance < 2.0f * radius + 1.0f) distance = 2.0f * radius + 1.0f;
    if (distance > 32760.0f) return 0;
    view->distance = FLOAT_TO_FIXED(distance);
    /* Dans le repère tournant u=cos(H)*x+sin(H)*y, v=-sin(H)*x+cos(H)*y,
     * le dénominateur commun est distance-sin(V)*z-cos(V)*u.
     */
    sv = FIXED_TO_FLOAT(sin_deg_int(view->angle_v));
    cv = FIXED_TO_FLOAT(cos_deg_int(view->angle_v));
    sw = FIXED_TO_FLOAT(sin_deg_int(view->angle_w));
    cw = FIXED_TO_FLOAT(cos_deg_int(view->angle_w));
    for (i = 0; i < model->vertices.vertex_count; ++i) {
        float x = FIXED_TO_FLOAT(model->vertices.x[i]);
        float y = FIXED_TO_FLOAT(model->vertices.y[i]);
        float z = FIXED_TO_FLOAT(model->vertices.z[i]);
        float radial2 = x*x + y*y;
        float depth = distance - sv*z;
        float ex = animation_axis_extent(radial2, depth, sw*sv, cw, -sw*cv*z, -cv);
        float ey = animation_axis_extent(radial2, depth, -cw*sv, sw, cw*cv*z, -cv);
        if (ex < 0.0f || ey < 0.0f) return 0;
        if (ex > extent_x) extent_x = ex;
        if (ey > extent_y) extent_y = ey;
    }
    if (extent_x <= 0.0f && extent_y <= 0.0f) return 0;
    /* Une capture carrée limite inutilement les modèles larges. Les 40 essais
     * sont scalaires : aucun sommet ni rendu supplémentaire n'est parcouru.
     */
    projection = 0.0f;
    for (width = 8; width <= 320; width += 8) {
        int height = (int)(ANIM_MAX_CAPTURE_PIXELS / (unsigned int)width) & ~1;
        float px, py, candidate;
        if (height > ANIM_MAX_HEIGHT) height = ANIM_MAX_HEIGHT;
        px = (extent_x > 0.0f) ? (float)(width/2 - 2) / extent_x : 10000.0f;
        py = (extent_y > 0.0f) ? (float)(height/2 - 2) / extent_y : 10000.0f;
        candidate = (px < py) ? px : py;
        if (candidate > projection) {
            projection = candidate;
            animation_width = width;
            animation_height = height;
        }
    }
    if (projection > 10000.0f) projection = 10000.0f;
    return FLOAT_TO_FIXED(projection);
}

/*
 * ----------------------------------------------------------------------------
 *  animation_render_frame
 * ----------------------------------------------------------------------------
 *  Rôle : préparer et rendre une orientation, sans classement automatique.
 *  Paramètres : [entrée-sortie] model ; [entrée] view, paramètres temporaires.
 *  Retour : 1 si les bornes et les réserves du moteur sont valides, 0 sinon.
 *  Fonctionnement : vérifier le format des arêtes et le rectangle choisi, puis
 *    employer le Z-buffer direct existant. L’appelant maintient framePolyOnly=1
 *    pour préserver la permutation du modèle pendant processModelFast.
 * ----------------------------------------------------------------------------
 */
static int animation_render_frame(Model3D *model, ObserverParams *view)
{
    int i, edges = 0;
    for (i = 0; i < model->faces.face_count; ++i) {
        long end = (long)model->faces.vertex_indices_ptr[i] + model->faces.vertex_count[i];
        if (model->faces.vertex_indices_ptr[i] < 0 || model->faces.vertex_count[i] < 0 ||
            end > 32767L) return 0;
        if (end > edges) edges = (int)end;
    }
    processModelFast(model, view, "");
    /* Refuser une capture tronquée plutôt que couper silencieusement le modèle. */
    for (i = 0; i < model->vertices.vertex_count; ++i) {
        if (model->vertices.zo[i] <= 0 || model->vertices.x2d[i] < ANIM_LEFT ||
            model->vertices.x2d[i] >= ANIM_LEFT + animation_width ||
            model->vertices.y2d[i] < ANIM_TOP ||
            model->vertices.y2d[i] >= ANIM_TOP + animation_height) return 0;
    }
    screenClearTopLines(200);
    renderModelFullscreenZBuffer(model);
    return zbfs_faces.capacity >= model->faces.face_count && zbfs_face_plane &&
           zbfs_vertices.capacity >= model->vertices.vertex_count &&
           zbfs_edge_capacity >= edges && zbfs_edge_buf;
}

/*
 * ----------------------------------------------------------------------------
 *  animate_model_rotation
 * ----------------------------------------------------------------------------
 *  Rôle : précalculer un tour complet puis jouer ses transitions jusqu’au clavier.
 *  Paramètres : [entrée-sortie] model ; [entrée] params, vue initiale inchangée.
 *  Retour : aucun ; revient en texte, prêt pour le redessin normal.
 *  Fonctionnement : sauvegarder visibilité, pan, projection et distance ; réduire
 *    temporairement le modèle, ouvrir QuickDraw sans afficher les calculs, réserver
 *    les images selon la mémoire disponible et compiler chaque transition, y compris
 *    le retour de la dernière à la première. Libérer toutes les réserves sur chaque
 *    sortie et recalculer la vue initiale sans toucher à l’ordre corrigé des faces.
 * ----------------------------------------------------------------------------
 */
static void animate_model_rotation(Model3D *model, const ObserverParams *params)
{
    InspectionViewState saved_view;
    ObserverParams view;
    Fixed32 saved_scale = s_global_proj_scale_fixed;
    Fixed32 saved_distance = current_observer_distance;
    Fixed32 scale;
    int saved_pan_x = pan_dx, saved_pan_y = pan_dy;
    int i, y, count = 0, cancelled = 0;
    const char *failure = NULL;
    DeviceRec text_output, graphics_output;
    char progress[80];
    unsigned char video_mode;
    animation_playing = 0;
    animation_view_ticks = ANIM_VIEW_TICKS;
    if (!model || !params || model->vertices.vertex_count <= 0 || model->faces.face_count <= 0)
        return;
    view = *params;
    scale = animation_fit(model, &view);
    if (!scale) {
        printf("Animation: model size or distance unsupported.\nPress any key.\n");
        keypress();
        return;
    }
    if (!inspection_view_begin(model, &saved_view)) {
        printf("Animation: unable to save view state.\nPress any key.\n");
        keypress();
        return;
    }
    /* Préparer les réserves du rendu avant de calculer le budget des images. */
    framePolyOnly = 1;
    pan_dx = pan_dy = 0;
    s_global_proj_scale_fixed = scale;
    current_observer_distance = view.distance;
    text_output = GetOutputDevice();
    startgraph(mode);
    graphics_output = GetOutputDevice();
    video_mode = *(volatile unsigned char *)0x00C029L;
    shgroff();
    /* startgraph redirige les sorties : rétablir explicitement le périphérique
     * texte et utiliser Text Tools pour rendre chaque progression immédiatement.
     */
    SetOutputDevice(text_output.deviceType, text_output.ptrOrSlot);
    WriteChar(12);
    sprintf(progress, "Preparing rotation, %d x %d pixels.\r\n", animation_width, animation_height);
    WriteCString(progress);
    if (!ZBuffer_Init() || !animation_render_frame(model, &view)) {
        failure = "Renderer allocation or bounds failed.";
        goto cleanup;
    }
    animation_initial.code = NewHandle(ANIM_CODE_BYTES, userid(),
                                      attrLocked | attrNoCross | attrNoSpec, 0L);
    if (toolerror() || !animation_initial.code) goto no_memory;
    count = animation_reserve_frames();
    if (!count) goto no_memory;
    /* Le maximum annoncé tient compte des allocations réussies et de la
     * fragmentation. Libérer immédiatement les vues refusées par l'utilisateur.
     */
    i = animation_select_frames(count);
    if (!i) { cancelled = 1; goto cleanup; }
    while (count > i) {
        --count;
        DisposeHandle(animation_frames[count].pixels);
        DisposeHandle(animation_frames[count].code);
        memset(&animation_frames[count], 0, sizeof(AnimationFrame));
    }
    WriteCString("Any key cancels preparation.\r\n");
    WriteCString("Playback: Right/Up speeds up.\r\nLeft/Down slows down; other keys stop.\r\n");
    sprintf(progress, "Precomputing %d frames.\r\nHold: %lu ticks per view.\r\n", count, ANIM_VIEW_TICKS);
    WriteCString(progress);
    for (i = 0; i < count; ++i) {
        unsigned char *pixels = (unsigned char *)*animation_frames[i].pixels;
        if (animation_stop_requested()) { cancelled = 1; goto cleanup; }
        sprintf(progress, "Rendering frame %d/%d...\r\n", i + 1, count);
        WriteCString(progress);
        view.angle_h = normalize_deg(params->angle_h + (int)(360L * i / count));
        /* La vue zéro est déjà rendue pour réserver les buffers avant le budget
         * mémoire ; le texte et la saisie n'ont pas modifié ses pixels SHGR.
         */
        if (i > 0 && !animation_render_frame(model, &view)) {
            failure = "Frame outside capture or insufficient memory.";
            goto cleanup;
        }
        /* Capturer uniquement le carré central, sans SCB ni palettes. */
        for (y = 0; y < animation_height; ++y)
            memcpy(pixels + y * ANIM_ROW_BYTES,
                   (void *)(0xE12000UL + (unsigned long)(ANIM_TOP + y) * 160UL + ANIM_LEFT/2),
                   ANIM_ROW_BYTES);
    }
    WriteCString("Compiling transitions...\r\n");
    for (i = 0; i < count; ++i) {
        int previous = (i == 0) ? count - 1 : i - 1;
        if (animation_stop_requested()) { cancelled = 1; goto cleanup; }
        sprintf(progress, "Transition %d/%d...\r\n", i + 1, count);
        WriteCString(progress);
        animation_compile_transition((unsigned char *)*animation_frames[previous].pixels,
                                     (unsigned char *)*animation_frames[i].pixels,
                                     &animation_frames[i]);
    }
    animation_compile_transition(NULL, (unsigned char *)*animation_frames[0].pixels,
                                 &animation_initial);
    /* Les pixels ne sont plus utilisés : ne conserver que le code de lecture. */
    for (i = 0; i < count; ++i) {
        DisposeHandle(animation_frames[i].pixels);
        animation_frames[i].pixels = NULL;
    }
    screenClearTopLines(200);
    /* Construire la première image tant que l'écran texte masque le SHGR. */
    if (!animation_play_transition(&animation_initial)) goto cleanup;
    SetOutputDevice(graphics_output.deviceType, graphics_output.ptrOrSlot);
    *(volatile unsigned char *)0x00C029L = video_mode;
    /* La capture s'arrête au plus à 181 : le texte reste hors des transferts. */
    SetSolidPenPat(COL_WHITE);
    MoveTo(3, 195);
    printf("Arrows: up/right +, down/left -");
    animation_playing = 1;
    /* La transition d’indice zéro relie la dernière image à la première. */
    i = 1;
    while (animation_hold_view() && animation_play_transition(&animation_frames[i])) {
        if (++i == count) i = 0;
    }
    goto cleanup;
no_memory:
    failure = "Insufficient memory for at least 4 views.";
cleanup:
    animation_playing = 0;
    /* Libérer avant la restauration pour rendre toute la mémoire au moteur. */
    animation_free_frames();
    s_global_proj_scale_fixed = saved_scale;
    current_observer_distance = saved_distance;
    pan_dx = saved_pan_x;
    pan_dy = saved_pan_y;
    framePolyOnly = 1;
    processModelFast(model, (ObserverParams *)params, "");
    inspection_view_end(model, &saved_view);
    /* endgraph attend le périphérique graphique qu'il a installé. */
    SetOutputDevice(graphics_output.deviceType, graphics_output.ptrOrSlot);
    endgraph();
    DoText();
    graph_changed = 1;
    if (cancelled) printf("Animation preparation cancelled.\n");
    /* Laisser lire l'erreur après fermeture du contexte graphique. */
    if (failure) {
        printf("Animation: %s\nPress any key.\n", failure);
        keypress();
    }
}

segment "data";
