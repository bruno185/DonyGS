/* zbuffer_fullscreen.c
 *
 * Implementation du Z-buffer plein ecran 16 bits, ET rendu
 * (renderModelFullscreenZBuffer, portage de
 * renderModelScanlineZBuffer_fast ET _biased, avec le meme
 * dispatcher selon cull_back_faces). Voir zbuffer_fullscreen.h pour le
 * codage de valeur retenu (distance inversee, 0xFFFF = vide) et
 * l'avertissement de precision.
 *
 * USAGE DANS engine.c :
 *   #include "zbuffer_fullscreen.c"
 * a placer APRES les definitions de Model3D, VertexArrays3D,
 * FaceArrays3D, ScanIntersection, MAX_SPAN_INTERSECTIONS,
 * SCREEN_WIDTH/SCREEN_HEIGHT, getFaceFillColor, getFaceFrameColor,
 * drawPixel, FIXED_TO_FLOAT, pan_dx/pan_dy, palette -- la partie
 * rendu de ce fichier (en bas) les utilise directement sans passer
 * par un header commun, donc elles doivent deja etre visibles au
 * moment ou le preprocesseur colle ce fichier.
 *
 * Appeler ZBuffer_Init() une fois au demarrage du programme (avant
 * le premier appel a renderModelFullscreenZBuffer) et
 * ZBuffer_Shutdown() en sortie.
 */

segment "ZBUF";
#include "zbuffer_fullscreen.h"

#ifdef ZBUF_DIAG_SCALE
#include <stdio.h>   /* fopen/fprintf pour le diagnostic de calibrage -- a retirer avec ZBUF_DIAG_SCALE */
#endif

FarWordPtr zbuf_row[ZBUF_HEIGHT];

static Handle  zbuf_handle = NULL;
static int     zbuf_bank_lo = 0;
static int     zbuf_bank_hi = 0;

/* ------------------------------------------------------------------
 * Allocation bank-alignee.
 *
 * On garde l'alignement sur une frontiere de bank -- non pas pour
 * empecher une ligne de la chevaucher (l'adressage indirect long
 * [ptr],y gere ca correctement, voir plus bas), mais pour que
 * ZBuffer_ClearFast puisse bêtement blaster deux banks entieres sans
 * avoir a calculer un decalage d'alignement au moment du clear.
 * ------------------------------------------------------------------ */

#define ZBUF_USABLE_SIZE   (2UL * 65536UL)               /* 2 banks utiles       */
#define ZBUF_ALLOC_SIZE    (ZBUF_USABLE_SIZE + 65536UL)  /* + marge d'alignement */

int ZBuffer_Init(void)
{
    ULONG32 raw;
    ULONG32 aligned;
    int     y;

    zbuf_handle = NewHandle(ZBUF_ALLOC_SIZE, userid(), attrFixed, 0L);
    if (toolerror() || zbuf_handle == NULL) {
        return 0;
    }

    raw     = (ULONG32) *zbuf_handle;
    aligned = (raw + 0xFFFFUL) & 0xFFFF0000UL;   /* arrondi a la frontiere de bank sup. */

    zbuf_bank_lo = (int)((aligned >> 16) & 0xFFUL);
    zbuf_bank_hi = zbuf_bank_lo + 1;

    /* Adressage lineaire simple : aucune ligne n'a besoin d'etre
     * traitee a part, meme celle qui chevauche la frontiere de bank
     * (voir ZBuffer_TestSetRow).
     *
     * ATTENTION debordement 16 bits (bug reel rencontre et corrige) :
     * (ULONG32)(y * ZBUF_ROW_BYTES) calcule d'abord "y * ZBUF_ROW_BYTES"
     * en arithmetique "int" (16 bits sur ORCA/C), qui deborde pour
     * y >= 52 (52*640 = 33280 > 32767) AVANT que le cast en ULONG32
     * n'intervienne -- trop tard, le debordement a deja eu lieu. Il
     * faut forcer le 32 bits DANS la multiplication elle-meme, pas
     * apres, en castant au moins un des deux operandes avant de
     * multiplier. */
    for (y = 0; y < ZBUF_HEIGHT; y++) {
        zbuf_row[y] = (FarWordPtr)(aligned + (ULONG32)y * (ULONG32)ZBUF_ROW_BYTES);
    }

    ZBuffer_Clear();
    return 1;
}

void ZBuffer_Shutdown(void)
{
    if (zbuf_handle != NULL) {
        DisposeHandle(zbuf_handle);
        zbuf_handle = NULL;
    }
}

void ZBuffer_Clear(void)
{
    int y, x;
    for (y = 0; y < ZBUF_HEIGHT; y++) {
        FarWordPtr row = zbuf_row[y];
        for (x = 0; x < ZBUF_WIDTH; x++) {
            row[x] = ZBUF_FAR_VALUE;
        }
    }
}

/* Echelle courante, recalculee chaque frame par
 * ZBuffer_SetScaleForFrame() -- voir zbuffer_fullscreen.h pour
 * pourquoi une valeur fixe ne marche pas des que le zoom change. */
static float zbuffer_scale = ZBUFFER_INV_Z_SCALE;

void ZBuffer_SetScaleForFrame(float max_inv_z)
{
    if (max_inv_z > 0.0000001f) {
        zbuffer_scale = ZBUFFER_TARGET_MAX_CODE / max_inv_z;
    }
    /* sinon : on garde l'echelle precedente plutot que de diviser par
     * zero ou d'ecraser avec une valeur aberrante pour une frame
     * degenerescente (aucun sommet valide). */
}

UWORD16 ZBuffer_QuantizeInvZ(float inv_z)
{
    float   scaled;
    long    q;

    if (inv_z < 0.0f) inv_z = 0.0f;   /* garde-fou, ne devrait pas arriver */

    scaled = inv_z * zbuffer_scale;
    q = (long)(scaled + 0.5f);
    if (q < 0)      q = 0;
    if (q > 0xFFFF) q = 0xFFFF;

    return (UWORD16)(0xFFFFUL - (ULONG32)q);
}

int ZBuffer_TestAndSet(int x, int y, UWORD16 z)
{
    FarWordPtr row = zbuf_row[y];
    if (z < row[x]) {
        row[x] = z;
        return 1;
    }
    return 0;
}

/* ====================================================================
 * Assembleur en ligne
 * ==================================================================== */

/* ZBuffer_ClearFast -- bank_lo/bank_hi obtenus directement depuis
 * ZBuffer_Init.
 *
 * ATTENTION largeur d'accumulateur : le changement de banque
 * (lda/pha/plb) doit se faire en 8 bits (PLB ne depile jamais qu'UN
 * octet, quelle que soit la largeur de A) -- une version precedente
 * de cette routine faisait le lda/pha en 16 bits (REP #0x30), ce qui
 * laissait un octet parasite sur la pile a chaque changement de
 * banque et finissait par decaler l'adresse de retour au RTL final :
 * corruption memoire arbitraire au retour de la fonction. Corrige
 * ci-dessous par un SEP #0x20 localise autour de chaque lda/pha/plb,
 * sur le meme principe que le SEP local deja utilise dans
 * getkeypress_openA (asm.h) pour un besoin similaire.
 */
asm void ZBuffer_ClearFast(int bank_lo, int bank_hi)
    {
        php

        phb                     // sauve le DBR appelant (1 octet)

        sep     #0x20           // 8 bits, pour que pha/plb restent equilibres (1 octet chacun)
        lda     bank_lo
        pha
        plb                     // DBR = premiere bank du Z-buffer

        rep     #0x30           // repasse en 16 bits pour la boucle de remplissage
        ldx     #0x0000
        lda     #0xFFFF
clear_bank_lo:
        sta     0x0000,x
        inx
        inx
        bne     clear_bank_lo   // X revient a 0 apres les 65536 octets

        sep     #0x20
        lda     bank_hi
        pha
        plb                     // DBR = seconde bank du Z-buffer

        rep     #0x30
        ldx     #0x0000
        lda     #0xFFFF
clear_bank_hi:
        sta     0x0000,x
        inx
        inx
        bne     clear_bank_hi

        plb                     // restaure le DBR appelant (1 octet, equilibre avec le phb du debut)
        plp
        rtl
    }

/* ZBuffer_TestSetRow
 *
 * NOTE D'ARCHITECTURE (correction par rapport a la version precedente) :
 * l'implementation initiale fixait le Data Bank Register une seule
 * fois (PLB) pour tout le span, puis adressait en absolu indexe. C'est
 * FAUX dans un cas rare mais reel : avec 200 lignes de 640 octets sur
 * 2 banks de 64 Ko, la ligne 102 chevauche exactement la frontiere de
 * banque en son milieu (a l'octet 65536, soit le pixel 128 de cette
 * ligne). Un DBR fixe pour tout le span donnerait une adresse fausse
 * pour les pixels situes apres la frontiere sur cette ligne precise.
 *
 * Version corrigee : adressage indirect long [ptr],y. Cette forme
 * d'adressage du 65816 fait une VRAIE addition 24 bits (report dans
 * l'octet de banque inclus) a chaque acces -- donc correcte quel que
 * soit l'endroit ou tombe la frontiere de banque, sans avoir besoin
 * de savoir a l'avance si le span la chevauche. Cout : le pointeur
 * far doit resider en direct page (zone $00 courante) ou etre un
 * pointeur normal ORCA/C (deja 24 bits, voir zbuffer_fullscreen.h),
 * ce que le
 * parametre zbuf_ptr est suppose satisfaire (ORCA/C place en general
 * les parametres de fonctions asm en zone accessible en direct page
 * -- A VERIFIER par compilation, comme deja signale dans le .h).
 */
asm void ZBuffer_TestSetRow(FarWordPtr zbuf_ptr, UWORD16 z_start,
                             SWORD16 dz_step, int count)
    {
        php
        rep     #0x30
        cld

        ldy     #0x0000
        lda     z_start
        ldx     count
        beq     zsr_done

zsr_loop:
        cmp     [zbuf_ptr],y    ; A (z courant, code "distance inversee") vs stocke
        bcs     zsr_skip        ; A >= stocke -> plus loin/egal, on saute
        sta     [zbuf_ptr],y    ; A < stocke  -> plus proche, on met a jour
zsr_skip:
        clc
        adc     dz_step         ; avance la profondeur interpolee (2-complement)
        iny
        iny
        dex
        bne     zsr_loop

zsr_done:
        plp
        rtl
    }

/* --------------------------------------------------------------------
 * NOTES D'INTEGRATION
 *
 * 1) Voir renderModelFullscreenZBuffer (fichier separe) pour le
 *    portage de renderModelScanlineZBuffer_fast vers ce module :
 *    ZBuffer_Init() une fois au demarrage, ZBuffer_Clear() une fois
 *    par FRAME (plus par ligne), ZBuffer_QuantizeInvZ() aux deux
 *    extremites de chaque span pour obtenir z_start/dz_step, puis
 *    ZBuffer_TestAndSet() + drawPixel() existant pour chaque pixel --
 *    version simple et correcte fournie en premier ; voir la note (3)
 *    pour la suite.
 *
 * 2) Les aretes elargies qui debordaient auparavant sur la ligne
 *    voisine (Z-buffer par ligne, jamais resolu) peuvent maintenant
 *    etre testees honnetement : la profondeur des lignes voisines
 *    existe encore en memoire au moment ou on les dessine.
 *
 * 3) ZBuffer_TestSetRow existe pour un passage ulterieur, plus rapide,
 *    quand vous voudrez fusionner test de profondeur ET ecriture
 *    couleur (nibble packing SHR, cf. drawPixel) dans une seule boucle
 *    assembleur. Point d'attention reel non resolu ici : le Z-buffer
 *    travaille naturellement en 16 bits (accumulateur A en mode REP
 *    #0x30) alors que l'ecriture nibble dans le framebuffer est en 8
 *    bits (SEP #0x20) -- melanger les deux dans la meme boucle oblige
 *    a alterner SEP/REP a chaque pixel, ce qui a un cout reel et peut
 *    annuler une bonne partie du gain. Alternative probablement plus
 *    rapide a essayer : deux passes separees par span -- une premiere
 *    boucle 100% 16 bits qui fait le test de profondeur et pose un
 *    bit dans un petit masque de visibilite (1 bit/pixel, donc 40
 *    octets max pour 320 pixels), puis une seconde boucle 100% 8 bits
 *    qui ecrit les pixels couleur en ne consultant que ce masque. A
 *    profiler sur materiel/emulateur reel avant de choisir entre les
 *    deux approches -- je n'ai pas de moyen de mesurer les cycles ici.
 * -------------------------------------------------------------------- */

/* ====================================================================
 * Rendu : portage de renderModelScanlineZBuffer_fast/_biased (voir
 * engine.c) vers ce Z-buffer plein ecran. Colle ici (plutot que
 * compile a part) car il utilise directement des types/fonctions
 * definis dans engine.c (Model3D, drawPixel, getFaceFillColor, etc.)
 * sans passer par un header commun -- voir la note d'inclusion en
 * tete de ce fichier pour ou le coller dans engine.c.
 *
 * Comme dans engine.c, deux variantes + un dispatcher :
 *  - _fast   : pas de biais anti Z-fighting, utilisee quand
 *              cull_back_faces est actif (pas de paire front/back
 *              coplanaire visible simultanement dans ce cas).
 *  - _biased : ajoute Z_FIGHT_BIAS aux faces front (plane_d[f] > 0)
 *              pour qu'elles gagnent de facon fiable le Z-test face a
 *              leur pendant back coplanaire -- utilisee quand
 *              cull_back_faces est desactive. C'etait la piste
 *              laissee de cote : sans elle, les deux faces d'une
 *              paire coplanaire se contestent pixel par pixel selon
 *              l'arrondi de quantification, d'ou la couleur de la
 *              face arriere qui perce par endroits a travers la face
 *              avant.
 * ==================================================================== */

void renderModelFullscreenZBuffer_fast(Model3D* model) {
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;
    int fcount = faces->face_count;
    int y, f, i;

    /* --- 1/z per vertex (inchange) --- */
    static float* inv_z = NULL;
    static int inv_z_capacity = 0;
    if (inv_z_capacity < vcount) {
        if (inv_z) free(inv_z);
        inv_z = (float*)malloc(vcount * sizeof(float));
        inv_z_capacity = vcount;
    }
    {
        float frame_max_inv_z = 0.0f;
        for (i = 0; i < vcount; i++) {
            float zo_f = FIXED_TO_FLOAT(vtx->zo[i]);
            inv_z[i] = (zo_f > 0.0f) ? (1.0f / zo_f) : 0.0f;
            if (inv_z[i] > frame_max_inv_z) frame_max_inv_z = inv_z[i];
        }
        /* Recalcule l'echelle de quantification sur le VRAI max de
         * cette frame (donc de ce niveau de zoom), plutot que sur une
         * constante figee -- voir zbuffer_fullscreen.h. */
        ZBuffer_SetScaleForFrame(frame_max_inv_z);
    }

#ifdef ZBUF_DIAG_SCALE
    /* --- Diagnostic temporaire pour calibrer ZBUFFER_INV_Z_SCALE ---
     * N'ecrit qu'UNE seule fois (static zdiag_logged) pour ne pas
     * ajouter d'I/O disque a chaque frame (deja un point sensible du
     * projet). Compiler avec ZBUF_DIAG_SCALE defini, lancer sur
     * quelques scenes/modeles representatifs (idealement le pire cas
     * en terme de plage de profondeur), relever "zdiag.txt" a chaque
     * fois (il est ecrase, pas cumule), puis retirer ce bloc et
     * ZBUF_DIAG_SCALE une fois ZBUFFER_INV_Z_SCALE recalibre. Adapter
     * fopen/fprintf a votre mecanisme de log fichier existant si
     * different (rappel : redirection stdout non disponible sous
     * ORCA/C sur cette cible). */
    {
        static int zdiag_logged = 0;
        if (!zdiag_logged) {
            float dbg_min = 1e30f, dbg_max = -1e30f;
            FILE* zdiag_f;
            for (i = 0; i < vcount; i++) {
                if (inv_z[i] > 0.0f) {
                    if (inv_z[i] < dbg_min) dbg_min = inv_z[i];
                    if (inv_z[i] > dbg_max) dbg_max = inv_z[i];
                }
            }
            zdiag_f = fopen("zdiag.txt", "w");
            if (zdiag_f) {
                fprintf(zdiag_f, "inv_z min=%f max=%f vcount=%d\n",
                        dbg_min, dbg_max, vcount);
                fclose(zdiag_f);
            }
            zdiag_logged = 1;
        }
    }
#endif

    ScanIntersection hits[MAX_SPAN_INTERSECTIONS];

    SetPenMode(0);
    applyPalette(palette);

    /* Bounds de clipping en "model space" (inchange) */
    int clip_x_min = -pan_dx;
    int clip_x_max = SCREEN_WIDTH - 1 - pan_dx;

    /* BUG CORRIGE (herite tel quel de renderModelScanlineZBuffer_fast/
     * _biased) : boucler "y" sur [0, SCREEN_HEIGHT) en espace modele
     * ne couvre PAS tout l'ecran des que pan_dy != 0. Exemple concret :
     * pan_dy = -60 -> l'ecran ligne 199 aurait besoin de y=259, jamais
     * atteint puisque la boucle s'arrete a y=199 -- ce qui laisse une
     * bande de |pan_dy| lignes non couverte en bas (ou en haut, selon
     * le signe), meme si le modele a bien du contenu a cet endroit.
     * Les bornes doivent suivre pan_dy pour que screenY couvre
     * exactement [0, SCREEN_HEIGHT-1] par construction -- a reporter
     * aussi dans renderModelScanlineZBuffer_fast/_biased (meme boucle,
     * meme defaut, probablement juste moins remarque jusqu'ici). */
    int y_lo = -pan_dy;
    int y_hi = SCREEN_HEIGHT - 1 - pan_dy;

    /* Z-buffer plein ecran : une seule remise a zero pour TOUTE la
     * frame, avant de parcourir les lignes -- plus de reset par ligne. */
    ZBuffer_Clear();

    for (y = y_lo; y <= y_hi; y++) {
        int screenY = y + pan_dy;   /* toujours dans [0, SCREEN_HEIGHT-1] par construction */

        for (f = 0; f < fcount; f++) {
            int n, offt, k, hit_count;

            if (!faces->display_flag[f]) continue;
            n = faces->vertex_count[f];
            if (n < 3) continue;
            if (y < faces->miny[f] || y > faces->maxy[f]) continue;

            int fillColor = getFaceFillColor(f);
            int frameColor = getFaceFrameColor(f, fillColor);

            offt = faces->vertex_indices_ptr[f];
            hit_count = 0;

            for (k = 0; k < n; k++) {
                int k2 = (k + 1 < n) ? (k + 1) : 0;
                int vid1 = faces->vertex_indices_buffer[offt + k] - 1;
                int vid2 = faces->vertex_indices_buffer[offt + k2] - 1;

                int y1 = vtx->y2d[vid1];
                int y2 = vtx->y2d[vid2];
                int dy = y2 - y1;

                if (dy == 0) continue;
                if (dy > 0) {
                    if (y < y1 || y >= y2) continue;
                } else {
                    if (y < y2 || y >= y1) continue;
                }

                int x1 = vtx->x2d[vid1];
                int x2 = vtx->x2d[vid2];

                float t = (float)(y - y1) / (float)dy;
                int xi = x1 + (int)((x2 - x1) * t + 0.5f);
                float izf = inv_z[vid1] + (inv_z[vid2] - inv_z[vid1]) * t;

                if (hit_count < MAX_SPAN_INTERSECTIONS) {
                    hits[hit_count].x = xi;
                    hits[hit_count].inv_z = izf;
                    hit_count++;
                }
            }

            if (hit_count < 2) continue;

            if (hit_count == 2) {
                if (hits[0].x > hits[1].x) {
                    ScanIntersection tmp = hits[0];
                    hits[0] = hits[1];
                    hits[1] = tmp;
                }
            } else {
                int a, b;
                for (a = 1; a < hit_count; a++) {
                    ScanIntersection key = hits[a];
                    b = a - 1;
                    while (b >= 0 && hits[b].x > key.x) {
                        hits[b + 1] = hits[b];
                        b--;
                    }
                    hits[b + 1] = key;
                }
            }

            int p;
            for (p = 0; p + 1 < hit_count; p += 2) {
                int xa = hits[p].x;
                int xb = hits[p + 1].x;
                float iza = hits[p].inv_z;
                float izb = hits[p + 1].inv_z;

                if (xa > xb) continue;
                if (xb < clip_x_min || xa > clip_x_max) continue;

                int x0 = xa;
                int x1 = xb;
                float iz0 = iza;

                if (x0 == x1) {
                    int sx0 = x0 + pan_dx;
                    UWORD16 zq = ZBuffer_QuantizeInvZ(iz0);
                    if (ZBuffer_TestAndSet(sx0, screenY, zq)) {
                        drawPixel(sx0, screenY, frameColor);
                    }
                    continue;
                }

                float dIz = (izb - iza) / (float)(xb - xa);

                if (x0 < clip_x_min) {
                    iz0 += dIz * (clip_x_min - x0);
                    x0 = clip_x_min;
                }
                if (x1 > clip_x_max) x1 = clip_x_max;
                if (x0 > x1) continue;

                int sx0 = x0 + pan_dx;
                int sx1 = x1 + pan_dx;

                if (x0 == x1) {
                    UWORD16 zq = ZBuffer_QuantizeInvZ(iz0);
                    if (ZBuffer_TestAndSet(sx0, screenY, zq)) {
                        drawPixel(sx0, screenY, frameColor);
                    }
                } else {
                    /* Premier pixel = contour */
                    UWORD16 zq0 = ZBuffer_QuantizeInvZ(iz0);
                    if (ZBuffer_TestAndSet(sx0, screenY, zq0)) {
                        drawPixel(sx0, screenY, frameColor);
                    }

                    /* Dernier pixel = contour -- profondeur exacte au
                     * clip, calculee directement en float (comme dans
                     * la version scanline) plutot qu'accumulee pixel
                     * par pixel. */
                    {
                        float   iz_end_f = iz0 + dIz * (float)(x1 - x0);
                        UWORD16 zq_end   = ZBuffer_QuantizeInvZ(iz_end_f);

                        /* Milieu = fill. Optimisation : la quantification
                         * (flottant->16 bits, coute une multiplication +
                         * arrondi + clamp en LOGICIEL sur 65816, pas de
                         * FPU) n'est plus faite QU'AUX DEUX EXTREMITES du
                         * span, pas par pixel. A l'interieur du span, on
                         * avance par un pas ENTIER 16 bits (une simple
                         * addition, native et rapide) au lieu de
                         * recalculer float->16 bits a chaque pixel --
                         * c'etait la cause tres probable du ralentissement
                         * 2-3x mesure par rapport a la version scanline. */
                        int     nsteps = x1 - x0;
                        SWORD16 dz_step_int = (SWORD16)(((long)zq_end - (long)zq0) / nsteps);
                        UWORD16 zq_cur = zq0;
                        int     x;

                        for (x = x0 + 1; x < x1; x++) {
                            int screenX = x + pan_dx;
                            zq_cur = (UWORD16)(zq_cur + dz_step_int);
                            if (ZBuffer_TestAndSet(screenX, screenY, zq_cur)) {
                                drawPixel(screenX, screenY, fillColor);
                            }
                        }

                        if (ZBuffer_TestAndSet(sx1, screenY, zq_end)) {
                            drawPixel(sx1, screenY, frameColor);
                        }
                    }
                }
            }
        }
    }
}

/* renderModelFullscreenZBuffer_biased -- identique a _fast, seule
 * difference : un biais (Z_FIGHT_BIAS, meme valeur que dans engine.c)
 * est ajoute a la profondeur interpolee des faces FRONT
 * (faces->plane_d[f] > 0), calcule une fois par face, avant
 * quantification. Cette fonction n'est appelee que quand
 * cull_back_faces est desactive (voir le dispatcher plus bas) : c'est
 * le seul cas ou les deux faces d'une paire coplanaire peuvent etre
 * visibles en meme temps et se disputer le Z-test.
 */
void renderModelFullscreenZBuffer_biased(Model3D* model) {
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;
    int fcount = faces->face_count;
    int y, f, i;
    static const float Z_FIGHT_BIAS = 0.001f;  /* meme valeur que engine.c */

    static float* inv_z = NULL;
    static int inv_z_capacity = 0;
    if (inv_z_capacity < vcount) {
        if (inv_z) free(inv_z);
        inv_z = (float*)malloc(vcount * sizeof(float));
        inv_z_capacity = vcount;
    }
    {
        float frame_max_inv_z = 0.0f;
        for (i = 0; i < vcount; i++) {
            float zo_f = FIXED_TO_FLOAT(vtx->zo[i]);
            inv_z[i] = (zo_f > 0.0f) ? (1.0f / zo_f) : 0.0f;
            if (inv_z[i] > frame_max_inv_z) frame_max_inv_z = inv_z[i];
        }
        ZBuffer_SetScaleForFrame(frame_max_inv_z);
    }

    ScanIntersection hits[MAX_SPAN_INTERSECTIONS];

    SetPenMode(0);
    applyPalette(palette);

    int clip_x_min = -pan_dx;
    int clip_x_max = SCREEN_WIDTH - 1 - pan_dx;

    int y_lo = -pan_dy;
    int y_hi = SCREEN_HEIGHT - 1 - pan_dy;

    ZBuffer_Clear();

    for (y = y_lo; y <= y_hi; y++) {
        int screenY = y + pan_dy;

        for (f = 0; f < fcount; f++) {
            int n, offt, k, hit_count;

            if (!faces->display_flag[f]) continue;
            n = faces->vertex_count[f];
            if (n < 3) continue;
            if (y < faces->miny[f] || y > faces->maxy[f]) continue;

            int fillColor = getFaceFillColor(f);
            int frameColor = getFaceFrameColor(f, fillColor);

            /* Biais anti Z-fighting -- cette fonction n'est appelee
             * que quand cull_back_faces est desactive (voir le
             * dispatcher), donc pas besoin de re-tester
             * cull_back_faces ici : toujours pertinent d'appliquer le
             * biais sur les faces front. */
            float depthBias = (faces->plane_d[f] > 0) ? Z_FIGHT_BIAS : 0.0f;

            offt = faces->vertex_indices_ptr[f];
            hit_count = 0;

            for (k = 0; k < n; k++) {
                int k2 = (k + 1 < n) ? (k + 1) : 0;
                int vid1 = faces->vertex_indices_buffer[offt + k] - 1;
                int vid2 = faces->vertex_indices_buffer[offt + k2] - 1;

                int y1 = vtx->y2d[vid1];
                int y2 = vtx->y2d[vid2];
                int dy = y2 - y1;

                if (dy == 0) continue;
                if (dy > 0) {
                    if (y < y1 || y >= y2) continue;
                } else {
                    if (y < y2 || y >= y1) continue;
                }

                int x1 = vtx->x2d[vid1];
                int x2 = vtx->x2d[vid2];

                float t = (float)(y - y1) / (float)dy;
                int xi = x1 + (int)((x2 - x1) * t + 0.5f);
                float izf = inv_z[vid1] + (inv_z[vid2] - inv_z[vid1]) * t;
                izf += depthBias;  /* biais applique une fois par intersection */

                if (hit_count < MAX_SPAN_INTERSECTIONS) {
                    hits[hit_count].x = xi;
                    hits[hit_count].inv_z = izf;
                    hit_count++;
                }
            }

            if (hit_count < 2) continue;

            if (hit_count == 2) {
                if (hits[0].x > hits[1].x) {
                    ScanIntersection tmp = hits[0];
                    hits[0] = hits[1];
                    hits[1] = tmp;
                }
            } else {
                int a, b;
                for (a = 1; a < hit_count; a++) {
                    ScanIntersection key = hits[a];
                    b = a - 1;
                    while (b >= 0 && hits[b].x > key.x) {
                        hits[b + 1] = hits[b];
                        b--;
                    }
                    hits[b + 1] = key;
                }
            }

            int p;
            for (p = 0; p + 1 < hit_count; p += 2) {
                int xa = hits[p].x;
                int xb = hits[p + 1].x;
                float iza = hits[p].inv_z;
                float izb = hits[p + 1].inv_z;

                if (xa > xb) continue;
                if (xb < clip_x_min || xa > clip_x_max) continue;

                int x0 = xa;
                int x1 = xb;
                float iz0 = iza;

                if (x0 == x1) {
                    int sx0 = x0 + pan_dx;
                    UWORD16 zq = ZBuffer_QuantizeInvZ(iz0);
                    if (ZBuffer_TestAndSet(sx0, screenY, zq)) {
                        drawPixel(sx0, screenY, frameColor);
                    }
                    continue;
                }

                float dIz = (izb - iza) / (float)(xb - xa);

                if (x0 < clip_x_min) {
                    iz0 += dIz * (clip_x_min - x0);
                    x0 = clip_x_min;
                }
                if (x1 > clip_x_max) x1 = clip_x_max;
                if (x0 > x1) continue;

                int sx0 = x0 + pan_dx;
                int sx1 = x1 + pan_dx;

                if (x0 == x1) {
                    UWORD16 zq = ZBuffer_QuantizeInvZ(iz0);
                    if (ZBuffer_TestAndSet(sx0, screenY, zq)) {
                        drawPixel(sx0, screenY, frameColor);
                    }
                } else {
                    UWORD16 zq0 = ZBuffer_QuantizeInvZ(iz0);
                    if (ZBuffer_TestAndSet(sx0, screenY, zq0)) {
                        drawPixel(sx0, screenY, frameColor);
                    }

                    {
                        float   iz_end_f = iz0 + dIz * (float)(x1 - x0);
                        UWORD16 zq_end   = ZBuffer_QuantizeInvZ(iz_end_f);

                        int     nsteps = x1 - x0;
                        SWORD16 dz_step_int = (SWORD16)(((long)zq_end - (long)zq0) / nsteps);
                        UWORD16 zq_cur = zq0;
                        int     x;

                        for (x = x0 + 1; x < x1; x++) {
                            int screenX = x + pan_dx;
                            zq_cur = (UWORD16)(zq_cur + dz_step_int);
                            if (ZBuffer_TestAndSet(screenX, screenY, zq_cur)) {
                                drawPixel(screenX, screenY, fillColor);
                            }
                        }

                        if (ZBuffer_TestAndSet(sx1, screenY, zq_end)) {
                            drawPixel(sx1, screenY, frameColor);
                        }
                    }
                }
            }
        }
    }
}

/* --- Dispatcher, meme logique que renderModelScanlineZBuffer dans
 * engine.c --------------------------------------------------------- */
void renderModelFullscreenZBuffer(Model3D* model) {
    if (cull_back_faces) {
        renderModelFullscreenZBuffer_fast(model);
    } else {
        renderModelFullscreenZBuffer_biased(model);
    }
}
