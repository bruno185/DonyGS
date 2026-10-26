/*
 * ============================================================================
 *  zbuffer_scanline.c — Z-buffer scanline — Variantes flottantes et
 *  quantifiées
 * ============================================================================
 *
 *  Rôle :
 *    Résoudre la visibilité par pixel avec une seule ligne de profondeur
 *    réutilisée, à partir des coordonnées projetées et des métriques déjà
 *    préparées par le moteur.
 *
 *  Organisation :
 *    Les versions flottantes et V2 sont conservées pour comparaison. Le
 *    dispatch public sélectionne V3, qui prépare les codes 16 bits de
 *    profondeur inverse, les arêtes et les faces actives, puis dessine
 *    chaque span par un noyau assembleur. Le fichier fournit aussi les
 *    couleurs effectives et drawPixel.
 *
 *  Dépendances et contraintes :
 *    Inclus par engine.c ; écran SHR 320x200, deux pixels par octet. Les
 *    anciens chemins et V3 ont des précisions et règles de contour
 *    différentes. Sans culling, la variante active emploie un biais. Les
 *    buffers statiques et paquets de spans sont non réentrants ; la mémoire
 *    Z seule ne résume pas leur consommation totale.
 *
 * ============================================================================
 */


typedef struct {
    int x;
    float inv_z;
} ScanIntersection;

/*
 * ----------------------------------------------------------------------------
 *  getFaceFillColor
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Résoudre la couleur de remplissage d'une face.
 *
 *  Paramètres :
 *    [entrée] face_id valide base zéro
 *
 *  Retour :
 *    Indice selon priorité shading, aléatoire disponible, choix utilisateur,
 *    défaut.
 *
 *  Fonctionnement :
 *    Aucun dessin ni modification de l'état. Les sélections spéciales
 *    doivent être cohérentes avec les buffers.
 *
 * ----------------------------------------------------------------------------
 */
int getFaceFillColor(int face_id) {
    int fill_color;
    /* Appliquer la priorité d'orientation avant les autres sources de couleur. */
    if (shaded_by_orientation) {
        fill_color = face_shade_color[face_id];
    } else if (user_fill_color == 16 && random_fill_colors != NULL && face_id < random_colors_capacity) {
        fill_color = random_fill_colors[face_id];
    } else if (user_fill_color >= 0) {
        fill_color = user_fill_color;
    } else {
        fill_color = COL_FILL_DEFAULT;
    }
    return fill_color;
}

/*
 * ----------------------------------------------------------------------------
 *  getFaceFrameColor
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Résoudre la couleur de contour d'une face.
 *
 *  Paramètres :
 *    [entrée] face_id valide et fill_color effectif
 *
 *  Retour :
 *    Remplissage si choix 17, couleur aléatoire disponible si 16, choix
 *    explicite ou COL_FRAME.
 *
 *  Fonctionnement :
 *    Aucun dessin ; ne recalcule pas le remplissage.
 *
 * ----------------------------------------------------------------------------
 */
int getFaceFrameColor(int face_id, int fill_color) {
    int frame_color;
    /* Traiter le contour identique au remplissage avant les sélections
     * indépendantes.
     */
    if (user_frame_color == 17) {
        frame_color = fill_color;
    } else if (user_frame_color == 16 && random_frame_colors != NULL && face_id < random_colors_capacity) {
        frame_color = random_frame_colors[face_id];
    } else if (user_frame_color >= 0) {
        frame_color = user_frame_color;
    } else {
        frame_color = COL_FRAME;
    }
    return frame_color;
}

/*
 * ----------------------------------------------------------------------------
 *  drawPixel
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Écrire directement un pixel SHR 320x200 en préservant son voisin.
 *
 *  Paramètres :
 *    [entrée] x dans [0,319], y dans [0,199], color masqué sur quatre bits.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Aucun clipping ni test Z. Adresse $E12000+y*160+x/2. L'assembleur passe
 *    A en 8 bits, laisse X sur 16 bits et termine A sur 16 bits. A/X et
 *    indicateurs modifiés ; DBR ignoré par l'adressage long. Pixel pair dans
 *    le nibble haut, impair dans le nibble bas.
 *
 * ----------------------------------------------------------------------------
 */
void drawPixel(int x, int y, int color)
{
    int offset;
    unsigned char value;

    color &= 0x0F;


    /* Calculer l'offset d'octet dans la banque SHR. */
    offset = y * 160 + (x >> 1);

    asm {
        /* Lire l'octet en mode A 8 bits et sauver sa valeur avant le choix de parité. */
        sep     #0x20

        ldx     offset
        lda     0xE12000,x


        pha

        lda     x
        and     #0x01
        bne     pixel_odd


        pla
        and     #0x0F
        sta     value

        lda     color
        asl     a
        asl     a
        asl     a
        asl     a
        and     #0xF0
        ora     value
        sta     0xE12000,x
        bra     done

/* Modifier uniquement le nibble bas pour x impair. */
pixel_odd:

        pla
        and     #0xF0
        sta     value

        lda     color
        and     #0x0F
        ora     value
        sta     0xE12000,x

/* Revenir au format d'accumulateur 16 bits attendu par le C. */
done:
        rep     #0x20
    }
}

/*
 * ----------------------------------------------------------------------------
 *  SL_ClearTop12Lines
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Effacer les douze premières lignes de pixels SHR avant le rendu.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Écrit 1920 octets depuis $E1:2000.
 *
 *    Compteur 16 bits décrémenté de deux, test BPL incluant le mot d'offset
 *    zéro ; accès longs indépendants de DBR.
 *
 *  Registres et indicateurs :
 *    Préserve P, A et X ; Y et DBR non utilisés.
 *
 * ----------------------------------------------------------------------------
 */
static void SL_ClearTop12Lines(void)
{
    asm {
        /* Sauvegarder le contexte puis effacer les mots de $077E à zéro inclus. */
        php
        rep #0x30
        pha
        phx
        lda #0
        ldx #0x077E
    sl_ct12_loop:
        sta >0xE12000,x
        dex
        dex
        bpl sl_ct12_loop
        plx
        pla
        plp
    }
}

/*
 * ----------------------------------------------------------------------------
 *  renderModelScanlineZBuffer_old
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Conserver la variante scanline initiale.
 *
 *  Paramètres :
 *    [entrée] model valide, coordonnées projetées et profondeurs calculées.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    écran SHR et buffers statiques ; les allocations ne sont pas toutes
 *    contrôlées.
 *
 *  Fonctionnement :
 *    Recalcule 1/z et balaie les faces pour chaque ligne, avec une seule
 *    ligne Z float. Aucun rapprochement de profondeur volontaire.
 *
 *    Variante conservée pour comparaison, hors dispatch public actuel. Les
 *    règles de contour diffèrent de V3 ; ne pas la supprimer comme doublon
 *    prouvé.
 *
 * ----------------------------------------------------------------------------
 */
void renderModelScanlineZBuffer_old (Model3D* model) {
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;
    int fcount = faces->face_count;
    int y, f, i;


    static float* inv_z = NULL;
    static int inv_z_capacity = 0;
    /* Réserver les inverses Z réutilisables puis les recalculer pour les sommets. */
    if (inv_z_capacity < vcount) {
        if (inv_z) free(inv_z);
        inv_z = (float*)malloc(vcount * sizeof(float));
        inv_z_capacity = vcount;
    }
    for (i = 0; i < vcount; i++) {
        float zo_f = FIXED_TO_FLOAT(vtx->zo[i]);
        inv_z[i] = (zo_f > 0.0f) ? (1.0f / zo_f) : 0.0f;
    }


    static float zbuffer_line[SCREEN_WIDTH];

    ScanIntersection hits[MAX_SPAN_INTERSECTIONS];

    SetPenMode(0);
    applyPalette(palette);

    SL_ClearTop12Lines();
    /* Effacer la profondeur de la ligne et chercher les spans de chaque face. */
    for (y = 0; y < SCREEN_HEIGHT; y++) {
        int screenY = y + pan_dy;
        if (screenY < 0 || screenY >= SCREEN_HEIGHT) continue;

        for (i = 0; i < SCREEN_WIDTH; i++) zbuffer_line[i] = -1.0f;

        /* Appliquer visibilité et bornes avant d'intersecter les arêtes. */
        for (f = 0; f < fcount; f++) {
            int n, offt, k, hit_count;

            if (!faces->display_flag[f]) continue;
            n = faces->vertex_count[f];
            if (n < 3) continue;
            if (y < faces->miny[f] || y > faces->maxy[f]) continue;

            offt = faces->vertex_indices_ptr[f];
            hit_count = 0;


            for (k = 0; k < n; k++) {
                int k2 = (k + 1 < n) ? (k + 1) : 0;
                int vid1 = faces->vertex_indices_buffer[offt + k] - 1;
                int vid2 = faces->vertex_indices_buffer[offt + k2] - 1;

                int y1 = vtx->y2d[vid1];
                int y2 = vtx->y2d[vid2];
                int x1 = vtx->x2d[vid1];
                int x2 = vtx->x2d[vid2];

                int ylo = (y1 < y2) ? y1 : y2;
                int yhi = (y1 < y2) ? y2 : y1;
                if (y < ylo || y >= yhi) continue;


                float t = (float)(y - y1) / (float)(y2 - y1);
                int xi = x1 + (int)((x2 - x1) * t + 0.5f);


                float izf = inv_z[vid1] + (inv_z[vid2] - inv_z[vid1]) * t;

                if (hit_count < MAX_SPAN_INTERSECTIONS) {
                    hits[hit_count].x = xi;
                    hits[hit_count].inv_z = izf;
                    hit_count++;
                }
            }

            /* Ignorer la ligne sans span puis ordonner les intersections disponibles. */
            if (hit_count < 2) continue;


            {
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


            {
                int p;
                for (p = 0; p + 1 < hit_count; p += 2) {
                    int xa = hits[p].x;
                    int xb = hits[p + 1].x;
                    float iza = hits[p].inv_z;
                    float izb = hits[p + 1].inv_z;
                    int x;

                    if (xa == xb) continue;


                    {
                        int fillColor = getFaceFillColor(f);
                        int frameColor = getFaceFrameColor(f, fillColor);


                        float dIz = (izb - iza) / (float)(xb - xa);
                        float iz_here = iza;

                        for (x = xa; x <= xb; x++) {
                            int screenX = x + pan_dx;
                            if (screenX >= 0 && screenX < SCREEN_WIDTH) {
                                if (iz_here > zbuffer_line[x]) {
                                    zbuffer_line[x] = iz_here;


                                    if (x == xa || x == xb) {
                                        drawPixel(screenX, screenY, frameColor);
                                    } else {
                                        drawPixel(screenX, screenY, fillColor);
                                    }
                                }
                            }
                            iz_here += dIz;
                        }
                    }
                }
            }
        }
    }
}

/*
 * ----------------------------------------------------------------------------
 *  renderModelScanlineZBuffer_fast
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Conserver la variante scanline flottante rapide.
 *
 *  Paramètres :
 *    [entrée] model valide, coordonnées projetées et profondeurs calculées.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    écran SHR et buffers statiques ; les allocations ne sont pas toutes
 *    contrôlées.
 *
 *  Fonctionnement :
 *    Recalcule 1/z et balaie les faces pour chaque ligne, avec une seule
 *    ligne Z float. Aucun rapprochement de profondeur volontaire.
 *
 *    Variante conservée pour comparaison, hors dispatch public actuel. Les
 *    règles de contour diffèrent de V3 ; ne pas la supprimer comme doublon
 *    prouvé.
 *
 * ----------------------------------------------------------------------------
 */
void renderModelScanlineZBuffer_fast(Model3D* model) {
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;
    int fcount = faces->face_count;
    int y, f, i;


    static float* inv_z = NULL;
    static int inv_z_capacity = 0;
    /* Réserver les inverses Z réutilisables puis les recalculer pour les sommets. */
    if (inv_z_capacity < vcount) {
        if (inv_z) free(inv_z);
        inv_z = (float*)malloc(vcount * sizeof(float));
        inv_z_capacity = vcount;
    }
    for (i = 0; i < vcount; i++) {
        float zo_f = FIXED_TO_FLOAT(vtx->zo[i]);
        inv_z[i] = (zo_f > 0.0f) ? (1.0f / zo_f) : 0.0f;
    }

    static float zbuffer_line[SCREEN_WIDTH];
    ScanIntersection hits[MAX_SPAN_INTERSECTIONS];

    SetPenMode(0);
    applyPalette(palette);


    int clip_x_min = -pan_dx;
    int clip_x_max = SCREEN_WIDTH - 1 - pan_dx;

    SL_ClearTop12Lines();
    /* Effacer la profondeur de la ligne et chercher les spans de chaque face. */
    for (y = 0; y < SCREEN_HEIGHT; y++) {
        int screenY = y + pan_dy;
        if (screenY < 0 || screenY >= SCREEN_HEIGHT) continue;


        float* zb_clear = zbuffer_line;
        float* zb_clear_end = zbuffer_line + SCREEN_WIDTH;
        while (zb_clear < zb_clear_end) *zb_clear++ = -1.0f;

        /* Appliquer visibilité et bornes avant d'intersecter les arêtes. */
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

            /* Ignorer la ligne sans span puis ordonner les intersections disponibles. */
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
                    if (iz0 > zbuffer_line[x0]) {
                        zbuffer_line[x0] = iz0;
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


                    if (iz0 > zbuffer_line[x0]) {
                        zbuffer_line[x0] = iz0;
                        drawPixel(sx0, screenY, frameColor);
                    }
                } else {

                    if (iz0 > zbuffer_line[x0]) {
                        zbuffer_line[x0] = iz0;
                        drawPixel(sx0, screenY, frameColor);
                    }


                    float iz = iz0 + dIz;
                    float* zb = &zbuffer_line[x0 + 1];
                    int x;
                    for (x = x0 + 1; x < x1; x++) {
                        if (iz > *zb) {
                            *zb = iz;
                            drawPixel(x + pan_dx, screenY, fillColor);
                        }
                        iz += dIz;
                        zb++;
                    }


                    if (iz > *zb) {
                        *zb = iz;
                        drawPixel(sx1, screenY, frameColor);
                    }
                }
            }
        }
    }
}

/*
 * ----------------------------------------------------------------------------
 *  renderModelScanlineZBuffer_biased
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Conserver la variante scanline flottante avec biais.
 *
 *  Paramètres :
 *    [entrée] model valide, coordonnées projetées et profondeurs calculées.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    écran SHR et buffers statiques ; les allocations ne sont pas toutes
 *    contrôlées.
 *
 *  Fonctionnement :
 *    Recalcule 1/z et balaie les faces pour chaque ligne, avec une seule
 *    ligne Z float. Ajoute un biais de 0,001 aux faces plane_d>0.
 *
 *    Variante conservée pour comparaison, hors dispatch public actuel. Les
 *    règles de contour diffèrent de V3 ; ne pas la supprimer comme doublon
 *    prouvé.
 *
 * ----------------------------------------------------------------------------
 */
void renderModelScanlineZBuffer_biased(Model3D* model) {
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;
    int fcount = faces->face_count;
    int y, f, i;
    static const float Z_FIGHT_BIAS = 0.001f;


    static float* inv_z = NULL;
    static int inv_z_capacity = 0;
    /* Réserver les inverses Z réutilisables puis les recalculer pour les sommets. */
    if (inv_z_capacity < vcount) {
        if (inv_z) free(inv_z);
        inv_z = (float*)malloc(vcount * sizeof(float));
        inv_z_capacity = vcount;
    }
    for (i = 0; i < vcount; i++) {
        float zo_f = FIXED_TO_FLOAT(vtx->zo[i]);
        inv_z[i] = (zo_f > 0.0f) ? (1.0f / zo_f) : 0.0f;
    }

    static float zbuffer_line[SCREEN_WIDTH];
    ScanIntersection hits[MAX_SPAN_INTERSECTIONS];

    SetPenMode(0);
    applyPalette(palette);


    int clip_x_min = -pan_dx;
    int clip_x_max = SCREEN_WIDTH - 1 - pan_dx;

    SL_ClearTop12Lines();
    /* Effacer la profondeur de la ligne et chercher les spans de chaque face. */
    for (y = 0; y < SCREEN_HEIGHT; y++) {
        int screenY = y + pan_dy;
        if (screenY < 0 || screenY >= SCREEN_HEIGHT) continue;


        float* zb_clear = zbuffer_line;
        float* zb_clear_end = zbuffer_line + SCREEN_WIDTH;
        while (zb_clear < zb_clear_end) *zb_clear++ = -1.0f;

        /* Appliquer visibilité et bornes avant d'intersecter les arêtes. */
        for (f = 0; f < fcount; f++) {
            int n, offt, k, hit_count;

            if (!faces->display_flag[f]) continue;
            n = faces->vertex_count[f];
            if (n < 3) continue;
            if (y < faces->miny[f] || y > faces->maxy[f]) continue;


            int fillColor = getFaceFillColor(f);
            int frameColor = getFaceFrameColor(f, fillColor);


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
                izf += depthBias;

                if (hit_count < MAX_SPAN_INTERSECTIONS) {
                    hits[hit_count].x = xi;
                    hits[hit_count].inv_z = izf;
                    hit_count++;
                }
            }

            /* Ignorer la ligne sans span puis ordonner les intersections disponibles. */
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
                    if (iz0 > zbuffer_line[x0]) {
                        zbuffer_line[x0] = iz0;
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


                    if (iz0 > zbuffer_line[x0]) {
                        zbuffer_line[x0] = iz0;
                        drawPixel(sx0, screenY, frameColor);
                    }
                } else {

                    if (iz0 > zbuffer_line[x0]) {
                        zbuffer_line[x0] = iz0;
                        drawPixel(sx0, screenY, frameColor);
                    }


                    float iz = iz0 + dIz;
                    float* zb = &zbuffer_line[x0 + 1];
                    int x;
                    for (x = x0 + 1; x < x1; x++) {
                        if (iz > *zb) {
                            *zb = iz;
                            drawPixel(x + pan_dx, screenY, fillColor);
                        }
                        iz += dIz;
                        zb++;
                    }


                    if (iz > *zb) {
                        *zb = iz;
                        drawPixel(sx1, screenY, frameColor);
                    }
                }
            }
        }
    }
}


typedef struct {
    float x;
    float inv_z;
    float x_step;
    float inv_z_step;
    int ya, yb;
} ScanEdgeFast;

/*
 * ----------------------------------------------------------------------------
 *  renderModelScanlineZBuffer_coreV2
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Rendre les surfaces par balayage scanline V2 flottant.
 *
 *  Paramètres :
 *    [entrée] model valide avec projection et métriques à jour ; biased,
 *    active le rapprochement des faces de plane_d>0 lorsque le culling est
 *    désactivé.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    couleurs, mémoire Z, écran ou framebuffer, buffers statiques non
 *    réentrants.
 *
 *  Fonctionnement :
 *    N’effectue ni transformation ni classement des faces ; les faces actives sont
 *    parcourues par identifiant pour définir les égalités de profondeur.
 *    Réserve/réutilise buffers par sommet/face/arête, calcule 1/z, prépare
 *    les arêtes, classe les débuts de face par histogramme puis balaie
 *    uniquement les faces actives. Ligne Z en float, interpolation de 1/z et
 *    biais flottant.
 *
 *    Le clipping du viewport est appliqué aux spans ; pas de clipping 3D
 *    général. Les réserves et le comportement d'échec restent propres à
 *    cette version.
 *
 * ----------------------------------------------------------------------------
 */
static void renderModelScanlineZBuffer_coreV2(Model3D* model, int biased)
{
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;
    int fcount = faces->face_count;
    int y, f, i, k, b, si, ai, keep, active_count;
    int y_lo, y_hi, clip_x_min, clip_x_max;
    int total_edges;
    static const float Z_FIGHT_BIAS = 0.001f;

    static float* inv_z = NULL;
    static int vert_capacity = 0;
    static int* face_start = NULL;
    static int* face_sorted = NULL;
    static int* face_active = NULL;
    static unsigned char* face_fill = NULL;
    static unsigned char* face_frame = NULL;
    static float* face_bias = NULL;
    static int face_capacity = 0;
    static ScanEdgeFast* edge_buf = NULL;
    static int edge_capacity = 0;
    static int bucket[SCREEN_HEIGHT + 1];
    static int bucket_cur[SCREEN_HEIGHT + 1];
    static float zbuffer_line[SCREEN_WIDTH];
    ScanIntersection hits[MAX_SPAN_INTERSECTIONS];

    if (vcount <= 0 || fcount <= 0) return;

    /* Agrandir les buffers de sommets et conserver leur capacité entre les rendus. */
    if (vert_capacity < vcount) {
        if (inv_z) free(inv_z);
        inv_z = (float*)malloc(vcount * sizeof(float));
        if (!inv_z) { vert_capacity = 0; return; }
        vert_capacity = vcount;
    }

    /* Réserver les tables de faces, couleurs et biais. */
    if (face_capacity < fcount) {
        if (face_start)  free(face_start);
        if (face_sorted) free(face_sorted);
        if (face_active) free(face_active);
        if (face_fill)   free(face_fill);
        if (face_frame)  free(face_frame);
        if (face_bias)   free(face_bias);
        face_start  = (int*)malloc(fcount * sizeof(int));
        face_sorted = (int*)malloc(fcount * sizeof(int));
        face_active = (int*)malloc(fcount * sizeof(int));
        face_fill   = (unsigned char*)malloc(fcount);
        face_frame  = (unsigned char*)malloc(fcount);
        face_bias   = (float*)malloc(fcount * sizeof(float));
        if (!face_start || !face_sorted || !face_active || !face_fill ||
            !face_frame || !face_bias) {
            face_capacity = 0;
            return;
        }
        face_capacity = fcount;
    }

    total_edges = 0;
    for (f = 0; f < fcount; f++) total_edges += faces->vertex_count[f];
    if (edge_capacity < total_edges) {
        if (edge_buf) free(edge_buf);
        edge_buf = (ScanEdgeFast*)malloc(total_edges * sizeof(ScanEdgeFast));
        if (!edge_buf) { edge_capacity = 0; return; }
        edge_capacity = total_edges;
    }


    /* Calculer les profondeurs inverses et les codes selon le format de
     * cette version.
     */
    for (i = 0; i < vcount; i++) {
        float zo_f = FIXED_TO_FLOAT(vtx->zo[i]);
        inv_z[i] = (zo_f > 0.0f) ? (1.0f / zo_f) : 0.0f;
    }

    clip_x_min = -pan_dx;
    clip_x_max = SCREEN_WIDTH - 1 - pan_dx;
    y_lo = -pan_dy;
    if (y_lo < 0) y_lo = 0;
    y_hi = SCREEN_HEIGHT - 1 - pan_dy;
    if (y_hi >= SCREEN_HEIGHT) y_hi = SCREEN_HEIGHT - 1;
    if (y_lo > y_hi) return;

    /* Transformer l'histogramme en offsets cumulés pour classer les débuts
     * de faces.
     */
    /* Initialiser l'histogramme des premières lignes ; préparer visibilité,
     * couleurs et arêtes.
     */
    for (b = 0; b <= SCREEN_HEIGHT; b++) bucket[b] = 0;


    for (f = 0; f < fcount; f++) {
        int n = faces->vertex_count[f];
        int offt = faces->vertex_indices_ptr[f];
        int first_y, last_y;
        int fill;

        face_start[f] = -1;
        if (!faces->display_flag[f] || n < 3) continue;

        first_y = faces->miny[f];
        last_y  = faces->maxy[f];
        if (first_y < y_lo) first_y = y_lo;
        if (last_y > y_hi) last_y = y_hi;
        if (first_y > last_y) continue;

        face_start[f] = first_y - y_lo;
        bucket[face_start[f] + 1]++;
        fill = getFaceFillColor(f);
        face_fill[f] = (unsigned char)fill;
        face_frame[f] = (unsigned char)getFaceFrameColor(f, fill);
        face_bias[f] = (biased && faces->plane_d[f] > 0) ? Z_FIGHT_BIAS : 0.0f;

        for (k = 0; k < n; k++) {
            int k2 = (k + 1 < n) ? (k + 1) : 0;
            int vid1 = faces->vertex_indices_buffer[offt + k] - 1;
            int vid2 = faces->vertex_indices_buffer[offt + k2] - 1;
            int y1 = vtx->y2d[vid1];
            int y2 = vtx->y2d[vid2];
            int dy = y2 - y1;
            ScanEdgeFast* ed = &edge_buf[offt + k];

            if (dy == 0) {
                ed->ya = 32767;
                ed->yb = 32767;
                ed->x = ed->inv_z = ed->x_step = ed->inv_z_step = 0.0f;
                continue;
            }

            ed->x_step = (float)(vtx->x2d[vid2] - vtx->x2d[vid1]) / (float)dy;
            ed->inv_z_step = (inv_z[vid2] - inv_z[vid1]) / (float)dy;

            if (dy > 0) {
                ed->ya = y1;
                ed->yb = y2;
                ed->x = (float)vtx->x2d[vid1];
                ed->inv_z = inv_z[vid1];
            } else {
                ed->ya = y2;
                ed->yb = y1;
                ed->x = (float)vtx->x2d[vid2];
                ed->inv_z = inv_z[vid2];
            }


            if (ed->ya < y_lo && ed->yb > y_lo) {
                float skip = (float)(y_lo - ed->ya);
                ed->x += ed->x_step * skip;
                ed->inv_z += ed->inv_z_step * skip;
            }
        }
    }

    for (b = 0; b < SCREEN_HEIGHT; b++) bucket[b + 1] += bucket[b];
    for (b = 0; b <= SCREEN_HEIGHT; b++) bucket_cur[b] = bucket[b];
    for (f = 0; f < fcount; f++) {
        b = face_start[f];
        if (b >= 0) face_sorted[bucket_cur[b]++] = f;
    }

    SetPenMode(0);
    applyPalette(palette);

    /* Démarrer la liste des faces actives, puis balayer les lignes visibles. */
    active_count = 0;
    SL_ClearTop12Lines();
    for (y = y_lo; y <= y_hi; y++) {
        int screenY = y + pan_dy;
        int row = y - y_lo;
        float* zp;
        float* zend;


        /* Insérer les faces commençant sur cette ligne en gardant l'ordre
         * d'identifiants.
         */
        for (si = bucket[row]; si < bucket[row + 1]; si++) {
            int nf = face_sorted[si];
            int pos = active_count;
            while (pos > 0 && face_active[pos - 1] > nf) {
                face_active[pos] = face_active[pos - 1];
                pos--;
            }
            face_active[pos] = nf;
            active_count++;
        }

        zp = zbuffer_line;
        zend = zbuffer_line + SCREEN_WIDTH;
        while (zp < zend) *zp++ = -1.0f;

        keep = 0;
        /* Écarter les faces terminées et extraire les intersections des
         * arêtes actives.
         */
        for (ai = 0; ai < active_count; ai++) {
            int n, offt, hit_count;
            int fillColor, frameColor;
            float depthBias;
            ScanEdgeFast* ed;

            f = face_active[ai];
            if (faces->maxy[f] < y) continue;
            face_active[keep++] = f;

            n = faces->vertex_count[f];
            offt = faces->vertex_indices_ptr[f];
            hit_count = 0;
            depthBias = face_bias[f];
            ed = edge_buf + offt;

            for (k = 0; k < n; k++, ed++) {
                if (y < ed->ya || y >= ed->yb) continue;
                if (hit_count < MAX_SPAN_INTERSECTIONS) {
                    hits[hit_count].x = (int)(ed->x + 0.5f);
                    hits[hit_count].inv_z = ed->inv_z + depthBias;
                    hit_count++;
                }
                ed->x += ed->x_step;
                ed->inv_z += ed->inv_z_step;
            }

            if (hit_count < 2) continue;
            /* Ordonner les intersections : échange simple pour deux, tri par
             * insertion au-delà.
             */
            if (hit_count == 2) {
                if (hits[0].x > hits[1].x) {
                    ScanIntersection tmp = hits[0];
                    hits[0] = hits[1];
                    hits[1] = tmp;
                }
            } else {
                int a, c;
                for (a = 1; a < hit_count; a++) {
                    ScanIntersection key = hits[a];
                    c = a - 1;
                    while (c >= 0 && hits[c].x > key.x) {
                        hits[c + 1] = hits[c];
                        c--;
                    }
                    hits[c + 1] = key;
                }
            }

            fillColor = face_fill[f];
            frameColor = face_frame[f];

            {
                int p;
                /* Apparier les intersections par parité, clipper les spans
                 * et les transmettre au noyau de dessin.
                 */
                for (p = 0; p + 1 < hit_count; p += 2) {
                    int xa = hits[p].x;
                    int xb = hits[p + 1].x;
                    float iza = hits[p].inv_z;
                    float izb = hits[p + 1].inv_z;
                    int x0, x1, x;
                    float dIz, iz0, iz;
                    float* zb;

                    if (xa > xb) continue;
                    if (xb < clip_x_min || xa > clip_x_max) continue;

                    x0 = xa;
                    x1 = xb;
                    iz0 = iza;

                    if (x0 == x1) {
                        int sx = x0 + pan_dx;
                        if (iz0 > zbuffer_line[x0]) {
                            zbuffer_line[x0] = iz0;
                            drawPixel(sx, screenY, frameColor);
                        }
                        continue;
                    }

                    dIz = (izb - iza) / (float)(xb - xa);
                    if (x0 < clip_x_min) {
                        iz0 += dIz * (float)(clip_x_min - x0);
                        x0 = clip_x_min;
                    }
                    if (x1 > clip_x_max) x1 = clip_x_max;
                    if (x0 > x1) continue;

                    if (x0 == x1) {
                        if (iz0 > zbuffer_line[x0]) {
                            zbuffer_line[x0] = iz0;
                            drawPixel(x0 + pan_dx, screenY, frameColor);
                        }
                        continue;
                    }

                    if (iz0 > zbuffer_line[x0]) {
                        zbuffer_line[x0] = iz0;
                        drawPixel(x0 + pan_dx, screenY, frameColor);
                    }

                    iz = iz0 + dIz;
                    zb = &zbuffer_line[x0 + 1];
                    for (x = x0 + 1; x < x1; x++, zb++) {
                        if (iz > *zb) {
                            *zb = iz;
                            drawPixel(x + pan_dx, screenY, fillColor);
                        }
                        iz += dIz;
                    }

                    if (iz > *zb) {
                        *zb = iz;
                        drawPixel(x1 + pan_dx, screenY, frameColor);
                    }
                }
            }
        }
        active_count = keep;
    }
}


typedef unsigned int SLZWord;
#define SL_Z_FRAC 12
#define SL_Z_ROUND 0x800L
#define SL_Z_BIAS 0.0005f
#define SL_Z_FAR 0xFFFF
#define SL_SHR_BASE 0xE12000UL
#define SL_TARGET_MAX_CODE 65000.0f

static SLZWord sl_zline[SCREEN_WIDTH];
static float sl_zscale = 1.0f;


/*
 * ----------------------------------------------------------------------------
 *  SL_ClearLine
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Réinitialiser la ligne Z réutilisée par le rendu scanline V3.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    Aucun ; écrit SCREEN_WIDTH mots $FFFF dans sl_zline.
 *
 *  Fonctionnement :
 *    Les accès globaux longs évitent la dépendance à DBR.
 *
 *  Registres et indicateurs :
 *    Préserve P, modifie A/X.
 *
 * ----------------------------------------------------------------------------
 */
static void SL_ClearLine(void)
{
    asm {
        php
        rep #0x30
        ldx #0
        lda #0xFFFF
/* Remplir chaque mot avant de rendre la ligne au parcours suivant. */
slcl_loop:
        sta >sl_zline,x
        inx
        inx
        cpx #SCREEN_WIDTH*2
        bne slcl_loop
        plp
    }
}

typedef struct { int x; SLZWord zq; } SLZHit;
typedef struct { long x,z,sx,sz; int ya,yb,xb; } SLZEdge;

/*
 * ----------------------------------------------------------------------------
 *  SL_SetScale
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Adapter la quantification scanline en réservant de la marge pour le
 *    biais.
 *
 *  Paramètres :
 *    [entrée] max_inv_z
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    sl_zscale=cible/(max_inv_z+SL_Z_BIAS) si le dénominateur dépasse 1e-7.
 *
 *  Fonctionnement :
 *    Sinon conserve l'échelle précédente.
 *
 * ----------------------------------------------------------------------------
 */
static void SL_SetScale(float max_inv_z)
{

    /* Inclure le biais maximal avant la division pour limiter la saturation. */
    float max_biased_inv_z = max_inv_z + SL_Z_BIAS;

    if (max_biased_inv_z > 0.0000001f)
        sl_zscale = SL_TARGET_MAX_CODE / max_biased_inv_z;
}

/*
 * ----------------------------------------------------------------------------
 *  SL_Quantize
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Quantifier une profondeur inverse en code 16 bits où petit signifie
 *    proche.
 *
 *  Paramètres :
 *    [entrée] inv_z, profondeur inverse flottante ; valeurs négatives
 *    ramenées à zéro.
 *
 *  Retour :
 *    65535 - round(inv_z*échelle), après saturation de la valeur quantifiée
 *    dans [0,65535]. $FFFF est la valeur vide/lointaine.
 *
 *  Fonctionnement :
 *    Aucun changement d'échelle.
 *
 * ----------------------------------------------------------------------------
 */
static SLZWord SL_Quantize(float inv_z)
{
    float scaled;
    long q;
    /* Appliquer l'échelle scanline, arrondir, saturer puis inverser le code. */
    if (inv_z < 0.0f) inv_z = 0.0f;
    scaled = inv_z * sl_zscale;
    q = (long)(scaled + 0.5f);
    if (q < 0) q = 0;
    if (q > 0xFFFFL) q = 0xFFFFL;
    return (SLZWord)(0xFFFFUL - (unsigned long)q);
}

/*
 * ----------------------------------------------------------------------------
 *  SL_LerpZ
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Interpoler linéairement un code Z entre deux abscisses.
 *
 *  Paramètres :
 *    [entrée] za/zb, codes aux positions xa/xb ; x, position demandée.
 *
 *  Retour :
 *    Za si xa==xb, sinon za+(zb-za)*(x-xa)/(xb-xa), calcul intermédiaire
 *    long.
 *
 *  Fonctionnement :
 *    Aucune saturation ; le domaine interpolé et les produits doivent être
 *    valides.
 *
 * ----------------------------------------------------------------------------
 */
static SLZWord SL_LerpZ(SLZWord za, SLZWord zb, int xa, int xb, int x)
{
    long num;
    /* Garder la profondeur gauche pour une abscisse unique, sinon interpoler
     * en long.
     */
    if (xb == xa) return za;
    num = ((long)zb - (long)za) * (long)(x - xa);
    return (SLZWord)((long)za + num / (long)(xb - xa));
}

long sl_p_zacc, sl_p_zstep;
unsigned long sl_p_zptr, sl_p_pptr;
int sl_p_n, sl_p_odd, sl_p_mode, sl_p_cl, sl_p_ci, sl_p_cr;
unsigned int sl_p_zl, sl_p_zr;

/*
 * ----------------------------------------------------------------------------
 *  SL_SpanRun
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Dessiner un span préclippé avec test Z strict dans un noyau 65C816.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Entrées et sorties implicites :
 *    Entrée : paquet partagé sl_p_* : pointeurs Z et pixels far, parité
 *    gauche, codes gauche/droite, couleurs gauche/intérieur/droite,
 *    accumulateur et pas 16.16, nombre de pixels intérieurs et mode 0
 *    (gauche seul) ou 1 (span complet).
 *
 *  Retour :
 *    Aucun ; écrit Z et les nibbles de couleur.
 *
 *  Registres et indicateurs :
 *    Préserve P et D ; A/X/Y modifiés, DBR inchangé. Mode décimal supposé
 *    désactivé. Cadre D de 32 octets : zacc à 0/2, zstep à 4/6, zptr à 8/10,
 *    pptr à 12/14, compte à 16, couleur basse/haute à 18/20, parité à 22,
 *    masque/valeur à 24/26. Les profondeurs [8],Y peuvent franchir une
 *    banque ; le pointeur de pixels n'est incrémenté que sur 16 bits et doit
 *    rester dans sa banque. Traite les extrémités exactement et l'intérieur
 *    par paires pair/impair. Les pixels de même code Z ne sont pas
 *    remplacés. Non réentrant.
 *
 * ----------------------------------------------------------------------------
 */
static void SL_SpanRun(void)
{
    asm {


        /* Sauvegarder le contexte et réserver 32 octets ; D est positionné
         * sur le premier octet utile.
         */
        php
        phd
        rep #0x30
        tsc
        sec
        sbc #32
        tcs
        inc a
        tcd


        /* Copier le paquet dans le cadre direct et préparer les deux nibbles
         * de couleur.
         */
        lda >sl_p_zacc
        sta 0
        lda >sl_p_zacc+2
        sta 2
        lda >sl_p_zstep
        sta 4
        lda >sl_p_zstep+2
        sta 6
        lda >sl_p_zptr
        sta 8
        lda >sl_p_zptr+2
        sta 10
        lda >sl_p_pptr
        sta 12
        lda >sl_p_pptr+2
        sta 14
        lda >sl_p_n
        sta 16

        lda >sl_p_ci
        and #0x000F
        sta 18
        asl a
        asl a
        asl a
        asl a
        sta 20

        lda >sl_p_odd
        eor #1
        sta 22


        ldy #0

        lda >sl_p_odd
        beq zbr_l_ev
        lda #0x00F0
        sta 24
        lda >sl_p_cl
        and #0x000F
        sta 26
        bra zbr_l_go
    zbr_l_ev:
        lda #0x000F
        sta 24
        lda >sl_p_cl
        and #0x000F
        asl a
        asl a
        asl a
        asl a
        sta 26
    /* Tester la profondeur exacte du pixel gauche avant toute écriture du nibble. */
    zbr_l_go:

        lda >sl_p_zl
        cmp [8],y
        bcs zbr_lsk
        sta [8],y
        sep #0x20
        lda [12]
        and 24
        ora 26
        sta [12]
        rep #0x20
    zbr_lsk:

        lda >sl_p_mode
        bne zbr_full
        brl zbr_done


    /* Avancer après le pixel gauche ; l'adresse couleur progresse seulement
     * après un pixel impair.
     */
    zbr_full:
        iny
        iny
        lda >sl_p_odd
        beq zbr_lev
        inc 12
    zbr_lev:


        lda 16
        bne zbr_int
        brl zbr_right


    /* Traiter seul un premier pixel impair pour aligner le reste sur des paires. */
    zbr_int:
        lda 22
        beq zbp_even

        clc
        lda 0
        adc 4
        sta 0
        lda 2
        adc 6
        sta 2
        cmp [8],y
        bcs zbp_o1
        sta [8],y
        sep #0x20
        lda [12]
        and #0xF0
        ora 18
        sta [12]
        rep #0x20
    zbp_o1:
        iny
        iny
        inc 12
        dec 16


    zbp_even:
        lda 16
        lsr a
        tax
        beq zbp_tail

    /* Avancer zacc en 32 bits pour chaque pixel ; propager C du mot bas au
     * mot haut, puis tester Z.
     */
    zbp_pair:
        clc
        lda 0
        adc 4
        sta 0
        lda 2
        adc 6
        sta 2
        cmp [8],y
        bcs zbp_e1
        sta [8],y
        sep #0x20
        lda [12]
        and #0x0F
        ora 20
        sta [12]
        rep #0x20
    zbp_e1:
        iny
        iny
        clc
        lda 0
        adc 4
        sta 0
        lda 2
        adc 6
        sta 2
        cmp [8],y
        bcs zbp_o2
        sta [8],y
        sep #0x20
        lda [12]
        and #0xF0
        ora 18
        sta [12]
        rep #0x20
    zbp_o2:
        iny
        iny
        inc 12
        dex
        bne zbp_pair


    /* Traiter l'éventuel pixel pair résiduel après les paires. */
    zbp_tail:
        lda 16
        and #1
        bne zbp_t1
        brl zbr_right
    zbp_t1:

        clc
        lda 0
        adc 4
        sta 0
        lda 2
        adc 6
        sta 2
        cmp [8],y
        bcs zbp_e2
        sta [8],y
        sep #0x20
        lda [12]
        and #0x0F
        ora 20
        sta [12]
        rep #0x20
    zbp_e2:
        iny
        iny


    /* Recalculer la parité et employer le code exact de l'extrémité droite. */
    zbr_right:
        lda >sl_p_n
        clc
        adc 22
        and #1
        beq zbr_r_ev
        lda #0x00F0
        sta 24
        lda >sl_p_cr
        and #0x000F
        sta 26
        bra zbr_r_go
    zbr_r_ev:
        lda #0x000F
        sta 24
        lda >sl_p_cr
        and #0x000F
        asl a
        asl a
        asl a
        asl a
        sta 26
    zbr_r_go:
        lda >sl_p_zr
        cmp [8],y
        bcs zbr_rsk
        sta [8],y
        sep #0x20
        lda [12]
        and 24
        ora 26
        sta [12]
        rep #0x20
    zbr_rsk:


    /* Libérer exactement le cadre avant de restaurer D et P. */
    zbr_done:
        rep #0x30
        tsc
        clc
        adc #32
        tcs
        pld
        plp
    }
}


/*
 * ----------------------------------------------------------------------------
 *  SL_PaintSpan
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Préparer un span V3 qui utilise la seule ligne de profondeur courante.
 *
 *  Paramètres :
 *    [entrée] x0/x1 inclus sans pan, screenY avec pan, z0/z1 aux extrémités,
 *    fillColor/frameColor et nudge en codes.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Span déjà clippé requis. Soustrait nudge avec borne zéro, prépare
 *    adresses et parité puis interpole Z en 16.16. Les deux extrémités
 *    utilisent la couleur de contour.
 *
 * ----------------------------------------------------------------------------
 */
static void SL_PaintSpan(int x0, int x1, int screenY,
                         SLZWord z0, SLZWord z1,
                         int fillColor, int frameColor, int nudge)
{
    int sx0, nsteps;
    long zstep, zacc;
    if (x0 > x1) return;
    /* Réduire les deux codes de profondeur sans passer sous zéro. */
    if (nudge) {
        z0 = (z0 > (SLZWord)nudge) ? (SLZWord)(z0-nudge) : 0;
        z1 = (z1 > (SLZWord)nudge) ? (SLZWord)(z1-nudge) : 0;
    }
    /* Préparer les pointeurs de la ligne Z et de l'écran au premier pixel. */
    sx0 = x0 + pan_dx;
    sl_p_zptr = (unsigned long)(sl_zline + sx0);
    sl_p_pptr = SL_SHR_BASE + (unsigned long)screenY * 160UL + (unsigned long)(sx0 >> 1);
    sl_p_odd = sx0 & 1;
    sl_p_zl = z0;
    sl_p_cl = frameColor;
    /* Le span ponctuel se limite au pixel gauche ; sinon préparer intérieur
     * et bord droit.
     */
    if (x0 == x1) {
        sl_p_mode = 0;
        SL_SpanRun();
        return;
    }
    nsteps = x1-x0;
    zstep = (((long)z1-(long)z0) << 16) / nsteps;
    zacc = ((long)z0 << 16) + 0x8000L;
    sl_p_zr = z1;
    sl_p_zacc = zacc;
    sl_p_zstep = zstep;
    sl_p_n = nsteps-1;
    sl_p_ci = fillColor;
    sl_p_cr = frameColor;
    sl_p_mode = 1;
    SL_SpanRun();
}

/*
 * ----------------------------------------------------------------------------
 *  renderModelScanlineZBuffer_coreV3
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Rendre les surfaces par balayage scanline V3 quantifié.
 *
 *  Paramètres :
 *    [entrée] model valide avec projection et métriques à jour ; biased,
 *    active le rapprochement des faces de plane_d>0 lorsque le culling est
 *    désactivé.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    couleurs, mémoire Z, écran ou framebuffer, buffers statiques non
 *    réentrants.
 *
 *  Fonctionnement :
 *    N’effectue ni transformation ni classement des faces ; les faces actives sont
 *    parcourues par identifiant pour définir les égalités de profondeur.
 *    Réserve/réutilise buffers par sommet/face/arête, calcule 1/z, prépare
 *    les arêtes, classe les débuts de face par histogramme puis balaie
 *    uniquement les faces actives. Ligne Z de 320 codes 16 bits ; arêtes à
 *    douze fractions, spans en 16.16 et petit nudge entier.
 *
 *    Le clipping du viewport est appliqué aux spans ; pas de clipping 3D
 *    général. Les réserves et le comportement d'échec restent propres à
 *    cette version.
 *
 * ----------------------------------------------------------------------------
 */
static void renderModelScanlineZBuffer_coreV3(Model3D* model, int biased)
{
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;
    int fcount = faces->face_count;
    int y, f, i, k, b, si, ai, keep, active_count;
    int total_edges, end;
    int clip_x_min, clip_x_max, y_lo, y_hi;
    float frame_max_inv_z;
    SLZEdge* ed;
    SLZHit hits[MAX_SPAN_INTERSECTIONS];

    static float* inv_z = NULL;
    static SLZWord* vzq = NULL;
    static SLZWord* vzq_b = NULL;
    static int vert_capacity = 0;

    static int* face_start = NULL;
    static int* face_sorted = NULL;
    static int* face_active = NULL;
    static unsigned char* face_fill = NULL;
    static unsigned char* face_frame = NULL;
    static unsigned char* face_nudge = NULL;
    static int face_capacity = 0;

    static SLZEdge* edge_buf = NULL;
    static int edge_capacity = 0;


    static int bucket[SCREEN_HEIGHT + 1];
    static int bucket_cur[SCREEN_HEIGHT + 1];


    /* Réserver les tables de faces, couleurs et biais. */
    if (face_capacity < fcount) {
        if (face_start)   free(face_start);
        if (face_sorted)  free(face_sorted);
        if (face_active)  free(face_active);
        if (face_fill)    free(face_fill);
        if (face_frame)   free(face_frame);
        if (face_nudge)   free(face_nudge);
        face_start  = (int*)malloc(fcount * sizeof(int));
        face_sorted = (int*)malloc(fcount * sizeof(int));
        face_active = (int*)malloc(fcount * sizeof(int));
        face_fill   = (unsigned char*)malloc(fcount);
        face_frame  = (unsigned char*)malloc(fcount);
        face_nudge  = (unsigned char*)malloc(fcount);
        face_capacity = (face_start && face_sorted && face_active &&
                         face_fill && face_frame && face_nudge) ? fcount : 0;
    }
    /* Agrandir les buffers de sommets et conserver leur capacité entre les rendus. */
    if (vert_capacity < vcount) {
        if (inv_z) free(inv_z);
        if (vzq)   free(vzq);
        if (vzq_b) free(vzq_b);
        inv_z = (float*)malloc(vcount * sizeof(float));
        vzq   = (SLZWord*)malloc(vcount * sizeof(SLZWord));
        vzq_b = (SLZWord*)malloc(vcount * sizeof(SLZWord));
        vert_capacity = (inv_z && vzq && vzq_b) ? vcount : 0;
    }
    total_edges = 0;
    for (f = 0; f < fcount; f++) {
        end = faces->vertex_indices_ptr[f] + faces->vertex_count[f];
        if (end > total_edges) total_edges = end;
    }
    if (edge_capacity < total_edges) {
        if (edge_buf) free(edge_buf);
        edge_buf = (SLZEdge*)malloc((size_t)total_edges * sizeof(SLZEdge));
        edge_capacity = edge_buf ? total_edges : 0;
    }
    if (face_capacity < fcount || vert_capacity < vcount || edge_capacity < total_edges)
        return;


    frame_max_inv_z = 0.0f;
    /* Calculer les profondeurs inverses et les codes selon le format de
     * cette version.
     */
    for (i = 0; i < vcount; i++) {
        float zo_f = FIXED_TO_FLOAT(vtx->zo[i]);
        inv_z[i] = (zo_f > 0.0f) ? (1.0f / zo_f) : 0.0f;
        if (inv_z[i] > frame_max_inv_z) frame_max_inv_z = inv_z[i];
    }
    SL_SetScale(frame_max_inv_z);

    for (i = 0; i < vcount; i++) {

        vzq[i]   = SL_Quantize(inv_z[i]);
        vzq_b[i] = biased ? SL_Quantize(inv_z[i] + SL_Z_BIAS) : vzq[i];
    }

    SetPenMode(0);
    applyPalette(palette);


    clip_x_min = -pan_dx;
    clip_x_max = SCREEN_WIDTH - 1 - pan_dx;
    y_lo = -pan_dy;
    y_hi = SCREEN_HEIGHT - 1 - pan_dy;


    /* Transformer l'histogramme en offsets cumulés pour classer les débuts
     * de faces.
     */
    /* Initialiser l'histogramme des premières lignes ; préparer visibilité,
     * couleurs et arêtes.
     */
    for (b = 0; b <= SCREEN_HEIGHT; b++) bucket[b] = 0;

    for (f = 0; f < fcount; f++) {
        int n, offt, fmin, fmax, fill;
        SLZWord* zsrc;

        face_start[f] = -1;
        if (!faces->display_flag[f]) continue;
        n = faces->vertex_count[f];
        if (n < 3) continue;
        fmin = faces->miny[f];
        fmax = faces->maxy[f];
        if (fmax < y_lo || fmin > y_hi) continue;


        b = ((fmin > y_lo) ? fmin : y_lo) - y_lo;
        face_start[f] = b;
        bucket[b + 1]++;

        fill = getFaceFillColor(f);
        face_fill[f]  = (unsigned char)fill;
        face_frame[f] = (unsigned char)getFaceFrameColor(f, fill);
        face_nudge[f] = (unsigned char)((biased && faces->plane_d[f] > 0) ? 2 : 0);

        offt = faces->vertex_indices_ptr[f];
        zsrc = (biased && faces->plane_d[f] > 0) ? vzq_b : vzq;
        ed = edge_buf + offt;

        for (k = 0; k < n; k++, ed++) {
            int k2 = (k + 1 < n) ? (k + 1) : 0;
            int vid1 = faces->vertex_indices_buffer[offt + k] - 1;
            int vid2 = faces->vertex_indices_buffer[offt + k2] - 1;
            int y1 = vtx->y2d[vid1];
            int y2 = vtx->y2d[vid2];
            int dy = y2 - y1;
            int ya, yb, xs;
            long zs, ex, ez, sx, sz, skip;

            if (dy == 0) {
                ed->ya = 32767; ed->yb = 32767;
                ed->x = 0L; ed->z = 0L; ed->sx = 0L; ed->sz = 0L;
                continue;
            }


            sx = (((long)vtx->x2d[vid2] - (long)vtx->x2d[vid1]) * 4096L) / (long)dy;
            sz = (((long)zsrc[vid2] - (long)zsrc[vid1]) * 4096L) / (long)dy;


            if (dy > 0) { ya = y1; yb = y2; xs = 0;                                        zs = (long)zsrc[vid1]; }
            else        { ya = y2; yb = y1; xs = vtx->x2d[vid2] - vtx->x2d[vid1];          zs = (long)zsrc[vid2]; }

            ex = (long)xs * 4096L;
            ez = zs * 4096L;


            if (ya < y_lo && yb > y_lo) {
                skip = (long)(y_lo - ya);
                ex += sx * skip;
                ez += sz * skip;
            }

            ed->ya = ya;  ed->yb = yb;
            ed->xb = vtx->x2d[vid1];
            ed->x  = ex + SL_Z_ROUND;
            ed->z  = ez + SL_Z_ROUND;
            ed->sx = sx;  ed->sz = sz;
        }
    }


    for (b = 0; b < SCREEN_HEIGHT; b++) bucket[b + 1] += bucket[b];
    for (b = 0; b <= SCREEN_HEIGHT; b++) bucket_cur[b] = bucket[b];
    for (f = 0; f < fcount; f++) {
        b = face_start[f];
        if (b >= 0) face_sorted[bucket_cur[b]++] = f;
    }


    /* Démarrer la liste des faces actives, puis balayer les lignes visibles. */
    active_count = 0;
    SL_ClearTop12Lines();
    for (y = y_lo; y <= y_hi; y++) {
        int screenY = y + pan_dy;
        int row = y - y_lo;

        SL_ClearLine();


        /* Insérer les faces commençant sur cette ligne en gardant l'ordre
         * d'identifiants.
         */
        for (si = bucket[row]; si < bucket[row + 1]; si++) {
            int nf = face_sorted[si];
            int pos = active_count;
            while (pos > 0 && face_active[pos - 1] > nf) {
                face_active[pos] = face_active[pos - 1];
                pos--;
            }
            face_active[pos] = nf;
            active_count++;
        }

        keep = 0;
        /* Écarter les faces terminées et extraire les intersections des
         * arêtes actives.
         */
        for (ai = 0; ai < active_count; ai++) {
            int n, offt, hit_count;
            int fillColor, frameColor, nudge;

            f = face_active[ai];
            if (faces->maxy[f] < y) continue;
            face_active[keep++] = f;

            n = faces->vertex_count[f];
            offt = faces->vertex_indices_ptr[f];
            hit_count = 0;


            ed = edge_buf + offt;
            for (k = 0; k < n; k++, ed++) {
                if (y < ed->ya || y >= ed->yb) continue;
                if (hit_count < MAX_SPAN_INTERSECTIONS) {


                    long t = ed->x;
                    hits[hit_count].x  = ed->xb + ((t >= 0L) ? (int)(t >> SL_Z_FRAC) : -(int)((-t) >> SL_Z_FRAC));
                    hits[hit_count].zq = (SLZWord)(ed->z >> SL_Z_FRAC);
                    hit_count++;
                }

                ed->x += ed->sx;
                ed->z += ed->sz;
            }

            if (hit_count < 2) continue;


            /* Ordonner les intersections : échange simple pour deux, tri par
             * insertion au-delà.
             */
            if (hit_count == 2) {
                if (hits[0].x > hits[1].x) {
                    SLZHit tmp = hits[0];
                    hits[0] = hits[1];
                    hits[1] = tmp;
                }
            } else {
                int a, c;
                for (a = 1; a < hit_count; a++) {
                    SLZHit key = hits[a];
                    c = a - 1;
                    while (c >= 0 && hits[c].x > key.x) {
                        hits[c + 1] = hits[c];
                        c--;
                    }
                    hits[c + 1] = key;
                }
            }

            fillColor  = face_fill[f];
            frameColor = face_frame[f];
            nudge      = face_nudge[f];


            {
                int p;
                /* Apparier les intersections par parité, clipper les spans
                 * et les transmettre au noyau de dessin.
                 */
                for (p = 0; p + 1 < hit_count; p += 2) {
                    int xa = hits[p].x;
                    int xb = hits[p + 1].x;
                    SLZWord zqa = hits[p].zq;
                    SLZWord zqb = hits[p + 1].zq;
                    int x0, x1;
                    SLZWord zq0, zq1;

                    if (xa > xb) continue;
                    if (xb < clip_x_min || xa > clip_x_max) continue;

                    x0 = xa;  x1 = xb;
                    zq0 = zqa; zq1 = zqb;

                    if (x0 < clip_x_min) { zq0 = SL_LerpZ(zqa, zqb, xa, xb, clip_x_min); x0 = clip_x_min; }
                    if (x1 > clip_x_max) { zq1 = SL_LerpZ(zqa, zqb, xa, xb, clip_x_max); x1 = clip_x_max; }
                    if (x0 > x1) continue;

                    SL_PaintSpan(x0, x1, screenY, zq0, zq1,
                                 fillColor, frameColor, nudge);
                }
            }
        }
        active_count = keep;
    }
}


/*
 * ----------------------------------------------------------------------------
 *  renderModelScanlineZBuffer_fastV3
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Appeler renderModelScanlineZBuffer_coreV3 sans rapprochement des faces
 *    de plane_d>0.
 *
 *  Paramètres :
 *    [entrée] model valide avec données de vue préparées.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Reprend les préconditions, effets mémoire et règles de dessin du noyau
 *    appelé.
 *
 * ----------------------------------------------------------------------------
 */
void renderModelScanlineZBuffer_fastV3(Model3D* model)
{
    /* Transmettre explicitement le choix du biais au noyau commun. */
    renderModelScanlineZBuffer_coreV3(model, 0);
}

/*
 * ----------------------------------------------------------------------------
 *  renderModelScanlineZBuffer_biasedV3
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Appeler renderModelScanlineZBuffer_coreV3 avec rapprochement des faces
 *    de plane_d>0.
 *
 *  Paramètres :
 *    [entrée] model valide avec données de vue préparées.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Reprend les préconditions, effets mémoire et règles de dessin du noyau
 *    appelé.
 *
 * ----------------------------------------------------------------------------
 */
void renderModelScanlineZBuffer_biasedV3(Model3D* model)
{
    /* Transmettre explicitement le choix du biais au noyau commun. */
    renderModelScanlineZBuffer_coreV3(model, 1);
}

/*
 * ----------------------------------------------------------------------------
 *  renderModelScanlineZBuffer_fastV2
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Appeler renderModelScanlineZBuffer_coreV2 sans rapprochement des faces
 *    de plane_d>0.
 *
 *  Paramètres :
 *    [entrée] model valide avec données de vue préparées.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Reprend les préconditions, effets mémoire et règles de dessin du noyau
 *    appelé.
 *
 * ----------------------------------------------------------------------------
 */
void renderModelScanlineZBuffer_fastV2(Model3D* model)
{
    /* Transmettre explicitement le choix du biais au noyau commun. */
    renderModelScanlineZBuffer_coreV2(model, 0);
}

/*
 * ----------------------------------------------------------------------------
 *  renderModelScanlineZBuffer_biasedV2
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Appeler renderModelScanlineZBuffer_coreV2 avec rapprochement des faces
 *    de plane_d>0.
 *
 *  Paramètres :
 *    [entrée] model valide avec données de vue préparées.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Reprend les préconditions, effets mémoire et règles de dessin du noyau
 *    appelé.
 *
 * ----------------------------------------------------------------------------
 */
void renderModelScanlineZBuffer_biasedV2(Model3D* model)
{
    /* Transmettre explicitement le choix du biais au noyau commun. */
    renderModelScanlineZBuffer_coreV2(model, 1);
}

/*
 * ----------------------------------------------------------------------------
 *  renderModelScanlineZBuffer
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Choisir la variante de rendu public selon le back-face culling.
 *
 *  Paramètres :
 *    [entrée] model valide et vue préparée.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Avec culling, chemin sans biais ; sans culling, chemin qui rapproche
 *    les faces plane_d>0. La version scanline active est V3 ; V2 et les
 *    anciennes versions restent hors de ce dispatch.
 *
 * ----------------------------------------------------------------------------
 */
void renderModelScanlineZBuffer(Model3D* model)
{
    /* Sélectionner le noyau avec la politique de biais correspondant à la
     * visibilité.
     */
    if (cull_back_faces)
        renderModelScanlineZBuffer_fastV3(model);
    else
        renderModelScanlineZBuffer_biasedV3(model);
}
