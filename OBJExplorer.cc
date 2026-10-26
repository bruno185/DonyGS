/*
 * ============================================================================
 *  ObjExplorer.cc — Explorateur OBJ — Programme principal
 * ============================================================================
 *
 *  Rôle :
 *    Piloter le chargement des modèles, la préparation des vues et les
 *    commandes de l'explorateur Apple IIgs. Le programme permet de comparer
 *    les méthodes de classement, les Z-buffers et les outils d'inspection.
 *
 *  Organisation :
 *    La fonction main distingue le chargement, le retraitement géométrique,
 *    le redessin, l'attente clavier et la libération par des labels. Les
 *    vues textuelles temporaires réutilisent la sauvegarde SHGR pour éviter
 *    un nouveau rendu.
 *
 *  Dépendances et contraintes :
 *    Inclut engine.c, qui rassemble les autres sources dans une même unité
 *    ORCA/C. Le clavier peut associer Open-Apple à un caractère. Les chaînes
 *    visibles de l'interface sont conservées. Hommage à Robert Dony, auteur
 *    de Calcul des parties cachées (Masson, 1986).
 *
 * ============================================================================
 */
#pragma memorymodel 1
#pragma optimize 1

#include "engine.c"


segment "main";

/*
 * ----------------------------------------------------------------------------
 *  main
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Piloter le chargement, les vues graphiques et les commandes de
 *    l'explorateur.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    0 à la sortie normale ; 1 sur les chemins d'erreur traités ici.
 *
 *  Fonctionnement :
 *    Possède le modèle et coordonne QuickDraw, palettes, sauvegarde SHGR et
 *    Z-buffers. Les labels séparent chargement (newmodel), retraitement
 *    géométrique (bigloop), redessin (loopReDraw/draw), attente clavier
 *    (getakey) et libération (end). Les données numériques restent dans le
 *    moteur ; les chaînes visibles de l'interface ne sont pas traduites dans
 *    cette révision des commentaires.
 *
 * ----------------------------------------------------------------------------
 */
    int main() {
        Model3D* model = NULL;
        ObserverParams params;
        char filename[100];

        int colorpalette = 0;
        long last_process_time_start = 0;
        long last_process_time_end = 0;
        long last_ZB_FS_render_time = 0;
        long last_ZB_SL_render_time = 0;
        int show_inconclusive = 0;

        int  oa;


        /* Réserver la sauvegarde SHGR avant toute vue graphique ; arrêter si
         * cette ressource obligatoire manque.
         */
        if (!initScreenSaveBuffer()) {
            printf("Error: Unable to initialize screen save buffer\n");
            keypress();
            destroyModel3D(model);
            return 1;
        }

    /* Créer une session de chargement : bannière, nouveau modèle et
     * ressources graphiques réutilisables.
     */
    newmodel:
        {
            const char* title = "3D OBJ file explorer (version 1.0)";
            int title_len = sizeof("3D OBJ file explorer (version 1.0)") - 1;
            int total_width = 80;
            int border_len = title_len + 6;
            if (border_len > total_width) border_len = total_width;
            int border_pad = (total_width - border_len) / 2;
            int title_pad = (total_width - title_len) / 2;
            char border[81];
            for (int i = 0; i < border_len; ++i) border[i] = '=';
            border[border_len] = '\0';
            printf("%*s%s\n", border_pad, "", border);
            printf("%*s%s\n", title_pad, "", title);
            printf("%*s%s\n\n", border_pad, "", border);
        }
        printf("A tribute to Robert DONY\n");
        printf("Author of \"Calcul des parties cachees\" (Masson, 1986)\n\n");


        /* Allouer les tableaux du modèle et traiter les échecs avant de poursuivre. */
        model = createModel3D();
        if (model == NULL) {
            printf("Error: Unable to allocate memory for 3D model\n");
            printf("Press any key to quit...\n");
            keypress();
            ZBuffer_Shutdown();
            Offscreen_Shutdown();
            return 1;
        }


        /* Initialiser projection, diagnostics et palettes pour ce chargement. */
        s_global_proj_scale_fixed = INT_TO_FIXED(100);


        inconclusive_pairs_count = 0;


        initPalettes();

        /* Initialiser le Z-buffer ; tenter aussi le framebuffer offscreen
         * sans arrêter ici en cas d'échec.
         */
        int zresult =ZBuffer_Init();


        if (!Offscreen_Init()) {


        }


        /* Demander un nom de fichier jusqu'au chargement réussi ; une saisie
         * vide quitte le programme.
         */
        while (1) {
            printf("Enter the filename to read (ENTER to exit): ");
            if (fgets(filename, sizeof(filename), stdin) != NULL) {
                size_t len = strlen(filename);
                if (len > 0 && filename[len-1] == '\n') {
                    filename[len-1] = '\0';
                }
            } else {

                printf("\nInput error or EOF. Exiting.\n");
                destroyModel3D(model);
                ZBuffer_Shutdown();
                Offscreen_Shutdown();
                return 1;
            }

            if (filename[0] == '\0') {
                printf("No filename entered. Exiting.\n");
                destroyModel3D(model);
                ZBuffer_Shutdown();
                Offscreen_Shutdown();
                return 0;
            }


            /* Après un échec, essayer le suffixe .obj si absent ; recréer le
             * modèle avant une nouvelle tentative.
             */
            if (loadModel3D(model, filename) < 0) {


                size_t flen = strlen(filename);
                int already_has_ext = (flen >= 4 && strcmp(filename + flen - 4, ".obj") == 0);
                int retry_failed = 1;

                if (!already_has_ext) {
                    char filename_ext[256];
                    if (flen + 4 < sizeof(filename_ext)) {
                        snprintf(filename_ext, sizeof(filename_ext), "%s.obj", filename);
                        destroyModel3D(model);
                        model = createModel3D();
                        if (model == NULL) {
                            printf("Error: Unable to allocate memory for 3D model after failed load. Exiting.\n");
                            printf("Press any key to quit...\n");
                            keypress();
                            ZBuffer_Shutdown();
                            Offscreen_Shutdown();
                            return 1;
                        }
                        if (loadModel3D(model, filename_ext) >= 0) {
                            retry_failed = 0;
                        }
                    }
                }

                /* Abandonner cette tentative et revenir à une structure de modèle vierge. */
                if (retry_failed) {
                    printf("\nError loading file '%s'. Please try again.\n", filename);

                    destroyModel3D(model);
                    model = createModel3D();
                    if (model == NULL) {
                        printf("Error: Unable to allocate memory for 3D model after failed load. Exiting.\n");
                        printf("Press any key to quit...\n");
                        keypress();
                        ZBuffer_Shutdown();
                        Offscreen_Shutdown();
                        return 1;
                    }
                    continue;
                }
            }

            printf("\nPerform 3D face intersection check? (y/N) ");
            fflush(stdout);
            /* Proposer le diagnostic d'intersections 3D et consommer le
             * reste de la ligne de réponse.
             */
            int cc = getchar();
            int reply = cc;
            while (cc != '\n' && cc != EOF) cc = getchar();
            if (reply == 'y' || reply == 'Y') {
                check_intersect(model);
            }
            break;
        }


        /* Appliquer les suggestions d'ajustement, puis permettre à
         * l'utilisateur de remplacer les paramètres.
         */
        params.angle_h = 30; params.angle_v = 20; params.angle_w = 0;
        if (model != NULL && model->auto_fit_ready) {
            params.distance = model->auto_suggested_distance;
            model->auto_proj_scale = model->auto_suggested_proj_scale;
            s_global_proj_scale_fixed = model->auto_suggested_proj_scale;
            model->auto_fit_applied = 1;
            printf("Auto-fit applied at load (distance=%.4f proj_scale=%.2f)\n", FIXED_TO_FLOAT(model->auto_suggested_distance), FIXED_TO_FLOAT(model->auto_suggested_proj_scale));


            /* Normaliser les métadonnées de distance et l'échelle du modèle
             * selon la formule existante.
             */
            normalizeAutoFitDistanceTo150(model);

            params.distance = model->auto_suggested_distance;
        } else {
            params.distance = FLOAT_TO_FIXED(30.0);
        }

        getObserverParams(&params, model);


    /* Recalculer transformation, métriques et ordre ; démarrer
     * temporairement QuickDraw pour les méthodes de classement qui utilisent ses régions.
     */
    bigloop:


        {
            Handle qd_dp = NULL;
            Word qd_id = 0;
            Word qd_err = 0;
            int qd_started = 0;
            int needs_qd = (painter_mode == PAINTER_MODE_NEWELL_SANCHA ||
                            painter_mode == PAINTER_MODE_GEO ||
                            painter_mode == PAINTER_MODE_GEOV3);

            /* Réserver la page directe Toolbox et gérer les erreurs de
             * démarrage sans laisser le modèle alloué.
             */
            if (needs_qd && !framePolyOnly) {
                qd_id = userid();
                qd_dp = NewHandle(0x300L, qd_id, 0xC015, NULL);
                qd_err = toolerror();
                if (qd_dp == NULL || qd_err != 0) {
                    printf("QuickDraw direct-page allocation failed: $%04X\n", qd_err);
                    if (qd_dp != NULL) DisposeHandle(qd_dp);
                    destroyModel3D(model);
                    ZBuffer_Shutdown();
                    Offscreen_Shutdown();
                    return 1;
                }

                QDStartUp((Word)*qd_dp, 0, mode320, qd_id);
                qd_err = toolerror();
                if (qd_err != 0) {
                    printf("QDStartUp failed: $%04X\n", qd_err);
                    DisposeHandle(qd_dp);
                    destroyModel3D(model);
                    ZBuffer_Shutdown();
                    Offscreen_Shutdown();
                    return 1;
                }
                qd_started = 1;
                shgroff();
            }


            /* Mesurer uniquement le traitement du modèle, hors démarrage et
             * arrêt temporaires de QuickDraw.
             */
            last_process_time_start = GetTick();
            printf("Processing model...\n");
            processModelFast(model, &params, filename);
            last_process_time_end = GetTick();

            /* Fermer QuickDraw temporaire et libérer sa page directe avant
             * le démarrage graphique normal.
             */
            if (qd_started) {
                QDShutDown();
                qd_err = toolerror();
                DisposeHandle(qd_dp);
                if (qd_err != 0) {
                    printf("QDShutDown failed: $%04X\n", qd_err);
                    destroyModel3D(model);
                    ZBuffer_Shutdown();
                    Offscreen_Shutdown();
                    return 1;
                }
            }
        }

    /* Redessiner à partir de la vue déjà calculée ; ne pas retrier sauf
     * commande revenant à bigloop.
     */
    loopReDraw:
        {
        int key = 0;
        int r = 0;

        if (model->faces.face_count > 0) {

            startgraph(mode);


            /* Capturer une fois la palette système avant d'appliquer la palette choisie. */
            if (!palette0_loaded) {
                palettes[0] = ReadPalette(0);
                palette0_loaded = 1;
            }


            applyPalette(palette);

            /* Dessiner les faces dans l'ordre courant, puis les
             * superpositions optionnelles.
             */
            draw:
            if (jitter) drawPolygons_jitter(model, model->faces.vertex_count, model->faces.face_count, model->vertices.vertex_count);

            else drawPolygons(model, model->faces.vertex_count, model->faces.face_count, model->vertices.vertex_count);


            if (colorpalette == 1) {
                DoColor();
            }


            if (show_inconclusive && inconclusive_pairs_count > 0) {
                frameInconclusivePairs(model);
            }


        /* Lire le caractère et le modificateur Open-Apple dans les deux
         * octets du résultat.
         */
        getakey:
        r = getkeypress_openA();
        key   = r & 0x00FF;
        oa = (r >> 8) & 1;


        /* Traiter d'abord les commandes qui restent dans la vue graphique active. */
        switch (key) {
            /* Sauvegarder une capture SHGR sans recalculer ni redessiner le modèle. */
            case '*': /* Touche : * */
                MoveTo(3, 10);
                saveNextScreenshot();
                goto getakey;
            /* Changer la palette matérielle directement ; la géométrie reste valide. */
            case 'G': /* Touche : G */
            case 'g': /* Touche : g */
                palette = (palette + 1) % 16;
                applyPalette(palette);
                goto getakey;
                break;

            /* Choisir le remplissage puis redessiner avec le nouvel état de couleur. */
            case '>': /* Touche : > */

                screen2Black();
                int c;
                if (user_fill_color < 0) c = COL_FILL_DEFAULT;
                else
                c = user_fill_color;
                c = colorChooser(0, &palette, c);
                if (c >= 0) user_fill_color = c;
                if (c == 16) {generate_random_colors(model->faces.face_count);}

                screen2Black();
                goto draw;

            /* Choisir le contour, y compris le choix spécial identique au remplissage. */
            case '<': /* Touche : < */
                screen2Black();
                c = user_frame_color;
                if (c < 0) c = COL_FRAME;
                c = colorChooser(1, &palette, c);
                if (c >= 0) user_frame_color = c;
                if (c == 16) {generate_random_colors(model->faces.face_count);}
                screen2Black();
                goto draw;


        }


        /* Sauvegarder la mémoire SHGR avant les vues textuelles ou les rendus
         * alternatifs qui en ont besoin.
         */
        if (key == ' ' || key == 'H' || key == 'h'|| key == 'F'|| key == 'f'|| key == 'M'
            || key == 'm'|| key == 'O' || key == 'o' || key == 'U' || key == 'u') {
        saveScreen();

        }

        endgraph();
        DoText();
        }


        switch (key) {
            /* Afficher les paramètres, options et temps puis restaurer
             * l'image sauvegardée.
             */
            case 32: /* Touche : Espace */
                printf("===================================\n");
                printf(" Model information and parameters\n");
                printf("===================================\n");
                printf("Model: %s\n", filename);
                printf("Vertices: %d, Faces: %d\n", model->vertices.vertex_count, model->faces.face_count);
                printf("Observer Parameters:\n");
                printf("    Distance: %.2f\n", FIXED_TO_FLOAT(params.distance));
                printf("    Horizontal Angle: %d deg\n", params.angle_h);
                printf("    Vertical Angle: %d deg\n", params.angle_v);
                printf("    Screen Rotation Angle: %d deg\n", params.angle_w);
                printf("    Projection scale: %.2f\n", FIXED_TO_FLOAT(s_global_proj_scale_fixed));
                if (painter_mode == PAINTER_MODE_FAST) printf("    Painter mode: FAST (simple face sorting only)\n");
                else if (painter_mode == PAINTER_MODE_BUBBLE_SORT) printf("    Painter mode: BUBBLE SORT (Fixed32/64)\n");
                else if (painter_mode == PAINTER_MODE_NEWELL_SANCHA) printf("    Painter mode: NEWELL SANCHA (full tests, Fixed32/64)\n");
                else if (painter_mode == PAINTER_MODE_CORRECT) printf("    Painter mode: CORRECT (painter_correct)\n");
                else if (painter_mode == PAINTER_MODE_GEO) printf("    Painter mode: GEO (geometry-only)\n");
                else if (painter_mode == PAINTER_MODE_CORRECTV2) printf("    Painter mode: CORRECT V2 (painter_correctV2 with face splitting detection)\n");
                else if (painter_mode == PAINTER_MODE_GEOV3) printf("    Painter mode: GEO V3 (see R. Dony book, without face splitting)\n");
                else printf("    Painter mode: FLOAT (float-based)\n\n");
                printf("    Back-face culling: %s\n", cull_back_faces ? "ON" : "OFF");
                printf("    Pan offset: (%d, %d)\n", pan_dx, pan_dy);
                if (shaded_by_orientation) printf("    Shading mode: orientation-based\n");
                if (user_fill_color == 16) printf("    Fill color: Random\n");
                else if (user_fill_color >= 0) printf("    Fill color: %d\n", user_fill_color);
                else printf("    Fill color: Default (COL_FILL_DEFAULT (#14))\n");
                if (user_frame_color == 16) printf("    Frame color: Random\n");
                else if (user_frame_color >= 0) printf("    Frame color: %d\n", user_frame_color);
                else printf("    Frame color: Default (COL_FRAME (#7))\n");
                printf ("Processing time: %ld ticks (1/60 sec.)\n", last_process_time_end - last_process_time_start);
                printf("Last Z-Buffer fullscreen render time: %ld ticks\n", last_ZB_FS_render_time);
                printf("Last Z-Buffer scanline render time: %ld ticks\n", last_ZB_SL_render_time);
                printf("===================================\n");
                printf("\n");
                printf("Free memory = %lu bytes\n", FreeMem());
                printf("Press any key to continue...\n");
                keypress();


                startgraph(mode);
                restoreScreen();
                goto getakey;


            /* Inspecter les échanges GEO V3 depuis la permutation courante. */
            case 59: /* Touche : ; */
                if (model == NULL) { printf("No model loaded\n"); goto loopReDraw; }
                repair_geov3(model);
                goto loopReDraw;

            /* Augmenter la projection de 10 %, borner l'échelle et
             * recalculer seulement les coordonnées projetées et leurs
             * bornes.
             */
            case 43: /* Touche : + */
                if (model != NULL) {


                    Fixed32 cur = s_global_proj_scale_fixed;
                    Fixed32 mul = FLOAT_TO_FIXED(1.1f);
                    Fixed32 new_scale = FIXED_MUL_64(cur, mul);

                    Fixed32 min_scale = FLOAT_TO_FIXED(1.0f);
                    Fixed32 max_scale = FLOAT_TO_FIXED(10000.0f);
                    if (new_scale < min_scale) new_scale = min_scale;
                    if (new_scale > max_scale) new_scale = max_scale;
                    s_global_proj_scale_fixed = new_scale;

                    model->auto_proj_scale = s_global_proj_scale_fixed;
                    model->auto_fit_applied = 1;
                    printf("Projection scale increased -> %.2f\n", FIXED_TO_FLOAT(s_global_proj_scale_fixed));
                } else {
                    printf("No model loaded.\n");
                }
                compute2DFromObserver(model, params.angle_w);
                goto loopReDraw;

            /* Réduire la projection de 10 % avec les mêmes bornes ; ne pas
             * modifier la distance observateur.
             */
            case 45: /* Touche : - */
                if (model != NULL) {

                    Fixed32 cur = s_global_proj_scale_fixed;
                    Fixed32 mul = FLOAT_TO_FIXED(0.9f);
                    Fixed32 new_scale = FIXED_MUL_64(cur, mul);
                    Fixed32 min_scale = FLOAT_TO_FIXED(1.0f);
                    Fixed32 max_scale = FLOAT_TO_FIXED(10000.0f);
                    if (new_scale < min_scale) new_scale = min_scale;
                    if (new_scale > max_scale) new_scale = max_scale;
                    s_global_proj_scale_fixed = new_scale;
                    model->auto_proj_scale = s_global_proj_scale_fixed;
                    model->auto_fit_applied = 1;
                    printf("Projection scale decreased -> %.2f\n", FIXED_TO_FLOAT(s_global_proj_scale_fixed));
                } else {
                    printf("No model loaded.\n");
                }
                compute2DFromObserver(model, params.angle_w);
                goto loopReDraw;

            /* Rapprocher l'observateur de 10 %, ou de 50 % avec Open-Apple,
             * puis recalculer toute la vue.
             */
            case 65: /* Touche : A */
            case 97: /* Touche : a */
                if (!oa) params.distance = params.distance - (params.distance / 10);
                else params.distance = params.distance - (params.distance / 2);
                printf("Distance decreased -> %.2f\n", FIXED_TO_FLOAT(params.distance));
                goto bigloop;

            /* Éloigner l'observateur avec le même choix de pas. */
            case 90: /* Touche : Z */
            case 122: /* Touche : z */
                if (!oa) params.distance = params.distance + (params.distance / 10);
                else params.distance = params.distance + (params.distance / 2);
                printf("Distance increased -> %.2f\n", FIXED_TO_FLOAT(params.distance));
                goto bigloop;

            /* Tourner horizontalement de +10 degrés, ou +1 avec Open-Apple. */
            case 21: /* Touche : flèche droite */
                if (oa == 0) {
                    params.angle_h = normalize_deg(params.angle_h + 10);
                } else {
                    params.angle_h = normalize_deg(params.angle_h + 1);
                }
                goto bigloop;

            /* Tourner horizontalement de -10 degrés, ou -1 avec Open-Apple. */
            case 8: /* Touche : flèche gauche */
                if (oa == 0) {
                    params.angle_h = normalize_deg(params.angle_h - 10);
                } else {
                    params.angle_h = normalize_deg(params.angle_h - 1);
                }
                goto bigloop;

            /* Réduire l'angle vertical de 10 degrés, ou de 1 avec Open-Apple. */
            case 10: /* Touche : flèche bas */
                if (oa == 0) {
                    params.angle_v = normalize_deg(params.angle_v - 10);
                } else {
                    params.angle_v = normalize_deg(params.angle_v - 1);
                }
                goto bigloop;

            /* Augmenter l'angle vertical avec le même choix de pas. */
            case 11: /* Touche : flèche haut */
                if (oa == 0) {
                    params.angle_v = normalize_deg(params.angle_v + 10);
                } else {
                    params.angle_v = normalize_deg(params.angle_v + 1);
                }
                goto bigloop;

            /* Augmenter la rotation dans le plan de l'écran ; recalculer les
             * coordonnées observateur.
             */
            case 87: /* Touche : W */
            case 119: /* Touche : w */
                if (oa == 0) {
                    params.angle_w = normalize_deg(params.angle_w + 10);
                } else {
                    params.angle_w = normalize_deg(params.angle_w + 1);
                }
                goto bigloop;

            /* Réduire la rotation dans le plan de l'écran. */
            case 88: /* Touche : X */
            case 120: /* Touche : x */
                if (oa == 0) {
                    params.angle_w = normalize_deg(params.angle_w - 10);
                } else {
                    params.angle_w = normalize_deg(params.angle_w - 1);
                }
                goto bigloop;


            /* Déplacer le dessin vers la gauche de dix pixels, ou d'un pixel
             * avec Open-Apple ; redessin seul.
             */
            case 69: /* Touche : E */
            case 101: /* Touche : e */
                if (!oa) pan_dx -= 10;
                else pan_dx -= 1;
                printf("Pan offset -> (%d,%d)\n", pan_dx, pan_dy);
                goto loopReDraw;

            /* Déplacer le dessin vers la droite avec le même choix de pas. */
            case 82: /* Touche : R */
            case 114: /* Touche : r */
                if (!oa) pan_dx += 10;
                else pan_dx += 1;
                printf("Pan offset -> (%d,%d)\n", pan_dx, pan_dy);
                goto loopReDraw;


            /* Déplacer le dessin vers le haut en réduisant pan_dy. */
            case 84: /* Touche : T */
            case 116: /* Touche : t */
                if (!oa) pan_dy -= 10;
                else pan_dy -= 1;
                printf("Pan offset -> (%d,%d)\n", pan_dx, pan_dy);
                goto loopReDraw;

            /* Déplacer le dessin vers le bas en augmentant pan_dy. */
            case 89: /* Touche : Y */
            case 121: /* Touche : y */
                if (!oa) pan_dy += 10;
                else pan_dy += 1;
                printf("Pan offset -> (%d,%d)\n", pan_dx, pan_dy);
                goto loopReDraw;

            /* Annuler les deux décalages écran. */
            case 48: /* Touche : 0 */
                pan_dx = 0; pan_dy = 0;
                printf("Pan reset -> (%d,%d)\n", pan_dx, pan_dy);
                goto loopReDraw;

            /* Afficher ou masquer la palette superposée. */
            case 67: /* Touche : C */
            case 99: /* Touche : c */
                colorpalette ^= 1;
                goto loopReDraw;

            /* Les commandes G/g sont déjà consommées par le premier switch ;
             * ces labels partagent ici le chemin du shading.
             */
            case 71: /* Touche : G */
            case 103: /* Touche : g */


            /* Activer ou désactiver les couleurs issues de l'orientation et redessiner. */
            case 33: /* Touche : ! */
                shaded_by_orientation ^= 1;
                printf("Orientation shading: %s\n", shaded_by_orientation ? "ON" : "OFF");
                if (shaded_by_orientation && model != NULL) {
                    computeOrientationShading(model);
                }
                goto loopReDraw;

            /* Afficher ou masquer les cadres des paires inconclusives. */
            case 73: /* Touche : I */
            case 105: /* Touche : i */
                show_inconclusive ^= 1;
                printf("Inconclusive pairs display: %s\n", show_inconclusive ? "ON" : "OFF");
                goto loopReDraw;

            /* Basculer le rendu avec décalages jitter. */
            case 74: /* Touche : J */
            case 106: /* Touche : j */
                jitter ^= 1;
                printf("Jitter rendering: %s\n", jitter ? "ON" : "OFF");
                goto loopReDraw;

            /* Ouvrir l'inspection des faces qui précèdent la face étudiée. */
            case 68: /* Touche : D */
            case 100: /* Touche : d */
                if (model == NULL) { printf("No model loaded\n"); goto loopReDraw; }
                inspect_faces_order(model, &params, 0);
                goto loopReDraw;

            /* Ouvrir l'inspecteur de face et ses commandes de visibilité et de winding. */
            case 86: /* Touche : V */
            case 118: /* Touche : v */
                if (model == NULL) { printf("No model loaded\n"); goto loopReDraw; }
                showFace(model, &params, filename);
                goto loopReDraw;

            /* Inspecter les faces placées après la cible. */
            case 83: /* Touche : S */
            case 115: /* Touche : s */
                if (model == NULL) { printf("No model loaded\n"); goto loopReDraw; }
                inspect_faces_order(model, &params, 1);
                goto loopReDraw;

            /* Afficher les identifiants des faces sur le modèle. */
            case 76: /* Touche : L */
            case 108: /* Touche : l */
                if (model == NULL) { printf("No model loaded\n"); goto loopReDraw; }
                display_model_face_ids(model, &params, filename);
                goto loopReDraw;

            /* Choisir FAST et vider les paires inconclusives avant retraitement. */
            case 49: /* Touche : 1 */
                painter_mode = PAINTER_MODE_FAST;
                printf("Painter mode: FAST (simple face sorting only)\n");
                inconclusive_pairs_count = 0;
                if (model != NULL) { printf("Reprocessing model with current mode...\n"); goto bigloop; }

            /* Choisir la méthode de classement BUBBLE SORT. */
            case 50: /* Touche : 2 */
                painter_mode = PAINTER_MODE_BUBBLE_SORT;
                printf("Painter mode: BUBBLE SORT (full tests, Fixed32/64)\n");
                if (model != NULL) { printf("Reprocessing model with current mode...\n"); goto bigloop; }

            /* Choisir la méthode de classement NEWELL SANCHA. */
            case 51: /* Touche : 3 */
                painter_mode = PAINTER_MODE_NEWELL_SANCHA;
                printf("Painter mode: NEWELL SANCHA (full tests, Fixed32/64)\n");
                if (model != NULL) { printf("Reprocessing model with current mode...\n"); goto bigloop; }

            /* Choisir GEO, implémenté par painter_geoV2. */
            case 52: /* Touche : 4 */
                painter_mode = PAINTER_MODE_GEO;
                printf("Painter mode: GEO (geometry-only)\n");
                printf("WARNING: This mode can be significantly slower than others with large models.\n");
                if (model != NULL) { printf("Reprocessing model with current mode...\n"); goto bigloop; }

            /* Choisir GEO V3, implémenté dans dony.c. */
            case 53: /* Touche : 5 */
                painter_mode = PAINTER_MODE_GEOV3;
                printf("Painter mode: GEO V3 (inspired by R. DONY's book)\n");
                printf("WARNING: This mode can be significantly slower than others with large models.\n");
                goto bigloop;
                goto loopReDraw;

            /* Choisir la première correction locale d'ordre. */
            case 54: /* Touche : 6 */
                painter_mode = PAINTER_MODE_CORRECT;
                printf("Painter mode: CORRECT (painter_correct)\n");
                if (model != NULL) { printf("Reprocessing model with current mode...\n"); goto bigloop; }

            /* Choisir la seconde correction locale ; le libellé de
             * l'interface ne prouve pas un découpage effectif.
             */
            case 55: /* Touche : 7 */
                painter_mode = PAINTER_MODE_CORRECTV2;
                printf("Painter mode: CORRECT V2 (painter_correctV2)\n");
                if (model != NULL) { printf("Reprocessing model with current mode...\n"); goto bigloop; }


            /* Générer des couleurs de remplissage aléatoires ; le choix du
             * contour reste inchangé.
             */
            case 56: /* Touche : 8 */
                user_fill_color = 16;

                if (model != NULL) {
                    generate_random_colors(model->faces.face_count);
                }
                printf("Colors set to RANDOM mode (new colors generated)\n");
                goto loopReDraw;


            /* Rétablir les couleurs par défaut, la palette système et le
             * shading désactivé.
             */
            case 57: /* Touche : 9 */
                user_fill_color = -1;
                user_frame_color = -1;
                palette = 0;
                shaded_by_orientation = 0;
                printf("Colors reset to defaults, palette reset to 0, shading OFF\n");
                goto loopReDraw;


            /* Rendre la vue courante avec le Z-buffer scanline et mémoriser
             * son temps en ticks.
             */
            case 79: /* Touche : O */
            case 111: /* Touche : o */
                startgraph(mode);
                MoveTo(3, 10);
                printf("Z-Buffer scanline rendering...\n");
                long startTime = GetTick();
                renderModelScanlineZBuffer(model);
                long endTime = GetTick();
                last_ZB_SL_render_time = endTime - startTime;
                goto getakey;

            /* Rendre par Z-buffer plein écran : buffer offscreen par défaut,
             * écran direct avec Open-Apple.
             */
            case 'U': /* Touche : U */
            case 'u': /* Touche : u */
                startgraph(mode);
                MoveTo(3, 10);
                if (!oa) printf("Z-Buffer fullscreen rendering (offscreen)...\n");
                else printf("Z-Buffer fullscreen rendering (onscreen)...\n");
                startTime = GetTick();
                if (!oa) {
                    Offscreen_Clear();
                    renderModelFullscreenZBuffer_offscreen(model);
                    Offscreen_FlushToScreen();
                }
                else {
                    renderModelFullscreenZBuffer(model);
                }

                endTime = GetTick();
                last_ZB_FS_render_time = endTime - startTime;
                goto getakey;


            /* Basculer entre filaire et surfaces ; le retour aux surfaces
             * impose un nouveau tri.
             */
            case 80: /* Touche : P */
            case 112: /* Touche : p */
                framePolyOnly ^= 1;
                printf("Frame-only polygons: %s\n", framePolyOnly ? "ON" : "OFF");
                if (!framePolyOnly) {

                    printf("Switching to filled mode: reprocessing model (sorting faces)...\n");
                    goto bigloop;
                }
                else {

                    goto loopReDraw;
                }

            /* Basculer le back-face culling puis recalculer les métriques et l'ordre. */
            case 66: /* Touche : B */
            case 98: /* Touche : b */
                cull_back_faces ^= 1;
                printf("Back-face culling: %s\n", cull_back_faces ? "ON" : "OFF");
                if (model != NULL) {
                    printf("Reprocessing model with culling %s...\n", cull_back_faces ? "ON" : "OFF");
                    goto bigloop;
                }


            /* Exporter plans, coordonnées projetées et ordre courant dans
             * trois fichiers, puis restaurer l'écran.
             */
            case 70: /* Touche : F */
            case 102: /* Touche : f */
                if (model != NULL) {
                    printf("Writing files...\n");


                    dumpFaceEquationsCSV(model, "Faces3D.csv", 1);

                    dumpFace2DCoordinates(model, "Faces2D.txt");

                    dumpSortedFaceIndices(model, "FacesOrder.txt");
                    printf("\nPress any key to continue...\n");
                    keypress();
                }


                startgraph(mode);
                restoreScreen();
                goto getakey;


            /* Libérer le modèle et réinitialiser les options explicitement
             * listées avant un nouveau chargement.
             */
            case 78: /* Touche : N */
            case 110: /* Touche : n */

                painter_mode = PAINTER_MODE_FAST;
                pan_dx = 0; pan_dy = 0;
                cull_back_faces = 1;
                user_fill_color = -1;
                user_frame_color = -1;
                s_global_proj_scale_fixed = INT_TO_FIXED(100);
                jitter = 0;
                destroyModel3D(model);
                goto newmodel;

            /* Modifier les paramètres de vue sans relire le fichier OBJ. */
            case 75: /* Touche : K */
            case 107: /* Touche : k */
                getObserverParams(&params, model);
                printf("Observer parameters updated.\n");
                goto bigloop;


            /* Afficher l'aide paginée puis restaurer l'image sauvegardée. */
            case 72: /* Touche : H */
            case 104: /* Touche : h */
                show_help_pager();


                startgraph(mode);
                restoreScreen();
                goto getakey;


                goto loopReDraw;

            /* Ouvrir l'inspecteur interactif de paires de faces. */
            case 81: /* Touche : Q */
            case 113: /* Touche : q */
                if (model == NULL) { printf("No model loaded\n"); goto loopReDraw; }
                inspect_face_pair_ui(model);
                goto loopReDraw;

            /* Comparer les méthodes de chevauchement avec le benchmark intégré. */
            case 47: /* Touche : / */
                if (model == NULL) { printf("No model loaded\n"); goto loopReDraw; }

                printf("Running projected overlap benchmark...\n");

                benchmark_projected_overlap_all(model, &params, 1);
                keypress();
                goto loopReDraw;

                /* Changer la méthode de chevauchement puis retraiter le modèle. */
                case 40: /* Touche : ( */
                overlap_method = select_overlap_method(overlap_method);
                goto bigloop;

            /* Quitter la boucle et libérer les ressources suivies par le programme. */
            case 27: /* Touche : Échap */
                goto end;


            default:
                goto loopReDraw;

        }
        }

        /* Déverrouiller et libérer le polygone QuickDraw, le modèle et les
         * buffers des rendus alternatifs.
         */
        end:


        if (globalPolyHandle != NULL) {
            if (poly_handle_locked) {
                HUnlock(globalPolyHandle);
            }
            DisposeHandle(globalPolyHandle);
            globalPolyHandle = NULL;
        }
        destroyModel3D(model);
        ZBuffer_Shutdown();
        Offscreen_Shutdown();
        return 0;
    }

