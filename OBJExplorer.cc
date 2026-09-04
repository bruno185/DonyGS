
/*
 * ============================================================================
 *              OBJExplorer.cc - Fixed32 Optimized Implementation
 * ============================================================================
 *
 * Purpose:
 *   High-performance Apple IIGS 3D OBJ viewer and explorer implementing the painter's family
 *   of algorithms with multiple painter modes (FAST / FIXED / CORRECT / CORRECTV2).
 *   Reads simplified OBJ files (vertices "v" and faces "f"), transforms them into observer space,
 *   projects to 2D screen coordinates and renders filled polygons.
 *
 * Highlights (2026):
 *   - Target: Apple IIGS (ORCA/C, QuickDraw)
 *   - Painter modes:
 *       * FAST  : z_mean + bbox tests (very fast, less robust)
 *       * FIXED : full fixed-point Newell/Sancha implementation with pairwise
 *                 tests and corrections (robust)
 *       * GEO   : Geometry-only painter mode for heuristic plane-based ordering
 *       * CORRECT : Advanced ordering correction with local face reordering
 *       * CORRECTV2 : Experimental local correction (painter_correctV2) for
 *                   pathological cases
 *   - Interactive debug helpers: `M` for `pair_plane_before` diagnostics, `Q`
 *     for interactive face-pair inspection, `D` / `S` for before/after inspection,
 *     and `I` for inconclusive pair display.
 *   - Observer-space back-face culling toggle (`B` key) that marks faces as
 *     non-displayable and restricts sorting to visible faces for correctness
 *     and speed when enabled.
 *   - Diagnostic helpers: inconclusive pair recording, `frameInconclusivePairs()`
 *     for visual debugging, and timing instrumentation to measure stages.
 *   - Performance: heavy Fixed32 optimizations, precomputed trig tables, buffer
 *     reuse to avoid allocations and selective sorting when culling is active.
 *
 * Responsibilities (per-frame):
 *   - Transform vertices (Fixed32) to observer-space (xo/yo/zo).
 *   - Project to integer screen coordinates (x2d/y2d).
 *   - Compute per-face metrics: z_min/z_max/z_mean, bbox, plane coefficients.
 *   - Dispatch to the selected painter to populate `sorted_face_indices`.
 *   - Provide debug overlays (frame inconclusive pairs) and logging.
 *
 * Notes:
 *   - This file has evolved beyond the original Apple IIGS reference to include
 *     platform-specific frontends while preserving the algorithmic core.
 *   - Experimental: `painter_correctV2` and the in-memory face-splitting logic
 *     are work-in-progress and may be unstable on complex models; use with
 *     caution and enable only for debugging/analysis.
 *   - The code is optimized for interactive use; use the FAST painter for
 *     high frame-rate, or FIXED/CORRECT modes for correctness on tricky geometry.
 *     (FLOAT is archived in `chutier.txt`.)
 *
 * Author: Bruno
 * Date: 2026-01-23
 * ============================================================================
 */

// ==============================================================
// THIS IS THE MAIN PROGRAM
// ==============================================================
//
#pragma memorymodel 1
#pragma optimize 1

#include "engine.c"


segment "code22";
    int main() {
        Model3D* model;
        ObserverParams params;
        char filename[100];
        char input[50];
        int colorpalette = 0; // default color palette
        int last_process_time_start = 0;
        int last_process_time_end = 0;
        int show_inconclusive = 0; // toggle: display inconclusive pair overlays (press 'i' to toggle)


    newmodel:
        {
            const char* title = "3D OBJ file explorer (version 0.9)";
            int title_len = sizeof("3D OBJ file explorer (version 0.9)") - 1;
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

        // Creer le modele 3D
        model = createModel3D();
        if (model == NULL) {
            printf("Error: Unable to allocate memory for 3D model\n");
            printf("Press any key to quit...\n");
            keypress();
            return 1;
        }

        /* Initialize global projection scale to a sensible default (pixels per projected unit) */
        s_global_proj_scale_fixed = INT_TO_FIXED(100);

        // Initialize inconclusive pairs counter
        inconclusive_pairs_count = 0; // clear inconclusive pairs

        // Initialize palettes and palette table storage
        initPalettes();

        // Ask for filename (loop until a non-empty filename is entered and the model loads)
        while (1) {
            printf("Enter the filename to read (ENTER to exit): ");
            if (fgets(filename, sizeof(filename), stdin) != NULL) {
                size_t len = strlen(filename);
                if (len > 0 && filename[len-1] == '\n') {
                    filename[len-1] = '\0';
                }
            } else {
                // EOF or input error - exit gracefully
                printf("\nInput error or EOF. Exiting.\n");
                destroyModel3D(model);
                return 1;
            }

            if (filename[0] == '\0') {
                printf("No filename entered. Exiting.\n");
                destroyModel3D(model);
                return 0; // Exit program when user presses ENTER with empty filename
            }

            // Try to load the model; if it fails, inform the user, reset model state, and re-prompt
            if (loadModel3D(model, filename) < 0) {
                // Retry once with a ".obj" extension appended, so the user
                // can type the filename without the extension
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
                            return 1;
                        }
                        if (loadModel3D(model, filename_ext) >= 0) {
                            retry_failed = 0;
                        }
                    }
                }

                if (retry_failed) {
                    printf("\nError loading file '%s'. Please try again.\n", filename);
                    // Destroy and recreate model to ensure clean state for next attempt
                    destroyModel3D(model);
                    model = createModel3D();
                    if (model == NULL) {
                        printf("Error: Unable to allocate memory for 3D model after failed load. Exiting.\n");
                        printf("Press any key to quit...\n");
                        keypress();
                        return 1;
                    }
                    continue;
                }
            }
            // Successfully loaded; ask the user about the overlap scan
            printf("\nPerform 3D face intersection check? (y/N) ");
            fflush(stdout);
            int cc = getchar();
            int reply = cc; /* remember first character */
            while (cc != '\n' && cc != EOF) cc = getchar();
            if (reply == 'y' || reply == 'Y') {
                check_intersect(model);
            }
            break;
        }

        // Initialize observer params defaults and apply auto-fit at load when available
        params.angle_h = 30; params.angle_v = 20; params.angle_w = 0;
        if (model != NULL && model->auto_fit_ready) {
            params.distance = model->auto_suggested_distance;
            model->auto_proj_scale = model->auto_suggested_proj_scale;
            s_global_proj_scale_fixed = model->auto_suggested_proj_scale; // sync global scale
            model->auto_fit_applied = 1;
            printf("Auto-fit applied at load (distance=%.4f proj_scale=%.2f)\n", FIXED_TO_FLOAT(model->auto_suggested_distance), FIXED_TO_FLOAT(model->auto_suggested_proj_scale));

            // Normalize the auto-fit so that the observer distance becomes 150
            // (keeps the apparent size unchanged by scaling vertices + projection scale)
            normalizeAutoFitDistanceTo150(model);
            // Ensure the observer distance used for rendering matches the normalized distance
            params.distance = model->auto_suggested_distance;
        } else {
            params.distance = FLOAT_TO_FIXED(30.0);
        }
        // Get observer parameters (user can override defaults by typing values)
        getObserverParams(&params, model);


    bigloop:
        // Process model with parameters - OPTIMIZED VERSION
            // Process model with parameters - OPTIMIZED VERSION
        last_process_time_start = GetTick();
        printf("Processing model...\n");
        // in processModelFast : if Frame-only mode is active, skip face sorting and use simple painter
        processModelFast(model, &params, filename);
        last_process_time_end = GetTick();


    loopReDraw:
        {
            int key = 0;
            char input[50];

            if (model->faces.face_count > 0) {
                // Initialize QuickDraw
                startgraph(mode);

                // Load the system palette into palettes[0] on first startup
                if (!palette0_loaded) {
                    palettes[0] = ReadPalette(0);
                    palette0_loaded = 1;
                }

                // Apply the selected palette and draw the 3D object
                applyPalette(palette);

                if (jitter) drawPolygons_jitter(model, model->faces.vertex_count, model->faces.face_count, model->vertices.vertex_count); 
                // draw model, according to current faces sorted list.
                else drawPolygons(model, model->faces.vertex_count, model->faces.face_count, model->vertices.vertex_count);
                
                // display available colors
                if (colorpalette == 1) { 
                    DoColor(); 
                }

                // If there are inconclusive pairs and display is enabled, underline them on screen
                if (show_inconclusive && inconclusive_pairs_count > 0) {
                    frameInconclusivePairs(model);  
                }
                // Wait for key press and get key code

        key = getkeypress();

        if (key == '*') {
            saveNextScreenshot(); // we need to save image to file here, before closing QuickDraw (which alters the palette)
        }

        endgraph();        // Close QuickDraw
        }

        DoText();           // Show text screen

        // Handle keyboard input with switch statement
        switch (key) {
            case 32:  // Space bar - display info and redraw
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
                else if (painter_mode == PAINTER_MODE_FIXED) printf("    Painter mode: NORMAL (Fixed32/64)\n");
                else if (painter_mode == PAINTER_MODE_CORRECT) printf("    Painter mode: CORRECT (painter_correct)\n");
                else if (painter_mode == PAINTER_MODE_GEO) printf("    Painter mode: GEO (geometry-only)\n");
                else if (painter_mode == PAINTER_MODE_CORRECTV2) printf("    Painter mode: CORRECT V2 (painter_correctV2 with face splitting detection)\n");
                else printf("    Painter mode: FLOAT (float-based)\n\n");
                printf("    Back-face culling: %s\n", cull_back_faces ? "ON" : "OFF");
                printf("    Pan offset: (%d, %d)\n", pan_dx, pan_dy);
                if (shaded_by_orientation) printf("    Shading mode: orientation-based\n");
                if (user_fill_color == 16) printf("    Fill color: Random\n");
                else if (user_fill_color >= 0) printf("    Fill color: %d\n", user_fill_color);
                else printf("    Fill color: Default (COL_FILL_DEFAULT /*14*/)\n");
                if (user_frame_color == 16) printf("    Frame color: Random\n");
                else if (user_frame_color >= 0) printf("    Frame color: %d\n", user_frame_color);
                else printf("    Frame color: Default (COL_FRAME /*7*/)\n");
                printf ("Processing time: %d ticks (1/60 sec.)\n", last_process_time_end - last_process_time_start);
                printf("===================================\n");
                printf("\n");
                printf("Press any key to continue...\n");
                keypress();
                goto loopReDraw;

            case 82:  // 'R' - pan right (was revert; revert disabled)
            case 114: // 'r'
                pan_dx += 10; // move right by 10 pixels
                printf("Pan offset -> (%d,%d)\n", pan_dx, pan_dy);
                goto loopReDraw;

            case 59: // ';' - Run check_sort_repair (repair ordering) and wait for key so user can read results
                printf("Running check_sort_repair (ray_cast verification & minimal repair)...\n");
                check_sort_repair(model, model->faces.face_count);
                printf("Press any key to continue...\n");
                keypress();
                goto loopReDraw;

            case 46: // '.' - Run check_sort_repair_fast (faster minimal repair using QD centroid)
                printf("Running check_sort_repair_fast (QD centroid + minimal repair)...\n");
                check_sort_repair_fast(model, model->faces.face_count);
                // printf("Press any key to continue...\n");
                // keypress();
                goto loopReDraw;



            case 43:  // '+' - increase projection scale by 10% (applies to current scale)
            case 61:  // '=' also acts as '+' on some keyboards
                if (model != NULL) {
                    // '+' now adjusts projection scale by +10%
                    // Compute new scale = current scale * 1.1 (fixed-point multiplication)
                    Fixed32 cur = s_global_proj_scale_fixed;
                    Fixed32 mul = FLOAT_TO_FIXED(1.1f);
                    Fixed32 new_scale = FIXED_MUL_64(cur, mul);
                    // clamp
                    Fixed32 min_scale = FLOAT_TO_FIXED(1.0f);
                    Fixed32 max_scale = FLOAT_TO_FIXED(10000.0f);
                    if (new_scale < min_scale) new_scale = min_scale;
                    if (new_scale > max_scale) new_scale = max_scale;
                    s_global_proj_scale_fixed = new_scale;
                    // sync to model field for visibility
                    model->auto_proj_scale = s_global_proj_scale_fixed;
                    model->auto_fit_applied = 1; // treat scale as explicitly set
                    printf("Projection scale increased -> %.2f\n", FIXED_TO_FLOAT(s_global_proj_scale_fixed));
                } else {
                    printf("No model loaded.\n");
                }
                compute2DFromObserver(model, params.angle_w);
                goto loopReDraw;

            case 45:  // '-' - decrease projection scale by 10% (applies to current scale)
                if (model != NULL) {
                    // '-' now adjusts projection scale by -10%
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

            case 65:  // 'A' - decrease distance
            case 97:  // 'a'
                params.distance = params.distance - (params.distance / 10);
                printf("Distance decreased -> %.2f\n", FIXED_TO_FLOAT(params.distance));
                goto bigloop;

            case 90:  // 'Z' - increase distance  
            case 122: // 'z'
                params.distance = params.distance + (params.distance / 10);
                printf("Distance increased -> %.2f\n", FIXED_TO_FLOAT(params.distance));
                goto bigloop;

            case 21:  // Right arrow - increase horizontal angle
                params.angle_h = normalize_deg(params.angle_h + 10);
                goto bigloop;

            case 8:   // Left arrow - decrease horizontal angle
                params.angle_h = normalize_deg(params.angle_h - 10);
                goto bigloop;

            case 10:  // Down arrow - decrease vertical angle
                params.angle_v = normalize_deg(params.angle_v - 10);
                goto bigloop;

            case 11:  // Up arrow - increase vertical angle
                params.angle_v = normalize_deg(params.angle_v + 10);
                goto bigloop;

            case 87:  // 'W' - increase screen rotation angle
            case 119: // 'w'
                params.angle_w = normalize_deg(params.angle_w + 10);
                goto bigloop;

            case 88:  // 'X' - decrease screen rotation angle
            case 120: // 'x'
                params.angle_w = normalize_deg(params.angle_w - 10);
                goto bigloop;

            /* 2D panning: E=left, R=right (R replaced revert), T=up, Y=down (and lowercase) */
            case 69: /* 'E' */
            case 101: /* 'e' */
                pan_dx -= 10; /* pan left */
                printf("Pan offset -> (%d,%d)\n", pan_dx, pan_dy);
                goto loopReDraw;

            case 84: /* 'T' */
            case 116: /* 't' */
                pan_dy -= 10; /* pan up = decrease Y */
                printf("Pan offset -> (%d,%d)\n", pan_dx, pan_dy);
                goto loopReDraw;

            case 89: /* 'Y' */
            case 121: /* 'y' */
                pan_dy += 10; /* pan down = increase Y */
                printf("Pan offset -> (%d,%d)\n", pan_dx, pan_dy);
                goto loopReDraw;

            case 48: /* '0' - reset pan */
                pan_dx = 0; pan_dy = 0;
                printf("Pan reset -> (%d,%d)\n", pan_dx, pan_dy);
                goto loopReDraw;
        
            case 67:  // 'C' - toggle color palette display
            case 99:  // 'c'
                colorpalette ^= 1; // Toggle between 0 and 1
                goto loopReDraw;

            case 71:  // 'G' - cycle the active QuickDraw palette
            case 103:  // 'g'
                palette++;
                if (palette >= 16) palette = 0; // Wrap around if exceeds available palettes
                goto loopReDraw;

            case 33:  // '!' - toggle orientation shading
                shaded_by_orientation ^= 1;
                printf("Orientation shading: %s\n", shaded_by_orientation ? "ON" : "OFF");
                if (shaded_by_orientation && model != NULL) {
                    computeOrientationShading(model);
                }
                goto loopReDraw;

            case 73:  // 'I' - toggle display of inconclusive face pairs
            case 105: // 'i'
                show_inconclusive ^= 1;
                printf("Inconclusive pairs display: %s\n", show_inconclusive ? "ON" : "OFF");
                goto loopReDraw;

            case 74:  // 'J' - toggle jittered rendering
            case 106: // 'j'
                jitter ^= 1;
                printf("Jitter rendering: %s\n", jitter ? "ON" : "OFF");
                goto loopReDraw;

            case 68:  // 'D' - inspect face ordering and show misplaced faces
            case 100: // 'd'
                if (model == NULL) { printf("No model loaded\n"); goto loopReDraw; }
                inspect_faces_before(model, &params, filename);
                goto loopReDraw;

            case 86:  // 'V' - show a single face in filled mode
            case 118: // 'v'
                if (model == NULL) { printf("No model loaded\n"); goto loopReDraw; }
                showFace(model, &params, filename);
                goto loopReDraw;

            case 83: // 'S' - inspect faces that are AFTER target but should be BEFORE (new)
            case 115: // 's'
                if (model == NULL) { printf("No model loaded\n"); goto loopReDraw; }
                inspect_faces_after(model, &params, filename);
                goto loopReDraw;

            case 77: // 'M' - debug pair_plane_before
            case 109: // 'm'
                if (model == NULL) { printf("No model loaded\n"); goto loopReDraw; }
                // pair_plane_before_debug(model, 0, 0);
                pair_plane_geometric_tests(model, -1, -1); // force type pair numbers
                goto loopReDraw;


            case 76: // 'L' - show model with face ID labels at polygon centers
            case 108: // 'l'
                if (model == NULL) { printf("No model loaded\n"); goto loopReDraw; }
                display_model_face_ids(model, &params, filename);
                goto loopReDraw;

            case 49: // '1' - set FAST painter
                painter_mode = PAINTER_MODE_FAST;
                printf("Painter mode: FAST (simple face sorting only)\n");
                inconclusive_pairs_count = 0; // clear inconclusive pairs in fast mode
                if (model != NULL) { printf("Reprocessing model with current mode...\n"); goto bigloop; }

            case 50: // '2' - set NORMAL (Fixed32/64) painter
                painter_mode = PAINTER_MODE_FIXED;
                printf("Painter mode: NORMAL (full tests, Fixed32/64)\n");
                if (model != NULL) { printf("Reprocessing model with current mode...\n"); goto bigloop; }

            case 51: // '3' - set GEO painter
                painter_mode = PAINTER_MODE_GEO;
                printf("Painter mode: GEO (geometry-only)\n");
                printf("WARNING: This mode can be significantly slower than others with large models.\n");
                if (model != NULL) { printf("Reprocessing model with current mode...\n"); goto bigloop; }

            case 52: // '4' - set CORRECT painter (runs painter_correct)
                painter_mode = PAINTER_MODE_CORRECT;
                printf("Painter mode: CORRECT (painter_correct)\n");
                if (model != NULL) { printf("Reprocessing model with current mode...\n"); goto bigloop; }

            case 53: // '5' - set CORRECTV2 painter (runs painter_correctV2 with face splitting detection)
                painter_mode = PAINTER_MODE_CORRECTV2;
                printf("Painter mode: CORRECT V2 (painter_correctV2)\n");
                if (model != NULL) { printf("Reprocessing model with current mode...\n"); goto bigloop; }

            case 55: // '7' - choose fill color
                {
                    printf("\n=== Fill color selection ===\n");
                    printf("Available colors:\n");
                    printf(" 0 : black\n");
                    printf(" 1 : grey\n");
                    printf(" 2 : brown\n");
                    printf(" 3 : purple\n");
                    printf(" 4 : blue\n");
                    printf(" 5 : green\n");
                    printf(" 6 : orange\n");
                    printf(" 7 : red\n");
                    printf(" 8 : rose\n");
                    printf(" 9 : yellow\n");
                    printf("10 : light green\n");
                    printf("11 : aqua\n");
                    printf("12 : pale purple\n");
                    printf("13 : light blue\n");
                    printf("14 : light gray\n");
                    printf("15 : white\n");
                    printf("16 : random\n\n");
                    printf("Enter fill color: ");
                    int c = -1;
                    if (scanf("%d", &c) == 1) {
                        if (c >= -1 && c <= 16) {
                            user_fill_color = c;
                            if (c == 16) {
                                if (model != NULL) {
                                    generate_random_colors(model->faces.face_count);
                                }
                                printf("Fill color set to RANDOM (new colors generated)\n");
                            }
                            else if (c >= 0) printf("Fill color set to %d\n", c);
                            else printf("Fill color reset to DEFAULT (14)\n");
                        } else {
                            printf("Invalid color (must be -1 to 16)\n");
                        }
                    }
                    int ch; while ((ch = getchar()) != '\n' && ch != EOF);
                    goto loopReDraw;
                }

            case 56: // '8' - choose frame color
                {
                    printf("\n=== Frame color selection ===\n");
                    printf("Available colors:\n");
                    printf(" 0 : black\n");
                    printf(" 1 : grey\n");
                    printf(" 2 : brown\n");
                    printf(" 3 : purple\n");
                    printf(" 4 : blue\n");
                    printf(" 5 : green\n");
                    printf(" 6 : orange\n");
                    printf(" 7 : red\n");
                    printf(" 8 : rose\n");
                    printf(" 9 : yellow\n");
                    printf("10 : light green\n");
                    printf("11 : aqua\n");
                    printf("12 : pale purple\n");
                    printf("13 : light blue\n");
                    printf("14 : light gray\n");
                    printf("15 : white\n");
                    printf("16 : random\n");
                    printf("17 : same as fill\n\n");
                    printf("Enter frame color: ");
                    int c = -1;
                    if (scanf("%d", &c) == 1) {
                        if (c >= -1 && c <= 17) {
                            user_frame_color = c;
                            if (c == 16) {
                                if (model != NULL) {
                                    generate_random_colors(model->faces.face_count);
                                }
                                printf("Frame color set to RANDOM (new colors generated)\n");
                            }
                            else if (c == 17) printf("Frame color set to SAME AS FILL\n");
                            else if (c >= 0) printf("Frame color set to %d\n", c);
                            else printf("Frame color reset to DEFAULT (7)\n");
                        } else {
                            printf("Invalid color (must be -1 to 17)\n");
                        }
                    }
                    int ch; while ((ch = getchar()) != '\n' && ch != EOF);
                    goto loopReDraw;
                }

            case 57: // '9' - reset colors to default
                user_fill_color = -1;
                user_frame_color = -1;
                palette = 0; // reset to system palette
                shaded_by_orientation = 0; // disable orientation shading
                printf("Colors reset to defaults, palette reset to 0, shading OFF\n");
                goto loopReDraw;

            case 54: // '6' - quick random mode for both colors
                user_fill_color = 16;
                user_frame_color = 16;
                if (model != NULL) {
                    generate_random_colors(model->faces.face_count);
                }
                printf("Colors set to RANDOM mode (new colors generated)\n");
                goto loopReDraw;

            case 80:  // 'P' - toggle frame-only polygon rendering
            case 112: // 'p'
                framePolyOnly ^= 1;
                printf("Frame-only polygons: %s\n", framePolyOnly ? "ON" : "OFF");
                if (!framePolyOnly) {
                    // Switched back to filled polygons — re-run full processing to recompute depths & ordering
                    printf("Switching to filled mode: reprocessing model (sorting faces)...\n");
                    goto bigloop;
                }
                else {
                    // Just redraw in frame-only mode
                    goto loopReDraw;
                }

            case 66:  // 'B' - toggle back-face culling (observer-space d<=0 test)
            case 98:  // 'b'
                cull_back_faces ^= 1;
                printf("Back-face culling: %s\n", cull_back_faces ? "ON" : "OFF");
                if (model != NULL) {
                    printf("Reprocessing model with culling %s...\n", cull_back_faces ? "ON" : "OFF");
                    goto bigloop;
                }
                //goto loopReDraw;

            case 70:  // 'F' - dump face equations to equ.csv
            case 102: // 'f'
                if (model != NULL) {
                    printf("Writing files...\n");
                    // Ensure the current observer parameters are applied before dumping
                    // processModelFast(model, &params, filename);
                    // Use semicolon column separators and comma decimal separator
                    dumpFaceEquationsCSV(model, "Faces3D.csv", 1);
                    // Also dump 2D per-face vertex coordinates to Faces2D.txt
                    dumpFace2DCoordinates(model, "Faces2D.txt");
                    // Dump sorted face indices in painter output order
                    dumpSortedFaceIndices(model, "FacesOrder.txt");
                    printf("\nPress any key to continue...\n");
                    keypress();
                }
                goto loopReDraw;

            case 78:  // 'N' - load new model
            case 110: // 'n'
                // Reset all parameters to startup defaults when loading a new model
                painter_mode = PAINTER_MODE_FAST;
                pan_dx = 0; pan_dy = 0;
                cull_back_faces = 1;
                user_fill_color = -1;
                user_frame_color = -1;
                s_global_proj_scale_fixed = INT_TO_FIXED(100);
                jitter = 0; /* reset jitter when loading new model */
                destroyModel3D(model);
                goto newmodel;

            case 75:  // 'K' - edit angles/distance (no reload; ENTER may auto-fit)
            case 107: // 'k'
                getObserverParams(&params, model);
                printf("Observer parameters updated.\n");
                goto bigloop;
        
            // display help (paged)
            case 72:  // 'H'
            case 104: // 'h'
                show_help_pager();
                goto loopReDraw;

            case 81: // 'Q' - interactive face-pair inspector (new)
            case 113: // 'q'
                if (model == NULL) { printf("No model loaded\n"); goto loopReDraw; }
                inspect_face_pair_ui(model);
                goto loopReDraw;

             // letter 'U'
            case 85:  // 'U' - toggle user-defined fill color mode (random if enabled)
            case 117: // 'u'
                // printf("Painter geo V1: toggle geometry-only painter mode for testing\n");
                // painter_geoV1(model, model->faces.face_count);
                startgraph(mode);
                // Implement the desired behavior for the 'O' key here
                int startTimeOld = GetTick();
                renderModelScanlineZBuffer_old(model);
                int endTimeOld = GetTick();
                key = getkeypress();
                if (key == '*') { saveNextScreenshot(); }

                endgraph();
                DoText();

                // printf("endtime = %d\n", endTime);
                printf("Z-Buffer scanline render time: %d ticks. Press a key to continue.\n", endTimeOld - startTimeOld);
                keypress();
                goto loopReDraw;


            case 27:  // ESC - quit
                goto end;
            
            
            // 'O' - some functionality for the 'O' key
            case 79:  // 'O'
            case 111: // 'o'
                startgraph(mode);
                // Implement the desired behavior for the 'O' key here
                int startTime = GetTick();
                renderModelScanlineZBuffer(model);
                int endTime = GetTick();
                key = getkeypress();
                if (key == '*') { saveNextScreenshot(); }

                endgraph();
                DoText();

                // printf("endtime = %d\n", endTime);
                printf("Z-Buffer scanline render time: %d ticks. Press a key to continue.\n", endTime - startTime);
                keypress();
                goto loopReDraw;

            default:  // All other keys - redraw
                goto loopReDraw;
            
        }
        }  // End of loopReDraw block

        end:
        // Cleanup and exit
        // Dispose of the global polygon handle if it was allocated
        if (globalPolyHandle != NULL) {
            if (poly_handle_locked) {
                HUnlock(globalPolyHandle);
            }
            DisposeHandle(globalPolyHandle);
            globalPolyHandle = NULL;
        }
        destroyModel3D(model);
        return 0;
    }