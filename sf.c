void showFace(Model3D* model, ObserverParams* params, const char* filename) {
    if (!model || !params) return;
    FaceArrays3D* faces = &model->faces;
    int face_count = faces->face_count;
    if (face_count <= 0) { printf("No faces in model\n"); return; }

    // Prompt user for face id
    printf("Enter face id (0..%d) to display: ", face_count - 1);
    int sel = -1;
    if (scanf("%d", &sel) != 1) {
        int ch; while ((ch = getchar()) != '\n' && ch != EOF) ;
        printf("Input cancelled\n");
        return;
    }
    // consume remaining chars on the line
    {
        int ch; while ((ch = getchar()) != '\n' && ch != EOF) ;
    }
    
    int target_face = sel;
    if (target_face < 0 || target_face >= face_count) {
        printf("Invalid face id\n");
        return;
    }

    printf("=> Face %d\n\n", target_face);
    printf("Use arrow keys to navigate (Left/Right: face ID, Up/Down: sorted list)\n");
    printf("Press SPACE to show detailed info about the selected face (hide/restore/restoreAll are there).\n");
    printf("Press N to toggle the face normal display.\n");
    printf("Use any other key to exit.\n\n");

    printf("Press any key to show model...\n");
    keypress();
    
    // Backup display flags
    unsigned char* backup_flags = (unsigned char*)malloc(faces->face_count);
    for (int i = 0; i < faces->face_count; ++i) backup_flags[i] = faces->display_flag[i];

    int old_frame = framePolyOnly;
    framePolyOnly = 1; // wireframe mode
    int show_normal = 0; // toggled by N/n
    
    // Navigation loop
    int quit = 0;
    while (!quit) {
        startgraph(mode);
        
        // Draw entire model in wireframe
        if (jitter) drawPolygons_jitter(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count); else drawPolygons(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count);

        // Overlay selected face in filled green (pen 10)
        // If the face is currently hidden, draw it in gray instead using its
        // saved vertex count, without permanently restoring it (geometry itself
        // was never modified by hideFace, only vertex_count was zeroed).
        if (faces->vertex_count[target_face] > 0) {
            faces->display_flag[target_face] = 1;
            drawFace(model, target_face, COL_LIGHT_GREEN, 0);
        } else if (faces->saved_vertex_count[target_face] > 0) {
            int saved_vc = faces->saved_vertex_count[target_face];
            int saved_df = faces->display_flag[target_face];

            faces->vertex_count[target_face] = saved_vc;
            faces->display_flag[target_face] = 1;
            drawFace(model, target_face, COL_GRAY, 0);

            // revert: keep the face hidden, saved_vertex_count untouched
            faces->vertex_count[target_face] = 0;
            faces->display_flag[target_face] = saved_df;
        }

        // --- Draw the face normal (toggle: N/n) ---
        // Strategy: project the centroid and a point slightly offset along
        // the normal (small 3D epsilon) to get the on-screen SLOPE of the
        // normal, then draw a FIXED 20px segment in that screen direction.
        // This avoids the earlier bug where stretching the normal by a fixed
        // 3D length gave wildly different screen lengths depending on zoom
        // (and could even wrap to negative/huge coordinates). Off-screen
        // endpoints are fine and left unclamped.
        if (show_normal && faces->vertex_count[target_face] >= 3) {
            int offt2 = faces->vertex_indices_ptr[target_face];
            int vn2 = faces->vertex_count[target_face];
            int k2;
            Fixed64 sx64 = 0, sy64 = 0, sz64 = 0;

            for (k2 = 0; k2 < vn2; ++k2) {
                int vid2 = faces->vertex_indices_buffer[offt2 + k2] - 1;
                sx64 += (Fixed64)model->vertices.xo[vid2];
                sy64 += (Fixed64)model->vertices.yo[vid2];
                sz64 += (Fixed64)model->vertices.zo[vid2];
            }

            float cxo = FIXED_TO_FLOAT((Fixed32)(sx64 / vn2));
            float cyo = FIXED_TO_FLOAT((Fixed32)(sy64 / vn2));
            float czo = FIXED_TO_FLOAT((Fixed32)(sz64 / vn2));

            float na = (float)FIXED64_TO_FLOAT(faces->plane_a[target_face]);
            float nb = (float)FIXED64_TO_FLOAT(faces->plane_b[target_face]);
            float nc = (float)FIXED64_TO_FLOAT(faces->plane_c[target_face]);
            float nlen = (float)sqrt((double)(na * na + nb * nb + nc * nc));

            // centroid must be in front of the camera to be projectable at all
            if (nlen > 0.0001f && czo > 0.0001f) {
                float scale_f = FIXED_TO_FLOAT(s_global_proj_scale_fixed);
                float centre_x_f2 = (float)CENTRE_X;
                float centre_y_f2 = (float)CENTRE_Y;

                float inv_zo_c = scale_f / czo;
                float c_screen_x = cxo * inv_zo_c + centre_x_f2;
                float c_screen_y = centre_y_f2 - cyo * inv_zo_c;

                // small 3D epsilon along the (unit) normal, just to sample slope
                const float EPS = 1.0f;
                float p2xo = cxo + (na / nlen) * EPS;
                float p2yo = cyo + (nb / nlen) * EPS;
                float p2zo = czo + (nc / nlen) * EPS;

                if (p2zo > 0.0001f) {
                    float inv_zo_p = scale_f / p2zo;
                    float p_screen_x = p2xo * inv_zo_p + centre_x_f2;
                    float p_screen_y = centre_y_f2 - p2yo * inv_zo_p;

                    float dx = p_screen_x - c_screen_x;
                    float dy = p_screen_y - c_screen_y;
                    float dlen = (float)sqrt((double)(dx * dx + dy * dy));

                    // dlen ~ 0 means the normal points straight at/away from
                    // the camera (edge-on in screen space) - nothing sensible
                    // to draw in that case
                    if (dlen > 0.0001f) {
                        const float NORMAL_SCREEN_LENGTH = 20.0f;
                        float ux = dx / dlen;
                        float uy = dy / dlen;

                        int c_x2d = (int)(c_screen_x + 0.5f);
                        int c_y2d = (int)(c_screen_y + 0.5f);
                        int e_x2d = (int)(c_screen_x + ux * NORMAL_SCREEN_LENGTH + 0.5f);
                        int e_y2d = (int)(c_screen_y + uy * NORMAL_SCREEN_LENGTH + 0.5f);

                        SetSolidPenPat(COL_YELLOW);
                        MoveTo(c_x2d, c_y2d);
                        LineTo(e_x2d, e_y2d);
                    }
                }
            }
        }

        // Annotate only the selected face ID after the full model is drawn
        drawFaceIndex(model, target_face);

        // Find position in sorted list for display
        int pos_in_sorted = -1;
        for (int i = 0; i < face_count; ++i) {
            if (faces->sorted_face_indices[i] == target_face) {
                pos_in_sorted = i;
                break;
            }
        }

        MoveTo(2, 188);
        // Orientation: front vs back (observer-space d > 0 => front)
        if (pos_in_sorted >= 0) {
            printf("Face %d (sorted pos %d) [%s]", target_face, pos_in_sorted, (faces->plane_d[target_face] > 0) ? "FRONT" : "BACK");
        } else {
            printf("Face %d [%s]", target_face, (faces->plane_d[target_face] > 0) ? "FRONT" : "BACK");
        }
        if (faces->vertex_count[target_face] == 0) {
            printf(" [HIDDEN]");
        }
        printf("\nArrows to navigate, SPACE for options, N to toggle normal");
        
        // Wait for key
        int key = getkeypress ();
        
        endgraph();
        
        // Process key - only arrow keys continue, any other key exits
        if (key == 8) { // Left arrow - previous face ID
            target_face--;
            if (target_face < 0) target_face = face_count - 1;
        } else if (key == 21) { // Right arrow - next face ID
            target_face++;
            if (target_face >= face_count) target_face = 0;
        } else if (key == 11) { // Up arrow - next position in sorted list
            if (pos_in_sorted >= 0 && pos_in_sorted < face_count - 1) {
                target_face = faces->sorted_face_indices[pos_in_sorted + 1];
            }
        } else if (key == 10) { // Down arrow - previous position in sorted list
            if (pos_in_sorted > 0) {
                target_face = faces->sorted_face_indices[pos_in_sorted - 1];
            }
        } else if (key == 'N' || key == 'n') {
            show_normal = !show_normal;
            continue;
        } else if (key == 32) { // Space - show textual details about the face
            // Switch to text mode and print detailed info, then return to graphics on keypress
            DoText();
            int vn = faces->vertex_count[target_face];
            // If hidden, still walk its vertices using the saved count -
            // the indices in vertex_indices_buffer are untouched by hideFace
            int effective_vn = (vn == 0 && faces->saved_vertex_count[target_face] > 0)
                                ? faces->saved_vertex_count[target_face] : vn;
            printf("\n=== Face detail (ID=%d) ===\n\n", target_face);
            printf("Orientation: %s [%s]\n", (faces->plane_d[target_face] > 0) ? "FRONT" : "BACK",
                   (vn == 0) ? "HIDDEN" : "NOT HIDDEN");
            // Plane equation (float)
            {
                float a = (float)FIXED64_TO_FLOAT(faces->plane_a[target_face]);
                float b = (float)FIXED64_TO_FLOAT(faces->plane_b[target_face]);
                float c = (float)FIXED64_TO_FLOAT(faces->plane_c[target_face]);
                float d = (float)FIXED64_TO_FLOAT(faces->plane_d[target_face]);
                printf("Plane equation: a=%f b=%f c=%f d=%f\n", a, b, c, d);
            }
            printf("Z min: %.6f   ;   ", FIXED_TO_FLOAT(faces->z_min[target_face]));
            printf("Z mean: %.6f   ;   ", FIXED_TO_FLOAT(faces->z_mean[target_face]));
            printf("Z max: %.6f\n\n", FIXED_TO_FLOAT(faces->z_max[target_face]));
            if (pos_in_sorted >= 0) printf("Position in sorted list: %d\n", pos_in_sorted);

            if (vn == 0) {
                printf("Vertex count: %d (hidden - showing %d saved vertices)\n\n", vn, effective_vn);
            } else {
                printf("Vertex count: %d\n\n", vn);
            }
            int offt = faces->vertex_indices_ptr[target_face];

            for (int k = 0; k < effective_vn; ++k) {
                int vid = faces->vertex_indices_buffer[offt + k] - 1;
                printf("vertex[%d] idx=%d model=(%f,%f,%f) obs=(%f,%f,%f) x2d=%d y2d=%d\n",
                       k, vid,
                       FIXED_TO_FLOAT(model->vertices.x[vid]), FIXED_TO_FLOAT(model->vertices.y[vid]), FIXED_TO_FLOAT(model->vertices.z[vid]),
                       FIXED_TO_FLOAT(model->vertices.xo[vid]), FIXED_TO_FLOAT(model->vertices.yo[vid]), FIXED_TO_FLOAT(model->vertices.zo[vid]),
                       model->vertices.x2d[vid], model->vertices.y2d[vid]);
            }

            printf("\nPress 'F' to save to file Face%d.txt, 'V' to reverse vertex order,\n", target_face);
            printf("'H' to hide, 'R' to restore, 'A' to restore all, any other key to return to graphics...\n");
            fflush(stdout);
            int tkey = getkeypress ();

            if (tkey == 'F' || tkey == 'f') {
                char fname[64]; sprintf(fname, "Face%d.txt", target_face);
                FILE *out = fopen(fname, "w");
                if (out) {
                    fprintf(out, "Face %d\n", target_face);
                    if (pos_in_sorted >= 0) fprintf(out, "Position in sorted list: %d\n", pos_in_sorted);
                    if (vn == 0) {
                        fprintf(out, "Vertex count: %d (hidden - showing %d saved vertices)\n", vn, effective_vn);
                    } else {
                        fprintf(out, "Vertex count: %d\n", vn);
                    }
                    for (int k = 0; k < effective_vn; ++k) {
                        int vid = faces->vertex_indices_buffer[offt + k] - 1;
                        fprintf(out, "v[%d] idx=%d model=(%f,%f,%f) obs=(%f,%f,%f) x2d=%d y2d=%d\n",
                                k, vid,
                                FIXED_TO_FLOAT(model->vertices.x[vid]), FIXED_TO_FLOAT(model->vertices.y[vid]), FIXED_TO_FLOAT(model->vertices.z[vid]),
                                FIXED_TO_FLOAT(model->vertices.xo[vid]), FIXED_TO_FLOAT(model->vertices.yo[vid]), FIXED_TO_FLOAT(model->vertices.zo[vid]),
                                model->vertices.x2d[vid], model->vertices.y2d[vid]);
                    }
                    float a = (float)FIXED64_TO_FLOAT(faces->plane_a[target_face]);
                    float b = (float)FIXED64_TO_FLOAT(faces->plane_b[target_face]);
                    float c = (float)FIXED64_TO_FLOAT(faces->plane_c[target_face]);
                    float d = (float)FIXED64_TO_FLOAT(faces->plane_d[target_face]);
                    fprintf(out, "Plane equation: a=%f b=%f c=%f d=%f\n", a, b, c, d);
                    fprintf(out, "Orientation: %s [%s]\n", (faces->plane_d[target_face] > 0) ? "FRONT" : "BACK",
                            (vn == 0) ? "HIDDEN" : "NOT HIDDEN");
                    if (vn == 0) {
                        fprintf(out, "Saved vertex count (before hide): %d\n", faces->saved_vertex_count[target_face]);
                    }
                    fprintf(out, "Z min: %.6f\n", FIXED_TO_FLOAT(faces->z_min[target_face]));
                    fprintf(out, "Z mean: %.6f\n", FIXED_TO_FLOAT(faces->z_mean[target_face]));
                    fprintf(out, "Z max: %.6f\n", FIXED_TO_FLOAT(faces->z_max[target_face]));
                    fclose(out);
                    printf("Saved to %s\n", fname); fflush(stdout);
                } else {
                    printf("Error: unable to open %s for writing\n", fname); fflush(stdout);
                }
                printf("Press any key to return to graphics...\n"); fflush(stdout);
                int tmpk = getkeypress ();

                // Return to graphics (loop will redraw)
                continue;
            } else if (tkey == 'V' || tkey == 'v') {
                // printf("Reversing vertex order...\n"); // useless since now face order reversing is instantaneous.
                reverseFaceVertexOrder(model, target_face);
                backup_flags[target_face] = faces->display_flag[target_face];
                printf("Face %d vertex order reversed.\n\n", target_face);
                printf("Press any key to return to graphics...\n"); fflush(stdout);
                int tmpk = getkeypress ();
                // Return to graphics (loop will redraw)
                continue;
            } else if (tkey == 'H' || tkey == 'h') {
                // Hide the currently selected face (vertex_count saved for later restore)
                hideFace(model, target_face);
                backup_flags[target_face] = faces->display_flag[target_face];
                printf("Face %d hidden.\n\n", target_face);
                printf("Press any key to return to graphics...\n"); fflush(stdout);
                int tmpk = getkeypress ();
                continue;
            } else if (tkey == 'R' || tkey == 'r') {
                // Restore the currently selected face and recompute its plane/display_flag
                // (the model may have rotated since it was hidden)
                restoreFace(model, target_face);
                calculateFaceDepths(model, faces, faces->face_count);
                backup_flags[target_face] = faces->display_flag[target_face];
                printf("Face %d restored.\n\n", target_face);
                printf("Press any key to return to graphics...\n"); fflush(stdout);
                int tmpk = getkeypress ();
                continue;
            } else if (tkey == 'A' || tkey == 'a') {
                // Restore every hidden face, then a single global recompute
                restoreAllFaces(model);
                calculateFaceDepths(model, faces, faces->face_count);
                for (int k2 = 0; k2 < faces->face_count; ++k2) backup_flags[k2] = faces->display_flag[k2];
                printf("All faces restored.\n\n");
                printf("Press any key to return to graphics...\n"); fflush(stdout);
                int tmpk = getkeypress ();
                continue;
            } else {
                // Any other key returns to graphics
                continue;
            }
        } else {
            // Any other key exits
            quit = 1;
        }
    }
    
    DoText();

    // Restore state
    framePolyOnly = old_frame;
    for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = backup_flags[i];
    free(backup_flags);
}