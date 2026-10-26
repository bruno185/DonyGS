/*
 * ============================================================================
 *  engine.c — Moteur géométrique, classement des faces et outils d’inspection
 * ============================================================================
 *
 *  Rôle :
 *    Charger le modèle OBJ, transformer et projeter ses sommets, calculer
 *    les métriques des faces, construire un ordre de dessin et fournir les
 *    outils de rendu et de diagnostic.
 *
 *  Organisation :
 *    Les segments regroupent chargement et préparation objet, transformation et
 *    classement des faces, tests d'ordre et ray casting, dessin QuickDraw, couleurs,
 *    inspection et exports. Les implémentations spécialisées sont intégrées
 *    par inclusion de sources C.
 *
 *  Dépendances et contraintes :
 *    Inclus par ObjExplorer.cc après les pragmas ORCA/C. Les caches et
 *    paquets assembleur sont partagés et non réentrants. Les interfaces de
 *    plans, de rayons et de caches n'emploient pas toutes les mêmes signes,
 *    échelles et tolérances ; leurs contrats font foi. Aucun algorithme
 *    n'est fusionné dans cette révision.
 *
 * ============================================================================
 */
/* Unité monolithique ORCA/C : les fichiers C inclus partagent types, états
 * statiques et ordre de déclaration.
 */
#include "declare.c"
#include "ordering_cache.c"
#include "dony.c"
#include "screen_save_restore.c"
#include "color.c"

#include "zbuffer_scanline.c"
#include "zbuffer_fullscreen.c"
#include "zbuffer_fullscreen_offscreen.c"


/* Chargement OBJ et outils de préparation de la géométrie objet. */
segment "model_loading";
/*
 * ----------------------------------------------------------------------------
 *  createModel3D
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Allouer le modèle et ses tableaux parallèles à capacité fixe.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    Modèle possédé par l'appelant, ou NULL sur échec traité.
 *
 *  Sorties et effets de bord :
 *    allocations et messages de diagnostic avec attente clavier sur échec.
 *
 *  Fonctionnement :
 *    Réserver MAX_VERTICES sommets et MAX_FACES métriques de faces ; le
 *    buffer d'indices est dimensionné à MAX_FACES*5, et non
 *    MAX_FACES*MAX_FACE_VERTICES. vertex_count contient initialement la
 *    capacité ; loadModel3D le remplace par le nombre lu. Les géométries et
 *    métriques restent à remplir après allocation.
 *
 * ----------------------------------------------------------------------------
 */
Model3D* createModel3D(void) {

    /* Réserver la structure propriétaire avant les tableaux ; sortir
     * immédiatement si elle manque.
     */
    Model3D* model = (Model3D*)malloc(sizeof(Model3D));
    if (model == NULL) {
        return NULL;
    }
    /* Initialiser les métadonnées de centrage et d'ajustement ; la capacité
     * sert d'abord de vertex_count.
     */
    int n = MAX_VERTICES;
    model->vertices.vertex_count = n;

    model->auto_scale = FIXED_ONE;
    model->auto_center_x = 0;
    model->auto_center_y = 0;
    model->auto_center_z = 0;
    model->auto_scaled = 0;
    model->auto_centered = 0;
    model->orig_x = NULL;
    model->orig_y = NULL;
    model->orig_z = NULL;

    model->faces.minx3 = NULL;
    model->faces.maxx3 = NULL;
    model->faces.miny3 = NULL;
    model->faces.maxy3 = NULL;
    model->faces.minz3 = NULL;
    model->faces.maxz3 = NULL;

    model->auto_suggested_distance = 0;
    model->auto_suggested_proj_scale = 0;
    model->auto_proj_scale = 0;
    model->auto_fit_ready = 0;
    model->auto_fit_applied = 0;
    model->coord_buf = NULL;
    model->coord_buf_capacity = 0;


    /* Allouer les coordonnées objet, observateur et écran ; chaque chemin
     * d'échec libère les allocations explicitement listées.
     */
    model->vertices.x = (Fixed32*)malloc(n * sizeof(Fixed32));
    model->vertices.y = (Fixed32*)malloc(n * sizeof(Fixed32));
    model->vertices.z = (Fixed32*)malloc(n * sizeof(Fixed32));
    model->vertices.xo = (Fixed32*)malloc(n * sizeof(Fixed32));
    model->vertices.yo = (Fixed32*)malloc(n * sizeof(Fixed32));
    model->vertices.zo = (Fixed32*)malloc(n * sizeof(Fixed32));
    model->vertices.x2d = (int*)malloc(n * sizeof(int));
    model->vertices.y2d = (int*)malloc(n * sizeof(int));

    if (!model->vertices.x || !model->vertices.y || !model->vertices.z ||
        !model->vertices.xo || !model->vertices.yo || !model->vertices.zo ||
        !model->vertices.x2d || !model->vertices.y2d) {
        printf("Error: Unable to allocate memory for vertex arrays\n");
        keypress();

        if (model->vertices.x) free(model->vertices.x);
        if (model->vertices.y) free(model->vertices.y);
        if (model->vertices.z) free(model->vertices.z);
        if (model->vertices.xo) free(model->vertices.xo);
        if (model->vertices.yo) free(model->vertices.yo);
        if (model->vertices.zo) free(model->vertices.zo);
        if (model->vertices.x2d) free(model->vertices.x2d);
        if (model->vertices.y2d) free(model->vertices.y2d);
        free(model);
        return NULL;
    }


    model->vertices.xHandle = NULL;
    model->vertices.yHandle = NULL;
    model->vertices.zHandle = NULL;
    model->vertices.xoHandle = NULL;
    model->vertices.yoHandle = NULL;
    model->vertices.zoHandle = NULL;
    model->vertices.x2dHandle = NULL;
    model->vertices.y2dHandle = NULL;


    /* Réserver les listes et métriques des faces indépendamment des
     * coordonnées des sommets.
     */
    int nf = MAX_FACES;


    model->faces.vertex_count = (int*)malloc(nf * sizeof(int));
    if (!model->faces.vertex_count) {
        printf("Error: Unable to allocate memory for face vertex_count array\n");
        keypress();
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model);
        return NULL;
    }


    model->faces.saved_vertex_count = (int*)calloc(nf, sizeof(int));
    if (!model->faces.saved_vertex_count) {
        printf("Error: Unable to allocate memory for face saved_vertex_count array\n");
        keypress();
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model->faces.vertex_count);
        free(model);
        return NULL;
    }


    /* Réserver le buffer compact pour une moyenne de cinq indices par face ;
     * aucune extension n'est effectuée ici.
     */
    int estimated_total_indices = nf * 5;
    model->faces.vertex_indices_buffer = (int*)malloc(estimated_total_indices * sizeof(int));
    if (!model->faces.vertex_indices_buffer) {
        printf("Error: Unable to allocate memory for vertex_indices_buffer\n");
        keypress();
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model->faces.vertex_count);
        free(model->faces.saved_vertex_count);
        free(model);
        return NULL;
    }


    model->faces.vertex_indices_ptr = (int*)malloc(nf * sizeof(int));
    if (!model->faces.vertex_indices_ptr) {
        printf("Error: Unable to allocate memory for vertex_indices_ptr array\n");
        keypress();
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model->faces.vertex_count);
        free(model->faces.saved_vertex_count);
        free(model->faces.vertex_indices_buffer);
        free(model);
        return NULL;
    }


    /* Allouer les intervalles de profondeur et la clé moyenne utilisée par FAST. */
    model->faces.z_min = (Fixed32*)malloc(nf * sizeof(Fixed32));
    model->faces.z_max = (Fixed32*)malloc(nf * sizeof(Fixed32));
    if (!model->faces.z_min || !model->faces.z_max) {
        printf("Error: Unable to allocate memory for face depth arrays\n");
        keypress();
        if (model->faces.z_min) free(model->faces.z_min);
        if (model->faces.z_max) free(model->faces.z_max);
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model->faces.vertex_count);
        free(model->faces.saved_vertex_count);
        free(model->faces.vertex_indices_buffer);
        free(model->faces.vertex_indices_ptr);
        free(model);
        return NULL;
    }

    model->faces.z_mean = (Fixed32*)malloc(nf * sizeof(Fixed32));
    if (!model->faces.z_mean) {
        printf("Error: Unable to allocate memory for face z_mean array\n");
        keypress();
        if (model->faces.z_mean) free(model->faces.z_mean);
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model->faces.vertex_count);
        free(model->faces.saved_vertex_count);
        free(model->faces.vertex_indices_buffer);
        free(model->faces.vertex_indices_ptr);
        if (model->faces.z_min) free(model->faces.z_min);
        if (model->faces.z_max) free(model->faces.z_max);
        free(model);
        return NULL;
    }

    /* Allouer les coefficients 64 bits des plans observateur. */
    model->faces.plane_a = (Fixed64*)malloc(nf * sizeof(Fixed64));
    model->faces.plane_b = (Fixed64*)malloc(nf * sizeof(Fixed64));
    model->faces.plane_c = (Fixed64*)malloc(nf * sizeof(Fixed64));
    model->faces.plane_d = (Fixed64*)malloc(nf * sizeof(Fixed64));
    if (!model->faces.plane_a || !model->faces.plane_b || !model->faces.plane_c || !model->faces.plane_d) {
        printf("Error: Unable to allocate memory for face plane arrays\n");
        keypress();
        if (model->faces.plane_a) free(model->faces.plane_a);
        if (model->faces.plane_b) free(model->faces.plane_b);
        if (model->faces.plane_c) free(model->faces.plane_c);
        if (model->faces.plane_d) free(model->faces.plane_d);
        if (model->faces.z_mean) free(model->faces.z_mean);
        if (model->faces.z_min) free(model->faces.z_min);
        if (model->faces.z_max) free(model->faces.z_max);
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model->faces.vertex_count);
        free(model->faces.saved_vertex_count);
        free(model->faces.vertex_indices_buffer);
        free(model->faces.vertex_indices_ptr);
        free(model->faces.z_max);
        free(model);
        return NULL;
    }

    /* Allouer les bornes projetées servant aux rejets rapides de paires. */
    model->faces.minx = (int*)malloc(nf * sizeof(int));
    model->faces.maxx = (int*)malloc(nf * sizeof(int));
    model->faces.miny = (int*)malloc(nf * sizeof(int));
    model->faces.maxy = (int*)malloc(nf * sizeof(int));
    if (!model->faces.minx || !model->faces.maxx || !model->faces.miny || !model->faces.maxy) {
        printf("Error: Unable to allocate memory for face bounding box arrays\n");
        keypress();
        if (model->faces.minx) free(model->faces.minx);
        if (model->faces.maxx) free(model->faces.maxx);
        if (model->faces.miny) free(model->faces.miny);
        if (model->faces.maxy) free(model->faces.maxy);
        if (model->faces.plane_a) free(model->faces.plane_a);
        if (model->faces.plane_b) free(model->faces.plane_b);
        if (model->faces.plane_c) free(model->faces.plane_c);
        if (model->faces.plane_d) free(model->faces.plane_d);
        if (model->faces.z_mean) free(model->faces.z_mean);
        if (model->faces.z_min) free(model->faces.z_min);
        if (model->faces.z_max) free(model->faces.z_max);
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model->faces.vertex_count);
        free(model->faces.saved_vertex_count);
        free(model->faces.vertex_indices_buffer);
        free(model->faces.vertex_indices_ptr);
        free(model->faces.z_max);
        free(model);
        return NULL;
    }


    /* Réserver les drapeaux de visibilité et la permutation de rendu. */
    model->faces.display_flag = (int*)malloc(nf * sizeof(int));
    if (!model->faces.display_flag) {
        printf("Error: Unable to allocate memory for face display_flag array\n");
        keypress();
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model->faces.vertex_count);
        free(model->faces.saved_vertex_count);
        free(model->faces.vertex_indices_buffer);
        free(model->faces.vertex_indices_ptr);
        if (model->faces.z_min) free(model->faces.z_min);
        if (model->faces.z_max) free(model->faces.z_max);
        if (model->faces.z_mean) free(model->faces.z_mean);
        free(model);
        return NULL;
    }


    model->faces.sorted_face_indices = (int*)malloc(nf * sizeof(int));
    if (!model->faces.sorted_face_indices) {
        printf("Error: Unable to allocate memory for sorted_face_indices array\n");
        keypress();
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model->faces.vertex_count);
        free(model->faces.saved_vertex_count);
        free(model->faces.vertex_indices_buffer);
        free(model->faces.vertex_indices_ptr);
        if (model->faces.z_min) free(model->faces.z_min);
        if (model->faces.z_max) free(model->faces.z_max);
        if (model->faces.z_mean) free(model->faces.z_mean);
        free(model->faces.display_flag);
        free(model);
        return NULL;
    }


    /* Initialiser les handles non utilisés par ces allocations malloc et le
     * compteur d'indices.
     */
    model->faces.vertex_countHandle = NULL;
    model->faces.vertex_indicesBufferHandle = NULL;
    model->faces.vertex_indicesPtrHandle = NULL;
    model->faces.z_maxHandle = NULL;
    model->faces.display_flagHandle = NULL;
    model->faces.sorted_face_indicesHandle = NULL;
    model->faces.total_indices = 0;

    return model;
}
/*
 * ----------------------------------------------------------------------------
 *  destroyModel3D
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Libérer les buffers explicitement possédés par un modèle puis sa
 *    structure.
 *
 *  Paramètres :
 *    [entrée] model, créé par createModel3D ; NULL est accepté.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Invalide le pointeur fourni sans le remettre à NULL chez l'appelant.
 *    Libère aussi le cache de projection fractionnaire associé. Les
 *    ressources statiques des méthodes de classement et les Z-buffers ont des cycles de
 *    vie distincts.
 *
 * ----------------------------------------------------------------------------
 */
void destroyModel3D(Model3D* model) {
    /* Ne rien faire pour NULL ; les pointeurs du modèle doivent sinon
     * décrire un objet initialisé.
     */
    if (model != NULL) {
        /* Libérer le cache fractionnaire avant les tableaux qu'il référence. */
        fractional_projection_cache_release(model);

        if (model->vertices.x) free(model->vertices.x);
        if (model->vertices.y) free(model->vertices.y);
        if (model->vertices.z) free(model->vertices.z);
        if (model->vertices.xo) free(model->vertices.xo);
        if (model->vertices.yo) free(model->vertices.yo);
        if (model->vertices.zo) free(model->vertices.zo);
        if (model->vertices.x2d) free(model->vertices.x2d);
        if (model->vertices.y2d) free(model->vertices.y2d);


        /* Libérer les listes et métriques de faces, puis les copies et
         * buffers supplémentaires.
         */
        if (model->faces.vertex_count) free(model->faces.vertex_count);
        if (model->faces.vertex_indices_buffer) free(model->faces.vertex_indices_buffer);
        if (model->faces.vertex_indices_ptr) free(model->faces.vertex_indices_ptr);
        if (model->faces.z_max) free(model->faces.z_max);
        if (model->faces.z_mean) free(model->faces.z_mean);
        if (model->faces.z_min) free(model->faces.z_min);
        if (model->faces.plane_a) free(model->faces.plane_a);
        if (model->faces.plane_b) free(model->faces.plane_b);
        if (model->faces.plane_c) free(model->faces.plane_c);
        if (model->faces.plane_d) free(model->faces.plane_d);
        if (model->faces.minx) free(model->faces.minx);
        if (model->faces.maxx) free(model->faces.maxx);
        if (model->faces.miny) free(model->faces.miny);
        if (model->faces.maxy) free(model->faces.maxy);
        if (model->faces.minx3) free(model->faces.minx3);
        if (model->faces.maxx3) free(model->faces.maxx3);
        if (model->faces.miny3) free(model->faces.miny3);
        if (model->faces.maxy3) free(model->faces.maxy3);
        if (model->faces.minz3) free(model->faces.minz3);
        if (model->faces.maxz3) free(model->faces.maxz3);
        if (model->faces.display_flag) free(model->faces.display_flag);
        if (model->faces.sorted_face_indices) free(model->faces.sorted_face_indices);
        if (model->faces.saved_vertex_count) free(model->faces.saved_vertex_count);

        if (model->orig_x) free(model->orig_x);
        if (model->orig_y) free(model->orig_y);
        if (model->orig_z) free(model->orig_z);
        if (model->coord_buf) free(model->coord_buf);


        /* Libérer la structure en dernier ; aucun de ses tableaux ne reste utilisable. */
        free(model);
    }
}
/*
 * ----------------------------------------------------------------------------
 *  loadModel3D
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Charger sommets et faces OBJ dans un modèle déjà alloué.
 *
 *  Paramètres :
 *    [entrée] filename et model.
 *
 *  Retour :
 *    -1 pour arguments invalides ou échec de lecture des sommets ; 0 sinon.
 *
 *  Sorties et effets de bord :
 *    coordonnées, listes et compteurs du modèle.
 *
 *  Fonctionnement :
 *    Un échec de readFaces_model ne propage pas -1 : il laisse zéro face et
 *    un avertissement. readVertices_last_count reçoit le vertex_count
 *    antérieur à la lecture ; cette borne ne doit pas être décrite comme le
 *    nombre effectivement lu dans cet appel.
 *
 * ----------------------------------------------------------------------------
 */
int loadModel3D(Model3D* model, const char* filename) {

    /* Valider les pointeurs avant toute lecture. */
    if (model == NULL || filename == NULL) {
        return -1;
    }


    /* Conserver la borne préalable puis lire les sommets et leurs
     * suggestions de vue.
     */
    readVertices_last_count = model->vertices.vertex_count;

    int vcount = readVertices(filename, &model->vertices, MAX_VERTICES, model);
    if (vcount < 0) {
        return -1;
    }
    model->vertices.vertex_count = vcount;


    /* Relire le fichier pour les faces ; conserver un chargement sans faces
     * si cette étape échoue.
     */
    int fcount = readFaces_model(filename, model);
    if (fcount < 0) {

        printf("\nWarning: Unable to read faces\n");
        model->faces.face_count = 0;
    } else {
        model->faces.face_count = fcount;


    }
    return 0;
}
/*
 * ----------------------------------------------------------------------------
 *  readVertices
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Lire les lignes OBJ « v x y z », centrer la géométrie et proposer une
 *    vue.
 *
 *  Paramètres :
 *    [entrée] filename ; max_vertices, capacité de vtx ; owner, modèle
 *    associé ou NULL.
 *
 *  Retour :
 *    Nombre de sommets lus, ou -1 si fopen échoue ; vertex_count reste à
 *    fixer par l'appelant.
 *
 *  Sorties et effets de bord :
 *    vtx->x/y/z et, si owner est fourni, centrage et suggestions
 *    d'ajustement.
 *
 *  Fonctionnement :
 *    Les axes OBJ y et z sont échangés à l'import. L'ajustement utilise la
 *    vue 30°/20°, une distance de trois fois la dimension maximale et une
 *    marge écran de 90 %. Ne lit ni normales ni coordonnées de texture.
 *
 * ----------------------------------------------------------------------------
 */
int readVertices(const char* filename, VertexArrays3D* vtx, int max_vertices, Model3D* owner) {
    FILE *file;
    char line[MAX_LINE_LENGTH];
    int line_number = 1;
    int vertex_count = 0;


    /* Ouvrir le fichier en texte ; les autres types de lignes sont ignorés. */
    file = fopen(filename, "r");
    if (file == NULL) {
        return -1;
    }

    printf("\nReading vertices from file '%s' ", filename);


    while (fgets(line, sizeof(line), file) != NULL) {
        /* Convertir trois coordonnées en 16.16 dans la capacité disponible,
         * en permutant y et z.
         */
        if (line[0] == 'v' && line[1] == ' ') {
            if (vertex_count < max_vertices) {
                float x, y, z;
                if (sscanf(line + 2, "%f %f %f", &x, &y, &z) == 3) {
                    vtx->x[vertex_count] = FLOAT_TO_FIXED(x);

                    vtx->y[vertex_count] = FLOAT_TO_FIXED(z);
                    vtx->z[vertex_count] = FLOAT_TO_FIXED(y);
                    vertex_count++;
                    if (vertex_count % 10 == 0) printf("..");
                } else {
                    printf("[\nDEBUG] readVertices: sscanf failed at line %d: %s\n", line_number, line);
                    keypress();
                }
            } else {
                printf("\n[DEBUG] readVertices: vertex limit reached (%d)\n", max_vertices);
                keypress();
            }
        }
        line_number++;
    }
    printf("\n");
    printf("Reading vertices finished : %d vertices read.\n", vertex_count);

    fclose(file);

    /* Calculer les bornes objet et soustraire leur centre aux sommets lus. */
    if (vertex_count > 0 && owner) {
        Fixed32 xmin = vtx->x[0], xmax = vtx->x[0];
        Fixed32 ymin = vtx->y[0], ymax = vtx->y[0];
        Fixed32 zmin = vtx->z[0], zmax = vtx->z[0];
        for (int i = 1; i < vertex_count; ++i) {
            if (vtx->x[i] < xmin) xmin = vtx->x[i];
            if (vtx->x[i] > xmax) xmax = vtx->x[i];
            if (vtx->y[i] < ymin) ymin = vtx->y[i];
            if (vtx->y[i] > ymax) ymax = vtx->y[i];
            if (vtx->z[i] < zmin) zmin = vtx->z[i];
            if (vtx->z[i] > zmax) zmax = vtx->z[i];
        }
        owner->auto_center_x = (Fixed32)((xmin + xmax) / 2);
        owner->auto_center_y = (Fixed32)((ymin + ymax) / 2);
        owner->auto_center_z = (Fixed32)((zmin + zmax) / 2);

        for (int i = 0; i < vertex_count; ++i) {
            vtx->x[i] = FIXED_SUB(vtx->x[i], owner->auto_center_x);
            vtx->y[i] = FIXED_SUB(vtx->y[i], owner->auto_center_y);
            vtx->z[i] = FIXED_SUB(vtx->z[i], owner->auto_center_z);
        }
        owner->auto_centered = 1;


        /* Établir une distance suggérée à partir de la plus grande dimension. */
        Fixed32 dx = FIXED_SUB(xmax, xmin);
        Fixed32 dy = FIXED_SUB(ymax, ymin);
        Fixed32 dz = FIXED_SUB(zmax, zmin);
        Fixed32 max_dim = dx;
        if (dy > max_dim) max_dim = dy;
        if (dz > max_dim) max_dim = dz;

        owner->auto_suggested_distance = FIXED_MUL_64(max_dim, INT_TO_FIXED(3));

        /* Projeter provisoirement les sommets à la vue initiale pour mesurer
         * leur emprise.
         */
        int ah = 30, av = 20, aw = 0;
        Fixed32 cos_h = cos_deg_int(ah), sin_h = sin_deg_int(ah);
        Fixed32 cos_v = cos_deg_int(av), sin_v = sin_deg_int(av);
        const Fixed32 cos_h_cos_v = FIXED_MUL_64(cos_h, cos_v);
        const Fixed32 sin_h_cos_v = FIXED_MUL_64(sin_h, cos_v);
        const Fixed32 cos_h_sin_v = FIXED_MUL_64(cos_h, sin_v);
        const Fixed32 sin_h_sin_v = FIXED_MUL_64(sin_h, sin_v);
        float pxmin = 1e30f, pxmax = -1e30f, pymin = 1e30f, pymax = -1e30f;
        for (int i = 0; i < vertex_count; ++i) {
            Fixed32 x = vtx->x[i]; Fixed32 y = vtx->y[i]; Fixed32 z = vtx->z[i];
            Fixed32 term1 = FIXED_MUL_64(x, cos_h_cos_v);
            Fixed32 term2 = FIXED_MUL_64(y, sin_h_cos_v);
            Fixed32 term3 = FIXED_MUL_64(z, sin_v);
            Fixed32 zo = FIXED_ADD(FIXED_SUB(FIXED_SUB(FIXED_NEG(term1), term2), term3), owner->auto_suggested_distance);
            if (zo > 0) {
                Fixed32 xo = FIXED_ADD(FIXED_NEG(FIXED_MUL_64(x, sin_h)), FIXED_MUL_64(y, cos_h));
                Fixed32 yo = FIXED_ADD(FIXED_SUB(FIXED_NEG(FIXED_MUL_64(x, cos_h_sin_v)), FIXED_MUL_64(y, sin_h_sin_v)), FIXED_MUL_64(z, cos_v));
                float px = FIXED_TO_FLOAT(xo) / FIXED_TO_FLOAT(zo);
                float py = FIXED_TO_FLOAT(yo) / FIXED_TO_FLOAT(zo);
                if (px < pxmin) pxmin = px;
                if (px > pxmax) pxmax = px;
                if (py < pymin) pymin = py;
                if (py > pymax) pymax = py;
            }
        }

        /* Choisir l'échelle qui tient dans 90 % de la largeur et de la
         * hauteur disponibles.
         */
        float margin = 0.9f;
        float winw = (float)(CENTRE_X * 2);
        float winh = (float)(CENTRE_Y * 2);
        float s = 100.0f;
        if (pxmax > pxmin && pymax > pymin) {
            float s1 = (winw * margin) / (pxmax - pxmin);
            float s2 = (winh * margin) / (pymax - pymin);
            s = (s1 < s2) ? s1 : s2;
        }
        owner->auto_suggested_proj_scale = FLOAT_TO_FIXED(s);
        owner->auto_fit_ready = 1;
    }

    return vertex_count;
}
/*
 * ----------------------------------------------------------------------------
 *  readFaces_model
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Lire les lignes OBJ « f » dans le buffer compact des faces.
 *
 *  Paramètres :
 *    [entrée] filename et model déjà alloué ; readVertices_last_count sert
 *    de borne.
 *
 *  Retour :
 *    Nombre de faces non vides, ou -1 sur structure invalide, ouverture
 *    impossible ou indice supérieur à la borne globale.
 *
 *  Sorties et effets de bord :
 *    listes d'indices à base un, offsets, tailles, compteur total et ordre
 *    canonique.
 *
 *  Fonctionnement :
 *    Les suffixes /vt/vn sont sautés ; les indices relatifs négatifs ne sont
 *    pas interprétés. Au plus 16 indices sont retenus.
 *
 *  Limites :
 *    l'écriture du buffer compact ne contrôle pas ici sa capacité totale.
 *
 * ----------------------------------------------------------------------------
 */
int readFaces_model(const char* filename, Model3D* model) {
    FILE *file;
    char line[MAX_LINE_LENGTH];
    int line_number = 1;
    int face_count = 0;
    int i;


    /* Valider la structure avant l'ouverture du fichier. */
    if (model == NULL || model->faces.vertex_count == NULL) {
        printf("Error: Invalid model structure for readFaces_model\n");
        return -1;
    }


    file = fopen(filename, "r");
    if (file == NULL) {
        printf("Error: Unable to open file '%s' to read faces\n", filename);
        return -1;
    }

    printf("\nReading faces from file '%s' ", filename);

    int buffer_pos = 0;


    while (fgets(line, sizeof(line), file) != NULL) {

        /* Initialiser la face à l'offset courant puis analyser les premiers
         * indices positifs.
         */
        if (line[0] == 'f' && line[1] == ' ') {
            if (face_count < MAX_FACES) {

                model->faces.vertex_count[face_count] = 0;
                model->faces.display_flag[face_count] = 1;
                model->faces.saved_vertex_count[face_count] = 0;
                model->faces.vertex_indices_ptr[face_count] = buffer_pos;


                char *ptr = line + 2;
                int temp_indices[MAX_FACE_VERTICES];
                int temp_vertex_count = 0;
                int invalid_index_found = 0;


                /* Parcourir les tokens, ignorer les suffixes et s'arrêter à
                 * la limite par face.
                 */
                while (*ptr != '\0' && *ptr != '\n' && temp_vertex_count < MAX_FACE_VERTICES) {

                    while (*ptr == ' ' || *ptr == '\t') ptr++;

                    if (*ptr == '\0' || *ptr == '\n') break;


                    int vertex_index = 0;
                    while (*ptr >= '0' && *ptr <= '9') {
                        vertex_index = vertex_index * 10 + (*ptr - '0');
                        ptr++;
                    }


                    while (*ptr != '\0' && *ptr != ' ' && *ptr != '\t' && *ptr != '\n') {
                        ptr++;
                    }


                    if (vertex_index >= 1) {
                        if (vertex_index > readVertices_last_count) {

                            invalid_index_found = 1;
                        }
                        temp_indices[temp_vertex_count] = vertex_index;
                        temp_vertex_count++;
                    }
                }


                /* Refuser la face entière si un indice dépasse la borne de validation. */
                if (invalid_index_found) {
                    printf("\nERROR: Face at line %d references vertex index > %d vertices\n",
                           line_number, readVertices_last_count);
                    fclose(file);
                    return -1;
                } else {

                    for (i = 0; i < temp_vertex_count; i++) {
                        model->faces.vertex_indices_buffer[buffer_pos++] = temp_indices[i];
                    }
                    model->faces.vertex_count[face_count] = temp_vertex_count;
                    model->faces.total_indices += temp_vertex_count;


                    if (model->faces.vertex_count[face_count] > 0) {
                        face_count++;
                        if (face_count % 10 == 0) {printf(".");}
                    } else {
                        printf("     -> WARNING: Face without valid vertices ignored\n");
                    }
                }
            } else {
                printf("     -> WARNING: Face limit reached (%d)\n", MAX_FACES);
            }
        }

        line_number++;
    }


    fclose(file);

    /* Publier le compteur lu et reconstruire une permutation canonique des faces. */
    model->faces.face_count = face_count;


    for (i = 0; i < face_count; i++) {
        model->faces.sorted_face_indices[i] = i;
    }

    printf("\nReading faces finished : %d faces read.\n", face_count);
    return face_count;
}
/*
 * ----------------------------------------------------------------------------
 *  check_intersect
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Examiner les intersections objet et tenter des découpages avant la
 *    visualisation.
 *
 *  Paramètres :
 *    [entrée-sortie] model ; NULL est accepté.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Cette commande modifie potentiellement sommets, faces et bornes ; ce
 *    n'est pas un simple diagnostic. Filtre les paires par boîtes 3D, écarte
 *    celles partageant un sommet, estime la coplanarité puis compte les
 *    traversées avant split_face_by_plane. Recommence après chaque
 *    découpage. Les compteurs peuvent inclure plusieurs passages.
 *
 *  Limites :
 *    réserves des nouvelles faces/indices non vérifiées ici, échec des bbox
 *    non contrôlé et aucun budget de reprises ; outil expérimental.
 *
 * ----------------------------------------------------------------------------
 */
static void check_intersect(Model3D* model) {
    if (!model) return;
    /* Préparer les bornes objet utilisées comme premier filtre. */
    compute_face_bboxes3D(model);
    FaceArrays3D* faces = &model->faces;
    int fc = faces->face_count;

    if (fc <= 1) {

        keypress();
        return;
    }

    printf("check_intersect: scanning %d faces, please wait...\n", fc);

    const Fixed32 eps = FLOAT_TO_FIXED(0.02f);
    int count = 0;
    int ip_count = 0;


    int changed;
    /* Repartir du modèle modifié à chaque découpage accepté. */
    do {
        changed = 0;
        fc = faces->face_count;
        for (int i = 0; i < fc; ++i) {
            Fixed32 minx_i = faces->minx3[i];
            Fixed32 maxx_i = faces->maxx3[i];
            Fixed32 miny_i = faces->miny3[i];
            Fixed32 maxy_i = faces->maxy3[i];
            Fixed32 minz_i = faces->minz3[i];
            Fixed32 maxz_i = faces->maxz3[i];
            for (int j = i + 1; j < fc; ++j) {
                Fixed32 minx_j = faces->minx3[j];
                Fixed32 maxx_j = faces->maxx3[j];
                Fixed32 miny_j = faces->miny3[j];
                Fixed32 maxy_j = faces->maxy3[j];
                Fixed32 minz_j = faces->minz3[j];
                Fixed32 maxz_j = faces->maxz3[j];

                /* Limiter les tests coûteux aux boîtes proches selon epsilon. */
                if (minx_i <= maxx_j + eps && maxx_i >= minx_j - eps &&
                    miny_i <= maxy_j + eps && maxy_i >= miny_j - eps &&
                    minz_i <= maxz_j + eps && maxz_i >= minz_j - eps) {

                    Fixed32 ox = (minx_i < minx_j ? minx_j : minx_i);
                    Fixed32 ux = (maxx_i < maxx_j ? maxx_i : maxx_j);
                    Fixed32 oy = (miny_i < miny_j ? miny_j : miny_i);
                    Fixed32 uy = (maxy_i < maxy_j ? maxy_i : maxy_j);
                    Fixed32 oz = (minz_i < minz_j ? minz_j : minz_i);
                    Fixed32 uz = (maxz_i < maxz_j ? maxz_i : maxz_j);
                    float dx = FIXED_TO_FLOAT(ux - ox);
                    float dy = FIXED_TO_FLOAT(uy - oy);
                    float dz = FIXED_TO_FLOAT(uz - oz);


                    if (dx >= 0.0f && dy >= 0.0f) {

                        ++count;
                        /* Ignorer les faces ayant déjà un identifiant de sommet en commun. */
                        if (faces_share_vertex_or_edge(faces, i, j)) {

                        } else {
                            double a1,b1,c1,d1, a2,b2,c2,d2;
                            compute_face_plane(model, i, &a1, &b1, &c1, &d1);
                            compute_face_plane(model, j, &a2, &b2, &c2, &d2);
                            double norm1 = sqrt(a1*a1 + b1*b1 + c1*c1);
                            double norm2 = sqrt(a2*a2 + b2*b2 + c2*c2);
                            int coplanar = 0;
                            if (norm1 > 0 && norm2 > 0) {
                                double dot = a1*a2 + b1*b2 + c1*c2;
                                double cosang = dot / (norm1 * norm2);
                                if (fabs(fabs(cosang) - 1.0) < 0.01) {
                                    double dist = fabs(d1 - d2) / norm1;
                                    if (dist < FIXED_TO_FLOAT(eps)) {
                                        coplanar = 1;
                                    }
                                }
                            }
                            if (coplanar) {

                            }
                            /* Tester les arêtes seulement pour les paires non reconnues coplanaires. */
                            if (!coplanar) {


                                int interior = edge_crosses_interior(model, i, j) ||
                                               edge_crosses_interior(model, j, i);
                                if (interior) {
                                    ++ip_count;
                                } else {
                                }


                            /* Choisir la face à découper d'après les nombres
                             * de traversées dans chaque sens.
                             */
                            int cross_i_j = count_edge_crossings(model, i, j);
                            int cross_j_i = count_edge_crossings(model, j, i);


                            if (cross_i_j == 2 && cross_j_i == 0) {

                                if (split_face_by_plane(model, i, j, NULL, NULL, NULL, 0, 0)) {
                                    changed = 1;
                                    break;
                                }
                            } else if (cross_j_i == 2 && cross_i_j == 0) {

                                if (split_face_by_plane(model, j, i, NULL, NULL, NULL, 0, 0)) {
                                    changed = 1;
                                    break;
                                }
                            } else {

                                if (cross_i_j > 0) {
                                    int force = (cross_i_j == 1);
                                    if (split_face_by_plane(model, j, i, NULL, NULL, NULL, 0, force)) {
                                        changed = 1;
                                        break;
                                    }
                                }
                                if (!changed && cross_j_i > 0) {
                                    int force = (cross_j_i == 1);
                                    if (split_face_by_plane(model, i, j, NULL, NULL, NULL, 0, force)) {
                                        changed = 1;
                                        break;
                                    }
                                }
                            }
                            }
                            }
                        }
                    }
                }
            }
            if (changed) break;
    } while (changed);

    /* Afficher les compteurs accumulés et le nombre final de faces. */
    printf("Total overlapping pairs: %d\n", count);
    printf("Total interpenetrating pairs: %d\n", ip_count);
    printf("Final face count: %d\n", faces->face_count);
}
/*
 * ----------------------------------------------------------------------------
 *  faces_share_vertex_or_edge
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Détecter un identifiant de sommet commun à deux faces.
 *
 *  Paramètres :
 *    [entrée] faces, i et j à base zéro.
 *
 *  Retour :
 *    1 au premier identifiant commun, 0 sinon.
 *
 *  Fonctionnement :
 *    Aucun effet de bord. Le test ne requiert pas une arête entière commune
 *    et ne détecte pas les sommets géométriquement égaux portant des indices
 *    distincts.
 *
 * ----------------------------------------------------------------------------
 */
static int faces_share_vertex_or_edge(FaceArrays3D* faces, int i, int j) {
    int cnti = faces->vertex_count[i];
    int offi = faces->vertex_indices_ptr[i];
    int cntj = faces->vertex_count[j];
    int offj = faces->vertex_indices_ptr[j];
    /* Comparer les deux listes d'identifiants OBJ, sans test de coordonnées. */
    for (int a = 0; a < cnti; ++a) {
        int vi = faces->vertex_indices_buffer[offi + a];
        for (int b = 0; b < cntj; ++b) {
            if (vi == faces->vertex_indices_buffer[offj + b]) {
                return 1;
            }
        }
    }
    return 0;
}
/*
 * ----------------------------------------------------------------------------
 *  edge_crosses_interior
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Détecter une arête de ei qui traverse strictement l'intérieur de fj.
 *
 *  Paramètres :
 *    [entrée] model, ei et fj, identifiants de faces à base zéro.
 *
 *  Retour :
 *    1 au premier franchissement intérieur, 0 sinon.
 *
 *  Fonctionnement :
 *    Aucun changement de géométrie. Calcule le plan objet de fj sur les
 *    entiers 16.16 bruts, choisit sa projection dominante puis quantifie les
 *    tests d'appartenance en coordonnées entières.
 *
 *  Limites :
 *    seuil 0,02 appliqué à une évaluation non normalisée du plan ; pertes
 *    subpixel et allocations vx/vy non vérifiées. Ne constitue pas un test
 *    exact 3D.
 *
 * ----------------------------------------------------------------------------
 */
static int edge_crosses_interior(Model3D* model, int ei, int fj) {
    double a, b, c, d;
    /* Construire le plan et rejeter le cas dégénéré. */
    compute_face_plane(model, fj, &a, &b, &c, &d);
    if (a == 0.0 && b == 0.0 && c == 0.0) return 0;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int cnt = faces->vertex_count[ei]; if (cnt < 2) return 0;

    double abs_a = fabs(a), abs_b = fabs(b), abs_c = fabs(c);
    int proj = (abs_a >= abs_b && abs_a >= abs_c) ? 0
            : (abs_b >= abs_a && abs_b >= abs_c) ? 1 : 2;

    int nfj = faces->vertex_count[fj];
    /* Projeter le contour de fj sur l'axe dominant ; cette routine suppose
     * les allocations réussies.
     */
    int *vx = (int*)malloc(sizeof(int) * nfj);
    int *vy = (int*)malloc(sizeof(int) * nfj);
    for (int m = 0; m < nfj; ++m) {
        int vi = faces->vertex_indices_buffer[faces->vertex_indices_ptr[fj] + m] - 1;
        Fixed32 X = vtx->x[vi], Y = vtx->y[vi], Z = vtx->z[vi];
        int ix, iy;
        if (proj == 0) { ix = FIXED_TO_INT(Y); iy = FIXED_TO_INT(Z); }
        else if (proj == 1) { ix = FIXED_TO_INT(X); iy = FIXED_TO_INT(Z); }
        else { ix = FIXED_TO_INT(X); iy = FIXED_TO_INT(Y); }
        vx[m] = ix; vy[m] = iy;
    }


    int result = 0;
    /* Parcourir les segments de ei, évaluer les côtés et projeter leurs
     * intersections avec le plan.
     */
    for (int k = 0; k < cnt && !result; ++k) {
        int vi = faces->vertex_indices_buffer[faces->vertex_indices_ptr[ei] + k] - 1;
        int vj = faces->vertex_indices_buffer[faces->vertex_indices_ptr[ei] + ((k+1)%cnt)] - 1;
        double Ax = (double)vtx->x[vi], Ay = (double)vtx->y[vi], Az = (double)vtx->z[vi];
        double Bx = (double)vtx->x[vj], By = (double)vtx->y[vj], Bz = (double)vtx->z[vj];
        const double plane_eps = 0.02;
        double sdA = a*Ax + b*Ay + c*Az + d;
        double sdB = a*Bx + b*By + c*Bz + d;
        if (fabs(sdA) < plane_eps) sdA = 0.0;
        if (fabs(sdB) < plane_eps) sdB = 0.0;
        if (sdA * sdB < 0.0) {
            double t = -sdA / (sdB - sdA);
            double Px = Ax + t*(Bx - Ax);
            double Py = Ay + t*(By - Ay);
            double Pz = Az + t*(Bz - Az);
            int px, py;
            if (proj == 0) { px = FIXED_TO_INT((Fixed32)Py); py = FIXED_TO_INT((Fixed32)Pz); }
            else if (proj == 1) { px = FIXED_TO_INT((Fixed32)Px); py = FIXED_TO_INT((Fixed32)Pz); }
            else { px = FIXED_TO_INT((Fixed32)Px); py = FIXED_TO_INT((Fixed32)Py); }
            if (strict_inside(px, py, nfj, vx, vy)) result = 1;
        }
    }
    /* Libérer les deux contours temporaires avant de rendre le résultat. */
    free(vx); free(vy);
    return result;
}
/*
 * ----------------------------------------------------------------------------
 *  compute_face_plane_float
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Calculer le plan défini par les trois premiers sommets d'une face
 *    objet.
 *
 *  Paramètres :
 *    [entrée] model et fidx à base zéro.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    *a, *b, *c, *d
 *
 *  Fonctionnement :
 *    Une face invalide ou de moins de trois sommets donne un plan nul. Le
 *    produit vectoriel n'est pas normalisé ; trois sommets colinéaires
 *    donnent un plan dégénéré même si d'autres sommets de la face
 *    définissent un plan. Les coordonnées 16.16 sont converties en unités
 *    objet avant le calcul.
 *
 * ----------------------------------------------------------------------------
 */
static void compute_face_plane_float(Model3D* model, int fidx,
                                     double *a, double *b, double *c, double *d) {
    /* Traiter les arguments invalides et les faces trop courtes avant les
     * accès aux coordonnées.
     */
    if (!model || fidx < 0 || fidx >= model->faces.face_count) {
        *a = *b = *c = *d = 0.0;
        return;
    }
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int cnt = faces->vertex_count[fidx];
    if (cnt < 3) {
        *a = *b = *c = *d = 0.0;
        return;
    }
    /* Lire le triangle initial de la face dans le repère objet. */
    int off = faces->vertex_indices_ptr[fidx];
    int vi0 = faces->vertex_indices_buffer[off] - 1;
    int vi1 = faces->vertex_indices_buffer[off + 1] - 1;
    int vi2 = faces->vertex_indices_buffer[off + 2] - 1;
    double x0 = FIXED_TO_FLOAT(vtx->x[vi0]);
    double y0 = FIXED_TO_FLOAT(vtx->y[vi0]);
    double z0 = FIXED_TO_FLOAT(vtx->z[vi0]);
    double x1 = FIXED_TO_FLOAT(vtx->x[vi1]);
    double y1 = FIXED_TO_FLOAT(vtx->y[vi1]);
    double z1 = FIXED_TO_FLOAT(vtx->z[vi1]);
    double x2 = FIXED_TO_FLOAT(vtx->x[vi2]);
    double y2 = FIXED_TO_FLOAT(vtx->y[vi2]);
    double z2 = FIXED_TO_FLOAT(vtx->z[vi2]);
    /* Construire la normale par produit vectoriel puis choisir d pour que le
     * premier sommet appartienne au plan.
     */
    double ux = x1 - x0, uy = y1 - y0, uz = z1 - z0;
    double vx = x2 - x0, vy = y2 - y0, vz = z2 - z0;
    *a = uy * vz - uz * vy;
    *b = uz * vx - ux * vz;
    *c = ux * vy - uy * vx;
    *d = -(*a * x0 + *b * y0 + *c * z0);
}
/*
 * ----------------------------------------------------------------------------
 *  compute_face_plane
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Calculer le plan défini par les trois premiers sommets d'une face
 *    objet.
 *
 *  Paramètres :
 *    [entrée] model et fidx à base zéro.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    *a, *b, *c, *d
 *
 *  Fonctionnement :
 *    Une face invalide ou de moins de trois sommets donne un plan nul. Le
 *    produit vectoriel n'est pas normalisé ; trois sommets colinéaires
 *    donnent un plan dégénéré même si d'autres sommets de la face
 *    définissent un plan. Les doubles contiennent les entiers 16.16 bruts :
 *    les coefficients ne sont pas à la même échelle que
 *    compute_face_plane_float.
 *
 * ----------------------------------------------------------------------------
 */
static void compute_face_plane(Model3D* model, int fidx,
                               double *a, double *b, double *c, double *d) {
    /* Traiter les arguments invalides et les faces trop courtes avant les
     * accès aux coordonnées.
     */
    if (!model || fidx < 0 || fidx >= model->faces.face_count) {
        *a = *b = *c = *d = 0.0;
        return;
    }
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int cnt = faces->vertex_count[fidx];
    if (cnt < 3) {
        *a = *b = *c = *d = 0.0;
        return;
    }
    /* Lire le triangle initial de la face dans le repère objet. */
    int off = faces->vertex_indices_ptr[fidx];
    int vi0 = faces->vertex_indices_buffer[off] - 1;
    int vi1 = faces->vertex_indices_buffer[off + 1] - 1;
    int vi2 = faces->vertex_indices_buffer[off + 2] - 1;
    double x0 = (double)vtx->x[vi0];
    double y0 = (double)vtx->y[vi0];
    double z0 = (double)vtx->z[vi0];
    double x1 = (double)vtx->x[vi1];
    double y1 = (double)vtx->y[vi1];
    double z1 = (double)vtx->z[vi1];
    double x2 = (double)vtx->x[vi2];
    double y2 = (double)vtx->y[vi2];
    double z2 = (double)vtx->z[vi2];
    /* Construire la normale par produit vectoriel puis choisir d pour que le
     * premier sommet appartienne au plan.
     */
    double ux = x1 - x0, uy = y1 - y0, uz = z1 - z0;
    double vx = x2 - x0, vy = y2 - y0, vz = z2 - z0;
    *a = uy * vz - uz * vy;
    *b = uz * vx - ux * vz;
    *c = ux * vy - uy * vx;
    *d = -(*a * x0 + *b * y0 + *c * z0);
}
/*
 * ----------------------------------------------------------------------------
 *  strict_inside
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Tester l'appartenance stricte d'un point à un polygone entier.
 *
 *  Paramètres :
 *    [entrée] px/py ; nfj_local, taille ; vx_local/vy_local, coordonnées des
 *    sommets.
 *
 *  Retour :
 *    1 uniquement à l'intérieur hors contour, 0 sinon.
 *
 *  Fonctionnement :
 *    Aucun effet de bord. Filtre d'abord par point_in_poly_arrays_int puis
 *    exclut explicitement les sommets et les points alignés situés sur une
 *    arête avec des produits 64 bits.
 *
 * ----------------------------------------------------------------------------
 */
static int strict_inside(int px, int py, int nfj_local, int *vx_local, int *vy_local) {
    int m;

    /* Écarter les points déjà hors du polygone. */
    if (!point_in_poly_arrays_int(px, py, nfj_local, vx_local, vy_local))
        return 0;

    /* Rejeter les points du contour, y compris ses extrémités. */
    for (m = 0; m < nfj_local; ++m) {
        int x1 = vx_local[m], y1 = vy_local[m];
        if (px == x1 && py == y1) return 0;
        {
            int m2 = (m + 1) % nfj_local;
            int x2 = vx_local[m2], y2 = vy_local[m2];
            long long dx = (long long)x2 - x1, dy = (long long)y2 - y1;
            long long dxp = (long long)px - x1, dyp = (long long)py - y1;
            if (dx * dyp == dy * dxp) {

                if ((px >= x1 && px <= x2) || (px >= x2 && px <= x1)) {
                    if ((py >= y1 && py <= y2) || (py >= y2 && py <= y1))
                        return 0;
                }
            }
        }
    }
    return 1;
}
/*
 * ----------------------------------------------------------------------------
 *  count_edge_crossings
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Compter les arêtes de ei qui traversent fj ou y possèdent une extrémité
 *    coplanaire intérieure.
 *
 *  Paramètres :
 *    [entrée] model, ei et fj, identifiants de faces à base zéro.
 *
 *  Retour :
 *    Nombre de traversées comptées.
 *
 *  Fonctionnement :
 *    Aucun changement de géométrie. Calcule le plan objet de fj sur les
 *    entiers 16.16 bruts, choisit sa projection dominante puis quantifie les
 *    tests d'appartenance en coordonnées entières.
 *
 *  Limites :
 *    seuil 0,02 appliqué à une évaluation non normalisée du plan ; pertes
 *    subpixel et allocations vx/vy non vérifiées. Ne constitue pas un test
 *    exact 3D.
 *
 * ----------------------------------------------------------------------------
 */
static int count_edge_crossings(Model3D* model, int ei, int fj) {
    int count = 0;
    double a, b, c, d;
    /* Construire le plan et rejeter le cas dégénéré. */
    compute_face_plane(model, fj, &a, &b, &c, &d);
    if (a == 0.0 && b == 0.0 && c == 0.0) return 0;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int cnt = faces->vertex_count[ei];
    if (cnt < 2) return 0;
    double abs_a = fabs(a), abs_b = fabs(b), abs_c = fabs(c);
    int proj;
    if (abs_a >= abs_b && abs_a >= abs_c) proj = 0;
    else if (abs_b >= abs_a && abs_b >= abs_c) proj = 1;
    else proj = 2;
    int nfj = faces->vertex_count[fj];
    /* Projeter le contour de fj sur l'axe dominant ; cette routine suppose
     * les allocations réussies.
     */
    int *vx = (int*)malloc(sizeof(int) * nfj);
    int *vy = (int*)malloc(sizeof(int) * nfj);
    for (int m = 0; m < nfj; ++m) {
        int vi = faces->vertex_indices_buffer[faces->vertex_indices_ptr[fj] + m] - 1;
        Fixed32 X = vtx->x[vi], Y = vtx->y[vi], Z = vtx->z[vi];
        int ix, iy;
        if (proj == 0) { ix = FIXED_TO_INT(Y); iy = FIXED_TO_INT(Z); }
        else if (proj == 1) { ix = FIXED_TO_INT(X); iy = FIXED_TO_INT(Z); }
        else { ix = FIXED_TO_INT(X); iy = FIXED_TO_INT(Y); }
        vx[m] = ix;
        vy[m] = iy;
    }
    /* Parcourir les segments de ei, évaluer les côtés et projeter leurs
     * intersections avec le plan.
     */
    for (int k = 0; k < cnt; ++k) {
        int vi = faces->vertex_indices_buffer[faces->vertex_indices_ptr[ei] + k] - 1;
        int vj = faces->vertex_indices_buffer[faces->vertex_indices_ptr[ei] + ((k + 1) % cnt)] - 1;
        double Ax = (double)vtx->x[vi], Ay = (double)vtx->y[vi], Az = (double)vtx->z[vi];
        double Bx = (double)vtx->x[vj], By = (double)vtx->y[vj], Bz = (double)vtx->z[vj];
        const double plane_eps = 0.02;
        double sdA = a*Ax + b*Ay + c*Az + d;
        double sdB = a*Bx + b*By + c*Bz + d;
        if (fabs(sdA) < plane_eps) sdA = 0.0;
        if (fabs(sdB) < plane_eps) sdB = 0.0;
        if (sdA == 0.0 && sdB == 0.0) {
            int pax, pay, pbx, pby;
            if (proj == 0) { pax = FIXED_TO_INT((Fixed32)Ay); pay = FIXED_TO_INT((Fixed32)Az);
                             pbx = FIXED_TO_INT((Fixed32)By); pby = FIXED_TO_INT((Fixed32)Bz); }
            else if (proj == 1) { pax = FIXED_TO_INT((Fixed32)Ax); pay = FIXED_TO_INT((Fixed32)Az);
                                  pbx = FIXED_TO_INT((Fixed32)Bx); pby = FIXED_TO_INT((Fixed32)Bz); }
            else { pax = FIXED_TO_INT((Fixed32)Ax); pay = FIXED_TO_INT((Fixed32)Ay);
                   pbx = FIXED_TO_INT((Fixed32)Bx); pby = FIXED_TO_INT((Fixed32)By); }
            if (point_in_poly_arrays_int(pax, pay, nfj, vx, vy) ||
                point_in_poly_arrays_int(pbx, pby, nfj, vx, vy)) {
                count++;
            }
            continue;
        }
        if (sdA * sdB < 0.0) {
            double t = -sdA / (sdB - sdA);
            double Px = Ax + t*(Bx - Ax);
            double Py = Ay + t*(By - Ay);
            double Pz = Az + t*(Bz - Az);
            int px, py;
            if (proj == 0) { px = FIXED_TO_INT((Fixed32)Py); py = FIXED_TO_INT((Fixed32)Pz); }
            else if (proj == 1) { px = FIXED_TO_INT((Fixed32)Px); py = FIXED_TO_INT((Fixed32)Pz); }
            else { px = FIXED_TO_INT((Fixed32)Px); py = FIXED_TO_INT((Fixed32)Py); }
            if (point_in_poly_arrays_int(px, py, nfj, vx, vy)) {
                count++;
            }
        }
    }
    /* Libérer les deux contours temporaires avant de rendre le résultat. */
    free(vx); free(vy);
    return count;
}

/*
 * ----------------------------------------------------------------------------
 *  split_face_by_plane
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Découper expérimentalement f1 par le plan objet de f2.
 *
 *  Paramètres :
 *    [entrée-sortie] model, dont sommets, faces et buffer d'indices peuvent
 *    croître.
 *
 *    [entrée] f1/f2 à base zéro ; constraint_verts et constraint_n, segment
 *    de contrainte facultatif à indices base zéro ; force_one_edge, permet
 *    de poursuivre sans sommets strictement des deux côtés.
 *
 *  Retour :
 *    1 après réécriture de f1 et ajout éventuel des autres contours, 0
 *    sinon.
 *
 *  Sorties et effets de bord :
 *    out_newverts et out_n, nouveaux indices à base zéro, au plus
 *    MAX_FACE_VERTICES.
 *
 *  Fonctionnement :
 *    Classe les sommets, crée les intersections, déduplique et trie les
 *    contours, puis réécrit les listes à base un et recalcule les bornes 3D.
 *    Ce traitement n'est pas transactionnel : de nouveaux sommets peuvent
 *    subsister après un retour 0. Les capacités de faces et d'indices ne
 *    sont pas contrôlées ici. Les deux fonctions de plan utilisées ont des
 *    échelles différentes ; les tolérances et contraintes doivent être
 *    réévaluées avant une rationalisation.
 *
 * ----------------------------------------------------------------------------
 */
static int split_face_by_plane(Model3D* model, int f1, int f2,
                               int *out_newverts, int *out_n,
                               int *constraint_verts, int constraint_n,
                               int force_one_edge) {
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int cnt = faces->vertex_count[f1];
    if (cnt < 3) return 0;

    double a, b, c, d;
    /* Calculer le plan de coupe dans les unités objet, puis réserver les
     * classifications.
     */
    compute_face_plane_float(model, f2, &a, &b, &c, &d);
    if (a == 0.0 && b == 0.0 && c == 0.0) return 0;
    const double plane_eps = 0.02;

    double *dist = (double*)malloc(sizeof(double) * cnt);
    int *orig = (int*)malloc(sizeof(int) * cnt);
    if (!dist || !orig) {
        free(dist);
        free(orig);
        return 0;
    }
    /* Évaluer la distance signée non normalisée de chaque sommet au plan. */
    for (int i = 0; i < cnt; ++i) {
        int vi = faces->vertex_indices_buffer[faces->vertex_indices_ptr[f1] + i] - 1;
        orig[i] = vi;
        double x = FIXED_TO_FLOAT(vtx->x[vi]);
        double y = FIXED_TO_FLOAT(vtx->y[vi]);
        double z = FIXED_TO_FLOAT(vtx->z[vi]);
        double sd = a * x + b * y + c * z + d;
        if (fabs(sd) < plane_eps) sd = 0.0;
        dist[i] = sd;
    }
    /* Vérifier si les deux côtés sont présents, sauf demande de poursuite forcée. */
    int hasPos = 0, hasNeg = 0;
    for (int i = 0; i < cnt; ++i) {
        if (dist[i] > 0.0) hasPos = 1;
        if (dist[i] < 0.0) hasNeg = 1;
    }
    if (!hasPos || !hasNeg) {
        if (!force_one_edge) {
            free(dist);
            free(orig);
            return 0;
        } else {

        }
    }
    int pos_list[MAX_FACE_VERTICES * 2];
    int neg_list[MAX_FACE_VERTICES * 2];
    int cop_list[MAX_FACE_VERTICES];
    int pos_n = 0, neg_n = 0, cop_n = 0;

    int newbuf[MAX_FACE_VERTICES];
    int newcount = 0;

    double seg_min = 0, seg_max = 0;
    int have_constraint = 0;
    double dirx=0, diry=0, dirz=0;
    /* Construire une direction d'intersection et les limites du segment de
     * contrainte facultatif.
     */
    if (constraint_verts && constraint_n >= 2) {

        double a1,b1,c1,d1;
        compute_face_plane(model, f1, &a1, &b1, &c1, &d1);

        dirx = b * c1 - c * b1;
        diry = c * a1 - a * c1;
        dirz = a * b1 - b * a1;
        double len = sqrt(dirx*dirx + diry*diry + dirz*dirz);
        if (len > 0) {
            dirx /= len; diry /= len; dirz /= len;

            double p0x = FIXED_TO_FLOAT(model->vertices.x[constraint_verts[0]]);
            double p0y = FIXED_TO_FLOAT(model->vertices.y[constraint_verts[0]]);
            double p0z = FIXED_TO_FLOAT(model->vertices.z[constraint_verts[0]]);
            seg_min = seg_max = 0.0;
            for (int ci = 0; ci < constraint_n; ++ci) {
                int vi = constraint_verts[ci];
                double cx = FIXED_TO_FLOAT(model->vertices.x[vi]);
                double cy = FIXED_TO_FLOAT(model->vertices.y[vi]);
                double cz = FIXED_TO_FLOAT(model->vertices.z[vi]);
                double t = ( (cx - p0x) * dirx + (cy - p0y) * diry + (cz - p0z) * dirz );
                if (ci == 0 || t < seg_min) seg_min = t;
                if (ci == 0 || t > seg_max) seg_max = t;
            }
            have_constraint = 1;
            if (have_constraint) {
            }
        }
    }
    for (int i = 0; i < cnt; ++i) {
        int j = (i + 1) % cnt;
        double di = dist[i], dj = dist[j];
        int vi = orig[i], vj = orig[j];
        if (di == 0.0) cop_list[cop_n++] = vi;
        if (di >= 0.0) pos_list[pos_n++] = vi;
        if (di <= 0.0) neg_list[neg_n++] = vi;
        /* Créer un sommet pour chaque arête qui change strictement de côté,
         * sous réserve de la contrainte.
         */
        if (di * dj < 0.0) {
            double t = di / (di - dj);
            double ix = FIXED_TO_FLOAT(vtx->x[vi]) + t * (FIXED_TO_FLOAT(vtx->x[vj]) - FIXED_TO_FLOAT(vtx->x[vi]));
            double iy = FIXED_TO_FLOAT(vtx->y[vi]) + t * (FIXED_TO_FLOAT(vtx->y[vj]) - FIXED_TO_FLOAT(vtx->y[vi]));
            double iz = FIXED_TO_FLOAT(vtx->z[vi]) + t * (FIXED_TO_FLOAT(vtx->z[vj]) - FIXED_TO_FLOAT(vtx->z[vi]));

            if (have_constraint) {
                int refvi = constraint_verts[0];
                double rpx = FIXED_TO_FLOAT(model->vertices.x[refvi]);
                double rpy = FIXED_TO_FLOAT(model->vertices.y[refvi]);
                double rpz = FIXED_TO_FLOAT(model->vertices.z[refvi]);
                double pt = ((ix - rpx) * dirx + (iy - rpy) * diry + (iz - rpz) * dirz);
                if (pt < seg_min - 1e-6 || pt > seg_max + 1e-6) {
                    continue;
                }
            }
            int newvi = add_vertex(model, FLOAT_TO_FIXED((float)ix),
                                           FLOAT_TO_FIXED((float)iy),
                                           FLOAT_TO_FIXED((float)iz));
            if (newvi >= 0) {
                pos_list[pos_n++] = newvi;
                neg_list[neg_n++] = newvi;
                if (out_newverts && newcount < MAX_FACE_VERTICES) {
                    newbuf[newcount++] = newvi;
                }
            }
        }
    }
    /* Publier les indices nouveaux puis libérer les tableaux de classement. */
    if (out_newverts && out_n) {
        *out_n = newcount;
        for (int k = 0; k < newcount; ++k) out_newverts[k] = newbuf[k];
    }
    free(dist);
    free(orig);
    /* Nettoyer les listes et réordonner les contours ; les tests d'aire
     * suivants ne rejettent pas effectivement les petites aires.
     */
    cleanup_list(pos_list, &pos_n);
    cleanup_list(neg_list, &neg_n);
    cleanup_list(cop_list, &cop_n);
    remove_duplicates(pos_list, &pos_n);
    remove_duplicates(neg_list, &neg_n);
    if (cop_n >= 3) remove_duplicates(cop_list, &cop_n);


    if (!force_one_edge && (pos_n < 3 || neg_n < 3)) return 0;

    reorder_poly(model, f1, pos_list, &pos_n);
    reorder_poly(model, f1, neg_list, &neg_n);


    double a1, b1, c1, d1;
    compute_face_plane(model, f1, &a1, &b1, &c1, &d1);
    double area_pos = poly_signed_area(model, pos_list, pos_n, a1, b1, c1);
    double area_neg = poly_signed_area(model, neg_list, neg_n, a1, b1, c1);
    if (fabs(area_pos) < 0.001 || fabs(area_neg) < 0.001) {


    }
    reorder_poly(model, f1, pos_list, &pos_n);
    reorder_poly(model, f1, neg_list, &neg_n);
    int orig_cnt = faces->vertex_count[f1];
    int orig_off = faces->vertex_indices_ptr[f1];


    int *keep_list = NULL, keep_n = 0;
    int *other_list = NULL, other_n = 0;
    if (pos_n >= 3 && (neg_n < 3 || !force_one_edge)) {
        keep_list = pos_list; keep_n = pos_n;
        other_list = neg_list; other_n = neg_n;
    } else if (neg_n >= 3 && (pos_n < 3 || force_one_edge && pos_n < 3)) {
        keep_list = neg_list; keep_n = neg_n;
        other_list = pos_list; other_n = pos_n;
    } else {

        keep_list = pos_list; keep_n = pos_n;
        other_list = neg_list; other_n = neg_n;
    }
    /* Réutiliser l'ancien emplacement s'il suffit, sinon ajouter une
     * nouvelle liste au buffer compact.
     */
    if (keep_n <= orig_cnt) {
        for (int k = 0; k < keep_n; ++k)
            faces->vertex_indices_buffer[orig_off + k] = keep_list[k] + 1;
        faces->vertex_count[f1] = keep_n;
    } else {
        int new_off = faces->total_indices;
        for (int k = 0; k < keep_n; ++k)
            faces->vertex_indices_buffer[new_off + k] = keep_list[k] + 1;
        faces->vertex_indices_ptr[f1] = new_off;
        faces->vertex_count[f1] = keep_n;
        faces->total_indices += keep_n;
    }
    /* Créer les autres faces lorsqu'elles possèdent au moins trois indices ;
     * conserver le drapeau d'affichage d'origine.
     */
    int orig_disp = faces->display_flag[f1];

    if (other_n >= 3) {
        int fneg = faces->face_count++;
        faces->vertex_count[fneg] = other_n;
        faces->vertex_indices_ptr[fneg] = faces->total_indices;
        for (int k = 0; k < other_n; ++k)
            faces->vertex_indices_buffer[faces->total_indices + k] = other_list[k] + 1;
        faces->total_indices += other_n;
        faces->sorted_face_indices[fneg] = fneg;
        faces->display_flag[fneg] = orig_disp;
    }
    if (cop_n >= 3) {
        int fcop = faces->face_count++;
        faces->vertex_count[fcop] = cop_n;
        faces->vertex_indices_ptr[fcop] = faces->total_indices;
        for (int k = 0; k < cop_n; ++k)
            faces->vertex_indices_buffer[faces->total_indices + k] = cop_list[k] + 1;
        faces->total_indices += cop_n;
        faces->sorted_face_indices[fcop] = fcop;
        faces->display_flag[fcop] = orig_disp;
    }
    /* Rafraîchir les bornes objet ; les données observateur restent à recalculer. */
    compute_face_bboxes3D(model);
    return 1;
}
/*
 * ----------------------------------------------------------------------------
 *  add_vertex
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Ajouter un sommet au repère objet du modèle.
 *
 *  Paramètres :
 *    [entrée] model et x/y/z en 16.16.
 *
 *  Retour :
 *    Indice à base zéro, ou -1 pour modèle NULL ou capacité MAX_VERTICES
 *    atteinte.
 *
 *  Sorties et effets de bord :
 *    coordonnées, compteur augmenté, coordonnées observateur/écran
 *    initialisées à zéro et cache fractionnaire invalidé.
 *
 *  Fonctionnement :
 *    La transformation de vue devra être recalculée avant d'utiliser le
 *    sommet au rendu.
 *
 * ----------------------------------------------------------------------------
 */
static int add_vertex(Model3D* model, Fixed32 x, Fixed32 y, Fixed32 z) {
    /* Valider le modèle et la capacité avant l'écriture. */
    if (!model) return -1;
    VertexArrays3D* vtx = &model->vertices;
    if (vtx->vertex_count >= MAX_VERTICES) return -1;
    int idx = vtx->vertex_count;
    vtx->x[idx]  = x;
    vtx->y[idx]  = y;
    vtx->z[idx]  = z;
    /* Invalider les projections et marquer les coordonnées dérivées comme à
     * recalculer.
     */
    fractional_projection_cache_invalidate(model);
    vtx->xo[idx] = 0;
    vtx->yo[idx] = 0;
    vtx->zo[idx] = 0;
    vtx->x2d[idx] = 0;
    vtx->y2d[idx] = 0;
    vtx->vertex_count++;
    return idx;
}
/*
 * ----------------------------------------------------------------------------
 *  cleanup_list
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Compacter une liste cyclique d'indices en retirant les répétitions
 *    adjacentes.
 *
 *  Paramètres :
 *    [entrée-sortie] arr et *n.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Supprime aussi la répétition du premier indice en fin de liste ; les
 *    répétitions non adjacentes restent présentes.
 *
 * ----------------------------------------------------------------------------
 */
static void cleanup_list(int *arr, int *n) {
    if (*n <= 1) return;
    int w = 0;
    /* Conserver la première occurrence de chaque série adjacente. */
    for (int i = 0; i < *n; ++i) {
        if (i == 0 || arr[i] != arr[i-1]) {
            arr[w++] = arr[i];
        }
    }
    /* Retirer une fermeture explicite du cycle et publier la nouvelle longueur. */
    if (w > 1 && arr[0] == arr[w-1]) w--;
    *n = w;
}
/*
 * ----------------------------------------------------------------------------
 *  remove_duplicates
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Retirer toutes les répétitions d'identifiants en conservant leur
 *    premier ordre.
 *
 *  Paramètres :
 *    [entrée-sortie] arr et *n.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Parcours quadratique de la liste, sans allocation ; ne compare pas les
 *    coordonnées des sommets.
 *
 * ----------------------------------------------------------------------------
 */
static void remove_duplicates(int *arr, int *n) {
    int w = 0;
    /* Rechercher chaque valeur dans le préfixe déjà conservé avant de la recopier. */
    for (int i = 0; i < *n; ++i) {
        int found = 0;
        for (int j = 0; j < w; ++j) {
            if (arr[i] == arr[j]) { found = 1; break; }
        }
        if (!found) arr[w++] = arr[i];
    }
    *n = w;
}
/*
 * ----------------------------------------------------------------------------
 *  poly_signed_area
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Mesurer l'aire orientée de la projection dominante d'un contour objet.
 *
 *  Paramètres :
 *    [entrée] model ; list, indices à base zéro ; n ; a/b/c, normale
 *    choisissant l'axe à éliminer.
 *
 *  Retour :
 *    Aire signée en unités objet carrées, 0 pour n < 3.
 *
 *  Fonctionnement :
 *    Ce n'est pas l'aire 3D normalisée du polygone. Aucun effet de bord.
 *
 * ----------------------------------------------------------------------------
 */
static double poly_signed_area(Model3D* model, int *list, int n,
                               double a, double b, double c) {
    if (n < 3) return 0.0;
    /* Éliminer la composante de normale de plus grande amplitude pour
     * choisir la projection.
     */
    double abs_a = fabs(a), abs_b = fabs(b), abs_c = fabs(c);
    int proj;
    if (abs_a >= abs_b && abs_a >= abs_c) proj = 0;
    else if (abs_b >= abs_a && abs_b >= abs_c) proj = 1;
    else proj = 2;
    VertexArrays3D* vtx = &model->vertices;
    double area = 0;
    /* Appliquer la somme cyclique des déterminants puis diviser par deux. */
    for (int i = 0; i < n; ++i) {
        int vi = list[i];
        int vj = list[(i + 1) % n];
        double x1,y1,x2,y2;
        if (proj == 0) {
            x1 = FIXED_TO_FLOAT(vtx->y[vi]); y1 = FIXED_TO_FLOAT(vtx->z[vi]);
            x2 = FIXED_TO_FLOAT(vtx->y[vj]); y2 = FIXED_TO_FLOAT(vtx->z[vj]);
        } else if (proj == 1) {
            x1 = FIXED_TO_FLOAT(vtx->x[vi]); y1 = FIXED_TO_FLOAT(vtx->z[vi]);
            x2 = FIXED_TO_FLOAT(vtx->x[vj]); y2 = FIXED_TO_FLOAT(vtx->z[vj]);
        } else {
            x1 = FIXED_TO_FLOAT(vtx->x[vi]); y1 = FIXED_TO_FLOAT(vtx->y[vi]);
            x2 = FIXED_TO_FLOAT(vtx->x[vj]); y2 = FIXED_TO_FLOAT(vtx->y[vj]);
        }
        area += (x1 * y2 - x2 * y1);
    }
    return area * 0.5;
}
/*
 * ----------------------------------------------------------------------------
 *  reorder_poly
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Réordonner une liste de sommets par angle autour de leur centre moyen
 *    projeté.
 *
 *  Paramètres :
 *    [entrée] model et fidx, plan de référence ; *n, longueur.
 *
 *    [entrée-sortie] list, indices à base zéro.
 *
 *  Retour :
 *    Aucun ; laisse la liste inchangée sur échec de réserve.
 *
 *  Fonctionnement :
 *    Projection selon la normale dominante, atan2 puis tri par insertion des
 *    indices.
 *
 *  Limites :
 *    le test d'aire utilise les tableaux px/py dans leur ordre d'origine,
 *    pas dans l'ordre des indices triés ; ce n'est pas une garantie générale
 *    de winding. Un tri angulaire n'est pas une reconstruction générale des
 *    contours concaves.
 *
 * ----------------------------------------------------------------------------
 */
static void reorder_poly(Model3D* model, int fidx, int *list, int *n) {
    if (!model || *n < 3) return;

    double a, b, c, d;
    compute_face_plane(model, fidx, &a, &b, &c, &d);
    /* Choisir une projection 2D du plan de référence. */
    double abs_a = fabs(a), abs_b = fabs(b), abs_c = fabs(c);
    int proj;
    if (abs_a >= abs_b && abs_a >= abs_c) proj = 0;
    else if (abs_b >= abs_a && abs_b >= abs_c) proj = 1;
    else proj = 2;

    /* Réserver les coordonnées projetées ; abandonner sans changer la liste
     * si la réserve échoue.
     */
    double *px = (double*)malloc(sizeof(double) * (*n));
    double *py = (double*)malloc(sizeof(double) * (*n));
    if (!px || !py) {
        free(px); free(py);
        return;
    }
    VertexArrays3D* vtx = &model->vertices;
    for (int i = 0; i < *n; ++i) {
        int vi = list[i];
        double X = (double)vtx->x[vi];
        double Y = (double)vtx->y[vi];
        double Z = (double)vtx->z[vi];
        if (proj == 0) { px[i] = Y; py[i] = Z; }
        else if (proj == 1) { px[i] = X; py[i] = Z; }
        else { px[i] = X; py[i] = Y; }
    }

    /* Calculer le centre moyen puis l'angle de chaque sommet. */
    double cx = 0, cy = 0;
    for (int i = 0; i < *n; ++i) { cx += px[i]; cy += py[i]; }
    cx /= *n; cy /= *n;

    double *ang = (double*)malloc(sizeof(double) * (*n));
    if (!ang) { free(px); free(py); return; }
    for (int i = 0; i < *n; ++i) {
        ang[i] = atan2(py[i] - cy, px[i] - cx);
    }

    /* Trier les indices par angle croissant. */
    for (int i = 1; i < *n; ++i) {
        int idx = list[i];
        double a0 = ang[i];
        int j = i - 1;
        while (j >= 0 && ang[j] > a0) {
            ang[j+1] = ang[j];
            list[j+1] = list[j];
            j--;
        }
        ang[j+1] = a0;
        list[j+1] = idx;
    }

    /* Évaluer le signe du contour px/py d'origine et inverser la liste si ce
     * signe est négatif.
     */
    double area = 0;
    for (int i = 0; i < *n; ++i) {
        int k = (i + 1) % *n;
        double x1 = px[i], y1 = py[i];
        double x2 = px[k], y2 = py[k];
        area += (x1 * y2 - x2 * y1);
    }
    if (area < 0) {

        for (int i = 0; i < *n/2; ++i) {
            int t = list[i]; list[i] = list[*n-1-i]; list[*n-1-i] = t;
        }
    }
    /* Libérer les trois tableaux temporaires. */
    free(px); free(py); free(ang);
}
/*
 * ----------------------------------------------------------------------------
 *  compute_face_bboxes3D
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Recalculer les six bornes objet de chaque face.
 *
 *  Paramètres :
 *    [entrée-sortie] model.
 *
 *  Retour :
 *    Aucun ; modèle NULL ou sans face est ignoré.
 *
 *  Fonctionnement :
 *    Remplace les six tableaux min/max en 16.16. Sur échec d'allocation, les
 *    libère tous et les remet à NULL. Les indices des faces doivent être
 *    valides. L'appelant doit tenir compte de l'absence possible de bornes
 *    après cet échec.
 *
 * ----------------------------------------------------------------------------
 */
static void compute_face_bboxes3D(Model3D* model) {
    if (!model) return;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int fc = faces->face_count;
    if (fc <= 0) return;

    /* Libérer les anciennes bornes avant de réserver un jeu dimensionné au
     * nombre courant de faces.
     */
    if (faces->minx3) free(faces->minx3);
    if (faces->maxx3) free(faces->maxx3);
    if (faces->miny3) free(faces->miny3);
    if (faces->maxy3) free(faces->maxy3);
    if (faces->minz3) free(faces->minz3);
    if (faces->maxz3) free(faces->maxz3);
    faces->minx3 = (Fixed32*)malloc(fc * sizeof(Fixed32));
    faces->maxx3 = (Fixed32*)malloc(fc * sizeof(Fixed32));
    faces->miny3 = (Fixed32*)malloc(fc * sizeof(Fixed32));
    faces->maxy3 = (Fixed32*)malloc(fc * sizeof(Fixed32));
    faces->minz3 = (Fixed32*)malloc(fc * sizeof(Fixed32));
    faces->maxz3 = (Fixed32*)malloc(fc * sizeof(Fixed32));
    /* Ramener les six pointeurs à NULL si une allocation a échoué. */
    if (!faces->minx3||!faces->maxx3||!faces->miny3||!faces->maxy3||!faces->minz3||!faces->maxz3) {

        free(faces->minx3); free(faces->maxx3); free(faces->miny3);
        free(faces->maxy3); free(faces->minz3); free(faces->maxz3);
        faces->minx3 = faces->maxx3 = faces->miny3 = faces->maxy3 = faces->minz3 = faces->maxz3 = NULL;
        return;
    }
    /* Parcourir les sommets de chaque face ; une face vide conserve six
     * bornes nulles.
     */
    for (int fi = 0; fi < fc; ++fi) {
        int cnt = faces->vertex_count[fi];
        int off = faces->vertex_indices_ptr[fi];
        Fixed32 mnx=0,mxx=0,mny=0,mxy=0,mnz=0,mxz=0;
        for (int k = 0; k < cnt; ++k) {
            int vi = faces->vertex_indices_buffer[off + k] - 1;
            Fixed32 x = vtx->x[vi];
            Fixed32 y = vtx->y[vi];
            Fixed32 z = vtx->z[vi];
            if (k==0 || x < mnx) mnx = x;
            if (k==0 || x > mxx) mxx = x;
            if (k==0 || y < mny) mny = y;
            if (k==0 || y > mxy) mxy = y;
            if (k==0 || z < mnz) mnz = z;
            if (k==0 || z > mxz) mxz = z;
        }
        faces->minx3[fi] = mnx;
        faces->maxx3[fi] = mxx;
        faces->miny3[fi] = mny;
        faces->maxy3[fi] = mxy;
        faces->minz3[fi] = mnz;
        faces->maxz3[fi] = mxz;
    }
}

/* Transformation de vue et algorithmes d'ordre arrière-vers-avant. */
segment "painter_core";
/*
 * ----------------------------------------------------------------------------
 *  processModelFast
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Transformer les sommets, projeter, calculer les faces et exécuter la méthode
 *    de classement choisie.
 *
 *  Paramètres :
 *    [entrée] model, géométrie chargée ; params, angles en degrés et
 *    distance 16.16 ; filename, argument conservé par l'interface mais non
 *    utilisé ici.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    xo/yo/zo, x2d/y2d, métriques et permutation des faces, couleurs de
 *    shading.
 *
 *  Fonctionnement :
 *    Précalcule la matrice de rotation puis projette les sommets de
 *    profondeur positive. Pour zo <= 0, écrit xo=yo=0 et x2d=y2d=-1. En
 *    filaire, calcule encore les métriques mais saute le tri. Invalide les
 *    caches dépendant de la transformation.
 *
 * ----------------------------------------------------------------------------
 */
void processModelFast(Model3D* model, ObserverParams* params, const char* filename) {
    /* Invalider les rapports de projection avant de remplacer les
     * coordonnées observateur.
     */
    fractional_projection_cache_invalidate(model);
    int i;
    Fixed32 cos_h, sin_h, cos_v, sin_v, cos_w, sin_w;
    Fixed32 x, y, z, zo, xo, yo;
    Fixed32 inv_zo;


    /* Lire les six valeurs trigonométriques une seule fois pour la vue. */
    cos_h = cos_deg_int(params->angle_h);
    sin_h = sin_deg_int(params->angle_h);
    cos_v = cos_deg_int(params->angle_v);
    sin_v = sin_deg_int(params->angle_v);
    cos_w = cos_deg_int(params->angle_w);
    sin_w = sin_deg_int(params->angle_w);

    /* Précalculer les produits de la matrice en 16.16. */
    const Fixed32 cos_h_cos_v = fixed_mul32_fast(cos_h, cos_v);
    const Fixed32 sin_h_cos_v = fixed_mul32_fast(sin_h, cos_v);
    const Fixed32 cos_h_sin_v = fixed_mul32_fast(cos_h, sin_v);
    const Fixed32 sin_h_sin_v = fixed_mul32_fast(sin_h, sin_v);

    Fixed32 M00, M01, M02;
    Fixed32 M10, M11, M12;
    Fixed32 M20, M21, M22;

    Fixed32 scale = s_global_proj_scale_fixed;
    const Fixed32 centre_x_f = INT_TO_FIXED(CENTRE_X);
    const Fixed32 centre_y_f = INT_TO_FIXED(CENTRE_Y);
    Fixed32 distance = params->distance;


    VertexArrays3D* vtx = &model->vertices;
    Fixed32 *x_arr = vtx->x, *y_arr = vtx->y, *z_arr = vtx->z;
    Fixed32 *xo_arr = vtx->xo, *yo_arr = vtx->yo, *zo_arr = vtx->zo;
    int *x2d_arr = vtx->x2d, *y2d_arr = vtx->y2d;
    int vcount = vtx->vertex_count;

    /* Construire la matrice ; le cas angle_w nul évite les produits de
     * rotation écran.
     */
    M20 = FIXED_NEG(cos_h_cos_v);
    M21 = FIXED_NEG(sin_h_cos_v);
    M22 = FIXED_NEG(sin_v);
    if (params->angle_w == 0) {
        M00 = FIXED_NEG(sin_h);
        M01 = cos_h;
        M02 = 0;
        M10 = FIXED_NEG(cos_h_sin_v);
        M11 = FIXED_NEG(sin_h_sin_v);
        M12 = cos_v;
    } else {
        M00 = FIXED_ADD(fixed_mul32_fast(FIXED_NEG(cos_w), sin_h),
                        fixed_mul32_fast(sin_w, cos_h_sin_v));
        M01 = FIXED_SUB(fixed_mul32_fast(cos_w, cos_h),
                        fixed_mul32_fast(sin_w, FIXED_NEG(sin_h_sin_v)));
        M02 = fixed_mul32_fast(FIXED_NEG(sin_w), cos_v);
        M10 = FIXED_ADD(fixed_mul32_fast(FIXED_NEG(sin_w), sin_h),
                        fixed_mul32_fast(FIXED_NEG(cos_w), cos_h_sin_v));
        M11 = FIXED_ADD(fixed_mul32_fast(sin_w, cos_h),
                        fixed_mul32_fast(FIXED_NEG(cos_w), sin_h_sin_v));
        M12 = fixed_mul32_fast(cos_w, cos_v);
    }
    /* Transformer chaque sommet et appliquer la perspective seulement devant
     * l'observateur.
     */
    for (i = 0; i < vcount; i++) {
        x = x_arr[i];
        y = y_arr[i];
        z = z_arr[i];
        zo = FIXED_ADD(
                FIXED_ADD(
                    FIXED_ADD(fixed_mul32_fast(M20, x),
                            fixed_mul32_fast(M21, y)),
                    fixed_mul32_fast(M22, z)),
                distance);
        if (zo > 0) {
            xo = FIXED_ADD(FIXED_ADD(fixed_mul32_fast(M00, x),
                                    fixed_mul32_fast(M01, y)),
                        fixed_mul32_fast(M02, z));
            yo = FIXED_ADD(FIXED_ADD(fixed_mul32_fast(M10, x),
                                    fixed_mul32_fast(M11, y)),
                        fixed_mul32_fast(M12, z));
            zo_arr[i] = zo;
            xo_arr[i] = xo;
            yo_arr[i] = yo;
            inv_zo = FIXED_DIV_64(scale, zo);
            x2d_arr[i] = FIXED_ROUND_TO_INT(
                            FIXED_ADD(fixed_mul32_fast(xo, inv_zo), centre_x_f));
            y2d_arr[i] = FIXED_ROUND_TO_INT(
                            FIXED_SUB(centre_y_f, fixed_mul32_fast(yo, inv_zo)));
        } else {
            zo_arr[i] = zo;
            xo_arr[i] = 0;
            yo_arr[i] = 0;
            x2d_arr[i] = -1;
            y2d_arr[i] = -1;
        }
    }

    /* Recalculer plans et visibilité, puis les couleurs par orientation si
     * demandées.
     */
    calculateFaceDepths(model, NULL, model->faces.face_count);
    if (shaded_by_orientation) {
        computeOrientationShading(model);
    }
    /* Le filaire ne nécessite pas l'ordre des surfaces ; conserver le tri
     * pour les autres modes.
     */
    if (framePolyOnly) goto skip_calc;


    /* Répartir les sept modes utilisateur vers leurs implémentations actuelles. */
    if (painter_mode == PAINTER_MODE_FAST) {
        painter_newell_sancha_fast(model, model->faces.face_count);
    } else if (painter_mode == PAINTER_MODE_BUBBLE_SORT) {
        painter_bubble_sort(model, model->faces.face_count);
    } else if (painter_mode == PAINTER_MODE_NEWELL_SANCHA) {
        painter_newell_sancha(model, model->faces.face_count);
    } else if (painter_mode == PAINTER_MODE_CORRECT) {

        painter_correct(model, model->faces.face_count, 0);
    } else if (painter_mode == PAINTER_MODE_GEO) {
        painter_geoV2(model, model->faces.face_count);
    } else if (painter_mode == PAINTER_MODE_CORRECTV2) {

        painter_correctV2(model, model->faces.face_count, 0);
    }
    else if (painter_mode == PAINTER_MODE_GEOV3) {
        painter_geoV3(model, model->faces.face_count);
    }
    skip_calc:;
}
/*
 * ----------------------------------------------------------------------------
 *  cmp_faces_by_zmean
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Comparer deux identifiants de faces pour qsort.
 *
 *  Paramètres :
 *    [entrée] pa/pb, pointeurs vers int ; qsort_faces_ptr_for_cmp, métriques
 *    partagées.
 *
 *  Retour :
 *    Négatif si a précède b, positif sinon, zéro si même identifiant.
 *
 *  Fonctionnement :
 *    Ordre décroissant de z_mean ; les égalités sont départagées par
 *    identifiant croissant. Non réentrant en raison du pointeur global de
 *    contexte.
 *
 * ----------------------------------------------------------------------------
 */
static int cmp_faces_by_zmean(const void* pa, const void* pb) {
    int a = *(const int*)pa;
    int b = *(const int*)pb;
    /* Lire les profondeurs puis départager les égalités par identifiant pour
     * stabiliser le résultat.
     */
    Fixed32 za = qsort_faces_ptr_for_cmp->z_mean[a];
    Fixed32 zb = qsort_faces_ptr_for_cmp->z_mean[b];
    if (za > zb) return -1;
    if (za < zb) return 1;
    if (a < b) return -1;
    if (a > b) return 1;
    return 0;
}


/* Paquet non réentrant : deux entiers 32 bits little-endian en entrée,
 * quatre mots de produit brut dans geo_mul_p.
 */
unsigned int geo_mul_inputs[4];
unsigned int geo_mul_p[4];

/*
 * ----------------------------------------------------------------------------
 *  GeoV3_Mul32x32
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Multiplier exactement deux entiers signés 32 bits et publier le produit
 *    64 bits brut.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Entrées et sorties implicites :
 *    [entrée] geo_mul_inputs, a puis b en quatre mots little-endian
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    geo_mul_p, quatre mots de poids faible à fort.
 *
 *  Fonctionnement :
 *    Aucun paramètre de pile C. Deux entrées 16.16 donnent un produit 32.32,
 *    sans arrondi ni saturation.
 *
 *    Calcul binaire et accès globaux longs. Non réentrant : les paquets
 *    globaux sont partagés.
 *
 *    Algorithme par magnitudes, addition et rotation combinées : chemins
 *    8/16/32 itérations, raccourci pour zéro et un, factorisation des mots
 *    bas nuls.
 *
 *  Registres et indicateurs :
 *    Préserve P, A, X et D ; Y et DBR ne sont pas modifiés.
 *
 *  Organisation mémoire :
 *    Cadre D de 24 octets : signe à 1 ; échange à 3 ; M à 5/7 ; compte à 9 ;
 *    facteur 65536 à 11 ; Q à 13/15 ; produit à 17/19/21/23.
 *
 * ----------------------------------------------------------------------------
 */
asm void GeoV3_Mul32x32(void)
{


    /* Sauvegarder le contexte, imposer les tailles 16 bits et désactiver le
     * mode décimal.
     */
    php
    rep     #0x30
    pha
    phx
    phd
    cld


    /* Réserver 24 octets sous les registres sauvegardés ; D=S, premier mot
     * utile à D+1.
     */
    tsc
    sec
    sbc     #24
    tcs
    tcd


    /* Former la magnitude de a ; un complément à deux conserve correctement
     * |INT32_MIN| sur 32 bits non signés.
     */
    lda     >geo_mul_inputs
    sta     5
    lda     >geo_mul_inputs+2
    sta     7
    lda     #0
    sta     1
    sta     11

    lda     7
    bpl     geo_mul_a_positive


    lda     5
    eor     #0xFFFF
    clc
    adc     #1
    sta     5
    lda     7
    eor     #0xFFFF
    adc     #0
    sta     7
    lda     #1
    sta     1

/* Former la magnitude de b et combiner les signes par XOR. */
geo_mul_a_positive:


    lda     >geo_mul_inputs+4
    sta     13
    lda     >geo_mul_inputs+6
    sta     15
    lda     15
    bpl     geo_mul_b_positive

    lda     13
    eor     #0xFFFF
    clc
    adc     #1
    sta     13
    lda     15
    eor     #0xFFFF
    adc     #0
    sta     15
    lda     1
    eor     #1
    sta     1

/* Initialiser le produit à zéro avant les sorties anticipées. */
geo_mul_b_positive:


    lda     #0
    sta     17
    sta     19
    sta     21
    sta     23


    lda     5
    ora     7
    bne     geo_mul_check_q_zero
    brl     geo_mul_store
/* Éviter toute itération si une magnitude est nulle. */
geo_mul_check_q_zero:
    lda     13
    ora     15
    bne     geo_mul_nonzero
    brl     geo_mul_store
/* Choisir le chemin court si le multiplicateur tient dans un mot. */
geo_mul_nonzero:


    ldx     #16
    lda     15
    bne     geo_mul_q_wide
    brl     geo_mul_short16
/* Échanger les magnitudes si cela permet de placer le petit opérande dans Q. */
geo_mul_q_wide:


    lda     7
    bne     geo_mul_select32
    lda     5
    sta     3
    lda     13
    sta     5
    lda     15
    sta     7
    lda     3
    sta     13
    stz     15
    bra     geo_mul_short16

/* Factoriser 65536 lorsque le mot bas d'un opérande est nul ; le facteur
 * sera restauré par déplacement de mots.
 */
geo_mul_select32:


    lda     13
    bne     geo_mul_check_m_integer
    lda     15
    sta     13
    stz     15
    lda     #1
    sta     11
    bra     geo_mul_short16
geo_mul_check_m_integer:
    lda     5
    bne     geo_mul_full_width
    lda     7
    sta     3
    lda     13
    sta     5
    lda     15
    sta     7
    lda     3
    sta     13
    stz     15
    lda     #1
    sta     11
    bra     geo_mul_short16
geo_mul_full_width:
    brl     geo_mul_use_32


/* Une multiplication par un devient une copie ; sinon choisir 8 ou 16 bits
 * significatifs de Q.
 */
geo_mul_short16:
    lda     13
    cmp     #1
    bne     geo_mul_short_select

    lda     5
    sta     17
    lda     7
    sta     19
    brl     geo_mul_short_finish
geo_mul_short_select:
    ldx     #16
    cmp     #256
    bcs     geo_mul_short_count
    ldx     #8
geo_mul_short_count:
    stx     9
    lda     13
    sta     17
/* Tester le prochain bit puis additionner M au haut du registre combiné si
 * nécessaire.
 */
geo_mul_short16_loop:
    lda     17
    lsr     a
    bcc     geo_mul_short16_no_add
    clc
    lda     19
    adc     5
    sta     19
    lda     21
    adc     7
    sta     21
    bra     geo_mul_short16_shift
geo_mul_short16_no_add:
    clc
/* Propager la retenue dans les trois mots par ROR ; aucun CLC ne doit
 * interrompre cette chaîne.
 */
geo_mul_short16_shift:
    ror     21
    ror     19
    ror     17
    dex
    bne     geo_mul_short16_loop
    /* Après huit tours, supprimer l'alignement résiduel de huit bits par
     * échanges d'octets XBA.
     */
    lda     9
    cmp     #8
    bne     geo_mul_short_finish


    lda     17
    xba
    and     #0x00FF
    sta     3
    lda     19
    xba
    and     #0xFF00
    ora     3
    sta     17
    lda     19
    xba
    and     #0x00FF
    sta     3
    lda     21
    xba
    and     #0xFF00
    ora     3
    sta     19
    lda     21
    xba
    and     #0x00FF
    sta     21
/* Restaurer le facteur 65536 en déplaçant les mots sans nouvelle boucle de
 * bits.
 */
geo_mul_short_finish:
    lda     11
    beq     geo_mul_short_aligned

    lda     21
    sta     23
    lda     19
    sta     21
    lda     17
    sta     19
    stz     17
geo_mul_short_aligned:
    brl     geo_mul_apply_sign

/* Préparer les 32 itérations du chemin général. */
geo_mul_use_32:
    ldx     #32

geo_mul_loop:


    lda     13
    sta     17
    lda     15
    sta     19

/* Ajouter M à la moitié haute si le bit bas de Q est un ; conserver le carry
 * final.
 */
geo_mul_combined_loop:


    lda     17
    lsr     a
    bcc     geo_mul_combined_no_add


    clc
    lda     21
    adc     5
    sta     21
    lda     23
    adc     7
    sta     23
    bra     geo_mul_combined_shift

geo_mul_combined_no_add:
    clc

/* Décaler carry:haut:bas sur quatre mots ; les bits de Q cèdent
 * progressivement leur place au produit.
 */
geo_mul_combined_shift:


    ror     23
    ror     21
    ror     19
    ror     17
    dex
    bne     geo_mul_combined_loop


/* Appliquer le signe final par complément à deux 64 bits, avec propagation
 * des retenues.
 */
geo_mul_apply_sign:


    lda     1
    beq     geo_mul_store

    lda     17
    eor     #0xFFFF
    clc
    adc     #1
    sta     17
    lda     19
    eor     #0xFFFF
    adc     #0
    sta     19
    lda     21
    eor     #0xFFFF
    adc     #0
    sta     21
    lda     23
    eor     #0xFFFF
    adc     #0
    sta     23

/* Exporter les mots du produit avant de libérer exactement les 24 octets et
 * de restaurer le contexte.
 */
geo_mul_store:


    lda     17
    sta     >geo_mul_p
    lda     19
    sta     >geo_mul_p+2
    lda     21
    sta     >geo_mul_p+4
    lda     23
    sta     >geo_mul_p+6


    tsc
    clc
    adc     #24
    tcs
    pld
    plx
    pla
    plp
    rtl
}

/*
 * ----------------------------------------------------------------------------
 *  GeoV3_Mul32x32SetInputs
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Préparer les deux opérandes natifs et exécuter le multiplicateur
 *    assembleur.
 *
 *  Paramètres :
 *    [entrée] a/b, entiers signés 32 bits.
 *
 *  Retour :
 *    Aucun ; produit brut dans geo_mul_p. memcpy conserve les bits et
 *    l'ordre little-endian ORCA/C.
 *
 *  Fonctionnement :
 *    Paquets partagés non réentrants.
 *
 * ----------------------------------------------------------------------------
 */
static void GeoV3_Mul32x32SetInputs(Fixed32 a, Fixed32 b)
{
    /* Copier les deux opérandes contigus puis appeler le noyau sans
     * paramètre de pile.
     */
    memcpy(geo_mul_inputs,&a,sizeof(a));
    memcpy(geo_mul_inputs+2,&b,sizeof(b));
    GeoV3_Mul32x32();
}

/*
 * ----------------------------------------------------------------------------
 *  fixed_mul32_fast
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Multiplier deux valeurs 16.16 et conserver les bits 16..47 du produit.
 *
 *  Paramètres :
 *    [entrée] a/b en Fixed32.
 *
 *  Retour :
 *    Produit Fixed32, sans saturation ni arrondi.
 *
 *  Fonctionnement :
 *    Chemin C avec GEOV3_DISABLE_ASM_MUL ; sinon utilise le paquet
 *    assembleur partagé. L'extraction du mot central reproduit la réduction
 *    à 32 bits après décalage.
 *
 * ----------------------------------------------------------------------------
 */
static Fixed32 fixed_mul32_fast(Fixed32 a,Fixed32 b)
{
/* Choisir le produit C de référence ou extraire les deux mots centraux du
 * produit assembleur.
 */
#if defined(GEOV3_DISABLE_ASM_MUL)
    return (Fixed32)(((Fixed64)a*(Fixed64)b)>>FIXED_SHIFT);
#else
    Fixed32 result;
    GeoV3_Mul32x32SetInputs(a,b);
    memcpy(&result,geo_mul_p+1,sizeof(result));
    return result;
#endif
}

/*
 * ----------------------------------------------------------------------------
 *  fixed_product32_fast
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Calculer le produit signé brut 32x32 vers 64 bits.
 *
 *  Paramètres :
 *    [entrée] a/b.
 *
 *  Retour :
 *    Produit Fixed64, sans décalage de virgule fixe.
 *
 *  Fonctionnement :
 *    Utilise le C si GEOV3_DISABLE_ASM_MUL est défini, sinon le paquet
 *    assembleur non réentrant.
 *
 * ----------------------------------------------------------------------------
 */
static Fixed64 fixed_product32_fast(Fixed32 a,Fixed32 b)
{
/* Conserver l'ensemble des 64 bits pour les accumulations de Newell. */
#if defined(GEOV3_DISABLE_ASM_MUL)
    return (Fixed64)a*(Fixed64)b;
#else
    Fixed64 result;
    GeoV3_Mul32x32SetInputs(a,b);
    memcpy(&result,geo_mul_p,sizeof(result));
    return result;
#endif
}

/*
 * ----------------------------------------------------------------------------
 *  fixed_product64x32_fast
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Calculer un produit brut coefficient 64 bits par coordonnée 32 bits.
 *
 *  Paramètres :
 *    [entrée] a/b.
 *
 *  Retour :
 *    Produit Fixed64, sans saturation ; les bornes restent à garantir par
 *    l'appelant.
 *
 *  Fonctionnement :
 *    Si a est exactement représentable en 32 bits, utilise le multiplicateur
 *    rapide ; sinon conserve la multiplication C large.
 *
 * ----------------------------------------------------------------------------
 */
static Fixed64 fixed_product64x32_fast(Fixed64 a,Fixed32 b)
{
    /* Tester l'extension de signe exacte avant de choisir le chemin court. */
    Fixed32 a32=(Fixed32)a;
    if(a==(Fixed64)a32)return fixed_product32_fast(a32,b);
    return a*(Fixed64)b;
}

/*
 * ----------------------------------------------------------------------------
 *  GeoV3_Mul32x32SelfTest
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Comparer le noyau assembleur au produit C sur des cas déterministes.
 *
 *  Paramètres :
 *    [entrée] aucune.
 *
 *  Retour :
 *    0 si tout concorde, 1 au premier écart.
 *
 *  Sorties et effets de bord :
 *    test_index ; en cas d'écart, bad_a et bad_b, asm_product_out et
 *    c_product_out.
 *
 *    écrase les paquets du multiplicateur ; ne teste pas la voie C à sa
 *    place.
 *
 *  Fonctionnement :
 *    Teste 100 couples de bornes puis 2048 couples pseudo-aléatoires.
 *
 * ----------------------------------------------------------------------------
 */
int GeoV3_Mul32x32SelfTest(unsigned long *test_index,
                           Fixed32 *bad_a, Fixed32 *bad_b,
                           Fixed64 *asm_product_out,
                           Fixed64 *c_product_out)
{
    static const Fixed32 boundary[] = {
        0L, 1L, -1L, 2L, -2L,
        65535L, 65536L, -65536L,
        2147483647L, (-2147483647L - 1L)
    };
    unsigned long state;
    unsigned long index;
    unsigned long bits;
    unsigned int i, j;
    Fixed32 a, b;
    Fixed64 asm_product, c_product;

    /* Comparer toutes les combinaisons des valeurs limites. */
    index = 0;
    for (i = 0; i < (unsigned int)(sizeof(boundary) / sizeof(boundary[0])); ++i) {
        for (j = 0; j < (unsigned int)(sizeof(boundary) / sizeof(boundary[0])); ++j) {
            a = boundary[i];
            b = boundary[j];
            GeoV3_Mul32x32SetInputs(a, b);
            memcpy(&asm_product, geo_mul_p, sizeof(asm_product));
            c_product = (Fixed64)a * (Fixed64)b;
            if (asm_product != c_product) {
                *test_index = index;
                *bad_a = a;
                *bad_b = b;
                *asm_product_out = asm_product;
                *c_product_out = c_product;
                return 1;
            }
            ++index;
        }
    }

    /* Générer une suite reproductible de motifs de bits et arrêter au
     * premier produit différent.
     */
    state = 0x13579BDFUL;
    for (i = 0; i < 2048; ++i) {
        state = state * 1664525UL + 1013904223UL;
        bits = state;
        memcpy(&a, &bits, sizeof(a));
        state = state * 1664525UL + 1013904223UL;
        bits = state;
        memcpy(&b, &bits, sizeof(b));

        GeoV3_Mul32x32SetInputs(a, b);
        memcpy(&asm_product, geo_mul_p, sizeof(asm_product));
        c_product = (Fixed64)a * (Fixed64)b;
        if (asm_product != c_product) {
            *test_index = index;
            *bad_a = a;
            *bad_b = b;
            *asm_product_out = asm_product;
            *c_product_out = c_product;
            return 1;
        }
        ++index;
    }

    /* Publier le nombre total de cas seulement après réussite de toutes les
     * comparaisons.
     */
    *test_index = index;
    return 0;
}

/*
 * ----------------------------------------------------------------------------
 *  GeoV3_MulFixed64Fixed32Shift
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Évaluer une contribution de plan : (coefficient * coordinate) >> 16.
 *
 *  Paramètres :
 *    [entrée] coefficient 64 bits et coordinate 16.16.
 *
 *  Retour :
 *    Terme Fixed64.
 *
 *  Fonctionnement :
 *    Choisit le noyau 32x32 uniquement si le coefficient y tient exactement
 *    ; les coefficients plus larges utilisent C. Chaque terme est décalé
 *    séparément.
 *
 * ----------------------------------------------------------------------------
 */
static Fixed64 GeoV3_MulFixed64Fixed32Shift(Fixed64 coefficient,
                                            Fixed32 coordinate)
{
/* Respecter le chemin C de comparaison lorsque l'assembleur est désactivé. */
#if defined(GEOV3_DISABLE_ASM_MUL)

    return (coefficient * (Fixed64)coordinate) >> FIXED_SHIFT;
#else
    Fixed32 coefficient32;
    Fixed64 product;

    /* Vérifier l'extension exacte avant de reprendre le produit assembleur. */
    coefficient32 = (Fixed32)coefficient;
    if (coefficient == (Fixed64)coefficient32) {
        GeoV3_Mul32x32SetInputs(coefficient32, coordinate);
        memcpy(&product, geo_mul_p, sizeof(product));
        return product >> FIXED_SHIFT;
    }


    return (coefficient * (Fixed64)coordinate) >> FIXED_SHIFT;
#endif
}

/* Le noyau de classement réutilise le multiplicateur ci-dessus ;
 * GEOV3_DISABLE_ASM_RELATION force le chemin général.
 */
#include "geo_relation_asm.c"

/*
 * ----------------------------------------------------------------------------
 *  geometric_face_relation
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Classer f1 par rapport à f2 en utilisant le plan de f1 dans la vue
 *    observateur.
 *
 *  Paramètres :
 *    [entrée] model et f1/f2 à base zéro ; model=NULL demande une remise à
 *    zéro.
 *
 *  Retour :
 *    -1 si f1 est avant f2, +1 après, 0 si le plan ne tranche pas.
 *
 *  Sorties et effets de bord :
 *    caches de plans, de sommets et de relations ; paquets assembleur
 *    partagés.
 *
 *  Fonctionnement :
 *    Compare les sommets de f2 au côté de l'observateur en ignorant la bande
 *    epsilon, proportionnelle à current_observer_distance. Des côtés mixtes
 *    rendent 0.
 *
 *    Le chemin assembleur nécessite a/b/c 32 bits et les bornes préparées de
 *    d. Les caches denses et de hash vérifient propriétaire, génération et
 *    époque. La convention -1 avant est distincte de PairCache et du signe
 *    du ray casting.
 *
 * ----------------------------------------------------------------------------
 */
static int geometric_face_relation(Model3D* model, int f1, int f2)
{
    static int cache_f1[256];
    static int cache_f2[256];
    static signed char cache_result[256];
    static unsigned char cache_valid[256];
    static Fixed64 cached_epsilon = 0;
    static Fixed64 cached_distance = 0;
    static int epsilon_valid = 0;
    static Model3D *cache_model=NULL;
    static unsigned int cache_generation=0,cache_epoch=0;

    OrderFixedPlane *prepared;
    FaceArrays3D* faces;
    VertexArrays3D* vtx;
    Fixed64 epsilon;
    Fixed64 a1, b1, c1, d1;
    Fixed64 acc;
    unsigned slot;
    int obs;
    int n2, offset2;
    int all_same, all_opp;
    int k, v, side;
    int result;
    unsigned char *dense_entry;
    unsigned int dense_shift,dense_code;

    /* Traiter la demande explicite d'invalidation sans accéder au modèle. */
    if (model == NULL) {
        ordering_fixed_plane_reset();
        memset(cache_valid, 0, sizeof(cache_valid));
        return 0;
    }


    /* Éliminer les résultats d'un autre propriétaire, d'une autre vue ou
     * d'une autre époque.
     */
    if(order_cache_owner!=model)ordering_cache_invalidate(model);
    if(cache_model!=model || cache_generation!=order_cache_generation || cache_epoch!=order_fixed_plane_epoch){
        memset(cache_valid,0,sizeof(cache_valid));
        cache_model=model;cache_generation=order_cache_generation;cache_epoch=order_fixed_plane_epoch;
    }
    /* Recalculer epsilon uniquement si la distance change, et invalider les
     * relations locales.
     */
    if (!epsilon_valid ||
        current_observer_distance != cached_distance) {
        cached_epsilon = FIXED_MUL_64(
            current_observer_distance,
            FLOAT_TO_FIXED(0.001f));
        cached_distance = current_observer_distance;
        epsilon_valid = 1;
        memset(cache_valid, 0, sizeof(cache_valid));
    }

    /* Essayer le cache dense puis le hash à clé exacte. */
    dense_entry=ordering_geometry_slot(model,f1,f2,cached_epsilon);
    dense_shift=((unsigned int)f2&3U)<<1;
    if(dense_entry){
        dense_code=(*dense_entry>>dense_shift)&3U;
        if(dense_code)return (int)dense_code-2;
    }
    slot = (((unsigned)f1 * 33u) ^ (unsigned)f2) & 255u;
    if (cache_valid[slot] &&
        cache_f1[slot] == f1 &&
        cache_f2[slot] == f2) {
        return (int)cache_result[slot];
    }

    faces = &model->faces;
    vtx = &model->vertices;
    epsilon = cached_epsilon;

    /* Préparer les coefficients et le côté observateur ; un observateur dans
     * la tolérance ne tranche pas.
     */
    prepared=ordering_fixed_plane(model,f1,epsilon);
    result=0;
    obs=prepared->observer;
    if (!obs) goto store_result;


/* N'utiliser le noyau assembleur que si les bornes arithmétiques et le
 * nombre de sommets l'autorisent.
 */
#if !defined(GEOV3_DISABLE_ASM_MUL) && !defined(GEOV3_DISABLE_ASM_RELATION)
    if (prepared->asm_ready &&
        faces->vertex_count[f2] >= 0 &&
        faces->vertex_count[f2] <= MAX_FACE_VERTICES) {
        result = GeoRel_ClassifyPrepared(model,f2,prepared);
        goto store_result;
    }
#endif

    a1=prepared->a;b1=prepared->b;c1=prepared->c;d1=prepared->d;
    n2 = faces->vertex_count[f2];
    offset2 = faces->vertex_indices_ptr[f2];
    all_same = 1;
    all_opp = 1;

    /* Chemin général : évaluer les trois produits décalés puis éliminer les
     * hypothèses de côté.
     */
    for (k = 0; k < n2; ++k) {
        v = faces->vertex_indices_buffer[offset2 + k] - 1;

        acc = GeoV3_MulFixed64Fixed32Shift(a1, vtx->xo[v])
            + GeoV3_MulFixed64Fixed32Shift(b1, vtx->yo[v])
            + GeoV3_MulFixed64Fixed32Shift(c1, vtx->zo[v])
            + d1;

        if (acc > epsilon)
            side = 1;
        else if (acc < -epsilon)
            side = -1;
        else
            continue;

        if (side == obs)
            all_opp = 0;
        else
            all_same = 0;

        if (!all_same && !all_opp)
            goto store_result;
    }

    if (all_same)
        result = -1;
    else if (all_opp)
        result = 1;

/* Publier le résultat dans les caches de la paire orientée. */
store_result:
    cache_f1[slot] = f1;
    cache_f2[slot] = f2;
    cache_result[slot] = (signed char)result;
    cache_valid[slot] = 1;
    if(dense_entry)*dense_entry|=(unsigned char)((unsigned int)(result+2)<<dense_shift);
    return result;
}


/*
 * ----------------------------------------------------------------------------
 *  painter_newell_sancha_fast
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Construire l'ordre FAST par profondeur moyenne décroissante.
 *
 *  Paramètres :
 *    [entrée] model et face_count.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    sorted_face_indices
 *
 *  Fonctionnement :
 *    Avec culling, regroupe les faces affichables en tête et les autres en
 *    fin, puis trie uniquement le préfixe visible. Sans culling, trie toutes
 *    les faces. Ne teste ni plans, ni chevauchement, ni cycles ; le nom
 *    historique ne désigne pas ici l'algorithme Newell/Sancha complet.
 *
 * ----------------------------------------------------------------------------
 */
void painter_newell_sancha_fast(Model3D* model, int face_count) {
    FaceArrays3D* faces = &model->faces;
    if (!faces->z_mean) return;


    int i;
    int visible_count = face_count;
    /* Reconstruire la permutation avec les faces retenues en tête et les
     * faces masquées en queue.
     */
    if (cull_back_faces) {
        visible_count = 0;

        for (i = 0; i < face_count; ++i) {
            if (faces->display_flag[i]) faces->sorted_face_indices[visible_count++] = i;
        }

        int tail = visible_count;
        for (i = 0; i < face_count; ++i) {
            if (!faces->display_flag[i]) faces->sorted_face_indices[tail++] = i;
        }
    } else {
        for (i = 0; i < face_count; ++i) faces->sorted_face_indices[i] = i;
    }

    /* Installer le contexte du comparateur, trier le préfixe puis retirer le
     * pointeur partagé.
     */
    qsort_faces_ptr_for_cmp = faces;
    qsort(faces->sorted_face_indices, visible_count, sizeof(int), cmp_faces_by_zmean);
    qsort_faces_ptr_for_cmp = NULL;
}

/*
 * ----------------------------------------------------------------------------
 *  painter_bubble_sort
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Corriger l'ordre FAST par échanges de voisins avec tests géométriques.
 *
 *  Paramètres :
 *    [entrée] model et face_count.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    permutation et paires inconclusives
 *
 *  Fonctionnement :
 *    Restreint le parcours au préfixe visible lorsque le culling est actif.
 *    Après profondeur et boîtes, teste le chevauchement et les plans. Une
 *    table de paires empêche de retraiter les relations déjà rencontrées ;
 *    les ambiguïtés sont conservées pour inspection. En cas d'échec de
 *    réserve, revient à FAST. Ne découpe aucune face et ne garantit pas de
 *    résoudre tous les cycles.
 *
 * ----------------------------------------------------------------------------
 */
void painter_bubble_sort(Model3D* model, int face_count)
{

    if (face_count <= 1) {
        /* Initialiser l'ordre par profondeur moyenne et déterminer le préfixe traité. */
        painter_newell_sancha_fast(model, face_count);
        return;
    }
    FaceArrays3D* faces = &model->faces;

    (void)geometric_face_relation(NULL, 0, 0);


    painter_newell_sancha_fast(model, face_count);

    int visible_count = face_count;
    if (cull_back_faces) {
        visible_count = 0;
        for (int i = 0; i < face_count; ++i) {
            if (faces->display_flag[i]) {
                ++visible_count;
            }
        }
    }

    if (visible_count < 2) return;

    int* sorted = faces->sorted_face_indices;


    typedef struct {
        int face1;
        int face2;
        int next;
    } OrderedPair;


    const int HASH_SIZE = 2048;
    const int HASH_MASK = HASH_SIZE - 1;

    /* Réserver les paires de garde ; le hash emploie des chaînes et des clés
     * exactes.
     */
    int* hash_table = (int*)malloc(HASH_SIZE * sizeof(int));
    if (!hash_table) {

        painter_newell_sancha_fast(model, face_count);
        return;
    }

    for (int i = 0; i < HASH_SIZE; ++i) {
        hash_table[i] = -1;
    }


    int ordered_cap = face_count * 2;
    if (ordered_cap > 4096) ordered_cap = 4096;
    OrderedPair* ordered = (OrderedPair*)malloc(ordered_cap * sizeof(OrderedPair));
    int ordered_count = 0;
    if (!ordered) {
        free(hash_table);
        painter_newell_sancha_fast(model, face_count);
        return;
    }


    /* Réinitialiser le buffer de diagnostic des paires non décidées. */
    if (inconclusive_pairs) {
        free(inconclusive_pairs);
        inconclusive_pairs = NULL;
    }
    inconclusive_pairs_capacity = ordered_cap;
    inconclusive_pairs = (InconclusivePair*)malloc(ordered_cap * sizeof(InconclusivePair));
    if (!inconclusive_pairs) {
        inconclusive_pairs_capacity = 0;
    }
    inconclusive_pairs_count = 0;


    Fixed32* z_min = faces->z_min;
    Fixed32* z_max = faces->z_max;
    int* minx = faces->minx;
    int* maxx = faces->maxx;
    int* miny = faces->miny;
    int* maxy = faces->maxy;

    #define PAIR_HASH(a, b)  ((((unsigned)(a) * 73856093u) ^ ((unsigned)(b) * 19349663u)) & HASH_MASK)


    int swapped;
    /* Balayer les voisins jusqu'à une passe sans échange. */
    do {
        swapped = 0;
        for (int i = 0; i < visible_count - 1; ++i) {
            int f1 = sorted[i];
            int f2 = sorted[i + 1];


            unsigned h = PAIR_HASH(f1, f2);
            int idx = hash_table[h];
            int already_known = 0;
            while (idx >= 0) {
                if ((ordered[idx].face1 == f1 && ordered[idx].face2 == f2) ||
                    (ordered[idx].face1 == f2 && ordered[idx].face2 == f1)) {
                    already_known = 1;
                    break;
                }
                idx = ordered[idx].next;
            }
            if (already_known) {
                continue;
            }


            /* Enchaîner les rejets de profondeur et de boîtes avant les
             * tests de contour et de plans.
             */
            if (z_max[f2] <= z_min[f1]) continue;


            if (z_max[f1] <= z_min[f2]) goto do_swap;


            if (maxx[f1] <= minx[f2] || maxx[f2] <= minx[f1]) continue;
            if (maxy[f1] <= miny[f2] || maxy[f2] <= miny[f1]) continue;


            if (!projected_polygons_overlap(model, f1, f2)) {
                continue;
            }


            int geo = geometric_face_relation(model, f1, f2);
            if (geo == -1) continue;
            if (geo == 1)  goto do_swap;


            int geo2 = geometric_face_relation(model, f2, f1);
            if (geo2 == -1) goto do_swap;


            if (inconclusive_pairs_count < inconclusive_pairs_capacity) {
                inconclusive_pairs[inconclusive_pairs_count].face1 = f1;
                inconclusive_pairs[inconclusive_pairs_count].face2 = f2;
                ++inconclusive_pairs_count;
            }

            if (ordered_count < ordered_cap) {
                ordered[ordered_count].face1 = f1;
                ordered[ordered_count].face2 = f2;
                ordered[ordered_count].next  = hash_table[h];
                hash_table[h] = ordered_count;
                ++ordered_count;
            }
            continue;
        /* Échanger les deux voisins et mémoriser la paire dans le sens obtenu. */
        do_swap:

            sorted[i]     = f2;
            sorted[i + 1] = f1;
            swapped = 1;

            if (ordered_count < ordered_cap) {
                unsigned h2 = PAIR_HASH(f2, f1);
                ordered[ordered_count].face1 = f2;
                ordered[ordered_count].face2 = f1;
                ordered[ordered_count].next  = hash_table[h2];
                hash_table[h2] = ordered_count;
                ++ordered_count;
            }
        }
    } while (swapped);


    /* Libérer les gardes temporaires ; les paires de diagnostic restent
     * accessibles.
     */
    free(ordered);
    free(hash_table);
}


/*
 * ----------------------------------------------------------------------------
 *  painter_newell_sancha
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Corriger l'ordre FAST à partir des paires dont les intervalles Z se
 *    recouvrent.
 *
 *  Paramètres :
 *    [entrée] model et face_count
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    permutation
 *
 *  Fonctionnement :
 *    Un balayage des faces classées par zmin construit les candidats actifs.
 *    Pour chaque paire, les boîtes, les deux relations de plans, le
 *    chevauchement projeté puis le ray casting décident si une insertion
 *    doit avancer une face. Une paire candidate ne provoque qu'un
 *    déplacement via s_moved ; les tables de relations évitent les
 *    répétitions. Buffers statiques conservés entre appels. Les paires
 *    insolubles peuvent être ajoutées au buffer de diagnostic si disponible.
 *    Sur échec d'allocation, reprend FAST ; aucun découpage automatique.
 *
 * ----------------------------------------------------------------------------
 */
void painter_newell_sancha(Model3D* model, int face_count)
{
    FaceArrays3D* faces;
    int* sorted;
    int visible_count;
    typedef struct {
        int face1;
        int face2;
        int next;
    } OrderedPair;
    typedef struct {
        int a;
        int b;
    } CandidatePair;
#define HASH_SIZE_NS       2048
#define HASH_MASK_NS       (HASH_SIZE_NS - 1)
#define ORDERED_MAX_NS     4096

#define PAIR_HASH_NS(a,b) \
    ((((unsigned)(a) * 73856093u) ^ \
      ((unsigned)(b) * 19349663u)) & HASH_MASK_NS)


    static int* s_hash = NULL;
    static OrderedPair* s_ordered = NULL;
    static int* s_touched = NULL;
    static int* s_pos = NULL;
    static int s_pos_capacity = 0;
    static int* s_by_zmin = NULL;
    static int s_by_zmin_capacity = 0;
    static int* s_active = NULL;
    static int s_active_capacity = 0;
    static CandidatePair* s_candidates = NULL;
    static int s_candidates_capacity = 0;


    static unsigned char* s_moved = NULL;
    static int s_moved_capacity = 0;
    int ordered_count = 0;
    int touched_count = 0;
    int* pos;
    int* by_zmin;
    int* active;
    int active_count;
    int candidate_count;
    int changed;
    Fixed32* z_min;
    Fixed32* z_max;
    int* minx;
    int* maxx;
    int* miny;
    int* maxy;

    /* Réinitialiser les relations de plans et établir le préfixe visible de FAST. */
    (void)geometric_face_relation(NULL, 0, 0);


    if (face_count <= 1) {
        painter_newell_sancha_fast(model, face_count);
        return;
    }
    faces = &model->faces;
    painter_newell_sancha_fast(model, face_count);


    visible_count = face_count;
    if (cull_back_faces) {
        int i;
        visible_count = 0;
        for (i = 0; i < face_count; ++i) {
            if (faces->display_flag[i])
                ++visible_count;
        }
    }
    if (visible_count < 2)
        return;
    sorted = faces->sorted_face_indices;


    /* Réserver les tables persistantes de hash et de relations. */
    if (!s_hash) {
        int i;
        s_hash = (int*)malloc(HASH_SIZE_NS * sizeof(int));
        if (!s_hash) {
            painter_newell_sancha_fast(model, face_count);
            return;
        }
        for (i = 0; i < HASH_SIZE_NS; ++i)
            s_hash[i] = -1;
    }
    if (!s_ordered) {
        s_ordered =
            (OrderedPair*)malloc(
                ORDERED_MAX_NS * sizeof(OrderedPair));
        if (!s_ordered) {
            painter_newell_sancha_fast(model, face_count);
            return;
        }
    }
    if (!s_touched) {
        s_touched =
            (int*)malloc(
                ORDERED_MAX_NS * sizeof(int));
        if (!s_touched) {
            painter_newell_sancha_fast(model, face_count);
            return;
        }
    }
    /* Agrandir les buffers de positions et de balayage selon le nombre de faces. */
    if (s_pos_capacity < face_count) {
        int* tmp;
        tmp = (int*)realloc(
            s_pos,
            (size_t)face_count * sizeof(int));
        if (!tmp) {
            painter_newell_sancha_fast(model, face_count);
            return;
        }
        s_pos = tmp;
        s_pos_capacity = face_count;
    }
    if (s_by_zmin_capacity < visible_count) {
        int* tmp;
        tmp = (int*)realloc(
            s_by_zmin,
            (size_t)visible_count * sizeof(int));
        if (!tmp) {
            painter_newell_sancha_fast(model, face_count);
            return;
        }
        s_by_zmin = tmp;
        s_by_zmin_capacity = visible_count;
    }
    if (s_active_capacity < visible_count) {
        int* tmp;
        tmp = (int*)realloc(
            s_active,
            (size_t)visible_count * sizeof(int));
        if (!tmp) {
            painter_newell_sancha_fast(model, face_count);
            return;
        }
        s_active = tmp;
        s_active_capacity = visible_count;
    }
    /* Associer les buffers et tableaux de métriques à la vue courante. */
    pos = s_pos;
    by_zmin = s_by_zmin;
    active = s_active;
    z_min = faces->z_min;
    z_max = faces->z_max;
    minx = faces->minx;
    maxx = faces->maxx;
    miny = faces->miny;
    maxy = faces->maxy;


    {
        int i;
        for (i = 0; i < visible_count; ++i)
            pos[sorted[i]] = i;
    }


    {
        int i;
        int gap;
        for (i = 0; i < visible_count; ++i)
            by_zmin[i] = sorted[i];
        /* Classer les faces visibles par zmin avec un tri de Shell pour
         * préparer le balayage.
         */
        for (gap = visible_count / 2;
             gap > 0;
             gap /= 2) {
            for (i = gap; i < visible_count; ++i) {
                int f;
                Fixed32 z;
                int j;
                f = by_zmin[i];
                z = z_min[f];
                j = i;
                while (j >= gap &&
                       z_min[by_zmin[j - gap]] > z) {
                    by_zmin[j] =
                        by_zmin[j - gap];
                    j -= gap;
                }
                by_zmin[j] = f;
            }
        }
    }


    /* Construire les paires candidates en conservant les intervalles Z
     * encore actifs.
     */
    active_count = 0;
    candidate_count = 0;
    {
        int n;
        for (n = 0; n < visible_count; ++n) {
            int f;
            Fixed32 fmin;
            int dst;
            int a;
            f = by_zmin[n];
            fmin = z_min[f];


            dst = 0;
            for (a = 0; a < active_count; ++a) {
                int q = active[a];
                if (z_max[q] > fmin)
                    active[dst++] = q;
            }
            active_count = dst;


            for (a = 0; a < active_count; ++a) {
                /* Agrandir les listes de candidats et leurs drapeaux de
                 * déplacement ; replier sur FAST en cas d'échec.
                 */
                if (candidate_count >=
                    s_candidates_capacity) {
                    int newcap;
                    CandidatePair* tmp;
                    unsigned char* tmp_moved;
                    if (s_candidates_capacity == 0)
                        newcap = 64;
                    else
                        newcap =
                            s_candidates_capacity * 2;
                    tmp = (CandidatePair*)realloc(
                        s_candidates,
                        (size_t)newcap *
                        sizeof(CandidatePair));
                    if (!tmp) {
                        painter_newell_sancha_fast(
                            model, face_count);
                        return;
                    }
                    s_candidates = tmp;
                    s_candidates_capacity = newcap;


                    if (s_moved_capacity < newcap) {
                        tmp_moved =
                            (unsigned char*)realloc(
                                s_moved,
                                (size_t)newcap *
                                sizeof(unsigned char));
                        if (!tmp_moved) {
                            painter_newell_sancha_fast(
                                model, face_count);
                            return;
                        }
                        s_moved = tmp_moved;
                        s_moved_capacity = newcap;
                    }
                }
                s_candidates[candidate_count].a =
                    active[a];
                s_candidates[candidate_count].b =
                    f;


                s_moved[candidate_count] = 0;
                ++candidate_count;
            }
            active[active_count++] = f;
        }
    }


    /* Reconsidérer les candidats après les insertions en utilisant la table
     * inverse des positions.
     */
    do {
        int c;
        changed = 0;
        for (c = 0; c < candidate_count; ++c) {
            int A;
            int B;
            int i;
            int j;
            int P;
            int Q;
            unsigned h;
            int idx;
            int known;
            A = s_candidates[c].a;
            B = s_candidates[c].b;


            if (pos[A] < pos[B]) {
                i = pos[A];
                j = pos[B];
                P = A;
                Q = B;
            }
            else {
                i = pos[B];
                j = pos[A];
                P = B;
                Q = A;
            }


            h = PAIR_HASH_NS(P, Q);
            idx = s_hash[h];
            known = 0;
            while (idx >= 0) {
                if (s_ordered[idx].face1 == P &&
                    s_ordered[idx].face2 == Q) {
                    known = 1;
                    break;
                }
                idx = s_ordered[idx].next;
            }
            if (known)
                continue;


            /* Essayer les rejets projetés puis les relations de plans avant
             * le test de chevauchement.
             */
            if (maxx[P] <= minx[Q] ||
                maxx[Q] <= minx[P]) {
                goto order_valid;
            }


            if (maxy[P] <= miny[Q] ||
                maxy[Q] <= miny[P]) {
                goto order_valid;
            }


            {
                int geo_pq;
                geo_pq =
                    geometric_face_relation(
                        model, P, Q);
                if (geo_pq == -1)
                    goto order_valid;
            }


            {
                int geo_qp;
                geo_qp =
                    geometric_face_relation(
                        model, Q, P);
                if (geo_qp == 1)
                    goto order_valid;
            }


            if (!projected_polygons_overlap(
                    model, P, Q)) {
                goto order_valid;
            }


            {
                int rc;
                rc = ray_cast_hierarchical(
                    model, P, Q);
                if (rc > 0) {
                    goto order_valid;
                }
                if (rc < 0) {


                    /* Refuser une deuxième insertion causée par la même
                     * paire pour limiter les cycles.
                     */
                    if (s_moved[c]) {
                        goto order_unresolved;
                    }


                    s_moved[c] = 1;
                    goto do_move;
                }


                goto order_unresolved;
            }


/* Avancer Q à la position de P par insertion et mettre à jour toutes les
 * positions déplacées.
 */
do_move:
            {
                int tmp;
                int k;
                tmp = sorted[j];
                for (k = j; k > i; --k) {
                    sorted[k] = sorted[k - 1];
                    pos[sorted[k]] = k;
                }
                sorted[i] = tmp;
                pos[tmp] = i;
            }


            changed = 1;
            continue;


/* Mémoriser une relation résolue dans la table à chaînes. */
order_valid:
            if (ordered_count < ORDERED_MAX_NS) {
                unsigned hh;
                hh = PAIR_HASH_NS(P, Q);
                s_ordered[ordered_count].face1 = P;
                s_ordered[ordered_count].face2 = Q;
                s_ordered[ordered_count].next =
                    s_hash[hh];


                if (s_hash[hh] == -1 &&
                    touched_count < ORDERED_MAX_NS) {
                    s_touched[touched_count++] =
                        (int)hh;
                }
                s_hash[hh] = ordered_count;
                ++ordered_count;
            }
            continue;


/* Consigner l'ambiguïté lorsque le buffer de diagnostic le permet, puis
 * éviter son retraitement.
 */
order_unresolved:
            if (inconclusive_pairs != NULL &&
                inconclusive_pairs_count <
                inconclusive_pairs_capacity) {
                inconclusive_pairs[
                    inconclusive_pairs_count].face1 = P;
                inconclusive_pairs[
                    inconclusive_pairs_count].face2 = Q;
                ++inconclusive_pairs_count;
            }


            if (ordered_count < ORDERED_MAX_NS) {
                unsigned hh;
                hh = PAIR_HASH_NS(P, Q);
                s_ordered[ordered_count].face1 = P;
                s_ordered[ordered_count].face2 = Q;
                s_ordered[ordered_count].next =
                    s_hash[hh];
                if (s_hash[hh] == -1 &&
                    touched_count < ORDERED_MAX_NS) {
                    s_touched[touched_count++] =
                        (int)hh;
                }
                s_hash[hh] = ordered_count;
                ++ordered_count;
            }
        }
    } while (changed);


    {
        int i;
        /* Effacer seulement les buckets utilisés pour préparer l'appel suivant. */
        for (i = 0; i < touched_count; ++i)
            s_hash[s_touched[i]] = -1;
    }
#undef PAIR_HASH_NS
#undef HASH_SIZE_NS
#undef HASH_MASK_NS
#undef ORDERED_MAX_NS
}

/*
 * ----------------------------------------------------------------------------
 *  painter_newell_sancha_selection
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Variante de sélection Newell/Sancha conservée hors du dispatch
 *    principal.
 *
 *  Paramètres :
 *    [entrée] model et face_count.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    permutation
 *
 *  Fonctionnement :
 *    Prépare FAST, trie le préfixe visible par zmax, puis insère avant P un
 *    candidat dont les plans ou le rayon imposent l'antériorité. Reprend la
 *    passe avec ce Q. Un budget de reprises borne les cycles ; aucun
 *    découpage et aucun nouveau mode clavier ne sont associés à cette
 *    fonction dans processModelFast.
 *
 * ----------------------------------------------------------------------------
 */
void painter_newell_sancha_selection(Model3D* model, int face_count)
{
    FaceArrays3D* faces;
    int* sorted;
    int visible_count;
    int fs;
    int i;
    int p;
    int f2;
    int q;
    int geo_pq;
    int geo_qp;
    int rc;
    int tmp;
    int k;
    int restart_count;
    int restart_limit;

    if (!model || face_count <= 0)
        return;


    /* Invalider les relations et préparer le préfixe visible avant le tri zmax. */
    (void)geometric_face_relation(NULL, 0, 0);
    faces = &model->faces;
    sorted = faces->sorted_face_indices;


    painter_newell_sancha_fast(model, face_count);
    visible_count = face_count;
    if (cull_back_faces) {
        visible_count = 0;
        for (i = 0; i < face_count; ++i) {
            if (faces->display_flag[i])
                ++visible_count;
        }
    }
    if (visible_count < 2)
        return;


    sort_by_zmax(sorted, faces->z_max, visible_count);

    /* Fixer chaque tête de sélection et limiter les reprises à visible_count. */
    for (fs = 0; fs < visible_count - 1; ++fs) {
        p = sorted[fs];
        f2 = fs + 1;
        restart_count = 0;
        restart_limit = visible_count;

        /* Appliquer profondeur, boîtes, plans puis chevauchement et rayon
         * aux candidats.
         */
        while (f2 < visible_count) {
            q = sorted[f2];


            if (faces->z_max[q] <= faces->z_min[p]) {
                ++f2;
                continue;
            }


            if (faces->maxx[p] <= faces->minx[q] ||
                faces->maxx[q] <= faces->minx[p] ||
                faces->maxy[p] <= faces->miny[q] ||
                faces->maxy[q] <= faces->miny[p]) {
                ++f2;
                continue;
            }


            geo_pq = geometric_face_relation(model, p, q);
            geo_qp = geometric_face_relation(model, q, p);

            if (geo_pq == -1 || geo_qp == 1) {
                ++f2;
                continue;
            }


            if (!projected_polygons_overlap(model, p, q)) {
                ++f2;
                continue;
            }

            if (geo_pq == 1 || geo_qp == -1) {
                rc = -1;
            } else {

                rc = ray_cast_hierarchical(model, p, q);
            }

            if (rc >= 0) {
                ++f2;
                continue;
            }


            /* Abandonner l'insertion si le budget de reprises est épuisé. */
            if (restart_count >= restart_limit) {
                ++f2;
                continue;
            }
            ++restart_count;


            /* Insérer Q avant P et recommencer les comparaisons de cette passe. */
            tmp = sorted[f2];
            for (k = f2; k > fs; --k)
                sorted[k] = sorted[k - 1];
            sorted[fs] = tmp;
            p = tmp;
            f2 = fs + 1;
        }
    }
}


/*
 * ----------------------------------------------------------------------------
 *  pair_cache_create
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Allouer un cache de relations de paires avec sondage borné.
 *
 *  Paramètres :
 *    [entrée] capacity strictement positive.
 *
 *  Retour :
 *    Cache possédé par l'appelant, ou NULL sur échec.
 *
 *  Fonctionnement :
 *    Les slots commencent vides et doivent être libérés par
 *    pair_cache_destroy.
 *
 * ----------------------------------------------------------------------------
 */
static PairCache* pair_cache_create(int capacity) {
    /* Allouer la structure puis ses slots remis à zéro ; libérer la
     * structure si la seconde allocation échoue.
     */
    PairCache* c = (PairCache*)malloc(sizeof(PairCache));
    if (!c) return NULL;
    c->capacity = capacity;
    c->slots = (PairCacheEntry*)calloc(capacity, sizeof(PairCacheEntry));
    if (!c->slots) { free(c); return NULL; }
    return c;
}

/*
 * ----------------------------------------------------------------------------
 *  pair_cache_slot
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Calculer le slot initial d'une paire non orientée.
 *
 *  Paramètres :
 *    [entrée] c, cache de capacité positive ; f1/f2.
 *
 *  Retour :
 *    Indice du slot.
 *
 *  Fonctionnement :
 *    Trie les deux identifiants avant hash ; aucun test de présence ni effet
 *    de bord.
 *
 * ----------------------------------------------------------------------------
 */
static int pair_cache_slot(PairCache* c, int f1, int f2) {

    /* Normaliser les identifiants et réduire le hash modulo la capacité. */
    int a = (f1 < f2) ? f1 : f2;
    int b = (f1 < f2) ? f2 : f1;
    unsigned int h = (unsigned int)(a * 2654435761u ^ b * 2246822519u);
    return (int)(h % (unsigned int)c->capacity);
}

/*
 * ----------------------------------------------------------------------------
 *  pair_cache_find
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Lire une relation en sondant au plus huit slots.
 *
 *  Paramètres :
 *    [entrée] c, éventuellement NULL, et f1/f2.
 *
 *  Retour :
 *    +1 f1 avant f2, -1 sens inverse, 0 absent ou inconclusif.
 *
 *  Fonctionnement :
 *    Une paire stockée en sens inverse rend le signe opposé. Un slot vide
 *    termine la recherche ; aucun effet de bord.
 *
 * ----------------------------------------------------------------------------
 */
static int pair_cache_find(PairCache* c, int f1, int f2) {
    int slot, i;
    if (!c) return 0;
    slot = pair_cache_slot(c, f1, f2);
    /* Sonder huit cases au maximum, comparer les clés exactes et réorienter
     * le résultat.
     */
    for (i = 0; i < 8; i++) {
        int pos = (slot + i) % c->capacity;
        PairCacheEntry* e = &c->slots[pos];
        if (!e->occupied) return 0;
        if ((e->face1 == f1 && e->face2 == f2)) return  e->relation;
        if ((e->face1 == f2 && e->face2 == f1)) return -e->relation;
    }
    return 0;
}

/*
 * ----------------------------------------------------------------------------
 *  pair_cache_insert
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Mémoriser une relation en cherchant au plus huit slots.
 *
 *  Paramètres :
 *    [entrée] c, éventuellement NULL ; f1/f2 ; relation, +1 f1 avant, -1
 *    sens inverse.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    slot libre ou correspondant remplacé
 *
 *  Fonctionnement :
 *    Si les huit slots sont occupés par d'autres clés, la relation est
 *    simplement abandonnée.
 *
 * ----------------------------------------------------------------------------
 */
static void pair_cache_insert(PairCache* c, int f1, int f2, int8_t relation) {
    int slot, i;
    if (!c) return;
    slot = pair_cache_slot(c, f1, f2);
    /* Réutiliser le premier slot vide ou la paire déjà présente, dans l'un
     * ou l'autre sens.
     */
    for (i = 0; i < 8; i++) {
        int pos = (slot + i) % c->capacity;
        PairCacheEntry* e = &c->slots[pos];
        if (!e->occupied ||
            (e->face1 == f1 && e->face2 == f2) ||
            (e->face1 == f2 && e->face2 == f1)) {
            e->face1    = f1;
            e->face2    = f2;
            e->relation = relation;
            e->occupied = 1;
            return;
        }
    }
}

/*
 * ----------------------------------------------------------------------------
 *  pair_cache_destroy
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Libérer un cache de paires et ses slots.
 *
 *  Paramètres :
 *    [entrée] c, NULL accepté.
 *
 *  Retour :
 *    Aucun ; le pointeur de l'appelant devient invalide.
 *
 * ----------------------------------------------------------------------------
 */
static void pair_cache_destroy(PairCache* c) {
    /* Libérer les slots avant la structure propriétaire. */
    if (c) { free(c->slots); free(c); }
}
/*
 * ----------------------------------------------------------------------------
 *  painter_geoV2
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Corriger l'ordre FAST par passes de voisins et cache de paires borné.
 *
 *  Paramètres :
 *    [entrée] model et face_count.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    permutation et diagnostics d'ambiguïté
 *
 *  Fonctionnement :
 *    Teste les plans en premier, puis pour les cas ambigus les intervalles
 *    Z, le chevauchement et le ray casting hiérarchique. Réduit la limite au
 *    dernier échange et borne les passes à deux fois le nombre visible.
 *    Essaie des caches de 4096, 2048 puis 1024 entrées ; sans cache, garde
 *    FAST. La convention +1 avant de PairCache doit être convertie depuis
 *    les prédicats.
 *
 * ----------------------------------------------------------------------------
 */
void painter_geoV2(Model3D* model, int face_count) {
    FaceArrays3D* faces = &model->faces;
    int i;
    Fixed32* face_zmean = faces->z_mean;
    if (!face_zmean) return;
    Fixed32* face_zmin = faces->z_min;
    Fixed32* face_zmax = faces->z_max;

    /* Réinitialiser les relations et construire l'ordre initial FAST. */
    (void)geometric_face_relation(NULL, 0, 0);


    painter_newell_sancha_fast(model, face_count);
    int visible_count = face_count;
    if (cull_back_faces) {
        visible_count = 0;
        for (i = 0; i < face_count; ++i) {
            if (faces->display_flag[i]) ++visible_count;
        }
    }


    int swap_count = 0;
    int swapped;


    /* Chercher une réserve de cache adaptée ; aucun passage géométrique si
     * toutes échouent.
     */
    PairCache* cache = pair_cache_create(4096);
    if (!cache) {

        cache = pair_cache_create(2048);
        if (!cache)
            cache = pair_cache_create(1024);
    }
    if (!cache) {
        printf("painter_geoV2: warning - no pair cache, geometric pass disabled\n");
        return;
    }

    /* Recréer le diagnostic avec une capacité de quatre paires par face. */
    if (inconclusive_pairs) {
        free(inconclusive_pairs);
        inconclusive_pairs = NULL;
    }
    inconclusive_pairs_capacity = face_count * 4;
    inconclusive_pairs_count = 0;
    if (inconclusive_pairs_capacity > 0) {
        inconclusive_pairs = (InconclusivePair*)malloc(
            inconclusive_pairs_capacity * sizeof(InconclusivePair));
        if (!inconclusive_pairs)
            inconclusive_pairs_capacity = 0;
    }
    int limit = visible_count - 1;
    int new_limit;
    const int MAX_PASSES = visible_count * 2;
    int pass = 0;
    /* Parcourir les voisins non mémorisés jusqu'à stabilisation ou limite de
     * sécurité.
     */
    do {
        swapped = 0;
        new_limit = 0;
        ++pass;
        for (i = 0; i < limit; ++i) {
            int f1 = faces->sorted_face_indices[i];
            int f2 = faces->sorted_face_indices[i + 1];
            if (pair_cache_find(cache, f1, f2) != 0)
                continue;

            /* Essayer les plans dans les deux sens avant les replis projetés. */
            int geo = geometric_face_relation(model, f1, f2);
            if (geo == -1) {
                pair_cache_insert(cache, f1, f2, 1);
                continue;
            }
            if (geo == 1)
                goto do_swap;
            int geo2 = geometric_face_relation(model, f2, f1);
            if (geo2 == -1)
                goto do_swap;
            if (geo2 == 1) {
                pair_cache_insert(cache, f1, f2, 1);
                continue;
            }

            /* Éviter le rayon pour les intervalles Z disjoints ; tester
             * sinon le chevauchement.
             */
            int z_disjoint = 0;
            if (face_zmin && face_zmax) {
                if (face_zmax[f1] <= face_zmin[f2] ||
                    face_zmax[f2] <= face_zmin[f1])
                    z_disjoint = 1;
            } else {
                Fixed32 dz = face_zmean[f1] - face_zmean[f2];
                if (dz > FLOAT_TO_FIXED(1.0f) || dz < FLOAT_TO_FIXED(-1.0f))
                    z_disjoint = 1;
            }
            if (!z_disjoint && projected_polygons_overlap(model, f1, f2)) {
                int rc = ray_cast_hierarchical(model, f1, f2);
                if (rc < 0)
                    goto do_swap;
                if (rc > 0) {
                    pair_cache_insert(cache, f1, f2, 1);
                    continue;
                }
            }

            if (inconclusive_pairs &&
                inconclusive_pairs_count < inconclusive_pairs_capacity) {
                inconclusive_pairs[inconclusive_pairs_count].face1 = f1;
                inconclusive_pairs[inconclusive_pairs_count].face2 = f2;
                inconclusive_pairs_count++;
            }
            pair_cache_insert(cache, f1, f2, 1);
            continue;
        /* Échanger les voisins, mémoriser le sens obtenu et réduire la
         * prochaine passe.
         */
        do_swap:
            {
                int tmp = faces->sorted_face_indices[i];
                faces->sorted_face_indices[i]     = faces->sorted_face_indices[i + 1];
                faces->sorted_face_indices[i + 1] = tmp;
                swapped = 1;
                swap_count++;
                pair_cache_insert(cache, f2, f1, 1);
                new_limit = i;
            }
        }
        limit = new_limit;
    } while (swapped && limit > 0 && pass < MAX_PASSES);
    if (pass >= MAX_PASSES) {
        printf("painter_geoV2: safety limit reached (%d passes)\n", pass);
    }
    /* Libérer le cache propre à cet appel. */
    pair_cache_destroy(cache);

}
/*
 * ----------------------------------------------------------------------------
 *  painter_correct
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Corriger localement une permutation initiale FAST séparée selon le
 *    signe de d.
 *
 *  Paramètres :
 *    [entrée] model et face_count ; debug, paramètre conservé mais non
 *    exploité.
 *
 *  Retour :
 *    Compteur moves, pas une preuve de correction.
 *
 *  Sorties et effets de bord :
 *    sorted_face_indices
 *
 *  Fonctionnement :
 *    Désactive temporairement le culling, place les faces d<=0 avant les
 *    faces d>0, puis effectue des insertions fondées sur les boîtes, le
 *    chevauchement simple et les prédicats de plans. min_allowed_pos limite
 *    certains déplacements. La passe de faces arrière est sautée
 *    inconditionnellement par goto after_pass1. Aucun appel à
 *    split_face_by_plane : ce mode ne découpe pas les faces.
 *
 *  Limites :
 *    plusieurs retours d'échec d'allocation surviennent avant restauration
 *    de cull_back_faces ; ce comportement est conservé, pas corrigé ici.
 *
 * ----------------------------------------------------------------------------
 */
static int painter_correct(Model3D* model, int face_count, int debug) {
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    if (face_count <= 0) return 0;
    /* Sauvegarder l'option puis produire FAST sans culling pour inclure
     * toutes les faces.
     */
    int old_cull = cull_back_faces;
    cull_back_faces = 0;
    painter_newell_sancha_fast(model, face_count);
    int moves = 0;
    int n = face_count;


    int *temp_indices = (int*)malloc(n * sizeof(int)); if (!temp_indices) { printf("Error: painter_correct malloc temp_indices failed\n"); return 0; }
    /* Partitionner de façon stable les deux groupes de signes de d. */
    int back_idx = 0, front_idx = 0;

    for (int i = 0; i < n; ++i) {
        int f = faces->sorted_face_indices[i];
        if (faces->plane_d[f] <= 0) {
            temp_indices[back_idx++] = f;
        }
    }
    int front_start_pos = back_idx;
    for (int i = 0; i < n; ++i) {
        int f = faces->sorted_face_indices[i];
        if (faces->plane_d[f] > 0) {
            temp_indices[front_start_pos + front_idx++] = f;
        }
    }

    for (int i = 0; i < n; ++i) faces->sorted_face_indices[i] = temp_indices[i];
    free(temp_indices);

    /* Préparer les positions inverses et les limites de déplacement par face. */
    int *pos_of_face = (int*)malloc(n * sizeof(int)); if (!pos_of_face) { printf("Error: painter_correct malloc pos_of_face failed\n"); return 0; }
    for (int i = 0; i < n; ++i) pos_of_face[faces->sorted_face_indices[i]] = i;

    int *min_allowed_pos = (int*)malloc(n * sizeof(int)); if (!min_allowed_pos) { free(pos_of_face); printf("Error: painter_correct malloc min_allowed_pos failed\n"); return 0; }
    for (int i = 0; i < n; ++i) min_allowed_pos[i] = -1;


    /* La passe arrière ci-dessous est présente mais inaccessible sur ce chemin. */
    goto after_pass1;

    printf("Back faces:");
    if (old_cull == 0) {
        for (int target = 0; target < face_count; ++target) {
            if (faces->plane_d[target] > 0) continue;
        int pos = pos_of_face[target];
        if (pos < 0 || pos >= front_start_pos) continue;

        int best_before = -1;
        for (int i = 0; i < pos; ++i) {
            int f = faces->sorted_face_indices[i];
            if (i >= front_start_pos) break;

            if (faces->z_max[f] <= faces->z_min[target]) continue;
            if (faces->z_max[target] <= faces->z_min[f]) continue;


            if (faces->maxx[f] <= faces->minx[target]) continue;
            if (faces->maxx[target] <= faces->minx[f]) continue;
            if (faces->maxy[f] <= faces->miny[target]) continue;
            if (faces->maxy[target] <= faces->miny[f]) continue;


            if (!projected_polygons_overlap_simple(model, f, target)) continue;


            if (pair_plane_after(model, f, target)) {
                if (best_before == -1 || i < best_before) best_before = i;
            }
        }
        if (best_before != -1) {

            moves += move_element_remove_and_insert_pos(faces->sorted_face_indices, face_count, pos, best_before, pos_of_face);

            pos = pos_of_face[target];

            if (min_allowed_pos[target] == -1 || pos < min_allowed_pos[target]) min_allowed_pos[target] = pos;
        }

        pos = pos_of_face[target];
        if (pos < 0 || pos >= front_start_pos) continue;
        int best_after = -1;
        for (int i = pos + 1; i < front_start_pos; ++i) {
            int f = faces->sorted_face_indices[i];

            if (faces->z_max[f] <= faces->z_min[target]) continue;
            if (faces->z_max[target] <= faces->z_min[f]) continue;

            if (faces->maxx[f] <= faces->minx[target]) continue;
            if (faces->maxx[target] <= faces->minx[f]) continue;
            if (faces->maxy[f] <= faces->miny[target]) continue;
            if (faces->maxy[target] <= faces->miny[f]) continue;
            if (!projected_polygons_overlap_simple(model, f, target)) continue;
            if (pair_plane_before(model, f, target)) { best_after = i; }
        }
        if (best_after != -1) {
            int insert_idx;
            if (best_after < pos) insert_idx = best_after + 1; else insert_idx = best_after;
            if (min_allowed_pos[target] != -1 && insert_idx > min_allowed_pos[target]) insert_idx = min_allowed_pos[target];

            if (insert_idx >= front_start_pos) insert_idx = front_start_pos - 1;
            moves += move_element_remove_and_insert_pos(faces->sorted_face_indices, face_count, pos, insert_idx, pos_of_face);
            pos = pos_of_face[target];
            if (min_allowed_pos[target] == -1 || pos < min_allowed_pos[target]) min_allowed_pos[target] = pos;
        }

        printf(" %d",target);
        }
    }
    after_pass1: ;

    /* Corriger le groupe avant par insertions après les rejets de profondeur
     * et de projection.
     */
    printf("\nFront faces:");
    for (int target = 0; target < face_count; ++target) {
        if (faces->plane_d[target] <= 0) continue;
        int pos = pos_of_face[target];
        if (pos < 0 || pos < front_start_pos) continue;

        int best_before = -1;
        for (int i = front_start_pos; i < pos; ++i) {
            int f = faces->sorted_face_indices[i];

            if (faces->z_max[f] <= faces->z_min[target]) continue;
            if (faces->z_max[target] <= faces->z_min[f]) continue;

            if (faces->maxx[f] <= faces->minx[target]) continue;
            if (faces->maxx[target] <= faces->minx[f]) continue;
            if (faces->maxy[f] <= faces->miny[target]) continue;
            if (faces->maxy[target] <= faces->miny[f]) continue;
            if (!projected_polygons_overlap_simple(model, f, target)) continue;
            if (pair_plane_after(model, f, target)) {
                if (best_before == -1 || i < best_before) best_before = i;
            }
        }
        if (best_before != -1) {

            if (best_before < front_start_pos) best_before = front_start_pos;
            moves += move_element_remove_and_insert_pos(faces->sorted_face_indices, face_count, pos, best_before, pos_of_face);
            pos = pos_of_face[target];
            if (min_allowed_pos[target] == -1 || pos < min_allowed_pos[target]) min_allowed_pos[target] = pos;
        }

        pos = pos_of_face[target];
        if (pos < 0 || pos >= n) continue;
        int best_after = -1;
        for (int i = pos + 1; i < face_count; ++i) {
            int f = faces->sorted_face_indices[i];


            if (faces->z_max[f] <= faces->z_min[target]) continue;
            if (faces->z_max[target] <= faces->z_min[f]) continue;


            if (faces->maxx[f] <= faces->minx[target]) continue;
            if (faces->maxx[target] <= faces->minx[f]) continue;
            if (faces->maxy[f] <= faces->miny[target]) continue;


            if (!projected_polygons_overlap_simple(model, f, target)) continue;


            if (pair_plane_before(model, f, target)) { best_after = i;                               }

        }
        if (best_after != -1) {

            int insert_idx;
            if (best_after < pos) insert_idx = best_after + 1; else insert_idx = best_after;

            if (min_allowed_pos[target] != -1 && insert_idx > min_allowed_pos[target]) insert_idx = min_allowed_pos[target];

            moves += move_element_remove_and_insert_pos(faces->sorted_face_indices, face_count, pos, insert_idx, pos_of_face);

            pos = pos_of_face[target];
            if (min_allowed_pos[target] == -1 || pos < min_allowed_pos[target]) min_allowed_pos[target] = pos;
        }

        printf(" %d",target);
    }
    /* Restaurer le culling sur le chemin normal puis libérer les tables de
     * travail.
     */
    cull_back_faces = old_cull;
    free(pos_of_face); free(min_allowed_pos);
    return moves;
}
/*
 * ----------------------------------------------------------------------------
 *  painter_correctV2
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Corriger localement une permutation initiale FAST séparée selon le
 *    signe de d.
 *
 *  Paramètres :
 *    [entrée] model et face_count ; debug, paramètre conservé mais non
 *    exploité.
 *
 *  Retour :
 *    Compteur moves, pas une preuve de correction.
 *
 *  Sorties et effets de bord :
 *    sorted_face_indices
 *
 *  Fonctionnement :
 *    Désactive temporairement le culling, place les faces d<=0 avant les
 *    faces d>0, puis effectue des insertions fondées sur les boîtes, le
 *    chevauchement simple et les prédicats de plans. min_allowed_pos limite
 *    certains déplacements. Ajoute, sans culling initial, des corrections
 *    entre faces arrière et avant sur un snapshot ; leur total_moves n'est
 *    pas ajouté au compteur retourné. Aucun appel à split_face_by_plane : ce
 *    mode ne découpe pas les faces.
 *
 *  Limites :
 *    plusieurs retours d'échec d'allocation surviennent avant restauration
 *    de cull_back_faces ; ce comportement est conservé, pas corrigé ici.
 *
 * ----------------------------------------------------------------------------
 */
static int painter_correctV2(Model3D* model, int face_count, int debug) {


    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    if (face_count <= 0) return 0;

    /* Sauvegarder l'option puis produire FAST sans culling pour inclure
     * toutes les faces.
     */
    int old_cull = cull_back_faces;
    cull_back_faces = 0;


    painter_newell_sancha_fast(model, face_count);
    int moves = 0;
    int n = face_count;


    int *temp_indices = (int*)malloc(n * sizeof(int));
    if (!temp_indices) { printf("Error: painter_correctV2 malloc temp_indices failed\n"); return 0; }
    /* Partitionner de façon stable les deux groupes de signes de d. */
    int back_idx = 0, front_idx = 0;
    for (int i = 0; i < n; ++i) {
        int f = faces->sorted_face_indices[i];
        if (faces->plane_d[f] <= 0) {
            temp_indices[back_idx++] = f;
        }
    }
    int front_start_pos = back_idx;
    for (int i = 0; i < n; ++i) {
        int f = faces->sorted_face_indices[i];
        if (faces->plane_d[f] > 0) {
            temp_indices[front_start_pos + front_idx++] = f;
        }
    }
    for (int i = 0; i < n; ++i) faces->sorted_face_indices[i] = temp_indices[i];
    free(temp_indices);
    /* Préparer les positions inverses et les limites de déplacement par face. */
    int *pos_of_face = (int*)malloc(n * sizeof(int)); if (!pos_of_face) { printf("Error: painter_correctV2 malloc pos_of_face failed\n"); return 0; }
    for (int i = 0; i < n; ++i) pos_of_face[faces->sorted_face_indices[i]] = i;
    int *min_allowed_pos = (int*)malloc(n * sizeof(int)); if (!min_allowed_pos) { free(pos_of_face); printf("Error: painter_correctV2 malloc min_allowed_pos failed\n"); return 0; }
    for (int i = 0; i < n; ++i) min_allowed_pos[i] = -1;
    /* Traiter aussi les faces arrière uniquement si le culling était
     * initialement désactivé.
     */
    printf("Back faces:");


    /* Limiter la correction des faces arrière au cas où elles étaient
     * initialement autorisées.
     */
    if (old_cull == 0) {
        for (int target = 0; target < face_count; ++target) {
            printf(" %d",target);
            if (faces->plane_d[target] > 0) continue;
            int pos = pos_of_face[target];
            if (pos < 0 || pos >= front_start_pos) continue;
            /* Chercher la première face imposant une insertion de la cible
             * vers l'arrière de la liste de dessin.
             */
            int best_before = -1;


            for (int i = 0; i < pos; ++i) {
                int f = faces->sorted_face_indices[i];
                if (i >= front_start_pos) break;
                if (faces->z_max[f] <= faces->z_min[target]) continue;
                if (faces->z_max[target] <= faces->z_min[f]) continue;
                if (faces->maxx[f] <= faces->minx[target]) continue;
                if (faces->maxx[target] <= faces->minx[f]) continue;
                if (faces->maxy[f] <= faces->miny[target]) continue;
                if (faces->maxy[target] <= faces->miny[f]) continue;

                if (!projected_polygons_overlap_simple(model, f, target)) continue;
                if (pair_plane_after(model, f, target)) {
                    if (best_before == -1 || i < best_before) best_before = i;
                }
            }
            if (best_before != -1) {
                moves += move_element_remove_and_insert_pos(faces->sorted_face_indices, face_count, pos, best_before, pos_of_face);
                pos = pos_of_face[target];
                if (min_allowed_pos[target] == -1 || pos < min_allowed_pos[target]) min_allowed_pos[target] = pos;
            }
            pos = pos_of_face[target];
            if (pos < 0 || pos >= front_start_pos) continue;
            /* Chercher ensuite la dernière contrainte d'insertion inverse,
             * sous la limite déjà mémorisée.
             */
            int best_after = -1;

            for (int i = pos + 1; i < front_start_pos; ++i) {
                int f = faces->sorted_face_indices[i];
                if (faces->z_max[f] <= faces->z_min[target]) continue;
                if (faces->z_max[target] <= faces->z_min[f]) continue;
                if (faces->maxx[f] <= faces->minx[target]) continue;
                if (faces->maxx[target] <= faces->minx[f]) continue;
                if (faces->maxy[f] <= faces->miny[target]) continue;
                if (faces->maxy[target] <= faces->miny[f]) continue;
                if (!projected_polygons_overlap_simple(model, f, target)) continue;
                if (pair_plane_before(model, f, target)) { best_after = i; }
            }
            if (best_after != -1) {
                int insert_idx;
                if (best_after < pos) insert_idx = best_after + 1; else insert_idx = best_after;
                if (min_allowed_pos[target] != -1 && insert_idx > min_allowed_pos[target]) insert_idx = min_allowed_pos[target];
                if (insert_idx >= front_start_pos) insert_idx = front_start_pos - 1;
                moves += move_element_remove_and_insert_pos(faces->sorted_face_indices, face_count, pos, insert_idx, pos_of_face);
                pos = pos_of_face[target];
                if (min_allowed_pos[target] == -1 || pos < min_allowed_pos[target]) min_allowed_pos[target] = pos;
            }
        }
    }


    /* Corriger le groupe avant par insertions après les rejets de profondeur
     * et de projection.
     */
    printf("\n\nFront faces:");
    for (int target = 0; target < face_count; ++target) {
        printf(" %d",target);
        if (faces->plane_d[target] <= 0) continue;
        int pos = pos_of_face[target];
        if (pos < 0 || pos < front_start_pos) continue;
        int best_before = -1;
        for (int i = front_start_pos; i < pos; ++i) {
            int f = faces->sorted_face_indices[i];
            if (faces->z_max[f] <= faces->z_min[target]) continue;
            if (faces->z_max[target] <= faces->z_min[f]) continue;
            if (faces->maxx[f] <= faces->minx[target]) continue;
            if (faces->maxx[target] <= faces->minx[f]) continue;
            if (faces->maxy[f] <= faces->miny[target]) continue;
            if (faces->maxy[target] <= faces->miny[f]) continue;
            if (!projected_polygons_overlap_simple(model, f, target)) continue;
            if (pair_plane_after(model, f, target)) {
                if (best_before == -1 || i < best_before) best_before = i;
            }
        }
        if (best_before != -1) {
            if (best_before < front_start_pos) best_before = front_start_pos;
            moves += move_element_remove_and_insert_pos(faces->sorted_face_indices, face_count, pos, best_before, pos_of_face);
            pos = pos_of_face[target];
            if (min_allowed_pos[target] == -1 || pos < min_allowed_pos[target]) min_allowed_pos[target] = pos;
        }
        pos = pos_of_face[target];
        if (pos < 0 || pos < front_start_pos) continue;
        int best_after = -1;
        for (int i = pos + 1; i < face_count; ++i) {
            int f = faces->sorted_face_indices[i];
            if (faces->z_max[f] <= faces->z_min[target]) continue;
            if (faces->z_max[target] <= faces->z_min[f]) continue;
            if (faces->maxx[f] <= faces->minx[target]) continue;
            if (faces->maxx[target] <= faces->minx[f]) continue;
            if (faces->maxy[f] <= faces->miny[target]) continue;
            if (faces->maxy[target] <= faces->miny[f]) continue;
            if (!projected_polygons_overlap_simple(model, f, target)) continue;
            if (pair_plane_before(model, f, target)) { best_after = i; }
        }
        if (best_after != -1) {
            int insert_idx;
            if (best_after < pos) insert_idx = best_after + 1; else insert_idx = best_after;
            if (min_allowed_pos[target] != -1 && insert_idx > min_allowed_pos[target]) insert_idx = min_allowed_pos[target];
            moves += move_element_remove_and_insert_pos(faces->sorted_face_indices, face_count, pos, insert_idx, pos_of_face);
            pos = pos_of_face[target];
            if (min_allowed_pos[target] == -1 || pos < min_allowed_pos[target]) min_allowed_pos[target] = pos;
        }
    }


    if (old_cull == 0) {
        /* Conserver la permutation corrigée pendant la vérification croisée
         * supplémentaire.
         */
        int *snapshot = (int*)malloc(n * sizeof(int));
        if (!snapshot) { free(pos_of_face); free(min_allowed_pos); printf("Error: painter_correctV2 malloc snapshot failed\n"); return 0; }
        for (int i = 0; i < n; ++i) snapshot[i] = faces->sorted_face_indices[i];


        painter_newell_sancha_fast(model, face_count);
        printf("\n\nCross-section back-face checks:");

        FILE *logf = NULL;
        int total_scanned = 0, total_logged = 0, total_ov = 0, total_tch = 0, total_none = 0, total_moves = 0;

        /* Examiner les faces arrière de la liste FAST et rapprocher leurs
         * positions des contraintes avec les faces avant.
         */
        for (int idx = n - 1; idx >= 0; --idx) {
            int bf = faces->sorted_face_indices[idx];
            total_scanned++;

            if (faces->plane_d[bf] > 0) {
                break;
            }
            total_logged++;
            int orig_pos = pos_of_face[bf];


            int found = 0;
            int per_ov = 0, per_tch = 0;
            for (int j = 0; j < n; ++j) {
                int ff = faces->sorted_face_indices[j];

                if (faces->plane_d[ff] <= 0) continue;

                if (faces->z_max[ff] <= faces->z_min[bf]) continue;
                if (faces->z_max[bf] <= faces->z_min[ff]) continue;

                int bbox_strict = !(faces->maxx[ff] <= faces->minx[bf] || faces->maxx[bf] <= faces->minx[ff]
                                    || faces->maxy[ff] <= faces->miny[bf] || faces->maxy[bf] <= faces->miny[ff]);
                int bbox_touch = !(faces->maxx[ff] < faces->minx[bf] || faces->maxx[bf] < faces->minx[ff]
                                   || faces->maxy[ff] < faces->miny[bf] || faces->maxy[bf] < faces->miny[ff]);
                if (!bbox_touch) continue;
                int overlapping = 0;
                if (bbox_strict) {
                    if (projected_polygons_overlap_simple(model, ff, bf)) overlapping = 1;
                }
                if (overlapping || bbox_touch) {
                    int orig_pos_ff = pos_of_face[ff];

                    found++; if (overlapping) { per_ov++; total_ov++; } else { per_tch++; total_tch++; }


                    int plane_after = pair_plane_after(model, bf, ff);
                    int plane_before = pair_plane_before(model, bf, ff);
                    int decision = 0;
                    const char *reason = NULL;
                    if (plane_after == 1 && plane_before == 0) { decision = 1; reason = "plane_after"; }
                    else if (plane_before == 1 && plane_after == 0) { decision = -1; reason = "plane_before"; }
                    else {

                        float zbf = FIXED_TO_FLOAT(faces->z_mean[bf]);
                        float zff = FIXED_TO_FLOAT(faces->z_mean[ff]);
                        if (zbf < zff) { decision = 1; reason = "zmean"; } else { decision = -1; reason = "zmean"; }
                    }


                    /* Modifier le snapshot seulement lorsque la face arrière
                     * doit être placée après la face avant ; tenir à jour
                     * les positions inverses.
                     */
                    if (decision == 1 && pos_of_face[bf] < pos_of_face[ff]) {
                        int from = pos_of_face[bf];
                        int to = pos_of_face[ff] + 1;
                        if (from < to) to--;
                        if (from != to) {
                            int tmp = snapshot[from];
                            if (from < to) {
                                for (int s = from; s < to; ++s) { snapshot[s] = snapshot[s+1]; pos_of_face[snapshot[s]] = s; }
                                snapshot[to] = tmp; pos_of_face[tmp] = to;
                            } else {
                                for (int s = from; s > to; --s) { snapshot[s] = snapshot[s-1]; pos_of_face[snapshot[s]] = s; }
                                snapshot[to] = tmp; pos_of_face[tmp] = to;
                            }


                            total_moves++;
                        } else {

                        }


                    }
                }
            }
            if (!found) {                                        total_none++; }

        }


        for (int i = 0; i < n; ++i) faces->sorted_face_indices[i] = snapshot[i];
        free(snapshot);
    }

    /* Restaurer le culling sur le chemin normal puis libérer les tables de
     * travail.
     */
    cull_back_faces = old_cull;
    free(pos_of_face);
    free(min_allowed_pos);


    return moves;
}

/*
 * ----------------------------------------------------------------------------
 *  move_element_remove_and_insert
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Déplacer un élément d'un tableau en conservant l'ordre des autres.
 *
 *  Paramètres :
 *    [entrée-sortie] arr de longueur n
 *
 *    [entrée] from et insert_idx, position finale.
 *
 *  Retour :
 *    1 si déplacement, 0 si source invalide ou position inchangée.
 *
 *  Fonctionnement :
 *    La destination est bornée à [0,n-1] ; memmove traite les zones qui se
 *    chevauchent.
 *
 * ----------------------------------------------------------------------------
 */
static int move_element_remove_and_insert(int *arr, int n, int from, int insert_idx) {
    /* Valider la source, borner la destination et éviter les déplacements nuls. */
    if (from < 0 || from >= n) return 0;
    if (insert_idx < 0) insert_idx = 0;
    if (insert_idx > n-1) insert_idx = n-1;
    if (from == insert_idx) return 0;
    int val = arr[from];
    /* Décaler le segment dans le sens approprié puis réinsérer la valeur
     * sauvegardée.
     */
    if (insert_idx < from) {

        memmove(&arr[insert_idx+1], &arr[insert_idx], (from - insert_idx) * sizeof(int));
        arr[insert_idx] = val;
    } else {

        memmove(&arr[from], &arr[from+1], (insert_idx - from) * sizeof(int));
        arr[insert_idx] = val;
    }
    return 1;
}
/*
 * ----------------------------------------------------------------------------
 *  move_element_remove_and_insert_pos
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Déplacer un identifiant et maintenir la table inverse des positions.
 *
 *  Paramètres :
 *    [entrée-sortie] arr de longueur n et pos_of_face, facultatif
 *
 *    [entrée] from et insert_idx, position finale bornée.
 *
 *  Retour :
 *    1 si déplacement, 0 sinon.
 *
 *  Fonctionnement :
 *    Les valeurs d'arr doivent indexer pos_of_face lorsqu'il est fourni.
 *
 * ----------------------------------------------------------------------------
 */
static int move_element_remove_and_insert_pos(int *arr, int n, int from, int insert_idx, int *pos_of_face) {
    /* Valider et borner les positions avant de sauvegarder la valeur déplacée. */
    if (from < 0 || from >= n) return 0;
    if (insert_idx < 0) insert_idx = 0;
    if (insert_idx > n-1) insert_idx = n-1;
    if (from == insert_idx) return 0;
    int val = arr[from];
    /* Mettre à jour la position inverse de chaque élément déplacé, puis
     * celle de la valeur réinsérée.
     */
    if (insert_idx < from) {

        for (int i = from; i > insert_idx; --i) {
            arr[i] = arr[i-1];
            if (pos_of_face) pos_of_face[arr[i]] = i;
        }
        arr[insert_idx] = val;
        if (pos_of_face) pos_of_face[val] = insert_idx;
    } else {

        for (int i = from; i < insert_idx; ++i) {
            arr[i] = arr[i+1];
            if (pos_of_face) pos_of_face[arr[i]] = i;
        }
        arr[insert_idx] = val;
        if (pos_of_face) pos_of_face[val] = insert_idx;
    }
    return 1;
}

/* Les relations de plans et les rayons ont des conventions de signe
 * distinctes, documentées à chaque interface.
 */
segment "ordering";


segment "overlap";

/*
 * ----------------------------------------------------------------------------
 *  pair_plane_before
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Tester si les deux plans suffisent à établir f1 avant f2.
 *
 *  Paramètres :
 *    [entrée] model et f1/f2 à base zéro, métriques observateur calculées.
 *
 *  Retour :
 *    1 si un des tests suffit, 0 si aucun ne conclut ; aucun tri effectué.
 *
 *  Fonctionnement :
 *    Tolérance fixe 0,01 en 16.16, distincte d'epsilon proportionnel à la
 *    distance dans geometric_face_relation. Les points dans la bande sont
 *    ignorés.
 *
 * ----------------------------------------------------------------------------
 */
static int pair_plane_before(Model3D* model, int f1, int f2) {
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;

    Fixed32 epsilon = FLOAT_TO_FIXED(0.01f);
    int k;
    int n1 = faces->vertex_count[f1];
    int n2 = faces->vertex_count[f2];
    int offset1 = faces->vertex_indices_ptr[f1];
    int offset2 = faces->vertex_indices_ptr[f2];
    /* Évaluer d'abord les sommets de f2 vis-à-vis du plan de f1 et du côté
     * observateur.
     */
    Fixed64 a1 = faces->plane_a[f1]; Fixed64 b1 = faces->plane_b[f1]; Fixed64 c1 = faces->plane_c[f1]; Fixed64 d1 = faces->plane_d[f1];


    int obs_side1 = 0; int side; int all_same_side;
    obs_side1 = 0;
    if (d1 > (Fixed64)epsilon) obs_side1 = 1; else if (d1 < -(Fixed64)epsilon) obs_side1 = -1; else obs_side1 = 0;
    if (obs_side1 != 0) {
        all_same_side = 1;
        for (k = 0; k < n2; ++k) {
            int v = faces->vertex_indices_buffer[offset2+k]-1;
            Fixed64 acc = 0;
            acc  = (((Fixed64)a1 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)b1 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)c1 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
            acc += (Fixed64)d1;
            if (acc > (Fixed64)epsilon) side = 1; else if (acc < -(Fixed64)epsilon) side = -1; else continue;
            if (obs_side1 != side) { all_same_side = 0; break; }
        }
        if (all_same_side) return 1;
    }


    /* Si nécessaire, évaluer les sommets de f1 vis-à-vis du plan de f2. */
    Fixed64 a2 = faces->plane_a[f2]; Fixed64 b2 = faces->plane_b[f2]; Fixed64 c2 = faces->plane_c[f2]; Fixed64 d2 = faces->plane_d[f2];
    int obs_side2 = 0; int all_opposite_side = 0;
    obs_side2 = 0;
    if (d2 > (Fixed64)epsilon) obs_side2 = 1; else if (d2 < -(Fixed64)epsilon) obs_side2 = -1; else obs_side2 = 0;
    if (obs_side2 != 0) {
        all_opposite_side = 1;
        for (k = 0; k < n1; ++k) {
            int v = faces->vertex_indices_buffer[offset1+k]-1;
            Fixed64 acc = 0;
            acc  = (((Fixed64)a2 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)b2 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)c2 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
            acc += (Fixed64)d2;
            if (acc > (Fixed64)epsilon) side = 1; else if (acc < -(Fixed64)epsilon) side = -1; else continue;
            if (obs_side2 == side) { all_opposite_side = 0; break; }
        }
        if (all_opposite_side) return 1;
    }
    return 0;
}

/*
 * ----------------------------------------------------------------------------
 *  pair_plane_after
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Tester si les deux plans suffisent à établir f1 après f2.
 *
 *  Paramètres :
 *    [entrée] model et f1/f2 à base zéro, métriques observateur calculées.
 *
 *  Retour :
 *    1 si un des tests suffit, 0 si aucun ne conclut ; aucun tri effectué.
 *
 *  Fonctionnement :
 *    Tolérance fixe 0,01 en 16.16, distincte d'epsilon proportionnel à la
 *    distance dans geometric_face_relation. Les points dans la bande sont
 *    ignorés.
 *
 * ----------------------------------------------------------------------------
 */
static int pair_plane_after(Model3D* model, int f1, int f2) {
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;

    Fixed32 epsilon = FLOAT_TO_FIXED(0.01f);
    int k;
    int n1 = faces->vertex_count[f1];
    int n2 = faces->vertex_count[f2];
    int offset1 = faces->vertex_indices_ptr[f1];
    int offset2 = faces->vertex_indices_ptr[f2];
    /* Évaluer d'abord les sommets de f2 vis-à-vis du plan de f1 et du côté
     * observateur.
     */
    Fixed64 a1 = faces->plane_a[f1]; Fixed64 b1 = faces->plane_b[f1]; Fixed64 c1 = faces->plane_c[f1]; Fixed64 d1 = faces->plane_d[f1];


    int obs_side1 = 0; int side; int all_opposite_side;
    obs_side1 = 0; if (d1 > (Fixed64)epsilon) obs_side1 = 1; else if (d1 < -(Fixed64)epsilon) obs_side1 = -1; else obs_side1 = 0;
    if (obs_side1 != 0) {
        all_opposite_side = 1;
        for (k = 0; k < n2; ++k) {
            int v = faces->vertex_indices_buffer[offset2+k]-1;
            Fixed64 acc = 0;
            acc  = (((Fixed64)a1 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)b1 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)c1 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
            acc += (Fixed64)d1;
            if (acc > (Fixed64)epsilon) side = 1; else if (acc < -(Fixed64)epsilon) side = -1; else continue;
            if (obs_side1 == side) { all_opposite_side = 0; break; }
        }
        if (all_opposite_side) return 1;
    }


    /* Si nécessaire, évaluer les sommets de f1 vis-à-vis du plan de f2. */
    Fixed64 a2 = faces->plane_a[f2]; Fixed64 b2 = faces->plane_b[f2]; Fixed64 c2 = faces->plane_c[f2]; Fixed64 d2 = faces->plane_d[f2];
    int obs_side2 = 0; int all_same_side = 0;
    obs_side2 = 0; if (d2 > (Fixed64)epsilon) obs_side2 = 1; else if (d2 < -(Fixed64)epsilon) obs_side2 = -1; else obs_side2 = 0;
    if (obs_side2 != 0) {
        all_same_side = 1;
        for (k = 0; k < n1; ++k) {
            int v = faces->vertex_indices_buffer[offset1+k]-1;
            Fixed64 acc = 0;
            acc  = (((Fixed64)a2 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)b2 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)c2 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
            acc += (Fixed64)d2;
            if (acc > (Fixed64)epsilon) side = 1; else if (acc < -(Fixed64)epsilon) side = -1; else continue;
            if (obs_side2 != side) { all_same_side = 0; break; }
        }
        if (all_same_side) return 1;
    }
    return 0;
}
/*
 * ----------------------------------------------------------------------------
 *  evaluate_pair_tests
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Exposer séparément les sept tests d'ordre d'une paire pour le
 *    diagnostic.
 *
 *  Paramètres :
 *    [entrée] model et f1/f2 à base zéro.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    out[0..6]
 *
 *  Fonctionnement :
 *    Ordre des cases : profondeur, séparation X, séparation Y, puis quatre
 *    tests de côté des plans. -1 valide le sens courant, +1 indique
 *    l'inversion, 0 reste indéterminé. X/Y disjoints écrivent -1, ce qui
 *    n'est pas une preuve d'occlusion. Utilise epsilon fixe 0,01 et ne
 *    modifie pas la permutation.
 *
 * ----------------------------------------------------------------------------
 */
static void evaluate_pair_tests(Model3D* model, int f1, int f2, int out[7]) {
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    Fixed32 epsilon = FLOAT_TO_FIXED(0.01f);
    int k;
    /* Initialiser les résultats puis effectuer les rejets par profondeur et
     * bornes.
     */
    for (k = 0; k < 7; ++k) out[k] = 0;

    if (faces->z_max[f2] <= faces->z_min[f1]) out[0] = -1;
    else if (faces->z_max[f1] <= faces->z_min[f2]) out[0] = 1;

    {
        int minx1 = faces->minx[f1], maxx1 = faces->maxx[f1], minx2 = faces->minx[f2], maxx2 = faces->maxx[f2];
        if (maxx1 <= minx2 || maxx2 <= minx1) out[1] = -1;
    }

    {
        int miny1 = faces->miny[f1], maxy1 = faces->maxy[f1], miny2 = faces->miny[f2], maxy2 = faces->maxy[f2];
        if (maxy1 <= miny2 || maxy2 <= miny1) out[2] = -1;
    }

    /* Préparer les listes et plans pour les quatre hypothèses géométriques. */
    int n1 = faces->vertex_count[f1];
    int n2 = faces->vertex_count[f2];
    int offset1 = faces->vertex_indices_ptr[f1];
    int offset2 = faces->vertex_indices_ptr[f2];
    Fixed64 a1 = faces->plane_a[f1]; Fixed64 b1 = faces->plane_b[f1]; Fixed64 c1 = faces->plane_c[f1]; Fixed64 d1 = faces->plane_d[f1];
    Fixed64 a2 = faces->plane_a[f2]; Fixed64 b2 = faces->plane_b[f2]; Fixed64 c2 = faces->plane_c[f2]; Fixed64 d2 = faces->plane_d[f2];
    int obs_side1 = 0; int obs_side2 = 0; int side; int all_same_side; int all_opposite_side;

    /* Comparer le plan de f1 au côté observateur. */
    obs_side1 = 0; if (d1 > (Fixed64)epsilon) obs_side1 = 1; else if (d1 < -(Fixed64)epsilon) obs_side1 = -1; else obs_side1 = 0;
    if (obs_side1 != 0) {
        all_same_side = 1;
        for (k = 0; k < n2; ++k) {
            int v = faces->vertex_indices_buffer[offset2+k]-1;
            Fixed64 acc = 0;
            acc  = (((Fixed64)a1 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)b1 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)c1 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
            acc += (Fixed64)d1;
            if (acc > (Fixed64)epsilon) side = 1; else if (acc < -(Fixed64)epsilon) side = -1; else continue;
            if (obs_side1 != side) { all_same_side = 0; break; }
        }
        if (all_same_side) out[3] = -1; else out[3] = 0;
    } else out[3] = 0;

    /* Comparer ensuite le plan de f2 avec la convention de résultat du diagnostic. */
    obs_side2 = 0; if (d2 > (Fixed64)epsilon) obs_side2 = 1; else if (d2 < -(Fixed64)epsilon) obs_side2 = -1; else obs_side2 = 0;
    if (obs_side2 != 0) {
        all_opposite_side = 1;
        for (k = 0; k < n1; ++k) {
            int v = faces->vertex_indices_buffer[offset1+k]-1;
            Fixed64 acc = 0;
            acc  = (((Fixed64)a2 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)b2 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)c2 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
            acc += (Fixed64)d2;
            if (acc > (Fixed64)epsilon) side = 1; else if (acc < -(Fixed64)epsilon) side = -1; else continue;
            if (obs_side2 == side) { all_opposite_side = 0; break; }
        }
        if (all_opposite_side) out[4] = -1; else out[4] = 0;
    } else out[4] = 0;

    obs_side1 = 0; if (d1 > (Fixed64)epsilon) obs_side1 = 1; else if (d1 < -(Fixed64)epsilon) obs_side1 = -1; else obs_side1 = 0;
    if (obs_side1 != 0) {
        all_opposite_side = 1;
        for (k = 0; k < n2; ++k) {
            int v = faces->vertex_indices_buffer[offset2+k]-1;
            Fixed64 acc = 0;
            acc  = (((Fixed64)a1 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)b1 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)c1 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
            acc += (Fixed64)d1;
            if (acc > (Fixed64)epsilon) side = 1; else if (acc < -(Fixed64)epsilon) side = -1; else continue;
            if (obs_side1 == side) { all_opposite_side = 0; break; }
        }
        if (all_opposite_side) out[5] = 1; else out[5] = 0;
    } else out[5] = 0;

    obs_side2 = 0; if (d2 > (Fixed64)epsilon) obs_side2 = 1; else if (d2 < -(Fixed64)epsilon) obs_side2 = -1; else obs_side2 = 0;
    if (obs_side2 != 0) {
        all_same_side = 1;
        for (k = 0; k < n1; ++k) {
            int v = faces->vertex_indices_buffer[offset1+k]-1;
            Fixed64 acc = 0;
            acc  = (((Fixed64)a2 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)b2 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)c2 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
            acc += (Fixed64)d2;
            if (acc > (Fixed64)epsilon) side = 1; else if (acc < -(Fixed64)epsilon) side = -1; else continue;
            if (obs_side2 != side) { all_same_side = 0; break; }
        }
        if (all_same_side) out[6] = 1; else out[6] = 0;
    } else out[6] = 0;
}
/*
 * ----------------------------------------------------------------------------
 *  ray_cast_distances
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Calculer les paramètres d'intersection d'un rayon écran avec deux
 *    plans.
 *
 *  Paramètres :
 *    [entrée] model, f1/f2 à base zéro, cx/cy en pixels sans pan
 *
 *  Retour :
 *    1 si les deux dénominateurs sont utilisables, 0 pour arguments
 *    invalides ou quasi-parallélisme (seuil 1e-6).
 *
 *  Sorties et effets de bord :
 *    *out_tf1 et *out_tf2.
 *
 *    peut remplir le cache de conversion des plans.
 *
 *  Fonctionnement :
 *    Le rayon part de l'origine avec direction
 *    ((cx-Cx)/scale,(Cy-cy)/scale,1). Ne teste ni appartenance aux contours
 *    ni positivité des distances ; échelle non nulle requise.
 *
 * ----------------------------------------------------------------------------
 */
static int ray_cast_distances(Model3D* model, int f1, int f2, int cx, int cy, float *out_tf1, float *out_tf2) {
    if (!model || !out_tf1 || !out_tf2) return 0;
    FaceArrays3D* faces = &model->faces;
    if (f1 < 0 || f2 < 0 || f1 >= faces->face_count || f2 >= faces->face_count) return 0;
    double fallback1[4],fallback2[4];
    double *plane1,*plane2;
    /* Construire la direction écran avec la projection globale. */
    float proj_scale = FIXED_TO_FLOAT(s_global_proj_scale_fixed);
    float Dx = ((float)cx - (float)CENTRE_X) / proj_scale;
    float Dy = ((float)CENTRE_Y - (float)cy) / proj_scale;

    /* Récupérer les plans puis vérifier les dénominateurs avant les deux
     * divisions.
     */
    plane1=ordering_plane(model,f1,fallback1);
    float A1 = (float)plane1[0];
    float B1 = (float)plane1[1];
    float C1 = (float)plane1[2];
    float denom1 = A1 * Dx + B1 * Dy + C1;
    if (fabsf(denom1) < 1e-6f) return 0;

    plane2=ordering_plane(model,f2,fallback2);
    float A2 = (float)plane2[0];
    float B2 = (float)plane2[1];
    float C2 = (float)plane2[2];
    float denom2 = A2 * Dx + B2 * Dy + C2;
    if (fabsf(denom2) < 1e-6f) return 0;


    float D1 = (float)plane1[3];
    float D2 = (float)plane2[3];
    /* Publier les deux paramètres du rayon seulement si les calculs sont
     * possibles.
     */
    *out_tf1 = -D1 / denom1;
    *out_tf2 = -D2 / denom2;
    return 1;
}
/*
 * ----------------------------------------------------------------------------
 *  ray_cast_at
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Comparer deux plans au passage d'un rayon de coordonnées écran
 *    entières.
 *
 *  Paramètres :
 *    [entrée] model, f1/f2, cx/cy sans pan.
 *
 *  Retour :
 *    +1 si f1 est plus loin que f2 (donc à dessiner avant), -1 si elle est
 *    plus proche, 0 en cas d'égalité ou d'échec.
 *
 *  Fonctionnement :
 *    Cette convention est opposée à geometric_face_relation pour le sens
 *    avant. Ne vérifie pas si le point appartient réellement aux deux
 *    polygones.
 *
 * ----------------------------------------------------------------------------
 */
static int ray_cast_at(Model3D* model, int f1, int f2, int cx, int cy) {
    float tf1 = 0.0f, tf2 = 0.0f;
    /* Exiger les deux intersections puis comparer leurs paramètres sans
     * tolérance additionnelle.
     */
    if (!ray_cast_distances(model, f1, f2, cx, cy, &tf1, &tf2)) return 0;
    if (tf1 > tf2) return 1;
    else if (tf1 < tf2) return -1;
    else return 0;
}

/*
 * ----------------------------------------------------------------------------
 *  ray_cast_at_fractional
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Comparer deux plans au passage d'un rayon écran subpixel en double.
 *
 *  Paramètres :
 *    [entrée] model, f1/f2 et cx/cy en pixels fractionnaires sans pan.
 *
 *  Retour :
 *    +1 si f1 est plus loin, -1 si elle est plus proche, 0 sinon.
 *
 *  Sorties et effets de bord :
 *    cache partagé de conversion des plans.
 *
 *  Fonctionnement :
 *    Refuse l'échelle non positive et les dénominateurs quasi nuls. Ne teste
 *    pas l'appartenance aux contours.
 *
 * ----------------------------------------------------------------------------
 */
static int ray_cast_at_fractional(Model3D *model, int f1, int f2,
                                  double cx, double cy)
{
    FaceArrays3D *faces;
    double scale, dx, dy, denominator1, denominator2, t1, t2;
    double fallback1[4], fallback2[4];
    double *plane1, *plane2;
    /* Valider modèle, identifiants et projection avant la construction du rayon. */
    if (!model) return 0;
    faces = &model->faces;
    if (f1 < 0 || f2 < 0 || f1 >= faces->face_count || f2 >= faces->face_count) return 0;
    scale = (double)s_global_proj_scale_fixed / (double)FIXED_SCALE;
    if (scale <= 0.0) return 0;
    dx = (cx - (double)CENTRE_X) / scale;
    dy = ((double)CENTRE_Y - cy) / scale;
    /* Évaluer les deux plans en double avec le même rayon. */
    plane1 = ordering_plane(model, f1, fallback1);
    denominator1 = plane1[0] * dx + plane1[1] * dy + plane1[2];
    if (fabs(denominator1) < 1e-6) return 0;
    plane2 = ordering_plane(model, f2, fallback2);
    denominator2 = plane2[0] * dx + plane2[1] * dy + plane2[2];
    if (fabs(denominator2) < 1e-6) return 0;
    /* Comparer les paramètres des intersections calculées. */
    t1 = -plane1[3] / denominator1;
    t2 = -plane2[3] / denominator2;
    if (t1 > t2) return 1;
    if (t1 < t2) return -1;
    return 0;
}


#define RCH_ZOOM_MAX_VERTS (2 * MAX_FACE_VERTICES)


typedef struct {
    Fixed32 saved_scale;
    int idx[RCH_ZOOM_MAX_VERTS];
    int saved_x2d[RCH_ZOOM_MAX_VERTS];
    int saved_y2d[RCH_ZOOM_MAX_VERTS];
    int count;
    int f1_minx, f1_maxx, f1_miny, f1_maxy;
    int f2_minx, f2_maxx, f2_miny, f2_maxy;
} RCHZoomPatch;
/*
 * ----------------------------------------------------------------------------
 *  rch_gather_pair_vertex_indices
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Rassembler les identifiants distincts des sommets de deux faces.
 *
 *  Paramètres :
 *    [entrée] faces, f1/f2.
 *
 *  Retour :
 *    Nombre écrit.
 *
 *  Sorties et effets de bord :
 *    out_idx à base zéro
 *
 *  Préconditions :
 *    buffer de sortie assez grand pour la somme des longueurs. Déduplique
 *    par recherche linéaire en conservant l'ordre de première rencontre.
 *
 * ----------------------------------------------------------------------------
 */
static int rch_gather_pair_vertex_indices(FaceArrays3D* faces, int f1, int f2, int* out_idx) {
    int n = 0;

    int base1 = faces->vertex_indices_ptr[f1], vc1 = faces->vertex_count[f1];
    /* Ajouter d'abord les sommets distincts de f1. */
    for (int k = 0; k < vc1; ++k) {
        int vidx = faces->vertex_indices_buffer[base1 + k] - 1;
        int dup = 0;
        for (int m = 0; m < n; ++m) if (out_idx[m] == vidx) { dup = 1; break; }
        if (!dup) out_idx[n++] = vidx;
    }

    /* Compléter avec ceux de f2 qui ne sont pas encore présents. */
    int base2 = faces->vertex_indices_ptr[f2], vc2 = faces->vertex_count[f2];
    for (int k = 0; k < vc2; ++k) {
        int vidx = faces->vertex_indices_buffer[base2 + k] - 1;
        int dup = 0;
        for (int m = 0; m < n; ++m) if (out_idx[m] == vidx) { dup = 1; break; }
        if (!dup) out_idx[n++] = vidx;
    }
    return n;
}
/*
 * ----------------------------------------------------------------------------
 *  rch_recompute_face_bbox_cache
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Recalculer la boîte projetée d'une seule face après patch de
 *    projection.
 *
 *  Paramètres :
 *    [entrée] model et f, face non vide valide.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    minx/maxx/miny/maxy de f, sans invalidation globale des caches.
 *
 *  Fonctionnement :
 *    Cette routine étroite doit être utilisée avec sauvegarde et
 *    restauration cohérentes.
 *
 * ----------------------------------------------------------------------------
 */
static void rch_recompute_face_bbox_cache(Model3D* model, int f) {
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int base = faces->vertex_indices_ptr[f], vc = faces->vertex_count[f];
    /* Initialiser les quatre bornes avec le premier sommet, qui doit exister. */
    int v0 = faces->vertex_indices_buffer[base] - 1;
    int mnx = vtx->x2d[v0], mxx = vtx->x2d[v0];
    int mny = vtx->y2d[v0], mxy = vtx->y2d[v0];
    /* Étendre la boîte avec les autres sommets puis écrire les quatre valeurs. */
    for (int k = 1; k < vc; ++k) {
        int vidx = faces->vertex_indices_buffer[base + k] - 1;
        int x = vtx->x2d[vidx], y = vtx->y2d[vidx];
        if (x < mnx) mnx = x; if (x > mxx) mxx = x;
        if (y < mny) mny = y; if (y > mxy) mxy = y;
    }
    faces->minx[f] = mnx; faces->maxx[f] = mxx;
    faces->miny[f] = mny; faces->maxy[f] = mxy;
}
/*
 * ----------------------------------------------------------------------------
 *  rch_patch_pair_at_scale
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Sauvegarder puis reprojeter temporairement les sommets de deux faces.
 *
 *  Paramètres :
 *    [entrée] model, f1/f2 et test_scale en 16.16.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    *p, sauvegarde complète nécessaire à rch_unpatch_pair
 *
 *  Fonctionnement :
 *    Modifie x2d/y2d, les deux boîtes et l'échelle globale.
 *
 *  Préconditions :
 *    profondeurs non nulles et capacité de p suffisante pour les sommets
 *    distincts des deux faces. Aucune transformation 3D.
 *
 * ----------------------------------------------------------------------------
 */
static void rch_patch_pair_at_scale(Model3D* model, int f1, int f2, Fixed32 test_scale, RCHZoomPatch* p) {
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    const Fixed32 cx_f = INT_TO_FIXED(CENTRE_X);
    const Fixed32 cy_f = INT_TO_FIXED(CENTRE_Y);

    /* Sauvegarder échelle et bornes avant toute modification. */
    p->saved_scale = s_global_proj_scale_fixed;
    s_global_proj_scale_fixed = test_scale;

    p->f1_minx = faces->minx[f1]; p->f1_maxx = faces->maxx[f1];
    p->f1_miny = faces->miny[f1]; p->f1_maxy = faces->maxy[f1];
    p->f2_minx = faces->minx[f2]; p->f2_maxx = faces->maxx[f2];
    p->f2_miny = faces->miny[f2]; p->f2_maxy = faces->maxy[f2];

    /* Rassembler les sommets distincts et conserver leurs coordonnées projetées. */
    p->count = rch_gather_pair_vertex_indices(faces, f1, f2, p->idx);


    for (int i = 0; i < p->count; ++i) {
        p->saved_x2d[i] = vtx->x2d[p->idx[i]];
        p->saved_y2d[i] = vtx->y2d[p->idx[i]];
    }


    /* Reprojeter la paire à la nouvelle échelle puis recalculer uniquement
     * ses boîtes.
     */
    for (int i = 0; i < p->count; ++i) {
        int vidx = p->idx[i];
        Fixed32 inv_zo = FIXED_DIV_64(test_scale, vtx->zo[vidx]);
        Fixed32 xt = FIXED_ADD(FIXED_MUL_64(vtx->xo[vidx], inv_zo), cx_f);
        Fixed32 yt = FIXED_SUB(cy_f, FIXED_MUL_64(vtx->yo[vidx], inv_zo));
        vtx->x2d[vidx] = FIXED_ROUND_TO_INT(xt);
        vtx->y2d[vidx] = FIXED_ROUND_TO_INT(yt);
    }

    rch_recompute_face_bbox_cache(model, f1);
    rch_recompute_face_bbox_cache(model, f2);
}
/*
 * ----------------------------------------------------------------------------
 *  rch_unpatch_pair
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Restaurer l'état temporairement remplacé par rch_patch_pair_at_scale.
 *
 *  Paramètres :
 *    [entrée-sortie] model
 *
 *    [entrée] p, sauvegarde correspondante, et f1/f2.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Restaure coordonnées écran des sommets, bornes des deux faces et
 *    projection globale. Ne restaure pas d'autres métriques car elles n'ont
 *    pas changé.
 *
 * ----------------------------------------------------------------------------
 */
static void rch_unpatch_pair(Model3D* model, RCHZoomPatch* p, int f1, int f2) {
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    /* Restituer d'abord les coordonnées, puis les deux boîtes et l'échelle. */
    for (int i = 0; i < p->count; ++i) {
        vtx->x2d[p->idx[i]] = p->saved_x2d[i];
        vtx->y2d[p->idx[i]] = p->saved_y2d[i];
    }
    faces->minx[f1] = p->f1_minx; faces->maxx[f1] = p->f1_maxx;
    faces->miny[f1] = p->f1_miny; faces->maxy[f1] = p->f1_maxy;
    faces->minx[f2] = p->f2_minx; faces->maxx[f2] = p->f2_maxx;
    faces->miny[f2] = p->f2_miny; faces->maxy[f2] = p->f2_maxy;
    s_global_proj_scale_fixed = p->saved_scale;
}
/*
 * ----------------------------------------------------------------------------
 *  ray_cast_hierarchical_impl
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Essayer plusieurs points représentatifs pour départager deux faces.
 *
 *  Paramètres :
 *    [entrée] model, f1/f2.
 *
 *  Retour :
 *    Signe du ray casting (+1 f1 avant, -1 f2 avant), ou 0 si aucune
 *    tentative ne tranche.
 *
 *  Sorties et effets de bord :
 *    diagnostics et buffers de clipping partagés.
 *
 *  Fonctionnement :
 *    Essaye le centroïde SH dans les deux sens, le centre de bbox de région
 *    QuickDraw, puis le centre des boîtes communes. Le dernier repli porte
 *    sur des boîtes et n'assure pas un point intérieur commun.
 *
 * ----------------------------------------------------------------------------
 */
static int ray_cast_hierarchical_impl(Model3D* model, int f1, int f2)
{
    int cx = 0, cy = 0;
    long long area2 = 0;
    int rc = 0;
    int fallback = 0;

    if (!model) return 0;


    /* Essayer le centroïde du clipping numérique puis le rayon à son point entier. */
    if (compute_intersection_centroid_ordered_fixed_tryboth(
            model, f1, f2, &cx, &cy, &area2, &fallback) && area2 != 0) {
        rc = ray_cast_at(model, f1, f2, cx, cy);
        if (rc != 0) return rc;
    }


    area2 = 0;
    /* Replier sur la région QuickDraw si le premier essai ne tranche pas. */
    if (compute_intersection_centroid_ordered_qd_fixed(
            model, f1, f2, &cx, &cy, &area2) && area2 != 0) {
        rc = ray_cast_at(model, f1, f2, cx, cy);
        if (rc != 0) return rc;
    }


    /* Dernier repli : utiliser le centre des bornes communes, sans garantie
     * sur les contours.
     */
    if (compute_bbox_intersection_center(model, f1, f2, &cx, &cy)) {
        return ray_cast_at(model, f1, f2, cx, cy);
    }

    return 0;
}
/*
 * ----------------------------------------------------------------------------
 *  ray_cast_hierarchical
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Départager une paire avec grossissement temporaire et cache orienté.
 *
 *  Paramètres :
 *    [entrée] model et f1/f2 valides.
 *
 *  Retour :
 *    +1 f1 plus loin, -1 f2 plus loin, 0 si aucune méthode ne conclut.
 *
 *  Fonctionnement :
 *    Ne change pas l'ordre des faces. Essaye un centroïde fractionnaire avec
 *    projection multipliée par dix. Sinon reprojette temporairement les
 *    seuls sommets de la paire pour le chemin entier, puis restaure
 *    coordonnées, bornes et échelle. Les résultats non nuls du chemin
 *    fractionnaire sont seuls mémorisés ici ; état partagé non réentrant.
 *
 * ----------------------------------------------------------------------------
 */
static int ray_cast_hierarchical(Model3D* model, int f1, int f2) {
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    if (f1 < 0 || f2 < 0 || f1 >= faces->face_count || f2 >= faces->face_count) return 0;
    RCHZoomPatch patch;
    Fixed32 original_scale = s_global_proj_scale_fixed;
    /* Chercher une relation pour la paire orientée à l'échelle d'origine. */
    OrderPairEntry *entry = ordering_pair_slot(model, order_ray_pairs, f1, f2);
    ORDER_COUNT(ray_calls);
    if (ordering_pair_hit(entry, f1, f2, original_scale, 0)) {
        ORDER_COUNT(ray_hits);
        return entry->result;
    }


    Fixed32 test_scale = FIXED_MUL_64(original_scale, FLOAT_TO_FIXED(10.0f));
    double fx, fy, fractional_area2;
    int rc = 0;
    /* Grossir le calcul fractionnaire sans modifier les coordonnées écran
     * entières.
     */
    s_global_proj_scale_fixed = test_scale;
    if (compute_intersection_centroid_fractional_tryboth(
            model, f1, f2, &fx, &fy, &fractional_area2)) {
        rc = ray_cast_at_fractional(model, f1, f2, fx, fy);
    }
    s_global_proj_scale_fixed = original_scale;
    /* Mémoriser uniquement le résultat fractionnaire concluant. */
    if (rc != 0) {
        ordering_pair_store(entry, f1, f2, original_scale, 0, rc);
        return rc;
    }


    /* En dernier recours, sauvegarder/reprojeter la paire puis restaurer
     * impérativement son état.
     */
    rch_patch_pair_at_scale(model, f1, f2, test_scale, &patch);
    rc = ray_cast_hierarchical_impl(model, f1, f2);
    rch_unpatch_pair(model, &patch, f1, f2);
    return rc;
}

/* Méthodes de chevauchement et de clipping incluses après leurs appelants
 * déclarés dans declare.c.
 */
#include "overlap_raster.c"

segment "transform";


/* Définir FACE_DEPTH_DISABLE_ASM pour le parcours C ; GEOV3_DISABLE_ASM_MUL
 * désactive aussi ce noyau.
 */
#if !defined(FACE_DEPTH_DISABLE_ASM) && !defined(GEOV3_DISABLE_ASM_MUL)
#define FACE_DEPTH_USE_ASM 1
#include "face_depth_asm.c"
#endif

/*
 * ----------------------------------------------------------------------------
 *  calculateFaceDepths
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Recalculer les métriques et plans observateur des faces de la vue
 *    courante.
 *
 *  Paramètres :
 *    [entrée] model et face_count ; faces est un paramètre historique non
 *    utilisé.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    zmin/zmax/zmean, bornes écran, plans de Newell et display_flag.
 *
 *  Fonctionnement :
 *    Invalide les caches d'ordre avant de remplacer les métriques. Le
 *    parcours est assembleur par défaut si disponible, sinon C ; le
 *    centroïde est la moyenne des sommets valides. Les produits bruts de
 *    Newell sont cumulés avant un unique décalage de 16 bits ; d utilise
 *    ensuite ce centroïde. Une profondeur strictement négative invalide la
 *    face ; le culling optionnel rejette d <= 0. Le code distingue ce test
 *    du seuil zo <= 0 de la projection. Les faces dégénérées reçoivent des
 *    plans nuls ; cela ne force pas ici à lui seul display_flag à zéro. Les
 *    indices positifs doivent déjà être valides.
 *
 * ----------------------------------------------------------------------------
 */
void calculateFaceDepths(Model3D* model, Face3D* faces, int face_count) {
    /* Changer la génération des résultats géométriques avant le parcours. */
    ordering_cache_invalidate(model);
    int i, j;
    int culled_count = 0;
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* face_arrays = &model->faces;
#if defined(FACE_DEPTH_USE_ASM)
    FaceDepth_ScanSetup(vtx);
#endif
    /* Parcourir chaque liste d'indices, avec des résultats de même format
     * pour C et assembleur.
     */
    for (i = 0; i < face_count; i++) {
        Fixed32 z_min,z_max,sum;
        int display_flag,minx,maxx,miny,maxy,valid_n;
        int n=face_arrays->vertex_count[i];
        int offset=face_arrays->vertex_indices_ptr[i];
        Fixed64 a64,b64,c64,d64,sumx64,sumy64,sumz64;
        Fixed32 xc,yc,zc;
/* Récupérer en bloc les métriques du noyau assembleur. */
#if defined(FACE_DEPTH_USE_ASM)
        FaceDepth_ScanFace(face_arrays,offset,n);
        valid_n=face_depth_result.valid_n;display_flag=face_depth_result.display_flag;
        z_min=face_depth_result.z_min;z_max=face_depth_result.z_max;sum=face_depth_result.sum;
        minx=face_depth_result.minx;maxx=face_depth_result.maxx;
        miny=face_depth_result.miny;maxy=face_depth_result.maxy;
        sumx64=face_depth_result.sumx;sumy64=face_depth_result.sumy;sumz64=face_depth_result.sumz;
        a64=face_depth_result.a;b64=face_depth_result.b;c64=face_depth_result.c;
#else
        Fixed32 x_prev=0,y_prev=0,z_prev=0;
        Fixed32 x_first=0,y_first=0,z_first=0;
        int have_prev=0;
        z_min=FLOAT_TO_FIXED(9999.0);z_max=FLOAT_TO_FIXED(-9999.0);
        display_flag=1;sum=0;valid_n=0;
        minx=miny=9999;maxx=maxy=-9999;
        a64=b64=c64=0;sumx64=sumy64=sumz64=0;
        /* Chemin C : ignorer les indices négatifs après passage à base zéro
         * et cumuler profondeur, bornes et coordonnées.
         */
        for (j = 0; j < n; j++) {
            int vertex_idx = face_arrays->vertex_indices_buffer[offset + j] - 1;
            if (vertex_idx >= 0) {
                Fixed32 zo = vtx->zo[vertex_idx];
                Fixed32 xo = vtx->xo[vertex_idx];
                Fixed32 yo = vtx->yo[vertex_idx];
                if (zo < 0) display_flag = 0;
                if (zo < z_min) z_min = zo;
                if (zo > z_max) z_max = zo;
                sum += zo;
                int x2d = vtx->x2d[vertex_idx];
                int y2d = vtx->y2d[vertex_idx];
                if (x2d < minx) minx = x2d;
                if (x2d > maxx) maxx = x2d;
                if (y2d < miny) miny = y2d;
                if (y2d > maxy) maxy = y2d;

                sumx64 += (Fixed64)xo;
                sumy64 += (Fixed64)yo;
                sumz64 += (Fixed64)zo;
                /* Initialiser le contour au premier sommet valide ; les
                 * suivants contribuent à Newell.
                 */
                if (!have_prev) {
                    x_first = xo; y_first = yo; z_first = zo;
                    x_prev  = xo; y_prev  = yo; z_prev  = zo;
                    have_prev = 1;
                } else {
                    Fixed32 dy = FIXED_SUB(y_prev, yo);
                    Fixed32 dz = FIXED_SUB(z_prev, zo);
                    Fixed32 dx = FIXED_SUB(x_prev, xo);
                    Fixed32 sy = y_prev + yo;
                    Fixed32 sz = z_prev + zo;
                    Fixed32 sx = x_prev + xo;
                    a64 += fixed_product32_fast(dy,sz);
                    b64 += fixed_product32_fast(dz,sx);
                    c64 += fixed_product32_fast(dx,sy);
                    x_prev = xo; y_prev = yo; z_prev = zo;
                }
                valid_n++;
            }
        }
#endif
        /* Neutraliser les plans des faces derrière la caméra ou
         * insuffisamment définies.
         */
        if (!display_flag || n < 3 || valid_n < 3) {

            face_arrays->plane_a[i] = 0;
            face_arrays->plane_b[i] = 0;
            face_arrays->plane_c[i] = 0;
            face_arrays->plane_d[i] = 0;
            if (shaded_by_orientation) {
                face_shade_color[i] = COL_FILL_DEFAULT;
            }
        } else {
/* Fermer le cycle de Newell et réduire la virgule fixe seulement après la
 * somme complète.
 */
#if !defined(FACE_DEPTH_USE_ASM)

            Fixed32 dy = FIXED_SUB(y_prev, y_first);
            Fixed32 dz = FIXED_SUB(z_prev, z_first);
            Fixed32 dx = FIXED_SUB(x_prev, x_first);
            Fixed32 sy = y_prev + y_first;
            Fixed32 sz = z_prev + z_first;
            Fixed32 sx = x_prev + x_first;
            a64 += fixed_product32_fast(dy,sz);
            b64 += fixed_product32_fast(dz,sx);
            c64 += fixed_product32_fast(dx,sy);
            a64 >>= FIXED_SHIFT;
            b64 >>= FIXED_SHIFT;
            c64 >>= FIXED_SHIFT;
#endif
#if defined(FACE_DEPTH_USE_ASM)
            xc=FaceDepth_CentroidCoordinate(sumx64,valid_n);
            yc=FaceDepth_CentroidCoordinate(sumy64,valid_n);
            zc=FaceDepth_CentroidCoordinate(sumz64,valid_n);
#else
            xc = (Fixed32)(sumx64 / valid_n);
            yc = (Fixed32)(sumy64 / valid_n);
            zc = (Fixed32)(sumz64 / valid_n);
#endif
            /* Calculer le terme d à partir du centroïde puis publier les coefficients. */
            d64 = -(fixed_product64x32_fast(a64,xc) + fixed_product64x32_fast(b64,yc)
                   + fixed_product64x32_fast(c64,zc));
            d64 >>= FIXED_SHIFT;
            face_arrays->plane_a[i] = a64;
            face_arrays->plane_b[i] = b64;
            face_arrays->plane_c[i] = c64;
            face_arrays->plane_d[i] = d64;


            /* Appliquer le culling observateur au signe de d, sans modifier la géométrie. */
            if (cull_back_faces && display_flag) {
                if (d64 <= 0) {
                    display_flag = 0;
                    ++culled_count;
                }
            }
        }
        /* Publier les profondeurs et la visibilité ; les faces vides
         * reçoivent des métriques nulles.
         */
        face_arrays->z_min[i] = z_min;
        face_arrays->z_max[i] = z_max;
        face_arrays->display_flag[i] = display_flag;
        if (n > 0) {
            if (n == 4)      face_arrays->z_mean[i] = sum >> 2;
            else if (n == 3) face_arrays->z_mean[i] = sum / 3;
            else             face_arrays->z_mean[i] = sum / n;
            face_arrays->minx[i] = minx;
            face_arrays->maxx[i] = maxx;
            face_arrays->miny[i] = miny;
            face_arrays->maxy[i] = maxy;
        } else {
            face_arrays->z_mean[i] = 0;
            face_arrays->z_min[i] = 0;
            face_arrays->z_max[i] = 0;
            face_arrays->minx[i] = 0;
            face_arrays->maxx[i] = 0;
            face_arrays->miny[i] = 0;
            face_arrays->maxy[i] = 0;
        }
    }
}
/*
 * ----------------------------------------------------------------------------
 *  compute2DFromObserver
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Reprojeter une vue observateur existante après changement d'échelle.
 *
 *  Paramètres :
 *    [entrée-sortie] model, coordonnées projetées et bornes des faces.
 *
 *    [entrée] angle_w, ignoré car la rotation est déjà contenue dans
 *    xo/yo/zo.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Synchronise auto_proj_scale avec l'échelle globale et invalide les
 *    caches d'ordre via updateFace2DBounds.
 *
 *  Préconditions :
 *    model valide et tous les zo divisés non nuls ; contrairement à
 *    processModelFast, aucun rejet zo <= 0 n'est effectué ici. La
 *    transformation 3D et les plans ne sont pas recalculés.
 *
 * ----------------------------------------------------------------------------
 */
void compute2DFromObserver(Model3D* model, int angle_w) {
    /* Employer les coordonnées observateur déjà tournées et l'échelle
     * globale courante.
     */
    (void)angle_w;
    VertexArrays3D* vtx = &model->vertices;
    int vcount = vtx->vertex_count;
    Fixed32* xo = vtx->xo; Fixed32* yo = vtx->yo; Fixed32* zo = vtx->zo;
    int* x2d_out = vtx->x2d; int* y2d_out = vtx->y2d;
    Fixed32 scale = s_global_proj_scale_fixed;

    model->auto_proj_scale = scale;
    const Fixed32 centre_x_f = INT_TO_FIXED(CENTRE_X);
    const Fixed32 centre_y_f = INT_TO_FIXED(CENTRE_Y);
    /* Effectuer la division perspective et convertir en pixels sans filtre
     * de profondeur.
     */
    for (int i = 0; i < vcount; ++i) {
        Fixed32 xo_i = xo[i];
        Fixed32 yo_i = yo[i];
        Fixed32 zo_i = zo[i];
        Fixed32 inv_zo = FIXED_DIV_64(scale, zo_i);
        Fixed32 x2d_temp = FIXED_ADD(FIXED_MUL_64(xo_i, inv_zo), centre_x_f);
        Fixed32 y2d_temp = FIXED_SUB(centre_y_f, FIXED_MUL_64(yo_i, inv_zo));
        x2d_out[i] = FIXED_ROUND_TO_INT(x2d_temp);
        y2d_out[i] = FIXED_ROUND_TO_INT(y2d_temp);
    }


    /* Rafraîchir les bornes projetées pour éviter les rejets de paires
     * fondés sur l'ancien zoom.
     */
    updateFace2DBounds(model);
}
/*
 * ----------------------------------------------------------------------------
 *  updateFace2DBounds
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Recalculer les boîtes écran des faces après reprojection.
 *
 *  Paramètres :
 *    [entrée-sortie] model ; NULL est accepté.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Invalide les caches d'ordre, parcourt les indices et écrit les bornes
 *    projetées. Ne recalcule ni plans, ni profondeur, ni display_flag.
 *
 * ----------------------------------------------------------------------------
 */
static void updateFace2DBounds(Model3D* model) {
    /* Ignorer un modèle absent avant de toucher aux caches. */
    if (!model) return;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int face_count = faces->face_count;
    /* Étendre les bornes avec les coordonnées projetées courantes des
     * sommets de chaque face.
     */
    for (int fi = 0; fi < face_count; ++fi) {
        int n = faces->vertex_count[fi];
        if (n <= 0) {
            faces->minx[fi] = faces->miny[fi] = 0;
            faces->maxx[fi] = faces->maxy[fi] = 0;
            continue;
        }
        int off = faces->vertex_indices_ptr[fi];
        int minx = 9999, maxx = -9999, miny = 9999, maxy = -9999;
        for (int k = 0; k < n; ++k) {
            int vid = faces->vertex_indices_buffer[off + k] - 1;
            if (vid < 0 || vid >= vtx->vertex_count) continue;
            int x = vtx->x2d[vid];
            int y = vtx->y2d[vid];
            if (x < minx) minx = x;
            if (x > maxx) maxx = x;
            if (y < miny) miny = y;
            if (y > maxy) maxy = y;
        }
        if (maxx == -9999) { minx = maxx = miny = maxy = 0; }
        faces->minx[fi] = minx;
        faces->maxx[fi] = maxx;
        faces->miny[fi] = miny;
        faces->maxy[fi] = maxy;
    }
}
/*
 * ----------------------------------------------------------------------------
 *  normalizeAutoFitDistanceTo150
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Appliquer la normalisation existante des suggestions de vue vers une
 *    distance de 150.
 *
 *  Paramètres :
 *    [entrée-sortie] model ; NULL, distance ou projection non positives sont
 *    ignorés.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    géométrie et métadonnées modifiées, message affiché ; la vue dérivée
 *    doit ensuite être recalculée par processModelFast.
 *
 *  Fonctionnement :
 *    Multiplie les coordonnées objet et la projection par sqrt(150/D), fixe
 *    la distance suggérée et current_observer_distance à 150. La formule
 *    conserve un rapport d'échelle global ; elle ne démontre pas une
 *    invariance exacte de perspective pour des sommets de profondeurs
 *    différentes.
 *
 * ----------------------------------------------------------------------------
 */
void normalizeAutoFitDistanceTo150(Model3D* model) {
    if (model == NULL) return;
    /* Vérifier les deux suggestions avant le calcul du facteur. */
    Fixed32 D = model->auto_suggested_distance;
    Fixed32 P = model->auto_suggested_proj_scale;
    if (D <= 0 || P <= 0) return;
    const Fixed32 targetD = INT_TO_FIXED(150);


    /* Calculer le facteur racine puis le convertir en 16.16. */
    float factor_f = sqrtf(FIXED_TO_FLOAT(targetD) / FIXED_TO_FLOAT(D));
    if (factor_f <= 0.0f) return;
    Fixed32 factor = FLOAT_TO_FIXED(factor_f);

    VertexArrays3D* vtx = &model->vertices;
    /* Appliquer le facteur aux trois coordonnées objet. */
    for (int i = 0; i < vtx->vertex_count; ++i) {
        vtx->x[i] = FIXED_MUL_64(vtx->x[i], factor);
        vtx->y[i] = FIXED_MUL_64(vtx->y[i], factor);
        vtx->z[i] = FIXED_MUL_64(vtx->z[i], factor);
    }

    /* Synchroniser distance suggérée, projection globale et distance de tolérance. */
    model->auto_suggested_distance = targetD;
    model->auto_suggested_proj_scale = FIXED_MUL_64(P, factor);
    model->auto_proj_scale = model->auto_suggested_proj_scale;
    s_global_proj_scale_fixed = model->auto_proj_scale;

    current_observer_distance = targetD;
    printf("Auto-fit normalized: D=%.4f P=%.2f factor=%.4f\n",
        FIXED_TO_FLOAT(model->auto_suggested_distance),
        FIXED_TO_FLOAT(model->auto_suggested_proj_scale),
        FIXED_TO_FLOAT(factor));
}
/*
 * ----------------------------------------------------------------------------
 *  getObserverParams
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Saisir les angles et la distance de vue en conservant les valeurs
 *    proposées.
 *
 *  Paramètres :
 *    [entrée-sortie] params
 *
 *    [entrée-sortie] model pour les métadonnées d'ajustement, éventuellement
 *    NULL.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    entrées-sorties texte.
 *
 *  Fonctionnement :
 *    Les angles sont lus en degrés entiers, la distance par atof puis
 *    conversion 16.16 ; aucune validation numérique forte. Une distance vide
 *    ou un échec de lecture réutilise les suggestions si disponibles, sinon
 *    30. Met à jour current_observer_distance ; la projection globale n'est
 *    pas directement réaffectée dans cette fonction.
 *
 * ----------------------------------------------------------------------------
 */
void getObserverParams(ObserverParams* params, Model3D* model) {
    char userinput[50];


    printf("\nObserver parameters:\n");
    printf("============================\n");
    printf("(Press ENTER to use default values - apply suggested auto-fit if available)\n");


    /* Proposer successivement les trois angles ; une ligne vide conserve la
     * valeur existante.
     */
    printf("Horizontal angle (degrees, default %d): ", params->angle_h);
    if (fgets(userinput, sizeof(userinput), stdin) != NULL) {

        userinput[strcspn(userinput, "\n")] = 0;
        if (strlen(userinput) != 0) {
            params->angle_h = atoi(userinput);
        }
    }


    printf("Vertical angle (degrees, default %d): ", params->angle_v);
    if (fgets(userinput, sizeof(userinput), stdin) != NULL) {

        userinput[strcspn(userinput, "\n")] = 0;
        if (strlen(userinput) != 0) {
            params->angle_v = atoi(userinput);
        }
    }


    printf("Screen rotation angle (degrees, default %d): ", params->angle_w);
    if (fgets(userinput, sizeof(userinput), stdin) != NULL) {

        userinput[strcspn(userinput, "\n")] = 0;
        if (strlen(userinput) != 0) {
            params->angle_w = atoi(userinput);
        }
    }


    /* Traiter séparément la distance numérique et la sélection de
     * l'ajustement suggéré.
     */
    printf("Distance (ENTER = auto-scale, or enter a value): ");
    if (fgets(userinput, sizeof(userinput), stdin) != NULL) {

        userinput[strcspn(userinput, "\n")] = 0;
        if (strlen(userinput) == 0) {

            if (model != NULL && model->auto_fit_ready) {
                params->distance = model->auto_suggested_distance;

                model->auto_proj_scale = model->auto_suggested_proj_scale;
                model->auto_fit_applied = 1;
                printf("Auto-fit applied (distance=%.4f proj_scale=%.2f). Use +/- to adjust.\n", FIXED_TO_FLOAT(model->auto_suggested_distance), FIXED_TO_FLOAT(model->auto_suggested_proj_scale));
            } else if (model != NULL) {

                params->distance = FLOAT_TO_FIXED(30.0);
            } else {
                params->distance = FLOAT_TO_FIXED(30.0);
            }
        } else {

            params->distance = FLOAT_TO_FIXED(atof(userinput));
        }
    } else {

        if (model != NULL && model->auto_fit_ready) {
            params->distance = model->auto_suggested_distance;
            model->auto_proj_scale = model->auto_suggested_proj_scale;
            model->auto_fit_applied = 1;
            printf("Auto-fit applied (distance=%.4f proj_scale=%.2f). Use +/- to adjust.\n", FIXED_TO_FLOAT(model->auto_suggested_distance), FIXED_TO_FLOAT(model->auto_suggested_proj_scale));
        } else if (model != NULL) {
            params->distance = FLOAT_TO_FIXED(30.0);
        } else {
            params->distance = FLOAT_TO_FIXED(30.0);
        }
    }


    /* Publier la distance utilisée par les tolérances géométriques. */
    current_observer_distance = params->distance;
}

/* Dessin QuickDraw selon la permutation courante ; aucune correction d'ordre
 * pendant le remplissage.
 */
segment "rendering";
/*
 * ----------------------------------------------------------------------------
 *  drawPolygons
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Dessiner les faces affichables dans la permutation calculée par la méthode
 *    de classement.
 *
 *  Paramètres :
 *    [entrée] model et face_count ; vertex_count et vertex_count_total sont
 *    des paramètres historiques non utilisés dans ce corps.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    port QuickDraw, crayon et handle global de polygone verrouillé.
 *
 *  Fonctionnement :
 *    Construit chaque contour avec pan, rejette les indices invalides et les
 *    boîtes hors écran, puis remplit et encadre, ou encadre seulement en
 *    mode filaire. La priorité du remplissage est shading, aléatoire, choix
 *    utilisateur, défaut.
 *
 *  Préconditions :
 *    QuickDraw actif ; au plus MAX_FACE_VERTICES points par face. Ne calcule
 *    ni visibilité, ni plans, ni ordre, et n'efface pas l'écran.
 *
 * ----------------------------------------------------------------------------
 */
void drawPolygons(Model3D* model, int* vertex_count, int face_count, int vertex_count_total) {
    int i, j;
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    Handle polyHandle;
    DynamicPolygon *poly;
    int min_x, max_x, min_y, max_y;
    int valid_faces_drawn = 0;
    int invalid_faces_skipped = 0;
    int triangle_count = 0;
    int quad_count = 0;
    Pattern pat;


    int screenScale = mode / 320;
    int screenW = screenScale * (CENTRE_X * 2);
    int screenH = screenScale * (CENTRE_Y * 2);


    /* Réserver une fois un polygone de taille maximale puis verrouiller son
     * handle.
     */
    if (globalPolyHandle == NULL) {
        int max_polySize = 2 + 8 + (MAX_FACE_VERTICES * 4);
        globalPolyHandle = NewHandle((long)max_polySize, userid(), 0xC014, 0L);
        if (globalPolyHandle == NULL) {
            printf("Error: Unable to allocate global polygon handle\n");
            return;
        }
    }

    polyHandle = globalPolyHandle;


    if (poly_handle_locked) {
        HUnlock(polyHandle);
        poly_handle_locked = 0;
    }
    HLock(polyHandle);
    poly_handle_locked = 1;
    SetPenMode(0);


    SetSolidPenPat(COL_FILL_DEFAULT);


    int start_face = 0;
    int max_faces_to_draw = face_count;
    /* Parcourir la permutation de rendu et ignorer les faces non affichables. */
    for (i = start_face; i < start_face + max_faces_to_draw; i++) {
        int face_id = faces->sorted_face_indices[i];
        if (faces->display_flag[face_id] == 0) continue;
        if (faces->vertex_count[face_id] >= 3) {
            int offset = faces->vertex_indices_ptr[face_id];
            int vcount_face = faces->vertex_count[face_id];
            int *indices_base = &faces->vertex_indices_buffer[offset];

            /* Valider tous les indices avant de lire les coordonnées projetées. */
            int all_valid = 1;
            for (j = 0; j < vcount_face; ++j) {
                int vi = indices_base[j] - 1;
                if (vi < 0 || vi >= vtx->vertex_count) { all_valid = 0; break; }
            }
            if (!all_valid) { invalid_faces_skipped++; continue; }

            /* Écrire points et bornes QuickDraw avec les décalages écran. */
            int polySize = 2 + 8 + (vcount_face * 4);
            poly = (DynamicPolygon *)*polyHandle;
            poly->polySize = polySize;

            int *x2d = vtx->x2d;
            int *y2d = vtx->y2d;

            int first_vi = indices_base[0] - 1;
            int x = x2d[first_vi];
            int y = y2d[first_vi];
            poly->polyPoints[0].h = screenScale * (x + pan_dx);
            poly->polyPoints[0].v = y + pan_dy;
            min_x = max_x = x;
            min_y = max_y = y;

            for (j = 1; j < vcount_face; ++j) {
                int vi = indices_base[j] - 1;
                x = x2d[vi];
                y = y2d[vi];
                poly->polyPoints[j].h = screenScale * (x + pan_dx);
                poly->polyPoints[j].v = y + pan_dy;
                if (x < min_x) min_x = x;
                if (x > max_x) max_x = x;
                if (y < min_y) min_y = y;
                if (y > max_y) max_y = y;
            }
            poly->polyBBox.h1 = min_x + pan_dx;
            poly->polyBBox.v1 = min_y + pan_dy;
            poly->polyBBox.h2 = max_x + pan_dx;
            poly->polyBBox.v2 = max_y + pan_dy;

            int sc_min_x = screenScale * (min_x + pan_dx);
            int sc_max_x = screenScale * (max_x + pan_dx);
            int sc_min_y = (min_y + pan_dy);
            int sc_max_y = (max_y + pan_dy);
            /* Rejeter les polygones entièrement hors du viewport. */
            if (sc_max_x < 0 || sc_min_x >= screenW || sc_max_y < 0 || sc_min_y >= screenH) {

            } else {
                /* Séparer le choix du contour filaire des couleurs de surfaces. */
                if (framePolyOnly) {

                    int frame_color;
                    if (user_frame_color == 17) {

                        if (user_fill_color == 16 && random_fill_colors != NULL && face_id < random_colors_capacity) {
                            frame_color = random_fill_colors[face_id];
                        } else if (user_fill_color >= 0) {
                            frame_color = user_fill_color;
                        } else {
                            frame_color = COL_FILL_DEFAULT;
                        }
                    } else if (user_frame_color == 16 && random_frame_colors != NULL && face_id < random_colors_capacity) {
                        frame_color = random_frame_colors[face_id];
                    } else if (user_frame_color >= 0) {
                        frame_color = user_frame_color;
                    } else {
                        frame_color = 7;
                    }
                    SetSolidPenPat(frame_color);
                    FramePoly(polyHandle);
                    SetSolidPenPat(COL_FILL_DEFAULT);
                } else {

                    /* Choisir la couleur effective du remplissage avant FillPoly et FramePoly. */
                    int fill_color;
                    if (shaded_by_orientation) {
                        fill_color = face_shade_color[face_id];
                    } else if (user_fill_color == 16 && random_fill_colors != NULL && face_id < random_colors_capacity) {
                        fill_color = random_fill_colors[face_id];
                    } else if (user_fill_color >= 0) {
                        fill_color = user_fill_color;
                    } else {
                        fill_color = COL_FILL_DEFAULT;
                    }
                    SetSolidPenPat(fill_color);
                    GetPenPat(pat);
                    FillPoly(polyHandle, pat);


                    int frame_color;
                    if (user_frame_color == 17) {

                        frame_color = fill_color;
                    } else if (user_frame_color == 16 && random_frame_colors != NULL && face_id < random_colors_capacity) {
                        frame_color = random_frame_colors[face_id];
                    } else if (user_frame_color >= 0) {
                        frame_color = user_frame_color;
                    } else {
                        frame_color = COL_FRAME;
                    }
                    SetSolidPenPat(frame_color);
                    FramePoly(polyHandle);
                    SetSolidPenPat(COL_FILL_DEFAULT);
                }
                valid_faces_drawn++;
                if (vcount_face == 3) triangle_count++;
                else if (vcount_face == 4) quad_count++;
            }
        } else {
            invalid_faces_skipped++;
        }
    }
}
/*
 * ----------------------------------------------------------------------------
 *  drawPolygons_jitter
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Dessiner une vue avec un déplacement aléatoire temporaire de chaque
 *    sommet.
 *
 *  Paramètres :
 *    [entrée] model, vertex_count, face_count et vertex_count_total transmis
 *    au dessin.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Modifie x2d/y2d pendant l'appel, puis les restaure. Les déplacements
 *    indépendants sont positifs dans [0,jitter_max], pas centrés. Sur échec
 *    de réserve, dessine sans jitter. Ne recalcule pas l'ordre ni les plans.
 *
 * ----------------------------------------------------------------------------
 */
void drawPolygons_jitter(Model3D* model, int* vertex_count, int face_count, int vertex_count_total) {
    if (!model) return;
    VertexArrays3D* vtx = &model->vertices;
    int vcount = vertex_count_total;
    if (vcount <= 0) { drawPolygons(model, vertex_count, face_count, vertex_count_total); return; }

    /* Réserver la sauvegarde des coordonnées ou replier sur le dessin normal. */
    int *saved_x = (int*)malloc(vcount * sizeof(int));
    int *saved_y = (int*)malloc(vcount * sizeof(int));
    if (!saved_x || !saved_y) {
        if (saved_x) free(saved_x);
        if (saved_y) free(saved_y);

        drawPolygons(model, vertex_count, face_count, vertex_count_total);
        return;
    }
    /* Sauvegarder les coordonnées avant les décalages aléatoires. */
    for (int i = 0; i < vcount; ++i) {
        saved_x[i] = vtx->x2d[i];
        saved_y[i] = vtx->y2d[i];
        int dx = rand() % (jitter_max + 1);
        int dy = rand() % (jitter_max + 1);
        vtx->x2d[i] = saved_x[i] + dx;
        vtx->y2d[i] = saved_y[i] + dy;
    }

    drawPolygons(model, vertex_count, face_count, vertex_count_total);

    for (int i = 0; i < vcount; ++i) {
        /* Après dessin, restituer exactement les coordonnées et libérer les
         * sauvegardes.
         */
        vtx->x2d[i] = saved_x[i];
        vtx->y2d[i] = saved_y[i];
    }
    free(saved_x);
    free(saved_y);
}/*
 * ----------------------------------------------------------------------------
 *  drawFace
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Dessiner une face isolée pour les outils d'inspection.
 *
 *  Paramètres :
 *    [entrée] model ; face_id à base zéro ; fillPenPat >=0 impose un
 *    remplissage, sinon emploie le mode filaire ou le motif courant ;
 *    show_index affiche l'ID.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    QuickDraw, motif et handle global.
 *
 *  Fonctionnement :
 *    Vérifie visibilité, longueur et indices, puis dessine avec pan.
 *
 *  Préconditions :
 *    QuickDraw actif.
 *
 *  Limites :
 *    si cette fonction alloue elle-même le handle global, la réserve est
 *    dimensionnée pour quatre sommets seulement ; elle n'est pas agrandie
 *    ici.
 *
 * ----------------------------------------------------------------------------
 */
void drawFace(Model3D* model, int face_id, int fillPenPat, int show_index) {
    if (!model) return;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    /* Rejeter une face hors limites, masquée ou insuffisamment définie. */
    if (face_id < 0 || face_id >= faces->face_count) return;
    if (faces->display_flag[face_id] == 0) return;
    int vcount_face = faces->vertex_count[face_id];
    if (vcount_face < 3) return;
    int offset = faces->vertex_indices_ptr[face_id];
    int *indices_base = &faces->vertex_indices_buffer[offset];

    for (int j = 0; j < vcount_face; ++j) {
        int vi = indices_base[j] - 1;
        if (vi < 0 || vi >= vtx->vertex_count) return;
    }

    /* Réutiliser le handle global ; la réserve locale initiale ne couvre que
     * quatre points.
     */
    if (globalPolyHandle == NULL) {
        int max_polySize = 2 + 8 + (4 * 4);
        globalPolyHandle = NewHandle((long)max_polySize, userid(), 0xC014, 0L);
        if (globalPolyHandle == NULL) {
            printf("Error: Unable to allocate global polygon handle\n");
            return;
        }
    }
    Handle polyHandle = globalPolyHandle;
    if (poly_handle_locked) { HUnlock(polyHandle); poly_handle_locked = 0; }
    HLock(polyHandle); poly_handle_locked = 1;
    DynamicPolygon *poly = (DynamicPolygon *)*polyHandle;
    int polySize = 2 + 8 + (vcount_face * 4);
    poly->polySize = polySize;
    /* Assembler points et boîte de la face dans le port graphique. */
    int *x2d = vtx->x2d;
    int *y2d = vtx->y2d;

    int first_vi = indices_base[0] - 1;
    int px = x2d[first_vi];
    int py = y2d[first_vi];
    poly->polyPoints[0].h = mode / 320 * (px + pan_dx);
    poly->polyPoints[0].v = py + pan_dy;
    int min_x = px, max_x = px, min_y = py, max_y = py;
    for (int j = 1; j < vcount_face; ++j) {
        int vi = indices_base[j] - 1;
        px = x2d[vi];
        py = y2d[vi];
        poly->polyPoints[j].h = mode / 320 * (px + pan_dx);
        poly->polyPoints[j].v = py + pan_dy;
        if (px < min_x) min_x = px;
        if (px > max_x) max_x = px;
        if (py < min_y) min_y = py;
        if (py > max_y) max_y = py;
    }
    poly->polyBBox.h1 = min_x + pan_dx;
    poly->polyBBox.v1 = min_y + pan_dy;
    poly->polyBBox.h2 = max_x + pan_dx;
    poly->polyBBox.v2 = max_y + pan_dy;
    int screenScale = mode / 320;
    int screenW = screenScale * (CENTRE_X * 2);
    int screenH = screenScale * (CENTRE_Y * 2);
    int sc_min_x = screenScale * (min_x + pan_dx);
    int sc_max_x = screenScale * (max_x + pan_dx);
    int sc_min_y = (min_y + pan_dy);
    int sc_max_y = (max_y + pan_dy);
    if (sc_max_x < 0 || sc_min_x >= screenW || sc_max_y < 0 || sc_min_y >= screenH) {

    } else {
        /* Appliquer la couleur imposée, le filaire ou le motif courant selon
         * la demande.
         */
        if (fillPenPat >= 0) {
            SetSolidPenPat(fillPenPat);
            Pattern pat;
            GetPenPat(pat);
            FillPoly(polyHandle, pat);
            SetSolidPenPat(COL_FRAME);
            FramePoly(polyHandle);
            SetSolidPenPat(COL_FILL_DEFAULT);
        } else if (framePolyOnly) {


            SetPenMode(0);
            SetSolidPenPat(COL_FRAME);
            FramePoly(polyHandle);


            SetSolidPenPat(COL_FILL_DEFAULT);
        } else {
            Pattern pat;
            GetPenPat(pat);
            FillPoly(polyHandle, pat);
            SetSolidPenPat(COL_FRAME);
            FramePoly(polyHandle);
            SetSolidPenPat(COL_FILL_DEFAULT);
        }

        /* Ajouter l'identifiant au centre de la bbox, qui n'est pas
         * nécessairement intérieur au polygone.
         */
        if (show_index) {
            int center_x = (min_x + max_x) / 2;
            int center_y = (min_y + max_y) / 2;

            int screenCx = screenScale * (center_x + pan_dx);
            int screenCy = center_y + pan_dy;
            char tmp[32];
            int len = snprintf(tmp, sizeof(tmp), "%d", face_id);
            if (len > 15) len = 15;
            unsigned char pstr[16];
            pstr[0] = (unsigned char)len;
            memcpy(&pstr[1], tmp, len);
            MoveTo(screenCx, screenCy);
            DrawString(pstr);
        }
    }
}
/*
 * ----------------------------------------------------------------------------
 *  drawFaceIndex
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Écrire l'identifiant d'une face au centre de sa boîte projetée.
 *
 *  Paramètres :
 *    [entrée] model et face_id à base zéro.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Valide visibilité et indices ; applique le pan et l'échelle écran.
 *    Conserve et restaure le motif de crayon. Ne dessine pas le contour ni
 *    la surface.
 *
 * ----------------------------------------------------------------------------
 */
void drawFaceIndex(Model3D* model, int face_id) {
    if (!model) return;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    if (face_id < 0 || face_id >= faces->face_count) return;
    if (faces->display_flag[face_id] == 0) return;
    int vcount_face = faces->vertex_count[face_id];
    if (vcount_face < 3) return;
    int offset = faces->vertex_indices_ptr[face_id];
    int *indices_base = &faces->vertex_indices_buffer[offset];
    /* Mesurer la boîte projetée en validant les indices. */
    int min_x = 999999, max_x = -999999, min_y = 999999, max_y = -999999;
    for (int j = 0; j < vcount_face; ++j) {
        int vi = indices_base[j] - 1;
        if (vi < 0 || vi >= vtx->vertex_count) return;
        int x = vtx->x2d[vi];
        int y = vtx->y2d[vi];
        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;
    }
    /* Convertir son centre en coordonnées écran et construire une chaîne Pascal. */
    int screenScale = mode / 320;
    int center_x = (min_x + max_x) / 2;
    int center_y = (min_y + max_y) / 2;
    int screenCx = screenScale * (center_x + pan_dx);
    int screenCy = center_y + pan_dy;
    char tmp[32];
    int len = snprintf(tmp, sizeof(tmp), "%d", face_id);
    if (len > 15) len = 15;
    unsigned char pstr[16];
    pstr[0] = (unsigned char)len;
    memcpy(&pstr[1], tmp, len);
    /* Préserver le motif autour du dessin du libellé. */
    Pattern savedPat;
    GetPenPat(savedPat);
    SetSolidPenPat(COL_FRAME);
    MoveTo(screenCx, screenCy);
    DrawString(pstr);
    SetPenPat(savedPat);
}
/*
 * ----------------------------------------------------------------------------
 *  frameInconclusivePairs
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Surligner les faces des paires inconclusives conservées par les méthodes
 *    de classement.
 *
 *  Paramètres :
 *    [entrée] model et buffer global inconclusive_pairs.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    handle global et crayon QuickDraw ; les faces peuvent être dessinées
 *    plusieurs fois.
 *
 *  Fonctionnement :
 *    Malgré son nom, remplit les faces en vert clair et les encadre en
 *    jaune.
 *
 *    Les tests de boîte de ce diagnostic ne traitent pas le pan de façon
 *    identique au dessin des points.
 *
 * ----------------------------------------------------------------------------
 */
void frameInconclusivePairs(Model3D* model) {
    if (!model) return;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    /* Ignorer un diagnostic vide et assurer le handle de polygone partagé. */
    if (inconclusive_pairs == NULL || inconclusive_pairs_count == 0) return;

    if (globalPolyHandle == NULL) {
        int max_polySize = 2 + 8 + (MAX_FACE_VERTICES * 4);
        globalPolyHandle = NewHandle((long)max_polySize, userid(), 0xC014, 0L);
        if (globalPolyHandle == NULL) {
            printf("Error: Unable to allocate global polygon handle\n");
            return;
        }
    }
    Handle polyHandle = globalPolyHandle;
    if (poly_handle_locked) { HUnlock(polyHandle); poly_handle_locked = 0; }
    HLock(polyHandle); poly_handle_locked = 1;
    DynamicPolygon *poly = (DynamicPolygon *)*polyHandle;

    /* Parcourir les deux faces de chaque paire en contrôlant tailles et indices. */
    for (int p = 0; p < inconclusive_pairs_count; ++p) {
        int pair_faces[2] = { inconclusive_pairs[p].face1, inconclusive_pairs[p].face2 };
        for (int pf = 0; pf < 2; ++pf) {
            int face_id = pair_faces[pf];
            if (face_id < 0 || face_id >= faces->face_count) continue;
            if (faces->display_flag[face_id] == 0) continue;
            int vcount_face = faces->vertex_count[face_id];
            if (vcount_face < 3 || vcount_face > MAX_FACE_VERTICES) continue;
            int offset = faces->vertex_indices_ptr[face_id];
            int *indices_base = &faces->vertex_indices_buffer[offset];

            int all_valid = 1;
            for (int j = 0; j < vcount_face; ++j) {
                int vi = indices_base[j] - 1;
                if (vi < 0 || vi >= vtx->vertex_count) { all_valid = 0; break; }
            }
            if (!all_valid) continue;

            /* Assembler les points de surlignage et leur boîte de rejet. */
            int screenScale = mode / 320;
            int polySize = 2 + 8 + (vcount_face * 4);
            poly->polySize = polySize;
            int first_vi = indices_base[0] - 1;
            int px = vtx->x2d[first_vi];
            int py = vtx->y2d[first_vi];
            poly->polyPoints[0].h = screenScale * (px + pan_dx);
            poly->polyPoints[0].v = py + pan_dy;
            int min_x = px, max_x = px, min_y = py, max_y = py;
            for (int j = 1; j < vcount_face; ++j) {
                int vi = indices_base[j] - 1;
                px = vtx->x2d[vi];
                py = vtx->y2d[vi];
                poly->polyPoints[j].h = screenScale * (px + pan_dx);
                poly->polyPoints[j].v = py + pan_dy;
                if (px < min_x) min_x = px;
                if (px > max_x) max_x = px;
                if (py < min_y) min_y = py;
                if (py > max_y) max_y = py;
            }
            poly->polyBBox.h1 = min_x;
            poly->polyBBox.v1 = min_y;
            poly->polyBBox.h2 = max_x;
            poly->polyBBox.v2 = max_y;

            int screenW = screenScale * (CENTRE_X * 2);
            int screenH = screenScale * (CENTRE_Y * 2);
            int sc_min_x = screenScale * min_x;
            int sc_max_x = screenScale * max_x;
            int sc_min_y = screenScale * min_y;
            int sc_max_y = screenScale * max_y;
            if (sc_max_x < 0 || sc_min_x >= screenW || sc_max_y < 0 || sc_min_y >= screenH) continue;

            /* Remplir en vert et encadrer en jaune sans restaurer le motif antérieur. */
            SetSolidPenPat(COL_LIGHT_GREEN);
            Pattern pat;
            GetPenPat(pat);
            FillPoly(polyHandle, pat);
            SetSolidPenPat(COL_YELLOW);
            FramePoly(polyHandle);
        }
    }

}
/*
 * ----------------------------------------------------------------------------
 *  generate_random_colors
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Générer deux tableaux de couleurs non noires pour les faces.
 *
 *  Paramètres :
 *    [entrée] face_count.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    random_fill_colors, random_frame_colors et leur capacité ; valeurs
 *    entre 1 et 15 incluses.
 *
 *  Fonctionnement :
 *    Consomme rand sans initialiser sa graine ici.
 *
 *  Limites :
 *    les retours de realloc ne sont pas vérifiés avant utilisation ; cette
 *    révision conserve ce comportement.
 *
 * ----------------------------------------------------------------------------
 */
void generate_random_colors(int face_count) {
    if (face_count <= 0) return;


    /* Agrandir les deux buffers si nécessaire. */
    if (random_colors_capacity < face_count) {
        random_fill_colors = (unsigned char*)realloc(random_fill_colors, face_count);
        random_frame_colors = (unsigned char*)realloc(random_frame_colors, face_count);
        random_colors_capacity = face_count;
    }


    /* Générer indépendamment remplissage et contour, hors indice noir. */
    for (int i = 0; i < face_count; i++) {
        random_fill_colors[i] = (rand() % 15) + 1;
        random_frame_colors[i] = (rand() % 15) + 1;
    }
}
/*
 * ----------------------------------------------------------------------------
 *  display_model_face_ids
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Afficher une vue graphique annotée avec les identifiants des faces
 *    visibles.
 *
 *  Paramètres :
 *    [entrée] model, params requis mais non recalculés ; filename non
 *    utilisé.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Sauvegarde les drapeaux et l'option filaire, dessine le modèle puis les
 *    faces annotées, attend une touche et revient au texte.
 *
 *  Limites :
 *    l'allocation de la sauvegarde des drapeaux n'est pas vérifiée ici.
 *
 * ----------------------------------------------------------------------------
 */
void display_model_face_ids(Model3D* model, ObserverParams* params, const char* filename) {
    if (!model || !params) return;
    FaceArrays3D* faces = &model->faces;
    if (faces->face_count <= 0) { printf("No faces in model\n"); return; }
    /* Sauvegarder l'état de visibilité et ouvrir une vue d'inspection. */
    startgraph(mode);

    unsigned char* backup_flags = (unsigned char*)malloc(faces->face_count);
    for (int i = 0; i < faces->face_count; ++i) backup_flags[i] = faces->display_flag[i];
    int old_frame = framePolyOnly;
    framePolyOnly = 1;

    if (jitter) drawPolygons_jitter(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count); else drawPolygons(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count);
    int screenScale = mode / 320;

    /* Dessiner les faces dans l'ordre courant et placer les IDs aux centres
     * de leurs boîtes.
     */
    for (int si = 0; si < faces->face_count; ++si) {
        int f = faces->sorted_face_indices[si];
        if (!faces->display_flag[f]) continue;

        drawFace(model, f, COL_LIGHT_GREEN, 0);

        int offset = faces->vertex_indices_ptr[f];
        int n = faces->vertex_count[f];
        int min_x = 999999, max_x = -999999, min_y = 999999, max_y = -999999;
        for (int k = 0; k < n; ++k) {
            int vi = faces->vertex_indices_buffer[offset + k] - 1;
            if (vi < 0 || vi >= model->vertices.vertex_count) continue;
            int x = model->vertices.x2d[vi];
            int y = model->vertices.y2d[vi];
            if (x < min_x) min_x = x; if (x > max_x) max_x = x; if (y < min_y) min_y = y; if (y > max_y) max_y = y;
        }
        if (min_x <= max_x && min_y <= max_y) {
            int center_x = (min_x + max_x) / 2;
            int center_y = (min_y + max_y) / 2;
            int screenCx = screenScale * (center_x + pan_dx);
            int screenCy = center_y + pan_dy;
            char tmp[32];
            int len = snprintf(tmp, sizeof(tmp), "%d", f);
            if (len > 15) len = 15;
            unsigned char pstr[16];
            pstr[0] = (unsigned char)len;
            memcpy(&pstr[1], tmp, len);

            SetSolidPenPat(COL_LIGHT_GREEN);
            MoveTo(screenCx, screenCy);
            DrawString(pstr);
            SetSolidPenPat(COL_FILL_DEFAULT);
        }
    }
    MoveTo(5, 195);
    printf("Press any key to return\n");
    keypress();
    endgraph();
    DoText();

    /* Restaurer l'option de rendu et les drapeaux après l'attente clavier. */
    framePolyOnly = old_frame;
    for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = backup_flags[i];
    free(backup_flags);
}

/* Palettes logicielles et matérielles, effacement SHR et shading par
 * orientation.
 */
segment "color";
/*
 * ----------------------------------------------------------------------------
 *  SetColor
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Écrire directement une entrée de palette matérielle SHR.
 *
 *  Paramètres :
 *    [entrée] palette_num, 0..15 ; color_index, 0..15 ; r/g/b, 0..15.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Écrit le mot $0RGB à $E19E00 + 32*palette + 2*indice. Ne borne pas les
 *    arguments et ne met pas à jour le tableau logiciel palettes.
 *
 * ----------------------------------------------------------------------------
 */
void SetColor(int palette_num, int color_index, int r, int g, int b)
{
    unsigned long offset;
    unsigned int  color_word;


    /* Calculer l'offset en octets et assembler le mot RGB avant l'écriture
     * matérielle.
     */
    offset = (long)palette_num * 32L + (long)color_index * 2L;


    color_word = ((unsigned int)r << 8) |
                 ((unsigned int)g << 4) |
                 (unsigned int)b;


    *((unsigned int *)(0xE19E00L + offset)) = color_word;
}

/*
 * ----------------------------------------------------------------------------
 *  ReadPalette
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Lire seize mots de couleur à partir de l'adresse calculée pour une
 *    palette.
 *
 *  Paramètres :
 *    [entrée] palette_num.
 *
 *  Retour :
 *    GS_Palette copié par valeur ; aucune écriture matérielle.
 *
 *  Fonctionnement :
 *    Attention : PALETTE_BASE est un pointeur unsigned int ; son addition
 *    avec palette_num*32 est donc mise à l'échelle par sizeof(unsigned int).
 *    Le chemin principal appelle cette fonction avec 0, où ce décalage est
 *    nul.
 *
 * ----------------------------------------------------------------------------
 */
GS_Palette ReadPalette(int palette_num)
{
    GS_Palette  pal;
    int         i;
    unsigned int *src;

    /* Employer l'arithmétique de pointeurs existante puis copier les seize
     * entrées.
     */
    src = (unsigned int *)(PALETTE_BASE + (long)palette_num * 32L);
    for (i = 0; i < 16; i++)
        pal.color[i] = src[i];
    return pal;
}

/*
 * ----------------------------------------------------------------------------
 *  applyPalette
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Installer une palette logicielle dans la table matérielle active numéro
 *    zéro.
 *
 *  Paramètres :
 *    [entrée] palette_num, ramené à 0 s'il n'est pas dans [0,15].
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    SetColorTable ; requiert QuickDraw actif et palettes déjà préparées.
 *
 *  Fonctionnement :
 *    Le numéro sélectionne la source logicielle, pas le numéro de table
 *    destination.
 *
 * ----------------------------------------------------------------------------
 */
void applyPalette(int palette_num)
{
    /* Borner la sélection puis recopier les seize mots au format attendu par
     * QuickDraw.
     */
    if (palette_num < 0 || palette_num >= 16) palette_num = 0;
    Word color_table[16];
    for (int color_index = 0; color_index < 16; ++color_index) {
        color_table[color_index] = (Word)palettes[palette_num].color[color_index];
    }
    SetColorTable(0, color_table);
}

/*
 * ----------------------------------------------------------------------------
 *  initPalettes
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Construire les quinze palettes de dégradé et demander la capture de la
 *    palette zéro.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    palettes[1..15] et palette0_loaded=0.
 *
 *  Fonctionnement :
 *    La palette système zéro sera lue ultérieurement après le démarrage
 *    graphique.
 *
 * ----------------------------------------------------------------------------
 */
void initPalettes(void)
{
    /* Invalider la capture système puis remplir chaque entrée des dégradés. */
    palette0_loaded = 0;
    for (int p = 1; p < 16; ++p) {
        for (int i = 0; i < 16; ++i) {
            palettes[p].color[i] = paletteGradientColor(p, i);
        }
    }
}

/*
 * ----------------------------------------------------------------------------
 *  DoColor
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Afficher les seize couleurs disponibles avec leurs indices.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Entrées et sorties implicites :
 *    [entrée] options globales de mode graphique.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    rectangles, textes et motif QuickDraw ; la case noire est encadrée en
 *    blanc pour rester visible.
 *
 *  Fonctionnement :
 *    Le modèle n'est ni modifié ni recalculé.
 *
 * ----------------------------------------------------------------------------
 */
void DoColor() {
        Rect r;
        unsigned char pstr[4];
        SetRect (&r, 0, 1, mode / 320 *10, 11);
        /* Dessiner chaque échantillon, convertir son indice en chaîne Pascal
         * puis avancer le rectangle.
         */
        for (int i = 0; i < 16; i++) {
            SetSolidPenPat(i);
            PaintRect(&r);
            if (i == 0) {
                SetSolidPenPat(COL_WHITE);
                FrameRect(&r);
            }
            MoveTo(r.h1, r.v2+10);

            if (i < 10) {
                pstr[0] = 1;
                pstr[1] = '0' + i;
            } else {
                pstr[0] = 2;
                pstr[1] = '0' + (i / 10);
                pstr[2] = '0' + (i % 10);
            }
            DrawString(pstr);
            OffsetRect(&r, 20, 0);
        }
}

/*
 * ----------------------------------------------------------------------------
 *  screen2Black
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Mettre à zéro les mots SHR parcourus par la boucle d'effacement.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Écrit de $E1:9CFE vers $E1:2002 inclus. Le mot à $E1:2000 n'est pas
 *    atteint : la boucle sort lorsque X devient zéro.
 *
 *  Préconditions :
 *    A et X sur 16 bits. A, X et les indicateurs ne sont pas sauvegardés.
 *
 * ----------------------------------------------------------------------------
 */
void screen2Black() {
    asm
    {
        /* Effacer par mots en décrémentant X de deux ; BNE évite l'itération
         * d'offset zéro.
         */
        lda #0x0000
        ldx #0x7CFE
    loop:
        sta 0xE12000,x
        dex
        dex
        bne loop
    }
}

/*
 * ----------------------------------------------------------------------------
 *  computeOrientationShading
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Calculer une couleur selon l'orientation de chaque normale observateur.
 *
 *  Paramètres :
 *    [entrée] model, plans déjà calculés ; NULL accepté.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    face_shade_color, indice 0..15 arrondi depuis |c|/norme.
 *
 *  Fonctionnement :
 *    Une normale nulle reçoit COL_FILL_DEFAULT. Ce modèle de shading ne
 *    représente pas une source lumineuse positionnée et ne tient pas compte
 *    d'une occultation.
 *
 * ----------------------------------------------------------------------------
 */
static void computeOrientationShading(Model3D* model) {
    if (!model) return;
    FaceArrays3D* faces = &model->faces;
    int face_count = faces->face_count;
    /* Convertir les normales, calculer leur norme et quantifier
     * l'orientation par rapport à Z.
     */
    for (int i = 0; i < face_count; ++i) {
        float a_f = (float)FIXED64_TO_FLOAT(faces->plane_a[i]);
        float b_f = (float)FIXED64_TO_FLOAT(faces->plane_b[i]);
        float c_f = (float)FIXED64_TO_FLOAT(faces->plane_c[i]);
        float mag = sqrtf(a_f * a_f + b_f * b_f + c_f * c_f);
        if (mag <= 0.0f) {
            face_shade_color[i] = COL_FILL_DEFAULT;
        } else {
            float cosz = fabsf(c_f / mag);
            int shade = (int)(cosz * 15.0f + 0.5f);
            if (shade < 0) shade = 0;
            else if (shade > 15) shade = 15;
            face_shade_color[i] = shade;
        }
    }
}

/* Inspecteurs et réparations interactives ; certaines commandes changent la
 * permutation ou les faces.
 */
segment "inspection";
/*
 * ----------------------------------------------------------------------------
 *  inspect_faces_before
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Inspecter les faces placées avant une face choisie dans la permutation.
 *
 *  Paramètres :
 *    [entrée] model et params non NULL ; filename, contexte de l'interface.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    entrées-sorties texte, QuickDraw et buffers temporaires de diagnostic.
 *
 *  Fonctionnement :
 *    Lit un identifiant, filtre les boîtes puis évalue les relations et
 *    chevauchements ; permet de visualiser les anomalies et de déplacer la
 *    cible sur confirmation. Modifie éventuellement sorted_face_indices, pas
 *    la géométrie. Force temporairement le culling puis restaure son état
 *    sur les sorties traitées.
 *
 * ----------------------------------------------------------------------------
 */
void inspect_faces_before(Model3D* model, ObserverParams* params, const char* filename) {
    if (!model || !params) return;
    FaceArrays3D* faces = &model->faces;
    int face_count = faces->face_count;
    if (face_count <= 0) { printf("No faces in model\n"); return; }

    printf("Inspector: inspect faces placed BEFORE a selected face (diagnostics)\n\n");


    /* Conserver l'option avant de demander et valider l'identifiant cible. */
    int old_cull = cull_back_faces;
    cull_back_faces = 1;

    printf("Enter face id (0..%d) to inspect: ", face_count - 1);
    int sel = -1;
    if (scanf("%d", &sel) != 1) {
        int ch; while ((ch = getchar()) != '\n' && ch != EOF) ;
        printf("Input cancelled\n");
        cull_back_faces = old_cull; return;
    }

    {
        int ch; while ((ch = getchar()) != '\n' && ch != EOF) ;
    }

    int target_face = sel;
    if (target_face < 0 || target_face >= face_count) { printf("Invalid face id\n"); cull_back_faces = old_cull; return; }

    /* Localiser la cible dans la permutation courante. */
    int pos = -1;
    for (int i = 0; i < face_count; ++i) {
        if (faces->sorted_face_indices[i] == target_face) { pos = i; break; }
    }
    if (pos < 0) { printf("Selected face is not in sorted list\n"); cull_back_faces = old_cull; return; }

    printf("Selected face %d is at position %d in the ordered list (total faces = %d)\n", target_face, pos, face_count);

    int* misplaced = NULL;
    int misplaced_count = 0;

    misplaced = (int*)malloc(face_count * sizeof(int));
    misplaced_count = 0;
    /* Filtrer les paires et collecter les préférences incompatibles avec la
     * position courante.
     */
    int checked = 0, rel_neg = 0, rel_pos = 0, rel_zero = 0, bbox_skipped = 0;
    for (int i = 0; i < pos; ++i) {
        int f = faces->sorted_face_indices[i];

        if (faces->maxx[f] <= faces->minx[target_face] || faces->maxx[target_face] <= faces->minx[f]
            || faces->maxy[f] <= faces->miny[target_face] || faces->maxy[target_face] <= faces->miny[f]) {
            bbox_skipped++; continue;
        }
        checked++;
        int rel = pair_order_relation(model, f, target_face);
        if (rel == -1) rel_neg++; else if (rel == 1) rel_pos++; else rel_zero++;
        if (rel == 1) {
            misplaced[misplaced_count++] = f;
        }
    }
    printf("%d \"before\" faces (total): checked=%d ; bbox_skipped=%d ; should_be_before(-1): %d ;  should_be_after(+1): %d (misplaced: %d) ; inconclusive: %d\n",
           pos, checked, bbox_skipped, rel_neg, rel_pos, misplaced_count, rel_zero);
    printf("%d misplaced faces relative to face %d : ", misplaced_count, target_face);
    for (int i = 0; i < misplaced_count; ++i) {
        int f = misplaced[i];

        int pos_in_sorted = -1;
        for (int si = 0; si < faces->face_count; ++si) {
            if (faces->sorted_face_indices[si] == f) { pos_in_sorted = si; break; }
        }
        if (pos_in_sorted >= 0) printf("%d (pos=%d)", f, pos_in_sorted + 1);
        else printf("%d (pos=?)", f);
        if (i < misplaced_count - 1) printf(",");
    }

    int overlap_count = 0;
    int *overlaps = NULL;
    /* Séparer les préférences d'ordre incompatibles des paires qui se
     * chevauchent effectivement.
     */
    if (misplaced_count > 0) {
        overlaps = (int*)malloc(sizeof(int) * misplaced_count);
        for (int i = 0; i < misplaced_count; ++i) {
            int f = misplaced[i];
            if (projected_polygons_overlap(model, f, target_face)) overlaps[overlap_count++] = f;
        }
    }
    printf("\n");
    /* Présenter les paires dont le contour projeté confirme le
     * chevauchement.
     */
    if (overlap_count > 0) {
        printf("Overlap with target: %d face(s):", overlap_count);
        for (int i = 0; i < overlap_count; ++i) printf(" %d", overlaps[i]);
        printf("\n");
    } else {
        printf("Overlap with target: none\n");
    }


    printf("\n");
    keypress();

    for (int mi = 0; mi < misplaced_count; ++mi) {
        int f = misplaced[mi];
        /* Exposer chaque test séparément pour distinguer une contradiction
         * d'une absence de conclusion.
         */
        int tests[7];
        evaluate_pair_tests(model, f, target_face, tests);

        char passed[128]; passed[0] = '\0';
        char failed[128]; failed[0] = '\0';
        char incon[128]; incon[0] = '\0';
        int pcount = 0, fcount = 0, icount = 0;
        for (int t = 0; t < 7; ++t) {
            if (tests[t] == 1) {
                if (pcount) strncat(passed, ",", sizeof(passed)-strlen(passed)-1);
                char tmp[8]; snprintf(tmp, sizeof(tmp), "%d", t+1); strncat(passed, tmp, sizeof(passed)-strlen(passed)-1); pcount++;
            } else if (tests[t] == -1) {
                if (fcount) strncat(failed, ",", sizeof(failed)-strlen(failed)-1);
                char tmp[8]; snprintf(tmp, sizeof(tmp), "%d", t+1); strncat(failed, tmp, sizeof(failed)-strlen(failed)-1); fcount++;
            } else {
                if (icount) strncat(incon, ",", sizeof(incon)-strlen(incon)-1);
                char tmp[8]; snprintf(tmp, sizeof(tmp), "%d", t+1); strncat(incon, tmp, sizeof(incon)-strlen(incon)-1); icount++;
            }
        }
        if (pcount == 0) strncpy(passed, "(none)", sizeof(passed));
        if (fcount == 0) strncpy(failed, "(none)", sizeof(failed));
        if (icount == 0) strncpy(incon, "(none)", sizeof(incon));
        char after_list[64]; after_list[0] = '\0';
        char before_list[64]; before_list[0] = '\0';
        char incon_list[64]; incon_list[0] = '\0';
        int after_count = 0, before_count = 0, incon_count = 0;
        for (int t = 0; t < 7; ++t) {
            if (tests[t] == 1) {
                if (after_count) strncat(after_list, ",", sizeof(after_list)-strlen(after_list)-1);
                char tmp[8]; snprintf(tmp, sizeof(tmp), "%d", t+1); strncat(after_list, tmp, sizeof(after_list)-strlen(after_list)-1);
                after_count++;
            } else if (tests[t] == -1) {
                if (before_count) strncat(before_list, ",", sizeof(before_list)-strlen(before_list)-1);
                char tmp[8]; snprintf(tmp, sizeof(tmp), "%d", t+1); strncat(before_list, tmp, sizeof(before_list)-strlen(before_list)-1);
                before_count++;
            } else {
                if (incon_count) strncat(incon_list, ",", sizeof(incon_list)-strlen(incon_list)-1);
                char tmp[8]; snprintf(tmp, sizeof(tmp), "%d", t+1); strncat(incon_list, tmp, sizeof(incon_list)-strlen(incon_list)-1);
                incon_count++;
            }
        }

        if (before_count == 0) strncpy(before_list, "(none)", sizeof(before_list));
        if (after_count == 0) strncpy(after_list, "(none)", sizeof(after_list));
        if (incon_count == 0) strncpy(incon_list, "(none)", sizeof(incon_list));
        printf("Face %d: BEFORE tests: %s; AFTER tests: %s; inconclusive: %s\n", f, before_list, after_list, incon_list);


        if (after_count == 0 && before_count == 0) {
            printf("  Overall: inconclusive (no decisive tests)\n");
        } else {
            if (after_count) printf("Concluded: Face %d (selected) should be AFTER face %d (tests: %s)\n\n", target_face, f, after_list);
            if (before_count) printf("Concluded: Face %d (selected) should be BEFORE face %d (tests: %s)\n\n", target_face, f, before_list);
        }
    }
    printf("\nPress a key to preview model with highlight selected and misplaced faces (if any).\n");
    keypress();
    startgraph(mode);

    /* Sauvegarder la visibilité avant la vue graphique qui met en évidence
     * les faces candidates.
     */
    unsigned char* backup_flags = (unsigned char*)malloc(faces->face_count);
    for (int i = 0; i < faces->face_count; ++i) backup_flags[i] = faces->display_flag[i];

    int old_frame = framePolyOnly;
    framePolyOnly = 1;

    if (jitter) drawPolygons_jitter(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count); else drawPolygons(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count);

    for (int i = 0; i < misplaced_count; ++i) {
        int f = misplaced[i];
        faces->display_flag[f] = 1;
        drawFace(model, f, COL_ORANGE, 0);
    }


    faces->display_flag[target_face] = 1;
    drawFace(model, target_face, COL_LIGHT_GREEN, 1);
    MoveTo(2, 195);
    printf("%d face(s) should be after face %d, %d overlap%s\n", misplaced_count, target_face, overlap_count, overlap_count > 1 ? "s" : "");
    keypress();
    endgraph();
    DoText();

    framePolyOnly = old_frame;
    for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = backup_flags[i];
    free(backup_flags);

    if (misplaced_count > 0) {
        printf("\nPress 'A' to move face %d considering ALL misplaced faces, 'O' for overlaps only, or any other key to skip: ", target_face);
        int key = getkeypress ();

        /* Appliquer une correction uniquement sur demande, en choisissant
         * toutes les candidates ou seulement celles en chevauchement.
         */
        if (key == 'A' || key == 'a' || key == 'O' || key == 'o') {
            int min_pos = face_count;
            int target_to_move_before = -1;
            int use_overlaps = (key == 'O' || key == 'o');


            if (use_overlaps && overlap_count == 0) {
                printf("\nNo overlaps to process.\n");
            } else {

                int count_to_check = use_overlaps ? overlap_count : misplaced_count;
                int *faces_to_check = use_overlaps ? overlaps : misplaced;

                for (int i = 0; i < count_to_check; ++i) {
                    int f = faces_to_check[i];

                    for (int si = 0; si < face_count; ++si) {
                        if (faces->sorted_face_indices[si] == f) {
                            if (si < min_pos) {
                                min_pos = si;
                                target_to_move_before = f;
                            }
                            break;
                        }
                    }
                }

                if (target_to_move_before >= 0 && min_pos < face_count) {
                    printf("\nMoving face %d (currently at pos %d) to position %d (before face %d) - mode: %s...\n",
                           target_face, pos, min_pos, target_to_move_before, use_overlaps ? "overlaps" : "all");


                    move_element_remove_and_insert(faces->sorted_face_indices, face_count, pos, min_pos);

                    printf("Done! Face %d has been moved.\n", target_face);
                    printf("Press any key to continue...\n");
                    keypress();
                } else {
                    printf("\nError: could not determine target position.\n");
                }
            }
        } else {
            printf("\nSkipped.\n");
        }
    }
    /* Libérer les diagnostics et rétablir le culling à la sortie normale. */
    if (overlaps) free(overlaps);
    free(misplaced);
    cull_back_faces = old_cull;
}
/*
 * ----------------------------------------------------------------------------
 *  inspect_faces_after
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Inspecter les faces placées après une face choisie dans la permutation.
 *
 *  Paramètres :
 *    [entrée] model et params non NULL ; filename, contexte de l'interface.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    entrées-sorties texte, QuickDraw et buffers temporaires de diagnostic.
 *
 *  Fonctionnement :
 *    Lit un identifiant, filtre les boîtes puis évalue les relations et
 *    chevauchements ; permet de visualiser les anomalies et de déplacer la
 *    cible sur confirmation. Modifie éventuellement sorted_face_indices, pas
 *    la géométrie. Force temporairement le culling puis restaure son état
 *    sur les sorties traitées.
 *
 * ----------------------------------------------------------------------------
 */
 void inspect_faces_after(Model3D* model, ObserverParams* params, const char* filename) {
    if (!model || !params) return;
    FaceArrays3D* faces = &model->faces;
    int face_count = faces->face_count;
    if (face_count <= 0) { printf("No faces in model\n"); return; }
    /* Conserver l'option avant de demander et valider l'identifiant cible. */
    int old_cull = cull_back_faces;
    cull_back_faces = 1;

    printf("Inspector: inspect faces placed AFTER a selected face (diagnostics)\n\n");
    printf("Enter face id (0..%d) to inspect AFTER-list: ", face_count - 1);
    int sel = -1;
    if (scanf("%d", &sel) != 1) {
        int ch; while ((ch = getchar()) != '\n' && ch != EOF) ;
        printf("Input cancelled\n");
        cull_back_faces = old_cull; return;
    }

    {
        int ch; while ((ch = getchar()) != '\n' && ch != EOF) ;
    }
    int target_face = sel;
    if (target_face < 0 || target_face >= face_count) { printf("Invalid face id\n"); cull_back_faces = old_cull; return; }
    /* Localiser la cible dans la permutation courante. */
    int pos = -1;
    for (int i = 0; i < face_count; ++i) if (faces->sorted_face_indices[i] == target_face) { pos = i; break; }
    if (pos < 0) { printf("Selected face is not in sorted list\n"); cull_back_faces = old_cull; return; }
    int* misplaced = (int*)malloc(face_count * sizeof(int)); int misplaced_count = 0;
    /* Filtrer les paires et collecter les préférences incompatibles avec la
     * position courante.
     */
    int checked = 0, rel_neg = 0, rel_pos = 0, rel_zero = 0, bbox_skipped = 0;
    for (int i = pos + 1; i < face_count; ++i) {
        int f = faces->sorted_face_indices[i];

        if (faces->maxx[f] <= faces->minx[target_face] || faces->maxx[target_face] <= faces->minx[f]
            || faces->maxy[f] <= faces->miny[target_face] || faces->maxy[target_face] <= faces->miny[f]) {
            bbox_skipped++; continue;
        }
        int rel = pair_order_relation(model, f, target_face);
        checked++;
        if (rel == -1) rel_neg++; else if (rel == 1) rel_pos++; else rel_zero++;
        if (rel == -1) {
            misplaced[misplaced_count++] = f;
        }
    }
    printf("%d \"before\" faces (total): checked=%d ; bbox_skipped=%d ; should_be_before(-1): %d ;  should_be_after(+1): %d (misplaced: %d) ; inconclusive: %d\n",
           pos, checked, bbox_skipped, rel_neg, rel_pos, misplaced_count, rel_zero);
    printf("%d misplaced faces relative to face %d : ", misplaced_count, target_face);
    for (int i = 0; i < misplaced_count; ++i) {
        int f = misplaced[i];
        int pos_in_sorted = -1;
        for (int si = 0; si < faces->face_count; ++si) {
            if (faces->sorted_face_indices[si] == f) { pos_in_sorted = si; break; }
        }
        if (pos_in_sorted >= 0) printf("%d (pos=%d)", f, pos_in_sorted + 1);
        else printf("%d (pos=?)", f);
        if (i < misplaced_count - 1) printf(",");
    }
    printf("\n");

    int overlap_count = 0;
    int *overlaps = NULL;
    /* Séparer les préférences d'ordre incompatibles des paires qui se
     * chevauchent effectivement.
     */
    if (misplaced_count > 0) {
        overlaps = (int*)malloc(sizeof(int) * misplaced_count);
        for (int i = 0; i < misplaced_count; ++i) {
            int f = misplaced[i];
            if (projected_polygons_overlap(model, f, target_face)) overlaps[overlap_count++] = f;
        }
    }
    /* Présenter les paires dont le contour projeté confirme le
     * chevauchement.
     */
    if (overlap_count > 0) {
        printf("Overlap with target: %d face(s):", overlap_count);
        for (int i = 0; i < overlap_count; ++i) printf(" %d", overlaps[i]);
        printf("\n");
    } else {
        printf("Overlap with target: none\n");
    }

    printf("\n");
    keypress();
    for (int ai = 0; ai < misplaced_count; ++ai) {
        int f = misplaced[ai];

        if (faces->maxx[f] <= faces->minx[target_face] || faces->maxx[target_face] <= faces->minx[f]
            || faces->maxy[f] <= faces->miny[target_face] || faces->maxy[target_face] <= faces->miny[f]) {
            printf("Face %d: skipped (axis-aligned bbox separation)\n", f);
            continue;
        }
        /* Exposer chaque test séparément pour distinguer une contradiction
         * d'une absence de conclusion.
         */
        int tests[7]; evaluate_pair_tests(model, f, target_face, tests);
        char passed_t[128]; passed_t[0] = '\0';
        char failed_t[128]; failed_t[0] = '\0';
        char incon_t[128]; incon_t[0] = '\0';
        int pcount_t = 0, fcount_t = 0, icount_t = 0;
        for (int t = 0; t < 7; ++t) {
            if (tests[t] == 1) { if (pcount_t) strncat(passed_t, ",", sizeof(passed_t)-strlen(passed_t)-1); char tmp[8]; snprintf(tmp, sizeof(tmp), "%d", t+1); strncat(passed_t, tmp, sizeof(passed_t)-strlen(passed_t)-1); pcount_t++; }
            else if (tests[t] == -1) { if (fcount_t) strncat(failed_t, ",", sizeof(failed_t)-strlen(failed_t)-1); char tmp[8]; snprintf(tmp, sizeof(tmp), "%d", t+1); strncat(failed_t, tmp, sizeof(failed_t)-strlen(failed_t)-1); fcount_t++; }
            else { if (icount_t) strncat(incon_t, ",", sizeof(incon_t)-strlen(incon_t)-1); char tmp[8]; snprintf(tmp, sizeof(tmp), "%d", t+1); strncat(incon_t, tmp, sizeof(incon_t)-strlen(incon_t)-1); icount_t++; }
        }
        if (pcount_t == 0) strncpy(passed_t, "(none)", sizeof(passed_t));
        if (fcount_t == 0) strncpy(failed_t, "(none)", sizeof(failed_t));
        if (icount_t == 0) strncpy(incon_t, "(none)", sizeof(incon_t));
        char after_list_str[64]; after_list_str[0] = '\0';
        char before_list_str[64]; before_list_str[0] = '\0';
        char incon_list_str[64]; incon_list_str[0] = '\0';
        int after_count_t = 0, before_count_t = 0, incon_count_t = 0;
        for (int t = 0; t < 7; ++t) {
            if (tests[t] == 1) { if (after_count_t) strncat(after_list_str, ",", sizeof(after_list_str)-strlen(after_list_str)-1); char tmp[8]; snprintf(tmp, sizeof(tmp), "%d", t+1); strncat(after_list_str, tmp, sizeof(after_list_str)-strlen(after_list_str)-1); after_count_t++; }
            else if (tests[t] == -1) { if (before_count_t) strncat(before_list_str, ",", sizeof(before_list_str)-strlen(before_list_str)-1); char tmp[8]; snprintf(tmp, sizeof(tmp), "%d", t+1); strncat(before_list_str, tmp, sizeof(before_list_str)-strlen(before_list_str)-1); before_count_t++; }
            else { if (incon_count_t) strncat(incon_list_str, ",", sizeof(incon_list_str)-strlen(incon_list_str)-1); char tmp[8]; snprintf(tmp, sizeof(tmp), "%d", t+1); strncat(incon_list_str, tmp, sizeof(incon_list_str)-strlen(incon_list_str)-1); incon_count_t++; }
        }
        if (before_count_t == 0) strncpy(before_list_str, "(none)", sizeof(before_list_str));
        if (after_count_t == 0) strncpy(after_list_str, "(none)", sizeof(after_list_str));
        if (incon_count_t == 0) strncpy(incon_list_str, "(none)", sizeof(incon_list_str));
        printf("Face %d: BEFORE tests: %s; AFTER tests: %s; inconclusive: %s\n", f, before_list_str, after_list_str, incon_list_str);
        if (after_count_t == 0 && before_count_t == 0) {
            printf("  Overall: inconclusive (no decisive tests)\n");
        } else {
            if (before_count_t) printf("Concluded: Face %d should be BEFORE face %d (tests: %s)\n\n", f, target_face, before_list_str);
            if (after_count_t) printf("Concluded: Face %d should be AFTER face %d (tests: %s)\n\n", f, target_face, after_list_str);
        }
    }

    printf("\nPress a key to preview model with highlight selected and misplaced faces (if any).\n");
    keypress();
    startgraph(mode);
    /* Sauvegarder la visibilité avant la vue graphique qui met en évidence
     * les faces candidates.
     */
    unsigned char* backup_flags = (unsigned char*)malloc(faces->face_count);
    for (int i = 0; i < faces->face_count; ++i) backup_flags[i] = faces->display_flag[i];
    int old_frame = framePolyOnly;
    framePolyOnly = 1;

    if (jitter) drawPolygons_jitter(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count); else drawPolygons(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count);
    for (int i = 0; i < misplaced_count; ++i) {
        int f = misplaced[i];
        faces->display_flag[f] = 1;
        drawFace(model, f, COL_PALE_PURPLE, 0);
    }
    faces->display_flag[target_face] = 1;
    drawFace(model, target_face, COL_LIGHT_GREEN, 1);
    MoveTo(2, 195);
    printf("%d face(s) should be before face %d, %d overlap%s\n", misplaced_count, target_face, overlap_count, overlap_count > 1 ? "s" : "");
    keypress();
    endgraph();
    DoText();
    framePolyOnly = old_frame;
    for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = backup_flags[i];
    free(backup_flags);

    if (misplaced_count > 0) {
        printf("\nPress 'A' to move face %d considering ALL misplaced faces, 'O' for overlaps only, or any other key to skip: ", target_face);
        int key = getkeypress ();

        /* Appliquer une correction uniquement sur demande, en choisissant
         * toutes les candidates ou seulement celles en chevauchement.
         */
        if (key == 'A' || key == 'a' || key == 'O' || key == 'o') {
            int max_pos = -1;
            int target_to_move_after = -1;
            int use_overlaps = (key == 'O' || key == 'o');


            if (use_overlaps && overlap_count == 0) {
                printf("\nNo overlaps to process.\n");
            } else {

                int count_to_check = use_overlaps ? overlap_count : misplaced_count;
                int *faces_to_check = use_overlaps ? overlaps : misplaced;

                for (int i = 0; i < count_to_check; ++i) {
                    int f = faces_to_check[i];

                    for (int si = 0; si < face_count; ++si) {
                        if (faces->sorted_face_indices[si] == f) {
                            if (si > max_pos) {
                                max_pos = si;
                                target_to_move_after = f;
                            }
                            break;
                        }
                    }
                }

                if (target_to_move_after >= 0 && max_pos >= 0) {
                    printf("\nMoving face %d (currently at pos %d) to position %d (after face %d) - mode: %s...\n",
                           target_face, pos, max_pos, target_to_move_after, use_overlaps ? "overlaps" : "all");


                    move_element_remove_and_insert(faces->sorted_face_indices, face_count, pos, max_pos);

                    printf("Done! Face %d has been moved.\n", target_face);
                    printf("Press any key to continue...\n");
                    keypress();
                } else {
                    printf("\nError: could not determine target position.\n");
                }
            }
        } else {
            printf("\nSkipped.\n");
        }
    }
    /* Libérer les diagnostics et rétablir le culling à la sortie normale. */
    if (overlaps) free(overlaps);
    free(misplaced);
    cull_back_faces = old_cull;
}
/*
 * ----------------------------------------------------------------------------
 *  showFace
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Explorer une face, sa normale et ses données détaillées dans une vue
 *    interactive.
 *
 *  Paramètres :
 *    [entrée] model et params requis ; filename, contexte.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Les flèches naviguent par identifiant ou position triée ; N bascule la
 *    normale, Espace ouvre les informations et les commandes de masquage,
 *    restauration et inversion du winding. Peut modifier les listes de faces
 *    et leurs plans. Sauvegarde les options de rendu et les drapeaux ;
 *    l'inspection utilise QuickDraw et les entrées-sorties texte. La réserve
 *    initiale de backup_flags n'est pas vérifiée.
 *
 * ----------------------------------------------------------------------------
 */
void showFace(Model3D* model, ObserverParams* params, const char* filename) {
    if (!model || !params) return;
    FaceArrays3D* faces = &model->faces;
    int face_count = faces->face_count;
    if (face_count <= 0) { printf("No faces in model\n"); return; }

    /* Demander et valider la face avant d'ouvrir le parcours graphique. */
    printf("Enter face id (0..%d) to display: ", face_count - 1);
    int sel = -1;
    if (scanf("%d", &sel) != 1) {
        int ch; while ((ch = getchar()) != '\n' && ch != EOF) ;
        printf("Input cancelled\n");
        return;
    }

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


    /* Sauvegarder la visibilité et passer en filaire pour situer la face
     * inspectée.
     */
    unsigned char* backup_flags = (unsigned char*)malloc(faces->face_count);
    for (int i = 0; i < faces->face_count; ++i) backup_flags[i] = faces->display_flag[i];
    int old_frame = framePolyOnly;
    framePolyOnly = 1;
    int show_normal = 0;


    int quit = 0;
    /* Dessiner la vue et traiter navigation, normale et commandes de détail. */
    while (!quit) {
        startgraph(mode);


        if (jitter) drawPolygons_jitter(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count); else drawPolygons(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count);


        if (faces->vertex_count[target_face] > 0) {
            faces->display_flag[target_face] = 1;
            drawFace(model, target_face, COL_LIGHT_GREEN, 0);
        } else if (faces->saved_vertex_count[target_face] > 0) {
            int saved_vc = faces->saved_vertex_count[target_face];
            int saved_df = faces->display_flag[target_face];
            faces->vertex_count[target_face] = saved_vc;
            faces->display_flag[target_face] = 1;
            drawFace(model, target_face, COL_GREY, 0);

            faces->vertex_count[target_face] = 0;
            faces->display_flag[target_face] = saved_df;
        }


        /* Construire une extrémité de normale dans le repère observateur
         * puis projeter ce segment si ses profondeurs le permettent.
         */
        if (show_normal) {
            int offt2 = faces->vertex_indices_ptr[target_face];
            int vn2 = faces->vertex_count[target_face];

            if ((vn2 == 0) && (faces->saved_vertex_count[target_face] > 0)) {vn2 = faces->saved_vertex_count[target_face]; }


            if (vn2 >= 3) {
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
                if (nlen > 0.0001f) {


                    const float NORMAL_VISUAL_LENGTH = 50.0f;
                    float ex = cxo + (na / nlen) * NORMAL_VISUAL_LENGTH;
                    float ey = cyo + (nb / nlen) * NORMAL_VISUAL_LENGTH;
                    float ez = czo + (nc / nlen) * NORMAL_VISUAL_LENGTH;
                    Fixed32 cxo_fx = FLOAT_TO_FIXED(cxo);
                    Fixed32 cyo_fx = FLOAT_TO_FIXED(cyo);
                    Fixed32 czo_fx = FLOAT_TO_FIXED(czo);
                    Fixed32 exo_fx = FLOAT_TO_FIXED(ex);
                    Fixed32 eyo_fx = FLOAT_TO_FIXED(ey);
                    Fixed32 ezo_fx = FLOAT_TO_FIXED(ez);
                    int c_x2d = -1, c_y2d = -1, e_x2d = -1, e_y2d = -1;
                    if (czo_fx > 0) {
                        Fixed32 inv_zo_c = FIXED_DIV_64(s_global_proj_scale_fixed, czo_fx);
                        c_x2d = FIXED_ROUND_TO_INT(FIXED_ADD(FIXED_MUL_64(cxo_fx, inv_zo_c), INT_TO_FIXED(CENTRE_X)));
                        c_y2d = FIXED_ROUND_TO_INT(FIXED_SUB(INT_TO_FIXED(CENTRE_Y), FIXED_MUL_64(cyo_fx, inv_zo_c)));
                    }
                    if (ezo_fx > 0) {
                        Fixed32 inv_zo_e = FIXED_DIV_64(s_global_proj_scale_fixed, ezo_fx);
                        e_x2d = FIXED_ROUND_TO_INT(FIXED_ADD(FIXED_MUL_64(exo_fx, inv_zo_e), INT_TO_FIXED(CENTRE_X)));
                        e_y2d = FIXED_ROUND_TO_INT(FIXED_SUB(INT_TO_FIXED(CENTRE_Y), FIXED_MUL_64(eyo_fx, inv_zo_e)));
                    }

                     {
                        SetSolidPenPat(COL_YELLOW);

                        MoveTo(c_x2d + pan_dx, c_y2d + pan_dy);
                        LineTo(e_x2d + pan_dx, e_y2d + pan_dy);
                    }
                }
            }
        }

        drawFaceIndex(model, target_face);

        /* Retrouver la position de la face pour la navigation dans la
         * permutation, distincte de la navigation par identifiant.
         */
        int pos_in_sorted = -1;
        for (int i = 0; i < face_count; ++i) {
            if (faces->sorted_face_indices[i] == target_face) {
                pos_in_sorted = i;
                break;
            }
        }
        MoveTo(2, 188);

        if (pos_in_sorted >= 0) {
            printf("Face %d (sorted pos %d) [%s]", target_face, pos_in_sorted, (faces->plane_d[target_face] > 0) ? "FRONT" : "BACK");
        } else {
            printf("Face %d [%s]", target_face, (faces->plane_d[target_face] > 0) ? "FRONT" : "BACK");
        }
        if (faces->vertex_count[target_face] == 0) {
            printf(" [HIDDEN]");
        }
        printf("\nArrows: navigate,  N: normal, SPACE: options");


        int key = getkeypress ();

        endgraph();


        /* Changer de face par identifiant ou par position triée selon la
         * flèche reçue.
         */
        if (key == 8) {
            target_face--;
            if (target_face < 0) target_face = face_count - 1;
        } else if (key == 21) {
            target_face++;
            if (target_face >= face_count) target_face = 0;
        } else if (key == 11) {
            if (pos_in_sorted >= 0 && pos_in_sorted < face_count - 1) {
                target_face = faces->sorted_face_indices[pos_in_sorted + 1];
            }
        } else if (key == 10) {
            if (pos_in_sorted > 0) {
                target_face = faces->sorted_face_indices[pos_in_sorted - 1];
            }
        } else if (key == 'N' || key == 'n') {
            /* Basculer la superposition de la normale sans changer le
             * modèle.
             */
            show_normal = !show_normal;
            continue;
        } else if (key == 32) {
            /* Passer au détail textuel : coordonnées, métriques et actions
             * sur la face sélectionnée.
             */

            DoText();
            int vn = faces->vertex_count[target_face];
            int effective_vn = (vn == 0 && faces->saved_vertex_count[target_face] > 0)
                    ? faces->saved_vertex_count[target_face] : vn;
            printf("\n=== Face detail (ID=%d) ===\n\n", target_face);


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
            /* Exporter les informations de la face sans modifier la
             * géométrie.
             */
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

                continue;
            }
            else if (tkey == 'V' || tkey == 'v') {
                /* Inverser les indices et le plan de la face, puis
                 * actualiser la sauvegarde de visibilité de l'inspecteur.
                 */

                reverseFaceVertexOrder(model, target_face);
                backup_flags[target_face] = faces->display_flag[target_face];
                printf("Face %d vertex order reversed.\n\n", target_face);
                printf("Press any key to return to graphics...\n"); fflush(stdout);
                int tmpk = getkeypress ();

                continue;
            } else if (tkey == 'H' || tkey == 'h') {
                /* Masquer la face en conservant sa longueur de liste pour
                 * une restauration ultérieure.
                 */

                hideFace(model, target_face);
                backup_flags[target_face] = faces->display_flag[target_face];
                printf("Face %d hidden.\n\n", target_face);
                printf("Press any key to return to graphics...\n"); fflush(stdout);
                int tmpk = getkeypress ();
                continue;
            } else if (tkey == 'R' || tkey == 'r') {
                /* Restaurer la face sélectionnée et recalculer la vue
                 * nécessaire à son affichage.
                 */


                restoreFace(model, target_face);
                calculateFaceDepths(model, NULL, faces->face_count);
                backup_flags[target_face] = faces->display_flag[target_face];
                printf("Face %d restored.\n\n", target_face);
                printf("Press any key to return to graphics...\n"); fflush(stdout);
                int tmpk = getkeypress ();
                continue;
            } else if (tkey == 'A' || tkey == 'a') {
                /* Restaurer les faces masquées et synchroniser les drapeaux
                 * sauvegardés.
                 */

                restoreAllFaces(model);
                calculateFaceDepths(model, NULL, faces->face_count);
                for (int k2 = 0; k2 < faces->face_count; ++k2) backup_flags[k2] = faces->display_flag[k2];
                printf("All faces restored.\n\n");
                printf("Press any key to return to graphics...\n"); fflush(stdout);
                int tmpk = getkeypress ();
                continue;
            } else {

                continue;
            }
        } else {

            quit = 1;
        }
    }

    DoText();

    /* Restaurer les options et la sauvegarde de visibilité mise à jour par
     * les commandes.
     */
    framePolyOnly = old_frame;
    for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = backup_flags[i];
    free(backup_flags);
}

/*
 * ----------------------------------------------------------------------------
 *  reverseFaceVertexOrder
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Inverser le winding d'une face et son plan observateur.
 *
 *  Paramètres :
 *    [entrée] model et face_idx à base zéro.
 *
 *  Retour :
 *    Aucun ; arguments invalides ignorés.
 *
 *  Sorties et effets de bord :
 *    liste inversée, quatre coefficients opposés, display_flag réévalué.
 *
 *  Fonctionnement :
 *    Invalide les caches d'ordre ; les coordonnées et profondeurs ne
 *    changent pas. Ne retrie pas automatiquement les autres faces.
 *
 * ----------------------------------------------------------------------------
 */
static void reverseFaceVertexOrder(Model3D* model, int face_idx) {
    if (!model || face_idx < 0) return;
    FaceArrays3D* faces = &model->faces;
    if (face_idx >= faces->face_count) return;
    int n = faces->vertex_count[face_idx];
    if (n < 3) return;
    /* Invalider les relations puis inverser la liste d'indices en place. */
    ordering_cache_invalidate(model);
    int off = faces->vertex_indices_ptr[face_idx];
    for (int i = 0; i < n / 2; ++i) {
        int tmp = faces->vertex_indices_buffer[off + i];
        faces->vertex_indices_buffer[off + i] = faces->vertex_indices_buffer[off + n - 1 - i];
        faces->vertex_indices_buffer[off + n - 1 - i] = tmp;
    }


    /* Opposer les quatre coefficients avant de réappliquer profondeur et culling. */
    faces->plane_a[face_idx] = -faces->plane_a[face_idx];
    faces->plane_b[face_idx] = -faces->plane_b[face_idx];
    faces->plane_c[face_idx] = -faces->plane_c[face_idx];
    faces->plane_d[face_idx] = -faces->plane_d[face_idx];


    {
        int behind_camera = (faces->z_min[face_idx] < 0);
        if (behind_camera) {
            faces->display_flag[face_idx] = 0;
        } else if (cull_back_faces && faces->plane_d[face_idx] <= 0) {
            faces->display_flag[face_idx] = 0;
        } else {
            faces->display_flag[face_idx] = 1;
        }
    }


}

/*
 * ----------------------------------------------------------------------------
 *  hideFace
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Masquer une face en conservant sa longueur de liste pour restauration.
 *
 *  Paramètres :
 *    [entrée] model et face_idx valides.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    longueur sauvegardée si positive, vertex_count=0 et display_flag=0.
 *
 *  Fonctionnement :
 *    Invalide les caches d'ordre. Ne supprime ni indices ni sommets ; aucun
 *    contrôle d'arguments ici.
 *
 * ----------------------------------------------------------------------------
 */
void hideFace(Model3D* model, int face_idx) {
    /* Invalider les relations et sauvegarder la longueur avant de
     * neutraliser la face.
     */
    ordering_cache_invalidate(model);
    FaceArrays3D* faces = &model->faces;
    if (faces->vertex_count[face_idx] > 0) {
        faces->saved_vertex_count[face_idx] = faces->vertex_count[face_idx];
    }
    faces->display_flag[face_idx] = 0;
    faces->vertex_count[face_idx] = 0;
}

/*
 * ----------------------------------------------------------------------------
 *  restoreFace
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Restaurer la longueur d'une face précédemment masquée.
 *
 *  Paramètres :
 *    [entrée] model et face_idx valides.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Si sa longueur est zéro et une sauvegarde existe, la restaure puis
 *    efface la sauvegarde. Invalide les caches, mais ne remet pas
 *    display_flag à 1 et ne recalcule pas le plan.
 *
 * ----------------------------------------------------------------------------
 */
void restoreFace(Model3D* model, int face_idx) {
    ordering_cache_invalidate(model);
    FaceArrays3D* faces = &model->faces;


    /* Réactiver uniquement une liste effectivement masquée et sauvegardée. */
    if (faces->vertex_count[face_idx] == 0 && faces->saved_vertex_count[face_idx] > 0) {
        faces->vertex_count[face_idx] = faces->saved_vertex_count[face_idx];
        faces->saved_vertex_count[face_idx] = 0;
    }
}

/*
 * ----------------------------------------------------------------------------
 *  restoreAllFaces
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Restaurer les longueurs de toutes les faces précédemment masquées.
 *
 *  Paramètres :
 *    [entrée-sortie] model valide.
 *
 *  Retour :
 *    Aucun ; caches d'ordre invalidés.
 *
 *  Fonctionnement :
 *    Ne rétablit pas directement les drapeaux d'affichage : la vue doit être
 *    recalculée.
 *
 * ----------------------------------------------------------------------------
 */
void restoreAllFaces(Model3D* model) {
    ordering_cache_invalidate(model);
    FaceArrays3D* faces = &model->faces;
    int i;
    /* Restaurer chaque liste vide possédant une longueur sauvegardée. */
    for (i = 0; i < faces->face_count; ++i) {
        if (faces->vertex_count[i] == 0 && faces->saved_vertex_count[i] > 0) {
            faces->vertex_count[i] = faces->saved_vertex_count[i];
            faces->saved_vertex_count[i] = 0;
        }
    }
}
/*
 * ----------------------------------------------------------------------------
 *  enforce_mandatory_after_relations
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Réappliquer les contraintes « a après b » après une réparation d'ordre.
 *
 *  Paramètres :
 *    [entrée] model ; n ; relation_count et relations[][2].
 *
 *    [entrée-sortie] permutation du modèle et pos_of_face.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Déplace a à la position de b puis reprend depuis la première relation.
 *
 *  Préconditions :
 *    permutation et table inverse cohérentes ; relations compatibles et sans
 *    auto-relation. Aucun watchdog n'empêche une boucle sur contraintes
 *    cycliques.
 *
 * ----------------------------------------------------------------------------
 */
static void enforce_mandatory_after_relations(Model3D* model, int n, int *pos_of_face, int relation_count, const int (*relations)[2]) {
    if (!model || !pos_of_face || !relations) return;
    FaceArrays3D* faces = &model->faces;
    /* Parcourir les contraintes et ignorer leurs identifiants hors limites. */
    for (int i = 0; i < relation_count; ++i) {
        int a = relations[i][0];
        int b = relations[i][1];

        if (a < 0 || a >= n || b < 0 || b >= n) continue;
        int pa = pos_of_face[a];
        int pb = pos_of_face[b];

        /* Déplacer a derrière b, reconstruire les positions affectées et
         * vérifier à nouveau toutes les contraintes.
         */
        if (pa <= pb) {
            int tmp = faces->sorted_face_indices[pa];
            memmove(&faces->sorted_face_indices[pa], &faces->sorted_face_indices[pa + 1], sizeof(int) * (pb - pa));
            faces->sorted_face_indices[pb] = tmp;

            for (int k = pa; k <= pb; ++k) {
                pos_of_face[faces->sorted_face_indices[k]] = k;
            }


            i = -1;
        }
    }
}
/*
 * ----------------------------------------------------------------------------
 *  pair_order_relation
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Fournir une préférence d'ordre par profondeur, bornes puis plans.
 *
 *  Paramètres :
 *    [entrée] model et f1/f2 à base zéro
 *
 *  Retour :
 *    -1 f1 avant, +1 f2 avant, 0 inconclusif.
 *
 *  Sorties et effets de bord :
 *    caches de relations ; ne modifie pas la permutation.
 *
 *  Fonctionnement :
 *    Réinitialise explicitement le cache géométrique. Les séparations X/Y
 *    produisent une préférence déterministe même sans occlusion ; aucun ray
 *    casting ici.
 *
 * ----------------------------------------------------------------------------
 */
static int pair_order_relation(Model3D* model, int f1, int f2) {
    if (!model) return 0;

    /* Réinitialiser les plans préparés puis tester les intervalles de profondeur. */
    (void)geometric_face_relation(NULL, 0, 0);
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    Fixed32 epsilon = FLOAT_TO_FIXED(0.01f);

    if (faces->z_max[f2] <= faces->z_min[f1]) return -1;
    if (faces->z_max[f1] <= faces->z_min[f2]) return 1;


    /* Départager les séparations projetées avant les deux sens des relations
     * de plans.
     */
    int minx1 = faces->minx[f1], maxx1 = faces->maxx[f1], miny1 = faces->miny[f1], maxy1 = faces->maxy[f1];
    int minx2 = faces->minx[f2], maxx2 = faces->maxx[f2], miny2 = faces->miny[f2], maxy2 = faces->maxy[f2];
    if (maxx1 <= minx2) return -1;
    if (maxx2 <= minx1) return 1;
    if (maxy1 <= miny2) return -1;
    if (maxy2 <= miny1) return 1;


    int geo = geometric_face_relation(model, f1, f2);
    if (geo != 0) return geo;


    geo = geometric_face_relation(model, f2, f1);
    if (geo != 0) return -geo;

    return 0;
}
/*
 * ----------------------------------------------------------------------------
 *  check_sort_repair
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Examiner les paires et proposer des réparations minimales de la
 *    permutation.
 *
 *  Paramètres :
 *    [entrée] model et face_count.
 *
 *  Retour :
 *    Nombre de déplacements appliqués, ou 0 si aucun ou si la réserve
 *    initiale échoue.
 *
 *  Sorties et effets de bord :
 *    sorted_face_indices
 *
 *  Fonctionnement :
 *    Filtre visibilité, profondeur et chevauchement puis compare les
 *    distances des deux plans ; mémorise et réapplique les contraintes «
 *    face proche après face loin ». Essaye les plans, les centroïdes SH dans
 *    les deux sens puis le centre de bbox. L'utilisateur peut appliquer,
 *    sauter, interrompre ou passer en mode automatique. Les contraintes
 *    peuvent être cycliques ; leur réapplication n'est pas bornée.
 *
 * ----------------------------------------------------------------------------
 */
int check_sort_repair(Model3D* model, int face_count) {
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    if (face_count <= 0) return 0;
    int n = face_count;
    /* Réserver la table inverse et la liste des contraintes déjà acceptées. */
    int *pos_of_face = (int*)malloc(sizeof(int) * n);
    if (!pos_of_face) return 0;
    for (int i = 0; i < n; ++i) pos_of_face[faces->sorted_face_indices[i]] = i;


    int relation_capacity = n + 16;
    int relation_count = 0;
    int (*after_relations)[2] = (int(*)[2])malloc(sizeof(int) * 2 * relation_capacity);
    if (!after_relations) {
        free(pos_of_face);
        return 0;
    }
    int repairs = 0;
    const char* instructions = "\nWhen graphics is displayed  :\nESC: abort, RETURN: automatic mode (no graphics), N: skip, any other key: continue.\n\n";
    int automatic_mode = 0;
    printf("%s", instructions);
    /* Examiner les paires de faces affichables dont profondeur et projection
     * se recouvrent.
     */
    for (int f1 = 0; f1 < n; ++f1) {
        printf("%d ", f1);
        if (faces->display_flag[f1] == 0) continue;
        if (cull_back_faces && faces->plane_d[f1] <= 0) continue;
        for (int f2 = f1 + 1; f2 < n; ++f2) {
            if (faces->display_flag[f2] == 0) continue;
            if (cull_back_faces && faces->plane_d[f2] <= 0) continue;

            if (faces->z_max[f1] <= faces->z_min[f2] || faces->z_max[f2] <= faces->z_min[f1]) continue;

            int _ix0, _iy0, _ix1, _iy1;
            if (!compute_bbox_intersection(model, f1, f2, &_ix0, &_iy0, &_ix1, &_iy1)) continue;

            if (!projected_polygons_overlap(model, f1, f2)) continue;
            int pos1 = pos_of_face[f1];
            int pos2 = pos_of_face[f2];


            /* Chercher un point de comparaison puis évaluer les deux
             * distances sur le même rayon.
             */
            int cx = 0, cy = 0;
            int point_source = 0;
            int rc = 0;

            {
                int geo = geo_face_order(model, f1, f2);
                if (geo != 0) {


                    rc = (geo == -1) ? 1 : -1;
                    point_source = 4;
                }
            }
            long long a1 = 0, a2 = 0;
            int cx1 = 0, cy1 = 0, cx2 = 0, cy2 = 0;
            int have1 = 0;
            int have2 = 0;

            /* Si les plans ne tranchent pas, essayer successivement les
             * points de clipping et les replis.
             */
            if (rc == 0) {
                have1 = compute_intersection_centroid_ordered_fixed(model, f1, f2, &cx1, &cy1, &a1) && a1 > 0;
                if (have1) {
                    cx = cx1; cy = cy1; point_source = 1;
                    {
                        float _tf1 = 0.0f, _tf2 = 0.0f;
                        if (ray_cast_distances(model, f1, f2, cx, cy, &_tf1, &_tf2)) {
                            if (_tf1 < _tf2) rc = -1; else if (_tf1 > _tf2) rc = 1; else rc = 0;
                        } else rc = 0;
                    }
                }
            }
            if (rc == 0) {
                have2 = compute_intersection_centroid_ordered_fixed(model, f2, f1, &cx2, &cy2, &a2) && a2 > 0;
                if (!have1 && have2) {
                    cx = cx2; cy = cy2; point_source = 2;
                    {
                        float _tf1 = 0.0f, _tf2 = 0.0f;
                        if (ray_cast_distances(model, f1, f2, cx, cy, &_tf1, &_tf2)) {
                            if (_tf1 < _tf2) rc = -1; else if (_tf1 > _tf2) rc = 1; else rc = 0;
                        } else rc = 0;
                    }
                } else if (have1 && have2 && rc == 0) {
                    cx = cx2; cy = cy2; point_source = 2;
                    {
                        float _tf1 = 0.0f, _tf2 = 0.0f;
                        if (ray_cast_distances(model, f1, f2, cx, cy, &_tf1, &_tf2)) {
                            if (_tf1 < _tf2) rc = -1; else if (_tf1 > _tf2) rc = 1; else rc = 0;
                        } else rc = 0;
                    }
                }
            }


            if (rc == 0) {
                int ix0 = _ix0; int ix1 = _ix1; int iy0 = _iy0; int iy1 = _iy1;
                cx = (ix0 + ix1) / 2; cy = (iy0 + iy1) / 2; point_source = 3;
                {
                    float _tf1 = 0.0f, _tf2 = 0.0f;
                    if (ray_cast_distances(model, f1, f2, cx, cy, &_tf1, &_tf2)) {
                        if (_tf1 < _tf2) rc = -1; else if (_tf1 > _tf2) rc = 1; else rc = 0;
                    } else rc = 0;

                }
            }
            if (debug_overlap_subj == f1 && debug_overlap_clip == f2) {
                if (point_source == 1) printf("check_sort_repair: using centroid f1->f2 (%d,%d) area1=%lld\n", cx, cy, a1);
                else if (point_source == 2) printf("check_sort_repair: using centroid f2->f1 (%d,%d) area2=%lld\n", cx, cy, a2);
                else if (point_source == 4) printf("check_sort_repair: using geo_face_order decision (%d,%d)\n", cx, cy);
                else printf("check_sort_repair: using bbox-center (%d,%d)\n", cx, cy);
            }
            if (rc == 0) continue;

            if (rc == -1 || rc == 1) {


                /* Déplacer la face proche seulement si elle est placée trop
                 * tôt dans le rendu arrière-vers-avant.
                 */
                int closer = (rc == -1) ? f1 : f2;
                int other  = (rc == -1) ? f2 : f1;
                int pclos = pos_of_face[closer];
                int pother = pos_of_face[other];
                if (pclos <= pother) {
                    int tmp = faces->sorted_face_indices[pclos];
                    /* Demander une décision graphique tant que le mode
                     * automatique n'a pas été choisi.
                     */
                    if (!automatic_mode) {
                        char msg[160];
                        snprintf(msg, sizeof(msg), "Face %d after face %d ? \n(N=skip, ESC=abort, RET=auto, other=apply)", closer, other);
                        int _k = show_inspect_faces_with_message(model, closer, other, msg, 3, 188);
                        if (_k == 27) {           free(pos_of_face); free(after_relations); return repairs; }
                        if (_k == 'N' || _k == 'n') {

                            if (!automatic_mode) {
                                printf("Check and repair running...\n"); fflush(stdout);
                                printf("\n%s\n", instructions);
                                printf("Face %d\n", f1);
                            }
                            continue;
                        }
                        if (_k == 13 || _k == 10) {
                            automatic_mode = 1;

                            if (!automatic_mode) {
                                printf("Check and repair running...\n"); fflush(stdout);
                                printf("\n%s\n", instructions);
                                printf("Face %d\n", f1);
                            }
                        }
                    }
                    if (pclos < pother) {
                        memmove(&faces->sorted_face_indices[pclos], &faces->sorted_face_indices[pclos+1], sizeof(int) * (pother - pclos));
                        faces->sorted_face_indices[pother] = tmp;
                        for (int k = pclos; k <= pother; ++k) pos_of_face[faces->sorted_face_indices[k]] = k;
                        ++repairs;


                        /* Agrandir si possible la liste des contraintes ;
                         * une réserve insuffisante n'annule pas le
                         * déplacement déjà appliqué.
                         */
                        if (relation_count >= relation_capacity) {
                            int new_cap = relation_capacity * 2;
                            int (*tmp_rel)[2] = (int(*)[2])realloc(after_relations, sizeof(int) * 2 * new_cap);
                            if (tmp_rel) {
                                after_relations = tmp_rel;
                                relation_capacity = new_cap;
                            }
                        }
                        if (relation_count < relation_capacity) {
                            after_relations[relation_count][0] = closer;
                            after_relations[relation_count][1] = other;
                            relation_count++;
                        }
                        /* Réappliquer les décisions précédentes après le déplacement courant. */
                        enforce_mandatory_after_relations(model, n, pos_of_face, relation_count, after_relations);
                    }
                    if (!automatic_mode) {
                        printf("Check and repair running...\n"); fflush(stdout);
                        printf("\n%s\n", instructions);
                    }
                }
            }
        }
    }
    /* Libérer les contraintes et la table inverse sur le chemin normal. */
    free(pos_of_face);
    free(after_relations);
    printf("\ncheck_sort_repair: repairs=%d\n", repairs);
    return repairs;
}
/*
 * ----------------------------------------------------------------------------
 *  check_sort_repair_fast
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Réparer automatiquement l'ordre avec le centre de bbox de région
 *    QuickDraw.
 *
 *  Paramètres :
 *    [entrée] model et face_count.
 *
 *  Retour :
 *    Nombre de déplacements appliqués, ou 0 si aucun ou si la réserve
 *    initiale échoue.
 *
 *  Sorties et effets de bord :
 *    sorted_face_indices
 *
 *  Fonctionnement :
 *    Filtre visibilité, profondeur et chevauchement puis compare les
 *    distances des deux plans ; mémorise et réapplique les contraintes «
 *    face proche après face loin ». Ouvre QuickDraw, ignore les paires sans
 *    centroïde de région utilisable, affiche le bilan et attend une touche
 *    avant de revenir au texte.
 *
 * ----------------------------------------------------------------------------
 */
int check_sort_repair_fast(Model3D* model, int face_count) {
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    if (face_count <= 0) return 0;
    startgraph(mode);

    int n = face_count;
    /* Réserver la table inverse et la liste des contraintes déjà acceptées. */
    int *pos_of_face = (int*)malloc(sizeof(int) * n);
    if (!pos_of_face) {
        endgraph();
        return 0;
    }
    int relation_capacity = n + 16;
    int relation_count = 0;
    int (*after_relations)[2] = (int(*)[2])malloc(sizeof(int) * 2 * relation_capacity);
    if (!after_relations) {
        free(pos_of_face);
        endgraph();
        return 0;
    }
    MoveTo(3,10); printf("Computing repairs...");

    for (int i = 0; i < n; ++i) pos_of_face[faces->sorted_face_indices[i]] = i;
    int repairs = 0;
    /* Examiner les paires de faces affichables dont profondeur et projection
     * se recouvrent.
     */
    for (int f1 = 0; f1 < n; ++f1) {
        if (faces->display_flag[f1] == 0) continue;
        if (cull_back_faces && faces->plane_d[f1] <= 0) continue;
        for (int f2 = f1 + 1; f2 < n; ++f2) {
            if (faces->display_flag[f2] == 0) continue;
            if (cull_back_faces && faces->plane_d[f2] <= 0) continue;

            if (faces->z_max[f1] <= faces->z_min[f2] || faces->z_max[f2] <= faces->z_min[f1]) continue;

            int _ix0, _iy0, _ix1, _iy1;
            if (!compute_bbox_intersection(model, f1, f2, &_ix0, &_iy0, &_ix1, &_iy1)) continue;

            if (!projected_polygons_overlap(model, f1, f2)) continue;

            /* Chercher un point de comparaison puis évaluer les deux
             * distances sur le même rayon.
             */
            int cx = 0, cy = 0;
            long long area2 = 0;
            int got_centroid = 0;
            if (compute_intersection_centroid_ordered_qd_fixed(model, f1, f2, &cx, &cy, &area2) && area2 != 0) {
                got_centroid = 1;
            }
            if (!got_centroid) continue;
            float tf1 = 0.0f, tf2 = 0.0f;
            if (!ray_cast_distances(model, f1, f2, cx, cy, &tf1, &tf2)) continue;
            int rc = 0;
            if (tf1 < tf2) rc = -1;
            else if (tf1 > tf2) rc = 1;
            else rc = 0;
            if (rc == 0) continue;
            /* Déplacer la face proche seulement si elle est placée trop tôt
             * dans le rendu arrière-vers-avant.
             */
            int closer = (rc == -1) ? f1 : f2;
            int other  = (rc == -1) ? f2 : f1;
            int pclos = pos_of_face[closer];
            int pother = pos_of_face[other];
            if (pclos <= pother) {
                int tmp = faces->sorted_face_indices[pclos];
                if (pclos < pother) {
                    memmove(&faces->sorted_face_indices[pclos], &faces->sorted_face_indices[pclos+1], sizeof(int) * (pother - pclos));
                    faces->sorted_face_indices[pother] = tmp;
                    for (int k = pclos; k <= pother; ++k) pos_of_face[faces->sorted_face_indices[k]] = k;
                    ++repairs;

                    if (relation_count >= relation_capacity) {
                        int new_cap = relation_capacity * 2;
                        int (*tmp_rel)[2] = (int(*)[2])realloc(after_relations, sizeof(int) * 2 * new_cap);
                        if (tmp_rel) {
                            after_relations = tmp_rel;
                            relation_capacity = new_cap;
                        }
                    }
                    if (relation_count < relation_capacity) {
                        after_relations[relation_count][0] = closer;
                        after_relations[relation_count][1] = other;
                        relation_count++;
                    }
                    /* Réappliquer les décisions précédentes après le déplacement courant. */
                    enforce_mandatory_after_relations(model, n, pos_of_face, relation_count, after_relations);
                }
            }
        }
    }
    /* Libérer les contraintes et la table inverse sur le chemin normal. */
    free(pos_of_face);
    free(after_relations);
    MoveTo(3,20);
    printf("Repairs done: repairs=%d\n", repairs);
    printf("Press any key to continue...\n");
    keypress();
    endgraph();
    DoText();
    return repairs;
}

/*
 * ----------------------------------------------------------------------------
 *  geo_face_order
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Combiner les prédicats de plans dans les deux sens pour le diagnostic.
 *
 *  Paramètres :
 *    [entrée] model et f1/f2
 *
 *  Retour :
 *    -1 f1 avant, +1 f1 après, 0 indéterminé.
 *
 *  Fonctionnement :
 *    Priorité aux tests before puis after sur f1/f2, avant leur symétrique.
 *    Ne considère ni boîtes, ni profondeur seule, ni rayon ; ne modifie pas
 *    le tri.
 *
 * ----------------------------------------------------------------------------
 */
static int geo_face_order(Model3D* model, int f1, int f2) {
    if (!model) return 0;
    int r = 0;
    /* Appliquer les quatre prédicats dans leur ordre de priorité existant. */
    if (pair_plane_before(model, f1, f2)) return -1;
    if (pair_plane_after(model, f1, f2)) return 1;

    if (pair_plane_before(model, f2, f1)) return 1;
    if (pair_plane_after(model, f2, f1)) return -1;
    return r;
}

/* Résultats de diagnostic d'une paire : les codes 1/2 désignent f1/f2, pas
 * les identifiants numériques des faces.
 */
segment "raytrace";

typedef struct {
    int z_verdict;
    int bbox_ok;
    int bbox_ix0, bbox_iy0, bbox_ix1, bbox_iy1;
    int poly_overlap;
    int geo_verdict;
    int sh_centroid_ok;
    int sh_cx, sh_cy; long long sh_area2;
    int qd_centroid_ok;
    int qd_cx, qd_cy; long long qd_area2;
    int bbox_raycast;
    int sh_raycast;
    int qd_raycast;
    char results_text[16][128];
    int results_count;
} CompareFacesResult;
/*
 * ----------------------------------------------------------------------------
 *  run_raycast_test
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Adapter le résultat d'un rayon à la convention du diagnostic de paire.
 *
 *  Paramètres :
 *    [entrée] model, f1/f2, px/py entiers sans pan.
 *
 *  Retour :
 *    1 si f1 est plus proche, 2 si f2 est plus proche, 0 si indéterminé.
 *
 *  Fonctionnement :
 *    Ce ne sont ni des indices de face ni les signes de
 *    geometric_face_relation. Peut remplir le cache des plans.
 *
 * ----------------------------------------------------------------------------
 */
static int run_raycast_test(Model3D* model, int f1, int f2, int px, int py) {
    float d1 = 0.0f, d2 = 0.0f;
    /* Comparer les distances puis conserver le repli historique vers ray_cast_at. */
    if (ray_cast_distances(model, f1, f2, px, py, &d1, &d2)) {
        if (d1 < d2) return 1; else if (d1 > d2) return 2; else return 0;
    }
    int r = ray_cast_at(model, f1, f2, px, py);
    if (r == -1) return 1;
    if (r == 1) return 2;
    return 0;
}
/*
 * ----------------------------------------------------------------------------
 *  inspect_face_pair_ui
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Piloter l'inspecteur graphique et textuel de deux faces.
 *
 *  Paramètres :
 *    [entrée] model ; NULL ou modèle vide ignoré.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Lit les deux identifiants puis permet navigation, comparaison des
 *    méthodes, export de diagnostic et changements manuels de permutation.
 *    Met temporairement les faces en filaire visibles et restaure options et
 *    drapeaux à chaque tour. Les modifications d'ordre demandées restent
 *    effectives ; pas de fusion de faces.
 *
 * ----------------------------------------------------------------------------
 */
static void inspect_face_pair_ui(Model3D* model) {
    if (!model) return;
    FaceArrays3D* faces = &model->faces;
    int face_count = faces->face_count;
    if (face_count <= 0) { printf("No faces in model\n"); return; }

    printf("Inspector: face-pair full diagnostic.\nUse arrows to navigate, space for option\n\n");

    /* Valider les deux identifiants saisis avant le parcours. */
    printf("Enter face id 1 (0..%d): ", face_count - 1);
    int f1 = -1;
    if (scanf("%d", &f1) != 1) { int ch; while ((ch = getchar()) != '\n' && ch != EOF); printf("Input cancelled\n"); return; }
    { int ch; while ((ch = getchar()) != '\n' && ch != EOF); }
    if (f1 < 0 || f1 >= face_count) { printf("Invalid face id 1\n"); return; }
    printf("Enter face id 2 (0..%d): ", face_count - 1);
    int f2 = -1;
    if (scanf("%d", &f2) != 1) { int ch; while ((ch = getchar()) != '\n' && ch != EOF); printf("Input cancelled\n"); return; }
    { int ch; while ((ch = getchar()) != '\n' && ch != EOF); }
    if (f2 < 0 || f2 >= face_count) { printf("Invalid face id 2\n"); return; }

    /* Sauvegarder l'état, dessiner la paire et calculer les diagnostics de la vue. */
    while (1) {

        /* Réinitialiser le résultat partagé avant chaque paire affichée. */
        static CompareFacesResult r; memset(&r, 0, sizeof(r));

        unsigned char* backup_flags = (unsigned char*)malloc(faces->face_count);
        if (!backup_flags) { printf("Memory allocation failed\n"); return; }
        for (int i = 0; i < faces->face_count; ++i) backup_flags[i] = faces->display_flag[i];
        int old_frame = framePolyOnly;
        startgraph(mode);
        framePolyOnly = 1; for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = 1;
        if (jitter) drawPolygons_jitter(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count);
        else drawPolygons(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count);

        unsigned char saved_f1 = faces->display_flag[f1]; unsigned char saved_f2 = faces->display_flag[f2];
        faces->display_flag[f1] = 1; faces->display_flag[f2] = 1;
        drawFace(model, f1, COL_LIGHT_GREEN, 1); drawFace(model, f2, COL_ORANGE, 1);


        {

            int vertex_indices_buffer_limit = 0;
            for (int _ii = 0; _ii < faces->face_count; ++_ii) {
                int _end = faces->vertex_indices_ptr[_ii] + faces->vertex_count[_ii];
                if (_end > vertex_indices_buffer_limit) vertex_indices_buffer_limit = _end;
            }


            compare_faces_diagnostic(model, f1, f2, 1, &r);
        }

        int screenScale = mode / 320;


        /* Choisir la superposition disponible : bbox de région QuickDraw,
         * boîte commune ou contour issu du clipping SH.
         */
        int draw_mode = 0;
        if (r.qd_centroid_ok && r.poly_overlap) draw_mode = 1;
        else if (r.bbox_ok && r.poly_overlap) draw_mode = 2;
        else if (r.sh_centroid_ok) draw_mode = 3;
        /* Obtenir la bbox de région pour cette paire ; le point associé est
         * un centre de boîte, pas un centroïde de surface.
         */
        if (draw_mode == 1) {
            Rect rr;
            SetSolidPenPat(COL_YELLOW);

            int rb_ix0 = r.bbox_ix0, rb_iy0 = r.bbox_iy0, rb_ix1 = r.bbox_ix1, rb_iy1 = r.bbox_iy1;
            long long rb_area2 = 0;
            if (!compute_intersection_region_bbox(model, f1, f2, &rb_ix0, &rb_iy0, &rb_ix1, &rb_iy1, &rb_area2)) {

            }
            int sc_ix0 = screenScale * (rb_ix0 + pan_dx);
            int sc_ix1 = screenScale * (rb_ix1 + pan_dx);
            int sc_iy0 = screenScale * (rb_iy0 + pan_dy);
            int sc_iy1 = screenScale * (rb_iy1 + pan_dy);
            SetRect(&rr, sc_ix0, sc_iy0, sc_ix1, sc_iy1); FrameRect(&rr);

            int sc_cx = screenScale * (r.qd_cx + pan_dx);
            int sc_cy = screenScale * (r.qd_cy + pan_dy);
            int d = 4 * screenScale; int th = screenScale > 0 ? screenScale : 1;

            PenState ps_cross; GetPenState(&ps_cross);
            SetSolidPenPat(COL_BLACK);
            Rect hr; SetRect(&hr, sc_cx - d, sc_cy - (th/2), sc_cx + d, sc_cy + (th/2) + 1); FrameRect(&hr);
            Rect vr; SetRect(&vr, sc_cx - (th/2), sc_cy - d, sc_cx + (th/2) + 1, sc_cy + d); FrameRect(&vr);
            SetPenState(&ps_cross);
        } else if (draw_mode == 2) {
            Rect rr;
            SetSolidPenPat(COL_WHITE);
            int sc_ix0 = screenScale * (r.bbox_ix0 + pan_dx);
            int sc_ix1 = screenScale * (r.bbox_ix1 + pan_dx);
            int sc_iy0 = screenScale * (r.bbox_iy0 + pan_dy);
            int sc_iy1 = screenScale * (r.bbox_iy1 + pan_dy);
            SetRect(&rr, sc_ix0, sc_iy0, sc_ix1, sc_iy1); FrameRect(&rr);
            int fcx = (r.bbox_ix0 + r.bbox_ix1) / 2; int fcy = (r.bbox_iy0 + r.bbox_iy1) / 2;
            int sc_cx = screenScale * (fcx + pan_dx); int sc_cy = screenScale * (fcy + pan_dy);
            int d = 4 * screenScale; int th = screenScale > 0 ? screenScale : 1;
            Rect hr; SetRect(&hr, sc_cx - d, sc_cy - (th/2), sc_cx + d, sc_cy + (th/2) + 1); FrameRect(&hr);
            Rect vr; SetRect(&vr, sc_cx - (th/2), sc_cy - d, sc_cx + (th/2) + 1, sc_cy + d); FrameRect(&vr);
        } else if (draw_mode == 3) {
            if (debug_clip_fixed_vcount >= 3) {

                int minx = debug_clip_fixed_vx[0], maxx = debug_clip_fixed_vx[0], miny = debug_clip_fixed_vy[0], maxy = debug_clip_fixed_vy[0];
                for (int ii = 1; ii < debug_clip_fixed_vcount; ++ii) {
                    if (debug_clip_fixed_vx[ii] < minx) minx = debug_clip_fixed_vx[ii];
                    if (debug_clip_fixed_vx[ii] > maxx) maxx = debug_clip_fixed_vx[ii];
                    if (debug_clip_fixed_vy[ii] < miny) miny = debug_clip_fixed_vy[ii];
                    if (debug_clip_fixed_vy[ii] > maxy) maxy = debug_clip_fixed_vy[ii];
                }
                Rect rr; SetSolidPenPat(COL_BLUE);
                int sc_ix0 = screenScale * (minx + pan_dx);
                int sc_ix1 = screenScale * (maxx + pan_dx);
                int sc_iy0 = screenScale * (miny + pan_dy);
                int sc_iy1 = screenScale * (maxy + pan_dy);
                SetRect(&rr, sc_ix0, sc_iy0, sc_ix1, sc_iy1); FrameRect(&rr);

                int sc_cx = screenScale * (r.sh_cx + pan_dx);
                int sc_cy = screenScale * (r.sh_cy + pan_dy);
                int d = 4 * screenScale; int th = screenScale > 0 ? screenScale : 1;
                Rect hr; SetRect(&hr, sc_cx - d, sc_cy - (th/2), sc_cx + d, sc_cy + (th/2) + 1); FrameRect(&hr);
                Rect vr; SetRect(&vr, sc_cx - (th/2), sc_cy - d, sc_cx + (th/2) + 1, sc_cy + d); FrameRect(&vr);
            }
        }


        /* Afficher le contour numérique disponible et calculer ses repères
         * de diagnostic.
         */
        if (r.sh_centroid_ok && debug_clip_fixed_vcount >= 3) {
            SetSolidPenPat(COL_FRAME);
            int first_sx = screenScale * (debug_clip_fixed_vx[0] + pan_dx);
            int first_sy = screenScale * (debug_clip_fixed_vy[0] + pan_dy);
            MoveTo(first_sx, first_sy);
            for (int ii = 1; ii < debug_clip_fixed_vcount; ++ii) {
                int sx = screenScale * (debug_clip_fixed_vx[ii] + pan_dx);
                int sy = screenScale * (debug_clip_fixed_vy[ii] + pan_dy);
                LineTo(sx, sy);
            }
            LineTo(first_sx, first_sy);
            SetSolidPenPat(COL_WHITE);
            for (int ii = 0; ii < debug_clip_fixed_vcount; ++ii) {
                int sx = screenScale * (debug_clip_fixed_vx[ii] + pan_dx);
                int sy = screenScale * (debug_clip_fixed_vy[ii] + pan_dy);
                Rect vr2; SetRect(&vr2, sx-1, sy-1, sx+1, sy+1); FrameRect(&vr2);
            }


            {
                double s_cross = 0.0, cx_sum = 0.0, cy_sum = 0.0;
                for (int ii = 0; ii < debug_clip_fixed_vcount; ++ii) {
                    int jj = (ii + 1) % debug_clip_fixed_vcount;
                    double xi = (double)debug_clip_fixed_vx[ii]; double yi = (double)debug_clip_fixed_vy[ii];
                    double xj = (double)debug_clip_fixed_vx[jj]; double yj = (double)debug_clip_fixed_vy[jj];
                    double cross = xi * yj - xj * yi;
                    s_cross += cross;
                    cx_sum += (xi + xj) * cross;
                    cy_sum += (yi + yj) * cross;
                }
                int draw_cx = r.sh_cx; int draw_cy = r.sh_cy;
                if (s_cross != 0.0) {
                    double centroid_fx = cx_sum / (3.0 * s_cross);
                    double centroid_fy = cy_sum / (3.0 * s_cross);
                    draw_cx = (int)(centroid_fx + 0.5);
                    draw_cy = (int)(centroid_fy + 0.5);
                } else {
                    int minx = debug_clip_fixed_vx[0], maxx = debug_clip_fixed_vx[0], miny = debug_clip_fixed_vy[0], maxy = debug_clip_fixed_vy[0];
                    for (int ii = 1; ii < debug_clip_fixed_vcount; ++ii) {
                        if (debug_clip_fixed_vx[ii] < minx) minx = debug_clip_fixed_vx[ii];
                        if (debug_clip_fixed_vx[ii] > maxx) maxx = debug_clip_fixed_vx[ii];
                        if (debug_clip_fixed_vy[ii] < miny) miny = debug_clip_fixed_vy[ii];
                        if (debug_clip_fixed_vy[ii] > maxy) maxy = debug_clip_fixed_vy[ii];
                    }
                    draw_cx = (minx + maxx) / 2;
                    draw_cy = (miny + maxy) / 2;
                }

                if (draw_mode == 3) {
                    int sc_cx = screenScale * (draw_cx + pan_dx);
                    int sc_cy = (draw_cy + pan_dy);
                    int d2 = 4 * screenScale; int th2 = screenScale > 0 ? screenScale : 1;
                    Rect hr2; SetRect(&hr2, sc_cx - d2, sc_cy - (th2/2), sc_cx + d2, sc_cy + (th2/2) + 1); FrameRect(&hr2);
                    Rect vr3; SetRect(&vr3, sc_cx - (th2/2), sc_cy - d2, sc_cx + (th2/2) + 1, sc_cy + d2); FrameRect(&vr3);
                }
            }
        }

        MoveTo(3, 188);
        if (r.poly_overlap) {
            printf("f%d overlaps f%d", f1, f2);

            if (r.qd_raycast == 1) printf(",  f%d in front", f1);
            else if (r.qd_raycast == 2) printf(", f%d in front", f2);
            else if (r.sh_raycast == 1) printf(", f%d in front", f1);
            else if (r.sh_raycast == 2) printf(", f%d in front", f2);
            else if (r.bbox_raycast == 1) printf(", f%d in front", f1);
            else if (r.bbox_raycast == 2) printf(", f%d in front", f2);
        } else {
            printf("f%d do not overlap f%d", f1, f2);
        }
        printf("\nArrows: nav. SPACE: details, ESC: exit");

        int key = getkeypress ();

        /* Quitter en restaurant les drapeaux et options ; les flèches
         * suivantes sélectionnent une autre paire.
         */
        if (key == 27) {
            framePolyOnly = old_frame;
            for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = backup_flags[i];
            free(backup_flags);
            endgraph(); DoText(); return;
        } else if (key == 8) {
            f2 = (f2 - 1 + face_count) % face_count;


            if (face_count > 1 && f2 == f1) f2 = (f1 - 1 + face_count) % face_count;
        } else if (key == 21) {
            f2 = (f2 + 1) % face_count;
            if (face_count > 1 && f2 == f1) f2 = (f2 + 1) % face_count;
        } else if (key == 11) {
            f1 = (f1 + 1) % face_count;
            if (face_count > 1 && f1 == f2) f1 = (f1 + 1) % face_count;
        } else if (key == 10) {
            f1 = (f1 - 1 + face_count) % face_count;
            if (face_count > 1 && f1 == f2) f1 = (f1 - 1 + face_count) % face_count;
        } else if (key == 'F' || key == 'f') {
            /* Ajouter le résultat de la paire dans son fichier de
             * diagnostic.
             */
            dump_compare_results_to_file(&r, f1, f2);
        } else if (key == 32) {
            /* Ouvrir le détail textuel des verdicts et des distances
             * calculées aux différents points.
             */
            int details_qx0 = 0, details_qy0 = 0;
            int details_qx1 = 0, details_qy1 = 0;
            int details_have_qd_rect = 0;
            long long details_qarea2 = 0;

            framePolyOnly = old_frame;
            for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = backup_flags[i];
            free(backup_flags);


        static int zoom_bbox_raycast = 0, zoom_sh_raycast = 0, zoom_qd_raycast = 0;
        {


            CompareFacesResult* r_zoom = (CompareFacesResult*)malloc(sizeof(CompareFacesResult));
            RCHZoomPatch* patch = (RCHZoomPatch*)malloc(sizeof(RCHZoomPatch));
            /* Libérer les réserves temporaires préparées pour l'inspection
             * avant d'enchaîner les actions textuelles.
             */
            if (r_zoom && patch) {
                memset(r_zoom, 0, sizeof(*r_zoom));


                Fixed32 test_scale = FIXED_MUL_64(s_global_proj_scale_fixed, FLOAT_TO_FIXED(10.0f));
                rch_patch_pair_at_scale(model, f1, f2, test_scale, patch);
                compare_faces_diagnostic(model, f1, f2, 1, r_zoom);
                rch_unpatch_pair(model, patch, f1, f2);


                zoom_bbox_raycast = r_zoom->bbox_raycast;
                zoom_sh_raycast   = r_zoom->sh_raycast;
                zoom_qd_raycast   = r_zoom->qd_raycast;
            } else {

                zoom_bbox_raycast = zoom_sh_raycast = zoom_qd_raycast = 0;
            }
            if (r_zoom) free(r_zoom);
            if (patch) free(patch);
        }


            /* Présenter le point et les distances du chemin QuickDraw avec
             * ses conventions historiques.
             */
            if (r.qd_centroid_ok) {
                details_have_qd_rect = compute_intersection_region_bbox(
                    model, f1, f2, &details_qx0, &details_qy0,
                    &details_qx1, &details_qy1, &details_qarea2);
            }
            endgraph();
            DoText();


            printf("                =========== Face pair: f%d vs f%d ===========               \n", f1, f2);


            {
                int pos1=-1,pos2=-1;
                for(int ii=0;ii<faces->face_count;++ii){
                    int fid=faces->sorted_face_indices[ii];
                    if(fid==f1) pos1=ii;
                    if(fid==f2) pos2=ii;
                    if(pos1>=0 && pos2>=0) break;
                }
                if(pos1>=0 && pos2>=0){
                    if(pos1>pos2) printf("Sorted-list: f%d front (pos %d>%d)\n",f1,pos1,pos2);
                    else if(pos2>pos1) printf("Sorted-list: f%d front (pos %d>%d)\n",f2,pos2,pos1);
                    else printf("Sorted-list: tie (pos=%d)\n",pos1);
                } else printf("Sorted-list: unavailable\n");
            }


            char geo_str[16];
            if (r.geo_verdict == 1) sprintf(geo_str, "f%d", f1);
            else if (r.geo_verdict == 2) sprintf(geo_str, "f%d", f2);
            else strcpy(geo_str, "?");
            printf("Tests: Z-range = %s ; Bbox = %s ; 3D = %s ; Poly = %s\n",
                   (r.z_verdict==1?"f1<f2":(r.z_verdict==2?"f2<f1":"?")),
                   (r.bbox_ok?"yes":"no"),
                   geo_str,
                   (r.poly_overlap?"yes":"no"));


            int bbox_face = (r.bbox_raycast == 1) ? f1 : (r.bbox_raycast == 2) ? f2 : -1;
            int sh_face   = (r.sh_raycast   == 1) ? f1 : (r.sh_raycast   == 2) ? f2 : -1;
            int qd_face   = (r.qd_raycast   == 1) ? f1 : (r.qd_raycast   == 2) ? f2 : -1;
            printf("Raycast result: bbox = ");
            if (bbox_face >= 0) printf("f%d front", bbox_face); else printf("?");
            printf(" ; sh = ");
            if (sh_face >= 0) printf("f%d front", sh_face); else printf("?");
            printf(" ; qd = ");
            if (qd_face >= 0) printf("f%d front", qd_face); else printf("?");
            printf("\n");


            int bbox_face_z = (zoom_bbox_raycast == 1) ? f1 : (zoom_bbox_raycast == 2) ? f2 : -1;
            int sh_face_z   = (zoom_sh_raycast   == 1) ? f1 : (zoom_sh_raycast   == 2) ? f2 : -1;
            int qd_face_z   = (zoom_qd_raycast   == 1) ? f1 : (zoom_qd_raycast   == 2) ? f2 : -1;
            printf("Raycast result (zoom x10): bbox = ");
            if (bbox_face_z >= 0) printf("f%d front", bbox_face_z); else printf("?");
            printf(" ; sh = ");
            if (sh_face_z >= 0) printf("f%d front", sh_face_z); else printf("?");
            printf(" ; qd = ");
            if (qd_face_z >= 0) printf("f%d front", qd_face_z); else printf("?");
            printf("\n");
            printf("\n");

            {
                float a1=(float)FIXED64_TO_FLOAT(faces->plane_a[f1]);
                float b1=(float)FIXED64_TO_FLOAT(faces->plane_b[f1]);
                float c1=(float)FIXED64_TO_FLOAT(faces->plane_c[f1]);
                float d1=(float)FIXED64_TO_FLOAT(faces->plane_d[f1]);
                printf("F%d plane: a = %.2f ; b = %.2f ; c = %.2f ; d = %.2f\n",
                       f1,a1,b1,c1,d1);
                printf("Zmin=%.2f ; Zmean=%.2f ; Zmax=%.2f\n",
                       FIXED_TO_FLOAT(faces->z_min[f1]),FIXED_TO_FLOAT(faces->z_mean[f1]),FIXED_TO_FLOAT(faces->z_max[f1]));
            }

            {
                float a2=(float)FIXED64_TO_FLOAT(faces->plane_a[f2]);
                float b2=(float)FIXED64_TO_FLOAT(faces->plane_b[f2]);
                float c2=(float)FIXED64_TO_FLOAT(faces->plane_c[f2]);
                float d2=(float)FIXED64_TO_FLOAT(faces->plane_d[f2]);
                printf("F%d plane: a = %.2f ; b = %.2f ; c = %.2f ; d = %.2f\n",
                       f2,a2,b2,c2,d2);
                printf("Zmin=%.2f ; Zmean=%.2f ; Zmax=%.2f\n",
                       FIXED_TO_FLOAT(faces->z_min[f2]),FIXED_TO_FLOAT(faces->z_mean[f2]),FIXED_TO_FLOAT(faces->z_max[f2]));
            }
            printf("\n");

            /* Comparer les deux plans au centre de la boîte commune, sans
             * assimiler ce point à une intersection certaine des contours.
             */
            if (r.bbox_ok) {
                int bcx = (r.bbox_ix0 + r.bbox_ix1) / 2;
                int bcy = (r.bbox_iy0 + r.bbox_iy1) / 2;
                float bf1 = 0.0f, bf2 = 0.0f;
                if (ray_cast_distances(model, f1, f2, bcx, bcy, &bf1, &bf2)) {
                    printf("1) BBOX OK rect=%d,%d-%d,%d center=%d,%d dist:%d=%.4f %d=%.4f", r.bbox_ix0, r.bbox_iy0, r.bbox_ix1, r.bbox_iy1, bcx, bcy, f1, bf1, f2, bf2);
                } else {
                    printf("1) BBOX OK rect=%d,%d-%d,%d center=%d,%d dist:N/A", r.bbox_ix0, r.bbox_iy0, r.bbox_ix1, r.bbox_iy1, bcx, bcy);
                }

                if (r.bbox_raycast == 1) printf("\n   => face %d is in front", f1);
                else if (r.bbox_raycast == 2) printf("\n   => face %d is in front", f2);
                else printf("\n  -> undetermined");
            } else printf("1) BBOX KO rect=none center=none");

            if (r.sh_centroid_ok && debug_clip_fixed_vcount >= 3) {
                int sminx = debug_clip_fixed_vx[0], smaxx = debug_clip_fixed_vx[0];
                int sminy = debug_clip_fixed_vy[0], smaxy = debug_clip_fixed_vy[0];
                for (int ii = 1; ii < debug_clip_fixed_vcount; ++ii) {
                    if (debug_clip_fixed_vx[ii] < sminx) sminx = debug_clip_fixed_vx[ii];
                    if (debug_clip_fixed_vx[ii] > smaxx) smaxx = debug_clip_fixed_vx[ii];
                    if (debug_clip_fixed_vy[ii] < sminy) sminy = debug_clip_fixed_vy[ii];
                    if (debug_clip_fixed_vy[ii] > smaxy) smaxy = debug_clip_fixed_vy[ii];
                }
                float sf1 = 0.0f, sf2 = 0.0f;
                if (ray_cast_distances(model, f1, f2, r.sh_cx, r.sh_cy, &sf1, &sf2)) {
                    printf("\n2) SH OK rect=%d,%d-%d,%d center=%d,%d dist:%d=%.4f %d=%.4f", sminx, sminy, smaxx, smaxy, r.sh_cx, r.sh_cy, f1, sf1, f2, sf2);
                } else {
                    printf("\n2) SH OK rect=%d,%d-%d,%d c=%d,%d dist:N/A", sminx, sminy, smaxx, smaxy, r.sh_cx, r.sh_cy);
                }
                if (r.sh_raycast == 1) printf("\n   => face %d is in front", f1);
                else if (r.sh_raycast == 2) printf("\n   => face %d is in front", f2);
                else printf("\n  -> undetermined");
            } else printf("\n2) SH KO rect=none");

            if (r.qd_centroid_ok) {
                int qx0 = details_qx0, qy0 = details_qy0;
                int qx1 = details_qx1, qy1 = details_qy1;
                float qf1 = 0.0f, qf2 = 0.0f;
                int have_qd_rect = details_have_qd_rect;
                if (!have_qd_rect && r.bbox_ok) { qx0 = r.bbox_ix0; qy0 = r.bbox_iy0; qx1 = r.bbox_ix1; qy1 = r.bbox_iy1; have_qd_rect = 1; }
                if (have_qd_rect) printf("\n3) QD OK rect=%d,%d-%d,%d center=%d,%d  ", qx0, qy0, qx1, qy1, r.qd_cx, r.qd_cy);
                else printf("\n3) QD KO rect=none center=%d,%d  ", r.qd_cx, r.qd_cy);
                if (ray_cast_distances(model, f1, f2, r.qd_cx, r.qd_cy, &qf1, &qf2)) {
                    printf("dist:%d=%.4f %d=%.4f", f1, qf1, f2, qf2);
                    if (r.qd_raycast == 1) printf("\n   => face %d is in front\n", f1);
                    else if (r.qd_raycast == 2) printf("\n   => face %d is in front\n", f2);
                    else printf("\n   -> undetermined\n");
                } else printf("dist:N/A  -> undetermined\n");
            } else printf("\n3) QD KO rect=none\n");

            printf("\n'R' to move the back face in front\n'E' to move the front face behind\n'G' for advanced geometric details using plane equations\nAny other key to return to graphical inspector...\n");
            char cmd = getkeypress ();
            /* Retrouver les positions actuelles puis déplacer l'une des
             * faces selon l'action explicitement choisie.
             */
            if (cmd == 'R' || cmd == 'r' || cmd == 'E' || cmd == 'e') {
                int pos1 = -1, pos2 = -1;
                for (int ii = 0; ii < faces->face_count; ++ii) {
                    if (faces->sorted_face_indices[ii] == f1) pos1 = ii;
                    if (faces->sorted_face_indices[ii] == f2) pos2 = ii;
                    if (pos1 >= 0 && pos2 >= 0) break;
                }
                if (pos1 >= 0 && pos2 >= 0 && pos1 != pos2) {
                    int minPos = pos1 < pos2 ? pos1 : pos2;
                    int maxPos = pos1 < pos2 ? pos2 : pos1;
                    int movedFace;
                    int targetFace;
                    if (cmd == 'R' || cmd == 'r') {
                        movedFace = faces->sorted_face_indices[minPos];
                        targetFace = faces->sorted_face_indices[maxPos];
                        move_element_remove_and_insert(faces->sorted_face_indices, faces->face_count, minPos, maxPos);
                        printf("Face %d has been placed before face %d in the sorted list.\n", movedFace, targetFace);
                    } else {
                        movedFace = faces->sorted_face_indices[maxPos];
                        targetFace = faces->sorted_face_indices[minPos];
                        move_element_remove_and_insert(faces->sorted_face_indices, faces->face_count, maxPos, minPos);
                        printf("Face %d has been placed after face %d in the sorted list.\n", movedFace, targetFace);
                    }
                } else {
                    printf("Unable to reorder sorted list for f1/f2.\n");
                }
                printf("Press any key to return to graphical inspector...\n");
                keypress();
            }
            /* Ouvrir le détail des tests de plans sur demande. */
            if (cmd == 'G' || cmd == 'g') {
                pair_plane_geometric_tests(model, f1, f2);
            }
                continue;
            }

        framePolyOnly = old_frame;
        for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = backup_flags[i];
        free(backup_flags);
        endgraph();
        DoText();
    }
}
/*
 * ----------------------------------------------------------------------------
 *  dump_compare_results_to_file
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Ajouter le diagnostic textuel d'une paire dans son fichier dédié.
 *
 *  Paramètres :
 *    [entrée] out, résultat rempli ; f1/f2 pour le nom fNVSfM.txt.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Ouvre en ajout ; NULL ou échec d'ouverture est ignoré. Ne recalcule
 *    aucun test.
 *
 * ----------------------------------------------------------------------------
 */
static void dump_compare_results_to_file(const CompareFacesResult *out, int f1, int f2) {
    if (!out) return;
    char fname[64];
    sprintf(fname, "f%dVSf%d.txt", f1, f2);
    /* Ouvrir en ajout et écrire les lignes déjà calculées, puis fermer le fichier. */
    FILE *f = fopen(fname, "a");
    if (!f) return;
    fprintf(f, "=== Face pair: f%d vs f%d ===\n", f1, f2);
    for (int i = 0; i < out->results_count; ++i) {
        fprintf(f, "%s\n", out->results_text[i]);
    }
    fprintf(f, "\n");
    fclose(f);
}

/*
 * ----------------------------------------------------------------------------
 *  compare_faces_diagnostic
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Comparer plusieurs méthodes d'ordre et conserver leurs résultats
 *    séparés.
 *
 *  Paramètres :
 *    [entrée] model, f1/f2 valides et use_qd, disponibilité des opérations
 *    QuickDraw.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    *out réinitialisé
 *
 *    caches et buffers partagés de clipping ; ne corrige pas la permutation.
 *
 *  Fonctionnement :
 *    Produit verdicts, boîtes, centroïdes, aires et temps en ticks. Les
 *    verdicts 1/2 désignent les positions f1/f2 de la paire ; certaines
 *    légendes historiques doivent être confrontées aux valeurs.
 *
 * ----------------------------------------------------------------------------
 */
static void compare_faces_diagnostic(Model3D* model, int f1, int f2, int use_qd, CompareFacesResult *out) {
    if (!model || !out) return;
    FaceArrays3D* faces = &model->faces;
    /* Effacer le diagnostic pour ne pas conserver de résultats de la paire
     * précédente.
     */
    memset(out, 0, sizeof(*out));

    {
        /* Chronométrer et consigner les intervalles de profondeur. */
        long t_z_start = GetTick();
        if (faces->z_max[f2] <= faces->z_min[f1]) {
            out->z_verdict = 1; long t_z_end = GetTick();
            snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Z test: f%d in front of f%d (%ld ticks)", f1, f2, t_z_end - t_z_start);
        } else if (faces->z_max[f1] <= faces->z_min[f2]) {
            out->z_verdict = 2; long t_z_end = GetTick();
            snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Z test: f%d in front of f%d (%ld ticks)", f2, f1, t_z_end - t_z_start);
        } else {
            out->z_verdict = 0; long t_z_end = GetTick();
            snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Z test: Undetermined (%ld ticks)", t_z_end - t_z_start);
        }
    }

    /* Comparer ensuite les boîtes et les différentes méthodes de centroïde
     * et de rayon.
     */
    int ix0=0, iy0=0, ix1=0, iy1=0;
    {
        long t_bbox_start = GetTick();
        int bbox_res = compute_bbox_intersection(model, f1, f2, &ix0, &iy0, &ix1, &iy1);
        long t_bbox_end = GetTick();
        if (bbox_res) {
            out->bbox_ok = 1; out->bbox_ix0 = ix0; out->bbox_iy0 = iy0; out->bbox_ix1 = ix1; out->bbox_iy1 = iy1;
            snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Bbox overlap: yes (rect=%d,%d - %d,%d) (%ld ticks)", ix0, iy0, ix1, iy1, t_bbox_end - t_bbox_start);
        } else {
            out->bbox_ok = 0;
            snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Bbox overlap: no (%ld ticks)", t_bbox_end - t_bbox_start);
        }
    }

    if (out->bbox_ok) {
        /* Chronométrer la méthode de chevauchement configurée,
         * indépendamment des rejets de boîtes.
         */
        long t_poly_start = GetTick();
        int p_overlap = projected_polygons_overlap(model, f1, f2);
        long t_poly_end = GetTick();

        if (p_overlap) {
            out->poly_overlap = 1;
            snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Polygons overlap: yes");
        } else {
            out->poly_overlap = 0;
            snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Polygons overlap: no");
        }
    } else {
        out->poly_overlap = 0; snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Polygons: skipped (bbox non-overlap)");
    }

    {
        /* Consigner le verdict géométrique selon la convention propre au
         * diagnostic.
         */
        long t_geo_start = GetTick();
        int geo = geo_face_order(model, f1, f2);
        long t_geo_end = GetTick();

        if (geo == -1) { out->geo_verdict = 2; snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "3D geometry: f%d in front of f%d (%ld ticks)", f2, f1, t_geo_end - t_geo_start); }
        else if (geo == 1) { out->geo_verdict = 1; snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "3D geometry: f%d in front of f%d (%ld ticks)", f1, f2, t_geo_end - t_geo_start); }
        else { out->geo_verdict = 0; snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "3D geometry: Undetermined (%ld ticks)", t_geo_end - t_geo_start); }
    }

    if (!out->poly_overlap) {
        out->sh_centroid_ok = 0; out->sh_area2 = 0;
        snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Centroid SH: N/A (no polygon overlap)");
        out->qd_centroid_ok = 0; out->qd_area2 = 0;
        if (use_qd) snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Centroid QD: N/A (no polygon overlap)");
        else snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Centroid QD: skipped (QD disabled)");
        snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Bbox raycast: N/A (no polygon overlap)");
        snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Centroid raycast: N/A (no polygon overlap)");
        snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "QD centroid raycast: N/A (no polygon overlap)");
    } else {

        {
            long long area2 = 0;
            /* Calculer le centroïde SH et conserver son aire et son temps
             * séparément des autres méthodes.
             */
            long t_sh_start = GetTick();
            int sh_fallback = 0;
            int sh_ok = compute_intersection_centroid_ordered_fixed_tryboth(model, f1, f2, &out->sh_cx, &out->sh_cy, &area2, &sh_fallback) && (area2 != 0);
            long t_sh_end = GetTick();
            out->sh_centroid_ok = sh_ok ? 1 : 0; out->sh_area2 = area2;
            if (out->sh_centroid_ok) snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Centroid SH: %d/%d (area2=%lld) (%ld ticks)", out->sh_cx, out->sh_cy, (long long)area2, t_sh_end - t_sh_start);
            else {

                if ((clip_last_bailout_subj == f1 && clip_last_bailout_clip == f2) || (clip_last_bailout_subj == f2 && clip_last_bailout_clip == f1)) {
                    snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Centroid SH: undetermined (clipping bail-out) (%ld ticks)", t_sh_end - t_sh_start);

                    clip_last_bailout_subj = -1; clip_last_bailout_clip = -1;
                } else {
                    snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Centroid SH: undetermined (%ld ticks)", t_sh_end - t_sh_start);
                }

                debug_clip_fixed_vcount = 0; debug_clip_raw_area2 = 0; debug_clip_fixed_area = 0.0;
                debug_clip_centroid_x = 0; debug_clip_centroid_y = 0;
            }
        }

        out->qd_centroid_ok = 0;
        if (use_qd) {
            long long qd_a2 = 0;
            /* Interroger la région QuickDraw lorsqu'elle est disponible ;
             * l'interface retourne un centre et une aire de bbox.
             */
            long t_qd_start = GetTick();
            int qd_ok = compute_intersection_centroid_ordered_qd_fixed(model, f1, f2, &out->qd_cx, &out->qd_cy, &qd_a2);
            long t_qd_end = GetTick();

            out->qd_centroid_ok = qd_ok ? 1 : 0; out->qd_area2 = qd_a2;
            if (out->qd_centroid_ok) snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Centroid QD: %d/%d (area2=%lld) (%ld ticks)", out->qd_cx, out->qd_cy, (long long)qd_a2, t_qd_end - t_qd_start);
            else snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Centroid QD: undetermined (%ld ticks)", t_qd_end - t_qd_start);
        } else {
            snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Centroid QD: skipped (QD disabled)");
        }

        if (out->bbox_ok) {
            int bcx = (out->bbox_ix0 + out->bbox_ix1) / 2; int bcy = (out->bbox_iy0 + out->bbox_iy1) / 2;
            /* Comparer les plans au centre de bbox et convertir le résultat
             * en codes de diagnostic 1/2.
             */
            long t_bbox_rc_start = GetTick();
            out->bbox_raycast = run_raycast_test(model, f1, f2, bcx, bcy);
            long t_bbox_rc_end = GetTick();

            if (out->bbox_raycast == 1) snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Bbox raycast: f%d in front of f%d (%ld ticks)", f1, f2, t_bbox_rc_end - t_bbox_rc_start);
            else if (out->bbox_raycast == 2) snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Bbox raycast: f%d in front of f%d (%ld ticks)", f2, f1, t_bbox_rc_end - t_bbox_rc_start);
            else snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Bbox raycast: undetermined (%ld ticks)", t_bbox_rc_end - t_bbox_rc_start);
        } else {
            snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Bbox raycast: skipped (no bbox)");
        }

        if (out->sh_centroid_ok) {
            /* Comparer les mêmes plans au centroïde numérique pour rendre
             * visible un éventuel désaccord de point d'échantillonnage.
             */
            long t_shrc_start = GetTick();
            out->sh_raycast = run_raycast_test(model, f1, f2, out->sh_cx, out->sh_cy);
            long t_shrc_end = GetTick();

            if (out->sh_raycast == 1) snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Centroid raycast: f%d in front of f%d (%ld ticks)", f1, f2, t_shrc_end - t_shrc_start);
            else if (out->sh_raycast == 2) snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Centroid raycast: f%d in front of f%d (%ld ticks)", f2, f1, t_shrc_end - t_shrc_start);
            else snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Centroid raycast: undetermined (%ld ticks)", t_shrc_end - t_shrc_start);
        } else {
            snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Centroid raycast: skipped (no SH centroid)");
        }

        if (use_qd && out->qd_centroid_ok) {
            /* Consigner séparément le rayon passant par le point QuickDraw. */
            long t_qdrc_start = GetTick();
            out->qd_raycast = run_raycast_test(model, f1, f2, out->qd_cx, out->qd_cy);
            long t_qdrc_end = GetTick();

            if (out->qd_raycast == 1) snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "QD centroid raycast: f%d in front of f%d (%ld ticks)", f1, f2, t_qdrc_end - t_qdrc_start);
            else if (out->qd_raycast == 2) snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "QD centroid raycast: f%d in front of f%d (%ld ticks)", f2, f1, t_qdrc_end - t_qdrc_start);
            else snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "QD centroid raycast: undetermined (%ld ticks)", t_qdrc_end - t_qdrc_start);
        } else {
            snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "QD centroid raycast: skipped (QD unavailable)");
        }
    }
}

#define PDBG(...)  if (!pair_plane_before_debug_quiet) printf(__VA_ARGS__)
#define PKEYP()    if (!pair_plane_before_debug_quiet) keypress()
static int pair_plane_before_debug_quiet = 0;

/*
 * ----------------------------------------------------------------------------
 *  pair_plane_geometric_tests
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Afficher les calculs détaillés des quatre tests de plans d'une paire.
 *
 *  Paramètres :
 *    [entrée] model ; f1/f2, ou valeurs négatives pour demander les IDs au
 *    clavier.
 *
 *  Retour :
 *    1 si le premier ou le deuxième test valide le sens courant, 0 sinon.
 *
 *  Fonctionnement :
 *    Affiche aussi des comparaisons numériques et attend une touche.
 *
 *  Préconditions :
 *    modèle valide et identifiants dans les bornes après saisie ; la
 *    fonction ne fournit pas de validation générale des indices.
 *
 * ----------------------------------------------------------------------------
 */
static int pair_plane_geometric_tests(Model3D* model, int f1, int f2) {
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int test1state = 0, test2state = 0, test3state = 0, test4state = 0;
    int c;
    /* Demander les identifiants uniquement si l'appel n'a pas déjà fourni la
     * paire.
     */
    if (f1 < 0 || f2 < 0) {
    printf("PAIR_DEBUG: enter face index f1: ");
    fflush(stdout);
    if (scanf("%d", &f1) != 1) {
        printf("invalid input\n");
        while ((c = getchar()) != '\n' && c != EOF);
        keypress();
        return 0;
    }
    while ((c = getchar()) != '\n' && c != EOF);
    printf("PAIR_DEBUG: enter face index f2: ");
    fflush(stdout);
    if (scanf("%d", &f2) != 1) {
        printf("invalid input\n");

        while ((c = getchar()) != '\n' && c != EOF);
        keypress();
        return 0;
    }
    while ((c = getchar()) != '\n' && c != EOF);
    }
    printf("\n=== Geometric test (using plane equations) comparing %d vs %d ===\n", f1, f2);

    /* Préparer les listes et les deux plans observateur pour les quatre
     * évaluations.
     */
    int n1 = faces->vertex_count[f1];
    int n2 = faces->vertex_count[f2];
    int offset1 = faces->vertex_indices_ptr[f1];
    int offset2 = faces->vertex_indices_ptr[f2];
    int k;
    Fixed64 a1 = faces->plane_a[f1];
    Fixed64 b1 = faces->plane_b[f1];
    Fixed64 c1 = faces->plane_c[f1];
    Fixed64 d1 = faces->plane_d[f1];
    Fixed64 a2 = faces->plane_a[f2];
    Fixed64 b2 = faces->plane_b[f2];
    Fixed64 c2 = faces->plane_c[f2];
    Fixed64 d2 = faces->plane_d[f2];


    Fixed32 epsilon;
    {
        long t0 = GetTick();
        /* Calculer une tolérance de diagnostic liée aux normes des plans et
         * lui imposer son plancher ; ce n'est pas l'epsilon de la méthode de classement.
         */
        double fa1 = FIXED64_TO_FLOAT(a1);
        double fb1 = FIXED64_TO_FLOAT(b1);
        double fc1 = FIXED64_TO_FLOAT(c1);
        double fa2 = FIXED64_TO_FLOAT(a2);
        double fb2 = FIXED64_TO_FLOAT(b2);
        double fc2 = FIXED64_TO_FLOAT(c2);
        double norm1 = sqrt(fa1*fa1 + fb1*fb1 + fc1*fc1);
        double norm2 = sqrt(fa2*fa2 + fb2*fb2 + fc2*fc2);
        double base = (norm1 > norm2) ? norm1 : norm2;
        double epsf = base * 0.001;
        if (epsf < 0.01) epsf = 0.01;
        epsilon = FLOAT_TO_FIXED((float)epsf);
        long t1 = GetTick();
        PDBG("Epsilon (calculated from plane normals) = %f n1=%f n2=%f ticks=%ld\n",
               epsf, norm1, norm2, t1 - t0);
    }
    int obs_side1 = 0;
    int obs_side2 = 0;
    int side;
    int all_same_side;
    int all_opposite_side;


    /* Tester les sommets de f2 du côté observateur du plan de f1. */
    PDBG("\n******** Test #1 : Testing if faces %d is in front of %d ********\n", f2, f1);

    obs_side1 = 0;
    if (d1 > (Fixed64)epsilon) obs_side1 = 1;
    else if (d1 < -(Fixed64)epsilon) obs_side1 = -1;
    else {
        test1state = 0;
        goto skipT1_debug;
    }
    all_same_side = 1;
    int test1_effective_count = 0;
    printf("Observer side (obs_side1) = %d\n", obs_side1);
    printf("If face %d entirely on the same observer-side of %d's plane, then\n", f2, f1);
    printf("test_values for all vertices of face %d have the same sign as obs.side.\n", f2);
    for (k=0; k<n2; k++) {
            int v = faces->vertex_indices_buffer[offset2+k]-1;

            Fixed64 acc = 0;
            acc  = (((Fixed64)a1 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)b1 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)c1 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
            acc += (Fixed64)d1;


            PDBG("Vertex #%d ; test_value = %f ", k, FIXED64_TO_FLOAT(acc));
            if (acc > (Fixed64)epsilon) side = 1;
            else if (acc < -(Fixed64)epsilon) side = -1;
            else
            {
                printf("==> ignored\n");
                continue;
            }
            test1_effective_count++;
            if (obs_side1 == side) {
                printf("==> OK\n");
            }
            if (obs_side1 != side) {


                all_same_side = 0;
                test1state = -1;
                printf("==> KO\n");
                PDBG("Test #1 failed for faces %d and %d\n", f1, f2);
                PKEYP();
                break;
            }
    }

    /* Distinguer l'absence de sommets effectifs, la réussite de l'hypothèse
     * et sa réfutation après application de la tolérance.
     */
    if (test1state != -1) {
        if (test1_effective_count == 0) {
            test1state = 0;
        } else if (all_same_side) {
            test1state = 1;
        } else {
            test1state = -1;
        }
    }
    if (test1state == 1) {
        PDBG("Test #1 passed for Faces %d and %d\n", f1, f2);
        PKEYP();
    } else if (test1state == 0) {
        PDBG("Test #1 inconclusive (all vertices ignored or observer on plane)\n");
        PKEYP();
    }
    skipT1_debug:


    /* Tester les sommets de f1 du côté opposé à l'observateur du plan de f2. */
    PDBG("\n******** Test #2 : Testing if faces %d is behind %d ********\n", f1, f2);

    obs_side2 = 0;
    if (d2 > (Fixed64)epsilon) obs_side2 = 1;
    else if (d2 < -(Fixed64)epsilon) obs_side2 = -1;
    if (obs_side2 == 0) {
        test2state = 0;
        goto skipT2_debug;
    }
    all_opposite_side = 1;
    int test2_effective_count = 0;
    printf("Observer side = %d\n", obs_side2);
    PDBG("If face %d is entirely on the opposite side of %d's plane, then\n", f1, f2);
    PDBG("test_values for all vertices of face %d have opposite sign to obs. side.\n", f1);
    for (k = 0; k < n1; k++) {
        int v = faces->vertex_indices_buffer[offset1+k] - 1;
        Fixed64 acc = 0;
        acc  = (((Fixed64)a2 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
        acc += (((Fixed64)b2 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
        acc += (((Fixed64)c2 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
        acc += (Fixed64)d2;


        PDBG("Vertex #%d ; test_value = %f ", k, FIXED64_TO_FLOAT(acc));

        if (acc > (Fixed64)epsilon) side = 1;
        else if (acc < -(Fixed64)epsilon) side = -1;
        else
        {
            printf("==> ignored\n");
            continue;
        }
        test2_effective_count++;
        if (obs_side2 != side) {
            printf("==> OK\n");
        }
         else {
            printf("==> KO\n");
            all_opposite_side = 0;
            test2state = -1;
            PDBG("Test #2 failed for faces %d and %d\n", f1, f2);
            PKEYP();
            break;
        }
    }

    /* Distinguer l'absence de sommets effectifs, la réussite de l'hypothèse
     * et sa réfutation après application de la tolérance.
     */
    if (test2state != -1) {
        if (test2_effective_count == 0) {
            test2state = 0;
        } else if (all_opposite_side) {
            test2state = 1;
        } else {
            test2state = -1;
        }
    }
    if (test2state == 1) {
        PDBG("Test #2 passed for Faces %d and %d\n", f1, f2);
        PKEYP();
    } else if (test2state == 0) {
        PDBG("Test #2 inconclusive (all vertices ignored or observer on plane)\n");
        PKEYP();
    }
    skipT2_debug:


    /* Tester les sommets de f1 du côté observateur du plan de f2. */
    PDBG("\n******** Test #3 : Testing if faces %d is in front of %d ********\n", f1, f2);
    obs_side1 = 0;
    if (d2 > (Fixed64)epsilon) obs_side1 = 1;
    else if (d2 < -(Fixed64)epsilon) obs_side1 = -1;
    else {
        test3state = 0;
        goto skipT3_debug;
    }
    all_same_side = 1;
    int test3_effective_count = 0;
    printf("Observer side (obs_side1) = %d\n", obs_side1);
    printf("If face %d entirely on the same observer-side of %d's plane, then\n", f1, f2);
    printf("test_values for all vertices of face %d have the same sign as obs.side.\n", f1);
    for (k = 0; k < n1; k++) {
        int v = faces->vertex_indices_buffer[offset1+k] - 1;
        Fixed64 acc = 0;
        acc  = (((Fixed64)a2 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
        acc += (((Fixed64)b2 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
        acc += (((Fixed64)c2 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
        acc += (Fixed64)d2;
        PDBG("Vertex #%d ; test_value = %f ", k, FIXED64_TO_FLOAT(acc));
        if (acc > (Fixed64)epsilon) side = 1;
        else if (acc < -(Fixed64)epsilon) side = -1;
        else {
            printf("==> ignored\n");
            continue;
        }
        test3_effective_count++;
        if (obs_side1 == side) {
            printf("==> OK\n");
        }
        if (obs_side1 != side) {
            printf("==> KO\n");
            all_same_side = 0;
            test3state = -1;
            PDBG("Test #3 failed for faces %d and %d\n", f1, f2);
            PKEYP();
            break;
        }
    }
    /* Distinguer l'absence de sommets effectifs, la réussite de l'hypothèse
     * et sa réfutation après application de la tolérance.
     */
    if (test3state != -1) {
        if (test3_effective_count == 0) {
            test3state = 0;
        } else if (all_same_side) {
            test3state = 1;
        } else {
            test3state = -1;
        }
    }
    if (test3state == 1) {
        PDBG("Test #3 passed for Faces %d and %d\n", f1, f2);
        PKEYP();
    } else if (test3state == 0) {
        PDBG("Test #3 inconclusive (all vertices ignored or observer on plane)\n");
        PKEYP();
    }
    skipT3_debug:


    /* Tester les sommets de f2 du côté opposé à l'observateur du plan de f1. */
    PDBG("\n******** Test #4 : Testing if faces %d is behind %d ********\n", f2, f1);
    obs_side2 = 0;
    if (d1 > (Fixed64)epsilon) obs_side2 = 1;
    else if (d1 < -(Fixed64)epsilon) obs_side2 = -1;
    if (obs_side2 == 0) {
        test4state = 0;
        goto skipT4_debug;
    }
    all_opposite_side = 1;
    int test4_effective_count = 0;
    printf("Observer side = %d\n", obs_side2);
    PDBG("If face %d is entirely on the opposite side of %d's plane,\n", f2, f1);
    PDBG("==> test_values for all vertices of face %d have opposite sign to obs. side.\n", f2);
    for (k = 0; k < n2; k++) {
        int v = faces->vertex_indices_buffer[offset2+k] - 1;
        Fixed64 acc = 0;
        acc  = (((Fixed64)a1 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
        acc += (((Fixed64)b1 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
        acc += (((Fixed64)c1 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
        acc += (Fixed64)d1;
        PDBG("Vertex #%d ; test_value = %f ", k, FIXED64_TO_FLOAT(acc));
        if (acc > (Fixed64)epsilon) side = 1;
        else if (acc < -(Fixed64)epsilon) side = -1;
        else {
            printf("==> ignored\n");
            continue;
        }
        test4_effective_count++;
        if (obs_side2 != side) {
            printf("==> OK\n");
        } else {
            printf("==> KO\n");
            all_opposite_side = 0;
            test4state = -1;
            PDBG("Test #4 failed for faces %d and %d\n", f2, f1);
            PKEYP();
            break;
        }
    }
    /* Distinguer l'absence de sommets effectifs, la réussite de l'hypothèse
     * et sa réfutation après application de la tolérance.
     */
    if (test4state != -1) {
        if (test4_effective_count == 0) {
            test4state = 0;
        } else if (all_opposite_side) {
            test4state = 1;
        } else {
            test4state = -1;
        }
    }
    if (test4state == 1) {
        PDBG("Test #4 passed for Faces %d and %d\n", f2, f1);
        PKEYP();
    } else if (test4state == 0) {
        PDBG("Test #4 inconclusive (all vertices ignored or observer on plane)\n");
        PKEYP();
    }
    skipT4_debug:
    printf("\n**** SUMMARY for faces %d and %d: ****\n", f1, f2);
    /* Afficher les quatre résultats avant de rendre la validation du sens
     * courant.
     */
    if (test1state == 1) printf("Test #1: face %d in front of %d => OK\n", f2, f1);
    else if (test1state == -1) printf("Test #1: failed\n");
    else printf("Test #1: inconclusive\n");
    if (test2state == 1) printf("Test #2: face %d behind %d => OK\n", f1, f2);
    else if (test2state == -1) printf("Test #2: failed\n");
    else printf("Test #2: inconclusive\n");
    if (test3state == 1) printf("Test #3: face %d in front of %d => OK\n", f1, f2);
    else if (test3state == -1) printf("Test #3: failed\n");
    else printf("Test #3: inconclusive\n");
    if (test4state == 1) printf("Test #4: face %d behind %d => OK\n", f2, f1);
    else if (test4state == -1) printf("Test #4: failed\n");
    else printf("Test #4: inconclusive\n");
    keypress();
    /* Rendre seulement la validation des deux premiers tests, pas une
     * relation complète à trois états.
     */
    if (test1state == 1 || test2state == 1) return 1; else return 0;
}
/*
 * ----------------------------------------------------------------------------
 *  show_inspect_faces_with_message
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Afficher deux faces sur le modèle filaire avec un message et attendre
 *    une réponse.
 *
 *  Paramètres :
 *    [entrée] model, f1/f2 valides, msg, screenx/screeny en pixels.
 *
 *  Retour :
 *    Touche pressée sur le chemin normal.
 *
 *  Limites :
 *    les retours pour modèle NULL ou réserve impossible n'ont pas de valeur
 *    définie malgré le type int. Sauvegarde puis restaure options et
 *    drapeaux, libère le backup et revient au texte.
 *
 * ----------------------------------------------------------------------------
 */
static int show_inspect_faces_with_message(Model3D* model, int f1, int f2, const char* msg, int screenx, int screeny) {
    if (!model) return;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    /* Réserver une sauvegarde de visibilité avant de forcer la vue filaire. */
    unsigned char* backup_flags = (unsigned char*)malloc(faces->face_count);
    if (backup_flags == NULL) { printf("Memory allocation failed\n"); return; }
    for (int i = 0; i < faces->face_count; ++i) backup_flags[i] = faces->display_flag[i];
    int old_frame = framePolyOnly;
    startgraph(mode);
    framePolyOnly = 1;
    for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = 1;
    if (jitter) drawPolygons_jitter(model, faces->vertex_count, faces->face_count, vtx->vertex_count);
    else drawPolygons(model, faces->vertex_count, faces->face_count, vtx->vertex_count);
    unsigned char saved_f1 = faces->display_flag[f1]; unsigned char saved_f2 = faces->display_flag[f2];
    faces->display_flag[f1] = 1; faces->display_flag[f2] = 1;
    /* Surligner les deux faces et placer le message de décision. */
    drawFace(model, f1, COL_LIGHT_GREEN, 1);
    drawFace(model, f2, COL_ORANGE, 1);
    faces->display_flag[f1] = saved_f1; faces->display_flag[f2] = saved_f2;
    MoveTo(screenx, screeny);
    printf("%s", msg);

    /* Attendre la réponse puis restaurer l'état avant de la transmettre. */
    int key = getkeypress ();
    framePolyOnly = old_frame;
    for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = backup_flags[i];
    free(backup_flags);
    endgraph(); DoText();
    return key;
}
/*
 * ----------------------------------------------------------------------------
 *  dumpFaceEquationsCSV
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Exporter les plans, profondeurs et coordonnées observateur des faces.
 *
 *  Paramètres :
 *    [entrée] model et csv_filename ; alt_format vrai choisit séparateur
 *    point-virgule et virgule décimale, faux choisit virgule et point
 *    décimal.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Écrase le fichier cible ; n'effectue pas de recalcul de vue. Les IDs de
 *    face sont à base zéro et les indices de sommets restent au format OBJ.
 *
 * ----------------------------------------------------------------------------
 */
void dumpFaceEquationsCSV(Model3D* model, const char* csv_filename, int alt_format) {
    if (model == NULL || csv_filename == NULL) return;
    /* Ouvrir le fichier en écriture et traiter une ouverture impossible. */
    FILE* f = fopen(csv_filename, "w");
    if (!f) {
        printf("Error: cannot open '%s' for writing\n", csv_filename);
        return;
    }
    FaceArrays3D* faces = &model->faces;
    int face_count = faces->face_count;
    char col_sep = alt_format ? ';' : ',';
    int use_comma_decimal = alt_format ? 1 : 0;

    /* Dimensionner les colonnes de sommets selon la plus grande face du modèle. */
    int max_v = 0;
    for (int ii = 0; ii < face_count; ++ii) {
        if (faces->vertex_count[ii] > max_v) max_v = faces->vertex_count[ii];
    }


    fprintf(f, "face"); fprintf(f, "%c", col_sep);
    fprintf(f, "a"); fprintf(f, "%c", col_sep);
    fprintf(f, "b"); fprintf(f, "%c", col_sep);
    fprintf(f, "c"); fprintf(f, "%c", col_sep);
    fprintf(f, "d"); fprintf(f, "%c", col_sep);
    fprintf(f, "z_min"); fprintf(f, "%c", col_sep);
    fprintf(f, "z_mean"); fprintf(f, "%c", col_sep);
    fprintf(f, "z_max"); fprintf(f, "%c", col_sep);
    fprintf(f, "vertex_indices");
    for (int k = 0; k < max_v; ++k) {
        fprintf(f, "%c", col_sep);
        fprintf(f, "v%d_idx", k+1);
        fprintf(f, "%c", col_sep);
        fprintf(f, "v%d_xo", k+1);
        fprintf(f, "%c", col_sep);
        fprintf(f, "v%d_yo", k+1);
        fprintf(f, "%c", col_sep);
        fprintf(f, "v%d_zo", k+1);
    }
    fprintf(f, "\n");
    /* Convertir les métriques et écrire une ligne par face, avec les
     * séparateurs choisis.
     */
    for (int i = 0; i < face_count; ++i) {
        float a = (float)FIXED64_TO_FLOAT(faces->plane_a[i]);
        float b = (float)FIXED64_TO_FLOAT(faces->plane_b[i]);
        float c = (float)FIXED64_TO_FLOAT(faces->plane_c[i]);
        float d = (float)FIXED64_TO_FLOAT(faces->plane_d[i]);
        float zmin = FIXED_TO_FLOAT(faces->z_min[i]);
        float zmean = FIXED_TO_FLOAT(faces->z_mean[i]);
        float zmax = FIXED_TO_FLOAT(faces->z_max[i]);
        char buf[64];

        fprintf(f, "%d", i);
        fprintf(f, "%c", col_sep);

        #define PRINTF_FLOAT(val) do { snprintf(buf, sizeof(buf), "%.6f", (double)(val)); if (use_comma_decimal) { for (char *_p = buf; *_p; ++_p) if (*_p == '.') *_p = ','; } fprintf(f, "%s", buf); } while(0)
        PRINTF_FLOAT(a); fprintf(f, "%c", col_sep);
        PRINTF_FLOAT(b); fprintf(f, "%c", col_sep);
        PRINTF_FLOAT(c); fprintf(f, "%c", col_sep);
        PRINTF_FLOAT(d); fprintf(f, "%c", col_sep);
        PRINTF_FLOAT(zmin); fprintf(f, "%c", col_sep);
        PRINTF_FLOAT(zmean); fprintf(f, "%c", col_sep);
        PRINTF_FLOAT(zmax);

        fprintf(f, "%c\"", col_sep);
        int offset = faces->vertex_indices_ptr[i];
        int n = faces->vertex_count[i];
        for (int j = 0; j < n; ++j) {
            if (j) fputc(' ', f);
            int vid = faces->vertex_indices_buffer[offset + j];
            fprintf(f, "%d", vid);
        }
        fprintf(f, "\"");


        for (int k = 0; k < max_v; ++k) {
            if (k < n) {
                int vid = faces->vertex_indices_buffer[offset + k];
                int vidx = vid - 1;
                if (vidx >= 0) {
                    float xo = FIXED_TO_FLOAT(model->vertices.xo[vidx]);
                    float yo = FIXED_TO_FLOAT(model->vertices.yo[vidx]);
                    float zo = FIXED_TO_FLOAT(model->vertices.zo[vidx]);

                    fprintf(f, "%c%d", col_sep, vid);

                    snprintf(buf, sizeof(buf), "%.6f", (double)xo); if (use_comma_decimal) for (char *_p = buf; *_p; ++_p) if (*_p == '.') *_p = ','; fprintf(f, "%c%s", col_sep, buf);
                    snprintf(buf, sizeof(buf), "%.6f", (double)yo); if (use_comma_decimal) for (char *_p = buf; *_p; ++_p) if (*_p == '.') *_p = ','; fprintf(f, "%c%s", col_sep, buf);
                    snprintf(buf, sizeof(buf), "%.6f", (double)zo); if (use_comma_decimal) for (char *_p = buf; *_p; ++_p) if (*_p == '.') *_p = ','; fprintf(f, "%c%s", col_sep, buf);
                } else {

                    fprintf(f, "%c%d%c%c%c", col_sep, vid, col_sep, col_sep, col_sep);
                }
            } else {

                fprintf(f, "%c%c%c%c", col_sep, col_sep, col_sep, col_sep);
            }
        }
        fprintf(f, "\n");
    }
    /* Fermer le fichier puis annoncer le nombre de faces exportées. */
    fclose(f);
    printf("Wrote face equations to %s (%d faces)\n", csv_filename, face_count);
}
/*
 * ----------------------------------------------------------------------------
 *  dumpFace2DCoordinates
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Exporter les coordonnées écran de chaque sommet de chaque face.
 *
 *  Paramètres :
 *    [entrée] model et txt_filename.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Écrase le fichier ; écrit IDs de faces base zéro et sommets base un,
 *    sans pan appliqué au dessin. Ignore les indices de sommets invalides ;
 *    ne recalcule pas la projection.
 *
 * ----------------------------------------------------------------------------
 */
static void dumpFace2DCoordinates(Model3D* model, const char* txt_filename) {
    if (!model || !txt_filename) return;
    /* Ouvrir la destination en écriture. */
    FILE* f = fopen(txt_filename, "w");
    if (!f) {
        printf("Error: cannot open '%s' for writing\n", txt_filename);
        return;
    }
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int face_count = faces->face_count;
    /* Écrire chaque face puis les coordonnées projetées des indices valides. */
    for (int fi = 0; fi < face_count; ++fi) {
        int n = faces->vertex_count[fi];
        fprintf(f, "face %d (%d verts):\n", fi, n);
        int offset = faces->vertex_indices_ptr[fi];
        for (int j = 0; j < n; ++j) {
            int vid_obj = faces->vertex_indices_buffer[offset + j];
            int vid = vid_obj - 1;
            if (vid < 0 || vid >= vtx->vertex_count) continue;
            fprintf(f, "  %d: %d %d\n", vid_obj, vtx->x2d[vid], vtx->y2d[vid]);
        }
    }
    fclose(f);
    printf("Wrote face 2D coordinates to %s (%d faces)\n", txt_filename, face_count);
}
/*
 * ----------------------------------------------------------------------------
 *  dumpSortedFaceIndices
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Exporter la permutation courante dans l'ordre de dessin de la méthode de classement.
 *
 *  Paramètres :
 *    [entrée] model et txt_filename.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Écrase le fichier ; une ligne par identifiant de face à base zéro, y
 *    compris les faces actuellement masquées.
 *
 * ----------------------------------------------------------------------------
 */
static void dumpSortedFaceIndices(Model3D* model, const char* txt_filename) {
    if (!model || !txt_filename) return;
    /* Ouvrir la destination puis écrire l'en-tête et toute la permutation. */
    FILE* f = fopen(txt_filename, "w");
    if (!f) {
        printf("Error: cannot open '%s' for writing\n", txt_filename);
        return;
    }
    FaceArrays3D* faces = &model->faces;
    int face_count = faces->face_count;
    fprintf(f, "# Sorted face indices in painter draw order (0-based face id); %d faces\n", face_count);
    for (int i = 0; i < face_count; ++i) {
        int face_id = faces->sorted_face_indices[i];
        fprintf(f, "%d\n", face_id);
    }
    fclose(f);
    printf("Wrote sorted face indices to %s (%d faces)\n", txt_filename, face_count);
}

/* Aide textuelle, métadonnées GS/OS et captures SHR brutes. */
segment "utilities";
/*
 * ----------------------------------------------------------------------------
 *  show_help_pager
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Afficher cycliquement les pages d'aide textuelle jusqu'à Échap.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    sortie texte et lecture clavier.
 *
 *  Fonctionnement :
 *    Réserve l'espace de l'en-tête et du pied ; recommence à la première
 *    page après la dernière. Les chaînes d'aide existantes ne sont pas la
 *    référence de cette révision documentaire : certaines sont en retard sur
 *    le dispatch clavier.
 *
 * ----------------------------------------------------------------------------
 */
static void show_help_pager(void) {
    const char* header[] = {
        "===================================",
        "    HELP - Keyboard Controller",
        "===================================",
        ""
    };
    const int header_count = sizeof(header)/sizeof(header[0]);
    /* Définir le contenu textuel historique des commandes. */
    const char* lines[] = {
        "Space: Display model info",
        "A/Z: Decrease/Increase distance",
        "+/-: Adjust projection scale (+/-10%)",
        "K: Edit angles/distance (ENTER may auto-fit)",
        "Arrow Left/Right: Change horizontal angle",
        "Arrow Up/Down: Change vertical angle",
        "W/X: Change screen rotation angle",
        "N: Load new model",
        "*: save SHGR screen as a PIC ($C1) not compressed file",
        "C: Toggle color palette display",
        "G: Cycle palettes",
        "!: Toggle orientation shading",
        "J: Toggle jittered rendering",
        ";: Run check_sort_repair (verify+minimal fix, ESC to abort, RETURN auto next)",
        ".: Run check_sort_repair_fast (faster QD centroid minimal repair)",
        "1: Painter = FAST (simple sort only)",
        "2: Painter = BUBBLE SORT",
        "3: Painter = NEWELL SANCHA (but no face splitting, nor cyclic overlap handling)",
        "4: Painter = GEO (geometry-only). Can be slow for large models",
        "5: Painter = GEOV3 (inspired by R. Dony's book). Can be slow for large models",
        "6: Painter = CORRECT (homebrew implementation)",
        "7: Painter = CORRECTV2 (homebrew implementation V2)",
        "O: Render scanline Z-Buffer (alternative to painter algorithm)",
        "8: Both colors RANDOM mode (quick)",
        "9: Reset colors, palette 0, and shading OFF",
        ">: Choose fill color",
        "<: Choose frame color",
        "P: Toggle frame-only polygons",
        "B: Toggle back-face culling",
        "I: Toggle display of inconclusive pairs",
        "E/e: Pan left",
        "R/r: Pan right",
        "T/t: Pan up",
        "Y/y: Pan down",
        "0: Reset pan",
        "D: Inspect faces BEFORE target",
        "S: Inspect faces AFTER target",
        "V: Show single face (navigate with arrows, show normals, space for details and other actions including hide/restore a face or all faces)",
        "Q: Interactive face-pair inspector (space for details, then R/E/G for more actions)",
        "M: Debug pair_plane_before",
        "L: Label faces with IDs",
        "F: Dump face data (3D, 2D, sort order) to file",
        "H: Display this help message",
        "ESC: Quit program"
    };
    int n = sizeof(lines)/sizeof(lines[0]);
    const int max_lines = 20;
    int content_per_page = max_lines - header_count;
    if (content_per_page <= 0) content_per_page = 1;
    int pos = 0;
    /* Afficher une page puis attendre Échap ou la continuation cyclique. */
    while (pos < n) {

        for (int h = 0; h < header_count; ++h) printf("%s\n", header[h]);

        int end = pos + content_per_page;
        for (int i = pos; i < end && i < n; ++i) printf("%s\n", lines[i]);

        printf("\nPress ESC to quit, any other key to continue...\n");
        char key = getkeypress();
        if (key == 27) return;


        if (end >= n) {
            pos = 0;
        } else {
            pos = end;
        }

        DoText();
        printf("\n");
    }
}
/*
 * ----------------------------------------------------------------------------
 *  DoText
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Revenir à l'affichage texte et envoyer le caractère d'effacement écran.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Appelle shroff et émet le caractère 12 sur stdout. Ne ferme pas
 *    QuickDraw ; l'appelant gère séparément son cycle de vie.
 *
 * ----------------------------------------------------------------------------
 */
void DoText() {
        /* Désactiver SHR avant le contrôle texte d'effacement. */
        shroff();
        putchar((char) 12);
}
/*
 * ----------------------------------------------------------------------------
 *  setProDOSFileType
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Affecter les métadonnées GS/OS d'un fichier existant.
 *
 *  Paramètres :
 *    [entrée] filename, nom tenant dans GSString255 ; fileType et auxType.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Construit FileInfoRecGS et appelle SetFileInfoGS avec accès $C3. Ne
 *    vérifie ni la longueur maximale ni le résultat Toolbox ; aucun contenu
 *    écrit.
 *
 * ----------------------------------------------------------------------------
 */
void setProDOSFileType(const char* filename, int fileType, long auxType) {
    FileInfoRecGS pb;
    GSString255 gsName;
    /* Construire le chemin GS/OS et le bloc des métadonnées avant l'appel système. */
    gsName.length = strlen(filename);
    strncpy((char*)gsName.text, filename, gsName.length);
    pb.pCount = 5;
    pb.pathname = &gsName;
    pb.access = 0xC3;
    pb.fileType = fileType;
    pb.auxType = auxType;
    pb.storageType = 0;
    pb.optionList = 0;
    SetFileInfoGS(&pb);
}
#define SHR_BASE        0xE12000UL
#define SHR_SIZE        32768UL
/*
 * ----------------------------------------------------------------------------
 *  saveSHRAsRawPic
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Écrire la zone SHR brute dans un fichier PIC non compressé.
 *
 *  Paramètres :
 *    [entrée] filename
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Lit SHR_SIZE octets depuis SHR_BASE, écrase le fichier et affecte le
 *    type ProDOS $C1, auxiliaire zéro. Échec d'ouverture affiché ; les
 *    erreurs d'écriture ne sont pas contrôlées ici.
 *
 * ----------------------------------------------------------------------------
 */
void saveSHRAsRawPic(const char *filename)
{
    FILE *out;
    unsigned char *shr;
    unsigned long i;


    /* Ouvrir la destination binaire et pointer la zone vidéo. */
    shr = (volatile unsigned char *)SHR_BASE;
    out = fopen(filename, "wb");
    if (out == NULL) {
        printf("Error: unable to open %s for writing\n", filename);
        return;
    }

    /* Copier les octets sans compression puis affecter les métadonnées du fichier. */
    for (i = 0; i < SHR_SIZE; i++) {
        fputc(shr[i], out);
    }
    fclose(out);

    setProDOSFileType(filename, 0xC1, 0x0000);
    printf("%s saved.\n", filename);
}
/*
 * ----------------------------------------------------------------------------
 *  saveNextScreenshot
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Choisir le premier nom screen000.PIC à screen999.PIC qui ne peut être
 *    ouvert en lecture.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    Appelle saveSHRAsRawPic pour ce nom, ou affiche un échec si les mille
 *    noms sont utilisés. Un échec d'ouverture est traité comme un nom
 *    disponible ; la sélection n'est pas une réservation atomique de
 *    fichier.
 *
 * ----------------------------------------------------------------------------
 */
void saveNextScreenshot(void) {
    char fname[16];
    int idx;
    FILE* test;
    /* Sonder les noms successifs en fermant ceux qui existent. */
    for (idx = 0; idx < 1000; idx++) {
        sprintf(fname, "screen%03d.PIC", idx);
        test = fopen(fname, "rb");
        if (test == NULL) {
            break;
        }
        fclose(test);
    }
    /* Refuser l'écriture si tous les noms sont occupés, sinon déclencher la
     * sauvegarde.
     */
    if (idx >= 1000) {
        printf("Error: no available screen index (000-999 all used)\n");
        return;
    }
    saveSHRAsRawPic(fname);
}
