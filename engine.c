#include "declare.c"
#include "dony.c"
#include "screen_save_restore.c"

segment "model_loading";
// ================================================================================
// 0. MODEL LOADING
// ================================================================================

/*
 * CREATING A NEW 3D MODEL
 * ========================
 * 
 * This function dynamically allocates all structures necessary
 * for a 3D model. Dynamic allocation is crucial on Apple IIGS
 * because the stack is limited and cannot contain large arrays.
 * 
 * ALLOCATION STRATEGY:
 * 1. Main Model3D structure allocation
 * 2. Vertex array allocation (MAX_VERTICES elements)
 * 3. Face array allocation (MAX_FACES elements)
 * 4. On failure: cleanup of previous allocations
 * 
 * ERROR HANDLING:
 * - Check each allocation
 * - Automatic cascade cleanup on partial failure
 * - Return NULL if unable to allocate
 */
Model3D* createModel3D(void) {
    // Step 1: Main structure allocation
    Model3D* model = (Model3D*)malloc(sizeof(Model3D));
    if (model == NULL) {
        return NULL;
    }
    int n = MAX_VERTICES;
    model->vertices.vertex_count = n;

    // Initialize auto-scale metadata
    model->auto_scale = FIXED_ONE;
    model->auto_center_x = 0;
    model->auto_center_y = 0;
    model->auto_center_z = 0;
    model->auto_scaled = 0;
    model->auto_centered = 0;
    model->orig_x = NULL;
    model->orig_y = NULL;
    model->orig_z = NULL;
    /* initialize new 3D bbox pointers */
    model->faces.minx3 = NULL;
    model->faces.maxx3 = NULL;
    model->faces.miny3 = NULL;
    model->faces.maxy3 = NULL;
    model->faces.minz3 = NULL;
    model->faces.maxz3 = NULL;

    /* Auto-fit suggestion defaults */
    model->auto_suggested_distance = 0;
    model->auto_suggested_proj_scale = 0;
    model->auto_proj_scale = 0;
    model->auto_fit_ready = 0;
    model->auto_fit_applied = 0;

    model->coord_buf = NULL;
    model->coord_buf_capacity = 0;
    
    // Step 2: Allocate vertex arrays using malloc (handles bank crossing better)
    // Note: malloc() should handle bank boundaries better than NewHandle()
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
        // Allocation failed, cleanup
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
    
    // Set dummy handles to NULL (not used with malloc)
    model->vertices.xHandle = NULL;
    model->vertices.yHandle = NULL;
    model->vertices.zHandle = NULL;
    model->vertices.xoHandle = NULL;
    model->vertices.yoHandle = NULL;
    model->vertices.zoHandle = NULL;
    model->vertices.x2dHandle = NULL;
    model->vertices.y2dHandle = NULL;
    
    // Step 3: Face array allocation using parallel arrays (like vertices)
    // Each element stored separately to fit 32KB limit per allocation
    int nf = MAX_FACES;
    
    // Allocate vertex count array: nf * 4 bytes = 24KB
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

    // Allocate saved_vertex_count array (hide/restore backup), zero-initialized:
    // 0 means "not currently hidden / no backup" for every face by default.
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
    
    // Allocate SINGLE packed buffer for all vertex indices
    // Estimate: average 3.5 indices per face (mix of triangles and quads)
    // For 6000 faces: ~21KB. We allocate conservatively at 5 per face = 120KB max
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
    
    // Allocate offset array: one offset per face into the packed buffer
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
    
    // Allocate z_min and z_max arrays (min and max depth per face)
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
    // Allocate z_mean array (mean depth per face) - used by painter_bubble_sort
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

    // Allocate plane coefficient arrays (normalized normals + d term)
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

    // Allocate cached 2D bounding boxes arrays
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
    
    // Allocate display_flag array: nf * 4 bytes = 24KB
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
    

    
    // Allocate sorted_face_indices array: nf * 4 bytes = 24KB max
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
    
    // Initialize structure
    model->faces.vertex_countHandle = NULL;
    model->faces.vertex_indicesBufferHandle = NULL;
    model->faces.vertex_indicesPtrHandle = NULL;
    model->faces.z_maxHandle = NULL;
    model->faces.display_flagHandle = NULL;
    model->faces.sorted_face_indicesHandle = NULL;
    model->faces.total_indices = 0;
    

    return model;
}

/**
 * MEMORY CLEANUP - DESTROY MODEL 3D
 * 
 * This function frees all memory allocated by createModel3D().
 * Must be called when the model is no longer needed to prevent memory leaks.
 * 
 * CLEANUP SEQUENCE:
 * 1. Free all vertex arrays (x, y, z, xo, yo, zo, x2d, y2d)
 * 2. Free all face arrays (vertex_count, vertex_indices_buffer, vertex_indices_ptr, z_max, display_flag, sorted_face_indices)
 * 3. Free main Model3D structure
 */

void destroyModel3D(Model3D* model) {
    if (model != NULL) {
        // Free all vertex arrays
        if (model->vertices.x) free(model->vertices.x);
        if (model->vertices.y) free(model->vertices.y);
        if (model->vertices.z) free(model->vertices.z);
        if (model->vertices.xo) free(model->vertices.xo);
        if (model->vertices.yo) free(model->vertices.yo);
        if (model->vertices.zo) free(model->vertices.zo);
        if (model->vertices.x2d) free(model->vertices.x2d);
        if (model->vertices.y2d) free(model->vertices.y2d);
        
        // Free all face arrays (now simplified with packed buffer)
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

        // Free optional buffers and backups
        if (model->orig_x) free(model->orig_x);
        if (model->orig_y) free(model->orig_y);
        if (model->orig_z) free(model->orig_z);
        if (model->coord_buf) free(model->coord_buf);
        
        // Free main structure
        free(model);
    }
}

/**
 * COMPLETE 3D MODEL LOADING
 * ==========================
 * 
 * This function coordinates the complete loading of an OBJ file
 * by successively calling the vertex and face reading functions.
 * 
 * LOADING PIPELINE:
 * 1. Input parameter validation
 * 2. Read vertices from file
 * 3. Read faces from file  
 * 4. Update counters in structure
 * 
 * ERROR HANDLING:
 * - Vertex reading failure: immediate stop
 * - Face reading failure: warning but continue
 *   (vertices-only model remains usable)
 */
int loadModel3D(Model3D* model, const char* filename) {
    // Input parameter validation
    if (model == NULL || filename == NULL) {
        return -1;  // Invalid parameters
    }
    
    // Step 1: Read vertices from OBJ file
    // --- Update global counter for face index verification ---
    readVertices_last_count = model->vertices.vertex_count;
    
    int vcount = readVertices(filename, &model->vertices, MAX_VERTICES, model);
    if (vcount < 0) {
        return -1;  // Critical failure: unable to read vertices
    }
    model->vertices.vertex_count = vcount;
    
    // Step 2: Read faces from OBJ file (using chunked allocation)
    // This function handles reading faces into 2 chunks transparently
    int fcount = readFaces_model(filename, model);
    if (fcount < 0) {
        // Critical failure: unable to read faces
        printf("\nWarning: Unable to read faces\n");
        model->faces.face_count = 0;  // No faces available
    } else {
        model->faces.face_count = fcount;
        /* compute 3D bounding boxes for all faces */
        /* overlap scan is controlled by caller */
    }
    return 0;  // Success: model loaded (with or without faces)
}

/**
 * VERTEX READING FROM OBJ FILE
 * ============================
 * 
 * This function parses an OBJ format file to extract
 * vertices (3D points). It searches for lines starting with "v "
 * and extracts X, Y, Z coordinates.
 * 
 * OBJ FORMAT FOR VERTICES:
 *   v 1.234 5.678 9.012
 *   v -2.5 0.0 3.14159
 * 
 * ALGORITHM:
 * 1. Open file in read mode
 * 2. Read line by line with fgets()
 * 3. Detect "v " lines with character verification
 * 4. Extract coordinates with sscanf()
 * 5. Store in array with bounds checking
 * 6. Progressive display for user feedback
 * 
 * ERROR HANDLING:
 * - File opening verification
 * - Array overflow protection
 * - Coordinate format validation
 */
int readVertices(const char* filename, VertexArrays3D* vtx, int max_vertices, Model3D* owner) {
    FILE *file;
    char line[MAX_LINE_LENGTH];
    int line_number = 1;
    int vertex_count = 0;
    
    // Open file in read mode
    file = fopen(filename, "r");
    if (file == NULL) {
        return -1;  // Return -1 on error
    }
    
    printf("\nReading vertices from file '%s' ", filename);
    
    // Read file line by line
    while (fgets(line, sizeof(line), file) != NULL) {
        if (line[0] == 'v' && line[1] == ' ') {
            if (vertex_count < max_vertices) {
                float x, y, z;  // Temporary reading in float
                if (sscanf(line + 2, "%f %f %f", &x, &y, &z) == 3) {
                    vtx->x[vertex_count] = FLOAT_TO_FIXED(x);
                    // OBJ files often use Z as up; convert to viewer convention by swapping axes
                    vtx->y[vertex_count] = FLOAT_TO_FIXED(z);   // treat OBJ Z as up
                    vtx->z[vertex_count] = FLOAT_TO_FIXED(y);  // rotate -90° around X, corrected orientation
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

    // Close file
    fclose(file);

    // Compute bbox center and save into owner (do not modify vertex coords)
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
        // Apply centering to vertex coordinates (default behavior requested)
        for (int i = 0; i < vertex_count; ++i) {
            vtx->x[i] = FIXED_SUB(vtx->x[i], owner->auto_center_x);
            vtx->y[i] = FIXED_SUB(vtx->y[i], owner->auto_center_y);
            vtx->z[i] = FIXED_SUB(vtx->z[i], owner->auto_center_z);
        }
        owner->auto_centered = 1; // indicate coords were centered

        // --- Brute auto-fit suggestion (Fixed32) ---
        // Compute model-space max dimension (bbox metric) and suggest a distance = k * max_dim
        Fixed32 dx = FIXED_SUB(xmax, xmin);
        Fixed32 dy = FIXED_SUB(ymax, ymin);
        Fixed32 dz = FIXED_SUB(zmax, zmin);
        Fixed32 max_dim = dx;
        if (dy > max_dim) max_dim = dy;
        if (dz > max_dim) max_dim = dz;
        // k = 3 (user-specified)
        owner->auto_suggested_distance = FIXED_MUL_64(max_dim, INT_TO_FIXED(3));

        // Compute projected bounds using default observer angles (30,20,0) to suggest projection scale
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
        // Compute projection scale (pixels per projected unit) to fit viewport with margin m
        float margin = 0.9f;
        float winw = (float)(CENTRE_X * 2);
        float winh = (float)(CENTRE_Y * 2);
        float s = 100.0f; // fallback
        if (pxmax > pxmin && pymax > pymin) {
            float s1 = (winw * margin) / (pxmax - pxmin);
            float s2 = (winh * margin) / (pymax - pymin);
            s = (s1 < s2) ? s1 : s2;
        }
        owner->auto_suggested_proj_scale = FLOAT_TO_FIXED(s);
        owner->auto_fit_ready = 1;
    }

    // printf("\n\nAnalyse terminee. %d lignes lues.\n", line_number - 1);
    return vertex_count;  // Return the number of vertices read
}

// Function to read faces into parallel arrays in FaceArrays3D structure
int readFaces_model(const char* filename, Model3D* model) {
    FILE *file;
    char line[MAX_LINE_LENGTH];
    int line_number = 1;
    int face_count = 0;
    int i;
    
    // Validate model structure
    if (model == NULL || model->faces.vertex_count == NULL) {
        printf("Error: Invalid model structure for readFaces_model\n");
        return -1;
    }
    
    // Open file in read mode
    file = fopen(filename, "r");
    if (file == NULL) {
        printf("Error: Unable to open file '%s' to read faces\n", filename);
        return -1;
    }
    
    printf("\nReading faces from file '%s' ", filename);
    
    int buffer_pos = 0;  // Current position in the packed buffer
    
    // Read file line by line
    while (fgets(line, sizeof(line), file) != NULL) {
        // Check if line starts with "f " (face)
        if (line[0] == 'f' && line[1] == ' ') {
            if (face_count < MAX_FACES) {
                // Initialize face data
                model->faces.vertex_count[face_count] = 0;
                model->faces.display_flag[face_count] = 1;  // Displayable by default
                model->faces.saved_vertex_count[face_count] = 0;  // no hide/restore backup yet
                model->faces.vertex_indices_ptr[face_count] = buffer_pos;  // Store offset to this face's indices
                
                // Parse vertices from this face
                char *ptr = line + 2;  // Start after "f "
                int temp_indices[MAX_FACE_VERTICES];
                int temp_vertex_count = 0;
                int invalid_index_found = 0;
                
                // Parse character by character
                while (*ptr != '\0' && *ptr != '\n' && temp_vertex_count < MAX_FACE_VERTICES) {
                    // Skip spaces and tabs
                    while (*ptr == ' ' || *ptr == '\t') ptr++;
                    
                    if (*ptr == '\0' || *ptr == '\n') break;
                    
                    // Read the number
                    int vertex_index = 0;
                    while (*ptr >= '0' && *ptr <= '9') {
                        vertex_index = vertex_index * 10 + (*ptr - '0');
                        ptr++;
                    }
                    
                    // Skip texture/normal data (after /)
                    while (*ptr != '\0' && *ptr != ' ' && *ptr != '\t' && *ptr != '\n') {
                        ptr++;
                    }
                    
                    // Validate vertex index
                    if (vertex_index >= 1) {
                        if (vertex_index > readVertices_last_count) {
                            // Index out of bounds
                            invalid_index_found = 1;
                        }
                        temp_indices[temp_vertex_count] = vertex_index;
                        temp_vertex_count++;
                    }
                }
                
                // Check for errors
                if (invalid_index_found) {
                    printf("\nERROR: Face at line %d references vertex index > %d vertices\n", 
                           line_number, readVertices_last_count);
                    fclose(file);
                    return -1;
                } else {
                    // Store valid indices into the packed buffer
                    for (i = 0; i < temp_vertex_count; i++) {
                        model->faces.vertex_indices_buffer[buffer_pos++] = temp_indices[i];
                    }
                    model->faces.vertex_count[face_count] = temp_vertex_count;
                    model->faces.total_indices += temp_vertex_count;
                    
                    // Skip faces with zero vertices
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
    
    // Close file
    fclose(file);
    
    model->faces.face_count = face_count;
    
    // Initialize sorted_face_indices with identity mapping (will be sorted later)
    for (i = 0; i < face_count; i++) {
        model->faces.sorted_face_indices[i] = i;
    }
    
    printf("\nReading faces finished : %d faces read.\n", face_count);
    return face_count;
}

/*
 * Return non-zero if any edge of face `fi` crosses the infinite plane of
 * face `fj`.  Crossing is detected when the signed distance of an edge's
 * endpoints to `fj`'s plane have opposite signs.  Zero distance is treated
 * as touching and does not count as crossing.
 */
static void check_intersect(Model3D* model) {
    if (!model) return;
    compute_face_bboxes3D(model);
    FaceArrays3D* faces = &model->faces;
    int fc = faces->face_count;
    /* no logging in check_intersect per request */
    if (fc <= 1) {
        /* no output when insufficient faces */
        keypress();
        return;
    }

    /* let user know the process may take some time */
    printf("check_intersect: scanning %d faces, please wait...\n", fc);

    /* tolerance in fixed-point (16.16) */
    const Fixed32 eps = FLOAT_TO_FIXED(0.02f);

    int count = 0;
    int ip_count = 0; /* interpenetration counter */
    /* we may split faces as we go; if a split occurs we restart the scan from
       the beginning so that newly created pieces are also checked. */
    int changed;
    do {
        changed = 0;
        fc = faces->face_count; /* update after any splits */
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
                /* check overlap on all three axes with epsilon tolerance */
                if (minx_i <= maxx_j + eps && maxx_i >= minx_j - eps &&
                    miny_i <= maxy_j + eps && maxy_i >= miny_j - eps &&
                    minz_i <= maxz_j + eps && maxz_i >= minz_j - eps) {
                    /* compute overlap extents along each axis */
                    Fixed32 ox = (minx_i < minx_j ? minx_j : minx_i);
                    Fixed32 ux = (maxx_i < maxx_j ? maxx_i : maxx_j);
                    Fixed32 oy = (miny_i < miny_j ? miny_j : miny_i);
                    Fixed32 uy = (maxy_i < maxy_j ? maxy_i : maxy_j);
                    Fixed32 oz = (minz_i < minz_j ? minz_j : minz_i);
                    Fixed32 uz = (maxz_i < maxz_j ? maxz_i : maxz_j);
                    float dx = FIXED_TO_FLOAT(ux - ox);
                    float dy = FIXED_TO_FLOAT(uy - oy);
                    float dz = FIXED_TO_FLOAT(uz - oz);
                    /* log raw fixed bbox values for debugging */

                    /* log computed floats regardless of overlap */

                    /* normally ignore pure-touching boxes, but we still
                     * want to run the plane‑crossing check when one dimension
                     * collapses to zero (as happens after centering), because
                     * the face may still pass through the plane of the other.
                     * dz>=0 is already allowed earlier; now accept dy>=0 as well.
                     */
                    if (dx >= 0.0f && dy >= 0.0f) {

                        /* overlap detected; previously printed details */
                        ++count;
                        if (faces_share_vertex_or_edge(faces, i, j)) {
                            /* share info suppressed */

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
                                /* coplanar skip message suppressed */

                            }
                            if (!coplanar) {
                                /* use the stricter interior-crossing predicate for
                                   statistics. boundary contacts (points/edges lying on
                                   the other face) no longer count as interpenetrations.
                                   this avoids the dozens of false positives seen earlier. */
                                int interior = edge_crosses_interior(model, i, j) ||
                                               edge_crosses_interior(model, j, i);
                                if (interior) {
                                    ++ip_count;

                                } else {

                                }
                                /* now perform splitting decisions using the original
                                   crossing-count heuristic (boundary hits may still
                                   justify a cut). */
                                /* compute crossing counts in both directions once */
                            int cross_i_j = count_edge_crossings(model, i, j);
                            int cross_j_i = count_edge_crossings(model, j, i);
                            /* special rule: if exactly two edges of i cross j and none
                               of j cross i, we want to split face i by plane j instead
                               of the usual orientation. */
                            if (cross_i_j == 2 && cross_j_i == 0) {
                                /* two-edge reverse case, no output */
                                if (split_face_by_plane(model, i, j, NULL, NULL, NULL, 0, 0)) {
                                    changed = 1;
                                    break;
                                }
                            } else if (cross_j_i == 2 && cross_i_j == 0) {
                                /* two-edge normal case, no output */
                                if (split_face_by_plane(model, j, i, NULL, NULL, NULL, 0, 0)) {
                                    changed = 1;
                                    break;
                                }
                            } else {
                                /* fall back to standard behaviour */
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
    /* report statistics before exiting */
    printf("Total overlapping pairs: %d\n", count);
    printf("Total interpenetrating pairs: %d\n", ip_count);
    printf("Final face count: %d\n", faces->face_count);

}

/*
 * Return !0 if faces i and j share a vertex or an entire edge.
 * Comparison is done on the original 1-based vertex indices stored in
 * the face buffers.  Sharing just one vertex is sufficient to consider the
 * pair "touching" for the purposes of the interpenetration test.
 */
static int faces_share_vertex_or_edge(FaceArrays3D* faces, int i, int j) {
    int cnti = faces->vertex_count[i];
    int offi = faces->vertex_indices_ptr[i];
    int cntj = faces->vertex_count[j];
    int offj = faces->vertex_indices_ptr[j];
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
 * Return non-zero if any edge of face `ei` actually intersects the interior of
 * face `fj`.  This refines the previous plane-crossing test by computing the
 * intersection point and verifying it lies within the polygon of `fj`.
 */

/* Helper: return !0 if an edge of face `ei` crosses the interior of
 * face `fj` **strictly** (intersection point lies in the polygon interior,
 * not on a boundary).  This is used to decide whether a pair really
 * interpenetrates for the purpose of counting, ignoring mere boundary
 * contacts that do not represent true crossings.
 */
/* forward declaration of helper used by edge_crosses_interior */

static int edge_crosses_interior(Model3D* model, int ei, int fj) {
    double a, b, c, d;
    compute_face_plane(model, fj, &a, &b, &c, &d);
    if (a == 0.0 && b == 0.0 && c == 0.0) return 0;

    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int cnt = faces->vertex_count[ei]; if (cnt < 2) return 0;

    /* choose 2D projection plane */
    double abs_a = fabs(a), abs_b = fabs(b), abs_c = fabs(c);
    int proj = (abs_a >= abs_b && abs_a >= abs_c) ? 0
            : (abs_b >= abs_a && abs_b >= abs_c) ? 1 : 2;

    /* build polygon for fj */
    int nfj = faces->vertex_count[fj];
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

    /* we'll call strict_inside below with the current polygon data; the
       real definition lives after this function (see forward declaration
       above). */

    int result = 0;
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
    free(vx); free(vy);
    return result;
}

/*
 * Helper: compute the plane equation (a x + b y + c z + d = 0) for a face.
 * Uses the first three vertices of the face; if the face has <3 vertices the
 * coefficients are zeroed.  The coordinates are taken from the original
 * object-space arrays (fixed-point values cast to double).
 */
/* compute_plane_float: same as compute_face_plane but converts
   fixed-point coordinates to floating values before computing coefficients.
   The resulting plane is in the same units as FIXED_TO_FLOAT() and thus
   compatible with distance calculations that also use FIXED_TO_FLOAT. */
static void compute_face_plane_float(Model3D* model, int fidx,
                                     double *a, double *b, double *c, double *d) {
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
    double ux = x1 - x0, uy = y1 - y0, uz = z1 - z0;
    double vx = x2 - x0, vy = y2 - y0, vz = z2 - z0;
    *a = uy * vz - uz * vy;
    *b = uz * vx - ux * vz;
    *c = ux * vy - uy * vx;
    *d = -(*a * x0 + *b * y0 + *c * z0);
}

/* original compute_face_plane retained for callers that expect fixed-coord
   planes; kept so that older routines (e.g. check_intersect) continue to
   operate exactly as before. */
static void compute_face_plane(Model3D* model, int fidx,
                               double *a, double *b, double *c, double *d) {
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
    double ux = x1 - x0, uy = y1 - y0, uz = z1 - z0;
    double vx = x2 - x0, vy = y2 - y0, vz = z2 - z0;
    *a = uy * vz - uz * vy;
    *b = uz * vx - ux * vz;
    *c = ux * vy - uy * vx;
    *d = -(*a * x0 + *b * y0 + *c * z0);
}

/* Definition of strict_inside helper moved outside of any other function
   to satisfy the compiler.  See the forward declaration at the top of this
   section. */
static int strict_inside(int px, int py, int nfj_local, int *vx_local, int *vy_local) {
    int m;
    /* inside or boundary? reuse existing test */
    if (!point_in_poly_arrays_int(px, py, nfj_local, vx_local, vy_local))
        return 0;
    /* if point equals any vertex or lies on any edge, reject */
    for (m = 0; m < nfj_local; ++m) {
        int x1 = vx_local[m], y1 = vy_local[m];
        if (px == x1 && py == y1) return 0;
        {
            int m2 = (m + 1) % nfj_local;
            int x2 = vx_local[m2], y2 = vy_local[m2];
            long long dx = (long long)x2 - x1, dy = (long long)y2 - y1;
            long long dxp = (long long)px - x1, dyp = (long long)py - y1;
            if (dx * dyp == dy * dxp) {
                /* collinear, now check within segment bounds */
                if ((px >= x1 && px <= x2) || (px >= x2 && px <= x1)) {
                    if ((py >= y1 && py <= y2) || (py >= y2 && py <= y1))
                        return 0;
                }
            }
        }
    }
    return 1;
}

/* existing count_edge_crossings follows here unchanged */
static int count_edge_crossings(Model3D* model, int ei, int fj) {
    int count = 0;
    double a, b, c, d;
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
    free(vx); free(vy);
    return count;
}

static int edge_intersects_face(Model3D* model, int ei, int fj) {

    double a, b, c, d;
    compute_face_plane(model, fj, &a, &b, &c, &d);
    if (a == 0.0 && b == 0.0 && c == 0.0) {
        /* degenerate plane */
        return 0;
    }

    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int cnt = faces->vertex_count[ei];
    if (cnt < 2) return 0;

    /* choose projection plane by largest component of normal */
    double abs_a = fabs(a), abs_b = fabs(b), abs_c = fabs(c);
    int proj; // 0=drop x,1=drop y,2=drop z
    if (abs_a >= abs_b && abs_a >= abs_c) proj = 0;
    else if (abs_b >= abs_a && abs_b >= abs_c) proj = 1;
    else proj = 2;

    /* build projected vertex arrays for face fj */
    int nfj = faces->vertex_count[fj];
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

    for (int k = 0; k < cnt; ++k) {
        int vi = faces->vertex_indices_buffer[faces->vertex_indices_ptr[ei] + k] - 1;
        int vj = faces->vertex_indices_buffer[faces->vertex_indices_ptr[ei] + ((k + 1) % cnt)] - 1;
        double Ax = (double)vtx->x[vi], Ay = (double)vtx->y[vi], Az = (double)vtx->z[vi];
        double Bx = (double)vtx->x[vj], By = (double)vtx->y[vj], Bz = (double)vtx->z[vj];
        const double plane_eps = 0.02; /* tolerance */
        double sdA = a*Ax + b*Ay + c*Az + d;
        double sdB = a*Bx + b*By + c*Bz + d;
        if (fabs(sdA) < plane_eps) sdA = 0.0;
        if (fabs(sdB) < plane_eps) sdB = 0.0;
        /* if both endpoints lie on plane, we still need to check whether the
           segment actually overlaps the interior of fj (coplanar case). */
        if (sdA == 0.0 && sdB == 0.0) {
            /* project A and B to 2D and test if either lies inside the
               polygon (ignoring boundary).  This catches the t.obj case
               where the entire vertical face sits in the horizontal plane. */
            int pax, pay, pbx, pby;
            if (proj == 0) { pax = FIXED_TO_INT((Fixed32)Ay); pay = FIXED_TO_INT((Fixed32)Az);
                             pbx = FIXED_TO_INT((Fixed32)By); pby = FIXED_TO_INT((Fixed32)Bz); }
            else if (proj == 1) { pax = FIXED_TO_INT((Fixed32)Ax); pay = FIXED_TO_INT((Fixed32)Az);
                                  pbx = FIXED_TO_INT((Fixed32)Bx); pby = FIXED_TO_INT((Fixed32)Bz); }
            else { pax = FIXED_TO_INT((Fixed32)Ax); pay = FIXED_TO_INT((Fixed32)Ay);
                   pbx = FIXED_TO_INT((Fixed32)Bx); pby = FIXED_TO_INT((Fixed32)By); }
            if (point_in_poly_arrays_int(pax, pay, nfj, vx, vy) ||
                point_in_poly_arrays_int(pbx, pby, nfj, vx, vy)) {


                free(vx); free(vy);
                return 1;
            }
            /* otherwise continue to next edge */
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
                free(vx); free(vy);
                return 1;
            } else {
            }
        }
    }

    free(vx); free(vy);
    return 0;
}

/* Split face `f1` by the infinite plane defined by face `f2`.  The
   behaviour is unchanged from the previous implementation except that the
   caller may supply an output buffer to receive the indices of any new
   vertices generated along the cutting line (these are the intersection
   points inserted during clipping).  This allows the caller to reuse the
   same points when splitting the other face, ensuring geometric consistency.

   Additionally, the caller may provide an optional array of
   `constraint_verts` lying along the intersection line.  When supplied the
   routine projects each candidate intersection point onto the line and
   ignores those outside the interval spanned by the constraint points.  This
   prevents the opposite face from being split by the full plane when only a
   finite segment of the line lies within the overlapping region.

   If `out_newverts` is non-null, up to `*out_n` indices will be written and
   `*out_n` updated.  The function returns 1 if a split occurred (bboxes are
   recomputed), or 0 otherwise. */
/*
 * Split face `f1` by the infinite plane defined by face `f2`.
 *
 * New behaviour relative to earlier revisions:
 *   - When `force_one_edge` is nonzero the routine will attempt the split
 *     even if all vertices of `f1` lie on the same side of the cutter plane.
 *     This supports the "one-edge crossing" rule requested by the user:
 *     if exactly one edge of `f1` intersects the interior of `f2` the face
 *     will still be subdivided.  (The opposite direction is still tested by
 *     the caller when appropriate.)
 *   - When forced, the routine will also bypass the usual vertex-count
 *     check; the original face is retained even if the opposite fragment has
 *     fewer than three vertices.  A degenerate fragment (count<3) is simply
 *     discarded rather than causing failure.
 *
 * All other parameters behave as before.  The caller may still supply
 * `out_newverts`/`out_n` and `constraint_verts`/`constraint_n` for clipping.
 */
/* build identifier printed in logs so user can verify they ran
   the newly compiled binary.  Update this string each time the splitting logic
   is modified. */

static int split_face_by_plane(Model3D* model, int f1, int f2,
                               int *out_newverts, int *out_n,
                               int *constraint_verts, int constraint_n,
                               int force_one_edge) {

    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int cnt = faces->vertex_count[f1];

    if (cnt < 3) return 0;


    /* plane of cutting face (float units for distance calc) */
    double a, b, c, d;
    compute_face_plane_float(model, f2, &a, &b, &c, &d);

    if (a == 0.0 && b == 0.0 && c == 0.0) return 0;

    const double plane_eps = 0.02;

    /* collect distances for each vertex of f1 */
    double *dist = (double*)malloc(sizeof(double) * cnt);
    int *orig = (int*)malloc(sizeof(int) * cnt);
    if (!dist || !orig) {
        free(dist);
        free(orig);
        return 0;
    }
    for (int i = 0; i < cnt; ++i) {
        int vi = faces->vertex_indices_buffer[faces->vertex_indices_ptr[f1] + i] - 1;
        orig[i] = vi;
        double x = FIXED_TO_FLOAT(vtx->x[vi]);
        double y = FIXED_TO_FLOAT(vtx->y[vi]);
        double z = FIXED_TO_FLOAT(vtx->z[vi]);

        double sd = a * x + b * y + c * z + d;  /* now consistent scaling */
        if (fabs(sd) < plane_eps) sd = 0.0;
        dist[i] = sd;

    }

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
            /* forced split: log that we're ignoring one-sided result */

        }
    }

    int pos_list[MAX_FACE_VERTICES * 2];
    int neg_list[MAX_FACE_VERTICES * 2];
    int cop_list[MAX_FACE_VERTICES];
    int pos_n = 0, neg_n = 0, cop_n = 0;

    /* temporary buffer to record intersections if requested */
    int newbuf[MAX_FACE_VERTICES];
    int newcount = 0;

    /* prepare constraint segment if provided */
    double seg_min = 0, seg_max = 0;
    int have_constraint = 0;
    double dirx=0, diry=0, dirz=0;
    if (constraint_verts && constraint_n >= 2) {
        /* compute direction of intersection line from normals of both planes */
        double a1,b1,c1,d1;
        compute_face_plane(model, f1, &a1, &b1, &c1, &d1);
        /* n1 = plane normal of f2 (a,b,c from above), n2 = normal of f1 */
        dirx = b * c1 - c * b1;
        diry = c * a1 - a * c1;
        dirz = a * b1 - b * a1;
        double len = sqrt(dirx*dirx + diry*diry + dirz*dirz);
        if (len > 0) {
            dirx /= len; diry /= len; dirz /= len;
            /* compute projection of each constraint point onto this direction */
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

        if (di * dj < 0.0) {
            double t = di / (di - dj);
            double ix = FIXED_TO_FLOAT(vtx->x[vi]) + t * (FIXED_TO_FLOAT(vtx->x[vj]) - FIXED_TO_FLOAT(vtx->x[vi]));
            double iy = FIXED_TO_FLOAT(vtx->y[vi]) + t * (FIXED_TO_FLOAT(vtx->y[vj]) - FIXED_TO_FLOAT(vtx->y[vi]));
            double iz = FIXED_TO_FLOAT(vtx->z[vi]) + t * (FIXED_TO_FLOAT(vtx->z[vj]) - FIXED_TO_FLOAT(vtx->z[vi]));
            /* if constraints provided, ensure intersection lies within segment */
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

    if (out_newverts && out_n) {
        *out_n = newcount;
        for (int k = 0; k < newcount; ++k) out_newverts[k] = newbuf[k];
    }

    free(dist);
    free(orig);

    cleanup_list(pos_list, &pos_n);
    cleanup_list(neg_list, &neg_n);
    cleanup_list(cop_list, &cop_n);
    remove_duplicates(pos_list, &pos_n);
    remove_duplicates(neg_list, &neg_n);
    if (cop_n >= 3) remove_duplicates(cop_list, &cop_n);


    /* normally we require both sides to have at least three verts, but a
       forced one-edge split bypasses this check. */
    if (!force_one_edge && (pos_n < 3 || neg_n < 3)) return 0;

    /* ensure polygon vertex order is sensible before measuring area */
    reorder_poly(model, f1, pos_list, &pos_n);
    reorder_poly(model, f1, neg_list, &neg_n);

    /* calculate areas using the plane of the face being split (f1) rather
       than the cutter plane.  This avoids the zero-area issue when the
       two faces are perpendicular. */
    double a1, b1, c1, d1;
    compute_face_plane(model, f1, &a1, &b1, &c1, &d1);
    double area_pos = poly_signed_area(model, pos_list, pos_n, a1, b1, c1);
    double area_neg = poly_signed_area(model, neg_list, neg_n, a1, b1, c1);
    if (fabs(area_pos) < 0.001 || fabs(area_neg) < 0.001) {
        /* area seems small or zero; log for debugging but do not abort the
           split.  numeric quantization sometimes makes a valid polygon appear
           degenerate (see t.obj case). */

        /* continue regardless */
    }

    reorder_poly(model, f1, pos_list, &pos_n);
    reorder_poly(model, f1, neg_list, &neg_n);

    int orig_cnt = faces->vertex_count[f1];
    int orig_off = faces->vertex_indices_ptr[f1];
    /* choose which side becomes the "kept" polygon.  if one side lacks
       enough vertices we keep the other, unless both have >=3 in which case
       keep the positive side by convention. */
    int *keep_list = NULL, keep_n = 0;
    int *other_list = NULL, other_n = 0;
    if (pos_n >= 3 && (neg_n < 3 || !force_one_edge)) {
        keep_list = pos_list; keep_n = pos_n;
        other_list = neg_list; other_n = neg_n;
    } else if (neg_n >= 3 && (pos_n < 3 || force_one_edge && pos_n < 3)) {
        keep_list = neg_list; keep_n = neg_n;
        other_list = pos_list; other_n = pos_n;
    } else {
        /* both valid or both too small; default to positive side */
        keep_list = pos_list; keep_n = pos_n;
        other_list = neg_list; other_n = neg_n;
    }

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
    int orig_disp = faces->display_flag[f1];



    /* create opposite fragment only if it has enough vertices */
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

    compute_face_bboxes3D(model);
    return 1;
}

/* utility: append a new vertex to the model's vertex arrays and return
   0-based index, or -1 if capacity exceeded */
static int add_vertex(Model3D* model, Fixed32 x, Fixed32 y, Fixed32 z) {
    if (!model) return -1;
    VertexArrays3D* vtx = &model->vertices;
    if (vtx->vertex_count >= MAX_VERTICES) return -1;
    int idx = vtx->vertex_count;
    vtx->x[idx]  = x;
    vtx->y[idx]  = y;
    vtx->z[idx]  = z;
    vtx->xo[idx] = 0;
    vtx->yo[idx] = 0;
    vtx->zo[idx] = 0;
    vtx->x2d[idx] = 0;
    vtx->y2d[idx] = 0;
    vtx->vertex_count++;
    return idx;
}

/* utility: remove consecutive duplicate indices from a polygon list; also
   check wrap-around duplicates */
static void cleanup_list(int *arr, int *n) {
    if (*n <= 1) return;
    int w = 0;
    for (int i = 0; i < *n; ++i) {
        if (i == 0 || arr[i] != arr[i-1]) {
            arr[w++] = arr[i];
        }
    }
    if (w > 1 && arr[0] == arr[w-1]) w--;
    *n = w;
}

/* Remove any duplicate index appearing later in the list, preserving only
   the first occurrence. This prevents repeated vertices from creating
   self-intersections. */
static void remove_duplicates(int *arr, int *n) {
    int w = 0;
    for (int i = 0; i < *n; ++i) {
        int found = 0;
        for (int j = 0; j < w; ++j) {
            if (arr[i] == arr[j]) { found = 1; break; }
        }
        if (!found) arr[w++] = arr[i];
    }
    *n = w;
}

/* Clip the polygon of face `fidx` against the half-space defined by
   plane `a*x + b*y + c*z + d >= 0` when `keepPositive` is true, otherwise
   the complementary <=0 side is retained.  The resulting vertex indices are
   written into `outbuf` with count returned via `*outcnt`.  Intersection
   points are created with `add_vertex`. */
static void clip_face_plane(Model3D* model, int fidx,
                             double a, double b, double c, double d,
                             int keepPositive, int *outbuf, int *outcnt) {
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int cnt = faces->vertex_count[fidx];
    *outcnt = 0;
    if (cnt <= 0) return;
    int off = faces->vertex_indices_ptr[fidx];
    int prev_vi = faces->vertex_indices_buffer[off] - 1;
    double x = (double)vtx->x[prev_vi];
    double y = (double)vtx->y[prev_vi];
    double z = (double)vtx->z[prev_vi];
    double sd_prev = a*x + b*y + c*z + d;
    for (int k = 0; k < cnt; ++k) {
        int vi = faces->vertex_indices_buffer[off + k] - 1;
        x = (double)vtx->x[vi];
        y = (double)vtx->y[vi];
        z = (double)vtx->z[vi];
        double sd = a*x + b*y + c*z + d;
        int in_prev = keepPositive ? (sd_prev >= 0) : (sd_prev <= 0);
        int in_curr = keepPositive ? (sd >= 0) : (sd <= 0);
        if (in_curr) {
            outbuf[*outcnt] = vi;
            (*outcnt)++;
        }
        if (in_prev ^ in_curr) {
            /* compute intersection point */
            double t = sd_prev / (sd_prev - sd);
            double ix = (double)vtx->x[prev_vi] + t * ((double)vtx->x[vi] - (double)vtx->x[prev_vi]);
            double iy = (double)vtx->y[prev_vi] + t * ((double)vtx->y[vi] - (double)vtx->y[prev_vi]);
            double iz = (double)vtx->z[prev_vi] + t * ((double)vtx->z[vi] - (double)vtx->z[prev_vi]);
            int newvi = add_vertex(model,
                                   FLOAT_TO_FIXED((float)ix),
                                   FLOAT_TO_FIXED((float)iy),
                                   FLOAT_TO_FIXED((float)iz));
            if (newvi >= 0) {
                outbuf[*outcnt] = newvi;
                (*outcnt)++;
            }
        }
        prev_vi = vi;
        sd_prev = sd;
    }
}
/* compute signed area of a polygon (vertex list) projected onto a plane
   defined by normal (a,b,c).  returns positive for CCW orientation. */
static double poly_signed_area(Model3D* model, int *list, int n,
                               double a, double b, double c) {
    if (n < 3) return 0.0;
    double abs_a = fabs(a), abs_b = fabs(b), abs_c = fabs(c);
    int proj;
    if (abs_a >= abs_b && abs_a >= abs_c) proj = 0;
    else if (abs_b >= abs_a && abs_b >= abs_c) proj = 1;
    else proj = 2;
    VertexArrays3D* vtx = &model->vertices;
    double area = 0;
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

/* Reorder `list` of vertex indices (length `*n`) so that they are sorted
   counter‑clockwise around the centroid when projected onto the plane of
   face `fidx`.  This ensures a simple, non‑self‑intersecting polygon. */
static void reorder_poly(Model3D* model, int fidx, int *list, int *n) {
    if (!model || *n < 3) return;
    /* compute projection plane from face normal */
    double a, b, c, d;
    compute_face_plane(model, fidx, &a, &b, &c, &d);
    double abs_a = fabs(a), abs_b = fabs(b), abs_c = fabs(c);
    int proj;
    if (abs_a >= abs_b && abs_a >= abs_c) proj = 0;
    else if (abs_b >= abs_a && abs_b >= abs_c) proj = 1;
    else proj = 2;

    /* gather 2D coordinates */
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
    /* compute centroid */
    double cx = 0, cy = 0;
    for (int i = 0; i < *n; ++i) { cx += px[i]; cy += py[i]; }
    cx /= *n; cy /= *n;
    /* compute angles */
    double *ang = (double*)malloc(sizeof(double) * (*n));
    if (!ang) { free(px); free(py); return; }
    for (int i = 0; i < *n; ++i) {
        ang[i] = atan2(py[i] - cy, px[i] - cx);
    }
    /* simple insertion sort on angles (small n) */
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
    /* ensure orientation matches original sign (ccw positive) */
    double area = 0;
    for (int i = 0; i < *n; ++i) {
        int k = (i + 1) % *n;
        double x1 = px[i], y1 = py[i];
        double x2 = px[k], y2 = py[k];
        area += (x1 * y2 - x2 * y1);
    }
    if (area < 0) {
        /* reverse order */
        for (int i = 0; i < *n/2; ++i) {
            int t = list[i]; list[i] = list[*n-1-i]; list[*n-1-i] = t;
        }
    }
    free(px); free(py); free(ang);
}

/* Compute object-space axis-aligned bounding box for every face.  Allocates
 * arrays inside faces->minx3 etc.  Must be called after vertices and faces are
 * loaded; it does nothing if face_count is zero.
 */
static void compute_face_bboxes3D(Model3D* model) {
    if (!model) return;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int fc = faces->face_count;
    if (fc <= 0) return;
    /* allocate arrays (free previous if any) */
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
    if (!faces->minx3||!faces->maxx3||!faces->miny3||!faces->maxy3||!faces->minz3||!faces->maxz3) {
        /* if allocation failed, free and clear */
        free(faces->minx3); free(faces->maxx3); free(faces->miny3);
        free(faces->maxy3); free(faces->minz3); free(faces->maxz3);
        faces->minx3 = faces->maxx3 = faces->miny3 = faces->maxy3 = faces->minz3 = faces->maxz3 = NULL;
        return;
    }
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


segment "painter_core";
// ============================================================================
//  1. PAINTER CORE
//  Newell/Sancha algorithm and all painter variants
// ============================================================================

/**
 * processModelFast -- Combined transformation, projection, and painter invocation
 * ============================================================================
 * Responsibilities:
 *  - Transform model vertices into observer-space (xo/yo/zo) using Fixed32 arithmetic.
 *  - Project to integer screen coords (x2d/y2d) using configured projection scale.
 *  - If using the FLOAT painter mode, populate float caches (float_xo/float_yo/float_zo etc.)
 *    used by the float-based painter for efficient per-face computations.
 *  - Call `calculateFaceDepths()` to compute per-face depth metrics, bboxes and plane coeffs.
 *  - Dispatch to the selected painter implementation (FAST, FIXED, or FLOAT) to build
 *    the final `sorted_face_indices` for rendering.
 *
 * Notes:
 *  - Autoscaling is **not** performed here; any autoscale (previously implemented in an archived helper) must be applied
 *    before calling `processModelFast()` (e.g. at load time or via explicit user action). The archived helper's implementation is in `chutier.txt`.
 *  - The painter may only sort the visible faces when back-face culling is enabled. Culled
 *    faces are appended after visible faces to preserve index stability.
 *  - Timing instrumentation prints per-stage costs when not in PERFORMANCE_MODE.
 *  - OPTIMISATION : pre calculate matrix coefficients and use direct table access 
 *   for trig functions to avoid function call overhead. 100% Fixed32 loop with no conversions for maximum speed.
 */

void processModelFast(Model3D* model, ObserverParams* params, const char* filename) {
    int i;

    Fixed32 cos_h, sin_h, cos_v, sin_v, cos_w, sin_w;
    Fixed32 x, y, z, zo, xo, yo;
    Fixed32 inv_zo, x2d_temp, y2d_temp;
    
    // Direct table access - ultra-fast! (no function calls)
    cos_h = cos_deg_int(params->angle_h);
    sin_h = sin_deg_int(params->angle_h);
    cos_v = cos_deg_int(params->angle_v);
    sin_v = sin_deg_int(params->angle_v);
    cos_w = cos_deg_int(params->angle_w);
    sin_w = sin_deg_int(params->angle_w);

    // Pre-calculate all trigonometric products in Fixed32 - using 64-bit multiply
    const Fixed32 cos_h_cos_v = FIXED_MUL_64(cos_h, cos_v);
    const Fixed32 sin_h_cos_v = FIXED_MUL_64(sin_h, cos_v);
    const Fixed32 cos_h_sin_v = FIXED_MUL_64(cos_h, sin_v);
    const Fixed32 sin_h_sin_v = FIXED_MUL_64(sin_h, sin_v);

    /* Precomputed rotation matrix - 9 coefficients */
    Fixed32 M00, M01, M02;
    Fixed32 M10, M11, M12;
    Fixed32 M20, M21, M22;

    // Use global projection scale, which is synced with auto-fit when applied
    Fixed32 scale = s_global_proj_scale_fixed;
    const Fixed32 centre_x_f = INT_TO_FIXED(CENTRE_X);
    const Fixed32 centre_y_f = INT_TO_FIXED(CENTRE_Y);
    Fixed32 distance = params->distance;
    // Apply auto-scale if any
    //distance = FIXED_MUL_64(distance, INT_TO_FIXED(4));
    
    // 100% Fixed32 loop - ZERO conversions, maximum speed!
    VertexArrays3D* vtx = &model->vertices;
    Fixed32 *x_arr = vtx->x, *y_arr = vtx->y, *z_arr = vtx->z;
    Fixed32 *xo_arr = vtx->xo, *yo_arr = vtx->yo, *zo_arr = vtx->zo;
    int *x2d_arr = vtx->x2d, *y2d_arr = vtx->y2d;
    int vcount = vtx->vertex_count;

    long t_loop_start = GetTick();
    
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
        M00 = FIXED_ADD(FIXED_MUL_64(FIXED_NEG(cos_w), sin_h),
                        FIXED_MUL_64(sin_w, cos_h_sin_v));
        M01 = FIXED_SUB(FIXED_MUL_64(cos_w, cos_h),
                        FIXED_MUL_64(sin_w, FIXED_NEG(sin_h_sin_v)));
        M02 = FIXED_MUL_64(FIXED_NEG(sin_w), cos_v);
        M10 = FIXED_ADD(FIXED_MUL_64(FIXED_NEG(sin_w), sin_h),
                        FIXED_MUL_64(FIXED_NEG(cos_w), cos_h_sin_v));
        M11 = FIXED_ADD(FIXED_MUL_64(sin_w, cos_h),
                        FIXED_MUL_64(FIXED_NEG(cos_w), sin_h_sin_v));
        M12 = FIXED_MUL_64(cos_w, cos_v);
    }


    for (i = 0; i < vcount; i++) {
        x = x_arr[i];
        y = y_arr[i];
        z = z_arr[i];

        zo = FIXED_ADD(
                FIXED_ADD(
                    FIXED_ADD(FIXED_MUL_64(M20, x),
                            FIXED_MUL_64(M21, y)),
                    FIXED_MUL_64(M22, z)),
                distance);

        if (zo > 0) {
            xo = FIXED_ADD(FIXED_ADD(FIXED_MUL_64(M00, x),
                                    FIXED_MUL_64(M01, y)),
                        FIXED_MUL_64(M02, z));
            yo = FIXED_ADD(FIXED_ADD(FIXED_MUL_64(M10, x),
                                    FIXED_MUL_64(M11, y)),
                        FIXED_MUL_64(M12, z));
            zo_arr[i] = zo;
            xo_arr[i] = xo;
            yo_arr[i] = yo;
            inv_zo = FIXED_DIV_64(scale, zo);
            x2d_arr[i] = FIXED_ROUND_TO_INT(
                            FIXED_ADD(FIXED_MUL_64(xo, inv_zo), centre_x_f));
            y2d_arr[i] = FIXED_ROUND_TO_INT(
                            FIXED_SUB(centre_y_f, FIXED_MUL_64(yo, inv_zo)));
        } else {
            zo_arr[i] = zo;
            xo_arr[i] = 0;
            yo_arr[i] = 0;
            x2d_arr[i] = -1;
            y2d_arr[i] = -1;
        }
    }

    long t_loop_end = GetTick();

    // Face sorting after transformation
    long t_start, t_end;
    t_start = GetTick();
    calculateFaceDepths(model, NULL, model->faces.face_count);
    if (shaded_by_orientation) {
        computeOrientationShading(model);
    }
    t_end = GetTick();


    if (framePolyOnly) goto skip_calc; // Skip face calculations for framed polygons only display

    // CRITICAL: Reset sorted_face_indices before each sort to prevent corruption
    // for (i = 0; i < model->faces.face_count; i++) {
    //     model->faces.sorted_face_indices[i] = i;
    // }
    t_start = GetTick();
    if (painter_mode == PAINTER_MODE_FAST) {
        painter_newell_sancha_fast(model, model->faces.face_count);
    } else if (painter_mode == PAINTER_MODE_BUBBLE_SORT) {
        painter_bubble_sort(model, model->faces.face_count);
    } else if (painter_mode == PAINTER_MODE_NEWELL_SANCHA) {
        painter_newell_sancha(model, model->faces.face_count);
    } else if (painter_mode == PAINTER_MODE_CORRECT) {
        /* painter_correct acts as a sorting mode: it will adjust faces->sorted_face_indices in-place */
        painter_correct(model, model->faces.face_count, 0);
    } else if (painter_mode == PAINTER_MODE_GEO) {
        painter_geoV2(model, model->faces.face_count);
    } else if (painter_mode == PAINTER_MODE_CORRECTV2) {
        /* painter_correctV2: experimental face splitting version */
        painter_correctV2(model, model->faces.face_count, 0);
    } 
    else if (painter_mode == PAINTER_MODE_GEOV3) {
        painter_geoV3(model, model->faces.face_count);
    }
    t_end = GetTick();

    skip_calc:;
}

// Comparator support for qsort in painter_bubble_sort
static int cmp_faces_by_zmean(const void* pa, const void* pb) {
    int a = *(const int*)pa;
    int b = *(const int*)pb;
    Fixed32 za = qsort_faces_ptr_for_cmp->z_mean[a];
    Fixed32 zb = qsort_faces_ptr_for_cmp->z_mean[b];
    if (za > zb) return -1;   // larger z_mean first (descending)
    if (za < zb) return 1;
    if (a < b) return -1;     // tie-breaker: smaller index first
    if (a > b) return 1;
    return 0;
}

/* geometric relation: plane-based tests only (no bbox or depth).  Returns
   -1 if f1 is geometrically before f2, 1 if after, 0 if indeterminate. */
/* =====================================================================
 * geometric_face_relation  (OPTIMIZED)
 * ---------------------------------------------------------------------
 * Plane-based ordering test between two faces: returns -1 if f1 is
 * geometrically before f2, 1 if after, 0 if indeterminate.
 *
 * WHAT CHANGED AND WHY IT SHOULD NOT AFFECT THE RESULT
 *
 * `epsilon` used to be recomputed on every single call via:
 *     Fixed64 epsilon = FIXED_MUL_64(current_observer_distance, FLOAT_TO_FIXED(0.001f));
 * but its value depends only on `current_observer_distance`, a
 * camera/observer parameter that is constant for the whole duration of a
 * single painter sort pass (it only changes once per frame, computed
 * earlier in processModelFast() via getObserverParams() /
 * normalizeAutoFitDistanceTo150(), before any painter/ordering code runs -
 * see call graph section 3 vs section 1).
 *
 * This function has a very high fan-in (called from painter_bubble_sort,
 * painter_geoV2, evaluate_pair_tests, geo_face_order, pair_order_relation),
 * so it can run thousands of times per frame. FIXED_MUL_64 is a 64-bit
 * fixed-point multiply; on 65816 (no hardware multiplier) this is almost
 * certainly a software multi-precision routine, i.e. one of the more
 * expensive operations available - and it was being redone identically,
 * every single call, for a value that had not changed since the previous
 * call in the overwhelming majority of cases.
 *
 * The static cache below memoizes the last computed epsilon together with
 * the observer distance it was computed from, plus an explicit validity
 * flag (rather than a sentinel distance value) so the cache logic never
 * has to assume anything about how Fixed64 represents negative numbers or
 * what values current_observer_distance can plausibly take. On each call:
 *   - if the cache is valid AND current_observer_distance is unchanged
 *     since the cached value, we reuse the cached epsilon (same value the
 *     original formula would have produced, just not recomputed);
 *   - otherwise we recompute exactly as the original always did, and
 *     refresh the cache (marking it valid).
 * In both branches the epsilon value used is bit-for-bit identical to
 * what the original, unconditional recomputation would have produced for
 * that same current_observer_distance - so the result of this function is
 * unchanged for every input. The only added cost is one flag check plus
 * one Fixed64 comparison per call, negligible next to a software 64-bit
 * multiply.
 *
 * The (Fixed64) casts on a1/b1/c1 inside the per-vertex loop were also
 * removed - a1/b1/c1 are already declared Fixed64, so those casts were
 * no-ops; purely cosmetic, no behavior change.
 *
 * Everything else (the obs sign test, the per-vertex side test with its
 * epsilon tolerance, the all_same/all_opp bookkeeping, and the early-exit
 * once neither hypothesis can still hold) is untouched, byte-for-byte,
 * from the original.
 * ===================================================================== */
static int geometric_face_relation(Model3D* model, int f1, int f2) {
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    static Fixed64 cached_geo_epsilon = 0;
    static Fixed64 cached_geo_epsilon_distance = 0;
    static int cached_geo_epsilon_valid = 0; /* 0 until the first real computation */

    /* epsilon only depends on current_observer_distance, which is constant
     * for an entire painter sort pass (one frame) - see header comment
     * above for the full justification of this cache. */
    Fixed64 epsilon;
    if (cached_geo_epsilon_valid && current_observer_distance == cached_geo_epsilon_distance) {
        epsilon = cached_geo_epsilon;
    } else {
        epsilon = FIXED_MUL_64(current_observer_distance, FLOAT_TO_FIXED(0.001f));
        cached_geo_epsilon = epsilon;
        cached_geo_epsilon_distance = current_observer_distance;
        cached_geo_epsilon_valid = 1;
    }

    Fixed64 a1 = faces->plane_a[f1], b1 = faces->plane_b[f1],
            c1 = faces->plane_c[f1], d1 = faces->plane_d[f1];

    int obs;
    if (d1 > epsilon) obs = 1;
    else if (d1 < -epsilon) obs = -1;
    else return 0;  /* obs == 0 : comportement identique à l'original,
                        les deux blocs sautaient déjà leur boucle dans ce cas */

    int n2 = faces->vertex_count[f2];
    int offset2 = faces->vertex_indices_ptr[f2];
    int all_same = 1, all_opp = 1;

    for (int k = 0; k < n2; ++k) {
        int v = faces->vertex_indices_buffer[offset2 + k] - 1;
        Fixed64 acc = ((a1 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT)
                    + ((b1 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT)
                    + ((c1 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT)
                    + d1;

        int side;
        if (acc > epsilon) side = 1;
        else if (acc < -epsilon) side = -1;
        else continue;  /* sommet ambigu : ne contredit ni l'une ni l'autre hypothèse */

        if (side == obs) all_opp = 0; else all_same = 0;

        if (!all_same && !all_opp) return 0;  /* sortie anticipée : plus rien à prouver */
    }

    if (all_same) return -1;
    if (all_opp) return 1;
    return 0;
}
/**
 * PAINTER'S ALGORITHM - NEWELL / SANCHA (detailed)
 * =================================================
 * Overview:
 *  - The painter builds an ordering of faces for correct filled-polygon rendering without
 *    performing geometric splits. The algorithm follows the Newell/Sancha approach:
 *      1) Compute per-face depth metrics (z_min/z_max/z_mean) and planar coefficients.
 *      2) Initially sort faces by z_mean (descending) to obtain a broadly correct depth order.
 *      3) Use axis-aligned bounding-box overlap tests (X and Y) to detect candidate overlapping faces.
 *      4) For overlapping pairs, apply robust pairwise ordering tests using plane/triangle votes and
 *         local sampling; if the order is ambiguous, record the pair in the `inconclusive_pairs`
 *         buffer for diagnostic framing and later inspection (no automatic splitting).
 *
 * Visibility & culling:
 *  - When `cull_back_faces` is enabled, the painter operates primarily on the subset of faces
 *    with `display_flag == 1` (visible). Culled faces are appended afterwards to preserve
 *    stable indices and deterministic behavior of `sorted_face_indices`.
 *
 * Modes:
 *  - FAST: performs only low-cost tests (depth + bbox) and skips plane calculations for speed.
 *  - FIXED: Fixed-point Newell/Sancha implementation.

 * Notes:
 *  - The algorithm is intentionally conservative: inconclusive pairs are stored rather than
 *    forcing a risky swap that may introduce visual artifacts.
 *  - Use `frameInconclusivePairs()` in diagnostics to highlight such pairs in the rendered view.
 *
 * Usage:
 *  - Call `painter_bubble_sort(model, face_count)` (or a mode-specific variant) after
 *    `calculateFaceDepths()` / `processModelFast()` has computed observer-space coordinates.
 */


/**
 * FAST PUBLISHABLE PASS (painter_newell_sancha_fast)
 * --------------------------------------------------
 * Purpose:
 *   Very fast ordering pass intended for interactive / high-frame-rate use. This mode:
 *    - Sorts visible faces by `z_mean` only (stable sort via qsort with tie-breaker on index).
 *    - Performs only inexpensive overlap tests (axis-aligned X and Y bounding boxes) to
 *      detect obvious separations. No plane coefficients, no Newell plane tests, and no
 *      ordering corrections are computed.
 *
 * Tradeoffs:
 *   - Much faster and suitable for large models, but may leave ordering ambiguities
 *     unresolved (which shows up as graphical overlap artifacts for complex geometry).
 *   - When `cull_back_faces` is set, this pass operates on the subset of faces with
 *     `display_flag == 1` and appends culled faces after the visible list to keep indices
 *     stable for downstream users.
 */
void painter_newell_sancha_fast(Model3D* model, int face_count) {
    FaceArrays3D* faces = &model->faces;
    if (!faces->z_mean) return; // safety

    // Build list of faces to sort: when culling is enabled, only visible faces are sorted
    int i;
    int visible_count = face_count;
    if (cull_back_faces) {
        visible_count = 0;
        // put visible faces first
        for (i = 0; i < face_count; ++i) {
            if (faces->display_flag[i]) faces->sorted_face_indices[visible_count++] = i;
        }
        // append culled faces to keep array content stable
        int tail = visible_count;
        for (i = 0; i < face_count; ++i) {
            if (!faces->display_flag[i]) faces->sorted_face_indices[tail++] = i;
        }
    } else {
        for (i = 0; i < face_count; ++i) faces->sorted_face_indices[i] = i;
    }

    // Initial stable sort by z_mean (descending) with tie-breaker on index, only on visible faces
    qsort_faces_ptr_for_cmp = faces;
    qsort(faces->sorted_face_indices, visible_count, sizeof(int), cmp_faces_by_zmean);
    qsort_faces_ptr_for_cmp = NULL;
}

void painter_bubble_sort_old(Model3D* model, int face_count) {
    // ...existing code...
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int i, j;
    
    Fixed32* face_zmean = faces->z_mean;
    if (!face_zmean) return; // safety


    // * * * * * 
    // Step 1: Stable descending Z sort on the mean, with tie-breaker on original index
    // * * * * *

    // delegate initial ordering to the fast variant which already implements
    // visible-face filtering and the stable z-mean sort.
    painter_newell_sancha_fast(model, face_count);

    // recompute `visible_count` for the correction passes
    int visible_count = face_count;
    if (cull_back_faces) {
        visible_count = 0;
        for (i = 0; i < face_count; ++i) {
            if (faces->display_flag[i]) ++visible_count;
        }
    }
    
    int swap_count = 0;
    int swapped = 0; // flag used for the correction loop

    // Ordered pairs cache: store definitive pairwise relations to avoid re-testing.
    typedef struct {
        int face1;  // Face that must be drawn before (farther)
        int face2;  // Face that must be drawn after (closer)
    } OrderedPair;

    int ordered_pairs_capacity = face_count * 4; // heuristic capacity to reduce reallocs
    OrderedPair* ordered_pairs = NULL;
    if (ordered_pairs_capacity > 0) {
        ordered_pairs = (OrderedPair*)malloc(ordered_pairs_capacity * sizeof(OrderedPair));
        if (!ordered_pairs) {
            ordered_pairs_capacity = 0; // fall back to not caching ordered pairs
        }
    }
    int ordered_pairs_count = 0;


    // Prepare global inconclusive buffer for diagnostic recording. These pairs are
    // intentionally left unresolved in order to avoid mesh splits; they can be
    // inspected with `frameInconclusivePairs()` for debugging.
    if (inconclusive_pairs) {
        free(inconclusive_pairs);
        inconclusive_pairs = NULL;
    }
    inconclusive_pairs_capacity = face_count * 4;
    if (inconclusive_pairs_capacity > 0) {
        inconclusive_pairs = (InconclusivePair*)malloc(inconclusive_pairs_capacity * sizeof(InconclusivePair));
        if (!inconclusive_pairs) {
            inconclusive_pairs_capacity = 0;
        }
    }
    inconclusive_pairs_count = 0;

    // Bubble-like correction passes (iterate until stable)
    do {
        swapped = 0;
   
        // Iterate consecutive pairs (only over visible faces if culling is enabled)
        for (i = 0; i < visible_count-1; i++) {
            int f1 = faces->sorted_face_indices[i];
            int f2 = faces->sorted_face_indices[i+1];


            // Skip pairs already declared ordered by previous swaps or tests.
            // ordered_pairs stores definitive relations discovered earlier in the pass to
            // avoid repeated work (e.g., if we previously determined f2 < f1 we won't re-evaluate).
            int already_ordered = 0;
            int p;
            for (p = 0; p < ordered_pairs_count; p++) {
                if (ordered_pairs[p].face1 == f1 && ordered_pairs[p].face2 == f2) {
                    already_ordered = 1; break; // f1 before f2
                }
                if (ordered_pairs[p].face1 == f2 && ordered_pairs[p].face2 == f1) {
                    already_ordered = 1; break; // f2 before f1 (inverse relation)
                }
            }
            if (already_ordered) {
                // printf("Already ordered: %d <-> %d\n", f1, f2);
                continue;
            }

            // Test 1 : Depth overlap
            // Depth quick test: if farthest point of P2 is in front of nearest point of P1, order is respected
            if (faces->z_max[f2] <= faces->z_min[f1]) continue;
            if (faces->z_max[f1] <= faces->z_min[f2]) goto do_swap;
            
            // Bounding Box 2D overlap (split into X and Y parts)
            // Use cached bounding boxes (computed in calculateFaceDepths)

            // Test 2 : X overlap only (axis-aligned bbox separation test)
            // If bounding boxes do not intersect on X, faces cannot overlap on screen and
            // the current order is safe.
            int minx1 = faces->minx[f1], maxx1 = faces->maxx[f1], miny1 = faces->miny[f1], maxy1 = faces->maxy[f1];
            int minx2 = faces->minx[f2], maxx2 = faces->maxx[f2], miny2 = faces->miny[f2], maxy2 = faces->maxy[f2];
            if (maxx1 <= minx2 || maxx2 <= minx1) continue; // separated on X
            
            // Test 3 : Y overlap only (axis-aligned bbox separation test)
            // Similarly, if separated on Y, faces do not overlap and no swap is needed.
            if (maxy1 <= miny2 || maxy2 <= miny1) continue; // separated on Y


            // Polygon overlap test 
            if (!projected_polygons_overlap(model, f1, f2)) {
                // bounding boxes overlap but polygons do not overlap
                continue;
            }

            /* Tests 4 5 : perform purely geometric plane tests here, replacing the lengthy
               inline code that followed previously */
            {
                int geo = geometric_face_relation(model, f1, f2);
                if (geo == -1) continue;
                if (geo == 1) goto do_swap;
            }

            /* Remaining geometric checks (tests 5 & 7) can be handled by calling
               the helper again with swapped arguments.  The result is negated just
               like `pair_order_relation` does. */
            {
                int geo2 = geometric_face_relation(model, f2, f1);
                if (geo2 == -1) goto do_swap; // swapped result means f2 before f1 -> swap
                // if (geo2 == 1) continue;     // f2 after f1 -> order correct
                if (geo2 == 1) goto inconclusive;     // still inconclusive, fall through to record as inconclusive pair */
            }
        
                
            do_swap: {
                // Perform adjacent swap: this is an in-place stable operation and keeps
                // changes local (simple bubble logic). We record the definitive ordered
                // relation (f2 before f1 after swap) into `ordered_pairs` when possible
                // so subsequent passes skip redundant checks.
                int tmp = faces->sorted_face_indices[i];
                faces->sorted_face_indices[i] = faces->sorted_face_indices[i+1];
                faces->sorted_face_indices[i+1] = tmp;
                swapped = 1;
                swap_count++;

                // Record the pair as an established ordering (if capacity permits).
                // If we exceed capacity we silently drop the record — this only affects
                // performance (more re-evaluation), not correctness.
                if (ordered_pairs != NULL && ordered_pairs_count < ordered_pairs_capacity) {
                    ordered_pairs[ordered_pairs_count].face1 = f2; // now before
                    ordered_pairs[ordered_pairs_count].face2 = f1; // now after
                    ordered_pairs_count++;
                }

                // After a swap we skip ahead to the next pass iteration (goto ends current pair loop)
                goto endfor;
            }

        inconclusive: ;
        // If we reach this point, no test was able to draw a conclusion
        // We should perform clipping of f1 by f2 (or vice versa), but it is not done now
  
       if (inconclusive_pairs != NULL && inconclusive_pairs_count < inconclusive_pairs_capacity) {
                inconclusive_pairs[inconclusive_pairs_count].face1 = f1;
                inconclusive_pairs[inconclusive_pairs_count].face2 = f2;
                inconclusive_pairs_count++;
            }
        // add them to the ordered-pairs list so they are not retested
        // if tests were inconclusive, preserve the current ordering
        if (ordered_pairs != NULL && ordered_pairs_count < ordered_pairs_capacity) {
                ordered_pairs[ordered_pairs_count].face1 = f2;
                ordered_pairs[ordered_pairs_count].face2 = f1;
                ordered_pairs_count++;
            }
        
        endfor: ;
        } // END of the for loop int i=0; i<face_count-1; i++

    } while (swapped);
    // End of bubble sort

    
    // Free the memory of the ordered pairs list
    if (ordered_pairs) {
        free(ordered_pairs);
    }  
}

//  painter_newell_sancha
void painter_bubble_sort(Model3D* model, int face_count)
{
    // Early exit for trivial cases
    if (face_count <= 1) {
        painter_newell_sancha_fast(model, face_count);
        return;
    }

    FaceArrays3D* faces = &model->faces;

    // -------------------------------------------------
    // 1. Initial fast sort + back-face culling
    // -------------------------------------------------
    // This performs a stable descending Z-mean sort and
    // optionally filters out back-facing polygons.
    painter_newell_sancha_fast(model, face_count);

    // Recalculate the number of visible faces after culling
    int visible_count = face_count;
    if (cull_back_faces) {
        visible_count = 0;
        for (int i = 0; i < face_count; ++i) {
            if (faces->display_flag[i]) {
                ++visible_count;
            }
        }
    }

    // Nothing to correct if fewer than 2 faces are visible
    if (visible_count < 2) return;

    // Shortcut to the sorted index array
    int* sorted = faces->sorted_face_indices;

    // -------------------------------------------------
    // 2. Ordered-pair cache using a simple hash table
    // -------------------------------------------------
    // We store definitive ordering relations so we never
    // re-test the same pair of faces in later passes.
    // This dramatically reduces the number of expensive
    // geometric tests when many faces are present.

    typedef struct {
        int face1;      // Face that should be drawn first (farther)
        int face2;      // Face that should be drawn second (closer)
        int next;       // Next entry in the hash collision chain
    } OrderedPair;

    // Hash table size (must be a power of two)
    // 2048 is a good compromise between memory usage and
    // collision rate on the Apple IIGS.
    const int HASH_SIZE = 2048;
    const int HASH_MASK = HASH_SIZE - 1;

    // Allocate and initialize the hash table
    int* hash_table = (int*)malloc(HASH_SIZE * sizeof(int));
    if (!hash_table) {
        // Out of memory → fall back to the fast path only
        painter_newell_sancha_fast(model, face_count);
        return;
    }

    // Mark all buckets as empty (-1)
    for (int i = 0; i < HASH_SIZE; ++i) {
        hash_table[i] = -1;
    }

    // Capacity of the ordered-pair buffer
    // We limit it to avoid excessive memory usage on the IIGS
    int ordered_cap = face_count * 2;
    if (ordered_cap > 4096) ordered_cap = 4096;

    OrderedPair* ordered = (OrderedPair*)malloc(ordered_cap * sizeof(OrderedPair));
    int ordered_count = 0;

    if (!ordered) {
        free(hash_table);
        painter_newell_sancha_fast(model, face_count);
        return;
    }

    // -------------------------------------------------
    // Inconclusive pairs buffer (for diagnostics)
    // -------------------------------------------------
    // These pairs could not be ordered by any geometric test.
    // They are recorded so they can be inspected later.
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

    // -------------------------------------------------
    // Local pointers for faster access inside the loop
    // -------------------------------------------------
    Fixed32* z_min = faces->z_min;
    Fixed32* z_max = faces->z_max;
    int* minx = faces->minx;
    int* maxx = faces->maxx;
    int* miny = faces->miny;
    int* maxy = faces->maxy;

    // Simple hash function for a pair of face indices
    #define PAIR_HASH(a, b)  ((((unsigned)(a) * 73856093u) ^ ((unsigned)(b) * 19349663u)) & HASH_MASK)

    // -------------------------------------------------
    // 3. Correction passes (bubble-style)
    // -------------------------------------------------
    // We repeatedly scan adjacent pairs and swap them
    // when geometric tests prove the order is wrong.
    // The loop stops when a full pass produces no swaps.

    int swapped;

    do {
        swapped = 0;

        for (int i = 0; i < visible_count - 1; ++i) {
            int f1 = sorted[i];
            int f2 = sorted[i + 1];

            // -------------------------------------------------
            // Check whether this pair has already been decided
            // -------------------------------------------------
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
                continue;   // Skip already resolved pairs
            }

            // -------------------------------------------------
            // Test 1 : Depth separation (very cheap)
            // -------------------------------------------------
            // If the farthest point of f2 is in front of the
            // nearest point of f1, the current order is safe.
            if (z_max[f2] <= z_min[f1]) continue;

            // If the farthest point of f1 is in front of the
            // nearest point of f2, we must swap.
            if (z_max[f1] <= z_min[f2]) goto do_swap;

            // -------------------------------------------------
            // Test 2 & 3 : Axis-aligned bounding box overlap
            // -------------------------------------------------
            // If the projected bounding boxes do not overlap
            // on X or on Y, the faces cannot hide each other.
            if (maxx[f1] <= minx[f2] || maxx[f2] <= minx[f1]) continue;
            if (maxy[f1] <= miny[f2] || maxy[f2] <= miny[f1]) continue;

            // -------------------------------------------------
            // Test 4 : Projected polygon overlap
            // -------------------------------------------------
            if (!projected_polygons_overlap(model, f1, f2)) {
                continue;   // Bounding boxes overlap but polygons do not
            }

            // -------------------------------------------------
            // Tests 5 & 6 : Geometric plane / face relation
            // -------------------------------------------------
            int geo = geometric_face_relation(model, f1, f2);
            if (geo == -1) continue;          // Current order is correct
            if (geo == 1)  goto do_swap;      // Must swap

            // Try the opposite direction
            int geo2 = geometric_face_relation(model, f2, f1);
            if (geo2 == -1) goto do_swap;     // Opposite order is required

            // -------------------------------------------------
            // Inconclusive case
            // -------------------------------------------------
            // No test could decide the correct order.
            // We keep the current order and record the pair
            // for later diagnostics.
            if (inconclusive_pairs_count < inconclusive_pairs_capacity) {
                inconclusive_pairs[inconclusive_pairs_count].face1 = f1;
                inconclusive_pairs[inconclusive_pairs_count].face2 = f2;
                ++inconclusive_pairs_count;
            }

            // Remember this pair so we never test it again
            if (ordered_count < ordered_cap) {
                ordered[ordered_count].face1 = f1;
                ordered[ordered_count].face2 = f2;
                ordered[ordered_count].next  = hash_table[h];
                hash_table[h] = ordered_count;
                ++ordered_count;
            }
            continue;

        do_swap:
            // Perform the adjacent swap
            sorted[i]     = f2;
            sorted[i + 1] = f1;
            swapped = 1;

            // Record the new definitive ordering
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

    // -------------------------------------------------
    // Cleanup
    // -------------------------------------------------
    free(ordered);
    free(hash_table);
}

/*
 * painter_newell_sanchaV2
 * ========================
 *
 * Draw-order correction (depth sort) for the painter's algorithm,
 * inspired by Newell, Newell & Sancha (1972), "A New Approach to the
 * Shading of the Faces of Polyhedra" -- hence the name. The original
 * Newell-Newell-Sancha idea is: start from an approximate depth (Z)
 * sort, then apply a series of increasingly expensive tests (Z-interval
 * overlap, bounding-box overlap, relative position of the faces'
 * planes) to detect and fix cases where two faces end up in the wrong
 * order despite the approximate sort.
 *
 * TWO DELIBERATE SIMPLIFICATIONS compared to the original algorithm:
 *
 *   1. No face splitting. When the original Newell-Newell-Sancha
 *      algorithm cannot determine a valid order between two faces that
 *      genuinely intersect in 3D (none of the tests can decide), the
 *      "proper" fix is to split one of the two faces along the other's
 *      plane, producing two sub-faces that CAN then be ordered without
 *      ambiguity. This function does not do that: such a case is simply
 *      marked "inconclusive" (see below) and left in its current order,
 *      with no correction and no splitting.
 *
 *   2. No cycle detection. The original algorithm can, in rare
 *      configurations, produce circular dependencies (A must come
 *      before B, B before C, C before A) that require explicit
 *      detection and cycle-breaking (typically via the face splitting
 *      above). This function does not detect such cycles: it fixes
 *      pairs locally on the fly (do_move below) and relies on the
 *      do...while loop to converge to a stable state. In the presence
 *      of a genuine cycle, this loop may never fully satisfy every
 *      constraint at once -- this is an accepted trade-off for this
 *      project (see the rest of the DonyGS/ObjExplorer pipeline), not
 *      an oversight.
 *
 * Everything else in this function (Z sweep, cache of already-decided
 * pairs, persistent buffers) is pure performance optimization around
 * this same logic -- see the inline comments below for the details of
 * each step.
 */
void painter_newell_sancha(Model3D* model, int face_count)
{
    // Trivial case: nothing to correct with 0 or 1 face.
    if (face_count <= 1) {
        painter_newell_sancha_fast(model, face_count);
        return;
    }

    FaceArrays3D* faces = &model->faces;

    // -------------------------------------------------
    // 1. Initial fast sort + back-face culling
    // -------------------------------------------------
    // painter_newell_sancha_fast produces an APPROXIMATE depth order
    // (typically by average or minimum Z per face) and places visible
    // (non-culled) faces at the front of sorted[]. Everything below only
    // CORRECTS the errors in that approximate order where they are
    // visually significant -- it is not a second full sort.
    painter_newell_sancha_fast(model, face_count);

    // Recount how many faces are actually visible after culling:
    // sorted[0 .. visible_count-1] is the working range, the rest of
    // sorted[] (culled faces) is ignored by everything below.
    int visible_count = face_count;
    if (cull_back_faces) {
        visible_count = 0;
        for (int i = 0; i < face_count; ++i) {
            if (faces->display_flag[i]) {
                ++visible_count;
            }
        }
    }

    // Nothing to correct with fewer than 2 visible faces.
    if (visible_count < 2) return;

    // sorted[] IS the final result: this function modifies it in place,
    // via local permutations (see do_move below), until no correction
    // is needed anymore.
    int* sorted = faces->sorted_face_indices;

    // -------------------------------------------------
    // 2. Cache of already-decided pairs (hash table) + sweep buffers
    // -------------------------------------------------
    // All of these are PERSISTENT (static) working buffers, allocated
    // once on the very first call and then reused (and grown via
    // realloc if needed) on every subsequent call. Goal: avoid a
    // malloc()/free() every frame, while staying on the heap (no large
    // static array in BSS) so as not to compete with the model's
    // NewHandle-based memory.

    typedef struct {
        int face1;      // of the two faces in the pair, the one that must be drawn first (farther)
        int face2;      // the one that must be drawn second (closer)
        int next;       // next link in the hash collision chain
    } OrderedPair;

    typedef struct {
        int a;          // a candidate face pair per the Z sweep (see step 5)
        int b;
    } CandidatePair;

    #define HASH_SIZE_V2        2048
    #define HASH_MASK_V2        (HASH_SIZE_V2 - 1)
    #define ORDERED_MAX_CAP_V2  4096

    static int*          s_hash_table      = NULL; // hash table (fixed size, allocated once)
    static OrderedPair*  s_ordered         = NULL; // entries for already-decided pairs (correct or inconclusive)
    static int*          s_touched_buckets = NULL; // hash buckets touched during the current call (for a targeted reset at the end)
    static InconclusivePair* s_inconclusive_pairs = NULL; // "inconclusive" pairs for this call (for diagnostics/inspection)

    // --- Z-sweep scratch buffers, persistent + grow-on-demand -----------
    static int*  s_pos              = NULL; // pos[face_id] = current slot of this face in sorted[]
    static int   s_pos_capacity     = 0;    // sized on face_count (total faces in the model)
    static int*  s_by_zmin          = NULL; // copy of visible faces, sorted by ascending z_min
    static int   s_by_zmin_capacity = 0;    // sized on visible_count
    static int*  s_active           = NULL; // list of faces "active" during the sweep (see step 5)
    static int   s_active_capacity  = 0;    // sized on visible_count
    static CandidatePair* s_candidates = NULL; // list of face pairs that overlap in Z
    static int   s_candidates_capacity = 0;    // grows as needed (doubling)

    // One-time allocation of the hash table (fixed size, never resized)
    if (!s_hash_table) {
        s_hash_table = (int*)malloc(HASH_SIZE_V2 * sizeof(int));
        if (!s_hash_table) { painter_newell_sancha_fast(model, face_count); return; }
        // -1 means "empty bucket": mandatory initial state before any use.
        for (int i = 0; i < HASH_SIZE_V2; ++i) s_hash_table[i] = -1;
    }
    // One-time allocation of the decided-pairs array (fixed max capacity)
    if (!s_ordered) {
        s_ordered = (OrderedPair*)malloc(ORDERED_MAX_CAP_V2 * sizeof(OrderedPair));
        if (!s_ordered) { painter_newell_sancha_fast(model, face_count); return; }
    }
    // One-time allocation of the list of buckets to reset at the end of the call
    if (!s_touched_buckets) {
        s_touched_buckets = (int*)malloc(ORDERED_MAX_CAP_V2 * sizeof(int));
        if (!s_touched_buckets) { painter_newell_sancha_fast(model, face_count); return; }
    }
    // One-time allocation of the inconclusive-pairs diagnostic buffer.
    // Best-effort: if this fails, inconclusive_pairs_capacity is set to 0
    // below, exactly matching the original fallback behavior.
    if (!s_inconclusive_pairs) {
        s_inconclusive_pairs = (InconclusivePair*)malloc(ORDERED_MAX_CAP_V2 * sizeof(InconclusivePair));
    }

    // pos[] must be able to index any face id of the current model
    // (0 .. face_count-1) -- grow it if the current model has more
    // faces than previous calls anticipated.
    if (s_pos_capacity < face_count) {
        int newcap = face_count;
        int* tmp = (int*)realloc(s_pos, newcap * sizeof(int));
        if (!tmp) { painter_newell_sancha_fast(model, face_count); return; }
        s_pos = tmp;
        s_pos_capacity = newcap;
    }
    // by_zmin[] and active[] must each be able to hold up to
    // visible_count faces.
    if (s_by_zmin_capacity < visible_count) {
        int* tmp = (int*)realloc(s_by_zmin, visible_count * sizeof(int));
        if (!tmp) { painter_newell_sancha_fast(model, face_count); return; }
        s_by_zmin = tmp;
        s_by_zmin_capacity = visible_count;
    }
    if (s_active_capacity < visible_count) {
        int* tmp = (int*)realloc(s_active, visible_count * sizeof(int));
        if (!tmp) { painter_newell_sancha_fast(model, face_count); return; }
        s_active = tmp;
        s_active_capacity = visible_count;
    }

    int* hash_table   = s_hash_table;
    OrderedPair* ordered = s_ordered;
    int touched_count = 0; // number of buckets touched during THIS call (for the final targeted reset)

    // Logical cache capacity for this call: bounded both by
    // face_count*2 (never need more pairs than that in practice) and by
    // the physically allocated capacity (ORDERED_MAX_CAP_V2).
    int ordered_cap = face_count * 2;
    if (ordered_cap > ORDERED_MAX_CAP_V2) ordered_cap = ORDERED_MAX_CAP_V2;
    int ordered_count = 0; // number of entries actually used in ordered[] for this call

    // The diagnostic buffer is shared through a global variable (used
    // elsewhere, e.g. the debug inspector): point it at our persistent
    // buffer and reset the counter on every call.
    inconclusive_pairs = s_inconclusive_pairs;
    inconclusive_pairs_capacity = s_inconclusive_pairs ? ordered_cap : 0;
    inconclusive_pairs_count = 0;

    // Local pointers to the faces' geometry arrays, to avoid
    // dereferencing faces-> on every access inside the hot loops below.
    Fixed32* z_min = faces->z_min;
    Fixed32* z_max = faces->z_max;
    int* minx = faces->minx;
    int* maxx = faces->maxx;
    int* miny = faces->miny;
    int* maxy = faces->maxy;

    #define PAIR_HASH_V2(a, b)  ((((unsigned)(a) * 73856093u) ^ ((unsigned)(b) * 19349663u)) & HASH_MASK_V2)

    // -------------------------------------------------
    // 3. Build the reverse index pos[]
    // -------------------------------------------------
    // pos[face] gives the CURRENT position of `face` in sorted[].
    // Needed because the steps below reason by face id (coming from the
    // Z sweep, step 5), not by position -- and sorted[] changes as
    // corrections are applied (do_move, step 6), so pos[] must be kept
    // up to date at all times (see the update inside do_move).
    int* pos = s_pos;
    for (int idx = 0; idx < visible_count; ++idx) {
        pos[sorted[idx]] = idx;
    }

    // -------------------------------------------------
    // 4. Sort visible faces by ascending z_min (shell sort)
    // -------------------------------------------------
    // Prerequisite for the step-5 sweep: faces must be visited in order
    // of their minimum Z bound to detect [z_min, z_max] interval
    // overlaps in a single pass. Shell sort is chosen on purpose instead
    // of a recursive sort (quicksort): no recursion, hence no risk of
    // stack overflow on 65816/ORCA-C -- and it behaves well on input
    // that is already roughly sorted (which is the case here, since
    // sorted[] already comes out of the step-1 fast sort).
    int* by_zmin = s_by_zmin;
    for (int idx = 0; idx < visible_count; ++idx) {
        by_zmin[idx] = sorted[idx];
    }
    {
        int gap = 1;
        while (gap < visible_count / 3) gap = gap * 3 + 1; // Knuth-ish gap sequence
        for (; gap > 0; gap /= 3) {
            for (int idx = gap; idx < visible_count; ++idx) {
                int tmp = by_zmin[idx];
                Fixed32 tmp_zmin = z_min[tmp];
                int k = idx;
                while (k >= gap && z_min[by_zmin[k - gap]] > tmp_zmin) {
                    by_zmin[k] = by_zmin[k - gap];
                    k -= gap;
                }
                by_zmin[k] = tmp;
            }
        }
    }

    // -------------------------------------------------
    // 5. Sweep: enumerate every face pair that overlaps in Z, exactly
    //    once.
    // -------------------------------------------------
    // This is the equivalent of "Test 1" (depth separation) from the
    // Newell-Newell-Sancha algorithm, but found in O(n log n + k)
    // instead of testing all n*(n-1)/2 pairs one by one: we sweep F
    // along the Z axis (ascending z_min order) while maintaining an
    // "active" list of faces whose [z_min, z_max] interval hasn't ended
    // yet. Any face still active when we reach F necessarily overlaps F
    // in Z (since its z_max is still ahead of F's z_min). k = the number
    // of pairs that actually overlap, typically a small fraction of the
    // total on a typical scene.
    int* active = s_active;
    int active_count = 0;
    int candidate_count = 0;

    for (int idx = 0; idx < visible_count; ++idx) {
        int F = by_zmin[idx];
        Fixed32 f_zmin = z_min[F];

        // Drop finished intervals from the active list (their z_max can
        // no longer overlap a future z_min, since z_min only increases
        // from here on).
        int w = 0;
        for (int a = 0; a < active_count; ++a) {
            if (z_max[active[a]] > f_zmin) {
                active[w++] = active[a];
            }
        }
        active_count = w;

        // Every face still active overlaps F in Z: record the candidate
        // pair (it will be re-tested against the finer criteria --
        // bbox, polygons, planes -- in step 6).
        for (int a = 0; a < active_count; ++a) {
            if (s_candidates_capacity <= candidate_count) {
                // List full: double its capacity (realloc, never freed
                // between calls -- see the note at the top of the file
                // about persistent buffers).
                int newcap = s_candidates_capacity > 0 ? s_candidates_capacity * 2 : 256;
                CandidatePair* tmp = (CandidatePair*)realloc(s_candidates, newcap * sizeof(CandidatePair));
                if (!tmp) {
                    // Out of memory for the candidate list: fall back to
                    // the plain fast sort for this call only (no fine
                    // correction this time, but no crash either).
                    painter_newell_sancha_fast(model, face_count);
                    return;
                }
                s_candidates = tmp;
                s_candidates_capacity = newcap;
            }
            s_candidates[candidate_count].a = active[a];
            s_candidates[candidate_count].b = F;
            ++candidate_count;
        }

        // F in turn becomes active for the faces that follow in the sweep.
        active[active_count++] = F;
    }

    CandidatePair* candidates = s_candidates;

    // -------------------------------------------------
    // 6. Correction passes
    // -------------------------------------------------
    // For each candidate pair (from the Z sweep above), apply the same
    // criteria as the original Newell-Newell-Sancha algorithm, from
    // cheapest to most expensive: bounding-box overlap, actual overlap
    // of the projected polygons, then the relative position of the two
    // faces' planes. If the current order is wrong, move the offending
    // face (do_move). Repeat as long as a full pass still produces at
    // least one change (do...while) -- necessary because moving one
    // face can reveal or resolve other violations elsewhere in
    // sorted[].
    //
    // Reminder of the two simplifications assumed here (see the header
    // comment at the top of the file): an "inconclusive" case (none of
    // the tests can decide, typically two faces that genuinely
    // intersect) is NOT resolved by face splitting -- it is simply left
    // in its current order and marked so it won't be re-tested. Likewise,
    // no explicit cycle detection is performed: if several pairs'
    // constraints contradict each other, the loop below may converge to
    // a state that doesn't satisfy every constraint at once -- this is
    // an accepted trade-off for this project, not an oversight.
    int changed;

    do {
        changed = 0;

        for (int c = 0; c < candidate_count; ++c) {
            int A = candidates[c].a;
            int B = candidates[c].b;

            // Determine which of the two is currently drawn first (P,
            // position i) and which is drawn second (Q, position j) --
            // this is a property of THEIR CURRENT POSITION in sorted[],
            // which can change from one pass to the next as other
            // pairs' moves (do_move) shift things around.
            int i, j, P, Q;
            if (pos[A] < pos[B]) { i = pos[A]; j = pos[B]; P = A; Q = B; }
            else                 { i = pos[B]; j = pos[A]; P = B; Q = A; }

            // -------------------------------------------------
            // Has this pair already been decided (correct or
            // inconclusive) in a previous pass? If so, no need to redo
            // the geometric tests -- the verdict only depends on the
            // two faces' geometry, not on their position.
            // -------------------------------------------------
            unsigned h = PAIR_HASH_V2(P, Q);
            int idx2 = hash_table[h];
            int already_known = 0;

            while (idx2 >= 0) {
                if ((ordered[idx2].face1 == P && ordered[idx2].face2 == Q) ||
                    (ordered[idx2].face1 == Q && ordered[idx2].face2 == P)) {
                    already_known = 1;
                    break;
                }
                idx2 = ordered[idx2].next;
            }
            if (already_known) continue;

            // Test 1 (Z separation) is already guaranteed by
            // construction here: this pair comes from the step-5 sweep,
            // which only enumerated pairs whose Z intervals overlap. No
            // need to re-check it.

            // Test 2 & 3: 2D bounding-box overlap (cheap rejection
            // before the more expensive polygon test)
            if (maxx[P] <= minx[Q] || maxx[Q] <= minx[P]) continue;
            if (maxy[P] <= miny[Q] || maxy[Q] <= miny[P]) continue;

            // Test 4: actual overlap of the polygons as projected on screen
            if (!projected_polygons_overlap(model, P, Q)) continue;

            // Tests 5 & 6: position of Q relative to P's plane.
            // geo == -1: the current order (P before Q) is correct
            // geo ==  1: must invert (Q must be drawn before P)
            // geo ==  0: undetermined (see the "inconclusive" handling below)
            int geo = geometric_face_relation(model, P, Q);

            if (geo == -1) {
                // Order confirmed correct: remember it so this pair is
                // never tested again.
                if (ordered_count < ordered_cap) {
                    ordered[ordered_count].face1 = P;
                    ordered[ordered_count].face2 = Q;
                    ordered[ordered_count].next  = hash_table[h];
                    hash_table[h] = ordered_count;
                    s_touched_buckets[touched_count++] = h;
                    ++ordered_count;
                }
                continue;
            }

            if (geo == 1) {
                // Unambiguous verdict in the P->Q direction: must invert.
                goto do_move;
            }

            // The P->Q test couldn't decide: try the opposite direction.
            {
                int geo2 = geometric_face_relation(model, Q, P);
                if (geo2 == -1) {
                    // The reverse order is required.
                    goto do_move;
                }
            }

            // -------------------------------------------------
            // "Inconclusive" case: neither P->Q nor Q->P can decide
            // (typically, the two faces genuinely intersect in 3D). As
            // noted at the top of the file, this function does not
            // split faces to resolve this case: the current order is
            // simply left as-is, the pair is recorded for diagnostics
            // (inconclusive_pairs, used by the debug inspector), and it
            // is marked "already decided" so it won't be needlessly
            // re-tested in later passes.
            // -------------------------------------------------
            if (inconclusive_pairs_count < inconclusive_pairs_capacity) {
                inconclusive_pairs[inconclusive_pairs_count].face1 = P;
                inconclusive_pairs[inconclusive_pairs_count].face2 = Q;
                ++inconclusive_pairs_count;
            }
            if (ordered_count < ordered_cap) {
                ordered[ordered_count].face1 = P;
                ordered[ordered_count].face2 = Q;
                ordered[ordered_count].next  = hash_table[h];
                hash_table[h] = ordered_count;
                s_touched_buckets[touched_count++] = h;
                ++ordered_count;
            }
            continue;

        do_move:
            // Move Q (currently at position j) to just before P
            // (position i): shift everything between i and j-1 one slot
            // to the right, then place Q at position i. pos[] is kept in
            // sync for every shifted slot so it always matches sorted[].
            {
                int tmp = sorted[j];
                for (int k = j; k > i; --k) {
                    sorted[k] = sorted[k - 1];
                    pos[sorted[k]] = k;
                }
                sorted[i] = tmp;
                pos[tmp] = i;
            }

            // A change happened: the do...while loop will need at least
            // one more full pass to check that this move didn't break
            // anything elsewhere.
            changed = 1;

            // Remember the now-correct reversed order (Q before P), so
            // this pair is never tested again.
            if (ordered_count < ordered_cap) {
                unsigned h2 = PAIR_HASH_V2(Q, P);
                ordered[ordered_count].face1 = Q;
                ordered[ordered_count].face2 = P;
                ordered[ordered_count].next  = hash_table[h2];
                hash_table[h2] = ordered_count;
                s_touched_buckets[touched_count++] = h2;
                ++ordered_count;
            }
        }
    } while (changed);

    // -------------------------------------------------
    // 7. Cleanup: only reset the hash-table buckets actually touched
    //    during this call (instead of all HASH_SIZE_V2 entries), so the
    //    table is fully back to -1 for the next call -- exactly as if
    //    it had been entirely cleared, but at a cost proportional to the
    //    number of pairs actually processed.
    // -------------------------------------------------
    for (int t = 0; t < touched_count; ++t) {
        hash_table[s_touched_buckets[t]] = -1;
    }
}

// --- Pair cache (used by painter_geoV2) ---
static PairCache* pair_cache_create(int capacity) {
    PairCache* c = (PairCache*)malloc(sizeof(PairCache));
    if (!c) return NULL;
    c->capacity = capacity;
    c->slots = (PairCacheEntry*)calloc(capacity, sizeof(PairCacheEntry));
    if (!c->slots) { free(c); return NULL; }
    return c;
}

static int pair_cache_slot(PairCache* c, int f1, int f2) {
    // Canonical key independent of order
    int a = (f1 < f2) ? f1 : f2;
    int b = (f1 < f2) ? f2 : f1;
    unsigned int h = (unsigned int)(a * 2654435761u ^ b * 2246822519u);
    return (int)(h % (unsigned int)c->capacity);
}

static int pair_cache_find(PairCache* c, int f1, int f2) {
    int slot, i;
    if (!c) return 0;
    slot = pair_cache_slot(c, f1, f2);
    for (i = 0; i < 8; i++) {
        int pos = (slot + i) % c->capacity;
        PairCacheEntry* e = &c->slots[pos];
        if (!e->occupied) return 0;  // slot vide -> paire inconnue
        if ((e->face1 == f1 && e->face2 == f2)) return  e->relation;
        if ((e->face1 == f2 && e->face2 == f1)) return -e->relation;
    }
    return 0;
}

static void pair_cache_insert(PairCache* c, int f1, int f2, int8_t relation) {
    int slot, i;
    if (!c) return;
    slot = pair_cache_slot(c, f1, f2);
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

static void pair_cache_destroy(PairCache* c) {
    if (c) { free(c->slots); free(c); }
}

// --- Painter variants ---

/* PAIR_CACHE_MAX_BYTES : budget mémoire accepté pour le cache de paires
 * (fixé par le propriétaire du projet à 128 Ko). */
#define PAIR_CACHE_MAX_BYTES (128 * 1024)

// ADDED: Utility functions for calculating the next power of 2 and the maximum cache capacity based on the memory budget.
/* Plus petite puissance de 2 >= n. Utilisée pour garantir que la capacité
 * du cache est toujours une puissance de 2, condition nécessaire pour
 * remplacer le modulo (%) par un masque bit à bit (&) dans pair_cache_slot/
 * find/insert sans changer aucun résultat. */
static int pair_cache_next_pow2(int n) {
    int p = 1;
    while (p < n) p <<= 1;
    return p;
}

/* Plus grande puissance de 2 <= n. Utilisée uniquement pour calculer le
 * nombre maximal d'entrées tenant dans PAIR_CACHE_MAX_BYTES. */
static int pair_cache_next_pow2_floor(int n) {
    int p = 1;
    while (p * 2 <= n) p <<= 1;
    return p;
}

/* Nombre maximal d'entrées du cache tenant dans PAIR_CACHE_MAX_BYTES,
 * arrondi à la puissance de 2 inférieure (pair_cache_create arrondit lui
 * à la puissance de 2 SUPÉRIEURE en interne - ici on veut la borne haute
 * côté appelant, donc on descend, pour ne jamais dépasser le budget une
 * fois pair_cache_create() appliqué son propre arrondi). sizeof() est
 * évalué par le compilateur : ce calcul reste correct quelle que soit la
 * taille réelle de int et le padding de la structure sur cette
 * plateforme. */
static int pair_cache_max_capacity_for_budget(void) {
    int max_entries = PAIR_CACHE_MAX_BYTES / (int)sizeof(PairCacheEntry);
    return pair_cache_next_pow2_floor(max_entries);
}

void painter_geoV2(Model3D* model, int face_count) {
    FaceArrays3D* faces = &model->faces;
    int i;

    Fixed32* face_zmean = faces->z_mean;
    if (!face_zmean) return;

    Fixed32* face_zmin = faces->z_min;   /* may be NULL */
    Fixed32* face_zmax = faces->z_max;

    // printf("Running painter_geoV2 with %d faces (cull_back_faces=%d)...\n",
    //        face_count, cull_back_faces);

    painter_newell_sancha_fast(model, face_count);

    int visible_count = face_count;
    if (cull_back_faces) {
        visible_count = 0;
        for (i = 0; i < face_count; ++i) {
            if (faces->display_flag[i]) ++visible_count;
        }
    }

    // if (visible_count < 2) {
    //     printf("Total swaps: 0\n");
    //     return;
    // }

    int swap_count = 0;
    int swapped;

    /* ------------------------------------------------------------------
       Fixed-size hash cache.
       Independent of face_count → predictable memory use and high
       chance of successful allocation even for 6000-face models.
       4096 entries is largely sufficient for the number of unique
       pairs actually touched by the adjacent bubble.
       ------------------------------------------------------------------ */
    PairCache* cache = pair_cache_create(4096);
    if (!cache) {
        /* Last-chance smaller sizes */
        cache = pair_cache_create(2048);
        if (!cache)
            cache = pair_cache_create(1024);
    }
    if (!cache) {
        printf("painter_geoV2: warning - no pair cache, geometric pass disabled\n");
        return;   /* better to keep the initial sort than run extremely slowly */
    }

    /* Inconclusive buffer */
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

    do {
        swapped = 0;
        new_limit = 0;
        ++pass;

        for (i = 0; i < limit; ++i) {
            int f1 = faces->sorted_face_indices[i];
            int f2 = faces->sorted_face_indices[i + 1];

            if (pair_cache_find(cache, f1, f2) != 0)
                continue;

            /* Geometric tests – source of truth */
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

            /* Z test – only to skip the expensive ray-cast */
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

            /* Inconclusive */
            if (inconclusive_pairs &&
                inconclusive_pairs_count < inconclusive_pairs_capacity) {
                inconclusive_pairs[inconclusive_pairs_count].face1 = f1;
                inconclusive_pairs[inconclusive_pairs_count].face2 = f2;
                inconclusive_pairs_count++;
            }
            pair_cache_insert(cache, f1, f2, 1);
            continue;

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

    pair_cache_destroy(cache);

    // printf("Total swaps: %d (passes: %d)\n", swap_count, pass);
}


/* painter_correct
 * ----------------
 * Deterministically adjust `faces->sorted_face_indices` in-place using the
 * exact geometric plane tests performed by `pair_plane_before` and `pair_plane_after`.
 *
 * Algorithm overview (safe & conservative):
 *  - First, partition faces into back-faces (plane_d <= 0) and front-faces (plane_d > 0)
 *    while preserving z_mean order within each group.
 *  - For each face id `target` within its partition, run two checks:
 *      1) Inspect faces *before* `target` in the current order. If one or more
 *         of these faces are *misplaced* (the pairwise plane tests indicate they
 *         should be after `target`) AND their projected polygons *overlap*
 *         (strict 2D overlap; touching-only is NOT overlap here), then move
 *         `target` to be *before* the earliest such misplaced face.
 *      2) Inspect faces *after* `target`. If one or more of these faces are
 *         misplaced (they should be before `target`) AND overlap with `target`,
 *         move `target` to be *after* the latest such misplaced face.
 *
 * Performance & correctness notes:
 *  - We first perform a fast axis-aligned bounding-box (AABB) rejection for
 *    each candidate pair. Only if bounding boxes intersect do we call the
 *    heavier `projected_polygons_overlap` (edge-intersection + containment)
 *    and the pairwise plane tests (`pair_plane_before`/`pair_plane_after`).
 *  - `projected_polygons_overlap` treats touching-only cases (shared edge or
 *    single vertex) as NON-overlap; this is intentional and consistent with
 *    the rest of the program.
 *  - The routine only performs local moves that are conservative with respect
 *    to the pair tests — it does not attempt global reordering heuristics.
 *  - Back-faces are corrected only if backface culling is OFF; otherwise they
 *    won't be rendered and correction would be wasted effort.
 *
 * Return value: number of element moves applied to `sorted_face_indices`.
 */
static int painter_correct(Model3D* model, int face_count, int debug) {
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    if (face_count <= 0) return 0;

    int old_cull = cull_back_faces;
    cull_back_faces = 0; // needed by painter_newell_sancha_fast to ensure all faces are included in the initial sort and partitioning; we'll handle back-face correction regardless of culling mode
    painter_newell_sancha_fast(model, face_count);


    int moves = 0;
    int n = face_count;

    /* Partition faces: back-faces (plane_d <= 0) go to beginning, front-faces after.
     * Maintain z_mean order within each partition using stable partitioning.
     * Both groups will be independently corrected (back-faces only if culling OFF).
     * This separation ensures plane equation tests remain valid within each group. */
    int *temp_indices = (int*)malloc(n * sizeof(int)); if (!temp_indices) { printf("Error: painter_correct malloc temp_indices failed\n"); return 0; }
    int back_idx = 0, front_idx = 0;

    /* First pass: count and separate */
    for (int i = 0; i < n; ++i) {
        int f = faces->sorted_face_indices[i];
        if (faces->plane_d[f] <= 0) {
            temp_indices[back_idx++] = f;  /* back-faces at beginning of temp_indices (starting at index 0) */
        }
    }
    int front_start_pos = back_idx;  /* Mark where front-faces start */
    for (int i = 0; i < n; ++i) {
        int f = faces->sorted_face_indices[i];
        if (faces->plane_d[f] > 0) {
            temp_indices[front_start_pos + front_idx++] = f;  /* front-faces after back-faces in temp_indices */
        }
    }
    /* Copy back to sorted_face_indices */
    for (int i = 0; i < n; ++i) faces->sorted_face_indices[i] = temp_indices[i];
    free(temp_indices);

    /* Inverse map for quick position lookup: face_id -> index in sorted_face_indices */
    int *pos_of_face = (int*)malloc(n * sizeof(int)); if (!pos_of_face) { printf("Error: painter_correct malloc pos_of_face failed\n"); return 0; }
    for (int i = 0; i < n; ++i) pos_of_face[faces->sorted_face_indices[i]] = i;
    /* Per-target minimal allowed index once moved: -1 means not moved yet. */
    int *min_allowed_pos = (int*)malloc(n * sizeof(int)); if (!min_allowed_pos) { free(pos_of_face); printf("Error: painter_correct malloc min_allowed_pos failed\n"); return 0; }
    for (int i = 0; i < n; ++i) min_allowed_pos[i] = -1;

    // if culling is on we skip directly to correcting front-faces 
    // since back-faces won't be rendered and correction would be wasted effort

    /* Two independent passes: 
     * 1) Correct back-faces order within themselves (indices 0 to front_start_pos-1)
     * 2) Correct front-faces order within themselves (indices front_start_pos to n-1)
     * This ensures back-faces stay before front-faces while both groups benefit from corrections. */


    // we can skip the entire first pass since back-faces are already sorted by painter_newell_sancha_fast 
    // use painter_correctV2 for a better back faces sorting 
    // that doesn't rely only on a basic Z sorting.
    goto after_pass1;

    /* PASS 1: Correct back-faces (plane_d <= 0) - only if culling is OFF */
    printf("Back faces:");
    if (old_cull == 0) { /* Skip this pass entirely if culling is ON (back-faces won't be displayed) */
        for (int target = 0; target < face_count; ++target) {
            if (faces->plane_d[target] > 0) continue; /* Skip front-faces in this pass */

        int pos = pos_of_face[target];
        if (pos < 0 || pos >= front_start_pos) continue; /* Must be in back-face section */

        /* 1) BEFORE test: check back-faces placed before target */
        int best_before = -1;
        for (int i = 0; i < pos; ++i) {
            int f = faces->sorted_face_indices[i];
            if (i >= front_start_pos) break; /* Don't cross into front-face section */

            /* depth (Z) quick rejection (cheap): if Z ranges do not overlap skip */
            if (faces->z_max[f] <= faces->z_min[target]) continue;
            if (faces->z_max[target] <= faces->z_min[f]) continue;

            /* bounding-box quick rejection (very cheap): if AABBs don't intersect we
             * skip the expensive overlap check entirely. Touching-only bbox cases
             * will pass here and be filtered by the overlap test (which treats
             * touching-only as NON-overlap).
             */
            if (faces->maxx[f] <= faces->minx[target]) continue;
            if (faces->maxx[target] <= faces->minx[f]) continue;
            if (faces->maxy[f] <= faces->miny[target]) continue;
            if (faces->maxy[target] <= faces->miny[f]) continue;    

            /* require actual projected polygon overlap before considering swap
             * (projected_polygons_overlap performs proper edge intersection and
             * containment tests; expensive but necessary for correctness).
             */
            if (!projected_polygons_overlap_simple(model, f, target)) continue;

            /* Use specialized plane-only 'after' test for BEFORE loop: quick check
             * to see if `f` is geometrically after (in front of) `target`.
             */
            if (pair_plane_after(model, f, target)) {
                if (best_before == -1 || i < best_before) best_before = i;
            }
        }
        if (best_before != -1) {
            /* move target to be BEFORE best_before (use optimized variant updating inverse map) */
            moves += move_element_remove_and_insert_pos(faces->sorted_face_indices, face_count, pos, best_before, pos_of_face);
            /* update pos to new location via inverse map */
            pos = pos_of_face[target];
            /* record minimal allowed index for this target (disallow later moves placing it at indices > pos) */
            if (min_allowed_pos[target] == -1 || pos < min_allowed_pos[target]) min_allowed_pos[target] = pos;
        }

        /* 2) AFTER test: check back-faces placed after target (within back-face section) */
        pos = pos_of_face[target];
        if (pos < 0 || pos >= front_start_pos) continue;
        int best_after = -1;
        for (int i = pos + 1; i < front_start_pos; ++i) { /* Stay within back-face section */
            int f = faces->sorted_face_indices[i];

            /* depth (Z) quick rejection (cheap) */
            if (faces->z_max[f] <= faces->z_min[target]) continue;
            if (faces->z_max[target] <= faces->z_min[f]) continue;

            /* bounding-box quick rejection (cheap) */
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
            /* Ensure we don't move into front-face section */
            if (insert_idx >= front_start_pos) insert_idx = front_start_pos - 1;
            moves += move_element_remove_and_insert_pos(faces->sorted_face_indices, face_count, pos, insert_idx, pos_of_face);
            pos = pos_of_face[target];
            if (min_allowed_pos[target] == -1 || pos < min_allowed_pos[target]) min_allowed_pos[target] = pos;
        }
        
        printf(" %d",target);
        }
    } /* End of PASS 1 (back-faces) */

    after_pass1: ;
    /* PASS 2: Correct front-faces (plane_d > 0) */
    printf("\nFront faces:");
    for (int target = 0; target < face_count; ++target) {
        if (faces->plane_d[target] <= 0) continue; /* Skip back-faces in this pass */

        int pos = pos_of_face[target];
        if (pos < 0 || pos < front_start_pos) continue; /* Must be in front-face section */

        /* 1) BEFORE test: check front-faces placed before target (don't test back-faces) */
        int best_before = -1;
        for (int i = front_start_pos; i < pos; ++i) { /* Start from front-face section */
            int f = faces->sorted_face_indices[i];

            /* depth (Z) quick rejection (cheap) */
            if (faces->z_max[f] <= faces->z_min[target]) continue;
            if (faces->z_max[target] <= faces->z_min[f]) continue;

            /* bounding-box quick rejection (cheap) */
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
            /* Ensure best_before is not in back-face section */
            if (best_before < front_start_pos) best_before = front_start_pos;
            moves += move_element_remove_and_insert_pos(faces->sorted_face_indices, face_count, pos, best_before, pos_of_face);
            pos = pos_of_face[target];
            if (min_allowed_pos[target] == -1 || pos < min_allowed_pos[target]) min_allowed_pos[target] = pos;
        }

        /* 2) AFTER test: check front-faces placed after target */
        pos = pos_of_face[target];
        if (pos < 0 || pos >= n) continue;
        int best_after = -1;
        for (int i = pos + 1; i < face_count; ++i) {
            int f = faces->sorted_face_indices[i];
            /* Note: loop goes to face_count which may include back-faces, but they
             * have plane_d <= 0 so they won't interfere with front-face tests */

            /* depth (Z) quick rejection (cheap) */
            if (faces->z_max[f] <= faces->z_min[target]) continue;
            if (faces->z_max[target] <= faces->z_min[f]) continue;

            /* bounding-box quick rejection (cheap) */
            // if (faces->maxx[f] <= faces->minx[target] || faces->maxx[target] <= faces->minx[f]
            //     || faces->maxy[f] <= faces->miny[target] || faces->maxy[target] <= faces->miny[f]) continue;
            if (faces->maxx[f] <= faces->minx[target]) continue;
            if (faces->maxx[target] <= faces->minx[f]) continue;
            if (faces->maxy[f] <= faces->miny[target]) continue;

            /* require actual projected polygon overlap before considering swap
             * (the overlap test is expensive; we avoid it whenever bbox rejects).
             */
            if (!projected_polygons_overlap_simple(model, f, target)) continue;
            /* Use specialized plane-only 'before' test for AFTER loop: quick check
             * to see if `f` is geometrically before `target`.
             */
            if (pair_plane_before(model, f, target)) { best_after = i; /* wants the largest index */ }

            
        }
        if (best_after != -1) {
            /* We want to move target AFTER best_after. Compute insertion index after removal: */
            int insert_idx;
            if (best_after < pos) insert_idx = best_after + 1; else insert_idx = best_after; /* as analyzed */
            /* If target was previously moved forward, clamp insertion so it cannot be placed at an index greater than its minimal reached index */
            if (min_allowed_pos[target] != -1 && insert_idx > min_allowed_pos[target]) insert_idx = min_allowed_pos[target];
            /* perform remove and insert (use optimized variant to update inverse map) */
            moves += move_element_remove_and_insert_pos(faces->sorted_face_indices, face_count, pos, insert_idx, pos_of_face);
            /* update recorded minimal position in case the move moved it even earlier */
            pos = pos_of_face[target];
            if (min_allowed_pos[target] == -1 || pos < min_allowed_pos[target]) min_allowed_pos[target] = pos;
        }
        
        printf(" %d",target);
    }

    cull_back_faces = old_cull;
    free(pos_of_face); free(min_allowed_pos);
    return moves;
}

/* painter_correctV2
 * ------------------
 * Version V2: Three-pass algorithm with partitioning and cross-section boundary test
 * 1. Partitions faces like painter_correct (back-faces / front-faces)
 * 2. PASS 1: Correct back-faces among themselves
 * 3. PASS 2: Correct front-faces among themselves  
 * 4. PASS 3 (only if culling OFF): Cross-section test between back/front boundary
 *    - For each back-face, test overlap with ALL front-faces
 *    - If overlap + back-face closer (z_mean) → move back-face after front-face
 */
static int painter_correctV2(Model3D* model, int face_count, int debug) {
    /*
     * painter_correctV2: experimental variation of painter_correct that keeps the
     * same two-pass local correction (back-faces then front-faces) but adds
     * diagnostics and limited corrective actions when culling is OFF.
     *
     * NOTE: The following comments aim to document intent and control flow only.
     *       No behavior is changed — this is strictly explanatory text.
     *
     * Summary of algorithmic steps (informative):
     *  1) Seed an initial ordering using the full Newell-Sancha sort (`painter_bubble_sort`) (more thorough than the fast variant)
     *  2) Partition faces into BACK (plane_d <= 0) then FRONT (plane_d > 0)
     *     and run local corrective passes inside each partition (Pass 1 for backs,
     *     Pass 2 for fronts). These passes perform only local swaps and rely on
     *     cheap quick-rejects (z-min/max, bbox on X/Y) before invoking
     *     expensive geometry tests (projected overlap, plane tests).
     *  3) If back-face culling was disabled: snapshot the result of passes 1+2,
     *     perform a Z-only global sort, and scan the nearest faces backward to
     *     locate BACK faces that actually appear near the front of the scene.
     *     For each such BACK face, we search for overlapping FRONT faces and
     *     attempt to resolve pair ordering using plane-based geometric tests;
     *     if geometric tests are inconclusive, z_mean is used as a deterministic
     *     fallback. Only local, minimal moves are applied to the snapshot.
     *
     * Rationale: the function is intentionally conservative — it avoids global
     *            reorders and prefers deterministic, local changes to preserve
     *            stability of the painter's output in most cases.
     */
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    if (face_count <= 0) return 0;

    /* Save and disable culling locally so cross-partition cases are considered. */
    int old_cull = cull_back_faces;
    cull_back_faces = 0;

    /* Seed initial ordering using the full Newell-Sancha sort (`painter_bubble_sort`).
     * Rationale: the full sort performs more extensive per-face computation (plane
     * conversions, bounding boxes and z_mean) which reduces inconclusive geometric
     * tests later during local corrections. This improves correctness at the cost
     * of increased CPU work compared to a lightweight 'fast' seed.
     */
    // painter_bubble_sort(model, face_count);
    painter_newell_sancha_fast(model, face_count); 
    int moves = 0;
    int n = face_count;

    // Partition faces: back-faces (plane_d <= 0) go to beginning, front-faces after
    // This mirrors painter_correct's partition so that passes 1/2 operate within
    // homogenous groups (back-only then front-only) which simplifies plane tests.
    int *temp_indices = (int*)malloc(n * sizeof(int)); 
    if (!temp_indices) { printf("Error: painter_correctV2 malloc temp_indices failed\n"); return 0; }
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

    int *pos_of_face = (int*)malloc(n * sizeof(int)); if (!pos_of_face) { printf("Error: painter_correctV2 malloc pos_of_face failed\n"); return 0; }
    for (int i = 0; i < n; ++i) pos_of_face[faces->sorted_face_indices[i]] = i;
    int *min_allowed_pos = (int*)malloc(n * sizeof(int)); if (!min_allowed_pos) { free(pos_of_face); printf("Error: painter_correctV2 malloc min_allowed_pos failed\n"); return 0; }
    for (int i = 0; i < n; ++i) min_allowed_pos[i] = -1;

    printf("Back faces:");
    // PASS 1: Correct back-faces (plane_d <= 0) - only if culling is OFF
    // This pass attempts local moves within the back-face partition using
    // cheap quick-rejects followed by more expensive geometric tests.
    // Rationale: we iterate each BACK face and scan nearby faces inside the
    // partition. We only perform local swaps (to preserve stability) and we
    // always prefer the cheaper z/bbox tests before calling projected overlap
    // and plane-based tests which are heavier.
    if (old_cull == 0) {
        for (int target = 0; target < face_count; ++target) {
            printf(" %d",target);
            if (faces->plane_d[target] > 0) continue;
            int pos = pos_of_face[target];
            if (pos < 0 || pos >= front_start_pos) continue;
            int best_before = -1; // search for a preceding face 'f' that should be after target
            // Order of checks: cheap z/bbox quick-rejects first; only then call
            // projected_polygons_overlap() (expensive) and finally plane-based test.
            for (int i = 0; i < pos; ++i) {
                int f = faces->sorted_face_indices[i];
                if (i >= front_start_pos) break;
                if (faces->z_max[f] <= faces->z_min[target]) continue;
                if (faces->z_max[target] <= faces->z_min[f]) continue;
                if (faces->maxx[f] <= faces->minx[target]) continue;
                if (faces->maxx[target] <= faces->minx[f]) continue;
                if (faces->maxy[f] <= faces->miny[target]) continue;
                if (faces->maxy[target] <= faces->miny[f]) continue;
                // printf("here");
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
            int best_after = -1; // search for a succeeding face 'f' that should be before target
            // Same quick-reject ordering as above to avoid heavy geometry checks when possible.
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

    // PASS 2: Correct front-faces (plane_d > 0)
    // Symmetric to PASS 1 but operating on front-face partition; same quick
    // rejection and plane-based logic is applied to refine local ordering.
    // Note: identical conservative checks are applied to avoid unexpected
    // global reorderings; we only accept local moves that are supported by
    // decisive geometrical or depth tests.
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

    // PASS 3: Cross-section test between back/front boundary (only if culling is OFF)

    /*
     * Rationale:
     *  - painter_correct separates backs and fronts; when culling is OFF a BACK
     *    face can legitimately be physically in front of the scene and overlap
     *    FRONT faces. Such cases require cross-partition handling.
     * Approach:
     *  - Save a snapshot of the ordering produced by passes 1+2 (so we can modify
     *    locally without losing the original two-pass result if we choose to).
     *  - Run a fast Z-only sort to get a global depth ordering, then scan from
     *    nearest faces back until a FRONT face is encountered.
     *  - For each BACK face encountered (i.e., back-face appearing near the front):
     *     * find overlapping FRONT faces (bbox quick reject + projected overlap)
     *     * try plane-based geometric tests between the BACK and each FRONT
     *       (preferential decision). If inconclusive, fallback to comparing
     *       z_mean as a deterministic tie-breaker.
     *     * upon a decisive result, perform a local move in the snapshot ordering
     *       so that the final ordering respects the concrete geometric decision.
     */
    if (old_cull == 0) {
        int *snapshot = (int*)malloc(n * sizeof(int));
        if (!snapshot) { free(pos_of_face); free(min_allowed_pos); printf("Error: painter_correctV2 malloc snapshot failed\n"); return 0; }
        for (int i = 0; i < n; ++i) snapshot[i] = faces->sorted_face_indices[i];

        /* Z-only sort (fast) */
        // painter_bubble_sort(model, face_count);
        // faster, but ???
        painter_newell_sancha_fast(model, face_count);

        printf("\n\nCross-section back-face checks:");
        /* FILE *logf = fopen("incfaces.log", "a"); */
        FILE *logf = NULL; /* logging disabled */
        int total_scanned = 0, total_logged = 0, total_ov = 0, total_tch = 0, total_none = 0, total_moves = 0;
        /* Scan from highest index downwards */
        for (int idx = n - 1; idx >= 0; --idx) {
            int bf = faces->sorted_face_indices[idx];
            total_scanned++;
            /* If we hit a front-face, stop and restore original order */
            if (faces->plane_d[bf] > 0) {
                break;
            }

            total_logged++;
            int orig_pos = pos_of_face[bf];
            /*(logf) fprintf(logf, "BACK idx=%d pos=%d face=%d zmean=%.6f ", idx, orig_pos, bf, FIXED_TO_FLOAT(faces->z_mean[bf])); */
            /* Find front faces that overlap or touch this back-face */
            int found = 0;
            int per_ov = 0, per_tch = 0;
            for (int j = 0; j < n; ++j) {
                int ff = faces->sorted_face_indices[j];
                // printf("\n  => testing ff=%d", ff);

                if (faces->plane_d[ff] <= 0) continue; /* only front faces */
                /* Z quick reject */
                if (faces->z_max[ff] <= faces->z_min[bf]) continue;
                if (faces->z_max[bf] <= faces->z_min[ff]) continue;
                /* bbox tests (strict and non-strict to detect touching-only) */
                int bbox_strict = !(faces->maxx[ff] <= faces->minx[bf] || faces->maxx[bf] <= faces->minx[ff]
                                    || faces->maxy[ff] <= faces->miny[bf] || faces->maxy[bf] <= faces->miny[ff]);
                int bbox_touch = !(faces->maxx[ff] < faces->minx[bf] || faces->maxx[bf] < faces->minx[ff]
                                   || faces->maxy[ff] < faces->miny[bf] || faces->maxy[bf] < faces->miny[ff]);
                if (!bbox_touch) continue; /* no even-touch bbox => skip */
                int overlapping = 0;
                if (bbox_strict) {
                    if (projected_polygons_overlap_simple(model, ff, bf)) overlapping = 1;
                }
                if (overlapping || bbox_touch) {
                    int orig_pos_ff = pos_of_face[ff];
                    /* if (logf) fprintf(logf, "FF idx=%d pos=%d face=%d zmean=%.6f%s ", j, orig_pos_ff, ff, FIXED_TO_FLOAT(faces->z_mean[ff]), overlapping?"(ov)":"(tch)"); */
                    found++; if (overlapping) { per_ov++; total_ov++; } else { per_tch++; total_tch++; }

                    /* Decide ordering: try plane tests (bf vs ff) first. Policy:
                     * - If plane_after indicates bf is decisively after ff, prefer that
                     *   and move bf after ff.
                     * - If plane_before indicates bf is decisively before ff, prefer
                     *   that and keep bf before ff.
                     * - Only when plane tests are inconclusive or conflicting, we
                     *   use z_mean as a deterministic fallback. This keeps changes
                     *   local and reproducible across runs.
                     */
                    int plane_after = pair_plane_after(model, bf, ff); /* 1 if bf after ff */
                    int plane_before = pair_plane_before(model, bf, ff); /* 1 if bf before ff */

                    int decision = 0; /* 1 => move bf after ff, -1 => keep bf before ff */
                    const char *reason = NULL;
                    if (plane_after == 1 && plane_before == 0) { decision = 1; reason = "plane_after"; }
                    else if (plane_before == 1 && plane_after == 0) { decision = -1; reason = "plane_before"; }
                    else {
                        /* inconclusive: fallback to zmean */
                        float zbf = FIXED_TO_FLOAT(faces->z_mean[bf]);
                        float zff = FIXED_TO_FLOAT(faces->z_mean[ff]);
                        if (zbf < zff) { decision = 1; reason = "zmean"; } else { decision = -1; reason = "zmean"; }
                    }

                    /* Apply decision: if move needed, update snapshot and pos_of_face */
                    // Only move bf after ff if bf is currently before ff in the sorted order
                    if (decision == 1 && pos_of_face[bf] < pos_of_face[ff]) {
                        int from = pos_of_face[bf];
                        int to = pos_of_face[ff] + 1; /* insert after ff */
                        if (from < to) to--; /* account for removal shifting */
                        if (from != to) {
                            int tmp = snapshot[from];
                            if (from < to) {
                                for (int s = from; s < to; ++s) { snapshot[s] = snapshot[s+1]; pos_of_face[snapshot[s]] = s; }
                                snapshot[to] = tmp; pos_of_face[tmp] = to;
                            } else {
                                for (int s = from; s > to; --s) { snapshot[s] = snapshot[s-1]; pos_of_face[snapshot[s]] = s; }
                                snapshot[to] = tmp; pos_of_face[tmp] = to;
                            }
                            /* Log move */
                            /* if (logf) fprintf(logf, "  ACTION move bf->after ff (reason=%s) from=%d to=%d\n", reason, from, to); */
                            total_moves++;
                        } else {
                            /* if (logf) fprintf(logf, "  ACTION decision=%s but already at desired pos\n", reason); */
                        }
                        // Once a swap has been made (backface moved after a frontface),
                        // this backface should not be tested with other frontfaces.
                        // This prevents multiple swaps and ensures local ordering consistency.
                        //break; ==> NO : we nned to check all front faces to sort out the correct order
                    }
                }
            }
            if (!found) { /* if (logf) fprintf(logf, "none"); */ total_none++; }
            /* if (logf) fprintf(logf, " -- overlaps=%d touch=%d\n", per_ov, per_tch); */
        }
        /* if (logf) {
            fprintf(logf, "SUMMARY scanned=%d logged=%d total_ov=%d total_tch=%d total_none=%d total_moves=%d\n", total_scanned, total_logged, total_ov, total_tch, total_none, total_moves);
            fclose(logf);
        } */

        /* Restore original order from passes 1/2 */
        for (int i = 0; i < n; ++i) faces->sorted_face_indices[i] = snapshot[i];
        free(snapshot);
    }
    

    cull_back_faces = old_cull;
    free(pos_of_face);
    free(min_allowed_pos);
    // Display identical to painter_correct
    // printf("Faces:");
    // for (int i = 0; i < n; ++i) printf(" %d", faces->sorted_face_indices[i]);
    // printf("\n");
    return moves;
}

static int move_element_remove_and_insert(int *arr, int n, int from, int insert_idx) {
    if (from < 0 || from >= n) return 0;
    if (insert_idx < 0) insert_idx = 0;
    if (insert_idx > n-1) insert_idx = n-1;
    if (from == insert_idx) return 0;
    int val = arr[from];
    if (insert_idx < from) {
        /* shift left region [insert_idx..from-1] right by 1 */
        memmove(&arr[insert_idx+1], &arr[insert_idx], (from - insert_idx) * sizeof(int));
        arr[insert_idx] = val;
    } else {
        /* insert_idx > from: shift region [from+1..insert_idx] left by 1 */
        memmove(&arr[from], &arr[from+1], (insert_idx - from) * sizeof(int));
        arr[insert_idx] = val;
    }
    return 1;
}

/* Optimized variant: performs the same remove+insert but updates an optional
 * inverse map `pos_of_face` in a single pass to avoid a separate update loop.
 * If `pos_of_face` is NULL, this behaves like the classic version (but using
 * explicit loops instead of memmove so we can update positions inline).
 */
static int move_element_remove_and_insert_pos(int *arr, int n, int from, int insert_idx, int *pos_of_face) {
    if (from < 0 || from >= n) return 0;
    if (insert_idx < 0) insert_idx = 0;
    if (insert_idx > n-1) insert_idx = n-1;
    if (from == insert_idx) return 0;
    int val = arr[from];
    if (insert_idx < from) {
        /* shift right region [insert_idx..from-1] -> [insert_idx+1..from] */
        for (int i = from; i > insert_idx; --i) {
            arr[i] = arr[i-1];
            if (pos_of_face) pos_of_face[arr[i]] = i;
        }
        arr[insert_idx] = val;
        if (pos_of_face) pos_of_face[val] = insert_idx;
    } else {
        /* shift left region [from+1..insert_idx] -> [from..insert_idx-1] */
        for (int i = from; i < insert_idx; ++i) {
            arr[i] = arr[i+1];
            if (pos_of_face) pos_of_face[arr[i]] = i;
        }
        arr[insert_idx] = val;
        if (pos_of_face) pos_of_face[val] = insert_idx;
    }
    return 1;
}



segment "ordering";
// ============================================================================
//  2. ORDERING TESTS & OVERLAP
//  Geometric relation tests, polygon overlap, ray casting
// ============================================================================

/* projected_polygons_overlap
 * --------------------------
 *
 * Parameters:
 *  - model: pointer to the 3D model containing the faces
 *  - f1: index of the first face
 *  - f2: index of the second face
 *
 * Returns:
 *  - 1 if the projected polygons of f1 and f2 overlap
 *  - 0 if they do not overlap (including touching-only cases)
 */
/*
 projected_polygons_overlap(model, f1, f2)
 -----------------------------------------
 Determines whether the 2D projections of faces `f1` and `f2` overlap (simple contact = NO overlap).
 The decision is made by a sequence of increasingly expensive tests to remain conservative and fast.
 Steps (in order):
  1) Fast AABB rejection: if the whole boxes are disjoint or only touching -> NO overlap.
  2) Early acceptance rule (heuristic): if an edge of one polygon has **≥ 2*proper intersections
     with the other polygon (entry + exit), accept immediately (certain overlap for that edge).
     This rule avoids expensive clipping in obvious cases. Note: the result depends on
     `segs_intersect_int` behaviour (and its internal tolerance), so tolerance can affect this test.
  3) Edge-to-edge check (proper intersection): for each edge pair, AABB quick-reject then
     full intersection test; a proper intersection marks the pair as a *candidate(do not accept
     immediately but continue with sampling and clipping tests).
  4) Containment tests: check whether a vertex of one polygon is strictly inside the other
     (boundary points count as outside).
  5) Special case: identical polygons (same vertex sequence) are considered overlapping.
  6) Candidate handling and sampling: compute the full intersection bbox and try quick samples
     (center then 3×3 grid) for fast acceptance.
  7) If sampling fails, fall back to exact clipping (Sutherland–Hodgman) in both directions (f1→f2 and f2→f1).
  8) Clipping area validity test: accept only if area >= MIN_INTERSECTION_AREA_PIXELS and <= bbox area (+eps).
 When a clipping result is valid in both directions, prefer (f1 clipped by f2) to preserve Python semantics.
 Debug hooks print clipping and centroid details when enabled. 
 */

/* =====================================================================
 * projected_polygons_overlap_old
 * ---------------------------------------------------------------------
 * ORIGINAL (unmodified) implementation. Kept as a reference/fallback.
 * See projected_polygons_overlap() below for the optimized version and
 * a full explanation of what changed and why it is safe.
 * ===================================================================== */
static int projected_polygons_overlap_old(Model3D* model, int f1, int f2) {
    /* ... corps original inchangé, tel quel ... */
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;

    /* Unit tests for segs_intersect: run with command-line flag --run-segs-test */
    static int use_fixed_clipping = 1; /* enable Fixed64 clipping by default (non-invasive) */
    // static void set_use_fixed_clipping(int v) { use_fixed_clipping = v ? 1 : 0; }

    int n1 = faces->vertex_count[f1];
    int n2 = faces->vertex_count[f2];
    if (n1 < 3 || n2 < 3) return 0;

    overlapCheckCount++;

    long proj_start_tick = GetTick();
    const long PROJ_OVERLAP_WATCHDOG_MS = 1000;

    int minx1 = faces->minx[f1], maxx1 = faces->maxx[f1], miny1 = faces->miny[f1], maxy1 = faces->maxy[f1];
    int minx2 = faces->minx[f2], maxx2 = faces->maxx[f2], miny2 = faces->miny[f2], maxy2 = faces->maxy[f2];

    if (maxx1 <= minx2 || maxx2 <= minx1 || maxy1 <= miny2 || maxy2 <= miny1) return 0;

    int off1 = faces->vertex_indices_ptr[f1];
    int off2 = faces->vertex_indices_ptr[f2];

    int dbg_pair = 0;
{
    int count;
    for (int i = 0; i < n1; ++i) {
        if (GetTick() - proj_start_tick > PROJ_OVERLAP_WATCHDOG_MS) return 0;
        int i2 = (i+1) % n1;
        int va = faces->vertex_indices_buffer[off1 + i]  - 1;
        int vb = faces->vertex_indices_buffer[off1 + i2] - 1;
        int ax = vtx->x2d[va], ay = vtx->y2d[va];
        int bx = vtx->x2d[vb], by = vtx->y2d[vb];
        count = 0;
        for (int j = 0; j < n2; ++j) {
            if ( (j & 0x1f) == 0 ) { }
            int j2 = (j+1) % n2;
            int vc = faces->vertex_indices_buffer[off2 + j]  - 1;
            int vd = faces->vertex_indices_buffer[off2 + j2] - 1;
            int cx = vtx->x2d[vc], cy = vtx->y2d[vc];
            int dx = vtx->x2d[vd], dy = vtx->y2d[vd];
            if (segs_intersect_int(ax,ay,bx,by,cx,cy,dx,dy)) {
                count++;
                if (count >= 2) return 1;
            }
        }
    }
    for (int j = 0; j < n2; ++j) {
        int j2 = (j+1) % n2;
        int vc = faces->vertex_indices_buffer[off2 + j]  - 1;
        int vd = faces->vertex_indices_buffer[off2 + j2] - 1;
        int cx = vtx->x2d[vc], cy = vtx->y2d[vc];
        int dx = vtx->x2d[vd], dy = vtx->y2d[vd];
        count = 0;
        for (int i = 0; i < n1; ++i) {
            int i2 = (i+1) % n1;
            int va = faces->vertex_indices_buffer[off1 + i]  - 1;
            int vb = faces->vertex_indices_buffer[off1 + i2] - 1;
            int ax = vtx->x2d[va], ay = vtx->y2d[va];
            int bx = vtx->x2d[vb], by = vtx->y2d[vb];
            if (segs_intersect_int(cx,cy,dx,dy,ax,ay,bx,by)) {
                count++;
                if (count >= 2) return 1;
            }
        }
    }
}

    int candidate = 0;
    for (int i = 0; i < n1; ++i) {
        if (GetTick() - proj_start_tick > PROJ_OVERLAP_WATCHDOG_MS) return 0;
        int i2 = (i+1) % n1;
        int va = faces->vertex_indices_buffer[off1 + i] - 1;
        int vb = faces->vertex_indices_buffer[off1 + i2] - 1;
        int ax = vtx->x2d[va], ay = vtx->y2d[va];
        int bx = vtx->x2d[vb], by = vtx->y2d[vb];
        int aminx = ax < bx ? ax : bx; int amaxx = ax > bx ? ax : bx;
        int aminy = ay < by ? ay : by; int amaxy = ay > by ? ay : by;
        for (int j = 0; j < n2; ++j) {
            if ( (j & 0x1f) == 0 ) { }
            int j2 = (j+1) % n2;
            int vc = faces->vertex_indices_buffer[off2 + j] - 1;
            int vd = faces->vertex_indices_buffer[off2 + j2] - 1;
            int cx = vtx->x2d[vc], cy = vtx->y2d[vc];
            int dx = vtx->x2d[vd], dy = vtx->y2d[vd];
            int cminx = cx < dx ? cx : dx; int cmaxx = cx > dx ? cx : dx;
            int cminy = cy < dy ? cy : dy; int cmaxy = cy > dy ? cy : dy;
            if (amaxx <= cminx || cmaxx <= aminx || amaxy <= cminy || cmaxy <= aminy) continue;
            if (segs_intersect_int(ax,ay,bx,by,cx,cy,dx,dy)) {
                candidate = 1;
                overlapCheckCount++; overlapSegiAccept++;
                break;
            }
        }
        if (candidate) return 1;
    }

    if (!candidate) {
        for (int ii = 0; ii < n1; ++ii) {
            if (GetTick() - proj_start_tick > PROJ_OVERLAP_WATCHDOG_MS) return 0;
            int vid = faces->vertex_indices_buffer[off1 + ii] - 1;
            if (vid < 0 || vid >= vtx->vertex_count) continue;
            int px = vtx->x2d[vid], py = vtx->y2d[vid];
            if (px < minx2 || px > maxx2 || py < miny2 || py > maxy2) continue;
            if (point_in_poly_int(px, py, faces, vtx, f2, n2)) { candidate = 1; break; }
        }
    }
    if (!candidate) {
        for (int jj = 0; jj < n2; ++jj) {
            if (GetTick() - proj_start_tick > PROJ_OVERLAP_WATCHDOG_MS) return 0;
            int vid = faces->vertex_indices_buffer[off2 + jj] - 1;
            if (vid < 0 || vid >= vtx->vertex_count) continue;
            int px = vtx->x2d[vid], py = vtx->y2d[vid];
            if (px < minx1 || px > maxx1 || py < miny1 || py > maxy1) continue;
            if (point_in_poly_int(px, py, faces, vtx, f1, n1)) { candidate = 1; break; }
        }
    }

    if (!candidate) {
        if (faces_vertices_equal(faces, vtx, f1, f2)) return 1;
        return 0;
    }

    int oxmin = minx1 > minx2 ? minx1 : minx2;
    int oxmax = maxx1 < maxx2 ? maxx1 : maxx2;
    int oymin = miny1 > miny2 ? miny1 : miny2;
    int oymax = maxy1 < maxy2 ? maxy1 : maxy2;
    if (oxmin > oxmax || oymin > oymax) return 0;

    int cx = (oxmin + oxmax) / 2; int cy = (oymin + oymax) / 2;
    if (point_in_poly_int(cx, cy, faces, vtx, f1, n1) && point_in_poly_int(cx, cy, faces, vtx, f2, n2)) {
        overlapSampleAccept++;
        return 1;
    }

    int ixmin = oxmin, ixmax = oxmax, iymin = oymin, iymax = oymax;
    int W = ixmax - ixmin; int H = iymax - iymin;
    int sample_accept = 0; int N = 3;
    for (int sx = 0; sx < N; ++sx) {
        for (int sy = 0; sy < N; ++sy) {
            int tx = ixmin + (((2*sx + 1) * W + (2*N - 1)) / (2*N));
            int ty = iymin + (((2*sy + 1) * H + (2*N - 1)) / (2*N));
            if (point_in_poly_int(tx, ty, faces, vtx, f1, n1) && point_in_poly_int(tx, ty, faces, vtx, f2, n2)) { sample_accept = 1; break; }
        }
        if (sample_accept) break;
    }

    if (sample_accept) {
        overlapSampleAccept++;
        return 1;
    }

    int icx1 = 0, icy1 = 0; double iarea1 = 0.0;
    int icx2 = 0, icy2 = 0; double iarea2 = 0.0;
    overlapClipCalls++;

    unsigned long long area2_1 = 0, area2_2 = 0;
    int ok1 = 0, ok2 = 0;
    if (use_fixed_clipping) {
        int tx=0, ty=0; long long a2 = 0; debug_overlap_subj = f1; debug_overlap_clip = f2; 
        ok1 = compute_intersection_centroid_ordered_fixed(model, f1, f2, &tx, &ty, &a2); 
        debug_overlap_subj = -1; debug_overlap_clip = -1; 
        area2_1 = (unsigned long long)(a2 >= 0 ? a2 : -a2);
        debug_clip_fixed_area = (double)area2_1 * 0.5;
        int tx2=0, ty2=0; long long a22 = 0; debug_overlap_subj = f2; debug_overlap_clip = f1; ok2 = compute_intersection_centroid_ordered_fixed(model, f2, f1, &tx2, &ty2, &a22); debug_overlap_subj = -1; debug_overlap_clip = -1; area2_2 = (unsigned long long)(a22 >= 0 ? a22 : -a22);
    }

    double bbox_area = 0.0; if (!(oxmin > oxmax || oymin > oymax)) bbox_area = (double)(oxmax - oxmin) * (double)(oymax - oymin);

    const double eps = 1e-9;
    int valid1 = 0, valid2 = 0;
    if (use_fixed_clipping) {
        unsigned long long min_area2_fixed = (unsigned long long)(2.0 * MIN_INTERSECTION_AREA_PIXELS * (double)FIXED_SCALE * (double)FIXED_SCALE + 0.5);
        unsigned long long bbox_limit = 0;
        if (!(oxmin > oxmax || oymin > oymax)) {
            unsigned long long bbox_pixels = (unsigned long long)(oxmax - oxmin) * (unsigned long long)(oymax - oymin);
            unsigned long long scale2 = (unsigned long long)FIXED_SCALE * (unsigned long long)FIXED_SCALE;
            bbox_limit = 2ULL * bbox_pixels * scale2;
        }
        valid1 = (ok1 && debug_clip_fixed_vcount >= 3 && area2_1 >= min_area2_fixed && area2_1 <= bbox_limit);
        valid2 = (ok2 && debug_clip_fixed_vcount >= 3 && area2_2 >= min_area2_fixed && area2_2 <= bbox_limit);
    } else {
        valid1 = (ok1 && iarea1 >= MIN_INTERSECTION_AREA_PIXELS && iarea1 <= bbox_area + eps);
        valid2 = (ok2 && iarea2 >= MIN_INTERSECTION_AREA_PIXELS && iarea2 <= bbox_area + eps);
    }

    if (valid1 || valid2) {
        if (valid1) { overlapClipAccept++; return 1; }
        else { overlapClipAccept++; return 1; }
    }
    return 0;
}

/* =====================================================================
 * projected_polygons_overlap  (OPTIMIZED)
 * ---------------------------------------------------------------------
 * This is a performance-optimized rewrite of projected_polygons_overlap_old().
 * The original is kept under the _old suffix for reference and as a safe
 * fallback if a discrepancy is ever suspected.
 *
 * WHAT CHANGED AND WHY IT SHOULD NOT AFFECT THE RESULT
 *
 * 1) Removed the old "Step 2" (double-intersection early-accept heuristic).
 *    The old Step 2 ran a full O(n1*n2) scan (in BOTH directions: edges of
 *    f1 against f2, then edges of f2 against f1) looking for any single
 *    edge that has >= 2 proper intersections with the other polygon, and
 *    accepted immediately if found.
 *
 *    The edge/edge scan below (formerly "Step 3") performs essentially the
 *    same pairwise segment test, but accepts on the FIRST proper
 *    intersection found (a strictly weaker condition than ">= 2"), and adds
 *    a per-edge AABB rejection that Step 2 did not have (so it is cheaper
 *    per pair too). Any pair of edges that would have satisfied the old
 *    Step 2's ">= 2 intersections" condition necessarily contains at least
 *    one proper intersection - which the edge/edge scan below is guaranteed
 *    to find and accept on. In other words, Step 2 could never accept a
 *    case that the edge/edge scan would not also accept, so it was pure
 *    duplicated work in the case where no early accept happens (i.e. it
 *    used to scan the same O(n1*n2) pairs up to three times: Step2-forward,
 *    Step2-backward, then Step3). Removing it changes nothing except the
 *    number of non-overlapping / weakly-overlapping cases that get scanned
 *    redundantly.
 *
 *    Known caveat (discussed and accepted by the project owner): this
 *    argument assumes segs_intersect_int()/segs_intersect_int_fixed64() is
 *    symmetric in its argument order, i.e. segs_intersect_int(a,b,c,d) ==
 *    segs_intersect_int(c,d,a,b) for all inputs. Looking at
 *    segs_intersect_int_fixed64(): the orientation test and the four
 *    point/segment tolerance checks are symmetric by construction, but the
 *    intersection point itself is computed by parameterizing along
 *    "segment 1" using integer division and Fixed64 shifts, which truncate.
 *    Swapping which segment is "segment 1" can therefore change the
 *    intersection point by up to 1 fixed-point unit due to truncation,
 *    which in an astronomically rare edge case (intersection point sitting
 *    almost exactly on the segs_intersect_tol_px tolerance boundary near an
 *    endpoint) could theoretically flip accept/reject depending on argument
 *    order. This function has knowingly kept the single-order test (as
 *    used by the old Step 3 / forward Step 2) for maximum performance,
 *    since this is considered an acceptable, vanishingly rare numerical
 *    edge case rather than a real geometric difference. If this is ever a
 *    concern, both call sites below can be changed to:
 *        segs_intersect_int(ax,ay,bx,by,cx,cy,dx,dy) ||
 *        segs_intersect_int(cx,cy,dx,dy,ax,ay,bx,by)
 *    to remove this residual risk entirely, at a small extra cost limited
 *    to AABB-passing edge pairs whose first-order test fails.
 *
 * 2) Precompute f2's edges once, before the loop over f1's edges.
 *    In the original code, for every edge i of f1, the inner loop over f2
 *    recomputed cx,cy,dx,dy and the edge's AABB from scratch for every j -
 *    meaning each of f2's edges was recomputed n1 times. Below, f2's edge
 *    endpoints and AABBs are computed exactly once (same formulas, same
 *    order of operations, same integer truncation - i.e. bit-for-bit
 *    identical values) into small local arrays, then simply read back
 *    inside the i-loop. This is a pure "compute once, reuse many times"
 *    change: no formula is altered, so results are identical. It is only
 *    used when n2 fits in MAX_FACE_VERTICES (always true in practice,
 *    since that constant already bounds face vertex counts elsewhere in
 *    the codebase); if a face ever exceeded that bound, the code
 *    transparently falls back to recomputing on the fly, exactly as the
 *    original always did, so correctness is preserved either way.
 *
 * 3) Removed dead trace code (`if ((j & 0x1f) == 0) { }`), which had its
 *    body already stripped out and did nothing but spend a compare and a
 *    branch on every inner-loop iteration.
 *
 * 4) Step 9 (exact clipping fallback) now short-circuits: the original
 *    always computed BOTH compute_intersection_centroid_ordered_fixed()
 *    calls (f1-clipped-by-f2 AND f2-clipped-by-f1) - the single most
 *    expensive operation in the whole function - even though the final
 *    result only depends on `valid1 || valid2`, with `valid1` preferred
 *    whenever both are valid. So whenever valid1 already holds, computing
 *    ok2/area2_2 could never change the outcome (valid1 wins regardless of
 *    valid2). Below, the second clipping call is only performed if the
 *    first one did not already produce a valid result. This is a pure
 *    "skip unnecessary work" change: whenever valid1 is true, the original
 *    would have returned 1 too (via the `if (valid1) ... else ...` branch,
 *    both of which return 1) - we just return 1 sooner without computing
 *    ok2. Whenever valid1 is false, both versions compute ok2/valid2 and
 *    behave identically. The result is byte-for-byte identical to the
 *    original in every case.
 *
 * 5) Removed the entire `!use_fixed_clipping` (float clipping) branch and
 *    its associated dead variables (iarea1, iarea2, icx1, icy1, icx2,
 *    icy2). `use_fixed_clipping` is declared as
 *        static int use_fixed_clipping = 1;
 *    and its only setter, set_use_fixed_clipping(), is commented out
 *    (confirmed with the project owner) - so this flag is permanently 1
 *    for the entire lifetime of the program and can never take any other
 *    value. The `!use_fixed_clipping` branch was therefore unreachable
 *    dead code: `ok1`/`ok2` were never assigned in that branch (always
 *    0), making `valid1`/`valid2` always false there regardless of the
 *    actual geometry - i.e. that branch could never have accepted an
 *    overlap even when one genuinely existed. Since it can never execute
 *    given the current (and only ever observed) value of
 *    use_fixed_clipping, removing it has no effect on behavior. If
 *    set_use_fixed_clipping() is ever un-commented and called with 0 in
 *    the future, a proper float-based clipping implementation (populating
 *    ok1/ok2/iarea1/iarea2 for real) would need to be written from
 *    scratch at that point - simply restoring this dead branch would not
 *    fix anything, since it never worked to begin with.
 *
 * 6) Everything else (Step 1 AABB reject, the edge/edge scan's accept
 *    condition, containment tests, the identical-polygon special case,
 *    center/3x3 sampling, and the area thresholds/validity checks in
 *    Step 9's Fixed64 path) is untouched, byte-for-byte, from the
 *    original.
 *
 * WATCHDOG NOTE: because this version does strictly less redundant work,
 * it can only finish sooner than the original for the same inputs - it can
 * never trigger the GetTick() timeout bail-out (return 0) in a case where
 * the original would not have. The reverse is also true in principle: on
 * a pair of faces so pathological that the original timed out, this
 * version might finish in time and return a real (non-timeout) result
 * instead. That is a correction of a timing-dependent false negative, not
 * a change in geometric logic - the timeout was already a source of
 * non-determinism in the original code, tied to wall-clock time rather
 * than face data.
 * ===================================================================== */
 static int projected_polygons_overlap(Model3D* model, int f1, int f2) {
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;

    int n1 = faces->vertex_count[f1];
    int n2 = faces->vertex_count[f2];
    if (n1 < 3 || n2 < 3) return 0; /* degenerate face, cannot overlap */

    /* instrumentation, unchanged from original */
    overlapCheckCount++;

    /* Watchdog: bail out conservatively (return "no overlap") if this pair
     * is taking pathologically long, to avoid freezing the UI. Checked at
     * the top of every outer-loop iteration below, exactly like the
     * original. */
    long proj_start_tick = GetTick();
    const long PROJ_OVERLAP_WATCHDOG_MS = 1000;

    int minx1 = faces->minx[f1], maxx1 = faces->maxx[f1], miny1 = faces->miny[f1], maxy1 = faces->maxy[f1];
    int minx2 = faces->minx[f2], maxx2 = faces->maxx[f2], miny2 = faces->miny[f2], maxy2 = faces->maxy[f2];

    /* ---- Step 1: cheap whole-polygon AABB rejection ----
     * If the integer bounding boxes are disjoint or only touch at an
     * edge/point, the polygons cannot overlap. Touching-only counts as
     * non-overlap (hence the strict <=). This is unchanged from the
     * original and is the single cheapest filter, so it stays first. */
    if (maxx1 <= minx2 || maxx2 <= minx1 || maxy1 <= miny2 || maxy2 <= miny1) return 0;

    int off1 = faces->vertex_indices_ptr[f1];
    int off2 = faces->vertex_indices_ptr[f2];

    /* ---- (Old "Step 2" intentionally removed here — see the big header
     * comment above for the full justification.) ---- */

    /* ---- Precompute f2's edges once ----
     * Build small local arrays holding, for every edge j of f2:
     *   - its two endpoints (f2_cx/f2_cy -> f2_dx/f2_dy)
     *   - its axis-aligned bounding box (f2_minx/f2_maxx/f2_miny/f2_maxy)
     * These are exactly the same values the original code recomputed from
     * scratch on every (i, j) pair inside the loop below - computing them
     * once here and reading them back is a pure reuse optimization with
     * zero effect on the numbers themselves.
     *
     * MAX_FACE_VERTICES already bounds the vertex count of any face
     * elsewhere in the codebase, so the "happy path" below always applies
     * in practice. The `n2 <= MAX_FACE_VERTICES` guard is defensive only:
     * if it were ever false, we transparently fall back to recomputing
     * f2's edge data on the fly inside the loop, i.e. exactly what the
     * original always did - so this can never silently produce a wrong
     * result even if that assumption is violated in the future. */
    int f2_precomputed = (n2 <= MAX_FACE_VERTICES);
    int f2_cx[MAX_FACE_VERTICES], f2_cy[MAX_FACE_VERTICES];
    int f2_dx[MAX_FACE_VERTICES], f2_dy[MAX_FACE_VERTICES];
    int f2_minx[MAX_FACE_VERTICES], f2_maxx[MAX_FACE_VERTICES];
    int f2_miny[MAX_FACE_VERTICES], f2_maxy[MAX_FACE_VERTICES];

    if (f2_precomputed) {
        for (int j = 0; j < n2; ++j) {
            int j2 = (j+1) % n2;
            int vc = faces->vertex_indices_buffer[off2 + j]  - 1;
            int vd = faces->vertex_indices_buffer[off2 + j2] - 1;
            int cx = vtx->x2d[vc], cy = vtx->y2d[vc];
            int dx = vtx->x2d[vd], dy = vtx->y2d[vd];
            f2_cx[j] = cx; f2_cy[j] = cy;
            f2_dx[j] = dx; f2_dy[j] = dy;
            f2_minx[j] = cx < dx ? cx : dx; f2_maxx[j] = cx > dx ? cx : dx;
            f2_miny[j] = cy < dy ? cy : dy; f2_maxy[j] = cy > dy ? cy : dy;
        }
    }

    /* ---- Edge-vs-edge proper intersection scan (formerly "Step 3") ----
     * For every edge of f1, walk every edge of f2:
     *   - quick per-edge AABB reject (touching-only = non-overlap, hence <=)
     *   - if the AABBs overlap, run the real integer/fixed segment test
     * Accept (return 1) on the very first proper intersection found. This
     * alone is enough to detect any case of two polygons whose boundaries
     * genuinely cross - see the header comment for why this makes the old
     * Step 2 redundant. */
    int candidate = 0;
    for (int i = 0; i < n1; ++i) {
        if (GetTick() - proj_start_tick > PROJ_OVERLAP_WATCHDOG_MS) {
            /* watchdog triggered - conservative bail-out, same as original */
            return 0;
        }
        int i2 = (i+1) % n1;
        int va = faces->vertex_indices_buffer[off1 + i] - 1;
        int vb = faces->vertex_indices_buffer[off1 + i2] - 1;
        int ax = vtx->x2d[va], ay = vtx->y2d[va];
        int bx = vtx->x2d[vb], by = vtx->y2d[vb];
        int aminx = ax < bx ? ax : bx; int amaxx = ax > bx ? ax : bx;
        int aminy = ay < by ? ay : by; int amaxy = ay > by ? ay : by;

        for (int j = 0; j < n2; ++j) {
            int cx, cy, dx, dy;
            int cminx, cmaxx, cminy, cmaxy;

            if (f2_precomputed) {
                /* fast path: just read back the precomputed edge data */
                cx = f2_cx[j]; cy = f2_cy[j];
                dx = f2_dx[j]; dy = f2_dy[j];
                cminx = f2_minx[j]; cmaxx = f2_maxx[j];
                cminy = f2_miny[j]; cmaxy = f2_maxy[j];
            } else {
                /* defensive fallback: recompute on the fly, exactly like
                 * the original code always did */
                int j2 = (j+1) % n2;
                int vc = faces->vertex_indices_buffer[off2 + j]  - 1;
                int vd = faces->vertex_indices_buffer[off2 + j2] - 1;
                cx = vtx->x2d[vc]; cy = vtx->y2d[vc];
                dx = vtx->x2d[vd]; dy = vtx->y2d[vd];
                cminx = cx < dx ? cx : dx; cmaxx = cx > dx ? cx : dx;
                cminy = cy < dy ? cy : dy; cmaxy = cy > dy ? cy : dy;
            }

            /* per-edge AABB quick reject before the (more expensive)
             * exact segment intersection test */
            if (amaxx <= cminx || cmaxx <= aminx || amaxy <= cminy || cmaxy <= aminy) continue;

            if (segs_intersect_int(ax,ay,bx,by,cx,cy,dx,dy)) {
                candidate = 1;
                overlapCheckCount++; overlapSegiAccept++;
                break; /* no need to keep scanning f2's edges for this i */
            }
        }
        if (candidate) return 1; /* early accept on proper intersection */
    }

    /* ---- Step 4: containment tests (unchanged) ----
     * At this point no edges of f1 and f2 cross. The polygons can still
     * overlap if one is entirely (or partially, without crossing - e.g.
     * exact vertex coincidence handled by Step 5) inside the other. Check
     * every vertex of f1 against f2, skipping vertices clearly outside
     * f2's bbox, and vice versa. A vertex lying exactly on the other
     * polygon's boundary is NOT considered inside (point_in_poly_int's
     * convention), matching the original. */
    if (!candidate) {
        for (int ii = 0; ii < n1; ++ii) {
            if (GetTick() - proj_start_tick > PROJ_OVERLAP_WATCHDOG_MS) { return 0; }
            int vid = faces->vertex_indices_buffer[off1 + ii] - 1;
            if (vid < 0 || vid >= vtx->vertex_count) continue;
            int px = vtx->x2d[vid], py = vtx->y2d[vid];
            if (px < minx2 || px > maxx2 || py < miny2 || py > maxy2) continue;
            if (point_in_poly_int(px, py, faces, vtx, f2, n2)) { candidate = 1; break; }
        }
    }
    if (!candidate) {
        for (int jj = 0; jj < n2; ++jj) {
            if (GetTick() - proj_start_tick > PROJ_OVERLAP_WATCHDOG_MS) { return 0; }
            int vid = faces->vertex_indices_buffer[off2 + jj] - 1;
            if (vid < 0 || vid >= vtx->vertex_count) continue;
            int px = vtx->x2d[vid], py = vtx->y2d[vid];
            if (px < minx1 || px > maxx1 || py < miny1 || py > maxy1) continue;
            if (point_in_poly_int(px, py, faces, vtx, f1, n1)) { candidate = 1; break; }
        }
    }

    if (!candidate) {
        /* ---- Step 5: identical-polygon special case (unchanged) ----
         * No edge crossing and no vertex containment found. The only
         * remaining way these two faces can "overlap" is if their 2D
         * projected vertex sequences are literally identical (possibly
         * reversed - e.g. coincident/duplicate faces). Otherwise, they
         * truly do not overlap. */
        if (faces_vertices_equal(faces, vtx, f1, f2)) return 1;
        return 0;
    }

    /* ---- Steps 6-9: sampling then exact clipping fallback ----
     * A containment candidate was found (Step 4). Confirm and, more
     * importantly, make sure the overlapping area actually clears
     * MIN_INTERSECTION_AREA_PIXELS before accepting - a vertex can be
     * "inside" the other polygon by a hair without the true overlap area
     * being meaningful. */
    int oxmin = minx1 > minx2 ? minx1 : minx2;
    int oxmax = maxx1 < maxx2 ? maxx1 : maxx2;
    int oymin = miny1 > miny2 ? miny1 : miny2;
    int oymax = maxy1 < maxy2 ? maxy1 : maxy2;
    if (oxmin > oxmax || oymin > oymax) return 0; /* integer bbox empty -> less than 1 pixel */

    /* Step 7: cheap check of the intersection bbox's center pixel */
    int cx = (oxmin + oxmax) / 2; int cy = (oymin + oymax) / 2;
    if (point_in_poly_int(cx, cy, faces, vtx, f1, n1) && point_in_poly_int(cx, cy, faces, vtx, f2, n2)) {
        overlapSampleAccept++;
        return 1;
    }

    /* Step 8: adaptive 3x3 sampling around the intersection bbox interior
     * (center point already tested above) */
    int ixmin = oxmin, ixmax = oxmax, iymin = oymin, iymax = oymax;
    int W = ixmax - ixmin; int H = iymax - iymin;
    int sample_accept = 0; int N = 3;
    for (int sx = 0; sx < N; ++sx) {
        for (int sy = 0; sy < N; ++sy) {
            int tx = ixmin + (((2*sx + 1) * W + (2*N - 1)) / (2*N));
            int ty = iymin + (((2*sy + 1) * H + (2*N - 1)) / (2*N));
            if (point_in_poly_int(tx, ty, faces, vtx, f1, n1) && point_in_poly_int(tx, ty, faces, vtx, f2, n2)) { sample_accept = 1; break; }
        }
        if (sample_accept) break;
    }

    if (sample_accept) {
        overlapSampleAccept++;
        return 1;
    }

    /* Step 9: exact clipping fallback (Sutherland-Hodgman, Fixed64), tried
     * f1-clipped-by-f2 first, and f2-clipped-by-f1 ONLY if the first
     * order didn't already produce a valid, large-enough intersection.
     *
     * use_fixed_clipping is always 1 in this codebase (see header comment,
     * point 5), so the Fixed64 path below is the only path that can ever
     * run; the former float-clipping fallback branch has been removed as
     * dead code. */
    overlapClipCalls++;

    unsigned long long min_area2_fixed =
        (unsigned long long)(2.0 * MIN_INTERSECTION_AREA_PIXELS * (double)FIXED_SCALE * (double)FIXED_SCALE + 0.5);
    unsigned long long bbox_limit = 0;
    if (!(oxmin > oxmax || oymin > oymax)) {
        unsigned long long bbox_pixels = (unsigned long long)(oxmax - oxmin) * (unsigned long long)(oymax - oymin);
        unsigned long long scale2 = (unsigned long long)FIXED_SCALE * (unsigned long long)FIXED_SCALE;
        bbox_limit = 2ULL * bbox_pixels * scale2;
    }

    /* --- try f1 clipped by f2 first --- */
    int tx = 0, ty = 0; long long a2 = 0;
    debug_overlap_subj = f1; debug_overlap_clip = f2;
    int ok1 = compute_intersection_centroid_ordered_fixed(model, f1, f2, &tx, &ty, &a2);
    debug_overlap_subj = -1; debug_overlap_clip = -1;
    unsigned long long area2_1 = (unsigned long long)(a2 >= 0 ? a2 : -a2);
    debug_clip_fixed_area = (double)area2_1 * 0.5;

    int valid1 = (ok1 && debug_clip_fixed_vcount >= 3 && area2_1 >= min_area2_fixed && area2_1 <= bbox_limit);
    if (valid1) {
        overlapClipAccept++;
        return 1;
    }

    /* --- first order was not valid: try f2 clipped by f1 --- */
    int tx2 = 0, ty2 = 0; long long a22 = 0;
    debug_overlap_subj = f2; debug_overlap_clip = f1;
    int ok2 = compute_intersection_centroid_ordered_fixed(model, f2, f1, &tx2, &ty2, &a22);
    debug_overlap_subj = -1; debug_overlap_clip = -1;
    unsigned long long area2_2 = (unsigned long long)(a22 >= 0 ? a22 : -a22);

    int valid2 = (ok2 && debug_clip_fixed_vcount >= 3 && area2_2 >= min_area2_fixed && area2_2 <= bbox_limit);
    if (valid2) {
        overlapClipAccept++;
        return 1;
    }

    return 0;
}
 
/*
 * projected_polygons_overlap_simple (legacy / simple overlap test)
 * ---------------------------------------------------------------
 * Purpose:
 *  - Provide a fast, legacy-style screen-space overlap test used historically
 *    by the renderer. It implements a straightforward sequence of inexpensive
 *    checks and avoids the more costly sampling+clipping steps found in the
 *    stricter `projected_polygons_overlap` implementation.
 *
 * Algorithm summary (simple):
 *  1) Axis-aligned bbox quick-reject (if bboxes are disjoint -> NON-overlap)
 *  2) Edge-vs-edge *proper* intersection tests with per-edge bbox quick-reject
 *     (returns overlap on any proper intersection; colinear or endpoint-touching
 *     do not count as proper intersection)
 *  3) Containment checks via ray-casting for vertices that fall inside the
 *     other's bbox (points on edges count as OUTSIDE)
 *  - Returns 1 for detected intersection/containment, 0 otherwise.
 *
 * Performance & semantics:
 *  - This variant is faster and tends to accept borderline overlaps (including
 *    small/degenerate cases) because it does not enforce a minimum clipped area
 *    threshold. It is thus useful for compatibility and to avoid strict rejections
 *    that can cause ordering artifacts in some meshes.
 *
 * projected_polygons_overlap (strict / precise overlap test)
 * ----------------------------------------------------------
 * Purpose:
 *  - A more conservative and precise overlap test that reduces false positives
 *    for touching or sub-pixel intersections.
 *
 * Algorithm summary (strict):
 *  1) Axis-aligned bbox quick-reject (same as simple)
 *  2) Edge intersection + containment quick checks; if these indicate a candidate
 *     intersection, the function samples a 3×3 grid of interior pixel centers in
 *     the integer intersection bbox (fast acceptance if any sample lies inside
 *     both polygons).
 *  3) If sampling fails, the strict algorithm performs exact polygon clipping
 *     (Sutherland–Hodgman) and computes the clipped polygon area. The overlap
 *     is accepted only if the clipped area >= MIN_INTERSECTION_AREA_PIXELS (1.0).
 *
 * Rationale & guidance:
 *  - Use `projected_polygons_overlap_simple` when you need legacy/compat behavior
 *    (performance or to reproduce older rendering decisions).
 *  - Use `projected_polygons_overlap` when you require conservative detection
 *    that treats touching-only or tiny intersections as NON-overlap.
 *
 * Backwards compatibility:
 *  - Historically some callers used the name `projected_polygons_overlapV0` to
 *    refer to the legacy behavior. `projected_polygons_overlap_simple` preserves
 *    that semantics. An alias may be provided for compatibility.
 */
static int projected_polygons_overlap_simple(Model3D* model, int f1, int f2) {
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;

    int n1 = faces->vertex_count[f1];
    int n2 = faces->vertex_count[f2];
    if (n1 < 3 || n2 < 3) return 0;

    int minx1 = faces->minx[f1], maxx1 = faces->maxx[f1], miny1 = faces->miny[f1], maxy1 = faces->maxy[f1];
    int minx2 = faces->minx[f2], maxx2 = faces->maxx[f2], miny2 = faces->miny[f2], maxy2 = faces->maxy[f2];
    if (maxx1 < minx2 || maxx2 < minx1 || maxy1 < miny2 || maxy2 < miny1) return 0;

    int off1 = faces->vertex_indices_ptr[f1];
    int off2 = faces->vertex_indices_ptr[f2];

    /* Edge-vs-edge proper intersection with per-edge bbox quick-reject.
     * This avoids expensive orientation tests for clearly separated edges.
     * We use <= in bbox checks so that touching-only edges are treated as
     * non-overlapping (consistent with the semantics). */
    for (int i = 0; i < n1; ++i) {
        int i2 = (i+1) % n1;
        int va = faces->vertex_indices_buffer[off1 + i] - 1;
        int vb = faces->vertex_indices_buffer[off1 + i2] - 1;
        int ax = vtx->x2d[va], ay = vtx->y2d[va];
        int bx = vtx->x2d[vb], by = vtx->y2d[vb];
        int aminx = ax < bx ? ax : bx; int amaxx = ax > bx ? ax : bx;
        int aminy = ay < by ? ay : by; int amaxy = ay > by ? ay : by;
        for (int j = 0; j < n2; ++j) {
            int j2 = (j+1) % n2;
            int vc = faces->vertex_indices_buffer[off2 + j] - 1;
            int vd = faces->vertex_indices_buffer[off2 + j2] - 1;
            int cx = vtx->x2d[vc], cy = vtx->y2d[vc];
            int dx = vtx->x2d[vd], dy = vtx->y2d[vd];
            int cminx = cx < dx ? cx : dx; int cmaxx = cx > dx ? cx : dx;
            int cminy = cy < dy ? cy : dy; int cmaxy = cy > dy ? cy : dy;
            /* quick reject if edge AABBs do not overlap (<= to consider touching as non-overlap) */
            if (amaxx <= cminx || cmaxx <= aminx || amaxy <= cminy || cmaxy <= aminy) continue;
            if (segs_intersect_int(ax,ay,bx,by,cx,cy,dx,dy)) return 1;
        }
    }

    /* Containment tests: check *all* vertices of poly1 against poly2, and vice versa.
     * This avoids missing a containment when the polygon's first vertex lies on a shared
     * boundary point (touching) which is treated as outside. Only checks the candidate
     * point's bbox first (cheap) before the full ray-cast in point_in_poly_int, skipping
     * expensive loops for points obviously outside the other polygon's bbox. */
    for (int ii = 0; ii < n1; ++ii) {
        int vid = faces->vertex_indices_buffer[off1 + ii] - 1;
        if (vid < 0 || vid >= vtx->vertex_count) continue;
        int px = vtx->x2d[vid], py = vtx->y2d[vid];
        if (px < minx2 || px > maxx2 || py < miny2 || py > maxy2) continue;
        if (point_in_poly_int(px, py, faces, vtx, f2, n2)) return 1;
    }

    for (int jj = 0; jj < n2; ++jj) {
        int vid = faces->vertex_indices_buffer[off2 + jj] - 1;
        if (vid < 0 || vid >= vtx->vertex_count) continue;
        int px = vtx->x2d[vid], py = vtx->y2d[vid];
        if (px < minx1 || px > maxx1 || py < miny1 || py > maxy1) continue;
        if (point_in_poly_int(px, py, faces, vtx, f1, n1)) return 1;
    }

    return 0;
}


/* Plane-based pair relation used by `painter_correct` fast path:
 * - Only evaluates geometric plane tests (equivalent to tests 4..7)
 * - Returns -1 if f1 is before f2, 1 if f1 is after f2, 0 if inconclusive
 *
 * This avoids depth/bbox checks which are unnecessary for the local
 * geometric decision in `painter_correct`.
 */
/* Specialized plane-only checks for painter_correct fast path
 * ------------------------------------------------------------
 * These functions implement the geometric depth tests from Newell-Sancha algorithm
 * using only plane equations (no bounding box checks - caller must do those first).
 *
 * - pair_plane_after(f1,f2): returns 1 if f1 is geometrically after (in front of) f2
 *   Implements Test6: all vertices of f2 on opposite side of f1's plane from observer
 *   Falls back to Test7: all vertices of f1 on same side of f2's plane as observer
 *
 * - pair_plane_before(f1,f2): returns 1 if f1 is geometrically before (behind) f2  
 *   Implements Test4: all vertices of f2 on same side of f1's plane as observer
 *   Falls back to Test5: all vertices of f1 on opposite side of f2's plane from observer
 *
 * Both functions:
 *   - Use Fixed64 arithmetic for plane equation evaluation
 *   - Apply epsilon tolerance (0.01) to handle near-coplanar cases
 *   - Return 0 if tests are inconclusive
 *   - Are only valid for front-faces (plane_d > 0); back-faces need special handling
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
    Fixed64 a1 = faces->plane_a[f1]; Fixed64 b1 = faces->plane_b[f1]; Fixed64 c1 = faces->plane_c[f1]; Fixed64 d1 = faces->plane_d[f1];

    /* Test4: Check if all vertices of f2 are on the same side of f1's plane as the observer.
     * If observer is in front of f1 and all f2 vertices are also in front,
     * then f2 occludes f1 -> f1 is before (should be drawn first). */
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
    /* Test5 fallback: Check if all vertices of f1 are on opposite side of f2's plane from observer.
     * If observer is in front of f2 and all f1 vertices are behind f2's plane, then f1 is before f2. */
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

static int pair_plane_after(Model3D* model, int f1, int f2) {
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    
    Fixed32 epsilon = FLOAT_TO_FIXED(0.01f);
    int k;
    int n1 = faces->vertex_count[f1];
    int n2 = faces->vertex_count[f2];
    int offset1 = faces->vertex_indices_ptr[f1];
    int offset2 = faces->vertex_indices_ptr[f2];
    Fixed64 a1 = faces->plane_a[f1]; Fixed64 b1 = faces->plane_b[f1]; Fixed64 c1 = faces->plane_c[f1]; Fixed64 d1 = faces->plane_d[f1];

    /* Test6: Check if all vertices of f2 are on the opposite side of f1's plane from the observer.
     * If observer is in front of f1 (d1 > 0) and all f2 vertices are behind f1's plane,
     * then f1 occludes f2 -> f1 is after (should be drawn later). */
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
    /* Test7 fallback: Check if all vertices of f1 are on the same side of f2's plane as the observer.
     * If observer is in front of f2 and all f1 vertices are also in front, then f1 is after f2. */
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


/* Evaluate tests 1..7 individually for a pair (f1, f2).
 * out[0]..out[6] will be filled with:
 *   -1 => test concludes f1 is before f2
 *    1 => test concludes f1 is after f2
 *    0 => test inconclusive
 */
static void evaluate_pair_tests(Model3D* model, int f1, int f2, int out[7]) {
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    Fixed32 epsilon = FLOAT_TO_FIXED(0.01f);
    int k;

    for (k = 0; k < 7; ++k) out[k] = 0;

    // Test 1: depth separation
    if (faces->z_max[f2] <= faces->z_min[f1]) out[0] = -1;
    else if (faces->z_max[f1] <= faces->z_min[f2]) out[0] = 1;

    // Test 2: X bbox separation
    {
        int minx1 = faces->minx[f1], maxx1 = faces->maxx[f1], minx2 = faces->minx[f2], maxx2 = faces->maxx[f2];
        if (maxx1 <= minx2 || maxx2 <= minx1) out[1] = -1;
    }

    // Test 3: Y bbox separation
    {
        int miny1 = faces->miny[f1], maxy1 = faces->maxy[f1], miny2 = faces->miny[f2], maxy2 = faces->maxy[f2];
        if (maxy1 <= miny2 || maxy2 <= miny1) out[2] = -1;
    }

    // Plane-based tests 4..7
    int n1 = faces->vertex_count[f1];
    int n2 = faces->vertex_count[f2];
    int offset1 = faces->vertex_indices_ptr[f1];
    int offset2 = faces->vertex_indices_ptr[f2];
    Fixed64 a1 = faces->plane_a[f1]; Fixed64 b1 = faces->plane_b[f1]; Fixed64 c1 = faces->plane_c[f1]; Fixed64 d1 = faces->plane_d[f1];
    Fixed64 a2 = faces->plane_a[f2]; Fixed64 b2 = faces->plane_b[f2]; Fixed64 c2 = faces->plane_c[f2]; Fixed64 d2 = faces->plane_d[f2];
    int obs_side1 = 0; int obs_side2 = 0; int side; int all_same_side; int all_opposite_side;

    // Test 4
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

    // Test 5
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

    // Test 6
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

    // Test 7
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


// RAY CAST & INSPECTOR IMPLEMENTATION 
// Returns: 
// 0 if error or undetermined
// 1 if f1 farer than f2
// -1 if f1 closer than f2
/* Compute ray cast distances (tf) for faces f1 and f2 at screen coordinate (cx,cy).
 * Returns 1 and fills *out_tf1/*out_tf2 on success, 0 if denom near zero (coplanar/parallel) or invalid.
 */
/* =====================================================================
 * ray_cast_distances  (OPTIMIZED)
 * ---------------------------------------------------------------------
 * WHAT CHANGED AND WHY IT SHOULD NOT AFFECT THE RESULT
 *
 * 1) Removed Dz. It was always set to the literal 1.0f and only ever used
 *    as a multiplicand (C1 * Dz, C2 * Dz). Multiplying any finite float
 *    by exactly 1.0f is a guaranteed no-op under IEEE-754 (x * 1.0f == x,
 *    bit-for-bit, with no rounding possible) - so C1 * Dz and C2 * Dz have
 *    been replaced directly with C1 and C2. This is not an approximation;
 *    it is an exact identity of the floating-point standard itself.
 *
 * 2) Reordered the work so f2's plane data is only touched, and denom2 is
 *    only computed, after confirming f1's own denom1 test passes. The
 *    original always computed both faces' A/B/C/D and both denom1/denom2
 *    before checking `fabsf(denom1) < 1e-6f || fabsf(denom2) < 1e-6f` and
 *    returning 0 on either failure. Since the function returns the exact
 *    same value (0) regardless of which of the two denom checks fails,
 *    checking denom1 immediately after computing it - before ever reading
 *    f2's plane data - skips work that would have been discarded anyway
 *    whenever f1 alone already disqualifies the pair. Whenever f1 passes,
 *    f2's data is read and denom2 checked exactly as before, with an
 *    identical outcome in every case.
 *
 * 3) D1/D2 are now converted from Fixed64 only once both denom checks
 *    have passed, i.e. only when the function is actually going to
 *    succeed and write to *out_tf1/*out_tf2. In the original, D1/D2 were
 *    read unconditionally up front even though they are never used when
 *    the function returns 0. This is again a pure "skip work whose result
 *    would be discarded" change with no effect on any returned value.
 *
 * Everything else (parameter/bounds validation, Dx/Dy computation, the
 * 1e-6f tolerance, and the final t = -D/denom formulas) is untouched,
 * byte-for-byte, from the original.
 * ===================================================================== */
static int ray_cast_distances(Model3D* model, int f1, int f2, int cx, int cy, float *out_tf1, float *out_tf2) {
    if (!model || !out_tf1 || !out_tf2) return 0;
    FaceArrays3D* faces = &model->faces;
    if (f1 < 0 || f2 < 0 || f1 >= faces->face_count || f2 >= faces->face_count) return 0;

    float proj_scale = FIXED_TO_FLOAT(s_global_proj_scale_fixed);
    float Dx = ((float)cx - (float)CENTRE_X) / proj_scale;
    float Dy = ((float)CENTRE_Y - (float)cy) / proj_scale;

    /* --- test f1 first; bail out before touching f2's data if it already fails --- */
    float A1 = (float)FIXED64_TO_FLOAT(faces->plane_a[f1]);
    float B1 = (float)FIXED64_TO_FLOAT(faces->plane_b[f1]);
    float C1 = (float)FIXED64_TO_FLOAT(faces->plane_c[f1]);

    float denom1 = A1 * Dx + B1 * Dy + C1; /* was "+ C1 * Dz" with Dz == 1.0f */
    if (fabsf(denom1) < 1e-6f) return 0; /* coplanar or parallel */

    /* --- f1 passed: now test f2 --- */
    float A2 = (float)FIXED64_TO_FLOAT(faces->plane_a[f2]);
    float B2 = (float)FIXED64_TO_FLOAT(faces->plane_b[f2]);
    float C2 = (float)FIXED64_TO_FLOAT(faces->plane_c[f2]);

    float denom2 = A2 * Dx + B2 * Dy + C2; /* was "+ C2 * Dz" with Dz == 1.0f */
    if (fabsf(denom2) < 1e-6f) return 0; /* coplanar or parallel */

    /* Both denoms are valid: only now convert D1/D2, since they are the
     * only remaining values still needed to produce the output. */
    float D1 = (float)FIXED64_TO_FLOAT(faces->plane_d[f1]);
    float D2 = (float)FIXED64_TO_FLOAT(faces->plane_d[f2]);

    *out_tf1 = -D1 / denom1;
    *out_tf2 = -D2 / denom2;
    return 1;
}
/* Ray cast at explicit screen coordinate (cx,cy) in projected 2D coords */
static int ray_cast_at(Model3D* model, int f1, int f2, int cx, int cy) {
    float tf1 = 0.0f, tf2 = 0.0f;
    if (!ray_cast_distances(model, f1, f2, cx, cy, &tf1, &tf2)) return 0;
    if (tf1 > tf2) return 1;  /* f1 is farther than f2 */
    else if (tf1 < tf2) return -1; /* f1 is closer than f2 */
    else return 0;
}

/* Hierarchical ray-cast: prefer QD centroid, then SH centroid, then bbox center */
/* ============================================================================
 *  ray_cast_hierarchical — artificial x n zoom patch (self-contained)
 * ----------------------------------------------------------------------------
 *  PROBLEM BEING FIXED
 *  --------------------
 *  ray_cast_hierarchical() decides which of two overlapping faces is in
 *  front by sampling a single screen-space point (a centroid) and casting
 *  a ray through it. That centroid comes from vtx->x2d/y2d, which are
 *  produced by compute2DFromObserver() and rounded to the nearest INTEGER
 *  pixel (FIXED_ROUND_TO_INT).
 *
 *  For a pair of faces whose overlap polygon on screen is small (a few
 *  pixels), that integer rounding is not negligible: it can shift the
 *  sampled centroid enough to land on the wrong side of the true
 *  intersection line between the two faces' planes, flipping the
 *  front/back verdict. This is exactly why the same face pair can give a
 *  different (and sometimes wrong) answer depending on the current
 *  display zoom level: zooming happens to change the projection scale,
 *  which happens to change how much this rounding error matters.
 *
 *  KEY INSIGHT
 *  -----------
 *  compute2DFromObserver() computes, for each vertex:
 *      x2d = xo * scale / zo + CENTRE_X
 *      y2d = CENTRE_Y - yo * scale / zo
 *  where xo/yo/zo are the already-rotated observer-space coordinates and
 *  do NOT depend on `scale`. So varying `scale` alone, at fixed xo/yo/zo,
 *  is an EXACT uniform scaling of the projected 2D points around
 *  (CENTRE_X, CENTRE_Y) — no perspective distortion is introduced. This
 *  means we can locally re-run the same projection formula at a larger
 *  scale, purely to reduce the *relative* size of the integer rounding
 *  error for this one pair, without moving the camera or changing what
 *  is displayed on screen.
 *
 *  STRATEGY
 *  --------
 *  Before running the actual test, we:
 *    1) temporarily double the projection scale,
 *    2) re-project ONLY the vertices belonging to f1 and f2 (not the
 *       whole model — cheap, since a face has at most MAX_FACE_VERTICES
 *       vertices),
 *    3) recompute the cached 2D bounding boxes of f1 and f2 (some of the
 *       downstream tests read faces->minx/maxx/miny/maxy directly instead
 *       of recomputing them from vtx->x2d/y2d, so the cache must be kept
 *       consistent with the patched coordinates),
 *    4) run the existing, UNCHANGED decision logic,
 *    5) restore everything exactly as it was (original vertex 2D coords,
 *       original per-face bbox cache, original global scale).
 *
 *  LOCALITY
 *  --------
 *  This patch is fully self-contained in this file:
 *    - the public function name and signature `ray_cast_hierarchical`
 *      are unchanged, so every existing caller (painter_geoV2,
 *      painter_geoV3, check_sort_repair, check_sort_repair_fast) keeps
 *      working exactly as before, with no code changes on their side;
 *    - the original body is kept verbatim, just renamed to
 *      ray_cast_hierarchical_impl(), and is called from a new wrapper
 *      that does the zoom patch/unpatch around it;
 *    - no global state is left modified after the call returns: the
 *      patch is applied and reverted within a single call to
 *      ray_cast_hierarchical().
 *
 *  A SUBTLETY THIS CODE HANDLES ON PURPOSE
 *  ----------------------------------------
 *  f1 and f2 may share one or more vertices (common for adjacent faces).
 *  If we patched f1's vertices and f2's vertices as two fully independent
 *  loops, a shared vertex would be "saved" a second time (for f2) AFTER
 *  it was already overwritten by f1's patch — corrupting the value used
 *  for restoration. To avoid this, we first collect the UNIQUE set of
 *  vertex indices touched by either face, save each one exactly once
 *  BEFORE any modification, then patch each one exactly once.
 *
 *  KNOWN CAVEATS (left as-is by this patch, not fixed here)
 *  ----------------------------------------------------------
 *  - This patch runs UNCONDITIONALLY on every call, not only on
 *    ambiguous pairs. It adds a small, bounded cost (reprojecting at
 *    most 2 * MAX_FACE_VERTICES vertices and recomputing two bboxes)
 *    even for pairs that were already unambiguous. If this cost ever
 *    matters, an ambiguity check can be added later to skip the patch
 *    for clear-cut pairs, without changing the patch mechanism itself.
 *  - Step 1 of the original logic (QuickDraw region centroid path) still
 *    calls compute_intersection_region_bbox(), which performs a real
 *    QuickDraw PaintRgn() as a side effect. This patch does not remove
 *    that side effect — it simply makes it happen with the scale x 10
 *    coordinates instead of the current display scale.
 * ==========================================================================*/

#define RCH_ZOOM_MAX_VERTS (2 * MAX_FACE_VERTICES)

/* Holds everything needed to fully undo the temporary zoom patch for one
 * face pair: the previous global projection scale, the previous 2D
 * screen coordinates of every touched vertex, and the previous cached
 * 2D bounding boxes of f1 and f2. */
typedef struct {
    Fixed32 saved_scale;                    /* previous s_global_proj_scale_fixed */
    int idx[RCH_ZOOM_MAX_VERTS];            /* unique 0-based vertex indices touched */
    int saved_x2d[RCH_ZOOM_MAX_VERTS];      /* their original x2d, in the same order as idx[] */
    int saved_y2d[RCH_ZOOM_MAX_VERTS];      /* their original y2d, in the same order as idx[] */
    int count;                              /* number of valid entries in idx/saved_x2d/saved_y2d */
    int f1_minx, f1_maxx, f1_miny, f1_maxy; /* previous cached 2D bbox of f1 */
    int f2_minx, f2_maxx, f2_miny, f2_maxy; /* previous cached 2D bbox of f2 */
} RCHZoomPatch;

/* Collect the set of UNIQUE 0-based vertex indices used by f1 and f2,
 * converting from the OBJ-style 1-based indices stored in
 * vertex_indices_buffer. A vertex shared by both faces appears only
 * once in out_idx, which is what makes the save/patch/restore sequence
 * safe even when f1 and f2 are adjacent (sharing an edge or a vertex).
 * out_idx must have room for at least RCH_ZOOM_MAX_VERTS entries.
 * Returns the number of unique indices written. */
static int rch_gather_pair_vertex_indices(FaceArrays3D* faces, int f1, int f2, int* out_idx) {
    int n = 0;

    /* Walk f1's vertices */
    int base1 = faces->vertex_indices_ptr[f1], vc1 = faces->vertex_count[f1];
    for (int k = 0; k < vc1; ++k) {
        int vidx = faces->vertex_indices_buffer[base1 + k] - 1; /* OBJ 1-based -> 0-based */
        int dup = 0;
        for (int m = 0; m < n; ++m) if (out_idx[m] == vidx) { dup = 1; break; }
        if (!dup) out_idx[n++] = vidx;
    }

    /* Walk f2's vertices, skipping anything already collected from f1 */
    int base2 = faces->vertex_indices_ptr[f2], vc2 = faces->vertex_count[f2];
    for (int k = 0; k < vc2; ++k) {
        int vidx = faces->vertex_indices_buffer[base2 + k] - 1;
        int dup = 0;
        for (int m = 0; m < n; ++m) if (out_idx[m] == vidx) { dup = 1; break; }
        if (!dup) out_idx[n++] = vidx;
    }

    return n;
}

/* Recompute the cached 2D bounding box (faces->minx/maxx/miny/maxy) of a
 * single face from its CURRENT vtx->x2d/y2d values. This must be called
 * after patching a face's vertex coordinates, because some downstream
 * tests (e.g. compute_bbox_intersection / compute_bbox_intersection_center,
 * used as ray_cast_hierarchical's final fallback) read this cache
 * directly instead of recomputing it from the vertex arrays. Without
 * this step, the fallback path would silently use stale, pre-zoom
 * bounding boxes while everything else uses the zoomed coordinates. */
static void rch_recompute_face_bbox_cache(Model3D* model, int f) {
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int base = faces->vertex_indices_ptr[f], vc = faces->vertex_count[f];

    int v0 = faces->vertex_indices_buffer[base] - 1;
    int mnx = vtx->x2d[v0], mxx = vtx->x2d[v0];
    int mny = vtx->y2d[v0], mxy = vtx->y2d[v0];

    for (int k = 1; k < vc; ++k) {
        int vidx = faces->vertex_indices_buffer[base + k] - 1;
        int x = vtx->x2d[vidx], y = vtx->y2d[vidx];
        if (x < mnx) mnx = x; if (x > mxx) mxx = x;
        if (y < mny) mny = y; if (y > mxy) mxy = y;
    }

    faces->minx[f] = mnx; faces->maxx[f] = mxx;
    faces->miny[f] = mny; faces->maxy[f] = mxy;
}

/* Apply the temporary zoom: re-project every vertex touched by f1/f2 at
 * `test_scale` instead of the current global projection scale, writing
 * directly into the shared vtx->x2d/y2d arrays (which is what every
 * downstream overlap/centroid/bbox function reads from). Everything
 * needed to undo this is recorded into *p* BEFORE any value is
 * overwritten. Also temporarily overwrites s_global_proj_scale_fixed,
 * since ray_cast_distances() reads it directly to de-project the tested
 * screen point back into a 3D ray. */
static void rch_patch_pair_at_scale(Model3D* model, int f1, int f2, Fixed32 test_scale, RCHZoomPatch* p) {
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    const Fixed32 cx_f = INT_TO_FIXED(CENTRE_X);
    const Fixed32 cy_f = INT_TO_FIXED(CENTRE_Y);

    /* --- Save + swap the global projection scale --- */
    p->saved_scale = s_global_proj_scale_fixed;
    s_global_proj_scale_fixed = test_scale;

    /* --- Save the current (pre-patch) per-face 2D bbox cache --- */
    p->f1_minx = faces->minx[f1]; p->f1_maxx = faces->maxx[f1];
    p->f1_miny = faces->miny[f1]; p->f1_maxy = faces->maxy[f1];
    p->f2_minx = faces->minx[f2]; p->f2_maxx = faces->maxx[f2];
    p->f2_miny = faces->miny[f2]; p->f2_maxy = faces->maxy[f2];

    /* --- Determine which vertices will be touched (deduplicated) --- */
    p->count = rch_gather_pair_vertex_indices(faces, f1, f2, p->idx);

    /* --- Save the TRUE original 2D coords, before touching anything ---
     * This must happen as its own pass, strictly before the patch pass
     * below: doing save+patch in a single interleaved loop would, for a
     * vertex shared between f1 and f2, end up "saving" an already-patched
     * value instead of the original one. */
    for (int i = 0; i < p->count; ++i) {
        p->saved_x2d[i] = vtx->x2d[p->idx[i]];
        p->saved_y2d[i] = vtx->y2d[p->idx[i]];
    }

    /* --- Re-project each unique vertex at test_scale ---
     * Mirrors compute2DFromObserver()'s formula exactly, just with a
     * different scale. xo/yo/zo (rotated observer-space coordinates)
     * are untouched by this — only the projection scale changes. */
    for (int i = 0; i < p->count; ++i) {
        int vidx = p->idx[i];
        Fixed32 inv_zo = FIXED_DIV_64(test_scale, vtx->zo[vidx]);
        Fixed32 xt = FIXED_ADD(FIXED_MUL_64(vtx->xo[vidx], inv_zo), cx_f);
        Fixed32 yt = FIXED_SUB(cy_f, FIXED_MUL_64(vtx->yo[vidx], inv_zo));
        vtx->x2d[vidx] = FIXED_ROUND_TO_INT(xt);
        vtx->y2d[vidx] = FIXED_ROUND_TO_INT(yt);
    }

    /* --- Keep the per-face bbox cache consistent with the patched coords --- */
    rch_recompute_face_bbox_cache(model, f1);
    rch_recompute_face_bbox_cache(model, f2);
}

/* Undo everything rch_patch_pair_at_scale() did: restore the original
 * vtx->x2d/y2d for every touched vertex, restore the original per-face
 * bbox cache for f1 and f2, and restore the original global projection
 * scale. After this call, the model is left exactly as it was before
 * the patch — no visible or hidden side effect remains. */
static void rch_unpatch_pair(Model3D* model, RCHZoomPatch* p, int f1, int f2) {
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;

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

/* ----------------------------------------------------------------------
 * Original decision logic, UNCHANGED, just renamed.
 * This is exactly the previous body of ray_cast_hierarchical(): try the
 * QuickDraw region centroid first, then the Sutherland-Hodgman centroid,
 * then fall back to the bbox-intersection center. Whatever coordinates
 * happen to be in vtx->x2d/y2d and faces->minx/maxx/... at the time this
 * runs are what it will use — it has no knowledge of the zoom patch
 * wrapped around it.
 * ---------------------------------------------------------------------- */
static int ray_cast_hierarchical_impl(Model3D* model, int f1, int f2) {
    if (!model) return 0;
    int cx = 0, cy = 0; long long area2 = 0; int rc = 0; int fallback = 0;

    /* 1) Prefer QuickDraw region centroid (most representative for complex overlaps) */
    if (compute_intersection_centroid_ordered_qd_fixed(model, f1, f2, &cx, &cy, &area2) && area2 != 0) {
        rc = ray_cast_at(model, f1, f2, cx, cy);
        if (rc != 0) return rc;
    }

    /* 2) Try Sutherland–Hodgman centroid */
    area2 = 0; fallback = 0;
    if (compute_intersection_centroid_ordered_fixed_tryboth(model, f1, f2, &cx, &cy, &area2, &fallback) && area2 != 0) {
        rc = ray_cast_at(model, f1, f2, cx, cy);
        if (rc != 0) return rc;
    }

    /* 3) Fallback to bbox center */
    if (compute_bbox_intersection_center(model, f1, f2, &cx, &cy)) {
        return ray_cast_at(model, f1, f2, cx, cy);
    }

    return 0;
}

/* ----------------------------------------------------------------------
 * New public entry point — SAME NAME, SAME SIGNATURE as before, so every
 * existing caller (painter_geoV2, painter_geoV3, check_sort_repair,
 * check_sort_repair_fast) keeps compiling and behaving the same way
 * from their point of view. The only difference is what happens inside:
 * the projection scale is temporarily multiplied by 10 and the two faces'
 * vertices are re-projected before the (unchanged) decision logic runs,
 * then everything is restored before returning.
 * ---------------------------------------------------------------------- */
static int ray_cast_hierarchical(Model3D* model, int f1, int f2) {
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    if (f1 < 0 || f2 < 0 || f1 >= faces->face_count || f2 >= faces->face_count) return 0;

    RCHZoomPatch patch;

    /* Systematic x n zoom, applied on every call (not conditioned on
     * detecting ambiguity first — see "KNOWN CAVEATS" above). */
    Fixed32 test_scale = FIXED_MUL_64(s_global_proj_scale_fixed, FLOAT_TO_FIXED(10.0f));
    // Fixed32 test_scale = s_global_proj_scale_fixed << 4;

    rch_patch_pair_at_scale(model, f1, f2, test_scale, &patch);
    int rc = ray_cast_hierarchical_impl(model, f1, f2);
    rch_unpatch_pair(model, &patch, f1, f2);

    return rc;
}
/* Compute center of intersection of two faces' projected axis-aligned bboxes.
 * Returns 1 and writes (*outx,*outy) on success, 0 if no intersection or invalid ids.
 */
static int compute_bbox_intersection_center(Model3D* model, int f1, int f2, int *outx, int *outy) {
    if (!model || !outx || !outy) return 0;
    int ix0, iy0, ix1, iy1;
    if (!compute_bbox_intersection(model, f1, f2, &ix0, &iy0, &ix1, &iy1)) return 0;
    *outx = (ix0 + ix1) / 2;
    *outy = (iy0 + iy1) / 2;
    return 1;
}

/* Compute axis-aligned integer intersection bbox of projected face bboxes.
 * Returns 1 and fills (*ix0,*iy0,*ix1,*iy1) on success, 0 if no intersection.
 */
static int compute_bbox_intersection(Model3D* model, int f1, int f2, int *ix0, int *iy0, int *ix1, int *iy1) {
    if (!model || !ix0 || !iy0 || !ix1 || !iy1) return 0;
    FaceArrays3D* faces = &model->faces;
    if (f1 < 0 || f2 < 0 || f1 >= faces->face_count || f2 >= faces->face_count) return 0;
    int minx1 = faces->minx[f1], maxx1 = faces->maxx[f1], miny1 = faces->miny[f1], maxy1 = faces->maxy[f1];
    int minx2 = faces->minx[f2], maxx2 = faces->maxx[f2], miny2 = faces->miny[f2], maxy2 = faces->maxy[f2];
    int a0 = (minx1 > minx2) ? minx1 : minx2;
    int a1 = (maxx1 < maxx2) ? maxx1 : maxx2;
    int b0 = (miny1 > miny2) ? miny1 : miny2;
    int b1 = (maxy1 < maxy2) ? maxy1 : maxy2;
    if (a0 > a1 || b0 > b1) return 0;
    *ix0 = a0; *iy0 = b0; *ix1 = a1; *iy1 = b1;
    return 1;
}

/* Integer point-in-polygon test using ray-crossing on integer vertex arrays.
 * Returns 1 if point (px,py) is inside or on edge, 0 otherwise.
 */
static int point_in_poly_arrays_int(int px, int py, int n, const int vx[], const int vy[]) {
    if (n < 3) return 0;
    int inside = 0;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        int xi = vx[i], yi = vy[i];
        int xj = vx[j], yj = vy[j];
        /* Check if point is exactly on a vertex */
        if (px == xi && py == yi) return 1;
        /* Ray crossing: test if edge (j->i) crosses horizontal line at py */
        int yi_gt_py = (yi > py);
        int yj_gt_py = (yj > py);
        if (yi_gt_py != yj_gt_py) {
            /* Compute intersection X coordinate in integer-safe way: xi + (py - yi) * (xj - xi) / (yj - yi) */
            long long dx = (long long)(xj - xi);
            long long dy = (long long)(yj - yi);
            long long lhs = (long long)(px - xi) * dy;
            long long rhs = (long long)(py - yi) * dx;
            if ((dy > 0 && lhs < rhs) || (dy < 0 && lhs > rhs)) inside = !inside;
        }
    }
    return inside;
}


static long long orient_ll(long long ax,long long ay,long long bx,long long by,long long cx,long long cy) {
    return (bx-ax)*(cy-ay) - (by-ay)*(cx-ax);
}
static int on_seg_ll(long long ax,long long ay,long long bx,long long by,long long cx,long long cy) {
    if (( (ax<=cx && cx<=bx) || (bx<=cx && cx<=ax) ) && ((ay<=cy && cy<=by) || (by<=cy && cy<=ay))) return 1;
    return 0;
}
/* tolerance (in pixels) below which an intersection is considered too close
 * to a vertex/edge and thus ignored. Default set by user request to 1.0 px. */
static double segs_intersect_tol_px = 1.0;
/*
 * segs_intersect_int_dbl
 * ----------------------
 * Double-precision core used by the integer wrapper and unit tests.
 *
 * Steps and rationale (detailed) :
 * 1) Orientation test (proper intersection):
 *    - We round coordinates to the nearest integer when computing orientations
 *      (via floor(x+0.5)) in order to preserve the original integer semantics
 *      used elsewhere in the codebase (avoids surprising behaviour due to tiny
 *      floating-point perturbations when deciding "proper" orientation).
 *    - A proper intersection requires strict sign differences for the pair of
 *      orientations on each segment (standard robust test).
 *
 * 2) Intersection point computation:
 *    - If orientation indicates a proper intersection, we compute the exact
 *      intersection point in double precision (t parameter and point coords).
 *    - Guard against near-parallel segments using a small epsilon on the
 *      determinant (fabs(denom) < 1e-12) to avoid numerical instability.
 *
 * 3) Proximity filtering (pixel tolerance):
 *    - We consider a global tolerance `segs_intersect_tol_px` (in pixels).
 *    - If the computed intersection point lies within `tol` of any of the 4
 *      segment endpoints, we treat the intersection as spurious and return 0.
 *      This removes sub-pixel / endpoint-touch cases that would otherwise be
 *      reported as proper intersections due to integer orientation tests.
 *
 * 4) Endpoint-segment proximity test :
 *    - Additionally, if any of the 4 segment endpoints is within `tol` of
 *      the other segment (distance point→segment < tol), we also ignore the
 *      intersection (returned 0). This handles cases where an endpoint lies
 *      very near the interior of the other segment but rounding/orientation
 *      might have flagged a proper intersection.
 *
 * 5) Final decision : return 1 only if the intersection is proper and passes
 *    both proximity filters. This yields a conservative behavior tuned to
 *    avoid visual artifacts from sub-pixel intersections while preserving
 *    real, well-separated crossings.
 */
/**
 * Compute squared distance (in fixed units) from point P(px,py) to segment AB (ax,ay)-(bx,by).
 * Returns an unsigned long long representing (distance_in_fixed_units)^2.
 * Implementation notes:
 *  - The function works entirely in integer arithmetic to avoid floating point.
 *  - We compute numsq = (wx*d - c*vx)^2 + (wy*d - c*vy)^2 and then divide by d^2,
 *    performing division before multiplying by FIXED_SCALE^2 to avoid overflow.
 */
static unsigned long long point_seg_dist2_fixed_int(int px,int py,int ax,int ay,int bx,int by) {
    long long vx = (long long)bx - (long long)ax, vy = (long long)by - (long long)ay;
    long long wx = (long long)px - (long long)ax, wy = (long long)py - (long long)ay;
    long long c = vx*wx + vy*wy;
    long long d = vx*vx + vy*vy;
    if (d == 0) {
        long long dx0 = px - ax, dy0 = py - ay;
        unsigned long long r = (unsigned long long)dx0*(unsigned long long)dx0 + (unsigned long long)dy0*(unsigned long long)dy0;
        return r * (unsigned long long)FIXED_SCALE * (unsigned long long)FIXED_SCALE;
    }
    if (c <= 0) {
        long long dx0 = px - ax, dy0 = py - ay;
        unsigned long long r = (unsigned long long)dx0*(unsigned long long)dx0 + (unsigned long long)dy0*(unsigned long long)dy0;
        return r * (unsigned long long)FIXED_SCALE * (unsigned long long)FIXED_SCALE;
    } else if (c >= d) {
        long long dx0 = px - bx, dy0 = py - by;
        unsigned long long r = (unsigned long long)dx0*(unsigned long long)dx0 + (unsigned long long)dy0*(unsigned long long)dy0;
        return r * (unsigned long long)FIXED_SCALE * (unsigned long long)FIXED_SCALE;
    } else {
        long long numx = wx * d - c * vx;
        long long numy = wy * d - c * vy;
        unsigned long long numsq = (unsigned long long)numx * (unsigned long long)numx + (unsigned long long)numy * (unsigned long long)numy;
        unsigned long long den = (unsigned long long)d * (unsigned long long)d;
        unsigned long long q = numsq / den;
        unsigned long long rem = numsq % den;
        unsigned long long scale2 = (unsigned long long)FIXED_SCALE * (unsigned long long)FIXED_SCALE;
        unsigned long long out = q * scale2 + (rem * scale2) / den;
        return out;
    }
}

static int segs_intersect_int_fixed64(int x1,int y1,int x2,int y2,int x3,int y3,int x4,int y4) {
    long long o1 = orient_ll(x1,y1,x2,y2,x3,y3);
    long long o2 = orient_ll(x1,y1,x2,y2,x4,y4);
    long long o3 = orient_ll(x3,y3,x4,y4,x1,y1);
    long long o4 = orient_ll(x3,y3,x4,y4,x2,y2);
    /* Proper intersection only: require strict orientation differences */
    if (!(((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) && ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0)))) return 0;

    long long dx1 = x2 - x1, dy1 = y2 - y1;
    long long dx2 = x4 - x3, dy2 = y4 - y3;
    long long denom = dx1 * (-dy2) - dy1 * (-dx2);
    if (denom == 0) return 0; /* numerically parallel */

    long long tnum = (long long)(x3 - x1) * (-dy2) - (long long)(y3 - y1) * (-dx2);

    /* intersection point in Fixed64 */
    Fixed64 t_fixed = ((Fixed64)tnum << FIXED_SHIFT) / (Fixed64)denom;
    Fixed64 ix_fixed = (((Fixed64)x1) << FIXED_SHIFT) + ((t_fixed * (((Fixed64)dx1) << FIXED_SHIFT)) >> FIXED_SHIFT);
    Fixed64 iy_fixed = (((Fixed64)y1) << FIXED_SHIFT) + ((t_fixed * (((Fixed64)dy1) << FIXED_SHIFT)) >> FIXED_SHIFT);

    /* tolerance in fixed units */
    Fixed64 tol_fixed = (Fixed64)(segs_intersect_tol_px * (double)FIXED_SCALE + 0.5);
    unsigned long long tol2_fixed = (unsigned long long)tol_fixed * (unsigned long long)tol_fixed; /* fixed^2 */

    /* distance to endpoints (in fixed units squared) */
    long long ddx, ddy; unsigned long long dist2u;
    ddx = (long long)ix_fixed - ((long long)x1 << FIXED_SHIFT); ddy = (long long)iy_fixed - ((long long)y1 << FIXED_SHIFT);
    dist2u = (unsigned long long)ddx * (unsigned long long)ddx + (unsigned long long)ddy * (unsigned long long)ddy; if (dist2u < tol2_fixed) return 0;
    ddx = (long long)ix_fixed - ((long long)x2 << FIXED_SHIFT); ddy = (long long)iy_fixed - ((long long)y2 << FIXED_SHIFT);
    dist2u = (unsigned long long)ddx * (unsigned long long)ddx + (unsigned long long)ddy * (unsigned long long)ddy; if (dist2u < tol2_fixed) return 0;
    ddx = (long long)ix_fixed - ((long long)x3 << FIXED_SHIFT); ddy = (long long)iy_fixed - ((long long)y3 << FIXED_SHIFT);
    dist2u = (unsigned long long)ddx * (unsigned long long)ddx + (unsigned long long)ddy * (unsigned long long)ddy; if (dist2u < tol2_fixed) return 0;
    ddx = (long long)ix_fixed - ((long long)x4 << FIXED_SHIFT); ddy = (long long)iy_fixed - ((long long)y4 << FIXED_SHIFT);
    dist2u = (unsigned long long)ddx * (unsigned long long)ddx + (unsigned long long)ddy * (unsigned long long)ddy; if (dist2u < tol2_fixed) return 0;



    if (point_seg_dist2_fixed_int(x1,y1,x3,y3,x4,y4) < tol2_fixed) return 0;
    if (point_seg_dist2_fixed_int(x2,y2,x3,y3,x4,y4) < tol2_fixed) return 0;
    if (point_seg_dist2_fixed_int(x3,y3,x1,y1,x2,y2) < tol2_fixed) return 0;
    if (point_seg_dist2_fixed_int(x4,y4,x1,y1,x2,y2) < tol2_fixed) return 0;

    return 1;
}

/* Double-based regression helpers removed: Fixed (integer) implementations are now authoritative.
 * The old `segs_intersect_int_dbl` and associated double point/segment helpers were deleted
 * to remove redundant floating-point paths and reduce maintenance surface.
 */
static int segs_intersect_int(int x1,int y1,int x2,int y2,int x3,int y3,int x4,int y4) {
    return segs_intersect_int_fixed64(x1,y1,x2,y2,x3,y3,x4,y4);
}
static int point_in_poly_int(int px,int py,FaceArrays3D* faces, VertexArrays3D* vtx, int f, int n) {
    int cnt = 0; int off = faces->vertex_indices_ptr[f];
    for (int i = 0; i < n; ++i) {
        int j = (i+1)%n;
        int vi = faces->vertex_indices_buffer[off + i] - 1;
        int vj = faces->vertex_indices_buffer[off + j] - 1;
        int xi = vtx->x2d[vi], yi = vtx->y2d[vi];
        int xj = vtx->x2d[vj], yj = vtx->y2d[vj];
        /* If the point lies exactly on the edge, consider it OUTSIDE (no overlap).
         * Use orient==0 + on-segment test to detect boundary points. */
        if (orient_ll(xi, yi, xj, yj, px, py) == 0 && on_seg_ll(xi, yi, xj, yj, px, py)) return 0;
        if (((yi > py) != (yj > py)) && (px < (long long)(xj - xi) * (py - yi) / (yj - yi) + xi)) cnt++;
    }
    return (cnt & 1);
}

/* Return 1 if faces f1 and f2 have identical 2D vertex sequences (allowing rotation
 * and reversed order). Comparison uses exact integer 2D coordinates. */
static int faces_vertices_equal(FaceArrays3D* faces, VertexArrays3D* vtx, int f1, int f2) {
    int n1 = faces->vertex_count[f1];
    int n2 = faces->vertex_count[f2];
    if (n1 != n2 || n1 <= 0) return 0;
    int off1 = faces->vertex_indices_ptr[f1];
    int off2 = faces->vertex_indices_ptr[f2];

    /* Forward rotation check */
    for (int shift = 0; shift < n1; ++shift) {
        int ok = 1;
        for (int k = 0; k < n1; ++k) {
            int idx1 = faces->vertex_indices_buffer[off1 + k] - 1;
            int idx2 = faces->vertex_indices_buffer[off2 + ((shift + k) % n1)] - 1;
            if (idx1 < 0 || idx2 < 0) { ok = 0; break; }
            if (vtx->x2d[idx1] != vtx->x2d[idx2] || vtx->y2d[idx1] != vtx->y2d[idx2]) { ok = 0; break; }
        }
        if (ok) return 1;
    }

    /* Reverse rotation check */
    for (int shift = 0; shift < n1; ++shift) {
        int ok = 1;
        for (int k = 0; k < n1; ++k) {
            int idx1 = faces->vertex_indices_buffer[off1 + k] - 1;
            int idx2 = faces->vertex_indices_buffer[off2 + ((shift - k + n1) % n1)] - 1;
            if (idx1 < 0 || idx2 < 0) { ok = 0; break; }
            if (vtx->x2d[idx1] != vtx->x2d[idx2] || vtx->y2d[idx1] != vtx->y2d[idx2]) { ok = 0; break; }
        }
        if (ok) return 1;
    }
    return 0;
}

static int segs_intersect_unit_tests(void) {
    struct {
        double x1,y1,x2,y2,x3,y3,x4,y4; int expect; const char *name;
    } tests[] = {
        {233.0,114.0, 98.0,88.0, 191.0,106.0, 212.0,101.0, 0, "case_7_vs_45"},
        {0.0,0.0, 10.0,0.0, 5.0,-5.0, 5.0,5.0, 1, "orthogonal_mid"},
        {0.0,0.0, 10.0,0.0, 9.1,-1.0, 9.1,1.0, 0, "near_endpoint"},
        {0.0,0.0, 10.0,0.0, 10.0,-1.0, 10.0,1.0, 0, "touch_endpoint"},
        {0.0,0.0, 10.0,0.0, 1.0,-1.0, 1.0,1.0, 1, "near_mid"}
    };
    int n = sizeof(tests)/sizeof(tests[0]);
    printf("Running segs_intersect unit tests (tol=%.3f px)\n", segs_intersect_tol_px);
    int failed = 0;
    for (int i = 0; i < n; ++i) {
        int res_fixed = segs_intersect_int((int)tests[i].x1,(int)tests[i].y1,(int)tests[i].x2,(int)tests[i].y2,(int)tests[i].x3,(int)tests[i].y3,(int)tests[i].x4,(int)tests[i].y4);
        printf(" test %s: expected=%d fixed=%d\n", tests[i].name, tests[i].expect, res_fixed);
        if (res_fixed != tests[i].expect) { printf("  -> FIXED FAIL\n"); failed++; } else printf("  -> FIXED PASS\n");
    }
    if (failed == 0) printf("All segs_intersect tests passed\n"); else printf("%d segs_intersect tests FAILED\n", failed);
    return failed;
}

static int compute_intersection_centroid_ordered_fixed(Model3D* model, int subj, int clip, int* outx, int* outy, long long* out_area2) {
    /* Robust integer implementation for diagnostics and fixed-area comparisons.
     * - Clips `subj` by `clip` using Sutherland–Hodgman (double intermediates).
     * - Produces integer output vertices (rounded), computes signed raw area2
     *   (shoelace: 2 * pixel-area) and centroid (integer-rounded).
     * - Returns 1 if a clipped polygon (>=3 verts) was produced; 0 otherwise.
     * - Writes *out_area2 in **pixel^2** units (raw_area2).  This matches
     *   compute_intersection_centroid_ordered_qd_fixed and simplifies callers.
     */
    if (!model || out_area2 == NULL) return 0;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    if (subj < 0 || clip < 0 || subj >= faces->face_count || clip >= faces->face_count) return 0;

    int sn = faces->vertex_count[subj];
    int cn = faces->vertex_count[clip];
    if (sn < 3 || cn < 3) {
        debug_clip_fixed_vcount = 0; debug_clip_raw_area2 = 0; debug_clip_fixed_area = 0.0; return 0;
    }

    /* Watchdog and iteration-cap disabled for repro testing (temporary). */
    long local_start = GetTick();
    int sh_steps = 0; /* kept for diagnostics but cap removed */

    /* Gather integer input polygons (screen coords) */
    int *sx = (int*)malloc(sizeof(int) * sn);
    int *sy = (int*)malloc(sizeof(int) * sn);
    int *cx = (int*)malloc(sizeof(int) * cn);
    int *cy = (int*)malloc(sizeof(int) * cn);
    if (!sx || !sy || !cx || !cy) {
        if (sx) free(sx); if (sy) free(sy); if (cx) free(cx); if (cy) free(cy);
        debug_clip_fixed_vcount = 0; debug_clip_raw_area2 = 0; debug_clip_fixed_area = 0.0; *out_area2 = 0; return 0;
    }

    /* alloc OK (persistent log removed) */

    for (int i = 0; i < sn; ++i) {
        int vi = faces->vertex_indices_buffer[faces->vertex_indices_ptr[subj] + i] - 1;
        sx[i] = vtx->x2d[vi]; sy[i] = vtx->y2d[vi];
    }

    /* subj verts logging removed */

    for (int i = 0; i < cn; ++i) {
        int vi = faces->vertex_indices_buffer[faces->vertex_indices_ptr[clip] + i] - 1;
        cx[i] = vtx->x2d[vi]; cy[i] = vtx->y2d[vi];
    }

    /* clip verts logging removed */
    /* debug trace removed */

    /* Normalize winding: ensure both polygons use CCW orientation so SH is deterministic
     * regardless of the input vertex order. If signed area < 0, reverse vertex order. */
    {
        long long area2_subj = 0;
        /* area2_subj trace removed */
        for (int i = 0; i < sn; ++i) {
            /* area2_subj per-vertex trace removed */
            int j = (i + 1) % sn;
            area2_subj += (long long)sx[i] * (long long)sy[j] - (long long)sx[j] * (long long)sy[i];
        }
        /* record signed subject area for repro pair */
        /* area2_subj value trace removed */
        if (area2_subj < 0) {
            /* reverse subj arrays in-place */
            for (int a = 0, b = sn - 1; a < b; ++a, --b) {
                int tx = sx[a]; sx[a] = sx[b]; sx[b] = tx;
                int ty = sy[a]; sy[a] = sy[b]; sy[b] = ty;
            }
        }

        long long area2_clip = 0;
        /* area2_clip trace removed */
        for (int i = 0; i < cn; ++i) {
            int j = (i + 1) % cn;
            area2_clip += (long long)cx[i] * (long long)cy[j] - (long long)cx[j] * (long long)cy[i];
        }
        if (area2_clip < 0) {
            /* reverse clip arrays in-place */
            for (int a = 0, b = cn - 1; a < b; ++a, --b) {
                int tx = cx[a]; cx[a] = cx[b]; cx[b] = tx;
                int ty = cy[a]; cy[a] = cy[b]; cy[b] = ty;
            }
        }
    }

    /* Optional detailed debug dump for failing cases (writes centroidlog.txt) */
    FILE* dlog = NULL;
    if ((debug_overlap_subj == subj && debug_overlap_clip == clip) || (subj == 12 && clip == 50)) {
        //dlog = fopen("centroidlog.txt", "w");
        if (dlog) {
            fprintf(dlog, "--- compute_intersection_centroid_ordered_fixed debug (subj=%d clip=%d) ---\n", subj, clip);
            fprintf(dlog, "input subj verts (count=%d):\n", sn);
            for (int i = 0; i < sn; ++i) fprintf(dlog, "%d %d\n", sx[i], sy[i]);
            fprintf(dlog, "input clip verts (count=%d):\n", cn);
            for (int i = 0; i < cn; ++i) fprintf(dlog, "%d %d\n", cx[i], cy[i]);
            fflush(dlog);
        }
    }

    /* Working buffers on heap to avoid stack exhaustion for large polygons */
    double *buf_x1 = (double*)malloc(sizeof(double) * DEBUG_CLIP_MAX);
    double *buf_y1 = (double*)malloc(sizeof(double) * DEBUG_CLIP_MAX);
    double *buf_x2 = (double*)malloc(sizeof(double) * DEBUG_CLIP_MAX);
    double *buf_y2 = (double*)malloc(sizeof(double) * DEBUG_CLIP_MAX);
    int *out_px = (int*)malloc(sizeof(int) * DEBUG_CLIP_MAX);
    int *out_py = (int*)malloc(sizeof(int) * DEBUG_CLIP_MAX);
    if (!buf_x1 || !buf_y1 || !buf_x2 || !buf_y2 || !out_px || !out_py) {
        free(sx); free(sy); free(cx); free(cy);
        if (buf_x1) free(buf_x1); if (buf_y1) free(buf_y1); if (buf_x2) free(buf_x2); if (buf_y2) free(buf_y2);
        if (out_px) free(out_px); if (out_py) free(out_py);
        debug_clip_fixed_vcount = 0; debug_clip_raw_area2 = 0; debug_clip_fixed_area = 0.0; *out_area2 = 0; return 0;
    }
/* allocs OK trace removed */

    int cur_n = sn;
    /* before copy trace removed */
    for (int i = 0; i < cur_n && i < DEBUG_CLIP_MAX; ++i) {
        buf_x1[i] = (double)sx[i]; buf_y1[i] = (double)sy[i];
        /* per-vertex copy trace removed */
    }
    /* after copy trace removed */

    /* Sutherland–Hodgman: clip against each edge of `clip` */
    for (int ci = 0; ci < cn && cur_n > 0; ++ci) {
        /* watchdog disabled for repro testing — do not bail out here. */
        /* SH outer-loop entry trace for repro pair */
        /* SH outer trace removed */
        int ci2 = (ci + 1) % cn;
        double cx1d = (double)cx[ci], cy1d = (double)cy[ci];
        double cx2d = (double)cx[ci2], cy2d = (double)cy[ci2];
        int out_n_tmp = 0;
        for (int si = 0; si < cur_n; ++si) {
            /* iteration cap disabled for repro testing */
            ++sh_steps;
            int sj = (si + 1) % cur_n;
            double sx1d = buf_x1[si], sy1d = buf_y1[si];
            double sx2d = buf_x1[sj], sy2d = buf_y1[sj];
            double side1 = (cx2d - cx1d) * (sy1d - cy1d) - (cy2d - cy1d) * (sx1d - cx1d);
            double side2 = (cx2d - cx1d) * (sy2d - cy1d) - (cy2d - cy1d) * (sx2d - cx1d);
            int inside1 = (side1 >= 0.0);
            int inside2 = (side2 >= 0.0);

            /* verbose SH inner logging removed */
            /* per-edge debug trace removed */

            if (inside1 && inside2) {
                if (out_n_tmp < DEBUG_CLIP_MAX) { buf_x2[out_n_tmp] = sx2d; buf_y2[out_n_tmp] = sy2d; out_n_tmp++; }
            } else if (inside1 && !inside2) {
                double dx = sx2d - sx1d, dy = sy2d - sy1d;
                double ex = cx2d - cx1d, ey = cy2d - cy1d;
                /* denom = cross(segment, clipEdge) */
                double denom = dx * ey - dy * ex;
                double t = 0.0;
                if (fabs(denom) > 1e-12) t = ((cx1d - sx1d) * ey - (cy1d - sy1d) * ex) / denom;
                double ix = sx1d + t * dx; double iy = sy1d + t * dy;
                /* verbose entering/exiting log removed */
                if (out_n_tmp < DEBUG_CLIP_MAX) { buf_x2[out_n_tmp] = ix; buf_y2[out_n_tmp] = iy; out_n_tmp++; }
            } else if (!inside1 && inside2) {
                double dx = sx2d - sx1d, dy = sy2d - sy1d;
                double ex = cx2d - cx1d, ey = cy2d - cy1d;
                /* denom = cross(segment, clipEdge) */
                double denom = dx * ey - dy * ex;
                double t = 0.0;
                if (fabs(denom) > 1e-12) t = ((cx1d - sx1d) * ey - (cy1d - sy1d) * ex) / denom;
                double ix = sx1d + t * dx; double iy = sy1d + t * dy;
                /* verbose entering/exiting log removed */
                if (out_n_tmp < DEBUG_CLIP_MAX) { buf_x2[out_n_tmp] = ix; buf_y2[out_n_tmp] = iy; out_n_tmp++; }
                if (out_n_tmp < DEBUG_CLIP_MAX) { buf_x2[out_n_tmp] = sx2d; buf_y2[out_n_tmp] = sy2d; out_n_tmp++; }
            }
        }
        cur_n = out_n_tmp;
        for (int k = 0; k < cur_n; ++k) { buf_x1[k] = buf_x2[k]; buf_y1[k] = buf_y2[k]; }
        if (dlog) {
            fprintf(dlog, "after clip edge %d -> vertex_count=%d\n", ci, cur_n);
            for (int kk = 0; kk < cur_n; ++kk) fprintf(dlog, "  %.6f %.6f\n", buf_x1[kk], buf_y1[kk]);
            fflush(dlog);
        }
    }

    /* Round and remove duplicate/colinear vertices into out_px/out_py */
    int final_n = 0;
    for (int i = 0; i < cur_n; ++i) {
        int px = (int)floor(buf_x1[i] + 0.5);
        int py = (int)floor(buf_y1[i] + 0.5);
        if (final_n > 0 && out_px[final_n-1] == px && out_py[final_n-1] == py) continue;
        if (final_n >= 2) {
            int x1 = out_px[final_n-2], y1 = out_py[final_n-2];
            int x2 = out_px[final_n-1], y2 = out_py[final_n-1];
            long long cross = (long long)(x2 - x1) * (long long)(py - y1) - (long long)(y2 - y1) * (long long)(px - x1);
            if (cross == 0) { out_px[final_n-1] = px; out_py[final_n-1] = py; continue; }
        }
        if (final_n < DEBUG_CLIP_MAX) { out_px[final_n] = px; out_py[final_n] = py; final_n++; }
    }
    if (final_n >= 2 && out_px[0] == out_px[final_n-1] && out_py[0] == out_py[final_n-1]) final_n--;

    if (final_n < 3) {
        /* final_n < 3 trace removed */
        debug_clip_fixed_vcount = 0; debug_clip_raw_area2 = 0; debug_clip_fixed_area = 0.0; *out_area2 = 0;
        if (dlog) { fprintf(dlog, "final_n < 3 -> no polygon\n"); fclose(dlog); }
        free(sx); free(sy); free(cx); free(cy);
        free(buf_x1); free(buf_y1); free(buf_x2); free(buf_y2); free(out_px); free(out_py);
        return 0;
    }

    /* Copy to debug buffers */
    if (final_n > DEBUG_CLIP_MAX) final_n = DEBUG_CLIP_MAX;
    for (int i = 0; i < final_n; ++i) { debug_clip_fixed_vx[i] = out_px[i]; debug_clip_fixed_vy[i] = out_py[i]; }
    debug_clip_fixed_vcount = final_n;
    /* final_n debug log removed */
    if (dlog) {
        fprintf(dlog, "rounded final polygon (count=%d):\n", final_n);
        for (int i = 0; i < final_n; ++i) fprintf(dlog, "%d %d\n", out_px[i], out_py[i]);
        fflush(dlog);
    }

    /* Compute signed raw area2 and centroid numerators */
    long long raw_area2_signed = 0;
    long long cx_num = 0, cy_num = 0;
    for (int i = 0; i < final_n; ++i) {
        int j = (i + 1) % final_n;
        long long xi = (long long)out_px[i], yi = (long long)out_py[i];
        long long xj = (long long)out_px[j], yj = (long long)out_py[j];
        long long cross = xi * yj - yi * xj;
        raw_area2_signed += cross;
        cx_num += (xi + xj) * cross;
        cy_num += (yi + yj) * cross;
    }

    if (raw_area2_signed == 0) {
        debug_clip_raw_area2 = 0; debug_clip_fixed_area = 0.0; *out_area2 = 0;
        if (dlog) { fprintf(dlog, "raw_area2_signed == 0 -> degenerate polygon\n"); fflush(dlog); }
        if (outx) *outx = out_px[0]; if (outy) *outy = out_py[0];
        free(sx); free(sy); free(cx); free(cy);
        free(buf_x1); free(buf_y1); free(buf_x2); free(buf_y2); free(out_px); free(out_py);
        if (dlog) fclose(dlog);
        return 1;
    }

    long long abs_raw = (raw_area2_signed >= 0) ? raw_area2_signed : -raw_area2_signed;
    debug_clip_raw_area2 = abs_raw;
    debug_clip_fixed_area = fabs((double)raw_area2_signed) * 0.5;

    /* Centroid formula: C = (1/(6*A)) * sum((xi+xj)*cross), with raw_area2_signed == 2*A
     * => Cx = cx_num / (3 * raw_area2_signed)
     */
    long long numer_x = cx_num;
    long long numer_y = cy_num;
    long long denom = 3 * raw_area2_signed;
    if (denom < 0) { denom = -denom; numer_x = -numer_x; numer_y = -numer_y; }
    long long cx_i = (numer_x >= 0) ? (numer_x + denom/2) / denom : -(( -numer_x + denom/2) / denom);
    long long cy_i = (numer_y >= 0) ? (numer_y + denom/2) / denom : -(( -numer_y + denom/2) / denom);
    if (outx) *outx = (int)cx_i; if (outy) *outy = (int)cy_i;

    /* Return raw area2 in pixel^2 (absolute value of signed raw_area2). */
    if (out_area2) *out_area2 = (long long)abs_raw;

    /* final DONE debug log removed */

    free(sx); free(sy); free(cx); free(cy);
    free(buf_x1); free(buf_y1); free(buf_x2); free(buf_y2); free(out_px); free(out_py);
    return 1;
}

/* Try both clipping orders: call compute_intersection_centroid_ordered_fixed(subj,clip)
 * and if it produces fewer than 3 output vertices, try compute_intersection_centroid_ordered_fixed(clip,subj).
 * This keeps existing callers intact and provides a single-call fallback for UI/diagnostic use.
 */
static int compute_intersection_centroid_ordered_fixed_tryboth(Model3D* model, int subj, int clip, int* outx, int* outy, long long* out_area2, int* out_fallback) {
    if (!model || out_area2 == NULL) return 0;
    if (out_fallback) *out_fallback = 0;

    /* Try primary order first */
    int res = compute_intersection_centroid_ordered_fixed(model, subj, clip, outx, outy, out_area2);
    if (debug_clip_fixed_vcount >= 3) { if (out_fallback) *out_fallback = 0; return res; }

    /* Try reversed order */
    int res2 = compute_intersection_centroid_ordered_fixed(model, clip, subj, outx, outy, out_area2);
    if (debug_clip_fixed_vcount >= 3) { if (out_fallback) *out_fallback = 0; return res2; }

    /* No drawable clipped polygon found in either order — perform bbox/sample fallback
     * (mirror the inspector behavior): find a sample point inside the integer
     * intersection bbox that lies inside both polygons; otherwise use bbox center.
     * Report fallback via out_fallback (nullable).
     */
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;

    int ix0=0, iy0=0, ix1=0, iy1=0;
    int found = 0;

    if (compute_bbox_intersection(model, subj, clip, &ix0, &iy0, &ix1, &iy1)) {
        int cx0 = (ix0 + ix1) / 2; int cy0 = (iy0 + iy1) / 2;
        if (point_in_poly_int(cx0, cy0, faces, vtx, subj, faces->vertex_count[subj]) && point_in_poly_int(cx0, cy0, faces, vtx, clip, faces->vertex_count[clip])) {
            if (outx) *outx = cx0; if (outy) *outy = cy0; found = 1;
        } else {
            int W = ix1 - ix0; int H = iy1 - iy0;
            for (int gx = 0; gx < 3 && !found; ++gx) for (int gy = 0; gy < 3 && !found; ++gy) {
                int tx = ix0 + (((2*gx + 1) * W + 3) / 6);
                int ty = iy0 + (((2*gy + 1) * H + 3) / 6);
                if (point_in_poly_int(tx, ty, faces, vtx, subj, faces->vertex_count[subj]) && point_in_poly_int(tx, ty, faces, vtx, clip, faces->vertex_count[clip])) { if (outx) *outx = tx; if (outy) *outy = ty; found = 1; break; }
            }
        }
    } else {
        /* Fallback to intersecting per-face bboxes if compute_bbox_intersection failed */
        ix0 = faces->minx[subj] > faces->minx[clip] ? faces->minx[subj] : faces->minx[clip];
        ix1 = faces->maxx[subj] < faces->maxx[clip] ? faces->maxx[subj] : faces->maxx[clip];
        iy0 = faces->miny[subj] > faces->miny[clip] ? faces->miny[subj] : faces->miny[clip];
        iy1 = faces->maxy[subj] < faces->maxy[clip] ? faces->maxy[subj] : faces->maxy[clip];
        int cx0 = (ix0 + ix1) / 2; int cy0 = (iy0 + iy1) / 2;
        if (point_in_poly_int(cx0, cy0, faces, vtx, subj, faces->vertex_count[subj]) && point_in_poly_int(cx0, cy0, faces, vtx, clip, faces->vertex_count[clip])) {
            if (outx) *outx = cx0; if (outy) *outy = cy0; found = 1;
        } else {
            int W = ix1 - ix0; int H = iy1 - iy0;
            for (int gx = 0; gx < 3 && !found; ++gx) for (int gy = 0; gy < 3 && !found; ++gy) {
                int tx = ix0 + (((2*gx + 1) * W + 3) / 6);
                int ty = iy0 + (((2*gy + 1) * H + 3) / 6);
                if (point_in_poly_int(tx, ty, faces, vtx, subj, faces->vertex_count[subj]) && point_in_poly_int(tx, ty, faces, vtx, clip, faces->vertex_count[clip])) { if (outx) *outx = tx; if (outy) *outy = ty; found = 1; break; }
            }
        }
    }

    if (!found) { /* final fallback: center of the intersection bbox */
        if (outx) *outx = (ix0 + ix1) / 2;
        if (outy) *outy = (iy0 + iy1) / 2;
    }

    /* Report fallback and clear clipped-polygon diagnostics */
    if (out_fallback) *out_fallback = 1;
    debug_clip_fixed_vcount = 0; debug_clip_raw_area2 = 0; debug_clip_fixed_area = 0.0;
    debug_clip_centroid_x = outx ? *outx : 0; debug_clip_centroid_y = outy ? *outy : 0;
    if (out_area2) *out_area2 = 0; /* no polygon area */
    return 1;
}

/* Alternative implementation using QuickDraw regions (faster to reason about for
 * overlapping faces and robust to colinear/edge-sharing cases). This does NOT
 * compute the exact area-centroid, it returns the center of the region bbox.
 * Returns 1 and sets (*outx,*outy) and *out_area2 (bbox area) on success, 0 on no-overlap.
 */
static int compute_intersection_region_bbox(Model3D* model, int subj, int clip, int* ix0, int* iy0, int* ix1, int* iy1, long long* out_area2) {
    if (!model || !ix0 || !iy0 || !ix1 || !iy1) return 0;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    if (subj < 0 || clip < 0 || subj >= faces->face_count || clip >= faces->face_count) return 0;

    int n1 = faces->vertex_count[subj];
    int n2 = faces->vertex_count[clip];
    if (n1 <= 0 || n2 <= 0) return 0;

    /* Vertex index offsets used later */
    int off1 = faces->vertex_indices_ptr[subj];
    int off2 = faces->vertex_indices_ptr[clip];

    /* Compute per-face bounding boxes (used to clamp the region bbox) */
    int minx1, maxx1, miny1, maxy1;
    {
        int vi = faces->vertex_indices_buffer[off1] - 1;
        minx1 = maxx1 = vtx->x2d[vi];
        miny1 = maxy1 = vtx->y2d[vi];
        for (int i = 1; i < n1; ++i) {
            vi = faces->vertex_indices_buffer[off1 + i] - 1;
            int vx = vtx->x2d[vi], vy = vtx->y2d[vi];
            if (vx < minx1) minx1 = vx; if (vx > maxx1) maxx1 = vx;
            if (vy < miny1) miny1 = vy; if (vy > maxy1) maxy1 = vy;
        }
    }
    int minx2, maxx2, miny2, maxy2;
    {
        int wi = faces->vertex_indices_buffer[off2] - 1;
        minx2 = maxx2 = vtx->x2d[wi];
        miny2 = maxy2 = vtx->y2d[wi];
        for (int i = 1; i < n2; ++i) {
            wi = faces->vertex_indices_buffer[off2 + i] - 1;
            int wx = vtx->x2d[wi], wy = vtx->y2d[wi];
            if (wx < minx2) minx2 = wx; if (wx > maxx2) maxx2 = wx;
            if (wy < miny2) miny2 = wy; if (wy > maxy2) maxy2 = wy;
        }
    }

    /* Logging removed: region debug logs were deleted per request */

    RegionHndl r1 = NewRgn();
    RegionHndl r2 = NewRgn();
    RegionHndl r3 = NewRgn();
    if (!r1 || !r2 || !r3) {
        if (r1) DisposeRgn(r1);
        if (r2) DisposeRgn(r2);
        if (r3) DisposeRgn(r3);
        return 0;
    }

    /* Build region for subj — construct polygon then FramePoly into r1 */
    {
        Handle subj_poly = OpenPoly();
        off1 = faces->vertex_indices_ptr[subj];
        int v0 = faces->vertex_indices_buffer[off1] - 1;
        MoveTo(vtx->x2d[v0], vtx->y2d[v0]);
        for (int i = 1; i < n1; ++i) {
            int vi = faces->vertex_indices_buffer[off1 + i] - 1;
            LineTo(vtx->x2d[vi], vtx->y2d[vi]);
        }
        ClosePoly();
        PenState ps_subj; GetPenState(&ps_subj); 
        OpenRgn(); FramePoly(subj_poly); CloseRgn(r1);
        SetPenState(&ps_subj);
        KillPoly(subj_poly);
    }

    /* Build region for clip — construct polygon then FramePoly into r2 */
    {
        Handle clip_poly = OpenPoly();
        off2 = faces->vertex_indices_ptr[clip];
        int w0 = faces->vertex_indices_buffer[off2] - 1;
        MoveTo(vtx->x2d[w0], vtx->y2d[w0]);
        for (int i = 1; i < n2; ++i) {
            int wi = faces->vertex_indices_buffer[off2 + i] - 1;
            LineTo(vtx->x2d[wi], vtx->y2d[wi]);
        }
        ClosePoly();
        PenState ps_clip; GetPenState(&ps_clip); 
        OpenRgn(); FramePoly(clip_poly); CloseRgn(r2);
        SetPenState(&ps_clip);
        KillPoly(clip_poly);
    }


    /* Intersection */
    SectRgn(r1, r2, r3);

    /* runs-based parsing removed — QuickDraw region header bbox (r3->rgnBBox) is authoritative */

    int has = 0;
    if (!EmptyRgn(r3)) {

        /* Read region header (size + bbox words) from r3 */
        HLock((Handle)r3);
        short* sp = (short*)(*r3);
        int w0 = (int)sp[0];
        int raw_top, raw_left, raw_bottom, raw_right;
        if (w0 >= 10 && (w0/2) >= 5) {
            raw_top = (int)sp[1]; raw_left = (int)sp[2]; raw_bottom = (int)sp[3]; raw_right = (int)sp[4];
        } else {
            /* Fallback: use first four words as before */
            raw_top = (int)sp[0]; raw_left = (int)sp[1]; raw_bottom = (int)sp[2]; raw_right = (int)sp[3];
        }
        /* Normalize X and Y orientations into (left,top,right,bottom) from header */
        int h_top = (raw_top < raw_bottom) ? raw_top : raw_bottom;
        int h_bottom = (raw_top < raw_bottom) ? raw_bottom : raw_top;
        int h_left = (raw_left < raw_right) ? raw_left : raw_right;
        int h_right = (raw_left < raw_right) ? raw_right : raw_left;

        /* runs parsing removed — use region header bbox (r3->rgnBBox) directly */
        /* Use the region header bbox (r3->rgnBBox) as authoritative for the QD region rectangle.
         * Keep runs parsing only for diagnostics (do not override the region header bbox). */
        *ix0 = h_left; *iy0 = h_top; *ix1 = h_right; *iy1 = h_bottom;

        if (*ix0 <= *ix1 && *iy0 <= *iy1) {
            if (out_area2) *out_area2 = (long long)((*ix1 - *ix0) * (*iy1 - *iy0));
            /* Paint region for visual confirmation (apply current pan so visual matches overlays) */
            PenState ps_dbg; GetPenState(&ps_dbg); SetSolidPenPat(COL_YELLOW);
            /* shift the QuickDraw region by the global pan so PaintRgn draws at the
               same position as the other overlays which apply `pan_dx/pan_dy` at
               render time */
            OffsetRgn(r3, pan_dx, pan_dy);
            PaintRgn(r3); SetPenState(&ps_dbg);
            HUnlock((Handle)r3);
            DisposeRgn(r1); DisposeRgn(r2); DisposeRgn(r3);
            return 1;
        }
        HUnlock((Handle)r3);
    }

    DisposeRgn(r1); DisposeRgn(r2); DisposeRgn(r3);
    return has;
}

static int compute_intersection_centroid_ordered_qd_fixed(Model3D* model, int subj, int clip, int* outx, int* outy, long long* out_area2) {
    int ix0,iy0,ix1,iy1; long long a2=0;
    if (!compute_intersection_region_bbox(model, subj, clip, &ix0, &iy0, &ix1, &iy1, &a2)) return 0;
    if (outx) *outx = (ix0 + ix1) / 2;
    if (outy) *outy = (iy0 + iy1) / 2;
    if (out_area2) *out_area2 = a2;
    return 1;
}



segment "transform";
// ============================================================================
//  3. TRANSFORM & PROJECTION
//  Observer-space transform, depth calculation, 2D projection
// ============================================================================

/**
 * CALCULATING MINIMUM FACE DEPTHS AND VISIBILITY FLAGS
 * =====================================================
 * 
 * This function calculates for each face:
 * 1. The minimum depth (z_min) of all its vertices in the observer coordinate system
 * 2. The display visibility flag based on vertex positions relative to camera
 * 
 * The z_min value is used for face sorting during rendering (painter's algorithm).

 * We use minimum (closest point) for correct occlusion in the painter's algorithm.
 * The display_flag is used to cull faces that have vertices behind the camera.
 * 
 * PARAMETERS:
 *   vertices   : Array of vertices with coordinates in observer system
 *   faces      : Array of faces to process  
 *   face_count : Number of faces
 * 
 * PER-FACE DEPTH, BBOX, & PLANE COMPUTATION (calculateFaceDepths)
 * ================================================================
 * Purpose:
 *   Compute per-face metrics required by the painters and diagnostics:
 *     - z_min, z_max and z_mean (observer-space depths) used for coarse sorting
 *     - screen-space axis-aligned bounding box (min/max X/Y) used for cheap overlap tests
 *     - planar coefficients (Newell method) A/B/C/D for robust face-plane tests
 *     - `display_flag` indicating basic visibility (behind-camera or culled by plane test)
 *
 * Algorithm summary (per face):
 *   1) Iterate vertices in observer-space (xo/yo/zo). If any vertex has zo <= 0, mark
 *      `display_flag = 0` (non-displayable) to avoid projection artefacts.
 *   2) Accumulate z_min/z_max and compute z_mean (used by initial z-sorting in the painter).
 *   3) Compute the face's axis-aligned bbox from integer screen coords (x2d/y2d) for quick
 *      overlap tests (used by Newell/Sancha tests 2 and 3).
 *   4) Compute planar coefficients (A,B,C,D) using Newell's method; these are used to
 *      perform finer pairwise tests and the optional **observer-space back-face culling**
 *      if `cull_back_faces` is enabled (test D <= 0 indicates a back-face relative to observer).
 *
 * Implementation notes:
 *   - Plane coefficients are kept both in Fixed32 (for fixed-mode painter) and cached/converted
 *     to float buffers for the FLOAT painter mode. Conversion flags track whether a float copy
 *     is available to avoid repeated conversions.
 *   - `display_flag == 1` means the face is eligible for drawing; `0` indicates it is either
 *     behind the camera or culled by the observer-space back-face test when enabled.
 *   - This function must be called AFTER vertex transforms into observer-space (e.g. by
 *     `processModelFast()` or `transformToObserver()`), because it reads xo/yo/zo/x2d/y2d arrays.
 *
 * Performance & debug:
 *   - The function optionally logs culling counts when not in PERFORMANCE_MODE.
 *   - The function is intentionally designed to be O(n) over faces and linear in face vertex counts.
 */

void calculateFaceDepths(Model3D* model, Face3D* faces, int face_count) {
    int i, j;
    int culled_count = 0; // diagnostic: number of faces culled by back-face test (observer-space)
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* face_arrays = &model->faces;

    for (i = 0; i < face_count; i++) {
        Fixed32 z_min = FLOAT_TO_FIXED(9999.0);  // Initialize to very large value (closest)
        Fixed32 z_max = FLOAT_TO_FIXED(-9999.0); // Initialize to very small value (farthest)
        int display_flag = 1;
        Fixed32 sum = 0;
        int n = face_arrays->vertex_count[i];
        int minx = 9999, maxx = -9999, miny = 9999, maxy = -9999;

        // Access indices from the packed buffer using the offset
        int offset = face_arrays->vertex_indices_ptr[i];

        // --- Newell accumulators (plane normal via ALL vertices, robust to concave faces) ---
        Fixed64 a64 = 0, b64 = 0, c64 = 0, d64 = 0;
        Fixed64 sumx64 = 0, sumy64 = 0, sumz64 = 0;
        Fixed32 x_prev = 0, y_prev = 0, z_prev = 0;
        Fixed32 x_first = 0, y_first = 0, z_first = 0;
        Fixed32 xc = 0, yc = 0, zc = 0;
        int valid_n = 0;
        int have_prev = 0;

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

                // centroid accumulation (Fixed64 to avoid overflow with many vertices)
                sumx64 += (Fixed64)xo;
                sumy64 += (Fixed64)yo;
                sumz64 += (Fixed64)zo;

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

                    a64 += (Fixed64)dy * sz;
                    b64 += (Fixed64)dz * sx;
                    c64 += (Fixed64)dx * sy;

                    x_prev = xo; y_prev = yo; z_prev = zo;
                }
                valid_n++;
            }
        }

        if (!display_flag || n < 3 || valid_n < 3) {
            // face is behind camera or degenerate: zero coefficients
            face_arrays->plane_a[i] = 0;
            face_arrays->plane_b[i] = 0;
            face_arrays->plane_c[i] = 0;
            face_arrays->plane_d[i] = 0;
            if (shaded_by_orientation) {
                face_shade_color[i] = COL_FILL_DEFAULT;
            }
        } else {
            // close the Newell cycle: last valid vertex -> first valid vertex
            Fixed32 dy = FIXED_SUB(y_prev, y_first);
            Fixed32 dz = FIXED_SUB(z_prev, z_first);
            Fixed32 dx = FIXED_SUB(x_prev, x_first);
            Fixed32 sy = y_prev + y_first;
            Fixed32 sz = z_prev + z_first;
            Fixed32 sx = x_prev + x_first;

            a64 += (Fixed64)dy * sz;
            b64 += (Fixed64)dz * sx;
            c64 += (Fixed64)dx * sy;

            a64 >>= FIXED_SHIFT;
            b64 >>= FIXED_SHIFT;
            c64 >>= FIXED_SHIFT;

            xc = (Fixed32)(sumx64 / valid_n);
            yc = (Fixed32)(sumy64 / valid_n);
            zc = (Fixed32)(sumz64 / valid_n);

            d64 = -(((Fixed64)a64 * xc) + ((Fixed64)b64 * yc) + ((Fixed64)c64 * zc));
            d64 >>= FIXED_SHIFT;

            face_arrays->plane_a[i] = a64;
            face_arrays->plane_b[i] = b64;
            face_arrays->plane_c[i] = c64;
            face_arrays->plane_d[i] = d64;

            // Optional back-face culling in observer-space: if the plane D term is <= 0,
            // the plane faces away from the observer (origin), so cull the face when enabled.
            if (cull_back_faces && display_flag) {
                if (d64 <= 0) {
                    display_flag = 0;
                    ++culled_count;
                }
            }
        }

        face_arrays->z_min[i] = z_min;  // Store minimum depth for this face (closest)
        face_arrays->z_max[i] = z_max;  // Store maximum depth for this face (farthest)
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


// Simple helper: compute 2D projected coordinates from observer-space coords and a projection scale
// Minimal version taking only Model3D* and angle_w (unused here)
// - model: model containing vertices and x2d/y2d output arrays
// - angle_w: ignored because observer-space rotation is already applied in processModelFast()
// This function projects xo/yo/zo to screen coordinates and keeps face bboxes in sync.
void compute2DFromObserver(Model3D* model, int angle_w) {
    (void)angle_w; // angle_w is ignored here; projection uses already-rotated observer coords
    VertexArrays3D* vtx = &model->vertices;
    int vcount = vtx->vertex_count;
    Fixed32* xo = vtx->xo; Fixed32* yo = vtx->yo; Fixed32* zo = vtx->zo;
    int* x2d_out = vtx->x2d; int* y2d_out = vtx->y2d;
    Fixed32 scale = s_global_proj_scale_fixed;
    // Update model's stored projection scale to reflect the scale used
    model->auto_proj_scale = scale;

    const Fixed32 centre_x_f = INT_TO_FIXED(CENTRE_X);
    const Fixed32 centre_y_f = INT_TO_FIXED(CENTRE_Y);
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

    /* Keep per-face 2D bounding boxes in sync with the newly computed x2d/y2d.
     * This avoids stale bbox-based quick-rejects after interactive zoom/pan. */
    updateFace2DBounds(model);
}

// Helper: recompute per-face 2D bounding boxes from the current `x2d/y2d` arrays
// This is much cheaper than `calculateFaceDepths()` and is sufficient when only
// the screen projection (scale/rotation/pan) changes.
static void updateFace2DBounds(Model3D* model) {
    if (!model) return;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int face_count = faces->face_count;

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

// Normalize auto-fit parameters so that observer distance becomes 150
// by scaling model vertices and projection scale accordingly.
//
// For a model where auto-fit produced:
//   D = auto_suggested_distance
//   P = auto_suggested_proj_scale
// the normalization applies:
//   factor = D / 150
//   new D = D * factor
//   new P = P * factor
//   vertices *= factor
//
// This keeps the apparent size on-screen constant while forcing the
// observer distance to be 150.
void normalizeAutoFitDistanceTo150(Model3D* model) {
    if (model == NULL) return;
    Fixed32 D = model->auto_suggested_distance;
    Fixed32 P = model->auto_suggested_proj_scale;
    if (D <= 0 || P <= 0) return;

    const Fixed32 targetD = INT_TO_FIXED(150);
    // Apply the pure “distance forced to 150 with unchanged on-screen size” formula:
    //   factor = sqrt(150 / D)
    // Here sqrt is required because screen projection is proportional to (P * M) / D,
    // so scaling both M and P by k changes screen size by k^2.
    float factor_f = sqrtf(FIXED_TO_FLOAT(targetD) / FIXED_TO_FLOAT(D));
    if (factor_f <= 0.0f) return;
    Fixed32 factor = FLOAT_TO_FIXED(factor_f);

    // Scale model vertices
    VertexArrays3D* vtx = &model->vertices;
    for (int i = 0; i < vtx->vertex_count; ++i) {
        vtx->x[i] = FIXED_MUL_64(vtx->x[i], factor);
        vtx->y[i] = FIXED_MUL_64(vtx->y[i], factor);
        vtx->z[i] = FIXED_MUL_64(vtx->z[i], factor);
    }

    // Update metadata to match the applied scaling
    model->auto_suggested_distance = targetD;
    model->auto_suggested_proj_scale = FIXED_MUL_64(P, factor);
    model->auto_proj_scale = model->auto_suggested_proj_scale;
    s_global_proj_scale_fixed = model->auto_proj_scale;
    // Keep epsilon scaling (used in geometry tests) consistent with the forced distance.
    current_observer_distance = targetD;

    printf("Auto-fit normalized: D=%.4f P=%.2f factor=%.4f\n",
        FIXED_TO_FLOAT(model->auto_suggested_distance),
        FIXED_TO_FLOAT(model->auto_suggested_proj_scale),
        FIXED_TO_FLOAT(factor));
}

/**
 * OBSERVER PARAMETER INPUT
 * ========================
 * 
 * This function presents a text interface to allow the
 * user to specify 3D visualization parameters.
 * 
 * REQUESTED PARAMETERS:
 * - Horizontal angle: rotation around Y-axis (left/right view)
 * - Vertical angle: rotation around X-axis (up/down view)
 * - Distance: observer distance (zoom)
 * - Screen rotation angle: final rotation in 2D plane
 * 
 * ERROR HANDLING:
 * - Default values if input failure
 * - Automatic string->Fixed32 conversion with atof() then FLOAT_TO_FIXED
 */
void getObserverParams(ObserverParams* params, Model3D* model) {
    char userinput[50];  // Buffer for user input
    
    // Display section header
    printf("\nObserver parameters:\n");
    printf("============================\n");
    printf("(Press ENTER to use default values - apply suggested auto-fit if available)\n");
    
    // Input horizontal angle (rotation around Y)
    printf("Horizontal angle (degrees, default %d): ", params->angle_h);
    if (fgets(userinput, sizeof(userinput), stdin) != NULL) {
        // Remove newline
        userinput[strcspn(userinput, "\n")] = 0;
        if (strlen(userinput) != 0) {
            params->angle_h = atoi(userinput);  // Parse integer degrees from input
        }
    }
    
    // Input vertical angle (rotation around X)  
    printf("Vertical angle (degrees, default %d): ", params->angle_v);
    if (fgets(userinput, sizeof(userinput), stdin) != NULL) {
        // Remove newline
        userinput[strcspn(userinput, "\n")] = 0;
        if (strlen(userinput) != 0) {
            params->angle_v = atoi(userinput);  // Parse integer degrees from input
        }
    }
    

    // Input screen rotation angle (final 2D rotation)
    printf("Screen rotation angle (degrees, default %d): ", params->angle_w);
    if (fgets(userinput, sizeof(userinput), stdin) != NULL) {
        // Remove newline
        userinput[strcspn(userinput, "\n")] = 0;
        if (strlen(userinput) != 0) {
            params->angle_w = atoi(userinput);  // Parse integer degrees from input
        }
    }

    // Input observation distance (zoom/perspective).
    // Press ENTER = auto-scale + center using sphere-based fit (default target),
    // or enter a numeric value to use that distance directly (no scaling).
    printf("Distance (ENTER = auto-scale, or enter a value): ");

    if (fgets(userinput, sizeof(userinput), stdin) != NULL) {
        // Remove newline
        userinput[strcspn(userinput, "\n")] = 0;
        if (strlen(userinput) == 0) {
            // User pressed ENTER: if we have auto-fit suggestions from load, apply them; otherwise use default distance
            if (model != NULL && model->auto_fit_ready) {
                params->distance = model->auto_suggested_distance;
                // Apply suggested projection scale to model so subsequent processing uses it
                model->auto_proj_scale = model->auto_suggested_proj_scale;
                model->auto_fit_applied = 1;
                printf("Auto-fit applied (distance=%.4f proj_scale=%.2f). Use +/- to adjust.\n", FIXED_TO_FLOAT(model->auto_suggested_distance), FIXED_TO_FLOAT(model->auto_suggested_proj_scale));
            } else if (model != NULL) {
                // No precomputed suggestion: fallback to sensible default
                params->distance = FLOAT_TO_FIXED(30.0);
            } else {
                params->distance = FLOAT_TO_FIXED(30.0);
            }
        } else {
            // User provided a distance value -> use it directly (no auto-scale)
            params->distance = FLOAT_TO_FIXED(atof(userinput)); // String->Fixed32 conversion
        }
    } else {
        // fgets failed; default behavior: if we have a model, apply precomputed auto-fit suggestions, else fallback distance
        if (model != NULL && model->auto_fit_ready) {
            params->distance = model->auto_suggested_distance;
            model->auto_proj_scale = model->auto_suggested_proj_scale;
            model->auto_fit_applied = 1;
            printf("Auto-fit applied (distance=%.4f proj_scale=%.2f). Use +/- to adjust.\n", FIXED_TO_FLOAT(model->auto_suggested_distance), FIXED_TO_FLOAT(model->auto_suggested_proj_scale));
        } else if (model != NULL) {
            params->distance = FLOAT_TO_FIXED(30.0);
        } else {
            params->distance = FLOAT_TO_FIXED(30.0);        // Default distance: balanced view (Fixed Point)
        }
    }

    // Store the current observer distance for use in geometric epsilon scaling 
    // and other distance-dependent calculations.
    current_observer_distance = params->distance;
}




segment "rendering";
// ============================================================================
//  4. RENDERING (QuickDraw + Scanline Z-Buffer)
// ============================================================================

// Function to draw polygons with QuickDraw
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
    
    // Precompute screen scale and screen bounds for culling
    int screenScale = mode / 320;
    int screenW = screenScale * (CENTRE_X * 2);
    int screenH = screenScale * (CENTRE_Y * 2);

    // Use global persistent handle to avoid repeated NewHandle/DisposeHandle
    // Each call allocates fresh if needed, but reuses same handle block
    if (globalPolyHandle == NULL) {
        int max_polySize = 2 + 8 + (MAX_FACE_VERTICES * 4);  // Max for MAX_FACE_VERTICES vertices
        globalPolyHandle = NewHandle((long)max_polySize, userid(), 0xC014, 0L);
        if (globalPolyHandle == NULL) {
            printf("Error: Unable to allocate global polygon handle\n");
            return;
        }
    }
    
    polyHandle = globalPolyHandle;
    
    // Make sure handle is unlocked before locking
    if (poly_handle_locked) { 
        HUnlock(polyHandle); 
        poly_handle_locked = 0; 
    }
    HLock(polyHandle); 
    poly_handle_locked = 1;

    SetPenMode(0);
    // printf("\nDrawing polygons on screen:\n");
    // printf("\nDrawing polygons on screen:\n");

    // Set fill pen once per frame (reduces state changes)
    SetSolidPenPat(COL_FILL_DEFAULT);

    // Use sorted_face_indices to draw in correct depth order
    // Draw ALL faces - painter's algorithm handles occlusion
    int start_face = 0;
    int max_faces_to_draw = face_count;
    for (i = start_face; i < start_face + max_faces_to_draw; i++) {
        int face_id = faces->sorted_face_indices[i];
        if (faces->display_flag[face_id] == 0) continue;
        if (faces->vertex_count[face_id] >= 3) {
            int offset = faces->vertex_indices_ptr[face_id];
            int vcount_face = faces->vertex_count[face_id];
            int *indices_base = &faces->vertex_indices_buffer[offset];

            // Quick validity pass: ensure all indices are valid to avoid undefined points
            int all_valid = 1;
            for (j = 0; j < vcount_face; ++j) {
                int vi = indices_base[j] - 1;
                if (vi < 0 || vi >= vtx->vertex_count) { all_valid = 0; break; }
            }
            if (!all_valid) { invalid_faces_skipped++; continue; }

            // Calculate polySize for this specific face
            int polySize = 2 + 8 + (vcount_face * 4);
            poly = (DynamicPolygon *)*polyHandle;
            poly->polySize = polySize;

            // Cache arrays
            int *x2d = vtx->x2d;
            int *y2d = vtx->y2d;

            // Initialize min/max with the first vertex to avoid sentinel checks
            int first_vi = indices_base[0] - 1;
            int x = x2d[first_vi];
            int y = y2d[first_vi];
            poly->polyPoints[0].h = screenScale * (x + pan_dx);
            poly->polyPoints[0].v = y + pan_dy;
            min_x = max_x = x;
            min_y = max_y = y;

            // Fill remaining points using cached data
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

            // Bounding-box culling (convert to screen pixels, apply pan)
            int sc_min_x = screenScale * (min_x + pan_dx);
            int sc_max_x = screenScale * (max_x + pan_dx);
            int sc_min_y = (min_y + pan_dy);
            int sc_max_y = (max_y + pan_dy);
            if (sc_max_x < 0 || sc_min_x >= screenW || sc_max_y < 0 || sc_min_y >= screenH) {
                // Off-screen; skip drawing
            } else {
                if (framePolyOnly) {
                    // Frame-only rendering (no fill)
                    int frame_color;
                    if (user_frame_color == 17) {
                        // Same as fill color
                        if (user_fill_color == 16 && random_fill_colors != NULL && face_id < random_colors_capacity) {
                            frame_color = random_fill_colors[face_id];
                        } else if (user_fill_color >= 0) {
                            frame_color = user_fill_color;
                        } else {
                            frame_color = COL_FILL_DEFAULT; // default fill color
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
                    SetSolidPenPat(COL_FILL_DEFAULT); // keep fill pen as default for next faces
                } else {
                    // Fill color
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
                    
                    // Frame color
                    int frame_color;
                    if (user_frame_color == 17) {
                        // Same as fill color
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
                    SetSolidPenPat(COL_FILL_DEFAULT); // restore fill pen
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

// drawPolygons_jitter: same behavior as drawPolygons but applies a small random
// pixel offset (0..jitter_max) to each vertex 2D coordinate prior to drawing.
// This function mirrors `drawPolygons()` but applies per-vertex jitter at render time.
void drawPolygons_jitter(Model3D* model, int* vertex_count, int face_count, int vertex_count_total) {
    if (!model) return;
    VertexArrays3D* vtx = &model->vertices;

    int vcount = vertex_count_total;
    if (vcount <= 0) { drawPolygons(model, vertex_count, face_count, vertex_count_total); return; }

    // Backup original coordinates
    int *saved_x = (int*)malloc(vcount * sizeof(int));
    int *saved_y = (int*)malloc(vcount * sizeof(int));
    if (!saved_x || !saved_y) {
        if (saved_x) free(saved_x);
        if (saved_y) free(saved_y);
        // Fallback to normal draw
        drawPolygons(model, vertex_count, face_count, vertex_count_total);
        return;
    }

    for (int i = 0; i < vcount; ++i) {
        saved_x[i] = vtx->x2d[i];
        saved_y[i] = vtx->y2d[i];
        int dx = rand() % (jitter_max + 1); // 0..jitter_max
        int dy = rand() % (jitter_max + 1);
        vtx->x2d[i] = saved_x[i] + dx;
        vtx->y2d[i] = saved_y[i] + dy;
    }

    // Draw using modified coordinates
    drawPolygons(model, vertex_count, face_count, vertex_count_total);

    // Restore original coordinates
    for (int i = 0; i < vcount; ++i) {
        vtx->x2d[i] = saved_x[i];
        vtx->y2d[i] = saved_y[i];
    }

    free(saved_x);
    free(saved_y);
}// Diagnostic: Frame in white all polygons listed in `inconclusive_pairs`.

// Draw a single face (by face index) using the same QuickDraw path as drawPolygons
// - respects face validity and display_flag
// - uses globalPolyHandle and poly_handle_locked like drawPolygons

void drawFace(Model3D* model, int face_id, int fillPenPat, int show_index) {
    if (!model) return;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    if (face_id < 0 || face_id >= faces->face_count) return;
    if (faces->display_flag[face_id] == 0) return;
    int vcount_face = faces->vertex_count[face_id];
    if (vcount_face < 3) return;

    int offset = faces->vertex_indices_ptr[face_id];
    int *indices_base = &faces->vertex_indices_buffer[offset];

    // Quick validity pass
    for (int j = 0; j < vcount_face; ++j) {
        int vi = indices_base[j] - 1;
        if (vi < 0 || vi >= vtx->vertex_count) return; // invalid face -> nothing to draw
    }

    // Ensure global handle exists
    if (globalPolyHandle == NULL) {
        int max_polySize = 2 + 8 + (4 * 4);  // Max for quad (4 vertices)
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

    int *x2d = vtx->x2d;
    int *y2d = vtx->y2d;

    // Initialize first point
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
        // Off-screen; nothing to draw
    } else {
        if (fillPenPat >= 0) {
            SetSolidPenPat(fillPenPat);
            Pattern pat;
            GetPenPat(pat);
            FillPoly(polyHandle, pat);
            SetSolidPenPat(COL_FRAME);
            FramePoly(polyHandle);
            SetSolidPenPat(COL_FILL_DEFAULT);
        } else if (framePolyOnly) {
            /* Draw explicit solid wireframe using MoveTo/LineTo to avoid dotted
             * outlines produced by FramePoly in certain pen/pattern states. */
            SetPenMode(0); // PenMode = Copy ==> avoid dotted lines
            SetSolidPenPat(COL_FRAME);
            FramePoly(polyHandle);
            // int first_h = poly->polyPoints[0].h;
            // int first_v = poly->polyPoints[0].v;
            // MoveTo(first_h, first_v);
            // for (int kk = 1; kk < vcount_face; ++kk) {
            //     LineTo(poly->polyPoints[kk].h, poly->polyPoints[kk].v);
            // }
            // LineTo(first_h, first_v);
            SetSolidPenPat(COL_FILL_DEFAULT);
        } else {
            Pattern pat;
            GetPenPat(pat);
            FillPoly(polyHandle, pat);
            SetSolidPenPat(COL_FRAME);
            FramePoly(polyHandle);
            SetSolidPenPat(COL_FILL_DEFAULT);
        }

        // Draw face index centered in the polygon using QuickDraw MoveTo + DrawString
        if (show_index) {
            int center_x = (min_x + max_x) / 2;
            int center_y = (min_y + max_y) / 2;
            // Use same transformation as polygon points: horizontal scaled, vertical and pan applied
            int screenCx = screenScale * (center_x + pan_dx);
            int screenCy = center_y + pan_dy;

            char tmp[32];
            int len = snprintf(tmp, sizeof(tmp), "%d", face_id);
            if (len > 15) len = 15; // fit into Pascal string buffer
            unsigned char pstr[16];
            pstr[0] = (unsigned char)len;
            memcpy(&pstr[1], tmp, len);

            MoveTo(screenCx, screenCy);
            DrawString(pstr);
        }
    }
}

// Draw only the face index label for a given face without rendering the polygon geometry.
// This is used by the face inspector to overlay face numbers once all faces are visible.
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

    Pattern savedPat;
    GetPenPat(savedPat);
    SetSolidPenPat(COL_FRAME);
    MoveTo(screenCx, screenCy);
    DrawString(pstr);
    SetPenPat(savedPat);
}

// This helper iterates the recorded ambiguous pairs and draws a white outline around both
// polygons in each pair so developers can visually inspect cases where the painter could
// not conclusively determine a stable order. It is a pure rendering aid (no state changes
// beyond drawing) and is invoked only in diagnostic/debug modes or on explicit user request.
void frameInconclusivePairs(Model3D* model) {
    if (!model) return;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    if (inconclusive_pairs == NULL || inconclusive_pairs_count == 0) return;

    // Ensure global handle exists (allocate based on maximum supported vertices)
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

    // Iterate the inconclusive pairs and frame both faces (face1 and face2)
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

            // Quick validity pass
            int all_valid = 1;
            for (int j = 0; j < vcount_face; ++j) {
                int vi = indices_base[j] - 1;
                if (vi < 0 || vi >= vtx->vertex_count) { all_valid = 0; break; }
            }
            if (!all_valid) continue;

            // Build polygon using the same conventions as drawPolygons (screenScale applied to X)
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

            // Bounding-box culling (convert to screen pixels)
            int screenW = screenScale * (CENTRE_X * 2);
            int screenH = screenScale * (CENTRE_Y * 2);
            int sc_min_x = screenScale * min_x;
            int sc_max_x = screenScale * max_x;
            int sc_min_y = screenScale * min_y;
            int sc_max_y = screenScale * max_y;
            if (sc_max_x < 0 || sc_min_x >= screenW || sc_max_y < 0 || sc_min_y >= screenH) continue;

            // Frame polygon in white (use same pen as drawPolygons framing)
            SetSolidPenPat(COL_LIGHT_GREEN);
            Pattern pat;
            GetPenPat(pat);
            FillPoly(polyHandle, pat);
            SetSolidPenPat(COL_YELLOW);
            FramePoly(polyHandle);
        }
    }

    // Keep handle locked (consistent with drawing code)
}

// Generate random colors for all faces
void generate_random_colors(int face_count) {
    if (face_count <= 0) return;
    
    // Ensure capacity
    if (random_colors_capacity < face_count) {
        random_fill_colors = (unsigned char*)realloc(random_fill_colors, face_count);
        random_frame_colors = (unsigned char*)realloc(random_frame_colors, face_count);
        random_colors_capacity = face_count;
    }
    
    // Generate random colors (1-15, skip 0 which is black background)
    for (int i = 0; i < face_count; i++) {
        random_fill_colors[i] = (rand() % 15) + 1;
        random_frame_colors[i] = (rand() % 15) + 1;
    }
}


/* display_model_face_ids
 * ----------------------
 * Draw the model in wireframe and overlay each face id centered on its polygon.
 * Implementation details:
 *  - Sets `framePolyOnly = 1` to render the wireframe for clarity,
 *  - Ensures all faces are visible, draws polygon frames, then calls `drawFace`
 *    with `show_index == 1` for each face to render the numeric ID in the center.
 */
void display_model_face_ids(Model3D* model, ObserverParams* params, const char* filename) {
    if (!model || !params) return;
    FaceArrays3D* faces = &model->faces;
    if (faces->face_count <= 0) { printf("No faces in model\n"); return; }

    startgraph(mode);

    // Backup display flags
    unsigned char* backup_flags = (unsigned char*)malloc(faces->face_count);
    for (int i = 0; i < faces->face_count; ++i) backup_flags[i] = faces->display_flag[i];

    int old_frame = framePolyOnly;
    framePolyOnly = 1; // wireframe for clear labels

    if (jitter) drawPolygons_jitter(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count); else drawPolygons(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count);

    int screenScale = mode / 320;
    // Draw each face in the current sorted order (use existing faces->sorted_face_indices)
    for (int si = 0; si < faces->face_count; ++si) {
        int f = faces->sorted_face_indices[si];
        if (!faces->display_flag[f]) continue;
        // Draw face filled with green (fillPenPat = 10)
        drawFace(model, f, COL_LIGHT_GREEN, 0);

        // Compute integer bounding box center (use same logic as drawFace)
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

            // Draw index in green
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

    // Restore
    framePolyOnly = old_frame;
    for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = backup_flags[i];
    free(backup_flags);
}

// --- Scanline Z-Buffer ---
// Alternative renderer, triggered by a dedicated key (e.g. Z/z) in the main
// viewer loop. Entirely additive: does not touch processModelFast,
// calculateFaceDepths, or the existing painter's-algorithm pipeline. Consumes
// the same already-computed vtx->x2d/y2d/zo and faces->vertex_indices_buffer.
//
// PIXEL PLOT: was done by MoveTo(x,y)+LineTo(x,y) (QuickDraw II, same point =
// single pixel), replaced with inline 65816
// assembly later (direct SHR nibble read-modify-write), once this scanline
// logic is validated. Marked below with "PLOT PIXEL HERE".
//
// DEPTH PRECISION: this Z-buffer is kept entirely in native float (NOT
// Fixed32) on purpose. Diagnosed with a real case (two perpendicular faces
// of the model, ids 13 and 18, whose observer-space depths interleave within
// less than 1% of each other in their screen overlap region) - a Fixed32
// round-trip for inv_z was not resolving such close depth conflicts
// correctly, causing consistently wrong occlusion between those faces
// regardless of rotation. Float32 has far more usable relative precision at
// this magnitude, so all inv_z math below stays in float from vertex
// computation through to the buffer comparison itself.
//
// Adapt SCREEN_WIDTH/SCREEN_HEIGHT and MAX_SPAN_INTERSECTIONS to your
// actual constants/limits.

typedef struct {
    int x;        // screen-space x of the intersection
    float inv_z;  // interpolated 1/z at that intersection (native float)
} ScanIntersection;

// Extracted verbatim from drawPolygons's fill/frame color logic, so the
// scanline Z-buffer renderer reproduces the exact same user choices
// (default colors, user overrides, orientation shading, palette cycling).

int getFaceFillColor(int face_id) {
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
    return fill_color;
}

int getFaceFrameColor(int face_id, int fill_color) {
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
    return frame_color;
}

void drawPixel(int x, int y, int color)
{
    int offset;
    unsigned char value;

    color &= 0x0F;

    /* Offset (0..31999) into SHR bank $E1 memory - computed in C,
       passed to assembly via the X register (16-bit, plenty of range) */
    offset = y * 160 + (x >> 1);

    asm {
        sep     #0x20        ; 8-bit accumulator

        ldx     offset       ; X = byte offset (index register stays 16-bit)
        lda     0xE12000,x    ; read the byte containing both pixels
                             ; (absolute LONG indexed addressing - bank $E1
                             ; is baked into the instruction itself, no
                             ; pointer variable involved at all)

        pha                  ; save original byte

        lda     x
        and     #0x01
        bne     pixel_odd

        /* x EVEN: pixel = high nibble */
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

pixel_odd:
        /* x ODD: pixel = low nibble */
        pla
        and     #0xF0
        sta     value

        lda     color
        and     #0x0F
        ora     value
        sta     0xE12000,x

done:
        rep     #0x20
    }
}

void renderModelScanlineZBuffer_old (Model3D* model) {
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;
    int fcount = faces->face_count;
    int y, f, i;

    // --- 1/z per vertex, recomputed locally once per call (not stored
    // elsewhere; existing pipeline untouched). Perspective-correct depth
    // interpolation needs 1/z, which is linear in screen space - z itself
    // is not. Kept in float (see header comment on precision). ---
    static float* inv_z = NULL;
    static int inv_z_capacity = 0;
    if (inv_z_capacity < vcount) {
        if (inv_z) free(inv_z);
        inv_z = (float*)malloc(vcount * sizeof(float));
        inv_z_capacity = vcount;
    }
    for (i = 0; i < vcount; i++) {
        float zo_f = FIXED_TO_FLOAT(vtx->zo[i]);
        inv_z[i] = (zo_f > 0.0f) ? (1.0f / zo_f) : 0.0f;
    }

    // --- One-scanline Z-buffer, reused every line (320 floats only,
    // not a full-screen buffer) ---
    static float zbuffer_line[SCREEN_WIDTH];

    ScanIntersection hits[MAX_SPAN_INTERSECTIONS];

    SetPenMode(0);
    applyPalette(palette);

    for (y = 0; y < SCREEN_HEIGHT; y++) {
        int screenY = y + pan_dy;
        if (screenY < 0 || screenY >= SCREEN_HEIGHT) continue;

        for (i = 0; i < SCREEN_WIDTH; i++) zbuffer_line[i] = -1.0f; // nothing drawn yet

        for (f = 0; f < fcount; f++) {
            int n, offt, k, hit_count;

            if (!faces->display_flag[f]) continue;
            n = faces->vertex_count[f];
            if (n < 3) continue;
            if (y < faces->miny[f] || y > faces->maxy[f]) continue;

            offt = faces->vertex_indices_ptr[f];
            hit_count = 0;

            // --- Gather all edge/scanline intersections for this face ---
            // Standard scan-conversion rule (y1 <= y < y2, taking edge
            // direction into account) avoids double-counting a vertex that
            // sits exactly on the scanline - this also makes concave faces
            // (e.g. the star) work correctly via pair-wise (even-odd) fill,
            // same principle as the Newell/shoelace robustness discussed
            // earlier for orientation.
            for (k = 0; k < n; k++) {
                int k2 = (k + 1 < n) ? (k + 1) : 0; // avoid modulo (no native op on 65816)
                int vid1 = faces->vertex_indices_buffer[offt + k] - 1;
                int vid2 = faces->vertex_indices_buffer[offt + k2] - 1;

                int y1 = vtx->y2d[vid1];
                int y2 = vtx->y2d[vid2];
                int x1 = vtx->x2d[vid1];
                int x2 = vtx->x2d[vid2];

                int ylo = (y1 < y2) ? y1 : y2;
                int yhi = (y1 < y2) ? y2 : y1;
                if (y < ylo || y >= yhi) continue; // half-open range, skips horizontal edges too

                // linear interpolation fraction along the edge in screen space
                float t = (float)(y - y1) / (float)(y2 - y1);
                int xi = x1 + (int)((x2 - x1) * t + 0.5f);

                // 1/z is linear in screen space along this edge (perspective-
                // correct interpolation) - plain float lerp, no Fixed32
                // round-trip (see header comment).
                float izf = inv_z[vid1] + (inv_z[vid2] - inv_z[vid1]) * t;

                if (hit_count < MAX_SPAN_INTERSECTIONS) {
                    hits[hit_count].x = xi;
                    hits[hit_count].inv_z = izf;
                    hit_count++;
                }
            }

            if (hit_count < 2) continue;

            // --- Sort intersections by x (simple insertion sort - hit_count
            // is small, typically <= number of face vertices) ---
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

            // --- Pair up consecutive intersections (even-odd rule) and
            // fill each span with a Z-test per pixel ---
            {
                int p;
                for (p = 0; p + 1 < hit_count; p += 2) {
                    int xa = hits[p].x;
                    int xb = hits[p + 1].x;
                    float iza = hits[p].inv_z;
                    float izb = hits[p + 1].inv_z;
                    int x;

                    if (xa == xb) continue; // degenerate span

                    // Color depends only on the face (f), never on x/y -
                    // compute once per span instead of once per pixel.
                    {
                        int fillColor = getFaceFillColor(f);
                        int frameColor = getFaceFrameColor(f, fillColor);

                        // iz_here varies linearly across the span - compute
                        // the per-pixel step ONCE (one division), then just
                        // add it each pixel instead of recomputing a full
                        // division + multiplication every pixel. This is
                        // the hottest loop in the function (executed once
                        // per pixel drawn, vs once per span/edge for the
                        // other optimizations), so this is where the real
                        // cost was.
                        float dIz = (izb - iza) / (float)(xb - xa);
                        float iz_here = iza;

                        for (x = xa; x <= xb; x++) {
                            int screenX = x + pan_dx;
                            if (screenX >= 0 && screenX < SCREEN_WIDTH) {
                                if (iz_here > zbuffer_line[x]) {
                                    zbuffer_line[x] = iz_here;

                                    // --- PLOT PIXEL HERE ---
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

void renderModelScanlineZBuffer_fast(Model3D* model) {
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;
    int fcount = faces->face_count;
    int y, f, i;

    // --- 1/z per vertex (inchangé) ---
    static float* inv_z = NULL;
    static int inv_z_capacity = 0;
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

    // Bounds de clipping en "model space" (constantes pour toute la frame)
    int clip_x_min = -pan_dx;
    int clip_x_max = SCREEN_WIDTH - 1 - pan_dx;

    for (y = 0; y < SCREEN_HEIGHT; y++) {
        int screenY = y + pan_dy;
        if (screenY < 0 || screenY >= SCREEN_HEIGHT) continue;

        // Reset Z-buffer (parcours pointeur, plus facile à optimiser pour le compilateur)
        float* zb_clear = zbuffer_line;
        float* zb_clear_end = zbuffer_line + SCREEN_WIDTH;
        while (zb_clear < zb_clear_end) *zb_clear++ = -1.0f;

        for (f = 0; f < fcount; f++) {
            int n, offt, k, hit_count;

            if (!faces->display_flag[f]) continue;
            n = faces->vertex_count[f];
            if (n < 3) continue;
            if (y < faces->miny[f] || y > faces->maxy[f]) continue;

            // OPTIM 1 : couleurs une seule fois par face
            int fillColor = getFaceFillColor(f);
            int frameColor = getFaceFrameColor(f, fillColor);

            offt = faces->vertex_indices_ptr[f];
            hit_count = 0;

            // --- Intersections edge/scanline ---
            for (k = 0; k < n; k++) {
                int k2 = (k + 1 < n) ? (k + 1) : 0;
                int vid1 = faces->vertex_indices_buffer[offt + k] - 1;
                int vid2 = faces->vertex_indices_buffer[offt + k2] - 1;

                int y1 = vtx->y2d[vid1];
                int y2 = vtx->y2d[vid2];
                int dy = y2 - y1;

                if (dy == 0) continue;                    // edge horizontal
                if (dy > 0) {
                    if (y < y1 || y >= y2) continue;      // half-open [y1, y2)
                } else {
                    if (y < y2 || y >= y1) continue;      // half-open [y2, y1)
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

            // OPTIM 2 : tri spécialisé (cas triangle = 2 hits, 90% du temps)
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

            // --- Remplissage des spans ---
            int p;
            for (p = 0; p + 1 < hit_count; p += 2) {
                int xa = hits[p].x;
                int xb = hits[p + 1].x;
                float iza = hits[p].inv_z;
                float izb = hits[p + 1].inv_z;

                if (xa > xb) continue;

                // OPTIM 3 : clipping X avant la boucle, pas dedans
                if (xb < clip_x_min || xa > clip_x_max) continue;

                int x0 = xa;
                int x1 = xb;
                float iz0 = iza;

                // --- FIX : cas dégénéré (span d'1 pixel) traité à part,
                // AVANT toute division, pour éviter une division par
                // zéro sur (xb - xa) == 0. Le clipping peut aussi
                // réduire un span normal à 1 pixel après ajustement
                // (voir plus bas), donc ce cas est revérifié après clip. ---
                if (x0 == x1) {
                    int sx0 = x0 + pan_dx;
                    if (iz0 > zbuffer_line[x0]) {
                        zbuffer_line[x0] = iz0;
                        drawPixel(sx0, screenY, frameColor);
                    }
                    continue;
                }

                // dIz calculé seulement si xb != xa (span >= 2 pixels avant clip)
                float dIz = (izb - iza) / (float)(xb - xa);

                if (x0 < clip_x_min) {
                    iz0 += dIz * (clip_x_min - x0);  // pas de division, juste un fmul
                    x0 = clip_x_min;
                }
                if (x1 > clip_x_max) x1 = clip_x_max;
                if (x0 > x1) continue;

                int sx0 = x0 + pan_dx;
                int sx1 = x1 + pan_dx;

                // OPTIM 4 : pas de branche contour/fill dans la boucle interne
                if (x0 == x1) {
                    // Span réduit à 1 pixel par le clipping (pas par les
                    // coordonnées d'origine) - dIz est valide ici puisqu'il
                    // a été calculé avant clip, donc pas de division par 0.
                    if (iz0 > zbuffer_line[x0]) {
                        zbuffer_line[x0] = iz0;
                        drawPixel(sx0, screenY, frameColor);
                    }
                } else {
                    // Premier pixel = contour
                    if (iz0 > zbuffer_line[x0]) {
                        zbuffer_line[x0] = iz0;
                        drawPixel(sx0, screenY, frameColor);
                    }

                    // Milieu = fill (boucle la plus chaude : 0 branche, accès pointeur)
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

                    // Dernier pixel = contour (iz accumulé = profondeur exacte au clip)
                    if (iz > *zb) {
                        *zb = iz;
                        drawPixel(sx1, screenY, frameColor);
                    }
                }
            }
        }
    }
}

/* Render a 3D model using a scanline Z-buffer algorithm with a small bias to reduce Z-fighting.
Slower than the non-biased version (renderModelScanlineZBuffer_fast) due to the additional bias calculations.
Called only when back-face culling is off.
*/
void renderModelScanlineZBuffer_biased(Model3D* model) {
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    int vcount = vtx->vertex_count;
    int fcount = faces->face_count;
    int y, f, i;
    static const float Z_FIGHT_BIAS = 0.001f;  // or 0.0001f;

    // --- 1/z per vertex (inchangé) ---
    static float* inv_z = NULL;
    static int inv_z_capacity = 0;
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

    // Bounds de clipping en "model space" (constantes pour toute la frame)
    int clip_x_min = -pan_dx;
    int clip_x_max = SCREEN_WIDTH - 1 - pan_dx;

    for (y = 0; y < SCREEN_HEIGHT; y++) {
        int screenY = y + pan_dy;
        if (screenY < 0 || screenY >= SCREEN_HEIGHT) continue;

        // Reset Z-buffer (parcours pointeur, plus facile à optimiser pour le compilateur)
        float* zb_clear = zbuffer_line;
        float* zb_clear_end = zbuffer_line + SCREEN_WIDTH;
        while (zb_clear < zb_clear_end) *zb_clear++ = -1.0f;

        for (f = 0; f < fcount; f++) {
            int n, offt, k, hit_count;

            if (!faces->display_flag[f]) continue;
            n = faces->vertex_count[f];
            if (n < 3) continue;
            if (y < faces->miny[f] || y > faces->maxy[f]) continue;

            // OPTIM 1 : couleurs une seule fois par face
            int fillColor = getFaceFillColor(f);
            int frameColor = getFaceFrameColor(f, fillColor);

            // Biais anti Z-fighting - cette fonction n'est appelee que quand
            // cull_back_faces est desactive (voir le dispatcher
            // renderModelScanlineZBuffer), donc pas besoin de re-tester
            // cull_back_faces ici : toujours pertinent d'appliquer le biais
            // sur les faces front.
            float depthBias = (faces->plane_d[f] > 0) ? Z_FIGHT_BIAS : 0.0f;

            offt = faces->vertex_indices_ptr[f];
            hit_count = 0;

            // --- Intersections edge/scanline ---
            for (k = 0; k < n; k++) {
                int k2 = (k + 1 < n) ? (k + 1) : 0;
                int vid1 = faces->vertex_indices_buffer[offt + k] - 1;
                int vid2 = faces->vertex_indices_buffer[offt + k2] - 1;

                int y1 = vtx->y2d[vid1];
                int y2 = vtx->y2d[vid2];
                int dy = y2 - y1;

                if (dy == 0) continue;                    // edge horizontal
                if (dy > 0) {
                    if (y < y1 || y >= y2) continue;      // half-open [y1, y2)
                } else {
                    if (y < y2 || y >= y1) continue;      // half-open [y2, y1)
                }

                int x1 = vtx->x2d[vid1];
                int x2 = vtx->x2d[vid2];

                float t = (float)(y - y1) / (float)dy;
                int xi = x1 + (int)((x2 - x1) * t + 0.5f);
                float izf = inv_z[vid1] + (inv_z[vid2] - inv_z[vid1]) * t;
                izf += depthBias; // biais applique une fois par intersection

                if (hit_count < MAX_SPAN_INTERSECTIONS) {
                    hits[hit_count].x = xi;
                    hits[hit_count].inv_z = izf;
                    hit_count++;
                }
            }

            if (hit_count < 2) continue;

            // OPTIM 2 : tri spécialisé (cas triangle = 2 hits, 90% du temps)
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

            // --- Remplissage des spans ---
            int p;
            for (p = 0; p + 1 < hit_count; p += 2) {
                int xa = hits[p].x;
                int xb = hits[p + 1].x;
                float iza = hits[p].inv_z;
                float izb = hits[p + 1].inv_z;

                if (xa > xb) continue;

                // OPTIM 3 : clipping X avant la boucle, pas dedans
                if (xb < clip_x_min || xa > clip_x_max) continue;

                int x0 = xa;
                int x1 = xb;
                float iz0 = iza;

                // --- FIX : cas degenere (span d'1 pixel) traite a part,
                // AVANT toute division, pour eviter une division par zero
                // sur (xb - xa) == 0. ---
                if (x0 == x1) {
                    int sx0 = x0 + pan_dx;
                    if (iz0 > zbuffer_line[x0]) {
                        zbuffer_line[x0] = iz0;
                        drawPixel(sx0, screenY, frameColor);
                    }
                    continue;
                }

                // dIz calcule seulement si xb != xa (span >= 2 pixels avant clip)
                float dIz = (izb - iza) / (float)(xb - xa);

                if (x0 < clip_x_min) {
                    iz0 += dIz * (clip_x_min - x0);  // pas de division, juste un fmul
                    x0 = clip_x_min;
                }
                if (x1 > clip_x_max) x1 = clip_x_max;
                if (x0 > x1) continue;

                int sx0 = x0 + pan_dx;
                int sx1 = x1 + pan_dx;

                // OPTIM 4 : pas de branche contour/fill dans la boucle interne
                if (x0 == x1) {
                    // Span reduit a 1 pixel par le clipping (pas par les
                    // coordonnees d'origine) - dIz est valide ici puisqu'il
                    // a ete calcule avant clip, donc pas de division par 0.
                    if (iz0 > zbuffer_line[x0]) {
                        zbuffer_line[x0] = iz0;
                        drawPixel(sx0, screenY, frameColor);
                    }
                } else {
                    // Premier pixel = contour
                    if (iz0 > zbuffer_line[x0]) {
                        zbuffer_line[x0] = iz0;
                        drawPixel(sx0, screenY, frameColor);
                    }

                    // Milieu = fill (boucle la plus chaude : 0 branche, acces pointeur)
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

                    // Dernier pixel = contour (iz accumule = profondeur exacte au clip)
                    if (iz > *zb) {
                        *zb = iz;
                        drawPixel(sx1, screenY, frameColor);
                    }
                }
            }
        }
    }
}

// --- Dispatcher --------------------------------------------------------
// Choisit la version adaptee selon cull_back_faces :
//  - actif  -> pas de faces back rendues, donc pas de collision front/back
//              coplanaire possible -> version rapide, sans aucun surcout
//              lie au biais anti Z-fighting.
//  - inactif -> les deux faces d'une paire coplanaire peuvent etre visibles
//              -> version avec biais, seule capable de forcer le front a
//              gagner le Z-test de facon fiable.
void renderModelScanlineZBuffer(Model3D* model) {
    if (cull_back_faces) {
        renderModelScanlineZBuffer_fast(model);
    } else {
        renderModelScanlineZBuffer_biased(model);
    }
}


segment "color";
// ============================================================================
//  5. PALETTE & COLOR MANAGEMENT
// ============================================================================

void SetColor(int palette_num, int color_index, int r, int g, int b)
{
    unsigned long offset;
    unsigned int  color_word;
    
    /* Chaque palette = 16 couleurs × 2 octets = 32 octets */
    offset = (long)palette_num * 32L + (long)color_index * 2L;
    
    /* Encode R, G, B into nibbles (0-15 each) */
    color_word = ((unsigned int)r << 8) | 
                 ((unsigned int)g << 4) | 
                 (unsigned int)b;
    
    /* Write into video RAM (bank $E1) */
    *((unsigned int *)(0xE19E00L + offset)) = color_word;
}

GS_Palette ReadPalette(int palette_num)
{
    GS_Palette  pal;
    int         i;
    unsigned int *src;

    /* Chaque palette fait 16 × 2 octets = 32 octets */
    src = (unsigned int *)(PALETTE_BASE + (long)palette_num * 32L);

    for (i = 0; i < 16; i++)
        pal.color[i] = src[i];

    return pal;
}

void applyPalette(int palette_num)
{
    if (palette_num < 0 || palette_num >= 16) palette_num = 0;

    Word color_table[16];
    for (int color_index = 0; color_index < 16; ++color_index) {
        color_table[color_index] = (Word)palettes[palette_num].color[color_index];
    }
    SetColorTable(0, color_table);
}

void initPalettes(void)
{
    palette0_loaded = 0;

    for (int p = 1; p < 16; ++p) {
        for (int i = 0; i < 16; ++i) {
            palettes[p].color[i] = paletteGradientColor(p, i);
        }
    }
}

void DoColor() {
        Rect r;
        unsigned char pstr[4];  // Pascal string: [length][characters...]]

        SetRect (&r, 0, 1, mode / 320 *10, 11);
        for (int i = 0; i < 16; i++) {
            SetSolidPenPat(i);
            PaintRect(&r);

            if (i == 0) {
                SetSolidPenPat(COL_WHITE); // White frame for black background
                FrameRect(&r);
            }

            MoveTo(r.h1, r.v2+10);
            // Create a Pascal string to display the number
            if (i < 10) {
                pstr[0] = 1;           // Length: 1 character
                pstr[1] = '0' + i;     // Digit 0-9
            } else {
                pstr[0] = 2;           // Length: 2 characters
                pstr[1] = '0' + (i / 10);      // Tens (1 for 10-15)
                pstr[2] = '0' + (i % 10);      // Units (0-5 for 10-15)
            }
            DrawString(pstr);
            OffsetRect(&r, 20, 0);
        }
}

void screen2Black() {
    asm 
    {
        lda #0x0000
        ldx #0x7CFE
    loop:
        sta 0xE12000,x
        dex
        dex
        bne loop
    }
}

static void computeOrientationShading(Model3D* model) {
    if (!model) return;
    FaceArrays3D* faces = &model->faces;
    int face_count = faces->face_count;

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



segment "inspection";
// ============================================================================
//  6. INSPECTION TOOLS & DEBUGGING
// ============================================================================

void debug_two_faces(Model3D* model, int f1, int f2) {
    startgraph(mode);
    drawFace(model, f1, COL_LIGHT_GREEN, 1);
    keypress();
    drawFace(model, f2, COL_LIGHT_GREEN, 1);
    keypress();
    endgraph();
    DoText();
}

/* inspect_faces_before
 * --------------------
 * Purpose:
 *  - Interactively inspect faces that are placed BEFORE a selected face in the
 *    painter's ordered list but that the pairwise tests conclude should be AFTER.
 * Behavior:
 *  - Prompts the user for a face id, locates it in `sorted_face_indices` and checks
 *    faces earlier in the ordered list using `pair_order_relation` and `evaluate_pair_tests`.
 *  - Prints a compact per-face diagnostic and offers a wireframe preview with the
 *    selected face highlighted (green) and misplaced faces highlighted (orange).
 */
void inspect_faces_before(Model3D* model, ObserverParams* params, const char* filename) {
    if (!model || !params) return;
    FaceArrays3D* faces = &model->faces;
    int face_count = faces->face_count;
    if (face_count <= 0) { printf("No faces in model\n"); return; }

    /* Title / welcome (text-mode) */
    printf("Inspector: inspect faces placed BEFORE a selected face (diagnostics)\n\n");

    // Ensure back-face culling on and recompute depths & ordering
    // helper: enable culling and run painter sort; returns previous culling state
    int old_cull = cull_back_faces;
    cull_back_faces = 1;

    // Prompt user for face id
    printf("Enter face id (0..%d) to inspect: ", face_count - 1);
    int sel = -1;
    if (scanf("%d", &sel) != 1) {
        int ch; while ((ch = getchar()) != '\n' && ch != EOF) ;
        printf("Input cancelled\n");
        cull_back_faces = old_cull; return;
    }
    // consume remaining chars on the line (newline) to avoid disturbing later fgets()
    {
        int ch; while ((ch = getchar()) != '\n' && ch != EOF) ;
    }
    // Use the exact number provided by the user (do not convert 1-based to 0-based)
    int target_face = sel;
    if (target_face < 0 || target_face >= face_count) { printf("Invalid face id\n"); cull_back_faces = old_cull; return; }

    // Find position of target_face in the sorted array
    int pos = -1;
    for (int i = 0; i < face_count; ++i) {
        if (faces->sorted_face_indices[i] == target_face) { pos = i; break; }
    }
    if (pos < 0) { printf("Selected face is not in sorted list\n"); cull_back_faces = old_cull; return; }

    // Inform user about target position
    printf("Selected face %d is at position %d in the ordered list (total faces = %d)\n", target_face, pos, face_count);

    // Ensure variables are visible throughout the function (used later for preview)
    int* misplaced = NULL;
    int misplaced_count = 0;

    // Check faces placed before the selected face (only consider visible faces)
    misplaced = (int*)malloc(face_count * sizeof(int));
    misplaced_count = 0;
    int checked = 0, rel_neg = 0, rel_pos = 0, rel_zero = 0, bbox_skipped = 0;
    for (int i = 0; i < pos; ++i) {
        int f = faces->sorted_face_indices[i];
        // Bounding-box quick rejection: if separated on X or Y, skip - no overlap
        if (faces->maxx[f] <= faces->minx[target_face] || faces->maxx[target_face] <= faces->minx[f]
            || faces->maxy[f] <= faces->miny[target_face] || faces->maxy[target_face] <= faces->miny[f]) {
            bbox_skipped++; continue;
        }
        checked++;
        int rel = pair_order_relation(model, f, target_face);
        if (rel == -1) rel_neg++; else if (rel == 1) rel_pos++; else rel_zero++;
        if (rel == 1) { // this face should be after target -> misplaced
            misplaced[misplaced_count++] = f;
        }
    }

    printf("%d \"before\" faces (total): checked=%d ; bbox_skipped=%d ; should_be_before(-1): %d ;  should_be_after(+1): %d (misplaced: %d) ; inconclusive: %d\n",
           pos, checked, bbox_skipped, rel_neg, rel_pos, misplaced_count, rel_zero);

    printf("%d misplaced faces relative to face %d : ", misplaced_count, target_face);
    for (int i = 0; i < misplaced_count; ++i) {
        int f = misplaced[i];
        // Find position of the misplaced face in the sorted list (1-based for readability)
        int pos_in_sorted = -1;
        for (int si = 0; si < faces->face_count; ++si) {
            if (faces->sorted_face_indices[si] == f) { pos_in_sorted = si; break; }
        }
        if (pos_in_sorted >= 0) printf("%d (pos=%d)", f, pos_in_sorted + 1);
        else printf("%d (pos=?)", f);
        if (i < misplaced_count - 1) printf(",");
    }
    /* Report overlapping subset among misplaced faces (global summary) */
    int overlap_count = 0;
    int *overlaps = NULL;
    if (misplaced_count > 0) {
        overlaps = (int*)malloc(sizeof(int) * misplaced_count);
        for (int i = 0; i < misplaced_count; ++i) {
            int f = misplaced[i];
            if (projected_polygons_overlap(model, f, target_face)) overlaps[overlap_count++] = f;
        }
    }
    printf("\n");
    if (overlap_count > 0) {
        printf("Overlap with target: %d face(s):", overlap_count);
        for (int i = 0; i < overlap_count; ++i) printf(" %d", overlaps[i]);
        printf("\n");
    } else {
        printf("Overlap with target: none\n");
    }
    // Note: overlaps will be freed at the end of the function

    printf("\n");
    keypress();

    // For each misplaced face, evaluate tests 1..7 and display which succeeded/failed/inconclusive
    for (int mi = 0; mi < misplaced_count; ++mi) {
        int f = misplaced[mi];
        int tests[7];
        evaluate_pair_tests(model, f, target_face, tests);

        // Build readable lists
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

        // Compact grouped output (single-line)
        if (before_count == 0) strncpy(before_list, "(none)", sizeof(before_list));
        if (after_count == 0) strncpy(after_list, "(none)", sizeof(after_list));
        if (incon_count == 0) strncpy(incon_list, "(none)", sizeof(incon_list));

        printf("Face %d: BEFORE tests: %s; AFTER tests: %s; inconclusive: %s\n", f, before_list, after_list, incon_list);
        
        // Conclusion lines (keep existing style)
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

    // Backup display flags
    unsigned char* backup_flags = (unsigned char*)malloc(faces->face_count);
    for (int i = 0; i < faces->face_count; ++i) backup_flags[i] = faces->display_flag[i];

    // 1) Show entire model in wireframe
    int old_frame = framePolyOnly;
    framePolyOnly = 1; // wireframe
    // Wireframe preview: handled via the `framePolyOnly` flag and `drawPolygons()` (no separate `processModelWireframe()` function)
    if (jitter) drawPolygons_jitter(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count); else drawPolygons(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count);


    // 2) Overlay misplaced faces in orange (pen 6)
    for (int i = 0; i < misplaced_count; ++i) {
        int f = misplaced[i];
        faces->display_flag[f] = 1;
        drawFace(model, f, COL_ORANGE, 0);
    }

    // 3) Overlay: selected face in green (pen 10)
    // Ensure target face is visible for drawFace
    faces->display_flag[target_face] = 1;
    drawFace(model, target_face, COL_LIGHT_GREEN, 1);

    MoveTo(2, 195);
    printf("%d face(s) should be after face %d, %d overlap%s\n", misplaced_count, target_face, overlap_count, overlap_count > 1 ? "s" : "");
    keypress();
    endgraph();
    DoText();

    // Restore state
    framePolyOnly = old_frame;
    for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = backup_flags[i];
    free(backup_flags);

    // If there are misplaced faces, offer to fix by moving target before the smallest index
    if (misplaced_count > 0) {
        printf("\nPress 'A' to move face %d considering ALL misplaced faces, 'O' for overlaps only, or any other key to skip: ", target_face);
        int key = getkeypress ();
        
        if (key == 'A' || key == 'a' || key == 'O' || key == 'o') {
            int min_pos = face_count; // start with max possible
            int target_to_move_before = -1;
            int use_overlaps = (key == 'O' || key == 'o');
            
            // Determine which list to use and check if available
            if (use_overlaps && overlap_count == 0) {
                printf("\nNo overlaps to process.\n");
            } else {
                // Find the smallest position among selected faces
                int count_to_check = use_overlaps ? overlap_count : misplaced_count;
                int *faces_to_check = use_overlaps ? overlaps : misplaced;
                
                for (int i = 0; i < count_to_check; ++i) {
                    int f = faces_to_check[i];
                    // Find position in sorted list
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
                    
                    // Move target_face to min_pos (before the misplaced face with smallest index)
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

    if (overlaps) free(overlaps);
    free(misplaced);
    cull_back_faces = old_cull;
}

/* inspect_faces_after
 * -------------------
 * Purpose:
 *  - Interactively inspect faces that are placed AFTER a selected face in the
 *    painter's ordered list but that the pairwise tests conclude should be BEFORE.
 * Behavior:
 *  - Prompts the user for a face id, locates it in `sorted_face_indices` and checks
 *    visible faces later in the ordered list using `pair_order_relation` and `evaluate_pair_tests`.
 *  - Prints a compact per-face diagnostic and offers a wireframe preview with the
 *    selected face highlighted (green) and misplaced faces highlighted (pink).
 */
 void inspect_faces_after(Model3D* model, ObserverParams* params, const char* filename) {
    if (!model || !params) return;
    FaceArrays3D* faces = &model->faces;
    int face_count = faces->face_count;
    if (face_count <= 0) { printf("No faces in model\n"); return; }

    int old_cull = cull_back_faces;
    cull_back_faces = 1;

    /* Title / welcome (text-mode) */
    printf("Inspector: inspect faces placed AFTER a selected face (diagnostics)\n\n");

    printf("Enter face id (0..%d) to inspect AFTER-list: ", face_count - 1);
    int sel = -1;

    if (scanf("%d", &sel) != 1) {
        int ch; while ((ch = getchar()) != '\n' && ch != EOF) ;
        printf("Input cancelled\n");
        cull_back_faces = old_cull; return;
    }
    // consume remaining chars on the line (newline) to avoid disturbing later fgets()
    {
        int ch; while ((ch = getchar()) != '\n' && ch != EOF) ;
    }
    int target_face = sel;
    if (target_face < 0 || target_face >= face_count) { printf("Invalid face id\n"); cull_back_faces = old_cull; return; }

    int pos = -1;
    for (int i = 0; i < face_count; ++i) if (faces->sorted_face_indices[i] == target_face) { pos = i; break; }
    if (pos < 0) { printf("Selected face is not in sorted list\n"); cull_back_faces = old_cull; return; }

    int* misplaced = (int*)malloc(face_count * sizeof(int)); int misplaced_count = 0;
    int checked = 0, rel_neg = 0, rel_pos = 0, rel_zero = 0, bbox_skipped = 0;
    for (int i = pos + 1; i < face_count; ++i) {
        int f = faces->sorted_face_indices[i];
        // Bounding-box quick rejection: if separated on X or Y, skip - no overlap
        if (faces->maxx[f] <= faces->minx[target_face] || faces->maxx[target_face] <= faces->minx[f]
            || faces->maxy[f] <= faces->miny[target_face] || faces->maxy[target_face] <= faces->miny[f]) {
            bbox_skipped++; continue;
        }
        int rel = pair_order_relation(model, f, target_face); // correct parameter order
        checked++;
        if (rel == -1) rel_neg++; else if (rel == 1) rel_pos++; else rel_zero++;
        if (rel == -1) { // f should be before target -> misplaced
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

    /* Report overlapping subset among the misplaced list (global summary) */
    int overlap_count = 0;
    int *overlaps = NULL;
    if (misplaced_count > 0) {
        overlaps = (int*)malloc(sizeof(int) * misplaced_count);
        for (int i = 0; i < misplaced_count; ++i) {
            int f = misplaced[i];
            if (projected_polygons_overlap(model, f, target_face)) overlaps[overlap_count++] = f;
        }
    }
    if (overlap_count > 0) {
        printf("Overlap with target: %d face(s):", overlap_count);
        for (int i = 0; i < overlap_count; ++i) printf(" %d", overlaps[i]);
        printf("\n");
    } else {
        printf("Overlap with target: none\n");
    }
    // Note: overlaps will be freed at the end of the function
    printf("\n");
    keypress();

    for (int ai = 0; ai < misplaced_count; ++ai) {
        int f = misplaced[ai];
        // Quick bbox rejection: if separated on X or Y, skip detailed tests
        if (faces->maxx[f] <= faces->minx[target_face] || faces->maxx[target_face] <= faces->minx[f]
            || faces->maxy[f] <= faces->miny[target_face] || faces->maxy[target_face] <= faces->miny[f]) {
            printf("Face %d: skipped (axis-aligned bbox separation)\n", f);
            continue;
        }

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

    // Offer a preview (always, as in inspect_faces_before)
    printf("\nPress a key to preview model with highlight selected and misplaced faces (if any).\n");
    keypress();
    startgraph(mode);

    unsigned char* backup_flags = (unsigned char*)malloc(faces->face_count);
    for (int i = 0; i < faces->face_count; ++i) backup_flags[i] = faces->display_flag[i];

    int old_frame = framePolyOnly;
    framePolyOnly = 1; // wireframe
    // Wireframe preview: handled via the `framePolyOnly` flag and `drawPolygons()` (no separate `processModelWireframe()` function)
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

    // If there are misplaced faces, offer to fix by moving target after the largest index
    if (misplaced_count > 0) {
        printf("\nPress 'A' to move face %d considering ALL misplaced faces, 'O' for overlaps only, or any other key to skip: ", target_face);
        int key = getkeypress ();
        
        if (key == 'A' || key == 'a' || key == 'O' || key == 'o') {
            int max_pos = -1;
            int target_to_move_after = -1;
            int use_overlaps = (key == 'O' || key == 'o');
            
            // Determine which list to use and check if available
            if (use_overlaps && overlap_count == 0) {
                printf("\nNo overlaps to process.\n");
            } else {
                // Find the largest position among selected faces
                int count_to_check = use_overlaps ? overlap_count : misplaced_count;
                int *faces_to_check = use_overlaps ? overlaps : misplaced;
                
                for (int i = 0; i < count_to_check; ++i) {
                    int f = faces_to_check[i];
                    // Find position in sorted list
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
                    
                    // Move target_face to max_pos (after the misplaced face with largest index)
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

    if (overlaps) free(overlaps);
    free(misplaced);
    cull_back_faces = old_cull;
}


/* showFace
 * --------
 * Purpose:
 *  - Display a single face in filled mode (green) over the wireframe model.
 * Behavior:
 *  - Prompts user for a face id (0..face_count-1).
 *  - Draws the entire model in wireframe mode.
 *  - Overlays the selected face in filled green (pen 10).
 *  - Navigate: Left/Right arrows = prev/next face ID
 *              Up/Down arrows = next/prev position in sorted list
 *  - ESC to exit
 */
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
            drawFace(model, target_face, COL_GREY, 0);

            // revert: keep the face hidden, saved_vertex_count untouched
            faces->vertex_count[target_face] = 0;
            faces->display_flag[target_face] = saved_df;
        }

        // --- Draw the face normal (toggle: N/n) ---
        // Centroid and plane coefficients (A,B,C) are already in observer
        // space (computed from vtx->xo/yo/zo in calculateFaceDepths), so we
        // only need the observer->screen projection step from
        // processModelFast (rotation is already applied), not the full
        // rotation matrix.
        if (show_normal) {
            int offt2 = faces->vertex_indices_ptr[target_face];
            int vn2 = faces->vertex_count[target_face];
            // if face is hidden, use saved_vertex_count
            if ((vn2 == 0) && (faces->saved_vertex_count[target_face] > 0)) {vn2 = faces->saved_vertex_count[target_face]; }

            // NOTE: fixed - a bare "continue" here would skip endgraph()/getkeypress()
            // for the rest of this loop iteration (main while(!quit) loop, not a
            // sub-loop), leaving the graphics context unbalanced and the loop
            // spinning without ever waiting for a key. Wrap the rest of the
            // normal-drawing logic in "if (vn2 >= 3)" instead.
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
                    // visual length of the normal arrow, in observer-space units
                    // -- tune to your model's typical scale
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

                    // Draw face nomal
                     {
                        SetSolidPenPat(COL_YELLOW);
                        // SetSolidPenPat(15); // White
                        MoveTo(c_x2d + pan_dx, c_y2d + pan_dy);
                        LineTo(e_x2d + pan_dx, e_y2d + pan_dy);
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
        printf("\nArrows: navigate,  N: normal, SPACE: options");
        
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
            int effective_vn = (vn == 0 && faces->saved_vertex_count[target_face] > 0)
                    ? faces->saved_vertex_count[target_face] : vn;
            printf("\n=== Face detail (ID=%d) ===\n\n", target_face);
            // printf("Orientation: %s [%s]\n", (faces->plane_d[target_face] > 0) ? "FRONT" : "BACK",
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
            }  
            else if (tkey == 'V' || tkey == 'v') {
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
                calculateFaceDepths(model, NULL, faces->face_count);
                backup_flags[target_face] = faces->display_flag[target_face];
                printf("Face %d restored.\n\n", target_face);
                printf("Press any key to return to graphics...\n"); fflush(stdout);
                int tmpk = getkeypress ();
                continue;
            } else if (tkey == 'A' || tkey == 'a') {
                // Restore every hidden face, then a single global recompute
                restoreAllFaces(model);
                calculateFaceDepths(model, NULL, faces->face_count);
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

static void reverseFaceVertexOrder(Model3D* model, int face_idx) {
    if (!model || face_idx < 0) return;
    FaceArrays3D* faces = &model->faces;
    if (face_idx >= faces->face_count) return;

    int n = faces->vertex_count[face_idx];
    if (n < 3) return;
    int off = faces->vertex_indices_ptr[face_idx];
    for (int i = 0; i < n / 2; ++i) {
        int tmp = faces->vertex_indices_buffer[off + i];
        faces->vertex_indices_buffer[off + i] = faces->vertex_indices_buffer[off + n - 1 - i];
        faces->vertex_indices_buffer[off + n - 1 - i] = tmp;
    }
    
    // set new plane coefs
    faces->plane_a[face_idx] = -faces->plane_a[face_idx];
    faces->plane_b[face_idx] = -faces->plane_b[face_idx];
    faces->plane_c[face_idx] = -faces->plane_c[face_idx];
    faces->plane_d[face_idx] = -faces->plane_d[face_idx];
    
    // set display_flag accordingly
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
    
    // XXXX useless since new calculateFaceDepths using all vertices to calculte orientation.
        // calculateFaceDepths(model, NULL, faces->face_count);
        // painter_newell_sancha_fast(model, faces->face_count); ==> was useless anyway
}

void hideFace(Model3D* model, int face_idx) {
    FaceArrays3D* faces = &model->faces;
    if (faces->vertex_count[face_idx] > 0) {
        faces->saved_vertex_count[face_idx] = faces->vertex_count[face_idx];
    }
    faces->display_flag[face_idx] = 0;
    faces->vertex_count[face_idx] = 0;
}

void restoreFace(Model3D* model, int face_idx) {
    FaceArrays3D* faces = &model->faces;
    // double condition : la face doit être ACTUELLEMENT masquée (vertex_count==0)
    // ET avoir une sauvegarde valide — empêche d'écraser une face jamais masquée
    // si saved_vertex_count contenait une valeur garbage
    if (faces->vertex_count[face_idx] == 0 && faces->saved_vertex_count[face_idx] > 0) {
        faces->vertex_count[face_idx] = faces->saved_vertex_count[face_idx];
        faces->saved_vertex_count[face_idx] = 0;
    }
}

void restoreAllFaces(Model3D* model) {
    FaceArrays3D* faces = &model->faces;
    int i;
    for (i = 0; i < faces->face_count; ++i) {
        if (faces->vertex_count[i] == 0 && faces->saved_vertex_count[i] > 0) {
            faces->vertex_count[i] = faces->saved_vertex_count[i];
            faces->saved_vertex_count[i] = 0;
        }
    }
}

/* Helper: enforce mandatory after-relations.
 * This mechanism preserves the invariant simply: after a move discovered by
 * `check_sort_repair`, it records the relation 'a after b'.
 * If a later move changes the global order, it revalidates all recorded
 * relations and adjusts the list to explicitly restore `a` after `b`.
 *
 * By design, this code is conservative: it applies relations one by one and
 * restarts the pass after each correction to avoid skipping a relation that
 * would become invalid after a move.
 */
static void enforce_mandatory_after_relations(Model3D* model, int n, int *pos_of_face, int relation_count, const int (*relations)[2]) {
    if (!model || !pos_of_face || !relations) return;
    FaceArrays3D* faces = &model->faces;

    for (int i = 0; i < relation_count; ++i) {
        int a = relations[i][0];
        int b = relations[i][1];

        // Skip invalid or out-of-bounds relations
        if (a < 0 || a >= n || b < 0 || b >= n) continue;

        int pa = pos_of_face[a];
        int pb = pos_of_face[b];

        // If a is not after b, move a immediately after b
        if (pa <= pb) {
            int tmp = faces->sorted_face_indices[pa];
            memmove(&faces->sorted_face_indices[pa], &faces->sorted_face_indices[pa + 1], sizeof(int) * (pb - pa));
            faces->sorted_face_indices[pb] = tmp;

            // Repair the position table in O(delta)
            for (int k = pa; k <= pb; ++k) {
                pos_of_face[faces->sorted_face_indices[k]] = k;
            }

            // Restart from zero to ensure the next relation is evaluated on the
            // updated order (simple convergence guarantee).
            i = -1;
        }
    }
}

/* pair_order_relation
 * -------------------
 * Purpose:
 *  - Apply the same per-pair tests used by the painter (tests 1..7) to decide
 *    whether two faces are conclusively ordered.
 * Behavior / Return values:
 *  - returns -1 if f1 is conclusively BEFORE f2
 *  - returns  1 if f1 is conclusively AFTER f2
 *  - returns  0 if inconclusive
 * Notes:
 *  - Uses depth (z_min/z_max), axis-aligned bbox separation (X/Y) and plane-based
 *    vertex-side tests (observer-side checks) mirroring painter logic.
 */
static int pair_order_relation(Model3D* model, int f1, int f2) {
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    Fixed32 epsilon = FLOAT_TO_FIXED(0.01f);

    // Test 1: depth separation
    if (faces->z_max[f2] <= faces->z_min[f1]) return -1; // f1 before f2
    if (faces->z_max[f1] <= faces->z_min[f2]) return 1;  // f1 after f2

    // Test 2/3: bbox separation (symmetric)
    int minx1 = faces->minx[f1], maxx1 = faces->maxx[f1], miny1 = faces->miny[f1], maxy1 = faces->maxy[f1];
    int minx2 = faces->minx[f2], maxx2 = faces->maxx[f2], miny2 = faces->miny[f2], maxy2 = faces->maxy[f2];
    if (maxx1 <= minx2) return -1; // f1 left of f2
    if (maxx2 <= minx1) return 1;  // f1 right of f2
    if (maxy1 <= miny2) return -1; // f1 below f2
    if (maxy2 <= miny1) return 1;  // f1 above f2

    // plane-only geometry tests factored out into helper
    int geo = geometric_face_relation(model, f1, f2);
    if (geo != 0) return geo;
    /* swapped orientation covers the symmetric tests (5 & 7 in the old
       numbering).  Invert the result because the arguments are reversed. */
    geo = geometric_face_relation(model, f2, f1);
    if (geo != 0) return -geo;

    // Non-conclusive
    return 0;
}


/* check_sort_repair
 * -----------------
 * Minimal deterministic repair of face ordering. For each pair that
 * `check_sort` would report as a mismatch, move the offending face by
 * a single minimal step so the ordering matches `ray_cast`:
 *  - If ray_cast == -1 (f1 closer than f2), ensure f1 is immediately AFTER f2.
 *  - If ray_cast ==  1 (f1 farther  than f2), ensure f1 is immediately BEFORE f2.
 * Returns the number of repairs performed.
 */
int check_sort_repair(Model3D* model, int face_count) {
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    if (face_count <= 0) return 0;

    int n = face_count;
    int *pos_of_face = (int*)malloc(sizeof(int) * n);
    if (!pos_of_face) return 0;
    for (int i = 0; i < n; ++i) pos_of_face[faces->sorted_face_indices[i]] = i;

    /* Mandatory relations recorded during the repair pass.
     * Each entry is [a,b] meaning: a must come after b in
     * faces->sorted_face_indices (pos[a] > pos[b]).
     *
     * This implements the requested mechanism: if A was moved after B, the
     * constraint is preserved and is continuously checked when other moves may
     * reorder B.
     */
    int relation_capacity = n + 16;
    int relation_count = 0;
    int (*after_relations)[2] = (int(*)[2])malloc(sizeof(int) * 2 * relation_capacity);
    if (!after_relations) {
        free(pos_of_face);
        return 0;
    }

    int repairs = 0;
    const char* instructions = "\nWhen graphics is displayed  :\nESC: abort, RETURN: automatic mode (no graphics), N: skip, any other key: continue.\n\n";
    int automatic_mode = 0; /* 0=graphical inspect, 1=automatic (no graphics) */

    printf("%s", instructions);
    for (int f1 = 0; f1 < n; ++f1) {
        printf("%d ", f1);
        if (faces->display_flag[f1] == 0) continue;
        if (cull_back_faces && faces->plane_d[f1] <= 0) continue;
        for (int f2 = f1 + 1; f2 < n; ++f2) {
            if (faces->display_flag[f2] == 0) continue;
            if (cull_back_faces && faces->plane_d[f2] <= 0) continue;

            /* Quick Z reject: no 2D overlap is possible if depth ranges do not intersect */
            if (faces->z_max[f1] <= faces->z_min[f2] || faces->z_max[f2] <= faces->z_min[f1]) continue;

            /* Quick AABB reject using existing helper */
            int _ix0, _iy0, _ix1, _iy1;
            if (!compute_bbox_intersection(model, f1, f2, &_ix0, &_iy0, &_ix1, &_iy1)) continue;

            // If projected polygons do not overlap (touching considered non-overlap), skip
            if (!projected_polygons_overlap(model, f1, f2)) continue;

            int pos1 = pos_of_face[f1];
            int pos2 = pos_of_face[f2];
            /* Try both intersection centroids (both orders) and prefer the one
             * with the larger positive clipped area. If the selected point yields
             * an indeterminate ray_cast (rc==0), try the other centroid, then
             * finally fall back to bbox-center as last resort. This mirrors the
             * inspector's behaviour but is slightly more tolerant (tries multiple
             * candidates) to avoid missing detectable inversions.
             */
            int cx = 0, cy = 0;
            int point_source = 0; /* 1=cent1, 2=cent2, 3=bbox */
            int rc = 0;

            /* Try a lightweight geometry test first, before running SH clipping. */
            {
                int geo = geo_face_order(model, f1, f2);
                if (geo != 0) {
                    /* geo_face_order returns -1 if f1 is before f2, 1 if f1 is after f2.
                     * check_sort_repair uses rc == -1 to mean f1 is in front.
                     * Invert the sign to match raycast semantics.
                     */
                    rc = (geo == -1) ? 1 : -1;
                    point_source = 4; /* geo */
                }
            }

            long long a1 = 0, a2 = 0;
            int cx1 = 0, cy1 = 0, cx2 = 0, cy2 = 0;
            int have1 = 0;
            int have2 = 0;

            /* Only perform SH centroid clipping if the geometry test is inconclusive. */
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

            // Potential extension: if geo is also indeterminate, we could try a
            // QuickDraw-based centroid raycast here using the QD intersection
            // centroid computed by compute_intersection_centroid_ordered_qd_fixed.
            // That would look something like:
            //
            //   if (rc == 0 && use_qd) {
            //       int qd_cx = 0, qd_cy = 0;
            //       long long qd_area2 = 0;
            //       if (compute_intersection_centroid_ordered_qd_fixed(model, f1, f2, &qd_cx, &qd_cy, &qd_area2) && qd_area2 != 0) {
            //           float _tf1 = 0.0f, _tf2 = 0.0f;
            //           if (ray_cast_distances(model, f1, f2, qd_cx, qd_cy, &_tf1, &_tf2)) {
            //               if (_tf1 < _tf2) rc = -1;
            //               else if (_tf1 > _tf2) rc = 1;
            //               else rc = 0;
            //           }
            //       }
            //   }
            //
            // We leave it commented because it may slow down check_sort_repair too much.

            /* Last resort: bbox center */
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
            if (rc == 0) continue; // undetermined

            if (rc == -1 || rc == 1) {
                /* Single-direction strategy: always move the *closer* face forward
                 * (towards higher indices) to be immediately AFTER the other face.
                 */
                int closer = (rc == -1) ? f1 : f2;
                int other  = (rc == -1) ? f2 : f1;
                int pclos = pos_of_face[closer];
                int pother = pos_of_face[other];
                if (pclos <= pother) {
                    int tmp = faces->sorted_face_indices[pclos];
                    if (!automatic_mode) {
                        char msg[160];
                        snprintf(msg, sizeof(msg), "Face %d after face %d ? \n(N=skip, ESC=abort, RET=auto, other=apply)", closer, other);
                        int _k = show_inspect_faces_with_message(model, closer, other, msg, 3, 188);
                        if (_k == 27) { /* ESC */ free(pos_of_face); free(after_relations); return repairs; }
                        if (_k == 'N' || _k == 'n') {
                            // skip move, continue to next pair
                            if (!automatic_mode) {
                                printf("Check and repair running...\n"); fflush(stdout);
                                printf("\n%s\n", instructions);
                                printf("Face %d\n", f1);
                            }
                            continue;
                        }
                        if (_k == 13 || _k == 10) { /* RETURN/ENTER */
                            automatic_mode = 1;
                            // Fall through to move execution, but also show text status for return.
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

                        /* 
                         * Record the constraint "closer must be after other".
                         * This relation becomes an obligation to maintain after
                         * every change that affects the ordering.
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

    free(pos_of_face);
    free(after_relations);
    printf("\ncheck_sort_repair: repairs=%d\n", repairs);
    return repairs;
}

/* check_sort_repair_fast
 * ----------------------
 * Faster variant of check_sort_repair that:
 *  - uses the QuickDraw region intersection centroid (QD) as the test point,
 *  - uses ray_cast_distances() at that single point,
 *  - performs the same minimal forward-only reordering when the ray cast
 *    indicates the current sort order is incorrect.
 *
 * The intent is to keep the same repair strategy, but with fewer centroid
 * candidates and less per-pair overhead.
 */
int check_sort_repair_fast(Model3D* model, int face_count) {
    if (!model) return 0;
    FaceArrays3D* faces = &model->faces;
    if (face_count <= 0) return 0;

    startgraph(mode); /* Required for QuickDraw region operations */
    
    int n = face_count;
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

    MoveTo(3,10); printf("Computing repairs..."); /* Inform user that the batch test is running (prevents blank-screen confusion) */
    
    for (int i = 0; i < n; ++i) pos_of_face[faces->sorted_face_indices[i]] = i;

    int repairs = 0;

    for (int f1 = 0; f1 < n; ++f1) {
        if (faces->display_flag[f1] == 0) continue;
        if (cull_back_faces && faces->plane_d[f1] <= 0) continue;
        for (int f2 = f1 + 1; f2 < n; ++f2) {
            if (faces->display_flag[f2] == 0) continue;
            if (cull_back_faces && faces->plane_d[f2] <= 0) continue;

            /* Quick Z reject: no 2D overlap is possible if depth ranges do not intersect */
            if (faces->z_max[f1] <= faces->z_min[f2] || faces->z_max[f2] <= faces->z_min[f1]) continue;

            /* Quick AABB reject */
            int _ix0, _iy0, _ix1, _iy1;
            if (!compute_bbox_intersection(model, f1, f2, &_ix0, &_iy0, &_ix1, &_iy1)) continue;

            /* Only consider true overlap (not touching) */
            if (!projected_polygons_overlap(model, f1, f2)) continue;

            /* Use QuickDraw region centroid (fast and representative) as first attempt. */
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

                    // Record required relation A after B and enforce all relations.
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
                    enforce_mandatory_after_relations(model, n, pos_of_face, relation_count, after_relations);
                }
            }
        }
    }
    free(pos_of_face);
    free(after_relations);
    MoveTo(3,20); 
    printf("Repairs done: repairs=%d\n", repairs); /* Inform user that repairs are done */
    printf("Press any key to continue...\n");
    keypress();
    endgraph();
    DoText(); /* Return to text mode */
    return repairs;
}

/* Interactive inspector: single UI to run the full diagnostic battery on a face pair.
 * - Displays model in wireframe and highlights f1 (green) and f2 (orange) with face numbers.
 * - Runs all tests (uses QuickDraw-only tests while in graphical mode).
 * - Stores textual results in memory and only prints them when user presses SPACE.
 * - Keyboard: ESC=return, Left/Right/Up/Down navigation (custom wrap rules), SPACE=show text.
 */

/* Helper: geometric painter-based order wrapper
 * ------------------------------------------------
 * Returns: -1 if f1 is before f2, 1 if f1 is after f2, 0 if undetermined
 * Behavior: runs pair_plane_before/after and, if inconclusive, retries with faces swapped.
 */
static int geo_face_order(Model3D* model, int f1, int f2) {
    if (!model) return 0;
    int r = 0;
    if (pair_plane_before(model, f1, f2)) return -1;
    if (pair_plane_after(model, f1, f2)) return 1;
    /* retry with inverted order as fallback */
    if (pair_plane_before(model, f2, f1)) return 1;  /* f2 before f1 => f1 after f2 */
    if (pair_plane_after(model, f2, f1)) return -1;   /* f2 after f1 => f1 before f2 */
    return r;
}

segment "raytrace";
/* Diagnostic result container for face-pair comparison */
typedef struct {
    int z_verdict;           /* 0=undetermined, 1=f1 in front, 2=f2 in front */
    int bbox_ok;             /* 0/1 */
    int bbox_ix0, bbox_iy0, bbox_ix1, bbox_iy1;
    int poly_overlap;        /* 0/1 */
    int geo_verdict;         /* same encoding as z_verdict */
    int sh_centroid_ok;      /* Sutherland–Hodgman centroid */
    int sh_cx, sh_cy; long long sh_area2;
    int qd_centroid_ok;      /* QuickDraw-region centroid */
    int qd_cx, qd_cy; long long qd_area2;
    int bbox_raycast;        /* 0=undetermined, 1=f1 front, 2=f2 front */
    int sh_raycast;
    int qd_raycast;
    char results_text[16][128];
    int results_count;
} CompareFacesResult;

/* Helper: run ray-cast comparison at a point, return 1 if f1 in front, 2 if f2 in front, 0 undetermined */
static int run_raycast_test(Model3D* model, int f1, int f2, int px, int py) {
    float d1 = 0.0f, d2 = 0.0f;
    if (ray_cast_distances(model, f1, f2, px, py, &d1, &d2)) {
        if (d1 < d2) return 1; else if (d1 > d2) return 2; else return 0;
    }
    int r = ray_cast_at(model, f1, f2, px, py);
    if (r == -1) return 1;     /* f1 closer */
    if (r == 1) return 2;      /* f2 closer */
    return 0;
}
static void inspect_face_pair_ui(Model3D* model) {
    if (!model) return;
    FaceArrays3D* faces = &model->faces;
    int face_count = faces->face_count;
    if (face_count <= 0) { printf("No faces in model\n"); return; }

    /* Title / welcome (text-mode) */
    printf("Inspector: face-pair full diagnostic.\nUse arrows to navigate, space for option\n\n");

    /* Step 1: prompt and validate face IDs */
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

    /* Interactive graphical loop */
    while (1) {
        /* Run diagnostics while QuickDraw is active (use_qd = 1) */
        static CompareFacesResult r; memset(&r, 0, sizeof(r));

        /* Prepare and draw model/wireframe */
        unsigned char* backup_flags = (unsigned char*)malloc(faces->face_count);
        if (!backup_flags) { printf("Memory allocation failed\n"); return; }
        for (int i = 0; i < faces->face_count; ++i) backup_flags[i] = faces->display_flag[i];
        int old_frame = framePolyOnly;

        startgraph(mode);
        framePolyOnly = 1; for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = 1;
        if (jitter) drawPolygons_jitter(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count);
        else drawPolygons(model, faces->vertex_count, faces->face_count, model->vertices.vertex_count);

        /* Highlight faces and show indices */
        unsigned char saved_f1 = faces->display_flag[f1]; unsigned char saved_f2 = faces->display_flag[f2];
        faces->display_flag[f1] = 1; faces->display_flag[f2] = 1;
        drawFace(model, f1, COL_LIGHT_GREEN, 1); drawFace(model, f2, COL_ORANGE, 1);

        /* Compute diagnostics (we are in graphical mode so allow QD tests) */
        /* Defensive check + file-backed trace to capture crashes that occur for specific face pairs. */
        {
            /* compute vertex_indices buffer limit (safe upper bound) */
            int vertex_indices_buffer_limit = 0;
            for (int _ii = 0; _ii < faces->face_count; ++_ii) {
                int _end = faces->vertex_indices_ptr[_ii] + faces->vertex_count[_ii];
                if (_end > vertex_indices_buffer_limit) vertex_indices_buffer_limit = _end;
            }

            /* pre-compare debug trace removed */

            /* Run full diagnostic — compare_faces_diagnostic contains its own guards. */
            compare_faces_diagnostic(model, f1, f2, 1, &r);

        }

        /* Graphical overlays per specification */
        int screenScale = mode / 320;
        /* Decide which test supplies the displayed rectangle + cross.
         * Priority: QD -> bbox -> SH.  Only the selected test draws a rectangle+cross.
         * SH polygon vertices are still drawn separately when available. */
        int draw_mode = 0; /* 0=none, 1=QD, 2=BBOX, 3=SH */
        if (r.qd_centroid_ok && r.poly_overlap) draw_mode = 1;
        else if (r.bbox_ok && r.poly_overlap) draw_mode = 2;
        else if (r.sh_centroid_ok) draw_mode = 3;

        if (draw_mode == 1) { /* QD: yellow rectangle + cross at QD centroid (use region-bbox if available) */
            Rect rr;
            SetSolidPenPat(COL_YELLOW);
            /* Prefer QuickDraw region bbox for the yellow rectangle; fall back to AABB (r.bbox_*) */
            int rb_ix0 = r.bbox_ix0, rb_iy0 = r.bbox_iy0, rb_ix1 = r.bbox_ix1, rb_iy1 = r.bbox_iy1;
            long long rb_area2 = 0;
            if (!compute_intersection_region_bbox(model, f1, f2, &rb_ix0, &rb_iy0, &rb_ix1, &rb_iy1, &rb_area2)) {
                /* fallback already assigned from r.bbox_* */
            }
            int sc_ix0 = screenScale * (rb_ix0 + pan_dx);
            int sc_ix1 = screenScale * (rb_ix1 + pan_dx);
            int sc_iy0 = screenScale * (rb_iy0 + pan_dy);
            int sc_iy1 = screenScale * (rb_iy1 + pan_dy);
            SetRect(&rr, sc_ix0, sc_iy0, sc_ix1, sc_iy1); FrameRect(&rr);
            /* cross at QD centroid coords (QD centroid still used for cross) */
            int sc_cx = screenScale * (r.qd_cx + pan_dx);
            int sc_cy = screenScale * (r.qd_cy + pan_dy);
            int d = 4 * screenScale; int th = screenScale > 0 ? screenScale : 1;
            /* Draw centroid cross in black so it's visible on yellow region */
            PenState ps_cross; GetPenState(&ps_cross);
            SetSolidPenPat(COL_BLACK);
            Rect hr; SetRect(&hr, sc_cx - d, sc_cy - (th/2), sc_cx + d, sc_cy + (th/2) + 1); FrameRect(&hr);
            Rect vr; SetRect(&vr, sc_cx - (th/2), sc_cy - d, sc_cx + (th/2) + 1, sc_cy + d); FrameRect(&vr);
            SetPenState(&ps_cross);
        } else if (draw_mode == 2) { /* BBOX: white rectangle + cross at bbox center */
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
        } else if (draw_mode == 3) { /* SH: blue rectangle + cross at SH centroid */
            if (debug_clip_fixed_vcount >= 3) {
                /* compute AABB around SH polygon */
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
                /* cross at SH centroid returned by diagnostic */
                int sc_cx = screenScale * (r.sh_cx + pan_dx);
                int sc_cy = screenScale * (r.sh_cy + pan_dy);
                int d = 4 * screenScale; int th = screenScale > 0 ? screenScale : 1;
                Rect hr; SetRect(&hr, sc_cx - d, sc_cy - (th/2), sc_cx + d, sc_cy + (th/2) + 1); FrameRect(&hr);
                Rect vr; SetRect(&vr, sc_cx - (th/2), sc_cy - d, sc_cx + (th/2) + 1, sc_cy + d); FrameRect(&vr);
            }
        } /* else draw_mode == 0: no rectangle/cross */

        /* Draw Sutherland–Hodgman intersection polygon **only if SH succeeded** (avoid stale globals).
         * Require both r.sh_centroid_ok and a non-empty debug polygon buffer. */
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

            /* Draw centroid cross computed from rounded polygon verts (robust) —
             * only draw the SH centroid cross when SH is the selected display mode.
             * The vertex markers remain drawn regardless. */
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
                /* Only draw SH centroid cross when SH is the selected display mode */
                if (draw_mode == 3) {
                    int sc_cx = screenScale * (draw_cx + pan_dx);
                    int sc_cy = (draw_cy + pan_dy);
                    int d2 = 4 * screenScale; int th2 = screenScale > 0 ? screenScale : 1;
                    Rect hr2; SetRect(&hr2, sc_cx - d2, sc_cy - (th2/2), sc_cx + d2, sc_cy + (th2/2) + 1); FrameRect(&hr2);
                    Rect vr3; SetRect(&vr3, sc_cx - (th2/2), sc_cy - d2, sc_cx + (th2/2) + 1, sc_cy + d2); FrameRect(&vr3);
                }
            }
        }

        /* Polygons overlap indicator at bottom (graphical only) */
        MoveTo(3, 188);
        if (r.poly_overlap) {
            printf("f%d overlaps f%d", f1, f2);
            /* Decide which face is in front with priority: QD -> SH -> BBOX */
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


        /* Wait for key (same inline read used elsewhere) */
        int key = getkeypress ();

        /* Handle keys (simplified): ←/→ change face2 only, ↑/↓ change face1 only */
        if (key == 27) { /* ESC: exit inspector */
            framePolyOnly = old_frame;
            for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = backup_flags[i];
            free(backup_flags);
            endgraph(); DoText(); return;
        } else if (key == 8) { /* Left arrow: decrement face2 */
            f2 = (f2 - 1 + face_count) % face_count;
            /* Avoid comparing face with itself: if decrement landed on f1, jump to f1-1 (skip f1)
             * instead of returning to f1+1. */
            if (face_count > 1 && f2 == f1) f2 = (f1 - 1 + face_count) % face_count;
        } else if (key == 21) { /* Right arrow: increment face2 */
            f2 = (f2 + 1) % face_count;
            if (face_count > 1 && f2 == f1) f2 = (f2 + 1) % face_count;
        } else if (key == 11) { /* Up arrow: increment face1 */
            f1 = (f1 + 1) % face_count;
            if (face_count > 1 && f1 == f2) f1 = (f1 + 1) % face_count;
        } else if (key == 10) { /* Down arrow: decrement face1 */
            f1 = (f1 - 1 + face_count) % face_count;
            if (face_count > 1 && f1 == f2) f1 = (f1 - 1 + face_count) % face_count;
        } else if (key == 'F' || key == 'f') { /* dump current diagnostic to file */
            dump_compare_results_to_file(&r, f1, f2);
        } else if (key == 32) { /* SPACE -> compact textual summary (80x24) + optional verbose */
            /* restore QuickDraw and enter text-mode */
            framePolyOnly = old_frame;
            for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = backup_flags[i];
            free(backup_flags);

        /* Also compute the same raycast verdicts with an artificial x10 zoom,
        * for later display in the text-mode summary (SPACE key).
        * This MUST run here, while QuickDraw is still active (before endgraph()),
        * otherwise the QD centroid path (use_qd=1) silently fails and always
        * reports "undetermined" regardless of the zoom factor. */
        static int zoom_bbox_raycast = 0, zoom_sh_raycast = 0, zoom_qd_raycast = 0;
        {
            /* Heap-allocated on purpose: both structures are large enough to risk
            * stack exhaustion if declared as local variables on this 65816/ORCA-C
            * target, and we don't need them to persist beyond this block, so a
            * permanent static allocation would waste memory for no benefit. */
            CompareFacesResult* r_zoom = (CompareFacesResult*)malloc(sizeof(CompareFacesResult));
            RCHZoomPatch* patch = (RCHZoomPatch*)malloc(sizeof(RCHZoomPatch));

            if (r_zoom && patch) {
                memset(r_zoom, 0, sizeof(*r_zoom));

                /* Temporarily re-project only f1/f2's vertices at a larger scale
                * (see ray_cast_hierarchical's zoom patch for the full rationale),
                * re-run the diagnostic, then fully restore the original state. */
                Fixed32 test_scale = FIXED_MUL_64(s_global_proj_scale_fixed, FLOAT_TO_FIXED(10.0f));
                rch_patch_pair_at_scale(model, f1, f2, test_scale, patch);
                compare_faces_diagnostic(model, f1, f2, 1, r_zoom); /* use_qd=1: QuickDraw is active here */
                rch_unpatch_pair(model, patch, f1, f2);

                /* Keep only the three small integers we actually need later;
                * the rest of CompareFacesResult (including its text buffers)
                * is freed right away and never kept around. */
                zoom_bbox_raycast = r_zoom->bbox_raycast;
                zoom_sh_raycast   = r_zoom->sh_raycast;
                zoom_qd_raycast   = r_zoom->qd_raycast;
            } else {
                /* Allocation failed: fall back to "undetermined" rather than crash. */
                zoom_bbox_raycast = zoom_sh_raycast = zoom_qd_raycast = 0;
            }
            if (r_zoom) free(r_zoom);
            if (patch) free(patch);
        }

            endgraph(); 
            DoText();
            // now in text mode

            // Title for the face pair section
            printf("                =========== Face pair: f%d vs f%d ===========               \n", f1, f2);
            
            /* sorted list position */
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

            /* combine key tests in one line */
            // printf("\n"); /* blank line before BBOX info */
            /* build string for 3D geometry result using actual face IDs */
            char geo_str[16];
            if (r.geo_verdict == 1) sprintf(geo_str, "f%d", f1);
            else if (r.geo_verdict == 2) sprintf(geo_str, "f%d", f2);
            else strcpy(geo_str, "?");
            printf("Tests: Z-range = %s ; Bbox = %s ; 3D = %s ; Poly = %s\n",
                   (r.z_verdict==1?"f1<f2":(r.z_verdict==2?"f2<f1":"?")),
                   (r.bbox_ok?"yes":"no"),
                   geo_str,
                   (r.poly_overlap?"yes":"no"));

            
            // printf("Ray offsets: bbox = %d ; sh = %d ; qd = %d\n", r.bbox_raycast,r.sh_raycast,r.qd_raycast);
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

            /* Second line: same verdicts, but computed earlier with the artificial
            * x10 zoom (see the block added right after the main compare_faces_diagnostic
            * call, while still in graphical mode). Only three small static ints were
            * kept from that computation — nothing heavy is read here. */
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


            printf("\n"); // blank line between faces
            /* face equations and Z stats on one line each */
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
            // printf("\n"); // blank line between faces
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

            printf("\n"); /* blank line before BBOX info */
            /* 1) Bbox (single line) - now shows rect + OK/KO */
            if (r.bbox_ok) {
                int bcx = (r.bbox_ix0 + r.bbox_ix1) / 2;
                int bcy = (r.bbox_iy0 + r.bbox_iy1) / 2;
                float bf1 = 0.0f, bf2 = 0.0f;
                if (ray_cast_distances(model, f1, f2, bcx, bcy, &bf1, &bf2)) {
                    printf("1) BBOX OK rect=%d,%d-%d,%d center=%d,%d dist:%d=%.4f %d=%.4f", r.bbox_ix0, r.bbox_iy0, r.bbox_ix1, r.bbox_iy1, bcx, bcy, f1, bf1, f2, bf2);
                } else {
                    printf("1) BBOX OK rect=%d,%d-%d,%d center=%d,%d dist:N/A", r.bbox_ix0, r.bbox_iy0, r.bbox_ix1, r.bbox_iy1, bcx, bcy);
                }
                /* verdict (one-line break before verdict) */
                if (r.bbox_raycast == 1) printf("\n   => face %d is in front", f1);
                else if (r.bbox_raycast == 2) printf("\n   => face %d is in front", f2);
                else printf("\n  -> undetermined");
            } else printf("1) BBOX KO rect=none center=none");

            /* 2) Centroid SH (single line) - prefix OK/KO */
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

            /* 3) Centroid QD (single line) - use region bbox when available; fallback to AABB; prefix OK/KO */
            if (r.qd_centroid_ok) {
                int qx0=0,qy0=0,qx1=0,qy1=0; long long qarea2=0; float qf1=0.0f, qf2=0.0f;
                int have_qd_rect = compute_intersection_region_bbox(model, f1, f2, &qx0, &qy0, &qx1, &qy1, &qarea2);
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


            /* Press R/E to reorder the sorted list, any other key to return */
            printf("\n'R' to move the back face in front\n'E' to move the front face behind\n'G' for advanced geometric details using plane equations\nAny other key to return to graphical inspector...\n");
            char cmd = getkeypress ();

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

            if (cmd == 'G' || cmd == 'g') {
                pair_plane_geometric_tests(model, f1, f2);
            }
                continue; /* redraw graphical inspector */
            }

        /* restore and loop to redraw with updated IDs */
        framePolyOnly = old_frame;
        for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = backup_flags[i];
        free(backup_flags);
        endgraph(); 
        DoText();
    }
}

/* Core diagnostic: performs the ordered suite of tests (text-mode safe unless use_qd=1)
 * Tests executed in the order requested by the UI design and records textual lines.
 */
/* Helper: dump diagnostic results into a text file (appends).
 * The filename uses the convention f<id1>VSf<id2>.txt so each pair gets
 * its own file.  A header line and all saved result_text entries are
 * written before closing the file.
 */
static void dump_compare_results_to_file(const CompareFacesResult *out, int f1, int f2) {
    if (!out) return;
    char fname[64];
    sprintf(fname, "f%dVSf%d.txt", f1, f2);
    FILE *f = fopen(fname, "a");
    if (!f) return;
    fprintf(f, "=== Face pair: f%d vs f%d ===\n", f1, f2);
    for (int i = 0; i < out->results_count; ++i) {
        fprintf(f, "%s\n", out->results_text[i]);
    }
    fprintf(f, "\n");
    fclose(f);
}

static void compare_faces_diagnostic(Model3D* model, int f1, int f2, int use_qd, CompareFacesResult *out) {

    if (!model || !out) return;
    FaceArrays3D* faces = &model->faces;
    memset(out, 0, sizeof(*out));

    /* 1) Z-range test (timed) */
    {
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

    /* 2) bbox overlap (timed) */
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

    /* 3) polygon overlap (only if bbox overlap, timed) */
    if (out->bbox_ok) {
        long t_poly_start = GetTick();
        int p_overlap = projected_polygons_overlap(model, f1, f2);
        long t_poly_end = GetTick();
        /* compare: poly_overlap trace removed */
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

    /* 4) 3D geometry (painter tests via plane relations, timed) */
    {
        long t_geo_start = GetTick();
        int geo = geo_face_order(model, f1, f2);
        long t_geo_end = GetTick();
        /* geo == -1 means f1 is behind f2, so f2 is in front */
        if (geo == -1) { out->geo_verdict = 2; snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "3D geometry: f%d in front of f%d (%ld ticks)", f2, f1, t_geo_end - t_geo_start); }
        else if (geo == 1) { out->geo_verdict = 1; snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "3D geometry: f%d in front of f%d (%ld ticks)", f1, f2, t_geo_end - t_geo_start); }
        else { out->geo_verdict = 0; snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "3D geometry: Undetermined (%ld ticks)", t_geo_end - t_geo_start); }
    }

    /* If polygons do not overlap, mark tests 5..9 as non-applicable (N/A) */
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
        /* 5) Centroid via Sutherland–Hodgman (integer) */
        {
            long long area2 = 0;
            long t_sh_start = GetTick();
            int sh_fallback = 0;
            int sh_ok = compute_intersection_centroid_ordered_fixed_tryboth(model, f1, f2, &out->sh_cx, &out->sh_cy, &area2, &sh_fallback) && (area2 != 0);
            long t_sh_end = GetTick();
            out->sh_centroid_ok = sh_ok ? 1 : 0; out->sh_area2 = area2;
            if (out->sh_centroid_ok) snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Centroid SH: %d/%d (area2=%lld) (%ld ticks)", out->sh_cx, out->sh_cy, (long long)area2, t_sh_end - t_sh_start);
            else {
                /* If clipping previously bailed out for this pair, report that explicitly */
                if ((clip_last_bailout_subj == f1 && clip_last_bailout_clip == f2) || (clip_last_bailout_subj == f2 && clip_last_bailout_clip == f1)) {
                    snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Centroid SH: undetermined (clipping bail-out) (%ld ticks)", t_sh_end - t_sh_start);
                    /* clear marker after reporting */
                    clip_last_bailout_subj = -1; clip_last_bailout_clip = -1;
                } else {
                    snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Centroid SH: undetermined (%ld ticks)", t_sh_end - t_sh_start);
                }
                /* Explicitly clear SH debug polygon to avoid stale visuals elsewhere */
                debug_clip_fixed_vcount = 0; debug_clip_raw_area2 = 0; debug_clip_fixed_area = 0.0;
                debug_clip_centroid_x = 0; debug_clip_centroid_y = 0;
            }
        }

        /* 6) Centroid via QuickDraw region (only if use_qd==1 and region available) */
        out->qd_centroid_ok = 0;
        if (use_qd) {
            long long qd_a2 = 0;
            long t_qd_start = GetTick();
            int qd_ok = compute_intersection_centroid_ordered_qd_fixed(model, f1, f2, &out->qd_cx, &out->qd_cy, &qd_a2);
            long t_qd_end = GetTick();
            /* compare: QD result trace removed */
            out->qd_centroid_ok = qd_ok ? 1 : 0; out->qd_area2 = qd_a2;
            if (out->qd_centroid_ok) snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Centroid QD: %d/%d (area2=%lld) (%ld ticks)", out->qd_cx, out->qd_cy, (long long)qd_a2, t_qd_end - t_qd_start);
            else snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Centroid QD: undetermined (%ld ticks)", t_qd_end - t_qd_start);
        } else {
            snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Centroid QD: skipped (QD disabled)");
        }

        /* 7) Ray cast at bbox center */
        if (out->bbox_ok) {
            int bcx = (out->bbox_ix0 + out->bbox_ix1) / 2; int bcy = (out->bbox_iy0 + out->bbox_iy1) / 2;
            long t_bbox_rc_start = GetTick();
            out->bbox_raycast = run_raycast_test(model, f1, f2, bcx, bcy);
            long t_bbox_rc_end = GetTick();
            /* compare: bbox_raycast trace removed */
            if (out->bbox_raycast == 1) snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Bbox raycast: f%d in front of f%d (%ld ticks)", f1, f2, t_bbox_rc_end - t_bbox_rc_start);
            else if (out->bbox_raycast == 2) snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Bbox raycast: f%d in front of f%d (%ld ticks)", f2, f1, t_bbox_rc_end - t_bbox_rc_start);
            else snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Bbox raycast: undetermined (%ld ticks)", t_bbox_rc_end - t_bbox_rc_start);
        } else {
            snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Bbox raycast: skipped (no bbox)");
        }

        /* 8) Ray cast at SH centroid */
        if (out->sh_centroid_ok) {
            long t_shrc_start = GetTick();
            out->sh_raycast = run_raycast_test(model, f1, f2, out->sh_cx, out->sh_cy);
            long t_shrc_end = GetTick();
            /* compare: SH_raycast trace removed */
            if (out->sh_raycast == 1) snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Centroid raycast: f%d in front of f%d (%ld ticks)", f1, f2, t_shrc_end - t_shrc_start);
            else if (out->sh_raycast == 2) snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Centroid raycast: f%d in front of f%d (%ld ticks)", f2, f1, t_shrc_end - t_shrc_start);
            else snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Centroid raycast: undetermined (%ld ticks)", t_shrc_end - t_shrc_start);
        } else {
            snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "Centroid raycast: skipped (no SH centroid)");
        }

        /* 9) Ray cast at QD centroid */
        if (use_qd && out->qd_centroid_ok) {
            long t_qdrc_start = GetTick();
            out->qd_raycast = run_raycast_test(model, f1, f2, out->qd_cx, out->qd_cy);
            long t_qdrc_end = GetTick();
            /* compare: QD_raycast trace removed */
            if (out->qd_raycast == 1) snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "QD centroid raycast: f%d in front of f%d (%ld ticks)", f1, f2, t_qdrc_end - t_qdrc_start);
            else if (out->qd_raycast == 2) snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "QD centroid raycast: f%d in front of f%d (%ld ticks)", f2, f1, t_qdrc_end - t_qdrc_start);
            else snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "QD centroid raycast: undetermined (%ld ticks)", t_qdrc_end - t_qdrc_start);
        } else {
            snprintf(out->results_text[out->results_count++], sizeof(out->results_text[0]), "QD centroid raycast: skipped (QD unavailable)");
        }
    }
}

/* pair_plane_geometric_tests
 * --------------------------------
 * Fixed-point debug wrapper for the normal `pair_plane_before()` ordering test.
 *
 * Purpose:
 * - Provide a diagnostic version of the plane-based face ordering test that can
 *   be used interactively or in quiet/autotest mode.
 * - Print detailed debug output for plane coefficients, vertex-side comparisons,
 *   and test pass/fail decisions.
 * - Help developers understand why two faces are considered ordered or inconclusive.
 *
 * Behavior:
 * - If `pair_plane_before_debug_quiet` is set, this function forwards directly to
 *   `pair_plane_before()` and suppresses interactive debugging.
 * - Otherwise it prompts the user for face indices (unless auto-pair values are set),
 *   then evaluates Tests 4 and 5 with verbose output.
 * - It uses cached plane coefficients from `calculateFaceDepths()` and converts them
 *   to doubles only for reporting, while the actual geometric decision logic is still
 *   conceptually the same as the underlying plane tests.
 *
 * Test semantics:
 * - Test 4: Verify whether all vertices of face2 lie on the same observer-side
 *   of face1's plane. If so, face2 is in front of face1 and the ordering is confirmed.
 * - Test 5: Verify whether all vertices of face1 lie on the opposite observer-side
 *   of face2's plane. If so, face1 is behind face2 and the ordering is confirmed.
 *
 * Note:
 * - This function is primarily a debug aid, not a production ordering routine.
 * - It prints per-vertex values and pauses via `PKEYP()` so the developer can inspect
 *   intermediate results.
 */

/* helper macros for guarded prints */
#define PDBG(...)  if (!pair_plane_before_debug_quiet) printf(__VA_ARGS__)
#define PKEYP()    if (!pair_plane_before_debug_quiet) keypress()

/* globals used when autotest calls debug function without interaction */
static int autopair_f1 = -1;
static int autopair_f2 = -1;

/* flag used to silence pair_plane_before_debug during autotest */
static int pair_plane_before_debug_quiet = 0;

/* debug version of pair_plane_before: prompts user, prints diagnostics */
static int pair_plane_before_debug(Model3D* model, int f1, int f2) {
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;

    /* if autopair values are set, use them quietly */
    if (autopair_f1 >= 0 && autopair_f2 >= 0) {
        f1 = autopair_f1;
        f2 = autopair_f2;
    }
    /* if running in silent/autotest mode, just forward to non-debug implementation */
    if (pair_plane_before_debug_quiet) {
        return pair_plane_before(model, f1, f2);
    }
    else {
        PDBG("PAIR_DEBUG: enter two face indices f1 f2 (or any numbers): "); fflush(stdout);
        if (scanf("%d %d", &f1, &f2) != 2) {
            PDBG("invalid input\n"); PKEYP(); return 0;
        }
        int c; while ((c = getchar()) != '\n' && c != EOF) ;
        PDBG("\n PAIR_DEBUG: comparing %d vs %d\n", f1, f2);
    }

    // Use cached plane normals and d terms computed in calculateFaceDepths
    int n1 = faces->vertex_count[f1];
    int n2 = faces->vertex_count[f2];
    int offset1 = faces->vertex_indices_ptr[f1];
    int offset2 = faces->vertex_indices_ptr[f2];
    int k;
    /* convert planes to doubles for debug arithmetic */
    double a1 = FIXED64_TO_FLOAT(faces->plane_a[f1]);
    double b1 = FIXED64_TO_FLOAT(faces->plane_b[f1]);
    double c1 = FIXED64_TO_FLOAT(faces->plane_c[f1]);
    double d1 = FIXED64_TO_FLOAT(faces->plane_d[f1]);
    double a2 = FIXED64_TO_FLOAT(faces->plane_a[f2]);
    double b2 = FIXED64_TO_FLOAT(faces->plane_b[f2]);
    double c2 = FIXED64_TO_FLOAT(faces->plane_c[f2]);
    double d2 = FIXED64_TO_FLOAT(faces->plane_d[f2]);
    double epsilon = 0.01;

    int obs_side1 = 0; // observer side relative to plane of f1: +1, -1, or 0 (inconclusive)
    int obs_side2 = 0; // observer side relative to plane of f2: +1, -1, or 0 (inconclusive)
    int side;           // vertex side.
    int all_same_side; // flag indicating whether all vertices are on the same side
    int all_opposite_side; // flag indicating whether all vertices are on the opposite side
    /* values will be computed as doubles below */

    // ********************* TEST 4 *********************

    // Test 4: Is f2 entirely on the same observer-side of f1's plane?
    // - Evaluate face-plane sign for the observer (d1) and then test every vertex of f2
    //   against f1's plane (A1*x + B1*y + C1*z + D1). If all vertices are on the
    //   same side as the observer (within epsilon), then f2 is conclusively in front
    //   of f1 (no swap required). If any vertex is on the opposite side, the test fails
    //   and we continue to stronger tests.


    PDBG("\n**** Test 4 : Testing faces %d and %d\n", f1, f2);
    PDBG("Face coefs: a1=%f, b1=%f, c1=%f, d1=%f\n", a1, b1, c1, d1);
    obs_side1 = 0; // sign of d1: +1, -1 or 0 (inconclusive)
    if (d1 > epsilon) obs_side1 = 1; 
    else if (d1 < -epsilon) obs_side1 = -1;
    else goto skipT4_debug; // si l'observateur est sur le plan, on ne peut rien conclure, il faut faire d'autres tests
    all_same_side = 1;

    printf("FOR loop start\n");
    printf("obs_side1 = %d\n", obs_side1);
    printf("test_values for face %d must be of the same sign as obs_side1\n", f2);

    for (k=0; k<n2; k++) {
            int v = faces->vertex_indices_buffer[offset2+k]-1;
            /* compute using doubles */
            double vx = FIXED_TO_FLOAT(vtx->xo[v]);
            double vy = FIXED_TO_FLOAT(vtx->yo[v]);
            double vz = FIXED_TO_FLOAT(vtx->zo[v]);
            double acc = a1 * vx + b1 * vy + c1 * vz + d1;

            PDBG("k = %d, vertex index = %d, vtx = (%f, %f, %f)\n", k, v+1, vx, vy, vz);
            PDBG("test_value = %f\n", acc);

            if  (acc > epsilon) side = 1;
            else if (acc < -epsilon) side = -1;
            else continue; // si le vertex est sur le plan, on l'ignore et on passe au vertex suivant
            if (obs_side1 != side) { 
                // if a vertex is on the other side, break the loop
                // and set the flag to 0 to indicate the test failed (move to next test)
                all_same_side = 0; 
                
                    printf("Test 4 failed for faces %d and %d\n", f1, f2);
                    keypress(); 
                break; 
                }
    }

        PDBG("FOR loop stop\n");

    // test 4 passed
    if (all_same_side) { 
        PDBG("Test 4 passed for Faces %d and %d\n", f1, f2);
        PKEYP();
        return 1; // faces are ordered correctly, move to next pair
    }

    skipT4_debug:

    // ********************* TEST 5 ********************
    // Test 5: Is f1 entirely on the opposite side of f2's plane relative to the observer?
    // - This is symmetric to Test 4: if every vertex of f1 is strictly on the opposite
    //   side of f2's plane from the observer, then f1 is behind f2 and no swap is needed.
    // - Both Test 4 and Test 5 are relatively cheap and frequently decisive on planar geometry.


    PDBG("\n\nTest 5 : Testing faces %d and %d\n", f1, f2);
    PDBG("Face coefs: a2=%f, b2=%f, c2=%f, d2=%f\n", a2, b2, c2, d2 );

    obs_side2 = 0; // sign of d2: +1, -1 or 0 (inconclusive)
    if (d2 > epsilon) obs_side2 = 1; 
    else if (d2 < -epsilon) obs_side2 = -1;
    else return 0; // si l'observateur est sur le plan, on ne peut rien conclure, il faut faire d'autres tests
    all_opposite_side = 1;

    PDBG("FOR loop start\n");
    PDBG("obs_side2 = %d\n", obs_side2);
    PDBG("test_values for face %d must be of the opposite sign as obs_side2\n", f1);

    for (k = 0; k < n1; k++) {
        int v = faces->vertex_indices_buffer[offset1 + k] - 1;
        /* compute using doubles */
        double vx = FIXED_TO_FLOAT(vtx->xo[v]);
        double vy = FIXED_TO_FLOAT(vtx->yo[v]);
        double vz = FIXED_TO_FLOAT(vtx->zo[v]);
        double acc = a2 * vx + b2 * vy + c2 * vz + d2;

        PDBG("k = %d, vertex index = %d, vtx = (%f, %f, %f)\n", k, v+1, vx, vy, vz);
        PDBG("test_value = %f\n", acc);

        if (acc > epsilon) side = 1;
        else if (acc < -epsilon) side = -1;
        //else continue; // si le vertex est sur le plan, on l'ignore et on passe au vertex suivant
        // if (obs_side2 == side) {
        //     // if a vertex is on the same side, break the loop
        //     // and set the flag to 0 to indicate the test failed (move to next test)
        //     all_opposite_side = 0;

        //     printf("Test 5 failed for faces %d and %d\n", f1, f2);
        //     keypress();
        //     //break; 
        //     }
    }
    PDBG("FOR loop stop\n");

        // test 5 passed
        if (all_opposite_side) { // faces are ordered correctly, move to next pair
            PDBG("Test 5 passed for Faces %d and %d\n", f1, f2);
            PKEYP();
            return 1;
        }
}

static int pair_plane_geometric_tests(Model3D* model, int f1, int f2) {
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int test1state = 0, test2state = 0, test3state = 0, test4state = 0; /* 0=inconclusive, 1=passed, -1=failed */
    int c;

    if (f1 == -1 && f2 == -1) {
        printf("PAIR_DEBUG: enter face index f1: "); fflush(stdout);
        if (scanf("%d", &f1) != 1) {
            printf("invalid input\n"); keypress(); return 0;
        }
        while ((c = getchar()) != '\n' && c != EOF) ;

        printf("PAIR_DEBUG: enter face index f2: "); fflush(stdout);
        if (scanf("%d", &f2) != 1) {
            printf("invalid input\n"); keypress(); return 0;
        }
        while ((c = getchar()) != '\n' && c != EOF) ;
    }
    printf("\n=== Geometric test (using plane equations) comparing %d vs %d ===\n", f1, f2);

    // Use cached plane normals and d terms computed in calculateFaceDepths
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

    /* compute epsilon based on magnitude of the two plane normals
       instead of scanning every vertex; this keeps value bounded even when
       observer-space coordinates are large.
       
       Formula:
       - norm1 = length(normal1)
       - norm2 = length(normal2)
       - epsf = max(norm1, norm2) * 0.001   (0.1% of the larger normal)
       - epsf = max(epsf, 0.01)              (minimum tolerance)
       - epsilon = convert epsf to Fixed32 scale
    */
    Fixed32 epsilon;
    {
        long t0 = GetTick();
        double fa1 = FIXED64_TO_FLOAT(a1);
        double fb1 = FIXED64_TO_FLOAT(b1);
        double fc1 = FIXED64_TO_FLOAT(c1);
        double fa2 = FIXED64_TO_FLOAT(a2);
        double fb2 = FIXED64_TO_FLOAT(b2);
        double fc2 = FIXED64_TO_FLOAT(c2);
        double norm1 = sqrt(fa1*fa1 + fb1*fb1 + fc1*fc1);
        double norm2 = sqrt(fa2*fa2 + fb2*fb2 + fc2*fc2);
        double base = (norm1 > norm2) ? norm1 : norm2;
        double epsf = base * 0.001;  /* 0.1% of larger normal length */
        if (epsf < 0.01) epsf = 0.01;
        epsilon = FLOAT_TO_FIXED((float)epsf);
        long t1 = GetTick();
        PDBG("Epsilon (calculated from plane normals) = %f n1=%f n2=%f ticks=%ld\n",
               epsf, norm1, norm2, t1 - t0);
    }

    int obs_side1 = 0; // observer side relative to plane of f1: +1, -1, or 0 (inconclusive)
    int obs_side2 = 0; // observer side relative to plane of f2: +1, -1, or 0 (inconclusive)
    int side;           // vertex side.
    int all_same_side; // flag indicating whether all vertices are on the same side of the observer
    int all_opposite_side; // flag indicating whether all vertices are on the opposite side of the observer
    /* test_value replaced by Fixed64 accumulators inside loops to avoid Fixed32 overflow */

    // ********************* TEST 1 *********************

    // Test 1: Is f2 entirely on the same observer-side of f1's plane?
    // - Evaluate face-plane sign for the observer (d1) and then test every vertex of f2
    //   against f1's plane (A1*x + B1*y + C1*z + D1). If all vertices are on the
    //   same side as the observer (within epsilon), then f2 is conclusively in front
    //   of f1 (no swap required). If any vertex is on the opposite side, the test fails
    //   and we continue to stronger tests.


    PDBG("\n******** Test #1 : Testing if faces %d is in front of %d ********\n", f2, f1);
    // PDBG("Face coefs: a1=%f, b1=%f, c1=%f, d1=%f\n", FIXED64_TO_FLOAT(a1), FIXED64_TO_FLOAT(b1), FIXED64_TO_FLOAT(c1), FIXED64_TO_FLOAT(d1));
    obs_side1 = 0; // sign of d1: +1, -1 or 0 (inconclusive)
    if (d1 > (Fixed64)epsilon) obs_side1 = 1; 
    else if (d1 < -(Fixed64)epsilon) obs_side1 = -1;
    else {
        test1state = 0;
        goto skipT1_debug; // si l'observateur est sur le plan, on ne peut rien conclure, il faut faire d'autres tests
    }
    all_same_side = 1;
    int test1_effective_count = 0;

    printf("Observer side (obs_side1) = %d\n", obs_side1);
    printf("If face %d entirely on the same observer-side of %d's plane, then\n", f2, f1);
    printf("test_values for all vertices of face %d have the same sign as obs.side.\n", f2);

    for (k=0; k<n2; k++) {
            int v = faces->vertex_indices_buffer[offset2+k]-1;
            /* Accumulate in 64-bit to avoid overflow (each product is >> FIXED_SHIFT to keep Fixed32 scale) */
            Fixed64 acc = 0;
            acc  = (((Fixed64)a1 * (Fixed64)vtx->xo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)b1 * (Fixed64)vtx->yo[v]) >> FIXED_SHIFT);
            acc += (((Fixed64)c1 * (Fixed64)vtx->zo[v]) >> FIXED_SHIFT);
            acc += (Fixed64)d1; // d1 already Fixed32 scale

            // PDBG("k = %d, vertex index = %d, vtx = (%f, %f, %f)\n",
            //        k, v+1,
            //        FIXED_TO_FLOAT(vtx->xo[v]),
            //        FIXED_TO_FLOAT(vtx->yo[v]),
            //        FIXED_TO_FLOAT(vtx->zo[v]));

            /* show raw fixed value, integer part (shifted) and float equivalent */
            // PDBG("test_value (fixed64 raw) = %lld\n", (long long)acc);
            // PDBG("test_value (fixed64 >>%d) = %lld\n", FIXED_SHIFT, (long long)(acc >> FIXED_SHIFT));

            PDBG("Vertex #%d ; test_value = %f ", k, FIXED64_TO_FLOAT(acc));

            if (acc > (Fixed64)epsilon) side = 1;
            else if (acc < -(Fixed64)epsilon) side = -1;
            else 
            {
                printf("==> ignored\n");   
                continue; // ignore vertices that are effectively on the plane
            }

            test1_effective_count++;
            if (obs_side1 == side) {
                printf("==> OK\n");
            }            

            if (obs_side1 != side) {
                /* if a vertex of f2 is on the opposite side of f1's plane than the observer,
                 * Test #1 cannot conclude the ordering; continue to later tests. */
                all_same_side = 0;
                test1state = -1;
                printf("==> KO\n");
                PDBG("Test #1 failed for faces %d and %d\n", f1, f2);
                PKEYP();
                break;
            }
    }
    // PDBG("FOR loop stop\n");
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

    // ********************* TEST #2 ********************
    // Test #2: Is f1 entirely on the opposite side of f2's plane relative to the observer?
    // - This is symmetric to Test #1: if every vertex of f1 is strictly on the opposite
    //   side of f2's plane from the observer, then f1 is behind f2 and the ordering is confirmed.
    // - This test is performed only if the observer is clearly on one side of f2's plane.

    PDBG("\n******** Test #2 : Testing if faces %d is behind %d ********\n", f1, f2);
    //PDBG("Face coefs: a2=%f, b2=%f, c2=%f, d2=%f\n", FIXED64_TO_FLOAT(a2), FIXED64_TO_FLOAT(b2), FIXED64_TO_FLOAT(c2), FIXED64_TO_FLOAT(d2));

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

        // PDBG("k = %d, vertex index = %d, vtx = (%f, %f, %f)\n",
        //      k, v+1,
        //      FIXED_TO_FLOAT(vtx->xo[v]),
        //      FIXED_TO_FLOAT(vtx->yo[v]),
        //      FIXED_TO_FLOAT(vtx->zo[v]));

        PDBG("Vertex #%d ; test_value = %f ", k, FIXED64_TO_FLOAT(acc));
        // PDBG("test_value (float equiv)        = %f\n", FIXED64_TO_FLOAT(acc));

        if (acc > (Fixed64)epsilon) side = 1;
        else if (acc < -(Fixed64)epsilon) side = -1;
        else 
        {
            printf("==> ignored\n");   
            continue; // ignore vertices that are effectively on the plane
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
    // PDBG("FOR loop stop\n");

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

    // ********************* TEST #3 ********************
    // Test #3: Copy of Test #1 with faces swapped (check if f1 is in front of f2).
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

    // ********************* TEST #4 ********************
    // Test #4: Copy of Test #2 with faces swapped (check if f2 is behind f1).
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
    if (test1state == 1 || test2state == 1) return 1; else return 0;
}

/* Helper: show model wireframe, highlight two faces and display a message (MoveTo + printf).
 * - Renders the model in wireframe, highlights `f1`/`f2`, performs MoveTo(3,198) then prints `msg`,
 *   waits for a key, and restores rendering state. Kept minimal and non-invasive.
 */
static int show_inspect_faces_with_message(Model3D* model, int f1, int f2, const char* msg, int screenx, int screeny) {
    if (!model) return;
    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    unsigned char* backup_flags = (unsigned char*)malloc(faces->face_count);
    if (backup_flags == NULL) { printf("Memory allocation failed\n"); return; }
    for (int i = 0; i < faces->face_count; ++i) backup_flags[i] = faces->display_flag[i];
    int old_frame = framePolyOnly;

    startgraph(mode);
    framePolyOnly = 1; /* wireframe */
    for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = 1;
    if (jitter) drawPolygons_jitter(model, faces->vertex_count, faces->face_count, vtx->vertex_count);
    else drawPolygons(model, faces->vertex_count, faces->face_count, vtx->vertex_count);

    unsigned char saved_f1 = faces->display_flag[f1]; unsigned char saved_f2 = faces->display_flag[f2];
    faces->display_flag[f1] = 1; faces->display_flag[f2] = 1;
    drawFace(model, f1, COL_LIGHT_GREEN, 1);
    drawFace(model, f2, COL_ORANGE, 1);
    faces->display_flag[f1] = saved_f1; faces->display_flag[f2] = saved_f2;

    MoveTo(screenx, screeny);
    printf("%s", msg);
    // Wait for key
    int key = getkeypress ();

    framePolyOnly = old_frame;
    for (int i = 0; i < faces->face_count; ++i) faces->display_flag[i] = backup_flags[i];
    free(backup_flags);
    endgraph(); DoText();
    return key;
}


// Dump face plane coefficients and depth stats to CSV
// Columns: face;a;b;c;d;z_min;z_mean;z_max;vertex_indices (or CSV style depending on flag)

// New signature: last parameter 'alt_format' == 0 -> default (commas and dot decimal)
//                                   == 1 -> use semicolon column separator and comma decimal separator
void dumpFaceEquationsCSV(Model3D* model, const char* csv_filename, int alt_format) {
    if (model == NULL || csv_filename == NULL) return;
    FILE* f = fopen(csv_filename, "w");
    if (!f) {
        printf("Error: cannot open '%s' for writing\n", csv_filename);
        return;
    }
    FaceArrays3D* faces = &model->faces;
    int face_count = faces->face_count;

    char col_sep = alt_format ? ';' : ',';
    int use_comma_decimal = alt_format ? 1 : 0;

    // Determine maximum number of vertices in a face so we can create fixed per-vertex columns
    int max_v = 0;
    for (int ii = 0; ii < face_count; ++ii) {
        if (faces->vertex_count[ii] > max_v) max_v = faces->vertex_count[ii];
    }

    // Header: base columns + per-vertex groups (vN_idx,vN_xo,vN_yo,vN_zo)
    // Use selected column separator
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

    for (int i = 0; i < face_count; ++i) {
        float a = (float)FIXED64_TO_FLOAT(faces->plane_a[i]);
        float b = (float)FIXED64_TO_FLOAT(faces->plane_b[i]);
        float c = (float)FIXED64_TO_FLOAT(faces->plane_c[i]);
        float d = (float)FIXED64_TO_FLOAT(faces->plane_d[i]);
        float zmin = FIXED_TO_FLOAT(faces->z_min[i]);
        float zmean = FIXED_TO_FLOAT(faces->z_mean[i]);
        float zmax = FIXED_TO_FLOAT(faces->z_max[i]);

        char buf[64];
        // face index
        fprintf(f, "%d", i);
        fprintf(f, "%c", col_sep);

        // Helper: print float value respecting decimal separator
        #define PRINTF_FLOAT(val) do { snprintf(buf, sizeof(buf), "%.6f", (double)(val)); if (use_comma_decimal) { for (char *_p = buf; *_p; ++_p) if (*_p == '.') *_p = ','; } fprintf(f, "%s", buf); } while(0)

        PRINTF_FLOAT(a); fprintf(f, "%c", col_sep);
        PRINTF_FLOAT(b); fprintf(f, "%c", col_sep);
        PRINTF_FLOAT(c); fprintf(f, "%c", col_sep);
        PRINTF_FLOAT(d); fprintf(f, "%c", col_sep);
        PRINTF_FLOAT(zmin); fprintf(f, "%c", col_sep);
        PRINTF_FLOAT(zmean); fprintf(f, "%c", col_sep);
        PRINTF_FLOAT(zmax);

        // vertex_indices as quoted field (space-separated)
        fprintf(f, "%c\"", col_sep);
        int offset = faces->vertex_indices_ptr[i];
        int n = faces->vertex_count[i];
        for (int j = 0; j < n; ++j) {
            if (j) fputc(' ', f);
            int vid = faces->vertex_indices_buffer[offset + j]; // keep OBJ 1-based index
            fprintf(f, "%d", vid);
        }
        fprintf(f, "\""); // close indices field

        // Then print fixed per-vertex columns up to max_v: idx,xo,yo,zo (empty if face has fewer vertices)
        for (int k = 0; k < max_v; ++k) {
            if (k < n) {
                int vid = faces->vertex_indices_buffer[offset + k]; // OBJ 1-based
                int vidx = vid - 1;
                if (vidx >= 0) {
                    float xo = FIXED_TO_FLOAT(model->vertices.xo[vidx]);
                    float yo = FIXED_TO_FLOAT(model->vertices.yo[vidx]);
                    float zo = FIXED_TO_FLOAT(model->vertices.zo[vidx]);
                    // idx
                    fprintf(f, "%c%d", col_sep, vid);
                    // xo, yo, zo
                    snprintf(buf, sizeof(buf), "%.6f", (double)xo); if (use_comma_decimal) for (char *_p = buf; *_p; ++_p) if (*_p == '.') *_p = ','; fprintf(f, "%c%s", col_sep, buf);
                    snprintf(buf, sizeof(buf), "%.6f", (double)yo); if (use_comma_decimal) for (char *_p = buf; *_p; ++_p) if (*_p == '.') *_p = ','; fprintf(f, "%c%s", col_sep, buf);
                    snprintf(buf, sizeof(buf), "%.6f", (double)zo); if (use_comma_decimal) for (char *_p = buf; *_p; ++_p) if (*_p == '.') *_p = ','; fprintf(f, "%c%s", col_sep, buf);
                } else {
                    // index present but coordinates missing
                    fprintf(f, "%c%d%c%c%c", col_sep, vid, col_sep, col_sep, col_sep);
                }
            } else {
                // no vertex: write four empty CSV fields
                fprintf(f, "%c%c%c%c", col_sep, col_sep, col_sep, col_sep);
            }
        }
        fprintf(f, "\n");
    }
    fclose(f);
    printf("Wrote face equations to %s (%d faces)\n", csv_filename, face_count);
}

// Dump 2D coordinates of projected face vertices into a text file formatted by face.
// Each face entry has the form:
// face <i> (<n> verts):
//   <vertex_idx>: <x2d> <y2d>
//   ...
static void dumpFace2DCoordinates(Model3D* model, const char* txt_filename) {
    if (!model || !txt_filename) return;
    FILE* f = fopen(txt_filename, "w");
    if (!f) {
        printf("Error: cannot open '%s' for writing\n", txt_filename);
        return;
    }

    FaceArrays3D* faces = &model->faces;
    VertexArrays3D* vtx = &model->vertices;
    int face_count = faces->face_count;

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

// Dump sorted face indices in the order used by the painter (sorted_face_indices array)
// Each line contains a face index (0-based) in draw order.
static void dumpSortedFaceIndices(Model3D* model, const char* txt_filename) {
    if (!model || !txt_filename) return;
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

segment "utilities";
// ============================================================================
//  7. 3D MODEL MANAGEMENT, UTILITIES & SYSTEM HELPERS
// ============================================================================

/* show_help_pager
 * ----------------
 * Display the interactive help menu in pages limited to 20 lines each.
 * The header is repeated at the top of every page. The user can press
 * SPACE to continue to the next page or 'Q'/'q' to quit the help early.
 */
static void show_help_pager(void) {
    const char* header[] = {
        "===================================",
        "    HELP - Keyboard Controller",
        "===================================",
        "" /* blank line */
    };
    const int header_count = sizeof(header)/sizeof(header[0]);

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
        "2: Painter = BUBBLE SORT (Fixed32/64)",
        "=: Painter = NEWELL SANCHA (full tests, Fixed32/64)",
        "3: Painter = GEOV2 (geometry-only). Can be slow for large models",
        "4: Painter = CORRECT (painter_correct)",
        "5: Painter = CORRECTV2",
        "U: Painter = GEOV3. Can be slow for large models",
        "O: Render scanline Z-Buffer (alternative to painter algorithm)",
        "6: Both colors RANDOM mode",
        "7: Choose fill color",
        "8: Choose frame color",
        "9: Reset colors, palette 0, and shading OFF",
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

    const int max_lines = 20; // page height limit
    int content_per_page = max_lines - header_count;
    if (content_per_page <= 0) content_per_page = 1;

    int pos = 0;
    while (pos < n) {
        // Print header
        for (int h = 0; h < header_count; ++h) printf("%s\n", header[h]);
        // Print a page of content
        int end = pos + content_per_page;
        for (int i = pos; i < end && i < n; ++i) printf("%s\n", lines[i]);

        // Prompt for next action on every page.
        printf("\nPress ESC to quit, any other key to continue...\n");
        char key = getkeypress(); 

        if (key == 27) return; // ESC quits

        // Any other key advances page. If on final page, wrap to first page.
        if (end >= n) {
            pos = 0;
        } else {
            pos = end;
        }

        // clear a separating line between pages
        DoText();
        printf("\n");
    }
}

void DoText() {
        shroff();
        putchar((char) 12); // Clear screen    
}
void setProDOSFileType(const char* filename, int fileType, long auxType) {
    FileInfoRecGS pb;
    GSString255 gsName;

    gsName.length = strlen(filename);
    strncpy((char*)gsName.text, filename, gsName.length);

    pb.pCount = 5;              // verify against your GSOS.h - number of params for this call
    pb.pathname = &gsName;
    pb.access = 0xC3;           // standard full-access default, unchanged
    pb.fileType = fileType;     // e.g. 0xC1
    pb.auxType = auxType;       // e.g. 0x0000
    pb.storageType = 0;         // 0 = leave unchanged (per GS/OS SetFileInfo semantics)
    pb.optionList = 0;

    SetFileInfoGS(&pb);
}

#define SHR_BASE        0xE12000UL
#define SHR_SIZE        32768UL
void saveSHRAsRawPic(const char *filename)
{
    FILE *out;
    unsigned char *shr;
    unsigned long i;

    /*
     * Actual SHR memory address:
     * $E1:2000 = $E12000
     * The 32-bit pointer cast allows ORCA/C
     * to use long addressing.
     */
    shr = (volatile unsigned char *)SHR_BASE;
    out = fopen(filename, "wb");
    if (out == NULL) {
        printf("Error: unable to open %s for writing\n", filename);
        return;
    }

    //  Copy  $E12000 -> $E19FFF
    for (i = 0; i < SHR_SIZE; i++) {
        fputc(shr[i], out);
    }
    fclose(out);

    // PIC, not compressed
    setProDOSFileType(filename, 0xC1, 0x0000);
    printf("%s saved.\n", filename);
}

// --- Generic screenshot entry point, callable after ANY renderer ---
void saveNextScreenshot(void) {
    char fname[16];
    int idx;
    FILE* test;
    for (idx = 0; idx < 1000; idx++) {
        sprintf(fname, "screen%03d.PIC", idx);
        test = fopen(fname, "rb");
        if (test == NULL) {
            break; // this index is free
        }
        fclose(test);
    }
    if (idx >= 1000) {
        printf("Error: no available screen index (000-999 all used)\n");
        return;
    }
    saveSHRAsRawPic(fname);
}

// // Helper macro to swap face indices in the sorted_face_indices array
// // (We swap indices, not the faces themselves, to keep the buffer intact)
// #define SWAP_FACE(faces, i, j) \
//     do { \
//         int temp_idx = faces->sorted_face_indices[i]; \
//         faces->sorted_face_indices[i] = faces->sorted_face_indices[j]; \
//         faces->sorted_face_indices[j] = temp_idx; \
//     } while (0)
