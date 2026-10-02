# Call Flow (start at `main`) ✅

> Visual style:
> - **Function** names are bolded and use `code` formatting
> - *Purpose* and short description follow on the same line
> - Subcalls are indented and prefixed with `→` for readability

---

## Main flow

### Startup / Load

- 🔧 **`main()`** — Entry point
  - → 🔧 **`createModel3D()`** — allocate arrays, init metadata (auto-scale / backup pointers)
  - → 🔧 **`loadModel3D(model, filename)`** — orchestrates model loading
    - → 🔧 **`readVertices()`** — read `v x y z` into Fixed arrays
    - → 🔧 **`readFaces_model()`** — parse faces into packed index buffers

### Parameter parsing / Auto-fit

- 🔧 **`getObserverParams(&params, model)`** — interactive parameter parsing
  - reads angles H/V/W and screen rotation
  - If user presses ENTER for distance: a precomputed auto-fit suggestion may be applied if available; on-the-fly auto-fit is disabled.
  - else: set `params->distance` from user input

---

## Main render loop

- Enter main render loop (`bigloop`):
  - 🔧 **`processModelFast(model, &params, filename)`** — runs every frame; ultra-fast transformation + projection
    - precompute trig products (Fixed32)
    - For each vertex (tight Fixed32 loop): transform → compute `xo/yo/zo` → project to `x2d/y2d`
    - **Note:** runtime auto-fit is *not* applied inside `processModelFast` — any autoscale must be applied ahead of time; the auto-fit helper has been removed and archived to `chutier.txt`. `processModelFast` operates on already-scaled model vertices.
    - 🔧 **`calculateFaceDepths()`** — compute per-face `z_min/z_max/z_mean`, display flags, planar coefficients (Newell). Optionally performs observer-space back-face culling (plane D <= 0) when the `B` toggle is enabled.
    - 🔧 **`painter_newell_sancha()`** — sort faces by depth and correct ambiguous order (qsort + corrections). When back-face culling is enabled, the painter builds and sorts a list limited to faces with `display_flag == 1` (visible faces), performs order corrections only on that sub-list for efficiency and correctness, and appends culled faces afterward to preserve `sorted_face_indices` stability.
      - Collects *inconclusive pairs* (pairs of faces where order is ambiguous) into an **in-memory buffer** for later inspection; note: the buffer `inconclusive_pairs` is now a global buffer (preallocated for performance) used by diagnostic and framing helpers.

    - ⚠️ **Wireframe mode** — handled via the `framePolyOnly` flag and `drawPolygons()` (no separate `processModelWireframe()` function in the current codebase). Use `framePolyOnly=1` to render wireframe previews.

- 🔧 **`drawPolygons(model, faces, face_count, vert_count)`** — render loop
  - uses sorted faces; for each face builds QuickDraw polygon from `vtx->x2d/y2d` (these already reflect any model-space auto-scaling applied ahead of time; auto-fit is disabled — archived implementation is in `chutier.txt`). When back-face culling is active, `faces->sorted_face_indices` contains visible faces first (sorted) followed by culled faces; `drawPolygons` still checks `display_flag` and skips any face with `display_flag == 0` at draw time.
  - Fill + Frame polygon (QuickDraw)
  - There is also a helper `frameInconclusivePairs()` that can frame (in white) polygons listed in the `inconclusive_pairs` buffer for debugging/diagnostic display.

### UI / Input handling

- `startgraph()` / render / `endgraph()` / `DoText()` / optional `DoColor()`
- Keys: Space (info), N (new model), Arrows / A Z (angles/distance), `K` (edit angles/distance without reloading model), `+`/`-` (adjust autoscale), `B` (toggle back-face culling: observer-space d<=0 test)
  - **New keys (recent additions):**
    - `D` / `d`: Inspect faces placed BEFORE a selected face in the painter order and report misplaced faces (previews in orange)
    - `S` / `s`: Inspect faces placed AFTER a selected face that should be BEFORE it (previews in pink)
    - `Q` / `q`: Interactive face-pair inspector — compare two faces and display ordering diagnostics
    - `M` / `m`: Interactive `pair_plane_before` debugger — prompts for two face IDs separately and reports detailed plane/vertex-sidedness diagnostics
    - `L` / `l`: Label mode — show the model with each face's ID drawn at its polygon center
- `K` invokes `getObserverParams(&params, model)` interactively and applies new angles/distance without requiring a reload.
- `+`/`-` behavior: if the model is not yet auto-scaled, these keys previously performed an **auto-fit** via `fitModelToView()`; auto-fit has been disabled (see `chutier.txt` for the archived implementation). They now only increase or decrease the current `params->distance` and update `model->auto_scale`. Automatic recomputation via bounding-sphere is disabled; distance adjustments are manual. The action prints a short message (e.g., "Distance increased" / "Distance decreased").

---

## Supporting / fallback functions (short purpose)


- Bounding-sphere metric removed; auto-fit uses bbox heuristic (O(n) at load remains but is simpler)

- 🔧 **Non-destructive backup support removed** — per-vertex backup API was removed to simplify flow
- 🔧 **Wireframe mode** — implemented via the `framePolyOnly` flag and `drawPolygons()` (no separate `processModelWireframe()` function)
- 🔧 **`destroyModel3D(Model3D* model)`** — frees all memory allocated by `createModel3D()`; must be called to avoid leaks
- 🔧 **`readVertices()` / `readFaces_model()`** — file parsing helpers
- 🔧 **`frameInconclusivePairs(Model3D* model)`** — utility: frames in white all polygons currently recorded in the global `inconclusive_pairs` buffer (diagnostic; no runtime side effects beyond rendering) 
- 🔧 **`projected_polygons_overlap(Model3D* model, int f1, int f2)`** — screen-space test that returns 1 if two faces' projected 2D polygons *overlap* (proper edge intersection or containment), **0 if disjoint**. Important: *touching-only* cases (shared edge or single-vertex contact) are considered **NON-overlap** and return 0. The algorithm uses integer segment intersection (proper intersection only) then ray-casting containment; points on edges are treated as outside.
- 🔧 `void inspect_faces_before(Model3D* model, ObserverParams* params, const char* filename)` — `GS3Dp.cc:2032` — interactive wrapper bound to `D`/`d`: prints a compact per-face diagnostic and offers a wireframe preview that highlights faces placed BEFORE a selected face (misplaced faces highlighted).
- 🔧 `void inspect_faces_after(Model3D* model, ObserverParams* params, const char* filename)` — `GS3Dp.cc:2219` — interactive wrapper bound to `S`/`s`: prints faces placed AFTER a selected face that should be BEFORE it and offers a wireframe preview with highlights.
- 🔧 **`inspect_polygons_overlap`** — archived interactive wrapper removed from the active build and retained only in `chutier.txt`; the active code uses `projected_polygons_overlap()` internally for overlap checks.
- 🔧 `void display_model_face_ids(Model3D* model, ObserverParams* params, const char* filename)` — `GS3Dp.cc:2449` — label mode bound to `L`/`l`: draws the model in wireframe and overlays each face's ID centered on that face (uses `drawFace(..., show_index=1)`), useful for debugging face ordering and references.

**Notes:** Automatic distance estimation has been disabled; distance adjustments are manual.

---

## Notes

- This Markdown preserves the visual grouping: **function name** + **purpose/action**, and uses `→` to show call flow.
- If you want more detail (e.g., exact file:line references or prototypes), I can append them in a table or add a per-function section with signatures.

---

## Function signatures & detailed references

Below are per-function entries expanded with: signature, file:line (in this tree: `DONYGS.cc`), brief purpose, and direct dependencies / callers to make navigation easier.

---

### Core / Public APIs
- **`Model3D* createModel3D(void)`** — `DONYGS.cc:2540` 🔧
  - Purpose: allocate and initialize `Model3D` structure and its internal buffers.
  - Calls: internal alloc helpers; called by `main()` and `loadModel3D()` on new model creation.
  - Notes: returns NULL on allocation failure.

- **`void destroyModel3D(Model3D* model)`** — `DONYGS.cc:2860` 🔧
  - Purpose: free all memory owned by the model (vertices, faces, buffers) and clear handles.
  - Called by: `main()` (on exit or failures) and on reloads.

- **`int loadModel3D(Model3D* model, const char* filename)`** — `DONYGS.cc:2920` 🔧
  - Purpose: orchestrates model loading pipeline: vertices then faces; updates counters and model metadata.
  - Calls: `readVertices()` (`DONYGS.cc:3220`) and `readFaces_model()` (`DONYGS.cc:3351`).
  - Returns: 0 on success, -1 on fatal errors.

- **`int readVertices(const char* filename, VertexArrays3D* vtx, int max_vertices, Model3D* owner)`** — `DONYGS.cc:3220` 🔧
  - Purpose: parse OBJ `v x y z` lines into fixed-point `x/y/z` arrays.
  - Notes: bounds checks against `max_vertices`, uses `fgets()`/`sscanf()`.

- **`int readFaces_model(const char* filename, Model3D* model)`** — `DONYGS.cc:3351` 🔧
  - Purpose: parse OBJ `f` lines into packed face index buffers and pointer table.
  - Notes: builds `vertex_indices_buffer` and `vertex_indices_ptr` with space-efficient packing.

---

### Rendering / Painter (core pipeline)
- **`void processModelFast(Model3D* model, ObserverParams* params, const char* filename)`** — `DONYGS.cc:3077` 🔧
  - Purpose: per-frame transformation & projection (Fixed32 fast loop), then face depth computation and painter dispatch.
  - Calls: `calculateFaceDepths()` (`DONYGS.cc:3517`), then one of the painters (`painter_newell_sancha_fast`, `painter_newell_sancha`, `painter_newell_sancha_float`).
  - Notes: uses precomputed trig tables and full Fixed32 arithmetic for speed.

- **`void calculateFaceDepths(Model3D* model, Face3D* faces, int face_count)`** — `DONYGS.cc:3517` 🔧
  - Purpose: compute each face's z_min/z_max/z_mean, screen bbox (minx/maxx/miny/maxy) and plane coefficients (Newell method), set `display_flag` (behind-camera/culling).
  - Side-effect: writes plane coefficients (Fixed64) and face metrics used by painters and diagnostics.

- **`void painter_newell_sancha_fast(Model3D* model, int face_count)`** — `DONYGS.cc:732` 🔧
  - Purpose: FAST painter: stable sort by `z_mean` only (qsort) with bbox-based quick rejection — very fast but may leave ambiguities.
  - Notes: when `cull_back_faces` enabled, only visible faces are sorted and culled faces appended afterward.

- **`void painter_newell_sancha(Model3D* model, int face_count)`** — `DONYGS.cc:772` 🔧
  - Purpose: FIXED/robust painter: initial sort by z_mean + multiple bubble-like correction passes applying Tests 1..7 (depth/bbox/plane tests) to resolve ordering.
  - Calls/utilizes: `projected_polygons_overlap()` for strict 2D overlap test; records definitive `ordered_pairs` and pushes ambiguous pairs to the global `inconclusive_pairs` buffer for diagnostics.
  - Notes: heavy Fixed64 usage to avoid overflow and keep determinism.

- **`void painter_newell_sancha_float(Model3D* model, int face_count)`** — `DONYGS.cc:1325` 🔧
  - Purpose: Float-based painter that mirrors Windows numeric behavior; converts per-vertex observer coords to floats and runs similar painter logic with float math for performance and numeric parity on float-capable platforms.
  - Uses: reusable float buffers (see `ensure_vertex_capacity`, `ensure_face_capacity`) to avoid repeated alloc/free.

- **`void drawFace(Model3D* model, int face_id, int fillPenPat, int show_index)`** — `DONYGS.cc:3794` 🔧
  - Purpose: draw a single face (fill+frame) and optionally draw its index. Used by diagnostics and overlay operations.

- **`void drawPolygons(Model3D* model, int* vertex_count, int face_count, int vertex_count_total)`** — `DONYGS.cc:3913` 🔧
  - Purpose: main rasterization loop: iterate `faces->sorted_face_indices` and render polygons (Fill+Frame) using a persistent `globalPolyHandle` (QuickDraw GPI operations).
  - Checks: verifies index validity and performs bounding-box culling (screen space) before drawing.

- **`void frameInconclusivePairs(Model3D* model)`** — `DONYGS.cc:4055` 🔧
  - Purpose: diagnostic helper — draws a white frame around polygons recorded as ambiguous in `inconclusive_pairs` for visual inspection.

---

### Geometry helpers & inspectors
- **`static int projected_polygons_overlap(Model3D* model, int f1, int f2)`** — `DONYGS.cc:1850` 🔧
  - Purpose: robust screen-space overlap test between two faces' projected polygons. Returns `1` for proper overlap (positive-area intersection or containment), `0` for disjoint or touching-only.
  - Algorithm: AABB quick-reject → edge-vs-edge proper intersection (`segs_intersect_int`) → containment via `point_in_poly_int`.
  - Helpers: `segs_intersect_int()` (`DONYGS.cc:1784`), `point_in_poly_int()` (`DONYGS.cc:1797`), `orient_ll()`/`on_seg_ll()`.

- **`static int projected_polygons_overlap(Model3D* model, int f1, int f2)`** — `DONYGS.cc:2935` 🔧
  - Purpose: strict screen-space overlap test between two faces' projected polygons. Used by painter correction and inspection internals when evaluating whether faces actually overlap in 2D.

- **`void inspect_faces_before/after(...)`** — `DONYGS.cc:2032` / `DONYGS.cc:2219` 🔧
  - Purpose: interactive wrappers bound to `D`/`S` keys to report faces misplaced BEFORE/AFTER a selected face and optionally preview them.
  - Notes: rely on sorting helpers such as `prepare_inspector_sort()` (`DONYGS.cc:2040`).

- **`void display_model_face_ids(Model3D* model, ObserverParams* params, const char* filename)`** — `DONYGS.cc:2449` 🔧
  - Purpose: label mode (key `L`) — draw wireframe and overlay each face's ID at polygon center to aid debugging ordering.

---

### Utilities / I/O / UI
- **`void getObserverParams(ObserverParams* params, Model3D* model)`** — `DONYGS.cc:2971` 🔧
  - Purpose: interactive text input to set angles `H/V/W` and `distance` (ENTER triggers applying precomputed auto-fit suggestion if available).

- **`void dumpFaceEquationsCSV(Model3D* model, const char* csv_filename, int alt_format)`** — `DONYGS.cc:3671` 🔧
  - Purpose: export per-face plane coefficients and depth stats to CSV for offline analysis.

- **`void compute2DFromObserver(Model3D* model, int angle_w)`** — `DONYGS.cc:4155` 🔧
  - Purpose: minimal reprojection helper (applies `angle_w` rotation and `s_global_proj_scale_fixed` to produce `x2d/y2d` arrays).

- **`void DoColor()` / `void DoText()`** — `DONYGS.cc:4183, 4213` 🔧
  - Purpose: helpers to draw palette and clear screen / textual overlays.

- **Help & UI**
  - `show_help_pager()` — prints keyboard help in pages (bound to `H`).
  - Keyboard bindings: `1/2/3` painter modes, `B` back-face culling (observer-space D<=0), `P`/`p` frame-only, `+/ -` adjust projection scale (manual), `A/Z` adjust distance, `K` edit angles/distance (ENTER may apply auto-fit suggestion), `N` load new model, `E` export equations.

---

### Internal / performance & safety notes
- Fixed-point arithmetic: 16.16 Fixed32 and Fixed64 intermediates are used across transforms and plane computations (see macros: `FIXED_MUL_64`, `FIXED_DIV_64`, `FIXED_ROUND_TO_INT`).
- Trig tables: `sin_table` / `cos_table` (0..359) for extremely fast deg-based trig lookup.
- Memory re-use: global persistent `globalPolyHandle`, float painter reusable buffers (`ensure_*_capacity`) and `order_buf` to avoid per-frame allocations.

---

*This section augments the short signatures list with per-function descriptions and direct references to their location in `DONYGS.cc`. If you'd like, I can convert this into a table or append short code excerpts for the most critical functions.*

---

*Generated/updated from `DONYGS.cc` (scan performed) — lines are approximate and reflect the current file version.*
