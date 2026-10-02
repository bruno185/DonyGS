# CLAUDE.md — 3D OBJ Explorer for Apple IIGS

Context for AI assistants working on this repository. Read this first, then
`README.md` for the user-facing feature and keyboard reference (it is accurate
and current — do not duplicate it here).

## Working language

Bruno writes in **French** — reply in French. Code, comments, identifiers,
commit messages and this file stay in **English**, matching the existing repo.

## What this is

An interactive 3D model viewer for the **Apple IIGS** (2.8 MHz 65C816), written
in **ORCA/C** against the **QuickDraw** API. It reads simplified Wavefront OBJ
files and renders them with several **painter's algorithm** variants — no
Z-buffer in the main pipeline. A tribute to Robert DONY, *"Calcul des parties
cachées"* (Masson, 1986).

The platform is the whole point: every design decision trades generality for
speed and memory on 1986 hardware.

## Repository map

| File | Role |
|---|---|
| `OBJExplorer.cc` | **The entire application** — 9 677 lines, ~490 KB |
| `README.md` | Feature reference, keyboard map, algorithm descriptions — current |
| `sf.c` | Standalone helper (305 lines) |
| `asm.h` | 65816 inline-assembly declarations (keyboard polling) |
| `DEPLOY.py` | Copies the linked binary onto the disk image |
| `OBJ.po` | ProDOS disk image (32 MB) for emulator / real hardware |
| `OBJExplorer` | Build artifact from `iix link` |
| `chutier.txt` | **Scrapyard** — archived code, incl. the retired FLOAT painter. Not compiled. |
| `call_flow.md` | ⚠️ **OBSOLETE — do not read or trust it.** |

## Reading `OBJExplorer.cc` — do not read it whole

At 9 677 lines it is ~125 000 tokens. Reading it entirely is slow, and on this
project it is billed against a small prepaid credit balance. **Use `Grep` to
locate a symbol, then `Read` with `offset`/`limit` around it.** The map below
gives you the offsets directly.

| Lines | Area |
|---|---|
| 59–72 | Includes, `#pragma memorymodel 1` |
| 89–300 | Color constants, `PAINTER_MODE_*`, palette base address |
| 278–430 | Fixed-point typedefs, macros, trig helpers |
| 470–668 | **Core data structures** (see below) |
| 802–1372 | Painters: `painter_newell_sancha_fast` (876), `painter_geoV1` (1005), `painter_geoV2` (1274), `painter_newell_sancha` (1372) |
| 1598–2254 | `painter_correct` (1647), `painter_correctV2` (1891) |
| 2336–2710 | `check_sort_repair` (2336), `check_sort_repair_fast` (2581) |
| 2710–3860 | 2D geometry primitives: segment intersection, point-in-poly, `projected_polygons_overlap` (3078), intersection centroid/bbox |
| 3861–4712 | Pair ordering tests: `evaluate_pair_tests` (3861), `pair_plane_after` (3992), `pair_plane_geometric_tests` (4247), `pair_plane_before` (4643) |
| 4713–4950 | Ray casting: `ray_cast` (4713), `ray_cast_distances` (4743), `ray_cast_hierarchical` (4777) |
| 4966–5533 | Diagnostics UI: `compare_faces_diagnostic` (4966), `inspect_face_pair_ui` (5130) |
| 5534–5619 | Face manipulation: `reverseFaceVertexOrder` (5534), `hideFace` (5589), `restoreFace` (5598), `restoreAllFaces` (5609) |
| 5620–6552 | Inspectors: `showFace` (5620), `inspect_faces_before` (5963), `inspect_faces_after` (6233), `display_model_face_ids` (6462) |
| 6553–7066 | Model lifecycle: `createModel3D` (6553), `destroyModel3D` (6903), `compute_face_bboxes3D` (6975), `loadModel3D` (7026) |
| 7067–7149 | `compute_face_plane_float` (7067), `compute_face_plane` (7104) |
| 7150–8100 | Intersection detection & face splitting: `split_face_by_plane` (7672), `check_intersect` (7924) |
| 8104–8381 | `getObserverParams` (8104), `processModelFast` (8210) |
| 8382–8724 | OBJ parsing: `readVertices` (8382), `readFaces_model` (8559), `normalizeAutoFitDistanceTo150` (8520) |
| 8725–8896 | `calculateFaceDepths` (8725), `computeOrientationShading` (8868) |
| 8897–9075 | Debug exports: `dumpFaceEquationsCSV`, `dumpFace2DCoordinates`, `dumpSortedFaceIndices` |
| 9076–9586 | Rendering: `drawFace` (9076), `drawPolygons` (9251), `drawPolygons_jitter` (9446), `frameInconclusivePairs` (9490) |
| 9587–9780 | `updateFace2DBounds`, `compute2DFromObserver` (9626), `DoColor`, `DoText`, `show_help_pager` |
| 9781–9877 | Palette management: `SetColor`, `ReadPalette`, `applyPalette`, `initPalettes` |
| 9878+ | `renderModelScanlineZBuffer` — experimental, outside the painter pipeline |

Line numbers drift as the file is edited. Confirm with `Grep` before trusting them.

## Core data model

The model uses **parallel arrays**, not arrays of structs — deliberate, for
65816 addressing and memory locality. Do not "modernize" this.

- `VertexArrays3D` (470) — model, observer and 2D coordinates per vertex
- `FaceArrays3D` (502) — per-face data, including `plane_d` and `saved_vertex_count`
- `Model3D` (641) — owns both, plus sort order and camera state
- `Face3D` (557) — legacy//transitional struct; `calculateFaceDepths` still takes it
- `ObserverParams` (610) — distance, horizontal/vertical/screen angles, pan

## Invariants — get these wrong and rendering breaks silently

1. **Front/back is decided in observer space by the sign of `plane_d`**:
   `plane_d > 0` → FRONT, `plane_d <= 0` → BACK. It is *not* the OBJ winding
   order. Reversing a face's vertex order flips its normal and therefore its
   classification — `reverseFaceVertexOrder` must update `display_flag`
   accordingly.
2. **Hidden faces keep their geometry in `saved_vertex_count`.** `hideFace` sets
   the live vertex count to 0 but preserves the original count there, so
   `restoreFace` and the inspectors can still read the vertices. Any code that
   walks a face's vertices must use `saved_vertex_count` when the face is
   hidden — this was the source of several past bugs.
3. **Newell orientation uses *all* vertices**, not just the first three
   (fixed in `calculateFaceDepths`). Faces are not guaranteed planar.
4. **OBJ face indices are 1-based**; internal arrays are 0-based.
5. **Fixed-point everywhere in hot paths.** `Fixed32` = 16.16 (`long`),
   `Fixed64` = 32.32 (`long long`). Use the `FIXED_*` macros. `float`/`double`
   appear only in diagnostics and `compute_face_plane_float` — never introduce
   them into a painter or drawing path.

## Hard limits

```c
MAX_VERTICES      6000
MAX_FACES         6000
MAX_FACE_VERTICES 16
CENTRE_X / CENTRE_Y  160 / 100   // 320x200 screen
```

Capacity grows via `ensure_vertex_capacity()` / `ensure_face_capacity()` up to
those ceilings. Performance degrades badly above ~500 faces in NORMAL mode.

## Painter mode constants are non-contiguous

```c
PAINTER_MODE_FAST      0   // key 1
PAINTER_MODE_FIXED     1   // key 2  (NORMAL / Newell-Sancha)
PAINTER_MODE_CORRECT   3   // key 4
PAINTER_MODE_GEO       5   // key 3
PAINTER_MODE_CORRECTV2 6   // key 5
```

Keyboard digits, mode constants and function names do **not** line up. Never
infer one from another — check the dispatch site.

## Build & deploy

```bash
iix compile OBJExplorer.cc
iix link OBJExplorer
python DEPLOY.py          # copies the binary onto OBJ.po
```

Requires ORCA/C 2.2.1+ and Golden Gate. There is no test suite and no CI; the
only verification is running the binary in an emulator (AppleWin/GSplus) or on
hardware. `segs_intersect_unit_tests` (2909) is a self-check for the segment
intersection primitive.

Compile-time switches: `ENABLE_DEBUG_SAVE` (debug dumps, ~5 KB binary and ~2 %
painter speed), `PERFORMANCE_MODE` (suppresses `printf`).

## Conventions

- Filled-polygon drawing goes through **persistent locked QuickDraw handles** —
  reuse the existing buffers rather than allocating per frame.
- Debug/diagnostic output is written to files (`Faces3D.csv`, `Faces2D.txt`,
  `FacesOrder.txt`, `Face<ID>.txt`) and read back for analysis. Prefer extending
  that pattern over adding on-screen logging.
- Any `malloc` in `createModel3D` needs its matching `free` in `destroyModel3D`
  — they have drifted apart before.
- Commit messages: English, one line, describe the behavioural change and the
  function touched.

## Known discrepancies with the README

- README mentions `painter_geo`; the code has `painter_geoV1` and
  `painter_geoV2` (GEO mode uses V2).
- README lists a Z-buffer mode as a future enhancement, but
  `renderModelScanlineZBuffer` already exists as an experiment.
- README's "Quick start keys" list has duplicate entries for `6` and `8`.
- README's Development History stops at 2026-01-15; recent work (face hiding,
  normals display, face splitting) is only in `git log`.

When README and code disagree, **the code wins** — and mention the drift to
Bruno so the README can be corrected.
