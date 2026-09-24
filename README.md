# 3D OBJ Explorer for Apple IIGS

**A Tribute to Robert DONY**  
*Author of "Calcul des parties cachées", Masson, 1986*

A 3D model viewer and explorer implementing multiple painter's algorithms, specifically designed for the Apple IIGS computer using ORCA/C and the QuickDraw graphics API.

![Main Interface](Screenshots/main_interface.png)
![Cross Subdivision Shaded](Screenshots/crossSub_shaded.png)
![Charpente Shaded](Screenshots/charpente_shaded.png)
![Monkey](Screenshots/monkey.png)

## Overview

3D Explorer is an interactive 3D rendering application that reads simplified Wavefront OBJ files and displays them with full manipulation capabilities. The project demonstrates advanced rendering techniques on resource-constrained hardware (Apple IIGS with 2.8 MHz 65C816 CPU) through careful optimization and multiple algorithm implementations. It also performs satisfactorily on modern computers and in emulation, despite the large number of calculations required.

It is built around the painter's algorithm: faces are sorted and drawn back-to-front so that nearer polygons naturally occlude farther ones without a Z-buffer. The application includes several painter variants, from fast Z-mean ordering to more robust geometric correction passes for overlapping and intersecting faces.

### Why use this explorer?

- Optimized for Apple IIGS hardware and demonstrates fixed-point 3D rendering techniques
- Includes advanced painter algorithms for difficult overlapping geometry
- Provides an experimental scanline Z-buffer renderer
- Offers interactive inspection and debugging tools for face order, overlap, and visibility issues

### Known Limitations

- Speed: requires an accelerator or emulator for comfortable use
- Limits on number of vertices/faces and vertices per face
- Very limited handling of intersecting faces
- Cyclic overlap is correctly handled only by the scanline Z-buffer (key `O`)

Example of cyclic overlap that painters cannot resolve correctly:

![Cyclic Overlap](Screenshots/cyclic_overlap.png)

The Z-buffer scanline mode (triggered by `O`) handles this case correctly.

## Rendering Modes Summary

| Key | Mode          | Description                                      | Speed     | Robustness | Notes                          |
|-----|---------------|--------------------------------------------------|-----------|------------|--------------------------------|
| `1` | FAST          | Simple Z-mean sorting + bounding-box tests       | Fastest   | Low        | Best for simple geometry       |
| `2` | BUBBLE SORT   | Full pairwise bubble-sort ordering               | Medium    | Medium     | Fixed32/64 arithmetic          |
| `3` | NEWELL SANCHA | Full Newell-Sancha algorithm                     | Medium    | High       | Most robust classic painter    |
| `4` | GEO           | Geometry-only heuristics (plane tests)           | Slow      | Medium     | Avoid on large models          |
| `5` | GEO V3        | Inspired by R. Dony (no face splitting)          | Slow      | Medium     | Avoid on large models          |
| `6` | CORRECT       | NEWELL + local reordering (homebrew)             | Slow      | High       | Good for inconclusive pairs    |
| `7` | CORRECT V2    | Experimental local correction V2                 | Slow      | High       | Best when culling is off       |
| `O` | Z-BUFFER      | Experimental scanline Z-buffer (prototype)       | Very slow | Highest    | Handles interpenetration       |

## Getting Started

1. **Load a model**  
   Run the program and enter the path to a simplified OBJ file.  
   After entering the path, press `Enter` five times to accept the default values for distance, horizontal angle, vertical angle, screen rotation, and any additional prompts. These can still be changed later.

2. The 3D object appears centred on the screen.

3. **Basic controls**
   - Navigate with the keyboard
   - Switch rendering modes with keys `1`–`7`
   - Press `G` to cycle color palettes
   - Press `>` / `<` to choose fill / frame colors
   - Press `!` to toggle orientation-based shading
   - Press `H` for the full help screen
   - Press `;` or `.` for face-order repair helpers

## Keyboard Controls

### Camera & View

| Key              | Action                  | Description                                      |
|------------------|-------------------------|--------------------------------------------------|
| `A` / `Z`        | Distance                | Move camera closer / farther                     |
| `Left` / `Right` | Horizontal rotation     | Rotate around vertical axis                      |
| `Up` / `Down`    | Vertical rotation       | Rotate around horizontal axis                    |
| `W` / `X`        | Screen rotation         | Rotate around screen Z-axis                      |
| `+` / `-`        | Projection scale        | Increase / decrease perspective (±10 %)          |
| `K`              | Edit parameters         | Manually enter distance and angles               |
| `E` / `R`        | Pan left / right        | 2D screen offset (10 pixels)                     |
| `T` / `Y`        | Pan up / down           | 2D screen offset (10 pixels)                     |
| `0`              | Reset pan               | Return to center (0, 0)                          |
| `B`              | Back-face culling       | Toggle observer-space culling                    |
| `P`              | Wireframe               | Toggle filled / frame-only polygons              |
| `J`              | Jitter                  | Toggle random per-vertex 2D offset (0–10 px)     |
| `C`              | Palette overlay         | Show / hide color palette                        |
| `G`              | Cycle palette           | Cycle through the 16 SHR palettes                |
| `!`              | Orientation shading     | Toggle orientation-based flat shading            |

### Rendering Modes

| Key | Mode          |
|-----|---------------|
| `1` | FAST          |
| `2` | BUBBLE SORT   |
| `3` | NEWELL SANCHA |
| `4` | GEO           |
| `5` | GEO V3        |
| `6` | CORRECT       |
| `7` | CORRECT V2    |
| `O` | Z-BUFFER (experimental) |

### Color Management

| Key | Action                     | Description                                                                 |
|-----|----------------------------|-----------------------------------------------------------------------------|
| `8` | Random colors (quick)      | Set both fill and frame to random (new colors each press)                   |
| `>` | Choose fill color          | Interactive color chooser (0–15 or random)                                  |
| `<` | Choose frame color         | Interactive color chooser (0–15, random, or “same as fill”)                 |
| `9` | Reset colors               | Restore defaults (fill=14, frame=7), palette 0, disable orientation shading |

**Interactive color chooser (`>` / `<`)**  
- Displays the 16 colors of the current palette + “Random” (and “Same as fill” for frame)  
- `Up` / `Down`: change palette  
- `Left` / `Right`: move selection  
- Any other key: confirm and close

![Fill Color](Screenshots/fill.png)
![Frame Color](Screenshots/frame.png)
![Random Colors](Screenshots/random_colors.png)

### Diagnostic & Inspection Tools

| Key | Action                  | Description                                                                 |
|-----|-------------------------|-----------------------------------------------------------------------------|
| `V` | Inspect single face     | Navigate faces, view details, reverse winding, hide/restore                 |
| `Q` | Inspect face pair       | Diagnose ordering anomalies between two faces                               |
| `D` | Inspect before          | Analyze faces before selected face in sorted order                          |
| `S` | Inspect after           | Analyze faces after selected face in sorted order                           |
| `M` | Debug pair plane        | Interactive `pair_plane_before` diagnostics                                 |
| `;` | Repair face order       | Run `check_sort_repair`                                                     |
| `.` | Fast repair             | Run `check_sort_repair_fast` (QuickDraw-centroid based)                     |
| `I` | Toggle inconclusive     | Show / hide inconclusive face pairs                                         |
| `L` | Face ID labels          | Display face numbers on polygons                                            |
| `F` | Export debug data       | Dump `Faces3D.csv`, `Faces2D.txt`, `FacesOrder.txt`                         |

### Navigation & System

| Key     | Action            | Description                                              |
|---------|-------------------|----------------------------------------------------------|
| `Space` | Model info        | Display vertices, faces, camera params, performance      |
| `N`     | New model         | Load a different OBJ file (resets to FAST mode)          |
| `H`     | Help              | Show paginated keyboard reference                        |
| `*`     | Save screenshot   | Save current SHR screen as `screenNNN.PIC`               |
| `ESC`   | Quit              | Exit application                                         |

## Detailed Mode Descriptions

### FAST (Key `1`)
Simple face sorting by Z-mean with bounding-box overlap tests.  
Highest performance. May produce artifacts on complex overlapping polygons.

### BUBBLE SORT (Key `2`)
Full pairwise-comparison sort using a bubble-sort ordering pass.  
Uses Fixed32 (16.16) / Fixed64 (32.32) arithmetic. More thorough than FAST but without the full geometric test battery of NEWELL SANCHA.

### NEWELL SANCHA (Key `3`)
Full Newell-Sancha pairwise comparison algorithm (`painter_newell_sancha`).

Comprehensive geometric tests per face pair:
1. Z-extents overlap
2. X-extents overlap
3. Y-extents overlap
4. Projected polygon overlap
5. All vertices of f2 on observer’s side of f1’s plane
6. All vertices of f1 on opposite side of f2’s plane
7. All vertices of f2 on opposite side of f1’s plane (requires swap)
8. All vertices of f1 on observer’s side of f2’s plane (requires swap)

Most robust classic painter implementation. Uses Fixed32/64 arithmetic throughout.

### GEO (Key `4`) — ⚠️ Can be very slow on large models
Geometry-only mode using plane-based ordering heuristics and ray-casting (`painter_geoV2`).  
Recommended only for small models or diagnostic purposes.

### GEO V3 (Key `5`) — ⚠️ Can be very slow on large models
Geometry-only mode inspired by R. Dony’s book (`painter_geoV3`), without face-splitting handling.  
Same performance caveats as GEO.

### CORRECT (Key `6`)
Extends NEWELL SANCHA with local face reordering after separating faces into FRONT and BACK groups.  
Useful for geometries with many inconclusive pairs. Slower.

### CORRECT V2 (Key `7`)
Experimental local correction (`painter_correctV2`).  
Separates faces into FRONT/BACK groups before applying local corrections. Improves robustness when back-face culling is off. Uses geometric plane tests with Z-mean as deterministic fallback.

### Scanline Z-Buffer (Key `O`)
Completely separate from the painter pipeline.  
Rasterizes each face scanline-by-scanline with per-pixel depth resolution using a 320-entry buffer of interpolated `1/z` (perspective-correct).  

- Correctly handles interpenetrating and cyclically overlapping faces  
- Supports concave faces (even-odd fill)  
- Borders are drawn as a free byproduct of scan conversion  
- Uses a hand-written 65816 `drawPixel` routine for performance  
- **Status**: experimental/prototype — significantly slower than any painter mode

## Technical Architecture

### Graphics Pipeline

1. **Model Loading** — Parse simplified OBJ files (`v` and `f` only)
2. **3D Transformation** — World → observer space
3. **Projection** — Perspective projection to 2D screen space
4. **Face Sorting** — Selected painter algorithm
5. **Rasterization** — QuickDraw filled polygons (or custom scanline for Z-buffer)

### Mathematical Implementation

**Fixed-point arithmetic**
```c
#define FIXED_SHIFT 16
#define FIXED_SCALE (1L << FIXED_SHIFT)   // 65536
typedef long     Fixed32;                 // 16.16
typedef int64_t  Fixed64;                 // 32.32
