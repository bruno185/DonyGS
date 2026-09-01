#!/usr/bin/env python3
"""Split ObjExplorer.cc into thematic modules included from a thin root file.

The split is purely textual: every line keeps its position in the concatenated
stream, so the compiled binary is unchanged. Cut points are chosen between
functions, preferring the ORCA `segment "..."` directives already present.

Usage:
    python tools/split_source.py --dry-run   # show the cut points only
    python tools/split_source.py --apply     # write src/ and rewrite the root
"""
import argparse
import os
import sys

SRC = "ObjExplorer.cc"
OUTDIR = "src"
SENTINEL = b"// ---- generated header above; original source follows ----"

# (output file, anchor line = first symbol of the module, description)
# The anchor is walked upwards over its leading comment block / segment
# directive, so the doc comment travels with the function it documents.
MODULES = [
    ("common.h",    1,     "Includes, pragmas, fixed-point macros, constants, structures, prototypes"),
    ("painters.cc", 803,   "Painter's algorithm variants + check_sort_repair"),
    ("geom2d.cc",   2544,  "2D primitives: segment intersection, point-in-poly, projected overlap"),
    ("pairtest.cc", 3695,  "Pairwise ordering tests + ray casting"),
    ("diagnos.cc",  4786,  "Face-pair comparison diagnostics and the inspector UI"),
    ("inspect.cc",  5368,  "Face manipulation (hide/restore/reverse) and face inspectors"),
    ("model.cc",    6387,  "Model3D lifecycle: create / destroy"),
    ("geom3d.cc",   6809,  "3D geometry: bboxes, plane equations, face splitting, intersections"),
    ("observer.cc", 7938,  "Observer parameters and the fast processing pipeline"),
    ("objread.cc",  8216,  "Wavefront OBJ parsing"),
    ("depths.cc",   8559,  "Face depth computation, orientation shading, debug dumps"),
    ("render.cc",   8910,  "Face and polygon drawing, 2D projection from observer"),
    ("ui.cc",       9519,  "Text mode and the help pager"),
    ("palette.cc",  9617,  "Palette read/apply, per-face colors, raw pixel plotting"),
    ("zbuffer.cc",  9805,  "Experimental scanline Z-buffer renderer"),
    ("screen.cc",   9962,  "ProDOS file typing and SHR screenshot saving"),
    ("main.cc",     10056, "Main program: event loop and keyboard dispatch"),
]

ROOT_TEMPLATE = """/*
 * ============================================================================
 * 3D OBJ Explorer for Apple IIGS
 * ============================================================================
 * This file is only a table of contents. The program is a SINGLE translation
 * unit: each module below is textually included, in the exact order it had
 * when the source was one file. That order matters -- statics and globals are
 * declared where they are first needed, so modules must not be reordered.
 *
 * Build (unchanged):
 *     iix compile ObjExplorer.cc
 *     iix link ObjExplorer
 *     python DEPLOY.py
 *
 * Do NOT compile the src/*.cc files separately: they are fragments, not
 * translation units.
 * ============================================================================
 */

{includes}
"""


def is_boundary_walkable(line):
    """True if `line` should be pulled up into the module that follows it."""
    s = line.strip()
    if s == b"":
        return True
    if s.startswith(b"//") or s.startswith(b"/*") or s.startswith(b"*"):
        return True
    if s.startswith(b"segment ") or s.startswith(b"#pragma"):
        return True
    return False


def adjust_anchor(lines, anchor):
    """Walk an anchor upwards over its leading comment / segment block.

    Stops as soon as the preceding line is code (a closing brace, a statement,
    a declaration), so no function body is ever cut in half.
    """
    i = anchor  # 1-based anchor -> lines[anchor-1]; look at lines[i-2] going up
    while i > 1 and is_boundary_walkable(lines[i - 2]):
        i -= 1
    # Do not swallow trailing blank lines that belong to the previous module.
    while i < anchor and lines[i - 1].strip() == b"":
        i += 1
    return i


def compute_ranges(lines):
    total = len(lines)
    starts = []
    for name, anchor, desc in MODULES:
        starts.append((name, 1 if anchor == 1 else adjust_anchor(lines, anchor), desc))
    ranges = []
    for idx, (name, start, desc) in enumerate(starts):
        end = starts[idx + 1][1] - 1 if idx + 1 < len(starts) else total
        ranges.append((name, start, end, desc))
    return ranges


def header_for(name, start, end, desc, eol):
    title = "OBJExplorer -- %s" % name
    body = [
        b"/*",
        b" * " + title.encode("latin-1"),
        b" * " + desc.encode("latin-1"),
        b" *",
        b" * Fragment of ObjExplorer.cc (was lines %d-%d). Included from the root" % (start, end),
        b" * file, not compiled on its own. The active ORCA `segment` on entry is",
        b" * whatever the previous include left set -- see the root file's order.",
        b" */",
        SENTINEL,
    ]
    return eol.join(body) + eol


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    if not (args.apply or args.dry_run):
        ap.error("choose --dry-run or --apply")

    raw = open(SRC, "rb").read()
    eol = b"\r\n" if raw.count(b"\r\n") > raw.count(b"\n") / 2 else b"\n"
    lines = raw.split(b"\n")
    # Keep the trailing empty element produced by a final newline out of the way.
    trailing_newline = raw.endswith(b"\n")
    if trailing_newline:
        lines = lines[:-1]
    print("source: %s, %d lines, EOL=%r" % (SRC, len(lines), eol))

    ranges = compute_ranges(lines)

    print("\n%-12s %6s %6s %6s" % ("file", "start", "end", "lines"))
    for name, start, end, desc in ranges:
        print("%-12s %6d %6d %6d" % (name, start, end, end - start + 1))

    print("\n--- cut points (last 2 lines of previous / first 3 of new) ---")
    for name, start, end, desc in ranges[1:]:
        print("\n>>> %s begins at %d" % (name, start))
        for n in range(max(1, start - 2), min(len(lines), start + 2) + 1):
            mark = "NEW>" if n >= start else "    "
            print("  %s %5d %s" % (mark, n, lines[n - 1].decode("latin-1")[:96]))

    # Contiguity / coverage check.
    covered = 0
    prev_end = 0
    for name, start, end, desc in ranges:
        assert start == prev_end + 1, "gap or overlap before %s" % name
        assert end >= start, "empty range for %s" % name
        prev_end = end
        covered += end - start + 1
    assert covered == len(lines), "coverage %d != %d" % (covered, len(lines))
    print("\ncoverage OK: %d lines, contiguous, no overlap" % covered)

    if args.dry_run:
        return 0

    if not os.path.isdir(OUTDIR):
        os.mkdir(OUTDIR)

    for name, start, end, desc in ranges:
        chunk = eol.join(lines[start - 1:end]) + eol
        out = os.path.join(OUTDIR, name)
        with open(out, "wb") as f:
            f.write(header_for(name, start, end, desc, eol))
            f.write(chunk)
        print("wrote %s (%d lines)" % (out, end - start + 1))

    # Verify: strip each generated header, concatenate, compare with the source.
    rebuilt = []
    for name, start, end, desc in ranges:
        data = open(os.path.join(OUTDIR, name), "rb").read()
        pos = data.index(SENTINEL) + len(SENTINEL)
        body = data[pos:]
        if body.startswith(b"\r\n"):
            body = body[2:]
        elif body.startswith(b"\n"):
            body = body[1:]
        rebuilt.append(body)
    rebuilt = b"".join(rebuilt)
    original = eol.join(lines) + eol
    if rebuilt == original:
        print("\nVERIFY OK: concatenation of src/* is byte-identical to the source")
    else:
        print("\nVERIFY FAILED: %d vs %d bytes" % (len(rebuilt), len(original)))
        for i in range(min(len(rebuilt), len(original))):
            if rebuilt[i] != original[i]:
                print("first difference at byte %d" % i)
                print("  rebuilt : %r" % rebuilt[max(0, i - 60):i + 60])
                print("  original: %r" % original[max(0, i - 60):i + 60])
                break
        return 1

    includes = eol.join(
        b'#include "%s/%s"' % (OUTDIR.encode(), name.encode()) for name, _, _, _ in ranges
    )
    with open(SRC, "wb") as f:
        f.write(ROOT_TEMPLATE.replace("\n", eol.decode()).encode("latin-1")
                .replace(b"{includes}", includes))
    print("rewrote %s as a %d-line table of contents" % (SRC, len(ranges) + 20))
    return 0


if __name__ == "__main__":
    sys.exit(main())
