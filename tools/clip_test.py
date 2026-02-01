#!/usr/bin/env python3
"""Fixed-point Sutherland-Hodgman test tool
Usage: python tools/clip_test.py [--pair f1,f2] [--csv overlap.csv]
Defaults to first pair in overlap.csv (or pair 7,44 if file missing).

Prints:
 - subj and clip integer vertex lists
 - clip orientation (signed area)
 - intermediate clip vertices (per-edge)
 - final polygon vertices (fixed -> float/px) and raw area (area2) and area px^2
 - integer bbox intersection area
"""
import argparse
import math
import csv
from pathlib import Path

FIXED_SHIFT = 16
FIXED_SCALE = 1 << FIXED_SHIFT


def read_overlap_csv(path):
    # returns (f1,f2, poly1, poly2)
    p = Path(path)
    if not p.exists():
        return (7,44, [(191,106),(212,101),(206,105)], [(226,124),(91,97),(98,88),(233,114)])
    with p.open('r', newline='') as f:
        rows = [r for r in csv.reader(f) if r]
    # find header row with face1
    f1 = f2 = None
    i = 0
    for r in rows:
        if len(r) >= 6 and r[0].strip().lower() == 'face1':
            try:
                f1 = int(r[1]); f2 = int(r[3])
            except:
                pass
            break
        i += 1
    # parse vertex lines after header label row
    poly1 = []
    poly2 = []
    # find indices of 'face_id' header
    for j in range(i, len(rows)):
        if rows[j] and rows[j][0].strip().lower() == 'face_id':
            # subsequent rows until blank or eof are vertices
            for k in range(j+1, len(rows)):
                rr = rows[k]
                if not rr or len(rr) < 5: break
                try:
                    fid = int(rr[0]); x = int(rr[3]); y = int(rr[4])
                except:
                    continue
                if fid == f1: poly1.append((x,y))
                elif fid == f2: poly2.append((x,y))
            break
    if not poly1 or not poly2:
        # fallback to default
        return (7,44, [(191,106),(212,101),(206,105)], [(226,124),(91,97),(98,88),(233,114)])
    return (f1,f2,poly1,poly2)


def signed_area_int(poly):
    s = 0
    n = len(poly)
    for i in range(n):
        x0,y0 = poly[i]
        x1,y1 = poly[(i+1)%n]
        s += x0*y1 - x1*y0
    return s


def clip_subject_by_clip_fixed(subj, clip, verbose=False):
    # subj and clip are lists of integer (x,y)
    # operate on 16.16 fixed copies of subject
    sx = [((x) << FIXED_SHIFT) for (x,y) in subj]
    sy = [((y) << FIXED_SHIFT) for (x,y) in subj]
    curr_n = len(sx)

    # compute clip orientation sign from integer coords
    clip_area2 = signed_area_int(clip)
    sign = 1 if clip_area2 >= 0 else -1

    if verbose:
        print('clip orientation signed_area2 (int):', clip_area2, 'sign:', sign)

    # clip against each edge of clip polygon
    clip_off = clip
    for j in range(len(clip_off)):
        if curr_n == 0: break
        cx1 = (clip_off[j][0] << FIXED_SHIFT); cy1 = (clip_off[j][1] << FIXED_SHIFT)
        cx2 = (clip_off[(j+1)%len(clip_off)][0] << FIXED_SHIFT); cy2 = (clip_off[(j+1)%len(clip_off)][1] << FIXED_SHIFT)
        out_tx = []
        out_ty = []
        if verbose:
            print('\nClip edge', j, 'clip pts (fixed):', (cx1,cy1), (cx2,cy2))
        for i in range(curr_n):
            ii = i; jj = (i+1) % curr_n
            s1x = sx[ii]; s1y = sy[ii]
            s2x = sx[jj]; s2y = sy[jj]
            cross1 = (cx2 - cx1) * (s1y - cy1) - (cy2 - cy1) * (s1x - cx1)
            cross2 = (cx2 - cx1) * (s2y - cy1) - (cy2 - cy1) * (s2x - cx1)
            in1 = (sign * cross1 > 0)
            in2 = (sign * cross2 > 0)
            if in1 and in2:
                out_tx.append(s2x); out_ty.append(s2y)
            elif in1 and not in2:
                denom = ((s2x - s1x) * (cy1 - cy2) + (s2y - s1y) * (cx2 - cx1))
                if denom != 0:
                    num = ((s1x - cx1) * (cy1 - cy2) + (s1y - cy1) * (cx2 - cx1))
                    dx = s2x - s1x; dy = s2y - s1y
                    ix = s1x + (num * dx) // denom
                    iy = s1y + (num * dy) // denom
                    out_tx.append(ix); out_ty.append(iy)
            elif not in1 and in2:
                denom = ((s2x - s1x) * (cy1 - cy2) + (s2y - s1y) * (cx2 - cx1))
                if denom != 0:
                    num = ((s1x - cx1) * (cy1 - cy2) + (s1y - cy1) * (cx2 - cx1))
                    dx = s2x - s1x; dy = s2y - s1y
                    ix = s1x + (num * dx) // denom
                    iy = s1y + (num * dy) // denom
                    out_tx.append(ix); out_ty.append(iy)
                out_tx.append(s2x); out_ty.append(s2y)
        if len(out_tx) == 0:
            sx = []; sy = []; curr_n = 0; break
        sx = out_tx; sy = out_ty; curr_n = len(sx)
        if verbose:
            pts = [(x>>FIXED_SHIFT, y>>FIXED_SHIFT) for x,y in zip(sx,sy)]
            print('after clip edge -> verts:', pts)

    final_poly = []
    if curr_n >= 3:
        area2 = 0
        cx_acc = 0
        cy_acc = 0
        for i in range(curr_n):
            j = (i+1)%curr_n
            x0 = sx[i]; y0 = sy[i]
            x1 = sx[j]; y1 = sy[j]
            cross = x0 * y1 - x1 * y0
            area2 += cross
            cx_acc += (x0 + x1) * cross
            cy_acc += (y0 + y1) * cross
            final_poly.append((x0, y0))
        area = abs(area2) * 0.5 / (FIXED_SCALE * FIXED_SCALE)
        signed_area = area2 * 0.5 / (FIXED_SCALE * FIXED_SCALE)
        Cx = (cx_acc) / (6.0 * signed_area * FIXED_SCALE * FIXED_SCALE) if abs(signed_area) > 1e-12 else 0
        Cy = (cy_acc) / (6.0 * signed_area * FIXED_SCALE * FIXED_SCALE) if abs(signed_area) > 1e-12 else 0
    else:
        area2 = 0; area = 0.0; Cx = Cy = 0.0
    return {
        'final_fixed_vertices': final_poly,
        'raw_area2': area2,
        'area': area,
        'centroid': (Cx, Cy)
    }


def bbox_intersection_area(poly1, poly2):
    minx1 = min(p[0] for p in poly1); maxx1 = max(p[0] for p in poly1)
    miny1 = min(p[1] for p in poly1); maxy1 = max(p[1] for p in poly1)
    minx2 = min(p[0] for p in poly2); maxx2 = max(p[0] for p in poly2)
    miny2 = min(p[1] for p in poly2); maxy2 = max(p[1] for p in poly2)
    ix0 = max(minx1, minx2); ix1 = min(maxx1, maxx2); iy0 = max(miny1, miny2); iy1 = min(maxy1, maxy2)
    if ix0 <= ix1 and iy0 <= iy1:
        return (ix0,ix1,iy0,iy1,(ix1-ix0)*(iy1-iy0))
    return (ix0,ix1,iy0,iy1,0)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', default='overlap.csv')
    ap.add_argument('--pair', default=None, help='f1,f2 override')
    ap.add_argument('--verbose', action='store_true')
    args = ap.parse_args()

    f1,f2,poly1,poly2 = read_overlap_csv(args.csv)
    if args.pair:
        try:
            p1,p2 = args.pair.split(',')
            if int(p1)!=f1 or int(p2)!=f2:
                print('Note: pair override ignored for CSV parsing; use custom polygons by editing script')
        except:
            pass

    print('Pair:', f1, f2)
    print('poly1:', poly1)
    print('poly2:', poly2)
    ix0,ix1,iy0,iy1,bbox_area = bbox_intersection_area(poly1, poly2)
    print('\nInteger bbox intersection: ix0=%d ix1=%d iy0=%d iy1=%d area=%d' % (ix0,ix1,iy0,iy1,bbox_area))

    print('\n--- Clip poly1 (subj) by poly2 (clip)')
    r1 = clip_subject_by_clip_fixed(poly1, poly2, verbose=args.verbose)
    print('final verts (px):', [(x>>FIXED_SHIFT, y>>FIXED_SHIFT) for x,y in r1['final_fixed_vertices']])
    print('raw_area2:', r1['raw_area2'], 'area px^2:', r1['area'], 'centroid:', r1['centroid'])

    print('\n--- Clip poly2 (subj) by poly1 (clip)')
    r2 = clip_subject_by_clip_fixed(poly2, poly1, verbose=args.verbose)
    print('final verts (px):', [(x>>FIXED_SHIFT, y>>FIXED_SHIFT) for x,y in r2['final_fixed_vertices']])
    print('raw_area2:', r2['raw_area2'], 'area px^2:', r2['area'], 'centroid:', r2['centroid'])

    print('\nSummary: area1=%.6f area2=%.6f bbox_area=%d' % (r1['area'], r2['area'], bbox_area))
