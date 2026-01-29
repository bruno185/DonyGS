#!/usr/bin/env python3
"""Analyze overlapall.csv and verify each pair using a Python reimplementation
of projected_polygons_overlap identical in semantics to the C code.

Outputs a summary and a CSV `mismatches.csv` with any mismatches.
"""
import sys
import math
from collections import defaultdict

MIN_INTERSECTION_AREA_PIXELS = 1.0
EPS = 1e-9


def orient_ll(ax, ay, bx, by, cx, cy):
    return (bx-ax)*(cy-ay) - (by-ay)*(cx-ax)


def on_seg_ll(ax, ay, bx, by, cx, cy):
    if ((ax <= cx <= bx) or (bx <= cx <= ax)) and ((ay <= cy <= by) or (by <= cy <= ay)):
        return True
    return False


def segs_intersect_int(a, b, c, d):
    x1,y1 = a; x2,y2 = b; x3,y3 = c; x4,y4 = d
    o1 = orient_ll(x1,y1,x2,y2,x3,y3)
    o2 = orient_ll(x1,y1,x2,y2,x4,y4)
    o3 = orient_ll(x3,y3,x4,y4,x1,y1)
    o4 = orient_ll(x3,y3,x4,y4,x2,y2)
    if ((o1 > 0 and o2 < 0) or (o1 < 0 and o2 > 0)) and ((o3 > 0 and o4 < 0) or (o3 < 0 and o4 > 0)):
        return True
    return False


def point_on_edge(px, py, poly):
    n = len(poly)
    for i in range(n):
        j = (i+1)%n
        xi, yi = poly[i]
        xj, yj = poly[j]
        if orient_ll(xi, yi, xj, yj, px, py) == 0 and on_seg_ll(xi, yi, xj, yj, px, py):
            return True
    return False


def point_in_poly_int(px, py, poly):
    # Ray casting, returns 1 if inside, 0 if outside or on edge
    if point_on_edge(px,py,poly):
        return 0
    cnt = 0
    n = len(poly)
    for i in range(n):
        j = (i+1)%n
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > py) != (yj > py)):
            # px < (xj - xi) * (py - yi) / (yj - yi) + xi
            # Use integer arithmetic equivalent -> use float but equivalent sign
            xint = (xj - xi) * (py - yi) / (yj - yi) + xi
            if px < xint:
                cnt += 1
    return cnt & 1


def faces_vertices_equal(poly1, poly2):
    n1 = len(poly1); n2 = len(poly2)
    if n1 != n2 or n1 == 0: return False
    # forward rotation
    for shift in range(n1):
        ok = True
        for k in range(n1):
            if poly1[k] != poly2[(shift + k) % n1]:
                ok = False; break
        if ok: return True
    # reverse rotation
    for shift in range(n1):
        ok = True
        for k in range(n1):
            if poly1[k] != poly2[(shift - k) % n1]:
                ok = False; break
        if ok: return True
    return False


def sutherland_hodgman(subject, clipper):
    # clip subject polygon by clipper polygon (counterclockwise edges), return clipped polygon (list of points)
    output = subject[:]
    if not output: return []
    nclip = len(clipper)
    for j in range(nclip):
        input_list = output[:]
        output = []
        if not input_list: break
        cx1, cy1 = clipper[j]
        cx2, cy2 = clipper[(j+1)%nclip]
        for i in range(len(input_list)):
            sx1, sy1 = input_list[i]
            sx2, sy2 = input_list[(i+1)%len(input_list)]
            cross1 = (cx2 - cx1) * (sy1 - cy1) - (cy2 - cy1) * (sx1 - cx1)
            cross2 = (cx2 - cx1) * (sy2 - cy1) - (cy2 - cy1) * (sx2 - cx1)
            in1 = cross1 > EPS
            in2 = cross2 > EPS
            if in1 and in2:
                output.append((sx2, sy2))
            elif in1 and not in2:
                # leaving: compute intersection
                denom = ( (sx2 - sx1) * (cy1 - cy2) + (sy2 - sy1) * (cx2 - cx1) )
                if abs(denom) < EPS:
                    continue
                t = ( (sx1 - cx1) * (cy1 - cy2) + (sy1 - cy1) * (cx2 - cx1) ) / denom
                ix = sx1 + t * (sx2 - sx1)
                iy = sy1 + t * (sy2 - sy1)
                output.append((ix, iy))
            elif not in1 and in2:
                # entering: intersection then end
                denom = ( (sx2 - sx1) * (cy1 - cy2) + (sy2 - sy1) * (cx2 - cx1) )
                if abs(denom) < EPS:
                    output.append((sx2, sy2))
                else:
                    t = ( (sx1 - cx1) * (cy1 - cy2) + (sy1 - cy1) * (cx2 - cx1) ) / denom
                    ix = sx1 + t * (sx2 - sx1)
                    iy = sy1 + t * (sy2 - sy1)
                    output.append((ix, iy))
                    output.append((sx2, sy2))
            else:
                # both out -> nothing
                pass
    return output


def polygon_area_and_centroid(poly):
    # poly as list of (x,y) floats
    n = len(poly)
    if n < 3: return 0.0, (0.0, 0.0)
    A = 0.0
    Cx = 0.0; Cy = 0.0
    for i in range(n):
        x0,y0 = poly[i]
        x1,y1 = poly[(i+1)%n]
        cross = x0*y1 - x1*y0
        A += cross
        Cx += (x0 + x1) * cross
        Cy += (y0 + y1) * cross
    A = A * 0.5
    if abs(A) < EPS:
        return 0.0, (0.0, 0.0)
    Cx = Cx / (6.0 * A)
    Cy = Cy / (6.0 * A)
    return abs(A), (Cx, Cy)


def compute_intersection_centroid(poly1, poly2):
    # clip poly1 by poly2
    clipped = sutherland_hodgman(poly1, poly2)
    if not clipped: return 0.0, (0.0,0.0)
    area, centroid = polygon_area_and_centroid(clipped)
    return area, centroid


def projected_polygons_overlap_from_polys(poly1, poly2):
    # poly as list of integer (x,y)
    if not poly1 or not poly2: return False
    n1 = len(poly1); n2 = len(poly2)
    if n1 < 3 or n2 < 3: return False
    xs1 = [p[0] for p in poly1]; ys1 = [p[1] for p in poly1]
    xs2 = [p[0] for p in poly2]; ys2 = [p[1] for p in poly2]
    minx1 = min(xs1); maxx1 = max(xs1); miny1 = min(ys1); maxy1 = max(ys1)
    minx2 = min(xs2); maxx2 = max(xs2); miny2 = min(ys2); maxy2 = max(ys2)
    # bbox quick reject/touching
    if maxx1 <= minx2 or maxx2 <= minx1 or maxy1 <= miny2 or maxy2 <= miny1:
        return False
    # edge-edge proper intersection
    candidate = False
    for i in range(n1):
        a = poly1[i]; b = poly1[(i+1)%n1]
        # per-edge bbox quick reject
        aminx = min(a[0], b[0]); amaxx = max(a[0], b[0])
        aminy = min(a[1], b[1]); amaxy = max(a[1], b[1])
        for j in range(n2):
            c = poly2[j]; d = poly2[(j+1)%n2]
            cminx = min(c[0], d[0]); cmaxx = max(c[0], d[0])
            cminy = min(c[1], d[1]); cmaxy = max(c[1], d[1])
            if amaxx <= cminx or cmaxx <= aminx or amaxy <= cminy or cmaxy <= aminy: continue
            if segs_intersect_int(a,b,c,d):
                candidate = True; break
        if candidate: break
    # containment tests: any vertex of poly1 inside poly2 or vice versa
    if not candidate:
        for (px,py) in poly1:
            if px < minx2 or px > maxx2 or py < miny2 or py > maxy2: continue
            if point_in_poly_int(px, py, poly2): candidate = True; break
    if not candidate:
        for (px,py) in poly2:
            if px < minx1 or px > maxx1 or py < miny1 or py > maxy1: continue
            if point_in_poly_int(px, py, poly1): candidate = True; break
    if not candidate:
        # identical polygon check
        if faces_vertices_equal(poly1, poly2):
            return True
        return False
    # candidate found: do 3x3 sampling
    oxmin = max(minx1, minx2); oxmax = min(maxx1, maxx2)
    oymin = max(miny1, miny2); oymax = min(maxy1, maxy2)
    if oxmin > oxmax or oymin > oymax: return False
    ixmin = oxmin; ixmax = oxmax; iymin = oymin; iymax = oymax
    W = ixmax - ixmin; H = iymax - iymin
    for sx in range(3):
        for sy in range(3):
            tx = ixmin + (((2*sx + 1) * W + 3) // 6)
            ty = iymin + (((2*sy + 1) * H + 3) // 6)
            if point_in_poly_int(tx, ty, poly1) and point_in_poly_int(tx, ty, poly2):
                return True
    # sampling failed -> exact clipping
    area, centroid = compute_intersection_centroid([(float(x),float(y)) for x,y in poly1], [(float(x),float(y)) for x,y in poly2])
    if area >= MIN_INTERSECTION_AREA_PIXELS:
        return True
    return False


def parse_overlapall(path):
    pairs = []
    with open(path, 'r', encoding='utf-8') as f:
        lines = [l.rstrip('\n') for l in f]
    i = 0
    n = len(lines)
    while i < n:
        line = lines[i].strip()
        if not line:
            i+=1; continue
        if line.startswith('face1,'):
            parts = line.split(',')
            f1 = int(parts[1]); f2 = int(parts[3]); reported = parts[5]
            i+=1
            # next line is header
            hdr = lines[i].strip(); i+=1
            poly1 = []
            poly2 = []
            # read lines until blank or next face1
            while i < n and lines[i].strip():
                parts = lines[i].split(',')
                fid = int(parts[0]); order = int(parts[1]); vid = int(parts[2]); x = int(parts[3]); y = int(parts[4])
                if fid == f1:
                    poly1.append((x,y))
                else:
                    poly2.append((x,y))
                i+=1
            pairs.append({'f1':f1,'f2':f2,'reported':reported,'poly1':poly1,'poly2':poly2})
        else:
            i+=1
    return pairs


def analyze(path):
    pairs = parse_overlapall(path)
    mismatches = []
    total = len(pairs)
    for idx,p in enumerate(pairs):
        calc = projected_polygons_overlap_from_polys(p['poly1'], p['poly2'])
        reported = (p['reported'] == 'YES')
        if calc != reported:
            # compute clipped area too for info
            area, _ = compute_intersection_centroid([(float(x),float(y)) for x,y in p['poly1']], [(float(x),float(y)) for x,y in p['poly2']])
            mismatches.append((idx, p['f1'], p['f2'], reported, calc, area))
    # report
    print(f"Analyzed {total} pairs, mismatches: {len(mismatches)}")
    if mismatches:
        out = 'mismatches.csv'
        with open(out,'w',encoding='utf-8') as g:
            g.write('idx,face1,face2,reported,calculated,clipped_area\n')
            for m in mismatches:
                g.write(f"{m[0]},{m[1]},{m[2]},{'YES' if m[3] else 'NO'},{'YES' if m[4] else 'NO'},{m[5]:.6f}\n")
        print(f"Wrote mismatches to {out}")
    else:
        print('No mismatches found')


if __name__ == '__main__':
    path = 'overlapall.csv'
    if len(sys.argv) > 1: path = sys.argv[1]
    analyze(path)
