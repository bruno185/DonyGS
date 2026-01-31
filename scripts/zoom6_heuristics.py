#!/usr/bin/env python3
"""Try multiple robust heuristics for overlap detection on a CSV of pairs.
Usage: python scripts/zoom6_heuristics.py [input.csv]
"""
import sys
import math
from analyze_overlapall import parse_overlapall, sutherland_hodgman, polygon_area_and_centroid, segs_intersect_int, point_in_poly_int, projected_polygons_overlap_from_polys, compute_intersection_centroid

inpath = 'overlapZ6.csv'
if len(sys.argv) > 1: inpath = sys.argv[1]

pairs = parse_overlapall(inpath)
if not pairs:
    print('No pairs in', inpath); sys.exit(1)

# helper: pad polygon outward by delta pixels (approx by moving vertices away from centroid)
def pad_polygon(poly, delta):
    if not poly: return poly
    polyf = [(float(x),float(y)) for x,y in poly]
    area, centroid = polygon_area_and_centroid(polyf)
    if area == 0.0:
        # centroid fallback: average
        cx = sum(x for x,y in polyf)/len(polyf)
        cy = sum(y for x,y in polyf)/len(polyf)
    else:
        cx, cy = centroid
    out = []
    for x,y in polyf:
        dx = x - cx; dy = y - cy
        d = math.hypot(dx,dy)
        if d == 0:
            out.append((x,y))
        else:
            nx = x + (dx/d) * delta
            ny = y + (dy/d) * delta
            out.append((nx,ny))
    return out

# sampling check with NxN grid
def sampling_check(poly1, poly2, N=3):
    xs1=[p[0] for p in poly1]; ys1=[p[1] for p in poly1]
    xs2=[p[0] for p in poly2]; ys2=[p[1] for p in poly2]
    oxmin = max(min(xs1),min(xs2)); oxmax = min(max(xs1),max(xs2))
    oymin = max(min(ys1),min(ys2)); oymax = min(max(ys1),max(ys2))
    if oxmin > oxmax or oymin > oymax: return False
    W = oxmax - oxmin; H = oymax - oymin
    for sx in range(N):
        for sy in range(N):
            tx = oxmin + (((2*sx + 1) * W + (2*N-1)) // (2*N))
            ty = oymin + (((2*sy + 1) * H + (2*N-1)) // (2*N))
            if point_in_poly_int(tx,ty,poly1) and point_in_poly_int(tx,ty,poly2):
                return True
    return False

# run heuristics per pair (but we'll focus on the first pair, zoom6 input has one pair)
for p in pairs:
    f1 = p['f1']; f2 = p['f2']
    print('\nPair', f1, f2, 'reported=', p['reported'])
    poly1 = p['poly1']; poly2 = p['poly2']

    # baseline python strict
    baseline = projected_polygons_overlap_from_polys(poly1, poly2)
    print('  baseline (python strict) =', baseline)

    # edge intersection quick accept
    segi = False
    for i in range(len(poly1)):
        a = poly1[i]; b = poly1[(i+1)%len(poly1)]
        for j in range(len(poly2)):
            c = poly2[j]; d = poly2[(j+1)%len(poly2)]
            if segs_intersect_int(a,b,c,d): segi = True; break
        if segi: break
    print('  seg_intersect proper =', segi)

    # sampling grid tests
    for N in (3,5,7):
        sres = sampling_check(poly1, poly2, N=N)
        print(f'  sampling {N}x{N} ->', sres)

    # clipping areas both orders
    a12, c12 = compute_intersection_centroid([(float(x),float(y)) for x,y in poly1], [(float(x),float(y)) for x,y in poly2])
    a21, c21 = compute_intersection_centroid([(float(x),float(y)) for x,y in poly2], [(float(x),float(y)) for x,y in poly1])
    print('  clipped area poly1⊓poly2 =', a12)
    print('  clipped area poly2⊓poly1 =', a21)

    # small area acceptance thresholds
    for thr in (1.0, 0.1, 0.01, 0.0):
        accept = (a12 >= thr) or (a21 >= thr)
        print(f'  accept if area >= {thr} ->', accept)

    # padding tests: expand by 0.5px and 1.0px
    for delta in (0.5, 1.0, 2.0):
        p1 = pad_polygon(poly1, delta); p2 = pad_polygon(poly2, delta)
        res = projected_polygons_overlap_from_polys([(int(round(x)),int(round(y))) for x,y in p1], [(int(round(x)),int(round(y))) for x,y in p2])
        print(f'  padded by {delta}px ->', res)

    # hybrid heuristic: segi OR sampling 5x5 OR area >= 0.01
    hybrid = segi or sampling_check(poly1, poly2, N=5) or (a12 >= 0.01) or (a21 >= 0.01)
    print('  hybrid (segi or samp5 or area>=0.01) ->', hybrid)

    # done for pair
    break
