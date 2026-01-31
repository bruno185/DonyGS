#!/usr/bin/env python3
import sys
from analyze_overlapall import parse_overlapall, sutherland_hodgman, polygon_area_and_centroid, point_in_poly_int, projected_polygons_overlap_from_polys

path = 'overlapZ0.csv'
if len(sys.argv) > 1: path = sys.argv[1]
ps = parse_overlapall(path)
if not ps:
    print('No pairs in', path); sys.exit(1)
for p in ps:
    print('file',path,'pair',p['f1'],p['f2'],'reported=',p['reported'])
    poly1 = [(float(x),float(y)) for x,y in p['poly1']]
    poly2 = [(float(x),float(y)) for x,y in p['poly2']]
    clip12 = sutherland_hodgman(poly1, poly2)
    clip21 = sutherland_hodgman(poly2, poly1)
    a12, c12 = polygon_area_and_centroid(clip12) if clip12 else (0.0,(0.0,0.0))
    a21, c21 = polygon_area_and_centroid(clip21) if clip21 else (0.0,(0.0,0.0))
    print('  clipped area 1in2=',a12,'cent=',c12)
    print('  clipped area 2in1=',a21,'cent=',c21)
    # sampling
    xs1=[p[0] for p in p['poly1']]; ys1=[p[1] for p in p['poly1']]
    xs2=[p[0] for p in p['poly2']]; ys2=[p[1] for p in p['poly2']]
    oxmin = max(min(xs1),min(xs2)); oxmax = min(max(xs1),max(xs2))
    oymin = max(min(ys1),min(ys2)); oymax = min(max(ys1),max(ys2))
    if oxmin<=oxmax and oymin<=oymax:
        W=oxmax-oxmin; H=oymax-oymin
        for sx in range(3):
            for sy in range(3):
                tx = oxmin + (((2*sx + 1) * W + 3) // 6)
                ty = oymin + (((2*sy + 1) * H + 3) // 6)
                in1 = point_in_poly_int(tx,ty,p['poly1'])
                in2 = point_in_poly_int(tx,ty,p['poly2'])
                print('  sample',tx,ty,'in1',in1,'in2',in2)
    print('  calc_python_overlap:', projected_polygons_overlap_from_polys(p['poly1'], p['poly2']))
    print()
