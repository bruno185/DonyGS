#!/usr/bin/env python3
from analyze_overlapall import parse_overlapall, sutherland_hodgman, polygon_area_and_centroid
p = parse_overlapall('overlap.csv')[0]
poly1 = [(float(x),float(y)) for x,y in p['poly1']]
poly2 = [(float(x),float(y)) for x,y in p['poly2']]
clip12 = sutherland_hodgman(poly1, poly2)
clip21 = sutherland_hodgman(poly2, poly1)
print('clip12 verts:', clip12)
print('area12,cent', polygon_area_and_centroid(clip12))
print('clip21 verts:', clip21)
print('area21,cent', polygon_area_and_centroid(clip21))
