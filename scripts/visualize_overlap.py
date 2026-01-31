#!/usr/bin/env python3
"""Create an SVG visualising two polygons and their intersection for a given pair in overlap.csv
Usage: python scripts/visualize_overlap.py [face1] [face2] [input.csv]
If face1/face2 omitted, uses first pair in input.csv.
"""
import sys
from analyze_overlapall import parse_overlapall, sutherland_hodgman, polygon_area_and_centroid, point_in_poly_int

inpath = 'overlap.csv'
f1_req = None; f2_req = None
if len(sys.argv) > 1:
    try:
        f1_req = int(sys.argv[1]); f2_req = int(sys.argv[2])
    except Exception:
        pass
    if len(sys.argv) > 3:
        inpath = sys.argv[3]

pairs = parse_overlapall(inpath)
if not pairs:
    print('No pairs in', inpath); sys.exit(1)

# pick pair
pair = None
if f1_req is not None:
    for p in pairs:
        if p['f1'] == f1_req and p['f2'] == f2_req:
            pair = p; break
    if pair is None:
        print(f'Pair {f1_req},{f2_req} not found in {inpath}'); sys.exit(1)
else:
    pair = pairs[0]

poly1 = pair['poly1']; poly2 = pair['poly2']
print('Selected pair:', pair['f1'], pair['f2'], 'reported=', pair['reported'])

# get clipped polygon in both orders
clip12 = sutherland_hodgman([(float(x),float(y)) for x,y in poly1], [(float(x),float(y)) for x,y in poly2])
clip21 = sutherland_hodgman([(float(x),float(y)) for x,y in poly2], [(float(x),float(y)) for x,y in poly1])
area12, cent12 = polygon_area_and_centroid(clip12) if clip12 else (0.0, (0.0,0.0))
area21, cent21 = polygon_area_and_centroid(clip21) if clip21 else (0.0, (0.0,0.0))
print('clipped area poly1 clipped by poly2 =', area12, 'centroid=', cent12)
print('clipped area poly2 clipped by poly1 =', area21, 'centroid=', cent21)
print('clipped poly12 verts:', clip12)
print('clipped poly21 verts:', clip21)

# sampling points in overlap bbox
xs1=[p[0] for p in poly1]; ys1=[p[1] for p in poly1]
xs2=[p[0] for p in poly2]; ys2=[p[1] for p in poly2]
oxmin = max(min(xs1), min(xs2)); oxmax = min(max(xs1), max(xs2))
oymin = max(min(ys1), min(ys2)); oymax = min(max(ys1), max(ys2))
sampling_points = []
if oxmin <= oxmax and oymin <= oymax:
    W = oxmax - oxmin; H = oymax - oymin
    for sx in range(3):
        for sy in range(3):
            tx = oxmin + (((2*sx + 1) * W + 3) // 6)
            ty = oymin + (((2*sy + 1) * H + 3) // 6)
            inside1 = point_in_poly_int(tx,ty,poly1)
            inside2 = point_in_poly_int(tx,ty,poly2)
            sampling_points.append((tx,ty,inside1,inside2))
print('sampling points (tx,ty,in1,in2):', sampling_points)

# Write SVG
out_svg = f"overlap_{pair['f1']}_{pair['f2']}.svg"
margin = 10
minx = min(min(xs1), min(xs2)) - margin; maxx = max(max(xs1), max(xs2)) + margin
miny = min(min(ys1), min(ys2)) - margin; maxy = max(max(ys1), max(ys2)) + margin
w = maxx - minx; h = maxy - miny
scale = 1

def pts_to_svg_path(pts):
    if not pts: return ''
    s = 'M ' + ' '.join(f"{x},{y}" for (x,y) in pts) + ' Z'
    return s

with open(out_svg,'w',encoding='utf-8') as f:
    f.write(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{minx} {miny} {w} {h}" width="{w}" height="{h}">\n')
    f.write('<rect x="{0}" y="{1}" width="{2}" height="{3}" fill="#ffffff" stroke="none"/>\n'.format(minx,miny,w,h))
    # poly1
    f.write('<path d="'+pts_to_svg_path(poly1)+'" fill="none" stroke="green" stroke-width="2" opacity="0.9"/>\n')
    # poly2
    f.write('<path d="'+pts_to_svg_path(poly2)+'" fill="none" stroke="orange" stroke-width="2" opacity="0.9"/>\n')
    # clipped poly12
    if clip12:
        f.write('<path d="'+pts_to_svg_path(clip12)+'" fill="red" opacity="0.4" stroke="red" stroke-width="1"/>\n')
    if clip21:
        # if different, also show with hatch-ish stroke
        f.write('<!-- alternate clipping (poly2 clipped by poly1) shown with blue stroke -->\n')
        f.write('<path d="'+pts_to_svg_path(clip21)+'" fill="none" stroke="blue" stroke-width="1" opacity="0.7" stroke-dasharray="4,2"/>\n')
    # centroid points
    def write_dot(x,y,color): f.write(f'<circle cx="{x}" cy="{y}" r="2" fill="{color}" stroke="black" stroke-width="0.2"/>\n')
    if cent12: write_dot(cent12[0], cent12[1],'black')
    if cent21: write_dot(cent21[0], cent21[1],'black')
    for (tx,ty,in1,in2) in sampling_points:
        col = 'blue' if in1 and in2 else ('lightblue' if in1 or in2 else 'grey')
        f.write(f'<circle cx="{tx}" cy="{ty}" r="2" fill="{col}" stroke="none" opacity="0.9"/>\n')
    f.write('</svg>\n')

print('Wrote', out_svg)
print('Open it (browser) to inspect why your screen differs.')
