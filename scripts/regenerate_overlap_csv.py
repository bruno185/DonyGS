#!/usr/bin/env python3
"""Regenerate a corrected overlap CSV: recompute overlap using strict test and write a fixed CSV.
Usage: python scripts/regenerate_overlap_csv.py [input.csv] [output.csv]
"""
import sys
from analyze_overlapall import parse_overlapall, projected_polygons_overlap_from_polys, compute_intersection_centroid

inpath = 'overlap.csv'
outpath = 'overlap_fixed.csv'
if len(sys.argv) > 1: inpath = sys.argv[1]
if len(sys.argv) > 2: outpath = sys.argv[2]

pairs = parse_overlapall(inpath)
if not pairs:
    print(f"No pairs found in {inpath}")
    sys.exit(0)

fixed_buf = []
changed = 0
for p in pairs:
    f1 = p['f1']; f2 = p['f2']
    reported_yes = (p['reported'] == 'YES')
    calc_yes = projected_polygons_overlap_from_polys(p['poly1'], p['poly2'])
    area, centroid = compute_intersection_centroid([(float(x),float(y)) for x,y in p['poly1']], [(float(x),float(y)) for x,y in p['poly2']])
    if calc_yes != reported_yes:
        changed += 1
    fixed_buf.append({'f1':f1,'f2':f2,'overlap': 'YES' if calc_yes else 'NO','poly1':p['poly1'],'poly2':p['poly2'],'area':area})

with open(outpath,'w',encoding='utf-8') as g:
    for p in fixed_buf:
        g.write(f"face1,{p['f1']},face2,{p['f2']},overlap,{p['overlap']}\n")
        g.write("face_id,vertex_order,vertex_index,x2d,y2d\n")
        for vi, (x,y) in enumerate(p['poly1']):
            g.write(f"{p['f1']},{vi},0,{x},{y}\n")
        for vi, (x,y) in enumerate(p['poly2']):
            g.write(f"{p['f2']},{vi},0,{x},{y}\n")
        g.write('\n')

print(f"Wrote {outpath}: {len(fixed_buf)} pairs, {changed} entries changed compared to reported")
