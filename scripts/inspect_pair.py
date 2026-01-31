#!/usr/bin/env python3
from analyze_overlapall import parse_overlapall, point_in_poly_int, segs_intersect_int
pairs = parse_overlapall('overlap.csv')
p = pairs[0]
print('pair',p['f1'],p['f2'],'reported',p['reported'])
print('poly1',p['poly1'])
print('poly2',p['poly2'])
print('\nPer-vertex containment:')
for i,(x,y) in enumerate(p['poly1']):
    print('poly1 vertex',i,(x,y),'in poly2?', point_in_poly_int(x,y,p['poly2']))
for i,(x,y) in enumerate(p['poly2']):
    print('poly2 vertex',i,(x,y),'in poly1?', point_in_poly_int(x,y,p['poly1']))
print('\nEdge proper intersections:')
segi=False
for i in range(len(p['poly1'])):
    a=p['poly1'][i]; b=p['poly1'][(i+1)%len(p['poly1'])]
    for j in range(len(p['poly2'])):
        c=p['poly2'][j]; d=p['poly2'][(j+1)%len(p['poly2'])]
        if segs_intersect_int(a,b,c,d):
            print('intersect edge',i,'of poly1 with edge',j,'of poly2')
            segi=True
print('segi=',segi)
