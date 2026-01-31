#!/usr/bin/env python3
from analyze_overlapall import parse_overlapall, segs_intersect_int, point_in_poly_int, projected_polygons_overlap_from_polys, compute_intersection_centroid

pairs = parse_overlapall('overlap.csv')
for idx,p in enumerate(pairs):
    poly1 = p['poly1']
    poly2 = p['poly2']
    # proper segment intersections
    segi = False
    for i in range(len(poly1)):
        a = poly1[i]; b = poly1[(i+1)%len(poly1)]
        for j in range(len(poly2)):
            c = poly2[j]; d = poly2[(j+1)%len(poly2)]
            if segs_intersect_int(a,b,c,d): segi = True; break
        if segi: break
    # containment
    cont12 = any(point_in_poly_int(px,py,poly2) for (px,py) in poly1)
    cont21 = any(point_in_poly_int(px,py,poly1) for (px,py) in poly2)
    # sampling
    sampling = False
    xs1=[p[0] for p in poly1]; ys1=[p[1] for p in poly1]
    xs2=[p[0] for p in poly2]; ys2=[p[1] for p in poly2]
    oxmin = max(min(xs1), min(xs2)); oxmax = min(max(xs1), max(xs2))
    oymin = max(min(ys1), min(ys2)); oymax = min(max(ys1), max(ys2))
    if oxmin <= oxmax and oymin <= oymax:
        W = oxmax - oxmin; H = oymax - oymin
        for sx in range(3):
            for sy in range(3):
                tx = oxmin + (((2*sx + 1) * W + 3) // 6)
                ty = oymin + (((2*sy + 1) * H + 3) // 6)
                if point_in_poly_int(tx,ty,poly1) and point_in_poly_int(tx,ty,poly2): sampling=True
    area, centroid = compute_intersection_centroid([(float(x),float(y)) for x,y in poly1], [(float(x),float(y)) for x,y in poly2])
    reported = p['reported']
    calc = projected_polygons_overlap_from_polys(poly1, poly2)
    print(f"idx={idx} faces={p['f1']},{p['f2']} reported={reported} calc={calc} segi={segi} cont12={cont12} cont21={cont21} sampling={sampling}")
    print("  clipped_area=", repr(area))
