from math import fabs

A = [(226,124),(91,97),(98,88),(233,114)]  # face 44
B = [(218,108),(218,114),(206,112),(206,105)]  # face 3

EPS = 1e-9

def point_in_poly(pt, poly):
    # winding ray crossing (point on edge considered outside)
    x, y = pt
    cnt = 0
    n = len(poly)
    for i in range(n):
        j=(i+1)%n
        xi, yi = poly[i]
        xj, yj = poly[j]
        # point on edge
        cross = (xj-xi)*(y-yi) - (yj-yi)*(x-xi)
        if abs(cross) < EPS and min(xi,xj) <= x <= max(xi,xj) and min(yi,yj) <= y <= max(yi,yj):
            return False
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            cnt += 1
    return (cnt & 1) == 1

# bbox
minx1=min(x for x,y in A); maxx1=max(x for x,y in A); miny1=min(y for x,y in A); maxy1=max(y for x,y in A)
minx2=min(x for x,y in B); maxx2=max(x for x,y in B); miny2=min(y for x,y in B); maxy2=max(y for x,y in B)
ixmin = max(minx1,minx2); ixmax = min(maxx1,maxx2); iymin = max(miny1,miny2); iymax = min(maxy1,maxy2)
print('Int bbox', ixmin,iymin,ixmax,iymax)

strategies = {}
# current strategy: endpoints + mid (sx * (ixmax - ixmin) / 2)
pts = []
for sx in range(3):
    for sy in range(3):
        tx = ixmin + int((sx * (ixmax - ixmin)) / 2)
        ty = iymin + int((sy * (iymax - iymin)) / 2)
        pts.append((tx,ty))
strategies['endpoints_mid'] = pts

# fractional interior 1/6,1/2,5/6
pts = []
for sx in [1/6, 1/2, 5/6]:
    for sy in [1/6, 1/2, 5/6]:
        tx = int(round(ixmin + sx * (ixmax - ixmin)))
        ty = int(round(iymin + sy * (iymax - iymin)))
        pts.append((tx,ty))
strategies['frac_3'] = pts

# denser 5x5 fractions
pts=[]
for sx in [1/10,3/10,5/10,7/10,9/10]:
    for sy in [1/10,3/10,5/10,7/10,9/10]:
        tx=int(round(ixmin + sx*(ixmax-ixmin)))
        ty=int(round(iymin + sy*(iymax-iymin)))
        pts.append((tx,ty))
strategies['frac_5'] = pts

for name, pts in strategies.items():
    hit=False
    for p in pts:
        inA = point_in_poly(p,A)
        inB = point_in_poly(p,B)
        print(name, 'sample', p, 'inA', inA, 'inB', inB)
        if inA and inB:
            hit=True
    print('->', name, 'hit=', hit)
    print('---')
