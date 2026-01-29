from math import fabs

A = [(265,135),(226,124),(233,114),(270,128)]  # face 36
B = [(241,127),(109,100),(106,113),(238,141)]  # face 47

EPS = 1e-9

def point_in_poly(pt, poly):
    x, y = pt
    cnt = 0
    n = len(poly)
    for i in range(n):
        j=(i+1)%n
        xi, yi = poly[i]
        xj, yj = poly[j]
        # point on edge considered outside
        cross = (xj-xi)*(y-yi) - (yj-yi)*(x-xi)
        if abs(cross) < EPS and min(xi,xj) <= x <= max(xi,xj) and min(yi,yj) <= y <= max(yi,yj):
            return False
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            cnt += 1
    return (cnt & 1) == 1

minx1=min(x for x,y in A); maxx1=max(x for x,y in A); miny1=min(y for x,y in A); maxy1=max(y for x,y in A)
minx2=min(x for x,y in B); maxx2=max(x for x,y in B); miny2=min(y for x,y in B); maxy2=max(y for x,y in B)
ixmin = max(minx1,minx2); ixmax = min(maxx1,maxx2); iymin = max(miny1,miny2); iymax = min(maxy1,maxy2)
print('Int bbox', ixmin,iymin,ixmax,iymax)

pts=[]
for sx in [1/6,1/2,5/6]:
    for sy in [1/6,1/2,5/6]:
        tx = ixmin + int(round(sx*(ixmax-ixmin)))
        ty = iymin + int(round(sy*(iymax-iymin)))
        pts.append((tx,ty))
for p in pts:
    inA = point_in_poly(p,A)
    inB = point_in_poly(p,B)
    print('sample',p,'inA',inA,'inB',inB)
print('any both=', any(point_in_poly(p,A) and point_in_poly(p,B) for p in pts))
