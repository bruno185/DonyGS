from math import fabs

A = [(191,106),(212,101),(206,105)]  # face 7
B = [(226,124),(91,97),(98,88),(233,114)]  # face 44

EPS = 1e-9

def area(poly):
    a = 0.0
    n = len(poly)
    for i in range(n):
        x1,y1 = poly[i]
        x2,y2 = poly[(i+1)%n]
        a += x1*y2 - x2*y1
    return 0.5*a

def centroid(poly):
    a2 = 0.0
    cx = 0.0
    cy = 0.0
    n = len(poly)
    for i in range(n):
        j=(i+1)%n
        x1,y1 = poly[i]
        x2,y2 = poly[j]
        cross = x1*y2 - x2*y1
        a2 += cross
        cx += (x1 + x2) * cross
        cy += (y1 + y2) * cross
    a = 0.5*a2
    if fabs(a) < EPS:
        return 0.0, (0,0)
    return a, (cx/(6*a), cy/(6*a))

# Sutherland-Hodgman clipping function: clip subject by a single clip edge (p1->p2)
def clip_polygon(subject, p1, p2):
    out = []
    n = len(subject)
    if n == 0: return out
    cx1,cy1 = p1
    cx2,cy2 = p2
    for i in range(n):
        x1,y1 = subject[i]
        x2,y2 = subject[(i+1)%n]
        cross1 = (cx2-cx1)*(y1-cy1) - (cy2-cy1)*(x1-cx1)
        cross2 = (cx2-cx1)*(y2-cy1) - (cy2-cy1)*(x2-cx1)
        in1 = cross1 > EPS
        in2 = cross2 > EPS
        if in1 and in2:
            out.append((x2,y2))
        elif in1 and not in2:
            denom = (x1-x2)*(cy1-cy2) - (y1-y2)*(cx1-cx2)
            if fabs(denom) > 1e-12:
                numx = (x1*y2 - y1*x2)*(cx1-cx2) - (x1-x2)*(cx1*cy2 - cy1*cx2)
                numy = (x1*y2 - y1*x2)*(cy1-cy2) - (y1-y2)*(cx1*cy2 - cy1*cx2)
                ix = numx/denom
                iy = numy/denom
                out.append((ix,iy))
        elif not in1 and in2:
            denom = (x1-x2)*(cy1-cy2) - (y1-y2)*(cx1-cx2)
            if fabs(denom) > 1e-12:
                numx = (x1*y2 - y1*x2)*(cx1-cx2) - (x1-x2)*(cx1*cy2 - cy1*cx2)
                numy = (x1*y2 - y1*x2)*(cy1-cy2) - (y1-y2)*(cx1*cy2 - cy1*cx2)
                ix = numx/denom
                iy = numy/denom
                out.append((ix,iy))
            out.append((x2,y2))
    return out

# Perform full clipping
subject = A[:]
for j in range(len(B)):
    subject = clip_polygon(subject, B[j], B[(j+1)%len(B)])

if len(subject) >= 3:
    a, c = centroid(subject)
else:
    a, c = 0.0, (0,0)

print('Intersection vertices:', subject)
print('Area:', a)
print('Centroid:', c)
