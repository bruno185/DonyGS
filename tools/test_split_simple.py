import math

# load t.obj
verts=[]
faces=[]
with open(r'f:\Bruno\Dev\AppleWin\Projets\ORCA\OBJ Viewer Review\DonyGS\t.obj') as f:
    for line in f:
        if line.startswith('v '):
            parts=line.split()
            verts.append(tuple(float(x) for x in parts[1:4]))
        elif line.startswith('f '):
            parts=line.split()
            idxs=[int(x.split('/')[0])-1 for x in parts[1:]]
            faces.append(idxs)

print('loaded',len(verts),'verts',len(faces),'faces')

# helpers

def plane_from_face(face):
    p0=verts[face[0]]
    p1=verts[face[1]]
    p2=verts[face[2]]
    ux=[p1[i]-p0[i] for i in range(3)]
    vx=[p2[i]-p0[i] for i in range(3)]
    a = ux[1]*vx[2]-ux[2]*vx[1]
    b = ux[2]*vx[0]-ux[0]*vx[2]
    c = ux[0]*vx[1]-ux[1]*vx[0]
    d = -(a*p0[0]+b*p0[1]+c*p0[2])
    return a,b,c,d


def point_in_poly(px,py,poly):
    inside=False
    n=len(poly)
    for k in range(n):
        n2=(k+1)%n
        xi,yi=verts[poly[k]][0],verts[poly[k]][1]
        xj,yj=verts[poly[n2]][0],verts[poly[n2]][1]
        if ((yi>py)!=(yj>py)) and (px < (xj-xi)*(py-yi)/(yj-yi+1e-12)+xi):
            inside=not inside
    return inside


def edge_intersects_face(ei,fj):
    a,b,c,d = plane_from_face(faces[fj])
    for idx in range(len(faces[ei])):
        vi=faces[ei][idx]
        vj=faces[ei][(idx+1)%len(faces[ei])]
        x1,y1,z1=verts[vi]
        x2,y2,z2=verts[vj]
        sd1=a*x1+b*y1+c*z1+d
        sd2=a*x2+b*y2+c*z2+d
        if abs(sd1)<0.02: sd1=0.0
        if abs(sd2)<0.02: sd2=0.0
        if sd1*sd2<0.0:
            t=sd1/(sd1-sd2)
            ix=x1+t*(x2-x1)
            iy=y1+t*(y2-y1)
            if point_in_poly(int(ix),int(iy),faces[fj]):
                return True
    return False

# apply simple rule
for i in range(len(faces)):
    for j in range(i+1,len(faces)):
        print('pair',i,j)
        if edge_intersects_face(i,j):
            print(' split face',j,'by plane',i)
        elif edge_intersects_face(j,i):
            print(' split face',i,'by plane',j)
