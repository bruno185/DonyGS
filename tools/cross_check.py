# compute edge_intersects_face results for t.obj faces without numpy

def cross(u,v):
    return (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])

def dot(u,v):
    return u[0]*v[0]+u[1]*v[1]+u[2]*v[2]

def sub(u,v):
    return (u[0]-v[0], u[1]-v[1], u[2]-v[2])

# faces
verts=[(-18.518536,-198.491813,0.0),
       (381.481464,-198.491813,0.0),
       (-18.518536,201.508187,0.0),
       (381.481464,201.508187,0.0),
       (-200.0,0.0,200.0),
       (200.0,0.0,200.0),
       (-200.0,0.0,-200.0),
       (200.0,0.0,-200.0)]
faces=[[1,3,2,0],[5,7,6,4]]

def compute_plane(face):
    p0=verts[face[0]]
    p1=verts[face[1]]
    p2=verts[face[2]]
    ux=sub(p1,p0)
    vx=sub(p2,p0)
    norm=cross(ux,vx)
    d=-(dot(norm,p0))
    return norm,d

plane_eps=0.02
def point_in_poly(px,py,proj2):
    inside=False
    n=len(proj2)
    for i in range(n):
        j=(i+1)%n
        xi,yi=proj2[i]
        xj,yj=proj2[j]
        if ((yi>py)!=(yj>py)) and (px < (xj-xi)*(py-yi)/(yj-yi)+xi):
            inside = not inside
    return inside

def edge_intersects(ei,fj):
    norm,d = compute_plane(faces[fj])
    abs_vals = [abs(norm[0]),abs(norm[1]),abs(norm[2])]
    proj = abs_vals.index(max(abs_vals))
    proj_poly=[]
    for v in faces[fj]:
        X,Y,Z=verts[v]
        if proj==0: proj_poly.append((Y,Z))
        elif proj==1: proj_poly.append((X,Z))
        else: proj_poly.append((X,Y))
    cnt=len(faces[ei])
    for k in range(cnt):
        vi=faces[ei][k]
        vj=faces[ei][(k+1)%cnt]
        A=verts[vi]; B=verts[vj]
        sdA=dot(norm,A)+d
        sdB=dot(norm,B)+d
        if abs(sdA)<plane_eps: sdA=0.0
        if abs(sdB)<plane_eps: sdB=0.0
        if sdA*sdB < 0:
            t = -sdA/(sdB-sdA)
            P=(A[0]+t*(B[0]-A[0]),A[1]+t*(B[1]-A[1]),A[2]+t*(B[2]-A[2]))
            if proj==0: px,py=P[1],P[2]
            elif proj==1: px,py=P[0],P[2]
            else: px,py=P[0],P[1]
            if not any(px==p[0] and py==p[1] for p in proj_poly):
                onb=False
                for m in range(len(proj_poly)):
                    n=(m+1)%len(proj_poly)
                    x1,y1=proj_poly[m]; x2,y2=proj_poly[n]
                    cross2=(px-x1)*(y2-y1)-(py-y1)*(x2-x1)
                    if abs(cross2) < 1e-9:
                        if not ((px < x1 and px < x2) or (px > x1 and px > x2) or (py < y1 and py < y2) or (py > y1 and py > y2)):
                            onb=True; break
                if not onb and point_in_poly(px,py,proj_poly):
                    return True
    return False

print('i->j',edge_intersects(0,1))
print('j->i',edge_intersects(1,0))
