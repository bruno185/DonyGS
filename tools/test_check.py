import numpy as np

# replicates updated check_intersect logic: try both splits for an
# interpenetrating pair before restarting scan.

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
    p0= np.array(verts[face[0]])
    p1= np.array(verts[face[1]])
    p2= np.array(verts[face[2]])
    norm=np.cross(p1-p0,p2-p0)
    d=-np.dot(norm,p0)
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
            inside=not inside
    return inside


def edge_intersects(ei,fj):
    norm,d=compute_plane(faces[fj])
    abs_vals=np.abs(norm)
    proj=np.argmax(abs_vals)
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
        A=np.array(verts[vi]); B=np.array(verts[vj])
        sdA=np.dot(norm,A)+d
        sdB=np.dot(norm,B)+d
        if abs(sdA)<plane_eps: sdA=0.0
        if abs(sdB)<plane_eps: sdB=0.0
        if sdA*sdB<0:
            t=-sdA/(sdB-sdA)
            P=A+t*(B-A)
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


# strict interior test mirrors the C helper edge_crosses_interior
# boundary hits (vertex or edge) are rejected.
def edge_crosses_interior(ei,fj):
    norm,d=compute_plane(faces[fj])
    abs_vals=np.abs(norm)
    proj=np.argmax(abs_vals)
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
        A=np.array(verts[vi]); B=np.array(verts[vj])
        sdA=np.dot(norm,A)+d
        sdB=np.dot(norm,B)+d
        if abs(sdA)<plane_eps: sdA=0.0
        if abs(sdB)<plane_eps: sdB=0.0
        if sdA*sdB<0:
            t=-sdA/(sdB-sdA)
            P=A+t*(B-A)
            if proj==0: px,py=P[1],P[2]
            elif proj==1: px,py=P[0],P[2]
            else: px,py=P[0],P[1]
            # reject if on any vertex
            if any(px==p[0] and py==p[1] for p in proj_poly):
                continue
            # reject if on any edge
            onb=False
            for m in range(len(proj_poly)):
                n=(m+1)%len(proj_poly)
                x1,y1=proj_poly[m]; x2,y2=proj_poly[n]
                cross2=(px-x1)*(y2-y1)-(py-y1)*(x2-x1)
                if abs(cross2) < 1e-9:
                    if not ((px < x1 and px < x2) or (px > x1 and px > x2) or (py < y1 and py < y2) or (py > y1 and py > y2)):
                        onb=True; break
            if onb:
                continue
            if point_in_poly(px,py,proj_poly):
                return True
    return False


def split_face(f1,f2):
    norm,d=compute_plane(faces[f2])
    cnt=len(faces[f1])
    dist=[]
    for vi in faces[f1]:
        x,y,z=verts[vi]
        sd=np.dot(norm,(x,y,z))+d
        if abs(sd)<plane_eps: sd=0.0
        dist.append(sd)
    hasPos=any(s>0 for s in dist)
    hasNeg=any(s<0 for s in dist)
    if not hasPos or not hasNeg:
        return False
    pos_list=[]; neg_list=[]; cop_list=[]
    for i in range(cnt):
        j=(i+1)%cnt
        di=dist[i]; dj=dist[j]
        vi=faces[f1][i]; vj=faces[f1][j]
        if di==0: cop_list.append(vi)
        if di>=0: pos_list.append(vi)
        if di<=0: neg_list.append(vi)
        if di*dj < 0:
            t=di/(di-dj)
            A=np.array(verts[vi]); B=np.array(verts[vj])
            P=A+t*(B-A)
            verts.append(tuple(P))
            newvi=len(verts)-1
            pos_list.append(newvi); neg_list.append(newvi)
    if len(pos_list)<3 or len(neg_list)<3:
        return False
    faces[f1]=pos_list
    faces.append(neg_list)
    return True

print('before',faces)
changed=True
ip_count=0
while changed:
    changed=False
    fc=len(faces)
    for i in range(fc):
        for j in range(i+1,fc):
            # simplified: test edges of i against j first
            if edge_intersects(i,j):
                print('pair',i,j,'edge i->j intersects, cutting j')
                # count only if the crossing is strictly interior
                if edge_crosses_interior(i,j) or edge_crosses_interior(j,i):
                    ip_count += 1
                if split_face(j,i):
                    print('split',j,'by',i)
                    changed=True
                    break
            elif edge_intersects(j,i):
                print('pair',i,j,'edge j->i intersects, cutting i')
                if edge_crosses_interior(i,j) or edge_crosses_interior(j,i):
                    ip_count += 1
                if split_face(i,j):
                    print('split',i,'by',j)
                    changed=True
                    break
        if changed: break
print('after',faces)
print('interpenetration count',ip_count)

