# replicate split_face_by_plane behaviour on t.obj for specific cut
import math

# load t.obj
verts=[]
faces=[]
with open(r'f:\Bruno\Dev\AppleWin\Projets\ORCA\OBJ Viewer Review\DonyGS\t.obj') as f:
    for line in f:
        if line.startswith('v '):
            parts=line.split()
            verts.append(tuple(float(x) for x in parts[1:4]))
        if line.startswith('f '):
            parts=line.split()
            idxs=[int(x.split('/')[0]) - 1 for x in parts[1:]]
            faces.append(idxs)

print('faces',faces)

# switch to faces index numbering like C (0-based)

def compute_plane(face):
    p0=verts[face[0]]
    p1=verts[face[1]]
    p2=verts[face[2]]
    ux=[p1[i]-p0[i] for i in range(3)]
    vx=[p2[i]-p0[i] for i in range(3)]
    a=ux[1]*vx[2]-ux[2]*vx[1]
    b=ux[2]*vx[0]-ux[0]*vx[2]
    c=ux[0]*vx[1]-ux[1]*vx[0]
    d=-(a*p0[0]+b*p0[1]+c*p0[2])
    return a,b,c,d


plane_eps=0.02

def split_face(f1,f2):
    a,b,c,d=compute_plane(faces[f2])
    cnt=len(faces[f1])
    dist=[]
    for vi in faces[f1]:
        x,y,z=verts[vi]
        sd=a*x+b*y+c*z+d
        if abs(sd)<plane_eps: sd=0.0
        dist.append(sd)
    print('distance f%d wrt plane%d:'%(f1,f2),dist)
    hasPos=any(s>0 for s in dist); hasNeg=any(s<0 for s in dist)
    print('hasPos',hasPos,'hasNeg',hasNeg)
    if not hasPos or not hasNeg:
        print('no split')
        return
    pos_list=[]; neg_list=[]; cop_list=[]
    orig=faces[f1][:]
    for i in range(cnt):
        j=(i+1)%cnt
        di=dist[i]; dj=dist[j]
        vi=orig[i]; vj=orig[j]
        if di==0: cop_list.append(vi)
        if di>=0: pos_list.append(vi)
        if di<=0: neg_list.append(vi)
        if di*dj<0:
            t=di/(di-dj)
            x1,y1,z1=verts[vi]
            x2,y2,z2=verts[vj]
            ix=x1+t*(x2-x1)
            iy=y1+t*(y2-y1)
            iz=z1+t*(z2-z1)
            newvi=len(verts)
            print('intersection',ix,iy,iz,'newvi',newvi)
            verts.append((ix,iy,iz))
            pos_list.append(newvi); neg_list.append(newvi)
    print('pos_list',pos_list,'neg_list',neg_list,'cop',cop_list)
    # compute area
    # reorder not implemented

print('try split face1 by plane0')
split_face(1,0)
