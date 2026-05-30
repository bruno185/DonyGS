# replicate split_face_by_plane logic with fixed arithmetic to see why split might fail for t.obj

def float_to_fixed(f): return int(round(f*65536.0))
def fixed_to_float(fx): return fx/65536.0

vertices=[(-18.518536,-198.491813,0.0),
          (381.481464,-198.491813,0.0),
          (-18.518536,201.508187,0.0),
          (381.481464,201.508187,0.0),
          (-200.0,0.0,200.0),
          (200.0,0.0,200.0),
          (-200.0,0.0,-200.0),
          (200.0,0.0,-200.0)]
faces=[[1,3,2,0],[5,7,6,4]]

plane_eps = 0.02

import math

def compute_plane(face):
    p0=vertices[face[0]]
    p1=vertices[face[1]]
    p2=vertices[face[2]]
    ux=(p1[0]-p0[0],p1[1]-p0[1],p1[2]-p0[2])
    vx=(p2[0]-p0[0],p2[1]-p0[1],p2[2]-p0[2])
    a=ux[1]*vx[2]-ux[2]*vx[1]
    b=ux[2]*vx[0]-ux[0]*vx[2]
    c=ux[0]*vx[1]-ux[1]*vx[0]
    d=-(a*p0[0]+b*p0[1]+c*p0[2])
    return a,b,c,d

# mimic clip_face_plane behaviour to compute pos/neg lists exactly

def clip(face, a,b,c,d, keepPositive):
    cnt=len(face)
    out=[]
    prev_vi=face[-1]
    x,y,z=vertices[prev_vi]
    sd_prev=a*x+b*y+c*z+d
    for k in range(cnt):
        vi=face[k]
        x,y,z=vertices[vi]
        sd=a*x+b*y+c*z+d
        in_prev = (sd_prev >= 0) if keepPositive else (sd_prev <= 0)
        in_curr = (sd >= 0) if keepPositive else (sd <= 0)
        if in_curr:
            out.append(vi)
        if in_prev ^ in_curr:
            t = sd_prev / (sd_prev - sd)
            A=vertices[prev_vi]; B=vertices[vi]
            ix=A[0]+t*(B[0]-A[0])
            iy=A[1]+t*(B[1]-A[1])
            iz=A[2]+t*(B[2]-A[2])
            vertices.append((ix,iy,iz))
            newvi = len(vertices)-1
            out.append(newvi)
        prev_vi=vi
        sd_prev=sd
    return out

# run debug for splitting face 0 by face1 etc
for f1,f2 in [(0,1),(1,0)]:
    print('split candidate',f1,'by',f2)
    a,b,c,d=compute_plane(faces[f2])
    pos=clip(faces[f1],a,b,c,d,1)
    neg=clip(faces[f1],a,b,c,d,0)
    print(' pos',pos)
    print(' neg',neg)
    # compute coplanar
    cop=[]
    for vi in faces[f1]:
        x,y,z=vertices[vi]
        sd=a*x+b*y+c*z+d
        if abs(sd)<plane_eps: cop.append(vi)
    print(' cop',cop)
    # remove duplicates
    def clean(lst):
        out=[]
        for x in lst:
            if not out or out[-1]!=x: out.append(x)
        if len(out)>1 and out[0]==out[-1]: out.pop()
        return out
    # reorder polygons around centroid (similar to C reorder_poly)
    def reorder(listidx, faceidx):
        if len(listidx) < 3: return listidx
        # compute plane normal for faceidx
        aN,bN,cN,_ = compute_plane(faces[faceidx])
        abs_a,abs_b,abs_c = abs(aN), abs(bN), abs(cN)
        if abs_a>=abs_b and abs_a>=abs_c: proj=0
        elif abs_b>=abs_a and abs_b>=abs_c: proj=1
        else: proj=2
        pts2 = []
        for vi in listidx:
            X,Y,Z = vertices[vi]
            if proj==0: pts2.append((Y,Z))
            elif proj==1: pts2.append((X,Z))
            else: pts2.append((X,Y))
        # centroid
        cx = sum(p[0] for p in pts2)/len(pts2)
        cy = sum(p[1] for p in pts2)/len(pts2)
        angs = [math.atan2(p[1]-cy, p[0]-cx) for p in pts2]
        order = sorted(range(len(pts2)), key=lambda i: angs[i])
        reordered = [listidx[i] for i in order]
        # ensure positive orientation
        area2 = 0
        for i in range(len(order)):
            x1,y1 = pts2[order[i]]
            x2,y2 = pts2[order[(i+1)%len(pts2)]]
            area2 += x1*y2 - x2*y1
        if area2 < 0:
            reordered.reverse()
        return reordered
    pos = reorder(pos, f1)
    neg = reorder(neg, f1)
    pos=clean(pos); neg=clean(neg); cop=clean(cop)
    print(' cleaned pos',pos,'neg',neg,'cop',cop)
    print('counts',len(pos),len(neg))
    # compute areas
    def area(listidx,face_norm):
        # compute area projected according to the normal of the face being split
        if len(listidx)<3: return 0
        aN,bN,cN = face_norm
        abs_a,abs_b,abs_c=abs(aN),abs(bN),abs(cN)
        if abs_a>=abs_b and abs_a>=abs_c: proj=0
        elif abs_b>=abs_a and abs_b>=abs_c: proj=1
        else: proj=2
        coords=[]
        for vi in listidx:
            X,Y,Z=vertices[vi]
            if proj==0: coords.append((Y,Z))
            elif proj==1: coords.append((X,Z))
            else: coords.append((X,Y))
        ar=0
        for i in range(len(coords)):
            x1,y1=coords[i]
            x2,y2=coords[(i+1)%len(coords)]
            ar += x1*y2 - x2*y1
        return ar*0.5
    # compute face1 normal for area tests
    a1,b1,c1,_ = compute_plane(faces[f1])
    print('areas',area(pos,(a1,b1,c1)),area(neg,(a1,b1,c1)))
    # area test
def area(listidx,a,b,c):
    if len(listidx)<3: return 0
    abs_a,abs_b,abs_c=abs(a),abs(b),abs(c)
    if abs_a>=abs_b and abs_a>=abs_c: proj=0
    elif abs_b>=abs_a and abs_b>=abs_c: proj=1
    else: proj=2
    coords=[]
    for vi in listidx:
        X,Y,Z=vertices[vi]
        if proj==0: coords.append((Y,Z))
        elif proj==1: coords.append((X,Z))
        else: coords.append((X,Y))
    ar=0
    for i in range(len(coords)):
        x1,y1=coords[i]
        x2,y2=coords[(i+1)%len(coords)]
        ar += x1*y2 - x2*y1
    return ar*0.5

    print('areas',area(pos,a,b,c),area(neg,a,b,c))
