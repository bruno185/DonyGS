import math

# parse inter.obj
verts=[]
faces=[]
with open(r'f:\Bruno\Dev\AppleWin\Projets\ORCA\OBJ Viewer Review\DonyGS\inter.obj') as f:
    for line in f:
        if line.startswith('v '):
            parts=line.split()
            verts.append(tuple(float(x) for x in parts[1:4]))
        if line.startswith('f '):
            parts=line.split()
            faces.append([int(x)-1 for x in parts[1:]])

print('vertices',verts)
print('faces',faces)

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

# run splitting logic like C

def split_face(f1,f2):
    a,b,c,d=plane_from_face(faces[f2])
    cnt=len(faces[f1])
    dist=[]
    for vi in faces[f1]:
        x,y,z=verts[vi]
        sd=a*x+b*y+c*z+d
        dist.append(sd)
    print(f"face {f1} dist vs plane f{f2}: {dist}")
    pos_exist=any(s>0 for s in dist)
    neg_exist=any(s<0 for s in dist)
    if not pos_exist or not neg_exist:
        return False
    pos_list=[]; neg_list=[]; cop_list=[]
    eps=0.02
    for k in range(cnt):
        k2 = (k + 1) % cnt
        vi = faces[f1][k]
        sd = dist[k]
        sd2 = dist[k2]
        # add vertex to appropriate side(s)
        if sd >= 0: pos_list.append(vi)
        if sd <= 0: neg_list.append(vi)
        if abs(sd) <= eps: cop_list.append(vi)
        # crossing edge -> insert intersection
        if sd * sd2 < 0:
            t = sd / (sd - sd2)
            vj = faces[f1][k2]
            x1,y1,z1 = verts[vi]
            x2,y2,z2 = verts[vj]
            ix = x1 + t*(x2 - x1)
            iy = y1 + t*(y2 - y1)
            iz = z1 + t*(z2 - z1)
            verts.append((ix,iy,iz))
            newvi = len(verts) - 1
            print('   created intersection',newvi,'coord',(ix,iy,iz),'from edge',vi,'->',vj)
            pos_list.append(newvi); neg_list.append(newvi); cop_list.append(newvi)
    def clean(arr):
        out=[]
        for e in arr:
            if not out or out[-1]!=e: out.append(e)
        if len(out)>1 and out[0]==out[-1]: out.pop()
        return out
    pos_list=clean(pos_list)
    neg_list=clean(neg_list)
    cop_list=clean(cop_list)
    # compute area like C
    def plane_from_face(face):
        p0=verts[face[0]]
        p1=verts[face[1]]
        p2=verts[face[2]]
        ux=[p1[i]-p0[i] for i in range(3)]
        vx=[p2[i]-p0[i] for i in range(3)]
        a = ux[1]*vx[2]-ux[2]*vx[1]
        b = ux[2]*vx[0]-ux[0]*vx[2]
        c = ux[0]*vx[1]-ux[1]*vx[0]
        return a,b,c
    def poly_area(list_idx, a,b,c):
        n=len(list_idx)
        if n<3: return 0
        abs_a,abs_b,abs_c=abs(a),abs(b),abs(c)
        if abs_a>=abs_b and abs_a>=abs_c: proj=0
        elif abs_b>=abs_a and abs_b>=abs_c: proj=1
        else: proj=2
        pts=[]
        for vi in list_idx:
            X,Y,Z=verts[vi]
            if proj==0: pts.append((Y,Z))
            elif proj==1: pts.append((X,Z))
            else: pts.append((X,Y))
        area=0
        for i in range(n):
            x1,y1=pts[i]
            x2,y2=pts[(i+1)%n]
            area += x1*y2 - x2*y1
        return area*0.5
    # skip reorder step to mirror C behaviour
    # pos_list = reorder(pos_list)
    # neg_list = reorder(neg_list)
    # cop_list = reorder(cop_list)
        if len(list_idx) < 3: return list_idx
        # projection plane same as face f1; compute normal
        a,b,c,d = plane_from_face(faces[f1])
        abs_a,abs_b,abs_c = abs(a),abs(b),abs(c)
        if abs_a >= abs_b and abs_a >= abs_c: proj=0
        elif abs_b >= abs_a and abs_b >= abs_c: proj=1
        else: proj=2
        pts = []
        for vi in list_idx:
            X,Y,Z = verts[vi]
            if proj == 0: pts.append((Y,Z))
            elif proj == 1: pts.append((X,Z))
            else: pts.append((X,Y))
        cx = sum(p[0] for p in pts)/len(pts)
        cy = sum(p[1] for p in pts)/len(pts)
        angs = [math.atan2(p[1]-cy, p[0]-cx) for p in pts]
        order = sorted(range(len(pts)), key=lambda i: angs[i])
        reordered = [list_idx[i] for i in order]
        # check signed area
        area = 0
        for i in range(len(reordered)):
            x1,y1 = pts[order[i]]
            x2,y2 = pts[order[(i+1)%len(pts)]]
            area += x1*y2 - x2*y1
        if area < 0:
            reordered.reverse()
        return reordered
    # skip reorder step to mirror C behaviour
    # pos_list = reorder(pos_list)
    # neg_list = reorder(neg_list)
    # cop_list = reorder(cop_list)
    print('split',f1,'by',f2,'pos',pos_list,'neg',neg_list,'cop',cop_list)
    # check orientation in XY of positive polygon
    if pos_list:
        coords = [verts[v] for v in pos_list]
        area = 0
        for p,q in zip(coords, coords[1:]+coords[:1]):
            area += (p[0]*q[1] - q[0]*p[1])
        print('  pos poly signed area', area/2.0)
    if neg_list:
        coords = [verts[v] for v in neg_list]
        area = 0
        for p,q in zip(coords, coords[1:]+coords[:1]):
            area += (p[0]*q[1] - q[0]*p[1])
        print('  neg poly signed area', area/2.0)
    if cop_list:
        coords = [verts[v] for v in cop_list]
        area = 0
        for p,q in zip(coords, coords[1:]+coords[:1]):
            area += (p[0]*q[1] - q[0]*p[1])
        print('  cop poly signed area', area/2.0)
    return True

for i in range(len(faces)):
    for j in range(i+1,len(faces)):
        print(f"calling split_face({i},{j})")
        res = split_face(i,j)
        print(f"  returned {res}")
