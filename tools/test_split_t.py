import math

# read t.obj
verts=[]
faces=[]
with open(r'f:\Bruno\Dev\AppleWin\Projets\ORCA\OBJ Viewer Review\DonyGS\t.obj') as f:
    for line in f:
        if line.startswith('v '):
            parts=line.split()
            verts.append(tuple(float(x) for x in parts[1:4]))
        if line.startswith('f '):
            parts=line.split()
            # ignore texture/normal indices
            idxs=[int(x.split('/')[0]) - 1 for x in parts[1:]]
            faces.append(idxs)

print('loaded',len(verts),'verts',len(faces),'faces')

# new algorithm replication

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


def split_face(f1,f2,constraint=None):
    a,b,c,d = plane_from_face(faces[f2])
    cnt=len(faces[f1])
    dist=[]
    eps=0.02
    for vi in faces[f1]:
        x,y,z=verts[vi]
        sd = a*x+b*y+c*z+d
        if abs(sd) < eps: sd = 0.0
        dist.append(sd)
    print('face',f1,'dists',dist)
    hasPos = any(s>0 for s in dist)
    hasNeg = any(s<0 for s in dist)
    print('hasPos',hasPos,'hasNeg',hasNeg)
    if not hasPos or not hasNeg:
        print('no split due to one-sided')
        return False
    pos_list=[]; neg_list=[]; cop_list=[]

    # prepare constraint projection if given
    have_constraint = False
    seg_min = seg_max = 0.0
    dir_vec = (0,0,0)
    refpt = (0,0,0)
    if constraint and len(constraint) >= 2:
        # direction = cross(n_cut, n_face)
        a1,b1,c1,d1 = plane_from_face(faces[f1])
        # normals:
        n_cut = (a,b,c)
        n_face = (a1,b1,c1)
        dir_vec = (n_cut[1]*n_face[2] - n_cut[2]*n_face[1],
                   n_cut[2]*n_face[0] - n_cut[0]*n_face[2],
                   n_cut[0]*n_face[1] - n_cut[1]*n_face[0])
        norm = math.sqrt(dir_vec[0]**2 + dir_vec[1]**2 + dir_vec[2]**2)
        if norm > 0:
            dir_vec = (dir_vec[0]/norm, dir_vec[1]/norm, dir_vec[2]/norm)
            refpt = verts[constraint[0]]
            for ci in constraint:
                cx,cy,cz = verts[ci]
                t = ( (cx-refpt[0])*dir_vec[0] + (cy-refpt[1])*dir_vec[1] + (cz-refpt[2])*dir_vec[2] )
                if ci == constraint[0] or t < seg_min: seg_min = t
                if ci == constraint[0] or t > seg_max: seg_max = t
            have_constraint = True

    for i in range(cnt):
        j=(i+1)%cnt
        di=dist[i]; dj=dist[j]
        vi=faces[f1][i]; vj=faces[f1][j]
        if di==0.0: cop_list.append(vi)
        if di>=0: pos_list.append(vi)
        if di<=0: neg_list.append(vi)
        if di*dj < 0.0:
            t = di/(di-dj)
            x1,y1,z1=verts[vi]
            x2,y2,z2=verts[vj]
            ix=x1+t*(x2-x1)
            iy=y1+t*(y2-y1)
            iz=z1+t*(z2-z1)
            if have_constraint:
                pt = ((ix-refpt[0])*dir_vec[0] + (iy-refpt[1])*dir_vec[1] + (iz-refpt[2])*dir_vec[2])
                if pt < seg_min - 1e-6 or pt > seg_max + 1e-6:
                    print('  skipping intersection outside constraint interval',pt,seg_min,seg_max)
                    continue
            verts.append((ix,iy,iz))
            newvi=len(verts)-1
            print(' intersection at',ix,iy,iz,'-> newvi',newvi)
            pos_list.append(newvi); neg_list.append(newvi)
    print('pos_list',pos_list,'neg_list',neg_list,'cop',cop_list)
    return True

for i in range(len(faces)):
    for j in range(i+1,len(faces)):
        print('checking pair',i,j)
        # first split f1 by plane of f2, capturing intersection vertices
        newpts = []
        res1 = split_face(i,j)
        if res1:
            newpts = list(range(len(verts)-2, len(verts)))
            print('   new intersection vertices', newpts)
        # propagate points into face j (simple bbox/point-in-poly check)
        for vi in newpts:
            x,y,z = verts[vi]
            def point_in_poly(px,py,face):
                inside=False
                n=len(face)
                for m in range(n):
                    n2=(m+1)%n
                    xi,yi = verts[face[m]][0], verts[face[m]][1]
                    xj,yj = verts[face[n2]][0], verts[face[n2]][1]
                    if ((yi>py)!=(yj>py)) and (px < (xj-xi)*(py-yi)/(yj-yi+1e-12)+xi):
                        inside = not inside
                return inside
            if point_in_poly(x,y,faces[j]):
                faces[j].append(vi)
                print('   appended vertex',vi,'to face',j)
        # now split other face, passing constraint vertices from the first split
        res2 = split_face(j,i,constraint=newpts if res1 else None)
        if res2:
            print('  performed split on',j,'by',i,'(reversed)')
        if res1:
            print('  performed split on',i,'by',j)
