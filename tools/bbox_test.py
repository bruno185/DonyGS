# compute bbox overlap with fixed arithmetic for t.obj

verts=[(-18.518536,-198.491813,0.0),
       (381.481464,-198.491813,0.0),
       (-18.518536,201.508187,0.0),
       (381.481464,201.508187,0.0),
       (-200.0,0.0,200.0),
       (200.0,0.0,200.0),
       (-200.0,0.0,-200.0),
       (200.0,0.0,-200.0)]
faces=[[1,3,2,0],[5,7,6,4]]

def tofix(f):
    return int(round(f*65536))

def get_bbox(face):
    xs=[]; ys=[]; zs=[]
    for vi in face:
        x,y,z=verts[vi]
        xs.append(tofix(x)); ys.append(tofix(y)); zs.append(tofix(z))
    return min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)

eps= int(round(0.02*65536))
for i in range(2):
    print('bbox',i,get_bbox(faces[i]))

i,j=0,1
minx_i,maxx_i,miny_i,maxy_i,minz_i,maxz_i = get_bbox(faces[i])
minx_j,maxx_j,miny_j,maxy_j,minz_j,maxz_j = get_bbox(faces[j])
ox = minx_i if minx_i>minx_j else minx_j
ux = maxx_i if maxx_i<maxx_j else maxx_j
oy = miny_i if miny_i>miny_j else miny_j
uy = maxy_i if maxy_i<maxy_j else maxy_j
oz = minz_i if minz_i>minz_j else minz_j
uz = maxz_i if maxz_i<maxz_j else maxz_j
print('ox,ux,oy,uy,oz,uz',ox,ux,oy,uy,oz,uz)
dx=(ux-ox); dy=(uy-oy); dz=(uz-oz)
print('dx dy dz',dx,dy,dz)
print('dx>=0?',dx>=0,'dy>=0?',dy>=0,'dz>=0?',dz>=0)
