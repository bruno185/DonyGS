import numpy as np

f1=[np.array([-18.518536,-198.491813,0.0]),
    np.array([381.481464,-198.491813,0.0]),
    np.array([-18.518536,201.508187,0.0]),
    np.array([381.481464,201.508187,0.0])]

f2=[np.array([-200.0,0.0,200.0]),
    np.array([200.0,0.0,200.0]),
    np.array([-200.0,0.0,-200.0]),
    np.array([200.0,0.0,-200.0])]

def plane(pa,pb,pc):
    ux=pb-pa; vx=pc-pa
    norm=np.cross(ux,vx)
    d=-np.dot(norm,pa)
    return norm,d

for (name,face,other) in [('f1','f1','f2'),('f2','f2','f1')]:
    if name=='f1': pts=f1; oth=f2
    else: pts=f2; oth=f1
    norm,d=plane(oth[1],oth[3],oth[2])
    print(f"plane of {other} norm,d {norm},{d}")
    for i,p in enumerate(pts):
        sd=np.dot(norm,p)+d
        print(' ',name,i,'sd',sd)
