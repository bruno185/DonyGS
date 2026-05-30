import numpy as np

# face1 (z=0 quad)
f1=[np.array([-18.518536,-198.491813,0.0]),
    np.array([381.481464,-198.491813,0.0]),
    np.array([-18.518536,201.508187,0.0]),
    np.array([381.481464,201.508187,0.0])]
# face2 (y=0 quad)
f2=[np.array([-200.0,0.0,200.0]),
    np.array([200.0,0.0,200.0]),
    np.array([-200.0,0.0,-200.0]),
    np.array([200.0,0.0,-200.0])]

# plane of f2
norm=np.cross(f2[1]-f2[0],f2[2]-f2[0])
d=-np.dot(norm,f2[0])
print('norm,d',norm,d)
for i,p in enumerate(f1):
    sd=np.dot(norm,p)+d
    print('vertex',i,'sd',sd)
