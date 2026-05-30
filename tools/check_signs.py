import numpy as np
v0=np.array([-0.5,-1.235901,11.528229])
v1=np.array([0.0,0.0,12.0])
v2=np.array([-0.5,0.643494,10.844177])
v3=np.array([0.0,0.0,12.0])
face0=[v0,v1,v2,v3]
v5=np.array([-1.482056,0.020264,13.517624])
v6=np.array([1.482056,-0.959961,10.824432])
v7=np.array([1.482056,-0.020264,10.482407])
v8=np.array([-1.482056,0.959961,13.175598])
face1=[v5,v6,v7,v8]
norm=np.cross(face1[1]-face1[0],face1[2]-face1[0])
d=-np.dot(norm,face1[0])
print('plane norm,d',norm,d)
for idx,pt in enumerate(face0):
    sd=np.dot(norm,pt)+d
    print('sd',idx,sd)
print('pos exists?',any(np.dot(norm,pt)+d>=0 for pt in face0))
print('neg exists?',any(np.dot(norm,pt)+d<=0 for pt in face0))
