import numpy as np

# same coordinates as before
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

# compute plane for face1
norm=np.cross(face1[1]-face1[0], face1[2]-face1[0])
d=-np.dot(norm, face1[0])
print('plane1 norm,d',norm,d)

# function to test segment-plane intersection and if intersection inside polygon

def point_in_poly(px, py, verts):
    # ray casting
    inside = False
    n = len(verts)
    for i in range(n):
        j = (i + 1) % n
        xi, yi = verts[i]
        xj, yj = verts[j]
        if ((yi > py) != (yj > py)) and \
           (px < (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
    return inside

# projection plane choose by largest normal component
a, b, c = norm
abs_vals = np.abs(norm)
proj = np.argmax(abs_vals)

print('proj',proj)

# build projected vertices for face1
proj_1 = []
for p in face1:
    if proj == 0:
        proj_1.append((p[1],p[2]))
    elif proj == 1:
        proj_1.append((p[0],p[2]))
    else:
        proj_1.append((p[0],p[1]))

print('face1 proj',proj_1)

plane_eps = 0.02

# check edges of face0 against plane of face1
for i in range(len(face0)):
    A = face0[i]
    B = face0[(i+1) % len(face0)]
    sdA = np.dot(norm, A) + d
    sdB = np.dot(norm, B) + d
    if abs(sdA) < plane_eps: sdA = 0.0
    if abs(sdB) < plane_eps: sdB = 0.0
    print('edge',i,'sdA',sdA,'sdB',sdB)
    if sdA * sdB < 0:
        t = -sdA / (sdB - sdA)
        P = A + t*(B-A)
        if proj == 0:
            px, py = P[1], P[2]
        elif proj == 1:
            px, py = P[0], P[2]
        else:
            px, py = P[0], P[1]
        inside = point_in_poly(px, py, proj_1)
        print(' intersection at',P,'proj',px,py,'inside?',inside)

# also check reversed

print('---- check edges of face1 against plane of face0 ----')
norm0 = np.cross(face0[1]-face0[0], face0[2]-face0[0])
d0 = -np.dot(norm0, face0[0])
print('plane0 norm,d',norm0,d0)
abs_vals0 = np.abs(norm0)
proj0 = np.argmax(abs_vals0)
print('proj0',proj0)
proj_0 = []
for p in face0:
    if proj0 == 0:
        proj_0.append((p[1],p[2]))
    elif proj0 == 1:
        proj_0.append((p[0],p[2]))
    else:
        proj_0.append((p[0],p[1]))
print('face0 proj',proj_0)

for i in range(len(face1)):
    A = face1[i]
    B = face1[(i+1) % len(face1)]
    sdA = np.dot(norm0, A) + d0
    sdB = np.dot(norm0, B) + d0
    if abs(sdA) < plane_eps: sdA = 0.0
    if abs(sdB) < plane_eps: sdB = 0.0
    print('edge1',i,'sdA',sdA,'sdB',sdB)
    if sdA * sdB < 0:
        t = -sdA / (sdB - sdA)
        P = A + t*(B-A)
        if proj0 == 0:
            px, py = P[1], P[2]
        elif proj0 == 1:
            px, py = P[0], P[2]
        else:
            px, py = P[0], P[1]
        inside = point_in_poly(px, py, proj_0)
        print(' intersection at',P,'proj',px,py,'inside?',inside)
