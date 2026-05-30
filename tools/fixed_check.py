# replicate C behaviour (fixed-point and integer projections) for t.obj

def float_to_fixed(f):
    return int(round(f * 65536.0))

def fixed_to_int(fx):
    return fx >> 16

def point_in_poly_int(px, py, vx, vy):
    inside = False
    n = len(vx)
    for i in range(n):
        j = (i + 1) % n
        xi, yi = vx[i], vy[i]
        xj, yj = vx[j], vy[j]
        if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
    return inside

# vertices as in t.obj
vertices = [(-18.518536,-198.491813,0.0),
            (381.481464,-198.491813,0.0),
            (-18.518536,201.508187,0.0),
            (381.481464,201.508187,0.0),
            (-200.0,0.0,200.0),
            (200.0,0.0,200.0),
            (-200.0,0.0,-200.0),
            (200.0,0.0,-200.0)]
faces = [[1,3,2,0],[5,7,6,4]]

import math

def compute_plane(face):
    p0 = vertices[face[0]]
    p1 = vertices[face[1]]
    p2 = vertices[face[2]]
    ux = (p1[0]-p0[0], p1[1]-p0[1], p1[2]-p0[2])
    vx_ = (p2[0]-p0[0], p2[1]-p0[1], p2[2]-p0[2])
    # cross
    a = ux[1]*vx_[2]-ux[2]*vx_[1]
    b = ux[2]*vx_[0]-ux[0]*vx_[2]
    c = ux[0]*vx_[1]-ux[1]*vx_[0]
    d = -(a*p0[0] + b*p0[1] + c*p0[2])
    return a,b,c,d

plane_eps = 0.02

# mimic edge_intersects_face

def edge_intersects_face(ei, fj):
    a,b,c,d = compute_plane(faces[fj])
    # projection
    abs_a,abs_b,abs_c = abs(a),abs(b),abs(c)
    if abs_a >= abs_b and abs_a >= abs_c:
        proj = 0
    elif abs_b >= abs_a and abs_b >= abs_c:
        proj = 1
    else:
        proj = 2

    # build projected vertex arrays for face fj
    nfj = len(faces[fj])
    vx = []
    vy = []
    for m in range(nfj):
        vi = faces[fj][m]
        X,Y,Z = vertices[vi]
        ix = iy = 0
        if proj == 0:
            ix = fixed_to_int(float_to_fixed(Y))
            iy = fixed_to_int(float_to_fixed(Z))
        elif proj == 1:
            ix = fixed_to_int(float_to_fixed(X))
            iy = fixed_to_int(float_to_fixed(Z))
        else:
            ix = fixed_to_int(float_to_fixed(X))
            iy = fixed_to_int(float_to_fixed(Y))
        vx.append(ix); vy.append(iy)

    cnt = len(faces[ei])
    for k in range(cnt):
        vi = faces[ei][k]
        vj = faces[ei][(k+1) % cnt]
        Ax,Ay,Az = vertices[vi]
        Bx,By,Bz = vertices[vj]
        sdA = a*Ax + b*Ay + c*Az + d
        sdB = a*Bx + b*By + c*Bz + d
        if abs(sdA) < plane_eps: sdA = 0.0
        if abs(sdB) < plane_eps: sdB = 0.0
        print('  checking edge',k,'sdA',sdA,'sdB',sdB)
        if sdA * sdB < 0.0:
            t = -sdA / (sdB - sdA)
            Px = Ax + t*(Bx - Ax)
            Py = Ay + t*(By - Ay)
            Pz = Az + t*(Bz - Az)
            if proj == 0:
                px = fixed_to_int(float_to_fixed(Py)); py = fixed_to_int(float_to_fixed(Pz))
            elif proj == 1:
                px = fixed_to_int(float_to_fixed(Px)); py = fixed_to_int(float_to_fixed(Pz))
            else:
                px = fixed_to_int(float_to_fixed(Px)); py = fixed_to_int(float_to_fixed(Py))
            print('   intersection at',Px,Py,Pz,'proj',px,py)
            inside = point_in_poly_int(px, py, vx, vy)
            print('    inside?',inside)
            if inside:
                return True
    return False

print('edge 0->1',edge_intersects_face(0,1))
print('edge 1->0',edge_intersects_face(1,0))
