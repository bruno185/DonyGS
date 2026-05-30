# compute area using same logic as poly_signed_area in C (fixed->float projection)

def float_to_fixed(f): return int(round(f * 65536.0))
def fixed_to_float(fx): return fx / 65536.0

# vertices of t.obj after intersections added earlier
# we will reconstruct from split_debug earlier: global vertices initial plus two intersections
verts=[(-18.518536,-198.491813,0.0),
       (381.481464,-198.491813,0.0),
       (-18.518536,201.508187,0.0),
       (381.481464,201.508187,0.0),
       (-200.0,0.0,200.0),
       (200.0,0.0,200.0),
       (-200.0,0.0,-200.0),
       (200.0,0.0,-200.0),
       (381.481464,0.0,0.0),  # index 8
       (-18.518536,0.0,0.0)] # index 9

# face0 pos_list from earlier: [3,8,2,9]
pos=[3,8,2,9]
neg=[1,8,9,0]

import math

def poly_signed_area(list_idx, normal):
    a,b,c = normal
    if len(list_idx) < 3: return 0.0
    abs_a,abs_b,abs_c = abs(a),abs(b),abs(c)
    if abs_a>=abs_b and abs_a>=abs_c: proj=0
    elif abs_b>=abs_a and abs_b>=abs_c: proj=1
    else: proj=2
    area=0.0
    for i in range(len(list_idx)):
        vi=list_idx[i]
        vj=list_idx[(i+1)%len(list_idx)]
        X1,Y1,Z1=verts[vi]
        X2,Y2,Z2=verts[vj]
        if proj==0:
            x1=fixed_to_float(float_to_fixed(Y1)); y1=fixed_to_float(float_to_fixed(Z1))
            x2=fixed_to_float(float_to_fixed(Y2)); y2=fixed_to_float(float_to_fixed(Z2))
        elif proj==1:
            x1=fixed_to_float(float_to_fixed(X1)); y1=fixed_to_float(float_to_fixed(Z1))
            x2=fixed_to_float(float_to_fixed(X2)); y2=fixed_to_float(float_to_fixed(Z2))
        else:
            x1=fixed_to_float(float_to_fixed(X1)); y1=fixed_to_float(float_to_fixed(Y1))
            x2=fixed_to_float(float_to_fixed(X2)); y2=fixed_to_float(float_to_fixed(Y2))
        area += (x1 * y2 - x2 * y1)
    return area * 0.5

# compute normal of face0 (face being split)
a0 = (verts[3][1]-verts[0][1])*(verts[2][2]-verts[0][2]) - (verts[3][2]-verts[0][2])*(verts[2][1]-verts[0][1])
b0 = (verts[3][2]-verts[0][2])*(verts[2][0]-verts[0][0]) - (verts[3][0]-verts[0][0])*(verts[2][2]-verts[0][2])
c0 = (verts[3][0]-verts[0][0])*(verts[2][1]-verts[0][1]) - (verts[3][1]-verts[0][1])*(verts[2][0]-verts[0][0])

print('normal f0',a0,b0,c0)
print('area pos',poly_signed_area(pos,(a0,b0,c0)))
print('area neg',poly_signed_area(neg,(a0,b0,c0)))
