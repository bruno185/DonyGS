import math

def float_to_fixed(f): return int(round(f*65536.0))
def fixed_to_int(fx): return fx>>16

vertices=[(-18.518536,-198.491813,0.0),
          (381.481464,-198.491813,0.0),
          (-18.518536,201.508187,0.0),
          (381.481464,201.508187,0.0),
          (-200.0,0.0,200.0),
          (200.0,0.0,200.0),
          (-200.0,0.0,-200.0),
          (200.0,0.0,-200.0)]
faces=[[1,3,2,0],[5,7,6,4]]

plane_eps=0.02

def compute_plane(face):
    p0=vertices[face[0]]
    p1=vertices[face[1]]
    p2=vertices[face[2]]
    ux=(p1[0]-p0[0],p1[1]-p0[1],p1[2]-p0[2])
    vx_=(p2[0]-p0[0],p2[1]-p0[1],p2[2]-p0[2])
    a=ux[1]*vx_[2]-ux[2]*vx_[1]
    b=ux[2]*vx_[0]-ux[0]*vx_[2]
    c=ux[0]*vx_[1]-ux[1]*vx_[0]
    d=-(a*p0[0]+b*p0[1]+c*p0[2])
    return a,b,c,d

for ei,fj in [(0,1),(1,0)]:
    print('---',ei,'->',fj,'---')
    a,b,c,d=compute_plane(faces[fj])
    abs_a,abs_b,abs_c=abs(a),abs(b),abs(c)
    if abs_a>=abs_b and abs_a>=abs_c: proj=0
    elif abs_b>=abs_a and abs_b>=abs_c: proj=1
    else: proj=2
    print('proj',proj)
    vx=[]; vy=[]
    for v in faces[fj]:
        X,Y,Z=vertices[v]
        if proj==0: ix=fixed_to_int(float_to_fixed(Y)); iy=fixed_to_int(float_to_fixed(Z))
        elif proj==1: ix=fixed_to_int(float_to_fixed(X)); iy=fixed_to_int(float_to_fixed(Z))
        else: ix=fixed_to_int(float_to_fixed(X)); iy=fixed_to_int(float_to_fixed(Y))
        vx.append(ix); vy.append(iy)
    print('proj poly',list(zip(vx,vy)))
    cnt=len(faces[ei])
    found=False
    for k in range(cnt):
        vi=faces[ei][k]; vj=faces[ei][(k+1)%cnt]
        A=vertices[vi]; B=vertices[vj]
        sdA=a*A[0]+b*A[1]+c*A[2]+d
        sdB=a*B[0]+b*B[1]+c*B[2]+d
        if abs(sdA)<plane_eps: sdA=0.0
        if abs(sdB)<plane_eps: sdB=0.0
        print(' edge',k,'sdA',sdA,'sdB',sdB)
        if sdA*sdB<0:
            t=-sdA/(sdB-sdA)
            P=(A[0]+t*(B[0]-A[0]),A[1]+t*(B[1]-A[1]),A[2]+t*(B[2]-A[2]))
            if proj==0: px=fixed_to_int(float_to_fixed(P[1])); py=fixed_to_int(float_to_fixed(P[2]))
            elif proj==1: px=fixed_to_int(float_to_fixed(P[0])); py=fixed_to_int(float_to_fixed(P[2]))
            else: px=fixed_to_int(float_to_fixed(P[0])); py=fixed_to_int(float_to_fixed(P[1]))
            print('   P',P,'proj',px,py)
            inside=False
            for m in range(len(vx)):
                n=(m+1)%len(vx)
                xi,yi=vx[m],vy[m]; xj,yj=vx[n],vy[n]
                if ((yi>py)!=(yj>py)) and (px < (xj-xi)*(py-yi)/(yj-yi+1e-12)+xi):
                    inside=not inside
            print('    inside',inside)
            if inside:
                found=True
                break
    print(' result',found)
