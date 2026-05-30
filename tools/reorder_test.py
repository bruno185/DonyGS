import math

def float_to_fixed(f): return int(round(f*65536.0))
def fixed_to_float(fx): return fx/65536.0

verts=[(-18.518536,-198.491813,0.0),
       (381.481464,-198.491813,0.0),
       (-18.518536,201.508187,0.0),
       (381.481464,201.508187,0.0),
       (-200.0,0.0,200.0),
       (200.0,0.0,200.0),
       (-200.0,0.0,-200.0),
       (200.0,0.0,-200.0),
       (381.481464,0.0,0.0),  # intersection 8
       (-18.518536,0.0,0.0)] # intersection 9

f1=0
pos=[3,8,2,9]

# compute face normal f1
p0=verts[1]; p1=verts[3]; p2=verts[2]
ux=(p1[0]-p0[0],p1[1]-p0[1],p1[2]-p0[2])
vx=(p2[0]-p0[0],p2[1]-p0[1],p2[2]-p0[2])
a=ux[1]*vx[2]-ux[2]*vx[1]
b=ux[2]*vx[0]-ux[0]*vx[2]
c=ux[0]*vx[1]-ux[1]*vx[0]
normal=(a,b,c)
print('normal',normal)

# reorder_poly imitation
abs_a,abs_b,abs_c=abs(a),abs(b),abs(c)
if abs_a>=abs_b and abs_a>=abs_c: proj=0
elif abs_b>=abs_a and abs_b>=abs_c: proj=1
else: proj=2
print('proj',proj)

px=[]; py=[]
for vi in pos:
    X,Y,Z=verts[vi]
    if proj==0:
        px.append(fixed_to_float(float_to_fixed(Y)))
        py.append(fixed_to_float(float_to_fixed(Z)))
    elif proj==1:
        px.append(fixed_to_float(float_to_fixed(X)))
        py.append(fixed_to_float(float_to_fixed(Z)))
    else:
        px.append(fixed_to_float(float_to_fixed(X)))
        py.append(fixed_to_float(float_to_fixed(Y)))
print('coords',list(zip(px,py)))

# centroid
cx=sum(px)/len(px); cy=sum(py)/len(py)
angles=[math.atan2(py[i]-cy, px[i]-cx) for i in range(len(px))]
print('angles',angles)

# insertion sort
lst=pos[:]
angs=angles[:]
for i in range(1,len(lst)):
    idx=lst[i]; a0=angs[i]
    j=i-1
    while j>=0 and angs[j] > a0:
        angs[j+1]=angs[j]; lst[j+1]=lst[j]; j-=1
    angs[j+1]=a0; lst[j+1]=idx
print('ordered',lst)

# orientation check area
area=0
for i in range(len(lst)):
    j=(i+1)%len(lst)
    area += px[pos.index(lst[i])]*py[pos.index(lst[j])] - px[pos.index(lst[j])]*py[pos.index(lst[i])]
print('area',area/2)
