from pathlib import Path
import re,math,struct,json,csv
ROOT=Path(__file__).resolve().parents[2]
S=65536;f32=lambda x:struct.unpack('f',struct.pack('f',x))[0];mul=lambda a,b:a*b>>16
src=(ROOT/'declare.c').read_text(encoding='utf-8-sig');tables={n:list(map(int,re.findall(r'-?\d+',re.search(n+r'\[360\] = \{(.*?)\};',src,re.S)[1]))) for n in ['sin_table','cos_table']}
def rotate(v):
 sh,ch=tables['sin_table'][30],tables['cos_table'][30];sv,cv=tables['sin_table'][20],tables['cos_table'][20];x,y,z=v
 return -mul(x,sh)+mul(y,ch),-mul(x,mul(ch,sv))-mul(y,mul(sh,sv))+mul(z,cv),-mul(x,mul(ch,cv))-mul(y,mul(sh,cv))-mul(z,sv)
def load(p):
 vs=[];fs=[]
 for l in p.read_text(errors='replace').splitlines():
  if l.startswith('v '):
   x,y,z=map(float,l.split()[1:4]);vs.append(tuple(int(f32(f32(t)*S)) for t in (x,z,y)))
  elif l.startswith('f '):fs.append([int(t.split('/')[0])-1 for t in l.split()[1:17] if int(t.split('/')[0])>0])
 mn=[min(v[i] for v in vs) for i in range(3)];mx=[max(v[i] for v in vs) for i in range(3)];vs=[tuple(v[i]-int((mn[i]+mx[i])/2) for i in range(3)) for v in vs];D=max(b-a for a,b in zip(mn,mx))*3
 temp=[rotate(v) for v in vs];xy=[(f32(f32(x)/f32(z+D)),f32(f32(y)/f32(z+D))) for x,y,z in temp if z+D>0]
 spans=[max(p[i] for p in xy)-min(p[i] for p in xy) for i in range(2)]
 proj=min(f32(f32(320*f32(.9))/f32(spans[0])),f32(f32(200*f32(.9))/f32(spans[1]))) if all(t>0 for t in spans) else 100
 P=int(f32(proj*S));factor=int(f32(f32(math.sqrt(f32(150*S/D)))*S));vs=[tuple(mul(v,factor) for v in q) for q in vs];P=mul(P,factor)
 obs=[(x,y,z+150*S) for x,y,z in [rotate(v) for v in vs]]
 rnd=lambda x:(x+(32768 if x>=0 else -32768))>>16
 pts=[]
 for x,y,z in obs:
  inv=int((P<<16)/z);pts.append((rnd(160*S+mul(x,inv)),rnd(100*S-mul(y,inv))))
 metrics=[]
 for ids in fs:
  zz=[obs[i][2] for i in ids];xx=[pts[i][0] for i in ids];yy=[pts[i][1] for i in ids]
  aa=bb=cc=0
  for i,j in zip(ids,ids[1:]+ids[:1]):
   x,y,z=obs[i];u,v,w=obs[j];aa+=(y-v)*(z+w);bb+=(z-w)*(x+u);cc+=(x-u)*(y+v)
  aa>>=16;bb>>=16;cc>>=16;cent=[int(sum(obs[i][k] for i in ids)/len(ids)) for k in range(3)];dd=-(aa*cent[0]+bb*cent[1]+cc*cent[2])>>16
  metrics.append(dict(zmin=min(zz),zmax=max(zz),zmean=int(sum(zz)/len(zz)),minx=min(xx),maxx=max(xx),miny=min(yy),maxy=max(yy),plane=[aa,bb,cc,dd]))
 return dict(vertices=vs,faces=fs,observer=obs,points=pts,metrics=metrics,scale=P,distance=150*S)
if __name__=='__main__':
 out=[];directory=Path(__file__).resolve().parent;directory.mkdir(parents=True,exist_ok=True)
 for name in ['a','gear2.obj','other03.obj','ring.obj','star.obj','other01.obj','monkey.obj','stool.obj','tore.obj']:
  p=(ROOT/'models')/name
  if not p.exists():continue
  d=load(p);m=d['metrics'];order=sorted(range(len(m)),key=lambda i:(m[i]['zmin'],-m[i]['zmean'],i));active=[];candidates=xy=0;maxactive=0
  for f in order:
   active=[q for q in active if m[q]['zmax']>m[f]['zmin']];candidates+=len(active);maxactive=max(maxactive,len(active))
   for q in active:
    if not(m[f]['maxx']<=m[q]['minx'] or m[q]['maxx']<=m[f]['minx'] or m[f]['maxy']<=m[q]['miny'] or m[q]['maxy']<=m[f]['miny']):xy+=1
   active.append(f)
  row=dict(model=name,vertices=len(d['vertices']),faces=len(m),z_candidates=candidates,xyz_bbox_candidates=xy,max_active=maxactive,capacity_needed=2**max(6,(max(1,candidates)-1).bit_length()),cache4096_exceeded=candidates>4096)
  out.append(row);(directory/(name.replace('.obj','')+'_geometry.json')).write_text(json.dumps(d),encoding='utf-8')
 print(json.dumps(out,indent=2));(directory/'candidats.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
