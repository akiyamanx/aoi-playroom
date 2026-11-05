# interference check: neutral + posed (PLACEHOLDER geometry). Reports overlaps > 0.5 cm³
import json, itertools, cadquery as cq
L=json.load(open('layout.json'));J=L['joints']
def load(n): return cq.importers.importStep(n+'.step').val()
P={n:load(n) for n in L['parts']}
def pose(n,q):
    par=L['parts'][n]['parent'];s=P[n]
    if par in('torso',):return s
    s=s  # apply chain from distal to proximal
    chain=[];side=par[-1]
    if par.startswith('pitch'):chain=['pitch','roll','yaw']
    elif par.startswith('roll'):chain=['roll','yaw']
    elif par.startswith('yaw'):chain=['yaw']
    sg=1 if side=='L' else -1
    for c in chain:
        j=J['hip_%s_%s'%(c,side)];ax={'yaw':(0,0,1),'roll':(1,0,0),'pitch':(0,1,0)}[c]
        a={'yaw':sg*q.get('yaw',0),'roll':sg*q.get('roll',0),'pitch':-q.get('pitch',0)}[c]
        s=s.rotate(cq.Vector(*j),cq.Vector(*j)+cq.Vector(*ax),a)
    return s
for q in [{},{'pitch':90},{'pitch':-30},{'roll':30},{'roll':-20},{'yaw':30,'pitch':45},{'roll':-20,'yaw':-30}]:
    S={n:pose(n,q) for n in P};hits=[]
    for a,b in itertools.combinations(S,2):
        bb=S[a].BoundingBox();cc=S[b].BoundingBox()
        if bb.xmax<cc.xmin or cc.xmax<bb.xmin or bb.ymax<cc.ymin or cc.ymax<bb.ymin or bb.zmax<cc.zmin or cc.zmax<bb.zmin:continue
        v=S[a].intersect(S[b]).Volume()/1000
        if v>0.5:hits.append(f'{a}×{b}:{v:.1f}cm³')
    print(q or 'neutral',hits or 'OK')
