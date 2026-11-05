# whole-body interference check: pelvis-v1.2 (minus thigh stubs) + legs-v1. Overlaps > 0.5 cm³ reported.
# Ignored pairs: designed contacts (rod↔foot ball-joint ears, cross↔foot clevis/shin, actuator↔its own plates).
import json, itertools, cadquery as cq
PLy=json.load(open('../pelvis-v1/layout.json')); LL=json.load(open('layout.json'))
J=dict(PLy['joints']); J.update(LL['joints'])
P={};PAR={}
for n,p in PLy['parts'].items():
    if n.startswith('thigh_stub') or p['parent']=='torso': continue
    P[n]=cq.importers.importStep('../pelvis-v1/'+n+'.step').val(); PAR[n]=p['parent']
for n,p in LL['parts'].items():
    P[n]=cq.importers.importStep(n+'.step').val(); PAR[n]=p['parent']
ORDER=['aroll','apitch','knee','pitch','roll','yaw']
JN={'aroll':'ankle_roll','apitch':'ankle_pitch','knee':'knee','pitch':'hip_pitch','roll':'hip_roll','yaw':'hip_yaw'}
AX={'aroll':(1,0,0),'apitch':(0,1,0),'knee':(0,1,0),'pitch':(0,1,0),'roll':(1,0,0),'yaw':(0,0,1)}
def ang(k,q,sg):
    v=q.get(k,0); return {'yaw':sg*v,'roll':sg*v,'pitch':-v,'knee':v,'apitch':v,'aroll':sg*v}[k]
def pose(n,Q):
    par=PAR[n]
    if '_' not in par: return P[n]
    k,side=par.rsplit('_',1); s=P[n]; q=Q.get(side,{}); sg=1 if side=='L' else -1
    for c in ORDER[ORDER.index(k):]:
        j=J[f'{JN[c]}_{side}']; a=ang(c,q,sg)
        if a: s=s.rotate(cq.Vector(*j),cq.Vector(*j)+cq.Vector(*AX[c]),a)
    return s
def ok_pair(a,b):
    sa=a[-1] if a[-2]!='_' else a[-1]
    t={a.split('_')[0]+'_'+a.split('_')[1] if a.startswith('ankle') else a.split('_')[0], b.split('_')[0]+'_'+b.split('_')[1] if b.startswith('ankle') else b.split('_')[0]}
    return t in ({'ankle_rods','foot'},{'ankle_cross','foot'},{'ankle_cross','shin'},{'fsr','foot'})
def both(q): return {'L':q,'R':q}
POSES=[('neutral',{}),('walk swing L: hip30 knee60 / R stance push-off ankle20',{'L':{'pitch':30,'knee':60},'R':{'pitch':-10,'apitch':20}}),
 ('knee90',both({'knee':90})),('knee120 hip60 ankle-25 (squat)',both({'pitch':60,'knee':120,'apitch':-25})),
 ('ankle pitch +25',both({'apitch':25})),('ankle pitch -25',both({'apitch':-25})),('ankle roll ±15',{'L':{'aroll':15},'R':{'aroll':-15}}),
 ('hip roll 20 + ankle roll -20',both({'roll':20,'aroll':-20})),('hip roll -15 (adduct) L only',{'L':{'roll':-15}}),
 ('hip yaw 30 + knee 60 L',{'L':{'yaw':30,'knee':60}}),('hip yaw -30 L / +30 R',{'L':{'yaw':-30},'R':{'yaw':30}}),('hip -30 knee 0',both({'pitch':-30}))]
allok=True
for name,Q in POSES:
    S={n:pose(n,Q) for n in P}; hits=[]
    for a,b in itertools.combinations(S,2):
        if ok_pair(a,b) or (PAR[a]==PAR[b] and not a.startswith('ankle') and False): continue
        bb=S[a].BoundingBox();cc=S[b].BoundingBox()
        if bb.xmax<cc.xmin or cc.xmax<bb.xmin or bb.ymax<cc.ymin or cc.ymax<bb.ymin or bb.zmax<cc.zmin or cc.zmax<bb.zmin:continue
        v=S[a].intersect(S[b]).Volume()/1000
        if v>0.5:hits.append(f'{a}×{b}:{v:.1f}cm³')
    allok&=not hits; print(name,'→',hits or 'OK')
print('ALL OK' if allok else 'HITS')
