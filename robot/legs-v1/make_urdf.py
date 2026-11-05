# full-body URDF: torso(+arms lumped) + pelvis-v1.2 + legs-v1. Dummy inertia tensors; masses/CoMs from build results. PLACEHOLDER.
import json
PL=json.load(open('../pelvis-v1/layout.json'));PR=json.load(open('../pelvis-v1/results.json'))['parts']
LL=json.load(open('layout.json'));LR=json.load(open('results.json'))['parts']
J=dict(PL['joints']);J.update(LL['joints'])
R={k:dict(v,path='../pelvis-v1/'+k+'.stl',parent=PL['parts'][k]['parent']) for k,v in PR.items() if not k.startswith('thigh_stub')}
for k,v in LR.items(): R[k]=dict(v,path=(k+'.stl') if not k.startswith('hardware') else None)
WJ=json.load(open('waist.json'));DZ=WJ['summary']['params']['DZ'];ZP=WJ['summary']['params']['ZP']
for k,v in R.items():
    if v['parent']=='torso': v['parent']='wyaw'   # waist adapter + yaw servo now ride the yaw (v1.2)
for k,v in WJ['parts'].items(): R[k]=dict(v,path=None if k.endswith('_hw') else k+'.stl')
m=lambda v:' '.join(f'{x/1000:.4f}' for x in v)
fr={'torso':[0,0,ZP],'wpitch':[0,0,ZP],'wyaw':J['waist'],'pelvis':J['waist']}
for s in 'LR':
    for f,j in (('yaw','hip_yaw'),('roll','hip_roll'),('pitch','hip_pitch'),('knee','knee'),('apitch','ankle_pitch'),('aroll','ankle_roll')): fr[f+'_'+s]=J[j+'_'+s]
out=['<?xml version="1.0"?>','<!-- cocomi full body: torso-v1.3-low-nobatt + arms (lumped) + pelvis-v1.2 + legs-v1.1 + waist pitch (legs-v1.2), root=pelvis. PLACEHOLDER dims/masses, dummy inertia. -->','<robot name="cocomi_legs_v1">']
def link(name,f):
    ps=[n for n in R if R[n]['parent']==(f if f!='torso' else 'wpitch')]; mm=sum(R[n]['mass_g'] for n in ps)/1000
    c=[sum(R[n]['mass_g']*R[n]['com_mm'][i] for n in ps)/1000/max(mm,1e-9) for i in range(3)]
    if f=='torso':
        mu=PL['upper_g']/1000;cu=list(json.load(open('../pelvis-v1/torso/layout.json'))['com_upper_mm']);cu[2]+=DZ;c=[(mm*c[i]+mu*cu[i])/(mm+mu) for i in range(3)];mm+=mu
    o=fr[f]
    out.append(f'  <link name="{name}"><inertial><origin xyz="{m([c[i]-o[i] for i in range(3)])}"/><mass value="{mm:.4f}"/><inertia ixx="1e-4" iyy="1e-4" izz="1e-4" ixy="0" ixz="0" iyz="0"/></inertial>')
    for n in ps:
        if R[n]['path']: out.append(f'    <visual><origin xyz="{m([-x for x in o])}"/><geometry><mesh filename="{R[n]["path"]}" scale="0.001 0.001 0.001"/></geometry></visual>')
    out.append('  </link>')
def joint(n,p,c,pf,cf,ax,lo,hi,eff):
    a=fr[pf];b=fr[cf];out.append(f'  <joint name="{n}" type="revolute"><parent link="{p}"/><child link="{c}"/><origin xyz="{m([b[i]-a[i] for i in range(3)])}"/><axis xyz="{ax}"/><limit lower="{lo}" upper="{hi}" effort="{eff}" velocity="6"/></joint>')
link('pelvis_link','pelvis');link('waist_link','wyaw');link('torso_link','torso')
# v1.2 chain: pelvis -> waist_yaw (STS3215, vertical, no gravity load) -> waist_link -> waist_pitch (AK45-36, +Y = bow forward) -> torso (lifted DZ)
joint('waist_yaw','pelvis_link','waist_link','pelvis','wyaw','0 0 1',-0.79,0.79,2.9)
joint('waist_pitch','waist_link','torso_link','wyaw','torso','0 1 0',-0.26,1.05,24)
for s in 'LR':
    for nm,f in (('hip_yaw_link','yaw'),('hip_roll_link','roll'),('thigh','pitch'),('shin','knee'),('ankle_cross','apitch'),('foot','aroll')): link(f'{nm}_{s}',f'{f}_{s}')
    joint(f'hip_yaw_{s}','pelvis_link',f'hip_yaw_link_{s}','pelvis','yaw_'+s,'0 0 1',-0.52,0.52,2.9)
    joint(f'hip_roll_{s}',f'hip_yaw_link_{s}',f'hip_roll_link_{s}','yaw_'+s,'roll_'+s,'1 0 0',-0.52,0.52,12)
    joint(f'hip_pitch_{s}',f'hip_roll_link_{s}',f'thigh_{s}','roll_'+s,'pitch_'+s,'0 -1 0',-0.52,1.57,12)
    joint(f'knee_{s}',f'thigh_{s}',f'shin_{s}','pitch_'+s,'knee_'+s,'0 1 0',0,2.09,12)
    joint(f'ankle_pitch_{s}',f'shin_{s}',f'ankle_cross_{s}','knee_'+s,'apitch_'+s,'0 1 0',-0.44,0.44,5)
    joint(f'ankle_roll_{s}',f'ankle_cross_{s}',f'foot_{s}','apitch_'+s,'aroll_'+s,'1 0 0',-0.26,0.26,5)
out.append('</robot>');open('cocomi_legs_v1.urdf','w').write('\n'.join(out));print('ok',len(out))
