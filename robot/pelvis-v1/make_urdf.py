import json
L=json.load(open('layout.json'));J=L['joints'];P=L['parts'];R=json.load(open('results.json'))['parts']
m=lambda v:' '.join(f'{x/1000:.4f}' for x in v)
frames={'torso':[0,0,0],'pelvis':J['waist']}
for s in 'LR':
    frames['yaw_'+s]=J['hip_yaw_'+s];frames['roll_'+s]=J['hip_roll_'+s];frames['pitch_'+s]=J['hip_pitch_'+s]
out=['<?xml version="1.0"?>','<!-- pelvis-v1 (PLACEHOLDER dims/masses). torso_link = torso-v1 upper body lumped. -->','<robot name="cocomi_pelvis_v1">']
def link(name,fr,extra=''):
    ps=[n for n in P if P[n]['parent']==fr]
    mm=sum(R[n]['mass_g'] for n in ps)/1000; c=[sum(R[n]['mass_g']*R[n]['com_mm'][i] for n in ps)/1000/max(mm,1e-9) for i in range(3)]
    if fr=='torso':
        mu=L['upper_g']/1000;cu=json.load(open('../torso-v1/layout.json'))['com_upper_mm']
        c=[(mm*c[i]+mu*cu[i])/(mm+mu) for i in range(3)];mm+=mu
    o=frames[fr];rel=[c[i]-o[i] for i in range(3)]
    out.append(f'  <link name="{name}"><inertial><origin xyz="{m(rel)}"/><mass value="{mm:.4f}"/><inertia ixx="1e-4" iyy="1e-4" izz="1e-4" ixy="0" ixz="0" iyz="0"/></inertial>')
    for n in ps: out.append(f'    <visual><origin xyz="{m([-x for x in o])}"/><geometry><mesh filename="{n}.stl" scale="0.001 0.001 0.001"/></geometry></visual>')
    out.append('  </link>')
link('torso_link','torso');link('pelvis_link','pelvis')
def joint(n,par,ch,pf,cf,ax,lo,hi,eff):
    a=frames[pf];b=frames[cf]
    out.append(f'  <joint name="{n}" type="revolute"><parent link="{par}"/><child link="{ch}"/><origin xyz="{m([b[i]-a[i] for i in range(3)])}"/><axis xyz="{ax}"/><limit lower="{lo}" upper="{hi}" effort="{eff}" velocity="6"/></joint>')
joint('waist_yaw','torso_link','pelvis_link','torso','pelvis','0 0 1',-0.79,0.79,2.9)
for s in 'LR':
    link(f'hip_yaw_link_{s}','yaw_'+s);link(f'hip_roll_link_{s}','roll_'+s);link(f'thigh_stub_{s}','pitch_'+s)
    joint(f'hip_yaw_{s}','pelvis_link',f'hip_yaw_link_{s}','pelvis','yaw_'+s,'0 0 1',-0.52,0.52,2.9)
    joint(f'hip_roll_{s}',f'hip_yaw_link_{s}',f'hip_roll_link_{s}','yaw_'+s,'roll_'+s,'1 0 0',-0.52,0.52,12)
    joint(f'hip_pitch_{s}',f'hip_roll_link_{s}',f'thigh_stub_{s}','roll_'+s,'pitch_'+s,'0 1 0',-1.57,0.52,12)
out.append('</robot>');open('cocomi_pelvis_v1.urdf','w').write('\n'.join(out));print('ok')
