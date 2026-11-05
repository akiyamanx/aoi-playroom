# legs-v1 — thigh + knee pitch (AK45-10 class) + shin + 2-axis parallel-rod ankle (2× STS3215) + flat angular sole
# Units mm, frame = torso/pelvis: origin hip-plate TOP centre, +Z up +Y left +X fwd. ALL dims/masses PLACEHOLDER.
import json, math, cadquery as cq
from cadquery import exporters
W=lambda: cq.Workplane("XY")
PV=json.load(open('../pelvis-v1/results.json')); PL=json.load(open('../pelvis-v1/layout.json'))
JP=PV['summary']['joints_mm']; z_hip=JP['hip_pitch_L'][2]; HY=PV['summary']['params']['hip_y']
A=dict(
  head_top=314.6,                 # from torso head_placeholder bbox
  thigh=72, shin=72, ankle_h=46,  # hip→knee, knee→ankle, ankle axis→floor (thigh:shin = 1:1, human-like)
  act_d=53, act_h=45, m_act=0.260,
  sts=(45.0,25.0,35.0), m_sts=0.055,
  sole_l=140, sole_w=80, sole_t=10, sole_x0=-45,   # Gundam-like flat sole, heel at x=-45 from ankle
  fsr_d=14, fsr_t=2, m_fsr=0.002,                  # 4 corner pressure sensors PLACEHOLDER (FSR)
  rod_r=1.5, crank=16, rod_x=34, rod_dy=34,        # parallel ankle push-rods
  rho=0.00124, print_factor=0.5, m_hw=0.020,
)
zk=z_hip-A['thigh']; za=zk-A['shin']; zf=za-A['ankle_h']
sL,sW,sH=A['sts']; r_=A['act_d']/2; ah=A['act_h']; pt=4
def box(c,d): return W().box(*d).translate(tuple(c))
def cyl(c,r,h,axis='Z'):
    d={'X':(1,0,0),'Y':(0,1,0),'Z':(0,0,1)}[axis]; base=[c[i]-d[i]*h/2 for i in range(3)]
    return cq.Workplane(cq.Plane(origin=tuple(base),xDir=(0,1,0) if axis=='X' else (1,0,0),normal=d)).circle(r).extrude(h)
def plate(y,z1,z2,w=28,rr=20):   # side plate XZ with round ends at z1,z2
    p=cq.Workplane('XZ',origin=(0,y+pt/2,0)).center(0,z1).circle(rr).extrude(pt)
    p=p.union(cq.Workplane('XZ',origin=(0,y+pt/2,0)).center(0,z2).circle(rr).extrude(pt))
    return p.union(box([0,y,(z1+z2)/2],[w,pt,abs(z1-z2)]))
J={}; parts={}; parent={}
for s,n in ((1,'L'),(-1,'R')):
    y=s*HY; J['knee_'+n]=[0,y,zk]; J['ankle_pitch_'+n]=[0,y,za]; J['ankle_roll_'+n]=[0,y,za]
    off=ah/2+pt/2+1
    # THIGH (child of hip pitch): two side plates hip→knee + front/back webs, holds knee actuator stator
    th=plate(y+off,z_hip,zk).union(plate(y-off,z_hip,zk))
    th=th.union(box([20,y,z_hip-43],[4,2*off+pt,16]))   # front web below the hip-roll cradle
    parts['thigh_'+n]=th; parent['thigh_'+n]='pitch_'+n
    parts['knee_act_'+n]=cyl(J['knee_'+n],r_,ah,'Y'); parent['knee_act_'+n]='pitch_'+n
    # SHIN (child of knee): side plates knee→ankle, ankle clevis at bottom
    offs=off+pt+1   # shin plates OUTSIDE thigh plates
    sh=plate(y+offs,zk,za,w=26,rr=16).union(plate(y-offs,zk,za,w=26,rr=16))
    sh=sh.union(box([-13,y,(zk+za)/2-6],[4,2*offs+pt,abs(zk-za)-30]))
    # ankle servo bay on the FRONT of the knee (mass high): 2× STS3215, shafts along Y
    bx=r_+3+sW/2
    for k,dy in ((0,-sH/2-0.5),(1,sH/2+0.5)):
        nm=f'ankle_servo_{n}{"io"[k] if s>0 else "oi"[k]}'
        parts[nm]=box([bx,y+dy,zk-8],[sW,sH,sL]); parent[nm]='knee_'+n
    bay=box([bx,y,zk-8],[sW+6,2*sH+7,sL+4]).cut(box([bx,y,zk-8],[sW+0.6,2*sH+1.6,sL+6]))
    sh=sh.union(bay).union(box([(r_+3)/2+6,y,zk-8-sL/2],[r_-3,2*offs+pt,4]))
    parts['shin_'+n]=sh; parent['shin_'+n]='knee_'+n
    # push-rods: horn tip (front, outer faces) → foot ball-joints (front of ankle)
    hz=zk-8+sL/2-10; fz=za-14
    rods=None
    for dy in (-A['rod_dy'],A['rod_dy']):
        yy=y+dy+(2 if dy>0 else -2)*0
        rd=cyl([A['rod_x'],y+dy+math.copysign(A['act_h']*0+8,dy),(hz+fz)/2],A['rod_r'],hz-fz)
        rods=rd if rods is None else rods.union(rd)
    parts['ankle_rods_'+n]=rods; parent['ankle_rods_'+n]='knee_'+n
    # ankle cross (gimbal) — pitch axis Y and roll axis X intersect at (0,y,za)
    parts['ankle_cross_'+n]=cyl(J['ankle_pitch_'+n],7,2*off-6,'Y').union(cyl(J['ankle_pitch_'+n],7,40,'X')); parent['ankle_cross_'+n]='apitch_'+n
    # FOOT: flat angular (chamfered) sole + roll clevis + rod ball-joint ears; optional toe joint line at x=+60
    L_,Wd,T=A['sole_l'],A['sole_w'],A['sole_t']; x0=A['sole_x0']; xc=x0+L_/2
    so=box([xc,y,zf+T/2],[L_,Wd,T]).edges('|Z').chamfer(14).edges('>Z').chamfer(3)
    so=so.cut(box([60,y,zf+T],[2,Wd+2,4]))                                  # toe-joint groove (future)
    for xx in (x0+18, x0+L_-18):
        for yy in (y-Wd/2+14, y+Wd/2-14):
            so=so.cut(cyl([xx,yy,zf+A['fsr_t']/2],A['fsr_d']/2+1,A['fsr_t']+0.2))  # sensor pockets
    cl=box([-24,y,(zf+T+za)/2+4],[6,26,za-zf-T+8]).union(box([24,y,(zf+T+za)/2+4],[6,26,za-zf-T+8]))
    cl=cl.cut(cyl(J['ankle_pitch_'+n],3.2,60,'X'))
    ears=None
    for dy in (-A['rod_dy'],A['rod_dy']):
        e=box([A['rod_x'],y+dy+math.copysign(8,dy),(zf+T+fz)/2],[12,10,fz-zf-T+6])
        ears=e if ears is None else ears.union(e)
    parts['foot_'+n]=so.union(cl).union(ears); parent['foot_'+n]='aroll_'+n
    fs=None
    for xx in (x0+18, x0+L_-18):
        for yy in (y-Wd/2+14, y+Wd/2-14):
            c_=cyl([xx,yy,zf+A['fsr_t']/2],A['fsr_d']/2,A['fsr_t']); fs=c_ if fs is None else fs.union(c_)
    parts['fsr_'+n]=fs; parent['fsr_'+n]='aroll_'+n
BOUGHT={}
for n in 'LR':
    BOUGHT['knee_act_'+n]=A['m_act']; BOUGHT['fsr_'+n]=4*A['m_fsr']
    for k in 'io': BOUGHT[f'ankle_servo_{n}{k}']=A['m_sts']
res={}; bodies=[]
for n,wp in parts.items():
    sv=wp.val(); v=sv.Volume()/1000; c=sv.Center()
    exporters.export(wp,n+'.stl',tolerance=0.2,angularTolerance=0.3); exporters.export(wp,n+'.step')
    m=BOUGHT[n] if n in BOUGHT else (v*1000*A['rho']*A['print_factor'] if not n.startswith('ankle_rods') else v*7.9)  # rods steel
    res[n]=dict(mass_g=m*1000 if n in BOUGHT else m,com_mm=[c.x,c.y,c.z],bought=n in BOUGHT,valid=sv.isValid(),parent=parent[n])
    print(f"{n:22s} {res[n]['mass_g']:7.1f} g valid={sv.isValid()}")
for n in 'LR': res['hardware_'+n]=dict(mass_g=A['m_hw']*1000,com_mm=[0,HY*(1 if n=='L' else -1),zk],bought=True,valid=True,parent='knee_'+n)
leg_g=sum(d['mass_g'] for d in res.values())
leg_c=[sum(d['mass_g']*d['com_mm'][i] for d in res.values())/leg_g for i in range(3)]
# whole robot: pelvis-v1.2 total minus its thigh stubs + legs
stub=[PV['parts']['thigh_stub_'+n] for n in 'LR']
Mt=PV['summary']['total_g']; Ct=PV['summary']['com_total_mm']
M0=Mt-sum(p['mass_g'] for p in stub); C0=[(Mt*Ct[i]-sum(p['mass_g']*p['com_mm'][i] for p in stub))/M0 for i in range(3)]
M=M0+leg_g; C=[(M0*C0[i]+leg_g*leg_c[i])/M for i in range(3)]
height=A['head_top']-zf
one_leg=leg_g/2; one_leg_c=[sum(d['mass_g']*d['com_mm'][i] for k,d in res.items() if d['parent'].endswith('_L'))/one_leg for i in range(3)]
sole_poly=[[A['sole_x0']+a,s*HY+b] for s in (1,-1) for a,b in ((0,-A['sole_w']/2),(A['sole_l'],-A['sole_w']/2),(A['sole_l'],A['sole_w']/2),(0,A['sole_w']/2))]
S=dict(version='legs-v1',total_height_mm=round(height,1),floor_z=zf,thigh_mm=A['thigh'],shin_mm=A['shin'],ankle_height_mm=A['ankle_h'],
  hip_to_floor_mm=round(z_hip-zf,1),total_g=round(M,1),legs_g=round(leg_g,1),one_leg_g=round(one_leg,1),
  one_leg_com_below_hip_mm=round(z_hip-one_leg_c[2],1),com_mm=[round(x,1) for x in C],com_above_floor_mm=round(C[2]-zf,1),
  sole_mm=[A['sole_l'],A['sole_w'],A['sole_t']],sole_x_range=[A['sole_x0'],A['sole_x0']+A['sole_l']],
  com_inside_double_support=bool(A['sole_x0']<C[0]<A['sole_x0']+A['sole_l'] and abs(C[1])<HY+A['sole_w']/2),
  stance_width_outer_mm=2*HY+A['sole_w'],params=A)
J['hip_pitch_L']=JP['hip_pitch_L'];J['hip_pitch_R']=JP['hip_pitch_R']
json.dump(dict(parts=res,summary=S),open('results.json','w'),indent=1)
json.dump(dict(joints=J,floor_z=zf,com_total_mm=C,total_g=M,legs_g=leg_g,upper_g=PL['upper_g'],pelvis_g=PL['pelvis_g'],
  sole=dict(x0=A['sole_x0'],l=A['sole_l'],w=A['sole_w']),hip_y=HY,height=height,parts={k:{'parent':v} for k,v in parent.items()}),open('layout.json','w'),indent=1)
print(json.dumps({k:v for k,v in S.items() if k!='params'}))
