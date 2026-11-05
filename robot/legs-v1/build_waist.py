# legs-v1.2 — waist PITCH module between torso hip-plate and the waist-yaw adapter (torso→pitch→yaw→pelvis).
# Coordinates = pelvis/legs frame (unchanged). The whole torso (+arms, head) is lifted by DZ and pitches about (0,0,ZP).
# ALL dims/masses PLACEHOLDER.
import json, math, cadquery as cq
from cadquery import exporters
W=lambda: cq.Workplane("XY")
A=dict(DZ=64, ZP=20,                       # torso lift; pitch axis z (adapter top −12 … hip-plate bottom −12+DZ)
  act='AK45-36', act_d=55, act_h=56.5, m_act=0.349,   # geared pancake (36:1) rated 8 N·m / peak 24 N·m (CubeMars)
  t=4, m_loop=0.015, m_hw=0.030, rho=0.00124, pf=0.5, lim=[-15,60])
DZ,ZP,t=A['DZ'],A['ZP'],A['t']; r=A['act_d']/2; h=A['act_h']
zl=-12; zu=-12+DZ                            # adapter top, hip plate bottom (lifted)
def box(c,d): return W().box(*d).translate(tuple(c))
def cyl(c,rr,hh,axis='Z'):
    d={'X':(1,0,0),'Y':(0,1,0),'Z':(0,0,1)}[axis]; b=[c[i]-d[i]*hh/2 for i in range(3)]
    return cq.Workplane(cq.Plane(origin=tuple(b),xDir=(0,1,0) if axis=='X' else (1,0,0),normal=d)).circle(rr).extrude(hh)
def cheek(y,z1,z2,w=46,rr=24):  # XZ plate, round end at the axis
    zc=ZP; p=cq.Workplane('XZ',origin=(0,y+t/2,0)).center(0,zc).circle(rr).extrude(t)
    zz=z1 if abs(z1-zc)>abs(z2-zc) else z2
    return p.union(box([0,y,(zc+zz)/2],[w,t,abs(zz-zc)])).cut(cyl([0,y,zc],8,t+2,'Y'))
P={}; PAR={}
yi=h/2+1+t/2; yo=yi+t+1
lo=box([0,0,zl+t/2],[70,2*yi+t,t]).union(cheek(yi,zl,ZP)).union(cheek(-yi,zl,ZP))
P['waist_pitch_lower_yoke']=lo; PAR['waist_pitch_lower_yoke']='wyaw'
P['waist_pitch_act']=cyl([0,0,ZP],r,h,'Y'); PAR['waist_pitch_act']='wyaw'   # stator on lower yoke (rides on the yaw)
up=box([0,0,zu-t/2],[80,2*yo+t,t]).union(cheek(yo,ZP,zu,w=40,rr=20)).union(cheek(-yo,ZP,zu,w=40,rr=20))
P['waist_pitch_upper_yoke']=up; PAR['waist_pitch_upper_yoke']='wpitch'
# pitch cable service loop: 200° arc behind the axis, outside the right cheek
lr=36; yl=-(yo+t+6)
arc=cq.Workplane('XZ',origin=(0,yl,0)).moveTo(lr*math.cos(math.radians(70)),ZP+lr*math.sin(math.radians(70))).threePointArc(
    (lr*math.cos(math.radians(180)),ZP+lr*math.sin(math.radians(180))),(lr*math.cos(math.radians(290)),ZP+lr*math.sin(math.radians(290))))
prof=cq.Workplane('XY',origin=(lr*math.cos(math.radians(70)),yl,ZP+lr*math.sin(math.radians(70)))).circle(3.5)
P['waist_pitch_cable_loop']=prof.sweep(arc); PAR['waist_pitch_cable_loop']='wyaw'
BOUGHT={'waist_pitch_act':A['m_act'],'waist_pitch_cable_loop':A['m_loop']}
res={}
for n,wp in P.items():
    sv=wp.val(); c=sv.Center(); exporters.export(wp,n+'.stl',tolerance=0.2,angularTolerance=0.3); exporters.export(wp,n+'.step')
    m=BOUGHT[n]*1000 if n in BOUGHT else sv.Volume()*A['rho']*A['pf']
    res[n]=dict(mass_g=m,com_mm=[c.x,c.y,c.z],valid=sv.isValid(),parent=PAR[n]); print(n,round(m,1),sv.isValid())
res['waist_pitch_hw']=dict(mass_g=A['m_hw']*1000,com_mm=[0,0,ZP],valid=True,parent='wyaw')
# ---- whole robot ----
LR=json.load(open('results.json')); LS=LR['summary']; LY=json.load(open('layout.json'))
TL=json.load(open('../pelvis-v1/torso/layout.json')); mu=TL['mass_upper_g']; cu=list(TL['com_upper_mm'])
up_yoke=res['waist_pitch_upper_yoke']
Mold=LS['total_g']; Cold=LS['com_mm']
add=sum(v['mass_g'] for v in res.values())
M=Mold+add
C=[(Mold*Cold[i]+(mu*DZ if i==2 else 0)+sum(v['mass_g']*v['com_mm'][i] for v in res.values()))/M for i in range(3)]
# torque about the pitch axis: rotating part = upper body (lifted) + upper yoke
mr=mu+up_yoke['mass_g']; cr=[(mu*cu[0]+up_yoke['mass_g']*up_yoke['com_mm'][0])/mr,0,(mu*(cu[2]+DZ)+up_yoke['mass_g']*up_yoke['com_mm'][2])/mr]
d=cr[2]-ZP; x0=cr[0]
def tq(th):
    a=math.radians(th); arm=d*math.sin(a)+x0*math.cos(a); return round(mr/1000*9.81*arm/1000,2), round(arm,1)
def com_at(th):   # whole-robot CoM with torso pitched th (deg) about (0,0,ZP)
    a=math.radians(th); rx=x0+0; c=[cr[0],0,cr[2]-ZP]; xr=c[0]*math.cos(a)+c[2]*math.sin(a); zr=-c[0]*math.sin(a)+c[2]*math.cos(a)+ZP
    return [round((M*C[0]-mr*cr[0]+mr*xr)/M,1),round((M*C[2]-mr*cr[2]+mr*zr)/M,1)]
fz=LS['floor_z']; height=LS['total_height_mm']+DZ
T={str(a):tq(a) for a in (0,15,30,45,60,-15)}
S=dict(version='legs-v1.2',waist_pitch=dict(order='torso→PITCH→yaw→pelvis',axis_z=ZP,lift_mm=DZ,actuator=A['act'],limits_deg=A['lim'],
  rotating_mass_g=round(mr,1),rotating_com_above_axis_mm=round(d,1),rotating_com_fwd_mm=round(x0,1),torque_Nm_and_arm_mm=T,
  rated_Nm=8.0,peak_Nm=24.0,ak45_10_rated_Nm=2.5,ak45_10_peak_Nm=7.0,module_g=round(add,1)),
  total_height_mm=round(height,1),total_g=round(M,1),com_mm=[round(x,1) for x in C],com_above_floor_mm=round(C[2]-fz,1),
  com_bow30=com_at(30),com_bow60=com_at(60),sole_x_range=LS['sole_x_range'],params=A)
for k in ('com_bow30','com_bow60'): S[k+'_inside_sole_x']=LS['sole_x_range'][0]<S[k][0]<LS['sole_x_range'][1]
json.dump(dict(parts=res,summary=S,joints={'waist_pitch':[0,0,ZP]}),open('waist.json','w'),indent=1)
print(json.dumps({k:v for k,v in S.items() if k!='params'},ensure_ascii=False))
