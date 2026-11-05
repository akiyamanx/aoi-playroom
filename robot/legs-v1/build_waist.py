# legs-v1.6 — COMPACT waist: AK45-36 beside the nested yaw servo, pitch axis 32 mm above the yaw plane (was 73). (v1.2: waist PITCH module between torso hip-plate and the waist-yaw adapter (torso→pitch→yaw→pelvis).
# Coordinates = pelvis/legs frame (unchanged). The whole torso (+arms, head) is lifted by DZ and pitches about (0,0,ZP).
# ALL dims/masses PLACEHOLDER.
import json, math, cadquery as cq
from cadquery import exporters
W=lambda: cq.Workplane("XY")
A=dict(ZP=-21, HP=45,                      # v1.6 compact: pitch axis z (was +20); hip-plate top = axis + HP  → DZ = ZP+HP
  act='AK45-36', act_d=55, act_h=56.5, m_act=0.349,   # geared pancake (36:1) rated 8 N·m / peak 24 N·m (CubeMars)
  t=4, m_loop=0.015, m_hw=0.030, rho=0.00124, pf=0.5, lim=[-15,60],
  ya=0, yb=0)                               # filled below: actuator y span
ZP=A['ZP']; DZ=ZP+A['HP']; A['DZ']=DZ; t=A['t']; r=A['act_d']/2; h=A['act_h']
ZW=-53                                       # waist YAW joint plane (STS3215 output face on the bearing) – unchanged
def box(c,d): return W().box(*d).translate(tuple(c))
def cyl(c,rr,hh,axis='Z'):
    d={'X':(1,0,0),'Y':(0,1,0),'Z':(0,0,1)}[axis]; b=[c[i]-d[i]*hh/2 for i in range(3)]
    return cq.Workplane(cq.Plane(origin=tuple(b),xDir=(0,1,0) if axis=='X' else (1,0,0),normal=d)).circle(rr).extrude(hh)
def cheek(y0,y1,z1,w=46,rr=24,hole=8):  # XZ plate between y0..y1, round end at the axis, straight to z1
    p=cyl([0,(y0+y1)/2,ZP],rr,y1-y0,'Y').union(box([0,(y0+y1)/2,(ZP+z1)/2],[w,y1-y0,abs(z1-ZP)]))
    return p.cut(cyl([0,(y0+y1)/2,ZP],hole,y1-y0+2,'Y')) if hole else p
P={}; PAR={}
# yaw servo (pelvis-v1, x±22 y±12 z −53…−18) is NESTED between the stator cheek (+y) and the passive pivot post (−y);
# the AK45-36 sits beside it on +y, so its axis (a Y line through x=0) still crosses the yaw axis.
ya=12.5+t+0.5; yb=ya+h; A['ya'],A['yb']=ya,yb
car=cyl([0,0,ZW+2],36,4).cut(cyl([0,0,ZW+2],18,6))                                # ring on the bearing's turning race
car=car.union(box([0,0,-34],[52,32,30]).cut(box([0,0,-34],[45,24.6,34])))          # servo pocket walls (z −49…−19)
car=car.union(cheek(12.5,12.5+t,ZW+4,w=52,rr=24,hole=10))                         # stator cheek (+y), centre bore for the cable
car=car.union(cheek(-12.5-t,-12.5,ZW+4,w=40,rr=14,hole=4))                         # passive pivot post (−y), Ø8 pin
car=car.cut(cyl([0,0,ZW+2],12.5,6))
P['waist_yaw_carrier']=car; PAR['waist_yaw_carrier']='wyaw'
P['waist_pitch_act']=cyl([0,(ya+yb)/2,ZP],r,h,'Y'); PAR['waist_pitch_act']='wyaw'   # stator on carrier
zpl=ZP+A['HP']-12                                   # hip plate bottom
up=box([0,(yb+1+t-12.5-t-4-t)/2+0,zpl-t/2],[80,(yb+1+t)-(-12.5-t-4-t),t])
up=up.union(cheek(yb+1,yb+1+t,zpl,w=40,rr=20,hole=0)).union(cheek(-12.5-t-4-t,-12.5-t-4,zpl,w=34,rr=12,hole=4))
P['waist_pitch_upper_yoke']=up; PAR['waist_pitch_upper_yoke']='wpitch'
# pitch cable service loop: 220° arc behind the axis on the passive (−y) side
lr=24; yl=-12.5-t-4-t-6
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
AD=json.load(open('../pelvis-v1/results.json'));AD=AD.get('parts',AD)['waist_adapter']   # v1.6: old adapter removed (carrier replaces it)
M=Mold+add-AD['mass_g']
C=[(Mold*Cold[i]+(mu*DZ if i==2 else 0)-AD['mass_g']*AD['com_mm'][i]+sum(v['mass_g']*v['com_mm'][i] for v in res.values()))/M for i in range(3)]
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
S=dict(version='legs-v1.6',waist_pitch=dict(order='torso→PITCH→yaw→pelvis',axis_z=ZP,lift_mm=DZ,yaw_plane_z=ZW,axis_gap_mm=ZP-ZW,old=dict(axis_z=20,lift_mm=64,axis_gap_mm=73),actuator=A['act'],limits_deg=A['lim'],
  rotating_mass_g=round(mr,1),rotating_com_above_axis_mm=round(d,1),rotating_com_fwd_mm=round(x0,1),torque_Nm_and_arm_mm=T,
  rated_Nm=8.0,peak_Nm=24.0,ak45_10_rated_Nm=2.5,ak45_10_peak_Nm=7.0,module_g=round(add,1)),
  total_height_mm=round(height,1),total_g=round(M,1),com_mm=[round(x,1) for x in C],com_above_floor_mm=round(C[2]-fz,1),
  com_bow30=com_at(30),com_bow60=com_at(60),sole_x_range=LS['sole_x_range'],params=A)
for k in ('com_bow30','com_bow60'): S[k+'_inside_sole_x']=LS['sole_x_range'][0]<S[k][0]<LS['sole_x_range'][1]
json.dump(dict(parts=res,summary=S,joints={'waist_pitch':[0,0,ZP]}),open('waist.json','w'),indent=1)
print(json.dumps({k:v for k,v in S.items() if k!='params'},ensure_ascii=False))
