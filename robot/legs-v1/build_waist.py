# legs-v1.7 — joint ORDER SWAPPED: pelvis → waist PITCH (AK45-36, low) → waist YAW (STS3215 in a big thin-section bearing) → torso.
# Coordinates = pelvis/legs frame (unchanged). The whole torso (+arms, head) is lifted by DZ and pitches about (0,0,ZP).
# ALL dims/masses PLACEHOLDER.
import json, math, cadquery as cq
from cadquery import exporters
W=lambda: cq.Workplane("XY")
A=dict(ZP=-29, ZW=3, HP=82,                # v1.7: pitch axis LOW (just above the pelvis block, top z −60); yaw plane ZW above it; DZ=ZP+HP
  act='AK45-36', act_d=55, act_h=56.5, m_act=0.349,   # geared pancake (36:1) rated 8 N·m / peak 24 N·m (CubeMars)
  brg='thin-section 60×80×10 (6812-class / crossed-roller placeholder)', m_brg=0.120, m_yaw_servo=0.055,
  t=4, m_loop=0.015, m_hw=0.030, rho=0.00124, pf=0.5, lim=[-15,60])
ZP=A['ZP']; ZW=A['ZW']; DZ=ZP+A['HP']; A['DZ']=DZ; t=A['t']; r=A['act_d']/2; h=A['act_h']
def box(c,d): return W().box(*d).translate(tuple(c))
def cyl(c,rr,hh,axis='Z'):
    d={'X':(1,0,0),'Y':(0,1,0),'Z':(0,0,1)}[axis]; b=[c[i]-d[i]*hh/2 for i in range(3)]
    return cq.Workplane(cq.Plane(origin=tuple(b),xDir=(0,1,0) if axis=='X' else (1,0,0),normal=d)).circle(rr).extrude(hh)
def cheek(y0,y1,z1,w=46,rr=24,hole=8):  # XZ plate between y0..y1, round end at the axis, straight to z1
    p=cyl([0,(y0+y1)/2,ZP],rr,y1-y0,'Y').union(box([0,(y0+y1)/2,(ZP+z1)/2],[w,y1-y0,abs(z1-ZP)]))
    return p.cut(cyl([0,(y0+y1)/2,ZP],hole,y1-y0+2,'Y')) if hole else p
P={}; PAR={}
# v1.7 order: pelvis → waist PITCH (AK45-36, low, centred on y=0) → waist YAW (STS3215 standing inside a big bearing) → torso
ya=h/2+1                                                                           # actuator y −28.25…28.25 (centred → no side CoM offset)
lo=None
for sg,hole in ((1,6),(-1,17)):                                                    # +y: stator flange + Ø12 cable bore; −y: Ø34 output-hub bore
    c=cheek(sg*ya,sg*(ya+t),-60,w=40,rr=17,hole=hole) if sg>0 else cheek(-ya-t,-ya,-60,w=40,rr=21,hole=hole)
    c=c.union(box([0,sg*(ya+t/2+3),-58],[50,t+6,4]))                               # foot on the pelvis block top
    lo=c if lo is None else lo.union(c)
P['waist_pitch_bracket']=lo; PAR['waist_pitch_bracket']='pelvis'
P['waist_pitch_act']=cyl([0,0,ZP],r,h,'Y'); PAR['waist_pitch_act']='pelvis'       # stator on the bracket, output flange on −y
yo=ya+t+1                                                                          # pitch carrier cheeks outside the bracket
car=box([0,0,ZW-1.5],[84,2*(yo+t),3])                                              # deck z 0…3 (actuator top −1.5)
car=car.union(cheek(yo,yo+t,ZW-3,w=40,rr=16,hole=6)).union(cheek(-yo-t,-yo,ZW-3,w=40,rr=21,hole=0))
P['waist_pitch_carrier']=car.cut(cyl([0,0,ZW-1.5],8,5)); PAR['waist_pitch_carrier']='wpitch'   # Ø16 cable hole in the deck centre
P['waist_yaw_bearing']=cyl([0,0,ZW+5],40,10).cut(cyl([0,0,ZW+5],30,12)); PAR['waist_yaw_bearing']='wpitch'
P['waist_yaw_servo']=box([0,0,ZW+17.5],[45,25,35]).union(cyl([0,0,ZW+36.5],10,3)); PAR['waist_yaw_servo']='wpitch'   # STS3215 inside the bearing bore, horn up
P['waist_yaw_skirt']=cyl([0,0,ZW+24],40,28).cut(cyl([0,0,ZW+24],36,30)).union(cyl([0,0,ZW+39.5],12,3).cut(cyl([0,0,ZW+39.5],3,4)))
P['waist_yaw_skirt']=P['waist_yaw_skirt'].union(box([0,0,ZW+37],[76,8,2]))   # horn bridge → skirt; hip plate bolts on top (z ZW+41)
PAR['waist_yaw_skirt']='wyaw'
# yaw cable loop: 270° horizontal arc between servo and skirt
lr=31; zl=ZW+24
arc=cq.Workplane('XY',origin=(0,0,zl)).moveTo(lr*math.cos(math.radians(45)),lr*math.sin(math.radians(45))).threePointArc((-lr,0),(lr*math.cos(math.radians(315)),lr*math.sin(math.radians(315))))
P['waist_yaw_cable_loop']=cq.Workplane('YZ',origin=(lr*math.cos(math.radians(45)),lr*math.sin(math.radians(45)),zl)).transformed(rotate=(0,-45,0)).circle(2.5).sweep(arc); PAR['waist_yaw_cable_loop']='wpitch'
# pitch cable loop: 220° arc around the axis outside the +y carrier cheek
lr=24; yl=yo+t+4
arc=cq.Workplane('XZ',origin=(0,yl,0)).moveTo(lr*math.cos(math.radians(70)),ZP+lr*math.sin(math.radians(70))).threePointArc(
    (lr*math.cos(math.radians(180)),ZP+lr*math.sin(math.radians(180))),(lr*math.cos(math.radians(290)),ZP+lr*math.sin(math.radians(290))))
prof=cq.Workplane('XY',origin=(lr*math.cos(math.radians(70)),yl,ZP+lr*math.sin(math.radians(70)))).circle(3.5)
P['waist_pitch_cable_loop']=prof.sweep(arc); PAR['waist_pitch_cable_loop']='pelvis'
BOUGHT={'waist_pitch_act':A['m_act'],'waist_pitch_cable_loop':A['m_loop'],'waist_yaw_cable_loop':A['m_loop'],'waist_yaw_bearing':A['m_brg'],'waist_yaw_servo':A['m_yaw_servo']}
res={}
for n,wp in P.items():
    sv=wp.val(); c=sv.Center(); exporters.export(wp,n+'.stl',tolerance=0.2,angularTolerance=0.3); exporters.export(wp,n+'.step')
    m=BOUGHT[n]*1000 if n in BOUGHT else sv.Volume()*A['rho']*A['pf']
    res[n]=dict(mass_g=m,com_mm=[c.x,c.y,c.z],valid=sv.isValid(),parent=PAR[n]); print(n,round(m,1),sv.isValid())
res['waist_pitch_hw']=dict(mass_g=A['m_hw']*1000,com_mm=[0,0,ZP],valid=True,parent='wpitch')
# ---- whole robot ----
LR=json.load(open('results.json')); LS=LR['summary']; LY=json.load(open('layout.json'))
TL=json.load(open('../pelvis-v1/torso/layout.json')); mu=TL['mass_upper_g']; cu=list(TL['com_upper_mm'])
Mold=LS['total_g']; Cold=LS['com_mm']
add=sum(v['mass_g'] for v in res.values())
PRp=json.load(open('../pelvis-v1/results.json'));PRp=PRp.get('parts',PRp)
REMV=['waist_adapter','waist_servo_STS3215','waist_bearing','waist_cable_loop']   # v1.7: old pelvis-side yaw stack replaced
rm=sum(PRp[n]['mass_g'] for n in REMV)
M=Mold+add-rm
C=[(Mold*Cold[i]+(mu*DZ if i==2 else 0)-sum(PRp[n]['mass_g']*PRp[n]['com_mm'][i] for n in REMV)+sum(v['mass_g']*v['com_mm'][i] for v in res.values()))/M for i in range(3)]
# torque about the pitch axis: rotating part = upper body (lifted) + upper yoke
RO=[v for v in res.values() if v['parent'] in ('wpitch','wyaw')]
mr=mu+sum(v['mass_g'] for v in RO); cr=[(mu*cu[0]+sum(v['mass_g']*v['com_mm'][0] for v in RO))/mr,0,(mu*(cu[2]+DZ)+sum(v['mass_g']*v['com_mm'][2] for v in RO))/mr]
d=cr[2]-ZP; x0=cr[0]
def tq(th):
    a=math.radians(th); arm=d*math.sin(a)+x0*math.cos(a); return round(mr/1000*9.81*arm/1000,2), round(arm,1)
def com_at(th):   # whole-robot CoM with torso pitched th (deg) about (0,0,ZP)
    a=math.radians(th); rx=x0+0; c=[cr[0],0,cr[2]-ZP]; xr=c[0]*math.cos(a)+c[2]*math.sin(a); zr=-c[0]*math.sin(a)+c[2]*math.cos(a)+ZP
    return [round((M*C[0]-mr*cr[0]+mr*xr)/M,1),round((M*C[2]-mr*cr[2]+mr*zr)/M,1)]
fz=LS['floor_z']; height=LS['total_height_mm']+DZ
T={str(a):tq(a) for a in (0,15,30,45,60,-15)}
S=dict(version='legs-v1.7',waist_pitch=dict(order='pelvis→PITCH→YAW→torso',axis_z=ZP,lift_mm=DZ,yaw_plane_z=ZW,axis_gap_mm=ZW-ZP,old=dict(v16=dict(axis_z=-21,lift_mm=24,axis_gap_mm=32,height=903.6),v15=dict(axis_gap_mm=73)),actuator=A['act'],limits_deg=A['lim'],
  rotating_mass_g=round(mr,1),rotating_com_above_axis_mm=round(d,1),rotating_com_fwd_mm=round(x0,1),torque_Nm_and_arm_mm=T,
  rated_Nm=8.0,peak_Nm=24.0,ak45_10_rated_Nm=2.5,ak45_10_peak_Nm=7.0,module_g=round(add,1)),
  total_height_mm=round(height,1),total_g=round(M,1),com_mm=[round(x,1) for x in C],com_above_floor_mm=round(C[2]-fz,1),
  com_bow30=com_at(30),com_bow60=com_at(60),sole_x_range=LS['sole_x_range'],params=A)
for k in ('com_bow30','com_bow60'): S[k+'_inside_sole_x']=LS['sole_x_range'][0]<S[k][0]<LS['sole_x_range'][1]
json.dump(dict(parts=res,summary=S,joints={'waist_pitch':[0,0,ZP],'waist_yaw':[0,0,ZW]},removed_pelvis=REMV),open('waist.json','w'),indent=1)
print(json.dumps({k:v for k,v in S.items() if k!='params'},ensure_ascii=False))
