# legs-v1.4 — electronics: split-ring clamps on the 2 Ø12 spine pipes → stays → vertical tray (rack) behind the pipes,
# boards on its back face; E-stop through the back cover; IMU moved to the pelvis centre; foot-sensor reader under the pelvis.
# Torso parts in TORSO frame; pelvis parts in PELVIS(=legs) frame. ALL dims/masses PLACEHOLDER.
import json, math, glob, cadquery as cq
from cadquery import exporters
W=lambda: cq.Workplane("XY")
SP=json.load(open('spine.json')); SA=SP['summary']['params']
px,py,bore,R=SA['pipe_x'],SA['pipe_y'],SA['bore'],SA['boss_d']/2
A=dict(TZ=32, TZ_range=None,      # tray bottom height on the pipes (torso z); slides along the pipes
  plate_x=(-12,-8), plate_y=56, plate_h=134, ring_h=16, ring_dz=(8,80),
  comp=(30,100,80), m_comp=0.175,  # main computer box x×y×z (Jetson Orin Nano dev-kit class 100×79 + cooler; Pi 5 + cooler fits inside)
  drv=(15,55,42), m_drv=0.035,     # actuator driver: CAN (AK45 ×9) + TTL serial bus (STS3215) — e.g. CAN-FD hat + bus adapter
  pwr=(22,50,42), m_pwr=0.060,     # power distribution + DC-DC (6S → 5 V 5 A / 12 V)
  estop_c=(-58,0,178), m_estop=0.045,   # 22 mm panel-mount mushroom E-stop on the back cover (cap Ø40 outside)
  m_wire=0.030,
  imu=(25,20,5), m_imu=0.005, pad_t=2.0, m_pad=0.001,   # IMU on gel pad, rigid printed cup under the pelvis centre
  fsb=(40,30,10), m_fsb=0.012,     # foot-sensor reader (8× FSR → ADC/MCU, sends over the serial bus)
  rho=1.27e-3, pf=0.5)
def box(c,d): return W().box(*d).translate(tuple(c))
def cylz(x,y,z,r,h): return W().workplane(offset=z).center(x,y).circle(r).extrude(h)
def cyl(c,r,h,ax):
    d={'X':(1,0,0),'Y':(0,1,0)}[ax]; b=[c[i]-d[i]*h/2 for i in range(3)]
    return cq.Workplane(cq.Plane(origin=tuple(b),xDir=(0,0,1),normal=d)).circle(r).extrude(h)
def ring(za,h):
    s=None
    for sg in (1,-1):
        y=sg*py; r=cylz(px,y,za,R,h).union(box([px+R+4,y,za+h/2],[12,13.5,h]))
        r=r.cut(cylz(px,y,za-1,bore/2,h+2)).cut(box([px+R+3,y,za+h/2],[2*R,1.5,h+2]))
        r=r.cut(cyl([px+R+6,y+3.8,za+h/2],1.7,6,'Y')).cut(cyl([px+R+6,y-3.6,za+h/2],2.0,5.7,'Y'))
        r=r.union(box([(px-R+A['plate_x'][1])/2+1,y,za+h/2],[px-R-A['plate_x'][1]+4,8,h]))   # stay → tray plate
        s=r if s is None else s.union(r)
    return s
def torso_parts(TZ):
    P={}
    x0,x1=A['plate_x']; zt=TZ; h=A['plate_h']
    pl=box([(x0+x1)/2,0,zt+h/2],[x1-x0,2*A['plate_y'],h]).edges("|X").fillet(4)
    for zz in (zt+12,zt+40,zt+70,zt+100,zt+122):            # lightening + zip-tie slots
        pl=pl.cut(box([(x0+x1)/2,0,zz],[10,60,8]))
    P['elec_tray']=pl
    P['elec_clamp_lo']=ring(zt+A['ring_dz'][0],A['ring_h']); P['elec_clamp_hi']=ring(zt+A['ring_dz'][1],A['ring_h'])
    cx,cy,cz=A['comp']; P['elec_computer']=box([x0-cx/2-0.5,0,zt+4+cz/2],[cx,cy,cz])
    dx,dy,dz=A['drv']; P['elec_driver_CAN_bus']=box([x0-dx/2-0.5,4+dy/2,zt+88+dz/2],[dx,dy,dz])
    wx,wy,wz=A['pwr']; P['elec_power_dcdc']=box([x0-wx/2-0.5,-4-wy/2,zt+88+wz/2],[wx,wy,wz])
    ex,ey,ez=A['estop_c']
    P['elec_estop']=cyl([ex-9,ey,ez],20,18,'X').union(cyl([ex+18,ey,ez],11,36,'X'))   # cap outside, body inside (x −58…−22)
    return P
PE={}
# pelvis: IMU cup under the pelvis block centre (block bottom z −116), right under the waist axis (0,0)
z0=-116; ix,iy,iz=A['imu']
cup=box([0,0,z0-5],[ix+10,iy+10,10]).edges("|Z").fillet(3).cut(box([0,0,z0-5-1],[ix+0.6,iy+0.6,10]))
for dx_,dy_ in ((-15,-12),(15,-12),(-15,12),(15,12)): cup=cup.union(cylz(dx_,dy_,z0-4,3,4)).cut(cylz(dx_,dy_,z0-5,1.7,6))   # 4× M3 to the block (inserts in block)
PE['pelvis_imu_cup']=cup
PE['pelvis_imu_pad']=box([0,0,z0-A['pad_t']/2],[ix,iy,A['pad_t']])
PE['pelvis_imu']=box([0,0,z0-A['pad_t']-iz/2],[ix,iy,iz])
fx,fy,fz=A['fsb']
PE['pelvis_footsensor_board']=box([-48,0,z0-2-fz/2],[fx,fy,fz]).union(box([-48,0,z0-1],[fx+6,fy+6,2]))
# ---- intra-torso collision over the tray travel ----
others={f.split('/')[-1][:-5]:cq.importers.importStep(f).val() for f in glob.glob('../pelvis-v1/torso/*.step')
        if not any(k in f for k in ('spine_frame','pulley','tensioner','electronics_bay','cover_front','cover_back'))}
others['cover_front_v13']=cq.importers.importStep('cover_front_v13.step').val()
cb=cq.Workplane(obj=cq.importers.importStep('../pelvis-v1/torso/cover_back.step').val()).cut(cyl([A['estop_c'][0],0,A['estop_c'][2]],11.3,12,'X'))
exporters.export(cb,'cover_back_v14.stl',tolerance=0.2,angularTolerance=0.3); exporters.export(cb,'cover_back_v14.step'); others['cover_back_v14']=cb.val()
for n in SP['parts']:
    if n!='spine_hw': others[n]=cq.importers.importStep(n+'.step').val()
L=json.load(open('../pelvis-v1/torso/layout.json'))
for i,f in enumerate(L['finger_servos']):
    others[f'finger_servo_{i}']=box(f['servo_c'],[23,12.2,12.2]).val()
    others[f'pulley_{i}']=cyl([f['pulley_c'][0]+3,f['pulley_c'][1],f['pulley_c'][2]],9,6,'X').val()
DES={('elec_clamp_lo','spine_pipe_L'),('elec_clamp_lo','spine_pipe_R'),('elec_clamp_hi','spine_pipe_L'),('elec_clamp_hi','spine_pipe_R'),('elec_estop','cover_back_v14')}
def check(TZ,verbose=False):
    P=torso_parts(TZ); hits=[]
    for n,wp in P.items():
        for o,sv in others.items():
            bb=wp.val().BoundingBox();cc=sv.BoundingBox()
            if bb.xmax<cc.xmin or cc.xmax<bb.xmin or bb.ymax<cc.ymin or cc.ymax<bb.ymin or bb.zmax<cc.zmin or cc.zmax<bb.zmin: continue
            v=wp.val().intersect(sv).Volume()
            if v>1.0 and (n,o) not in DES: hits.append([n,o,round(v,1)])
    for a in P:
        for b in P:
            if a<b and not ({a,b}&{'elec_tray'} and ({a,b}&{'elec_clamp_lo','elec_clamp_hi'})):
                v=P[a].val().intersect(P[b].val()).Volume()
                if v>1.0: hits.append([a,b,round(v,1)])
    return P,hits
okz=[]
for TZ in range(16,52,4):
    _,h=check(TZ); print('TZ',TZ,h or 'OK'); okz.append(TZ) if not h else None
P,hits=check(A['TZ']); assert not hits, hits
A['TZ_range']=[min(okz),max(okz)]
# pelvis-side check vs pelvis + legs parts (neutral; deeper poses in collide.py)
PLy=json.load(open('../pelvis-v1/layout.json'));LL=json.load(open('layout.json'))
pel={n:cq.importers.importStep('../pelvis-v1/'+n+'.step').val() for n in PLy['parts'] if not n.startswith('thigh_stub')}
ph=[]
for n,wp in PE.items():
    for o,sv in pel.items():
        v=wp.val().intersect(sv).Volume()
        if v>1.0: ph.append([n,o,round(v,1)])
print('pelvis hits',ph)
BOUGHT={'elec_computer':A['m_comp'],'elec_driver_CAN_bus':A['m_drv'],'elec_power_dcdc':A['m_pwr'],'elec_estop':A['m_estop'],
        'pelvis_imu':A['m_imu'],'pelvis_imu_pad':A['m_pad'],'pelvis_footsensor_board':A['m_fsb']}
res={}
for grp,parent in ((P,'torso'),(PE,'pelvis')):
    for n,wp in grp.items():
        sv=wp.val(); c=sv.Center(); exporters.export(wp,n+'.stl',tolerance=0.15,angularTolerance=0.25); exporters.export(wp,n+'.step')
        m=BOUGHT[n]*1000 if n in BOUGHT else sv.Volume()*A['rho']*A['pf']
        res[n]=dict(mass_g=m,com_mm=[c.x,c.y,c.z],valid=sv.isValid(),frame=parent,tray=n.startswith('elec_') and n!='elec_estop'); print(n,round(m,1))
res['elec_wiring']=dict(mass_g=A['m_wire']*1000,com_mm=[-25,0,A['TZ']+60],valid=True,frame='torso',tray=True)
res['elec_hw']=dict(mass_g=12.0,com_mm=[0,0,A['TZ']+50],valid=True,frame='torso',tray=True)   # 4 clamp M3×16 + inserts, 8× M2.5 standoffs/screws
# ---- removed (torso v1.3 electronics_bay with IMU mount, RPi, 2 drivers, IMU) ----
TR=json.load(open('../pelvis-v1/torso/results.json'));TR=TR.get('parts',TR)
REM=[(TR['electronics_bay']['mass_print_g'],TR['electronics_bay']['com_mm']),(60,[-25,0,120]),(30,[-25,30,95]),(30,[-25,-30,95]),(5,[0,0,53])]
S=SP['summary'];WJ=json.load(open('waist.json'));DZ=WJ['summary']['params']['DZ'];ZP=WJ['summary']['params']['ZP']
tb=lambda c,f:[c[0],c[1],c[2]+(DZ if f=='torso' else 0)]
Mt=S['total_g'];Ct=S['com_mm']
mrem=sum(m for m,_ in REM); madd=sum(r['mass_g'] for r in res.values())
Mn=Mt-mrem+madd
Cn=[(Mt*Ct[i]-sum(m*tb(c,'torso')[i] for m,c in REM)+sum(r['mass_g']*tb(r['com_mm'],r['frame'])[i] for r in res.values()))/Mn for i in range(3)]
# rotating mass about waist pitch (torso-frame items only)
mr0=S['rotating_mass_g'];h0=S['rotating_com_above_axis_mm'];f0=S['rotating_com_fwd_mm']
tor=[(r['mass_g'],r['com_mm']) for r in res.values() if r['frame']=='torso']
mr=mr0-mrem+sum(m for m,_ in tor)
zc=lambda c:c[2]+DZ-ZP
h1=(mr0*h0-sum(m*zc(c) for m,c in REM)+sum(m*zc(c) for m,c in tor))/mr; f1=(mr0*f0-sum(m*c[0] for m,c in REM)+sum(m*c[0] for m,c in tor))/mr
tq={a:round(mr/1000*9.81*(h1*math.sin(math.radians(a))+f1*math.cos(math.radians(a)))/1000,2) for a in (0,15,30,45,60,-15)}
fl=WJ['summary']['com_above_floor_mm']-WJ['summary']['com_mm'][2]
summary=dict(version='legs-v1.4',params=A,removed_g=mrem,added_g=round(madd,1),delta_g=round(madd-mrem,1),total_g=round(Mn,1),com_mm=[round(v,1) for v in Cn],
  com_above_floor_mm=round(Cn[2]+fl,1),rotating_mass_g=round(mr,1),rotating_com_above_axis_mm=round(h1,1),rotating_com_fwd_mm=round(f1,1),torque_Nm=tq,
  torso_hits=hits,pelvis_hits=ph,removed=REM)
json.dump(dict(summary=summary,parts=res),open('elec.json','w'),indent=1)
print(json.dumps({k:summary[k] for k in ('removed_g','added_g','delta_g','total_g','com_mm','com_above_floor_mm','torque_Nm')},ensure_ascii=False),A['TZ_range'])
