# legs-v1.3 — SPINE redesign: 2 aluminium pipes (home-centre stock) + printed clamps (PETG / PLA+) with brass heat-set inserts.
# Replaces the printed torso spine_frame (column + ribs + shoulder beam + neck). Coordinates = TORSO frame
# (origin hip-plate top centre, +X fwd, +Y left, +Z up); in the whole body the torso sits +DZ and bows about the waist pitch.
# ALL dims/masses PLACEHOLDER.
import json, math, glob, cadquery as cq
from cadquery import exporters
W=lambda: cq.Workplane("XY")
A=dict(pipe_od=12.0, pipe_t=1.0, pipe_x=30.0, pipe_y=18.0, pipe_z0=3.0, pipe_z1=204.0,   # Ø12×1 A6063, 2 pcs
  boss_d=20.0,        # clamp ring OD → printed wall (20-12.2)/2 ≈ 3.9 mm
  bore=12.2, slit=1.5, ear_t=6.0, ear_w=12.0,
  ins_hole=4.0, ins_len=5.7, clr=3.4,          # M3 heat-set insert (M3×5.7, OD≈4.6, hole Ø4.0)
  rho_al=2.70e-3, rho_pr=1.27e-3, pf=0.5,      # PETG 1.27 g/cm³ × 0.5 (4 walls + 40 % infill)
  E_al=69000, sy_al=145)                       # A6063-T5 MPa
pod,pt,px,py=A['pipe_od'],A['pipe_t'],A['pipe_x'],A['pipe_y']; z0,z1=A['pipe_z0'],A['pipe_z1']; R=A['boss_d']/2
def box(c,d): return W().box(*d).translate(tuple(c))
def cylz(x,y,z,r,h): return W().workplane(offset=z).center(x,y).circle(r).extrude(h)
def cyl(c,r,h,ax):
    d={'X':(1,0,0),'Y':(0,1,0)}[ax]; b=[c[i]-d[i]*h/2 for i in range(3)]
    return cq.Workplane(cq.Plane(origin=tuple(b),xDir=(0,0,1),normal=d)).circle(r).extrude(h)
HW=[]   # (name, qty, unit g) bought hardware
def clamp(sol,za,zb,sides=(1,-1)):
    """pipe ring bosses at (px,±py) from za..zb, slit toward +X, ear with M3 clearance (+Y side) and insert (−Y side)."""
    h=zb-za; zm=(za+zb)/2
    for s in sides:
        y=s*py
        sol=sol.union(cylz(px,y,za,R,h)).union(box([px+R+A['ear_w']/2-2,y,zm],[A['ear_w']+4,2*A['ear_t']+A['slit'],h]))
        sol=sol.cut(cylz(px,y,za-1,A['bore']/2,h+2)).cut(box([px+R+3,y,zm],[2*R,A['slit'],h+2]))
        bx=px+R+A['ear_w']/2
        sol=sol.cut(cyl([bx,y+A['slit']/2+A['ear_t']/2+0.01,zm],A['clr']/2,A['ear_t']+0.02,'Y'))
        sol=sol.cut(cyl([bx,y-A['slit']/2-A['ins_len']/2,zm],A['ins_hole']/2,A['ins_len']+0.02,'Y'))
        HW.append(('clamp M3×16 + heat-set insert',1,1.6))
    return sol
P={}
# pipes
for s,n in ((1,'L'),(-1,'R')):
    P['spine_pipe_'+n]=cylz(px,s*py,z0,pod/2,z1-z0).cut(cylz(px,s*py,z0-1,pod/2-pt,z1-z0+2))
# 1. bottom mount (to waist-pitch upper yoke through the hip plate, old 4×M3 pattern ±10)
bm=box([13,0,3],[62,70,6]).edges("|Z").fillet(5)
bm=clamp(bm,0,18)   # stays below the front cover's bottom lip (z 20)
for dx,dy in [(-10,-10),(-10,10),(10,-10),(10,10)]: bm=bm.cut(cylz(dx,dy,-1,1.7,8))
bm=bm.cut(cylz(px,py,-1,4,3)).cut(cylz(px,-py,-1,4,3))     # cable exit from inside the pipes, down to the pelvis
for _ in range(4): HW.append(('M3×12 base→hip plate→yoke (insert in yoke)',1,1.2))
P['spine_base_mount']=bm
# 2. chest mounts L/R — clamp one pipe each, flange on the inner face of the finger-servo bay (y=29), above the electronics bay
ZC0,ZC1=138,162
for s,n in ((1,'L'),(-1,'R')):
    cm=box([12,s*24.5,(ZC0+ZC1)/2],[24,5,ZC1-ZC0])                     # flange x 0..24 against bay inner face
    cm=cm.union(box([22,s*(py+3.5),(ZC0+ZC1)/2],[8,9,ZC1-ZC0]))           # web to the ring
    cm=clamp(cm,ZC0,ZC1,sides=(s,))
    for zz in (ZC0+6,ZC1-6):
        cm=cm.cut(cyl([7,s*25.6,zz],A['ins_hole']/2,A['ins_len'],'Y'))      # inserts; M3×8 from inside the bay plate
        HW.append(('M3×8 servo bay→chest mount + insert',1,1.0))
    P['spine_chest_mount_'+n]=cm
# 3. shoulder yoke: rings on both pipes + flanges for the shoulder-bracket tabs (|y|=57..65, M3 at x±16, z≈182) + neck tube
ZS0,ZS1=172,204
yk=box([15,0,ZS1-4],[54,130,8])                                          # top plate (ties both pipes = frame)
for s in (1,-1):
    yk=yk.union(box([0,s*61,(ZS0+ZS1)/2],[50,8,ZS1-ZS0]))                # side flanges
    yk=yk.union(box([12,s*40,ZS1-10],[24,36,6]))
    for dx in (-16,16):
        yk=yk.cut(cyl([dx,s*(65-A['ins_len']/2),182],A['ins_hole']/2,A['ins_len']+0.02,'Y')); HW.append(('M3×10 shoulder bracket→yoke + insert',1,1.1))
    yk=yk.cut(cylz(px,s*py,ZS1-8-1,A['bore']/2,10))
yk=clamp(yk,ZS0,ZS1-8)
yk=yk.union(cylz(0,0,ZS1,14,28)).cut(cylz(0,0,ZS1-9,11,40))               # neck Ø28 (wall 3), wires up to the head
for _ in range(2): HW.append(('M3×8 head→neck (insert)',1,1.0))
for zz,lab in ((ZS0,'yoke'),(0,'base')): pass
P['spine_shoulder_yoke']=yk
EXPL={'spine_pipe_L':[45,25,0],'spine_pipe_R':[45,-25,0],'spine_base_mount':[0,0,-60],'spine_chest_mount_L':[0,60,0],
      'spine_chest_mount_R':[0,-60,0],'spine_shoulder_yoke':[0,0,70]}
res={}
for n,wp in P.items():
    sv=wp.val(); c=sv.Center(); exporters.export(wp,n+'.stl',tolerance=0.15,angularTolerance=0.25); exporters.export(wp,n+'.step')
    m=sv.Volume()*(A['rho_al'] if 'pipe' in n else A['rho_pr']*A['pf'])
    bb=sv.BoundingBox()
    res[n]=dict(mass_g=m,com_mm=[c.x,c.y,c.z],valid=sv.isValid(),explode=EXPL[n],bbox=[round(v,1) for v in (bb.xmin,bb.xmax,bb.ymin,bb.ymax,bb.zmin,bb.zmax)]); print(n,round(m,1),sv.isValid())
hw_g=sum(q*u for _,q,u in HW)+4*0.0   # inserts counted in unit g
res['spine_hw']=dict(mass_g=hw_g,com_mm=[px,0,110],valid=True,explode=[0,0,0])
# ---- collisions vs the other torso parts (rigid torso, so neutral is enough) + waist upper yoke ----
others={f.split('/')[-1][:-5]:cq.importers.importStep(f).val() for f in glob.glob('../pelvis-v1/torso/*.step')
        if not any(k in f for k in ('spine_frame','pulley','tensioner'))}
WJ=json.load(open('waist.json'));DZ=WJ['summary']['params']['DZ']
others['waist_yaw_skirt']=cq.importers.importStep('waist_yaw_skirt.step').val().translate(cq.Vector(0,0,-DZ))
L=json.load(open('../pelvis-v1/torso/layout.json'))
for i,f in enumerate(L['finger_servos']):   # finger servo bodies 23(x)×12.2(y)×(z 12.2 band) PLACEHOLDER
    others[f'finger_servo_{i}']=box(f['servo_c'],[23,12.2,12.2]).val()
    others[f'pulley_{i}']=cyl([f['pulley_c'][0]+3,f['pulley_c'][1],f['pulley_c'][2]],9,6,'X').val()
OK={('spine_base_mount','hip_plate'),('spine_shoulder_yoke','shoulder_bracket_L'),('spine_shoulder_yoke','shoulder_bracket_R'),
    ('spine_chest_mount_L','servo_bay_L'),('spine_chest_mount_R','servo_bay_R')}   # designed bolted contact faces
hits=[]
for n,wp in P.items():
    for o,sv in others.items():
        try: v=wp.val().intersect(sv).Volume()
        except Exception: v=0
        if v>1.0: hits.append([n,o,round(v,1),(n,o) in OK])
for n in P:
    for m in P:
        if n<m:
            v=P[n].val().intersect(P[m].val()).Volume()
            if v>1.0: hits.append([n,m,round(v,1),False])
# front cover: its bottom/top lips get Ø13 holes for the pipes (cover_front_v13)
cf=cq.Workplane(obj=others['cover_front'])
for s_ in (1,-1): cf=cf.cut(cylz(px,s_*py,-5,6.5,260))
exporters.export(cf,'cover_front_v13.stl',tolerance=0.2,angularTolerance=0.3); exporters.export(cf,'cover_front_v13.step')
others['cover_front']=cf.val(); hits=[h for h in hits if h[1]!='cover_front']
for n,wp in P.items():
    v=wp.val().intersect(cf.val()).Volume()
    if v>1.0: hits.append([n,'cover_front_v13',round(v,1),False])
print('hits',hits)
# ---- mass / CoM update ----
TR=json.load(open('../pelvis-v1/torso/results.json'));TR=TR.get('parts',TR)
old=TR['spine_frame']; om=old['mass_print_g']; oc=old['com_mm']
new_m=sum(r['mass_g'] for r in res.values()); new_c=[sum(r['mass_g']*r['com_mm'][i] for r in res.values())/new_m for i in range(3)]
S=WJ['summary']; Mt=S['total_g']; Ct=S['com_mm']
tb=lambda c:[c[0],c[1],c[2]+DZ]
Mn=Mt-om+new_m; Cn=[(Mt*Ct[i]-om*tb(oc)[i]+new_m*tb(new_c)[i])/Mn for i in range(3)]
# torque about the waist pitch axis (rotating mass)
ZP=S['params']['ZP']; wq=S['waist_pitch']
mr0=wq['rotating_mass_g']; h0=wq['rotating_com_above_axis_mm']; f0=wq['rotating_com_fwd_mm']
mr=mr0-om+new_m
zc=lambda c:c[2]+DZ-ZP
h1=(mr0*h0-om*zc(oc)+new_m*zc(new_c))/mr; f1=(mr0*f0-om*oc[0]+new_m*new_c[0])/mr
tq={a:round(mr/1000*9.81*(h1*math.sin(math.radians(a))+f1*math.cos(math.radians(a)))/1000,2) for a in (0,15,30,45,60,-15)}
# ---- pipe strength: worst case = whole bow-60 torque taken by the spine base, 2 pipes in parallel, ×3 dynamic ----
def sec(od,t): di=od-2*t; I=math.pi/64*(od**4-di**4); return I, I/(od/2), math.pi/4*(od**2-di**2)
CAND={}
for od,t in ((10,1),(12,1),(15,1),(16,1)):
    I,Wm,Ar=sec(od,t); M=tq[60]*1000*3
    sig=M/(2*Wm); F=M/3/180; dl=F*180**3/(3*A['E_al']*2*I)
    CAND[f'{od}x{t}']=dict(I_mm4=round(I,1),W_mm3=round(Wm,1),g_per_m=round(Ar*2.7,1),sigma_MPa_x3=round(sig,1),SF_yield=round(A['sy_al']/sig,2),tip_defl_mm=round(dl,2),id_mm=od-2*t)
for k,v in CAND.items(): print(k,v)
pipe_len=z1-z0
from collections import Counter
hwc=Counter(n for n,_,_ in HW)
summary=dict(version='legs-v1.3',pipe=dict(size='Ø12×1 A6063 (home-centre stock, 1 m → cut 2 pcs)',count=2,length_mm=pipe_len,x=px,y=[py,-py]),
  candidates=CAND,removed_spine_frame_g=round(om,1),new_spine_g=round(new_m,1),delta_g=round(new_m-om,1),hardware=dict(hwc),
  total_g=round(Mn,1),com_mm=[round(v,1) for v in Cn],com_above_floor_mm=round(S['com_above_floor_mm']+(Cn[2]-Ct[2]),1),
  rotating_mass_g=round(mr,1),rotating_com_above_axis_mm=round(h1,1),rotating_com_fwd_mm=round(f1,1),torque_Nm=tq,
  torque_v12_Nm={k:v[0] for k,v in wq['torque_Nm_and_arm_mm'].items()} if isinstance(wq['torque_Nm_and_arm_mm'],dict) else wq['torque_Nm_and_arm_mm'],
  hits=hits,params=A,spine_com_torso_mm=[round(v,1) for v in new_c],old_spine_com_torso_mm=[round(v,1) for v in oc])
json.dump(dict(summary=summary,parts=res),open('spine.json','w'),indent=1)
print(json.dumps({k:summary[k] for k in ('removed_spine_frame_g','new_spine_g','delta_g','total_g','com_mm','com_above_floor_mm','torque_Nm','torque_v12_Nm','hardware')},ensure_ascii=False))
