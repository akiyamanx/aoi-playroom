# COCOMI robot arm v5 — centered wrist (U-yoke) + palm faces inward (ここちゃん)
# Based on v4 (5 independent tendon servos). Units: mm.
# Each part: joint axis convention as noted; arm extends toward -X.
# Hand: palm side = -Z (medial / inward at roll=0), fingers curl toward -Z.
# Wrist pitch axis = +Y through forearm centerline (flexion/extension toward palm/back).
import cadquery as cq, json, math
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
P = dict(
  # shoulder (same as v1) - AK45-10 dims APPROX, check datasheet
  act_d=53, act_h=45, flange_d=40, flange_t=3, bolt_pcd=30, bolt_n=8, bolt_d=3.2,
  wall=3, clear=0.5, cup_depth=35,
  # servos
  servo=(45.2, 24.7, 35.0), servo_clear=0.25, servo_wall=3, servo_shaft_from_end=10.5,
  horn_gap=6.0, horn_pcd=14.0, horn_hole=2.2,
  tservo=(12.2, 23.0, 29.0), tservo_clear=0.25, tservo_rib=2.0, tservo_n=5, bay_x0=-8,
  # links
  upper_len=120, upper_w=28, upper_h=18,
  fore_len=110, fore_w=26, fore_h=16,
  # hand
  palm_len=72, palm_w=68, palm_t=22, palm_back=10,
  fin_w=14, fin_t=13, pin_d=2.2, tendon_d=1.8, knuckle_gap=0.4, root_off=7,
  fingers=dict(
    index=(25.5,[26,18,15]), middle=(8.5,[28,20,16]), ring=(-8.5,[26,18,15]), pinky=(-25.5,[21,15,13])),
  thumb=dict(x=-26, y=34, yaw_deg=55, segs=[24,18]),
  # v3 forearm roll (STS3215 lying, shaft toward -X) + 6806 thin bearing (30x42x7) placeholder
  roll_x_face=-52, brg_id=30, brg_od=42, brg_w=7, brg_fit=0.1, plate_t=8, fore_dist_len=124,
  m_bearing=0.018,
  # v5 wrist U-yoke (pivot on forearm centerline, axis = Y)
  yoke_gap=30,       # clear between inner faces of yoke arms
  yoke_arm_t=9,      # arm thickness along Y
  yoke_arm_z=40,     # arm height along Z
  yoke_reach=20,     # arms extend this far past pivot toward -X
  yoke_root=18,      # arms extend this far proximal of pivot (+X)
  wrist_pin_d=8.0,   # idler pin / bushing bore
  m_wrist_pin=0.005, # pin + small bushing approx
  rho=1.24e-3, print_factor=0.5,
  m_sts3215=0.055, m_tservo=0.011, m_tendon_misc=0.006, m_act=0.26,
)
A=P; RHO=A['rho']
r=A['act_d']/2
def W(): return cq.Workplane()
# ---------- shoulder: actuator placeholder + cup (v1 geometry)
act=W().circle(r).extrude(-A['act_h'])
act=act.union(W().circle(A['flange_d']/2).extrude(A['flange_t']).faces(">Z").workplane()
     .polarArray(A['bolt_pcd']/2,0,360,A['bolt_n']).hole(2.5,A['flange_t']))
ri=r+A['clear']; ro=ri+A['wall']; zb=-A['act_h']-A['wall']; zt=-A['act_h']+A['cup_depth']
cup=W().workplane(offset=zb).circle(ro).extrude(zt-zb).cut(W().workplane(offset=-A['act_h']).circle(ri).extrude(60))
tab=W().workplane(offset=zb).center(0,ro+6).rect(50,16).extrude(6)
tab=tab.faces(">Z").workplane().pushPoints([(-17,ro+8),(17,ro+8)]).hole(3.4)
cup=cup.union(tab)
cup=cup.cut(W().workplane(offset=zb-1).polarArray(18,45,360,4).circle(1.7).extrude(5))
cup=cup.cut(W().workplane(offset=zb-1).circle(8).extrude(5))
cup=cup.cut(W().workplane(offset=-A['act_h']+2).center(0,-ri-2).rect(14,10).extrude(10))
cup=cup.cut(cq.Workplane("YZ").workplane(offset=-ro-1).center(0,zt-8).circle(1.6).extrude(2*ro+2))
try: cup=cup.edges("%CIRCLE and >Z").fillet(1.0)
except Exception as e: print("cup fillet skip",e)
# ---------- generic link with servo pocket at the far end (upper arm / elbow)
sx,sy,sz=A['servo']; c2=2*A['servo_clear']; sw=A['servo_wall']
px,py=sx+c2,sy+c2
bx,by,bz=px+2*sw,py+2*sw,sz+sw
soff=sx/2-A['servo_shaft_from_end']
def link(L,Wd,H,z0,hub):
    ex=-L; xc=ex+soff
    beam=W().workplane(offset=z0).center(-(L-soff)/2,0).rect(L-soff,Wd).extrude(H).edges("|Z").fillet(6)
    block=W().workplane(offset=z0).center(xc,0).rect(bx,by).extrude(bz).edges("|Z").fillet(6)
    s=hub.union(beam).union(block)
    try: s=s.faces(">Z").edges().fillet(2)
    except Exception as e: print("link fillet skip",e)
    n=max(1,int((L-60)//24)); xs=[-30-i*((L-62)/max(1,n-1) if n>1 else 0) for i in range(n)]
    for cx in xs:
        s=s.cut(W().workplane(offset=z0+2.5).center(cx,0).rect(16,Wd-6).extrude(H).edges("|Z").fillet(4))
        s=s.cut(cq.Workplane("XZ").workplane(offset=-Wd).center(cx,z0+H/2+1.5).circle(3.5).extrude(2*Wd))
    s=s.cut(W().workplane(offset=z0+sw).center(xc,0).rect(px,py).extrude(bz))
    s=s.cut(W().workplane(offset=z0-1).center(xc+12,0).circle(5).extrude(10))
    s=s.cut(cq.Workplane("XZ").workplane(offset=-by).center(xc+bx/2-sw/2,z0+bz-6).circle(1.0).extrude(2*by))
    s=s.cut(cq.Workplane("XZ").workplane(offset=-by).center(xc-bx/2+sw/2,z0+bz-6).circle(1.0).extrude(2*by))
    return s, z0+sw+sz+A['horn_gap']
def horn_hub(rad=13,t=4):
    h=W().circle(rad).extrude(t)
    h=h.cut(W().workplane(offset=-1).polarArray(A['horn_pcd']/2,45,360,4).circle(A['horn_hole']/2).extrude(t+2))
    return h.cut(W().workplane(offset=-1).circle(3.2).extrude(t+2))
z0=A['flange_t']
hub=W().workplane(offset=z0).circle(24).extrude(6)
upper,z_elbow=link(A['upper_len'],A['upper_w'],A['upper_h'],z0,hub)
upper=upper.cut(W().workplane(offset=z0-1).polarArray(A['bolt_pcd']/2,0,360,A['bolt_n']).circle(A['bolt_d']/2).extrude(10))
upper=upper.cut(W().workplane(offset=z0-1).circle(6).extrude(10))
# ---------- forearm proximal (elbow horn -> roll servo + bearing) — same as v4
xf=A['roll_x_face']; zr=sw+sy/2+A['servo_clear']
ry0=-A['servo_shaft_from_end']
cy0,cy1=ry0-A['servo_clear']-sw, ry0+sx+A['servo_clear']+sw
cx1=xf+35+2*A['servo_clear']+sw
prox=horn_hub()
prox=prox.union(W().center((cx1+0)/2-2,0).rect(-cx1+4,A['fore_w']).extrude(A['fore_h']).edges("|Z").fillet(4))
prox=prox.union(W().center((xf+cx1)/2,(cy0+cy1)/2).rect(cx1-xf,cy1-cy0).extrude(zr+sy/2+2).edges("|Z").fillet(3))
prox=prox.cut(W().workplane(offset=sw).center((xf+cx1-sw)/2-0.5,ry0+sx/2).rect(cx1-sw-xf+1,sx+2*A['servo_clear']).extrude(60))
pt=A['plate_t']; R=A['brg_od']/2+5
plate=cq.Workplane("YZ").workplane(offset=xf-pt).center((-R+cy1)/2,zr).rect(cy1+R,2*R).extrude(pt).edges("|X").fillet(6)
prox=prox.union(plate)
prox=prox.cut(cq.Workplane("YZ").workplane(offset=xf-pt-1).center(0,zr).circle(18).extrude(pt+2))
prox=prox.cut(cq.Workplane("YZ").workplane(offset=xf-pt-1).center(0,zr).circle(A['brg_od']/2+A['brg_fit']).extrude(A['brg_w']+1))
prox=prox.cut(W().workplane(offset=-1).polarArray(A['horn_pcd']/2,45,360,4).circle(A['horn_hole']/2).extrude(8))
prox=prox.cut(W().workplane(offset=-1).center(xf+20,ry0+sx-6).circle(4).extrude(10))
xr=xf-A['horn_gap']
ri_=A['brg_id']/2-0.05
dhub=cq.Workplane("YZ").workplane(offset=-6).circle(ri_).extrude(6)
dhub=dhub.union(cq.Workplane("YZ").circle(ri_).circle(12.5).extrude(xf-xr-1))
# ---------- forearm distal: beam + tendon bay + U-yoke wrist (NO top-mounted wrist servo)
wx=-A['fore_dist_len']   # wrist pivot X (on centerline Y=0,Z=0)
# Beam from roll hub to yoke root (no end servo block)
beam_end=wx+A['yoke_root']+2
Lbeam=-beam_end  # positive length along -X from 0
zbeam=-A['fore_h']/2
dist=dhub.union(W().workplane(offset=zbeam).center(-Lbeam/2,0).rect(Lbeam,A['fore_w']).extrude(A['fore_h']).edges("|Z").fillet(5))
# lightening along beam (avoid bay region later)
for cx in (-25,-45,-65,-85):
    if cx > beam_end+8:
        dist=dist.cut(W().workplane(offset=zbeam+2.5).center(cx,0).rect(14,A['fore_w']-6).extrude(A['fore_h']).edges("|Z").fillet(3))
        dist=dist.cut(cq.Workplane("XZ").workplane(offset=-A['fore_w']).center(cx,0).circle(3.2).extrude(2*A['fore_w']))
dist=dist.cut(cq.Workplane("YZ").workplane(offset=-8).polarArray(A['horn_pcd']/2,45,360,4).circle(A['horn_hole']/2).extrude(10))
dist=dist.cut(cq.Workplane("YZ").workplane(offset=-8).circle(3.2).extrude(10))
# tendon servo bay (same as v4)
tx_,ty_,tz_=A['tservo']; tc=A['tservo_clear']; rib=A['tservo_rib']; nS=A['tservo_n']
pitch=tx_+2*tc+rib; bay_len=nS*pitch+rib; bx0=A['bay_x0']; bx1=bx0-bay_len
bay_w=ty_+2*tc+2*2.5; bz0=-tz_/2-2.5; bz1=tz_/2
bay=W().workplane(offset=bz0).center((bx0+bx1)/2,0).rect(bay_len,bay_w).extrude(bz1-bz0).edges("|Z").fillet(3)
servo_cs=[]
for i in range(nS):
    cx=bx0-rib-tc-tx_/2-i*pitch
    bay=bay.cut(W().workplane(offset=-tz_/2).center(cx,0).rect(tx_+2*tc,ty_+2*tc).extrude(tz_+5))
    bay=bay.cut(W().workplane(offset=bz0-1).center(cx,0).rect(6,ty_-8).extrude(4))
    servo_cs.append([cx,0,0])
bay=bay.cut(cq.Workplane("XZ").workplane(offset=-bay_w).center((bx0+bx1)/2,bz0+8).rect(bay_len-10,8).extrude(2*bay_w))
gy=[-10,-5,0,5,10]; gz=bz1+5
comb_=W().workplane(offset=bz1-1).center(bx1-2,0).rect(4,bay_w).extrude(10)
for y in gy: comb_=comb_.cut(cq.Workplane("YZ").workplane(offset=bx1-6).center(y,gz).circle(1.1).extrude(10))
dist=dist.union(bay).union(comb_)
for (cx,_,_) in servo_cs:
    dist=dist.cut(W().workplane(offset=-tz_/2).center(cx,0).rect(tx_+2*tc,ty_+2*tc).extrude(tz_+5))
# ----- U-yoke at wrist (servo +Y, idler -Y, pivot through centerline)
yg=A['yoke_gap']; yat=A['yoke_arm_t']; yaz=A['yoke_arm_z']
y_reach=A['yoke_reach']; y_root=A['yoke_root']
arm_x0=wx-y_reach; arm_x1=wx+y_root; arm_len=arm_x1-arm_x0
# +Y arm (servo side)
y_out_p=yg/2+yat
arm_p=W().workplane(offset=-yaz/2).center((arm_x0+arm_x1)/2,(yg/2+y_out_p)/2).rect(arm_len,yat).extrude(yaz)
try: arm_p=arm_p.edges("|Z").fillet(3)
except Exception: pass
# servo pocket in +Y arm: shaft along Y, body toward +Y, long dim along X
# shaft at (wx, *, 0); output face at y=yg/2; body extends +Y into the arm and beyond a bit
sc=A['servo_clear']
# footprint in XZ at shaft: sx along X, sy along Z; body depth along Y ≈ sz
servo_xc=wx+soff   # most of body proximal (+X)
# pocket cut from inner face outward (+Y), open toward -Y (center)
pocket=W().workplane(offset=-sy/2-sc).center(servo_xc, yg/2+sz/2+sc/2).rect(sx+2*sc, sz+sc).extrude(sy+2*sc)
# Actually better: cut a box in the +Y arm for the servo body
# Servo occupies: x in [servo_xc±sx/2], y in [yg/2, yg/2+sz], z in [±sy/2]  (approx, shaft at top face y=yg/2)
servo_box_x=sx+2*sc; servo_box_y=sz+2; servo_box_z=sy+2*sc
pocket=W().workplane(offset=-servo_box_z/2).center(servo_xc, yg/2+servo_box_y/2-0.5).rect(servo_box_x, servo_box_y).extrude(servo_box_z)
arm_p=arm_p.cut(pocket)
# shaft / horn clearance bore through arm along Y at (wx,0)
arm_p=arm_p.cut(cq.Workplane("XZ").workplane(offset=-y_out_p-1).center(wx,0).circle(4.5).extrude(yat+yg+2))
# cable exit
arm_p=arm_p.cut(W().workplane(offset=-yaz/2-1).center(servo_xc+sx/2-4, yg/2+sz*0.6).circle(4).extrude(yaz+2))
# -Y arm (idler)
y_out_n=-(yg/2+yat)
arm_n=W().workplane(offset=-yaz/2).center((arm_x0+arm_x1)/2,(y_out_n-yg/2)/2).rect(arm_len,yat).extrude(yaz)
try: arm_n=arm_n.edges("|Z").fillet(3)
except Exception: pass
arm_n=arm_n.cut(cq.Workplane("XZ").workplane(offset=y_out_n-1).center(wx,0).circle(A['wrist_pin_d']/2+0.15).extrude(yat+2))
# bushing seat (slightly larger, shallow)
arm_n=arm_n.cut(cq.Workplane("XZ").workplane(offset=-yg/2-0.5).center(wx,0).circle(A['wrist_pin_d']/2+1.2).extrude(3))
# bridge joining arms on proximal side (above/below beam)
bridge_t=6
bridge=W().workplane(offset=-yaz/2).center(arm_x1-bridge_t/2,0).rect(bridge_t, yg+2*yat).extrude(yaz)
bridge=bridge.cut(W().workplane(offset=-A['fore_h']/2-0.5).center(arm_x1-bridge_t/2,0).rect(bridge_t+2,A['fore_w']+1).extrude(A['fore_h']+1))  # leave beam pass? actually union with beam
# tendon tube tunnels near pivot (5 holes through bridge / near axis, along X)
tube_ys=[-8,-4,0,4,8]
for ty in tube_ys:
    bridge=bridge.cut(cq.Workplane("YZ").workplane(offset=arm_x1-bridge_t-1).center(ty,0).circle(2.2).extrude(bridge_t+4))
dist=dist.union(arm_p).union(arm_n).union(bridge)
# re-cut servo pocket after union
dist=dist.cut(W().workplane(offset=-servo_box_z/2).center(servo_xc, yg/2+servo_box_y/2-0.5).rect(servo_box_x, servo_box_y).extrude(servo_box_z))
dist=dist.cut(cq.Workplane("XZ").workplane(offset=-y_out_p-1).center(wx,0).circle(4.5).extrude(yat+2))
dist=dist.cut(cq.Workplane("XZ").workplane(offset=y_out_n-1).center(wx,0).circle(A['wrist_pin_d']/2+0.15).extrude(yat+2))
for ty in tube_ys:
    dist=dist.cut(cq.Workplane("YZ").workplane(offset=arm_x1-bridge_t-1).center(ty,0).circle(2.2).extrude(bridge_t+4))
# wrist servo center for viz / mass
wrist_servo_c=[servo_xc, yg/2+sz/2, 0]
# ---------- hand: palm centered on Z=0, wrist lug along Y for yoke
PL,PW,PT=A['palm_len'],A['palm_w'],A['palm_t']; fw,ft=A['fin_w'],A['fin_t']; zc=0.0  # centered!
def fork(length_back):
    f=W().workplane(offset=-ft/2).center(length_back/2,0).rect(length_back,fw).extrude(ft)
    f=f.union(cq.Workplane("XZ").workplane(offset=-fw/2).circle(ft/2).extrude(fw))
    f=f.cut(W().workplane(offset=-ft/2-1).center(-ft/2,0).rect(ft*2+2,fw/3+A['knuckle_gap']).extrude(ft+2))
    return f
def seg(l,last):
    b=W().workplane(offset=-ft/2).center(-l/2,0).rect(l,fw).extrude(ft)
    b=b.union(cq.Workplane("XZ").workplane(offset=-fw/2).circle(ft/2).extrude(fw))
    b=b.union(cq.Workplane("XZ").workplane(offset=-fw/2).center(-l,0).circle(ft/2).extrude(fw))
    tw=fw/3; side=(fw-tw)/2+0.01
    for sg in (1,-1):
        b=b.cut(W().workplane(offset=-ft/2-1).center(0.5,sg*(tw/2+side/2+0.0)).rect(ft+3,side+0.02).extrude(ft+2))
    b=b.cut(cq.Workplane("XZ").workplane(offset=-fw).circle(A['pin_d']/2).extrude(2*fw))
    if not last:
        b=b.cut(W().workplane(offset=-ft/2-1).center(-l,0).rect(ft+3,tw+A['knuckle_gap']).extrude(ft+2))
        b=b.cut(cq.Workplane("XZ").workplane(offset=-fw).center(-l,0).circle(A['pin_d']/2).extrude(2*fw))
    b=b.cut(cq.Workplane("YZ").workplane(offset=-l-ft).center(0,ft*0.28).circle(A['tendon_d']/2).extrude(l+2*ft))
    b=b.cut(cq.Workplane("YZ").workplane(offset=-l-ft).center(0,-ft*0.28).circle(A['tendon_d']/2).extrude(l+2*ft))
    try: b=b.edges("|X").fillet(2.0)
    except Exception: pass
    return b
# palm body centered on Z=0 (from -PT/2 to +PT/2); palm face = -Z
palm=W().workplane(offset=-PT/2).center(-(PL+A['palm_back'])/2+A['palm_back'],0).rect(PL+A['palm_back'],PW).extrude(PT).edges("|Z").fillet(14)
try: palm=palm.edges().fillet(5)
except Exception as e: print("palm fillet skip",e)
layout=dict(fingers={},thumb=None,palm_tendon_exit=None)
for nm,(y,segs) in A['fingers'].items():
    x=-PL-A['root_off']
    palm=palm.union(fork(A['root_off']+6).translate((x,y,zc)))
    palm=palm.cut(cq.Workplane("XZ").workplane(offset=-y-fw).center(x,zc).circle(A['pin_d']/2).extrude(2*fw))
    layout['fingers'][nm]=dict(pos=[x,y,zc],yaw=0,segs=segs)
T=A['thumb']; ya=math.radians(T['yaw_deg'])
dx,dy=-math.cos(ya),math.sin(ya); tx,ty=T['x']+dx*8,T['y']+dy*4
tf=fork(16).rotate((0,0,0),(0,0,1),-T['yaw_deg']).translate((tx,ty,zc))
palm=palm.union(tf)
palm=palm.cut(cq.Workplane("XZ").workplane(offset=-fw).circle(A['pin_d']/2).extrude(2*fw).rotate((0,0,0),(0,0,1),-T['yaw_deg']).translate((tx,ty,zc)))
layout['thumb']=dict(pos=[tx,ty,zc],yaw=-T['yaw_deg'],segs=T['segs'])
# wrist lug: fits in yoke gap, pivot at origin of hand frame (= wrist joint)
# lug spans Y across nearly yoke_gap, X around 0 (heel), Z short
lug_y=yg-1.2; lug_x=22; lug_z=PT-2
lug=W().workplane(offset=-lug_z/2).center(2,0).rect(lug_x,lug_y).extrude(lug_z).edges("|Z").fillet(3)
# through bore along Y for pin / horn boss
lug=lug.cut(cq.Workplane("XZ").workplane(offset=-lug_y/2-1).center(0,0).circle(4.0).extrude(lug_y+2))
# horn screw pattern on +Y face (matches servo horn)
lug=lug.cut(cq.Workplane("XZ").workplane(offset=-lug_y/2-1).center(0,0).polarArray(A['horn_pcd']/2,45,360,4).circle(A['horn_hole']/2).extrude(8))
lug=lug.cut(cq.Workplane("XZ").workplane(offset=-lug_y/2-1).center(0,0).circle(3.2).extrude(6))
# recess for horn on +Y face
lug=lug.cut(cq.Workplane("XZ").workplane(offset=lug_y/2-4).center(0,0).circle(13.0).extrude(5))
palm=palm.union(lug)
# lightening pockets from the BACK (+Z)
for yc in (17,0,-17):
    palm=palm.cut(W().workplane(offset=PT/2+1).center(-40,yc).rect(48,10.5).extrude(-(PT-2.5+1)).edges("|Z").fillet(3))
for yc in (26,-26):
    palm=palm.cut(W().workplane(offset=PT/2+1).center(-7,yc).rect(14,7).extrude(-(PT-2.5+1)).edges("|Z").fillet(3))
def hole(p0,p1,rad):
    v=cq.Vector(*p1)-cq.Vector(*p0)
    return cq.Workplane().add(cq.Solid.makeCylinder(rad,v.Length,cq.Vector(*p0),v.normalized()))
tz=-ft*0.28; exits={}   # flexor on palm side (-Z)
for nm,d in layout['fingers'].items():
    x,y,_=d['pos']; ey=y*0.45
    palm=palm.cut(hole((x+4,y,tz),(-30,y,tz),1.1)).cut(hole((-30,y,tz),(A['palm_back']+2,ey,tz),1.1)); exits[nm]=[A['palm_back'],ey,tz]
palm=palm.cut(hole((tx-4*dx,ty-4*dy,tz),(A['palm_back']+2,22,tz),1.1)); exits['thumb']=[A['palm_back'],22,tz]
# tendon entries at heel pass near lug (through lug X+ side toward forearm tubes)
for ey in (-8,-4,0,4,8):
    palm=palm.cut(cq.Workplane("YZ").workplane(offset=-2).center(ey,tz).circle(1.2).extrude(16))
layout['palm_tendon_exit']=exits
parts={'actuator_AK45-10_placeholder':act,'shoulder_cup':cup,'upper_arm':upper,'forearm_proximal':prox,'forearm_distal':dist,'palm':palm}
for nm,d in list(layout['fingers'].items())+[('thumb',layout['thumb'])]:
    for i,l in enumerate(d['segs']):
        parts[f'finger_{nm}_{i+1}']=seg(l,i==len(d['segs'])-1)
res={}
for n,w in parts.items():
    s=w.val(); g=GProp_GProps(); BRepGProp.VolumeProperties_s(s.wrapped,g)
    v=g.Mass(); c=g.CentreOfMass(); M=g.MatrixOfInertia()
    I=[[M.Value(i,j) for j in (1,2,3)] for i in (1,2,3)]
    cq.exporters.export(w,f"{n}.step"); cq.exporters.export(w,f"{n}.stl",tolerance=0.05,angularTolerance=0.15)
    res[n]=dict(volume_cm3=v/1000,mass_solid_g=v*RHO,mass_print_g=v*RHO*A['print_factor'],com_mm=[c.X(),c.Y(),c.Z()],
                I_kgm2_solid=[[x*RHO*1e-9 for x in row] for row in I],valid=s.isValid())
frames=dict(elbow=[-A['upper_len'],0,z_elbow],roll=[xr,0,zr],wrist=[wx,0,0],
            roll_servo_c=[xf+17.5,ry0+sx/2,zr],bearing_c=[xf-pt+A['brg_w']/2,0,zr],
            elbow_servo_c=[-A['upper_len']+soff,0,z0+sw+sz/2],wrist_servo_c=wrist_servo_c,
            wrist_idler_c=[wx, -(yg/2+yat/2), 0],
            tendon_servo_c=servo_cs, tendon_comb=[bx1-2,0,gz], tendon_comb_y=gy,
            tendon_tubes_y=tube_ys, yoke_gap=yg)
json.dump(dict(params=P,frames=frames,layout=layout,parts=res),open("results.json","w"),indent=1)
json.dump(dict(frames=frames,layout=layout),open("layout.json","w"),indent=1)
for n,d in res.items(): print(f"{n:32s} {d['mass_solid_g']:7.1f} g valid={d['valid']} com={[round(x,1) for x in d['com_mm']]}")
print(frames)
