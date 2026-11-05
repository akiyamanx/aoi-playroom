# COCOMI robot arm v3 (+forearm roll) (ここちゃん) - shoulder→hand, parametric CadQuery. Units: mm
# Each part has its own frame: joint axis = +Z through origin, part extends toward -X.
# Hand: palm side = +Z, fingers curl toward +Z about local +Y (pin axis).
import cadquery as cq, json, math
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
P = dict(
  # shoulder (same as v1) - AK45-10 dims APPROX, check datasheet
  act_d=53, act_h=45, flange_d=40, flange_t=3, bolt_pcd=30, bolt_n=8, bolt_d=3.2,
  wall=3, clear=0.5, cup_depth=35,
  # servos
  servo=(45.2, 24.7, 35.0), servo_clear=0.25, servo_wall=3, servo_shaft_from_end=10.5,  # STS3215 (shaft pos approx)
  horn_gap=6.0, horn_pcd=14.0, horn_hole=2.2,           # servo top -> next link bottom (horn+spline), horn screws approx
  grip_servo=(22.5, 22.8, 12.2),                         # SG90/MG90S class, lying flat (x=height, y=length, z=width)
  # links
  upper_len=120, upper_w=28, upper_h=18,
  fore_len=110, fore_w=26, fore_h=16,
  # hand
  palm_len=72, palm_w=68, palm_t=22, palm_back=10,
  fin_w=14, fin_t=13, pin_d=2.2, tendon_d=1.8, knuckle_gap=0.4, root_off=7,
  fingers=dict(  # name: (y, segment lengths)  pin-to-pin
    index=(25.5,[26,18,15]), middle=(8.5,[28,20,16]), ring=(-8.5,[26,18,15]), pinky=(-25.5,[21,15,13])),
  thumb=dict(x=-26, y=34, yaw_deg=55, segs=[24,18]),
  # v3 forearm roll (STS3215 lying, shaft toward -X) + 6806 thin bearing (30x42x7) placeholder
  roll_x_face=-52, brg_id=30, brg_od=42, brg_w=7, brg_fit=0.1, plate_t=8, fore_dist_len=80,
  m_bearing=0.018,
  rho=1.24e-3, print_factor=0.5,
  m_sts3215=0.055, m_grip_servo=0.0134, m_act=0.26,
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
# ---------- generic link with servo pocket at the far end
sx,sy,sz=A['servo']; c2=2*A['servo_clear']; sw=A['servo_wall']
px,py=sx+c2,sy+c2                       # pocket (+0.25 each side)
bx,by,bz=px+2*sw,py+2*sw,sz+sw
soff=sx/2-A['servo_shaft_from_end']     # servo centre is this far toward +X from shaft
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
    s=s.cut(W().workplane(offset=z0+sw).center(xc,0).rect(px,py).extrude(bz))          # servo pocket
    s=s.cut(W().workplane(offset=z0-1).center(xc+12,0).circle(5).extrude(10))           # cable hole
    s=s.cut(cq.Workplane("XZ").workplane(offset=-by).center(xc+bx/2-sw/2,z0+bz-6).circle(1.0).extrude(2*by))
    s=s.cut(cq.Workplane("XZ").workplane(offset=-by).center(xc-bx/2+sw/2,z0+bz-6).circle(1.0).extrude(2*by))
    return s, z0+sw+sz+A['horn_gap']   # z of next link bottom (horn face)
def horn_hub(rad=13,t=4):
    h=W().circle(rad).extrude(t)
    h=h.cut(W().workplane(offset=-1).polarArray(A['horn_pcd']/2,45,360,4).circle(A['horn_hole']/2).extrude(t+2))
    return h.cut(W().workplane(offset=-1).circle(3.2).extrude(t+2))
z0=A['flange_t']
hub=W().workplane(offset=z0).circle(24).extrude(6)
upper,z_elbow=link(A['upper_len'],A['upper_w'],A['upper_h'],z0,hub)
upper=upper.cut(W().workplane(offset=z0-1).polarArray(A['bolt_pcd']/2,0,360,A['bolt_n']).circle(A['bolt_d']/2).extrude(10))
upper=upper.cut(W().workplane(offset=z0-1).circle(6).extrude(10))
# ---------- v3 forearm: proximal (elbow horn -> roll servo cradle + bearing plate), distal (roll -> wrist servo)
xf=A['roll_x_face']; zr=sw+sy/2+A['servo_clear']            # roll axis height (servo lies on its 24.7 side)
ry0=-A['servo_shaft_from_end']                             # servo spans y: shaft is 10.5 from one end
cy0,cy1=ry0-A['servo_clear']-sw, ry0+sx+A['servo_clear']+sw # cradle y range
cx1=xf+35+2*A['servo_clear']+sw                             # cradle back x (servo body 35 along X)
prox=horn_hub()
prox=prox.union(W().center((cx1+0)/2-2,0).rect(-cx1+4,A['fore_w']).extrude(A['fore_h']).edges("|Z").fillet(4))
prox=prox.union(W().center((xf+cx1)/2,(cy0+cy1)/2).rect(cx1-xf,cy1-cy0).extrude(zr+sy/2+2).edges("|Z").fillet(3))
prox=prox.cut(W().workplane(offset=sw).center((xf+cx1-sw)/2-0.5,ry0+sx/2).rect(cx1-sw-xf+1,sx+2*A['servo_clear']).extrude(60))  # servo pocket, open top
pt=A['plate_t']; R=A['brg_od']/2+5
plate=cq.Workplane("YZ").workplane(offset=xf-pt).center((-R+cy1)/2,zr).rect(cy1+R,2*R).extrude(pt).edges("|X").fillet(6)
prox=prox.union(plate)
prox=prox.cut(cq.Workplane("YZ").workplane(offset=xf-pt-1).center(0,zr).circle(18).extrude(pt+2))                     # through bore (lip)
prox=prox.cut(cq.Workplane("YZ").workplane(offset=xf-pt-1).center(0,zr).circle(A['brg_od']/2+A['brg_fit']).extrude(A['brg_w']+1))  # bearing seat
prox=prox.cut(W().workplane(offset=-1).polarArray(A['horn_pcd']/2,45,360,4).circle(A['horn_hole']/2).extrude(8))
prox=prox.cut(W().workplane(offset=-1).center(xf+20,ry0+sx-6).circle(4).extrude(10))  # cable hole
xr=xf-A['horn_gap']                                        # distal frame origin: roll horn face on axis
ri_=A['brg_id']/2-0.05
dhub=cq.Workplane("YZ").workplane(offset=-6).circle(ri_).extrude(6)
dhub=dhub.union(cq.Workplane("YZ").circle(ri_).circle(12.5).extrude(xf-pt+A['brg_w']-xr+1.0 if False else (xf-xr-1)))  # spigot through bearing inner race
dist,zw=link(A['fore_dist_len'],A['fore_w'],A['fore_h'],-A['fore_h']/2,dhub)
dist=dist.cut(cq.Workplane("YZ").workplane(offset=-8).polarArray(A['horn_pcd']/2,45,360,4).circle(A['horn_hole']/2).extrude(10))
dist=dist.cut(cq.Workplane("YZ").workplane(offset=-8).circle(3.2).extrude(10))
z_wrist=zw
# ---------- hand
PL,PW,PT=A['palm_len'],A['palm_w'],A['palm_t']; fw,ft=A['fin_w'],A['fin_t']; zc=PT/2
def fork(length_back):   # knuckle fork, local frame: pin at origin along Y, body toward +X
    f=W().workplane(offset=-ft/2).center(length_back/2,0).rect(length_back,fw).extrude(ft)
    f=f.union(cq.Workplane("XZ").workplane(offset=-fw/2).circle(ft/2).extrude(fw))
    f=f.cut(W().workplane(offset=-ft/2-1).center(-ft/2,0).rect(ft*2+2,fw/3+A['knuckle_gap']).extrude(ft+2))
    return f
def seg(l,last):
    b=W().workplane(offset=-ft/2).center(-l/2,0).rect(l,fw).extrude(ft)
    b=b.union(cq.Workplane("XZ").workplane(offset=-fw/2).circle(ft/2).extrude(fw))
    b=b.union(cq.Workplane("XZ").workplane(offset=-fw/2).center(-l,0).circle(ft/2).extrude(fw))
    # proximal tongue: keep centre width fw/3
    tw=fw/3; side=(fw-tw)/2+0.01
    for sg in (1,-1):
        b=b.cut(W().workplane(offset=-ft/2-1).center(0.5,sg*(tw/2+side/2+0.0)).rect(ft+3,side+0.02).extrude(ft+2))
    b=b.cut(cq.Workplane("XZ").workplane(offset=-fw).circle(A['pin_d']/2).extrude(2*fw))
    if not last:  # distal fork slot + pin
        b=b.cut(W().workplane(offset=-ft/2-1).center(-l,0).rect(ft+3,tw+A['knuckle_gap']).extrude(ft+2))
        b=b.cut(cq.Workplane("XZ").workplane(offset=-fw).center(-l,0).circle(A['pin_d']/2).extrude(2*fw))
    b=b.cut(cq.Workplane("YZ").workplane(offset=-l-ft).center(0,ft*0.28).circle(A['tendon_d']/2).extrude(l+2*ft))  # flexor tendon
    b=b.cut(cq.Workplane("YZ").workplane(offset=-l-ft).center(0,-ft*0.28).circle(A['tendon_d']/2).extrude(l+2*ft)) # extensor (return)
    try: b=b.edges("|X").fillet(2.0)
    except Exception as e: pass
    return b
palm=W().center(-(PL+A['palm_back'])/2+A['palm_back'],0).rect(PL+A['palm_back'],PW).extrude(PT).edges("|Z").fillet(14)
try: palm=palm.edges().fillet(5)
except Exception as e: print("palm fillet skip",e)
layout=dict(fingers={},thumb=None)
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
# horn interface + grip servo pocket (open bottom) + tendon channels
palm=palm.cut(W().workplane(offset=-1).polarArray(A['horn_pcd']/2,45,360,4).circle(A['horn_hole']/2).extrude(PT+2))
palm=palm.cut(W().workplane(offset=-1).circle(3.2).extrude(PT+2))
palm=palm.cut(W().workplane(offset=-1).circle(13.3).extrude(4.5))            # recess for horn hub
gx,gy,gz=A['grip_servo']; gxc=-38
palm=palm.cut(W().workplane(offset=-1).center(gxc,0).rect(gx+0.5,gy+0.5).extrude(gz+1.5))
palm=palm.cut(W().workplane(offset=-1).center(gxc+gx/2+6,0).rect(12,gy+0.5).extrude(gz+1.5))   # horn/spool space
for nm,d in layout['fingers'].items():
    x,y,_=d['pos']
    palm=palm.cut(cq.Workplane("YZ").workplane(offset=x).center(y,zc+ft*0.28).circle(1.0).extrude(-x+gxc))
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
frames=dict(elbow=[-A['upper_len'],0,z_elbow],roll=[xr,0,zr],wrist=[-A['fore_dist_len'],0,z_wrist],
            roll_servo_c=[xf+17.5,ry0+sx/2,zr],bearing_c=[xf-pt+A['brg_w']/2,0,zr],
            elbow_servo_c=[-A['upper_len']+soff,0,z0+sw+sz/2],wrist_servo_c=[-A['fore_dist_len']+soff,0,-A['fore_h']/2+sw+sz/2],
            grip_servo_c=[gxc,0,gz/2+0.5])
json.dump(dict(params=P,frames=frames,layout=layout,parts=res),open("results.json","w"),indent=1)
json.dump(dict(frames=frames,layout=layout),open("layout.json","w"),indent=1)
for n,d in res.items(): print(f"{n:32s} {d['mass_solid_g']:7.1f} g valid={d['valid']} com={[round(x,1) for x in d['com_mm']]}")
print(frames)
