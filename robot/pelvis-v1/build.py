# pelvis-v1.2 — waist yaw + pelvis + 2x 3-axis hip (yaw/roll/pitch) + thigh stubs
# Units mm. Frame = torso-v1: origin hip-plate TOP centre, +Z up, +Y left, +X forward.
# ALL dimensions / masses are PLACEHOLDERS (no datasheet check yet).
import json, math, cadquery as cq
from cadquery import exporters
W = lambda: cq.Workplane("XY")
TORSO = json.load(open('torso/layout.json'))  # torso-v1.3-low (built in ./torso from published torso-v1 v1.2)

A = dict(
  # interface (from torso-v1 hip_plate): plate bottom z=-8, motor pads to z=-12 at y=±38, 4×M3 on PCD32 (r16)
  hip_t=8, pad_t=4, pad_y=38, pad_pcd=32,   # torso polarArray radius 16 → PCD 32
  sts=(45.0, 25.0, 35.0), m_sts=0.055,          # STS3215 placeholder (L×W×H, g)
  act_d=53, act_h=45, m_act=0.260,              # AK45-10-class pancake placeholder
  bearing_od=72, bearing_id=52, bearing_t=7, m_bearing=0.045,  # waist crossed-roller/thin bearing PLACEHOLDER
  adapter_t=6,
  pelvis_w=110, pelvis_d=172, pelvis_h=56,      # v1.2 pelvis BOX (Y × X × Z) holds the battery; pitch actuators OUTSIDE ±55
  batt_l=155, batt_w=50, batt_h=45, m_batt=0.650,  # 6S 5000 mAh PLACEHOLDER, long side along X, below waist yaw
  floor_t=6, m_loop=0.025,                        # box floor; waist cable service loop PLACEHOLDER 25 g
  wing_w=200, wing_d=64, wing_t=6,             # pelvis floor wings carrying hip-yaw servos
  hip_y=80,                                     # leg line: yaw axis, pitch actuator centre (Y=±80)
  roll_x=-52,                                   # roll actuator centre (behind pitch)
  thigh=(46, 40, 80),                           # stub thigh top (X × Y × Z)
  rho=0.00124, print_factor=0.5,
  m_horn_hw=0.010,                              # horns+screws per STS joint PLACEHOLDER
)
# derived joint heights
z_ad_top = -A['hip_t'] - A['pad_t']            # -12  (bolts to pads)
z_ad_bot = z_ad_top - A['adapter_t']           # -18
sL, sW, sH = A['sts']
z_waist = z_ad_bot - sH                        # -53  STS3215 output face = WAIST YAW joint
z_pel_top = z_waist - A['bearing_t']           # -60
z_pel_bot = z_pel_top - A['pelvis_h']          # -96
z_hipyaw = z_pel_bot                           # hip yaw joint (servo body sits under pelvis? no: inside pocket)
z_yawout = z_pel_bot - sH                      # -131 yaw servo output face
z_rp = z_yawout - 6 - 38   # roll/pitch axis height: 38 mm under yoke so a ±30° rolled pitch actuator clears it
J = dict(waist=[0,0,z_waist])
for s,n in ((1,'L'),(-1,'R')):
    J['hip_yaw_'+n]=[0,s*A['hip_y'],z_pel_bot]
    J['hip_roll_'+n]=[A['roll_x'],s*A['hip_y'],z_rp]
    J['hip_pitch_'+n]=[0,s*A['hip_y'],z_rp]

def box(c,d): return W().box(*d).translate(tuple(c))
def cyl(c,r,h,axis='Z'):
    d={'X':(1,0,0),'Y':(0,1,0),'Z':(0,0,1)}[axis]
    base=[c[i]-d[i]*h/2 for i in range(3)]
    return cq.Workplane(cq.Plane(origin=tuple(base),xDir=(0,1,0) if axis=='X' else (1,0,0),normal=d)).circle(r).extrude(h)

parts={}
# 1. waist adapter: bolts to both torso hip-plate pads (4×M3 PCD32 each) + spine pattern; holds STS3215 pocket
ad = box([0,0,(z_ad_top+z_ad_bot)/2],[A['pelvis_d']-10,2*A['pad_y']+44,A['adapter_t']]).edges('|Z').fillet(8)
for s in (1,-1):
    for k in range(4):
        a=math.radians(90*k); ad=ad.cut(cyl([A['pad_pcd']/2*math.cos(a),s*A['pad_y']+A['pad_pcd']/2*math.sin(a),z_ad_bot+3],1.7,10))
# servo cradle walls around STS3215 (axis Z, long side along X)
cr = box([0,0,z_ad_bot-sH/2+2],[sL+8,sW+8,sH-4]).cut(box([0,0,z_ad_bot-sH/2],[sL+0.6,sW+0.6,sH+2]))
ad = ad.union(cr)
parts['waist_adapter']=ad
parts['waist_servo_STS3215']=box([0,0,z_ad_bot-sH/2],[sL,sW,sH])
# bearing ring (carries pelvis bending loads, servo only gives torque)
parts['waist_bearing']=cyl([0,0,z_waist-A['bearing_t']/2],A['bearing_od']/2,A['bearing_t']).cut(cyl([0,0,z_waist-A['bearing_t']/2],A['bearing_id']/2,A['bearing_t']+2))
# 2. pelvis block: hollow box + floor wings that reach out to the hip-yaw servos
pw,pd,ph=A['pelvis_w'],A['pelvis_d'],A['pelvis_h']
pel = box([0,0,(z_pel_top+z_pel_bot)/2],[pd,pw,ph]).edges('|Z').fillet(12)
pel = pel.cut(box([0,0,(z_pel_top+z_pel_bot)/2-2],[pd-8,pw-8,ph]).edges('|Z').fillet(8))
pel = pel.union(box([0,0,z_pel_bot+A['wing_t']/2],[A['wing_d'],A['wing_w'],A['wing_t']]).edges('|Z').fillet(10))
for s_ in (1,-1):
    pel = pel.cut(cyl([0,s_*A['hip_y'],z_pel_bot+3],6,8))
    pel = pel.cut(box([0,s_*(pw/2+ (A['wing_w']/2-pw/2)/2),z_pel_bot+A['wing_t']/2],[A['wing_d']-30,18,10]))  # lightening
pel = pel.union(box([0,0,z_pel_bot+A['floor_t']/2],[pd,pw,A['floor_t']]).edges('|Z').fillet(12))  # v1.2 full floor
pel = pel.cut(cyl([0,0,z_pel_top-2],10,6))   # cable pass-through from service loop
pel = pel.cut(box([-pd/2,0,z_pel_bot+A['floor_t']+A['batt_h']/2+2],[10,A['batt_w']+6,A['batt_h']+4]))  # rear hatch (battery swap) PLACEHOLDER
parts['pelvis_block']=pel
# v1.2 battery INSIDE the pelvis box, below the waist yaw → turns with the pelvis
bz0=z_pel_bot+A['floor_t']
parts['pelvis_battery_pack']=box([0,0,bz0+A['batt_h']/2],[A['batt_l'],A['batt_w'],A['batt_h']])
st=None
for dx in (-45,45):
    b_=box([dx,0,bz0+(A['batt_h']+2)/2],[20,A['batt_w']+4,A['batt_h']+2]).cut(box([dx,0,bz0+A['batt_h']/2-0.9],[22,A['batt_w']+0.4,A['batt_h']+2.2]))
    st=b_ if st is None else st.union(b_)
parts['pelvis_battery_strap']=st
# waist cable service loop: 300° slack arc around the waist servo, ±45° yaw (PLACEHOLDER)
lz=z_waist+12; lr=45
arc=cq.Workplane('XY').workplane(offset=lz).moveTo(lr,0).radiusArc((lr*math.cos(math.radians(300)),lr*math.sin(math.radians(300))),-lr)
prof=cq.Workplane('YZ',origin=(lr,0,lz)).circle(3.5)
parts['waist_cable_loop']=prof.sweep(arc).union(cyl([lr*math.cos(math.radians(300)),lr*math.sin(math.radians(300)),(lz+z_pel_top)/2],3.5,lz-z_pel_top))
ad_=A['act_d']; ah=A['act_h']; r_=ad_/2
for s,n in ((1,'L'),(-1,'R')):
    y=s*A['hip_y']
    # hip yaw: STS3215 hung under the wing, output downward (axis Z at y=±hip_y)
    parts['hip_yaw_servo_'+n]=box([0,y,z_pel_bot-sH/2],[sL,sW,sH])
    parts['hip_yaw_mount_'+n]=box([0,y,z_pel_bot-sH/2],[sL+8,sW+8,sH-6]).cut(box([0,y,z_pel_bot-sH/2],[sL+0.6,sW+0.6,sH]))
    # yaw output yoke → roll actuator (axis X) behind the leg line; its axis passes the leg line
    xr=A['roll_x']; xback=xr-ah/2
    yk = box([(xr+20)/2-3,y,z_yawout-3],[abs(xr)+46,40,6])
    yk = yk.union(box([xback-3,y,(z_yawout+z_rp)/2-3],[6,40,z_yawout-z_rp+6]))
    parts['hip_yaw_yoke_'+n]=yk
    parts['hip_roll_act_'+n]=cyl(J['hip_roll_'+n],r_,ah,'X')
    # roll output (front face x=xr+ah/2) → plate tangent to back of pitch stator + cradle under it
    xf=xr+ah/2
    rb = box([xf+1.5,y,z_rp],[3,ah+8,44])
    rb = rb.union(box([(xf+10)/2,y,z_rp-r_-3],[10-xf,ah-4,6]))   # under-cradle
    parts['hip_roll_bracket_'+n]=rb
    # pitch actuator: axis Y, OUTSIDE the pelvis block (|y| 57.5..102.5 vs block ±55)
    parts['hip_pitch_act_'+n]=cyl(J['hip_pitch_'+n],r_,ah,'Y')
    # thigh stub: two side plates on the actuator faces (outer=rotor flange, inner=idler) + tube
    tx,ty,tz=A['thigh']; pt=4
    th=None
    for yy in (y+s*(ah/2+pt/2+1), y-s*(ah/2+pt/2+1)):
        pl = cq.Workplane('XZ',origin=(0,yy+pt/2,0)).center(0,z_rp).circle(20).extrude(pt)
        pl = pl.union(box([0,yy,z_rp-tz/2],[36,pt,tz]))
        th = pl if th is None else th.union(pl)
    th = th.union(box([0,y,z_rp-tz+4],[36,ah+2*pt+2,8]))
    th = th.union(box([0,y,z_rp-tz-26],[30,30,60]).edges('|Z').fillet(6))
    parts['thigh_stub_'+n]=th

BOUGHT={'pelvis_battery_pack':A['m_batt'],'waist_cable_loop':A['m_loop'],'waist_servo_STS3215':A['m_sts'],'waist_bearing':A['m_bearing'],
        'hip_yaw_servo_L':A['m_sts'],'hip_yaw_servo_R':A['m_sts'],
        'hip_roll_act_L':A['m_act'],'hip_roll_act_R':A['m_act'],'hip_pitch_act_L':A['m_act'],'hip_pitch_act_R':A['m_act']}
res={}; bodies=[]
for n,wp in parts.items():
    s=wp.val(); v=s.Volume()/1000; c=s.Center()
    exporters.export(wp,n+'.stl',tolerance=0.2,angularTolerance=0.3); exporters.export(wp,n+'.step')
    m = BOUGHT[n] if n in BOUGHT else v*1000*A['rho']*A['print_factor']/1000
    res[n]=dict(volume_cm3=v,mass_g=m*1000,com_mm=[c.x,c.y,c.z],bought=n in BOUGHT,valid=s.isValid())
    bodies.append((m,[c.x,c.y,c.z]))
    print(f"{n:24s} {m*1000:7.1f} g valid={s.isValid()}")
hw_g = 3*A['m_horn_hw']*1000 + 4*15   # STS horns + 4× actuator flange screws(15 g each) PLACEHOLDER
bodies.append((hw_g/1000,[0,0,(z_waist+z_rp)/2]))
def comb(bs):
    M=sum(b[0] for b in bs); return M,[sum(b[0]*b[1][i] for b in bs)/M for i in range(3)]
mp,cp = comb(bodies)
mu,cu = TORSO['mass_upper_g']/1000, TORSO['com_upper_mm']
mt,ct = comb(bodies+[(mu,cu)])
batt_g_below_waist=A['m_batt']*1000
summary=dict(battery_below_waist_yaw_g=batt_g_below_waist,version='pelvis-v1.2',coords='origin=torso hip_plate top centre; +Z up +Y left +X fwd',
  joints_mm=J, joint_order_per_leg=['hip_yaw (Z, STS3215)','hip_roll (X, AK45-10 class)','hip_pitch (Y, AK45-10 class)'],
  pelvis_and_hips_g=round(mp*1000,1), com_pelvis_mm=[round(x,1) for x in cp],
  upper_body_g=round(mu*1000,1), com_upper_mm=[round(x,1) for x in cu],
  total_g=round(mt*1000,1), com_total_mm=[round(x,1) for x in ct],
  com_above_hip_pitch_axis_mm=round(ct[2]-z_rp,1),
  hardware_misc_g=hw_g,
  assumptions=['ALL dims/masses PLACEHOLDER','STS3215 45×25×35 mm 55 g','AK45-10 class 53 dia × 45 mm 260 g (same as torso-v1 shoulder)',
    'PLA 1.24 g/cm³ × 50% print factor','waist thin bearing 72/52×7 mm 45 g carries bending; servo torque only',
    'thigh = stub only; legs below hip pitch not counted','torso = torso-v1.3-low-nobatt: spine/shoulders −30 mm, NO battery (moved to pelvis box below waist yaw in v1.2)','battery 6S 5000 mAh 650 g PLACEHOLDER in pelvis box, rear hatch','waist cable service loop 300° arc r45 PLACEHOLDER 25 g','arms hanging (torso estimate, shoulders z=220)'],
  params=A)
json.dump(dict(parts=res,summary=summary),open('results.json','w'),indent=1)
json.dump(dict(joints=J,z_rp=z_rp,com_total_mm=ct,total_g=mt*1000,pelvis_g=mp*1000,upper_g=mu*1000,
  parts={n:{'parent':('torso' if n.startswith('waist_adapter') or n=='waist_servo_STS3215' else
     'pelvis' if n in('waist_bearing','pelvis_block','pelvis_battery_pack','pelvis_battery_strap','waist_cable_loop') or n.startswith('hip_yaw_servo') or n.startswith('hip_yaw_mount') else
     'yaw_'+n[-1] if n.startswith('hip_yaw_yoke') or n.startswith('hip_roll_act') else
     'roll_'+n[-1] if n.startswith('hip_roll_bracket') or n.startswith('hip_pitch_act') else 'pitch_'+n[-1])} for n in parts}),
  open('layout.json','w'),indent=1)
print('PELVIS+HIPS g',round(mp*1000,1),'TOTAL g',round(mt*1000,1),'CoM',[round(x,1) for x in ct],'z_rp',z_rp)
