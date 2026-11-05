# COCOMI robot torso v1 — internal layout for bipedal ~70cm humanoid (ここちゃん)
# Units: mm. Coordinate: origin = hip-plate top center; +Z up, +Y left, +X forward.
# Finger servos live in the chest (moved from forearm); Bowden cables to shoulders.
import cadquery as cq, json, math, os, shutil
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp

P = dict(
  # overall proportions (~70 cm robot; Mini Pi Plus class)
  hip_w=120, hip_d=90, hip_t=8,
  spine_h=230,          # v1.3-low: was 260 (−30)
  DZ=-30,          # hip top → neck base
  spine_w=28, spine_d=18, spine_wall=3,
  shoulder_y=100,       # half-width to shoulder pitch axis
  shoulder_z=220,       # was 250 (−30)       # height of shoulder pitch axis above hip
  chest_w=170, chest_d=110, chest_h=170,
  # battery tray — 6S 5000 mAh Li-ion soft pack PLACEHOLDER
  # Typical dims ~155×50×45 mm, mass ~650 g (cells+PCM+wrap). Removable, above waist.
  batt_l=155, batt_w=50, batt_h=45, m_batt=0.650,
  batt_x=-35,           # v1.3-low: pack behind spine column (spine passes at x=0)
  batt_z=2,             # v1.3-low: thin strap tray right on hip plate (was 35)
  # micro finger servos (same placeholder as arm-v5)
  tservo=(12.2, 23.0, 29.0), m_tservo=0.011,
  pulley_d=18, pulley_t=6,
  # spring tensioner (idler pulley + lever + screw adj) PLACEHOLDER
  tens_arm=28, tens_pulley_d=14, tens_pulley_t=5, m_tens=0.008,
  # shoulder actuator AK45-10 PLACEHOLDER (same as arm)
  act_d=53, act_h=45, flange_d=40, flange_t=3, bolt_pcd=30, bolt_n=8, bolt_d=3.2, m_act=0.26,
  # hip motor mount placeholders (for future legs) — STS3215-class footprint
  hip_servo=(45.2, 24.7, 35.0), hip_mount_y=38,
  # electronics
  rpi=(85, 56, 17), m_rpi=0.060,          # Pi 5 + short heatsink approx
  driver=(65, 45, 12), m_driver=0.030,    # servo driver board ×2
  imu=(25, 20, 5), m_imu=0.005,
  # neck / head placeholder
  neck_h=30, neck_d=28, head_r=42, m_head=0.080,
  # covers
  cover_t=2.0, cover_clear=3.0,
  # materials
  rho=1.24e-3, print_factor=0.5,
  # arm attachment (reuse arm-v5 masses; finger servos removed from arm tally)
  arm_total_g=705.6, arm_finger_servo_g=55.0,  # 5×11 g
  m_sts3215=0.055, m_tendon_misc=0.010,  # Bowden tubes+wire per side misc
)

A = P; RHO = A['rho']
W = lambda: cq.Workplane("XY")

def vol_props(shape):
    s = shape.val(); g = GProp_GProps(); BRepGProp.VolumeProperties_s(s.wrapped, g)
    v = g.Mass(); c = g.CentreOfMass(); M = g.MatrixOfInertia()
    I = [[M.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)]
    return v, [c.X(), c.Y(), c.Z()], I, s.isValid()

def export_part(name, workplane, res):
    v, com, I, valid = vol_props(workplane)
    cq.exporters.export(workplane, f"{name}.step")
    cq.exporters.export(workplane, f"{name}.stl", tolerance=0.08, angularTolerance=0.2)
    res[name] = dict(
        volume_cm3=v / 1000,
        mass_solid_g=v * RHO,
        mass_print_g=v * RHO * A['print_factor'],
        com_mm=com,
        I_kgm2_solid=[[x * RHO * 1e-9 for x in row] for row in I],
        valid=valid,
    )
    print(f"{name:28s} {res[name]['mass_solid_g']:7.1f} g valid={valid} com={[round(x,1) for x in com]}")
    return res[name]

# =============================================================================
# 1. HIP PLATE — low, centred; hip-motor mount placeholders for future legs
# =============================================================================
hw, hd, ht = A['hip_w'], A['hip_d'], A['hip_t']
hip = W().workplane(offset=-ht).rect(hd, hw).extrude(ht).edges("|Z").fillet(8)
# lightening
hip = hip.cut(W().workplane(offset=-ht - 1).rect(hd - 24, hw - 30).extrude(ht - 2).edges("|Z").fillet(6))
# spine bolt pattern (4× M3)
for dx, dy in [(-10, -10), (-10, 10), (10, -10), (10, 10)]:
    hip = hip.cut(W().workplane(offset=-ht - 1).center(dx, dy).circle(1.7).extrude(ht + 2))
# hip motor mount pads (left/right) — circular flange seats facing down/out
sx, sy, sz = A['hip_servo']
for side in (1, -1):
    pad = W().workplane(offset=-ht - 4).center(0, side * A['hip_mount_y']).circle(22).extrude(4)
    hip = hip.union(pad)
    # bolt ring + center bore (PLACEHOLDER spacing)
    hip = hip.cut(W().workplane(offset=-ht - 5).center(0, side * A['hip_mount_y']).circle(8).extrude(10))
    hip = hip.cut(
        W().workplane(offset=-ht - 5).center(0, side * A['hip_mount_y'])
        .polarArray(16, 0, 360, 4).circle(1.7).extrude(10)
    )

# =============================================================================
# 2. SPINE FRAME — backbone + ribs
# =============================================================================
sh = A['spine_h']; sw = A['spine_w']; sd = A['spine_d']; wall = A['spine_wall']
# vertical column (hollow-ish with side slots)
spine = W().rect(sd, sw).extrude(sh).edges("|Z").fillet(4)
# front/back lightening slots
for xo in (1, -1):
    spine = spine.cut(
        cq.Workplane("YZ").workplane(offset=xo * (sd / 2 + 1))
        .center(0, sh / 2).rect(sw - 10, sh - 40).extrude(-wall - 2).edges("|X").fillet(4)
    )
# cross ribs at battery, servo bay, electronics, shoulder
rib_zs = [110 + A['DZ'], 170 + A['DZ'], A['shoulder_z'] - 20]  # battery rib removed (strap holds pack)
ribs = None
for z in rib_zs:
    rib = W().workplane(offset=z - 3).rect(A['chest_d'] - 20, A['chest_w'] - 30).extrude(6)
    rib = rib.cut(W().workplane(offset=z - 4).rect(A['chest_d'] - 40, A['chest_w'] - 50).extrude(8))
    # pass-through for spine
    rib = rib.cut(W().workplane(offset=z - 4).rect(sd + 2, sw + 2).extrude(8))
    try:
        rib = rib.edges("|Z").fillet(3)
    except Exception:
        pass
    ribs = rib if ribs is None else ribs.union(rib)
spine = spine.union(ribs)
# shoulder cross-beam
beam = W().workplane(offset=A['shoulder_z'] - 12).rect(40, 2 * A['shoulder_y'] + 20).extrude(16)
beam = beam.edges("|Z").fillet(6)
spine = spine.union(beam)
# neck stub
neck = W().workplane(offset=sh).circle(A['neck_d'] / 2).extrude(A['neck_h'])
neck = neck.cut(W().workplane(offset=sh - 1).circle(A['neck_d'] / 2 - 3).extrude(A['neck_h'] + 2))
spine = spine.union(neck)
# cable pass holes through ribs / beam toward shoulders
for y in (-40, -20, 20, 40):
    spine = spine.cut(
        cq.Workplane("XZ").workplane(offset=-y - 5).center(0, A['shoulder_z'] - 4).circle(4).extrude(10)
    )

# =============================================================================
# 3. SHOULDER BRACKETS (L/R) — AK45-10 cup facing outward along ±Y
# =============================================================================
def shoulder_bracket(side):
    """side=+1 left (robot left = +Y), -1 right."""
    r = A['act_d'] / 2
    # cup body: cylinder along Y, open outward
    # local: build at origin then place
    clear = 0.5; wall = 3; depth = 35
    ri = r + clear; ro = ri + wall
    # build cup with axis along Z first, then rotate
    cup = W().workplane(offset=-A['act_h'] - wall).circle(ro).extrude(A['act_h'] + wall + depth)
    cup = cup.cut(W().workplane(offset=-A['act_h']).circle(ri).extrude(60))
    # mounting tab toward spine
    tab = W().workplane(offset=-A['act_h'] - wall).center(0, -(ro + 8)).rect(50, 18).extrude(8)
    tab = tab.faces(">Z").workplane().pushPoints([(-16, -(ro + 8)), (16, -(ro + 8))]).hole(3.4)
    cup = cup.union(tab)
    # bolt pattern on back
    cup = cup.cut(W().workplane(offset=-A['act_h'] - wall - 1).polarArray(18, 45, 360, 4).circle(1.7).extrude(6))
    cup = cup.cut(W().workplane(offset=-A['act_h'] - wall - 1).circle(8).extrude(6))
    # cable exit
    cup = cup.cut(W().workplane(offset=-A['act_h'] + 2).center(0, ri + 1).rect(14, 10).extrude(12))
    # rotate so cup axis = Y (side): Z→Y
    # rotate +90 about X: Z→Y, Y→-Z
    cup = cup.rotate((0, 0, 0), (1, 0, 0), 90)
    # now flange plane is at y=0 roughly; actuator body toward -Y after this
    # We want open end facing outward (+Y for left, -Y for right)
    if side < 0:
        cup = cup.mirror(mirrorPlane="XZ")
    # translate to shoulder position: axis at (0, side*shoulder_y, shoulder_z)
    # After rot, flange near y=0; move so flange sits at shoulder_y
    cup = cup.translate((0, side * A['shoulder_y'], A['shoulder_z']))
    return cup

bracket_L = shoulder_bracket(+1)
bracket_R = shoulder_bracket(-1)

# =============================================================================
# 4. BATTERY TRAY — removable, just above waist, centred
# =============================================================================
bl, bw, bh = A['batt_l'], A['batt_w'], A['batt_h']
tray_wall = 2.0; tray_floor = 2   # v1.3-low: THIN tray + strap (PLACEHOLDER)
tz0 = A['batt_z']
lip = 8
tray = W().workplane(offset=tz0).rect(bw + 2 * tray_wall, bl + 2 * tray_wall).extrude(tray_floor + lip)
tray = tray.cut(W().workplane(offset=tz0 + tray_floor).rect(bw + 0.6, bl + 0.6).extrude(lip + 2))
# spine pass-through (pack is split L/R of spine? NO — PLACEHOLDER: spine column stops at tray; see specs)
# strap slots (2×) in the lips + strap loops over the pack (hook-and-loop strap PLACEHOLDER 20 mm)
for dy in (-45, 45):
    tray = tray.cut(W().workplane(offset=tz0 + tray_floor + 2).center(0, dy).rect(bw + 10, 21).extrude(4))
strap = None
for dy in (-45, 45):
    st = (W().workplane(offset=tz0).center(0, dy).rect(bw + 2 * tray_wall + 2, 20).extrude(tray_floor + bh + 2)
          .cut(W().workplane(offset=tz0 - 1).center(0, dy).rect(bw + 2 * tray_wall, 22).extrude(tray_floor + bh + 1.5)))
    strap = st if strap is None else strap.union(st)
try:
    tray = tray.edges("|Z").fillet(1.5)
except Exception:
    pass
tray = tray.union(strap).translate((A['batt_x'], 0, 0))

# battery pack solid (for viz / mass) — not printed
batt = (
    W().workplane(offset=tz0 + tray_floor + 1)
    .rect(bw, bl)
    .extrude(bh - 2)
    .edges("|Z").fillet(4)
).translate((A['batt_x'], 0, 0))

# =============================================================================
# 5. FINGER SERVO BANKS + PULLEYS + SPRING TENSIONERS (5 per side)
# =============================================================================
tx, ty, tz = A['tservo']  # 12.2 × 23 × 29
# Layout: each side has a vertical column of 5 servos in the chest
# Servo body: long axis (29=tz) along X (fore-aft), shaft along Z (up), stacked along Z
# Actually for Bowden routing to shoulder: shaft along X (forward), pulley in sagittal plane,
# wire exits toward ±Y then up to shoulder.

def make_servo_bank(side):
    """Return list of (servo_solid, pulley, tensioner, centers) for one side.
    side=+1 left, -1 right. Servos sit in chest, medial of shoulder.
    """
    items = []
    # bank origin: lateral position, forward a bit, mid-chest height
    y0 = side * 48  # left/right of spine
    x0 = 18         # slightly forward of spine
    z_base = 95 + A['DZ']     # bottom servo centre
    pitch_z = 22    # vertical spacing
    for i in range(5):
        zc = z_base + i * pitch_z
        # servo body: dims (tx along X?); use (depth X=23=ty, width Y=12.2=tx, height Z=29=tz)
        # shaft along +X (forward), body extends -X
        sc = [x0 - ty / 2, y0, zc]  # body centre
        # tensioner sits lateral-outboard of servo (toward shoulder), same height
        tc = [x0, y0 + side * 28, zc]
        items.append(dict(servo_c=sc, tens_c=tc, pulley_c=[x0 + 2, y0, zc], idx=i, side=side))
    return items

bank_L = make_servo_bank(+1)
bank_R = make_servo_bank(-1)
all_banks = bank_L + bank_R

# Servo bay plate (printable mount rail for one side)
def servo_bay_plate(side, bank):
    y0 = side * 48
    z0 = bank[0]['servo_c'][2] - 16
    z1 = bank[-1]['servo_c'][2] + 16
    h = z1 - z0
    # back plate
    plate = (
        cq.Workplane("YZ")
        .workplane(offset=5)
        .center(y0, (z0 + z1) / 2)
        .rect(38, h)
        .extrude(4)
    )
    # shelves / pockets per servo
    for b in bank:
        zc = b['servo_c'][2]
        # U-pocket opening forward
        shelf = (
            W().workplane(offset=zc - tz / 2 - 1.5)
            .center(x0 := 8, y0)
            .rect(ty + 6, tx + 8)
            .extrude(3)
        )
        plate = plate.union(shelf)
        # cut servo pocket
        plate = plate.cut(
            W().workplane(offset=zc - tz / 2)
            .center(10, y0)
            .rect(ty + 0.5, tx + 0.5)
            .extrude(tz + 2)
        )
        # tensioner pivot hole
        ty_, tz_ = b['tens_c'][1], b['tens_c'][2]
        plate = plate.union(
            W().workplane(offset=tz_ - 4).center(12, ty_).circle(8).extrude(8)
        )
        plate = plate.cut(
            W().workplane(offset=tz_ - 5).center(12, ty_).circle(1.7).extrude(10)
        )
    try:
        plate = plate.edges("|X").fillet(1.5)
    except Exception:
        pass
    return plate

bay_L = servo_bay_plate(+1, bank_L)
bay_R = servo_bay_plate(-1, bank_R)

# Pulley (printable) — one representative exported; instances in viewer
pulley = (
    W().circle(A['pulley_d'] / 2)
    .extrude(A['pulley_t'])
    .faces(">Z").workplane()
    .hole(2.2)
)
# groove
pulley = pulley.cut(
    W().workplane(offset=A['pulley_t'] / 2 - 1.2).circle(A['pulley_d'] / 2 + 0.1).circle(A['pulley_d'] / 2 - 1.5).extrude(2.4)
)

# Spring tensioner assembly (printable arm + idler)
def tensioner_arm():
    arm_len = A['tens_arm']
    arm = W().center(arm_len / 2, 0).rect(arm_len, 10).extrude(4)
    # pivot boss
    arm = arm.union(W().circle(6).extrude(4))
    arm = arm.cut(W().workplane(offset=-1).circle(1.7).extrude(6))
    # idler axle at far end
    arm = arm.union(W().center(arm_len, 0).circle(5).extrude(4))
    arm = arm.cut(W().workplane(offset=-1).center(arm_len, 0).circle(1.7).extrude(6))
    # spring hook notch
    arm = arm.cut(W().workplane(offset=-1).center(arm_len * 0.45, 4).rect(4, 3).extrude(6))
    # adjustment screw tower
    tower = W().center(arm_len * 0.3, -8).rect(8, 10).extrude(10)
    tower = tower.cut(W().workplane(offset=-1).center(arm_len * 0.3, -8).circle(1.6).extrude(12))
    arm = arm.union(tower)
    try:
        arm = arm.edges("|Z").fillet(1)
    except Exception:
        pass
    return arm

tens_arm = tensioner_arm()
tens_pulley = (
    W().circle(A['tens_pulley_d'] / 2)
    .extrude(A['tens_pulley_t'])
    .faces(">Z").workplane().hole(2.2)
)

# =============================================================================
# 6. BOWDEN TUBE EXIT GUIDES — toward each shoulder
# =============================================================================
def bowden_exit(side):
    """Comb block near shoulder with 5 tube holes aiming toward arm."""
    y = side * (A['shoulder_y'] - 22)
    z = A['shoulder_z'] - 8
    block = W().workplane(offset=z - 12).center(5, y).rect(28, 22).extrude(24)
    # 5 holes along Z, exiting laterally
    for i, dz in enumerate([-8, -4, 0, 4, 8]):
        # hole along Y toward shoulder
        block = block.cut(
            cq.Workplane("XZ")
            .workplane(offset=-(y + side * 12))
            .center(5, z + dz)
            .circle(2.2)
            .extrude(side * 30)
        )
    # mount holes
    for dx in (-8, 8):
        block = block.cut(
            W().workplane(offset=z - 13).center(dx, y).circle(1.7).extrude(10)
        )
    try:
        block = block.edges("|Z").fillet(2)
    except Exception:
        pass
    return block

bowden_L = bowden_exit(+1)
bowden_R = bowden_exit(-1)

# =============================================================================
# 7. ELECTRONICS BAY + IMU MOUNT
# =============================================================================
# Bay behind spine, mid-torso (RPi + 2 drivers)
ex, ey, ez = A['rpi']
bay_z = 150 + A['DZ']
ebox = (
    W().workplane(offset=bay_z - ez / 2 - 3)
    .center(-A['chest_d'] / 2 + 30, 0)  # toward back
    .rect(ex + 10, ey + 16)
    .extrude(ez + 8)
)
ebox = ebox.cut(
    W().workplane(offset=bay_z - ez / 2)
    .center(-A['chest_d'] / 2 + 30, 0)
    .rect(ex + 1, ey + 1)
    .extrude(ez + 4)
)
# ventilation slots
for dy in (-20, 0, 20):
    ebox = ebox.cut(
        cq.Workplane("YZ")
        .workplane(offset=-A['chest_d'] / 2 + 30 - (ex + 10) / 2 - 1)
        .center(dy, bay_z)
        .rect(8, 20)
        .extrude(6)
    )
# IMU mount near CoM estimate (~ battery / lower chest centre)
imu_z = 80 + A['DZ']
imu_mount = (
    W().workplane(offset=imu_z)
    .center(0, 0)
    .rect(A['imu'][0] + 8, A['imu'][1] + 8)
    .extrude(4)
)
imu_mount = imu_mount.cut(
    W().workplane(offset=imu_z + 1)
    .center(0, 0)
    .rect(A['imu'][0] + 0.4, A['imu'][1] + 0.4)
    .extrude(4)
)
for dx, dy in [(-8, -6), (-8, 6), (8, -6), (8, 6)]:
    imu_mount = imu_mount.cut(
        W().workplane(offset=imu_z - 1).center(dx, dy).circle(1.2).extrude(8)
    )
ebox = ebox.union(imu_mount)

# =============================================================================
# 8. OUTER SHELL COVERS (translucent, simple, removable)
# =============================================================================
cw, cd, ch = A['chest_w'], A['chest_d'], A['chest_h']
ct = A['cover_t']
cover_z0 = 20  # above hip a bit
# Front half-shell
outer = (
    W().workplane(offset=cover_z0)
    .rect(cd + 2 * A['cover_clear'], cw + 2 * A['cover_clear'])
    .extrude(ch)
    .edges("|Z").fillet(18)
)
inner = (
    W().workplane(offset=cover_z0 + ct)
    .rect(cd + 2 * A['cover_clear'] - 2 * ct, cw + 2 * A['cover_clear'] - 2 * ct)
    .extrude(ch - ct)
    .edges("|Z").fillet(16)
)
shell = outer.cut(inner)
# split front / back by cutting at X=0 plane region
front_cover = shell.cut(W().workplane(offset=cover_z0 - 1).center(-cd, 0).rect(cd * 2, cw * 3).extrude(ch + 10))
back_cover = shell.cut(W().workplane(offset=cover_z0 - 1).center(cd, 0).rect(cd * 2, cw * 3).extrude(ch + 10))
# arm holes
for side in (1, -1):
    for cov in (front_cover, back_cover):
        pass
# cut shoulder arm openings in both
for name, cov in [('front', front_cover), ('back', back_cover)]:
    pass
front_cover = front_cover.cut(
    cq.Workplane("YZ").workplane(offset=-200).center(-A['shoulder_y'], A['shoulder_z']).circle(32).extrude(400)
)
front_cover = front_cover.cut(
    cq.Workplane("YZ").workplane(offset=-200).center(A['shoulder_y'], A['shoulder_z']).circle(32).extrude(400)
)
back_cover = back_cover.cut(
    cq.Workplane("YZ").workplane(offset=-200).center(-A['shoulder_y'], A['shoulder_z']).circle(32).extrude(400)
)
back_cover = back_cover.cut(
    cq.Workplane("YZ").workplane(offset=-200).center(A['shoulder_y'], A['shoulder_z']).circle(32).extrude(400)
)
# neck hole
front_cover = front_cover.cut(W().workplane(offset=sh - 5).circle(A['neck_d'] / 2 + 4).extrude(40))
back_cover = back_cover.cut(W().workplane(offset=sh - 5).circle(A['neck_d'] / 2 + 4).extrude(40))

# =============================================================================
# 9. HEAD PLACEHOLDER (simple)
# =============================================================================
head = (
    W().workplane(offset=sh + A['neck_h'] + A['head_r'] * 0.3)
    .sphere(A['head_r'])
)
# flat face cut
head = head.cut(
    W().workplane(offset=sh + A['neck_h'] + A['head_r'] * 0.3 + 20)
    .center(A['head_r'], 0)
    .rect(30, 80)
    .extrude(40)
)

# =============================================================================
# EXPORT ALL PRINTABLE / STRUCTURAL PARTS
# =============================================================================
parts = {
    'hip_plate': hip,
    'spine_frame': spine,
    'shoulder_bracket_L': bracket_L,
    'shoulder_bracket_R': bracket_R,
    'battery_tray': tray,
    'battery_pack': batt,  # placeholder solid for viz
    'servo_bay_L': bay_L,
    'servo_bay_R': bay_R,
    'pulley': pulley,
    'tensioner_arm': tens_arm,
    'tensioner_pulley': tens_pulley,
    'bowden_exit_L': bowden_L,
    'bowden_exit_R': bowden_R,
    'electronics_bay': ebox,
    'cover_front': front_cover,
    'cover_back': back_cover,
    'head_placeholder': head,
}

res = {}
for n, w in parts.items():
    export_part(n, w, res)

# =============================================================================
# MASS / CoM ASSEMBLY (upper body)
# =============================================================================
def box_body(m_kg, c_mm, dims_mm):
    a, b, c = [x / 1000 for x in dims_mm]
    return dict(
        m=m_kg,
        c=[x / 1000 for x in c_mm],
        I=[[m_kg * (b * b + c * c) / 12, 0, 0], [0, m_kg * (a * a + c * c) / 12, 0], [0, 0, m_kg * (a * a + b * b) / 12]],
    )

def part_body(name, solid=False, offset=(0, 0, 0)):
    d = res[name]
    k = 1 if solid else A['print_factor']
    return dict(
        m=d['mass_solid_g'] / 1000 * k,
        c=[(d['com_mm'][i] + offset[i]) / 1000 for i in range(3)],
        I=[[x * k for x in row] for row in d['I_kgm2_solid']],
    )

def comb(bs):
    M = sum(b['m'] for b in bs)
    C = [sum(b['m'] * b['c'][i] for b in bs) / M for i in range(3)]
    I = [[0] * 3 for _ in range(3)]
    for b in bs:
        d = [b['c'][i] - C[i] for i in range(3)]
        dd = sum(x * x for x in d)
        for i in range(3):
            for j in range(3):
                I[i][j] += b['I'][i][j] + b['m'] * ((dd if i == j else 0) - d[i] * d[j])
    return dict(m=M, c=C, I=I)

bodies = []
# printed structure (50% fill estimate)
for n in ['hip_plate', 'spine_frame', 'shoulder_bracket_L', 'shoulder_bracket_R',
          'battery_tray', 'servo_bay_L', 'servo_bay_R', 'bowden_exit_L', 'bowden_exit_R',
          'electronics_bay', 'cover_front', 'cover_back']:
    bodies.append(part_body(n))
# 10 pulleys + 10 tensioner arms + 10 tensioner pulleys
for b in all_banks:
    bodies.append(part_body('pulley', offset=tuple(b['pulley_c'])))
    bodies.append(part_body('tensioner_arm', offset=tuple(b['tens_c'])))
    bodies.append(part_body('tensioner_pulley', offset=(b['tens_c'][0], b['tens_c'][1] + 10, b['tens_c'][2])))
# head
bodies.append(part_body('head_placeholder'))

# purchased / placeholder masses
bodies.append(box_body(A['m_batt'], [A['batt_x'], 0, A['batt_z'] + A['batt_h'] / 2 + 2], [A['batt_w'], A['batt_l'], A['batt_h']]))
for b in all_banks:
    bodies.append(box_body(A['m_tservo'], b['servo_c'], [A['tservo'][1], A['tservo'][0], A['tservo'][2]]))
    bodies.append(box_body(A['m_tens'], b['tens_c'], [20, 15, 20]))  # spring+screw hardware
# electronics
rpi_c = [-A['chest_d'] / 2 + 30, 0, bay_z]
bodies.append(box_body(A['m_rpi'], rpi_c, list(A['rpi'])))
bodies.append(box_body(A['m_driver'], [rpi_c[0], 30, bay_z - 25], list(A['driver'])))
bodies.append(box_body(A['m_driver'], [rpi_c[0], -30, bay_z - 25], list(A['driver'])))
bodies.append(box_body(A['m_imu'], [0, 0, imu_z + 3], list(A['imu'])))
# Bowden misc per side
bodies.append(box_body(A['m_tendon_misc'], [0, 40, 180 + A['DZ']], [40, 40, 80]))
bodies.append(box_body(A['m_tendon_misc'], [0, -40, 180 + A['DZ']], [40, 40, 80]))
# head electronics placeholder already in print; add camera/board mass
bodies.append(box_body(A['m_head'], [0, 0, sh + A['neck_h'] + A['head_r']], [60, 60, 60]))

torso_only = comb(bodies)

# Arms: reuse arm-v5 total minus finger servos (now in torso). Each arm includes its AK45-10.
arm_g = A['arm_total_g'] - A['arm_finger_servo_g']  # ~650.6 g
# Place arm CoM roughly: shoulder at (0, ±shoulder_y, shoulder_z); arm hangs with CAD -X → -Z world
# Approximate arm CoM in torso frame: halfway down the reach, slightly forward
arm_com_z = A['shoulder_z'] - 140  # rough
arm_com_x = 20
bodies_arms = [
    box_body(arm_g / 1000, [arm_com_x, A['shoulder_y'], arm_com_z], [80, 40, 280]),
    box_body(arm_g / 1000, [arm_com_x, -A['shoulder_y'], arm_com_z], [80, 40, 280]),
]
upper = comb(bodies + bodies_arms)

# layout.json for viewer
layout = dict(
    coords="origin=hip_plate_top_center; +Z=up +Y=left +X=forward",
    hip_plate=dict(w=hw, d=hd, t=ht),
    shoulder=dict(y=A['shoulder_y'], z=A['shoulder_z']),
    battery=dict(
        type="6S 5000mAh Li-ion soft pack PLACEHOLDER",
        dims_mm=[A['batt_w'], A['batt_l'], A['batt_h']],
        mass_g=A['m_batt'] * 1000,
        tray_z=A['batt_z'],
        com_mm=[A['batt_x'], 0, A['batt_z'] + A['batt_h'] / 2 + 2],
    ),
    finger_servos=all_banks,
    electronics=dict(rpi_c=rpi_c, imu_c=[0, 0, imu_z + 3], drivers=[
        [rpi_c[0], 30, bay_z - 25], [rpi_c[0], -30, bay_z - 25]
    ]),
    bowden_exits=dict(L=[5, A['shoulder_y'] - 22, A['shoulder_z'] - 8], R=[5, -(A['shoulder_y'] - 22), A['shoulder_z'] - 8]),
    head=dict(c=[0, 0, sh + A['neck_h'] + A['head_r'] * 0.85], r=A['head_r']),
    com_torso_mm=[x * 1000 for x in torso_only['c']],
    com_upper_mm=[x * 1000 for x in upper['c']],
    mass_torso_g=torso_only['m'] * 1000,
    mass_upper_g=upper['m'] * 1000,
    mass_one_arm_without_finger_servos_g=arm_g,
    com_height_above_hip_mm=upper['c'][2] * 1000,
)

summary = dict(
    version="torso-v1.3-low (for pelvis-v1.1)",
    robot_height_target_cm=70,
    reference="HighTorque Mini Pi Plus ~68 cm / 15.4 kg / 22 DOF (inspiration only)",
    total_upper_body_g=round(upper['m'] * 1000, 1),
    torso_only_g=round(torso_only['m'] * 1000, 1),
    two_arms_without_finger_servos_g=round(2 * arm_g, 1),
    battery_g=A['m_batt'] * 1000,
    battery_note="6S 5000mAh Li-ion PLACEHOLDER ~155×50×45 mm, 650 g",
    finger_servos_g=10 * A['m_tservo'] * 1000,
    tensioners_hardware_g=10 * A['m_tens'] * 1000,
    com_upper_mm=[round(x * 1000, 1) for x in upper['c']],
    com_height_above_hip_mm=round(upper['c'][2] * 1000, 1),
    print_factor=A['print_factor'],
    rho_pla=A['rho'],
    assumptions=[
        "PLA density 1.24 g/cm³; printed parts mass ≈ 50% of solid (infill estimate)",
        "Battery: 6S 5000mAh Li-ion soft pack PLACEHOLDER 650 g — pick real pack later",
        "Micro servo 11 g PLACEHOLDER dims 12.2×23×29 mm (same as arm-v5)",
        "Tensioner hardware (spring+screw+axle) 8 g each PLACEHOLDER",
        "AK45-10 260 g included inside each arm-v5 mass",
        "Arm mass = arm-v5 705.6 g − 5×11 g finger servos (moved to torso) = 650.6 g each",
        "Arm CoM approximated as box at shoulder±Y, Z=shoulder−140 — not from full FK",
        "Head 80 g electronics PLACEHOLDER + printed shell",
        "Hip motors NOT included (legs future)",
        "No shape-memory alloy; Bowden PTFE+PE/stainless push-pull loops",
    ],
)

json.dump(dict(params=P, layout=layout, parts=res, summary=summary), open("results.json", "w"), indent=1)
json.dump(layout, open("layout.json", "w"), indent=1)
print("---")
print("torso_only_g", summary['torso_only_g'])
print("upper_g", summary['total_upper_body_g'])
print("CoM height above hip mm", summary['com_height_above_hip_mm'])
print("CoM", summary['com_upper_mm'])
