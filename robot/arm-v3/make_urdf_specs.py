import json
R=json.load(open("results.json")); P=R['params']; F=R['frames']; Pt=R['parts']; L=R['layout']; pf=P['print_factor']
def body(m,com,I=None):  # kg, m, kg m^2 (full tensor)
    return dict(m=m,c=com,I=I or [[0]*3 for _ in range(3)])
def part(n,offset=(0,0,0),solid=False):
    d=Pt[n]; k=1 if solid else pf
    return body(d['mass_solid_g']/1000*k,[(d['com_mm'][i]+offset[i])/1000 for i in range(3)],[[x*k for x in r] for r in d['I_kgm2_solid']])
def box(m,c_mm,dims_mm):
    a,b,cc=[x/1000 for x in dims_mm]
    return body(m,[x/1000 for x in c_mm],[[m*(b*b+cc*cc)/12,0,0],[0,m*(a*a+cc*cc)/12,0],[0,0,m*(a*a+b*b)/12]])
def comb(bs):
    M=sum(b['m'] for b in bs); C=[sum(b['m']*b['c'][i] for b in bs)/M for i in range(3)]
    I=[[0]*3 for _ in range(3)]
    for b in bs:
        d=[b['c'][i]-C[i] for i in range(3)]; dd=sum(x*x for x in d)
        for i in range(3):
            for j in range(3): I[i][j]+=b['I'][i][j]+b['m']*((dd if i==j else 0)-d[i]*d[j])
    return dict(m=M,c=C,I=I)
S=P['servo']; G=P['grip_servo']
links={
 'shoulder_base':comb([part('shoulder_cup'),box(P['m_act'],[0,0,-22.5],[53,53,45])]),
 'upper_arm':comb([part('upper_arm'),box(P['m_sts3215'],F['elbow_servo_c'],S)]),
 'forearm':comb([part('forearm_proximal'),box(P['m_sts3215'],F['roll_servo_c'],[35,S[0],S[1]]),
                  body(P['m_bearing'],[x/1000 for x in F['bearing_c']])]),
 'forearm_distal':comb([part('forearm_distal'),box(P['m_sts3215'],F['wrist_servo_c'],S)]),
 'hand':comb([part('palm'),box(P['m_grip_servo'],F['grip_servo_c'],G)]),
}
# simplified single "fingers" link (open pose), frame at middle-finger knuckle
fo=L['fingers']['middle']['pos']; fb=[]
import math
for nm,d in list(L['fingers'].items())+[('thumb',L['thumb'])]:
    x=0; yaw=math.radians(d['yaw'])
    for i,l in enumerate(d['segs']):
        b=part(f'finger_{nm}_{i+1}')
        lx,ly,lz=b['c'][0]+x/1000,b['c'][1],b['c'][2]
        b['c']=[d['pos'][0]/1000-fo[0]/1000+lx*math.cos(yaw)-ly*math.sin(yaw), d['pos'][1]/1000-fo[1]/1000+lx*math.sin(yaw)+ly*math.cos(yaw), d['pos'][2]/1000-fo[2]/1000+lz]
        fb.append(b); x-=l
links['fingers_simplified']=comb(fb)
f=lambda v:" ".join(f"{x:.5g}" for x in v)
def inert(b):
    I=b['I']; return f'''  <inertial><origin xyz="{f(b['c'])}"/><mass value="{b['m']:.5f}"/>
   <inertia ixx="{I[0][0]:.4e}" ixy="{I[0][1]:.4e}" ixz="{I[0][2]:.4e}" iyy="{I[1][1]:.4e}" iyz="{I[1][2]:.4e}" izz="{I[2][2]:.4e}"/></inertial>'''
def mesh(n,xyz="0 0 0",rpy="0 0 0"): return f'  <visual><origin xyz="{xyz}" rpy="{rpy}"/><geometry><mesh filename="{n}.stl" scale="0.001 0.001 0.001"/></geometry></visual>'
mm=lambda v:f([x/1000 for x in v])
fv=[]
for nm,d in list(L['fingers'].items())+[('thumb',L['thumb'])]:
    x=0; yaw=math.radians(d['yaw'])
    for i,l in enumerate(d['segs']):
        p=[d['pos'][0]-fo[0]+x*math.cos(yaw), d['pos'][1]-fo[1]+x*math.sin(yaw), d['pos'][2]-fo[2]]
        fv.append(mesh(f'finger_{nm}_{i+1}',mm(p),f"0 0 {yaw:.4f}")); x-=l
urdf=f'''<?xml version="1.0"?>
<!-- COCOMI / ここちゃん arm v3 (auto-generated). PLA mass = ~{int(pf*100)}% printed estimate; servos/actuator added as boxes.
     STS3215 {P['m_sts3215']*1000:.0f} g x3, grip, 6806 bearing {P['m_bearing']*1000:.0f} g servo {P['m_grip_servo']*1000:.1f} g, AK45-10 placeholder {P['m_act']*1000:.0f} g. Units m/kg/rad.
     finger_grip is SIMPLIFIED: one link holds all finger segments (open pose); real fingers are tendon-driven pin joints. -->
<robot name="cocomi_arm_v3">
 <link name="shoulder_base">
{inert(links['shoulder_base'])}
{mesh('shoulder_cup')}
{mesh('actuator_AK45-10_placeholder')}
 </link>
 <link name="upper_arm">
{inert(links['upper_arm'])}
{mesh('upper_arm')}
  <collision><geometry><mesh filename="upper_arm.stl" scale="0.001 0.001 0.001"/></geometry></collision>
 </link>
 <link name="forearm">
{inert(links['forearm'])}
{mesh('forearm_proximal')}
  <collision><geometry><mesh filename="forearm_proximal.stl" scale="0.001 0.001 0.001"/></geometry></collision>
 </link>
 <link name="forearm_distal">
{inert(links['forearm_distal'])}
{mesh('forearm_distal')}
  <collision><geometry><mesh filename="forearm_distal.stl" scale="0.001 0.001 0.001"/></geometry></collision>
 </link>
 <link name="hand">
{inert(links['hand'])}
{mesh('palm')}
  <collision><geometry><mesh filename="palm.stl" scale="0.001 0.001 0.001"/></geometry></collision>
 </link>
 <link name="fingers_simplified">
{inert(links['fingers_simplified'])}
{chr(10).join(fv)}
 </link>
 <joint name="shoulder_pitch" type="revolute"><parent link="shoulder_base"/><child link="upper_arm"/>
  <origin xyz="0 0 0"/><axis xyz="0 0 1"/><limit lower="-2.8" upper="0.8" effort="3" velocity="5"/></joint>
 <joint name="elbow" type="revolute"><parent link="upper_arm"/><child link="forearm"/>
  <origin xyz="{mm(F['elbow'])}"/><axis xyz="0 0 1"/><limit lower="-2.4" upper="0" effort="1.5" velocity="4"/></joint>
 <joint name="forearm_roll" type="revolute"><parent link="forearm"/><child link="forearm_distal"/>
  <origin xyz="{mm(F['roll'])}"/><axis xyz="1 0 0"/><limit lower="-1.5708" upper="1.5708" effort="1.5" velocity="4"/></joint>
 <joint name="wrist" type="revolute"><parent link="forearm_distal"/><child link="hand"/>
  <origin xyz="{mm(F['wrist'])}"/><axis xyz="0 0 1"/><limit lower="-1.2" upper="1.2" effort="1.5" velocity="4"/></joint>
 <joint name="finger_grip" type="revolute"><parent link="hand"/><child link="fingers_simplified"/>
  <origin xyz="{mm(fo)}"/><axis xyz="0 1 0"/><limit lower="0" upper="1.4" effort="0.2" velocity="3"/></joint>
</robot>
'''
open("cocomi_arm_v3.urdf","w").write(urdf)
JA={'actuator_AK45-10_placeholder':'モーター（ダミー形）','shoulder_cup':'肩カップ','upper_arm':'上うで','forearm_proximal':'前うで・根元（ひじ〜ひねりサーボ）','forearm_distal':'前うで・先（ひねり〜手首）','palm':'手のひら'}
rows=[];tot_s=tot_p=0
for n,d in Pt.items():
    if n.startswith('finger'): continue
    rows.append(f"| {JA[n]} | {d['mass_solid_g']:.0f} g | 約{d['mass_print_g']:.0f} g | ({d['com_mm'][0]:.0f}, {d['com_mm'][1]:.0f}, {d['com_mm'][2]:.0f}) |")
    if n!='actuator_AK45-10_placeholder': tot_s+=d['mass_solid_g']; tot_p+=d['mass_print_g']
fs=sum(d['mass_solid_g'] for n,d in Pt.items() if n.startswith('finger'))
rows.append(f"| 指（14こ合計） | {fs:.0f} g | 約{fs*pf:.0f} g | それぞれの部品で計算 |"); tot_s+=fs; tot_p+=fs*pf
servo=P['m_act']*1000+3*P['m_sts3215']*1000+P['m_bearing']*1000+P['m_grip_servo']*1000
tl=sum(b['m'] for k,b in links.items())*1000
fl=lambda n:"・".join(str(x) for x in n)
md=f'''# ここちゃんのうで v3 しよう（肩から手まで＋ひねり）

v2に、**前うでの「ひねり」関節（forearm_roll）** をたしました。前うでを「根元」と「先」の2つに分けて、ひじのすぐ先にSTS3215を**前うでの向きに寝かせて**入れ、先っぽ（前うで・先＋手首＋手）がくるっと±90°回ります。回るところには薄いベアリング（6806：内径30・外径42・厚さ7mm、**仮**）の受け座があります。ぜんぶ数字（パラメータ）で作っていて、`build.py` の上の数字を変えると形がかわります。PLAで3Dプリントできる形です。

## 関節（かんせつ）
| なまえ | 場所 | うごかすもの | 角度のはんい |
|---|---|---|---|
| shoulder_pitch（肩） | 肩 | AK45-10（ダミー） | −160°〜+46° |
| elbow（ひじ） | 上うでの先 | Feetech STS3215 | 0°〜137° 曲げる |
| forearm_roll（ひねり） | ひじのすぐ先 | Feetech STS3215（前うでの向き） | ±90° |
| wrist（手首） | 前うでの先 | Feetech STS3215 | ±69° |
| finger_grip（ぐーぱー） | 手のひら | 小さいサーボ（SG90/MG90Sくらい）＋糸 | 0°〜80°（**かんたん版**） |

## パーツ
1. **肩カップ＋モーターのダミー** … v1と同じ。AK45-10のネジ穴は**仮の値**（データシートで確認！）。
2. **上うで** … 長さ{P['upper_len']}mm（肩の軸→ひじの軸）。先にSTS3215が入るポケット（サーボより**片側0.25mm**大きい）。
3. **前うで・根元** … 根元の丸い板をひじサーボのホーンにネジどめ。ひねり用STS3215を寝かせて入れる「ゆりかご」（上があいている）と、前にベアリングの受け座（直径{P['brg_od']}+{P['brg_fit']}mm・深さ{P['brg_w']}mm）がある板。
3b. **前うで・先** … ひねりサーボのホーンにネジどめする丸い板＋ベアリングの内側に入る筒（外径{P['brg_id']-0.1}mm）。長さ{P['fore_dist_len']}mm（ひねりのホーン面→手首の軸）。先に手首用STS3215のポケット。
   ひじの軸→手首の軸は約{-F['roll'][0]+P['fore_dist_len']:.0f}mm（v2は110mm）。
4. **手のひら** … {P['palm_len']}×{P['palm_w']}×{P['palm_t']}mm、角はまるく。うら側に「ぐーぱー」用の小さいサーボのポケット、指の根元まで糸（テンドン）を通す穴。
5. **指** … 4本は3つの節（ふし）、親指は2つの節。節と節はピン（直径2.2mmの穴＝1.75mmフィラメントを差しこむ）でつなぐ。各節に糸の通る穴が2本（にぎる用・ひらく用）。
   InMoov（インムーブ）みたいに、サーボが糸を引くと指が曲がります。
   ⚠️ **指はかんたん版**です。本物は糸の通し方・ばね・指ごとの調整が必要。

指の長さ（ふしごと, mm）：人さし指 {fl(L['fingers']['index']['segs'])} / 中指 {fl(L['fingers']['middle']['segs'])} / くすり指 {fl(L['fingers']['ring']['segs'])} / 小指 {fl(L['fingers']['pinky']['segs'])} / 親指 {fl(L['thumb']['segs'])}

## 重さと重心
PLA（1.24 g/cm³）。「中身100%」と、ふつうの3Dプリントの**目安（約50%）**。重心はそのパーツ自身の座標（mm）。

| パーツ | 中身100% | 目安(50%) | 重心 X,Y,Z |
|---|---|---|---|
{chr(10).join(rows)}
| **プラ部品合計**（モーターダミー形はのぞく） | **{tot_s:.0f} g** | **約{tot_p:.0f} g** | |

モーター・サーボ：AK45-10（仮）{P['m_act']*1000:.0f} g ＋ STS3215 {P['m_sts3215']*1000:.0f} g ×3 ＋ ベアリング6806（仮）{P['m_bearing']*1000:.0f} g ＋ ぐーぱーサーボ {P['m_grip_servo']*1000:.1f} g ＝ {servo:.0f} g
→ **うで1本ぜんぶ（目安）：約{tl:.0f} g**（URDFの合計。プラは50%、ネジ・糸・ケーブルはふくまない）

## 座標のきまり
それぞれの部品で「関節の軸＝Z、うでは −X 方向へのびる」。サーボは立てて入れて、上にホーン → 次の部品をのせる「つみかさね」方式。
ひじの軸：上うで座標で ({F['elbow'][0]}, 0, {F['elbow'][2]}) mm、ひねりの軸（X向き）：前うで座標で ({F['roll'][0]}, 0, {F['roll'][2]}) mm、手首の軸：前うで・先の座標で ({F['wrist'][0]}, 0, {F['wrist'][2]}) mm。

## わかっていない所（仮の値）
- AK45-10の寸法・ネジ位置・重さ、STS3215の軸の位置（端から{P['servo_shaft_from_end']}mmと仮定）とホーンのネジ穴。
- ひじ・手首はホーン側だけでささえる形（反対側のベアリング支えは次の版で）。
- ひねりサーボは軸が片寄っている（サーボの端から{P['servo_shaft_from_end']}mm）ので、前うで・根元は少し横（+Y）にふくらんでいます。
- ベアリング6806は仮。内側の筒とホーンだけで止めているので、抜け止め（スナップリングやナット）とケーブルの通し方は次の版で。
- ひねり±90°のとき、サーボのケーブルがねじれるので余裕を持たせてね。
- 手のひらのサーボポケットのふた、糸を巻くプーリーは未作成。

## ファイル
- `*.step`（CAD用）・`*.stl`（3Dプリント用、パーツごと）
- `cocomi_arm_v3.urdf`（シミュレーション用、関節5つ）
- `index.html`（スマホで見る3Dビューア）・`shots/`（スクリーンショット）

作：あきやまん × 葵🦋（AI）
'''
open("specs.md","w").write(md)
for k,b in links.items(): print(k,round(b['m']*1000,1),'g',[round(x*1000,1) for x in b['c']])
print('total',tl,'plastic',tot_s,tot_p)
