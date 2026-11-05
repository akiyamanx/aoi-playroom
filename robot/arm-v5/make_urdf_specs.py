import math
import json
R=json.load(open("results.json")); P=R['params']; F=R['frames']; Pt=R['parts']; L=R['layout']; pf=P['print_factor']
def body(m,com,I=None):
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
S=P['servo']; TS=P['tservo']
# wrist servo oriented with shaft along Y: body ~ (sx along X, sz along Y, sy along Z)
links={
 'shoulder_base':comb([part('shoulder_cup'),box(P['m_act'],[0,0,-22.5],[53,53,45])]),
 'upper_arm':comb([part('upper_arm'),box(P['m_sts3215'],F['elbow_servo_c'],S)]),
 'forearm':comb([part('forearm_proximal'),box(P['m_sts3215'],F['roll_servo_c'],[35,S[0],S[1]]),
                  body(P['m_bearing'],[x/1000 for x in F['bearing_c']])]),
 'forearm_distal':comb([part('forearm_distal'),
                        box(P['m_sts3215'],F['wrist_servo_c'],[S[0],S[2],S[1]]),  # X,Y,Z dims for side-mounted
                        body(P['m_wrist_pin'],[x/1000 for x in F['wrist_idler_c']])]
                       +[box(P['m_tservo'],c,TS) for c in F['tendon_servo_c']]
                       +[body(P['m_tendon_misc'],[x/1000 for x in F['tendon_comb']])]),
 'hand':comb([part('palm')]),
}
FL=list(L['fingers'].items())+[('thumb',L['thumb'])]
f=lambda v:" ".join(f"{x:.5g}" for x in v)
def inert(b):
    I=b['I']; return f'''  <inertial><origin xyz="{f(b['c'])}"/><mass value="{b['m']:.5f}"/>
   <inertia ixx="{I[0][0]:.4e}" ixy="{I[0][1]:.4e}" ixz="{I[0][2]:.4e}" iyy="{I[1][1]:.4e}" iyz="{I[1][2]:.4e}" izz="{I[2][2]:.4e}"/></inertial>'''
def mesh(n,xyz="0 0 0",rpy="0 0 0"): return f'  <visual><origin xyz="{xyz}" rpy="{rpy}"/><geometry><mesh filename="{n}.stl" scale="0.001 0.001 0.001"/></geometry></visual>'
mm=lambda v:f([x/1000 for x in v])
JN={'thumb':'親指','index':'人差し指','middle':'中指','ring':'薬指','pinky':'小指'}
KM=[1,1.15,0.9]
fx=[]
for nm,d in FL:
    up=1.0 if nm=='thumb' else 1.4; yaw=math.radians(d['yaw']); par='hand'
    for i,l in enumerate(d['segs']):
        ln=f'finger_{nm}_{i+1}'; b=part(ln)
        fx.append(f' <link name="{ln}">\n{inert(b)}\n{mesh(ln)}\n </link>')
        org=f'xyz="{mm(d["pos"])}" rpy="0 0 {yaw:.4f}"' if i==0 else f'xyz="{-d["segs"][i-1]/1000:.5g} 0 0"'
        jn=f'{nm}_curl' if i==0 else f'{nm}_j{i+1}'
        mim='' if i==0 else f'<mimic joint="{nm}_curl" multiplier="{KM[i]}" offset="0"/>'
        fx.append(f' <joint name="{jn}" type="revolute"><parent link="{par}"/><child link="{ln}"/>\n  <origin {org}/><axis xyz="0 -1 0"/><limit lower="0" upper="{up*KM[i]:.3f}" effort="0.2" velocity="3"/>{mim}</joint>')
        par=ln
urdf=f'''<?xml version="1.0"?>
<!-- COCOMI / ここちゃん arm v5 (auto-generated). PLA mass = ~{int(pf*100)}% printed estimate; servos/actuator added as boxes.
     STS3215 {P['m_sts3215']*1000:.0f} g x3, 6806 bearing {P['m_bearing']*1000:.0f} g, wrist idler pin ~{P['m_wrist_pin']*1000:.0f} g,
     5 tendon micro servos {P['m_tservo']*1000:.0f} g each (PLACEHOLDER) in forearm_distal, AK45-10 placeholder {P['m_act']*1000:.0f} g. Units m/kg/rad.
     v5: wrist U-yoke — pivot on forearm centerline, axis Y (flexion/extension toward palm/back). Palm faces -Z (medial/inward at roll=0).
     Each finger has its own actuated joint <name>_curl; distal segments use <mimic>.
     Positive finger curl = toward the palm (axis 0 -1 0). Positive wrist = flexion toward palm (-Z); axis 0 -1 0 matches viewer wr.rotation.y=-J. -->
<robot name="cocomi_arm_v5">
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
 <joint name="shoulder_pitch" type="revolute"><parent link="shoulder_base"/><child link="upper_arm"/>
  <origin xyz="0 0 0"/><axis xyz="0 0 1"/><limit lower="-2.8" upper="0.8" effort="3" velocity="5"/></joint>
 <joint name="elbow" type="revolute"><parent link="upper_arm"/><child link="forearm"/>
  <origin xyz="{mm(F['elbow'])}"/><axis xyz="0 0 1"/><limit lower="-2.4" upper="0" effort="1.5" velocity="4"/></joint>
 <joint name="forearm_roll" type="revolute"><parent link="forearm"/><child link="forearm_distal"/>
  <origin xyz="{mm(F['roll'])}"/><axis xyz="1 0 0"/><limit lower="-1.5708" upper="1.5708" effort="1.5" velocity="4"/></joint>
 <joint name="wrist" type="revolute"><parent link="forearm_distal"/><child link="hand"/>
  <origin xyz="{mm(F['wrist'])}"/><axis xyz="0 -1 0"/><limit lower="-1.2" upper="1.2" effort="1.5" velocity="4"/></joint>
{chr(10).join(fx)}
</robot>
'''
open("cocomi_arm_v5.urdf","w").write(urdf)
JA={'actuator_AK45-10_placeholder':'モーター（ダミー形）','shoulder_cup':'肩カップ','upper_arm':'上うで','forearm_proximal':'前うで・根元（ひじ〜ひねりサーボ）','forearm_distal':'前うで・先（ひねり〜手首ヨーク）','palm':'手のひら'}
rows=[];tot_s=tot_p=0
for n,d in Pt.items():
    if n.startswith('finger'): continue
    rows.append(f"| {JA[n]} | {d['mass_solid_g']:.0f} g | 約{d['mass_print_g']:.0f} g | ({d['com_mm'][0]:.0f}, {d['com_mm'][1]:.0f}, {d['com_mm'][2]:.0f}) |")
    if n!='actuator_AK45-10_placeholder': tot_s+=d['mass_solid_g']; tot_p+=d['mass_print_g']
fs=sum(d['mass_solid_g'] for n,d in Pt.items() if n.startswith('finger'))
rows.append(f"| 指（14こ合計） | {fs:.0f} g | 約{fs*pf:.0f} g | それぞれの部品で計算 |"); tot_s+=fs; tot_p+=fs*pf
servo=P['m_act']*1000+3*P['m_sts3215']*1000+P['m_bearing']*1000+P['m_wrist_pin']*1000+5*P['m_tservo']*1000+P['m_tendon_misc']*1000
tl=sum(b['m'] for k,b in links.items())*1000+fs*pf
fl=lambda n:"・".join(str(x) for x in n)
md=f'''# ここちゃんのうで v5 しよう（手がうでのまんなか！）

v4（指が1本ずつ動く）をもとに、**手首のつけ方をやりなおしました**。v4では手が手首サーボの**外側（うえ）**についていて、前うでの中心からずれていました。v5は**U字のヨーク（クレビス）**で、手首の回転軸が**前うでの中心線を通る**ようにして、手が人の手みたいにうでの延長線上に来るようにしました。
手のひらは、ひねり0°のとき**体の内側（正中線のほう）を向く**向きです。両手を向かい合わせて、ものをはさめるイメージです。

## v4からのいちばんのちがい → 手首がうでのまんなか
- **U字ヨーク**：前うで・先の先端がU字。片側（+Y）に手首用STS3215、反対側（−Y）にアイドラのピン／ブッシュ。回転軸＝**Y軸**で、前うでの長軸（X）の中心を通ります。
- **手のひらの中心線**が前うでの長軸の上にのる（手の厚みのまんなか ＝ Z=0）。
- 手首を曲げると、**手のひら側／手の甲側**に曲がる（にんげんの手首の屈曲・伸展）。v4の「Z軸まわり」から変更。
- 糸（テンドン）のPTFEチューブは、ヨークの根元ちかく（軸のそば）を通す穴があるので、手首を曲げても糸の長さがあまり変わりません。
- 手のひらは厚み方向のまんなかが原点。肉ぬきは手の甲（+Z）、にぎる糸は手のひら側（−Z）。かかとにヨーク用のラグ（耳）があります。

## 指のサーボはどこ？ → 「前うで・先」（ひねりの先、手首の手前）※v4と同じ
- 5こ（たて{P['tservo'][2]}×よこ{P['tservo'][1]}×あつさ{P['tservo'][0]}mm、1こ約{P['m_tservo']*1000:.0f} g の**マイクロサーボ、仮の寸法**）を、前うでの向きに**1列にならべて立てて**入れます。
- **ひねり関節より先**なので、ひねっても糸はねじれません。
- 糸の通り道：サーボのスプール → くし → **PTFEチューブ**で手首ヨークをまたぐ（軸のそば） → 手のかかと → 各指。

## 関節（かんせつ）
| なまえ | 場所 | うごかすもの | 角度のはんい |
|---|---|---|---|
| shoulder_pitch（肩） | 肩 | AK45-10（ダミー） | −160°〜+46° |
| elbow（ひじ） | 上うでの先 | Feetech STS3215 | 0°〜137° 曲げる |
| forearm_roll（ひねり） | ひじのすぐ先 | Feetech STS3215（前うでの向き） | ±90° |
| wrist（手首・ピッチ） | 前うでの先（ヨーク） | Feetech STS3215 | ±69°（手のひら／甲へ） |
| thumb_curl（親指） | 手のひら | マイクロサーボ①＋糸 | 0°〜57° |
| index_curl（人差し指） | 手のひら | マイクロサーボ②＋糸 | 0°〜80° |
| middle_curl（中指） | 手のひら | マイクロサーボ③＋糸 | 0°〜80° |
| ring_curl（薬指） | 手のひら | マイクロサーボ④＋糸 | 0°〜80° |
| pinky_curl（小指） | 手のひら | マイクロサーボ⑤＋糸 | 0°〜80° |

URDFでは、指の2つめ・3つめの節は `mimic`（1つめ×1.15、×0.9）。プラスの指角度＝**手のひら側に曲がる**。手首のプラス＝手のひら側へ屈曲。

## パーツ
1. **肩カップ＋モーターのダミー** … v1と同じ。AK45-10のネジ穴は**仮の値**。
2. **上うで** … 長さ{P['upper_len']}mm。先にSTS3215のポケット。
3. **前うで・根元** … v3と同じ（ひじ〜ひねりサーボ＋ベアリング6806の受け座）。
3b. **前うで・先** … ひねりのホーン → 指サーボ5この箱 → くし → **U字ヨーク**（サーボ片側＋アイドラ反対側）。チューブ用の穴5つ。
4. **手のひら** … {P['palm_len']}×{P['palm_w']}×{P['palm_t']}mm。厚みのまんなかが原点。かかとにヨーク用ラグ。手の甲側に肉ぬき。
5. **指** … v4と同じ。4本は3節、親指は2節。

指の長さ（ふしごと, mm）：人さし指 {fl(L['fingers']['index']['segs'])} / 中指 {fl(L['fingers']['middle']['segs'])} / くすり指 {fl(L['fingers']['ring']['segs'])} / 小指 {fl(L['fingers']['pinky']['segs'])} / 親指 {fl(L['thumb']['segs'])}

## 重さと重心
PLA（1.24 g/cm³）。「中身100%」と、3Dプリントの**目安（約50%）**。

| パーツ | 中身100% | 目安(50%) | 重心 X,Y,Z |
|---|---|---|---|
{chr(10).join(rows)}
| **プラ部品合計**（モーターダミー形はのぞく） | **{tot_s:.0f} g** | **約{tot_p:.0f} g** | |

モーター・サーボ：AK45-10（仮）{P['m_act']*1000:.0f} g ＋ STS3215 {P['m_sts3215']*1000:.0f} g ×3 ＋ ベアリング6806（仮）{P['m_bearing']*1000:.0f} g ＋ 手首ピン（仮）{P['m_wrist_pin']*1000:.0f} g ＋ 指サーボ {P['m_tservo']*1000:.0f} g ×5（仮）＋ 糸・チューブ・くし {P['m_tendon_misc']*1000:.0f} g ＝ {servo:.0f} g
→ **うで1本ぜんぶ（目安）：約{tl:.0f} g**（v4は約690 g。プラは50%、ネジ・ケーブルはふくまない）

## 座標のきまり
それぞれの部品で「うでは −X 方向へのびる」。手首の軸＝**Y**（屈曲／伸展）。手のひら側＝−Z（内側／正中線向き、ひねり0°）、指はそちらへ曲がる。肉ぬきは手の甲（+Z）。
ひじの軸：上うで座標で ({F['elbow'][0]}, 0, {F['elbow'][2]}) mm、ひねりの軸：前うで座標で ({F['roll'][0]}, 0, {F['roll'][2]}) mm、手首の軸：前うで・先の座標で ({F['wrist'][0]}, {F['wrist'][1]}, {F['wrist'][2]}) mm（**中心線上**）。
指サーボの中心（前うで・先の座標, X）：{", ".join(f"{c[0]:.1f}" for c in F['tendon_servo_c'])} mm。

## わかっていない所・できないこと（仮の値）
- **指サーボの寸法・重さは仮**（23×12×29mm, 11 g）。買ったサーボに合わせて `build.py` の `tservo` を変えてね。糸まき（スプール）・ホーンの形は未作成。
- 手首ヨークのサーボ取り付け・ホーンとラグのすきま・ピンの材質は試作で要確認。ベアリングは簡易ブッシュ想定。
- 1本の糸で節を全部曲げるので、**節ごとの別々の動きはできない**。ひらく側はばね（ゴム）か2本目の糸が必要。
- 親指は「曲げる」だけ。**対向関節はなし**。指の横開きもなし。
- 手首を曲げると、チューブの中でも糸が少しずれて指が少し動く（ソフトで補正）。ヨーク近くを通しても完全には0にならない。
- AK45-10の寸法、STS3215の軸の位置（端から{P['servo_shaft_from_end']}mm と仮定）、ベアリング6806は仮。

## ファイル
- `*.step`・`*.stl`（パーツごと）、`cocomi_arm_v5.urdf`（関節：肩・ひじ・ひねり・手首ピッチ＋指5本）
- `index.html`（スマホで見る3Dビューア）・`shots/`（スクリーンショット）

作：あきやまん × 葵🦋（AI）
'''
open("specs.md","w").write(md)
for k,b in links.items(): print(k,round(b['m']*1000,1),'g',[round(x*1000,1) for x in b['c']])
print('total',tl,'plastic',tot_s,tot_p)
R['summary']=dict(version='v5',total_arm_g=round(tl,1),plastic_solid_g=round(tot_s,1),plastic_print_g=round(tot_p,1),actuators_g=round(servo,1),
  change='wrist U-yoke: hand centered on forearm axis, palm faces medial (-Z) at roll=0, wrist pitch about Y (flexion/extension)',
  palm_solid_g_v4=108,palm_solid_g_v5=round(Pt['palm']['mass_solid_g'],1),
  wrist_frame_mm=F['wrist'],yoke_gap_mm=F.get('yoke_gap'),
  tendon_servos='5x micro servo PLACEHOLDER 23x12x29mm 11g, in forearm_distal after roll joint',
  finger_joints=['thumb_curl','index_curl','middle_curl','ring_curl','pinky_curl'],mimic_multipliers=[1,1.15,0.9],
  link_masses_g={k:round(b['m']*1000,1) for k,b in links.items()})
json.dump(R,open("results.json","w"),indent=1)
