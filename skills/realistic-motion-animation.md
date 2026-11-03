---
name: realistic-motion-animation
description: >-
  Use when building an animated 3D or 2D model of something that moves (animal,
  fish, human, robot, machine) and the motion should feel real and alive; covers
  observation, research, measurable checks, iteration with feedback, and an
  honest flaw list.
---
# Realistic motion animation

## Mindset
- Observe the real thing first; don't animate from imagination.
- Research how it actually moves (papers, biomechanics, slow-motion video descriptions). Write down 5-10 concrete facts (timing ratios, which part leads, what moves together vs independently).
- Measure instead of eyeballing: write small scripts that check rules on every frame.
- Iterate in small versions, keep backups (index_v1.html, v2...), and change one aspect per round based on the user's words.
- Always report remaining flaws honestly.

## Steps
1. Clarify the moment(s) to show (idle, reach, eat, turn, swim away...) and build a phase timeline.
2. Research the motion. Useful kinds of facts:
   - Who leads: e.g. octopus jet goes body-first with arms trailing; fish swim head-first with the wave travelling to the tail; humans swing arms opposite to legs.
   - Timing ratios: e.g. octopus arm swimming opens slowly and closes fast, with recovery about 2.5x the power stroke.
   - Waves and propagation: bends travel base to tip; velocity profiles are bell-shaped, not linear.
   - Independence: limbs are partly autonomous. Avoid perfect sync except where the real animal syncs (e.g. a power stroke).
   - Roles: some limbs explore, some support, some drift. Rigid or straight poses should be rare.
   - Secondary motion: breathing (around 3-4 s cycle), drag lag on turns, tips whipping, bubbles or dust.
3. Build deterministically: expose window.renderFrame(frame) driven by seeded noise, so video capture and checks are repeatable.
4. Add checks: count straight limbs per frame, check floor penetration, measure camera jumps, check the subject stays in frame.
5. Render a video (headless Chrome with swiftshader + Puppeteer frame by frame, then ffmpeg h264 yuv420p, served over http, not file://). Make contact sheets and stills of key phases and actually look at them.
6. Show the user the video and stills. Translate their feeling words ("like breaststroke", "stiff like a stick", "spider-like") into a physical cause, research it if needed, and fix it.
7. Common fixes:
   - Synced motion: add per-limb phase and noise.
   - Stiff limbs: add base-to-tip waves, curls and lag.
   - A rigid turn: drive it with limb strokes and drag lag.
   - The wrong swim style: follow the real stroke (who pushes the water and when).
8. When publishing, add a making-of page: each version, the feedback, the reasoning, the references, the remaining flaws, and the next improvements. Publish only after the user's go.
