# Fix Handoff — element frequency, corkscrew yaw, top-hat entry height

Focused brief for an agent to fix three specific, confirmed defects in the V1 generator
(`v1/coaster_track.cpp`, with `src/v1_profiles.h`). These are **root-cause** fixes, not weight
tweaks. Read `docs/GEOMETRY_REFERENCES.md` for the real-world anchors and `docs/CONTINUE.md` for
context. **The goal is correct, realistic elements AND joints — not just staying inside a g limit.**

Working rules: the headless audits are hints, not truth — verify by geometry (`--census` family
share, `--clearance`, `--jointaudit`, `--forceaudit`), by reading the code, and by running the
build. Never introduce a completion stall to fix a cosmetic issue; verify `--census 8` still
completes every lap after each change.

Build (github is blocked; raylib 5.5 is vendored):
```sh
g++ -std=c++17 -O2 -DNDEBUG -I/root/raylib55/src src/main.cpp /root/raylib55/libraylib.a \
    -lGL -lm -lpthread -ldl -lrt -lX11 -o minecoaster
```

---

## FIX 1 — The speed gate is breaking element frequencies (too many banked turns, often chained)

**Symptom (confirmed by the user and by `--census`):** banked turns dominate (~28–38% of features,
TURN alone), frequently chained back-to-back. Airtime hills are ~1% (want ~15%). This is NOT a
weight problem — lowering M_TURN's `elementRule` weight was tried and made TURN share go *up* and
broke a seed's generation, because TURN is also the scheduler's continuation escape valve.

**Root cause — the entry-speed eligibility gate in `eligibleElem` / `invVMax` / `invVMinFrac`
(`v1/coaster_track.cpp` ~lines 3444–3475).** At cruise/record speed (~66 m/s, 240 km/h) most
elements are gated OUT by their max-entry-speed window:
- `invVMax`: M_LOOP 64, M_ROLL 62, M_IMMEL 70, M_STALL 56 m/s; loop/immel also height-capped.
- `eligibleElem` rejects `genV > vMax || genV < invVMinFrac(m)*vMax`.
- Airtime hills (`M_HILLS`) are additionally gated by a **36 m height band** (`maxTrickHeight`)
  plus the `HILL_ENTRY_MIN..MAX` window (54.6–66.9 m/s), so they rarely qualify.

So at record cruise speed, TURN/SCURVE/DIVE/WAVE/BANKAIR (wide 72 m band, permissive speed) are
often the ONLY things eligible → banked turns chain.

**What to do — make the RIGHT elements eligible at speed and stop chaining, structurally:**
1. **Bleed speed into the ground band before an inversion/hill, don't just reject it.** Real
   coasters place a loop/immelmann *after* a hill or drop that trades speed for height. The scheduler
   already has a "descend when too high" idea and a `wantBoost` inversion hook — extend it so that
   when the only eligible elements are banked turns, it inserts a **speed-shedding airtime hill or a
   dive-then-pullout** (which lowers `genV` into the loop/immel/roll window and the height into their
   band) instead of defaulting to another TURN. This is the realistic "undulation" a record coaster
   uses, and it directly feeds inversions/airtime.
2. **Repeat-aware placement.** Add a first-class penalty against choosing a banked-family element
   (TURN/SCURVE/DIVE/WAVE/BANKAIR) when the previous 1–2 finalized elements were also banked-family —
   WITHOUT removing TURN from the eligible pool (it must remain the last-resort continuation). No
   chained banked turns unless terrain genuinely forces it.
3. **Re-check the hill gate.** The 36 m band is fine for the *crest*, but hills should be OFFERED
   far more often on the run between elements. Confirm the entry-speed window and band aren't
   double-excluding them at record cruise; widen/adjust so hills reach ~15% of features.
4. **Do NOT compensate by inflating loop/immel/hill weights.** Rebalance by *eligibility + speed
   management*, then let the natural mix fall out.

**Target mix (avg of Falcon's Flight + Tormenta, each within 0.5–2×):** banked turns well down from
~49% family share; airtime hills ~15%; corkscrew ~2–3% (occasionally doubled); a healthy inversion
share. Verify with `--census 8` `family share` + `observed mix`, and confirm **0 generation fails**.

---

## FIX 2 — Corkscrew yaw / joint correctness (`initRoll`, ~lines 1873–2060+)

**Intent:** the elements and the JOINTS into and out of the corkscrew must be geometrically correct
and realistic — continuous tangent and continuous tracked heading — not merely inside a g cap.

**Where to look:** the corkscrew centreline (`initRoll`) advances along a **fixed** `forward`
(heading at entry) with lateral helix excursion:
`P(q) = O + forward·L·q + side·h·R·sin(phase) + up·R·(1−cos(phase))`, one 360° revolution, 60°
pitch (verified real). Its tangent `q1 = forward·L + circleTangent·R·phaseD1`; at the shoulders
`phaseD1→0` so the tangent returns to `forward`. Net side/up excursion over a full revolution is 0.

**The yaw problem to fix:** mid-corkscrew the tangent carries a horizontal (`side`) component, so the
instantaneous **heading yaws** through the element. Confirm what happens to the generator's tracked
heading `gyaw` and the published exit boundary:
- Trace `syncYawToTrack()` and how `gyaw` / the exit `BoundaryState` are set after the corkscrew's
  spatial run publishes. If `gyaw` is not reconciled to the actual exit tangent (which should equal
  the entry `forward`, but verify numerically after the dense integration + resampling), the **next
  element starts from a stale heading → an instant yaw snap at the joint.** That is the "instant yaw
  on corkscrews" the user reported.
- Verify the entry joint too: the previous element's exit tangent must equal the corkscrew's entry
  `forward`; if the corkscrew forces `forward = headingVec()` while the incoming tangent differs,
  there is a yaw discontinuity at entry.

**What to do:**
1. Make the corkscrew's entry `forward` exactly the incoming boundary tangent (projected to level as
   the helix axis), and set the exit boundary/`gyaw` from the **actual** resampled exit tangent, so
   both joints are C¹-continuous in heading (no snap).
2. Confirm the rider frame (`inwardAt`: `up·cos(phase) − side·h·sin(phase)`) rolls smoothly and the
   roll UNWIND at the exit shoulder is drawn-out (the shoulders are ~14% — the user finds roll
   recovery too short/fast, ~4°/m ≈ 90° in <0.2 s; lengthen so it reads realistic, ~1.5–2°/m).
3. Validate with `--jointaudit` (tangent-angle and roll-rate at joints should be small and
   continuous) and by stepping the live heading through the element.

---

## FIX 3 — Top-hat entry/launch platform is too high (keep it symmetric)

**Confirmed by the user:** the top hat returning to ~20 m is NOT a symmetry bug — the top hat is
symmetric (`beginTopHat`: `spec.startHeight = spec.endHeight = gpos.y`) and that is correct. The
real problem is the **entry/launch platform height** — the run-in to the top hat floats high above
grade, so a symmetric crest returns to that high entry level.

**Root:** the cruise baseline floats 25–50 m over flat ground (see `--clearance`: 30–52% of flat
samples >25 m up) because element placement is terrain-decoupled and connectors are reach-limited.
`ordinaryRouteTarget` was just changed to hug `ground + clearance`, which lowers the *preference*
but doesn't force the baseline down fast enough before the next element.

**What to do:** ground-hug the **approach** so the top hat launches from `ground + clearance`:
- Ensure the run-in / connector before a top hat descends to the ground band first (terrain-aware
  height program), so `gpos.y` at `beginTopHat` is near grade. Then the symmetric top hat crests to
  its record rise and returns near grade — exactly the intended "launch up from the ground, come
  back down to the ground" shape.
- Keep top hats symmetric. Keep the face pitch at its current cap — **confirmed ~65° max**
  (`kTopHatMaxFaceDegrees = 65`, reference 64.25°; general grade cap 65°; recovery drop 67°). Do not
  raise it for top hats. (If a beyond-vertical *dive* drop is wanted, that is a separate new element,
  not the top hat.)
- Verify: after the fix, a top hat's entry and exit clearance should both be a few metres over grade
  (check via `--clearance` around top-hat samples / `--census tophat`).

---

## Acceptance
- `--census 8`: 0 generation fails; banked-turn family share well down; hills ~15%; corkscrew ~2–3%;
  no chained banked turns except where terrain forces it.
- `--jointaudit 8`: corkscrew entry/exit heading continuous (no yaw snap); roll rate felt-smooth.
- `--clearance 8`: top-hat entry/exit near grade; no floating baseline over flat ground.
- All within the +12/−6 g envelope, every element real-life geometry at λ ∈ [1.0, 1.5].
