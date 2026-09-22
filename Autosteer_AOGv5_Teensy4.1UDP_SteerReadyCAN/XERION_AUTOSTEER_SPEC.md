# Claas Xerion ↔ AgOpenGPS — CAN Integration Spec

Handoff document for Claude Code. Goal: add proper Claas Xerion support to the Teensy CANBUS steering firmware (`CAN_All_Brands.ino` sketch) so AgOpenGPS (AOG) can steer the Xerion over its steering bus, and then feed the Xerion's crab/rear-axle angle into AOG so the machine is positioned and drawn correctly when it travels sideways.

Everything below comes from five SavvyCAN captures of the Xerion steering bus (CAN3 / V-Bus, 250 kbit/s), one capture of the cab/joystick bus, the existing Teensy code, and field notes from the owner. Each claim is tagged:

- **[CONFIRMED]** — directly visible in the logs, checked across several files.
- **[LIKELY]** — strong statistical evidence, but needs a targeted capture to be sure.
- **[UNKNOWN]** — observed but not decoded. Don't build logic on it.

---

## 1. Source material

| File | Bus | Content |
|---|---|---|
| `starting_up_the_tractor.csv` | V-Bus | Power-up, factory autosteer engaged, reversing, turns |
| `CAN3_XERION.csv` (in xerion.zip) | V-Bus | ~10 min, factory autosteer, headland turns, ends at standstill at full lock |
| `xerion disengage turn right then engage.csv` | V-Bus | Operator override → manual right turn → re-engage |
| `xerioncan3.csv` | V-Bus | Manual left turns, then straight lines |
| `joystick can.csv` | K-Bus (cab) | Contains the engage button frame `0x10613173` |
| `CAN_All_Brands.ino` | — | Existing Teensy code; Xerion is folded into `Brand == 0` (Claas) |
| screenshot | V-Bus | `0x0CAC1CD2` = `99 7D 34…`, `0x0CADD21C` = `A2 7D 01…` |

**Capture caveat:** the `Time Stamp` column in all CSVs is broken (non-monotonic, wraps at 65 535 000 — looks like a 16-bit ms counter ×1000). Row order is still chronological. Absolute rates in this document were derived from the GNSS time/date PGN 65254 (`0x18FEE61D`, seconds field) over a 592 s span of `CAN3_XERION.csv`. Future captures must have real timestamps (see §9).

---

## 2. Node map (V-Bus, 29-bit, 250 kbit/s)

| SA | Role | Evidence |
|---|---|---|
| `0xD2` | **Xerion steering controller** (valve side). Receives curvature commands, reports status. | Sends PGN 0xAC00 to 0x1C, receives PGN 0xAD00 from 0x1C [CONFIRMED] |
| `0x1C` | **Factory navigation controller** (the guidance system that was installed during the captures). This is the address our firmware must take over. | Sends `0x0CADD21C` curvature commands at 10 Hz [CONFIRMED] |
| `0x5A` | Vehicle-dynamics / heading ECU. Publishes true path curvature, yaw rate, heading. | `0x0CFFA25A` etc. [CONFIRMED signals, role LIKELY] |
| `0x5D` | Sensor cluster, very likely an IMU (3 frames × 3 × 16-bit axes at 50 Hz). | `0x0CFF975D` bytes 0-1 track yaw rate (r = −0.96) [LIKELY] |
| `0x1D` | GNSS receiver, publishing NMEA 2000 + J1939 position PGNs. | 129025/129026/129029/65254/65256/65267 decode to sane values [CONFIRMED] |
| `0x05, 0x26, 0xD3, 0xDA, 0xDB` | Other ECUs, low-rate heartbeats | [UNKNOWN] |

No address-claim traffic (PGN 0xEE00) appears in any capture, including the power-up one. The factory 0x1C does not appear to claim. [CONFIRMED for these captures]

**Critical consequence:** The factory nav controller at 0x1C was live during all captures. Our firmware must **replace** it (physically disconnected or disabled), never run alongside it. Two nodes transmitting `0x0CADD21C` will fight, and this is a plausible cause of the "steering error, needs restart" state seen last year.

---

## 3. Message dictionary

Byte numbering is 0-based (b0…b7). "LE" = little-endian (b0 is the low byte), "BE" = big-endian.

Curvature convention used throughout (ISO 11783-7 and AgOpenGPS `estCurve`/`setCurve`): `raw = curvature_km⁻¹ / 0.25 + 32128`, so 32128 (`0x7D80`) = straight, 0.25 km⁻¹ per bit, positive = right turn.

### 3.1 `0x0CADD21C` — Guidance System Command (PGN 0xAD00), 0x1C → 0xD2, **10 Hz** — WE TRANSMIT THIS

| Bytes | Meaning | Tag |
|---|---|---|
| b0-b1 LE | Commanded curvature (32128 offset, 0.25 km⁻¹/bit) | [CONFIRMED] |
| b2 | `0x01` = intend to steer, `0x00` = not steering | [CONFIRMED] |
| b3-b7 | Always `0x00` from the factory controller | [CONFIRMED] |

When not steering, the factory controller sends exactly `80 7D 00 00 00 00 00 00`.

The existing Teensy code for other brands sends `0xFD`/`0xFC` in b2 and `0x00` or `0xFF` padding. **For the Xerion, copy the factory bytes exactly** (`0x01`/`0x00` and zero padding). Reserved-bit differences are a candidate cause of the controller faulting.

Observed commanded range: about ±600 counts (±150 km⁻¹).

### 3.2 `0x0CAC1CD2` — Guidance Machine Status (PGN 0xAC00), 0xD2 → 0x1C, **1 Hz**

This is the "1 Hz angle message" from the field notes, and the first row of the screenshot. Note the rate: ISO 11783-7 expects 10 Hz, but the Xerion sends it at only 1 Hz.

| Bytes | Meaning | Tag |
|---|---|---|
| b0-b1 LE | Estimated curvature (32128 offset). In front-steer driving it tracks `0x0CFFA25A` curvature closely. | [CONFIRMED] |
| b2 | Steering state bitfield (below) | [CONFIRMED values] |
| b3-b7 | `0x00` | [CONFIRMED] |

**b2 values** (decimal as in the owner's serial notes):

| b2 | Dec | When observed | Interpretation |
|---|---|---|---|
| `0x34` | 52 | Factory autosteer actively steering | **Steering active** |
| `0x74` | 116 | Engaged-capable but not steering (after override, before engage) | **Ready, not steering** |
| `0x70` | 112 | Power-up for about 1 s; transiently at operator override; latched at the end of `CAN3_XERION.csv` at standstill, full lock, wheel-speed direction = N/A | **Not ready.** Owner saw this as "steering error, needs restart" |
| `0x30` | 48 | Owner's serial notes only ("disengage?"), not in these logs | Probably a transient "active but lost ready" |

Bit reading that fits every observation [LIKELY]:
- `0x04` = system ready. Cleared in 0x70/0x30. It mirrors bit 0 of `0x18FFE1D2` b0 (below).
- `0x40` = not steering / reset required. Clear only in 0x34/0x30.
- `0x30` = always set. This fits an ISO 2-bit "not available" field.

Map it against the ISO 11783-7 byte-3 layout (mechanical lockout / steering input position / readiness / request-reset fields) when you have the standard at hand. Don't hard-code ISO meanings without checking. Implement the logic as:

```c
bool xerionReady    = (st & 0x04) != 0;
bool xerionSteering = (st & 0x40) == 0;
```

Because this frame is only 1 Hz, **don't use it as the primary disengage detector** (see 3.4 and 3.5).

### 3.3 `0x0CFFA25A` — Xerion true curvature / yaw rate / heading, 0x5A, **20 Hz**

This is the "10 Hz good for autosteer" message from the notes (actual rate is 20 Hz). The existing code already reads b0-b1 correctly as big-endian.

| Bytes | Meaning | Tag |
|---|---|---|
| b0-b1 **BE** | Actual path curvature (32128 offset, 0.25 km⁻¹/bit). Reads ~0 while crabbing straight (owner's observation). **Use as `estCurve`.** | [CONFIRMED] |
| b2-b3 **BE** | Yaw rate, offset 32768. ≈558 counts per °/s (≈ 1/32000 rad/s per bit). Correlation with GNSS SOG × curvature = 0.998 over 3 154 samples. | [LIKELY scale] |
| b4-b5 **BE** | Heading, 360/65536 ° per bit. Matches GNSS COG within a few degrees when driving straight. **Holds body orientation while reversing** (COG flips 180°, this doesn't). Starts at 0 after power-up until the vehicle moves. | [CONFIRMED behaviour; whether it's body or travel heading in crab mode is UNKNOWN] |
| b6-b7 | Varies, undecoded | [UNKNOWN] |

`0x18FFA15A` b3-b4 BE is a second heading-like value. It's offset from b4-b5 above by ~10° on one field direction and ~2° on the other, which looks like a separate, differently-corrected estimate. [UNKNOWN] Don't use it.

### 3.4 `0x18EF1CD2` — Proprietary A, 0xD2 → 0x1C, **10 Hz**

| Bytes | Meaning | Tag |
|---|---|---|
| b0 bit2 (`0x04`) | **Steering engaged.** Clears within about 25 frames of an operator steering-wheel override, well before the 1 Hz status frame updates. | [CONFIRMED] |
| b0 bit0 (`0x01`) | Toggles around engage/disengage and during standstill. | [UNKNOWN] |
| b0 bit1 | Seen only at power-up (`0x02`) | [UNKNOWN] |
| b1-b2 | `CF 7B` in normal running (other values at start-up) | [UNKNOWN] |

The existing `Brand == 0` handler for this ID tests `buf[1]/buf[2]` for Claas combine layouts (0/0, 39/241, 0/125). The Xerion's `CF 7B` matches none of them, so today that handler never fires on a Xerion.

### 3.5 `0x18FFE1D2` — Steering controller state, 0xD2, **20 Hz**

| Bytes | Meaning | Tag |
|---|---|---|
| b0 | State code: `0x42` steering; `0x00` override in progress; `0x05` standby with intent from nav controller; `0x07` standby with no intent; `0x06` not ready (precedes AC status going to 0x70); `0x03` arming (between intent and steering); `0x01` power-up | [CONFIRMED values, meanings LIKELY] |
| b0 bit0 | Ready. Mirrors `0x04` in the 0xAC00 status. | [LIKELY] |
| b4-b5 **LE** | Curvature, 32128 offset (r = 0.95–0.99 with 0x5A curvature) | [LIKELY] |

Observed engage sequence (in `xerion disengage…csv`): nav controller sets AD b2 = 0x01 → E1 b0 `07→05→03→42` → EF b0 bit2 set → AC b2 `0x74→0x34` at the next 1 Hz frame.

Observed override sequence: E1 b0 `42→00` → EF b0 `05→01` → E1 `→06/07/05` → AC `0x34→0x74` (or a transient 0x70) → factory nav controller drops intent.

### 3.6 `0x0CFE48D2` — Wheel-Based Speed and Distance (PGN 65096), 0xD2, **10 Hz**

| Bytes | Meaning | Tag |
|---|---|---|
| b0-b1 LE | Speed, 0.001 m/s per bit | [CONFIRMED against GNSS SOG] |
| b7 bits0-1 | Direction: `01` forward, `00` **reverse**, `11` not available / standstill | [CONFIRMED on a reversing segment] |

This is useful for AOG reverse detection and for the "standstill ⇒ not ready" behaviour.

### 3.7 Other 0x1C → * frames from the factory nav controller (possible keep-alives)

If the Xerion ECUs expect these, we will have to emulate them after removing the factory controller.

| ID | Rate | Content | Tag |
|---|---|---|---|
| `0x1CEF5A1C` | 10 Hz | Constant `04 BF B7 1B D1 5F 9A 0A` in every capture | [UNKNOWN — heartbeat/auth to 0x5A?] |
| `0x1CFFCE1C` | 10 Hz | b0 = `0x31`, rest varies (b4-b7 look like signed values; possibly cross-track for the terminal) | [UNKNOWN] |
| `0x1CFFCC1C` | 1 Hz | `00 00 80 00 80 00 01 xx` | [UNKNOWN] |
| `0x1CFFCD1C` | 1 Hz | Constant `0D 98 7E A2 80 00 0C E4` | [UNKNOWN] |

`0x18EF1C5A` (0x5A → 0x1C, 1 Hz): `0x 00 41 b3 b4 b5 64 00`. b3 falls with ground speed (≈ km/h?), b4/b5 rise when slowing into turns. It looks like a speed-dependent limit, not an angle. [UNKNOWN]

### 3.8 GNSS on the V-Bus (0x1D, all 10 Hz unless noted)

`0x19F8011D` PGN 129025 (lat/lon rapid, 1e-7°), `0x19F8021D` PGN 129026 (COG 1e-4 rad at b2-b3, SOG 0.01 m/s at b4-b5), `0x19F8051D` PGN 129029 fast-packet, `0x18FEE61D` PGN 65254 time/date (1 Hz), `0x18FEE81D` PGN 65256, `0x18FEF31D` PGN 65267. [CONFIRMED decodes]

Optional future use: the Teensy could build PANDA sentences straight from the tractor's own receiver.

### 3.9 K-Bus (cab): `0x10613173`, SA 0x73

b0 = `0x01` / `0x03`. Owner's note: `0x01` steering on, `0x03` off. In `joystick can.csv` it toggles often, sometimes within a few hundred frames, which suggests it may be a momentary press/release rather than a latched state. [LIKELY, needs a capture with a written press log.] Handle it edge-triggered with debounce.

---

## 4. Problems in the current `CAN_All_Brands.ino` for the Xerion

1. **Wrong addresses on transmit.** `VBus_Send()` for `Brand == 0` sends `0x0CAD131E` (0x1E → 0x13, the Claas combine layout). The Xerion steering controller is at **0xD2** and listens to **0x1C**. As uploaded, the code can read the Xerion but can never command it.
2. **Wrong address claim.** It claims `0x18EEFF1E` on V-Bus and ISO-Bus. For the Xerion, if a claim is sent at all it must be for 0x1C, and the logs show no claims on this bus. `CANBUS_ModuleID` is also assigned twice (0x1E, then 0x1C).
3. **Readiness latches.** `if (buf[2] == 52) steeringValveReady = 16;` never clears it, so after an override the firmware still thinks the valve is ready.
4. **Claas and Xerion share `Brand == 0`.** The filters `0x0CAC1E13` and `0x1CFFE6D2` are combine-specific. The `0x18EF1CD2` parser branches don't match Xerion payloads.
5. **Intent byte format** differs from the factory controller (see 3.1).
6. **No detection of a foreign 0x1C.** Nothing stops the firmware from transmitting while the factory controller is still connected.

The conversion between `estCurve`/`setCurve` and AOG steer angle (degrees) is not in the uploaded file. Find it in the rest of the sketch and keep the 32128 / 0.25 km⁻¹ convention, since both `0x0CFFA25A` and `0x0CADD21C` use it.

---

## 5. Phase 1 — Firmware: a dedicated Xerion brand

### 5.1 Brand

Add a new brand constant (next free index, e.g. `10 = Claas Xerion`), selectable from the Service Tool like the others. Leave `Brand == 0` as Claas combine/tractor, and remove the Xerion bits from it (or leave them harmless).

### 5.2 V-Bus setup

- 250 kbit/s, FIFO, reject-all, then accept:
  `0x0CFFA25A`, `0x0CAC1CD2`, `0x18EF1CD2`, `0x18FFE1D2`, `0x0CFE48D2`, **and `0x0CADD21C`**. The last one is only there to detect a foreign 0x1C; our own TX is not received back unless loopback is enabled, so verify loopback is off.
- `CANBUS_ModuleID = 0x1C`. No address claim by default. Add a compile-time option to send a claim for 0x1C if testing shows the D2 needs one.

### 5.3 Receive handling

```c
// 0x0CFFA25A  (20 Hz) — true path curvature, BIG-endian
estCurve        = (buf[0] << 8) | buf[1];
xerionYawRaw    = ((buf[2] << 8) | buf[3]) - 32768;     // optional
xerionHeadingDeg= ((buf[4] << 8) | buf[5]) * 360.0f / 65536.0f; // optional
lastCurveMs     = millis();

// 0x0CAC1CD2  (1 Hz) — status + front-based curvature
xerionStatus    = buf[2];
xerionAcCurve   = buf[0] | (buf[1] << 8);               // LITTLE-endian, keep for crab (Phase 2)
lastStatusMs    = millis();

// 0x18EF1CD2  (10 Hz)
xerionEngagedFast = (buf[0] & 0x04) != 0;

// 0x18FFE1D2  (20 Hz)
xerionStateCode = buf[0];

// 0x0CFE48D2  (10 Hz)
wheelSpeed_mps  = (buf[0] | (buf[1] << 8)) * 0.001f;
dirBits         = buf[7] & 0x03;   // 1 fwd, 0 rev, 3 n/a

// 0x0CADD21C  — anyone else commanding the D2?
foreignNavSeenMs = millis();       // → inhibit our TX (5.6)
```

### 5.4 Readiness / engage mapping onto the existing firmware variables

- `steeringValveReady` must be **recomputed every frame, never latched**:
  - ready-to-engage = `(xerionStatus & 0x04)` **and** status frame fresh (< 2.5 s old).
  - Expose it to the rest of the firmware using whatever value the existing AOG logic treats as "ready" (other brands use `0x10`/`0x40`). Check how `steeringValveReady` is consumed elsewhere in the sketch before picking the value.
- Actually-steering (for AOG button/LED feedback) = `xerionEngagedFast` (10 Hz). `(xerionStatus & 0x40) == 0` is the 1 Hz confirmation.
- **Operator override:** if we were steering and `xerionEngagedFast` drops (or E1 b0 leaves `0x42`), then drop `intendToSteer` to 0 immediately, tell AOG autosteer is off, and require a fresh engage request (button or AOG) before re-asserting intent. This matches what the factory controller does, and avoids holding intent against a controller that has requested a reset.
- K-Bus `0x10613173`: treat a transition into `0x01` as an engage request, and into `0x03` as disengage. Debounce ~200 ms. Confirm semantics with a capture (§9).

### 5.5 Transmit `0x0CADD21C` at 10 Hz (100 ms, don't burst)

```c
msg.id = 0x0CADD21C; msg.flags.extended = 1; msg.len = 8;
msg.buf[0] = lowByte(setCurve);
msg.buf[1] = highByte(setCurve);
msg.buf[2] = intendToSteer ? 0x01 : 0x00;
msg.buf[3..7] = 0x00;
// when not steering: setCurve = 32128 → 80 7D 00 00 00 00 00 00 (identical to factory idle frame)
```

- Clamp `setCurve` to 32128 ± 800 counts. Factory commands stayed within ±600; the measured curvature hit ±804 at full lock.
- Add a slew-rate limit (tune on the tractor; start around 100 counts per 100 ms frame) so a jump in AOG's setpoint can't slam the valve.
- Keep transmitting the idle frame whenever the brand is active, including when not steering. The factory controller never goes silent.

### 5.6 Safety interlocks (all must be implemented)

1. `foreignNavSeenMs` within the last 1 s → **never transmit**. Report an error to AOG / serial ("factory nav controller still connected").
2. `0x0CFFA25A` stale for more than 250 ms, or `0x0CAC1CD2` stale for more than 2.5 s → force `intendToSteer = 0`.
3. `dirBits == 3` (standstill) or speed below the AOG minimum → intent 0. The D2 appears to drop "ready" at standstill anyway.
4. Status `0x70` for more than 3 s while running → surface "Xerion steering not ready — may need key cycle" to AOG/serial. Don't retry automatically.

### 5.7 Keep-alive emulation (only if needed)

First test **without** emulating the other factory 0x1C frames. If after removing the factory controller any of these happen, replay the frames from §3.7 at their observed rates, one at a time, to find the one that's required:

- `0x0CFFA25A` stops or freezes,
- the AC status never reaches `0x74`,
- the D2 ignores intent.

Start with `0x1CEF5A1C` (constant payload, 10 Hz). Put each emulated frame behind its own compile flag.

---

## 6. Phase 2 — Crab steering and AgOpenGPS

### 6.1 What happens physically

The Xerion can steer front-only, coordinated 4-wheel, or crab. In crab, both axles steer to the same angle δ. The body doesn't yaw, but every point of the machine travels at angle β = δ relative to the body axis.

- `0x0CFFA25A` curvature stays ~0: the path is straight. This is correct and is what the closed steering loop needs.
- `0x0CAC1CD2` (1 Hz) reported a non-zero value in crab (owner's observation). **Hypothesis [LIKELY]:** the D2 computes this ISO field from the **front axle angle only**, as `κ_AC = tan(δf)/L`. This fits the logs, where κ_AC ≈ κ_A2 in all the front-steer driving captured.

General two-axle kinematics, with δf and δr as front/rear wheel angles, wheelbase L, reference point at the rear axle centre:

```
κ_true = (tan δf − tan δr) · cos δr / L          (curvature at rear-axle centre)
β_rear = δr                                       (travel direction vs body at rear axle)
β_P    = atan( tan δr + d·κ_true / cos δr )       (at a point P a distance d ahead of rear axle)
```

Combining both CAN values under the hypothesis `tan δf = κ_AC·L`:

```
tan δr ≈ (κ_AC − κ_A2) · L          (small-angle; exact form: solve with cos δr term)
β_rear  = atan(tan δr)
```

Sanity checks: front-steer gives κ_AC ≈ κ_A2, so δr ≈ 0 ✓. Pure crab gives κ_A2 = 0, so δr = atan(κ_AC·L) = δf ✓.

Units: κ in m⁻¹ = (raw − 32128) · 0.25 / 1000. L = the Xerion's wheelbase. It must be a firmware setting, measured on the actual machine.

**If a capture turns up a direct rear-axle-angle signal (§9), use that instead of this derivation.**

κ_AC is only 1 Hz. Sample κ_A2 at the moment each AC frame arrives, then low-pass β (τ ≈ 2 s). Crab angle is set by the operator and changes slowly, so 1 Hz is fine for this purpose. It is **not** fine for steering control; keep using `0x0CFFA25A` for that.

### 6.2 Why AOG needs to know β, and single vs dual antenna

AOG assumes that heading = direction of travel = body axis. In crab these differ by β.

| Setup | What AOG gets today in crab | Consequence |
|---|---|---|
| Single antenna (fix-to-fix heading) | Travel heading | Steering loop behaves, but: (a) the tractor is drawn along the travel line, not sideways; (b) the antenna→pivot offset is applied along the wrong axis, so the pivot is misplaced laterally by `d_antenna·sin β` (2 m × sin 10° ≈ 0.35 m); (c) hitch/implement positions are wrong by `L_hitch·sin β`, so section control and coverage are offset. |
| Dual antenna | Body heading | Drawing is right, but the guidance thinks the vehicle is pointing β off the line and steers to "correct" it, fighting the crab. That gives a persistent cross-track offset or oscillation. |

### 6.3 Options

**Option A — Recommended: teach AOG about β.**

1. Protocol: the steer module sends β to AOG. Either add a field to the steer-module → AOG data PGN, or define a new small module PGN (e.g. `int16 β×100` degrees, 0x7FFF = not available). Check the AgOpenGPS/AgIO sources for free PGN numbers, and check that AgIO forwards unknown module PGNs unchanged. Don't assume either.
2. AOG keeps two headings:
   - `bodyHeading` — used to rotate vehicle-frame offsets (antenna → pivot, pivot → hitch/tool) and to draw the vehicle.
   - `travelHeading = bodyHeading + β` — used by the guidance controllers (Pure Pursuit / Stanley) and anything that predicts where the pivot moves.
3. Deriving them per heading source:
   - Single antenna: `travelHeading_antenna` = fix-to-fix/COG; `bodyHeading = travelHeading_antenna − β_antenna` (β_antenna ≈ β_rear in pure crab; use the β_P formula when turning).
   - Dual antenna: `bodyHeading` = dual heading; `travelHeading = bodyHeading + β_pivot`.
4. With β = 0 or not available, behaviour must be bit-for-bit identical to today. Every other tractor goes through the same code.
5. Locate in the AOG C# source (grep for these; names may differ by version): where the fix heading is chosen and fused (fix-to-fix / dual / IMU), where the pivot and steer-axle positions are computed from the antenna, where hitch/tool positions are computed, and the vehicle draw routine. Apply `bodyHeading` to the geometry and `travelHeading` to the guidance.

**Option B — Firmware-only fake heading (the owner's original idea). Not recommended except for display.**

Feeding AOG a modified heading via PANDA/PAOGI (e.g. travel − β as a fake dual heading) makes the tractor look sideways and fixes the implement geometry. But AOG's guidance then sees a heading error of β, and will steer against the crab, as in the dual-antenna row above. It only works if AOG is not closing the steering loop. If this is ever built as a stop-gap, put it behind an explicit setting that defaults to off.

**Decision:** Phase 1 (firmware) first; it works in crab already with a single antenna, apart from the geometry offsets. Then do Option A as an AOG patch plus the new PGN. Skip Option B.

---

## 7. Implementation order for Claude Code

1. Read the whole sketch (all tabs, not just `CAN_All_Brands.ino`). Find: brand selection/EEPROM, how `estCurve`, `setCurve`, `steeringValveReady`, `intendToSteer`, and `engageCAN` are used; the curve⇄angle conversion; and the AOG PGN handling.
2. Add brand `Claas Xerion` (§5.1–5.3). Leave all other brands untouched.
3. Implement readiness/engage logic (§5.4) and TX (§5.5) with interlocks (§5.6).
4. Add a serial debug mode that prints, at 5 Hz: status byte, E1 state, EF engaged bit, estCurve, setCurve, intent, dirBits, speed, foreign-nav flag.
5. Bench test on a CAN bench (PCAN/SavvyCAN playback of the captured logs): verify parsing and verify that TX is inhibited while `0x0CADD21C` from another node is present. The captured logs contain the factory controller, so interlock 1 **must** trip when they are replayed.
6. Tractor test per §8.
7. Phase 2 firmware: compute β from κ_AC, κ_A2 and wheelbase (setting), and send it on the new PGN. Behind a setting; default off.
8. Phase 2 AOG patch (separate branch/PR), per §6.3 Option A.

---

## 8. Tractor test procedure

1. Factory nav controller disconnected. Key on, engine running. Confirm on serial: no foreign 0x1C, status reaches `0x74`, `0x0CFFA25A` live.
2. Stationary: our idle frame only (`80 7D 00…`). Confirm status stays at `0x74`/`0x70`, and no fault after 5 min.
3. Slow straight line, 3–5 km/h, open field: engage from AOG. Expect E1 `→03→42`, EF bit2 set, status `0x34`.
4. Override test: turn the steering wheel. Expect a fast disengage (EF bit2 clears); firmware drops intent and AOG shows off. Re-engage must require a new request.
5. Step tests: small curvature steps, then line acquisition. Tune slew limit and AOG gains.
6. Only then headland turns, 4WS mode, and crab mode.
7. At every stage, log the full V-Bus with proper timestamps.

---

## 9. Captures still needed (with working timestamps)

Fix the timestamp problem first: use SavvyCAN with a GVRET/PCAN interface that provides µs timestamps, or check the Teensy capture firmware's timestamp width.

| # | Capture | Answers |
|---|---|---|
| 1 | Crab mode: set a crab angle left, drive straight 30 s; right; then back to 0. Dual antenna or a measured reference if possible. | Confirms `0x0CAC1CD2` = front-axle curvature; lets β = atan(κ_AC·L) be checked against COG − true heading. Also shows whether `0x0CFFA25A` b4-b5 is body or travel heading. |
| 2 | Stationary, engine running, rear axle steered by hand in each steering mode | Look for a direct rear-axle angle signal (check 0x5A/0x5D/0xD2 frames). |
| 3 | 4WS coordinated mode turns | Validates the κ_true formula. |
| 4 | K-Bus: press engage 5×, with a written log of press times | Momentary vs latched semantics of `0x10613173`. |
| 5 | Our firmware steering with the factory controller removed, if §5.7 is needed | Which keep-alives are required. |
| 6 | Reproduce the `0x70` "needs restart" state if possible, with 30 s of lead-in | What triggers the latch. |

---

## 10. Appendix — analysis helpers

Load any of the SavvyCAN CSVs; the timestamp column is ignored and the row index is used as time:

```python
import pandas as pd
def load(path):
    df = pd.read_csv(path, dtype=str, usecols=range(14))
    df.columns = ['ts','id','ext','dir','bus','len'] + [f'd{i}' for i in range(8)]
    df['row'] = range(len(df))
    for i in range(8):
        df[f'b{i}'] = df[f'd{i}'].fillna('0').apply(lambda s: int(s, 16))
    return df

def ser(df, can_id, fn):
    x = df[df.id == can_id]
    return pd.Series(fn(x).values, index=x.row.values)

# examples
curv_A2 = lambda d: ser(d, '0CFFA25A', lambda x: x.b0*256 + x.b1 - 32128)           # BE
curv_AC = lambda d: ser(d, '0CAC1CD2', lambda x: x.b1*256 + x.b0 - 32128)           # LE
head_A2 = lambda d: ser(d, '0CFFA25A', lambda x: (x.b4*256 + x.b5) * 360/65536)
cog     = lambda d: ser(d, '19F8021D', lambda x: (x.b3*256 + x.b2) * 1e-4 * 57.2958)
sog     = lambda d: ser(d, '19F8021D', lambda x: (x.b5*256 + x.b4) * 0.01)
```

Absolute rate recovery when timestamps are broken: count frames between changes of the seconds field in `0x18FEE61D` (PGN 65254, b0 = seconds × 4).

Measured rates (CAN3_XERION.csv, 592 s): `0x0CFFA25A` 20 Hz, `0x18FFE1D2` 20 Hz, `0x0CADD21C` 10 Hz, `0x18EF1CD2` 10 Hz, `0x0CFE48D2` 10 Hz, `0x1CEF5A1C` 10 Hz, `0x1CFFCE1C` 10 Hz, GNSS 129025 10 Hz, `0x0CFF975D` 50 Hz, `0x0CAC1CD2` 1 Hz, `0x18EF1C5A` 1 Hz, `0x1CFFCC1C` 1 Hz.
