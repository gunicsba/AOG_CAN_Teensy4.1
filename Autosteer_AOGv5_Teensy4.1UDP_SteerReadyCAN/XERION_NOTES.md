# Claas Xerion support (Brand 10) — status & call for testers

This firmware now has a dedicated brand for the Claas Xerion (`Brand == 10`,
Service Tool key `A`). It was built entirely from CAN bus captures and field
notes — **it has not been run on a Xerion.** The full reverse-engineering
writeup this is based on is [XERION_AUTOSTEER_SPEC.md](XERION_AUTOSTEER_SPEC.md);
this file is the short version: what's implemented, what's assumed, and what
we need a real machine to confirm.

If you own a Xerion and are willing to test, start at the "Bench test" and
"Tractor test" sections below — please report back with a CAN log either way.

## Before you power this up on a real machine

**The factory navigation controller at address `0x1C` must be physically
disconnected.** This firmware takes over `0x1C` and sends `0x0CADD21C`
curvature commands to the steering controller (`0xD2`) just like the factory
controller does. Two nodes doing that at once will fight each other, and is
a likely cause of the "steering error, needs restart" state some owners have
seen. The firmware detects a foreign `0x1C` (interlock below) and refuses to
transmit if it sees one, but that's a safety net, not a green light to leave
the factory controller wired in.

## What's implemented (Phase 1 — firmware only)

- V-Bus (CAN3, 250 kbit/s) receive parsing for all five Xerion frames:
  `0x0CFFA25A` (true curvature/yaw/heading, 20 Hz), `0x0CAC1CD2` (guidance
  machine status, 1 Hz), `0x18EF1CD2` (fast engaged bit, 10 Hz), `0x18FFE1D2`
  (steering controller state, 20 Hz), `0x0CFE48D2` (wheel speed/direction,
  10 Hz).
- Transmits `0x0CADD21C` (curvature command) with the factory's exact byte
  layout (`0x01`/`0x00` intent byte, zero padding — not the `0xFD`/`0xFC`
  scheme the other brands use) at a fixed 10 Hz, matching the factory nav
  controller's own rate rather than piggybacking on the main loop's 25 Hz.
  Curvature is pinned at `32128` (straight) whenever intent is 0, matching
  every idle frame in the captures byte-for-byte; while actively steering
  it's clamped to `32128 ± 800` counts.
- Community tip: also replay the factory controller's other `0x1C` broadcast
  frames (§3.7) at their own observed rates, not just the curvature command —
  `0x1CEF5A1C` and `0x1CFFCC1C`/`0x1CFFCD1C` at the confirmed rates (10 Hz,
  1 Hz, 1 Hz). `0x1CFFCE1C` is implemented but **off by default** — see
  "Verified against the raw captures" below for why. Each is behind its own
  compile flag in the main `.ino` (`XERION_EMULATE_...`) in case a specific
  frame turns out to upset something on a real machine.
- Sends a proper address claim for `0x1C` on power-up (`XERION_SEND_ADDRESS_CLAIM`
  in the main `.ino`, on by default — flip to `0` if a real Xerion turns out to
  dislike it; the captures never showed one being sent, but they also never
  showed the factory controller contending with anything for the address).
- Readiness is recomputed every cycle from the status bitfield, never latched,
  gated on frame freshness (`0x0CFFA25A` < 250 ms, `0x0CAC1CD2` < 2.5 s) and
  standstill (`0x0CFE48D2` direction bits == 3 = n/a).
- Fast operator-override detection off the 10 Hz `0x18EF1CD2` engaged bit
  (not the slow 1 Hz status frame) — dropping it forces a "not ready" state
  that requires a fresh engage request before intent is re-asserted, same
  mechanism the other brands already use for this.
- Hard interlock: if `0x0CADD21C` is seen from anyone but us, we stop
  transmitting entirely and print a warning (rate-limited) until it goes away.
- A 3 s-latched warning if status stays at `0x70` ("not ready") while running.
- K-Bus cab engage button (`0x10613173`), edge-triggered with ~200 ms
  debounce, wired into the existing generic engage path.
- 5 Hz debug summary line (status byte, E1 state, EF engaged bit, estCurve,
  setCurve, intent, direction bits, speed, foreign-nav flag) — printed when
  `ShowCANData` is on (`R` in the Service Tool) and brand is Xerion.
- Angle conversion, WAS mapping, Ackerman fix, and setCurve slew-rate
  limiting reuse the existing generic code paths (same as Claas/Valtra),
  no Xerion-specific changes were needed there.

## Verified against the raw captures

The spec above was already built from these captures, but the firmware was
written against the spec's *summary*, not the raw CAN logs. We went back and
checked the implementation against the four capture CSVs directly (`CAN3_XERION.csv`,
`xerion disengage turn right then engage.csv`, `xerioncan3.csv`, `joystick can.csv`
— all from `xerion.zip`; we don't have `starting_up_the_tractor.csv`). Findings:

- **Rates all check out exactly.** Using the spec's own GNSS-seconds recovery
  method (§9 appendix — the `Time Stamp` column is unusable, confirmed: it
  repeats the same value across tens of thousands of unrelated rows, it's not
  just wrapping), every rate we assumed matches: `0x0CAC1CD2` ~1.0 Hz,
  `0x18EF1CD2`/`0x0CFE48D2`/`0x0CADD21C`/`0x1CEF5A1C`/`0x1CFFCE1C` ~10 Hz,
  `0x18FFE1D2`/`0x0CFFA25A` ~20 Hz, `0x1CFFCC1C`/`0x1CFFCD1C` ~1 Hz, across
  all three V-Bus files.
- **Found and fixed: idle-frame curvature.** Every single `0x0CADD21C` frame
  with intent byte `0` (707 of them, across all three files) is byte-for-byte
  `80 7D 00 00 00 00 00 00` — the factory always pins curvature at `32128`
  when not steering, regardless of the vehicle's actual curvature. Our
  firmware previously sent whatever `setCurve` happened to hold (which tracks
  the actual curvature when idle, for AOG's angle display) — now fixed to
  send the fixed idle pattern on the wire while leaving the AOG-facing
  `setCurve`/angle-display value untouched.
- **Clamp range vs. what was actually observed.** Commanded curvature
  (`0x0CADD21C`) from the factory stayed within exactly `32128 ± 600` in every
  file — but *actual* curvature (`0x0CFFA25A`) hit as far as `32128 - 917` in
  one file and `32128 + 834` in another, i.e. slightly past our current
  `± 800` TX clamp on one side. We left the clamp at `± 800` (it's a safety
  margin on what we command, not a target to hit) rather than change a
  safety-relevant number without a real machine to check against — worth
  revisiting once someone can confirm true full-lock values on their tractor.
- **K-Bus button (`0x10613173`) semantics — resolved, mostly.** The frame is
  1 byte long (`LEN=1`), and across the whole `joystick can.csv` capture it
  transitions `0x01`/`0x03` **47 times, perfectly alternating, with zero
  chatter or repeats**, and is only ever sent when the value changes (89
  total frames across a 415k-row file — not periodic). That's a clean
  send-on-change state report, not bounce, which supports the owner's
  `0x01`=on/`0x03`=off reading and means our edge-trigger approach is on
  solid ground. What's still unconfirmed is whether `0x01` really means
  "engage" in the AOG sense (vs. some other cab state) — we don't have a
  press log correlated with what the operator was actually doing.
- **Found: `0x1CFFCE1C` payload is not a heartbeat.** Only byte 0 (`0x31`) is
  constant. The other 7 bytes are essentially unique on nearly every frame in
  all three V-Bus files (4796/5825, 781/783, and 493/493 distinct payloads
  respectively) and don't even share the same "typical" values between
  capture sessions. This is live data, not a fixed protocol constant, so a
  canned replay would be fabricated content rather than a best-effort guess —
  disabled by default (see above).
- **Everything else matches:** `0x0CAC1CD2` status byte only ever takes the
  four documented values (`0x34`/`0x74`/`0x70`/`0x30` — and `0x30` never
  actually appears in any of the four files); `0x18EF1CD2` bytes 1-2 are
  `CF 7B` in 100% of frames; `0x18FFE1D2` state codes never exceed the
  documented set; `0x0CADD21C` bytes 3-7 are always zero; `0x1CEF5A1C` and
  `0x1CFFCD1C` are byte-for-byte constant in all three files, matching the
  values already coded; `0x1CFFCC1C` is constant except its last byte (which
  the code already flagged as unconfirmed); no address-claim traffic
  (`PF=0xEE`) appears anywhere.

## What's deliberately not done yet

- **Crab/rear-axle angle (β) and the AgOpenGPS side of things.** The spec's
  Phase 2 — deriving β from `0x0CAC1CD2` vs `0x0CFFA25A` curvature and feeding
  it to AgOpenGPS so the machine draws and steers correctly while crabbing —
  needs a new AOG-facing CAN PGN and a matching AgOpenGPS (C#) patch. Both are
  out of scope for this firmware-only pass; see spec §6–§7.

## Biggest open questions (need a real machine)

These are flagged `[LIKELY]` or `[UNKNOWN]` in the spec — worth watching for
specifically during testing:

1. **Status byte bit meaning** (`0x0CAC1CD2` b2). We treat `0x04` as "ready"
   and `0x40` as "not steering", based on fitting every observed value
   (`0x34`, `0x74`, `0x70`, `0x30`) — but this hasn't been checked against
   the actual ISO 11783-7 byte-3 layout.
2. **Address claim.** Now sent by default (see above) — watch for the D2
   rejecting it or faulting.
3. **K-Bus button meaning** (`0x10613173` b0, `0x01`/`0x03`). The *pattern*
   is now confirmed clean (see above — a perfectly alternating send-on-change
   report, not bounce), so our edge-trigger handling is on solid ground
   mechanically. What's still unconfirmed is whether `0x01` really correlates
   with the operator asking for autosteer, specifically — needs a press log.
4. **`0x1CFFCE1C`.** Left disabled (see above) — if you want to help pin down
   what it actually carries, a capture with driving notes (speed, turning,
   cross-track) alongside it would help decode the varying bytes.
5. **`0x70` latch** — what actually triggers the "needs restart" state, and
   does our interlock logic avoid causing it?
6. **Clamp range.** Our `± 800` TX clamp is slightly inside the actual
   observed curvature extremes (`-917`/`+834`) — confirm real full-lock
   values and whether `± 800` is unnecessarily conservative.

## Test procedure

Bench first (replay the captured logs over PCAN/SavvyCAN and confirm parsing,
and specifically confirm interlock 1 trips — the captured logs contain the
factory controller, so TX must stay suppressed when they're replayed), then
on the tractor: factory controller disconnected, confirm no foreign `0x1C`
and status reaches `0x74`, stationary idle-frame check, slow straight-line
engage, operator-override test, then step tests before headland turns or
crab mode. Full procedure in spec §8. Please log the whole V-Bus with proper
timestamps for any test (see spec §1 capture caveat and §9 for what's still
needed) — even a failed test is useful data.
