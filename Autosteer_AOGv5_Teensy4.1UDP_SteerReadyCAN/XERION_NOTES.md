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
  scheme the other brands use), clamped to `32128 ± 800` counts.
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

## What's deliberately not done yet

- **Crab/rear-axle angle (β) and the AgOpenGPS side of things.** The spec's
  Phase 2 — deriving β from `0x0CAC1CD2` vs `0x0CFFA25A` curvature and feeding
  it to AgOpenGPS so the machine draws and steers correctly while crabbing —
  needs a new AOG-facing CAN PGN and a matching AgOpenGPS (C#) patch. Both are
  out of scope for this firmware-only pass; see spec §6–§7.
- **Keep-alive emulation** for the other `0x1C → *` frames (§3.7) — only
  wire these up if removing the factory controller turns out to make the
  Xerion drop `0x0CFFA25A`, stop reaching AC status `0x74`, or ignore intent.
  Each one should go behind its own compile flag if needed.

## Biggest open questions (need a real machine)

These are flagged `[LIKELY]` or `[UNKNOWN]` in the spec — worth watching for
specifically during testing:

1. **Status byte bit meaning** (`0x0CAC1CD2` b2). We treat `0x04` as "ready"
   and `0x40` as "not steering", based on fitting every observed value
   (`0x34`, `0x74`, `0x70`, `0x30`) — but this hasn't been checked against
   the actual ISO 11783-7 byte-3 layout.
2. **Address claim.** Now sent by default (see above) — watch for the D2
   rejecting it or faulting.
3. **K-Bus button semantics** (`0x10613173` b0, `0x01`/`0x03`). Owner's note
   says `0x01` = on, `0x03` = off, but the capture shows it toggling quickly
   enough that it might be momentary press/release rather than a latched
   state. If it's momentary, our edge-trigger-on-`0x01` handling is right;
   if it's latched and toggles for other reasons, it could misfire.
4. **Keep-alives** — does the D2/5A stack need any of the other `0x1C`
   broadcast frames once the factory controller is gone?
5. **`0x70` latch** — what actually triggers the "needs restart" state, and
   does our interlock logic avoid causing it?

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
