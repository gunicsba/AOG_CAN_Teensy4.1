# AOG_CAN_Teensy4.1
Steering navagation controller files for Teensy based boards to use with AgOpenGPS 

If your new to the teensy this is what ive had to do:
1. You need to update the Arduino IDE
2. Install the Teensy program
3. The new updated Teensy program 1.55 now inclucdes the latest FlexCAN_T4 & Native Ethernet so you no longer have to update manually.

CaseIH Connections:  
CAN1-H = Pin C Diagnostic connector right rear pillar  
CAN1-L = Pin D Diagnostic connector right rear pillar  
CAN3-H = Pin 3 Nav controller 40 pin connector X-716 or Pin H Diagnostic connector behind seat    
CAN3-L = Pin 13 Nav controller 40 pin connector X-716 or Pin J Diagnostic connector behind seat  

[CaseIH_GPS.pdf](https://github.com/MechanicTony/AOG_CAN_Teensy4.1/files/10528432/CaseIH_GPS.pdf)

Deutz Warrior
CAN1 = Pin CD Diagnostic connector
CAN3 = Pin EF Diagnostic connector

## Steering diagnostics CSV mode (Claas)

A read-only logging mode for finding the real wheel-angle scale, the centre offset and the cause of a jumpy wheel angle on Claas / Danfoss PVED-CL tractors (Brand 0). It sends nothing on the bus and does not change any PVED parameter. When it is off, firmware behaviour is unchanged.

**Start / stop:** send `D` on USB serial (115200), either from normal running or from inside the service tool. Selecting it from the service tool closes the service tool, because logging needs the main loop running. Send `D` again to stop. Only one CSV line is printed each time `0x0CAC1E13` arrives from the PVED (about every 80 ms). Lines starting with `#` are comments. The test procedure is printed when logging starts.

While logging is on, five extra V-Bus FIFO filters are added: `0x301`, `0x105` (11-bit), `0x18FFE2D2`, `0x0CAD131B` and `0x18FEF1FB`. When it stops, the normal Claas filters are put back.

**Marker keys** (logging on): `0` straight, `L` full left lock, `R` full right lock, `E` autosteer engaged, `M` free marker. A marker goes on the next CSV line only. In the Arduino serial monitor, type the key and press Enter.

**Capture:** use a terminal that logs to a file and still lets you type keys, such as PuTTY (Session > Logging > "All session output") or Tera Term (File > Log). The analysis script skips comments and any other firmware output.

| Column | Meaning |
|---|---|
| `t_ms` | Teensy `millis()` when `0x0CAC1E13` was received |
| `mark` | marker key, if one was pressed since the last line |
| `estCurve`, `valveState` | `0x0CAC1E13` bytes 0-1 (uint16 LE) and byte 2 |
| `curv_kmInv` | `estCurve * 0.25 - 8032` (km⁻¹) |
| `angle_div10` | `(estCurve - 32128) / 10`, the angle at the old scale (deg) |
| `angle_div23_3` | `(estCurve - 32128) / 23.3`, theoretical scale for a 3.0 m PVED vehicle length (deg) |
| `r301_raw`, `r301_cnt`, `r301_delta` | `0x301` bytes 0-1 (uint16), 2 (counter), 3-4 (int16) |
| `r301_age_ms` | time since the last `0x301` frame |
| `r105` | `0x105` bytes 0-1 (uint16) |
| `e2d2_delta` | `0x18FFE2D2` bytes 3-4 (int16) |
| `cmd131b_curve`, `cmd131b_status` | `0x0CAD131B` (Guidance System Command from SA 0x1B), bytes 0-1 and byte 2 |
| `speed_kmh` | CCVS `0x18FEF1FB` bytes 1-2 / 256 |
| `aog_setpoint_deg` | steer angle set point received from AOG |
| `aog_actual_sent_deg` | the actual-angle bytes of the last PGN 253 sent to AOG, i.e. exactly what AOG received |
| `steer_engaged` | `intendToSteer` (1 = firmware is steering) |
| `pwm_or_cmd_out` | `setCurve`, the curvature command sent to the PVED in `0x0CAD131E` |
| `r301_status` | `0x301` byte 5 |

A field is left empty if its message has not been received within the last 2 s. Every frame's DLC and 11/29-bit flag are checked before the frame is used. A frame that is too short is ignored, and a one-off `# WARNING` line is printed.

**Note:** while steering, the angle sent to AOG comes from `setCurve` (the command), not from `estCurve`.

### Analysis: `tools/steer_diag.py`

```
pip install numpy matplotlib
python tools/steer_diag.py capture.txt --left-deg 38.5 --right-deg 40.0
```

`--left-deg` / `--right-deg` are the front-wheel angles you measured at full lock. The script prints the following:
- capture summary: frame gaps, valve states, which messages were present, and `0x301` counter continuity
- centre offset and lock values from the `0` / `L` / `R` markers, and counts/° per side, including the effective vehicle length from κ = tan(δ)/L
- linear fit of `r301_raw` vs `estCurve` (slope, offset, R²) over the manual sweep, which runs by default from the first `0`/`L`/`R` marker to `E`
- noise (std, peak-to-peak) of `estCurve` and `r301_raw` in each `0` window
- every `estCurve` step over 20 counts (`--jump`), labelled as a single-sample spike (dropout) or a real step
- whether `0x301` moves while AOG steers the wheels: if it does not move, it is the steering-wheel SASA

It also saves `<capture>.png`: `estCurve` / `setCurve`, `r301_raw` and the AOG angles over time, with markers and the engaged periods shaded. Add `--show` to open the plot in a window.

**Test procedure**
1. Engine running, standing still, wheels straight: press `0` and hold for 10 s.
2. Turn the steering wheel slowly to full left, press `L`, hold for 3 s. Turn slowly to full right, press `R`, hold for 3 s. Return to straight and press `0`.
3. Repeat step 2 once, faster.
4. With AOG engaged, drive slowly on a straight AB line for 30 s and press `E`. Then let AOG steer a curve without touching the steering wheel.
5. Optional: if the service tool can show the PVED's live AD1 (WAS) value, read it at straight, full left and full right. Note each value next to the matching marker.
6. Measure the real front-wheel angle at full left and full right (angle app on the rim, or the turning-circle method).

## PVED WAS calibration and the atan wheel angle (Claas)

**Wheel angle:** by default Claas works as before: angle = (estCurve − 32128 + wasOffset) / AOG counts per degree. Once the PVED WAS calibration is real (65080/65083/65086, 65099/65100, 65112), you can switch to δ = atan(L × κ) with `V<mm>` in the service tool, with L = PVED 65112, e.g. `V2820`. `V0` switches back to linear, and `V` alone shows the current mode. The mode is saved in EEPROM address 76 and printed at startup. In atan mode, AOG's counts-per-degree setting is not used, but Zero WAS (wasOffset) still is, so set counts per degree to the value printed at startup. When 65112 is committed from the PVED config mode, the firmware L follows it only if atan mode is already on. It never switches atan mode on by itself.

One count is about 1/23° in atan mode instead of 1/CPD, so the autosteer loop may need a different Proportional Gain.

**Service tool `Z` (PVED config mode) commands:**
| Key | Action |
|---|---|
| `R` | read 22 parameters. Each reply also prints as `PARAM,<num>,<raw value>,<4 data bytes hex>,<A1/A3>`. **Save this log before changing anything**: it is the backup of the original values. |
| `P<num>=<raw>` | prepare a write (only 65080, 65083, 65086, 65099, 65100 or 65112). Prints old → new. |
| `Y` | send the prepared write and read it back |
| `C` | commit to PVED memory. If 65112 was written in this session and atan mode is on, the firmware L is updated too. |
| `T<8 hex bytes>` | send a raw frame to the PVED (`0x18EF13FD`), e.g. the StartStopStatus (XID 0x0FAB) request from your dump. Write and commit frames are blocked here. |
| `M<text>` | put a `MARK,<ms>,<text>` line in the log |

All other PVED proprietary frames (any priority or destination, e.g. the status stream) are printed as `STAT,<ms>,<id>,LEN n, <bytes>`, up to 4 per second. `S` shows every frame.

**Calibration steps**
1. `Z`, power on the PVED, `R`, and save the log (originals).
2. Start the AD1 stream with `T…` (frame from your dump). Read AD1 at full left, straight and full right, using `Mleft` / `Mstraight` / `Mright` markers.
3. Measure the real lock angles at the wheels.
4. Write them with `P65080=…`, `P65086=…`, `P65083=…`, then `P65099=…` / `P65100=…` (true angles, same raw format as the `R` read-out), and `P65112=…` if needed. Send `Y` after each one, then `C`. Power-cycle the PVED.
5. Check the result with the steering diagnostics mode (`D`): estCurve should follow tan(δ)/L, and the angle in AOG should match the measured angles.
