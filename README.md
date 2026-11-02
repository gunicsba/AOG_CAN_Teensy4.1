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

## Web page

The Teensy serves its own web page on port 80: `http://<subnet>.126/`, where the subnet is the one AgIO set (for example `http://192.168.5.126/`). The address is also printed on USB serial at start-up. There is no password: anyone on the module's network can open it.

**First install:** flash the `.hex` over USB with Teensy Loader as before. After that, updates can be done from the web page.

| Tab | What it does |
|---|---|
| Status | Steering (brand, valve state, wheel angle, curve, switches), AgOpenGPS connection, GPS data rate and fix (fix only in Panda mode), IMU, module (version, IP, CPU temperature) |
| Tractor & GPS | Tractor brand (0-9) and GPS mode (Forwarding / Panda, 115200 / 460800), the same settings as the serial service tool (`S`, then `0`-`9`, `f` / `F` / `p` / `P`). Saved in the EEPROM (70, 72), and the Teensy restarts. For Claas, the wheel angle mode (linear or atan with vehicle length L, the same as service tool `V`, EEPROM 76, no restart). Board serial ports of the GNSS, the TM171 and the RTK radio (EEPROM 200-203, restart): this board GNSS Serial3 / IMU Serial5 / radio Serial7 (default), AiO v4.x GNSS Serial7 / IMU Serial5 / radio Serial3. Serial1, 2 and 4 are not offered (CAN2, the work switch and remote inputs, the engage LED) |
| IMU | TM171 settings and calibrations (below). A BNO08x has nothing to set |
| PVED | Danfoss PVED-CL parameters: read, write, commit, the Claas hack, backup and restore, experimental valve auto-calibration, and the PVED's other frames (below) |
| Firmware | Update from a `.hex` file (Arduino IDE: Sketch > Export Compiled Binary), and restart |
| Messages | Firmware messages (TM171 replies, the frames sent and received, web actions). The same lines are printed on USB serial |

**Locks:** while autosteer is engaged, the page refuses brand and GPS changes, TM171 writes and calibrations, restart and firmware update. While the tractor moves (AgOpenGPS speed, or VTG in Panda mode, above 1 km/h), it also refuses the firmware update and the gyro and boot calibrations.

**TM171:** settings and calibration need the Teensy TX pin of the IMU port (TX5 = pin 20 on the default Serial5) wired to the TM171 RXD. Without it, roll and heading still work, but the TM171 never answers, and the page says so. The page reads the Setting object (TransducerM user guide V1.35, pages 39-42) and shows:
- Enable Gyro / Accel / Mag, GyroErrFilter with the accelerometer gain, Self-Adapt filter with the magnetometer gain
- power-on silent time (0-65 s), status output, boot mode, output data rate (25 / 50 / 100 Hz), and save to flash or this power-on only
- the UART baud rate and the inhibit time (read only), and the raw bytes

The guide gives the silent time as a 32-bit word at bytes 16-19. A real TM171 reads 25 (its output data rate) at bytes 18-19, so the firmware reads and writes the silent time as the 16-bit word at bytes 16-17 and leaves 18-19 alone. **Keep the silent time at 0:** the Teensy looks for the TM171 only once, about 3 s after power-on, and runs without an IMU if the TM171 is silent then. Before writing, the firmware checks the reserved bits and the baud word of what it read. If they don't match, writing is disabled and the bytes are shown. The roll-pitch-yaw and UART outputs always stay on.

Calibrations (Calibration object, guide pages 36-38):
| Button | Command | Notes |
|---|---|---|
| Gyro bias calibration (20 s) | 0x05 / 0x03, finished with 0x05 / 0x04 after 20 s | TM171 firmware V5.1+. Tractor still, engine may run. Kept until the TM171 is powered off. Finished early if autosteer engages or the tractor moves |
| Forced static calibration | 0x05 / 0x02 | the TM171 finishes it by itself |
| Revoke | 0x05 / 0x01 | back to the values before the forced calibration |
| Boot calibration (new install, CalibB) | 0x04 / 0x02 | about 20 s, saved to flash, cannot be stopped. Do not cut the power while it runs. Power-cycle afterwards. The frame is byte-identical to ImuAssistant's CalibB (`AA 55 10 17 08 80 57 04 02 35 AA 00…00 1C 25` for node ID 700); success is reported as type 0x14 / control 0x01 ("Boot calib: successful" in ImuAssistant) |
| Magnetometer hard-iron (flash / this power-on), clear (flash / this power-on) | 0x03 / 0x01, 0x02, 0x08, 0x09 | only useful with Enable Mag on. Progress is shown |
| Heading to magnetic north: start / stop and apply, and set heading | 0x03 / 0x0A, 0x0B, 0x0C | this power-on only |

Every calibration frame sent and every calibration report received is logged raw (`[tm171] calibration frame sent: …` / `calibration report received: …`) in the Messages tab and on USB serial, except the magnetometer progress reports.

**PVED tab:** the same service frames as the serial tool's `Z` mode (to `0x18EF13FD`: `0F A0` read, `0F A2` write, `0F A6 5A` commit; answers `0F A1` / `0F A3`), but the firmware keeps running.
- **Session:** opening it adds a V-Bus FIFO filter (slot 7) for every proprietary-A frame from the PVED (source address 0x13) and blocks autosteer. By default it also pauses the steering commands to the valve, as the serial tool did. It is refused while autosteer is engaged or the steering diagnostics CSV is on, and `D` is refused while it is open, because both need slot 7. It closes by itself a minute after the last page is closed. If the PVED does not answer, switch the PVED off and on while the session is open.
- **Parameters:** the 22 parameters of the serial tool's `R`, plus the sensor mapping and CAN address parameters from the PVED-CL Operation Manual appendix (64003, 64005, 64006, 64011, 64018, 65087, 65104, 65105, 65107, 65121), with value, bytes, original and age. 65104 PrimaryWheelAngleSensorInterface says where the wheel angle comes from (0 none, 1 AD1, 2 AD2, 4 CAN). The PVED, MMI and service tool addresses (64003 / 64005 / 64006) are read-only here. Every read is also logged as the serial tool's `PARAM,<num>,<raw>,<bytes>,<A1/A3>` line (Messages tab and USB serial). You can also read any parameter by its number. Requests go out one at a time: the next one after the answer, or after 1 s without one.
- **Writes:** only parameters in the list, and only once the parameter has been read since start-up. The first value read is kept as the original. Every write is read back. No write, commit or raw frame while the tractor moves. S16 values are sent sign-extended to 32 bits. Range checks: 64007 0-253, 65112 500-10000 mm, interfaces (65104 …) 0 / 1 / 2 / 4, presence flags 0 / 255. ⚠ marks the valve / cylinder calibration and the sensor mapping parameters.
- **Claas hack** (Claas only): the serial tool's `W`, i.e. 64007 = 30, 65101 = 255, 1027 = 0. Then Commit, wait 1 minute, and switch the PVED off and on.
- **Commit, backup, restore:** "Download backup" saves the current and original values as a CSV. "Restore originals" writes back every parameter that differs from its original (then Commit). After a commit that included 65112, the firmware vehicle length follows only if atan mode is already on, as with the serial tool.
- **Other frames:** the latest frame of each kind from the PVED, including its broadcasts PGN 0xFF01 / 0xFF02. You can send a raw 8-byte frame to the PVED, but write, RestoreDefaults, commit and calibration frames are refused there.
- **Valve auto-calibration (experimental):** the PVED-CL's own dead-band search (PVED-CL Operation Manual 11025583 rev CA, "Method 3: Valve auto-calibration", pages 29-33).
  - **Sequence:** 1. Enter calibration mode. This sends `0F A8 58` every 0.2 s for up to 20 s while asking for the mode (`0F A9`, answered by `0F AA` with 55 = calibration). 2. Start the auto-calibration with `0F B6`: XspStartSearch (u16 LE), XspIncrementSize, YactDiffThreshold, TimeOutPeriod (u16 LE ms). The PVED then moves the wheels, writes Xspl_0 / Xspr_0, shifts Xspl_1000 / Xspr_1000 and commits by itself. 3. Read the results. Then power-cycle the PVED.
  - **Monitoring:** status frames `0F B7` are shown raw, because the codes are not known. StartStopStatus `0F AB 01` turns on status set 1 (AD1, AD2, AD3, Xsp), shown as four 16-bit values from PGN 0xFF02 (a guess).
  - **Defaults:** the starting values come from the manual's table (EH dynamic 35/5/20/500, EH static and OSPEH 50/5/20/500, EHPS 130/5/20/500), preselected from ValveType (65121).
  - **Source address:** Danfoss says the start command "shall be sent from the MMI controller" (64005, default 252 = 0xFC); the diagnostics tool sends everything from 0xFD. The page lets you choose; which one the PVED accepts is not known yet.
  - **Requirements:** session open with the steering commands paused, tractor stopped, and the dead-band parameters read first, so "Restore originals" + Commit can undo the result.
  - **Danfoss preconditions:** WAS mapped and calibrated, SASA mapped, wheels straight, solid surface, no front load. Touching the steering wheel aborts.
  - **Seen on a real PVED (Claas Arion):** CommitDataResponse `0F A7 02` (02 = succeeded) and GetCurrentModeResponse `0F AA 55` (calibration mode).
  - **Source of the frames:** the XIDs and payloads are not from Danfoss' PVED-CL Communication Protocol (11025584, not public). They come from the XID table of a third-party "PVED-CL Diagnostics" tool, which agrees with every XID already used here: A0-A3 parameters, A4/A5 RestoreDefaults, A6/A7 commit, A8 EnterCalibrationMode, A9/AA GetCurrentMode, AB StartStopStatus, AD SetSpoolPosition, B3-B5 error entries, B6/B7 auto-calibration.

**Firmware update:** the Teensy stores the uploaded file in free flash (4 MB in) and checks all of it first: Intel HEX checksums, end record, start address, `fw_teensy41`, and this firmware's marker `AOG-CAN-T41-WEB-OTA-1`. Only then does it copy the file over the running firmware and restart (FlasherX). Every sector written is read back. The EEPROM (brand, GPS mode, AgOpenGPS settings) is kept. During the upload, steering is off and the valve gets "not steering" every 40 ms. Tick "Other firmware" to install a `.hex` without the marker (the next update then has to be over USB). A power cut during the final copy (a few seconds) leaves the Teensy without firmware. It then has to be reflashed over USB, but the Teensy itself is not damaged.

**Technical notes:**
- The network stack (FNET under NativeEthernet) is polled from `loop()` instead of from the library's 1 ms timer interrupt (`zNet.h`). That timer could change the sockets while `loop()` was using them, which crashed the AiO firmware this was ported from. Anything that runs its own long loop must call `netPoll()`. The serial service tool and the PVED config mode do.
- Web answers are sent a piece at a time from `loop()`, never more than the socket takes, so the 40 ms CAN steering cycle keeps running while the page loads.
- At boot, the Teensy first waits 0.5 s in `startup_middle_hook()`, before USB and `setup()`, and then another 0.5 s at the start of `setup()` as before.
- Ported from [renatorgr/Boards](https://github.com/renatorgr/Boards) `Firmware/v4.5_AiO_WEB` (GPL-3.0-or-later, see `COPYING` in the sketch folder). FlasherX by Joe Pasquariello is public domain.

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

   Note: 65080 / 65086 / 65083 scale the PVED's analogue input AD1. If 65104 = 4 the wheel angle sensor is on CAN (as on the Claas Arion, probably the 11-bit frame 0x105) and the PVED-CL manual lists no scaling parameters for it, so check first whether these three apply (compare `r105` with estCurve in a `D` capture). 65099 / 65100 and 65112 apply either way.
4. Write them with `P65080=…`, `P65086=…`, `P65083=…`, then `P65099=…` / `P65100=…` (MaxWheelAngleLeft / Right in millidegrees, measured on the wheel that carries the WAS, e.g. 38500 = 38.5°), and `P65112=…` (VehicleLength = wheelbase in mm) if needed (PVED-CL Operation Manual 11025583, p. 105). Send `Y` after each one, then `C`. Power-cycle the PVED.
5. Check the result with the steering diagnostics mode (`D`): estCurve should follow tan(δ)/L, and the angle in AOG should match the measured angles.
