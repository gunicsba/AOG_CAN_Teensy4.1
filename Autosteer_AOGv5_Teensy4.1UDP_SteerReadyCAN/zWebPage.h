/* zWebPage.h - the module's web page (served by zWeb.ino).
   AOG_CAN_Teensy4.1, 2026. Look, firmware update and TM171 settings form after the web page of
   renatorgr/Boards, Firmware/v4.5_AiO_WEB/zWebPage.h (written 2026 by Claude (Anthropic) for renatorgr).
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// One self-contained HTML file, no internet needed, kept in flash (PROGMEM). Dark, or light when the
// device asks for it; the header button switches and this browser remembers it.
// Tabs: Status (steering, AgOpenGPS, GPS, IMU, module), Tractor & GPS (brand and GPS mode as the serial
// service tool sets them, the Claas wheel angle mode), IMU (TM171 settings and calibrations), PVED (the Danfoss
// PVED-CL parameters: read, write, commit, the Claas hack, backup / restore, experimental valve auto-calibration,
// its other frames; zPved.ino),
// Firmware (update from a .hex file, restart), Messages (the webLog lines).
#ifndef ZWEBPAGE_H
#define ZWEBPAGE_H

const char WEB_PAGE[] PROGMEM = R"WEBPAGE(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="color-scheme" content="dark light">
<title>AOG CAN Teensy</title>
<style>
:root{color-scheme:dark;--bg:#121511;--card:#1b1f1a;--ink:#e7ebe3;--mute:#9aa395;--line:#2e352b;--ok:#66bb6a;--warn:#ffb74d;--bad:#ef5350;--acc:#8cc068;--accInk:#10140f;--soft:#232a21;--mono:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
:root[data-theme=light]{color-scheme:light;--bg:#f2f4ef;--card:#ffffff;--ink:#1c2318;--mute:#5f6b5a;--line:#d4dbcf;--ok:#2e7d32;--warn:#b45309;--bad:#c62828;--acc:#4c7d2a;--accInk:#ffffff;--soft:#e9eee5}
@media (prefers-color-scheme:light){:root:not([data-theme]){color-scheme:light;--bg:#f2f4ef;--card:#ffffff;--ink:#1c2318;--mute:#5f6b5a;--line:#d4dbcf;--ok:#2e7d32;--warn:#b45309;--bad:#c62828;--acc:#4c7d2a;--accInk:#ffffff;--soft:#e9eee5}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);accent-color:var(--acc);font:15px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
#top{position:sticky;top:0;z-index:3}
header{display:flex;align-items:center;gap:10px;padding:10px 16px;background:var(--card);border-bottom:1px solid var(--line)}
header h1{font-size:16px;margin:0;font-weight:650;white-space:nowrap}
header .sub{color:var(--mute);font-size:13px;flex:1;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.dot{width:11px;height:11px;border-radius:50%;background:var(--bad);flex:none}
.dot.on{background:var(--ok)}
nav{display:flex;gap:6px;padding:8px 12px;overflow-x:auto;background:var(--card);border-bottom:1px solid var(--line)}
nav button{border:1px solid var(--line);background:var(--card);color:var(--ink);border-radius:999px;padding:7px 14px;font:inherit;font-size:14px;cursor:pointer;white-space:nowrap}
nav button.on{background:var(--acc);border-color:var(--acc);color:var(--accInk)}
main{padding:14px 16px 40px;max-width:1100px;margin:0 auto}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:12px;margin-bottom:12px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px 14px;min-width:0}
.card h2{font-size:12.5px;text-transform:uppercase;letter-spacing:.05em;color:var(--mute);margin:0 0 8px;font-weight:650}
.card h3{font-size:14px;margin:14px 0 2px;font-weight:650}
dl{display:grid;grid-template-columns:auto 1fr;gap:3px 12px;margin:0}
dt{color:var(--mute)}
dd{margin:0;text-align:right;font-variant-numeric:tabular-nums;overflow-wrap:anywhere}
.r{display:contents}
.pill{display:inline-block;padding:1px 9px;border-radius:999px;font-size:13px;font-weight:600;background:var(--soft)}
.ok{color:var(--ok)}.warn{color:var(--warn)}.bad{color:var(--bad)}
.pill.ok{background:color-mix(in srgb,var(--ok) 16%,transparent)}
.pill.warn{background:color-mix(in srgb,var(--warn) 18%,transparent)}
.pill.bad{background:color-mix(in srgb,var(--bad) 16%,transparent)}
.mono{font-family:var(--mono);font-size:12.5px;white-space:pre-wrap;overflow-wrap:anywhere}
.note{color:var(--mute);font-size:13.5px;margin:6px 0 0}
.note.bad{color:var(--bad)}.note.warn{color:var(--warn)}.note.ok{color:var(--ok)}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin:8px 0}
button.act,label.act{border:0;background:var(--acc);color:var(--accInk);border-radius:8px;padding:9px 16px;font:inherit;font-weight:600;cursor:pointer}
button.sec{border:1px solid var(--line);background:var(--card);color:var(--ink);border-radius:8px;padding:8px 14px;font:inherit;cursor:pointer}
button.danger{border:1px solid var(--bad);background:transparent;color:var(--bad);border-radius:8px;padding:8px 14px;font:inherit;cursor:pointer}
button:disabled{opacity:.45;cursor:default}
input:disabled,select:disabled,label.opt:has(input:disabled){opacity:.5;cursor:default}
.opt{display:flex;gap:10px;align-items:flex-start;justify-content:space-between;padding:5px 0}
.opt>input[type=checkbox]{margin-top:4px}
.opt>span:first-child,.opt>input+span{flex:1}
.opt small{display:block;color:var(--mute)}
select{font:inherit;padding:6px 8px;border-radius:8px;border:1px solid var(--line);background:var(--card);color:var(--ink);max-width:60%}
input[type=number]{font:inherit;padding:6px 8px;border:1px solid var(--line);border-radius:8px;background:var(--card);color:var(--ink);width:6.5em}
.pair{display:flex;gap:6px;align-items:center;white-space:nowrap}
fieldset{border:1px solid var(--line);border-radius:10px;padding:6px 12px 8px;margin:0;min-width:0}
legend{color:var(--mute);font-size:12.5px;text-transform:uppercase;letter-spacing:.05em;padding:0 4px}
.cols{display:grid;grid-template-columns:repeat(auto-fit,minmax(290px,1fr));gap:12px;margin:8px 0}
details summary{cursor:pointer;color:var(--mute);font-size:13.5px;margin-top:12px}
progress{width:100%;height:14px}
.hidden{display:none!important}
.alert{background:color-mix(in srgb,var(--warn) 16%,transparent);border:1px solid var(--warn);border-radius:12px;padding:9px 12px;margin:0 0 12px;font-weight:600}
#log{height:66vh;overflow:auto;background:var(--card);border:1px solid var(--line);border-radius:12px;padding:8px 10px}
.ll{display:flex;gap:10px;padding:2px 0;border-bottom:1px dashed var(--line)}
.ll .t{color:var(--mute);flex:none;font-variant-numeric:tabular-nums;width:86px}
ul.note{margin:6px 0 0;padding-left:20px}
.tw{overflow-x:auto}
table{border-collapse:collapse;width:100%;font-size:14px}
th,td{padding:4px 6px;border-bottom:1px solid var(--line);text-align:left;white-space:nowrap;vertical-align:middle}
th{color:var(--mute);font-weight:600;font-size:12.5px}
td.num{text-align:right;font-variant-numeric:tabular-nums}
td input[type=number]{width:8em}
tr.chg td{background:color-mix(in srgb,var(--warn) 12%,transparent)}
tr.na td.v{color:var(--bad)}
input[type=text]{font:inherit;font-family:var(--mono);padding:6px 8px;border:1px solid var(--line);border-radius:8px;background:var(--card);color:var(--ink);min-width:16em}
.tbtn{margin-left:auto;flex:none;border:1px solid var(--line);background:var(--soft);color:var(--ink);border-radius:999px;padding:3px 11px;font:inherit;font-size:13px;font-weight:650;cursor:pointer}
@media (max-width:520px){header .sub{display:none}}
</style>
</head>
<body>
<div id="top">
<header><span class="dot" id="dot" title="link to the module"></span><h1>AOG CAN Teensy 4.1</h1><span class="sub" id="hdr">connecting…</span><button class="tbtn" id="theme" type="button" title="Light / dark">☀</button></header>
<nav id="nav"><button data-t="status" class="on">Status</button><button data-t="config">Tractor &amp; GPS</button><button data-t="imu">IMU</button><button data-t="pved">PVED</button><button data-t="firmware">Firmware</button><button data-t="log">Messages</button></nav>
</div>
<main>
<div id="alerts"></div>

<section id="t-status">
<div class="grid">
 <div class="card"><h2>Steering</h2><dl>
  <dt>Tractor</dt><dd id="sBrand">–</dd>
  <dt>Valve</dt><dd id="sValve">–</dd>
  <dt>Autosteer</dt><dd id="sEng">–</dd>
  <dt>Wheel angle</dt><dd id="sAngle">–</dd>
  <dt>Set point</dt><dd id="sSet">–</dd>
  <dt>Curve est / set</dt><dd id="sCurve">–</dd>
  <dt>PWM</dt><dd id="sPwm">–</dd>
  <dt>Steer switch / work</dt><dd id="sSw">–</dd>
  <dt>CAN engage</dt><dd id="sEngCan">–</dd>
  <div class="r hidden" id="sClaasR"><dt>Claas angle</dt><dd id="sClaas">–</dd></div>
  <div class="r hidden" id="sDiagR"><dt>Steer diag CSV</dt><dd>on (USB serial)</dd></div>
 </dl></div>
 <div class="card"><h2>AgOpenGPS</h2><dl>
  <dt>Connection</dt><dd id="aSeen">–</dd>
  <dt>Speed</dt><dd id="aKmh">–</dd>
  <dt>Guidance</dt><dd id="aGuid">–</dd>
 </dl></div>
 <div class="card"><h2>GPS</h2><dl>
  <dt>Mode</dt><dd id="gMode">–</dd>
  <dt>Receiver data</dt><dd id="gBps">–</dd>
  <dt>NTRIP from AgIO</dt><dd id="gNtrip">–</dd>
  <dt>Radio (Serial7)</dt><dd id="gRadio">–</dd>
  <dt>Fix</dt><dd id="gFix">–</dd>
  <div class="r" id="gSatsR"><dt>Satellites / HDOP</dt><dd id="gSats">–</dd></div>
  <div class="r" id="gAgeR"><dt>Correction age</dt><dd id="gAge">–</dd></div>
 </dl></div>
 <div class="card"><h2>IMU</h2><dl>
  <dt>Type</dt><dd id="iType">–</dd>
  <div class="r" id="iRPR"><dt>Roll / pitch</dt><dd id="iRP">–</dd></div>
  <div class="r" id="iYawR"><dt>Heading</dt><dd id="iYaw">–</dd></div>
  <div class="r" id="iRateR"><dt>Output rate</dt><dd id="iRate">–</dd></div>
 </dl></div>
 <div class="card"><h2>Module</h2><dl>
  <dt>Firmware</dt><dd id="mFw">–</dd>
  <dt>Built</dt><dd id="mBuilt">–</dd>
  <dt>Running for</dt><dd id="mUp">–</dd>
  <dt>IP / link</dt><dd id="mIp">–</dd>
  <dt>CPU temperature</dt><dd id="mCpu">–</dd>
  <dt>Serial ports</dt><dd id="mPorts">–</dd>
 </dl></div>
</div>
</section>

<section id="t-config" class="hidden">
<div class="grid">
 <div class="card"><h2>Tractor</h2>
  <label class="opt"><span>Brand<small>Which CAN messages the Teensy uses to steer.</small></span><select id="cBrand">
   <option value="0">0 – Claas</option><option value="1">1 – Valtra / Massey / McCormick / Deutz</option><option value="2">2 – CaseIH / New Holland</option>
   <option value="3">3 – Fendt SCR, S4, Gen6</option><option value="4">4 – JCB</option><option value="5">5 – FendtOne</option><option value="6">6 – Lindner</option>
   <option value="7">7 – AgOpenGPS (CAN / PWM module)</option><option value="8">8 – Cat MT (late)</option><option value="9">9 – Cat MT (early)</option></select></label>
  <p class="note" id="cBrandNote"></p>
 </div>
 <div class="card"><h2>GPS receiver</h2>
  <label class="opt"><span>Mode<small>Forwarding: the receiver's NMEA goes to AgIO unchanged. Panda: the Teensy reads GGA / VTG and sends $PANDA with the IMU roll and heading.</small></span><select id="cOut"><option value="f">Forwarding</option><option value="p">Panda</option></select></label>
  <label class="opt"><span>Baud rate<small>Must match the receiver's UART.</small></span><select id="cBaud"><option value="115200">115200</option><option value="460800">460800</option></select></label>
 </div>
</div>
<div class="card" style="margin-bottom:12px"><h2>Board: serial ports</h2>
 <div class="cols">
  <label class="opt"><span>GNSS receiver</span><select id="pGnss"><option value="3">Serial3 (pins 14/15)</option><option value="5">Serial5 (pins 20/21)</option><option value="6">Serial6 (pins 24/25)</option><option value="7">Serial7 (pins 28/29)</option><option value="8">Serial8 (pins 34/35)</option></select></label>
  <label class="opt"><span>IMU (TM171)<small>A BNO08x is on I2C: no port.</small></span><select id="pImu"><option value="0">none</option><option value="3">Serial3 (pins 14/15)</option><option value="5">Serial5 (pins 20/21)</option><option value="6">Serial6 (pins 24/25)</option><option value="7">Serial7 (pins 28/29)</option><option value="8">Serial8 (pins 34/35)</option></select></label>
  <label class="opt"><span>RTK radio<small>Its corrections go to the GNSS receiver.</small></span><select id="pRadio"><option value="0">none</option><option value="3">Serial3 (pins 14/15)</option><option value="5">Serial5 (pins 20/21)</option><option value="6">Serial6 (pins 24/25)</option><option value="7">Serial7 (pins 28/29)</option><option value="8">Serial8 (pins 34/35)</option></select></label>
 </div>
 <div class="row"><button class="sec" type="button" id="pTony">This board: GNSS 3, IMU 5, radio 7</button><button class="sec" type="button" id="pAio">AiO v4.x: GNSS 7, IMU 5, radio 3</button></div>
 <p class="note">Serial1 (CAN2, ISO-Bus), Serial2 (work switch and remote inputs) and Serial4 (engage LED) are used by this firmware and not offered. The AiO's GPS2 port (Serial2) is not used here.</p>
</div>
<div class="row"><button class="act" id="cSave" data-lock="e">Save and restart</button><button class="sec" id="cUndo">Undo changes</button><span id="cMsg" class="note"></span></div>
<p class="note">The same settings as the serial service tool (S, then 0 – 9, f / F / p / P). They are kept in the EEPROM; the Teensy restarts to take them.</p>
<div class="card hidden" id="cClaas" style="margin-top:12px"><h2>Claas wheel angle</h2>
 <label class="opt"><input type="checkbox" id="cAtan"><span>atan mode: angle = atan(L × curvature)<small>Only once the PVED WAS calibration is real (65080 / 65083 / 65086, 65099 / 65100, 65112). Off (default): angle = (estCurve − 32128 + Zero WAS) / AgOpenGPS counts per degree. Service tool V&lt;mm&gt; / V0.</small></span></label>
 <label class="opt"><span>Vehicle length L<small>Must match PVED parameter 65112.</small></span><span class="pair"><input type="number" id="cLen" min="500" max="10000" step="1"> mm</span></label>
 <p class="note" id="cCpd"></p>
 <div class="row"><button class="act" id="cClaasSave">Save</button><span id="cClaasMsg" class="note"></span></div>
</div>
</section>

<section id="t-imu" class="hidden">
<div class="card hidden" id="imuNone"><h2>IMU</h2><p class="note" id="imuNoneTxt"></p></div>
<div id="imuTm" class="hidden">
<div class="grid">
 <div class="card"><h2>TM171</h2><dl>
  <dt>Roll / pitch</dt><dd id="tRP">–</dd>
  <dt>Heading (yaw)</dt><dd id="tYaw">–</dd>
  <dt>Output rate</dt><dd id="tRate">–</dd>
  <dt>Node ID</dt><dd id="tId">–</dd>
  <dt>Bad packets</dt><dd id="tCrc">–</dd>
  <div class="r hidden" id="tTempR"><dt>Temperature / QoS</dt><dd id="tTemp">–</dd></div>
  <dt>AgOpenGPS set-up</dt><dd id="tSetup">–</dd>
 </dl>
 <p class="note">As the TM171 sends them. Which one AgOpenGPS gets as roll follows "Use Y axis" in its IMU settings (now: <span id="tAxis">–</span>).</p></div>
 <div class="card"><h2>Calibration</h2>
  <p id="tcMsg" class="note"></p>
  <h3>Gyroscope</h3>
  <div class="row"><button class="sec" type="button" data-cal="gyro" data-lock="m">Gyro bias calibration (20 s)</button><button class="danger hidden" type="button" id="tcStop">Stop now</button></div>
  <p class="note">The TM171 measures its gyro bias while the tractor stands still (the engine can run); the firmware finishes it after 20 s. Kept until the TM171 is powered off. TM171 firmware V5.1 or later.</p>
  <div class="row"><button class="sec" type="button" data-cal="gyroAuto" data-lock="m">Forced static calibration</button><button class="sec" type="button" data-cal="gyroRevoke" data-lock="e">Revoke</button></div>
  <p class="note">Forced static: the TM171 decides when it has enough data. Revoke: back to the values before the forced calibration.</p>
  <label class="opt"><span>Accepted gyro error<small>Larger errors are not compensated: the calibration fails.</small></span><span class="pair"><input type="number" id="tcLimit" min="0.05" max="5" step="0.05" value="0.50"> °/s</span></label>
  <h3>Boot calibration (new install, CalibB)</h3>
  <div class="row"><button class="sec" type="button" data-cal="install" data-lock="m">Boot calibration</button></div>
  <p class="note">Saved in the TM171's flash. Once after installing it, or after a large temperature change. About 20 s with the tractor parked; it cannot be stopped. Do not cut the power while it runs (SYD: this can damage the TM171). Then switch the whole module off and on.</p>
  <details><summary>Magnetometer and heading</summary>
   <p class="note">Only useful with Enable Mag on (AgOpenGPS normally runs with it off).</p>
   <h3>Hard-iron calibration</h3>
   <div class="row"><button class="sec" type="button" data-cal="mag" data-lock="e">Start (save to flash)</button><button class="sec" type="button" data-cal="magTmp" data-lock="e">Start (this power-on)</button></div>
   <p class="note">Then turn the module through every direction (drive slow circles) until it reports done.</p>
   <div class="row"><button class="sec" type="button" data-cal="magClear">Clear (flash)</button><button class="sec" type="button" data-cal="magClearTmp">Clear (this power-on)</button></div>
   <h3>Heading to magnetic north</h3>
   <div class="row"><span class="pair">Collect for <input type="number" id="tcHeadT" min="0" max="90" step="1" value="8"> s</span><button class="sec" type="button" data-cal="headStart" data-lock="e">Start</button><button class="sec" type="button" data-cal="headStop">Stop and apply</button></div>
   <p class="note">0 s: until Stop. Keep roll and pitch near zero and turn around the vertical axis while it collects (3 – 8 s). This power-on only.</p>
   <h3>Set heading</h3>
   <div class="row"><span class="pair"><input type="number" id="tcHead" min="0" max="360" step="0.1" value="0"> °</span><button class="sec" type="button" data-cal="headSet" data-lock="e">Set heading</button></div>
  </details>
 </div>
</div>
<div class="card"><h2>TM171 settings</h2>
 <p id="tsMsg" class="note"></p>
 <div class="cols" id="tsForm">
  <fieldset><legend>Sensors</legend>
   <label class="opt"><input type="checkbox" id="tsGyro"><span>Enable Gyro</span></label>
   <label class="opt"><input type="checkbox" id="tsAcc"><span>Enable Accel</span></label>
   <label class="opt"><input type="checkbox" id="tsMag"><span>Enable Mag<small>Off for AgOpenGPS: the tractor's steel and electronics disturb it.</small></span></label>
  </fieldset>
  <fieldset><legend>Sensor fusion</legend>
   <label class="opt"><input type="checkbox" id="tsGyroErr"><span>GyroErrFilter<small>Compensates the gyro error while running. Keep the accelerometer gain at 2.00 or more with it (2.50 or more with strong vibration).</small></span></label>
   <label class="opt"><span>Accelerometer gain</span><input type="number" id="tsAccG" min="0.01" max="20" step="0.01"></label>
   <label class="opt"><input type="checkbox" id="tsSelf"><span>Self-Adapt filter<small>Compensates magnetic disturbances for the magnetometer heading.</small></span></label>
   <label class="opt"><span>Magnetometer gain</span><input type="number" id="tsMagG" min="0.01" max="20" step="0.01"></label>
  </fieldset>
  <fieldset><legend>Communication</legend>
   <label class="opt"><span>Power-on silent time<small>Seconds after power-on without data (requests are still answered). Keep 0: the Teensy looks for the TM171 about 3 s after power-on and runs without IMU if it is silent then.</small></span><span class="pair"><input type="number" id="tsSilent" min="0" max="65" step="1"> s</span></label>
   <label class="opt" id="tsOdrR"><span>Output data rate</span><select id="tsOdr"><option value="25">25 Hz</option><option value="50">50 Hz</option><option value="100">100 Hz</option></select></label>
   <label class="opt"><input type="checkbox" id="tsStatus"><span>Status output<small>Temperature and QoS.</small></span></label>
   <label class="opt"><span>Boot mode<small>Takes effect after a power cycle, and only when saved to flash.</small></span><select id="tsBoot"><option value="0">Auto</option><option value="1">Static</option><option value="2">Dynamic</option></select></label>
   <dl><dt>UART</dt><dd id="tsBaud">–</dd><dt>Inhibit time</dt><dd id="tsInhibit">–</dd><dt>Roll-pitch-yaw output</dt><dd>always on</dd></dl>
  </fieldset>
 </div>
 <label class="opt"><input type="checkbox" id="tsSave" checked><span>Save to the TM171's flash<small>Off: only until the TM171 is powered off (safer to try settings). Keep the power steady while saving.</small></span></label>
 <div class="row"><button class="sec" type="button" id="tsRead">Read from TM171</button><button class="sec" type="button" id="tsAog">AgOpenGPS defaults</button><button class="act" type="button" id="tsWrite" data-lock="e">Write to TM171</button></div>
 <p class="note mono" id="tsRaw"></p>
</div>
</div>
</section>

<section id="t-pved" class="hidden">
<div class="grid">
 <div class="card"><h2>PVED session</h2><dl>
  <dt>Session</dt><dd id="pvOn">–</dd>
  <dt>Steering commands to the valve</dt><dd id="pvQuietTxt">–</dd>
  <dt>Valve state / estCurve</dt><dd id="pvValve">–</dd>
  <dt>PVED woke up in</dt><dd id="pvMode">–</dd>
  <dt>Requests</dt><dd id="pvQueue">–</dd>
 </dl>
 <label class="opt"><input type="checkbox" id="pvQuiet" checked><span>Pause the steering commands to the valve<small>As the serial tool (Z) did: the PVED gets no set points while the session is open.</small></span></label>
 <div class="row"><button class="act" type="button" id="pvStart" data-lock="e">Open session</button><button class="sec" type="button" id="pvStop">Close session</button></div>
 <p class="note">While the session is open autosteer is blocked. It closes by itself a minute after the last page is closed. If the PVED does not answer, switch the PVED (steering) power off and on while the session is open.</p>
 <p id="pvMsg" class="note"></p>
 </div>
 <div class="card"><h2>Commit and backup</h2>
  <p class="note">The PVED keeps written values only after a commit. After the commit wait 1 minute, then switch the PVED off and on.</p>
  <div class="row"><button class="danger" type="button" id="pvCommit" data-pv="w">Commit to PVED memory</button></div>
  <p id="pvCommitMsg" class="note"></p>
  <h3>Backup</h3>
  <div class="row"><button class="sec" type="button" id="pvBackup">Download backup (.csv)</button><button class="sec" type="button" id="pvRestore" data-pv="w">Restore originals</button></div>
  <p class="note">Original = the first value read since the Teensy started. Download the backup before writing anything. Restore writes the originals back to every parameter that differs now (then Commit).</p>
 </div>
 <div class="card hidden" id="pvHackCard"><h2>Claas hack</h2>
  <p class="note">The serial tool's W: the PVED takes its set points from this module (address 30 = 0x1E instead of 27 = 0x1B), SASA present 255, no fast steer flow. Then Commit, wait 1 minute and switch the PVED off and on.</p>
  <dl><dt>64007 set point address</dt><dd id="pvH1">–</dd><dt>65101 SASA present</dt><dd id="pvH2">–</dd><dt>1027 fast steer flow</dt><dd id="pvH3">–</dd></dl>
  <div class="row"><button class="act" type="button" id="pvHack" data-pv="w">Write 64007 = 30, 65101 = 255, 1027 = 0</button></div>
  <p class="note warn">The operator is responsible for switching the ATP off when autosteer is not used. It most likely disables the dynamic (variable ratio) steering.</p>
 </div>
</div>
<div class="card"><h2>Parameters</h2>
 <div class="row"><button class="act" type="button" id="pvReadAll" data-pv="r">Read all</button><span class="pair">Any parameter <input type="number" id="pvAnyN" min="1" max="65535" step="1"></span><button class="sec" type="button" id="pvAnyRead" data-pv="r">Read</button><span id="pvAny" class="note"></span></div>
 <div class="tw"><table id="pvTab"><thead><tr><th>No.</th><th>Name</th><th>Value</th><th>Bytes</th><th>Original</th><th>Read</th><th>New value</th><th></th></tr></thead><tbody></tbody></table></div>
 <p class="note">⚠ = valve / cylinder calibration or sensor mapping: only change it if you know the right value. 65104 says where the wheel angle comes from (4 = CAN). Values differing from the original are marked. Every write is read back; nothing is written unless the parameter was read first.</p>
</div>
<div class="card" id="pvCalCard"><h2>Valve auto-calibration (experimental)</h2>
 <p class="note warn">The PVED moves the steered wheels by itself. Keep everybody away from the wheels and do not touch the steering wheel: that aborts the calibration.</p>
 <p class="note">Danfoss PVED-CL Operation Manual 11025583, "Method 3: Valve auto-calibration": the PVED steps the main spool from XspStartSearch until the wheel angle sensor sees the wheels move (a change above YactDiffThreshold within TimeOutPeriod), first right, then left. It then writes Xspl_0 / Xspr_0 = that point minus XspCalibrationOffset (758), shifts Xspl_1000 / Xspr_1000 by the same amount and commits them itself. Switch the PVED off and on afterwards: it stays in calibration mode until then, and the new dead-bands only work after it. The CAN frames come from a third-party PVED-CL diagnostics tool, not from Danfoss' protocol document, so this is experimental.</p>
 <details><summary>Before you start (the Danfoss preconditions)</summary>
  <ul class="note">
   <li>Wheel angle sensor installed, mapped (65104) and calibrated (65080 / 65086 / 65083). The auto-calibration measures the wheel movement with it.</li>
   <li>Steering wheel sensor (SASA) installed and mapped (65101 = 255).</li>
   <li>Wheels straight ahead (within 25 % of the maximum angle), on an even, solid surface such as concrete or asphalt, no front loader or front weight, tractor standing still.</li>
   <li>Engine running, oil at working temperature, steering bled. Steering pressure must stay below 80 % of the relief valve setting.</li>
   <li>Read all parameters and download the backup first. "Restore originals" then Commit puts the old dead-bands back.</li>
  </ul>
 </details>
 <dl><dt>PVED mode</dt><dd id="pvCalMode">–</dd><dt>Auto-calibration</dt><dd id="pvCalAuto">–</dd><dt>Last status (0F B7)</dt><dd id="pvCalSt" class="mono">–</dd></dl>
 <div class="row"><button class="sec" type="button" id="pvGetMode" data-pv="r">Ask mode</button><button class="act" type="button" id="pvEnterCal" data-pv="w">1. Enter calibration mode</button><span id="pvEnterTxt" class="note"></span></div>
 <p class="note">Sends 0F A8 58 every 0.2 s for up to 20 s until the PVED reports calibration mode. If the mode does not change, switch the PVED off and on while it is being asked.</p>
 <label class="opt"><span>Valve type<small>From ValveType (65121) once read. Starting values from the manual's table; the best values are found by testing.</small></span><select id="pvPreset"><option value="ehd">EH, dynamic spool</option><option value="ehs">EH, static spool</option><option value="osp">OSPEH</option><option value="ehps">EHPS</option></select></label>
 <div class="cols">
  <label class="opt"><span>XspStartSearch<small>Spool set-point where the search starts.</small></span><input type="number" id="pvXs" min="0" max="1000" step="1"></label>
  <label class="opt"><span>XspIncrementSize<small>Step per TimeOutPeriod (resolution).</small></span><input type="number" id="pvInc" min="1" max="100" step="1"></label>
  <label class="opt"><span>YactDiffThreshold<small>Wheel movement that counts as flow.</small></span><input type="number" id="pvYt" min="1" max="255" step="1"></label>
  <label class="opt"><span>TimeOutPeriod<small>ms per step (500 or 1000).</small></span><input type="number" id="pvTms" min="50" max="5000" step="50"></label>
 </div>
 <label class="opt"><span>Send the start command from<small>Danfoss: "shall be sent from the MMI controller" (its address is 64005, default 252 = 0xFC). The diagnostics tool sends it from the service tool address 0xFD. Which one the PVED takes is not known yet: if nothing happens with one, try the other.</small></span><select id="pvFrom"><option value="mmi">MMI address (64005)</option><option value="tool">Service tool 0xFD</option></select></label>
 <div class="row"><button class="danger" type="button" id="pvAutoCal" data-pv="w">2. Start auto-calibration</button><button class="sec" type="button" id="pvReadRes" data-pv="r">3. Read the results</button></div>
 <p class="note">The new Xspl_0 / Xspr_0 / Xspl_1000 / Xspr_1000 show up marked in the parameter list (compared with the originals).</p>
 <h3>Live AD values (status set 1)</h3>
 <div class="row"><button class="sec" type="button" id="pvSet1" data-pv="r">Start status set 1</button><button class="sec" type="button" id="pvSet0" data-pv="r">Stop (set 0)</button></div>
 <p class="note mono" id="pvSet1Txt"></p>
 <p class="note">AD1 = wheel angle sensor, AD2, AD3 = spool position, Xsp = spool set-point. The status data are the PVED's broadcast PGN 0xFF02, read as four 16-bit values: a guess. Its raw bytes are in the list below. Danfoss: the WAS noise should stay within ±4 counts.</p>
</div>
<div class="card"><h2>Other frames from the PVED</h2>
 <div class="tw"><table id="pvFr"><thead><tr><th>ID</th><th>Bytes</th><th>Count</th><th>Last</th></tr></thead><tbody></tbody></table></div>
 <div class="row"><input type="text" id="pvRawHex" placeholder="0F AB 00 00 00 00 00 00"><button class="sec" type="button" id="pvRawSend" data-pv="w">Send to the PVED</button></div>
 <p class="note">8 bytes to 0x18EF13FD. Refused here: write (0F A2), RestoreDefaults (0F A4, every parameter back to factory values), commit (0F A6), EnterCalibrationMode (0F A8) and StartValveAutoCalibration (0F B6).</p>
</div>
</section>

<section id="t-firmware" class="hidden">
<div class="grid">
 <div class="card"><h2>Firmware update</h2>
  <p class="note">The .hex file from the Arduino IDE (Sketch → Export Compiled Binary). The Teensy checks the whole file before it replaces the running firmware; the settings in the EEPROM stay. Autosteer must be off and the tractor stopped.</p>
  <div class="row"><label class="act">Choose .hex<input type="file" id="hexFile" accept=".hex" class="hidden"></label><span id="hexName" class="note">no file</span></div>
  <p id="hexInfo" class="note hidden"></p>
  <label class="opt"><input type="checkbox" id="force"><span>Other firmware<small>Install it even without this web page inside (the next update will then have to be by USB).</small></span></label>
  <div class="row"><button class="act" id="install" data-lock="m" data-off="1" disabled>Install</button></div>
  <progress id="prog" max="100" value="0" class="hidden"></progress>
  <p id="fwMsg" class="note"></p>
 </div>
 <div class="card"><h2>Restart</h2>
  <p class="note">Restarts the Teensy (a few seconds). Steering is switched off first.</p>
  <div class="row"><button class="danger" id="reboot" data-lock="e">Restart Teensy</button></div>
  <dl><dt>Running</dt><dd id="fRun">–</dd><dt>Web requests / errors / dropped</dt><dd id="fWeb">–</dd></dl>
 </div>
</div>
</section>

<section id="t-log" class="hidden"><div id="log"></div></section>
</main>
<script>
"use strict";
const $=id=>document.getElementById(id);
function set(id,t,c){const e=$(id);if(!e)return;e.textContent=t;if(c!==undefined)e.className=c;}
function pill(id,t,c){const e=$(id);if(!e)return;e.textContent="";const s=document.createElement("span");s.className="pill "+(c||"");s.textContent=t;e.appendChild(s);}
function vis(id,on){const e=$(id);if(e)e.classList.toggle("hidden",!on);}
async function getJ(u,o){const r=await fetch(u,Object.assign({cache:"no-store"},o||{}));return r.json();}
function post(u,b){return getJ(u,{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:b});}
function fx(v,d){return (v===undefined||v===null)?"–":Number(v).toFixed(d);}
function dur(s){const d=Math.floor(s/86400),h=Math.floor(s%86400/3600),m=Math.floor(s%3600/60),x=s%60;return (d?d+"d ":"")+(d||h?h+"h ":"")+(d||h||m?m+"m ":"")+x+"s";}
function ago(s){return s<0?"never":s<90?s+" s ago":s<5400?Math.round(s/60)+" min ago":Math.round(s/3600)+" h ago";}

// ---- theme
(function(){let t=null;try{t=localStorage.getItem("theme");}catch(e){}if(t)document.documentElement.dataset.theme=t;
 $("theme").addEventListener("click",()=>{const cur=document.documentElement.dataset.theme||(matchMedia("(prefers-color-scheme: light)").matches?"light":"dark");
  const n=cur==="light"?"dark":"light";document.documentElement.dataset.theme=n;try{localStorage.setItem("theme",n);}catch(e){}});})();

// ---- tabs
let tab="status";
function show(t){tab=t;for(const b of $("nav").querySelectorAll("button"))b.classList.toggle("on",b.dataset.t===t);
 for(const s of document.querySelectorAll("main>section"))s.classList.toggle("hidden",s.id!=="t-"+t);
 try{sessionStorage.setItem("tab",t);}catch(e){}if(t==="log")$("log").scrollTop=$("log").scrollHeight;if(t==="pved"&&typeof pollPved==="function")pollPved();}
$("nav").addEventListener("click",e=>{const b=e.target.closest("button");if(b)show(b.dataset.t);});
try{const h=location.hash.slice(1),t=$("t-"+h)?h:sessionStorage.getItem("tab");if(t&&$("t-"+t))show(t);}catch(e){}   /* #imu etc. opens that tab */

// ---- status
const FIX={"0":"no fix","1":"GPS","2":"DGPS","3":"PPS","4":"RTK fixed","5":"RTK float","6":"dead reckoning","7":"manual","8":"simulation"};
let lastS=null,waitingBoot=false,busy=false;
function setOnline(on){$("dot").classList.toggle("on",on);if(!on)set("hdr","no connection to the module");}
function renderAlerts(list){const box=$("alerts");box.textContent="";for(const t of list){const d=document.createElement("div");d.className="alert";d.textContent=t;box.appendChild(d);}}
// buttons the module refuses now: data-lock="e" while autosteer is engaged, "m" also while the tractor moves
function locks(s){$("install").dataset.off=($("hexFile").files[0]&&!busy)?"":"1";
 for(const b of document.querySelectorAll("[data-lock]")){const l=b.dataset.lock==="e"?s.engaged:(s.engaged||s.moving);b.dataset.locked=l?"1":"";b.disabled=l||b.dataset.off==="1";}}
function render(s){
 lastS=s;
 set("hdr",s.steer.brandName+" · "+s.gps.modeName+" · "+s.ip);
 const al=[];
 if(s.engaged)al.push("Autosteer is engaged: brand / GPS changes, TM171 writes and calibration, restart and firmware update are locked.");
 else if(s.moving)al.push("The tractor is moving ("+fx(s.kmh,1)+" km/h): the firmware update and the gyro / new install calibrations are locked.");
 if(s.pved&&s.pved.on)al.push("PVED session open: autosteer is blocked"+(s.pved.quiet?" and the steering commands to the valve are paused":"")+". Close it on the PVED tab.");
 if(s.imu.set&&s.imu.set.noWire)al.push("TM171 does not answer: wire Teensy TX"+s.ports.imu+" (pin "+s.ports.imuTx+") to the TM171 RXD to use its settings and calibration. Roll and heading still work.");
 renderAlerts(al);
 const st=s.steer;
 set("sBrand",st.brand+" – "+st.brandName);
 pill("sValve",(st.ready?"ready":"not ready")+" ("+st.valve+")",st.ready?"ok":"warn");
 pill("sEng",st.intend?"steering":"off",st.intend?"ok":"");
 set("sAngle",fx(st.angle,2)+"°");set("sSet",fx(s.aog.setAngle,2)+"°");
 set("sCurve",st.estCurve+" / "+st.setCurve);set("sPwm",String(st.pwm));
 set("sSw",(st.steerSw?"on":"off")+" / "+(st.work?"on":"off"));set("sEngCan",st.engageCAN?"pressed":"–");
 vis("sClaasR",st.brand===0);set("sClaas",st.claasLen?"atan, L = "+st.claasLen+" mm":"linear (counts per degree)");
 vis("sDiagR",st.diag);
 pill("aSeen",s.aog.seen?"connected":(s.aog.ago<0?"not heard yet":"silent, last "+ago(s.aog.ago)),s.aog.seen?"ok":"bad");
 set("aKmh",fx(s.aog.kmh,1)+" km/h");set("aGuid",(s.aog.guidance&1)?"on":"off");
 const g=s.gps;set("gMode",g.modeName);
 pill("gBps",g.bps?g.bps+" B/s":"no data",g.bps?"ok":"bad");
 set("gNtrip",g.ntripBps?g.ntripBps+" B/s":"–");set("gRadio",g.radioBps?g.radioBps+" B/s":"–");
 if(g.gga){pill("gFix",FIX[g.fix]||g.fix,g.fix==="4"?"ok":g.fix==="0"?"bad":"warn");set("gSats",g.sats+" / "+g.hdop);set("gAge",g.age?g.age+" s":"–");}
 else set("gFix",g.mode>=3?"no GGA from the receiver":"read by AgIO (forwarding mode)");
 vis("gSatsR",g.gga);vis("gAgeR",g.gga);
 const i=s.imu;set("iType",i.type||"none found at start-up");
 vis("iRPR",!!i.type);vis("iYawR",!!i.type);vis("iRateR",i.type==="TM171");
 if(i.type){set("iRP",fx(i.roll,2)+"° / "+fx(i.pitch,2)+"°");set("iYaw",fx(i.yaw,1)+"°");}
 if(i.type==="TM171")pill("iRate",i.rate+" Hz",i.fresh&&i.rate?"ok":"bad");
 set("mFw",s.ver);set("mBuilt",s.built);set("mUp",dur(s.up));
 set("mIp",s.ip+" / "+(s.link?"link up":"no link"));if(s.ports){const pn=v=>v?"Serial"+v:"none";set("mPorts","GNSS "+pn(s.ports.gnss)+" · IMU "+pn(s.ports.imu)+" · radio "+pn(s.ports.radio));}set("mCpu",fx(s.cpuC,1)+" °C");
 set("fRun",s.ver+", built "+s.built);set("fWeb",s.web.req+" / "+s.web.err+" / "+s.web.drop);
 renderImu(s);
 if(!cfgDirty&&cfg&&(cfg.brand!==st.brand))loadCfg();
 brandUi();
 locks(s);
}
function reloadAfterRestart(msg){try{sessionStorage.setItem("tab",tab);if(msg)sessionStorage.setItem("fwMsg",msg);}catch(e){}location.reload();}
async function pollStatus(){
 try{const s=await getJ("/api/status");
  if(lastS&&(s.up+3<lastS.up||s.built!==lastS.built)){reloadAfterRestart(waitingBoot?"Updated. Running: "+s.ver+", built "+s.built+".":"");return;}
  setOnline(true);render(s);
 }catch(e){setOnline(false);}
}

// ---- Tractor & GPS (the serial service tool's settings)
const BRAND_NOTE={0:"Claas: the PVED parameter 64007 (set point controller address) must be 30, set with the serial service tool (S, then Z, W, C). Engage with the steer button or from the tablet.",
 1:"Standard Danfoss ISO messages (navigation controller 0x1C).",3:"Needs the 3rd party steering unlock (ACP0595080) on the tractor.",5:"FendtOne: K-Bus at 500 kbit/s.",
 7:"For the AgOpenGPS remote CAN / PWM steering module.",8:"Cat MT (late models).",9:"Cat MT (early models)."};
let cfg=null,cfgDirty=false;
function gpsModeOf(){return ($("cOut").value==="p"?3:1)+($("cBaud").value==="460800"?1:0);}
const PORT_IDS=["pGnss","pImu","pRadio"],PORT_KEYS=["gnssPort","imuPort","radioPort"];
function portsOf(){return PORT_IDS.map(id=>+$(id).value);}
function portsChanged(){return !!cfg&&portsOf().some((v,i)=>v!==cfg[PORT_KEYS[i]]);}
function portsSet(g,i,r){$("pGnss").value=String(g);$("pImu").value=String(i);$("pRadio").value=String(r);cfgDirty=true;brandUi();set("cMsg","");}
function fillCfg(j){$("cBrand").value=String(j.brand);const m=j.gpsMode||1;$("cOut").value=m>=3?"p":"f";$("cBaud").value=(m===2||m===4)?"460800":"115200";
 if(j.gnssPort!==undefined){$("pGnss").value=String(j.gnssPort);$("pImu").value=String(j.imuPort);$("pRadio").value=String(j.radioPort);}
 $("cAtan").checked=j.claasLen>0;$("cLen").value=j.claasLen||2820;cfgDirty=false;claasUi();brandUi();}
async function loadCfg(){try{const j=await getJ("/api/config");if(j.ok){cfg=j;if(!cfgDirty)fillCfg(j);}}catch(e){}}
function brandUi(){const b=+$("cBrand").value;set("cBrandNote",BRAND_NOTE[b]||"");
 vis("cClaas",cfg?cfg.brand===0:false);
 const ch=cfg&&(b!==cfg.brand||gpsModeOf()!==cfg.gpsMode||portsChanged());$("cUndo").disabled=!ch;
 $("cSave").dataset.off=ch?"":"1";$("cSave").disabled=!ch||$("cSave").dataset.locked==="1";}
for(const id of ["cBrand","cOut","cBaud",...PORT_IDS])$(id).addEventListener("change",()=>{cfgDirty=true;brandUi();set("cMsg","");});
$("pTony").addEventListener("click",()=>portsSet(3,5,7));
$("pAio").addEventListener("click",()=>portsSet(7,5,3));
$("cUndo").addEventListener("click",()=>{if(cfg)fillCfg(cfg);set("cMsg","");});
$("cSave").addEventListener("click",async()=>{
 const b=+$("cBrand").value,m=gpsModeOf(),[pg,pi,pr]=portsOf();
 if(pg===pi||pg===pr||(pi&&pi===pr)){set("cMsg","Each device needs its own serial port.","note bad");return;}
 const pn=v=>v?"Serial"+v:"none";
 if(!confirm("Save tractor brand "+$("cBrand").selectedOptions[0].textContent+" and GPS "+$("cOut").selectedOptions[0].textContent+" @ "+$("cBaud").value+"?\n\nSerial ports: GNSS "+pn(pg)+", IMU "+pn(pi)+", RTK radio "+pn(pr)+"\n\nThe Teensy restarts to take them (a few seconds)."))return;
 try{const j=await post("/api/config","brand="+b+"&gpsMode="+m+"&gnssPort="+pg+"&imuPort="+pi+"&radioPort="+pr);
  if(!j.ok){set("cMsg","Not saved: "+j.error,"note bad");return;}
  set("cMsg",j.restart?"Saved. The Teensy is restarting…":"Nothing changed.","note ok");if(j.restart)waitingBoot=true;
 }catch(e){set("cMsg","Cannot reach the module.","note bad");}});
function claasUi(){const L=+$("cLen").value,on=$("cAtan").checked;$("cLen").disabled=!on;
 set("cCpd",on&&L>=500?"In atan mode the AgOpenGPS counts per degree is not used, but Zero WAS is: set counts per degree to "+Math.round(1/(L*0.001*0.00025)/57.29578)+" so Zero WAS works. One count is then about 1/23° instead of 1/CPD, so the Proportional Gain may need a change.":"");}
$("cAtan").addEventListener("change",claasUi);$("cLen").addEventListener("input",claasUi);
$("cClaasSave").addEventListener("click",async()=>{
 const on=$("cAtan").checked,L=Math.round(+$("cLen").value);
 if(on&&!(L>=500&&L<=10000)){set("cClaasMsg","Vehicle length: 500 – 10000 mm","note bad");return;}
 try{const j=await post("/api/config","claasLen="+(on?L:0));
  if(!j.ok){set("cClaasMsg","Not saved: "+j.error,"note bad");return;}
  set("cClaasMsg","Saved: "+(on?"atan mode, L = "+L+" mm":"linear")+".","note ok");loadCfg();
 }catch(e){set("cClaasMsg","Cannot reach the module.","note bad");}});

// ---- IMU: TM171 settings and calibration (TM171.ino)
let tsSeen=null,tsDirty=false,tsErrUntil=0,calErrUntil=0;
function tsSay(t,c){set("tsMsg",t,"note "+(c||""));tsErrUntil=Date.now()+6000;}
function calSay(t,c){set("tcMsg",t,"note "+(c||""));calErrUntil=Date.now()+6000;}
function odrSet(v){vis("tsOdrR",!!v);if(!v)return;const sel=$("tsOdr");
 if(![...sel.options].some(o=>+o.value===v)){const o=document.createElement("option");o.value=v;o.textContent=v+" Hz (as read)";sel.appendChild(o);}sel.value=String(v);}
function tsFill(s){$("tsGyro").checked=!!s.gyro;$("tsAcc").checked=!!s.acc;$("tsMag").checked=!!s.mag;$("tsGyroErr").checked=!!s.gyroErr;$("tsSelf").checked=!!s.selfAdapt;
 $("tsStatus").checked=!!s.status;$("tsBoot").value=String(s.boot||0);$("tsAccG").value=(s.accGain/100).toFixed(2);$("tsMagG").value=(s.magGain/100).toFixed(2);
 $("tsSilent").value=s.silent||0;if(s.odr!==undefined)odrSet(s.odr);tsDirty=false;}
$("tsForm").addEventListener("input",()=>{tsDirty=true;});
function renderImu(s){
 const i=s.imu,tm=i.type==="TM171";vis("imuTm",tm);vis("imuNone",!tm);
 if(!tm){set("imuNoneTxt",i.type==="BNO08x"?"BNO08x on I2C: nothing to set up here. The settings and calibrations on this tab are for the TM171.":"No IMU was found at start-up. The TM171 ("+(s.ports.imu?"Serial"+s.ports.imu:"no port set")+", 115200) or the BNO08x (I2C 0x4A / 0x4B) is looked for once, when the Teensy starts. The port is set in Tractor & GPS.");return;}
 set("tRP",fx(i.roll,2)+"° / "+fx(i.pitch,2)+"°");set("tYaw",fx(i.yaw,2)+"°");
 pill("tRate",i.rate+" Hz",i.fresh&&i.rate?"ok":"bad");set("tId",String(i.id));set("tCrc",String(i.crcErr));
 vis("tTempR",i.tempC!==undefined);if(i.tempC!==undefined)set("tTemp",fx(i.tempC,1)+" °C / "+i.qos);
 set("tAxis",i.yAxis?"on":"off");
 const t=i.set;
 if(t.age<0)set("tSetup","not read");else if(t.setupWhy)pill("tSetup","differs: "+t.setupWhy,"warn");else pill("tSetup","yes","ok");
 const key=t.age>=0?[t.sw,t.accGain,t.magGain,t.silent,t.odr].join(":"):null;
 if(key!==null&&key!==tsSeen&&(!tsDirty||t.age<2)){tsSeen=key;tsFill(t);}
 if(Date.now()>tsErrUntil){let m="",c="";
  if(t.pending)m="Waiting for the TM171…";
  else if(t.odd){m="Settings received, but the layout is not recognised. Writing is disabled; use ImuAssistant, and send the bytes below to the firmware's author.";c="bad";}
  else if(t.age<0){m=t.noWire?"No reply from the TM171: wire Teensy TX"+s.ports.imu+" (pin "+s.ports.imuTx+") to the TM171 RXD.":t.noAnswer?"No reply yet, asking again…":"Not read yet.";c=t.noWire?"bad":"";}
  else{m="Read "+ago(t.age)+(t.ackAge>=0?". Last write acknowledged by the TM171 "+ago(t.ackAge):"")+".";c=t.ackAge>=0&&t.ackAge<30?"ok":"";}
  set("tsMsg",m,"note "+c);}
 set("tsBaud",t.baud?t.baud+" bps":"–");set("tsInhibit",t.inhibit!==undefined?t.inhibit+" ms":"–");
 set("tsRaw",t.hex?"Setting object as read: "+t.hex+"  (switches 0x"+(t.sw>>>0).toString(16).padStart(8,"0")+")":"");
 for(const e of $("tsForm").querySelectorAll("input,select"))e.disabled=!t.valid;
 $("tsWrite").dataset.off=(t.valid&&!t.pending)?"":"1";$("tsWrite").disabled=$("tsWrite").dataset.off==="1"||s.engaged;
 $("tsAog").disabled=!t.valid;$("tsRead").disabled=!!t.pending;
 renderCal(i.cal);
}
function renderCal(c){
 $("tcStop").classList.toggle("hidden",!(c.busy&&c.code==="gyro"));
 if(Date.now()<calErrUntil)return;
 let t,cl="";
 if(c.busy){cl="warn";
  if(c.code==="gyro"&&c.state==="running")t="Gyro calibration: collecting data, "+c.left+" s left. Keep the tractor still.";
  else if(c.state==="starting")t=c.what+": waiting for the TM171…";
  else if(c.state==="finishing")t=c.what+": finishing…";
  else t=c.what+": running"+(c.progress>=0?", "+c.progress+" %":"")+"…";}
 else if(c.state){const a=c.ago>=0?" ("+ago(c.ago)+")":"";
  const M={ok:["Done: ","ok"],failed:["FAILED: ","bad"],noanswer:["No answer from the TM171: ","bad"],aborted:["Stopped: ","warn"],revoked:["Revoked: ","ok"],sent:["Sent, no report: ",""]}[c.state]||["",""];
  t=M[0]+c.what+a+(c.state==="ok"&&c.code==="install"?". Switch the whole module off and on.":"");cl=M[1];}
 else t="No calibration since start-up.";
 set("tcMsg",t,"note "+cl);
}
async function imuPost(b,say){try{const j=await post("/api/imu",b);if(!j.ok)say("Error: "+j.error,"bad");return j.ok;}catch(e){say("Cannot reach the module.","bad");return false;}}
$("tsRead").addEventListener("click",()=>{tsDirty=false;tsSeen=null;imuPost("act=setread",tsSay);});
$("tsAog").addEventListener("click",()=>{const cur=lastS&&lastS.imu.set||{};
 tsFill({gyro:1,acc:1,mag:0,gyroErr:1,selfAdapt:1,status:1,boot:0,accGain:350,magGain:100,silent:0,odr:cur.odr?25:0});tsDirty=true;
 tsSay("AgOpenGPS set-up loaded into the form: gyro and accelerometer on, magnetometer off, GyroErrFilter with gain 3.50, Self-Adapt with gain 1.00, status output, silent time 0, 25 Hz, Auto boot. Click Write to TM171 to apply.","warn");});
$("tsWrite").addEventListener("click",async()=>{
 const n=id=>Math.round(parseFloat($(id).value)*100),acc=n("tsAccG"),mag=n("tsMagG"),sil=Math.round(+$("tsSilent").value);
 if(!(acc>=1&&acc<=2000&&mag>=1&&mag<=2000)){tsSay("Gains: 0.01 – 20.00","bad");return;}
 if(!(sil>=0&&sil<=65)){tsSay("Power-on silent time: 0 – 65 s","bad");return;}
 let q="Write these settings to the TM171"+($("tsSave").checked?" and save them to its flash":" (until it is powered off)")+"?";
 if(!$("tsGyro").checked||!$("tsAcc").checked)q+="\n\nWARNING: with the gyro or the accelerometer off the TM171 cannot give a usable roll.";
 if($("tsGyroErr").checked&&acc<200)q+="\n\nSYD recommends an accelerometer gain of 2.00 or more with GyroErrFilter on.";
 if(sil>0)q+="\n\nWith a silent time above 0 the Teensy may not find the TM171 when both start together, and then runs without IMU.";
 if(!confirm(q))return;
 const ck=id=>$(id).checked?1:0,odr=$("tsOdrR").classList.contains("hidden")?0:+$("tsOdr").value;
 const b="act=setwrite&gyro="+ck("tsGyro")+"&acc="+ck("tsAcc")+"&mag="+ck("tsMag")+"&gyroErr="+ck("tsGyroErr")+"&selfAdapt="+ck("tsSelf")+"&status="+ck("tsStatus")
  +"&save="+ck("tsSave")+"&boot="+$("tsBoot").value+"&accGain="+acc+"&magGain="+mag+"&silent="+sil+"&odr="+odr;
 tsSeen=null;tsDirty=false;if(await imuPost(b,tsSay))tsSay("Sent. Reading back…","");});
const CAL_ASK={gyro:"Calibrate the gyro bias now?\n\nKeep the tractor parked for 20 s (the engine can run). Autosteer must be off.",
 gyroAuto:"Run the forced static gyro calibration?\n\nKeep the tractor parked until it reports done.",
 gyroRevoke:"Revoke the forced gyro calibration results?",
 install:"Run the TM171 boot calibration (new install, CalibB)?\n\nKeep the tractor parked for about 20 s and do NOT cut the power while it runs (SYD warns this can damage the TM171). It cannot be stopped once started. The result is saved to flash; switch the whole module off and on afterwards.",
 mag:"Start the magnetometer hard-iron calibration (saved to flash)?\n\nTurn the module through every direction until it reports done.",
 magTmp:"Start the magnetometer hard-iron calibration for this power-on?\n\nTurn the module through every direction until it reports done.",
 magClear:"Clear the magnetometer calibration in the TM171's flash?",magClearTmp:"Clear the magnetometer calibration for this power-on?",
 headStart:"Start collecting data for the heading to magnetic north?",headSet:"Set the TM171 heading to {h}° (this power-on only)?"};
async function calGo(c){
 let p2=0,p3=0;
 if(c==="gyro"||c==="gyroAuto"){p2=Math.round(parseFloat($("tcLimit").value)*100);if(!(p2>=5&&p2<=500)){calSay("Accepted gyro error: 0.05 – 5.00 °/s","bad");return;}}
 if(c==="headStart"){p2=Math.round(parseFloat($("tcHeadT").value)*1000);if(!(p2>=0&&p2<=90000)){calSay("Collection time: 0 – 90 s","bad");return;}}
 if(c==="headSet"){const h=parseFloat($("tcHead").value);if(!(h>=0&&h<=360)){calSay("Heading: 0 – 360°","bad");return;}p2=Math.round(h*1000);}
 const q=CAL_ASK[c];if(q&&!confirm(q.replace("{h}",(p2/1000).toFixed(1))))return;
 calErrUntil=0;await imuPost("act=cal&c="+c+"&p2="+p2+"&p3="+p3,calSay);}
for(const b of document.querySelectorAll("[data-cal]"))b.addEventListener("click",()=>calGo(b.dataset.cal));
$("tcStop").addEventListener("click",()=>calGo("gyroStop"));

// ---- PVED tab (zPved.ino)
let pv=null,pvBuilt=false,pvErrUntil=0;
function pvDec(t,raw){if(raw===undefined)return undefined;raw=raw>>>0;if(t==="u8")return raw&255;if(t==="u16")return raw&65535;if(t==="s16")return ((raw&65535)<<16)>>16;return raw;}
function pvEnc(t,v){return t==="u32"?v>>>0:(v|0)>>>0;}   /* as the firmware: S16 sign extended to 32 bits */
function hex4(raw){raw=raw>>>0;return [0,8,16,24].map(s=>((raw>>>s)&255).toString(16).toUpperCase().padStart(2,"0")).join(" ");}
function pvSay(t,c){set("pvMsg",t,"note "+(c||""));pvErrUntil=Date.now()+6000;}
async function pvPost(b){try{const j=await post("/api/pved",b);if(!j.ok){pvSay("Not done: "+j.error,"bad");return false;}pollPved();return true;}catch(e){pvSay("Cannot reach the module.","bad");return false;}}
function pvBuild(params){
 const tb=$("pvTab").tBodies[0];tb.textContent="";
 for(const p of params){const tr=document.createElement("tr");tr.id="pvr"+p.n;
  tr.innerHTML='<td class="num"></td><td></td><td class="num v"></td><td class="mono"></td><td class="num"></td><td></td><td><input type="number" step="1"></td><td><button class="sec" type="button" data-pv="w">Write</button> <button class="sec" type="button" data-pv="r">Read</button></td>';
  tr.cells[0].textContent=p.n;tr.cells[1].textContent=(p.risky?"⚠ ":"")+p.name;
  const bs=tr.querySelectorAll("button");bs[0].addEventListener("click",()=>pvWrite(p.n));bs[1].addEventListener("click",()=>pvPost("act=read&n="+p.n));
  if(p.ro){tr.cells[6].textContent="read only";tr.cells[6].className="note";bs[0].remove();}
  tb.appendChild(tr);}
 pvBuilt=true;}
function renderPved(p){
 pv=p;if(!pvBuilt)pvBuild(p.params);
 pill("pvOn",p.on?"open, "+dur(p.since):"closed",p.on?"warn":"");
 set("pvQuietTxt",p.on&&p.quiet?"paused":"running");
 set("pvValve",p.valve+" / "+p.est);
 set("pvMode",p.mode?p.mode+" mode, "+ago(p.modeAgo):"not heard");
 set("pvQueue",p.queue?p.queue+" left"+(p.cur>=0?", now "+p.curKind+" "+p.cur:""):"–");
 vis("pvStart",!p.on);if(p.on)$("pvQuiet").checked=p.quiet;
 const moving=!!(lastS&&lastS.moving);
 for(const e of document.querySelectorAll("#t-pved [data-pv]"))e.disabled=!p.on||(e.dataset.pv==="w"&&moving);
 if(Date.now()>pvErrUntil){
  let m="";if(!p.on)m="Open the session to talk to the PVED.";else if(moving)m="The tractor is moving: reading only.";
  set("pvMsg",m,"note "+(moving&&p.on?"warn":""));}
 for(const q of p.params){const tr=$("pvr"+q.n);if(!tr)continue;const c=tr.cells;
  c[2].textContent=q.k?String(pvDec(q.t,q.raw)):(q.na?"no answer":"–");c[3].textContent=q.k?q.hex:"";
  c[4].textContent=q.orig!==undefined?String(pvDec(q.t,q.orig)):"";c[5].textContent=q.k?ago(q.age):"";
  tr.classList.toggle("chg",q.k&&q.orig!==undefined&&q.raw!==q.orig);tr.classList.toggle("na",!!q.na&&!q.k);}
 const a=p.any;set("pvAny",a.n?(a.k?"Parameter "+a.n+": "+a.raw+" ("+a.hex+"), "+ago(a.age):a.na?"Parameter "+a.n+": no answer":"Parameter "+a.n+": waiting…"):"");
 if(p.commitAgo>=0)set("pvCommitMsg",p.commitAgo<60?"Commit sent: wait "+(60-p.commitAgo)+" s more, then switch the PVED off and on.":"Commit sent "+ago(p.commitAgo)+". Switch the PVED off and on if you have not yet.","note "+(p.commitAgo<60?"warn":"ok"));
 vis("pvHackCard",!!(lastS&&lastS.steer.brand===0));
 const pv1=n=>{const q=p.params.find(x=>x.n===n);return q&&q.k?String(pvDec(q.t,q.raw))+(q.orig!==undefined&&q.orig!==q.raw?" (was "+pvDec(q.t,q.orig)+")":""):"not read";};
 set("pvH1",pv1(64007));set("pvH2",pv1(65101));set("pvH3",pv1(1027));
 const fb=$("pvFr").tBodies[0];fb.textContent="";
 for(const f of p.frames.slice().sort((x,y)=>x.id<y.id?-1:x.id>y.id?1:x.hex<y.hex?-1:1)){const tr=document.createElement("tr");
  for(const [t,cl] of [["0x"+f.id,"mono"],[f.hex,"mono"],[String(f.n),"num"],[ago(f.age),""]]){const td=document.createElement("td");td.textContent=t;if(cl)td.className=cl;tr.appendChild(td);}
  fb.appendChild(tr);}
 renderPvCal(p);
 if(!p.frames.length){const tr=document.createElement("tr"),td=document.createElement("td");td.colSpan=4;td.className="note";td.textContent=p.on?"none yet":"open the session to see them";tr.appendChild(td);fb.appendChild(tr);}
}
async function pollPved(){try{const p=await getJ("/api/pved");if(p.ok)renderPved(p);}catch(e){}}
// ---- valve auto-calibration (experimental)
const PV_PRE={ehd:[35,5,20,500],ehs:[50,5,20,500],osp:[50,5,20,500],ehps:[130,5,20,500]};
let pvPreSet=false;
function pvPreFill(k){const v=PV_PRE[k];$("pvXs").value=v[0];$("pvInc").value=v[1];$("pvYt").value=v[2];$("pvTms").value=v[3];}
pvPreFill("ehs");
$("pvPreset").addEventListener("change",()=>{pvPreSet=true;pvPreFill($("pvPreset").value);});
for(const id of ["pvXs","pvInc","pvYt","pvTms"])$(id).addEventListener("input",()=>{pvPreSet=true;});
function renderPvCal(p){
 const c=p.cal||{};
 if(p.mode)pill("pvCalMode",p.mode+(p.modeAgo>=0?", "+ago(p.modeAgo):""),p.mode==="calibration"?"warn":p.mode==="operational"?"ok":"bad");else set("pvCalMode","not heard (Ask mode)");
 set("pvEnterTxt",c.entering?"asking… "+c.enterLeft+" s left":"");
 set("pvCalAuto",c.autoAgo>=0?"started "+ago(c.autoAgo)+" from 0x"+(c.sa||0).toString(16).toUpperCase()+" (start "+c.xs+", step "+c.inc+", threshold "+c.yt+", "+c.tms+" ms)":"not started");
 set("pvCalSt",c.st?c.st+"  ("+c.stN+" frames, last "+ago(c.stAgo)+")":(c.autoAgo>=0?"none yet":"–"));
 if(!pvPreSet){const vt=p.params.find(q=>q.n===65121);if(vt&&vt.k){$("pvPreset").value=pvDec(vt.t,vt.raw)===1?"ehps":"ehs";pvPreFill($("pvPreset").value);pvPreSet=true;}}
 const f2=p.frames.find(f=>f.id.substr(2,4)==="FF02");
 if(f2){const by=f2.hex.split(" ").map(h=>parseInt(h,16)),w=i=>by.length>i+1?by[i]|(by[i+1]<<8):"–";
  set("pvSet1Txt","AD1 "+w(0)+"   AD2 "+w(2)+"   AD3 "+w(4)+"   Xsp "+w(6)+"   ("+ago(f2.age)+", "+f2.n+" frames)");}
 else set("pvSet1Txt",p.on?"no status data yet":"");}
$("pvGetMode").addEventListener("click",()=>pvPost("act=getmode"));
$("pvEnterCal").addEventListener("click",()=>{
 if(!confirm("Put the PVED in calibration mode?\n\nIn calibration mode it does not steer by GPS, and it may stay there until it is switched off and on."))return;
 pvPost("act=entercal");});
$("pvAutoCal").addEventListener("click",()=>{
 const g=id=>Math.round(+$(id).value),xs=g("pvXs"),inc=g("pvInc"),yt=g("pvYt"),tms=g("pvTms");
 if(!(xs>=0&&xs<=1000&&inc>=1&&inc<=100&&yt>=1&&yt<=255&&tms>=50&&tms<=5000)){pvSay("Auto-calibration values out of range.","bad");return;}
 if(!confirm("Start the valve auto-calibration?\n\nXspStartSearch "+xs+", XspIncrementSize "+inc+", YactDiffThreshold "+yt+", TimeOutPeriod "+tms+" ms\n\nThe PVED will move the steered wheels by itself, right and left. Keep everybody away from the wheels, the tractor standing on a solid surface with the wheels straight. Do not touch the steering wheel (it aborts). Afterwards switch the PVED off and on."))return;
 let force=0;
 if(!pv||pv.mode!=="calibration"){if(!confirm("The PVED has not reported calibration mode. Send the command anyway?"))return;force=1;}
 pvPost("act=autocal&xs="+xs+"&inc="+inc+"&yt="+yt+"&tms="+tms+"&from="+$("pvFrom").value+"&force="+force);});
$("pvReadRes").addEventListener("click",()=>pvPost("act=readres"));
$("pvSet1").addEventListener("click",()=>pvPost("act=statusset&set=1"));
$("pvSet0").addEventListener("click",()=>pvPost("act=statusset&set=0"));
function pvWrite(n){
 const q=pv&&pv.params.find(x=>x.n===n);if(!q)return;
 const inp=$("pvr"+n).querySelector("input"),s=inp.value.trim();
 if(!q.k){pvSay("Read parameter "+n+" first.","bad");return;}
 if(!/^-?\d+$/.test(s)){pvSay("Parameter "+n+": enter a whole number in the New value box.","bad");return;}
 const v=Number(s);
 let m="Write parameter "+n+" ("+q.name+")?\n\n"+pvDec(q.t,q.raw)+"  →  "+v+"    (bytes "+hex4(pvEnc(q.t,v))+")\n\nThe PVED keeps it only after Commit.";
 if(q.risky)m+="\n\n⚠ This is the valve / cylinder calibration. A wrong value can make the steering misbehave.";
 if(!confirm(m))return;
 pvPost("act=write&n="+n+"&v="+v).then(ok=>{if(ok){inp.value="";pvSay("Writing "+n+" = "+v+", then reading it back…","");}});}
$("pvStart").addEventListener("click",()=>{
 if(!confirm("Open the PVED session?\n\nAutosteer is blocked"+($("pvQuiet").checked?" and the steering commands to the valve pause":"")+" until the session is closed."))return;
 pvPost("act=start&quiet="+($("pvQuiet").checked?1:0));});
$("pvQuiet").addEventListener("change",()=>{if(pv&&pv.on)pvPost("act=start&quiet="+($("pvQuiet").checked?1:0));});
$("pvStop").addEventListener("click",()=>pvPost("act=stop"));
$("pvReadAll").addEventListener("click",()=>pvPost("act=readall"));
$("pvAnyRead").addEventListener("click",()=>{const n=Math.round(+$("pvAnyN").value);if(!(n>=1&&n<=65535)){pvSay("Parameter number: 1 – 65535","bad");return;}pvPost("act=read&n="+n);});
$("pvCommit").addEventListener("click",()=>{
 if(!confirm("Commit the written values to the PVED memory?\n\nAfterwards wait 1 minute, then switch the PVED off and on."))return;pvPost("act=commit");});
$("pvRestore").addEventListener("click",()=>{
 const d=pv?pv.params.filter(q=>q.k&&q.orig!==undefined&&q.raw!==q.orig):[];
 if(!d.length){pvSay("Nothing differs from the values first read.","");return;}
 if(!confirm("Write the original values back?\n\n"+d.map(q=>q.n+" "+q.name+": "+pvDec(q.t,q.raw)+" → "+pvDec(q.t,q.orig)).join("\n")+"\n\nThen Commit."))return;
 pvPost("act=restore");});
$("pvHack").addEventListener("click",()=>{
 if(!confirm("Write the Claas hack?\n\n64007 set point address = 30 (0x1E)\n65101 SASA present = 255\n1027 fast steer flow = 0\n\nThen Commit, wait 1 minute and switch the PVED off and on."))return;pvPost("act=hack");});
$("pvRawSend").addEventListener("click",()=>{
 const h=$("pvRawHex").value.replace(/\s+/g,"");
 if(!/^[0-9a-fA-F]{16}$/.test(h)){pvSay("8 bytes = 16 hex digits, e.g. 0F AB 00 00 00 00 00 00","bad");return;}
 if(!confirm("Send "+h.toUpperCase().replace(/(..)(?!$)/g,"$1 ")+" to the PVED (0x18EF13FD)?"))return;pvPost("act=raw&hex="+h);});
$("pvBackup").addEventListener("click",()=>{
 if(!pv)return;
 const rows=["param,name,type,value,raw,bytes,original_value,original_raw,original_bytes"];
 for(const q of pv.params){if(!q.k&&q.orig===undefined)continue;
  rows.push([q.n,'"'+q.name+'"',q.t,q.k?pvDec(q.t,q.raw):"",q.k?q.raw:"",q.k?q.hex:"",q.orig!==undefined?pvDec(q.t,q.orig):"",q.orig!==undefined?q.orig:"",q.orig!==undefined?hex4(q.orig):""].join(","));}
 if(rows.length<2){pvSay("Nothing read yet: Read all first.","bad");return;}
 const d=new Date(),z=x=>String(x).padStart(2,"0"),name="pved_backup_"+d.getFullYear()+"-"+z(d.getMonth()+1)+"-"+z(d.getDate())+"_"+z(d.getHours())+z(d.getMinutes())+".csv";
 const a=document.createElement("a");a.href=URL.createObjectURL(new Blob([rows.join("\r\n")+"\r\n"],{type:"text/csv"}));a.download=name;
 document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(a.href),10000);pvSay("Downloaded "+name,"ok");});

// ---- firmware update (zOTA.ino)
function fwMsg(t,c){set("fwMsg",t,"note "+(c||""));}
// What is inside a chosen .hex: the data records put together, then the "[fw] AOG_CAN_Teensy4.1, built ..." line
// gives the build date; "fw_teensy41" says it is a Teensy 4.1 firmware, the marker that this web page is inside.
let hexBuild=null;
function hexRead(text){
 let s="";
 for(const ln of text.split("\n")){if(ln.charCodeAt(0)!==58||ln.substr(7,2)!=="00")continue;const n=parseInt(ln.substr(1,2),16);let o="";for(let i=0;i<n;i++)o+=String.fromCharCode(parseInt(ln.substr(9+2*i,2),16));s+=o;}
 const m=/\[fw\] (AOG_CAN_Teensy4\.1), built ([A-Z][a-z]{2} [ \d]\d \d{4} \d\d:\d\d:\d\d)/.exec(s);
 return {fw:m?m[1]:"",built:m?m[2]:"",t41:s.includes("fw_teensy41"),marker:s.includes("AOG-CAN-T41-WEB-OTA-1")};
}
$("hexFile").addEventListener("change",()=>{const f=$("hexFile").files[0];hexBuild=null;
 set("hexName",f?f.name+" ("+Math.round(f.size/1024)+" kB)":"no file");if(lastS)locks(lastS);fwMsg("");
 vis("hexInfo",!!f);if(!f)return;set("hexInfo","Reading the file…","note");
 const r=new FileReader();
 r.onload=()=>{const b=hexBuild=hexRead(r.result);let t,c="";
  if(!b.t41){t="Not a Teensy 4.1 firmware (no fw_teensy41 inside): the Teensy refuses it.";c="bad";}
  else if(!b.marker){t="Another firmware (no web update page inside). It needs Other firmware, and the next update will be by USB.";c="warn";}
  else{const same=!!lastS&&lastS.built===b.built;t=b.fw+", built "+b.built.replace("  "," ")+(same?" (the firmware running now)":"");c=same?"":"ok";}
  set("hexInfo",t,"note "+c);};
 r.onerror=()=>set("hexInfo","The file could not be read.","note bad");
 r.readAsText(f);});
$("install").addEventListener("click",()=>{
 const f=$("hexFile").files[0];if(!f||busy)return;
 if(!f.name.toLowerCase().endsWith(".hex")){fwMsg("Choose a .hex file.","bad");return;}
 const b=hexBuild&&hexBuild.fw?" ("+hexBuild.fw+", built "+hexBuild.built.replace("  "," ")+")":"";
 if(!confirm("Install "+f.name+b+"?\n\nAutosteer must be off and the tractor stopped. Steering is off during the update; do not switch the power off until the module is back."))return;
 busy=true;$("install").disabled=true;const p=$("prog");p.classList.remove("hidden");p.value=0;fwMsg("Sending the file…");
 const x=new XMLHttpRequest();
 x.open("POST","/api/ota"+($("force").checked?"?force=1":""));
 x.setRequestHeader("Content-Type","text/plain");
 x.upload.onprogress=e=>{if(e.lengthComputable)p.value=Math.round(e.loaded/e.total*95);};
 x.upload.onload=()=>fwMsg("File sent; the Teensy is checking and writing it…");
 const fail=t=>{busy=false;if(lastS)locks(lastS);p.classList.add("hidden");fwMsg(t,"bad");};
 x.onload=()=>{let j={};try{j=JSON.parse(x.responseText);}catch(e){}
  if(x.status===200&&j.ok){p.value=100;waitingBoot=true;setOnline(false);fwMsg("Checked ("+Math.round(j.size/1024)+" kB). Installing and restarting – do not switch the power off…","warn");}
  else fail("Not installed: "+(j.error||("error "+x.status))+". The current firmware keeps running.");};
 x.onerror=()=>fail("The connection failed. The current firmware keeps running.");
 x.send(f);});
$("reboot").addEventListener("click",async()=>{
 if(!confirm("Restart the Teensy now?"))return;
 try{const j=await post("/api/reboot","");if(j.ok){waitingBoot=true;fwMsg("Restarting…","warn");}else fwMsg("Not restarted: "+j.error,"bad");}catch(e){fwMsg("Cannot reach the module.","bad");}});
try{const m=sessionStorage.getItem("fwMsg");if(m){sessionStorage.removeItem("fwMsg");fwMsg(m,"ok");}}catch(e){}

// ---- messages
let logAfter=0;
async function pollLog(){
 try{const j=await getJ("/api/log?after="+logAfter);const box=$("log"),atEnd=box.scrollTop+box.clientHeight>=box.scrollHeight-20;
  for(const l of j.lines||[]){const d=document.createElement("div");d.className="ll";const t=document.createElement("span");t.className="t";t.textContent=dur(Math.floor(l.t/1000));
   const x=document.createElement("span");x.className="mono";x.textContent=l.x;d.appendChild(t);d.appendChild(x);box.appendChild(d);logAfter=l.s;}
  while(box.childNodes.length>400)box.removeChild(box.firstChild);
  if(atEnd)box.scrollTop=box.scrollHeight;
  if(j.lines&&j.lines.length&&logAfter<j.next-1)setTimeout(pollLog,50);
 }catch(e){}
}

loadCfg();pollStatus();pollLog();
setInterval(pollStatus,1000);setInterval(pollLog,2000);setInterval(()=>{if(tab==="pved")pollPved();},1000);
if(tab==="pved")pollPved();
</script>
</body>
</html>
)WEBPAGE";

#endif
