/* zPved.ino - the PVED tab of the web page: read, write and commit Danfoss PVED-CL parameters over the V-Bus
   (CAN3) while the firmware keeps running.
   AOG_CAN_Teensy4.1, 2026. The same service frames as the serial service tool (CAN_Service_Tool.ino, Z):
     to the PVED   0x18EF13FD (DA 0x13, service tool address 0xFD), byte 0 = 0x0F, byte 1 = XID:
                   A0 read, A2 write (parameter number LE in bytes 2-3, value 32 bits LE in 4-7), A6 5A commit
     from the PVED 0x18EF..13: 0F A1 read answer, 0F A3 write answer, 0F AA <mode> = GetCurrentModeResponse,
                   anything else is shown as it is (also its broadcasts PGN 0xFF01 / 0xFF02, status data)
   2026-11: experimental valve auto-calibration (PVED-CL Operation Manual 11025583 rev CA, "Method 3: Valve
   auto-calibration", pages 29-33). The frames are NOT from Danfoss' PVED-CL Communication Protocol (11025584,
   not public) but from the XID table of a third-party "PVED-CL Diagnostics" tool, which matches every XID already
   known here (A0-A3, A6, AA): A8 EnterCalibrationMode (payload 58), A9 GetCurrentMode -> AA <mode> (55 calibration,
   AA operational, AF reduced, FF fault), AB StartStopStatus <set>, B6 StartValveAutoCalibration <XspStartSearch
   u16 LE> <XspIncrementSize u8> <YactDiffThreshold u8> <TimeOutPeriod u16 LE>, B7 ValveAutoCalibrationStatus
   (codes not known: shown raw). Unused bytes 0xFF, as that tool sends them.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// The page opens a "session": a V-Bus FIFO filter for every proprietary-A frame from the PVED is added, autosteer
// is blocked, and (by default, like the serial tool whose loop stopped the firmware) the steering commands to the
// valve pause. Requests go out one at a time from a queue: the next one after the answer, or after 1 s without.
// A write is always read back. Nothing is written unless that parameter was read in this boot first (its first
// value is kept as the "original", for the backup and "Restore originals"), and no write, commit or raw frame
// while the tractor moves. The session closes by itself when no page has asked for a minute.
// Valve auto-calibration: the PVED moves the wheels by itself, searching the spool dead-bands with the wheel angle
// sensor, then writes Xspl_0 / Xspr_0 (and shifts Xspl_1000 / Xspr_1000) and commits them itself; it stays in
// calibration mode until power is cycled. Only with the session open, the steering commands paused, the tractor
// stopped and the valve parameters read first (backup, "Restore originals").

#define PVED_FILTER_SLOT   7         // V-Bus FIFO filter of the session (brands use 0-4, the Claas steer diag 3-7)
#define PVED_FILTER_SLOT2  6         // ... and one for the PVED's broadcasts (PF 0xFF: 0xFF01 mode, 0xFF02 status data)
#define PVED_ENTER_MS      20000     // EnterCalibrationMode is repeated this long (the PVED may take it only at power-up)
#define PVED_AUTO_MS       600000    // the auto-calibration is watched this long at most
#define PVED_ANSWER_MS     1000
#define PVED_GAP_MS        50
#define PVED_IDLE_MS       60000     // no web page for this long: the session closes
#define PVED_Q_N           48
#define PVED_FRAMES        16

enum { PV_U8, PV_U16, PV_S16, PV_U32 };
struct PvedDef { uint16_t num; uint8_t type; bool risky; const char* name; bool ro; };

// The parameters the serial service tool reads with R, then the sensor mapping and the CAN addresses (PVED-CL
// Operation Manual 11025583 rev CA, appendix pages 120-122). risky = valve calibration / sensor mapping: only change
// it knowing the right value. ro = read only here: the PVED's own address and the addresses of the MMI and of the
// service tool this page talks as (0xFD); changing those would cut the page off from the PVED.
static const PvedDef pvDefs[] = {
    {   508, PV_S16, true,  "Kp gain, GPS steering (0-200)" },
    {   706, PV_U16, true,  "Vcap, valve capacity (L/min)" },
    {   707, PV_U16, true,  "StrkVol, cylinder stroke volume (ccm)" },
    {   729, PV_S16, true,  "Xspl_1000, left max spool" },
    {   737, PV_S16, true,  "Xspl_0, left dead-band" },
    {   738, PV_S16, true,  "Xspr_0, right dead-band" },
    {   747, PV_S16, true,  "Xspr_1000, right max spool" },
    {   748, PV_S16, true,  "ClosedLoopXspOffset" },
    {   758, PV_S16, true,  "XspCalibrationOffset (auto-calibration)" },
    {  1027, PV_S16, false, "Qm max port flow, steering wheel / fast steer (0-1000 = 0-100 %)" },
    {  5027, PV_S16, false, "Qm max port flow, GPS steering (0-1000 = 0-100 %)" },
    { 64003, PV_U8,  false, "PvedSourceAddress (19 = 0x13)", true },
    { 64005, PV_U8,  false, "ControlDeviceSourceAddress, MMI (252 = 0xFC)", true },
    { 64006, PV_U8,  false, "ConfigurationDeviceSourceAddress, service tool (253 = 0xFD)", true },
    { 64007, PV_U8,  false, "HPExtSourceAddress, GPS set-point controller (Claas hack: 30)" },
    { 64011, PV_U8,  false, "WAReducedModeAllowed, WAS fault -> reduced mode (0 / 255)" },
    { 64018, PV_U16, false, "WAPowerUpTimeout, WAS at power-up, ms" },
    { 64022, PV_U16, false, "StwDxActivationThreshold (SASA disengage check, 1 = 1.4 rpm)" },
    { 64023, PV_U32, false, "StwActivationTimeout, ms (2147483647 = check off)" },
    { 65080, PV_U16, false, "AD1_1000_Left (WAS left)" },
    { 65083, PV_U16, false, "AD1_1000_Right (WAS right)" },
    { 65086, PV_U16, false, "AD1_Neutral (WAS centre)" },
    { 65099, PV_U32, false, "MaxWheelAngleLeft, mdeg (at the WAS wheel)" },
    { 65100, PV_U32, false, "MaxWheelAngleRight, mdeg (at the WAS wheel)" },
    { 65101, PV_U8,  false, "Steering wheel sensor SASA (0 none, 255 CAN)" },
    { 65087, PV_U8,  true,  "AD1_Linear (255 = 3-point, 0 = 5-point)" },
    { 65104, PV_U8,  true,  "PrimaryWheelAngleSensorInterface: WAS from 0 none, 1 AD1, 2 AD2, 4 CAN" },
    { 65105, PV_U8,  true,  "ExternalSetPointControllerPresent, GPS (0 / 255)" },
    { 65107, PV_U8,  true,  "RedundantWheelAngleSensorPresent (0 / 255)" },
    { 65112, PV_U16, false, "VehicleLength = wheelbase, mm" },
    { 65121, PV_U8,  false, "ValveType (1 EHPS, 2 EH / OSPE)" },
};
#define PV_N (sizeof(pvDefs) / sizeof(pvDefs[0]))

struct PvedVal { bool known, origKnown, noAnswer; uint32_t raw, orig, ms; };
static PvedVal pvVal[PV_N];

bool pvedWebOn = false;              // session open (zWeb.h: the main loop blocks autosteer, SteerDiag.ino stays off)
bool pvedWebQuiet = false;           // steering commands to the valve paused (main loop)
static uint32_t pvSinceMs = 0, pvSeenMs = 0, pvCommitMs = 0, pvModeMs = 0;
static uint8_t  pvMode = 0;          // 0x55 calibration mode, 0xAA operation mode (0 = not heard)
static uint16_t pvSent65112 = 0;     // vehicle length written this boot, for the firmware L at commit

// one parameter read by its number (any number, not only the list)
static uint16_t pvAnyNum = 0;
static bool     pvAnyKnown = false, pvAnyNoAnswer = false;
static uint32_t pvAnyRaw = 0, pvAnyMs = 0;

// the request queue
enum { PVOP_READ, PVOP_WRITE };
struct PvedOp { uint8_t kind; uint16_t num; uint32_t raw; };
static PvedOp  pvQ[PVED_Q_N];
static uint8_t pvQHead = 0, pvQTail = 0;
static PvedOp  pvCur;
static uint8_t pvStage = 0;          // 0 idle, 1 waiting for a read answer, 2 waiting for a write answer
static uint32_t pvOpMs = 0, pvGapMs = 0;

// valve auto-calibration (experimental)
static uint32_t pvEnterMs = 0, pvEnterSendMs = 0, pvModePollMs = 0;   // entering calibration mode
static uint32_t pvAutoMs = 0;                                           // StartValveAutoCalibration sent (0 = not)
static uint16_t pvAutoXs = 0, pvAutoTms = 0;
static uint8_t  pvAutoInc = 0, pvAutoYt = 0;
static uint8_t  pvAutoSt[8], pvAutoStLen = 0;                           // the last ValveAutoCalibrationStatus
static uint32_t pvAutoStMs = 0, pvAutoStN = 0;

// the other frames from the PVED: the latest one of each kind (0F xx answers: ID + first two bytes; else the ID)
struct PvedFrame { uint32_t id; uint8_t len; uint8_t d[8]; uint32_t n, ms; };
static PvedFrame pvF[PVED_FRAMES];

static int8_t pvIndex(uint16_t num)
{
    for (uint8_t i = 0; i < PV_N; i++) if (pvDefs[i].num == num) return i;
    return -1;
}

static uint8_t pvQCount() { return (uint8_t)((pvQTail + PVED_Q_N - pvQHead) % PVED_Q_N); }

static bool pvQPush(uint8_t kind, uint16_t num, uint32_t raw)
{
    const uint8_t next = (pvQTail + 1) % PVED_Q_N;
    if (next == pvQHead) return false;
    pvQ[pvQTail] = { kind, num, raw };
    pvQTail = next;
    return true;
}

static const char* pvModeName(uint8_t m)
{
    switch (m)
    {
        case 0x55: return "calibration";
        case 0xAA: return "operational";
        case 0xAF: return "reduced";
        case 0xFF: return "fault";
        default:   return m ? "unknown" : "";
    }
}

// one service frame to the PVED from source address sa: 0F <xid> <data...>, the rest 0xFF
// (pvSendXid: from the service tool address 0xFD; no default argument: the Arduino IDE's prototypes repeat it)
static void pvSendXidFrom(uint8_t sa, uint8_t xid, const uint8_t* d, uint8_t n)
{
    CAN_message_t msg;
    msg.id = 0x18EF1300 | sa;
    msg.flags.extended = true;
    msg.len = 8;
    memset(msg.buf, 0xFF, 8);
    msg.buf[0] = 0x0F;
    msg.buf[1] = xid;
    for (uint8_t i = 0; i < n && i < 6; i++) msg.buf[2 + i] = d[i];
    V_Bus.write(msg);
}

static void pvSendXid(uint8_t xid, const uint8_t* d, uint8_t n) { pvSendXidFrom(0xFD, xid, d, n); }

static void pvBytes(uint32_t raw, char* out)          // "1E 00 00 00"
{
    snprintf(out, 12, "%02X %02X %02X %02X", (unsigned)(raw & 0xFF), (unsigned)((raw >> 8) & 0xFF),
             (unsigned)((raw >> 16) & 0xFF), (unsigned)(raw >> 24));
}

// ---------------------------------------------------------------- frames from the PVED (VBus_Receive)
// True when the frame was the PVED's (proprietary A from SA 0x13): taken here, not by the brand code
bool pvedWebRx(const CAN_message_t& m)
{
    const uint8_t pf = (m.id >> 16) & 0xFF;
    if (!m.flags.extended || (pf != 0xEF && pf != 0xFF) || (m.id & 0xFF) != 0x13) return false;
    const uint32_t now = millis();
    if (m.len >= 8 && m.buf[0] == 0x0F && (m.buf[1] == 0xA1 || m.buf[1] == 0xA3))
    {
        const uint16_t num = m.buf[2] | (m.buf[3] << 8);
        const uint32_t raw = m.buf[4] | (m.buf[5] << 8) | ((uint32_t)m.buf[6] << 16) | ((uint32_t)m.buf[7] << 24);
        // the same PARAM line as the serial tool (backup / restore by hand)
        webLog("PARAM,%u,%lu,%02X %02X %02X %02X,%s", num, (unsigned long)raw, m.buf[4], m.buf[5], m.buf[6], m.buf[7],
               m.buf[1] == 0xA1 ? "A1" : "A3");
        if (m.buf[1] == 0xA1)
        {
            const int8_t i = pvIndex(num);
            if (i >= 0)
            {
                PvedVal& v = pvVal[i];
                v.raw = raw; v.known = true; v.noAnswer = false; v.ms = now | 1;
                if (!v.origKnown) { v.orig = raw; v.origKnown = true; }
            }
            if (num == pvAnyNum) { pvAnyRaw = raw; pvAnyKnown = true; pvAnyNoAnswer = false; pvAnyMs = now | 1; }
        }
        if (pvStage == 1 && m.buf[1] == 0xA1 && num == pvCur.num) { pvStage = 0; pvGapMs = now; }
        else if (pvStage == 2 && m.buf[1] == 0xA3 && num == pvCur.num)
        {
            pvedRequest(0xA0, num, 0);                      // read back what the PVED kept
            pvStage = 1; pvOpMs = now;
        }
        return true;
    }
    if (pf == 0xEF && m.len >= 3 && m.buf[0] == 0x0F && m.buf[1] == 0xAA)    // GetCurrentModeResponse
    {
        if (m.buf[2] != pvMode) webLog("[pved] PVED mode: %s (0x%02X)", pvModeName(m.buf[2]), m.buf[2]);
        pvMode = m.buf[2];
        pvModeMs = now | 1;
    }
    if (pf == 0xEF && m.len >= 2 && m.buf[0] == 0x0F && m.buf[1] == 0xB7)    // ValveAutoCalibrationStatus
    {
        const uint8_t n = m.len > 8 ? 8 : m.len;
        if (n != pvAutoStLen || memcmp(pvAutoSt, m.buf, n))
            webLog("[pved] auto-calibration status: %02X %02X %02X %02X %02X %02X %02X %02X", m.buf[0], m.buf[1], m.buf[2],
                   m.buf[3], m.buf[4], m.buf[5], m.buf[6], m.buf[7]);
        memcpy(pvAutoSt, m.buf, n);
        pvAutoStLen = n;
        pvAutoStMs = now | 1;
        pvAutoStN++;
    }
    // the latest frame of each kind
    int8_t k = -1, oldest = 0;
    for (uint8_t i = 0; i < PVED_FRAMES; i++)
    {
        if (!pvF[i].n) { if (k < 0) k = i; continue; }
        const bool byXid = pf == 0xEF && m.len >= 2 && m.buf[0] == 0x0F;   // service answers by XID, broadcasts by ID
        if (pvF[i].id == m.id && (!byXid || (pvF[i].len >= 2 && pvF[i].d[0] == m.buf[0] && pvF[i].d[1] == m.buf[1]))) { k = i; break; }
        if (pvF[i].ms < pvF[oldest].ms) oldest = i;
    }
    if (k < 0) { k = oldest; pvF[k].n = 0; }
    PvedFrame& f = pvF[k];
    if (!f.n) webLog("[pved] new frame 0x%08lX, %u bytes: %02X %02X ...", (unsigned long)m.id, m.len, m.buf[0], m.buf[1]);
    f.id = m.id; f.len = m.len > 8 ? 8 : m.len; memcpy(f.d, m.buf, 8); f.n++; f.ms = now | 1;
    return true;
}

// ---------------------------------------------------------------- session
// From the web page (/api/status and /api/pved): the page is still there
void pvedWebSeen() { pvSeenMs = millis(); }

bool pvedWebStart(bool quiet, bool engaged, const char** why)
{
    if (pvedWebOn)
    {
        if (quiet != pvedWebQuiet) webLog("[pved] steering commands to the valve %s", quiet ? "paused" : "running again");
        pvedWebQuiet = quiet;
        return true;
    }
    if (engaged)     { *why = "autosteer is engaged - disengage it first"; return false; }
    if (steerDiagOn) { *why = "the steering diagnostics CSV is on (USB serial D) - switch it off first"; return false; }
    if (!V_Bus.setFIFOUserFilter(PVED_FILTER_SLOT, 0x00EF0013, 0x00FF00FF, EXT) ||
        !V_Bus.setFIFOUserFilter(PVED_FILTER_SLOT2, 0x00FF0013, 0x00FF00FF, EXT)) { *why = "could not set the V-Bus filter"; return false; }
    pvedWebOn = true;
    pvedWebQuiet = quiet;
    pvSinceMs = pvSeenMs = millis();
    pvQHead = pvQTail = 0;
    pvStage = 0;
    webLog("[pved] session open: autosteer blocked, steering commands to the valve %s", quiet ? "paused" : "running");
    return true;
}

void pvedWebStop(const char* why)
{
    if (!pvedWebOn) return;
    pvedWebOn = false;
    pvedWebQuiet = false;
    pvQHead = pvQTail = 0;
    pvStage = 0;
    V_Bus.setFIFOFilter(PVED_FILTER_SLOT, 0x1FFFFFFF, EXT);   // an ID nobody sends (source address 0xFF): the slot is free again
    V_Bus.setFIFOFilter(PVED_FILTER_SLOT2, 0x1FFFFFFF, EXT);
    pvEnterMs = 0;
    webLog("[pved] session closed (%s), steering commands to the valve running", why);
}

// From webLoop(): the queue, the answer timeouts, the idle close
void pvedWebLoop(bool moving)
{
    if (!pvedWebOn) return;
    const uint32_t now = millis();
    if (now - pvSeenMs > PVED_IDLE_MS) { pvedWebStop("no web page for a minute"); return; }
    if (pvStage && now - pvOpMs > PVED_ANSWER_MS)
    {
        if (pvStage == 2)
        {
            webLog("[pved] %u: no write answer, reading it back", pvCur.num);
            pvedRequest(0xA0, pvCur.num, 0);
            pvStage = 1; pvOpMs = now;
            return;
        }
        const int8_t i = pvIndex(pvCur.num);
        if (i >= 0) pvVal[i].noAnswer = true;
        if (pvCur.num == pvAnyNum) pvAnyNoAnswer = true;
        webLog("[pved] %u: no answer (is the PVED powered? Switch it off and on while the session is open)", pvCur.num);
        pvStage = 0; pvGapMs = now;
    }
    // entering calibration mode: EnterCalibrationMode every 200 ms until the PVED reports it (or 20 s)
    if (pvEnterMs)
    {
        if (pvMode == 0x55 && (int32_t)(pvModeMs - pvEnterMs) > 0) { pvEnterMs = 0; webLog("[pved] calibration mode confirmed"); }
        else if (now - pvEnterMs > PVED_ENTER_MS) { pvEnterMs = 0; webLog("[pved] no calibration mode after 20 s (power-cycle the PVED while it is being asked?)"); }
        else if (now - pvEnterSendMs >= 200) { const uint8_t k = 0x58; pvSendXid(0xA8, &k, 1); pvEnterSendMs = now; }
    }
    // ask the mode once a second while entering, and every 2 s while an auto-calibration is watched
    const bool autoOn = pvAutoMs && now - pvAutoMs < PVED_AUTO_MS;
    if ((pvEnterMs && now - pvModePollMs >= 1000) || (autoOn && now - pvModePollMs >= 2000)) { pvSendXid(0xA9, nullptr, 0); pvModePollMs = now; }

    if (pvStage || pvQHead == pvQTail || now - pvGapMs < PVED_GAP_MS) return;
    pvCur = pvQ[pvQHead];
    pvQHead = (pvQHead + 1) % PVED_Q_N;
    if (pvCur.kind == PVOP_WRITE)
    {
        if (moving) { pvQHead = pvQTail = 0; webLog("[pved] writes cancelled: the tractor is moving"); return; }
        char b[12];
        pvBytes(pvCur.raw, b);
        webLog("[pved] write %u = %lu (%s)", pvCur.num, (unsigned long)pvCur.raw, b);
        if (pvCur.num == 65112) pvSent65112 = (uint16_t)pvCur.raw;
        pvedRequest(0xA2, pvCur.num, pvCur.raw);
        pvStage = 2;
    }
    else
    {
        pvedRequest(0xA0, pvCur.num, 0);
        pvStage = 1;
    }
    pvOpMs = now;
}

// ---------------------------------------------------------------- actions (zWeb.ino, POST /api/pved)
bool pvedWebReadAll(const char** why)
{
    if (!pvedWebOn) { *why = "open the PVED session first"; return false; }
    if (pvQCount() + PV_N >= PVED_Q_N - 1) { *why = "busy, try again in a moment"; return false; }
    for (uint8_t i = 0; i < PV_N; i++) pvQPush(PVOP_READ, pvDefs[i].num, 0);
    webLog("[pved] reading %u parameters", (unsigned)PV_N);
    return true;
}

bool pvedWebRead(uint16_t num, const char** why)
{
    if (!pvedWebOn) { *why = "open the PVED session first"; return false; }
    if (pvIndex(num) < 0) { pvAnyNum = num; pvAnyKnown = pvAnyNoAnswer = false; }
    if (!pvQPush(PVOP_READ, num, 0)) { *why = "busy, try again in a moment"; return false; }
    return true;
}

// value as shown on the page (signed for S16) -> the 32-bit raw value of the frame
static bool pvEncode(uint8_t i, int64_t v, uint32_t& raw, const char** why)
{
    const PvedDef& d = pvDefs[i];
    int64_t lo = 0, hi = 0;
    switch (d.type)
    {
        case PV_U8:  lo = 0;      hi = 255;         break;
        case PV_U16: lo = 0;      hi = 65535;       break;
        case PV_S16: lo = -32768; hi = 32767;       break;
        default:     lo = 0;      hi = 4294967295LL; break;
    }
    if (d.num == 64007) hi = 253;                       // a source address (254 = null, 255 = global)
    if (d.num == 65102 || d.num == 65103 || d.num == 65104)  // interface: none, AD1, AD2, CAN
        if (v != 0 && v != 1 && v != 2 && v != 4) { *why = "0 none, 1 AD1, 2 AD2 or 4 CAN"; return false; }
    if (d.num == 65087 || d.num == 65101 || d.num == 65105 || d.num == 65107 || d.num == 64011)
        if (v != 0 && v != 255) { *why = "0 or 255"; return false; }
    if (d.num == 65112) { lo = 500; hi = 10000; }
    if (v < lo || v > hi) { *why = d.num == 65112 ? "vehicle length: 500 - 10000 mm" : d.num == 64007 ? "address: 0 - 253" : "value out of range for this parameter"; return false; }
    raw = (uint32_t)(int32_t)v;                         // S16: sign extended to 32 bits
    if (d.type == PV_U32) raw = (uint32_t)v;
    return true;
}

static bool pvWriteChecks(bool moving, const char** why)
{
    if (!pvedWebOn) { *why = "open the PVED session first"; return false; }
    if (moving)     { *why = "the tractor is moving - stop first"; return false; }
    return true;
}

bool pvedWebWrite(uint16_t num, int64_t v, bool moving, const char** why)
{
    if (!pvWriteChecks(moving, why)) return false;
    const int8_t i = pvIndex(num);
    if (i < 0) { *why = "only the parameters in the list can be written"; return false; }
    if (pvDefs[i].ro) { *why = "read only here (an address this page or the PVED depends on)"; return false; }
    if (!pvVal[i].origKnown) { *why = "read it first (its original value is the backup)"; return false; }
    uint32_t raw;
    if (!pvEncode(i, v, raw, why)) return false;
    if (!pvQPush(PVOP_WRITE, num, raw)) { *why = "busy, try again in a moment"; return false; }
    return true;
}

// The Claas hack, as the serial tool's W: 64007 = 30 (the PVED takes set points from address 0x1E, this module),
// 65101 = 255 (SASA present), 1027 = 0 (no fast steer). Then Commit and switch the PVED off and on.
bool pvedWebClaasHack(bool moving, const char** why)
{
    if (!pvWriteChecks(moving, why)) return false;
    const uint16_t nums[3] = { 64007, 65101, 1027 };
    const uint32_t vals[3] = { 30, 255, 0 };
    for (uint8_t k = 0; k < 3; k++) if (!pvVal[pvIndex(nums[k])].origKnown) { *why = "read all parameters first (backup)"; return false; }
    if (pvQCount() + 3 >= PVED_Q_N - 1) { *why = "busy, try again in a moment"; return false; }
    for (uint8_t k = 0; k < 3; k++) pvQPush(PVOP_WRITE, nums[k], vals[k]);
    webLog("[pved] Claas hack: 64007 = 30, 65101 = 255, 1027 = 0 (commit afterwards)");
    return true;
}

// Write back the first value read in this boot of every parameter that differs now
bool pvedWebRestore(bool moving, const char** why)
{
    if (!pvWriteChecks(moving, why)) return false;
    uint8_t n = 0;
    for (uint8_t i = 0; i < PV_N; i++)
        if (pvVal[i].known && pvVal[i].origKnown && pvVal[i].raw != pvVal[i].orig) n++;
    if (!n) { *why = "nothing differs from the values first read"; return false; }
    if (pvQCount() + n >= PVED_Q_N - 1) { *why = "busy, try again in a moment"; return false; }
    for (uint8_t i = 0; i < PV_N; i++)
        if (pvVal[i].known && pvVal[i].origKnown && pvVal[i].raw != pvVal[i].orig) pvQPush(PVOP_WRITE, pvDefs[i].num, pvVal[i].orig);
    webLog("[pved] restoring %u parameters to the values first read (commit afterwards)", n);
    return true;
}

bool pvedWebCommit(bool moving, const char** why)
{
    if (!pvWriteChecks(moving, why)) return false;
    if (pvStage || pvQHead != pvQTail) { *why = "wait until the reads and writes are done"; return false; }
    pvedRequest(0xA6, 0x005A, 0);                       // 0F A6 5A 00 00 00 00 00
    pvCommitMs = millis() | 1;
    webLog("[pved] commit sent: wait 1 minute, then switch the PVED off and on");
    // In atan mode keep the firmware on the same vehicle length as the PVED, never switch the mode on by itself
    if (pvSent65112)
    {
        if (claasVehicleLength > 0) setClaasVehicleLength(pvSent65112);
        else webLog("[pved] firmware angle stays linear (AOG counts per degree); for atan mode with L = %u mm use Tractor & GPS", pvSent65112);
        pvSent65112 = 0;
    }
    return true;
}

// 8 bytes in hex (spaces allowed) to 0x18EF13FD. Refused here: write (A2), RestoreDefaults (A4: every parameter back to
// factory values), commit (A6), EnterCalibrationMode (A8) and StartValveAutoCalibration (B6): they have their own
// checked buttons, or none at all.
bool pvedWebRaw(const char* hex, bool moving, const char** why)
{
    if (!pvWriteChecks(moving, why)) return false;
    CAN_message_t msg;
    uint8_t n = 0;
    for (const char* p = hex; *p && n < 17; p++)
    {
        if (*p == ' ') continue;
        if (!isxdigit((unsigned char)*p)) { *why = "hex digits only"; return false; }
        if (n == 16) { n++; break; }
        const uint8_t v = isdigit((unsigned char)*p) ? *p - '0' : (toupper(*p) - 'A' + 10);
        if (n % 2 == 0) msg.buf[n / 2] = v << 4; else msg.buf[n / 2] |= v;
        n++;
    }
    if (n != 16) { *why = "8 bytes = 16 hex digits"; return false; }
    if (msg.buf[0] == 0x0F && (msg.buf[1] == 0xA2 || msg.buf[1] == 0xA4 || msg.buf[1] == 0xA6 || msg.buf[1] == 0xA8 || msg.buf[1] == 0xB6))
    { *why = "write, restore defaults, commit and calibration frames are refused here: use their buttons"; return false; }
    msg.id = 0x18EF13FD;
    msg.flags.extended = true;
    msg.len = 8;
    V_Bus.write(msg);
    webLog("[pved] raw frame sent 0x18EF13FD: %02X %02X %02X %02X %02X %02X %02X %02X",
           msg.buf[0], msg.buf[1], msg.buf[2], msg.buf[3], msg.buf[4], msg.buf[5], msg.buf[6], msg.buf[7]);
    return true;
}

// ---------------------------------------------------------------- valve auto-calibration (experimental)
static bool pvCalChecks(bool moving, const char** why)
{
    if (!pvWriteChecks(moving, why)) return false;
    if (!pvedWebQuiet) { *why = "pause the steering commands to the valve first (session option)"; return false; }
    const uint16_t need[] = { 729, 737, 738, 747 };
    for (uint8_t k = 0; k < 4; k++) if (!pvVal[pvIndex(need[k])].origKnown) { *why = "read all parameters first (backup of the dead-bands)"; return false; }
    return true;
}

bool pvedWebGetMode(const char** why)
{
    if (!pvedWebOn) { *why = "open the PVED session first"; return false; }
    pvSendXid(0xA9, nullptr, 0);
    pvModePollMs = millis();
    return true;
}

bool pvedWebEnterCal(bool moving, const char** why)
{
    if (!pvCalChecks(moving, why)) return false;
    pvEnterMs = millis() | 1;
    pvEnterSendMs = 0;
    webLog("[pved] EnterCalibrationMode (0F A8 58), repeated for up to 20 s until the PVED reports calibration mode");
    return true;
}

// mmi: send it from the MMI controller's address (64005 as read, else its default 252 = 0xFC), as the Danfoss manual
// says ("The valve auto-calibration command shall be sent from the MMI controller"); else from 0xFD, as the
// diagnostics tool does. Which one the PVED takes is not known yet.
static uint8_t pvAutoSa = 0xFD;
bool pvedWebAutoCal(uint16_t xs, uint8_t inc, uint8_t yt, uint16_t tms, bool mmi, bool force, bool moving, const char** why)
{
    if (!pvCalChecks(moving, why)) return false;
    if (pvMode != 0x55 && !force) { *why = "the PVED has not reported calibration mode"; return false; }
    if (xs > 1000 || inc < 1 || inc > 100 || yt < 1 || tms < 50 || tms > 5000) { *why = "value out of range"; return false; }
    const uint8_t d[6] = { (uint8_t)xs, (uint8_t)(xs >> 8), inc, yt, (uint8_t)tms, (uint8_t)(tms >> 8) };
    const int8_t im = pvIndex(64005);
    pvAutoSa = !mmi ? 0xFD : (im >= 0 && pvVal[im].known && pvVal[im].raw < 254) ? (uint8_t)pvVal[im].raw : 0xFC;
    pvSendXidFrom(pvAutoSa, 0xB6, d, 6);
    pvAutoMs = millis() | 1;
    pvAutoXs = xs; pvAutoInc = inc; pvAutoYt = yt; pvAutoTms = tms;
    pvAutoStLen = 0; pvAutoStMs = 0; pvAutoStN = 0;
    webLog("[pved] StartValveAutoCalibration from 0x%02X: XspStartSearch %u, XspIncrementSize %u, YactDiffThreshold %u, TimeOutPeriod %u ms%s",
           pvAutoSa, xs, inc, yt, tms, pvMode == 0x55 ? "" : " (calibration mode NOT confirmed)");
    return true;
}

// StartStopStatus: set 1 = AD1, AD2, AD3 (spool position), Xsp; 0 = stop (a guess)
bool pvedWebStatusSet(uint8_t set, const char** why)
{
    if (!pvedWebOn) { *why = "open the PVED session first"; return false; }
    pvSendXid(0xAB, &set, 1);
    webLog("[pved] StartStopStatus set %u", set);
    return true;
}

// Read the dead-band parameters again (after an auto-calibration)
bool pvedWebReadResults(const char** why)
{
    if (!pvedWebOn) { *why = "open the PVED session first"; return false; }
    const uint16_t nums[] = { 737, 738, 729, 747, 758 };
    if (pvQCount() + 5 >= PVED_Q_N - 1) { *why = "busy, try again in a moment"; return false; }
    for (uint8_t k = 0; k < 5; k++) pvQPush(PVOP_READ, nums[k], 0);
    return true;
}

// ---------------------------------------------------------------- GET /api/pved
void pvedWebJson(Json& j)
{
    const uint32_t now = millis();
    static const char* const tn[] = { "u8", "u16", "s16", "u32" };
    char b[12];
    j.raw("{");
    j.bo("ok", true);
    j.bo("on", pvedWebOn);
    j.bo("quiet", pvedWebQuiet);
    j.i("since", pvedWebOn ? (long)((now - pvSinceMs) / 1000) : -1);
    j.u("queue", pvQCount() + (pvStage ? 1 : 0));
    j.i("cur", pvStage ? pvCur.num : -1);
    j.str("curKind", pvStage == 2 ? "write" : pvStage == 1 ? "read" : "");
    j.str("mode", pvModeName(pvMode));
    j.i("modeAgo", pvModeMs ? (long)((now - pvModeMs) / 1000) : -1);
    j.i("commitAgo", pvCommitMs ? (long)((now - pvCommitMs) / 1000) : -1);
    j.obj("cal");
    j.bo("entering", pvEnterMs != 0);
    j.i("enterLeft", pvEnterMs ? (long)((PVED_ENTER_MS - (now - pvEnterMs)) / 1000) : -1);
    j.i("autoAgo", pvAutoMs ? (long)((now - pvAutoMs) / 1000) : -1);
    if (pvAutoMs) { j.u("xs", pvAutoXs); j.u("inc", pvAutoInc); j.u("yt", pvAutoYt); j.u("tms", pvAutoTms); j.u("sa", pvAutoSa); }
    if (pvAutoStMs)
    {
        char h[3 * 8 + 1];
        for (uint8_t k = 0; k < pvAutoStLen; k++) snprintf(h + 3 * k, 4, "%02X ", pvAutoSt[k]);
        h[pvAutoStLen ? 3 * pvAutoStLen - 1 : 0] = '\0';
        j.str("st", h);
        j.i("stAgo", (long)((now - pvAutoStMs) / 1000));
        j.u("stN", pvAutoStN);
    }
    j.close();
    j.i("valve", steeringValveReady);
    j.u("est", estCurve);
    j.arr("params");
    for (uint8_t i = 0; i < PV_N; i++)
    {
        const PvedVal& v = pvVal[i];
        j.open();
        j.u("n", pvDefs[i].num);
        j.str("t", tn[pvDefs[i].type]);
        j.str("name", pvDefs[i].name);
        if (pvDefs[i].risky) j.bo("risky", true);
        if (pvDefs[i].ro) j.bo("ro", true);
        j.bo("k", v.known);
        if (v.known) { j.u("raw", v.raw); pvBytes(v.raw, b); j.str("hex", b); j.i("age", (long)((now - v.ms) / 1000)); }
        if (v.origKnown) j.u("orig", v.orig);
        if (v.noAnswer) j.bo("na", true);
        j.close();
    }
    j.closeArr();
    j.obj("any");
    j.u("n", pvAnyNum);
    j.bo("k", pvAnyKnown);
    j.bo("na", pvAnyNoAnswer);
    if (pvAnyKnown) { j.u("raw", pvAnyRaw); pvBytes(pvAnyRaw, b); j.str("hex", b); j.i("age", (long)((now - pvAnyMs) / 1000)); }
    j.close();
    j.arr("frames");
    for (uint8_t i = 0; i < PVED_FRAMES; i++)
    {
        const PvedFrame& f = pvF[i];
        if (!f.n) continue;
        char id[12], hex[3 * 8 + 1];
        snprintf(id, sizeof(id), "%08lX", (unsigned long)f.id);
        for (uint8_t k = 0; k < f.len; k++) snprintf(hex + 3 * k, 4, "%02X ", f.d[k]);
        hex[f.len ? 3 * f.len - 1 : 0] = '\0';
        j.open(); j.str("id", id); j.str("hex", hex); j.u("n", f.n); j.i("age", (long)((now - f.ms) / 1000)); j.close();
    }
    j.closeArr();
    j.raw("}");
}
