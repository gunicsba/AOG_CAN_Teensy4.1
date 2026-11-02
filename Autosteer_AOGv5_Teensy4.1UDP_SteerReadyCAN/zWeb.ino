/* zWeb.ino - the module's web page: http://<subnet>.126/ (the subnet set by AgIO, e.g. http://192.168.5.126/).
   Ported from renatorgr/Boards, Firmware/v4.5_AiO_WEB/zWeb.ino and zEthernet.ino (written 2026 by
   Claude (Anthropic) for renatorgr).
   AOG_CAN_Teensy4.1, 2026: the pages of this firmware (status, tractor brand and GPS mode as in the serial
   service tool, Claas wheel angle mode, TM171 settings and calibration, firmware update, restart, messages).
   Answers are sent a piece at a time from loop(), only what the socket takes, so the 40 ms CAN steering cycle
   keeps running while a browser loads the page. The network stack is polled from loop() (zNet.h).
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// One request at a time, answered with "Connection: close". There is no password: anyone on the
// module's network can open the page. What could disturb steering (brand / GPS mode change and restart,
// TM171 write and calibration, firmware update, opening the PVED session) is refused while autosteer is
// engaged; the firmware update, the gyro calibrations and PVED writes / commit also while the tractor moves.

#include <stdarg.h>
#include "zWebPage.h"            // the page itself (HTML, CSS, JavaScript), kept in flash

#define WEB_PORT          80
#define WEB_HEAD_MAX      2048    // request line + headers + small body
#define WEB_BODY_MAX      512
#define WEB_TIMEOUT_MS    3000
#define WEB_FIRST_BYTE_MS 500     // a connection with no request yet (browser "preconnect") is closed after this
#define WEB_MOVING_KMH    1.0f    // no firmware update or gyro calibration above this speed
#define WEB_LOG_LINES     40
#define WEB_LOG_LEN       200

EthernetServer webServer(WEB_PORT);
static bool webRunning = false;

enum { WEB_IDLE, WEB_READ, WEB_SEND };
enum { WEB_AFTER_NONE, WEB_AFTER_RESTART };

static struct
{
    EthernetClient client;
    uint8_t  state;
    uint32_t startMs, lastMs;
    char     head[WEB_HEAD_MAX + 1];
    uint16_t len;
    int16_t  bodyAt;              // index of the body in head[] (-1: headers not complete yet)
    char     method[8];
    char     path[48];
    char     query[96];
    uint32_t contentLength;
    // the answer, sent a piece at a time by webSendSome()
    char        hdr[200];
    uint16_t    hdrLen, hdrSent;
    const char* body;
    uint32_t    bodyLen, bodySent;
    uint8_t     after;            // what to do once the answer is out (restart)
} rq;

static char webBuf[8192];        // JSON answers (stays valid until the answer is sent: one request at a time)
static uint32_t webReqN = 0, webErrN = 0, webDropN = 0;

// ---------------------------------------------------------------- messages (Messages tab, also USB serial)
struct WebLogLine { uint32_t seq, ms; char text[WEB_LOG_LEN]; };
static WebLogLine webLogLines[WEB_LOG_LINES];
static uint32_t webLogNext = 1;

void webLog(const char* fmt, ...)
{
    WebLogLine& l = webLogLines[webLogNext % WEB_LOG_LINES];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(l.text, sizeof(l.text), fmt, ap);
    va_end(ap);
    l.ms = millis();
    l.seq = webLogNext++;
    Serial.println(l.text);
}

// ---------------------------------------------------------------- the network stack, from loop() (zNet.h)
static bool     netPollOn = false;
static uint32_t netPollUs = 0;

void netPollFromLoop()
{
    netPollTimer(NetPollTimerTag())->end();       // no more fnet_poll() in the timer interrupt
    netPollOn = fnet_netif_get_default() != 0;    // FNET started (Ethernet.begin() worked)
    netPollUs = micros();
    if (netPollOn) fnet_poll();
}

void netPoll()
{
    if (!netPollOn) return;
    const uint32_t now = micros();
    if (now - netPollUs < 1000) return;           // once a millisecond, as the library's timer did
    netPollUs = now;
    fnet_poll();
}

void netDelay(uint32_t ms)
{
    const uint32_t t0 = millis();
    while (millis() - t0 < ms)
    {
        netPoll();
        delayMicroseconds(100);
    }
}

// ---------------------------------------------------------------- steering state
static bool webEngaged() { return watchdogTimer < WATCHDOG_THRESHOLD; }

// The speed from AgOpenGPS (PGN 254) while it is connected, or the receiver's own (VTG, Panda mode)
static float webKmh()
{
    const uint32_t now = millis();
    float kmh = 0;
    if (aogLastMs && now - aogLastMs < 3000) kmh = gpsSpeed;
    if (ggaLastMs && now - ggaLastMs < 3000) { const float v = atof(speedKnots) * 1.852f; if (v > kmh) kmh = v; }
    return kmh;
}
static bool webMoving() { return webKmh() > WEB_MOVING_KMH; }

// Steering off, as when AgOpenGPS stops steering: before a restart or a firmware update
static void webSteerOff()
{
    watchdogTimer = WATCHDOG_FORCE_VALUE;
    intendToSteer = 0;
    engageCAN = 0;
    setCurve = estCurve;
    pwmDrive = 0;
    if (Brand != 7) motorDrive();
    digitalWrite(PWM2_RPWM, 0);
    VBus_Send();                                  // "not steering" to the valve
}

// While the firmware update has the loop: keep telling the valve "not steering" every 40 ms
static void webValveKeepAlive()
{
    static uint32_t lastMs = 0;
    if (millis() - lastMs < LOOP_TIME) return;
    lastMs = millis();
    intendToSteer = 0;
    setCurve = estCurve;
    VBus_Send();
}

static const char* brandName(uint8_t b)
{
    switch (b)
    {
        case 0: return "Claas";
        case 1: return "Valtra / Massey";
        case 2: return "CaseIH / New Holland";
        case 3: return "Fendt SCR, S4, Gen6";
        case 4: return "JCB";
        case 5: return "FendtOne";
        case 6: return "Lindner";
        case 7: return "AgOpenGPS";
        case 8: return "Cat MT Late";
        case 9: return "Cat MT Early";
        default: return "not set";
    }
}

static const char* gpsModeName(uint8_t m)
{
    switch (m)
    {
        case 1: return "Forwarding @ 115200";
        case 2: return "Forwarding @ 460800";
        case 3: return "Panda @ 115200";
        case 4: return "Panda @ 460800";
        default: return "not set";
    }
}

// Bytes per second, updated once a second
static uint32_t gpsBps = 0, ntripBps = 0, radioBps = 0;
static void webRates()
{
    static uint32_t ms = 0, g = 0, n = 0, r = 0;
    const uint32_t now = millis();
    if (now - ms < 1000) return;
    const uint32_t dt = now - ms;
    ms = now;
    gpsBps   = (uint32_t)((uint64_t)(gpsRxBytes - g) * 1000 / dt);   g = gpsRxBytes;
    ntripBps = (uint32_t)((uint64_t)(ntripRxBytes - n) * 1000 / dt); n = ntripRxBytes;
    radioBps = (uint32_t)((uint64_t)(radioRxBytes - r) * 1000 / dt); r = radioRxBytes;
}

// ---------------------------------------------------------------- answers
static void webFinish()
{
    rq.client.stop();                             // graceful: the queued data still goes out, then FIN
    rq.state = WEB_IDLE;
    rq.after = WEB_AFTER_NONE;
}

static void webReply(int code, const char* type, const char* body, uint32_t len)
{
    const char* text = code == 200 ? "OK" : code == 400 ? "Bad Request" : code == 404 ? "Not Found" :
                       code == 409 ? "Conflict" : code == 413 ? "Payload Too Large" : "Error";
    rq.hdrLen = snprintf(rq.hdr, sizeof(rq.hdr),
                         "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %lu\r\n"
                         "Cache-Control: no-store\r\nConnection: close\r\n\r\n",
                         code, text, type, (unsigned long)len);
    rq.hdrSent = 0;
    rq.body = body;
    rq.bodyLen = len;
    rq.bodySent = 0;
    rq.state = WEB_SEND;
    rq.lastMs = millis();
}

static void webReplyJson(int code, const char* json) { webReply(code, "application/json", json, strlen(json)); }

static void webReplyError(int code, const char* msg)
{
    Json j(webBuf, sizeof(webBuf));
    j.raw("{");
    j.bo("ok", false);
    j.str("error", msg);
    j.raw("}");
    webErrN++;
    webReplyJson(code, webBuf);
}

// Send what the socket takes now, never more: NativeEthernet's write waits in a loop for room otherwise,
// and with the stack polled from loop() that room would never come.
static void webSendSome()
{
    EthernetClient& c = rq.client;
    for (uint8_t k = 0; k < 8; k++)
    {
        const bool inHdr = rq.hdrSent < rq.hdrLen;
        const char* p = inHdr ? rq.hdr + rq.hdrSent : rq.body + rq.bodySent;
        uint32_t n    = inHdr ? (uint32_t)(rq.hdrLen - rq.hdrSent) : rq.bodyLen - rq.bodySent;
        if (n == 0)
        {
            const uint8_t after = rq.after;
            webFinish();
            if (after == WEB_AFTER_RESTART)
            {
                netDelay(300);                    // let the answer leave
                SCB_AIRCR = 0x05FA0004;           // Teensy reset
                for (;;) {}
            }
            return;
        }
        const int room = c.availableForWrite();
        if (room <= 0) break;
        if (n > (uint32_t)room) n = room;
        if (n > 1460) n = 1460;
        const size_t w = c.write((const uint8_t*)p, n);
        if (w == 0) break;
        if (inHdr) rq.hdrSent += w; else rq.bodySent += w;
        rq.lastMs = millis();
    }
    if (millis() - rq.lastMs > WEB_TIMEOUT_MS || !c.connected()) { webDropN++; webFinish(); }
}

// ---------------------------------------------------------------- request helpers
// Value of "name" in a query string / urlencoded body, decoded. False if absent.
static bool formValue(const char* src, const char* name, char* out, size_t outSize)
{
    const size_t nl = strlen(name);
    const char* p = src;
    while (p && *p)
    {
        if (!strncmp(p, name, nl) && p[nl] == '=')
        {
            p += nl + 1;
            size_t k = 0;
            while (*p && *p != '&' && k < outSize - 1)
            {
                if (*p == '+') { out[k++] = ' '; p++; }
                else if (*p == '%' && hexNib(p[1]) >= 0 && hexNib(p[2]) >= 0) { out[k++] = (char)((hexNib(p[1]) << 4) | hexNib(p[2])); p += 3; }
                else out[k++] = *p++;
            }
            out[k] = '\0';
            return true;
        }
        p = strchr(p, '&');
        if (p) p++;
    }
    return false;
}

// Strict integer: the whole text must be a number
static bool parseIntStrict(const char* s, long& v)
{
    if (!*s) return false;
    char* end;
    v = strtol(s, &end, 10);
    return *end == '\0';
}

// A 64-bit whole number (the PVED's 32-bit unsigned parameters do not fit a long)
static bool parseI64Strict(const char* s, int64_t& v)
{
    if (!*s) return false;
    char* end;
    v = strtoll(s, &end, 10);
    return *end == '\0';
}

// A number field of a form: absent = dflt; present = must be a whole number in lo..hi
static bool formNum(const char* body, const char* name, long dflt, long lo, long hi, long& v)
{
    char t[16];
    if (!formValue(body, name, t, sizeof(t))) { v = dflt; return true; }
    return parseIntStrict(t, v) && v >= lo && v <= hi;
}

// ---------------------------------------------------------------- GET /api/status
static void apiStatus()
{
    const uint32_t now = millis();
    pvedWebSeen();                               // a page is open: keeps the PVED session alive
    Json j(webBuf, sizeof(webBuf));
    j.raw("{");
    j.str("fw", FW_NAME);
    j.str("ver", inoVersion.c_str() + 2);       // without the leading "\r\n"
    j.str("built", __DATE__ " " __TIME__);
    j.str("build", fwBuildInfo);
    j.str("marker", otaMarker);
    j.str("target", otaFlashId);
    j.u("up", now / 1000);
    {
        char t[20];
        IPAddress a = Ethernet.localIP();
        snprintf(t, sizeof(t), "%u.%u.%u.%u", (unsigned)a[0], (unsigned)a[1], (unsigned)a[2], (unsigned)a[3]);
        j.str("ip", t);
    }
    j.bo("link", Ethernet.linkStatus() == LinkON);
    j.num("cpuC", tempmonGetTemp(), 1);
    j.bo("engaged", webEngaged());
    j.bo("moving", webMoving());
    j.num("kmh", webKmh(), 1);

    j.obj("aog");
    j.bo("seen", aogLastMs != 0 && now - aogLastMs < 3000);
    j.i("ago", aogLastMs ? (long)((now - aogLastMs) / 1000) : -1);
    j.num("kmh", gpsSpeed, 1);
    j.i("guidance", guidanceStatus);
    j.num("setAngle", steerAngleSetPoint, 2);
    j.close();

    j.obj("steer");
    j.i("brand", Brand);
    j.str("brandName", brandName(Brand));
    j.i("valve", steeringValveReady);
    j.bo("ready", steeringValveReady == 20 || steeringValveReady == 16);
    j.bo("intend", intendToSteer);
    j.num("angle", steerAngleActual, 2);
    j.u("estCurve", estCurve);
    j.u("setCurve", setCurve);
    j.i("pwm", pwmDisplay);
    j.bo("steerSw", steerSwitch == 0);
    j.bo("work", workSwitch == 0);
    j.bo("engageCAN", engageCAN);
    j.i("claasLen", claasVehicleLength);
    j.bo("diag", steerDiagOn);
    j.close();

    j.obj("ports");                              // board setup (zBoard.ino), 0 = none
    j.u("gnss", portGnss);
    j.u("imu", portImu);
    j.u("radio", portRadio);
    j.u("imuTx", boardTxPin(portImu));
    j.close();

    j.obj("gps");
    j.i("mode", gpsMode);
    j.str("modeName", gpsModeName(gpsMode));
    j.u("bps", gpsBps);
    j.u("ntripBps", ntripBps);
    j.u("radioBps", radioBps);
    const bool gga = ggaLastMs != 0 && now - ggaLastMs < 3000;
    j.bo("gga", gga);
    if (gga)
    {
        j.str("fix", fixQuality);
        j.str("sats", numSats);
        j.str("hdop", HDOP);
        j.str("age", ageDGPS);
        j.num("kmh", atof(speedKnots) * 1.852, 1);
    }
    j.close();

    j.obj("imu");
    j.str("type", useBNO08x ? "BNO08x" : useTM171 ? "TM171" : "");
    j.bo("yAxis", steerConfig.IsUseY_Axis);
    if (useTM171)
    {
        j.num("roll", RollV.fValue, 2);
        j.num("pitch", PitchV.fValue, 2);
        j.num("yaw", YawV.fValue, 2);
        j.bo("fresh", TM171lastData < 500);
        j.u("rate", TM171rpyRate);
        j.u("crcErr", TM171crcErrors);
        j.u("id", tm171Id);
        if (TM171statusMs && now - TM171statusMs < 5000) { j.num("tempC", TemperatureV.fValue, 1); j.i("qos", qos); }
        tm171SettingJson(j);
        tm171CalJson(j);
    }
    else if (useBNO08x)
    {
        j.num("roll", roll / 10.0, 2);
        j.num("pitch", pitch / 10.0, 2);
        j.num("yaw", yaw / 10.0, 2);
    }
    j.close();

    j.obj("pved");                               // the PVED tab session (zPved.ino)
    j.bo("on", pvedWebOn);
    j.bo("quiet", pvedWebQuiet);
    j.close();

    j.obj("web");
    j.u("req", webReqN);
    j.u("err", webErrN);
    j.u("drop", webDropN);
    j.close();
    j.raw("}");
    webReplyJson(200, webBuf);
}

// ---------------------------------------------------------------- GET /api/log?after=N
static void apiLog()
{
    char v[16];
    long n = 0;
    const uint32_t after = (formValue(rq.query, "after", v, sizeof(v)) && parseIntStrict(v, n) && n > 0) ? (uint32_t)n : 0;
    Json j(webBuf, sizeof(webBuf));
    j.raw("{");
    j.u("next", webLogNext);
    j.u("now", millis());
    j.arr("lines");
    const uint32_t first = webLogNext > WEB_LOG_LINES ? webLogNext - WEB_LOG_LINES : 1;
    for (uint32_t s = (after + 1 > first ? after + 1 : first); s < webLogNext; s++)
    {
        const WebLogLine& l = webLogLines[s % WEB_LOG_LINES];
        if (l.seq != s) continue;
        if (j.n + WEB_LOG_LEN + 64 > j.cap) break;          // the page asks again for the rest
        j.open(); j.u("s", l.seq); j.u("t", l.ms); j.str("x", l.text); j.close();
    }
    j.closeArr();
    j.raw("}");
    webReplyJson(200, webBuf);
}

// ---------------------------------------------------------------- /api/config: the serial service tool's settings
static void apiConfigGet()
{
    Json j(webBuf, sizeof(webBuf));
    j.raw("{");
    j.bo("ok", true);
    j.i("brand", Brand);
    j.i("gpsMode", gpsMode);
    j.i("claasLen", claasVehicleLength);
    j.i("gnssPort", portGnss);
    j.i("imuPort", portImu);
    j.i("radioPort", portRadio);
    j.raw("}");
    webReplyJson(200, webBuf);
}

// brand (0-9) and gpsMode (1-4): saved in the EEPROM (70, 72) and the Teensy restarts, as the service tool
// does. claasLen (0 = linear, 500-10000 mm = atan mode, EEPROM 76): taken at once, no restart. gnssPort, imuPort,
// radioPort (board setup, zBoard.ino, EEPROM 200-203): saved, and the Teensy restarts.
static void apiConfigPost(const char* body)
{
    long brand, mode, len;
    if (!formNum(body, "brand", Brand, 0, 9, brand))        { webReplyError(400, "invalid brand"); return; }
    if (!formNum(body, "gpsMode", gpsMode, 1, 4, mode))     { webReplyError(400, "invalid GPS mode"); return; }
    if (!formNum(body, "claasLen", claasVehicleLength, 0, 10000, len) || (len != 0 && len < 500))
                                                            { webReplyError(400, "vehicle length: 0 (linear) or 500 - 10000 mm"); return; }
    long pg, pi, pr;
    if (!formNum(body, "gnssPort", portGnss, 0, 8, pg) || !formNum(body, "imuPort", portImu, 0, 8, pi) ||
        !formNum(body, "radioPort", portRadio, 0, 8, pr))  { webReplyError(400, "invalid serial port"); return; }
    const char* portWhy = boardPortsCheck((uint8_t)pg, (uint8_t)pi, (uint8_t)pr);
    if (portWhy) { webReplyError(400, portWhy); return; }
    const bool portsChanged = pg != portGnss || pi != portImu || pr != portRadio;
    const bool restart = brand != Brand || mode != gpsMode || portsChanged;
    if (restart && webEngaged()) { webReplyError(409, "autosteer is engaged - disengage it first"); return; }
    if (len != claasVehicleLength)
    {
        setClaasVehicleLength((uint16_t)len);
        if (len) webLog("[web] Claas angle: atan mode, vehicle length %ld mm (must match PVED 65112)", len);
        else     webLog("[web] Claas angle: linear, AOG counts per degree");
    }
    if (restart)
    {
        EEPROM.update(70, (uint8_t)brand);
        EEPROM.update(72, (uint8_t)mode);
        if (portsChanged) boardPortsSave((uint8_t)pg, (uint8_t)pi, (uint8_t)pr);
        webLog("[web] brand %s, GPS %s, ports GNSS Serial%ld IMU %ld radio %ld (0 = none) saved - restarting",
               brandName(brand), gpsModeName(mode), pg, pi, pr);
        webSteerOff();
        rq.after = WEB_AFTER_RESTART;
    }
    Json j(webBuf, sizeof(webBuf));
    j.raw("{");
    j.bo("ok", true);
    j.bo("restart", restart);
    j.raw("}");
    webReplyJson(200, webBuf);
}

// ---------------------------------------------------------------- POST /api/imu (TM171.ino)
// act=setread | setwrite (the settings form) | cal (c = calibration code, p2 / p3 its parameters)
static void apiImuPost(const char* body)
{
    char act[16], v[16];
    if (!formValue(body, "act", act, sizeof(act))) { webReplyError(400, "no act"); return; }
    if (!useTM171) { webReplyError(409, "no TM171 found at start-up"); return; }
    const char* why = "";
    if (!strcmp(act, "setread")) tm171SettingRead();
    else if (!strcmp(act, "setwrite"))
    {
        if (webEngaged()) { webReplyError(409, "autosteer is engaged - disengage it first"); return; }
        auto flag = [&](const char* n) { return formValue(body, n, v, sizeof(v)) && !strcmp(v, "1"); };
        TmSetForm f;
        f.gyro = flag("gyro"); f.acc = flag("acc"); f.mag = flag("mag");
        f.gyroErr = flag("gyroErr"); f.selfAdapt = flag("selfAdapt"); f.status = flag("status"); f.save = flag("save");
        long boot, acc, mg, silent, odr;
        if (!formNum(body, "boot", 0, 0, 2, boot) || !formNum(body, "accGain", TM171_AOG_ACC, 1, 2000, acc) ||
            !formNum(body, "magGain", TM171_AOG_MAG, 1, 2000, mg) || !formNum(body, "silent", 0, 0, 65, silent) ||
            !formNum(body, "odr", 0, 0, 1000, odr))
        { webReplyError(400, "value out of range"); return; }
        f.boot = (uint8_t)boot; f.accGain = (uint16_t)acc; f.magGain = (uint16_t)mg;
        f.silent = (uint16_t)silent; f.odr = (uint16_t)odr;
        if (!tm171SettingWrite(f, &why)) { webReplyError(409, why); return; }
    }
    else if (!strcmp(act, "cal"))
    {
        char code[16];
        long p2, p3;
        if (!formValue(body, "c", code, sizeof(code))) { webReplyError(400, "no calibration"); return; }
        if (!formNum(body, "p2", 0, 0, 360000, p2) || !formNum(body, "p3", 0, 0, 10000, p3)) { webReplyError(400, "value out of range"); return; }
        if (!tm171CalStart(code, (uint32_t)p2, (uint32_t)p3, webEngaged(), webMoving(), &why)) { webReplyError(409, why); return; }
    }
    else { webReplyError(400, "unknown act"); return; }
    webReplyJson(200, "{\"ok\":true}");
}

// ---------------------------------------------------------------- /api/pved: the PVED tab (zPved.ino)
static void apiPvedGet()
{
    pvedWebSeen();
    Json j(webBuf, sizeof(webBuf));
    pvedWebJson(j);
    webReplyJson(200, webBuf);
}

// act = start (quiet=1: steering commands paused) | stop | readall | read (n) | write (n, v) | hack | restore |
// commit | raw (hex) | experimental valve auto-calibration: getmode | entercal | autocal (xs, inc, yt, tms, from, force) |
// statusset (set) | readres
static void apiPvedPost(const char* body)
{
    char act[16], v[40];
    if (!formValue(body, "act", act, sizeof(act))) { webReplyError(400, "no act"); return; }
    pvedWebSeen();
    const bool moving = webMoving();
    const char* why = "";
    bool ok = true;
    long n = 0;
    if (!strcmp(act, "start"))        ok = pvedWebStart(!(formValue(body, "quiet", v, sizeof(v)) && !strcmp(v, "0")), webEngaged(), &why);
    else if (!strcmp(act, "stop"))    pvedWebStop("web page");
    else if (!strcmp(act, "readall")) ok = pvedWebReadAll(&why);
    else if (!strcmp(act, "read"))
    {
        if (!formNum(body, "n", -1, 1, 65535, n) || n < 1) { webReplyError(400, "parameter number: 1 - 65535"); return; }
        ok = pvedWebRead((uint16_t)n, &why);
    }
    else if (!strcmp(act, "write"))
    {
        int64_t val;
        if (!formNum(body, "n", -1, 1, 65535, n) || n < 1) { webReplyError(400, "parameter number: 1 - 65535"); return; }
        if (!formValue(body, "v", v, sizeof(v)) || !parseI64Strict(v, val)) { webReplyError(400, "value: a whole number"); return; }
        ok = pvedWebWrite((uint16_t)n, val, moving, &why);
    }
    else if (!strcmp(act, "hack"))    ok = pvedWebClaasHack(moving, &why);
    else if (!strcmp(act, "restore")) ok = pvedWebRestore(moving, &why);
    else if (!strcmp(act, "commit"))  ok = pvedWebCommit(moving, &why);
    else if (!strcmp(act, "raw"))
    {
        if (!formValue(body, "hex", v, sizeof(v))) { webReplyError(400, "no bytes"); return; }
        ok = pvedWebRaw(v, moving, &why);
    }
    else if (!strcmp(act, "getmode"))  ok = pvedWebGetMode(&why);
    else if (!strcmp(act, "entercal")) ok = pvedWebEnterCal(moving, &why);
    else if (!strcmp(act, "autocal"))
    {
        long xs, inc, yt, tms;
        if (!formNum(body, "xs", 35, 0, 1000, xs) || !formNum(body, "inc", 5, 1, 100, inc) ||
            !formNum(body, "yt", 20, 1, 255, yt) || !formNum(body, "tms", 500, 50, 5000, tms))
        { webReplyError(400, "value out of range"); return; }
        const bool force = formValue(body, "force", v, sizeof(v)) && !strcmp(v, "1");
        const bool mmi = !(formValue(body, "from", v, sizeof(v)) && !strcmp(v, "tool"));
        ok = pvedWebAutoCal((uint16_t)xs, (uint8_t)inc, (uint8_t)yt, (uint16_t)tms, mmi, force, moving, &why);
    }
    else if (!strcmp(act, "statusset"))
    {
        if (!formNum(body, "set", 1, 0, 255, n)) { webReplyError(400, "status set: 0 - 255"); return; }
        ok = pvedWebStatusSet((uint8_t)n, &why);
    }
    else if (!strcmp(act, "readres")) ok = pvedWebReadResults(&why);
    else { webReplyError(400, "unknown act"); return; }
    if (!ok) { webReplyError(409, why); return; }
    webReplyJson(200, "{\"ok\":true}");
}

// ---------------------------------------------------------------- POST /api/ota (zOTA.ino)
// Takes over the loop until the whole file is in (or fails); refused while autosteer is engaged or the
// tractor moves. The valve is told "not steering" every 40 ms meanwhile.
// A refused update is answered only once the browser has sent the whole file (read and dropped): an answer
// while it is still sending cuts the connection, and the page could only say "the connection failed".
static void otaDrain(EthernetClient& c, uint32_t got)
{
    static uint8_t buf[512];
    uint32_t last = millis();
    while (got < rq.contentLength && c.connected() && millis() - last < 5000)
    {
        netPoll();
        webValveKeepAlive();
        const int n = c.available();
        if (n > 0)
        {
            const int r = c.read(buf, n < (int)sizeof(buf) ? n : (int)sizeof(buf));
            if (r > 0) { got += r; last = millis(); }
        }
        else netDelay(1);
    }
}

static void otaRefuse(int code, const char* why, uint32_t got)
{
    webLog("[web] firmware update REFUSED: %s (old firmware kept)", why);
    if (rq.contentLength <= 8000000) otaDrain(rq.client, got);
    webReplyError(code, why);
}

static void apiOta(const uint8_t* pre, size_t preLen)
{
    EthernetClient& c = rq.client;
    char v[8];
    const bool force = formValue(rq.query, "force", v, sizeof(v)) && !strcmp(v, "1");
    if (webEngaged()) { otaRefuse(409, "autosteer is engaged - disengage it first", preLen); return; }
    if (webMoving())  { otaRefuse(409, "the tractor is moving - stop first", preLen); return; }
    if (rq.contentLength < 1000 || rq.contentLength > 8000000) { otaRefuse(400, "no file, or file size not plausible", preLen); return; }

    webSteerOff();
    webLog("[web] firmware update: receiving %lu bytes, steering off", (unsigned long)rq.contentLength);
    const char* err = otaBegin();
    if (!err) err = otaFeed(pre, preLen);
    uint32_t got = preLen, last = millis();
    static uint8_t buf[2048];
    while (!err && got < rq.contentLength)
    {
        netPoll();                                         // the network stack: ACKs, window
        webValveKeepAlive();
        const int n = c.available();
        if (n > 0)
        {
            const int r = c.read(buf, n < (int)sizeof(buf) ? n : (int)sizeof(buf));
            if (r > 0)
            {
                err = otaFeed(buf, r);
                got += r;
                last = millis();
            }
        }
        else if (!c.connected())              err = "connection lost during the upload";
        else if (millis() - last > 15000)     err = "upload stopped (timeout)";
        else                                  netDelay(1);
    }
    uint32_t size = 0;
    if (!err) err = otaFinish(force, &size);
    if (err) { otaRefuse(400, err, got); return; }
    webLog("[web] firmware update: %lu bytes checked, installing and restarting", (unsigned long)size);
    snprintf(webBuf, sizeof(webBuf), "{\"ok\":true,\"size\":%lu}", (unsigned long)size);
    webReplyJson(200, webBuf);
    const uint32_t t0 = millis();
    while (rq.state == WEB_SEND && millis() - t0 < WEB_TIMEOUT_MS) { netPoll(); webValveKeepAlive(); webSendSome(); }
    if (rq.state != WEB_IDLE) webFinish();
    netDelay(400);                                         // let the answer leave
    otaInstall(size);                                      // does not return
}

// ---------------------------------------------------------------- request handling
static void webDispatch()
{
    webReqN++;
    const bool get = !strcmp(rq.method, "GET"), post = !strcmp(rq.method, "POST");
    const char* body = rq.bodyAt >= 0 ? rq.head + rq.bodyAt : "";
    if      (get  && (!strcmp(rq.path, "/") || !strcmp(rq.path, "/index.html")))
        webReply(200, "text/html; charset=utf-8", WEB_PAGE, sizeof(WEB_PAGE) - 1);
    else if (get  && !strcmp(rq.path, "/api/status")) apiStatus();
    else if (get  && !strcmp(rq.path, "/api/log"))    apiLog();
    else if (get  && !strcmp(rq.path, "/api/config")) apiConfigGet();
    else if (post && !strcmp(rq.path, "/api/config")) apiConfigPost(body);
    else if (post && !strcmp(rq.path, "/api/imu"))    apiImuPost(body);
    else if (get  && !strcmp(rq.path, "/api/pved"))   apiPvedGet();
    else if (post && !strcmp(rq.path, "/api/pved"))   apiPvedPost(body);
    else if (post && !strcmp(rq.path, "/api/reboot"))
    {
        if (webEngaged()) { webReplyError(409, "autosteer is engaged - disengage it first"); return; }
        webLog("[web] restart requested");
        webSteerOff();
        rq.after = WEB_AFTER_RESTART;
        webReplyJson(200, "{\"ok\":true}");
    }
    else webReply(404, "text/plain", "Not found", 9);
}

// Request line and headers are complete in rq.head[0 .. headEnd)
static bool webParseHead(uint16_t headEnd)
{
    rq.head[headEnd] = '\0';
    char target[160] = {};
    if (sscanf(rq.head, "%7s %159s", rq.method, target) != 2) return false;
    char* q = strchr(target, '?');
    if (q) { *q = '\0'; const size_t k = strnlen(q + 1, sizeof(rq.query) - 1); memcpy(rq.query, q + 1, k); rq.query[k] = '\0'; }
    { const size_t k = strnlen(target, sizeof(rq.path) - 1); memcpy(rq.path, target, k); rq.path[k] = '\0'; }
    rq.contentLength = 0;
    for (char* l = strstr(rq.head, "\r\n"); l; l = strstr(l + 2, "\r\n"))
        if (!strncasecmp(l + 2, "Content-Length:", 15)) rq.contentLength = strtoul(l + 17, nullptr, 10);
    return true;
}

static void webRead()
{
    EthernetClient& c = rq.client;
    if (millis() - rq.startMs > WEB_TIMEOUT_MS ||
        (rq.len == 0 && millis() - rq.startMs > WEB_FIRST_BYTE_MS)) { if (rq.len) webDropN++; webFinish(); return; }

    const int n = c.available();
    if (n <= 0)
    {
        if (!c.connected()) { if (rq.len) webDropN++; webFinish(); }
        return;
    }
    const int room = WEB_HEAD_MAX - rq.len;
    if (room <= 0) { webReply(413, "text/plain", "Too large", 9); return; }
    const int r = c.read((uint8_t*)rq.head + rq.len, n < room ? n : room);
    if (r <= 0) return;
    rq.len += r;
    rq.head[rq.len] = '\0';

    if (rq.bodyAt < 0)
    {
        char* end = strstr(rq.head, "\r\n\r\n");
        if (!end) return;                                   // headers not complete yet
        const uint16_t headEnd = (uint16_t)(end - rq.head);
        rq.bodyAt = headEnd + 4;
        const uint16_t have = rq.len - rq.bodyAt;
        char saved[WEB_HEAD_MAX];                           // webParseHead puts a '\0' at headEnd
        memcpy(saved, rq.head + rq.bodyAt, have);
        if (!webParseHead(headEnd)) { webReply(400, "text/plain", "Bad request", 11); return; }
        memcpy(rq.head + rq.bodyAt, saved, have);
        rq.head[rq.len] = '\0';
        if (!strcmp(rq.method, "POST") && !strcmp(rq.path, "/api/ota"))
        {
            apiOta((const uint8_t*)rq.head + rq.bodyAt, have);
            return;
        }
        if (rq.contentLength > WEB_BODY_MAX) { webReply(413, "text/plain", "Too large", 9); return; }
    }
    if ((uint32_t)(rq.len - rq.bodyAt) >= rq.contentLength)
    {
        rq.head[rq.bodyAt + rq.contentLength] = '\0';
        webDispatch();
    }
}

// From setup(), after Ethernet.begin()
void webSetup()
{
    webServer.begin();
    webRunning = true;
    IPAddress a = Ethernet.localIP();
    webLog("[web] web page: http://%u.%u.%u.%u/", (unsigned)a[0], (unsigned)a[1], (unsigned)a[2], (unsigned)a[3]);
}

// From loop()
void webLoop()
{
    webRates();
    if (useTM171) tm171Loop(!webEngaged() && !webMoving());
    pvedWebLoop(webMoving());
    if (!webRunning) return;
    if (rq.state == WEB_IDLE)
    {
        EthernetClient c = webServer.accept();
        if (!c) return;
        rq.client = c;
        rq.state = WEB_READ;
        rq.startMs = millis();
        rq.len = 0;
        rq.bodyAt = -1;
        rq.method[0] = rq.path[0] = rq.query[0] = '\0';
        rq.contentLength = 0;
        rq.after = WEB_AFTER_NONE;
    }
    if (rq.state == WEB_READ) webRead();
    if (rq.state == WEB_SEND) webSendSome();
}
