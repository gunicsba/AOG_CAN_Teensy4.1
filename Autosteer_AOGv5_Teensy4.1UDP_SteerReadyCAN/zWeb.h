/* zWeb.h - declarations shared by the web page files (zWeb.ino, zOTA.ino, TM171.ino).
   AOG_CAN_Teensy4.1, 2026. Web page, firmware update and TM171 tools ported from renatorgr/Boards,
   Firmware/v4.5_AiO_WEB (written 2026 by Claude (Anthropic) for renatorgr).
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// In a header because the Arduino IDE puts the function prototypes it generates near the top of the
// sketch, before any type defined in an .ino file.
#ifndef ZWEB_H
#define ZWEB_H

#include <stdint.h>
#include "zWebJson.h"

#define FW_NAME "AOG_CAN_Teensy4.1"

// TM171 settings form from the web page (TM171.ino, zWeb.ino)
struct TmSetForm
{
    bool     gyro, acc, mag;           // sensors used by the fusion
    bool     gyroErr, selfAdapt;       // GyroErrFilter, magnetometer Self-Adapt filter
    bool     status;                   // status output (temperature, QoS)
    bool     save;                     // save to the TM171's flash (else this power-on only)
    uint8_t  boot;                     // 0 auto, 1 static, 2 dynamic
    uint16_t accGain, magGain;         // x 0.01
    uint16_t silent;                   // power-on silent time, s (0..65)
    uint16_t odr;                      // output data rate, Hz (0 = keep as read)
};

// A line for the Messages tab of the web page (and USB serial)
void webLog(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

// PVED tab session (zPved.ino): autosteer blocked while open; quiet = no steering commands to the valve
extern bool pvedWebOn, pvedWebQuiet;

// The TM171's serial port, chosen on the web page (TM171.ino, zBoard.ino); nullptr = no IMU port
extern HardwareSerial* SerialImu;

#endif
