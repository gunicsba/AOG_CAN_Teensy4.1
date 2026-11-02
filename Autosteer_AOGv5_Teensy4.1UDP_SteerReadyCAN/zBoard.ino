/* zBoard.ino - board setup: the Teensy serial ports of the GNSS receiver, the IMU (TM171) and the RTK radio.
   AOG_CAN_Teensy4.1, 2026. Chosen on the web page (Tractor & GPS), saved in the EEPROM (200-203), taken at
   start-up (boardPortsLoad, from setup()).
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Default = this board (Tony's Teensy 4.1 triple CAN PCB): GNSS Serial3, IMU Serial5, radio Serial7.
// AiO v4.x boards: GNSS Serial7, IMU Serial5, radio (XBee) Serial3 (their GPS2 port, Serial2, is not used here).
// Not offered, their pins are used by this firmware: Serial1 (pins 0/1 = CAN2, ISO-Bus), Serial2 (pins 7/8 = work
// switch and remote / steer encoder inputs), Serial4 (pins 16/17, 16 = engage LED).
// The port variables and pointers are in GPS.ino (portGnss, gpsPort, ...), before TM171.ino, which uses them too.

#define BOARD_EE_ADDR  200        // magic, GNSS, IMU, radio
#define BOARD_EE_MAGIC 0x5B

HardwareSerial* boardSerial(uint8_t n)
{
    switch (n)
    {
        case 1: return &Serial1;
        case 2: return &Serial2;
        case 3: return &Serial3;
        case 4: return &Serial4;
        case 5: return &Serial5;
        case 6: return &Serial6;
        case 7: return &Serial7;
        case 8: return &Serial8;
        default: return nullptr;
    }
}

// Teensy 4.1 TX pin of SerialN (for the messages: the TM171's RX must be wired to it)
uint8_t boardTxPin(uint8_t n)
{
    static const uint8_t tx[9] = { 0, 1, 8, 14, 17, 20, 24, 29, 35 };
    return n <= 8 ? tx[n] : 0;
}

static bool boardPortFree(uint8_t n) { return n == 3 || n == 5 || n == 6 || n == 7 || n == 8; }

// nullptr when the three ports can be used together, else why not
const char* boardPortsCheck(uint8_t gnss, uint8_t imu, uint8_t radio)
{
    if (!boardPortFree(gnss))                 return "GNSS port: Serial3, 5, 6, 7 or 8";
    if (imu && !boardPortFree(imu))           return "IMU port: none or Serial3, 5, 6, 7 or 8";
    if (radio && !boardPortFree(radio))       return "radio port: none or Serial3, 5, 6, 7 or 8";
    if (gnss == imu || gnss == radio || (imu && imu == radio)) return "each device needs its own serial port";
    return nullptr;
}

// From setup(), before the IMU and the GNSS are started
void boardPortsLoad()
{
    uint8_t b[4];
    for (uint8_t i = 0; i < 4; i++) b[i] = EEPROM.read(BOARD_EE_ADDR + i);
    if (b[0] == BOARD_EE_MAGIC && !boardPortsCheck(b[1], b[2], b[3])) { portGnss = b[1]; portImu = b[2]; portRadio = b[3]; }
    gpsPort   = boardSerial(portGnss);
    SerialImu = boardSerial(portImu);
    radioPort = boardSerial(portRadio);
    Serial.printf("\r\nSerial ports (web page, Tractor & GPS): GNSS Serial%u, IMU ", portGnss);
    if (portImu) Serial.printf("Serial%u", portImu); else Serial.print("none");
    Serial.print(", RTK radio ");
    if (portRadio) Serial.printf("Serial%u\r\n", portRadio); else Serial.println("none");
}

// From the web page; the caller restarts the Teensy
void boardPortsSave(uint8_t gnss, uint8_t imu, uint8_t radio)
{
    EEPROM.update(BOARD_EE_ADDR + 1, gnss);
    EEPROM.update(BOARD_EE_ADDR + 2, imu);
    EEPROM.update(BOARD_EE_ADDR + 3, radio);
    EEPROM.update(BOARD_EE_ADDR, BOARD_EE_MAGIC);
}
