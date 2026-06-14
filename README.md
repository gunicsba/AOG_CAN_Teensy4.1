# AOG CAN Teensy 4.1 (AgOpenGPS Autosteer Controller)

Firmware and setup files for a Teensy 4.1 CAN-based steering controller used with AgOpenGPS.

## Video

- YouTube Shorts demo: https://youtube.com/shorts/DkzHTf-jfCc

## Getting Started (Teensy 4.1 + Arduino IDE)

1. Update the Arduino IDE.
2. Install Teensyduino.
3. Teensyduino 1.55 includes newer FlexCAN_T4 and Native Ethernet support, so manual updates are usually not required.

## Case IH CAN Connections

- CAN1-H = Pin C, diagnostic connector (right rear pillar)
- CAN1-L = Pin D, diagnostic connector (right rear pillar)
- CAN3-H = Pin 3, nav controller 40-pin connector X-716, or Pin H diagnostic connector (behind seat)
- CAN3-L = Pin 13, nav controller 40-pin connector X-716, or Pin J diagnostic connector (behind seat)

## Reference

- [CaseIH_GPS.pdf](https://github.com/MechanicTony/AOG_CAN_Teensy4.1/files/10528432/CaseIH_GPS.pdf)
