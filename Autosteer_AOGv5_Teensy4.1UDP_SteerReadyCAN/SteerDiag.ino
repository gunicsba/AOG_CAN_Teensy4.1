
// Steering diagnostics CSV logger - Claas (Brand 0) only
// Toggle with 'D' on USB serial (main loop or service tool). Read-only: nothing is ever sent on the bus.
// While ON, extra V-Bus FIFO filters (3-7) are added and one CSV line is printed per 0x0CAC1E13.
// While OFF, the filters are restored to the normal Claas set so normal operation is unchanged.
// See README "Steering diagnostics CSV mode" and tools/steer_diag.py

#define DIAG_STALE_MS 2000      // older values are printed as empty fields
#define DIAG_FRESH(s) ((s).seen && (now - (s).t) <= DIAG_STALE_MS)

struct DiagSig {
  bool seen;
  uint32_t t;                   // millis() of last update
};

DiagSig dg301, dg105, dgE2D2, dg131B, dgCCVS;

uint16_t dg301Raw = 0;
uint8_t  dg301Cnt = 0;
int16_t  dg301Delta = 0;
uint8_t  dg301Status = 0;
uint16_t dg105Val = 0;
int16_t  dgE2D2Delta = 0;
uint16_t dg131BCurve = 0;
uint8_t  dg131BStatus = 0;
uint16_t dgSpeedRaw = 0;        // 1/256 km/h

char steerDiagMark = 0;         // marker for the next CSV line only
uint8_t steerDiagDlcWarned = 0; // one bit per message, warn only once

//---Enable / disable

void steerDiagToggle()
{
  if (steerDiagOn)
  {
    steerDiagOn = false;
    setClaasVBusFilters();      // back to the normal filter set
    Serial.println("# Steer diag OFF");
    return;
  }

  if (Brand != 0)
  {
    Serial.println("# Steer diag is only available for Claas (Brand 0)");
    return;
  }

  dg301.seen = dg105.seen = dgE2D2.seen = dg131B.seen = dgCCVS.seen = false;
  steerDiagMark = 0;
  steerDiagDlcWarned = 0;

  // Filters 0-2 are the normal Claas ones (CAN_setup), FIFO table has 8 entries
  bool ok = true;
  ok &= V_Bus.setFIFOFilter(3, 0x301, STD);         //Angle sensor (WAS or SASA?), 100 Hz
  ok &= V_Bus.setFIFOFilter(4, 0x105, STD);
  ok &= V_Bus.setFIFOFilter(5, 0x18FFE2D2, EXT);
  ok &= V_Bus.setFIFOFilter(6, 0x0CAD131B, EXT);    //Guidance System Command from SA 0x1B
  ok &= V_Bus.setFIFOFilter(7, 0x18FEF1FB, EXT);    //CCVS wheel based speed
  if (!ok) Serial.println("# WARNING: could not set all diag FIFO filters");

  steerDiagOn = true;

  Serial.println("# Steer diag ON - read only, send D to stop");
  Serial.println("# Markers: 0 = straight, L = full left, R = full right, E = AOG engaged, M = free marker");
  Serial.println("# Test procedure:");
  Serial.println("# 1. Engine running, standing still, wheels straight: press 0 and hold 10 s");
  Serial.println("# 2. Turn slowly to full left, press L, hold 3 s. Turn slowly to full right, press R, hold 3 s.");
  Serial.println("#    Return to straight and press 0");
  Serial.println("# 3. Repeat step 2 once, faster");
  Serial.println("# 4. AOG engaged, drive slowly on a straight AB line for 30 s and press E,");
  Serial.println("#    then let AOG steer a curve without touching the steering wheel");
  Serial.println("# 5. Optional: read PVED live AD1 (WAS) at straight / full left / full right, note it per marker");
  Serial.println("# 6. Measure the real front wheel angle at full left and full right (rim angle app or turning circle)");
  Serial.println("t_ms,mark,estCurve,valveState,curv_kmInv,angle_div10,angle_div23_3,"
                 "r301_raw,r301_cnt,r301_delta,r301_age_ms,r105,e2d2_delta,"
                 "cmd131b_curve,cmd131b_status,speed_kmh,"
                 "aog_setpoint_deg,aog_actual_sent_deg,steer_engaged,pwm_or_cmd_out,r301_status");
}

//---Serial keys while diag is running

void steerDiagKey(byte b)
{
  if (b == '0' || b == 'L' || b == 'R' || b == 'E' || b == 'M') steerDiagMark = (char)b;
}

//---CAN receive hook, called from VBus_Receive (Claas branch)

void steerDiagDlcWarn(uint8_t bit, const char* name, uint8_t len)
{
  if (steerDiagDlcWarned & bit) return;
  steerDiagDlcWarned |= bit;
  Serial.printf("# WARNING: %s DLC=%u shorter than expected, ignored\r\n", name, len);
}

void steerDiagRx(const CAN_message_t &msg)
{
  uint32_t now = millis();

  if (!msg.flags.extended)
  {
    if (msg.id == 0x301)
    {
      if (msg.len < 6) { steerDiagDlcWarn(0x01, "0x301", msg.len); return; }
      dg301Raw = msg.buf[0] | (msg.buf[1] << 8);
      dg301Cnt = msg.buf[2];
      dg301Delta = (int16_t)(msg.buf[3] | (msg.buf[4] << 8));
      dg301Status = msg.buf[5];
      dg301.seen = true; dg301.t = now;
    }
    else if (msg.id == 0x105)
    {
      if (msg.len < 2) { steerDiagDlcWarn(0x02, "0x105", msg.len); return; }
      dg105Val = msg.buf[0] | (msg.buf[1] << 8);
      dg105.seen = true; dg105.t = now;
    }
    return;
  }

  if (msg.id == 0x18FFE2D2)
  {
    if (msg.len < 5) { steerDiagDlcWarn(0x04, "0x18FFE2D2", msg.len); return; }
    dgE2D2Delta = (int16_t)(msg.buf[3] | (msg.buf[4] << 8));
    dgE2D2.seen = true; dgE2D2.t = now;
  }
  else if (msg.id == 0x0CAD131B)
  {
    if (msg.len < 3) { steerDiagDlcWarn(0x08, "0x0CAD131B", msg.len); return; }
    dg131BCurve = msg.buf[0] | (msg.buf[1] << 8);
    dg131BStatus = msg.buf[2];
    dg131B.seen = true; dg131B.t = now;
  }
  else if (msg.id == 0x18FEF1FB)
  {
    if (msg.len < 3) { steerDiagDlcWarn(0x10, "0x18FEF1FB", msg.len); return; }
    dgSpeedRaw = msg.buf[1] | (msg.buf[2] << 8);
    dgCCVS.seen = (dgSpeedRaw < 0xFB00);          // J1939 error / not available
    dgCCVS.t = now;
  }
  else if (msg.id == 0x0CAC1E13)
  {
    if (msg.len < 3) { steerDiagDlcWarn(0x20, "0x0CAC1E13", msg.len); return; }
    steerDiagPrintLine(now, msg.buf[0] | (msg.buf[1] << 8), msg.buf[2]);
  }
}

//---One CSV line, printed field by field (no heap, no String)

void steerDiagPrintLine(uint32_t now, uint16_t curve, uint8_t valveState)
{
  int32_t c = (int32_t)curve - 32128;

  Serial.printf("%lu,", (unsigned long)now);
  if (steerDiagMark) Serial.print(steerDiagMark);
  steerDiagMark = 0;

  Serial.printf(",%u,%u,%.2f,%.2f,%.2f", curve, valveState,
                curve * 0.25f - 8032.0f, c / 10.0f, c / 23.3f);

  if (DIAG_FRESH(dg301)) Serial.printf(",%u,%u,%d", dg301Raw, dg301Cnt, dg301Delta);
  else Serial.print(",,,");
  if (dg301.seen) Serial.printf(",%lu", (unsigned long)(now - dg301.t));
  else Serial.print(",");

  if (DIAG_FRESH(dg105)) Serial.printf(",%u", dg105Val);
  else Serial.print(",");
  if (DIAG_FRESH(dgE2D2)) Serial.printf(",%d", dgE2D2Delta);
  else Serial.print(",");
  if (DIAG_FRESH(dg131B)) Serial.printf(",%u,%u", dg131BCurve, dg131BStatus);
  else Serial.print(",,");
  if (DIAG_FRESH(dgCCVS)) Serial.printf(",%.2f", dgSpeedRaw / 256.0f);
  else Serial.print(",");

  // Exactly what AOG gets: the angle bytes of the last PGN 253 that was sent
  int16_t sentSA = (int16_t)(AOG[5] | (AOG[6] << 8));
  Serial.printf(",%.2f,%.2f,%u,%u", steerAngleSetPoint, sentSA / 100.0f, intendToSteer, setCurve);

  if (DIAG_FRESH(dg301)) Serial.printf(",%u\r\n", dg301Status);
  else Serial.print(",\r\n");
}
