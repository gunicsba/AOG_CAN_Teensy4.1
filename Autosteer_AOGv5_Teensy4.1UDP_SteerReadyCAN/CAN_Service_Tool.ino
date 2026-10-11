
// Danfoss PVED-CL Service Tool for use with AgOpenGPS
// The Danfoss PVED-CL parts are not in here yet, but will be copied over soon

//Ryan Claas addons
// 30-12-2022  Added PVED-CL parameter 64007 change code to CAN Service Tool page. Use to change CLAAS PVED-CL parameter 64007
//             Add CLAAS pre Mother reg tractor CSM button messages on main Autosteer_AOG_Teensy4.1 page line 156-160 Comment out lines not required to select pre MR or MR/Stage5

//----------------------------------------------------------
//Notes from Ryan (When Changing Claas Parameter 64007) 
//
//Mother Regulated CLAAS tractors:
//Should work on Axion A64, A44, A51, A61, A60, A50, 
//Arion A97, A96, A95, A94, A74, A76, A75, A77
//Only tested on Axion A60
//
//Parameter 64007(external set point generator address, AKA GPS)
//needs to be changed to 30 (dec) from 27 (dec) then commited to memory. Power to PEVD-CL must then be cycled
//before PVED will send/receive AOG teensy AD,AC messages.
//
//Work Switch is triggerd via CAN, set work triggers in CEBIS implement menu
//and make sure a task in CEBIS is playing 
//
//CAUTION
//This is a modification to the tractor standard safety checks
//Operator is responsible for ensuring ATP switch is off when not using Autosteer
//This modification will most likley disable dynamic (varable ratio)steering
//and CEBIS gives a warning when CLAAS engage button is pressed but it disapears after a second (MR Models)
//
//Pre Mother Regulated tractors:
//Arion A36,A37,A34,A35, Axion A23,A40,A41
//
//Need to make wiring to connect to tractor steering CAN at ATP module
//Wiring will make CLAAS steer rocker switch power PVED-CL directly along with safety/reaction valve
//could also wire so safety/reaction valve is powered by PCB only when autosteer is actualy steering
//
//---------------------------------------------------------

//PVED config mode state (P / Y / C), see bottom of this page
const uint16_t pvedWritable[]      = { 65080, 65083, 65086, 65099, 65100, 65112 };
const char* const pvedWritableName[] = { "AD1_1000_Left (WAS left)", "AD1_1000_Right (WAS right)", "AD1_Neutral (WAS centre)",
                                         "True Angle Left", "True Angle Right", "Vehicle Length mm" };
uint32_t pvedLastRead[6];
bool pvedKnown[6];
int8_t pvedPending = -1;        //index of the prepared write
uint32_t pvedPendingValue = 0;
uint16_t pvedSent65112 = 0;     //vehicle length written this session, copied to firmware on commit if atan mode is on

void Service_Tool (void) 
{
  Serial.println("\r\nAgOpenGPS CANBUS Service Tool Mode:");
  Help();
  
  while (Service == 1) 
  {
  
      if (Serial.available())   // Read Data From Serail Monitor 
      {    
        byte b = Serial.read();
        if ( b == '?') Help();          
        else if ( b == 'X') Service = 0; //Exit Service Mode
        else if ( b == '0') Claas();
        else if ( b == '1') Valtra();
        else if ( b == '2') CNH();
        else if ( b == '3') Fendt();
        else if ( b == '4') JCB();
        else if ( b == '5') FendtOne();
        else if ( b == '6') Lindner();
        else if ( b == '7') AgOpenGPS();
        else if ( b == '8') CatMT();
        else if ( b == '9') CatMT_Early();
        else if ( b == 'R') ReadCAN();
        else if ( b == 'S') StopCAN();
        else if ( b == 'Z') setupPVED();
        else if ( b == 'V')                       //Claas angle mode: V0 linear counts/deg, V<mm> atan(L x kappa), L = PVED 65112
        {
          char arg[16];
          size_t n = Serial.readBytesUntil('\n', arg, sizeof(arg) - 1);
          arg[n] = 0;
          uint16_t mm = strtoul(arg, NULL, 10);
          if (isdigit(arg[0]) && (mm == 0 || (mm >= 500 && mm <= 10000))) setClaasVehicleLength(mm);
          else
          {
            printClaasVehicleLength();
            Serial.println("V0 = linear (AOG counts per degree), V<mm> = atan mode with that vehicle length, e.g. V2820");
          }
        }
        else if ( b == 'D')                       //Steering diag needs the main loop running, so leave the service tool
        {
          steerDiagToggle();
          if (steerDiagOn)
          {
            Service = 0;
            Serial.println("# Service tool closed, marker keys are now active (S = service tool again)");
          }
        }
        else if ( b == 'f') gpsModeOne();
        else if ( b == 'F') gpsModeTwo();
        else if ( b == 'p') gpsModeThree();
        else if ( b == 'P') gpsModeFour();

        else
        {
          Serial.println("No command, send ? for help");
          Serial.println(" ");
          delay(50);
        }

        while (Serial.available())
        {
        Serial.read();                //Clear the serial buffer
        }
      }

      if (tempChecker > 10000)
      {
          tempChecker = 0;
          float temp = tempmonGetTemp();
          Serial.print(temp, 2);
          Serial.println(" degC CPU Temp");
      }

  }
}

//**************************************************************************************
void Help(){
  Serial.println("? = Help");
  Serial.println("X = Exit Service Mode");
  Serial.println("0 = Set Brand as Claas");
  Serial.println("1 = Set Brand as Valtra / Massey");
  Serial.println("2 = Set Brand as CaseIH / New Holland");
  Serial.println("3 = Set Brand as Fendt SCR, S4, Gen6");
  Serial.println("4 = Set Brand as JCB");
  Serial.println("5 = Set Brand as FendtOne");
  Serial.println("6 = Set Brand as Lindner");
  Serial.println("7 = Set Brand as AgOpenGPS");
  Serial.println("8 = Set Brand as Cat MT Late");
  Serial.println("9 = Set Brand as Cat MT Early");
  Serial.println("R = Show CAN Data");
  Serial.println("S = Stop Data");
  Serial.println("Z = Danfoss PVED parameter setup");
  Serial.println("D = Steering diagnostics CSV on/off (Claas, exits service tool, read only)");
  Serial.println("V = Claas angle mode: V0 = linear AOG counts per degree, V<mm> = atan(L x curvature), L = PVED 65112");
  Serial.println("    **GPS options**");
  Serial.println("Forwarding Mode: f = 115200, F = 460800");
  Serial.println("Panda Mode: p = 115200, P = 460800\r\n");
}

//**************************************************************************************
void Claas(){
  EEPROM.update(70,0); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set Claas, Restarting Teensy");
  delay(1000);
  SCB_AIRCR = 0x05FA0004; //Teensy Reset
  Serial.println(" ");
}

//**************************************************************************************
void Valtra(){
  EEPROM.update(70,1); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set Valtra, Restarting Teensy");
  delay(1000);
  SCB_AIRCR = 0x05FA0004; //Teensy Reset
  Serial.println(" ");
}

//**************************************************************************************
void CNH(){
  EEPROM.update(70,2); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set CaseIH / New Holland, Restarting Teensy");
  delay(1000);
  SCB_AIRCR = 0x05FA0004; //Teensy Reset
  Serial.println(" ");
}

//**************************************************************************************
void Fendt(){
  EEPROM.update(70,3); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set Fendt SCR/S4/Gen6, Restarting Teensy");
  delay(1000);
  SCB_AIRCR = 0x05FA0004; //Teensy Reset
  Serial.println(" ");
}
//**************************************************************************************
void JCB(){
  EEPROM.update(70,4); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set JCB, Restarting Teensy");
  delay(1000);
  SCB_AIRCR = 0x05FA0004; //Teensy Reset
  Serial.println(" ");
}
//**************************************************************************************
void FendtOne(){
  EEPROM.update(70,5); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set FendtOne, Restarting Teensy");
  delay(1000);
  SCB_AIRCR = 0x05FA0004; //Teensy Reset
  Serial.println(" ");
}
//**************************************************************************************
void Lindner(){
  EEPROM.update(70,6); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set Lindner, Restarting Teensy");
  delay(1000);
  SCB_AIRCR = 0x05FA0004; //Teensy Reset
  Serial.println(" ");
}
//**************************************************************************************
void AgOpenGPS(){
  EEPROM.update(70,7); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set AgOpenGPS, Restarting Teensy");
  delay(1000);
  SCB_AIRCR = 0x05FA0004; //Teensy Reset
  Serial.println(" ");
}
//**************************************************************************************
void CatMT() {
    EEPROM.update(70, 8);
    Brand = EEPROM.read(70);
    Serial.println("Brand Set Cat MT Late, Restarting Teensy");
    delay(1000);
    SCB_AIRCR = 0x05FA0004; //Teensy Reset
    Serial.println(" ");
}
//**************************************************************************************
void CatMT_Early() {
    EEPROM.update(70, 9);
    Brand = EEPROM.read(70);
    Serial.println("Brand Set Cat MT Early, Restarting Teensy");
    delay(1000);
    SCB_AIRCR = 0x05FA0004; //Teensy Reset
    Serial.println(" ");
}
//**************************************************************************************
void ReadCAN(){
ShowCANData = 1;
  Serial.println("CAN Data ON, Send X To Exit Service Tool");
  Serial.println(" ");
}
//**************************************************************************************
void StopCAN(){
ShowCANData = 0;
  Serial.println("CAN Data OFF, Send X To Exit Service Tool");
  Serial.println(" ");
}
//**************************************************************************************
void gpsModeOne() {
    EEPROM.update(72, 1);
    Serial.println("GPS Forwarding @ 115200, Restarting Teensy");
    delay(1000);
    SCB_AIRCR = 0x05FA0004; //Teensy Reset
    Serial.println(" ");
}
//**************************************************************************************
void gpsModeTwo() {
    EEPROM.update(72, 2);
    Serial.println("GPS Forwarding @ 460800, Restarting Teensy");
    delay(1000);
    SCB_AIRCR = 0x05FA0004; //Teensy Reset
    Serial.println(" ");
}
//**************************************************************************************
void gpsModeThree() {
    EEPROM.update(72, 3);
    Serial.println("GPS Panda @ 115200, Restarting Teensy");
    delay(1000);
    SCB_AIRCR = 0x05FA0004; //Teensy Reset
    Serial.println(" ");
}
//**************************************************************************************
void gpsModeFour() {
    EEPROM.update(72, 4);
    Serial.println("GPS Panda @ 460800, Restarting Teensy");
    delay(1000);
    SCB_AIRCR = 0x05FA0004; //Teensy Reset
    Serial.println(" ");
}
//**************************************************************************************
void setupPVED() {
    elapsedMillis parameterCheckTimer;
    static bool configPVED = true;
    static bool showMessage = false;
    static int ReadParameters = 0;
    int16_t S16Value = 9999;
    uint16_t U16Value = 9999;
    uint32_t U32Value = 9999;

    //Reset the CAN filters for config tool only
    V_Bus.setFIFOFilter(REJECT_ALL);
    V_Bus.setFIFOFilter(0, 0x18EFFD13, EXT);
    V_Bus.setFIFOFilter(1, 0x18EFFC13, EXT);
    V_Bus.setFIFOUserFilter(2, 0x00EF0013, 0x00FF00FF, EXT);  //Any Prop-A frame from the PVED (any priority / DA), e.g. status stream

    Serial.println("\r\nAgOpenGPS Danfoss PVED Config Mode:");
    Serial.println("Send X to exit and reset Teensy");
    Serial.println("Send S to print CAN message");
    Serial.println("Send s to stop printing message");
    Serial.println("Send R to read PVED parameters (save this log = original values, PARAM lines)");
    Serial.println("Send W to change parameter 64007 to 30(Dec) 1E(Hex)");
    Serial.println("Send C to commit settings to PVED");
    Serial.println("Send P<num>=<raw> to prepare a write, e.g. P65086=512, then Y to send it");
    Serial.println("    allowed: 65080 AD1 left, 65083 AD1 right, 65086 AD1 neutral, 65099/65100 true angle L/R, 65112 vehicle length");
    Serial.println("Send T<8 hex bytes> to send a raw frame to the PVED, e.g. T0FAB......");
    Serial.println("    (status stream request from your dump; write/commit XIDs are blocked here)");
    Serial.println("Send M<text> to put a marker in the log, e.g. Mleft");
    Serial.println("Other PVED frames (status stream) are printed as STAT lines, max 4 per second\r\n");

    Serial.println("Please power ON PVED valve...");

    while (Serial.available())
    {
        Serial.read();                //Clear the serial buffer
    }

    CAN_message_t ConfigData;
    while (V_Bus.read(ConfigData)) 
    {
        //Make sure theres no messages left
    }

    while (configPVED == true)
    {
        if (Serial.available())         // Read Data From Serail Monitor 
        {
            byte e = Serial.read();
            if (e == 'X')               //Exit Service Mode
            {
                Serial.println("Restarting Teensy...");
                delay(1000);
                SCB_AIRCR = 0x05FA0004; //Teensy Reset
            }
            else if (e == 'W') WriteParameters();   //Write Basic Parameters
            else if (e == 'C') Commit();            //Commit / Save Data
            else if (e == 'R') ReadParameters = 1;  //Get Basic Parameters
            else if (e == 'P' || e == 'T' || e == 'M')    //Commands with an argument, read the rest of the line
            {
                char arg[48];
                size_t n = Serial.readBytesUntil('\n', arg, sizeof(arg) - 1);
                arg[n] = 0;
                if (e == 'P') pvedPrepareWrite(arg);
                else if (e == 'T') pvedSendRaw(arg);
                else Serial.printf("MARK,%lu,%s\r\n", millis(), arg);
            }
            else if (e == 'Y') pvedSendWrite();     //Send the prepared parameter write
            else if (e == 'S')
            {
                showMessage = 1;
                Serial.println("Show CAN Data");
            }
            else if (e == 's')
            {
                showMessage = 0;
                Serial.println("Stop CAN Data");
            }
            else
            {
                Serial.println("No command, retry");
                Serial.println(" ");
                delay(50);
            }

            while (Serial.available())
            {
                Serial.read();                //Clear the serial buffer
            }
        }

        if (V_Bus.read(ConfigData))
        {
            if (showMessage == 1) 
            {
                Serial.print("V-Bus");
                Serial.print(", MB: "); Serial.print(ConfigData.mb);
                Serial.print(", ID: 0x"); Serial.print(ConfigData.id, HEX);
                Serial.print(", EXT: "); Serial.print(ConfigData.flags.extended);
                Serial.print(", LEN: "); Serial.print(ConfigData.len);
                Serial.print(", DATA: ");
                for (uint8_t i = 0; i < 8; i++) {
                    Serial.print(ConfigData.buf[i], DEC); Serial.print(", ");
                }
                Serial.println("");
            }

            //Valve current mode
            if ((ConfigData.buf[0] == 0x0F) && (ConfigData.buf[1] == 0xAA)) 
            {
                if ((ConfigData.buf[2] == 0x55)) Serial.println("PVED Awake In Calabration Mode");
                if ((ConfigData.buf[2] == 0xAA)) Serial.println("PVED Awake In Operation Mode");
            }

            //Anything else from the PVED (e.g. StartStopStatus stream), throttled so it stays readable
            else if (!((ConfigData.buf[0] == 0x0F) && ((ConfigData.buf[1] == 0xA1) || (ConfigData.buf[1] == 0xA3))))
            {
                static elapsedMillis statPrintTimer;
                if (showMessage == 0 && statPrintTimer >= 250)
                {
                    statPrintTimer = 0;
                    Serial.printf("STAT,%lu,0x%08lX,LEN %u,", millis(), (unsigned long)ConfigData.id, ConfigData.len);
                    for (uint8_t i = 0; i < ConfigData.len && i < 8; i++) Serial.printf(" %02X", ConfigData.buf[i]);
                    Serial.println();
                }
            }

            //Config data reading
            if ((ConfigData.buf[0] == 0x0F) && ((ConfigData.buf[1] == 0xA1) || (ConfigData.buf[1] == 0xA3)))
            {
                pvedParamReply(ConfigData.buf[1], ConfigData.len, ConfigData.buf);  //Machine readable PARAM line (backup / restore)

                if ((ConfigData.buf[2] == 0xFC) && (ConfigData.buf[3] == 0x01)) {
                    S16Value = (((int8_t)ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("508 Kp Gain = ");
                    Serial.print(S16Value);
                    Serial.println(" ");
                }

                else if ((ConfigData.buf[2] == 0xC2) && (ConfigData.buf[3] == 0x02)) {
                    U16Value = (((int8_t)ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("706 Vcap = ");
                    Serial.print(U16Value);
                    Serial.println(" ");
                }

                else if ((ConfigData.buf[2] == 0xC3) && (ConfigData.buf[3] == 0x02)) {
                    U16Value = (((uint8_t)ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("707 StrkVol (Cyd size) = ");
                    Serial.print(U16Value);
                    Serial.println(" ");
                }

                else if ((ConfigData.buf[2] == 0xD9) && (ConfigData.buf[3] == 0x02)) {
                    S16Value = (((int8_t)ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("729 Xspl_1000 = ");
                    Serial.print(S16Value);
                    Serial.println(" ");
                }

                else if ((ConfigData.buf[2] == 0xE1) && (ConfigData.buf[3] == 0x02)) {
                    S16Value = (((int8_t)ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("737 Xspl_0 = ");
                    Serial.print(S16Value);
                    Serial.println(" ");
                }


                else if ((ConfigData.buf[2] == 0xE2) && (ConfigData.buf[3] == 0x02)) {
                    S16Value = (((int8_t)ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("738 Xspr_0 = ");
                    Serial.print(S16Value);
                    Serial.println(" ");
                }

                else if ((ConfigData.buf[2] == 0xEB) && (ConfigData.buf[3] == 0x02)) {
                    S16Value = (((int8_t)ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("747 Xspr_1000 = ");
                    Serial.print(S16Value);
                    Serial.println(" ");
                }

                else if ((ConfigData.buf[2] == 0xEC) && (ConfigData.buf[3] == 0x02)) {
                    S16Value = (((int8_t)ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("748 Closed Loop Xsp Offset = ");
                    Serial.print(S16Value);
                    Serial.println(" ");
                }

                else if ((ConfigData.buf[2] == 0xF6) && (ConfigData.buf[3] == 0x02)) {
                    S16Value = (((int8_t)ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("758 Xsp Cal Offset = ");
                    Serial.print(S16Value);
                    Serial.println(" ");
                }

                else if ((ConfigData.buf[2] == 0x03) && (ConfigData.buf[3] == 0x04)) {
                    S16Value = (((int8_t)ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("1027 (Qm) Max Port Flow Faststeer = ");
                    Serial.print(S16Value);
                    Serial.println(" ");
                }

                else if ((ConfigData.buf[2] == 0xA3) && (ConfigData.buf[3] == 0x13)) {
                    S16Value = (((int8_t)ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("5027 (Qm) Max Port Flow Autosteer = ");
                    Serial.print(S16Value);
                    Serial.println(" ");
                }

                else if ((ConfigData.buf[2] == 0x07) && (ConfigData.buf[3] == 0xFA)) {
                    Serial.print("64007 (Setpoint Controller Address) = ");
                    Serial.print(ConfigData.buf[4], DEC);
                    Serial.print(",DEC ");
                    Serial.print(ConfigData.buf[4], HEX);
                    Serial.print(",HEX ");
                    Serial.println(" ");
                }

                //64022
                else if ((ConfigData.buf[2] == 0x16) && (ConfigData.buf[3] == 0xFA)) {
                    U16Value = (((uint8_t)ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("64022 SASA Cutout Speed = ");
                    Serial.print(U16Value);
                    Serial.println(" ");
                }

                //64023
                else if ((ConfigData.buf[2] == 0x17) && (ConfigData.buf[3] == 0xFA)) {
                    U32Value = (ConfigData.buf[7]);
                    U32Value = ((U32Value << 8) + (ConfigData.buf[6]));
                    U32Value = ((U32Value << 8) + (ConfigData.buf[5]));
                    U32Value = ((U32Value << 8) + (ConfigData.buf[4]));
                    Serial.print("64023 SASA Timeout = ");
                    Serial.print(U32Value);
                    Serial.println(" ");
                }

                //65080
                else if ((ConfigData.buf[2] == 0x38) && (ConfigData.buf[3] == 0xFE)) {
                    U16Value = ((ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("AD1_1000_Left (WAS Left Raw Counts) = ");
                    Serial.print(U16Value);
                    Serial.println(" ");
                }

                //65083
                else if ((ConfigData.buf[2] == 0x3B) && (ConfigData.buf[3] == 0xFE)) {
                    U16Value = ((ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("AD1_1000_Right (WAS Right Raw Counts) = ");
                    Serial.print(U16Value);
                    Serial.println(" ");
                }

                //65086
                else if ((ConfigData.buf[2] == 0x3E) && (ConfigData.buf[3] == 0xFE)) {
                    U16Value = ((ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("AD1_Neutral (WAS Centre Raw Counts) = ");
                    Serial.print(U16Value);
                    Serial.println(" ");
                }

                //65099
                else if ((ConfigData.buf[2] == 0x4B) && (ConfigData.buf[3] == 0xFE)) {
                    U32Value = (ConfigData.buf[7]);
                    U32Value = ((U32Value << 8) + (ConfigData.buf[6]));
                    U32Value = ((U32Value << 8) + (ConfigData.buf[5]));
                    U32Value = ((U32Value << 8) + (ConfigData.buf[4]));
                    Serial.print("True Angle Left = ");
                    Serial.print(U32Value);
                    Serial.println(" ");
                }

                //65100
                else if ((ConfigData.buf[2] == 0x4C) && (ConfigData.buf[3] == 0xFE)) {
                    U32Value = (ConfigData.buf[7]);
                    U32Value = ((U32Value << 8) + (ConfigData.buf[6]));
                    U32Value = ((U32Value << 8) + (ConfigData.buf[5]));
                    U32Value = ((U32Value << 8) + (ConfigData.buf[4]));
                    Serial.print("True Angle Right = ");
                    Serial.print(U32Value);
                    Serial.println(" ");
                }

                //65101
                else if ((ConfigData.buf[2] == 0x4D) && (ConfigData.buf[3] == 0xFE)) {
                    Serial.print("SASA Present = ");
                    Serial.print(ConfigData.buf[4],DEC);
                    Serial.println(" ");
                }

                //65104
                else if ((ConfigData.buf[2] == 0x50) && (ConfigData.buf[3] == 0xFE)) {
                    Serial.print("SASA Present = ");
                    Serial.print(ConfigData.buf[4], DEC);
                    Serial.println(" ");
                }

                //65112
                else if ((ConfigData.buf[2] == 0x58) && (ConfigData.buf[3] == 0xFE)) {
                    U16Value = ((ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("65112 Vehicle Length = ");
                    Serial.print(U16Value);
                    Serial.println(" ");
                }

                else  {
                    Serial.println("Im not sure ??");
                }

                Serial.println(" ");
            }

        }

        //Get Basic Parameters
        if (ReadParameters != 0) 
        {

            static CAN_message_t msgR;

            if (ReadParameters == 1) {
                Serial.println("Get Parameter 1/22 508");
                msgR.id = 0x98EF13FD;
                msgR.flags.extended = true;
                msgR.len = 8;
                msgR.buf[0] = 0x0F;
                msgR.buf[1] = 0xA0;
                msgR.buf[2] = 0xFC;
                msgR.buf[3] = 0x01;
                msgR.buf[4] = 0x00;
                msgR.buf[5] = 0x00;
                msgR.buf[6] = 0x00;
                msgR.buf[7] = 0x00;
                V_Bus.write(msgR);
                ReadParameters = 2;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 2 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 2/22 706");
                msgR.buf[2] = 0xC2;
                msgR.buf[3] = 0x02;
                V_Bus.write(msgR);
                ReadParameters = 3;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 3 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 3/22 707");
                msgR.buf[2] = 0xC3;
                msgR.buf[3] = 0x02;
                V_Bus.write(msgR);
                ReadParameters = 4;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 4 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 4/22 729");
                msgR.buf[2] = 0xD9;
                msgR.buf[3] = 0x02;
                V_Bus.write(msgR);
                ReadParameters = 5;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 5 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 5/22 737");
                msgR.buf[2] = 0xE1;
                msgR.buf[3] = 0x02;
                V_Bus.write(msgR);
                ReadParameters = 6;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 6 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 6/22 738");
                msgR.buf[2] = 0xE2;
                msgR.buf[3] = 0x02;
                V_Bus.write(msgR);
                ReadParameters = 7;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 7 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 7/22 747");
                msgR.buf[2] = 0xEB;
                msgR.buf[3] = 0x02;
                V_Bus.write(msgR);
                ReadParameters = 8;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 8 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 8/22 748");
                msgR.buf[2] = 0xEC;
                msgR.buf[3] = 0x02;
                V_Bus.write(msgR);
                ReadParameters = 9;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 9 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 9/22 758");
                msgR.buf[2] = 0xF6;
                msgR.buf[3] = 0x02;
                V_Bus.write(msgR);
                ReadParameters = 10;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 10 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 10/22 1027");
                msgR.buf[2] = 0x03;
                msgR.buf[3] = 0x04;
                V_Bus.write(msgR);
                ReadParameters = 11;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 11 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 11/22 5027");
                msgR.buf[2] = 0xA3;
                msgR.buf[3] = 0x13;
                V_Bus.write(msgR);
                ReadParameters = 12;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 12 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 12/22 64007");
                msgR.buf[2] = 0x07;
                msgR.buf[3] = 0xFA;
                V_Bus.write(msgR);
                ReadParameters = 13;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 13 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 13/22 64022");
                msgR.buf[2] = 0x16;
                msgR.buf[3] = 0xFA;
                V_Bus.write(msgR);
                ReadParameters = 14;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 14 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 14/22 64023");
                msgR.buf[2] = 0x17;
                msgR.buf[3] = 0xFA;
                V_Bus.write(msgR);
                ReadParameters = 15;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 15 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 15/22 65080");
                msgR.buf[2] = 0x38;
                msgR.buf[3] = 0xFE;
                V_Bus.write(msgR);
                ReadParameters = 16;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 16 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 16/22 65083");
                msgR.buf[2] = 0x3B;
                msgR.buf[3] = 0xFE;
                V_Bus.write(msgR);
                ReadParameters = 17;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 17 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 17/22 65086");
                msgR.buf[2] = 0x3E;
                msgR.buf[3] = 0xFE;
                V_Bus.write(msgR);
                ReadParameters = 18;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 18 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 18/22 65099");
                msgR.buf[2] = 0x4B;
                msgR.buf[3] = 0xFE;
                V_Bus.write(msgR);
                ReadParameters = 19;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 19 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 19/22 65100");
                msgR.buf[2] = 0x4C;
                msgR.buf[3] = 0xFE;
                V_Bus.write(msgR);
                ReadParameters = 20;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 20 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 20/22 65101");
                msgR.buf[2] = 0x4D;
                msgR.buf[3] = 0xFE;
                V_Bus.write(msgR);
                ReadParameters = 21;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 21 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 21/22 65104");
                msgR.buf[2] = 0x50;
                msgR.buf[3] = 0xFE;
                V_Bus.write(msgR);
                ReadParameters = 22;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 22 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 22/22 65112");
                msgR.buf[2] = 0x58;
                msgR.buf[3] = 0xFE;
                V_Bus.write(msgR);
                ReadParameters = 0;
                parameterCheckTimer = 0;
            }
        }
    }
}

//**************************************************************************************

void WriteParameters()
{
    // Change parameter 64007 to 30(Dec) 1E(Hex)
    CAN_message_t msgP;
    msgP.id = 0x98EF13FD;
    msgP.flags.extended = true;
    msgP.len = 8;
    msgP.buf[0] = 0x0F;
    msgP.buf[1] = 0xA2;
    msgP.buf[2] = 0x07;
    msgP.buf[3] = 0xFA;
    msgP.buf[4] = 0x1E;
    msgP.buf[5] = 0x00;
    msgP.buf[6] = 0x00;
    msgP.buf[7] = 0x00;
    V_Bus.write(msgP);
    
    delay(100);

     // Change parameter 64101 to 255(Dec) SASA present
    CAN_message_t msgsasa;
    msgsasa.id = 0x98EF13FD;
    msgsasa.flags.extended = true;
    msgsasa.len = 8;
    msgsasa.buf[0] = 0x0F;
    msgsasa.buf[1] = 0xA2;
    msgsasa.buf[2] = 0x4D;
    msgsasa.buf[3] = 0xFE;
    msgsasa.buf[4] = 0xFF;
    msgsasa.buf[5] = 0x00;
    msgsasa.buf[6] = 0x00;
    msgsasa.buf[7] = 0x00;
    V_Bus.write(msgsasa);

    delay(100);

 // Change parameter 1027 to 0(Dec) Speed steer amount, 0= none, 1000=full port flow
    CAN_message_t msgflow;
    msgflow.id = 0x98EF13FD;
    msgflow.flags.extended = true;
    msgflow.len = 8;
    msgflow.buf[0] = 0x0F;
    msgflow.buf[1] = 0xA2;
    msgflow.buf[2] = 0x03;
    msgflow.buf[3] = 0x04;
    msgflow.buf[4] = 0x00;
    msgflow.buf[5] = 0x00;
    msgflow.buf[6] = 0x00;
    msgflow.buf[7] = 0x00;
    V_Bus.write(msgflow);

    delay(100);
    Serial.println("Sent parameter change request, make sure you send commit command next");
}

//**************************************************************************************

void Commit()
{
    // Commit parameters to PVED-CL  
    CAN_message_t msgC;
    msgC.id = 0x98EF13FD;
    msgC.flags.extended = true;
    msgC.len = 8;
    msgC.buf[0] = 0x0F;
    msgC.buf[1] = 0xA6;
    msgC.buf[2] = 0x5A;
    msgC.buf[3] = 0x00;
    msgC.buf[4] = 0x00;
    msgC.buf[5] = 0x00;
    msgC.buf[6] = 0x00;
    msgC.buf[7] = 0x00;
    V_Bus.write(msgC);

    //In atan mode keep the firmware on the same vehicle length as the PVED, never switch the mode on by itself
    if (pvedSent65112 != 0)
    {
        if (claasVehicleLength > 0) setClaasVehicleLength(pvedSent65112);
        else Serial.printf("Firmware stays linear (AOG counts per degree). For atan mode send V%u in the service tool\r\n", pvedSent65112);
        pvedSent65112 = 0;
    }

    delay(1000);
    Serial.println("Commiting data wait 1min, then turn Off PVED valve & restart");
}

//**************************************************************************************
// WAS calibration writes (P / Y), raw frames (T) and PARAM backup lines

int8_t pvedWritableIndex(uint16_t num)
{
    for (int8_t i = 0; i < 6; i++) if (pvedWritable[i] == num) return i;
    return -1;
}

void pvedRequest(uint8_t xid, uint16_t num, uint32_t value)
{
    CAN_message_t msg;
    msg.id = 0x18EF13FD;
    msg.flags.extended = true;
    msg.len = 8;
    msg.buf[0] = 0x0F;
    msg.buf[1] = xid;               //0xA0 read, 0xA2 write
    msg.buf[2] = lowByte(num);
    msg.buf[3] = highByte(num);
    msg.buf[4] = value;
    msg.buf[5] = value >> 8;
    msg.buf[6] = value >> 16;
    msg.buf[7] = value >> 24;
    V_Bus.write(msg);
}

void pvedParamReply(uint8_t xid, uint8_t len, const uint8_t* buf)
{
    if (len < 8) return;
    uint16_t num = buf[2] | (buf[3] << 8);
    uint32_t raw = buf[4] | (buf[5] << 8) | ((uint32_t)buf[6] << 16) | ((uint32_t)buf[7] << 24);
    Serial.printf("PARAM,%u,%lu,%02X %02X %02X %02X,%s\r\n", num, (unsigned long)raw,
                  buf[4], buf[5], buf[6], buf[7], xid == 0xA1 ? "A1" : "A3");
    int8_t i = pvedWritableIndex(num);
    if (i >= 0 && xid == 0xA1)
    {
        pvedLastRead[i] = (num == 65099 || num == 65100) ? raw : (raw & 0xFFFF);
        pvedKnown[i] = true;
    }
}

void pvedPrepareWrite(char* arg)     //"65086=512"
{
    char* eq = strchr(arg, '=');
    if (eq == NULL) { Serial.println("Format: P<num>=<raw value>, e.g. P65086=512"); return; }
    uint16_t num = strtoul(arg, NULL, 10);
    uint32_t value = strtoul(eq + 1, NULL, 10);
    int8_t i = pvedWritableIndex(num);
    if (i < 0) { Serial.printf("Parameter %u is not allowed here\r\n", num); return; }
    if (num != 65099 && num != 65100 && value > 0xFFFF) { Serial.println("Value too big for a 16 bit parameter"); return; }
    if (num == 65112 && (value < 500 || value > 10000)) { Serial.println("Vehicle length must be 500-10000 mm"); return; }
    if (!pvedKnown[i]) Serial.println("WARNING: original value not read yet - send R first and save the log");

    pvedPending = i;
    pvedPendingValue = value;
    Serial.printf("Prepared: %u %s  ", num, pvedWritableName[i]);
    if (pvedKnown[i]) Serial.printf("%lu", (unsigned long)pvedLastRead[i]);
    else Serial.print("?");
    Serial.printf(" -> %lu   Send Y to write, then C to commit\r\n", (unsigned long)value);
}

void pvedSendWrite()
{
    if (pvedPending < 0) { Serial.println("Nothing prepared, use P<num>=<value> first"); return; }
    uint16_t num = pvedWritable[pvedPending];
    pvedRequest(0xA2, num, pvedPendingValue);
    if (num == 65112) pvedSent65112 = pvedPendingValue;
    Serial.printf("Write sent: %u = %lu, reading back...\r\n", num, (unsigned long)pvedPendingValue);
    pvedPending = -1;
    delay(100);
    pvedRequest(0xA0, num, 0);       //read back, reply shows as PARAM line
}

void pvedSendRaw(char* arg)          //16 hex digits, spaces allowed
{
    CAN_message_t msg;
    uint8_t n = 0;
    for (char* p = arg; *p && n < 16; p++)
    {
        if (!isxdigit(*p)) continue;
        uint8_t v = isdigit(*p) ? *p - '0' : (toupper(*p) - 'A' + 10);
        if (n % 2 == 0) msg.buf[n / 2] = v << 4;
        else msg.buf[n / 2] |= v;
        n++;
    }
    if (n != 16) { Serial.println("Format: T + 8 bytes in hex, e.g. T0FAB0000000000 00"); return; }
    if (msg.buf[0] == 0x0F && (msg.buf[1] == 0xA2 || msg.buf[1] == 0xA6))
    {
        Serial.println("Write / commit frames are blocked here, use P/Y and C");
        return;
    }
    msg.id = 0x18EF13FD;
    msg.flags.extended = true;
    msg.len = 8;
    V_Bus.write(msg);
    Serial.print("Sent 0x18EF13FD:");
    for (uint8_t i = 0; i < 8; i++) Serial.printf(" %02X", msg.buf[i]);
    Serial.println();
}

