#include "wit_c_sdk.h"
#include "REG.h"

static volatile char s_cDataUpdate = 0, s_cCmd = 0xff; 
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

#define WITSerial Serial7

int i;
float fAcc[3], fGyro[3], fAngle[3];
uint32_t LASTPRINT = 0;
float WITHeading = 0;
float WITRoll = 0;

// Conversion to Hexidecimal
const char* asciiHex = "0123456789ABCDEF";

// the new PANDA sentence buffer
char nmea[100];

// GGA
char fixTime[12];
char latitude[15];
char latNS[3];
char longitude[15];
char lonEW[3];
char fixQuality[2];
char numSats[4];
char HDOP[5];
char altitude[12];
char geoid[12];
char ageDGPS[10];

// VTG
char vtgHeading[12] = { };
char speedKnots[10] = { };

// IMU
char imuHeading[6];
char imuRoll[6];
char imuPitch[6];
char imuYawRate[6];

// If odd characters showed up.
void errorHandler()
{
  Serial.print("NMEA Parser error!!");
  Serial.print(parser.error());
  //nothing at the moment
}

void GGA_Handler() //Rec'd GGA
{
    // fix time
    parser.getArg(0, fixTime);

    String tempString = fixTime;
    utcTime = tempString.toFloat();

    // latitude
    parser.getArg(1, latitude);
    parser.getArg(2, latNS);

    // longitude
    parser.getArg(3, longitude);
    parser.getArg(4, lonEW);

    // fix quality
    parser.getArg(5, fixQuality);

    tempString = fixQuality;
    fixTypeGGA = tempString.toInt();

    // satellite #
    parser.getArg(6, numSats);

    tempString = numSats;
    satsGGA = tempString.toInt();

    // HDOP
    parser.getArg(7, HDOP);

    tempString = HDOP;
    hdopGGA = tempString.toFloat();

    // altitude
    parser.getArg(8, altitude);

    // height of geoid
    parser.getArg(10, geoid);

    tempString = geoid;
    geoidalGGA = tempString.toFloat();

    // time of last DGPS update
    parser.getArg(12, ageDGPS);

    tempString = ageDGPS;
    rtkAgeGGA = tempString.toFloat();

    bnoTimer = 0;
    bnoTrigger = true;

    if (useBNO08x)
    {
       imuHandler();          //Get IMU data ready
       BuildNmea();           //Build & send data GPS data to AgIO
    }

    else if (useBNO08xRVC || useWIT901)
    {
        BuildNmea();           //Build & send data GPS data to AgIO
    }

    else
    {
        itoa(0, imuYawRate, 10);
        itoa(0, imuRoll, 10);
        itoa(0, imuPitch, 10);
        itoa(65535, imuHeading, 10);       //65535 is max value to stop AgOpen using IMU in Panda
        BuildNmea();
    }
    /*
    Serial.println(utcTime,1);
    Serial.println(geoidalGGA, 2);
    Serial.println(fixTypeGGA);
    Serial.println(satsGGA);
    Serial.println(hdopGGA,2);
    Serial.println(rtkAgeGGA, 1);
    Serial.println("...........");
    */

    static int GPS_1hz = 0;

    if (sendGPStoISOBUS)
    {
        if (GPS_1hz > 9)
        {
            GPS_1hz = 0;
            sendISOBUS_129029();
        }

        GPS_1hz++;
    }

}

void VTG_Handler()
{
    // vtg heading
    parser.getArg(0, vtgHeading);

    // vtg Speed knots
    parser.getArg(4, speedKnots);


}

void ZDA_Handler()
{

}

void imuHandler()
{
    int16_t temp = 0;

    if (useBNO08x)
    {
        //BNO is reading in its own timer    
        // Fill rest of Panda Sentence - Heading
        temp = yaw;
        itoa(temp, imuHeading, 10);

        // the pitch x10
        temp = (int16_t)pitch;
        itoa(temp, imuPitch, 10);

        // the roll x10
        temp = (int16_t)roll;
        itoa(temp, imuRoll, 10);

        // YawRate - 0 for now
        itoa(0, imuYawRate, 10);
    }

    else if (useBNO08xRVC)
    {
        float angVel;

        // Fill rest of Panda Sentence - Heading
        itoa(bnoData.yawX10, imuHeading, 10);

        if (steerConfig.IsUseY_Axis)
        {
            // the pitch x100
            itoa(bnoData.pitchX10, imuPitch, 10);

            // the roll x100
            itoa(bnoData.rollX10, imuRoll, 10);
        }
        else
        {
            // the pitch x100
            itoa(bnoData.rollX10, imuPitch, 10);

            // the roll x100
            itoa(bnoData.pitchX10, imuRoll, 10);
        }

        //Serial.print(rvc.angCounter);
        //Serial.print(", ");
        //Serial.print(bnoData.angVel);
        //Serial.print(", ");
        // YawRate
        if (rvc.angCounter > 0)
        {
            angVel = ((float)bnoData.angVel) / (float)rvc.angCounter;
            angVel *= 10.0;
            rvc.angCounter = 0;
            bnoData.angVel = (int16_t)angVel;
        }
        else
        {
            bnoData.angVel = 0;
        }

        itoa(bnoData.angVel, imuYawRate, 10);
        bnoData.angVel = 0;
    }
}

void BuildNmea(void)
{
    strcpy(nmea, "");

    strcat(nmea, "$PANDA,");

    strcat(nmea, fixTime);
    strcat(nmea, ",");

    strcat(nmea, latitude);
    strcat(nmea, ",");

    strcat(nmea, latNS);
    strcat(nmea, ",");

    strcat(nmea, longitude);
    strcat(nmea, ",");

    strcat(nmea, lonEW);
    strcat(nmea, ",");

    // 6
    strcat(nmea, fixQuality);
    strcat(nmea, ",");

    strcat(nmea, numSats);
    strcat(nmea, ",");

    strcat(nmea, HDOP);
    strcat(nmea, ",");

    strcat(nmea, altitude);
    strcat(nmea, ",");

    //10
    strcat(nmea, ageDGPS);
    strcat(nmea, ",");

    //11
    strcat(nmea, speedKnots);
    strcat(nmea, ",");

    //12
    strcat(nmea, imuHeading);
    strcat(nmea, ",");

    //13
    strcat(nmea, imuRoll);
    strcat(nmea, ",");

    //14
    strcat(nmea, imuPitch);
    strcat(nmea, ",");

    //15
    strcat(nmea, imuYawRate);

    strcat(nmea, "*");

    CalculateChecksum();

    strcat(nmea, "\r\n");

//    Serial.println(nmea);
    //off to AOG
    int len = strlen(nmea);
    Udp.beginPacket(ipDestination, 9999);
    Udp.write(nmea, len);
    Udp.endPacket();

}

void CalculateChecksum(void)
{
  int16_t sum = 0;
  int16_t inx = 0;
  char tmp;

  // The checksum calc starts after '$' and ends before '*'
  for (inx = 1; inx < 200; inx++)
  {
    tmp = nmea[inx];

    // * Indicates end of data and start of checksum
    if (tmp == '*')
    {
      break;
    }

    sum ^= tmp;    // Build checksum
  }

  byte chk = (sum >> 4);
  char hex[2] = { asciiHex[chk], 0 };
  strcat(nmea, hex);

  chk = (sum % 16);
  char hex2[2] = { asciiHex[chk], 0 };
  strcat(nmea, hex2);
}

/*
  $PANDA
  (1) Time of fix

  position
  (2,3) 4807.038,N Latitude 48 deg 07.038' N
  (4,5) 01131.000,E Longitude 11 deg 31.000' E

  (6) 1 Fix quality:
    0 = invalid
    1 = GPS fix(SPS)
    2 = DGPS fix
    3 = PPS fix
    4 = Real Time Kinematic
    5 = Float RTK
    6 = estimated(dead reckoning)(2.3 feature)
    7 = Manual input mode
    8 = Simulation mode
  (7) Number of satellites being tracked
  (8) 0.9 Horizontal dilution of position
  (9) 545.4 Altitude (ALWAYS in Meters, above mean sea level)
  (10) 1.2 time in seconds since last DGPS update
  (11) Speed in knots

  FROM IMU:
  (12) Heading in degrees
  (13) Roll angle in degrees(positive roll = right leaning - right down, left up)

  (14) Pitch angle in degrees(Positive pitch = nose up)
  (15) Yaw Rate in Degrees / second

  CHKSUM
*/

/*
  $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M ,  ,*47
   0     1      2      3    4      5 6  7  8   9    10 11  12 13  14
        Time      Lat       Lon     FixSatsOP Alt
  Where:
     GGA          Global Positioning System Fix Data
     123519       Fix taken at 12:35:19 UTC
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     1            Fix quality: 0 = invalid
                               1 = GPS fix (SPS)
                               2 = DGPS fix
                               3 = PPS fix
                               4 = Real Time Kinematic
                               5 = Float RTK
                               6 = estimated (dead reckoning) (2.3 feature)
                               7 = Manual input mode
                               8 = Simulation mode
     08           Number of satellites being tracked
     0.9          Horizontal dilution of position
     545.4,M      Altitude, Meters, above mean sea level
     46.9,M       Height of geoid (mean sea level) above WGS84
                      ellipsoid
     (empty field) time in seconds since last DGPS update
     (empty field) DGPS station ID number
      47          the checksum data, always begins with


  $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
  0      1    2   3      4    5      6   7     8     9     10   11
        Time      Lat        Lon       knots  Ang   Date  MagV

  Where:
     RMC          Recommended Minimum sentence C
     123519       Fix taken at 12:35:19 UTC
     A            Status A=active or V=Void.
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     022.4        Speed over the ground in knots
     084.4        Track angle in degrees True
     230394       Date - 23rd of March 1994
     003.1,W      Magnetic Variation
      6A          The checksum data, always begins with

  $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48

    VTG          Track made good and ground speed
    054.7,T      True track made good (degrees)
    034.4,M      Magnetic track made good
    005.5,N      Ground speed, knots
    010.2,K      Ground speed, Kilometers per hour
     48          Checksum
*/


static void AutoScanSensor(void)
{
	int i, iRetry;
  const uint32_t c_uiBaud[8] = {0,4800, 9600, 19200, 38400, 57600, 115200, 230400};
	
	for(i = 0; i < sizeof(c_uiBaud)/sizeof(c_uiBaud[0]); i++)
	{
    Serial.print("Scan on Serial 1 with baud: ");
    Serial.println(c_uiBaud[i]);
		WITSerial.begin(c_uiBaud[i]);
    WITSerial.flush();
    for(int i = 0 ; i < 100; i++) {
       WITSerial.write('AA');
       delay(50);
    }
    WITSerial.flush();
    Serial.println("WITSerial flushed");
		iRetry = 2;
		s_cDataUpdate = 0;
		do
		{
			WitReadReg(AX, 3);
			delay(200);
      while (WITSerial.available())
      {
        WitSerialDataIn(WITSerial.read());
      }
			if(s_cDataUpdate != 0)
			{
				Serial.print(c_uiBaud[i]);
				Serial.print(" baud find sensor\r\n\r\n");
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
	Serial.print("can not find sensor\r\n");
	Serial.print("please check your connection\r\n");
}

void WIT901Setup(){
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(SensorDataUpdata);
  WitDelayMsRegister(Delayms);
//  delay(500);
//    AutoScanSensor(); 

  //Serial7.begin(0,  SERIAL_8N1, 16, 17);
  
  WITSerial.begin(9600);
  WITSerial.flush();

  WitReadReg(AX, 3);
	delay(200);
  while(WITSerial.available()){
    WitSerialDataIn(WITSerial.read());
  }
  if(s_cDataUpdate != 0) {
		Serial.print("9600 baud find sensor\r\n\r\n");
//    Serial.print(WitSetUartBaud(WIT_BAUD_115200)); delay(20);
//    Serial.println("Set 115200 baud");
 //   AutoScanSensor(); 
  }
  s_cDataUpdate = 0;
  WitReadReg(AX, 3);
	delay(200);
  while(WITSerial.available()){
    WitSerialDataIn(WITSerial.read());
  }
  if(s_cDataUpdate != 0) {
		Serial.print("9600 baud find sensor\r\n\r\n");
    useWIT901 = true;

  Serial.println("Setting up Sensor!");
  WitWriteReg(KEY, KEY_UNLOCK);   delay(20);
  WitWriteReg(AXIS6, 0x01);       delay(20);
  WitWriteReg(SAVE, SAVE_PARAM);  delay(20);
  WitWriteReg(KEY, KEY_UNLOCK);   delay(20);
  WitWriteReg(ACCFILT, 100);      delay(20);
  WitWriteReg(SAVE, SAVE_PARAM);  delay(20);

  WitReadReg(ACCFILT, 3);      delay(20);
  WitReadReg(AXIS6, 3);        delay(20);
  Serial.print("ACCFILT: ");
  Serial.println(sReg[ACCFILT]);
  Serial.print(" sRegAxis6:");
  Serial.println(sReg[AXIS6]);
  
  WitSetBandwidth(BANDWIDTH_256HZ); delay(20);
  WitSetOutputRate(RRATE_100HZ);  delay(20);
//  WitSetBandwidth(BANDWIDTH_94HZ); delay(20);
//  WitSetOutputRate(RRATE_100HZ);  delay(20);
  WitSetContent(RSW_ANGLE);  delay(20);


  delay(300);
      Serial.print("%%%%%%%      ACCFILT: ");
      Serial.print(sReg[ACCFILT]);
      Serial.print("   sRegAxis6: ");
      Serial.println(sReg[AXIS6]);
  } else {
    Serial.println("WIT901 is not found!!!");
  }
}

void WIT901Debug() {
    if( millis() > LASTPRINT + 5000){
    LASTPRINT = millis();
      WitReadReg(ACCFILT, 3);
      WitReadReg(AXIS6, 3);
      Serial.print(millis());
      Serial.print("%%%%%%%      ACCFILT: ");
      Serial.print(sReg[ACCFILT]);
      Serial.print("   sRegAxis6: ");
      Serial.print(sReg[AXIS6]);
      Serial.print("WITRoll: ");
      Serial.print(WITRoll, 3);
      Serial.print("WITHeading: ");
      Serial.println(WITHeading, 3);
  }
}

void WIT901Parse() {
  while(WITSerial.available()){
    WitSerialDataIn(WITSerial.read());
  }
  WIT901Debug();
  	if(s_cDataUpdate)
		{
			for(i = 0; i < 3; i++)
			{
				fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
				fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
				fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
			}
      if(s_cDataUpdate & ANGLE_UPDATE)
			{
//				Serial.print("WITanglePitch:");
//				Serial.print(fAngle[0], 3);
//				Serial.print(" WITRoll:");
        if(steerConfig.IsUseY_Axis)
          WITRoll = (fAngle[1]*10)-16;
        else
          WITRoll = (fAngle[0]*10)-16;
//				Serial.print(WITRoll, 3);
//				Serial.print(" WITHeading:");
        WITHeading = fAngle[2]*-10;
        if(WITHeading < 0 && WITHeading >= -1800) WITHeading += 3600;
//				Serial.print(WITHeading, 3);
//				Serial.print("\r\n");
				s_cDataUpdate &= ~ANGLE_UPDATE;

        itoa(WITHeading, imuHeading, 10);
        itoa(WITRoll, imuRoll, 10);
        // YawRate - 0 for now
        itoa(0, imuYawRate, 10);

			}
      s_cDataUpdate = 0;
		}
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
  WITSerial.write(p_data, uiSize);
  WITSerial.flush();
}
static void Delayms(uint16_t ucMs)
{
  delay(ucMs);
}
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}

