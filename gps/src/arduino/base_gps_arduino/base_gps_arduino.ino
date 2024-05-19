#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;
bool response;

void setup() {
  
  delay(1000);

  Serial.begin(115200);

  myGNSS.setRTCMOutputPort(Serial); // Output rtcm to arduino serial.

  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  int deviceAddress = 0x42;
  while (myGNSS.begin(Wire, deviceAddress) == false){delay(1000);}

  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); // Ensure RTCM3 is enabled on I2C
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save the communications port settings to flash and BBR

  bool response = myGNSS.newCfgValset(); // Create a new Configuration Item VALSET message
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_I2C, 1); //Enable message 1005 to output through I2C port, message every second
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1074_I2C, 1);
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1084_I2C, 1);
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1094_I2C, 1);
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1124_I2C, 1);
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_I2C, 10); // Enable message 1230 every 10 seconds
  response &= myGNSS.sendCfgValset(); // Send the VALSET

  if (response == true)
  {
    Serial.println(F("RTCM messages enabled"));
  }
  else
  {
    Serial.println(F("Failed to enable RTCM messages!"));
    exit(1);
  }

  // Check if survey is in progress.
  if (myGNSS.getSurveyInActive() == true)
  {
    Serial.println("Survey successfully started!");
  }

  // // Wait until survey is complete.
  // while (myGNSS.getSurveyInValid() == false)
  // {
  //   response = myGNSS.getSurveyStatus(2000); //Query module for survey-in stats

  //   if (response == true)
  //   {
  //     Serial.print(F("Time elapsed: "));
  //     Serial.println((String)myGNSS.getSurveyInObservationTimeFull());
  //   }
  //   else
  //   {
  //     Serial.println(F("\r\nSVIN request failed"));
  //   }

  //   delay(1000);
  // }

}

void loop() 
{
  myGNSS.checkUblox(); //Check for new data and process bytes.
  delay(250);
}

void DevUBLOXGNSS::processRTCM(uint8_t incoming)
{
  static uint16_t byteCounter = 0;

  //Pretty-print the HEX values to Serial
  if (myGNSS.rtcmFrameCounter == 1)
  {
    byteCounter = 0;
    Serial.println();
  }
  if (byteCounter % 16 == 0)
    Serial.println();
  
  if (incoming < 0x10) Serial.print(F("0"));
  Serial.print(incoming, HEX);
  
  byteCounter++;
}
