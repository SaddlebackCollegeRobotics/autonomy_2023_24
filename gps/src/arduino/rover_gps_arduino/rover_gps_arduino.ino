#include <Wire.h> //Needed for I2C to GNSS
#include <CRC32.h>
#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3

enum class WhichGPS
{
  MovingBase,
  MovingRover
};

class GPS_Module
{
public:

  GPS_Module(
    uint8_t deviceAddress, 
    void (*pvtCallbackFn)(UBX_NAV_PVT_data_t*), 
    void (*hpposllhCallbackFn)(UBX_NAV_HPPOSLLH_data_t*), 
    void (*relposnedCallbackFn)(UBX_NAV_RELPOSNED_data_t*)
  )
  {
    while (gps.begin(Wire, deviceAddress) == false){delay(1000);}

    gps.setAutoPVTcallbackPtr(pvtCallbackFn);
    gps.setAutoHPPOSLLHcallbackPtr(hpposllhCallbackFn);
    gps.setAutoRELPOSNEDcallbackPtr(relposnedCallbackFn);
  }

  void update()
  {
    gps.checkUblox();
    gps.checkCallbacks();
  }

  void pushRTCM(uint8_t *dataBytes, size_t numDataBytes)
  {
    gps.pushRawData(dataBytes, numDataBytes); 
  }

  int32_t getRelativeHeading(){return relativeHeading;}

  int32_t getLatitude(){return latitude;}
  int32_t getLongitude(){return longitude;}

  int8_t getLatitudeHP(){return latitudeHP;}
  int8_t getLongitudeHP(){return longitudeHP;}

  bool IsGNSSFixOK(){return gnssFixOK;}
  bool IsRelPosValid(){return relPosValid;}

  uint8_t getCarrSolnType(){return carrSolnType;}

  friend void updatePosition(WhichGPS which, UBX_NAV_PVT_data_t *ubxDataStruct);
  friend void updateHPPosition(WhichGPS which, UBX_NAV_HPPOSLLH_data_t *ubxDataStruct);
  friend void updateRelativeHeading(WhichGPS which, UBX_NAV_RELPOSNED_data_t *ubxDataStruct);


private:
  SFE_UBLOX_GNSS gps;

  int32_t relativeHeading = 0;

  int32_t latitude = 0;
  int32_t longitude = 0;

  int8_t latitudeHP = 0;
  int8_t longitudeHP = 0;

  bool gnssFixOK = false;
  bool relPosValid = false;

  uint8_t carrSolnType = 0; // 0=none, 1=float, 2=fixed
};

void updatePosition(WhichGPS which, UBX_NAV_PVT_data_t *ubxDataStruct)
{
  GPS_Module* obj = getGPS(which);

  obj->gnssFixOK = ubxDataStruct->flags.bits.gnssFixOK;

  if (obj->gnssFixOK == false) {
    return;
  }

  obj->latitude = ubxDataStruct->lat; // 1e^-7
  obj->longitude = ubxDataStruct->lon;// 1e^-7

  obj->carrSolnType = ubxDataStruct->flags.bits.carrSoln;
}

void updateHPPosition(WhichGPS which, UBX_NAV_HPPOSLLH_data_t *ubxDataStruct)
{
  GPS_Module* obj = getGPS(which);

  if (obj->gnssFixOK == false) {
    return;
  }

  obj->latitudeHP = ubxDataStruct->latHp;  // 1e^-9
  obj->longitudeHP = ubxDataStruct->lonHp; // 1e^-9
}

void updateRelativeHeading(WhichGPS which, UBX_NAV_RELPOSNED_data_t *ubxDataStruct)
{
  GPS_Module* obj = getGPS(which);

  obj->relPosValid = ubxDataStruct->flags.bits.relPosValid;

  if (obj->relPosValid == false || obj->gnssFixOK == false) {
    return;
  }

  // Do we need to check against real length? 
  // Serial.println(((double)ubxDataStruct->relPosLength / 100) + ((double)ubxDataStruct->relPosHPLength / 10000), 4); // Convert cm to m

  obj->relativeHeading = ubxDataStruct->relPosHeading; // Convert deg * 1e^-5 to degrees
}

GPS_Module* getGPS(WhichGPS which)
{
  static GPS_Module mb_GPS(
    0x42,
    [](UBX_NAV_PVT_data_t *d) { updatePosition(WhichGPS::MovingBase, d); },
    [](UBX_NAV_HPPOSLLH_data_t *d) { updateHPPosition(WhichGPS::MovingBase, d); },
    [](UBX_NAV_RELPOSNED_data_t *d) { updateRelativeHeading(WhichGPS::MovingBase, d); }
  );
  static GPS_Module mr_GPS(
    0x43,
    [](UBX_NAV_PVT_data_t *d) { updatePosition(WhichGPS::MovingRover, d); },
    [](UBX_NAV_HPPOSLLH_data_t *d) { updateHPPosition(WhichGPS::MovingRover, d); },
    [](UBX_NAV_RELPOSNED_data_t *d) { updateRelativeHeading(WhichGPS::MovingRover, d); }
  );

  switch (which) {
    case WhichGPS::MovingBase:
      return &mb_GPS;
    case WhichGPS::MovingRover:
      return &mr_GPS;
  }
}

GPS_Module *moving_base;
GPS_Module *moving_rover;

String outBuffer;
String inBuffer;

void writeOutGPSData()
{
  outBuffer = "";
  
  outBuffer += moving_rover->IsGNSSFixOK();
  outBuffer += ",";
  outBuffer += moving_rover->IsRelPosValid();
  outBuffer += ",";
  outBuffer += moving_base->getCarrSolnType();
  outBuffer += ",";
  outBuffer += moving_rover->getCarrSolnType();
  outBuffer += ",";
  outBuffer += moving_rover->getLatitude();
  outBuffer += ",";
  outBuffer += moving_rover->getLongitude();
  outBuffer += ",";
  outBuffer += moving_rover->getRelativeHeading(); 
  outBuffer += ",";

  int str_len = outBuffer.length() + 1; // +1 for null terminator
  char char_buf[str_len];

  outBuffer.toCharArray(char_buf, str_len);

  uint32_t checksum = CRC32::calculate(char_buf, str_len - 1); // -1 to ignore null terminator

  outBuffer += checksum;

  Serial.println(outBuffer);
}

uint8_t store[256 + 3];
uint16_t numBytes = 0; // Record the number of bytes received from Serial
// uint16_t numSeqOnes = 0; // Number of sequential 1's read in serial
// 16 1's in a row denotes the ending of a valid message

void readInRTCM()
{
  // Buffer and push the RTCM data to the module
  // Code provided by SparkFun
  if (!Serial.available()) {
    Serial.println("# Serial rtcm correction not available!");
  }

  while ((Serial.available()) && (numBytes < (256 + 3))) // Check if data has been received
  {
    store[numBytes++] = Serial.read(); // Read a byte from rtcmSerial and store it

    // if (store[numBytes-1] == 255) {
    //   numSeqOnes++;
    //   Serial.println("# Found one 0xff");
    // } else {
    //   numSeqOnes = 0;
    // }
    
    // if (numSeqOnes == 2) {
    //   Serial.println("# Received correct check!");
    //   break;
    // }
  }

  // if (numSeqOnes != 2) {
  //     Serial.print("# Did not receive check 0xffff!");
  //     Serial.println((int)numSeqOnes);
  //   return;
  // }
  
  if (numBytes > 0) // Check if data was received
  {
    Serial.println("# Successfully pushing rtcm corr to gps");
    moving_base->pushRTCM(((uint8_t *)&store), numBytes /*- numSeqOnes*/); // Push the RTCM data via I2C
    numBytes = 0; // Reset numBytes
    // numSeqOnes = 0;
  }

}

void setup()
{
  delay(1000);

  // Serial.begin(115200);
  Serial.begin(460800);

  Serial.flush();

  while (!Serial){}

  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  moving_base = getGPS(WhichGPS::MovingBase);
  moving_rover = getGPS(WhichGPS::MovingRover);
  
}

void loop()
{
  moving_base->update();
  moving_rover->update();

  readInRTCM();
  Serial.println("# Sending GPS data...");
  writeOutGPSData();

  delay(500);
}