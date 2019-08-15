// Based originally off of code from:
// https://github.com/rickkas7/LIS3DH
//
#include "Serial5/Serial5.h"
#include "Particle.h"
#include "lib/TinyGPS++.h"
#include "lib/ArduCAM.h"
#include "lib/memorysaver.h"
#include "lib/SDStuff/SdFat.h"

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

// SPI SELECT PINS
const int ACCEL_CS = A2;
const int SD_CS = A1;
const int ARDUCAM_CS = A0;

// SD Variables
SdFat sd;
SdFile outFile;

// Declare initial name for output files written to SD card
// The newer versions of SdFat library support long filenames
retained char gpstxtName[] = "/GPS/GPS_9999.TXT";
retained char photoName[] = "/PHOTO/PHOTO_9999.JPG";
retained int currentFileNum = -1;
int length = 66;
int testNum = 0;

// PUBLISH DATA
char data[128];

// A sample NMEA stream.
const char *gpsStream =

  "$GNGGA,200333.00,3401.84005,N,11816.01766,W,1,04,5.82,58.8,M,-32.8,M,,*4D\r\n"
  "$GNGSA,A,3,25,12,15,,,,,,,,,,7.33,5.82,4.46*12\r\n"
  "$GNGSA,A,3,65,,,,,,,,,,,,7.33,5.82,4.46*11\r\n"
  "$GPGSV,3,1,11,02,27,056,,05,53,063,,12,35,170,20,13,06,120,19*77\r\n"
  "$GPGSV,3,2,11,15,08,155,11,20,02,212,,21,25,264,10,25,62,223,16*76\r\n"
  "$GPGSV,3,3,11,26,06,323,,29,59,338,04,31,08,283,*41\r\n"
  "$GLGSV,3,1,10,65,38,140,11,71,13,030,,72,43,076,,77,26,223,*68\r\n"
  "$GLGSV,3,2,10,78,35,284,,79,10,332,,81,04,286,,86,09,079,*67\r\n"
  "$GLGSV,3,3,10,87,46,027,,88,38,320,*66\r\n"
  "$GNGLL,3401.84005,N,11816.01766,W,200333.00,A,A*62\r\n"
  "$GNRMC,200334.00,A,3401.83973,N,11816.01812,W,2.177,,010819,,,A*7D\r\n"
  "$GNVTG,,T,,M,2.177,N,4.032,K,A*3B\r\n";


// Arducam Variables
#define VERSION_SLUG "7n"
#define TX_BUFFER_MAX 1024
uint8_t buffer[32000];
ArduCAM myCAM(OV5642, ARDUCAM_CS);

FuelGauge batteryMonitor;
TinyGPSPlus gps;

// This is the name of the Particle event to publish for battery or movement detection events
const char *eventName = "gps";

// Various timing constants
//const uint32_t PUBLISH_INTERVAL_MS = 15 * 60 * 1000;     // Only publish every fifteen minutes
const uint32_t PUBLISH_INTERVAL_MS = 6 * 1000;     // Only publish every 6 seconds
const uint32_t PUBLISH_INTERVAL_SEC = PUBLISH_INTERVAL_MS / 1000;
const uint32_t MAX_TIME_TO_PUBLISH_MS = 6 * 1000;       // Only stay awake for 60 seconds trying to connect to the cloud and publish
const uint32_t MAX_TIME_FOR_GPS_FIX_MS = 3 * 60 * 1000;  // Only stay awake for 3 minutes trying to get a GPS fix
//const uint32_t MAX_TIME_FOR_GPS_FIX_MS = 6 * 1000;  // Only stay awake for 6 seconds trying to get a GPS fix
const uint32_t TIME_AFTER_PUBLISH_MS = 4 * 1000;         // After publish, wait 4 seconds for data to go out
const uint32_t TIME_AFTER_BOOT_MS = 5 * 1000;            // At boot, wait 5 seconds before going to sleep again (after coming online)
const uint32_t PUBLISH_TTL = 60;

// Stuff for the finite state machine
enum State {
    GPS_STATE,       // WAITS FOR PARTICLE TO CONNECT TO GPS NETWORK, TO: PHOTO_STATE IF GPS SIGNAL ACQUIRED AND SOME DISTANCE TRAVERSED, SLEEP_STATE IF NOT
                          // ONLY TAKE A PHOTO IF WE HAVE TRAVELLED SOME DISTANCE
                          // ONLY UPLOAD TO CLOUD IF WE HAVE TRAVELED A DISTANCE OR WAITED A TIME
    PHOTO_STATE,          // TAKES A PHOTO IS GPS WAS ACQUIRED, TO: ONLINE_WAIT_STATE
    ONLINE_WAIT_STATE,    // WAITS FOR PARTICLE TO CONNECT TO CELL NETWORK, TO: PUBLISH_STATE IF CONNECTED, SLEEP_STATE IF NOT
    PUBLISH_STATE,        // PUBLISHES DATA TO CLOUD, TO: SLEEP_STATE
    SLEEP_STATE           // SLEEPS, TO: GPS_STATE
};

State state = GPS_STATE;
uint32_t stateTime = 0;
uint32_t startFix = 0;
bool stateChangeFlag = TRUE;

void flashRgb(uint8_t r, uint8_t g, uint8_t b, uint8_t times, uint32_t delayMs) {
    for (uint8_t i = 0; i < times; ++i) {
        RGB.color(r, g, b);
        delay(delayMs);
        RGB.color(0, 0, 0);
        delay(delayMs);
    }
}

// GPS INITIALIZATIOIN PARAMETERS
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PGCMD_ANTENNA "$PGCMD,33,1*6C"
#define PGCMD_NOANTENNA "$PGCMD,33,0*6D"

void setup() {

    Serial.begin(9600);   // CONSOLE SERIAL PORT

    // The GPS module on the AssetTracker is connected to Serial1 and D6
    Serial1.begin(9600);

    // STATE RGB - RED
    RGB.control(true);
    RGB.brightness(255);
    flashRgb(255, 0, 0, 10, 50);
    RGB.color(255, 0, 0);

    // Settings D6 LOW powers up the GPS module
    pinMode(D6, OUTPUT);
    digitalWrite(D6, LOW);

    // INITIALIZE TIMESTAMP
    startFix = millis();

    // Initialize ArduCAM
    uint8_t vid,pid;
    uint8_t temp;

    Wire.setSpeed(CLOCK_SPEED_100KHZ);
    Wire.begin();

    // INITIALIZE ACCELEROMETER AND SD CARD SPI OFF
    pinMode(ARDUCAM_CS, OUTPUT);
    pinMode(ACCEL_CS, OUTPUT);
    pinMode(SD_CS, OUTPUT);
    myCAM.CS_LOW();
    digitalWrite(ACCEL_CS, HIGH);
    digitalWrite(SD_CS, HIGH);

    Serial.println("ArduCAM Start!\n");
    SPI.begin();
    delay(500);

    while(1) {
      Serial.println("Checking for camera...");

      //Check if the ArduCAM SPI bus is OK
      myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
      delay(1000);
      temp = myCAM.read_reg(ARDUCHIP_TEST1);
      delay(1000);
      if(temp != 0x55)
      {
        Serial.println("SPI interface Error!");
        Serial.println("myCam.read_reg said " + String(temp));
        delay(1000);
      }
      else {
        break;
      }
    }

    while(1){
      //Check if the camera module type is OV5642
      myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
      myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
      if ((vid != 0x56) || (pid != 0x42)){
        Serial.println(F("Can't find OV5642 module!"));
        delay(5000);
        continue;
      }
      else {
        Serial.println(F("OV5642 detected."));
        break;
      }
    }

    Serial.println("Camera found, initializing...");

    //Change MCU mode
    myCAM.set_format(JPEG);
    delay(100);

    myCAM.InitCAM();
    delay(100);

    myCAM.set_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
    delay(100);

    myCAM.write_reg(ARDUCHIP_FRAMES,0x00);
    delay(100);

    myCAM.CS_HIGH();


    ///// SD INITIALIZATION /////
    // Initialize SdFat or print a detailed error message and halt
    // Use half speed like the native library.
    // Change to SPI_FULL_SPEED for more performance.
    digitalWrite(SD_CS, LOW);
    if (!sd.begin(SD_CS, SPI_HALF_SPEED)) {
      sd.initErrorHalt();
      Serial.println("SD ERROR\n");
    }
    else {
      Serial.println("SD GOOD\n");
    }
    digitalWrite(SD_CS, HIGH);

    Serial.println("--- SETUP COMPLETE ---\n");
}

void loop() {

    /*
    // GPS READ ATTEMPTS
    while (Serial1.available() > 0) {
        char c = Serial1.read();
        //Serial.print(c);
        if (gps.encode(c)) {
        }
    }
    */

    ///*
    // FAKE GPS READS
    while (*gpsStream) {
      if (gps.encode(*gpsStream++)) {
      }
    }
    //*/


    switch(state) {

    /////////////////////////////////////////
    ///////////// GPS WAIT STATE ////////////
    /////////////////////////////////////////
    case GPS_STATE:

        if (stateChangeFlag == TRUE){
            Serial.println("--- GPS WAIT STATE ---\n");
            stateChangeFlag = FALSE;
            flashRgb(0, 255, 0, 10, 50);
        }

        // STATE RGB - GREEN
        RGB.color(0, 255, 0);

        if (gps.location.isValid()) {

          Serial.println(F("GPS ACQUIRED"));

          ///// UPDATE FILE NAMES /////
          currentFileNum++;
          if (photoName[16] == 0x39) {
            photoName[16] = 0x30;
            gpstxtName[12] = 0x30;
            if (photoName[15] == 0x39) {
              photoName[15] = 0x30;
              gpstxtName[11] = 0x30;
              if (photoName[14] == 0x39) {
                photoName[14] = 0x30;
                gpstxtName[10] = 0x30;
                if (photoName[13] == 0x39) {
                  photoName[13] = 0x30;
                  gpstxtName[9] = 0x30;
                }
                else {
                  photoName[13]++;
                  gpstxtName[9]++;
                }
              }
              else {
                photoName[14]++;
                gpstxtName[10]++;
              }
            }
            else {
              photoName[15]++;
              gpstxtName[11]++;
            }
          }
          else {
            photoName[16]++;
            gpstxtName[12]++;
          }

          ///// PULL BATTERY AND GPS DATA TO BE WRITTEN TO SD CARD /////
          //char data[128];
          float cellVoltage = batteryMonitor.getVCell();
          float stateOfCharge = batteryMonitor.getSoC();

          //snprintf(data, sizeof(data), "{\"volt\":%.02f, \"percent\":%.02f, \"lat\":%f, \"lng\":%f, \"hdop\":%ld, \"month\":%ld, \"day\":%ld, \"year\":%ld, \"hour\":%ld, \"minute\":%ld, \"second\":%ld}",
          snprintf(data, sizeof(data), "{\"v\":%.02f, \"p\":%.02f, \"lat\":%f, \"lng\":%f, \"mon\":%d, \"d\":%d, \"h\":%d, \"min\":%d}",
                   cellVoltage,
                   stateOfCharge,
                   (float) gps.location.lat(),
                   (float) gps.location.lng(),
                   //(int)gps.hdop.value();
                   (int) gps.date.month(),
                   (int) gps.date.day(),
                   //gps.date.year(),
                   (int) gps.time.hour(),
                   (int) gps.time.minute());
                   //gps.time.second();

          ///// WRITE GPS DATA TO SD CARD HERE /////
          digitalWrite(SD_CS, LOW);
          if (!outFile.open(gpstxtName, O_RDWR | O_CREAT | O_AT_END)) {
              // If file cannot be created, raise an error
              Serial.println(F("SD card error"));
          }
          else{
            Serial.println(F("CREATED GPS TEXT FILE"));
          }

          outFile.write(data);
          delay(100);
          outFile.close();
          digitalWrite(SD_CS, HIGH);

          Serial.println(F("GPS FILE SAVED\n"));

          state = PHOTO_STATE;
          stateChangeFlag = TRUE;
          break;
        }

        if (millis() - stateTime >= MAX_TIME_FOR_GPS_FIX_MS) {
            state = SLEEP_STATE; // SLEEP_STATE
            stateChangeFlag = TRUE;
            break;
        }
        break;

    /////////////////////////////////////////
    /////////////// PHOTO STATE /////////////
    /////////////////////////////////////////
    case PHOTO_STATE: {

        if (stateChangeFlag == TRUE){
            Serial.println("--- PHOTO STATE ---\n");
            stateChangeFlag = FALSE;
            flashRgb(0, 0, 255, 10, 50);
        }

        // STATE RGB - BLUE
        RGB.color(0, 0, 255);

        Serial.println("File Name is " + String(photoName));

        ///// TAKE PHOTO AND SAVE IT TO SD CARD HERE /////
        Serial.println("\nSay cheese...\n");
        delay(200);
        Serial.println("3\n");
        delay(200);
        Serial.println("2\n");
        delay(200);
        Serial.println("1\n");
        delay(200);
        Serial.println("*CLICK*\n");
        delay(200);

        ///// TAKE PHOTO /////
        myCAM.CS_LOW();
        delay(100);

        //myCAM.OV5642_set_JPEG_size(OV5642_2592x1944);
        myCAM.OV5642_set_JPEG_size(OV5642_1280x960);
        //myCAM.OV5642_set_JPEG_size(OV5642_640x480);
        //myCAM.OV5642_set_JPEG_size(OV5642_320x240);

        delay(100);

        bool success = false;

        while(!success)
        {
            myCAM.flush_fifo();
            delay(100);
            myCAM.clear_fifo_flag();
            delay(100);
            Serial.println("STARTING CAPTURE\n");
            delay(100);
            myCAM.start_capture();
            delay(500);

            unsigned long start_time = millis(),
                          last_publish = millis();

            /////  wait for the photo to be done /////
            while(!myCAM.get_bit(ARDUCHIP_TRIG , CAP_DONE_MASK)) {
              // do nothing while waiting for capture done flag
                delay(10);
                Serial.println("CAPTURING\n");
                delay(100);
                unsigned long now = millis();
                if ((now - last_publish) > 1000) {
                    Serial.println("Status = WAITING FOR PHOTO " + String(now-start_time) + "\n");
                    last_publish = now;
                }

                if ((now-start_time) > 2000) {
                    Serial.println("Status = BAILING...\n");
                    break;
                }
            }
            if(myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)){
              success = true;
            }
          }

        delay(100);

        uint8_t temp = 0, temp_last = 0;
        int bytesRead = 0;

        if(success)
        {
            delay(100);
            Serial.println("IMAGE SUCCESSFUL\n");
            length = myCAM.read_fifo_length();
            delay(100);

            myCAM.CS_HIGH();
            digitalWrite(SD_CS, LOW);
            if (!outFile.open(photoName, O_RDWR | O_CREAT | O_AT_END)) {
                // If file cannot be created, raise an error
                Serial.println(F("SD CARD ERROR!!!\n"));
            }
            else{
              Serial.println(F("CREATED FILE\n"));
            }
            digitalWrite(SD_CS, HIGH);

            SPI.beginTransaction(SPISettings(6*MHZ, MSBFIRST, SPI_MODE0));
            myCAM.CS_LOW();
            myCAM.set_fifo_burst();
            delay(100);

            int i = 0;
            bool is_header = false;
            bool done = false;
            int sizeWritten = 0;

            while (!done){

              temp_last = temp;
              temp = SPI.transfer(0x00);

              if( (temp == 0xD9) && (temp_last == 0xFF) ){
                buffer[i++] = temp;
                myCAM.CS_HIGH();
                SPI.endTransaction();
                digitalWrite(SD_CS, LOW);
                outFile.write(buffer, i);
                delay(100);
                digitalWrite(SD_CS, HIGH);
                sizeWritten += i;
                is_header = false;
                i = 0;
                done = true;
                Serial.println("FILE SIZE WRITTEN = " + String(sizeWritten));
              }

              if(is_header == true){
                if(i<sizeof(buffer)){
                  buffer[i++] = temp;
                }
                else{

                  myCAM.CS_HIGH();
                  SPI.endTransaction();

                  if(!( (buffer[3000] == 0x55) && (buffer[4000] == 0x55) && (buffer[5000] == 0x55) ) ){
                    digitalWrite(SD_CS, LOW);
                    outFile.write(buffer, sizeof(buffer));
                    delay(100);
                    Serial.println("WROTE " + String(sizeof(buffer)));
                    sizeWritten += sizeof(buffer);
                    digitalWrite(SD_CS, HIGH);
                  }
                  else{
                    Serial.println("CHUNK OF 0x55 SKIPPED");
                  }

                  i = 0;
                  if( !( (temp == 0x55) && (temp_last == 0x55) ) ){
                    buffer[i++] = temp;
                  }
                  else{
                    i++;
                  }

                  SPI.beginTransaction(SPISettings(6*MHZ, MSBFIRST, SPI_MODE0));
                  myCAM.CS_LOW();
                  myCAM.set_fifo_burst();
                  delay(100);
                }
              }
              else if ((temp == 0xD8) && (temp_last == 0xFF)){
                is_header = true;
                Serial.println("FOUND HEADER");
                buffer[i++] = temp_last;
                buffer[i++] = temp;
              }
              else{
                Serial.println("TEMP = " + String(temp));

                if( (temp == 0x55) && (temp_last == 0x55) ){
                  myCAM.CS_HIGH();
                  SPI.endTransaction();
                  delay(100);
                  Serial.println("INITIAL CHUNK OF 0x55 SKIPPED");
                  SPI.beginTransaction(SPISettings(6*MHZ, MSBFIRST, SPI_MODE0));
                  myCAM.CS_LOW();
                  myCAM.set_fifo_burst();
                  delay(100);
                }

              }

            }

            digitalWrite(SD_CS, LOW);
            outFile.close();
            digitalWrite(SD_CS, HIGH);
            Serial.println("\nOUTFILE CLOSED\n");

          }

        Serial.println("Sleeping 1 second...\n");
        delay(1 * 1000);

        /*
        testNum++;

        if (testNum > 5) {
          state = SLEEP_STATE;
          stateChangeFlag = TRUE;
        }
        else {
          //state = ONLINE_WAIT_STATE; // ONLINE_WAIT_STATE
          state = GPS_STATE;
          stateChangeFlag = TRUE;
        }*/

        state = ONLINE_WAIT_STATE;
        stateChangeFlag = TRUE;

        break;
    }

    /////////////////////////////////////////
    /////////// ONLINE WAIT STATE ///////////
    /////////////////////////////////////////
    case ONLINE_WAIT_STATE:

        if (stateChangeFlag == TRUE){
            Serial.println("--- ONLINE WAIT STATE ---\n");
            stateChangeFlag = FALSE;
            flashRgb(255, 0, 255, 10, 50);

            // ATTEMPT TO CONNECT
            Serial.println("Attempting Cellular Connection...\n");
            Cellular.on();
            Particle.connect();
        }

        // STATE RGB - MAGENTA
        RGB.color(255, 0, 255);

        if (Particle.connected()) {
            Serial.println("Particle connected!\n");
            state = PUBLISH_STATE;
            stateChangeFlag = TRUE;
        }

        if (millis() - stateTime > 5000) {
            stateTime = millis();
        }

        break;

    /////////////////////////////////////////
    ///////////// PUBLISH STATE /////////////
    /////////////////////////////////////////
    case PUBLISH_STATE: {

        if (stateChangeFlag == TRUE){
            Serial.println("--- PUBLISH STATE ---\n");
            stateChangeFlag = FALSE;
            flashRgb(0, 255, 255, 10, 50);
        }

        // STATE RGB - TEAL
        RGB.color(0, 255, 255);


        if (Particle.connected()) {
            Serial.println("(Here is where we would publish to the cloud.)\n");

            ///PUBLISH TO CLOUD
            Particle.publish("GPS_Data", data, 60, PRIVATE);

            Particle.disconnect();
            Cellular.off();
            Serial.println("Particle is now disconnected and cellular is off...\n");

            stateTime = millis();
            state = SLEEP_STATE;
            stateChangeFlag = TRUE;
        }
        else {
            if (millis() - stateTime >= MAX_TIME_TO_PUBLISH_MS) {
                state = SLEEP_STATE;
                stateChangeFlag = TRUE;
            }
        }
        break;
      }

    /////////////////////////////////////////
    ////////////// SLEEP STATE //////////////
    /////////////////////////////////////////
    case SLEEP_STATE:

        if (stateChangeFlag == TRUE){
            Serial.println("--- SLEEP STATE ---\n");
            stateChangeFlag = FALSE;
            flashRgb(255, 255, 0, 10, 50);
        }

        // STATE RGB - YELLOW
        RGB.color(255, 255, 0);

        Serial.println("GOING TO SLEEP\n");
        SPI.end();

        System.sleep(SLEEP_MODE_DEEP, PUBLISH_INTERVAL_SEC);

        // This delay should not be necessary, but sometimes things don't seem to work right
        // immediately coming out of sleep.
        delay(2000);

        startFix = millis();
        Serial.begin(9600);
        Serial.println("Good Morning!\n");
        state = GPS_STATE;
        stateChangeFlag = TRUE;
        stateTime = millis();
        break;
      }
}
