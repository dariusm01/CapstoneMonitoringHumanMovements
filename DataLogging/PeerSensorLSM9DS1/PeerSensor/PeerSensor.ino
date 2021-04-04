//EPROM Library
#include <EEPROM.h>
#include <Wire.h>
#include <SPI.h>
//Wifi Lib
#include <WiFi.h>
#include <esp_now.h>
//MicroSD Library from ESP32
#include "FS.h"
#include "SD.h"
#include <SparkFunLSM9DS1.h>

#define EEPROM_SIZE 64    //EEPROM size (1-512)
#define SD_CS 16          //slave select pin for SD (can be any GPIO)
#define NUMMAIN 20
#define CHANNEL 1
#define PRINTSCANRESULTS 0
#define PINGS 12
#define CLOCKDATA 8
#define BUFFERSIZE 11904
#define MAXMESSAGELENGTH 30
#define SCANTIME 12000

LSM9DS1 imu;

//global Integers for EEPROM
static const int EP_SET = 0;
static const int EP_S1  = 1;
static const int EP_S2 = 2;

//Defining the States that the Sensor can be in
//START: is the state when the board has just started and is setting up pins, structs and objects
//STANDBY: This is the state where the board is waiting for a input from the user
//FILE_CREATE: The Board will Create the directories needed for the current session
//PAIR: The board is actively searching for other sensors to sync with
//DATA_LOG: The Board is going to start logging data from the sensors
//SLEEP: The board is in sleep mode
enum ESP_state : uint8_t {
  START, STANDBY, FILE_CREATE, PAIR, DATA_LOG, SLEEP
};


//Defining the current state of the sensor to the startup state
ESP_state curr_state = START;

//The pin that controls the board LED
const int BoardLED = 13;

//Setting the touch constants
const int powerPin = 12;
const int wirelessPin = 27;
const int batVoltagePin = 35;

//Where the wakeup status will be stored
touch_pad_t touchPin;

//Boolean that checks if the ESP was alseep
boolean wasAsleep = false;

//Pin wire touch value: this is subject to change depending on the material setup with the touch sensor
const int pinTouch = 48;

//Button press type
uint8_t pressType;

//PWM Settings
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

//Creating session Global Variable
uint16_t session;

//DOESNT WORK
//Defining structs for file paths
//Raw data struct
//typedef struct rawDataPath {
//  const char* a_path;           //the file path for the raw accelerometer data
//  const char* g_path;           //the file path for the raw gyroscope data
//  const char* m_path;           //the file path for the raw magnetometer data
//} rawDataPath;                  //naming the struct
//
//typedef struct filePaths {
//  struct rawDataPath raw;
//  String base;                  //base path for folder
//  const char* config;           //I have no Idea what this was supposed to be
//  const char* base_c;
//} filePaths;
//
////Defining struct
//filePaths path;

//const char* a_path;
//const char* g_path;
//const char* m_path;
//const char* config_path;
//String baseFolder;
String a_path;
String g_path;
String m_path;
String folder;
const char* a_path_c;
const char* g_path_c;
const char* m_path_c;
String Sensor_ID = "2_ESP32";

esp_now_peer_info_t main[NUMMAIN] = {};
int mainCnt = 0;

bool registered = false;


typedef struct sensorInfo {
  unsigned long int mSentClk;
  unsigned long int pClk;
  bool synced;
} sensorInfo;

sensorInfo recvData;

typedef struct deltaClk {
  bool sign = true;
  unsigned long int clk = 0;
} deltaClk;

deltaClk dClkResult;

String Acc, Gyro, Mag;

String rawDataString;

bool stopSync = false;
unsigned long int stopSyncStart;

char wBuffer[BUFFERSIZE];

float calcVolt;

void setup() {
  //Start the serial monitor
  Serial.begin(115200);
  delay(1000);
  Serial.println("Initializing Board...");

  //Initializing EEORM
  EEPROM.begin(EEPROM_SIZE);

  session = session_count();
  Serial.print("Device Session Number: ");
  Serial.println(session);

  Serial.println("Generating Folders and Config File");
  Wire.begin(); //starting wires
  Wire.setClock(1000000);
  //initialize microSD card module
  while (!initializeMicroSD()) {
    Serial.println("Something wrong with microSD card or module");
    Serial.println("Check wiring or if card is present");
    delay(2000);  //will wait 5 seconds before trying again
  }
  createFiles();
  //  while (!baseFileCreate()) {
  //    Serial.println("Something went wrong and i cant make these files man.");
  //    delay(5000);
  //  }

  //  while(!imuRawFileCreate()){
  //    Serial.println("Something went wrong trying to create the IMU files");
  //    delay(5000);
  //  }

  //Setting the PWM channel settings and attaching BoardLED to the channel
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(BoardLED, ledChannel);

  //Setting up Touch Intterrupts
  touchAttachInterrupt(T5, callback, pinTouch); //Power
  touchAttachInterrupt(T7, callback, pinTouch); //Wireless

  //Setup is complete and switching to STANDBY
  Serial.println("Initializing is complete!");
  Serial.println("Standing by for next command");
  curr_state = STANDBY;
}

void loop() {
  //Checking the state
  switch (curr_state) {
    //Standby state is waiting for sensor to
    case STANDBY :
      //Was the Sensor in deep sleep? (Powered off)
      if (wasAsleep) {
        //this will never run
        //Save something to the EEPROM
        print_wakeup_reason();
        print_wakeup_touchpad();
        wasAsleep = false;
      }
      //Checking for Button Presses
      if (touchRead(powerPin) < pinTouch) {
        //The power pin has detected a touch and will now determine what type of press
        pressType = buttonPress(millis(), powerPin);
        switch (pressType) {
          case 1:
            Serial.println("Battery Level Requested");
            Serial.print("Current Voltage: ");
            calcVolt = (analogRead(batVoltagePin)*2*3.7)/4095;
            Serial.print(calcVolt);
            Serial.println("V");
            break;
          case 2:
            //Putting ESP32 to sleep to save power while 'off'
            curr_state = SLEEP;
            break;
        }
      }
      else if (touchRead(wirelessPin) < pinTouch) {
        //The wireless pin has detected a touch and will now determine what type of press
        pressType = buttonPress(millis(), wirelessPin);
        switch (pressType) {
          case 1:
            Serial.println("Scanning for sensors to pair to");
            curr_state = PAIR;
            initWiFi();
            break;
          case 2:
            //Putting ESP32 to sleep to save power while 'off'
            Serial.println("Starting to Datalog");
            //Start IMU
            //Generate IMU Files
            if (initIMU()) {
              curr_state = DATA_LOG;
            }
            break;
        }
      }
      delay(50); //this is so the loop doesnt keep running at full speed during standby
      break;
    case SLEEP :
      delay(2000);
      esp_sleep_enable_touchpad_wakeup();
      Serial.println("Entering Sleep Mode");
      //Nothing after this line will be called in this case statement
      esp_deep_sleep_start();
      break;
    case PAIR :
      if (!registered) {
        scanForMain();
        if (mainCnt > 0) {
          registered = true;
          managePeer();
          curr_state = STANDBY;
          Serial.println("Going to STANDBY");
        }
        else{
          delay(1000);
        }
      }
      break;
    case DATA_LOG :
      //      if (imu.gyroAvailable()) {
      //        imu.readGyro();
      //        Gyro = String(millis()) + "," + String(imu.gx) + "," + String(imu.gy) + "," + String(imu.gz) + "\r\n";
      //        appendFile(SD, g_path, Gyro.c_str());
      //      }
      //      if (imu.accelAvailable()) {
      //        imu.readAccel();
      //        Acc = String(millis()) + "," + String(imu.ax) + "," + String(imu.ay) + "," + String(imu.az) + "\r\n";
      //        appendFile(SD, a_path, Acc.c_str());
      //      }
      //      if (imu.magAvailable()) {
      //        imu.readMag();
      //        Mag = String(millis()) + "," + String(imu.mx) + "," + String(imu.my) + "," + String(imu.mz) + "\r\n";
      //        appendFile(SD, m_path, Mag.c_str());
      //      }
      if (dClkResult.sign) { //add
//        if (imu.gyroAvailable()) {
//          imu.readGyro();
//          Gyro = String(millis() + dClkResult.clk) + "," + String(imu.gx) + "," + String(imu.gy) + "," + String(imu.gz) + "\r\n";
//          appendFile(SD, a_path_c, Gyro.c_str());
//        }
//        if (imu.accelAvailable()) {
//          imu.readAccel();
//          Acc = String(millis() + dClkResult.clk) + "," + String(imu.ax) + "," + String(imu.ay) + "," + String(imu.az) + "\r\n";
//          appendFile(SD, g_path_c, Acc.c_str());
//        }
//        if (imu.magAvailable()) {
//          imu.readMag();
//          Mag = String(millis() + dClkResult.clk) + "," + String(imu.mx) + "," + String(imu.my) + "," + String(imu.mz) + "\r\n";
//          appendFile(SD, m_path_c, Mag.c_str());
//        }

        if (imu.accelAvailable()){
        imu.readAccel();
        rawDataString = String(millis()+ dClkResult.clk) + "," + String(imu.ax) + "," + String(imu.ay) + "," + String(imu.az)+ ",a\n";
        strcat(wBuffer,rawDataString.c_str());
        if(BUFFERSIZE - strlen(wBuffer) < MAXMESSAGELENGTH){
            //unsigned long int startWrite = millis();
            appendFile(SD,g_path_c,wBuffer);
            //Serial.println(millis() - startWrite);
            memset(wBuffer, 0, sizeof(wBuffer));
          }
      }

      if (imu.gyroAvailable()){
        imu.readGyro();
        rawDataString = String(millis()+ dClkResult.clk) + "," + String(imu.gx) + "," + String(imu.gy) + "," + String(imu.gz)+ ",g\n";
        strcat(wBuffer,rawDataString.c_str());
        if(BUFFERSIZE - strlen(wBuffer) < MAXMESSAGELENGTH){
            //unsigned long int startWrite = millis();
            appendFile(SD,g_path_c,wBuffer);
            //Serial.println(millis() - startWrite);
            memset(wBuffer, 0, sizeof(wBuffer));
          }
      }

      if (imu.magAvailable()){
        imu.readMag();
        rawDataString = String(millis()+ dClkResult.clk) + "," + String(imu.mx) + "," + String(imu.my) + "," + String(imu.mz)+ ",m\n";
        strcat(wBuffer,rawDataString.c_str());
        if(BUFFERSIZE - strlen(wBuffer) < MAXMESSAGELENGTH){
            //unsigned long int startWrite = millis();
            appendFile(SD,g_path_c,wBuffer);
            //Serial.println(millis() - startWrite);
            memset(wBuffer, 0, sizeof(wBuffer));
          }
      }


    if(touchRead(powerPin) < pinTouch && !stopSync){
      Serial.println("Attempt to stop collecting data");
      stopSyncStart = millis();
      stopSync = true;
    }

    if(stopSync){
      if(touchRead(powerPin) <  pinTouch){
        if(millis() - stopSyncStart > 3000){
          Serial.println("Power button Held...");
          curr_state = SLEEP;
        }
      }
      else{
        stopSync = false;
      }
    }


        
      }
      else {
//        if (imu.gyroAvailable()) {
//          imu.readGyro();
//          Gyro = String(millis() - dClkResult.clk) + "," + String(imu.gx) + "," + String(imu.gy) + "," + String(imu.gz) + "\r\n";
//          appendFile(SD, a_path_c, Gyro.c_str());
//        }
//        if (imu.accelAvailable()) {
//          imu.readAccel();
//          Acc = String(millis() - dClkResult.clk) + "," + String(imu.ax) + "," + String(imu.ay) + "," + String(imu.az) + "\r\n";
//          appendFile(SD, g_path_c, Acc.c_str());
//        }
//        if (imu.magAvailable()) {
//          imu.readMag();
//          Mag = String(millis() - dClkResult.clk) + "," + String(imu.mx) + "," + String(imu.my) + "," + String(imu.mz) + "\r\n";
//          appendFile(SD, m_path_c, Mag.c_str());
//        }
if (imu.accelAvailable()){
        imu.readAccel();
        rawDataString = String(millis()- dClkResult.clk) + "," + String(imu.ax) + "," + String(imu.ay) + "," + String(imu.az)+ ",a\n";
        strcat(wBuffer,rawDataString.c_str());
        if(BUFFERSIZE - strlen(wBuffer) < MAXMESSAGELENGTH){
            //unsigned long int startWrite = millis();
            appendFile(SD,g_path_c,wBuffer);
            //Serial.println(millis() - startWrite);
            memset(wBuffer, 0, sizeof(wBuffer));
          }
      }

      if (imu.gyroAvailable()){
        imu.readGyro();
        rawDataString = String(millis()- dClkResult.clk) + "," + String(imu.gx) + "," + String(imu.gy) + "," + String(imu.gz)+ ",g\n";
        strcat(wBuffer,rawDataString.c_str());
        if(BUFFERSIZE - strlen(wBuffer) < MAXMESSAGELENGTH){
            //unsigned long int startWrite = millis();
            appendFile(SD,g_path_c,wBuffer);
            //Serial.println(millis() - startWrite);
            memset(wBuffer, 0, sizeof(wBuffer));
          }
      }

      if (imu.magAvailable()){
        imu.readMag();
        rawDataString = String(millis()- dClkResult.clk) + "," + String(imu.mx) + "," + String(imu.my) + "," + String(imu.mz)+ ",m\n";
        strcat(wBuffer,rawDataString.c_str());
        if(BUFFERSIZE - strlen(wBuffer) < MAXMESSAGELENGTH){
            //unsigned long int startWrite = millis();
            appendFile(SD,g_path_c,wBuffer);
            //Serial.println(millis() - startWrite);
            memset(wBuffer, 0, sizeof(wBuffer));
          }
      }


    if(touchRead(powerPin) < pinTouch && !stopSync){
      Serial.println("Attempt to stop collecting data");
      stopSyncStart = millis();
      stopSync = true;
    }

    if(stopSync){
      if(touchRead(powerPin) <  pinTouch){
        if(millis() - stopSyncStart > 3000){
          Serial.println("Power button Held...");
          curr_state = SLEEP;
        }
      }
      else{
        stopSync = false;
      }
    }
      }

      break;
  }

}

boolean initIMU() {
  if (imu.begin() == false) {
    return false;
  }
  return true;
}

//boolean baseFileCreate() {
//  //creating directories
//  //create config file
//  String configFolder, cfig;
//  String Mac = WiFi.macAddress();
//  Mac.replace(":", "_");
//  baseFolder = "/" + Mac + "_" + String(session);
//
//  configFolder = baseFolder + "/config";
//  cfig = configFolder + "/config.txt";
//  config_path = cfig.c_str();
//  File configFile = SD.open(config_path);
//  if (!configFile) {
//    Serial.println("Creating Files and Directories");
//    createDir(SD, baseFolder.c_str());
//    createDir(SD, configFolder.c_str());
//    writeFile(SD, config_path, "------------------------------------");
//  }
//  else {
//    Serial.println("File already Exisits!");
//    return false;
//  }
//  configFile.close();
//  return true;
//}
//
//boolean imuRawFileCreate() {
//  String rawFolder = baseFolder + "/rawdata";
//  String a_pat, g_pat, m_pat;
//  a_pat = rawFolder + "/Acc.csv";
//  g_pat = rawFolder + "/Gyro.csv";
//  m_pat = rawFolder + "/Mag.csv";
//  a_path = a_pat.c_str();
//  g_path = g_pat.c_str();
//  m_path = m_pat.c_str();
//
//  File a_file = SD.open(a_path);
//  File g_file = SD.open(g_path);
//  File m_file = SD.open(m_path);
//
//  if (!a_file || !g_file || m_file) {
//    Serial.println("Files are not found");
//    Serial.println("Creating Directory and files...");
//    createDir(SD, rawFolder.c_str());
//    writeFile(SD, a_path, "Time(milliseconds),X,Y,Z\r\n");
//    writeFile(SD, g_path, "Time(milliseconds),X,Y,Z\r\n");
//    writeFile(SD, m_path, "Time(milliseconds),X,Y,Z\r\n");
//  }
//  else {
//    Serial.println("Files already exist! Something went wrong.");
//    return false;
//  }
//  String confup = "Successfully generated raw data IMU Files\r\n";
//  appendFile(SD, config_path, confup.c_str());
//  return true;
//}

void createFiles() {
  folder =  "/" + Sensor_ID + "_" + String(session);
  a_path = "/" + Sensor_ID + "_" + String(session) + "/" + Sensor_ID + "_" + String(session) +  "Acc.csv";
  g_path = "/" + Sensor_ID + "_" + String(session) + "/" + Sensor_ID + "_" + String(session) +  "Gyro.csv";
  m_path = "/" + Sensor_ID + "_" + String(session) + "/" + Sensor_ID + "_" + String(session) +  "Mag.csv";
  a_path_c = a_path.c_str();
  g_path_c = g_path.c_str();
  m_path_c = m_path.c_str();
  //creating files objects
  File a_file = SD.open(a_path_c);
  File g_file = SD.open(g_path_c);
  File m_file = SD.open(m_path_c);
  //Checking if any of the files exists
  if (!a_file || !g_file || m_file) {
    Serial.println("Files are not found");
    Serial.println("Creating Directory and files...");
    createDir(SD, folder.c_str());
    writeFile(SD, a_path_c, "Time(milliseconds),X,Y,Z\r\n");
    writeFile(SD, g_path_c, "Time(milliseconds),X,Y,Z\r\n");
    writeFile(SD, m_path_c, "Time(milliseconds),X,Y,Z\r\n");
  }
  else {
    Serial.println("Files already exists! Something went wrong");
    return;
  }
  //closing the files
  a_file.close();
  g_file.close();
  m_file.close();
}

void initWiFi() {
  WiFi.mode(WIFI_STA);            //setting device into station mode

  configDeviceAP();             //configure device
  InitESPNow();               //initialize ESP NOW

  esp_now_register_send_cb(OnDataSent);   //Registering callback function when data is sent
  esp_now_register_recv_cb(OnDataRecv);   //Registering callback funtion when data is recv
}

void deinitWiFi() {
  if (esp_now_deinit() == ESP_OK) {
    WiFi.mode(WIFI_OFF);
    WiFi.disconnect();
    //    String confup = "ESP NOW WiFi driver successfully stopped\r\n";
    //    appendFile(SD, config_path, confup.c_str());
  }
}

// config AP SSID
void configDeviceAP() {
  String Prefix = "Peer:";
  String Mac = WiFi.macAddress();
  String SSID = Prefix + Mac;
  String Password = "123456789";
  bool result = WiFi.softAP(SSID.c_str(), Password.c_str(), CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}

void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}





/*
   Description: Initalizes the microSD card module. Checks wiring and if card is present

   Input: NONE

   Output: Boolean saying if the initalization was completed or not

*/
boolean initializeMicroSD() {
  SD.begin(SD_CS);                                            //starting the SD card
  if (!SD.begin(SD_CS)) {                                     //If SD.begin returns a false it means the card failed to mount
    Serial.println("Card Mount Failed");
    return false;
  }
  uint8_t cardType = SD.cardType();                           //Saving the SD card type into a unsigned int of 8 bits
  if (cardType == CARD_NONE) {                                //checking if the card type is returning no card
    Serial.println("No SD card attached");                    //returned no card therefore there is no card
    return false;
  }                                                           //Card successful detected
  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("ERROR - SD card initialization failed!");
    return false;
  }
  return true;                                                //successful setup
}


/*
   Description: Will determine the button press made

   Input: start - the clock value at the inital button press
          pin - The number that was pressed

   Output:
          0 - Rejected press
          1 - Short press (101-2999 ms)
          2 - Long Press (3000ms)
*/
uint8_t buttonPress(unsigned long start, const int pin) {
  while (touchRead(pin) < pinTouch) {
    if (millis() - start > 3000) {
      Serial.print("Long Press on pin ");
      Serial.print(pin);
      Serial.print(" detected");
      Serial.println("");
      //want feedback for the user to remove their hand from button
      for (int dutyCycle = 255; dutyCycle >= 0; dutyCycle--) {
        ledcWrite(ledChannel, dutyCycle);
        delay(5);
      }
      return 2;
    }
  }
  if (millis() - start > 100) {
    Serial.print("Short Press on pin ");
    Serial.print(pin);
    Serial.print(" detected");
    Serial.println("");
    ledcWrite(ledChannel, 255);
    delay(100);
    ledcWrite(ledChannel, 0);
    return 1;
  }
  Serial.println("Rejected Press");
  return 0;
}

/*
   Description: Will determine the session number of the device. If the session is not set, it will set it.
                Uses the first three bytes of EEPROM: 0,1,2

   Input:
          EEPROM 0: If set - 1
          EEPROM 1:
          EEPROM 2:

   Output:
          unsigned 16 bit integer of the session number
*/
uint16_t session_count() {
  uint16_t s = 1;                                                   //local var for session number

  if (byte(EEPROM.read(EP_SET)) == 1) {                             //is the session saved in EEPROM set?
    s = byte(EEPROM.read(EP_S1));
    s = s << 8;
    s = s + byte(EEPROM.read(EP_S2));                              //Setting session number: s = S1 S2
    if (byte(EEPROM.read(EP_S2)) == 255) {                         //Checking if S2 will overflow
      EEPROM.write(EP_S1, byte(EEPROM.read(EP_S1)) + 1);            //Write: Add 1 to S1 because of overflow
      EEPROM.write(EP_S2, 0);                                       //Write: S2 to 0
    }
    else {                                                         //No Overflow in S2
      EEPROM.write(EP_S2, byte(EEPROM.read(EP_S2)) + 1);            //Write: Add 1 to S2
    }
  }
  else {                                                            //Not set
    EEPROM.write(EP_S1, 0);                                         //Write S1 to 0
    EEPROM.write(EP_S2, 1);                                         //Write S2 to 1
    EEPROM.write(EP_SET, 1);                                        //Setting set
  }
  EEPROM.commit();                                                  //saving session info
  return s;                                                         //returning session
}


/*
  Method to print the reason by which ESP32
  has been awaken from sleep
*/
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

/*
  Method to print the touchpad by which ESP32
  has been awaken from sleep
*/
void print_wakeup_touchpad() {
  touchPin = esp_sleep_get_touchpad_wakeup_status();
  switch (touchPin) {
    case 0  : Serial.println("Touch detected on GPIO 4"); break;
    case 1  : Serial.println("Touch detected on GPIO 0"); break;
    case 2  : Serial.println("Touch detected on GPIO 2"); break;
    case 3  : Serial.println("Touch detected on GPIO 15"); break;
    case 4  : Serial.println("Touch detected on GPIO 13"); break;
    case 5  : Serial.println("Touch detected on GPIO 12"); break;
    case 6  : Serial.println("Touch detected on GPIO 14"); break;
    case 7  : Serial.println("Touch detected on GPIO 27"); break;
    case 8  : Serial.println("Touch detected on GPIO 33"); break;
    case 9  : Serial.println("Touch detected on GPIO 32"); break;
    default : Serial.println("Wakeup not by touchpad"); break;
  }
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message) {
  //Serial.printf("Appending to file: %s\n", path);
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void createDir(fs::FS &fs, const char * path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}


// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  Serial.print("Data recieved size: ");
  Serial.println(data_len);
  if (data_len == PINGS) {
    if (registered) {
      const uint8_t *main_addr = main[0].peer_addr;
      memcpy(&recvData, data, sizeof(recvData));
      if (!recvData.synced) {
        recvData.pClk = millis();
        esp_err_t result = esp_now_send(main_addr, (uint8_t*) &recvData, sizeof(recvData));
        if (result == ESP_OK) {
          Serial.println("Successful valid response");
        }
      }
      else {
      }
    }
  }
  else if (data_len == CLOCKDATA) {
    memcpy(&dClkResult, data, sizeof(dClkResult));
    Serial.print("Clock Difference: ");
    if (!dClkResult.sign) {
      Serial.print("-");
    }
    Serial.println(dClkResult.clk);
    Serial.println("Recieved Synced Signal");
    Serial.println("Turning off WiFi device");
    if (esp_now_deinit() == ESP_OK) {
      Serial.println("WiFi driver stopped");
      //WiFi.mode(WIFI_OFF);
    }
    WiFi.mode(WIFI_OFF);
    WiFi.disconnect();
  }


}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Data sent successfully");
  }
  else {
    Serial.println("Failed to send!");
  }
}


// Scan for peers in AP mode
void scanForMain() {
  int8_t scanResults = WiFi.scanNetworks();
  //reset peers
  memset(main, 0, sizeof(main));
  mainCnt = 0;
  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `Main`
      if (SSID.indexOf("Main") == 0) {
        // SSID of interest
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Main
        int mac[6];

        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            main[mainCnt].peer_addr[ii] = (uint8_t) mac[ii];

          }
        }
        main[mainCnt].channel = CHANNEL; // pick a channel
        main[mainCnt].encrypt = 0; // no encryption
        mainCnt++;
      }
    }
  }

  if (mainCnt > 0) {
    Serial.print(mainCnt); Serial.println(" Main(s) found, processing..");
  } else {
    Serial.println("No Main Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

void managePeer() {
  if (mainCnt > 0) {
    for (int i = 0; i < mainCnt; i++) {
      Serial.print("Processing: ");
      for (int ii = 0; ii < 6; ++ii ) {
        Serial.print((uint8_t) main[i].peer_addr[ii], HEX);
        if (ii != 5) Serial.print(":");
      }
      Serial.print(" Status: ");
      // check if the peer exists
      bool exists = esp_now_is_peer_exist(main[i].peer_addr);
      if (exists) {
        // Peer already paired.
        Serial.println("Already Paired");
      } else {
        // Peer not paired, attempt pair
        esp_err_t addStatus = esp_now_add_peer(&main[i]);
        if (addStatus == ESP_OK) {
          // Pair success
          Serial.println("Pair success");
          //create sensorSync struct

        } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
          // How did we get so far!!
          Serial.println("ESPNOW Not Init");
        } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
          Serial.println("Add Peer - Invalid Argument");
        } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
          Serial.println("Peer list full");
        } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
          Serial.println("Out of memory");
        } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
          Serial.println("Peer Exists");
        } else {
          Serial.println("Not sure what happened");
        }
        delay(100);
      }
    }
  } else {
    // No peer found to process
    Serial.println("No Peers found to process");
  }
}


//Not going to use these because if held it will cause an error, but I need them
void callback() {
  //will be executed if awake
}
