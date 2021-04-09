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
//IMU Lib
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_Sensor.h>
//Screen Lib
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define EEPROM_SIZE 64    //EEPROM size (1-512)
#define SD_CS 16          //slave select pin for SD (can be any GPIO)
#define NUMPEERS 20               //max number of paired devices at one time
#define CHANNEL 1                 //channel to send info
#define PRINTSCANRESULTS 0
#define BUFFERSIZE 11904
#define MAXMESSAGELENGTH 90
#define SCANTIME 12000
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
sensors_event_t event,aevent, mevent;


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

//the two different states for the syncing process, using a enum for readability
enum syncingState : uint8_t {
  SCANNING, SYNCING
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

String a_path;
String g_path;
String m_path;
String folder;
const char* a_path_c;
const char* g_path_c;
const char* m_path_c;
String Sensor_ID = "1_ESP32";

//The struct of the data being sent to each esp for the syncing process
typedef struct sensorInfo {
  unsigned long int mSentClk;         //the clock value that is sent from the main sensor
  unsigned long int pClk;             //the peer clock
  //uint8_t packet = 0;                 //the number of packets sent
  bool synced = false;                //the sync status of the peer
} sensorInfo;                         //refer to the struct as sensorInfo

typedef struct deltaClk {
  bool sign;
  unsigned long int clk;
} deltaClk;

deltaClk dClk[NUMPEERS][10] = {};
deltaClk dClkResult;

uint8_t packet = 0;

//defining vars
syncingState currSyncStatus = SCANNING;       //setting Sync Status state
esp_now_peer_info_t peers[NUMPEERS] = {};     //esp now struct to save registered peer info
int peerCnt = 0;                              //Number of current peers found
//syncing loop vars
uint8_t peerIndex = 0, paired = 0, fpaired = 0; //index of current peer, # of paired and # of failed pairs
bool pairing = false;                         //is a peer being paired currently?
unsigned long int startPairingTime, scanStart;          //start of the pairing time

sensorInfo recvData;              //where the data recived is stored
sensorInfo sendData;              //where the data sent is stored


bool syncStatus[NUMPEERS] = {1};          //sync status of the peers

unsigned long int latency[NUMPEERS][10];    //latency of the pings

String rawDataString;

unsigned long int tempNum = 0;

bool stopSync = false;
unsigned long int stopSyncStart, screenRefresh;

char wBuffer[BUFFERSIZE];

float calcVolt;

void setup() {
  //Start the serial monitor
  Serial.begin(115200);
  delay(1000);
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  displayFakeInit();  //lmao
  //Initializing EEORM
  EEPROM.begin(EEPROM_SIZE);

  session = session_count();
  Serial.print("Device Session Number: ");
  Serial.println(session);

  Serial.println("Generating Folders and Config File");
  Wire.begin(); //starting wires
  Wire.setClock(400000);

  //initialize microSD card module
  while (!initializeMicroSD()) {
    Serial.println("Something wrong with microSD card or module");
    Serial.println("Check wiring or if card is present");
    displayNoSDCard();
    delay(2000);  //will wait 2 seconds before trying again
  }
  createFiles();


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
  displaySTANDBY();
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
            calcVolt = (analogRead(batVoltagePin)*2*(3.5))/4095;
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
            displayPairing();
            scanStart = millis();
            break;
          case 2:
            //Putting ESP32 to sleep to save power while 'off'
            Serial.println("Starting to Datalog");
            //Start IMU
            //Generate IMU Files
            if (initIMU()) {
              curr_state = DATA_LOG;
              displayDatalog();
              screenRefresh = millis();
            }
            else{
              displayNoIMU();
              delay(5000);
              curr_state = STANDBY;
              displaySTANDBY();
            }
            break;
        }
      }
      if(millis() - screenRefresh > 5000){
        displaySTANDBY();
      }
      delay(10); //this is so the loop doesnt keep running at full speed during standby
      break;
    case SLEEP :
      displayNoPower();
      delay(2000);
      display.clearDisplay();
      display.display();
      esp_sleep_enable_touchpad_wakeup();
      Serial.println("Entering Sleep Mode");
      //Nothing after this line will be called in this case statement
      esp_deep_sleep_start();
      break;
    case PAIR :
      while ( millis() - scanStart < SCANTIME) {
        switch (currSyncStatus) {
          case SCANNING :
            scanForPeers();
            if (peerCnt > 0) {
              managePeer();
              currSyncStatus = SYNCING;
            }
            delay(1000);
            break;
          case SYNCING :
            if (peerCnt > peerIndex) {
              if (pairing) {
                if (syncStatus[peerIndex]) {
                  calcAverageClk();

                  paired++;
                  peerIndex++;
                  pairing = false;
                  sendData.synced = false;
                  packet = 0;
                  Serial.println("---------------------------------------");
                }
                if ((millis() - startPairingTime) > 5000) {
                  fpaired++;
                  peerIndex++;
                  pairing = false;
                  sendData.synced = false;
                  packet = 0;
                  Serial.println("---------------------------------------");
                }
              }
              else {
                pairing = true;
                startPairingTime = millis();
                if (peerIndex == 0) {
                  Serial.println("Starting to Pair");
                }
                sensorSync();
              }
            }
            else {
              displayPairResults();
              Serial.println("Pairing is complete.");
              Serial.print("Attempted Pairs: ");
              Serial.println(paired + fpaired);
              Serial.print("Successful Pairs: ");
              Serial.println(paired);
              Serial.print("Failed Pairs: ");
              Serial.println(fpaired);
              //resetting everything (maybe a memory reset)
              peerIndex = 0;
              paired = 0;
              fpaired = 0;
              for (int i = 0; i < peerCnt; i++) {
                syncStatus[i] = true;
              }
              currSyncStatus = SCANNING;
            }
            break;
        }
      }
      if (millis() - scanStart > SCANTIME) {
        deinitWiFi();
        displayPairResults();
        delay(5000);
        displaySTANDBY();
        curr_state = STANDBY;
      }
      break;
    case DATA_LOG :
    	gyro.getEvent(&event);
    	accelmag.getEvent(&aevent, &mevent);
		rawDataString = String(event.timestamp) + ',' + 
						String(event.gyro.x) + ',' + String(event.gyro.y) + ',' + String(event.gyro.z) + "," + 
						String(aevent.acceleration.x) + ',' + String(aevent.acceleration.y) + ',' + String(aevent.acceleration.z) + "," +
						String(mevent.magnetic.x) + ',' + String(mevent.magnetic.y) + ',' + String(mevent.magnetic.z) + "\n";
		strcat(wBuffer,rawDataString.c_str());
    	if(BUFFERSIZE - strlen(wBuffer) < MAXMESSAGELENGTH){
    		appendFile(SD,g_path_c,wBuffer);
    		memset(wBuffer,0,sizeof(wBuffer));
    	}

      if(millis()-screenRefresh > 20000){
        display.clearDisplay();
        display.display();
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
      delay(9);
      break;
  }

}

boolean initIMU() {
  if (!gyro.begin()) {
    /* There was a problem detecting the FXAS21002C ... check your connections
     */
    Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    return false;
  }
  delay(100);
  if (!accelmag.begin(ACCEL_RANGE_4G)) {
  	/* There was a problem detecting the FXOS8700 ... check your connections */
  	Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
  	return false;
  }
  delay(100);
  return true;
}


void createFiles(){
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
  if(!a_file || !g_file || m_file){
    Serial.println("Files are not found");
    Serial.println("Creating Directory and files...");
    createDir(SD,folder.c_str());
    writeFile(SD,a_path_c,"Time(milliseconds),X,Y,Z\r\n");
    writeFile(SD,g_path_c,"Time(milliseconds),GX,GY,GZ,AX,AY,AZ,MX,MY,MZ\n");
    writeFile(SD,m_path_c,"Time(milliseconds),X,Y,Z\r\n");
  }
  else{
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

  esp_now_register_send_cb(onDataSent);   //Registering callback function when data is sent
  esp_now_register_recv_cb(onDataRecv);   //Registering callback funtion when data is recv
}

void deinitWiFi() {
  if (esp_now_deinit() == ESP_OK) {
    WiFi.mode(WIFI_OFF);
    WiFi.disconnect();
    //    String confup = "ESP NOW WiFi driver successfully stopped\r\n";
    //    appendFile(SD, config_path, confup.c_str());
  }
}

void configDeviceAP() {
  String Prefix = "Main:";  //setting the prefix of the device
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

void sensorSync() {
  delay(5);
  const uint8_t *peer_addr = peers[peerIndex].peer_addr;  //saving peer address to local var
  if (sendData.synced) {
    //populate delta clocks
    Serial.println("Sync Complete, sending synced clock info.");
    sendData.mSentClk = 0;
    esp_err_t result = esp_now_send(peer_addr, (uint8_t*) &sendData, sizeof(sendData)); //sending data
    //send deltaClk
    if (result == ESP_OK) {
      Serial.println("Processed sending correctly");
    }
    else {
      Serial.println("Didn't process correctly");
    }
  }
  else {
    Serial.println("Sending Data...");
    sendData.mSentClk = millis();
    esp_err_t result = esp_now_send(peer_addr, (uint8_t*) &sendData, sizeof(sendData)); //sending data
    if (result == ESP_OK) {
      Serial.println("Processed sending correctly");
    }
    else {
      Serial.println("Didn't process correctly");
    }
  }
}

void calcAverageClk() {
  //sign should always be the same
  //this will go tits up if we overflow, so try not to run this after a few days
  dClkResult.clk = 0;
  dClkResult.sign = dClk[peerIndex][1].sign;
  for ( int i = 0; i < 10; i++) {
    //Serial.println(dClk[peerIndex][i].clk);
    dClkResult.clk += dClk[peerIndex][i].clk;
  }
  dClkResult.clk = dClkResult.clk / 10;
  Serial.print("Average Delta Clock: ");
  Serial.println(dClkResult.clk);

  const uint8_t *peer_addr = peers[peerIndex].peer_addr;
  esp_err_t result = esp_now_send(peer_addr, (uint8_t*) &dClkResult, sizeof(dClkResult));
  if (result == ESP_OK) {
    Serial.println("Processed sending correctly");
  }
  else {
    Serial.println("Didn't process correctly");
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

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    if (!syncStatus[peerIndex]) {
      if (packet == 10) {
        Serial.println("Clock Sync Data sent successfully");
        syncStatus[peerIndex] = true;
      }
      else if (packet == 9) {
        Serial.println("Final ping successful");
        sendData.synced = true;
        packet++;
      }
      else {
        Serial.print("Packet: ");
        Serial.print(packet);
        Serial.println(" sucessfully sent");
        packet++;
        Serial.println("Iterating packet");
      }
    }
    else {
      Serial.println("Successfully sent Delta Clock");
    }

  }
  else {
    Serial.println("Failed to send, trying again");
    if (!syncStatus[peerIndex]) {
      if (packet != 0) {
        packet--;
      }
      sensorSync();
    }
    else {
      Serial.println("Failed to send Delta Clock!");
      Serial.println("No backup set");
    }
  }
}

void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) {
  Serial.println("Recieved Data");
  unsigned long int recTime = millis();
  memcpy(&recvData, incomingData, sizeof(recvData));
  if (sendData.mSentClk == recvData.mSentClk) {
    Serial.println("Information Matches");
    Serial.print("peerIndex: ");
    Serial.println(peerIndex);
    Serial.print("packet: ");
    Serial.println(packet);
    latency[peerIndex][packet] = recTime - sendData.mSentClk;
    Serial.print("Latency Calculated: ");
    Serial.println(latency[peerIndex][packet]);
    if (recvData.mSentClk >= recvData.pClk) {
      Serial.print("Index: [");
      Serial.print(peerIndex);
      Serial.print("][");
      Serial.print(packet - 1);
      Serial.println("]");
      dClk[peerIndex][packet - 1].sign = true;
      dClk[peerIndex][packet - 1].clk = recvData.mSentClk - (recvData.pClk - latency[peerIndex][packet] / 2);
      //deltaClk[peerIndex][packet] = recvData.mSentClk - (recvData.pClk - latency[peerIndex][packet]/2);
      Serial.println("Main Clock is larger");
      Serial.print("Delta Clock: ");
      Serial.println(dClk[peerIndex][packet - 1].clk);
    }
    else {
      Serial.print("Index: [");
      Serial.print(peerIndex);
      Serial.print("][");
      Serial.print(packet - 1);
      Serial.println("]");
      dClk[peerIndex][packet - 1].sign = false;
      dClk[peerIndex][packet - 1].clk = (recvData.pClk - latency[peerIndex][packet] / 2) - recvData.mSentClk;
      //deltaClk[peerIndex][packet] = (recvData.pClk - latency[peerIndex][packet]/2) - recvData.mSentClk;
      Serial.println("Peer Clock is larger");
      Serial.print("Delta Clock: ");
      Serial.println(dClk[peerIndex][packet - 1].clk);
    }

    //find which clock is bigger
    //find clock differences
    //
  }
  else {
    Serial.println("Data did not match. Trying again...");
    packet--;
    //something went wrong
  }
  sensorSync();
}

void managePeer() {
  if (peerCnt > 0) {
    for (int i = 0; i < peerCnt; i++) {
      Serial.print("Processing: ");
      for (int ii = 0; ii < 6; ++ii ) {
        Serial.print((uint8_t) peers[i].peer_addr[ii], HEX);
        if (ii != 5) Serial.print(":");
      }
      Serial.print(" Status: ");
      // check if the peer exists
      bool exists = esp_now_is_peer_exist(peers[i].peer_addr);
      if (exists) {
        // Peer already paired.
        Serial.println("Already Paired");
      } else {
        // Peer not paired, attempt pair
        esp_err_t addStatus = esp_now_add_peer(&peers[i]);
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

void scanForPeers() {
  int8_t scanResults = WiFi.scanNetworks();
  //reset peers
  memset(peers, 0, sizeof(peers));
  peerCnt = 0;
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
      // Check if the current device starts with `Peer`
      if (SSID.indexOf("Peer") == 0) {
        // SSID of interest
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];

        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            peers[peerCnt].peer_addr[ii] = (uint8_t) mac[ii];
          }
        }
        peers[peerCnt].channel = CHANNEL; // pick a channel
        peers[peerCnt].encrypt = 0; // no encryption
        syncStatus[peerCnt] = false;
        peerCnt++;
      }
    }
  }
}


//Not going to use these because if held it will cause an error, but I need them
void callback() {
  //will be executed if awake
}


void displayBattery(){
  calcVolt = (analogRead(batVoltagePin)*2*(3.5))/4095;
  String a ="Battery:" + String(calcVolt) + "V";
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(50,0);
  display.println(F(a.c_str()));
  display.display();
  screenRefresh = millis();
}

void displayNoSDCard(){
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,10);
  display.println(F("No SD Card"));
  display.display();
  displayBattery();
}

void displayNoIMU(){
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(25,10);
  display.println(F("No IMU"));
  display.display();
  displayBattery();
}

void displaySTANDBY(){
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(25,10);
  display.println(F("Standby"));
  display.display();
  displayBattery();
}

void displayNoPower(){
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(15,10);
  display.println(F("Power Off"));
  display.display();
  displayBattery();
}

void displayPairing(){
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(25,10);
  display.println(F("Pairing"));
  display.display();
  displayBattery();
}

void displayPaired(){
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(25,10);
  display.println(F("Paired"));
  display.display();
  displayBattery();
}

void displayDatalog(){
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(25,10);
  display.println(F("Datalog"));
  display.display();
  displayBattery();
}

void displayPairResults(){
  String a = "Pair Attempts: " + String(paired + fpaired);
  String b = "Successful Pairs: " + String(paired);
  String c = "Unsuccessful Pairs: " + String(fpaired);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(F(a.c_str()));
  display.println(F(b.c_str()));
  display.println(F(c.c_str()));
  display.display();
}

void displayFakeInit(){
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20,0);
  display.println(F("PSU 401"));
  display.display();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,25);
  display.println(F("____"));
  display.display();
  delay(800);
  display.setCursor(0,25);
  display.println(F("__________"));
  display.display();
  delay(500);
  display.setCursor(0,25);
  display.println(F("_________________"));
  display.display();
  delay(1000);
  display.setCursor(0,25);
  display.println(F("______________________"));
  display.display();
  delay(100);
}
