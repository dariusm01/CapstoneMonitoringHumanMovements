//
#include <EEPROM.h>

#define EEPROM_SIZE 64

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
enum ESP_state : uint8_t{
    START,STANDBY,FILE_CREATE,PAIR,DATA_LOG,SLEEP
  };

//Defining the current state of the sensor to the startup state
ESP_state curr_state = START;

//The pin that controls the board LED
const int BoardLED = 13;

//Setting the touch constants
const int powerPin = 12;
const int wirelessPin = 27;

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



void setup() {
  //Start the serial monitor
  Serial.begin(115200);
  delay(1000);
  Serial.println("Initializing Board...");

  //Initializing EEORM
  EEPROM.begin(EEPROM_SIZE);

  const uint16_t session = session_count();
  Serial.print("Device Session Number: ");
  Serial.println(session);
  
  //Setting the PWM channel settings and attaching BoardLED to the channel
  ledcSetup(ledChannel,freq,resolution);
  ledcAttachPin(BoardLED,ledChannel);

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
  switch(curr_state){
    //Standby state is waiting for sensor to 
    case STANDBY :
      //Was the Sensor in deep sleep? (Powered off)
      if(wasAsleep){
                                                    //this will never run
                                                    //Save something to the EEPROM
        print_wakeup_reason();
        print_wakeup_touchpad();
        wasAsleep = false;
      }
      //Checking for Button Presses
      if(touchRead(powerPin) < pinTouch){
        //The power pin has detected a touch and will now determine what type of press
        pressType = buttonPress(millis(),powerPin);
        switch(pressType){
          case 1:
            Serial.println("Battery Level Requested");
            break;
          case 2:
            //Putting ESP32 to sleep to save power while 'off'
            curr_state = SLEEP;
            break;
        }
      }
      else if(touchRead(wirelessPin) < pinTouch){
        //The wireless pin has detected a touch and will now determine what type of press
        pressType = buttonPress(millis(), wirelessPin);
        switch(pressType){
          case 1:
            Serial.println("Scanning for sensors to pair to");
            curr_state = PAIR;
            break;
          case 2:
            //Putting ESP32 to sleep to save power while 'off'
            Serial.println("Starting to Datalog");
            //Start IMU
            //Generate IMU Files
            curr_state = DATA_LOG;
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
      //Code
      break;
    case DATA_LOG :
      //Code
      break;   
  }
  
}



/*
 * Description: Will determine the button press made
 * 
 * Input: start - the clock value at the inital button press
 *        pin - The number that was pressed
 *        
 * Output: 
 *        0 - Rejected press
 *        1 - Short press (101-2999 ms)
 *        2 - Long Press (3000ms)
 */
uint8_t buttonPress(unsigned long start, const int pin){
  while(touchRead(pin) < pinTouch){
    if(millis() - start > 3000){
      Serial.print("Long Press on pin ");
      Serial.print(pin);
      Serial.print(" detected");
      Serial.println("");
      //want feedback for the user to remove their hand from button
      for(int dutyCycle = 255; dutyCycle >= 0; dutyCycle--){
        ledcWrite(ledChannel,dutyCycle);
        delay(5);
      }
      return 2;
    }
  }
  if(millis() - start > 100){
    Serial.print("Short Press on pin ");
    Serial.print(pin);
    Serial.print(" detected");
    Serial.println("");
    ledcWrite(ledChannel,255);
    delay(100);
    ledcWrite(ledChannel,0);
    return 1;
  }
  Serial.println("Rejected Press");
  return 0;
}

/*
 * Description: Will determine the session number of the device. If the session is not set, it will set it.
 *              Uses the first three bytes of EEPROM: 0,1,2
 * 
 * Input: 
 *        EEPROM 0: If set - 1
 *        EEPROM 1: 
 *        EEPROM 2:
 * 
 * Output: 
 *        unsigned 16 bit integer of the session number
 */
uint16_t session_count(){
  uint16_t s = 1;                                                   //local var for session number
  
  if(byte(EEPROM.read(EP_SET)) == 1){                               //is the session saved in EEPROM set?
     s = byte(EEPROM.read(EP_S1));
     s = s<<8;
     s = s + byte(EEPROM.read(EP_S2));                              //Setting session number: s = S1 S2
     if(byte(EEPROM.read(EP_S2)) == 255){                           //Checking if S2 will overflow
      EEPROM.write(EP_S1,byte(EEPROM.read(EP_S1))+1);               //Write: Add 1 to S1 because of overflow
      EEPROM.write(EP_S2,0);                                        //Write: S2 to 0
     }
     else{                                                          //No Overflow in S2
      EEPROM.write(EP_S2,byte(EEPROM.read(EP_S2))+1);               //Write: Add 1 to S2
     }
  }
  else{                                                             //Not set
    EEPROM.write(EP_S1,0);                                          //Write S1 to 0
    EEPROM.write(EP_S2,1);                                          //Write S2 to 1
    EEPROM.write(EP_SET,1);                                         //Setting set
  }
  EEPROM.commit();                                                  //saving session info
  return s;                                                         //returning session
}


/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason){
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

/*
Method to print the touchpad by which ESP32
has been awaken from sleep
*/
void print_wakeup_touchpad(){
  touchPin = esp_sleep_get_touchpad_wakeup_status();
  switch(touchPin){
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

//Not going to use these because if held it will cause an error, but I need them
void callback(){
  //will be executed if awake
}