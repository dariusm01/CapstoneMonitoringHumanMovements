
/*
Defining the States that the Sensor can be in
START: is the state when the board has just started and is setting up pins, structs and objects
STANDBY: This is the state where the board is waiting for a input from the user
FILE_CREATE: The Board will Create the directories needed for the current session
PAIR: The board is actively searching for other sensors to sync with
DATA_LOG: The Board is going to start logging data from the sensors
SLEEP: The board is in sleep mode
*/
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

touch_pad_t touchPin;

//Boolean that checks if the ESP was alseep
boolean wasAsleep = false;

//Pin wire touch value: this is subject to change depending on the material setup with the touch sensor
const int pinTouch = 48;


void setup() {
  //Start the serial monitor
  Serial.begin(115200);
  Serial.println("Initializing Board...");
  
  //Setting pin 13 as an output to control the onboard LED
  pinMode(BoardLED,OUTPUT);

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
    case STANDBY :
      if(wasAsleep){
        print_wakeup_reason();
        print_wakeup_touchpad();
        wasAsleep = false;
      }
      //detecting Button Press
      if (touchRead(powerPin)< pinTouch){
        
        Serial.println("Power Button Pressed");
        if (PowerPress(millis())){
          curr_state = SLEEP;
        }
      }
      delay(50); //this is so the loop doesnt keep running at full speed during standby
      break;
    case SLEEP : 
      curr_state = STANDBY;
      wasAsleep = true;
      delay(2000);
      //Nothing after this line will be called in this case statement
      esp_sleep_enable_touchpad_wakeup();
      Serial.println("Entering Sleep Mode");
      esp_deep_sleep_start(); 
      break;   
  }
  
}
/*
Will get the amount of time the Power Pin is Pressed and break at 3 seconds?
  0 is a press less than 100ms and will be rejected
  1 is a long press and will make the ESP enter LOW Power or SLEEP
*/
boolean PowerPress(unsigned long start){
  while(touchRead(powerPin) < pinTouch){
    if(millis()-start > 3000){
      Serial.println("Long press detected");
      return true;
    }
  }
  Serial.println("Not a long enough Press");
  return false;
}

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
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

  switch(touchPin)
  {
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


//placeholder for now
void callback(){
  //will be executed if awake
  //Figuring out if I want to change this
}


