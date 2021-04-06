#include <Adafruit_FXOS8700.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Wire.h>

Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); //setting to 400kHz but will be limited by pull down resistor
  if(!accelmag.begin(ACCEL_RANGE_8G) && !gyro.begin(GYRO_RANGE_2000DPS)){ //change accel range just change 2,4, or 8 to whatever || same with gyro 250,500,1000,2000
    Serial.println("There was a problem with one of the chips on the sensor!");
    while(1){
      delay(100);
    }
  }
}

void loop() {
  sensors_event_t gevent, aevent, mevent; //adafruits struct for sensors
  if(accelmag.getEvent(&aevent, &mevent) && gyro.getEvent(&gevent)){ //event happens aka new sensor data (returns true when successfully read)
//     Serial.println("Accelometer (m/s^2):");
//     Serial.println(aevent.acceleration.x); //how to access sensor data
//     Serial.println(aevent.acceleration.y);
//     Serial.println(aevent.acceleration.z);
//     Serial.println("Mangetometer (uTesla):");
//     Serial.println(mevent.magnetic.x);
//     Serial.println(mevent.magnetic.y);
//     Serial.println(mevent.magnetic.z);
     Serial.println("Gyroscope (rad/s):");
     Serial.println(gevent.gyro.x);
     Serial.println(gevent.gyro.y);
     Serial.println(gevent.gyro.z); //seem to work at around 80~ Hz (12ms delay between reads)
  }
}
