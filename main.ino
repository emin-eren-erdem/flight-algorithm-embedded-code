//libraries
#include "Wire.h" //Master to slave i2c communication library.
#include <SoftwareSerial.h> //Digitally created RX/TX pins for multiple serial communications.
#include <TinyGPS.h> //To decode GPS NMEA easily and extract latitude and longitude only.
#include <Adafruit_BME280.h> //Sensor library for smart control.
#include <Adafruit_LSM9DS1.h> //Sensor library for smart control.

//sensor and module objects
Adafruit_BME280 bme;
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
// I did not use SPI. All communication is I2C.
TinyGPS gps;
SoftwareSerial ss(4, 3); //GPS serial communication pins

//global variables
int packet_id;
int buzzer_pin = 6;
int led_pin = 7;

//these are the first decleration of the variables that play a key role in flight algorithm
boolean first_seperation;
boolean second_seperation;

void alertBuzzer() {
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(buzzer_pin, HIGH);
    digitalWrite(led_pin, HIGH);
    delay(50);
    digitalWrite(buzzer_pin, LOW);
    digitalWrite(led_pin, LOW);
    delay(50);
  }
}

void setup() {
  packet_id = 0;
  Serial.begin(115200); //main serial port for output. also used for transmitting to ground.
  ss.begin(9600); //max baudrate for gps. slightly slower than other comms.
  bme.begin(); //initialize bme280
  lsm.begin(); //initialize lsm9ds1

  pinMode(buzzer_pin, OUTPUT);
  pinMode(led_pin, OUTPUT);

  first_seperation == false;
  second_seperation == false;

  alertBuzzer();
}

void readSensor() {

  float flat, flon;
  unsigned long age;
  gps.f_get_position(&flat, &flon, &age);
  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);

  //the difference between these two variables will determine the rotation and state
  int altitude_1 = 0;
  int altitude_2 = 0;

  float sensor_temp  = bme.readTemperature();
  float sensor_alt   = bme.readAltitude(1013.25);
  float sensor_press = bme.readPressure() / 100.0F;

  float sensor_gyro_X = g.gyro.x;
  float sensor_gyro_Y = g.gyro.y;
  float sensor_gyro_Z = g.gyro.z;

  float sensor_acc_X = g.acceleration.x;
  float sensor_acc_Y = g.acceleration.y;
  float sensor_acc_Z = g.acceleration.z;

  
    Serial.print("1/1;" + (String)packet_id + ";" + (String)sensor_temp + ";" + (String)sensor_alt + ";" + (String)sensor_press + ";" + (String)sensor_gyro_X + ";" + (String)sensor_gyro_Y + ";" +
               (String)sensor_gyro_Z + ";" + (String)sensor_acc_X + ";" + (String)sensor_acc_Y + ";" + (String)sensor_acc_Z + ";");

    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);    //gpsX
    Serial.print(";");
    Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);    //gpsY
  

  //main flight algorithm

  if (second_seperation == false) {

    //for a stable value, get 5 continuous values
    //then take the averages for both time/altitude variables

    for (int i = 0; i < 4; i++) {
      altitude_1 += bme.readAltitude(1013.25);
    }
    altitude_1 = altitude_1 / 5;
    delay(100);

    for (int i = 0; i < 4; i++) {
      altitude_2 += bme.readAltitude(1013.25);
    }
    altitude_2 = altitude_2 / 5;
    //Serial.println("altitude_1 is = " + (String)altitude_1 + " and altitude_2 is = " + (String)altitude_2);

    if (altitude_1 < altitude_2) {
      //that means rocket is ascending
      //Serial.println("yes. " + (String)altitude_1 + " is smaller than " + (String)altitude_2);
      //Serial.println("Still going up.");
    }
    else if (altitude_1 = altitude_2) {
      //Serial.println("rocket is probably standing still.");
    }
    else
    {
      if (first_seperation == true) {
        //the first seperation is already happened
        //Serial.println("first seperation true");
        if (sensor_alt <= 50) {
          alertBuzzer();
          //set second seperation to true
          //Serial.println("rocket is below 500 meters");
          second_seperation = true;
        }
        else {
          //do nothing
          //Serial.println("rocket is above 500 meters");
        }
      }
      else {
        //the rocket stopped ascending. maxed out the altitude
        //make the first seperation happen.
        alertBuzzer();
        first_seperation = true;
        //Serial.println("first seperation happening.");
      }
    }
  }
  else {
    //that means there is nothing left to do. keep transmitting data.
    //Serial.println("second seperation happened. goodbye.");
  }
}

void loop() {

  readSensor();

  packet_id++;
  digitalWrite(led_pin, HIGH);
  delay(100);
  digitalWrite(led_pin, LOW);
  delay(400);
}
