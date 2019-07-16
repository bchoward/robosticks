#include <Adafruit_VL6180X.h>
#include "SoftwareServo.h" // works with Arduino IDE 1.8.9 and tinycore 1.2.4 by Spence Konde
  // see also https://github.com/electricsheeplabs/attinyServo/blob/master/servoControlWorking.ino
  // couldn't get this one to work: http://www.cunningturtle.com/attiny4585-servo-library/
  



/*
 * 9g servo  activates when range sensor VL6180x senses nearby object, releases when object is out of range
 * 
 * Threshold range set on startup
 * 
 * Use ISP programmer board to program attiny85
 * - burn bootloader if not previous done
 * - 
 * 
 * 
 * 
*/



/*                            ATTINY85
                              _______
         PB5 ADC0   RST <-  1 | o     | 8  -> VCC 5v                    
LED1  <= PB3 ADC3           2 |       | 7  ->              I2C / SCL   PB2 ADC1  ==> vl6180
SRV   <= PB4 ADC2           3 |       | 6  -> SPI / MISO               PB1       ==> LED2
                    GND <-  4 |       | 5  -> SPI / MOSI   I2C / SDA   PB0       ==> vl6180
                              -------
*/



// turn on and off serial output for debugging
#define DEBUG 0


Adafruit_VL6180X vl = Adafruit_VL6180X();
SoftwareServo grabber;    


const int SERVO = PB4;
const int R_LED = PB1; // indicates calibration (blinking along with green) or error state (solid)
const int G_LED = PB3; // turns green on boot, then activates with servo
const int SERVO_MAX = 180;
const int SERVO_MIN = 1;
const int LOOP_DELAY = 500; // ms
const int CALIB_DELAY = 50; // ms
const int REACT_DELAY = 200; // ms
const int PRE_CALIB_DELAY = 3000; // ms

int threshold = 0;
bool r_led_state = 0;
bool g_led_state = 0;



void setup() {
  pinMode(G_LED, OUTPUT);
  pinMode(R_LED, OUTPUT);
  vl.begin();
  grabber.attach(SERVO);
  grabber.write(SERVO_MIN); // set to initial position
  #ifdef DEBUG 
    Serial.begin(115200);
    while (!Serial) {
      delay(1);
  #endif
  }

  for (int i = 0; i < 30; i++) {
    digitalWrite(R_LED, r_led_state);
    digitalWrite(G_LED, r_led_state);
    r_led_state = !r_led_state;
    g_led_state = !g_led_state;
    delay(100);
  }

  // calibrate the sensor to determine initial distance
  //threshold = calibrateSensor();
  threshold = calibrateSensor(1, 3); // with blink on every third round

  #ifdef DEBUG
    Serial.print("setup complete:  threshold = ");
    Serial.println(threshold);
  #endif
  digitalWrite(R_LED, LOW);
  digitalWrite(G_LED, LOW);

}


// read sensor and return distance in mm or null for error

uint8_t readSensor() {
  uint8_t range = vl.readRange(); // in mm
  uint8_t status = vl.readRangeStatus();

  if (status == VL6180X_ERROR_NONE) {
    return range;
  } else {
    #ifdef DEBUG
      Serial.print("sensor read error = ");
      Serial.println(status);
    #endif
    return NULL;
  }
}


// calibrate sensor to find threshold at boot

int calibrateSensor() {
  calibrateSensor(0,0);
}

int calibrateSensor(bool bBlink, int everyntimes) {
  int numTests = 50;
  int sum = 0;
  int count = 0;
  for (int i=0; i<numTests; i++) {
    if (bBlink) {
      if (count % everyntimes == 0) {
        digitalWrite(R_LED, r_led_state);
        r_led_state = !r_led_state;
      }
      if (count == everyntimes) {
        count = 0;
      }
    }
    count++;
    int r = readSensor();
    if (r != NULL) {
      sum += (int) r;
      delay(CALIB_DELAY);
    }
  }
  return sum / numTests;
}


// delay while still updating servo
// EXPERIMENTAL

void delay_refresh(int delaytime){
  int step_time = 25;
  for(int x=0; x<delaytime; x+=step_time){
    delay(step_time);
    SoftwareServo::refresh();
   }
}



void loop() {
  bool grab_state = false;  // avoid spamming the servo?
  int r = readSensor();
  if (r != NULL) {
    // if item detected in reach, close servo
    if (r < threshold && !grab_state) {
      digitalWrite(G_LED, HIGH);
      digitalWrite(R_LED, LOW);
      #ifdef DEBUG
        Serial.println("grab activated");
      #endif 
      delay(REACT_DELAY);
      grabber.write(SERVO_MIN);
      SoftwareServo::refresh(); 
      grab_state = true;
    } else {
      if (grab_state) {
        digitalWrite(G_LED, LOW);
        digitalWrite(R_LED, LOW);
        #ifdef DEBUG
          Serial.println("release activated");
        #endif
        delay(REACT_DELAY);
        grabber.write(SERVO_MAX);
        SoftwareServo::refresh();
        grab_state = false;
      }
    }
    delay(LOOP_DELAY);
  } else { // if distance read is null, light red led for error
      digitalWrite(R_LED, HIGH); 
  }
}
