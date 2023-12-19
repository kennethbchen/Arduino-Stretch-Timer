#include <Arduino.h>
#include <Wire.h>

// 7 Segment LED Display
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

// Distance Sensor
#include <vl53l4cd_class.h>

//https://gitlab.com/yesbotics/libs/arduino/average-value
#include <AverageValue.h>

// ----------------
// ----- PINS -----

// LED Pins
const int redPin = 12;
const int greenPin = 11;
const int bluePin = 10;

const int motorPin = 4;

const int snoozeButtonPin = 5;

const int irInput = A0;

// Because of Pin type INPUT_PULLUP, when the button is pressed,
// Its read value is false
const bool snoozeButtonPressedValue = false;

// ----------------
// --- SETTINGS ---

// Don't leave on for actual use, it slows down arduino or something
#define PRINT_OUTPUT true

// Debug mode shortens stretch times and durations
#define DEBUG_MODE false

// Interval between stretches and duration of stretches in seconds
#if DEBUG_MODE
const float stretchInterval = 10;
const float stretchDuration = 5;
const float snoozeStretchInterval = 16;
#else
const float stretchInterval = 30 * 60;
const float stretchDuration = 30;
const float snoozeStretchInterval = 5 * 60;
#endif

// Seconds
const float motorCycleTime = 1.12;

// Value from 0 to 1
// Brightness multiplier for LED
const float globalBrightnessMultiplier = 0.2;



// ----------------
// ---- STATE -----

enum State {
  PAUSED,
  SITTING,
  REMINDING,
  STRETCHING
};

State currentState = SITTING;

bool motorOn = false;

// Seconds
float currentStretchInterval = stretchInterval;

// Seconds
float lastStretchTime = 0;

// Seconds
float stretchStart = 0;

// Seconds
float timePaused = 0;

// Seconds
float motorActionTime = -1;

// Running average of IR measurements
// Used to reduce noise in IR input
// Slows down significantly if printing to serial
AverageValue<double> dist_inches(1);

#define DEV_I2C Wire

VL53L4CD distanceSensor = VL53L4CD(&DEV_I2C, A1);

Adafruit_7segment matrix = Adafruit_7segment();

// ----------------
// ----- UTIL -----

const int red[] {255, 0, 0};
const int green[] {0, 255, 0};
const int blue[] {0, 0, 255};

// ----------------
// ----------------

void setup() {

  #if PRINT_OUTPUT
    Serial.begin(115200);
    Serial.println("Starting...");
  #endif
  
  
  // Setup pins
  /*
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  pinMode(motorPin, OUTPUT);
  pinMode(snoozeButtonPin, INPUT_PULLUP);
  */
  

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Configure VL53L4CD satellite component.
  distanceSensor.begin();

  // Switch off VL53L4CD satellite component.
  distanceSensor.VL53L4CD_Off();

  //Initialize VL53L4CD satellite component.
  distanceSensor.InitSensor();

  // Program the highest possible TimingBudget, without enabling the
  // low power mode. This should give the best accuracy
  distanceSensor.VL53L4CD_SetRangeTiming(200, 0);

  // Start Measurements
  distanceSensor.VL53L4CD_StartRanging();

  // Init Screen
  matrix.begin(0x70);

  lastStretchTime = getTimeInSeconds(millis());
}


void loop() {
  
  
  uint8_t NewDataReady = 0;
  VL53L4CD_Result_t results;
  uint8_t status;

  // Wait for distance sensor
  do {
    status = distanceSensor.VL53L4CD_CheckForDataReady(&NewDataReady);
  } while (!NewDataReady);

  

  if ((!status) && (NewDataReady != 0)) {
    // (Mandatory) Clear HW interrupt to restart measurements
    distanceSensor.VL53L4CD_ClearInterrupt();

    // Read measured distance. RangeStatus = 0 means valid data
    distanceSensor.VL53L4CD_GetResult(&results);
    
    // Get distance of the sensor
    dist_inches.push(results.distance_mm / 25.4);

    matrix.print(dist_inches.average());
    matrix.writeDisplay();
  }
  
  #if PRINT_OUTPUT
    //Serial.print("Measurement ");
    Serial.print(String(dist_inches.average()) + " | ");

    Serial.print("State " + String(currentState) + " | ");
    Serial.print("Range Status " + String(results.range_status) + " | ");
    Serial.println();



  #endif
  
  // ----------------
  // ---- PAUSED ----
  if (currentState == PAUSED) {
    
    setLED(255,0,255);

    if (isSitting()) {
      changeToSitting();
    }
    
  }

  // ----------------
  // ---- SITTING ---
  if (currentState == SITTING) {
    // Interpolate led color based on time till next stretch break
    interpolateLED(green, 
                   red, 
                   min((float) (getTimeInSeconds(millis()) - lastStretchTime) / stretchInterval, 1), 0.5);
    

    // If it is time for a stretch break
    if (getTimeInSeconds(millis()) - lastStretchTime > currentStretchInterval) {
      
      changeToReminding();
      
    } else if (!isSitting()) {
      
      changeToPaused();
      
    }
    
  }

  // ----------------
  // --- REMINDING --
  if (currentState == REMINDING) {

    setLED(255, 0, 0);

    // Time to stretch, motor on
  
    // Motor has been on or off for one second, toggle motor
    if (getTimeInSeconds(millis()) - motorActionTime >= motorCycleTime) {
  
       motorActionTime = getTimeInSeconds(millis());
       
       // Toggle Motor
       if (motorOn) {
          motorOn = false;
          digitalWrite(motorPin, LOW);
       } else {
          motorOn = true;
          digitalWrite(motorPin, HIGH);
       }
       
    } else if (motorActionTime == -1) {
      // Motor action time needs to be reset
      motorActionTime = getTimeInSeconds(millis());
    }

    // Check if snooze button is pressed
    if (digitalRead(snoozeButtonPin) == snoozeButtonPressedValue) {

      if (currentStretchInterval != stretchInterval) {
        // Snooze has been pressed before this cycle
        // Snooze has diminishing returns
        currentStretchInterval *= 0.3;
      } else {
        // Snooze has not been pressed before this cycle
        currentStretchInterval = snoozeStretchInterval;
      }
      
      // Reset the timer
      lastStretchTime = getTimeInSeconds(millis());
      
      changeToSitting();
      
    } else if (!isSitting()) {

      changeToStretching();

    }
    
  }

  // ----------------
  // -- STRETCHING --
  if (currentState == STRETCHING){
    
    interpolateLED(blue, 
                       green, 
                       min((float) (getTimeInSeconds(millis()) - stretchStart) / stretchDuration, 1),
                       0.5);
  
    // Check if time stretching is at least the stretch duration

    if (isSitting()) {

      // Stretch hasn't been completed yet
      changeToReminding();
      
    } else if( getTimeInSeconds(millis()) - stretchStart > stretchDuration) {
  
      // Stretch Successful, reset
      setLED(0, 0, 0);

      currentStretchInterval = stretchInterval;
  
      // Reset the timer
      lastStretchTime = getTimeInSeconds(millis());

      changeToSitting();
      
      delay(1000);
    }
    
  }
  
    
}

#pragma region STATE_METHODS

void changeToSitting() {
  currentState = SITTING;

  setMotorOff();
  
  // If needed, modify lastStretchTime to ignore time paused 
  if (timePaused != 0) {
    lastStretchTime = getTimeInSeconds(millis()) - (timePaused - lastStretchTime);
    timePaused = 0;
  }
}

void changeToPaused() {
  currentState = PAUSED;
  timePaused = getTimeInSeconds(millis());
}

void changeToReminding() {
  currentState = REMINDING;
}

void changeToStretching() {
  currentState = STRETCHING;

  setMotorOff();

  // Record stretch start time
  stretchStart = getTimeInSeconds(millis());
}

#pragma endregion STATE_METHODS

// ----------------------
// --- UTIL FUNCTIONS ---

void setMotorOff() {
  // Motor off
  motorActionTime = -1;
  digitalWrite(motorPin, LOW);
}

bool isSitting() {
  return dist_inches.average() < 10 ;
}

float getTimeInSeconds(float milliseconds) {
  return milliseconds / 1000;
}

void setLED(int red_light_value, int green_light_value, int blue_light_value) {

  return;
  analogWrite(redPin, red_light_value * globalBrightnessMultiplier);
  analogWrite(greenPin, green_light_value * globalBrightnessMultiplier);
  analogWrite(bluePin, blue_light_value * globalBrightnessMultiplier);
}

void interpolateLED(int from[], int to[], float proportion, float brightness) {
  if ( proportion < 1.0 ) {
    setLED(  (int) ( ( ( from[0] * (1 - proportion) ) + (to[0] * proportion) ) * brightness ) ,
             (int) ( ( ( from[1] * (1 - proportion) ) + (to[1] * proportion) ) * brightness ) ,
             (int) ( ( ( from[2] * (1 - proportion) ) + (to[2] * proportion) ) * brightness ) );
  }
}
