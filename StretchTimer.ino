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

// Digital
const int motorPin = 2;

// Digital
const int snoozeButtonPin = 5;

// Because of Pin type INPUT_PULLUP, when the button is pressed, its read value is false
const bool snoozeButtonPressedValue = false;

// ----------------
// --- SETTINGS ---

// Don't leave on for actual use, it slows down arduino or something
#define PRINT_OUTPUT false

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
// ----------------

void setup() {

  #if PRINT_OUTPUT
    Serial.begin(115200);
    Serial.println("Starting...");
  #endif


  pinMode(motorPin, OUTPUT);
  pinMode(snoozeButtonPin, INPUT_PULLUP);

  setMotorOff();
  
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
  uint8_t status = distanceSensor.VL53L4CD_CheckForDataReady(&NewDataReady);
 

  if ((!status) && (NewDataReady != 0)) {
    // Process Status

    // (Mandatory) Clear HW interrupt to restart measurements
    distanceSensor.VL53L4CD_ClearInterrupt();

    // Read measured distance. RangeStatus = 0 means valid data
    distanceSensor.VL53L4CD_GetResult(&results);
    
    // Get distance of the sensor
    dist_inches.push(results.distance_mm / 25.4);

    
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
    
    if (isSitting()) {
      changeToSitting();
    }
    
  }

  // ----------------
  // ---- SITTING ---
  if (currentState == SITTING) {

    // Show time until next stretch
    
    matrix.print( int( ( currentStretchInterval - (getTimeInSeconds(millis()) - lastStretchTime )) / 60 ) * 100);
    matrix.drawColon(true);
    matrix.writeDisplay();
    

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

    // Time to stretch, motor on
  
    // Motor has been on or off for one second, toggle motor
    if (getTimeInSeconds(millis()) - motorActionTime >= motorCycleTime) {
  
       motorActionTime = getTimeInSeconds(millis());
       
       // Toggle Motor and Matrix
       // TODO rename stuff because it's more than the motor
       if (motorOn) {
          matrix.print("") ;
          matrix.writeDisplay();

          motorOn = false;
          digitalWrite(motorPin, LOW);
       } else {
          
          matrix.print(0.00);
          matrix.writeDisplay();

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
  
    // Check if time stretching is at least the stretch duration

    matrix.print(stretchDuration - (getTimeInSeconds(millis()) - stretchStart));
    matrix.writeDisplay();

    if (isSitting()) {

      // Stretch hasn't been completed yet
      changeToReminding();
      
    } else if( getTimeInSeconds(millis()) - stretchStart > stretchDuration) {
  
      // Stretch Successful, reset

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

  matrix.print("");
  matrix.writeDisplay();
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
  return dist_inches.average() < 15 ;
}

float getTimeInSeconds(float milliseconds) {
  return milliseconds / 1000;
}
