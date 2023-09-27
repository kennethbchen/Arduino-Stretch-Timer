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
AverageValue<double> dist(100);

// ----------------
// ----- UTIL -----

const int red[] {255, 0, 0};
const int green[] {0, 255, 0};
const int blue[] {0, 0, 255};

// ----------------
// ----------------

void setup() {
  
  Serial.begin(9600);

  // Setup pins
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  pinMode(motorPin, OUTPUT);
  pinMode(snoozeButtonPin, INPUT_PULLUP);

  lastStretchTime = getTimeInSeconds(millis());
}

void setMotorOff() {
  // Motor off
  motorActionTime = -1;
  digitalWrite(motorPin, LOW);
}

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

void loop() {
  
  // Get distance of the IR sensor
  dist.push(analogRead(irInput));
  
  #if PRINT_OUTPUT
    Serial.print("Measurement ");
    Serial.print(String(dist.average()) + " | ");
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

// ----------------------
// --- UTIL FUNCTIONS ---

bool isSitting() {
  // dist.average should be within 0-1023 or something (based on output of analog read)
  return dist.average() > 500 ;
}

float getTimeInSeconds(float milliseconds) {
  return milliseconds / 1000;
}

void setLED(int red_light_value, int green_light_value, int blue_light_value) {
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
