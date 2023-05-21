//https://gitlab.com/yesbotics/libs/arduino/average-value
#include <AverageValue.h>

// ----------------
// ----- PINS -----

// LED Pins
const int redPin = 12;
const int greenPin = 11;
const int bluePin = 10;

const int motorPin = 4;

const int irInput = A0;

// Seconds
const float motorCycleTime = 0.75;

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
AverageValue<float> dist(50);


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
#else
const float stretchInterval = 30 * 60;
const float stretchDuration = 30;
#endif

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

  lastStretchTime = getTimeInSeconds(millis());
}

void loop() {
    
  // Get distance of the IR sensor
  float distance = 29.988 * pow(analogRead(irInput) * (5.0 / 1023.0), -1.173);
  dist.push(distance);
  
  #if PRINT_OUTPUT
    Serial.print("Measurement ");
    Serial.print(String(dist.average()) + " | ");
  #endif

  // ----------------
  // ---- PAUSED ----
  if (currentState == PAUSED) {
    
    setLED(255,0,255);

    if (isSitting()) {
      currentState = SITTING;

      // If needed, modify lastStretchTime to ignore time paused 
      if (timePaused != 0) {
        lastStretchTime = getTimeInSeconds(millis()) - (timePaused - lastStretchTime);
        timePaused = 0;
      }
      
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
    if (getTimeInSeconds(millis()) - lastStretchTime > stretchInterval) {
      
      currentState = REMINDING;
      
    } else if (!isSitting()) {
      
      currentState = PAUSED;
      timePaused = getTimeInSeconds(millis());
      
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
    
    if (!isSitting()) {

      currentState = STRETCHING;

      // Motor off
      motorActionTime = -1;
      digitalWrite(motorPin, LOW);

      // Record stretch start time
      stretchStart = getTimeInSeconds(millis());

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
      currentState = REMINDING;
      
    } else if( getTimeInSeconds(millis()) - stretchStart > stretchDuration) {
  
      // Stretch Successful, reset
      setLED(0, 0, 0);
  
      // Reset the timer
      lastStretchTime = getTimeInSeconds(millis());

      currentState = SITTING;
      
      delay(1000);
    }
    
  }
  
    
}

// ----------------------
// --- UTIL FUNCTIONS ---

bool isSitting() {
  return dist.average() < 12;
}

float getTimeInSeconds(float milliseconds) {
  return milliseconds / 1000;
}

void setLED(int red_light_value, int green_light_value, int blue_light_value) {
  analogWrite(redPin, red_light_value);
  analogWrite(greenPin, green_light_value);
  analogWrite(bluePin, blue_light_value);
}

void interpolateLED(int from[], int to[], float proportion, float brightness) {
  if ( proportion < 1.0 ) {
    setLED(  (int) ( (( from[0] * (1 - proportion) ) + (int) (to[0] * proportion)) * brightness  ) ,
             (int) ( (( from[1] * (1 - proportion) ) + (int) (to[1] * proportion)) * brightness  ) ,
             (int) ( (( from[2] * (1 - proportion) ) + (int) (to[2] * proportion)) * brightness  ) );
  }
}
