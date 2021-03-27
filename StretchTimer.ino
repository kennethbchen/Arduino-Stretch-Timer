// ----- PINS -----
const int redPin = 12;
const int greenPin = 11;
const int bluePin = 10;

const int powerButtonPin = 5;

const int motorPin = 4;

const int irInput = A0;

// ----------------
// ---- STATE -----

bool powerButtonPressed = false;
bool enabled = false;
bool sitting = false;
bool shouldStretch = false;
bool stretching = false;
bool paused = false;

// Seconds
float lastStretchTime = 0;

// Seconds
float stretchStart = 0;

// Seconds
float timePaused = 0;

// ----------------
// --- SETTINGS ---

// Seconds, Time between stretch breaks
const float stretchInterval = 30 * 60;

// Seconds, Duration of stretch break
const float stretchDuration = 30;

const int red[] {255, 0, 0};
const int green[] {0, 255, 0};
const int blue[] {0, 0, 255};

// ----------------


void setup() {
  Serial.begin(9600);

  // Setup pins
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  pinMode(powerButtonPin, INPUT_PULLUP);

  pinMode(motorPin, OUTPUT);

  
  //pinMode(irInput, INPUT_PULLUP);

  // Set last stretch time
  lastStretchTime = getTimeInSeconds(millis());
}

void loop() {
  
  // Detect if the power button is being pressed
  if (digitalRead(powerButtonPin) == 1 && !powerButtonPressed) {

    // Used to make each whole button press only excecute once
    powerButtonPressed = true;

    // Toggle on / off status
    if (enabled) {
      enabled = false;
    } else {
      enabled = true;
      
      // Reset Stretch Time
      lastStretchTime = getTimeInSeconds(millis());
    }

    /*
    Serial.print(digitalRead(powerButtonPin));
    Serial.print(lastStretchTime);
    Serial.println();
    */
  }

  // Reset the power button pressed state
  if (digitalRead(powerButtonPin) == 0 && powerButtonPressed) {
    powerButtonPressed = false;
  }

  // If the timer is on, then keep track

  // hard code enabled because button does not work
  if (true) {
    
    //Serial.println(paused);
    //Serial.println("paused " + String(paused) + " | sitting " + String(sitting) + " | shouldStretch " + String(shouldStretch) + " | stretching " + String(stretching));
    
    
    // Get distance of the IR sensor to detect if sitting
    float distance = 13.0 * pow(analogRead(irInput) * (5.0 / 1023.0), -1);
    
  
    if( distance < 12) {
      sitting = true;
    } else {
      sitting = false;
    }

    
    if (!shouldStretch && !paused) {
      // Interpolate led color based on time till next stretch break
      interpolateLED(green, 
                     red, 
                     min((float) (getTimeInSeconds(millis()) - lastStretchTime) / stretchInterval, 1),
                     0.5);
    }

    
    // If sitting, start detecting time 
    if(sitting) {

      // If paused, then update the last stretch time
      if (paused) {
        lastStretchTime = getTimeInSeconds(millis()) - (timePaused - lastStretchTime);

        paused = false;
        timePaused = 0;
      }
      
      // If it is time for a stretch break
      if (getTimeInSeconds(millis()) - lastStretchTime > stretchInterval) {
        shouldStretch = true;
      } else {
        shouldStretch = false;
      }
      
    } else if (!sitting && !shouldStretch) {
      // Not sitting, pause

      if (!paused) {
        timePaused = getTimeInSeconds(millis());
      }
      
      paused = true;

      
      

    }

    if (shouldStretch && sitting) {
      // Time to stretch, motor on
      digitalWrite(motorPin, HIGH);
      delay(1000);
      digitalWrite(motorPin, LOW);
      delay(1000);
    } else {
      // Else, motor off
      digitalWrite(motorPin, LOW);
    }

    // Got up after sitting
    if (shouldStretch && !sitting && !stretching) {
      stretching = true;

      // Set stretch start
      stretchStart = getTimeInSeconds(millis());
      
    } else if (shouldStretch && sitting) {
      stretching = false;
    }

    if (shouldStretch && !sitting) {

      interpolateLED(blue, 
                     green, 
                     min((float) (getTimeInSeconds(millis()) - stretchStart) / stretchDuration, 1),
                     0.5);

      // Check if time stretching is at least the stretch duration
      if( getTimeInSeconds(millis()) - stretchStart > stretchDuration) {

        // Stretch Successful, reset
        shouldStretch = false;
        stretching = false;

        // Not sitting, reset the timer
        lastStretchTime = getTimeInSeconds(millis());

        setLED(0, 0, 0);
        delay(1000);
        
      }
      
    }
    
  } else {
    // If timer is off, turn LED off
    setLED(0, 0, 0);

    // Set shouldStretch to false
    shouldStretch = false;

    // Turn off the vibration motor if it was on
    digitalWrite(motorPin, LOW);
  }


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
