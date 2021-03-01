// ----- PINS -----
const int redPin = 12;
const int greenPin = 11;
const int bluePin = 10;

const int powerButtonPin = 5;
const int stretchButtonPin = 6;

// ----------------
// ---- STATE -----

bool enabled = false;
bool buttonPressed = false;
bool shouldStretch = false;

// Seconds
float lastStretchTime = 0;

// ----------------
// --- SETTINGS ---
// Seconds, Time between stretch breaks
const float stretchInterval = 10;

// Seconds, Duration of stretch break
const 
float stretchDuration = 30;

// ----------------


void setup() {
  Serial.begin(9600);

  // Setup pins
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  pinMode(powerButtonPin, INPUT_PULLUP);
  pinMode(stretchButtonPin, INPUT_PULLUP);

  // Set last stretch time
  lastStretchTime = millis();
}

void loop() {

  // Detect if the power button is being pressed
  if (digitalRead(powerButtonPin) == 1 && !buttonPressed) {

    // Used to make each whole button press only excecute once
    buttonPressed = true;

    // Toggle on / off status
    if (enabled) {
      enabled = false;
    } else {
      enabled = true;
      
      // Reset Stretch Time
      lastStretchTime = getTimeInSeconds(millis());
    }

    Serial.print(digitalRead(powerButtonPin));
    Serial.print(lastStretchTime);
    Serial.println();
  }

  // Reset the power button pressed state
  if (digitalRead(powerButtonPin) == 0 && buttonPressed) {
    buttonPressed = false;
  }

  // If the timer is on, then keep track
  if (enabled) {

    // If the stretch button pin is pressed
    if (digitalRead(stretchButtonPin)) {

      // Reset last stretch time to current time
      lastStretchTime = getTimeInSeconds(millis());
    }
    
    // Interpolate led color based on time till next stretch break
    writeLED();

    // If it is time for a stretch break
    if (getTimeInSeconds(millis()) - lastStretchTime > stretchInterval) {
      shouldStretch = true;
    }

    if (shouldStretch) {
      
    }
    
  } else {
    // If timer is off, turn LED off
    setLED(0, 0, 0);
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

void writeLED() {

  // Calculate value between 0.0 and 1.0 that indicates percentage of stretch interval passed
  float proportionLeft = min((float) (getTimeInSeconds(millis()) - lastStretchTime) / stretchInterval, 1);

  if ( proportionLeft < 1.0 ) {
    Serial.println(proportionLeft);
    setLED((int) (255 * (proportionLeft)), (int) (255 * (1 - proportionLeft)), 0);
  }


}
