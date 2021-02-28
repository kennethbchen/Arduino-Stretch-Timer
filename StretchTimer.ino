// the setup function runs once when you press reset or power the board

int redPin= 12;
int greenPin = 11;
int bluePin = 10;

int buttonPin = 5;

int sitThreshold = 12; // Inches

bool enabled = false;
bool buttonPressed = false;

void setup() {
  Serial.begin(9600);
  
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  pinMode(buttonPin, INPUT);
}

// the loop function runs over and over again forever
void loop() {

  if(digitalRead(buttonPin) == 1 && !buttonPressed) {
    buttonPressed = true;

    if(enabled) {
      enabled = false;
      RGB_color(0,0,0);
    } else {
      enabled = true;
    }
    
    Serial.print(digitalRead(buttonPin));
    Serial.println();
  }

  if(digitalRead(buttonPin) == 0 && buttonPressed) {
    buttonPressed = false;
  }

  
  

  if (enabled) {
    RGB_color(255, 255, 255);
  } else {
    RGB_color(0, 0, 0);

  }
  

  
}

void RGB_color(int red_light_value, int green_light_value, int blue_light_value) {
  analogWrite(redPin, red_light_value);
  analogWrite(greenPin, green_light_value);
  analogWrite(bluePin, blue_light_value);
}

long microsecondsToInches(long microseconds) {
   return microseconds / 74 / 2;
}

void writeLED(float inches) {

  
  if(inches <= sitThreshold) {
    
    // If sitting
    // Green
    RGB_color(0, 255, 0);
    
  } else {
    
    // If not sitting
    // Yellow
    RGB_color(255, 125, 0);
    
  }
}
