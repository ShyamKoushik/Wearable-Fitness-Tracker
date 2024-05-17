#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET -1    // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // OLED display address (for I2C)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Initialize with I2C addr 0x3C
float threshold = 1; // Adjust this threshold based on your accelerometer sensitivity
int steps = 0;
float lastAcceleration = 0.0;
int ax_pin = A1;
int ay_pin = A2;
int az_pin = A3;

// Heart rate sensor settings
unsigned long previousMillisGetHR = 0;
unsigned long previousMillisResultHR = 0;
const long intervalGetHR = 20;
const long intervalResultHR = 10000;
int PulseSensorSignal;
const int PulseSensorHRWire = A0;
const int LED_A1 = A1;
int UpperThreshold = 550;
int LowerThreshold = 500;
int cntHB = 0;
boolean ThresholdStat = true;
int BPMval = 0;
int x = 0;
int y = 0;
int lastx = 0;
int lasty = 0;

// Heart icon for OLED
const unsigned char Heart_Icon[] PROGMEM = {
  0x00, 0x00, 0x18, 0x30, 0x3c, 0x78, 0x7e, 0xfc, 0xff, 0xfe, 0xff, 0xfe, 0xee, 0xee, 0xd5, 0x56,
  0x7b, 0xbc, 0x3f, 0xf8, 0x1f, 0xf0, 0x0f, 0xe0, 0x07, 0xc0, 0x03, 0x80, 0x01, 0x00, 0x00, 0x00
};


void setup() {
  Serial.begin(9600);

  // Initialize the OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;) ;
  }

  display.display(); // Clear the display buffer
  delay(2000); // Delay to give time for the OLED to initialize

  display.clearDisplay(); // Clear the display buffer
  display.setTextSize(1); // Set text size
  display.setTextColor(SSD1306_WHITE); // Set text color
  display.setCursor(0, 0); // Set cursor position
  display.println(F("Step Counter")); // Display welcome message
  display.display(); // Show welcome message
  delay(2000); // Delay to show welcome message
}

void loop() {
  display.clearDisplay(); // Clear the display buffer
  display.setTextSize(1); // Set text size
  display.setTextColor(SSD1306_WHITE); // Set text color
  display.setCursor(0, 0); // Set cursor position

  float currentAcceleration = readAcceleration();

  if (isPeak(currentAcceleration, lastAcceleration)) {
    steps++; // Increment step count
  }

  //Serial.print(steps);

  // Update step count on the display
  display.print(F("Steps: "));
  display.println(steps);
  display.display(); // Show step count
  //delay(10); // Delay before updating display

  lastAcceleration = currentAcceleration;
  // Read heart rate data
  GetHeartRate();

  // Display steps and heart rate
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(F("Steps: "));
  display.println(steps);
  display.drawBitmap(0, 47, Heart_Icon, 16, 16, WHITE);
  display.drawLine(0, 43, 127, 43, WHITE);
  display.setTextSize(2);
  display.setCursor(20, 48);
  display.print(": ");
  display.print(BPMval);
  display.print(" BPM");
  display.display();

  //delay(10);
}

// Function to read acceleration from the accelerometer
float readAcceleration() {
  float acc = analogRead(ax_pin);
  float acceleration = (acc-261)/50 - 1;
  Serial.println(acceleration);
  return acceleration;
}

// Function to detect peaks in acceleration data
bool isPeak(float current, float previous) {
  return current > threshold && previous <= threshold;
}

// Function to read and calculate heart rate
void GetHeartRate() {
  unsigned long currentMillisGetHR = millis();
  if (currentMillisGetHR - previousMillisGetHR >= intervalGetHR) {
    previousMillisGetHR = currentMillisGetHR;
    PulseSensorSignal = analogRead(PulseSensorHRWire);
    if (PulseSensorSignal > UpperThreshold && ThresholdStat == true) {
      cntHB++;
      ThresholdStat = false;
      digitalWrite(LED_A1, HIGH);
    }
    if (PulseSensorSignal < LowerThreshold) {
      ThresholdStat = true;
      digitalWrite(LED_A1, LOW);
    }
    DrawGraph();
  }

  unsigned long currentMillisResultHR = millis();
  if (currentMillisResultHR - previousMillisResultHR >= intervalResultHR) {
    previousMillisResultHR = currentMillisResultHR;
    BPMval = cntHB * 6;
    Serial.print("BPM : ");
    Serial.println(BPMval);
    cntHB = 0;
  }
}

// Function to draw heart rate graph
void DrawGraph() {
  if (x > 127) {
    display.fillRect(0, 0, 128, 42, BLACK);
    x = 0;
    lastx = 0;
  }

  int ySignal = PulseSensorSignal;
  if (ySignal > 850) ySignal = 850;
  if (ySignal < 350) ySignal = 350;

  int ySignalMap = map(ySignal, 350, 850, 0, 40);
  y = 40 - ySignalMap;

  display.writeLine(lastx, lasty, x, y, WHITE);
  display.display();

  lastx = x;
  lasty = y;
  x++;
}