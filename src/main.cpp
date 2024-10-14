/***************************************************************
Emmanuel Addey
KRONOS V.2
10/5/2024

Wiring
IO Pin  Variable    Type    IO Mode   Pull-up   Hardware
---------------------------------------------------------
17      SCL0_Pin    Digital   I2C       Yes     SCL Serial
18      SDA0_Pin    Digital   I2C       Yes     SDA Serial
9       HALL1       Analog    Input     N/A     Hall Effect Sensor
10      HALL2       Analog    Input     N/A     Hall Effect Sensor
14      HALL3       Analog    Input     N/A     Hall Effect Sensor
35                  Digital   FastLED   N/A     Button RGBs
48                  Digital   FastLED   N/A     RGB Array
***************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <FastLED.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <BleKeyboard.h>
#include <AS5600.h>
#include <String.h>

// Data Pins
#define DATA_PIN 48 // ARGB DATA PIN 
#define DATA_PIN2 38 // ARGB DATA PIN 2
#define SDA0_Pin 18   // ESP32 SDA PIN
#define SCL0_Pin 17   // ESP32 SCL PIN

// For ARGB ARRAY 5 X 10 + 1
#define NUM_LEDS 51 // ***Delete this when new board is made
#define NUM_DISLEDS 50 // ***Will need new logic when new board is made
#define ARGB_CHIPSET WS2812B
#define STATUS_LED 50 // ***Change to 0 for new board
static uint8_t hue = 0;
static uint8_t hue2 = 100;
CRGB leds[NUM_LEDS];

// For Addressable LEDS 2
#define NUM_LEDS2 3 
CRGB leds2[NUM_LEDS2];

// Hall Effect Rotary Encoder
AS5600 as5600;   //  use default Wire
static uint32_t newPosition = 0;
static uint32_t oldPosition = 0;
int rpm;
int lastEncoderValue = 0;
int currentEncoderValue = 0;


// Hall Effect Button Control
#define HALL0 3 // HALL EFFECT SENSOR 0 GPIO
#define HALL1 9 // HALL EFFECT SENSOR 1 GPIO
#define HALL2 10 // HALL EFFECT SENSOR 2 GPIO
#define HALL3 14 // HALL EFFECT SENSOR 3 GPIO
int hallsens = 3;
int currentHall = 1;

class HallX {
  private:
    static const int averagingSamples =  2;
    int readIndex = 0;
    int total = 0; // used in hallReadClean
    int average = 0; // used in hallReadClean
    int precision = 1; // precison 1 = 128 2= 256 3 = 512 4 = 1024 5 = 2048
    int trigPoint = 0;
    int value; 
    int readings[averagingSamples]; // used in hallReadClean
    int minVal=4095;
    int maxVal=0;
 
    // check if AS5600 library is included
    #if __has_include (<AS5600.h>)
      const bool USE_AS5600 = 1;
    #else
      const bool USE_AS5600 = 0;
    #endif

  public:
    byte pin;
    HallX(byte pin) {
      this->pin = pin;

    }

    // Return raw sensor value from 0 to 4095
    int hallRead(){
      value = analogRead(pin);
      return value;
    }

    // Set the min and max hall analog values
    int hallCal(){
      // record hall analog max value
      value = analogRead(pin);
      if (value > maxVal) {
        maxVal = value;
      }
      // record hall analog min value
      if (value < minVal) {
        minVal = value;
      }
      return value;
    }

    // set hall precision variable (128, 256, 512, 1024, 2048)
    void setSens(int mutliplier){
      precision = mutliplier;
    }

    // return hall analog value that has been proper precision **PREFFERED FOR RAW OUTPUT
    int hallReadCal(){
      int mapMax;
      mapMax = precision * 128;
      value = analogRead(pin)-35; // record hall analog value
      value = constrain(value, minVal, maxVal); // in case the hall analog value is outside the range seen during calibration67
      value = map(value, minVal, maxVal, 0, mapMax); // map the hall analog value to a range from 0 to 1000
      return value;
    }

    // return hall analog value with calibration and noise reduction !!SLOW!!
    int hallReadClean(){
      total = total - readings[readIndex];
      readings[readIndex] = hallReadCal(); // read from the hall analog:
      total = total + readings[readIndex]; // add the reading to the total:
      readIndex = readIndex + 1; // advance to the next position in the array:
    // if we're at the end of the array...
      if (readIndex >= averagingSamples) {
        // ...wrap around to the beginning:
        readIndex = 0;
        average = total / averagingSamples;
      }
    delay(1);
    return average;
    }

    // set hall trigger point
    int hallSetTrigRotary(){
      int hallTrigMax = (precision * 128);
      // check if AS5600 library is included ((determined by ))
      if (USE_AS5600){ 
        int hallSetCurrent = as5600.rawAngle();
        map(hallSetCurrent, 0, 4056, 0, hallTrigMax);
        trigPoint = hallSetCurrent / 2;
        return trigPoint;
      }
      else{
        trigPoint = 0;
        return trigPoint;
      }
    }

    bool checkHallTrig(int option){
      switch (option) {
        case 0:
          if (hallReadCal() < trigPoint) {
            return true;
          }   
          else {
            return false; 
          }
        break;

        case 1:
          if(hallRead() < 1300) {
            return true;
          }
          else {
            return false; 
          }
        break;

        default:
          return false;
        break;
        }
      }
};

//HallX h1(HALL1); // Hall Effect Sensor 1 Object
//HallX h2(HALL2); // Hall Effect Sensor 2 Object
//HallX h3(HALL3); // Hall Effect Sensor 3 Object
//HallX h[] = {h1, h2, h3};

HallX h[] = {HallX(HALL0), HallX(HALL1), HallX(HALL2), HallX(HALL3)};

// ARGB ARRAY mapping
const int MAPROWS = 10;
const int MAPCOLS = 5;
int displayrow;
int displaycol;
int ledmap[MAPROWS][MAPCOLS]={
  {0, 19, 20, 39, 40},
  {1, 18, 21, 38, 41}, 
  {2, 17, 22, 37, 42}, 
  {3, 16, 23, 36, 43}, 
  {4, 15, 24, 35, 44}, 
  {5, 14, 25, 34, 45}, 
  {6, 13, 26, 33, 46}, 
  {7, 12, 27, 32, 47}, 
  {8, 11, 28, 31, 48}, 
  {9, 10, 29, 30, 49}
};

// OLED Display
#define SCREEN_WIDTH 128 // OLED width,  in pixels
#define SCREEN_HEIGHT 64 // OLED height, in pixels
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
int displayLayer =0;
String displayText = "Press Key 1"; // 
String displayText2 = "Keep Pressing";

// Logo Bitmap for OLED Display
const unsigned char myBitmap [] PROGMEM = 
{
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xe0, 0x00, 0x0f, 0xff, 0x00, 0x00, 0x07, 0xf8, 0x00, 0x01, 0xf8, 0x00, 0x00, 0x07, 
	0xff, 0xff, 0xe0, 0x00, 0x0f, 0xfe, 0x00, 0x00, 0x0f, 0xf8, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x0f, 
	0xff, 0xff, 0xc0, 0x00, 0x0f, 0xfe, 0x00, 0x00, 0x1f, 0xf8, 0x00, 0x01, 0xe0, 0x00, 0x00, 0x0f, 
	0xff, 0xff, 0xc0, 0x00, 0x1f, 0xfc, 0x00, 0x00, 0x1f, 0xf8, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x1f, 
	0xff, 0xff, 0xc0, 0x00, 0x1f, 0xf8, 0x00, 0x00, 0x3f, 0xf8, 0x00, 0x00, 0x80, 0x00, 0x00, 0x3f, 
	0xff, 0xff, 0xc0, 0x00, 0x1f, 0xf0, 0x00, 0x00, 0x7f, 0xfc, 0x00, 0x00, 0x80, 0x00, 0x00, 0x7f, 
	0xff, 0xff, 0x80, 0x00, 0x1f, 0xe0, 0x00, 0x00, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 
	0xff, 0xff, 0x80, 0x00, 0x3f, 0xc0, 0x00, 0x01, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 
	0xff, 0xff, 0x80, 0x00, 0x3f, 0x80, 0x00, 0x03, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 
	0xff, 0xff, 0x80, 0x00, 0x3f, 0x80, 0x00, 0x07, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 
	0xff, 0xff, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x07, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 
	0xff, 0xff, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x0f, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 
	0xff, 0xff, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x1f, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 
	0xff, 0xff, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 
	0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x07, 0xff, 0xff, 
	0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x0f, 0xff, 0xff, 
	0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x1f, 0xff, 0xff, 
	0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x7f, 0xff, 0xff, 
	0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x7f, 0xff, 0xff, 
	0xff, 0xf0, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 
	0xff, 0xf0, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 
	0xff, 0xf0, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 
	0xff, 0xf0, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 
	0xff, 0xe0, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 
	0xff, 0xe0, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 
	0xff, 0xe0, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 
	0xff, 0xe0, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 
	0xff, 0xc0, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 
	0xff, 0xc0, 0x00, 0x08, 0x00, 0x01, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 
	0xff, 0xc0, 0x00, 0x18, 0x00, 0x01, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 
	0xff, 0xc0, 0x00, 0x1c, 0x00, 0x00, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 
	0xff, 0x80, 0x00, 0x1c, 0x00, 0x00, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 
	0xff, 0x80, 0x00, 0x1e, 0x00, 0x00, 0x7f, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 
	0xff, 0x80, 0x00, 0x3e, 0x00, 0x00, 0x7f, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 
	0xff, 0x80, 0x00, 0x3e, 0x00, 0x00, 0x3f, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 
	0xff, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x3f, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 
	0xff, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x3f, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 
	0xff, 0x00, 0x00, 0x7f, 0x80, 0x00, 0x1f, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 
	0xff, 0x00, 0x00, 0x7f, 0x80, 0x00, 0x1f, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 
	0xfe, 0x00, 0x00, 0x7f, 0xc0, 0x00, 0x0f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 
	0xfe, 0x00, 0x00, 0x7f, 0xc0, 0x00, 0x0f, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 
	0xfe, 0x00, 0x00, 0xff, 0xe0, 0x00, 0x07, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 
	0xfc, 0x00, 0x00, 0xff, 0xe0, 0x00, 0x07, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 
	0xfc, 0x00, 0x00, 0xff, 0xe0, 0x00, 0x03, 0xf8, 0x00, 0x00, 0x04, 0x00, 0x00, 0x7f, 0xff, 0xff, 
	0xfc, 0x00, 0x00, 0xff, 0xf0, 0x00, 0x03, 0xf0, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x7f, 0xff, 0xff, 
	0xfc, 0x00, 0x01, 0xff, 0xf0, 0x00, 0x03, 0xe0, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x7f, 0xff, 0xff, 
	0xf8, 0x00, 0x01, 0xff, 0xf8, 0x00, 0x01, 0xc0, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x3f, 0xff, 0xff, 
	0xf8, 0x00, 0x01, 0xff, 0xf8, 0x00, 0x01, 0xc0, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x3f, 0xff, 0xff, 
	0xf8, 0x00, 0x03, 0xff, 0xfc, 0x00, 0x00, 0x80, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};
 
//Bluetooth keyboard library stuff
BleKeyboard bleKeyboard; 

// Delay Control
unsigned long previousMillis = 0UL;
const long interval = 500UL;

// Delay Control 2
unsigned long previousMillis2 = 0UL;
const long interval2 = 100UL;

// Delay Control 3
unsigned long previousMillis3 = 0UL;
unsigned long currentMillis3 = 0UL; 
const long interval3 = 100UL;
bool blink3 = 0;

// Timer
bool blink= 0;
unsigned long startTime;
int timerDuration = 25 * 60; // 25 minutes in seconds

// Cycletime benchmark
long duration;

// Layer control 
int currentLayer = 0;
const int MAX_LAYERS = 3;
int tempcurrentLayer= 0 ;


/*******************************************************************END OF DEFINITIONS AND DECLERATIONS*****************************************************/
void timedblink() {
  currentMillis3 = millis(); 
  if (currentMillis3 - previousMillis3 >= interval2) { 
    if (blink3 == 0) {
      blink3 = 1;
    } else {
      blink3 = 0;
    }
  previousMillis3 = currentMillis3; // LEAVE THIS ALONE
	}
}
void rgbmap(int row,int column){
  switch (1)
  {
  case 1:
    leds[ledmap[row][column]] = CRGB::White;
    FastLED.show();
  break;

  case 2:
    for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Black;
    }
    leds[ledmap[row][column]] = CRGB::White;
    FastLED.show();
  break;

  default:

  break;
  }


}

void fadeall() { for(int i = 0; i < NUM_DISLEDS; i++) { leds[i].nscale8(250); } }

void rainbowtime() {
	Serial.print("x");
	// First slide the led in one direction
	for(int i = 0; i < NUM_DISLEDS; i++) {
		// Set the i'th led to red 
		leds[i] = CHSV(hue++, 255, 255);
		// Show the leds
		FastLED.show(); 
		// now that we've shown the leds, reset the i'th led to black
		// leds[i] = CRGB::Black;
		fadeall();
	}
  	Serial.print("x");

	// Now go in the other direction.  
	for(int i = (NUM_DISLEDS)-1; i >= 0; i--) {
		// Set the i'th led to red 
		leds[i] = CHSV(hue++, 255, 255);
		// Show the leds
		FastLED.show();
		// now that we've shown the leds, reset the i'th led to black
		// leds[i] = CRGB::Black;
		fadeall();
		// Wait a little bit before we loop around and do it again
  }
}
void updateDisplay(int timeLeft) {
  oled.clearDisplay(); // clear display
  switch (displayLayer) {
  // boot up screen
  case 0:
    oled.clearDisplay(); // clear display
    oled.setTextSize(2);         // set text size
    oled.setTextColor(WHITE);    // set text color
    oled.drawBitmap(0, 0, myBitmap, 128, 64, WHITE);
    oled.display();
    break;
  // calibration screen
  case 1:
    oled.clearDisplay(); // clear display
    oled.setTextSize (2);         // set text size
    oled.setTextColor(WHITE);    // set text color
    oled.setCursor(0, 2);       // set position to display (x, y)
    oled.println(displayText);
    oled.println(h[currentHall].hallCal());
    oled.display(); 
    break;
  
  // maintenance mode screen67
  case 2:
    oled.setTextSize(1);         // set text size
    oled.setTextColor(WHITE);    // set text color
    oled.setCursor(0, 2);       // set position to display (x, y)
    oled.print("Rotary:");
    oled.print("BLEKEY:");
    oled.println(bleKeyboard.isConnected());
    oled.print("HALL1:");
    oled.print(h[1].hallReadCal());
    oled.print(" HALL2:");
    oled.print(h[2].hallReadCal());
    oled.print(" HALL3:");
    oled.println(h[3].hallReadCal());
    oled.print("TempLayer");
    oled.println(tempcurrentLayer);
    //oled.print("ReadAngle: ");
    //oled.println(as5600.readAngle());
    oled.print("RawAngle: ");
    oled.println(as5600.rawAngle());
    //oled.println(as5600.rawAngle() * AS5600_RAW_TO_DEGREES);
    //oled.print("Cumulative: ");
    //oled.println(as5600.getCumulativePosition());
    //oled.print("Revolutions: ");
    //oled.println(as5600.getRevolutions());
    //oled.print("RPM: ");
    //oled.println(rpm);
    oled.print("Cycletime: ");
    oled.println(duration);
    oled.print("BLINK3: ");
    oled.println(blink3);
    oled.display();
    break;
  case 3:
    oled.clearDisplay();
    oled.setTextSize(2);
    oled.setTextColor(WHITE);
    oled.setCursor(0, 0);
    oled.print("Time: ");
    oled.print(timeLeft / 60);
    oled.print(":");
    oled.print(timeLeft % 60);
    oled.setCursor(0, 32);
    oled.print("Layer: ");
    oled.print(currentLayer);
    oled.display();
    break;
  default:
    oled.display();
    break;
  }
  }
void calibrateHallButtons() {
  int startTimer = 0;
  int timerSet = 5000;//millis
  bool calibrationComplete = false;
  bool arrayCalibrationComplete = false;
  currentHall = 1;
  displayLayer=1;

  while(currentHall <= 3) {
    calibrationComplete = 0;
    while(!calibrationComplete) {
 
      while(h[currentHall].checkHallTrig(1) == 0) {
        displayText = "PRESS HALL " + String(currentHall);
        updateDisplay(0);
      }
      startTimer = millis();
      arrayCalibrationComplete = 0;
      while((h[currentHall].checkHallTrig(1) == 1) && (!arrayCalibrationComplete)) {
        h[currentHall].hallCal();
        if ((startTimer + (timerSet / 2)) > millis()) {
          displayText = "FULLY PRESS";
          updateDisplay(0); 
        }
        else if ((startTimer + (timerSet/2)+(timerSet / 4)) > millis()) {
          displayText = "ALMOST THERE";
          updateDisplay(0);
        }
        else if((startTimer + timerSet) < millis()) {
          displayText = "HALL " + String(currentHall) + " CALIBRATED";
          updateDisplay(0);
          delay(1500);
          arrayCalibrationComplete = 1;
          calibrationComplete = 1;
          currentHall++;
        }
        updateDisplay(0);
      }
    }  
  }
}
void updateLEDs(int timeLeft, int totalTime) {
  int ledsOn = map(timeLeft, 0, totalTime, 0, NUM_LEDS);
  for (int i = 0; i < NUM_LEDS; i++) {
    if (i < ledsOn) {
      leds[i] = CRGB::White;
    } else {
      leds[i] = CRGB::Black;
    }
  }
  FastLED.show();
}
void checkMap() {
  for(int row = 0; row < MAPROWS; row++) {
    for(int column = 0; column < MAPCOLS; column++) {
      rgbmap(row, column);
    }
  } 
}

void handleEncoder() {
  currentEncoderValue = as5600.rawAngle();
}
void checkHallPress() {
  if(h[0].hallReadCal() == 0) {
    while(h[0].hallReadCal() == 0) {
    bleKeyboard.print("HALL1");
    oled.clearDisplay();
    oled.setCursor(0, 2);
    oled.setTextSize(2);
    oled.print("HALL1");
    oled.display();
    }

  }
  if(h[1].hallReadCal() == 0) {
    while(h[1].hallReadCal() == 0) {
    bleKeyboard.print("HALL2");
    oled.clearDisplay();
    oled.setCursor(0, 2);
    oled.setTextSize(2);
    oled.print("HALL2");
    oled.display();
    }
  }
  if(h[2].hallReadCal() == 0) {
    while(h[2].hallReadCal() == 0) {
    bleKeyboard.print("HALL3");
    oled.clearDisplay();
    oled.setCursor(0, 2);
    oled.setTextSize(2);
    oled.print("HALL3");
    oled.display();
  }

}
}

void setButtonRGB() {
  //leds2[0] = CHSV(hue2, 255, 255);
  //leds2[1] = CHSV(hue2, 255, 255);
  //leds2[2] = CHSV(hue2, 255, 255);
    //FastLED.show();
  }

void adjustVolume(int change) {
  // Implement volume adjustment logic here
}

void changeLayer(int layermode) {
  switch(layermode) {
  case 0:
    tempcurrentLayer = map(currentEncoderValue, 0, 4095, 0, 100);
    if((tempcurrentLayer>=0)&&(tempcurrentLayer<=33)){
      leds2[0] = CRGB::White;
      leds2[1] = CRGB::Black;
      leds2[2] = CRGB::Black;
    }
    else if ((tempcurrentLayer>34)&&(tempcurrentLayer<=66)){
      leds2[0] = CRGB::Black;
      leds2[1] = CRGB::White;
      leds2[2] = CRGB::Black;
    }
    else if (tempcurrentLayer>66){
      leds2[0] = CRGB::Black;
      leds2[1] = CRGB::Black;
      leds2[2] = CRGB::White;
    }
    FastLED.setBrightness(tempcurrentLayer);
  FastLED.show();
  break;
  case 1:
    currentLayer = (currentLayer + 1) % MAX_LAYERS;
    break;
      default:
      FastLED.show();
      break;
  }
}



void startTimer() {
  startTime = millis();
}

void pauseTimer() {
  
}

void stopTimer() {
  
}

int getTimeLeft() {
  unsigned long elapsedTime = (millis() - startTime) / 1000;
  int timeLeft = timerDuration - elapsedTime;
  return timeLeft > 0 ? timeLeft : 0;
}

void setup() { 
  //OLED Display
  //oled.setRotation(1); //rotates text on OLED 1=90 degrees, 2=180 degrees
  Wire.begin(SDA0_Pin, SCL0_Pin);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("failed to start SSD1306 OLED"));
    while (1);
  }
  
  //Misc IO
  Serial.begin (115200);


  //Fast LED
	FastLED.addLeds<ARGB_CHIPSET,DATA_PIN,RGB>(leds,NUM_LEDS);
	FastLED.addLeds<ARGB_CHIPSET,DATA_PIN2,RGB>(leds2,NUM_LEDS2);
	FastLED.setBrightness(25);
  
  //Keyboard
  bleKeyboard.begin();
  delay(1000);

  //Startup Screen
  updateDisplay(0);
  delay(3000);

  // calibration only at boot up **(add calibration prompt or use NVS storage to skip this if already done)
  calibrateHallButtons();
}
void loop() { 
  long start = micros();//KEEP AT BEGINNING OF LOOP, FOR TIMER
	unsigned long currentMillis = millis(); 
  unsigned long currentMillis2 = millis(); 

  //Everything Within Is Delayed 500MS
  if (currentMillis - previousMillis >= interval) { 
    if (blink){
      leds[STATUS_LED] = CHSV(hue2, 255, 255);
      FastLED.show();
      blink = 0;
    }
    else {
      leds[STATUS_LED] = CHSV(hue2, 255, 0);
      FastLED.show(); 
      blink = 1;
    }
    previousMillis = currentMillis; // LEAVE THIS ALONE
  }

  //Everything Within Is Delayed 100MS
  if (currentMillis2 - previousMillis2 >= interval2) { 
  previousMillis2 = currentMillis2; // LEAVE THIS ALONE
	}
  //updateLEDs(timeLeft);
  //rainbowtime();
  //checkHallPress();
  handleEncoder();
  changeLayer(0);
  timedblink();
  int timeLeft = getTimeLeft();
  displayLayer=2;
  updateDisplay(timeLeft);
  // Add any additional logic or delays as needed
  duration = micros() - start; // KEEP AT END, FOR TIMER
}
