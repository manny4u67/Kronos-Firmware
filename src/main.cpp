/***************************************************************
Emmanuel Addey
KRONOS V.2
10/5/2024
USB BRANCH
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
#include <Adafruit_TinyUSB.h>

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

// Timer Function to set a timer 
class TimerX {
  private:
    unsigned long previousMillis = 0UL; 
    unsigned long currentMillis = 0UL;
    long interval = 1000UL;

  public:
    bool timerActive = false;
    TimerX(){

    } 


    void startTimer(){
      previousMillis = millis();
      timerActive = true;
    }

    void stopTimer(){
      timerActive = false;
    }

    bool checkTimer(){
      currentMillis = millis();
      return timerActive && (currentMillis - previousMillis >= interval); //will return true if timer is active and time has passed
    }

    void resetTimer(){
      previousMillis = millis();
    }

    void setIntervalSeconds(long interval){
      this -> interval = interval * 1000;
    }

    void setIntervalMinutes(long interval){
      this -> interval = interval * 60 * 1000;
    }
    long getInterval(){
      return interval;
    }

    long getTimeLeft(){
      unsigned long elapsedTime = (millis() - previousMillis) / 1000;
      long timeLeft = interval - elapsedTime;
      return timeLeft;
    }

    unsigned long getPreviousMillis(){
      return previousMillis;
    }

    unsigned long getCurrentMillis(){
      return currentMillis;
    }

};
//
class Button
{
  private:
    uint8_t btn;
    uint16_t state;
  public:
    void begin() {
      state = 0;
    }
  bool debounce(bool output) {
    static uint16_t state = 0;
    state = (state << 1) | output | 0xfe00;
    return (state == 0xfe01);
  }
};

Button hall1;
Button hall2;
Button hall3;
Button hallall;

bool hallScanC1 = 0;
bool hallScanC2 = 0;
bool hallScanC3 = 0;

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
    int option=1;
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
            return hallReadCal() < trigPoint;
        break;

        case 1:
            return hallRead() < 1200;
        break;

        default:
          return false;
        break;
        }
      }
};

HallX h[] = {HallX(HALL0), HallX(HALL1), HallX(HALL2), HallX(HALL3)};
bool hallcalibrated = 0;

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

//USB keyboard library stuff
uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_KEYBOARD()
};

Adafruit_USBD_HID usb_hid;



// Delay Control
unsigned long previousMillis = 0UL;
const long interval = 500UL;

// Delay Control 2
unsigned long previousMillis2 = 0UL;
const long interval2 = 100UL;

// Delay Control 3 (Function Timer)
unsigned long previousMillis3 = 0UL;
unsigned long currentMillis3 = 0UL; 
const long interval3 = 750UL;
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

// Task Handler
TaskHandle_t Task1;
TaskHandle_t HallScan;
/*******************************************************************END OF DEFINITIONS AND DECLERATIONS*****************************************************/

void timedblink() {
  int crossValue = CRGB::Black;
  int indicies [] = {};
  currentMillis3 = millis(); 
  if (currentMillis3 - previousMillis3 >= interval3) { 
    if (blink3 == 0) {
      blink3 = 1;
      leds[20] = CRGB::Blue;
      leds[21] = CRGB::Blue;
      leds[22] = CRGB::Blue;
      leds[23] = CRGB::Blue;
      leds[24] = CRGB::Blue;
      leds[25] = CRGB::Blue;
      leds[26] = CRGB::Blue;
      leds[27] = CRGB::Blue;
      leds[28] = CRGB::Blue;
      leds[29] = CRGB::Blue;
      leds[6] = CRGB::Blue;
      leds[13] = CRGB::Blue;
      leds[33] = CRGB::Blue;
      leds[46] = CRGB::Blue;


    } else {
      blink3 = 0;
      leds[20] = CRGB::White;
      leds[21] = CRGB::White;
      leds[22] = CRGB::White;
      leds[23] = CRGB::White;
      leds[24] = CRGB::White;
      leds[25] = CRGB::White;
      leds[26] = CRGB::White;
      leds[27] = CRGB::White;
      leds[28] = CRGB::White;
      leds[29] = CRGB::White;
      leds[6] = CRGB::White;
      leds[13] = CRGB::White;
      leds[33] = CRGB::White;
      leds[46] = CRGB::White;
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
    oled.print("PID: "); oled.print(xPortGetCoreID()); oled.print(" CAL: "); oled.print(hallcalibrated); oled.print(" K1 "); oled.print(h[1].checkHallTrig(1));
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
  hallcalibrated = 0;
  int startTimer = 0;
  int timerSet = 1500;//millis
  bool calibrationComplete = false;
  bool arrayCalibrationComplete = false;
  currentHall = 1;
  displayLayer=1;

  log_e("Calibration Begin");
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
hallcalibrated = 1;
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
  if(h[1].hallReadCal() == 0) {
    while(h[1].hallReadCal() == 0) {
    bleKeyboard.print("HALL1");
    oled.clearDisplay();
    oled.setCursor(0, 2);
    oled.setTextSize(2);
    oled.print("HALL1");
    oled.display();
    }

  }
  if(h[2].hallReadCal() == 0) {
    while(h[2].hallReadCal() == 0) {
    bleKeyboard.print("HALL2");
    oled.clearDisplay();
    oled.setCursor(0, 2);
    oled.setTextSize(2);
    oled.print("HALL2");
    oled.display();
    }
  }
  if(h[3].hallReadCal() == 0) {
    while(h[3].hallReadCal() == 0) {
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

void Task1code ( void * pvparameters ) {
  for(;;) {
  timedblink();
  }
}

void HallScanCode( void * pvparameters) {

  for(;;) {
    
    if(hallall.debounce(h[1].checkHallTrig(1)&&h[2].checkHallTrig(1)&&h[3].checkHallTrig(1)&&hallcalibrated)) {
      bleKeyboard.print("KRONOS!!");
    }
    else{
      if(hall1.debounce(h[1].checkHallTrig(1)&&hallcalibrated)) {
        bleKeyboard.press(KEY_LEFT_CTRL);
        bleKeyboard.press('c');
        log_e("HALL 1 PRESSED");
        }
      else if (hall2.debounce(h[2].checkHallTrig(1)&&hallcalibrated)) {
        bleKeyboard.press(KEY_LEFT_CTRL);
        bleKeyboard.press('v');
        log_e("HALL 2 PRESSED");
      }
      else if (hall3.debounce(h[3].checkHallTrig(1)&&hallcalibrated)) {
        bleKeyboard.print("it's ");
        log_e("HALL 3 PRESSED");
      }
    }
    bleKeyboard.releaseAll();
  }
}

/* void hid_report_callback(uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
  (void) report_id;
  (void) bufsize;

  // LED indicator is output report with only 1 byte length
  if (report_type != HID_REPORT_TYPE_OUTPUT) return;

  // The LED bit map is as follows: (also defined by KEYBOARD_LED_* )
  // Kana (4) | Compose (3) | ScrollLock (2) | CapsLock (1) | Numlock (0)
  uint8_t ledIndicator = buffer[0];

  // turn on LED if capslock is set
  digitalWrite(LED_BUILTIN, ledIndicator & KEYBOARD_LED_CAPSLOCK);
} */
void setup() { 
  Serial.begin (115200);
  Serial.setDebugOutput(true);
 
  Serial.println("Starting KRONOS!");
  hall1.begin();
  hall2.begin();
  hall3.begin();
  hallall.begin();

  //Beyboard Initialization Check
  delay(1000);
    if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  // Setup HID
  usb_hid.setBootProtocol(HID_ITF_PROTOCOL_KEYBOARD);
  usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.setStringDescriptor("KRONOS");

  // Set up output report (on control endpoint) for Capslock indicator
  //usb_hid.setReportCallback(NULL, hid_report_callback); //THIS WILL ALLOW FOR REPORTING STATUS LIKE CAPS LOCK

  usb_hid.begin();

  // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  xTaskCreatePinnedToCore(
    Task1code, /* Function to implement the task */
    "Task1", /* Name of the task */
    10000, /* Stack size in words */
    NULL, /* Task input parameter */
    0, /* Priority of the task */
    &Task1, /* Task handle. */
    1); /* Core where the task should run */

  xTaskCreatePinnedToCore(
    HallScanCode, /* Function to implement the task */
    "HallScan", /* Name of the task */
    10000, /* Stack size in words */
    NULL, /* Task input parameter */
    0, /* Priority of the task */
    &HallScan, /* Task handle. */
    1); /* Core where the task should run */  

  //OLED Display
  //oled.setRotation(1); //rotates text on OLED 1=90 degrees, 2=180 degrees
  Wire.begin(SDA0_Pin, SCL0_Pin);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("failed to start SSD1306 OLED"));
    while (1);
  }
  
  //Misc IO



  //Fast LED
	FastLED.addLeds<ARGB_CHIPSET,DATA_PIN,RGB>(leds,NUM_LEDS);
	FastLED.addLeds<ARGB_CHIPSET,DATA_PIN2,RGB>(leds2,NUM_LEDS2);
	FastLED.setBrightness(25);
  
  //Keyboard
  bleKeyboard.begin();
  delay(1000);

  //Startup Screen
  updateDisplay(0);
  updateDisplay(0); 
  updateDisplay(0);
  delay(3000);

  // calibration only at boot up **(add calibration prompt or use NVS storage to skip this if already done)
  calibrateHallButtons();
  Serial.println("KRONOS INITIALIZED...");
}
void loop() { 
  uint8_t const report_id = 0;
  uint8_t const modifier = 0;
  uint8_t keycode [6] = {0x04, 0x00, 0x00, 0x00, 0x00, 0x00};
  long start = micros();//KEEP AT BEGINNING OF LOOP, FOR TIMER

	unsigned long currentMillis = millis(); 
  unsigned long currentMillis2 = millis(); 

    if (!TinyUSBDevice.mounted()) {
    log_e("not mounted");
    return;
  }
  log_e("past mount");
  usb_hid.keyboardReport(report_id, modifier, keycode);
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
  int timeLeft = getTimeLeft();
  displayLayer=2;
  updateDisplay(timeLeft);
  // Add any additional logic or delays as needed
  duration = micros() - start; // KEEP AT END, FOR TIMER
}
