#include <avdweb_AnalogReadFast.h> // use analogReadFast(adcPin);
#include "ssd1306.h"
#include "nano_gfx.h"
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>

// Pin config:
#define SCK 13
#define SSR2_EN A2
#define SSR1_EN A3
#define SDA A4
#define SCL A5
#define KNOB2 A7 // adjusting point of curve --> y
#define KNOB1 A6 // adjusting phase curve --> x
#define CS_TEMP1 10
#define CS_TEMP2 9
#define CS_TEMP3 8
#define BUTTON1 2 // ovenStatus
#define BUTTON2 7 // switching modes
#define ESTOP 5
#define BUZZER 4

//settings
#define filterFactor 0.3
#define timeDilation 1.0
#define temp1_Zero 4
#define temp2_Zero 0
#define temp3_Zero 0
#define temp1_Hundred 104.2
#define temp2_Hundred 100
#define temp3_Hundred 100

//PID settings
#define kp_p 0.5
#define kp_i 0.01
#define kp_d 10
#define kp_dd -3

const float cookTime[] = {0, 6420, 10020, 14220, 21420, 22956}; // Seconds
const float cookTemp[] = {0, 107,  107,   177,   177,   49}; // degrees Celcius
#define cookArrSize 6

// Status variables
float targetTemp; // target temperature
float currentTemp;

bool relayStatus = false; // oven itself is on/off
bool ovenStatus = false; // target for oven is on/off
uint8_t errorCode = 0b00000000; // Bitmap for error conditions: 0 = ok, 1 = error
// |Reserved|Reserved|Reserved|Reserved|Sensor3|Sensor2|Sensor1|Estop|

#define avgBufLength 10
float currentTempBuf[avgBufLength]; // running average
uint8_t avgBufIdx = 0;
float fps;
bool screenMode = false;
float previous_error = 0, error_integral = 0; //req vars for PID algorithm
float ovenPower = 0;
float p, d, dd, previous_d;

// SPI
uint16_t SPIReceive = 0;

// Timing stuff
uint32_t now;
uint16_t frameTime;
uint32_t lastSwitchTime;
uint16_t switchIntvl = 200;
uint32_t startTime = 0; // oven starts
uint32_t ovenTime = 0; // now - startTime
uint32_t timeLeft = 0; // time Left until CureA finishes
uint32_t CureATotalTime = cookTime[cookArrSize - 1] * 1000; // total ms it takes to Cure A
uint8_t switchCounter = 0;

int freeRam(void) {
  extern unsigned int __heap_start;
  extern void *__brkval;

  int free_memory;
  int stack_here;

  if (__brkval == 0)
    free_memory = (int) &stack_here - (int) &__heap_start;
  else
    free_memory = (int) &stack_here - (int) __brkval;

  return (free_memory);
}


void updatePID() {
  // TODO: switch between two PID profiles according to whether temperature is increasing or decreasing
  float error = targetTemp - currentTemp;
  error_integral += ((targetTemp - currentTemp) * frameTime / 1000.0 / timeDilation) * kp_i;
  p = kp_p * error;
  d += ((error - previous_error) / frameTime * 1000.0 * timeDilation - d) * 0.2;
  dd = d - previous_d;
  previous_d = d;
  error_integral = constrain(error_integral, 0, 1);
  d = constrain(d, -2, 2);
  dd = constrain(dd, -1, 1);
  ovenPower += (p + error_integral + d * kp_d + dd * kp_dd - ovenPower) * filterFactor;
  ovenPower = constrain(ovenPower, 0, 1);
  previous_error = error;
  Serial.print("P: ");    Serial.print(p, 3);
  Serial.print(" I: ");   Serial.print(error_integral, 3);
  Serial.print(" D: ");   Serial.print(d * kp_d, 3);
  Serial.print(" DD: ");   Serial.print(dd * kp_dd, 3);
  Serial.print(" Out: "); Serial.println(ovenPower);
}


char textBuffer[8]; // buffer for text on canvas
uint8_t canvasTextbuffer[128 * 16 / 8];
NanoCanvas canvasText(128, 16, canvasTextbuffer);
void displayUpdate() { // print data on screen
  dtostrf(targetTemp, 6, 2, textBuffer);
  canvasText.printFixed(42, 8, textBuffer, STYLE_NORMAL);
  sec2Clock(timeLeft / 1000, textBuffer); 
  canvasText.printFixed(85, 8, textBuffer, STYLE_NORMAL);

  if (errorCode) {
    sprintf(textBuffer, "Err%03d", errorCode);
    canvasText.printFixed(0, 8, textBuffer, STYLE_NORMAL);
  }
  else {
    dtostrf(currentTemp, 6, 2, textBuffer);
    canvasText.printFixed(0, 8, textBuffer, STYLE_NORMAL);
  }
  
  canvasText.blt(0, 0);
  //Serial.println(freeRam());
}

void sec2Clock(uint32_t seconds, char str[]) {
  uint8_t hours = seconds / 3600;
  seconds -= 3600 * hours;
  uint8_t minutes = seconds / 60;
  seconds -= 60 * minutes;
  str[0] = '0' + hours;
  str[1] = ':';
  str[2] = '0' + minutes / 10;
  str[3] = '0' + minutes % 10;
  str[4] = ':';
  str[5] = '0' + seconds / 10;
  str[6] = '0' + seconds % 10;
  str[7] = '\0';
}

uint8_t pastLinePosition = 0;
uint8_t canvasPlotbuffer[128 * 48 / 8];
NanoCanvas canvasPlot(128, 48, canvasPlotbuffer); // plots progress
void displayPlot(bool force) {
  uint8_t currLinePosition = ovenTime * 127 / CureATotalTime;
  if ((currLinePosition == pastLinePosition) && (!force)) {
    return;
  }
  canvasPlot.clear();
  pastLinePosition = currLinePosition;
  canvasPlot.drawVLine(ovenTime * 127 / CureATotalTime, 0, 47);
  for (uint8_t i = 0; i < 128; i++) {
    uint8_t thisY = uint8_t(CureTemp(float(i) / 127 * cookTime[cookArrSize - 1] * 1000) / 4);
    canvasPlot.putPixel(i, 47 - thisY);
  }
  if (ovenStatus) {
    canvasPlot.printFixed(40, 32, "Oven Active!", STYLE_NORMAL);
  }
  canvasPlot.blt(0, 2);
}


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float CureTemp(uint32_t time) {
  float seconds = time / 1000.0;
  for (uint8_t i = 0; i < cookArrSize - 1; i++) {
    if (cookTime[i + 1] > seconds) {
      return mapFloat(seconds, cookTime[i], cookTime[i + 1], cookTemp[i], cookTemp[i + 1]);
    }
  }
  return 0;
}


float readTemperature(uint8_t csPin) { // MAX6675
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csPin, LOW);
  SPIReceive = SPI.transfer16(0);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  if (SPIReceive & 0b100) {
    Serial.println("NC!");
    return -1; // error: thermocouple disconnected
  }
  return float(SPIReceive >> 3) * 0.25;
}

void setup() {
  Serial.begin(9600);

  SPI.begin();
  
  // pins setup
  pinMode(SSR1_EN, OUTPUT);
  pinMode(SSR2_EN, OUTPUT);
  pinMode(CS_TEMP1, OUTPUT);
  pinMode(CS_TEMP2, OUTPUT);
  pinMode(CS_TEMP3, OUTPUT);
  digitalWrite(CS_TEMP1, HIGH);
  digitalWrite(CS_TEMP2, HIGH);
  digitalWrite(CS_TEMP3, HIGH);
  pinMode(BUTTON1, INPUT_PULLUP); // responsible for oven status
  pinMode(BUTTON2, INPUT_PULLUP); // unsure of use

  ssd1306_128x64_i2c_init();
  Wire.setClock(400000);
  ssd1306_clearScreen();
  ssd1306_setFixedFont(ssd1306xled_font6x8);
  canvasText.clear();
  canvasText.printFixed(0, 0, " Temp  Target   Time", STYLE_NORMAL);
  displayPlot(true);
  byte value = EEPROM.read(5);

  delay(300);
}

void loop() {
  
  // Update timing variables
  now = millis() * timeDilation; // track current time

  // check if oven should be working
  if (!digitalRead(BUTTON1)) { // flip ovenStatus when button is pressed
    tone(BUZZER, 524);
    delay(10);
    noTone(BUZZER);
    ovenStatus = !ovenStatus && !errorCode;
    displayPlot(true);
    while (!digitalRead(BUTTON1));
  }
  if (!ovenStatus) { // turn off, update starting temperature
    startTime = now - (60000.0 * currentTemp / 1.0); // adding an offset
    targetTemp = 0;
  }
  relayStatus = (now % uint16_t(500 * timeDilation)) < (ovenPower * 500 * timeDilation);
  digitalWrite(SSR1_EN, relayStatus);
  digitalWrite(SSR2_EN, relayStatus);


  if (now - lastSwitchTime >= switchIntvl * timeDilation) {
    frameTime = now - lastSwitchTime; // track change in time
    lastSwitchTime = now;
    switchCounter++;
    if (switchCounter > 200) { // flip all pixels once a while to prevent screen burnout
      ssd1306_positiveMode();
      displayPlot(true);
    }
    else {
      ssd1306_negativeMode();
      displayPlot(true);
    }
    
    //currentTempBuf[avgBufIdx] = (readTemperature(CS_TEMP1) + readTemperature(CS_TEMP2) + readTemperature(CS_TEMP3)) * 0.333333333333333333333333;
    float newTemperature = readTemperature(CS_TEMP1);
    errorCode = (errorCode & 0b11111101) + ((newTemperature == -1) << 1);
    currentTempBuf[avgBufIdx] = mapFloat(newTemperature, temp1_Zero, temp1_Hundred, 0, 100);
    avgBufIdx++;
    if (avgBufIdx >= avgBufLength) avgBufIdx = 0;
    currentTemp = 0;
    for(uint8_t i = 0; i < avgBufLength; i++) {
      currentTemp += currentTempBuf[i];
    }
    currentTemp /= avgBufLength;

    if (ovenStatus) {
      ovenTime = now - startTime; // time since oven turned on
      targetTemp = CureTemp(ovenTime); // finds ideal temp to be at
    }
    updatePID();
    fps += (1000.0 * timeDilation / frameTime - fps) * 0.1;
    if (CureATotalTime > ovenTime) {
      timeLeft = CureATotalTime - ovenTime;
    }
    else {
      timeLeft = 0;
    }
    displayUpdate();
    displayPlot(false); // update plot on display
  }
}
