#include <avdweb_AnalogReadFast.h> // use analogReadFast(adcPin);
#include <avr/pgmspace.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include "ssd1306.h"
#include "nano_gfx.h"

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
#define BUTTON1 2 // ovenStatus --> on/off
#define BUTTON2 7 // confirms selection
#define ESTOP 5
#define BUZZER 4

//settings
#define cookArrSize 10
#define avgBufLength 20
#define filterFactor 0.0005
#define timeDilation 1.0
#define temp1_Zero 0
#define temp2_Zero 0
#define temp3_Zero 0
#define temp1_Hundred 100
#define temp2_Hundred 100
#define temp3_Hundred 100

//PID settings
#define kp_p 0.5
#define kp_i 0.01
#define kp_d 10
#define kp_dd -3

/* EEPROM STORAGE VISUALIZATION                  
 _______________________________________________________________________________________
           0     2      4       6       8      10     12   14   16   18              20 ｜
_________________________________________________________________________________________
cookTime | 0 | 6420 | 10020 | 14220 | 21420 | 22956 | -1 | -1 | -1 | -1 | x-point |  9  |
_________________________________________________________________________________________
           22   24     26       28      30     32     34   36   38   40              42 ｜
_________________________________________________________________________________________
cookTemp | 0 | 107  | 107   | 177   | 177   |  49   | -1 | -1 | -1 | -1 | reserve |  ?  |
_________________________________________________________________________________________ location 100: 255 if not written to, else, 1

*/
// Status variables
float targetTemp; // target temperature
float currentTemp; // current temperature

bool relayStatus = false; // oven itself is on/off
bool ovenStatus = false; // modes: on/off vs adjust
bool adjustStatus = false; // user changed settings
// store text in "PROGMEM" Flash: // strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[i])));
const char menu[] PROGMEM = "MENU";
const char back[] PROGMEM = "BACK";
const char adjust[] PROGMEM = "ADJUST";
const char xPos[] PROGMEM = "X";
const char yPos[] PROGMEM = "Y";
const char filler[] PROGMEM = "FILLER";
const char time[] PROGMEM = "TIME";
char progBuffer[30];
const char *const menuItems[] PROGMEM = {menu, back, adjust, xPos, yPos, filler, time};
uint8_t errorCode = 0b00000000; // Bitmap for error conditions: 0 = ok, 1 = error
// |Reserved|Reserved|Reserved|Reserved|Sensor3|Sensor2|Sensor1|Estop|

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
uint32_t timeLeft = 0; // time Left until Cure finishes
uint32_t CureATotalTime; // default cure time
uint8_t switchCounter = 0;

// Canvas Stuff
uint8_t knobY = 0;  // current knob position for y on NanoCanvas
uint8_t currCanvas = 0; // 0 = Graph, 1 = Menu, 2 = Adjust, 3 = Filler
uint8_t pastLinePosition = 0;
uint8_t canvasPlotbuffer[128 * 48 / 8];
uint8_t canvasTextbuffer[128 * 16 / 8];
char textBuffer[8]; // buffer for text on canvas
NanoCanvas canvasPlot(128, 48, canvasPlotbuffer); // draws progress & menu/settings
NanoCanvas canvasText(128, 16, canvasTextbuffer); // draws data

uint16_t eepromRead16(uint8_t addr) {
  return EEPROM.read(addr) << 8 + EEPROM.read(addr + 1);
}
void eepromUpdate16(uint8_t addr, uint16_t value) {
  EEPROM.update(addr, value >> 8);
  EEPROM.update(addr + 1, value);
}

void eepromWrite16(uint8_t addr, uint16_t value) {
  EEPROM.write(addr, value >> 8);
  EEPROM.write(addr + 1, value);
}

void initializeEEPROMData(){
  // initialize cookTime (2 bytes per data)
  eepromWrite16(0, 0); 
  eepromWrite16(2, 6420);
  eepromWrite16(4, 10020);
  eepromWrite16(6, 14220);
  eepromWrite16(8, 21420);
  eepromWrite16(10, 22956);
  eepromWrite16(12, -1);
  eepromWrite16(14, -1);
  eepromWrite16(16, -1);
  eepromWrite16(18, -1);
  // initialize cookTemp (2 bytes per data)
  eepromWrite16(22, 0); 
  eepromWrite16(24, 107);
  eepromWrite16(26, 107);
  eepromWrite16(28, 177);
  eepromWrite16(30, 177);
  eepromWrite16(32, 49);
  eepromWrite16(34, -1);
  eepromWrite16(36, -1);
  eepromWrite16(48, -1);
  eepromWrite16(40, -1);

  eepromWrite16(20, 0);
  eepromWrite16(42, 0);
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
void updatePID() {
  float error = targetTemp - currentTemp;
  error_integral += ((targetTemp - currentTemp) * frameTime / 1000.0 / timeDilation) * kp_i;
  p = kp_p * error;
  d += ((error - previous_error) / frameTime * 1000.0 * timeDilation - d) * 0.2;
  dd = d - previous_d;
  previous_d = d;
  error_integral = constrain(error_integral, 0, 1);
  d = constrain(d, -2, 2);
  dd = constrain(dd, -1, 1);
  ovenPower += (p + error_integral + d * kp_d + dd * kp_dd - ovenPower) * 0.333;
  ovenPower = constrain(ovenPower, 0, 1);
  previous_error = error;
  // Serial.print("P: ");    Serial.print(p, 3);
  // Serial.print(" I: ");   Serial.print(error_integral, 3);
  // Serial.print(" D: ");   Serial.print(d * kp_d, 3);
  // Serial.print(" DD: ");   Serial.print(dd * kp_dd, 3);
  // Serial.print(" Out: "); Serial.println(ovenPower);
}
uint8_t getMinCookTime(){
  uint8_t point = eepromRead16(20);
  if(point == 0) return 0;
  else{ return eepromRead16(eepromRead16(20) - 1) + 1;}
  // when user adjusts x-value, find the minimum value they can set
}
void recalibrateSettings(){
  // reset variables
  previous_error = 0, error_integral = 0;
  p = 0, d = 0, dd = 0;
  previous_d = 0;
  ovenTime = 0;
  ovenPower = 0;
  
  // reset timeLeft & CureATotalTime
  uint8_t i = 0;
  while(i < 10){
    float val = eepromRead16(2*i);
    if (val == -1) break;
    CureATotalTime = val*1000;
  }
  timeLeft = CureATotalTime - ovenTime;
}
void displayMenu(uint8_t option){ 
  /* // 1 = BACK, 2 = ADJUST, 3 = FILLER
   ___________________________
  |             MENU          |
  |---------------------------|
  |            BACK <         |  
  |---------------------------|
  |            ADJUST         |
  |---------------------------|
  |            FILLER         |
   ---------------------------
  */
  canvasPlot.clear();
  canvasPlot.drawRect(2, 2, 126, 46);
  canvasPlot.drawRect(3, 3, 125, 13); 
  canvasPlot.drawRect(3, 14, 125, 24);
  canvasPlot.drawRect(3, 25, 125, 35);
  canvasPlot.drawRect(3, 36, 125, 45);
  strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[0])));
  canvasPlot.printFixed(56, 8, progBuffer, STYLE_ITALIC);

  if(option == 1){ // back
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[1])));
    canvasPlot.printFixed(56, 18, progBuffer, STYLE_BOLD);
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[2])));
    canvasPlot.printFixed(56, 29, progBuffer, STYLE_NORMAL);
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[5])));
    canvasPlot.printFixed(56, 40, progBuffer, STYLE_NORMAL);
  }
  else if(option == 2){ // adjust
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[1])));
    canvasPlot.printFixed(56, 18, progBuffer, STYLE_NORMAL);
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[2])));
    canvasPlot.printFixed(56, 29, progBuffer, STYLE_BOLD);
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[5])));
    canvasPlot.printFixed(56, 40, progBuffer, STYLE_NORMAL);
  }
  else if(option == 3){ // filler
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[1])));
    canvasPlot.printFixed(56, 18, progBuffer, STYLE_NORMAL);
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[2])));
    canvasPlot.printFixed(56, 29, progBuffer, STYLE_NORMAL);
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[5])));
    canvasPlot.printFixed(56, 40, progBuffer, STYLE_BOLD);
  }
  else{ /* ERROR */  }
  canvasPlot.blt(0, 2);
}
void displayAdjust(uint8_t option){
  /* // 1 = X, 2 = time, 3 = Y, 4 = BACK
  ___________________________________
  |               ADJUST              |
  |-----------------------------------|
  |   X              |  9  |          |
  |-----------------------------------|
  |   TIME           | 243 |          | --> find minimum time you can set
  |-----------------------------------|
  |   Y              | 243 |          |
  |-----------------------------------|
  |                BACK               |
  ------------------------------------
  */
  canvasPlot.clear();
  canvasPlot.drawRect(2, 2,  125, 10); // adjust
  canvasPlot.drawRect(3, 11, 125, 19); // x point value
  canvasPlot.drawRect(3, 20, 125, 28); // x time value
  canvasPlot.drawRect(3, 29, 125, 38); // y value
  canvasPlot.drawRect(3, 39, 125, 44); // back button
  strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[2]))); // prints label
  canvasPlot.printFixed(54, 6, progBuffer, STYLE_ITALIC);
  if(option == 1){ 
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[3]))); // prints x-pos
    canvasPlot.printFixed(6, 14, progBuffer, STYLE_BOLD);
    dtostrf(eepromRead16(20), 100, 18, textBuffer);
    canvasPlot.printFixed(60, 14, textBuffer, STYLE_NORMAL);

    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[6]))); // prints time
    canvasPlot.printFixed(6, 23, progBuffer, STYLE_NORMAL);
    dtostrf(eepromRead16(2*eepromRead16(20)), 100, 18, textBuffer);
    canvasPlot.printFixed(60, 23, textBuffer, STYLE_NORMAL);

    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[4])));
    canvasPlot.printFixed(6, 32, progBuffer, STYLE_NORMAL);
    dtostrf(eepromRead16(2*eepromRead16(20) + 22), 100, 29, textBuffer); 
    canvasPlot.printFixed(60, 32, textBuffer, STYLE_NORMAL);

    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[1])));
    canvasPlot.printFixed(6, 42, progBuffer, STYLE_NORMAL);
  }
  else if(option == 2){// make sure this list is sorted in increasing order
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[3]))); // prints xpos
    canvasPlot.printFixed(6, 14, progBuffer, STYLE_NORMAL);
    dtostrf(eepromRead16(20), 100, 18, textBuffer);
    canvasPlot.printFixed(60, 14, textBuffer, STYLE_NORMAL);

    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[6]))); // prints time
    canvasPlot.printFixed(6, 23, progBuffer, STYLE_BOLD);
    dtostrf(eepromRead16(2*eepromRead16(20)), 100, 18, textBuffer);
    canvasPlot.printFixed(60, 23, textBuffer, STYLE_NORMAL);

    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[4]))); // prints ypos
    canvasPlot.printFixed(6, 32, progBuffer, STYLE_NORMAL);
    dtostrf(eepromRead16(2*eepromRead16(20) + 22), 100, 29, textBuffer); 
    canvasPlot.printFixed(60, 32, textBuffer, STYLE_NORMAL);

    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[1]))); // prints back
    canvasPlot.printFixed(6, 42, progBuffer, STYLE_NORMAL);
  }
  else if(option == 3){
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[3])));
    Serial.println(progBuffer);
    canvasPlot.printFixed(6, 14, progBuffer, STYLE_NORMAL);
    dtostrf(eepromRead16(20), 100, 18, textBuffer);
    Serial.println(textBuffer);
    canvasPlot.printFixed(60, 14, textBuffer, STYLE_NORMAL);

    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[6])));
    canvasPlot.printFixed(6, 23, progBuffer, STYLE_NORMAL);
    dtostrf(eepromRead16(2*eepromRead16(20)), 100, 18, textBuffer);
    canvasPlot.printFixed(60, 23, textBuffer, STYLE_NORMAL);

    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[4])));
    canvasPlot.printFixed(6, 32, progBuffer, STYLE_BOLD);
    dtostrf(eepromRead16(2*eepromRead16(20) + 22), 100, 29, textBuffer); 
    canvasPlot.printFixed(60, 32, textBuffer, STYLE_NORMAL);

    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[1])));
    canvasPlot.printFixed(6, 42, progBuffer, STYLE_NORMAL);
  }
  else if(option == 4){
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[3])));
    canvasPlot.printFixed(6, 14, progBuffer, STYLE_NORMAL);
    dtostrf(eepromRead16(20), 100, 18, textBuffer);
    canvasPlot.printFixed(60, 14, textBuffer, STYLE_NORMAL);

    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[6])));
    canvasPlot.printFixed(6, 23, progBuffer, STYLE_NORMAL);
    dtostrf(eepromRead16(2*eepromRead16(20)), 100, 18, textBuffer);
    canvasPlot.printFixed(60, 23, textBuffer, STYLE_NORMAL);

    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[4])));
    canvasPlot.printFixed(6, 32, progBuffer, STYLE_NORMAL);
    dtostrf(eepromRead16(2*eepromRead16(20) + 22), 100, 29, textBuffer); 
    canvasPlot.printFixed(60, 32, textBuffer, STYLE_NORMAL);

    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[1])));
    canvasPlot.printFixed(6, 42, progBuffer, STYLE_BOLD);
  }
  canvasPlot.blt(0, 2);
}
void displayUpdate() { 
  /* displayUpdate(): Print data on screen
  ___________________
  | MENU              |
  |-------------------| 
  | |                 |
  | |     LINE        |
  | |_______________  |
  | TIME  TARG  LEFT  | <-- canvasText
  | 0111  0000  0000  | <-- canvasText
  -------------------
  */
  canvasText.clear();
  canvasText.printFixed(0, 0,  " Temp  Target   Time", STYLE_NORMAL);
  // canvasText.printFixed(0, 11, "       Temp    State", STYLE_NORMAL);
  dtostrf(currentTemp, 6, 2, textBuffer);
  canvasText.printFixed(0, 8, textBuffer, STYLE_NORMAL);
  if (errorCode) {
    sprintf(textBuffer, "Err%03d", errorCode);
    canvasText.printFixed(0, 8, textBuffer, STYLE_NORMAL);
  }
  else {
    dtostrf(currentTemp, 6, 2, textBuffer);
    canvasText.printFixed(0, 8, textBuffer, STYLE_NORMAL);
    dtostrf(targetTemp, 6, 2, textBuffer);
    canvasText.printFixed(42, 8, textBuffer, STYLE_NORMAL);
    sec2Clock(timeLeft / 1000, textBuffer); 
    canvasText.printFixed(85, 8, textBuffer, STYLE_NORMAL);
  }
  canvasText.blt(0, 0);
}

void displayPlot(bool force) {
  /* displayPlot()
  ___________________
  | MENU  <           | <-- canvasPlot: MENU BUTTON
  |-------------------  <-- canvasPlot
  | |                 | <-- canvasPlot
  | |                 | <-- canvasPlot
  | |     LINE        | <-- canvasPlot
  | ----------------  | <-- canvasPlot
  | TIME  TARG  LEFT  |
  | 0111  0000  0000  |
  -------------------
  */
  uint8_t currLinePosition = ovenTime * 127 / CureATotalTime;
  if ((currLinePosition == pastLinePosition) && (!force)) {
    return;
  }
  pastLinePosition = currLinePosition;
  canvasPlot.clear();
  canvasPlot.drawVLine(ovenTime * 127 / CureATotalTime, 0, 47); // line for total cookTime
  for (uint8_t i = 0; i < 128; i++) {
    uint8_t thisY = uint8_t(CureTemp((float(i) / 127) * CureATotalTime) / 4); // gets y-value 
    // uint8_t thisY = uint8_t( CureTemp(float(i) / 127 * cookTime[cookArrSize - 1] * 1000)  / 4);
    canvasPlot.putPixel(i, 47 - thisY);
  } // if (ovenStatus) { canvasPlot.printFixed(40, 32, "Oven Active!", STYLE_NORMAL); }
  canvasPlot.blt(0, 2);
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float CureTemp(uint32_t time) {
  float seconds = time / 1000.0;
  for (uint8_t i = 0; i < cookArrSize - 1; i++) {
    if (eepromRead16(2*i + 2) > seconds) {
      return mapFloat(seconds, eepromRead16(2*i), eepromRead16(2*i + 2), eepromRead16(i+22), eepromRead16(i+24));
    }
  } return 0;
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
  return mapFloat((float(SPIReceive >> 3) * 0.25), 0, 100, temp1_Zero, temp1_Hundred);
}
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
void setup() {
  Serial.begin(9600);
  SPI.begin();
  Wire.setClock(400000);
  
  pinMode(SSR1_EN, OUTPUT);
  pinMode(SSR2_EN, OUTPUT);

  pinMode(CS_TEMP1, OUTPUT);
  pinMode(CS_TEMP2, OUTPUT);
  pinMode(CS_TEMP3, OUTPUT);
  digitalWrite(CS_TEMP1, HIGH);
  digitalWrite(CS_TEMP2, HIGH);
  digitalWrite(CS_TEMP3, HIGH);

  pinMode(BUTTON1, INPUT_PULLUP); // oven status
  pinMode(BUTTON2, INPUT_PULLUP); // confirm choice
  
  ssd1306_128x64_i2c_init();
  ssd1306_clearScreen();
  ssd1306_setFixedFont(ssd1306xled_font6x8);
  currCanvas = 0; // defaults to graph
  canvasText.clear();
  canvasPlot.clear();
  displayPlot(true);
  CureATotalTime = 22956 * 1000; // ms for Cure A
  delay(300);
}
void loop() {
  // Update timing variables
  Serial.print("knobY: "); Serial.println(knobY);
  now = millis() * timeDilation;

  // Update EEPROM initially
  if (eepromRead16(100) == 255){ // will execute when entering 1st loop()
    Serial.println("to initialize eeprom");
    initializeEEPROMData();
    eepromUpdate16(100, 0);
  }
  // check if oven should be working
  if (!digitalRead(BUTTON1)) { // flip ovenStatus when button is pressed
    tone(BUZZER, 524);
    delay(10);
    noTone(BUZZER);
    ovenStatus = !ovenStatus && !errorCode;
    displayPlot(true);
    while (!digitalRead(BUTTON1));
  }
  if (!ovenStatus) { // turn off, update starting temperature & time
    startTime = now - (60000.0 * currentTemp / 1.0); // offset
    targetTemp = 0;
  }

  relayStatus = (now % uint16_t(500 * timeDilation)) < (ovenPower * 500 * timeDilation);
  digitalWrite(SSR1_EN, relayStatus);
  digitalWrite(SSR2_EN, relayStatus);
  if (now - lastSwitchTime >= switchIntvl * timeDilation) {
    frameTime = now - lastSwitchTime; // change in time
    lastSwitchTime = now;
    switchCounter++;
    if (switchCounter > 200) { // prevent screen burnout
      ssd1306_positiveMode();
      // displayPlot(true);
    }
    else {
      ssd1306_negativeMode();
      // displayPlot(true);
    }

    // temperature updates
    currentTempBuf[avgBufIdx] = readTemperature(CS_TEMP1);
    //currentTempBuf[avgBufIdx] = (readTemperature(CS_TEMP1) + readTemperature(CS_TEMP2) + readTemperature(CS_TEMP3)) * 0.333333333333333333333333;
    float newTemperature = readTemperature(CS_TEMP1);
    errorCode = (errorCode & 0b11111101) + ((newTemperature == -1) << 1);
    currentTempBuf[avgBufIdx] = mapFloat(newTemperature, temp1_Zero, temp1_Hundred, 0, 100);

    avgBufIdx++;
    if (avgBufIdx >= avgBufLength) avgBufIdx = 0;
    currentTemp = 0;
    for(uint8_t i = 0; i < avgBufLength; i++) {
      currentTemp += currentTempBuf[i] / avgBufLength;
    }
    if (ovenStatus) {
      ovenTime = now - startTime; // time since oven turned on
      targetTemp = CureTemp(ovenTime); // finds ideal temp to be at
    }
    updatePID();
    fps += (1000.0 * timeDilation / frameTime - fps) * 0.1;
    if (CureATotalTime > ovenTime) {
      timeLeft = CureATotalTime - ovenTime;
    }
    else { timeLeft = 0; }
    // displayUpdate(); displayPlot(false); // update plot on display
  }

  // canvas state updates
  if(currCanvas == 0){ // graph
    displayUpdate();
    displayPlot(true); // update plot
      if(!digitalRead(BUTTON2)){ // menu button
        currCanvas = 1;
        while (!digitalRead(BUTTON2));
      }
  }
  else if(currCanvas == 1){ // menu
      knobY = analogRead(KNOB2) / 343; // [0-2]
      displayMenu(knobY+1); // defaults to 0 = back button
      if(!digitalRead(BUTTON2) && knobY == 0){ /* back button*/ currCanvas = 0; while (!digitalRead(BUTTON2));  }
      else if(!digitalRead(BUTTON2) && knobY == 1){ /* adjust button*/ currCanvas = 2; while (!digitalRead(BUTTON2));}
      else{ /* filler */ }
  }
  else if (currCanvas == 2){ // adjust
    adjustStatus = true;
    knobY = analogRead(KNOB2) / 256; // [0-1023] ~ [0, 1, 2, 3]

    if(knobY == 0){ // adjust_x
      displayAdjust(1);
      uint8_t kX = analogRead(KNOB1) / 103; // [0-1023] ~ [0-9]
      eepromUpdate16(20, kX); // x position
      if(eepromRead16(kX*2) == -1){ 
        eepromUpdate16(kX*2, getMinCookTime());
        eepromUpdate16(kX*2+22, 0);
      }
    }
    else if(knobY == 1){ // cookTime
      displayAdjust(2);
      Serial.println("exited displayAdjust(2)");
      float divisor = 1023.0 / (10000.0 - getMinCookTime());
      // Serial.print("Divisor = "); Serial.println(divisor);
      uint8_t kX = getMinCookTime() + floor(analogRead(KNOB1) / divisor); // [0-1023] ~ [min-10k]
      // Serial.print("kX = "); Serial.println(kX);
      eepromUpdate16(eepromRead16(20)*2, kX);
      // Serial.print("Updated cookTime");

    }
    else if(knobY == 2){ // yPos
      displayAdjust(3);
      // Serial.println("exited displayAdjust(3)");
      uint8_t kX = floor(analogRead(KNOB1) / 5.2); // maps [0-1023] ~ [0-200]
      // Serial.print("kX read: "); Serial.println(kx);
      eepromUpdate16(eepromRead16(eepromRead16(20)+22), kX);
      // Serial.print("kX read: "); Serial.println(kx);
    }
    else if (knobY == 3){ // back button
      displayAdjust(4);
      if(!digitalRead(BUTTON2)){ 
        currCanvas = 1;
        while (!digitalRead(BUTTON2));
      }
    }
    else { /* ERROR */ }
    // recalibration updates
    if(adjustStatus == true){
      recalibrateSettings();
      adjustStatus = false;
    }
  }
}