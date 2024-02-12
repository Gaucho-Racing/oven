// #include <avdweb_AnalogReadFast.h> // use analogReadFast(adcPin);
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
#define KNOB1 A6 // adjusting phase x
#define KNOB2 A7 // adjusting temp y
#define CS_TEMP1 10
#define CS_TEMP2 9
#define CS_TEMP3 8
#define BUTTON1 2 // ovenStatus --> on/off
#define BUTTON2 7 // confirms selection
#define ESTOP 5
#define BUZZER 4

//settings
#define cookArrSize 10
#define avgBufLength 10
// #define filterFactor 0.0005
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

Location 100: 255 if not written to, else, 1               
 ________________________________________________________________________________
|           0     2      4       6       8      10     12   14   16   18   20    |
|________________________________________________________________________________|
|cookTime | 0 | 6420 | 10020 | 14220 | 21420 | 22956 | -1 | -1 | -1 | -1 | lim   |
|________________________________________________________________________________|
|           22   24     26       28      30     32     34   36   38   40 | 100   |
|________________________________________________________________________________|
|cookTemp | 0 | 107  | 107   | 177   | 177   |  49   | -1 | -1 | -1 | -1 |   0   |
|________________________________________________________________________________|
*/

// Status variables
float targetTemp; // target temperature
float currentTemp; // current temperature
bool relayStatus = false; // oven itself is on/off
bool ovenStatus = false; // modes: on/off vs adjust
bool adjustStatus = false; // user changed settings
bool addedPoint = false; // maybe will use? user can only add 1 point at a time, 
                            // so they don't twist the knob and suddenly go to point 10?

char progBuffer[30];
const char menu[] PROGMEM = "MENU";
const char back[] PROGMEM = "BACK";
const char adjust[] PROGMEM = "ADJUST";
const char xPos[] PROGMEM = "X";
const char yPos[] PROGMEM = "TEMP";
const char reset_default[] PROGMEM = "RESET TO DEFAULT";
const char time[] PROGMEM = "TIME";
const char *const menuItems[] PROGMEM = {menu, back, adjust, xPos, yPos, reset_default, time};
uint16_t x_position = 0; // default value of the x-position in adjust mode

// |Reserved|Reserved|Reserved|Reserved|Sensor3|Sensor2|Sensor1|Estop|
uint8_t errorCode = 0b00000000; // Bitmap for error conditions: 0 = ok, 1 = error
uint8_t currCookArrSize = 6;
uint8_t avgBufIdx = 0;
float ovenPower = 0;
float currentTempBuf[avgBufLength]; // running average
float fps, previous_error = 0, error_integral = 0, p, d, dd, previous_d; // PID algorithm

// temporary vars changed b4 adjustStatus = true executes statement
uint16_t tempCookTime[] = {0, 6420, 10020, 14220, 21420, 22956, 0, 0, 0, 0}; 
uint16_t tempCookTemp[] = {0, 107,  107,   177,   177, 49, 0, 0, 0, 0}; // degrees Celcius

// SPI
uint16_t SPIReceive = 0;

// Timing stuff
uint8_t switchCounter = 0;

uint16_t frameTime;
uint16_t switchIntvl = 200;

uint32_t now;
uint32_t lastSwitchTime;
uint32_t startTime = 0; // oven starts
uint32_t ovenTime = 0; // now - startTime
uint32_t timeLeft = 0; // time Left until Cure finishes
uint32_t CureATotalTime = 0; // default cure time

// Canvas Stuff
bool screenMode = false;
char textBuffer[8]; // buffer for text on canvas
uint8_t currCanvas; // 0 = Graph, 1 = Menu, 2 = Adjust, 3 = Filler
uint8_t pastLinePosition = 0;
uint8_t canvasPlotbuffer[128 * 48 / 8];
uint8_t canvasTextbuffer[128 * 16 / 8];
uint16_t knobY = 0;
NanoCanvas canvasPlot(128, 48, canvasPlotbuffer); // draws progress & menu/settings
NanoCanvas canvasText(128, 16, canvasTextbuffer); // draws data (Time, Target, Temp)

/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

// EEPROM methods
uint16_t eepromRead16(uint8_t addr) {
  return (uint16_t(EEPROM.read(addr)) << 8) + EEPROM.read(addr + 1);
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
  // enters data into EEPROM when there is nothing in it --> very initial memory
  eepromWrite16(0, 0); 
  eepromWrite16(2, 6420);
  eepromWrite16(4, 10020);
  eepromWrite16(6, 14220);
  eepromWrite16(8, 21420);
  eepromWrite16(10, 22956);
  eepromWrite16(12, 0xffff);
  eepromWrite16(14, 0xffff);
  eepromWrite16(16, 0xffff);
  eepromWrite16(18, 0xffff);
  // initialize cookTemp (2 bytes per data)
  eepromWrite16(22, 0); 
  eepromWrite16(24, 107);
  eepromWrite16(26, 107);
  eepromWrite16(28, 177);
  eepromWrite16(30, 177);
  eepromWrite16(32, 49);
  eepromWrite16(34, 0xffff);
  eepromWrite16(36, 0xffff);
  eepromWrite16(38, 0xffff);
  eepromWrite16(40, 0xffff);

  // initializes reserved spots to 0
  eepromWrite16(20, 22956);
  eepromWrite16(42, 0);
}
// Calculation methods
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
  // Serial.print(F("P: "));    Serial.print(p, 3);
  // Serial.print(F(" I: "));   Serial.print(error_integral, 3);
  // Serial.print(F(" D: "));   Serial.print(d * kp_d, 3);
  // Serial.print(F(" DD: "));   Serial.print(dd * kp_dd, 3);
  // Serial.print(F(" Out: ")); Serial.println(ovenPower);
}
void recalibrateArrayfromEEPROM(){ // EEPROM --> ARRAY for saving adjustments
    for(uint8_t i = 0; i < 10; i++){
      tempCookTime[i] = eepromRead16(i*2);
      tempCookTemp[i] = eepromRead16((i*2)+22);
    }
}
void recalibrateEEPROMfromArray(){ // ARRAY --> EEPROM for displayUpdate()
    for(uint8_t i = 0; i < 10; i++){
      eepromWrite16(i*2, tempCookTime[i]);
      eepromUpdate16((i*2)+22, tempCookTemp[i]);
    }
}
void recalibrateSettings(){ // reset variables
  // previous_error = 0, error_integral = 0;
  // p = 0, d = 0, dd = 0; previous_d = 0; 
  // ovenTime = 0; ovenPower = 0;
  // uint8_t i = 0; // reset timeLeft & CureATotalTime
  recalibrateArrayfromEEPROM();
  CureATotalTime = uint32_t(eepromRead16(2*(currCookArrSize-1))) * 1000;
  timeLeft = CureATotalTime - ovenTime;
}
// Display methods
void displayMenu(uint8_t option){ // 1 = BACK, 2 = ADJUST, 3 = FILLER
  /* 
   __________________________
  |            MENU          |
  |--------------------------|
  |            BACK          |  
  |--------------------------|
  |           ADJUST         |
  |--------------------------|
  |            RESET         |
   --------------------------
  */
  canvasPlot.clear();
  canvasPlot.drawRect(3, 2, 125, 46);
  canvasPlot.drawHLine(3, 13, 125);
  canvasPlot.drawHLine(3, 25, 125);
  canvasPlot.drawHLine(3, 36, 125);
  strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[0])));
  canvasPlot.printFixed(56, 4, progBuffer, STYLE_ITALIC);

  if(option == 1){ // back
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[1])));
    canvasPlot.printFixed(56, 16, progBuffer, STYLE_BOLD);
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[2])));
    canvasPlot.printFixed(56, 27, progBuffer, STYLE_NORMAL);
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[5])));
    canvasPlot.printFixed(15, 38, progBuffer, STYLE_NORMAL);
  }
  else if(option == 2){ // adjust
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[1])));
    canvasPlot.printFixed(56, 16, progBuffer, STYLE_NORMAL);
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[2])));
    canvasPlot.printFixed(56, 27, progBuffer, STYLE_BOLD);
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[5])));
    canvasPlot.printFixed(15, 38, progBuffer, STYLE_NORMAL);
  }
  else if(option == 3){ // reset to default
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[1])));
    canvasPlot.printFixed(56, 16, progBuffer, STYLE_NORMAL);
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[2])));
    canvasPlot.printFixed(56, 27, progBuffer, STYLE_NORMAL);
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[5])));
    canvasPlot.printFixed(15, 38, progBuffer, STYLE_BOLD);
  }
  else{ /* ERROR */  }
  canvasPlot.blt(0, 2);
}
void displayAdjust(uint8_t option){ // 1 = X, 2 = time, 3 = Y, 4 = BACK
  /* 
  ________________________________
  |   X              |  9  |     |
  |------------------------------|
  |   TIME           | 243 |     |
  |------------------------------|
  |   Y              | 243 |     |
  |------------------------------|
  |   BACK                       |
  |______________________________|
  */
  canvasPlot.clear();
  canvasPlot.drawRect(3, 2, 125, 46);
  canvasPlot.drawHLine(3, 13, 125);
  canvasPlot.drawHLine(3, 25, 125);
  canvasPlot.drawHLine(3, 36, 125);
  recalibrateEEPROMfromArray();
  if(option == 1){ 
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[3]))); // prints x-pos
    canvasPlot.printFixed(6, 4, progBuffer, STYLE_BOLD);
    dtostrf(x_position, 6, 0, textBuffer);
    canvasPlot.printFixed(60, 4, textBuffer, STYLE_NORMAL);

    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[6]))); // prints time
    canvasPlot.printFixed(6, 16, progBuffer, STYLE_NORMAL);
    dtostrf(tempCookTime[x_position], 6, 0, textBuffer);
    canvasPlot.printFixed(60, 16, textBuffer, STYLE_NORMAL);

    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[4]))); // prints y-temp
    canvasPlot.printFixed(6, 27, progBuffer, STYLE_NORMAL);
    dtostrf(tempCookTemp[x_position], 6, 0, textBuffer); 
    canvasPlot.printFixed(60, 27, textBuffer, STYLE_NORMAL);

    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[1]))); // back
    canvasPlot.printFixed(6, 38, progBuffer, STYLE_NORMAL);
  }
  else if(option == 2){// make sure this list is sorted in increasing order
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[3]))); // prints xpos
    canvasPlot.printFixed(6, 4, progBuffer, STYLE_NORMAL);
    dtostrf(x_position, 6, 0, textBuffer);
    canvasPlot.printFixed(60, 4, textBuffer, STYLE_NORMAL);

    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[6]))); // prints time
    canvasPlot.printFixed(6, 16, progBuffer, STYLE_BOLD);
    dtostrf(tempCookTime[x_position], 6, 0, textBuffer);
    canvasPlot.printFixed(60, 16, textBuffer, STYLE_NORMAL);

    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[4]))); // prints ypos
    canvasPlot.printFixed(6, 27, progBuffer, STYLE_NORMAL);
    dtostrf(tempCookTemp[x_position], 6, 0, textBuffer); 
    canvasPlot.printFixed(60, 27, textBuffer, STYLE_NORMAL);

    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[1]))); // prints back
    canvasPlot.printFixed(6, 38, progBuffer, STYLE_NORMAL);
  }
  else if(option == 3){
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[3])));
    canvasPlot.printFixed(6, 4, progBuffer, STYLE_NORMAL);
    dtostrf(x_position, 6, 0, textBuffer);
    canvasPlot.printFixed(60, 4, textBuffer, STYLE_NORMAL);

    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[6])));
    canvasPlot.printFixed(6, 16, progBuffer, STYLE_NORMAL);
    dtostrf(tempCookTime[x_position], 6, 0, textBuffer);
    canvasPlot.printFixed(60, 16, textBuffer, STYLE_NORMAL);

    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[4])));
    canvasPlot.printFixed(6, 27, progBuffer, STYLE_BOLD);
    dtostrf(tempCookTemp[x_position], 6, 0, textBuffer); 
    canvasPlot.printFixed(60, 27, textBuffer, STYLE_NORMAL);

    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[1])));
    canvasPlot.printFixed(6, 38, progBuffer, STYLE_NORMAL);
  }
  else if(option == 4){
    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[3])));
    canvasPlot.printFixed(6, 4, progBuffer, STYLE_NORMAL);
    dtostrf(x_position, 6, 0, textBuffer);
    canvasPlot.printFixed(60, 4, textBuffer, STYLE_NORMAL);

    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[6])));
    canvasPlot.printFixed(6, 16, progBuffer, STYLE_NORMAL);
    dtostrf(tempCookTime[x_position], 6, 0, textBuffer);
    canvasPlot.printFixed(60, 16, textBuffer, STYLE_NORMAL);

    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[4])));
    canvasPlot.printFixed(6, 27, progBuffer, STYLE_NORMAL);
    dtostrf(tempCookTemp[x_position], 6, 0, textBuffer); 
    canvasPlot.printFixed(60, 27, textBuffer, STYLE_NORMAL);

    strcpy_P(progBuffer, (char *)pgm_read_ptr(&(menuItems[1])));
    canvasPlot.printFixed(6, 38, progBuffer, STYLE_BOLD);
  }
  canvasPlot.blt(0, 2);
}
void displayUpdate() { // Print data on screen
  /* 
 ____________________ 
| TIME  TARG  LEFT  | <-- canvasText
| 0111  0000  0000  | <-- canvasText
| |                 |
| |                 |
| |      LINE       |
| |_______________  |
|-------------------|
  */
  canvasText.clear();
  canvasText.printFixed(0, 1,  " Temp   Target  Time", STYLE_NORMAL);
  dtostrf(currentTemp, 6, 2, textBuffer);
  canvasText.printFixed(0, 9, textBuffer, STYLE_NORMAL);
  if (errorCode) {
    sprintf(textBuffer, "Err%03d", errorCode);
    canvasText.printFixed(0, 9, textBuffer, STYLE_NORMAL);
  }
  else {
    dtostrf(currentTemp, 6, 2, textBuffer);
    canvasText.printFixed(0, 9, textBuffer, STYLE_NORMAL);
    dtostrf(targetTemp, 6, 2, textBuffer);
    canvasText.printFixed(42, 9, textBuffer, STYLE_NORMAL);
    sec2Clock(timeLeft / 1000, textBuffer); 
    canvasText.printFixed(85, 9, textBuffer, STYLE_NORMAL);
  }
  canvasText.blt(0, 0);
}
void displayPlot(bool force) { // Prints canvas plot
  /* 
  ___________________
  | MENU  <           | <-- canvasPlot: MENU BUTTON
  |-------------------  <-- canvasPlot
  | TIME  TARG  LEFT  |
  | 0111  0000  0000  |
  | |                 | <-- canvasPlot
  | |                 | <-- canvasPlot
  | |     LINE        | <-- canvasPlot
  | ----------------  | <-- canvasPlot
  -------------------
  */
  uint8_t currLinePosition = ovenTime * 127 / CureATotalTime;
  // if ((currLinePosition == pastLinePosition) && (!force)) {
  //   return;
  // }
  pastLinePosition = currLinePosition;
  canvasPlot.clear();
  if (ovenStatus) canvasPlot.drawVLine(ovenTime * 127 / CureATotalTime, 0, 47); // line for total cookTime
  for (uint8_t i = 0; i < 128; i++) {
    uint8_t thisY = uint8_t(CureTemp(float(i) / 127 * CureATotalTime) / 4); // gets y-value 
    // original: uint8_t thisY = uint8_t( CureTemp(float(i) / 127 * cookTime[cookArrSize - 1] * 1000)  / 4);
    canvasPlot.putPixel(i, 47 - thisY);
  }
  canvasPlot.blt(0, 2);
}
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) { // map float
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
uint16_t map16(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) { // map uint16_t
  return (uint16_t)(1.0 * (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}
float CureTemp(uint32_t time) { // possible error: calling mapFloat but passing in uint16_t variables
  float seconds = time / 1000.0;
  for (uint8_t i = 0; i < cookArrSize - 1; i++) {
    if (eepromRead16(i*2 + 2) > seconds) {
      return mapFloat(seconds, eepromRead16(i*2), eepromRead16(i*2 + 2), eepromRead16(i*2 + 22), eepromRead16(i*2 + 24));
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
    Serial.println(F("NC!"));
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
  canvasText.clear();
  canvasPlot.clear();
  displayPlot(true);
  currCanvas = 0; // defaults to graph
  CureATotalTime = 22956000; // ms for Cure A

  if (eepromRead16(100) != 0){ // will execute when entering 1st loop()
    initializeEEPROMData();
    eepromUpdate16(100, 0);
  }

  delay(300);
}
void loop() {
  // Update timing variables
  now = millis() * timeDilation;
  if(errorCode){
    Serial.println("ERROR!");
    tone(BUZZER, 400); // lol
    delay(5000);
    noTone(BUZZER);
  }
  // check if oven should be working
  if (!digitalRead(BUTTON1)) { // flip ovenStatus when button is pressed
    tone(BUZZER, 524);
    delay(50); 
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
    // prevent screen burnout 
    if (switchCounter > 200) { ssd1306_positiveMode(); /* displayPlot(true); */ }
    else { ssd1306_negativeMode(); /* displayPlot(true); */ }

    // temperature updates
    currentTempBuf[avgBufIdx] = readTemperature(CS_TEMP1);
    float newTemperature1 = readTemperature(CS_TEMP1);
    float newTemperature2 = readTemperature(CS_TEMP2);
    float newTemperature3 = readTemperature(CS_TEMP3);
    errorCode = (errorCode & 0b11111101) + ((newTemperature1 == -1) << 1);
    errorCode = (errorCode & 0b11111011) + ((newTemperature2 == -1) << 2);
    errorCode = (errorCode & 0b11110111) + ((newTemperature3 == -1) << 3);
    currentTempBuf[avgBufIdx] = 0;
    currentTempBuf[avgBufIdx] += mapFloat(newTemperature1, temp1_Zero, temp1_Hundred, 0, 100) / 3;
    currentTempBuf[avgBufIdx] += mapFloat(newTemperature2, temp2_Zero, temp2_Hundred, 0, 100) / 3;
    currentTempBuf[avgBufIdx] += mapFloat(newTemperature3, temp3_Zero, temp3_Hundred, 0, 100) / 3;

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
  // CANVAS state updates
  if(currCanvas == 0){ // graph
    displayPlot(true); // update plot
      if(!digitalRead(BUTTON2)){ // menu button
        currCanvas = 1;
        while (!digitalRead(BUTTON2));
      }
  }
  else if(currCanvas == 1){ // menu
      knobY = map16(analogRead(KNOB2), 0, 1023, 0, 2); // range: [0-2]
      displayMenu(knobY+1); // defaults to 0 = back button
      if(!digitalRead(BUTTON2) && knobY == 0){ // back button*
        currCanvas = 0; 
        recalibrateArrayfromEEPROM();
        while (!digitalRead(BUTTON2)); 
      }
      else if(!digitalRead(BUTTON2) && knobY == 1){ /* adjust button*/ currCanvas = 2; while (!digitalRead(BUTTON2)); }
      else if (!digitalRead(BUTTON2) && knobY == 2){  // reset button
        initializeEEPROMData();
        currCookArrSize = 6;
        recalibrateSettings(); 
        while (!digitalRead(BUTTON2));
        currCanvas = 0;
      }
  }
  else if (currCanvas == 2){ // adjust --------------------------------> ISSUE: when temp set to 200, and knobY spins --> Canvas Starts Glitching
    knobY = map16(analogRead(KNOB2), 0, 1023, 0, 3); // [0, 1, 2, 3]
    if(knobY == 0){ // adjust x-position 
      displayAdjust(1);
      uint16_t reading = analogRead(KNOB1); // this one
      uint16_t newX = map16(reading, 0, 1023, 0, currCookArrSize); // range: [0-maxIndex+1]
      
      if(newX == currCookArrSize){  // initialize cookTime to be minimum cookTime + 1
        if(addedPoint == false){
          x_position = newX;
          currCookArrSize = (currCookArrSize + 1 > cookArrSize) ? currCookArrSize : currCookArrSize + 1;
          uint16_t nextTime = eepromRead16((x_position-1)*2)+1;
          tempCookTime[x_position] = nextTime;
          tempCookTemp[x_position] = 0;
          eepromUpdate16(x_position*2, tempCookTime[x_position]);
          eepromUpdate16(x_position*2+22, tempCookTemp[x_position]);
          addedPoint = true;
        }
        else{ x_position = newX-1; }
      }
      else{
        x_position = newX;
      }
      recalibrateArrayfromEEPROM();
    }
    else if(knobY == 1){ // cookTime
      displayAdjust(2);
      uint16_t reading = analogRead(KNOB1);
      uint16_t cookTime = map16(reading, 0, 1023, eepromRead16((x_position-1)*2)+1, 0xfffe); // range: [minCookTime()-10000]
      tempCookTime[x_position] = cookTime;
      if(!digitalRead(BUTTON2)){ // confirms time that for position knob
         adjustStatus = true;
         eepromUpdate16(x_position*2, cookTime);
        while(!digitalRead(BUTTON2));
        recalibrateArrayfromEEPROM();
      }
    }
    else if(knobY == 2){ // yPos
      displayAdjust(3);
      uint16_t reading = analogRead(KNOB1);
      uint16_t temp = map(reading, 0, 1023, 0, 200); // range: [0-200]
      tempCookTemp[x_position] = temp;
      if(!digitalRead(BUTTON2)){ // confirms time that for position knob
        adjustStatus = true;
        eepromUpdate16(eepromRead16(x_position*2 +22), temp);
        while(!digitalRead(BUTTON2));
        recalibrateArrayfromEEPROM();
      }
    }
    else if (knobY == 3){ // back button
      displayAdjust(4);
      if(!digitalRead(BUTTON2)){ 
        currCanvas = 1;
        while (!digitalRead(BUTTON2));
        recalibrateArrayfromEEPROM();
      }
    } else { /* ERROR */ }
  } else{ /* nothing happens*/ }

  if(adjustStatus == true){ // recalibration for updates if there were any
    recalibrateSettings();
    adjustStatus = false;
    if(addedPoint == true) addedPoint = false;
  }
  displayUpdate();
}