/**
 * Final Project
 * Group 2
 * Members:
 * - Humberto Cortez
 * - Mason Chacon
 * - Evan Barnett
 * - Riley Padilla
 */

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <DHT.h>

// Pin Definitions

#define PIN_LED_BLUE   7    // PH4 D7. OUT
#define PIN_LED_GREEN  8    // PH5 D8. OUT
#define PIN_LED_YELLOW 9    // PH6 D9. OUT
#define PIN_LED_RED    10   // PB4 D10 OUT

#define PIN_BTN_START  2    // PE4 D2  IN
#define PIN_BTN_STOP   3    // PE5 D3  IN
#define PIN_BTN_CLEAR  4    // PG5 D4  IN

#define PIN_FAN 12          // PB6 D12
#define PIN_DHT11 22        // PA0 D22
#define PIN_WATER_SENSOR 0  // Analog 0, to be used with adcRead()
#define PIN_POTENTIOMETER 1 // Analog 1, to be used with adcRead()

#define PIN_CLK_SDA 20      // 
#define PIN_CLK_SCL 21      // 

#define PIN_DIS_RS 30       // D30
#define PIN_DIS_EN 31       // D31
#define PIN_DIS_D4 32       // D32
#define PIN_DIS_D5 33       // D33
#define PIN_DIS_D6 34       // D34
#define PIN_DIS_D7 35       // D35

#define STEPPER_STEPS 1024
#define PIN_STEPPER_IN1 42  // D42
#define PIN_STEPPER_IN2 43  // D43
#define PIN_STEPPER_IN3 44  // D44
#define PIN_STEPPER_IN4 45  // D45

// ---------- IDLE thresholds (tune these) ----------
#define WATER_LOW_THRESHOLD   200   // ADC units (0-1023). Pick based on your sensor test.
#define TEMP_HIGH_THRESHOLD_C 20.0  // temp threshold for RUNNING

#define LCD_UPDATE_INTERVAL 5000UL  // 5 seconds (was 60000)

#define DHTTYPE DHT11

unsigned long lastLCDUpdate = 0;
extern LiquidCrystal lcd;

// DHT object
DHT dht(PIN_DHT11, DHTTYPE);

// Prototypes
void initIdleStateHardware();
void handleIdleState();
void handleErrorState();
void handleRunningState();

bool isStopPressed();
bool isClearPressed();
bool isWaterLow();
float readTempC_stub();
float readHumidity_stub();
void printString(const char* str);
void uartLog(const char* msg);

enum SystemState {
    STATE_DISABLED,
    STATE_IDLE,
    STATE_ERROR,
    STATE_RUNNING
};

volatile SystemState currentState = STATE_DISABLED;
volatile bool startPressed = false;

void ISR_startButton() {
    startPressed = true; // ISR start button
}

void initDisabledStateHardware() {

    // Yellow LED = PIN_LED_YELLOW = D9 = PH6
    DDRH |= (1 << 6);    // Sets PH6 as OUTPUT
    PORTH |= (1 << 6);   // LED ON at startup (DISABLED)

    DDRE &= ~(1 << 4);   // PE4 input
    PORTE |= (1 << 4);   // Enable pull-up

    attachInterrupt(digitalPinToInterrupt(2), ISR_startButton, FALLING);
}

void handleDisabledState() {
    static bool firstEntry = true;

    if (firstEntry) {
        // YELLOW ON, all others OFF
        PORTH |=  (1 << 6);                        // Yellow ON
        PORTH &= ~((1 << 4) | (1 << 5));           // Blue & Green OFF
        PORTB &= ~(1 << 4);                        // Red OFF

        // Fan OFF
        PORTB &= ~(1 << 6);

        // LCD message
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("DISABLED");

        firstEntry = false;
    }

    // Only action in disabled state: start button
    if (startPressed) {
      startPressed = false;

      // Turns off yellow LED when leaving disabled
      PORTH &= ~(1 << 6);

      lastLCDUpdate = 0;        // <<< ADD THIS LINE (forces LCD update)

      firstEntry = true;        // allow re-init next time
      currentState = STATE_IDLE;
    }

}


void initIdleStateHardware() {
  // LEDs: Blue PH4(D7), Green PH5(D8), Yellow PH6(D9), Red PB4(D10)
  DDRH |= (1 << 4) | (1 << 5) | (1 << 6);
  DDRB |= (1 << 4);

  // Fan: PB6 (D12) output
  DDRB |= (1 << 6);

  // Stop button: PE5 (D3) input with pull-up
  DDRE &= ~(1 << 5);
  PORTE |= (1 << 5);

  // Clear button: PG5 (D4) input with pull-up
  DDRG &= ~(1 << 5);
  PORTG |= (1 << 5);
}

bool isStopPressed() {
  return ((PINE & (1 << 5)) == 0);
}

bool isClearPressed() {
  return ((PING & (1 << 5)) == 0);
}

bool isWaterLow() {
  unsigned int w = adcRead(PIN_WATER_SENSOR);
  return (w < WATER_LOW_THRESHOLD);
}

// Replaced stub logic with real DHT11 reads
float readTempC_stub() {
  float t = dht.readTemperature();
  return isnan(t) ? -100.0 : t;
}

float readHumidity_stub() {
  float h = dht.readHumidity();
  return isnan(h) ? -1.0 : h;
}

void uartLog(const char* msg) {
  printString(msg);
  printString("\n");
}

void handleIdleState() {
  static bool firstEntry = true;
  unsigned long now = millis();

  if (firstEntry) {
    initIdleStateHardware();
    uartLog("[STATE] ENTER IDLE");

    // GREEN ON, others OFF
    PORTH |=  (1 << 5);
    PORTH &= ~(1 << 4);
    PORTH &= ~(1 << 6);
    PORTB &= ~(1 << 4);

    // Fan OFF
    PORTB &= ~(1 << 6);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("IDLE");

    lastLCDUpdate = now;
    firstEntry = false;
  } 


  // STOP if detects stop button
  if (isStopPressed()) {
    uartLog("[EVENT] STOP -> DISABLED");
    PORTB &= ~(1 << 6);     // Fan OFF
    firstEntry = true;
    currentState = STATE_DISABLED;
    return;
  }

  // Water level monitored continuously
  if (isWaterLow()) {
    uartLog("[EVENT] WATER LOW -> ERROR");
    PORTB &= ~(1 << 6);     // Fan OFF
    firstEntry = true;
    currentState = STATE_ERROR;
    return;
  }

  // -------- CONTROL LOGIC (NOT tied to LCD timing) --------
  float t_now = readTempC_stub();
  if (t_now > TEMP_HIGH_THRESHOLD_C) {
    uartLog("[EVENT] TEMP HIGH -> RUNNING");
    firstEntry = true;
    currentState = STATE_RUNNING;
    return;
  }

  // -------- LCD DISPLAY (timed separately) --------
  if (now - lastLCDUpdate >= LCD_UPDATE_INTERVAL) {
    float h = readHumidity_stub();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T:");
    lcd.print(t_now, 1);
    lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.print("H:");
    lcd.print(h, 0);
    lcd.print("%");

    lastLCDUpdate = now;
  }
}


void handleRunningState() {
  static bool firstEntry = true;
  unsigned long now = millis();

  if (firstEntry) {
    // BLUE ON, all others OFF
    PORTH |=  (1 << 4);                        // Blue ON
    PORTH &= ~((1 << 5) | (1 << 6));           // Green, Yellow OFF
    PORTB &= ~(1 << 4);                        // Red OFF

    // Fan ON
    PORTB |=  (1 << 6);

    // Force LCD to update immediately on entry
    lastLCDUpdate = 0;

    firstEntry = false;
  }

  // Water level has highest priority
  if (isWaterLow()) {
    PORTB &= ~(1 << 6); // Fan OFF
    firstEntry = true;
    currentState = STATE_ERROR;
    return;
  }

  // Transition to IDLE as soon as temperature drops below threshold
  float t_now = readTempC_stub();
  if (t_now <= TEMP_HIGH_THRESHOLD_C) {
    PORTB &= ~(1 << 6); // Fan OFF
    firstEntry = true;
    currentState = STATE_IDLE;
    return;
  }

  // LCD shows Temperature + Water reading (timed)
  if (now - lastLCDUpdate >= LCD_UPDATE_INTERVAL) {
    unsigned int w = adcRead(PIN_WATER_SENSOR);   // raw ADC 0-1023

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T:");
    lcd.print(t_now, 1);
    lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.print("W:");
    lcd.print(w);

    lastLCDUpdate = now;
  }
}


void handleErrorState() {
  static bool firstEntry = true;

  if (firstEntry) {
    PORTH &= ~((1 << 4) | (1 << 5) | (1 << 6));
    PORTB |=  (1 << 4);
    PORTB &= ~(1 << 6);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ERROR");
    lcd.setCursor(0, 1);
    lcd.print("Low Water!");

    firstEntry = false;
  }

  if (isClearPressed() && !isWaterLow()) {
    firstEntry = true;
    currentState = STATE_IDLE;
  }
}

// UART Pointers
volatile unsigned char* pUCSR0A = (unsigned char*) 0x00C0;
volatile unsigned char* pUCSR0B = (unsigned char*) 0x00C1;
volatile unsigned char* pUCSR0C = (unsigned char*) 0x00C2;
volatile unsigned int*  pUBRR0  = (unsigned  int*) 0x00C4;
volatile unsigned char* pUDR0   = (unsigned char*) 0x00C6;

// ADC Pointers
volatile unsigned char* pADMUX    = (unsigned char*) 0x7C;
volatile unsigned char* pADCSRB   = (unsigned char*) 0x7B;
volatile unsigned char* pADCSRA   = (unsigned char*) 0x7A;
volatile unsigned int*  pADC_DATA = (unsigned int*) 0x78;

// GPIO Pointers
volatile unsigned char* pDDRB  = (unsigned char*) 0x24;
volatile unsigned char* pPORTB = (unsigned char*) 0x25;
volatile unsigned char* pDDRH  = (unsigned char*) 0x101;
volatile unsigned char* pPORTH = (unsigned char*) 0x102;

// Functions Declarations
void U0Init(int baud);
unsigned char U0KbHit();
unsigned char U0getChar();
void U0putChar(unsigned char ch);
void printString(const char* str);

// Main Program

LiquidCrystal lcd(PIN_DIS_RS, PIN_DIS_EN, PIN_DIS_D4, PIN_DIS_D5, PIN_DIS_D6, PIN_DIS_D7);
Stepper stepper(STEPPER_STEPS, PIN_STEPPER_IN1, PIN_STEPPER_IN3, PIN_STEPPER_IN2, PIN_STEPPER_IN4);

int stepperPrev = 0;

void setup(){
    initDisabledStateHardware();
    U0Init(9600);
    adcInit();
    lcd.begin(16, 2);
    dht.begin();
    stepper.setSpeed(26);
    stepperPrev = adcRead(PIN_POTENTIOMETER);

    printString("--- Swap Cooler Final Project ---\n");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.println("Final Project");
    lcd.println("Swap Cooler");
    delay(2000);
}

void loop(){
    switch (currentState) {
        case STATE_DISABLED: handleDisabledState(); return;
        case STATE_IDLE:     handleIdleState();     return;
        case STATE_ERROR:    handleErrorState();    return;
        case STATE_RUNNING:  handleRunningState();  return;
    }

    int potValue = adcRead(PIN_POTENTIOMETER);
    int stepVal = potValue - stepperPrev;
    if(stepVal != 0) stepper.step(stepVal);
    stepperPrev = potValue;
}

// ADC Functions

void adcInit(){
  *pADCSRA |= 0b10000000;
  *pADCSRA &= 0b11011111;
  *pADCSRA &= 0b11110111;
  *pADCSRA &= 0b11111000;
  *pADCSRA |= 0b00000111;
  
  *pADCSRB &= 0b11110111;
  *pADCSRB &= 0b11111000;
  
  *pADMUX &= 0b01111111;
  *pADMUX |= 0b01000000;
  *pADMUX &= 0b11011111;
  *pADMUX &= 0b11100000;
}

unsigned int adcRead(unsigned char channel){
  *pADMUX &= 0b11100000;
  *pADCSRB &= 0b11110111;
  *pADMUX |= (channel & 0x1F);
  *pADCSRA |= 0b01000000;

  while((*pADCSRA & 0x40) != 0);
  return (*pADC_DATA & 0x03FF);
}

// UART Functions

void U0Init(int baud){
    unsigned long FCPU = 16000000;
    unsigned int tbaud = (FCPU / 16 / baud - 1);

    *pUCSR0A = 0x20;
    *pUCSR0B = 0x18;
    *pUCSR0C = 0x06;
    *pUBRR0  = tbaud;
}

unsigned char U0KbHit(){
    return *pUCSR0A & 0x80;
}

unsigned char U0getChar(){
    return *pUDR0;
}

void U0putChar(unsigned char ch){
    while((*pUCSR0A & 0x20) == 0);
    *pUDR0 = ch;
}

void printString(const char* str){
    int i = 0;
    while(str[i] != '\0'){
        U0putChar(str[i]);
        i++;
    }
}
