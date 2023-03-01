#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include "../include/knob.hh" //for mac users
//#include "..\include\knob.hh" //for windows users (chumps)
#include <ES_CAN.h>
// mutex to handle synchronization bug
SemaphoreHandle_t keyArrayMutex;

// Pin definitions
// Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

// Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

// Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

// Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

// Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

// Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//  store reading from each row.
volatile uint8_t keyArray[7];
const uint32_t stepSizes[] = {50953930, 54077542, 57396381, 60715219, 64229283, 68133799, 72233540, 76528508, 81018701, 85899345, 90975216, 96441538};

// volatile to allow for concurrency
volatile uint32_t currentStepSize;

const char *notes[12] = {"C", "C#", "D", "Eb", "E", "F", "F#", "G", "G#", "A", "Bb", "B"};
volatile uint8_t note;

// Global knobs
Knob knob0, knob1, knob2, knob3;

// CAN Stuff
volatile uint8_t TX_Message[8] = {0};

// Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value)
{
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN, value);
  digitalWrite(REN_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN, LOW);
}

// Read column bits
uint8_t readCols()
{
  uint8_t c0 = digitalRead(C0_PIN);
  uint8_t c1 = digitalRead(C1_PIN);
  uint8_t c2 = digitalRead(C2_PIN);
  uint8_t c3 = digitalRead(C3_PIN);

  return (c3 << 3) + (c2 << 2) + (c1 << 1) + c0;
}

// Set row bits
void setRow(uint8_t rowIdx)
{
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, rowIdx & 1);
  digitalWrite(RA1_PIN, rowIdx & 2);
  digitalWrite(RA2_PIN, rowIdx & 4);
  digitalWrite(REN_PIN, HIGH);
}

// Sample interrupt
void sampleISR()
{
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;

  int32_t Vout = phaseAcc >> 24;
  Vout = Vout >> (8 - knob3.get_rotation());

  analogWrite(OUTR_PIN, Vout + 128);
}

void updateDisplayTask(void *pvParameters)
{
  // Timing for the task
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // infinite loop for this task
  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    for (int i = 0; i < 4; ++i)
    {
      setRow(i);
      delayMicroseconds(3);
      xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      keyArray[i] = readCols();
      xSemaphoreGive(keyArrayMutex);
    }

    // Update display
    u8g2.clearBuffer();                 // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.setCursor(2, 20);

    // print volume
    u8g2.print(knob3.get_rotation(), DEC);

    // note showing
    u8g2.drawStr(2, 30, notes[note]);
    // direction of rotation

    //displaying the transfer message
    u8g2.setCursor(66,30);
    u8g2.print((char) TX_Message[0]);
    u8g2.print(TX_Message[1]);
    u8g2.print(TX_Message[2]);
    u8g2.sendBuffer(); // transfer internal memory to the display

    // Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}

void scanKeysTask(void *pvParameters)
{
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  uint8_t keymatrix, current_rotation;
  uint32_t curStep;
  uint16_t toAnd, keys;
  int prevKey;
  bool pressed;

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // use mutex to access keyarray
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    keys = (keyArray[2] << 8) + (keyArray[1] << 4) + keyArray[0];
    keymatrix = keyArray[3] & 0x03;
    xSemaphoreGive(keyArrayMutex);

    toAnd = 1;
    pressed = false;
    for (int i = 0; i < 12; ++i)
    {
      if (!((keys & toAnd) & 0xfff))
      {
        // note pressed
        curStep = stepSizes[i];
        note = i;
        pressed = true;
        //setting message to press
        prevKey = i;
        TX_Message[0] = 'P'; //press or release
        TX_Message[1] = 4; //octave -> need to come back to
        TX_Message[2] = i; //key being pressed
      }
      toAnd = toAnd << 1;
    }
    if (!pressed)
    {
      curStep = 0;
      //setting message to release
      TX_Message[0] = 'R';
      TX_Message[1] = 4;
      TX_Message[2] = prevKey;
    }

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    knob3.update_rotation(keymatrix);
    xSemaphoreGive(keyArrayMutex);

    __atomic_store_n(&currentStepSize, curStep, __ATOMIC_RELAXED);
  }
}

void setup()
{
  // put your setup code here, to run once:

  // Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  // Initialise display
  setOutMuxBit(DRST_BIT, LOW); // Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH); // Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH); // Enable display power supply

  // Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
      scanKeysTask,     /* Function that implements the task */
      "scanKeys",       /* Text name for the task */
      64,               /* Stack size in words, not bytes */
      NULL,             /* Parameter passed into the task */
      2,                /* Task priority */
      &scanKeysHandle); /* Pointer to store the task handle */

  TaskHandle_t updateDisplayHandle = NULL;
  xTaskCreate(
      updateDisplayTask,     /* Function that implements the task */
      "updateDisplay",       /* Text name for the task */
      64,                    /* Stack size in words, not bytes */
      NULL,                  /* Parameter passed into the task */
      1,                     /* Task priority */
      &updateDisplayHandle); /* Pointer to store the task handle */

  keyArrayMutex = xSemaphoreCreateMutex();

  // Timer
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);

  // Interrupt to execute the note playing
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  // setting knob 3 limits
  knob3.set_limits(0, 8);

  vTaskStartScheduler();
}

void loop()
{
}