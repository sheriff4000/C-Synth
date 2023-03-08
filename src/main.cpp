#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include "..\include\knob.hh"

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
  static uint32_t phaseAcc0 = 0;
  static uint32_t phaseAcc1 = 0;
  static uint32_t phaseAcc2 = 0;
  static uint32_t phaseAcc3 = 0;
  uint8_t knob2rotation = knob2.get_rotation();
  int32_t Vout;
  uint32_t currentStep = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);
  float_t angle;
  if (knob2rotation == 0)
  {
    // Sine waveform
    phaseAcc0 += currentStep;
    angle = ((float_t)phaseAcc0 / 4294967295) * 3.14159;
    Vout = sin(angle) * 255 - 128;
  }
  else if (knob2rotation == 1)
  {
    // triangular waveform.
    phaseAcc1 += currentStep;
    Vout = ((phaseAcc1 >> 31) & 1) ? -(phaseAcc2 >> 24) - 128 : (phaseAcc2 >> 24) - 128;
  }
  else if (knob2rotation == 2)
  {
    // Sawtooth
    phaseAcc2 += currentStep;
    Vout = (phaseAcc2 >> 24) - 128;
  }
  else if (knob2rotation == 3)
  { // Pulse waveform
    phaseAcc3 += currentStep;
    Vout = ((phaseAcc3 >> 31) & 1) ? -255 : 255;
  }

  Vout = Vout >> (8 - knob3.get_rotation());

  analogWrite(OUTR_PIN, Vout + 128);
}

void drawWaveform(uint8_t knob2rotation)
{
  if (knob2rotation == 0)
  {
    // sine waveform
    for (int i = 0; i < 40; i++)
    {
      float angle = i * 9;
      int x = 10 + sin(angle * 3.14159 / 180) * 7;
      u8g2.drawPixel(50 + i, x);
      u8g2.drawStr(95, 10, "Sine");
    }
  }
  else if (knob2rotation == 1)
  {
    // triangle waveform
    u8g2.drawLine(50, 15, 60, 5);
    u8g2.drawLine(60, 5, 70, 15);
    u8g2.drawLine(70, 15, 80, 5);
    u8g2.drawLine(80, 5, 90, 15);
    u8g2.drawStr(90, 10, "Triangle");
  }
  else if (knob2rotation == 2)
  {
    // sawtooth waveform
    u8g2.drawLine(50, 15, 60, 5);
    u8g2.drawLine(60, 5, 60, 15);
    u8g2.drawLine(60, 15, 70, 5);
    u8g2.drawLine(70, 5, 70, 15);
    u8g2.drawLine(70, 15, 80, 5);
    u8g2.drawStr(90, 10, "Sawtooth");
  }
  else if (knob2rotation == 3)
  {
    // pulse (square) waveform
    u8g2.drawLine(50, 15, 55, 15);
    u8g2.drawLine(55, 15, 55, 5);
    u8g2.drawLine(55, 5, 60, 5);
    u8g2.drawLine(60, 5, 60, 15);
    u8g2.drawLine(60, 15, 65, 15);
    u8g2.drawLine(65, 15, 65, 5);
    u8g2.drawLine(65, 5, 70, 5);
    u8g2.drawLine(70, 5, 70, 15);
    u8g2.drawLine(70, 15, 75, 15);
    u8g2.drawStr(90, 10, "Pulse");
  }
}

void updateDisplayTask(void *pvParameters)
{
  // Timing for the task
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  uint8_t knob2value;
  uint8_t knob3value;

  // infinite loop for this task
  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    knob3value = knob3.get_rotation();
    knob2value = knob2.get_rotation();

    // Update display
    u8g2.clearBuffer();                   // clear the internal memory
    u8g2.setFont(u8g2_font_profont10_tf); // choose a suitable font

    // Knob 2 (waveform)
    u8g2.drawStr(2, 10, "Waveform"); // write something to the internal memory
    drawWaveform(knob2value);

    // Knob 3 (volume)
    u8g2.drawStr(2, 30, "Vol"); // write something to the internal memory
    u8g2.setCursor(30, 30);
    u8g2.print(knob3value, DEC);

    // print volume (knob2)

    // note showing
    u8g2.drawStr(90, 30, "Note"); // write something to the internal memory
    u8g2.drawStr(120, 30, notes[note]);
    // direction of rotation
    u8g2.sendBuffer(); // transfer internal memory to the display

    // Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}

void scanKeysTask(void *pvParameters)
{
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  uint8_t keymatrix3, keymatrix2;
  uint32_t curStep;
  uint16_t toAnd, keys;
  bool pressed;

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    for (int i = 0; i < 4; ++i)
    {
      setRow(i);
      delayMicroseconds(3);
      keyArray[i] = readCols();
    }
    xSemaphoreGive(keyArrayMutex);

    // use mutex to access keyarray
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    keys = (keyArray[2] << 8) + (keyArray[1] << 4) + keyArray[0];
    // knob3
    keymatrix3 = keyArray[3] & 0x03;
    // knob2keymatri
    keymatrix2 = (keyArray[3] & 0x0C) >> 2;
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
      }
      toAnd = toAnd << 1;
    }
    if (!pressed)
    {
      curStep = 0;
    }

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    knob3.update_rotation(keymatrix3);
    knob2.update_rotation(keymatrix2);
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
  knob2.set_limits(0, 3);
  vTaskStartScheduler();
}

void loop()
{
}