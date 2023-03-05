#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include "..\include\knob.hh"

// mutex to handle synchronization bug
SemaphoreHandle_t keyArrayMutex;
SemaphoreHandle_t sampleBufferSemaphore;

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
volatile uint16_t g_note_states;

const char *notes[12] = {"C", "C#", "D", "Eb", "E", "F", "F#", "G", "G#", "A", "Bb", "B"};
volatile uint8_t note;

// Global knobs
Knob knob0, knob1, knob2, knob3;

// DMA
// TODO work out the proper way of determining sample buffer size
const int SAMPLE_BUFFER_SIZE = 100;
uint8_t sampleBuffer0[SAMPLE_BUFFER_SIZE];
uint8_t sampleBuffer1[SAMPLE_BUFFER_SIZE];
volatile bool writeBuffer1 = false;

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
  static uint32_t readCtr = 0;

  if (readCtr == SAMPLE_BUFFER_SIZE) {
    readCtr = 0;
    writeBuffer1 = !writeBuffer1;
    xSemaphoreGiveFromISR(sampleBufferSemaphore, NULL);
    }
    
  if (writeBuffer1)
    analogWrite(OUTR_PIN, sampleBuffer0[readCtr++]);
  else
    analogWrite(OUTR_PIN, sampleBuffer1[readCtr++]);
}

void sampleGenerationTask(void *pvParameters) {
  // BUFFER SIZE: 100; INITIATION INTERVAL = 4.5ms
  uint32_t phases[12] = {0};
  while(1){
    xSemaphoreTake(sampleBufferSemaphore, portMAX_DELAY);
    for (uint32_t writeCtr = 0; writeCtr < SAMPLE_BUFFER_SIZE; writeCtr++) {
      // TODO check if g_note_states is ok to be accessed here
      uint32_t toAnd = 1;
      uint32_t ss = g_note_states;
      int32_t Vout = 0;
      uint8_t volume = knob3.get_rotation();

      for (int i=0; i<12; ++i) {
        if (ss & toAnd) {
          phases[i] += stepSizes[i];
          // TODO volume = 0 plays same sound as 1
          Vout += ((phases[i] >> 24) - 128) >> (8 - volume);
        }
        toAnd = toAnd << 1;
      }

      if (writeBuffer1)
        sampleBuffer1[writeCtr] = Vout + 128;
      else
        sampleBuffer0[writeCtr] = Vout + 128;
    }
  }
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
    
    // Update display
    u8g2.clearBuffer();                 // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.setCursor(2, 20);

    // print volume
    u8g2.print(knob3.get_rotation_atomic(), DEC);

    // note showing
    // u8g2.drawStr(2, 30, notes[note]);

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

  uint8_t keymatrix, current_rotation;
  uint16_t toAnd, keys;
  bool pressed;

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // use mutex to access keyarray
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    for (int i = 0; i < 4; ++i)
    {
      setRow(i);
      delayMicroseconds(3);
      keyArray[i] = readCols();
    }
    xSemaphoreGive(keyArrayMutex);

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    keys = (keyArray[2] << 8) + (keyArray[1] << 4) + keyArray[0];
    keymatrix = keyArray[3] & 0x03;
    xSemaphoreGive(keyArrayMutex);

    // note_states represents a 12-bit state of all notes
    uint16_t note_states = 0;
    uint16_t toAnd = 1;
    bool pressed = false;

    for (int i = 0; i < 12; ++i)
    {
      if (!((keys & toAnd) & 0xfff))
      {
        // note pressed
        note_states += toAnd;
        note = i;
        pressed = true;
      }
      toAnd = toAnd << 1;
    }

    // TODO check if need to use mutex here
    knob3.update_rotation(keymatrix);

    __atomic_store_n(&g_note_states, note_states, __ATOMIC_RELAXED);
  }
}

void setup()
{
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

  // thread initialisation
  TaskHandle_t sampleGenerationHandle = NULL;
  xTaskCreate(
      sampleGenerationTask,     /* Function that implements the task */
      "sampleGeneration",       /* Text name for the task */
      64,                    /* Stack size in words, not bytes */
      NULL,                  /* Parameter passed into the task */
      3,                     /* Task priority */
      &sampleGenerationHandle); /* Pointer to store the task handle */

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
  sampleBufferSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(sampleBufferSemaphore);

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