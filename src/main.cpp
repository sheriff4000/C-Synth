#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include "../include/knob.hh"
#include <ES_CAN.h>
// mutex to handle synchronization bug
SemaphoreHandle_t keyArrayMutex;
SemaphoreHandle_t notesArrayMutex;
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

//handshaking variables
volatile bool east;
volatile bool west;

//keyboard defining variables
volatile int keyboardIndex;
volatile int numberOfKeyboards;

// Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//  store reading from each row.
volatile uint8_t keyArray[7];
const uint32_t stepSizes[] = {50953930, 54077542, 57396381, 60715219, 64229283, 68133799, 72233540, 76528508, 81018701, 85899345, 90975216, 96441538};

// volatile to allow for concurrency
volatile uint16_t g_note_states[3];
volatile uint64_t g_ss;

const char *notes[12] = {"C", "C#", "D", "Eb", "E", "F", "F#", "G", "G#", "A", "Bb", "B"};
volatile uint8_t note;
volatile uint32_t global_Vout;

// bendy bendy
volatile uint32_t bendStep;

// Global knobs
Knob local_knob0, local_knob1, local_knob2, local_knob3;

int global_knob0 = 0;
int global_knob1 = 0;
int global_knob2 = 0;
int global_knob3 = 0;
int global_knob4 = 0;
int global_knob5 = 0;
int global_knob6 = 0;
int global_knob7 = 0;

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
  global_Vout = 0;
  uint32_t lower_phases[12] = {0};
  uint32_t middle_phases[12] = {0};
  uint32_t upper_phases[12] = {0};
  uint64_t ss;
  while(1){
    xSemaphoreTake(sampleBufferSemaphore, portMAX_DELAY);
    for (uint32_t writeCtr = 0; writeCtr < SAMPLE_BUFFER_SIZE; writeCtr++) {
      // TODO check if g_note_states is ok to be accessed here

      //doing lowe keyboard
      ss = g_ss;
      uint32_t Vout = 0; 
      uint8_t volume = knob3.get_rotation();

      if(keyboardIndex == 0){
        for (int i=0; i<12; ++i) {
          if (ss & 1) {
            lower_phases[i] += (stepSizes[i] >> 1) + bendStep;
            Vout += ((lower_phases[i] >> 24) - 128) >> (8 - volume);
          }

          if (ss & 0x1000) {
            middle_phases[i] += stepSizes[i] + bendStep;
            Vout += ((middle_phases[i] >> 24) - 128) >> (8 - volume);
          }

          if (ss & 0x1000000) {
            upper_phases[i] += (stepSizes[i] << 1) + bendStep;
            Vout += ((upper_phases[i] >> 24) - 128) >> (8 - volume);
          }

          ss = ss >> 1;
        }

        global_Vout = Vout;
      }
      else{
        Vout = global_Vout;
      }
      

      if (writeBuffer1)
        sampleBuffer1[writeCtr] = Vout + 128;
      else
        sampleBuffer0[writeCtr] = Vout + 128;
    }
  }
}

void scanOtherBoardsTask(void *pvParameters){
  //CAN Message Variables
  uint32_t ID;
  uint8_t RX_Message[8]={0};
  
  //Timing for the task
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    while (CAN_CheckRXLevel()){
      CAN_RX(ID, RX_Message);
      xSemaphoreTake(notesArrayMutex, portMAX_DELAY);
      if(keyboardIndex == 0){
        g_note_states[RX_Message[0]] = ((RX_Message[3] & 0xf)  << 8) + ((RX_Message[2] & 0xf) << 4) + (RX_Message[1] & 0xf);
      }
      else{
        g_note_states[RX_Message[0]] = ((RX_Message[3] & 0xf)  << 8) + ((RX_Message[2] & 0xf) << 4) + (RX_Message[1] & 0xf); // change this to global vout
      }
      xSemaphoreGive(notesArrayMutex);
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
    //print east or west

    u8g2.print(numberOfKeyboards, DEC);
    u8g2.print(keyboardIndex, DEC);
    



    u8g2.setCursor(2, 30);
    xSemaphoreTake(notesArrayMutex, portMAX_DELAY);
    u8g2.print(g_ss, HEX);
    xSemaphoreGive(notesArrayMutex);
    
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
  uint8_t TX_Message[8]={0};
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
    for (int i = 0; i < 7; ++i)
    {
      setRow(i);
      digitalWrite(REN_PIN,1);          //Enable selected row
      delayMicroseconds(3);
      keyArray[i] = readCols();
      digitalWrite(REN_PIN,0);
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
    int msg_states = note_states;
    TX_Message[0] = keyboardIndex;
    TX_Message[1] = msg_states & 0xf;
    msg_states = msg_states >> 4;
    TX_Message[2] = msg_states & 0xf;
    msg_states = msg_states >> 4;
    TX_Message[3] = msg_states & 0xf;
    if(numberOfKeyboards > 1){
       CAN_TX(0x123, TX_Message);
    }
   
    // TODO check if need to use mutex here
    knob3.update_rotation(keymatrix);

    __atomic_store_n(&bendStep, 2 * 8080 * (512 - analogRead(JOYX_PIN)), __ATOMIC_RELAXED);

    xSemaphoreTake(notesArrayMutex, portMAX_DELAY);    
    g_note_states[0] = note_states;// need to protect this at some point!!
    g_ss = (uint64_t)g_note_states[0] + ((uint64_t)g_note_states[1] << 12) + ((uint64_t)g_note_states[2] << 24);
    xSemaphoreGive(notesArrayMutex);
  }
}


void handShake(){

  uint32_t startup = millis();
  uint32_t current = startup;
  while (current < startup+3000)//for 4 seconds
  {
    current = millis();
    // use mutex to access keyarray
    for (int i = 0; i < 7; ++i)
    {
      setRow(i);
      if(i==5||i==6){
        digitalWrite(OUT_PIN,1);
      }
       //Set value to latch in DFF
      digitalWrite(REN_PIN,1);          //Enable selected row
      delayMicroseconds(3);
      keyArray[i] = readCols();
      digitalWrite(REN_PIN,0);
    }
    if(!(keyArray[5] & 0b1000)){
      west = true;
    }
    if(!(keyArray[6] & 0b1000)){
      east = true;
    }
  }
  //transmit 1 if west and east. else transmit 0 or 2.
  uint8_t TX_Message[8]={0};
  if(west && east){
    TX_Message[0] = 1;
    CAN_TX(0x123, TX_Message);
    delay(500);
    CAN_TX(0x123, TX_Message); // send msg again in case of startup error or something
    numberOfKeyboards = 3;
    keyboardIndex = 1;
    return;
  }
  
  //TX_Message[0] = 2;
  uint8_t RX_Message[8]={0};
  uint32_t ID;
  uint32_t start = millis();
  uint32_t current2 = start;

  while (current2 < (start + 3000)){
    current2 = millis();
    if(CAN_CheckRXLevel() > 0){
      CAN_RX(ID, RX_Message);
      if(RX_Message[0] == 1){
        numberOfKeyboards = 3;
        if(east){
          keyboardIndex = 0;
        }
        else if(west){
          keyboardIndex = 2;
        }
        return;
    }
    }
    
  }
  
  if(east || west){
  numberOfKeyboards = 2;
    if(east){
    keyboardIndex = 0;
    }
    else if(west){
      keyboardIndex = 1;
    }
    return;
  }
  Serial.println("got here yay");
  numberOfKeyboards = 1;
  keyboardIndex = 0;
  return;
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
      128,                    /* Stack size in words, not bytes */
      NULL,                  /* Parameter passed into the task */
      4,                     /* Task priority */
      &sampleGenerationHandle); /* Pointer to store the task handle */

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
      scanKeysTask,     /* Function that implements the task */
      "scanKeys",       /* Text name for the task */
      64,               /* Stack size in words, not bytes */
      NULL,             /* Parameter passed into the task */
      3,                /* Task priority */
      &scanKeysHandle); /* Pointer to store the task handle */
  
  
  TaskHandle_t scanOtherBoards = NULL;
  xTaskCreate(
      scanOtherBoardsTask,     /* Function that implements the task */
      "scanOtherBoards",       /* Text name for the task */
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
  notesArrayMutex = xSemaphoreCreateMutex();
  sampleBufferSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(sampleBufferSemaphore);

  // Timer
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);

  // Interrupt to execute the note playing
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  // setting knob limits
  knob0.set_limits(0, 8);
  knob1.set_limits(0, 8);
  knob2.set_limits(0, 8);
  knob3.set_limits(0, 8);
  knob4.set_limits(0, 8);
  knob5.set_limits(0, 8);
  knob6.set_limits(0, 8);
  knob7.set_limits(0, 8);
   
  //Initialise CAN
  CAN_Init(false);
  setCANFilter(0x123,0x7ff);
  CAN_Start();  
  handShake();
  Serial.println("finished handshaking");
  vTaskStartScheduler();
}

void loop()
{
}

