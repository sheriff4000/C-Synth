#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include "../include/knob.hh"
#include <ES_CAN.h>

// mutex to handle synchronization bug
SemaphoreHandle_t keyArrayMutex;
SemaphoreHandle_t notesArrayMutex;
SemaphoreHandle_t sampleBufferSemaphore;
SemaphoreHandle_t noteMultiplierMutex;
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

// handshaking variables
volatile bool east;
volatile bool west;

// keyboard defining variables
volatile int keyboardIndex;
volatile int numberOfKeyboards;

// Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//  store reading from each row.
volatile uint8_t keyArray[7];
const uint32_t stepSizes[] = {50953930, 54077542, 57396381, 60715219, 64229283, 68133799, 72233540, 76528508, 81018701, 85899345, 90975216, 96441538};

// sine wave 

float_t sin_lut[1024] = {0, 0.00613588, 0.0122715, 0.0184067, 0.0245412, 0.0306748, 0.0368072, 0.0429383, 0.0490677, 0.0551952, 0.0613207, 0.0674439, 0.0735646, 0.0796824, 0.0857973, 0.091909, 0.0980171, 0.104122, 0.110222, 0.116319, 0.122411, 0.128498, 0.134581, 0.140658, 0.14673, 0.152797, 0.158858, 0.164913, 0.170962, 0.177004, 0.18304, 0.189069, 0.19509, 0.201105, 0.207111, 0.21311, 0.219101, 0.225084, 0.231058, 0.237024, 0.24298, 0.248928, 0.254866, 0.260794, 0.266713, 0.272621, 0.27852, 0.284408, 0.290285, 0.296151, 0.302006, 0.30785, 0.313682, 0.319502, 0.32531, 0.331106, 0.33689, 0.342661, 0.348419, 0.354164, 0.359895, 0.365613, 0.371317, 0.377007, 0.382683, 0.388345, 0.393992, 0.399624, 0.405241, 0.410843, 0.41643, 0.422, 0.427555, 0.433094, 0.438616, 0.444122, 0.449611, 0.455084, 0.460539, 0.465976, 0.471397, 0.476799, 0.482184, 0.48755, 0.492898, 0.498228, 0.503538, 0.50883, 0.514103, 0.519356, 0.52459, 0.529804, 0.534998, 0.540171, 0.545325, 0.550458, 0.55557, 0.560662, 0.565732, 0.570781, 0.575808, 0.580814, 0.585798, 0.59076, 0.595699, 0.600616, 0.605511, 0.610383, 0.615232, 0.620057, 0.624859, 0.629638, 0.634393, 0.639124, 0.643832, 0.648514, 0.653173, 0.657807, 0.662416, 0.667, 0.671559, 0.676093, 0.680601, 0.685084, 0.689541, 0.693971, 0.698376, 0.702755, 0.707107, 0.711432, 0.715731, 0.720003, 0.724247, 0.728464, 0.732654, 0.736817, 0.740951, 0.745058, 0.749136, 0.753187, 0.757209, 0.761202, 0.765167, 0.769103, 0.77301, 0.776888, 0.780737, 0.784557, 0.788346, 0.792107, 0.795837, 0.799537, 0.803208, 0.806848, 0.810457, 0.814036, 0.817585, 0.821103, 0.824589, 0.828045, 0.83147, 0.834863, 0.838225, 0.841555, 0.844854, 0.84812, 0.851355, 0.854558, 0.857729, 0.860867, 0.863973, 0.867046, 0.870087, 0.873095, 0.87607, 0.879012, 0.881921, 0.884797, 0.88764, 0.890449, 0.893224, 0.895966, 0.898674, 0.901349, 0.903989, 0.906596, 0.909168, 0.911706, 0.91421, 0.916679, 0.919114, 0.921514, 0.92388, 0.92621, 0.928506, 0.930767, 0.932993, 0.935184, 0.937339, 0.939459, 0.941544, 0.943593, 0.945607, 0.947586, 0.949528, 0.951435, 0.953306, 0.955141, 0.95694, 0.958703, 0.960431, 0.962121, 0.963776, 0.965394, 0.966976, 0.968522, 0.970031, 0.971504, 0.97294, 0.974339, 0.975702, 0.977028, 0.978317, 0.97957, 0.980785, 0.981964, 0.983105, 0.98421, 0.985278, 0.986308, 0.987301, 0.988258, 0.989177, 0.990058, 0.990903, 0.99171, 0.99248, 0.993212, 0.993907, 0.994565, 0.995185, 0.995767, 0.996313, 0.99682, 0.99729, 0.997723, 0.998118, 0.998476, 0.998795, 0.999078, 0.999322, 0.999529, 0.999699, 0.999831, 0.999925, 0.999981, 1, 0.999981, 0.999925, 0.999831, 0.999699, 0.999529, 0.999322, 0.999078, 0.998795, 0.998476, 0.998118, 0.997723, 0.99729, 0.99682, 0.996313, 0.995767, 0.995185, 0.994565, 0.993907, 0.993212, 0.99248, 0.99171, 0.990903, 0.990058, 0.989177, 0.988258, 0.987301, 0.986308, 0.985278, 0.98421, 0.983105, 0.981964, 0.980785, 0.97957, 0.978317, 0.977028, 0.975702, 0.974339, 0.97294, 0.971504, 0.970031, 0.968522, 0.966976, 0.965394, 0.963776, 0.962121, 0.960431, 0.958703, 0.95694, 0.955141, 0.953306, 0.951435, 0.949528, 0.947586, 0.945607, 0.943593, 0.941544, 0.939459, 0.937339, 0.935184, 0.932993, 0.930767, 0.928506, 0.92621, 0.92388, 0.921514, 0.919114, 0.916679, 0.91421, 0.911706, 0.909168, 0.906596, 0.903989, 0.901349, 0.898674, 0.895966, 0.893224, 0.890449, 0.88764, 0.884797, 0.881921, 0.879012, 0.87607, 0.873095, 0.870087, 0.867046, 0.863973, 0.860867, 0.857729, 0.854558, 0.851355, 0.84812, 0.844854, 0.841555, 0.838225, 0.834863, 0.83147, 0.828045, 0.824589, 0.821103, 0.817585, 0.814036, 0.810457, 0.806848, 0.803208, 0.799537, 0.795837, 0.792107, 0.788346, 0.784557, 0.780737, 0.776888, 0.77301, 0.769103, 0.765167, 0.761202, 0.757209, 0.753187, 0.749136, 0.745058, 0.740951, 0.736817, 0.732654, 0.728464, 0.724247, 0.720003, 0.715731, 0.711432, 0.707107, 0.702755, 0.698376, 0.693971, 0.689541, 0.685084, 0.680601, 0.676093, 0.671559, 0.667, 0.662416, 0.657807, 0.653173, 0.648514, 0.643832, 0.639124, 0.634393, 0.629638, 0.624859, 0.620057, 0.615232, 0.610383, 0.605511, 0.600616, 0.595699, 0.59076, 0.585798, 0.580814, 0.575808, 0.570781, 0.565732, 0.560662, 0.55557, 0.550458, 0.545325, 0.540171, 0.534998, 0.529804, 0.52459, 0.519356, 0.514103, 0.50883, 0.503538, 0.498228, 0.492898, 0.48755, 0.482184, 0.476799, 0.471397, 0.465976, 0.460539, 0.455084, 0.449611, 0.444122, 0.438616, 0.433094, 0.427555, 0.422, 0.41643, 0.410843, 0.405241, 0.399624, 0.393992, 0.388345, 0.382683, 0.377007, 0.371317, 0.365613, 0.359895, 0.354164, 0.348419, 0.342661, 0.33689, 0.331106, 0.32531, 0.319502, 0.313682, 0.30785, 0.302006, 0.296151, 0.290285, 0.284408, 0.27852, 0.272621, 0.266713, 0.260794, 0.254866, 0.248928, 0.24298, 0.237024, 0.231058, 0.225084, 0.219101, 0.21311, 0.207111, 0.201105, 0.19509, 0.189069, 0.18304, 0.177004, 0.170962, 0.164913, 0.158858, 0.152797, 0.14673, 0.140658, 0.134581, 0.128498, 0.122411, 0.116319, 0.110222, 0.104122, 0.0980171, 0.091909, 0.0857973, 0.0796824, 0.0735646, 0.0674439, 0.0613207, 0.0551952, 0.0490677, 0.0429383, 0.0368072, 0.0306748, 0.0245412, 0.0184067, 0.0122715, 0.00613588, 0, -0.00613588, -0.0122715, -0.0184067, -0.0245412, -0.0306748, -0.0368072, -0.0429383, -0.0490677, -0.0551952, -0.0613207, -0.0674439, -0.0735646, -0.0796824, -0.0857973, -0.091909, -0.0980171, -0.104122, -0.110222, -0.116319, -0.122411, -0.128498, -0.134581, -0.140658, -0.14673, -0.152797, -0.158858, -0.164913, -0.170962, -0.177004, -0.18304, -0.189069, -0.19509, -0.201105, -0.207111, -0.21311, -0.219101, -0.225084, -0.231058, -0.237024, -0.24298, -0.248928, -0.254866, -0.260794, -0.266713, -0.272621, -0.27852, -0.284408, -0.290285, -0.296151, -0.302006, -0.30785, -0.313682, -0.319502, -0.32531, -0.331106, -0.33689, -0.342661, -0.348419, -0.354164, -0.359895, -0.365613, -0.371317, -0.377007, -0.382683, -0.388345, -0.393992, -0.399624, -0.405241, -0.410843, -0.41643, -0.422, -0.427555, -0.433094, -0.438616, -0.444122, -0.449611, -0.455084, -0.460539, -0.465976, -0.471397, -0.476799, -0.482184, -0.48755, -0.492898, -0.498228, -0.503538, -0.50883, -0.514103, -0.519356, -0.52459, -0.529804, -0.534998, -0.540171, -0.545325, -0.550458, -0.55557, -0.560662, -0.565732, -0.570781, -0.575808, -0.580814, -0.585798, -0.59076, -0.595699, -0.600616, -0.605511, -0.610383, -0.615232, -0.620057, -0.624859, -0.629638, -0.634393, -0.639124, -0.643832, -0.648514, -0.653173, -0.657807, -0.662416, -0.667, -0.671559, -0.676093, -0.680601, -0.685084, -0.689541, -0.693971, -0.698376, -0.702755, -0.707107, -0.711432, -0.715731, -0.720003, -0.724247, -0.728464, -0.732654, -0.736817, -0.740951, -0.745058, -0.749136, -0.753187, -0.757209, -0.761202, -0.765167, -0.769103, -0.77301, -0.776888, -0.780737, -0.784557, -0.788346, -0.792107, -0.795837, -0.799537, -0.803208, -0.806848, -0.810457, -0.814036, -0.817585, -0.821103, -0.824589, -0.828045, -0.83147, -0.834863, -0.838225, -0.841555, -0.844854, -0.84812, -0.851355, -0.854558, -0.857729, -0.860867, -0.863973, -0.867046, -0.870087, -0.873095, -0.87607, -0.879012, -0.881921, -0.884797, -0.88764, -0.890449, -0.893224, -0.895966, -0.898674, -0.901349, -0.903989, -0.906596, -0.909168, -0.911706, -0.91421, -0.916679, -0.919114, -0.921514, -0.92388, -0.92621, -0.928506, -0.930767, -0.932993, -0.935184, -0.937339, -0.939459, -0.941544, -0.943593, -0.945607, -0.947586, -0.949528, -0.951435, -0.953306, -0.955141, -0.95694, -0.958703, -0.960431, -0.962121, -0.963776, -0.965394, -0.966976, -0.968522, -0.970031, -0.971504, -0.97294, -0.974339, -0.975702, -0.977028, -0.978317, -0.97957, -0.980785, -0.981964, -0.983105, -0.98421, -0.985278, -0.986308, -0.987301, -0.988258, -0.989177, -0.990058, -0.990903, -0.99171, -0.99248, -0.993212, -0.993907, -0.994565, -0.995185, -0.995767, -0.996313, -0.99682, -0.99729, -0.997723, -0.998118, -0.998476, -0.998795, -0.999078, -0.999322, -0.999529, -0.999699, -0.999831, -0.999925, -0.999981, -1, -0.999981, -0.999925, -0.999831, -0.999699, -0.999529, -0.999322, -0.999078, -0.998795, -0.998476, -0.998118, -0.997723, -0.99729, -0.99682, -0.996313, -0.995767, -0.995185, -0.994565, -0.993907, -0.993212, -0.99248, -0.99171, -0.990903, -0.990058, -0.989177, -0.988258, -0.987301, -0.986308, -0.985278, -0.98421, -0.983105, -0.981964, -0.980785, -0.97957, -0.978317, -0.977028, -0.975702, -0.974339, -0.97294, -0.971504, -0.970031, -0.968522, -0.966976, -0.965394, -0.963776, -0.962121, -0.960431, -0.958703, -0.95694, -0.955141, -0.953306, -0.951435, -0.949528, -0.947586, -0.945607, -0.943593, -0.941544, -0.939459, -0.937339, -0.935184, -0.932993, -0.930767, -0.928506, -0.92621, -0.92388, -0.921514, -0.919114, -0.916679, -0.91421, -0.911706, -0.909168, -0.906596, -0.903989, -0.901349, -0.898674, -0.895966, -0.893224, -0.890449, -0.88764, -0.884797, -0.881921, -0.879012, -0.87607, -0.873095, -0.870087, -0.867046, -0.863973, -0.860867, -0.857729, -0.854558, -0.851355, -0.84812, -0.844854, -0.841555, -0.838225, -0.834863, -0.83147, -0.828045, -0.824589, -0.821103, -0.817585, -0.814036, -0.810457, -0.806848, -0.803208, -0.799537, -0.795837, -0.792107, -0.788346, -0.784557, -0.780737, -0.776888, -0.77301, -0.769103, -0.765167, -0.761202, -0.757209, -0.753187, -0.749136, -0.745058, -0.740951, -0.736817, -0.732654, -0.728464, -0.724247, -0.720003, -0.715731, -0.711432, -0.707107, -0.702755, -0.698376, -0.693971, -0.689541, -0.685084, -0.680601, -0.676093, -0.671559, -0.667, -0.662416, -0.657807, -0.653173, -0.648514, -0.643832, -0.639124, -0.634393, -0.629638, -0.624859, -0.620057, -0.615232, -0.610383, -0.605511, -0.600616, -0.595699, -0.59076, -0.585798, -0.580814, -0.575808, -0.570781, -0.565732, -0.560662, -0.55557, -0.550458, -0.545325, -0.540171, -0.534998, -0.529804, -0.52459, -0.519356, -0.514103, -0.50883, -0.503538, -0.498228, -0.492898, -0.48755, -0.482184, -0.476799, -0.471397, -0.465976, -0.460539, -0.455084, -0.449611, -0.444122, -0.438616, -0.433094, -0.427555, -0.422, -0.41643, -0.410843, -0.405241, -0.399624, -0.393992, -0.388345, -0.382683, -0.377007, -0.371317, -0.365613, -0.359895, -0.354164, -0.348419, -0.342661, -0.33689, -0.331106, -0.32531, -0.319502, -0.313682, -0.30785, -0.302006, -0.296151, -0.290285, -0.284408, -0.27852, -0.272621, -0.266713, -0.260794, -0.254866, -0.248928, -0.24298, -0.237024, -0.231058, -0.225084, -0.219101, -0.21311, -0.207111, -0.201105, -0.19509, -0.189069, -0.18304, -0.177004, -0.170962, -0.164913, -0.158858, -0.152797, -0.14673, -0.140658, -0.134581, -0.128498, -0.122411, -0.116319, -0.110222, -0.104122, -0.0980171, -0.091909, -0.0857973, -0.0796824, -0.0735646, -0.0674439, -0.0613207, -0.0551952, -0.0490677, -0.0429383, -0.0368072, -0.0306748, -0.0245412, -0.0184067, -0.0122715, -0.00613588};
// volatile to allow for concurrency
volatile uint16_t g_note_states[3];
volatile uint64_t g_ss;

const char *notes[12] = {"C", "C#", "D", "Eb", "E", "F", "F#", "G", "G#", "A", "Bb", "B"};
volatile uint8_t note;
volatile uint32_t global_Vout;

// bendy bendy
volatile uint32_t bendStep;

// vibby vibby
volatile float vibFactor;
volatile int32_t vibStep;
volatile bool vibPositive;

//envylopey
volatile float noteMult[12] = {0.};
volatile bool envActive[12] = {false};

// local knobs
Knob local_knob0, local_knob1, local_knob2, local_knob3;

//global knobs
volatile int8_t global_knob0 = 0;
volatile int8_t global_knob1 = 0;
volatile int8_t global_knob2 = 0;
volatile int8_t global_knob3 = 0;
volatile int8_t global_knob4 = 0;
volatile int8_t global_knob5 = 0;
volatile int8_t global_knob6 = 0;
volatile int8_t global_knob7 = 0;
volatile int8_t global_knob8 = 0;
volatile int8_t global_knob9 = 0;
volatile int8_t global_knob10 = 0;
volatile int8_t global_knob11 = 0;
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

void setVibFactor(void *pvParameters){
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  long int joyChange;
  float abs_joyChange;
  while(1){
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    joyChange = 512 - analogRead(JOYY_PIN);
    abs_joyChange = (joyChange < 0) ? -1.0 * joyChange : joyChange;
    vibFactor = (abs_joyChange)/512.0;
    //Serial.println(vibFactor);
  }
}

void setVibStepTask(void *pvParameters){
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

   //vib things
  int32_t vibVout;
  float vibStepSize = 2049870/2; //1366580 * 2;
  static uint32_t vibAcc = 0;
  float_t vibAngle;

  long int joyChange;
  float abs_joyChange;
  int i = 0;
  float t;
  float freq;
  float tempVibStep;
  while(1){
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    joyChange = 512 - analogRead(JOYY_PIN);
    abs_joyChange = (joyChange < 0) ? -1.0 * joyChange : joyChange;
    if (abs_joyChange < 100){
      //abs_joyChange = 0;
      vibStep = 0;
      continue;
    }

    //vibFactor = (abs_joyChange)/512.0;
    freq = 2500.0 * (abs_joyChange/512);

    t = ((float)i) / 22000;
    
    tempVibStep = vibStepSize * sin(2 * 3.14 * freq * t);
    vibStep = (int32_t) tempVibStep;
    
    if(i == 22000){
      i = 0;
    }
    i++;
    //Serial.println(vibFactor);
  }
}

void keyPressExecution(void * pvParameters) {
  //Serial.println("entering/task stuff works");
  uint8_t note = (int)pvParameters;
  uint32_t attack = 100 * global_knob4;
  uint32_t decay = 50 * global_knob5;
  float sustain = global_knob6/8;
  uint32_t release = 100 * global_knob7;
  float voutMult = 0.0;
  int startTime = millis();
  int currentTime = startTime;
  float atkDif;

  noteMult[note] = voutMult;

  //attack
  while((currentTime < startTime + attack) && ((g_note_states[0] & (1 << note)) != 0)){
    //if (g_note_states[0] & (1 << note) == 0) break;
    //Serial.println("attacking");
    delay(50);
    currentTime = millis();
    //voutMult = 0 + (float)(currentTime-startTime) / (float)attack;
    if (currentTime > startTime + attack/10){
      atkDif = ((float)(currentTime - startTime) / (float)attack);
      voutMult = atkDif;
      xSemaphoreTake(noteMultiplierMutex, portMAX_DELAY);
      noteMult[note] = voutMult;
      xSemaphoreGive(noteMultiplierMutex);
    }
  }
  xSemaphoreTake(noteMultiplierMutex, portMAX_DELAY);
  noteMult[note] = 1;
  xSemaphoreGive(noteMultiplierMutex);
  startTime = millis();
  currentTime = startTime;
  //decay
  while(currentTime < startTime + decay){
    if (g_note_states[0] & (1 << note) == 0) break;
    //delay(50);
    currentTime = millis();
    voutMult = 1. - ((1.0 - sustain) * ((float)(currentTime - startTime) / (float)decay));
    xSemaphoreTake(noteMultiplierMutex, portMAX_DELAY);
    noteMult[note] = voutMult;
    xSemaphoreGive(noteMultiplierMutex);
  }
  voutMult = sustain;
  xSemaphoreTake(noteMultiplierMutex, portMAX_DELAY);
  noteMult[note] = voutMult;
  xSemaphoreGive(noteMultiplierMutex);
  // //sustain
  // Serial.println(g_note_states[0]);
  // Serial.println(g_note_states[0] & (1 << note));
  while ((g_note_states[0] & (1 << note)) != 0){
    //Serial.println("sustaining");
    //Serial.println(voutMult);
  }
  startTime = millis();
  currentTime = startTime;
  envActive[note] = false;
  //release
  if((g_note_states[0] & (1 << note)) == 0){
    g_note_states[0] += 1 << note;
  }
  while(currentTime < startTime + release){
    //Serial.println("releasing");
    delay(50);
    currentTime = millis();
    voutMult = sustain - sustain * ((float)(currentTime-startTime) / (float)release);
    if(voutMult < 0){
      voutMult = 0;
      xSemaphoreTake(noteMultiplierMutex, portMAX_DELAY);
      noteMult[note] = voutMult;
      xSemaphoreGive(noteMultiplierMutex);
      break;
    }
    xSemaphoreTake(noteMultiplierMutex, portMAX_DELAY);
    noteMult[note] = voutMult;
    xSemaphoreGive(noteMultiplierMutex);
    
    //Serial.println(voutMult);
  }
  //Serial.println("ending task");
  vTaskDelete(NULL);
}

void startEnvelopeTask(int note){
  TaskHandle_t envelopeTask = NULL;
  xTaskCreate(
      keyPressExecution,     /* Function that implements the task */
      "enveloper",       /* Text name for the task */
      128,                    /* Stack size in words, not bytes */
      (void *)note,                  /* Parameter passed into the task */
      1,                     /* Task priority */
      &envelopeTask);        /* Pointer to store the task handle */
}


// Sample interrupt
void sampleISR()
{
  static uint32_t readCtr = 0;

  if (readCtr == SAMPLE_BUFFER_SIZE)
  {
    readCtr = 0;
    writeBuffer1 = !writeBuffer1;
    xSemaphoreGiveFromISR(sampleBufferSemaphore, NULL);
  }

  if (writeBuffer1)
    analogWrite(OUTR_PIN, sampleBuffer0[readCtr++]);
  else
    analogWrite(OUTR_PIN, sampleBuffer1[readCtr++]);
}


void sampleGenerationTask(void *pvParameters)
{
  // BUFFER SIZE: 100; INITIATION INTERVAL = 4.5ms
  global_Vout = 0;
  // Sine wave phases
  uint32_t lower_phases0[12] = {0};
  uint32_t middle_phases0[12] = {0};
  uint32_t upper_phases0[12] = {0};

  // Triangle phases
  uint32_t lower_phases1[12] = {0};
  uint32_t middle_phases1[12] = {0};
  uint32_t upper_phases1[12] = {0};

  // Sawtooth phases
  uint32_t lower_phases2[12] = {0};
  uint32_t middle_phases2[12] = {0};
  uint32_t upper_phases2[12] = {0};

  // Pulse phases
  uint32_t lower_phases3[12] = {0};
  uint32_t middle_phases3[12] = {0};
  uint32_t upper_phases3[12] = {0};

 


  uint64_t ss;
  uint8_t volume;
  float pan;
  int32_t Vout;
  while (1)
  {
    xSemaphoreTake(sampleBufferSemaphore, portMAX_DELAY);
    
    for (uint32_t writeCtr = 0; writeCtr < SAMPLE_BUFFER_SIZE; writeCtr++)
    {
      // TODO check if g_note_states is ok to be accessed here

      // doing lowe keyboard
      ss = g_ss;
      Vout = 0;
      volume = global_knob3;
      pan = (float)global_knob4 / 8.0;
    
      
      int8_t wave_type = global_knob2;
      int8_t octave_shift = global_knob0;
      bool pos_shift = (octave_shift > 0) ? true : false;

      int32_t stepOffset = bendStep + vibStep;
      
      if (wave_type == 0)
        {
          // sine wave
          float_t angle;
          for (int i = 0; i < 12; ++i)
          {
            if (ss & 1)
            {
              lower_phases0[i] += (pos_shift ? (stepSizes[i] << (octave_shift - 1)) : (stepSizes[i] >> (1 - octave_shift))) + stepOffset;

              // Vout += ((sin_lut[lower_phases0[i] >> 22]) >> 24) - 128;
              Vout += (sin_lut[lower_phases0[i] >> 22]) * 128;
            }

            if (ss & 0x1000)
            {
              middle_phases0[i] += (pos_shift ? (stepSizes[i] << (octave_shift)) : (stepSizes[i] >> (-octave_shift)));
              // Vout += (sin_lut[middle_phases0[i] >> 22] >> 24) - 128;
              Vout += sin_lut[middle_phases0[i] >> 22] * 128;
            }

            if (ss & 0x1000000)
            {
              upper_phases3[i] += (pos_shift ? (stepSizes[i] << (1 + octave_shift)) : ((stepSizes[i] << 1) >> (-octave_shift))) + stepOffset;
              // Vout += (sin_lut[upper_phases0[i] >> 22] >> 24) - 128;
              Vout += sin_lut[upper_phases0[i] >> 22] * 128;
            }

            ss = ss >> 1;
          }
        }

      else if (wave_type == 1)
        {
          // triangle wave
          for (int i = 0; i < 12; ++i)
          {
            if (ss & 1)
            {
              lower_phases1[i] += (pos_shift ? (stepSizes[i] << (octave_shift - 1)) : (stepSizes[i] >> (1 - octave_shift))) + stepOffset;

              Vout += ((int)(lower_phases1[i] >> 31) & 1) ? -(int)(lower_phases1[i] >> 24) + 128 : (int)(lower_phases1[i] >> 24) - 128;
              // TEMP FIX
              Vout += 64;
            }

            if (ss & 0x1000)
            {
              middle_phases1[i] += (pos_shift ? (stepSizes[i] << (octave_shift)) : (stepSizes[i] >> (-octave_shift))) + stepOffset;
              Vout += ((int)(middle_phases1[i] >> 31) & 1) ? -(int)(middle_phases1[i] >> 24) + 128 : (int)(middle_phases1[i] >> 24) - 128;
              Vout += 64;
            }

            if (ss & 0x1000000)
            {
              upper_phases1[i] += (pos_shift ? (stepSizes[i] << (1 + octave_shift)) : ((stepSizes[i] << 1) >> (-octave_shift))) + stepOffset;
              Vout += ((int)(upper_phases1[i] >> 31) & 1) ? -(int)(upper_phases1[i] >> 24) + 128 : (int)(upper_phases1[i] >> 24) - 128;
              Vout += 64;
            }

            ss = ss >> 1;
          }
        }
        else if (wave_type == 2)
        {
          // sawtooth
          for (int i = 0; i < 12; ++i)
          {
            if (ss & 1)
            {
              lower_phases2[i] += (pos_shift ? (stepSizes[i] << (octave_shift - 1)) : (stepSizes[i] >> (1 - octave_shift))) + stepOffset;
              Vout += ((int)(lower_phases2[i] >> 24) - 128);
            }

            if (ss & 0x1000)
            {
              middle_phases2[i] += (pos_shift ? (stepSizes[i] << (octave_shift)) : (stepSizes[i] >> (-octave_shift))) + stepOffset;
              Vout += ((int)(middle_phases2[i] >> 24) - 128);
            }

            if (ss & 0x1000000)
            {
              upper_phases2[i] += (pos_shift ? (stepSizes[i] << (1 + octave_shift)) : ((stepSizes[i] << 1) >> (-octave_shift))) + stepOffset;
              Vout += ((int)(upper_phases2[i] >> 24) - 128);
            }

            ss = ss >> 1;
          }
        }

        else if (wave_type == 3)
        {
          // pulse wave

          for (int i = 0; i < 12; ++i)
          {
            //if (ss & 1)
            //{
              lower_phases3[i] += (pos_shift ? (stepSizes[i] << (octave_shift - 1)) : (stepSizes[i] >> (1 - octave_shift))) + stepOffset;
              Vout += (((lower_phases3[i] >> 31) & 1) ? -255 : 255) >> 2;
            //}

            //if (ss & 0x1000)
            //{
              middle_phases3[i] += (pos_shift ? (stepSizes[i] << (octave_shift)) : (stepSizes[i] >> (-octave_shift))) + stepOffset;
              Vout += (((middle_phases3[i] >> 31) & 1) ? -255 : 255) >> 2;
            //}

            //if (ss & 0x1000000)
            //{
              upper_phases3[i] += (pos_shift ? (stepSizes[i] << (1 + octave_shift)) : ((stepSizes[i] << 1) >> (-octave_shift))) + stepOffset;
              Vout += (((upper_phases3[i] >> 31) & 1) ? -255 : 255)>> 2;
            //}

            ss = ss >> 1;
          }
        }

        Vout >>= (8 - volume);
        
      if (Vout > 127)
      {
        Vout = 127;
      }
      else if (Vout < -128)
      {
        Vout = -128;
      }



      if(numberOfKeyboards == 2){
        if(keyboardIndex == 0){
          Vout = Vout * (1-pan);
        }
        else{
          Vout = Vout * pan;
        }
      }
      if(numberOfKeyboards == 3){
        if(keyboardIndex == 0){
          Vout = Vout * (1-pan);
        }
        else if(keyboardIndex == 2){
          Vout = Vout *pan;
        }
      }

      if (writeBuffer1)
        sampleBuffer1[writeCtr] = Vout + 128;
      else
        sampleBuffer0[writeCtr] = Vout + 128;
    }
  }
}

void scanOtherBoardsTask(void *pvParameters)
{
  // CAN Message Variables
  uint32_t ID;
  uint8_t RX_Message[8] = {0};

  // Timing for the task
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    while (CAN_CheckRXLevel())
    {
      CAN_RX(ID, RX_Message);
      xSemaphoreTake(notesArrayMutex, portMAX_DELAY);
     
      g_note_states[RX_Message[0]] = ((RX_Message[3] & 0xf) << 8) + ((RX_Message[2] & 0xf) << 4) + (RX_Message[1] & 0xf);
      int8_t tempknob0, tempknob1, tempknob2, tempknob3;
      
      tempknob0 = RX_Message[4];
      tempknob1 = RX_Message[5];
      tempknob2 = RX_Message[6];
      tempknob3 = RX_Message[7];
      
      if(RX_Message[0] == 0){
        global_knob0 = tempknob0;
        global_knob1 = tempknob1;
        global_knob2 = tempknob2;
        global_knob3 = tempknob3;
      }
      if(RX_Message[0] == 1){
        global_knob4 = tempknob0;
        global_knob5 = tempknob1;
        global_knob6 = tempknob2;
        global_knob7 = tempknob3;
      }
      else{
        global_knob8 = tempknob0;
        global_knob9 = tempknob1;
        global_knob10 = tempknob2;
        global_knob11 = tempknob3;          
      }
    
      xSemaphoreGive(notesArrayMutex);
    }
  }
}

void drawKnobLevel(int xCoordinate, uint8_t knob_value){
  for(int i = 0; i < knob_value; i++){
    u8g2.drawLine(xCoordinate, 30-2*i, xCoordinate+15, 30-2*i);
    u8g2.drawLine(xCoordinate, 29-2*i, xCoordinate+15, 29-2*i);
    drawKnobLevel(5, global_knob0);
  }
}
void drawWaveform(uint8_t knob2rotation)
{
  if (knob2rotation == 0)
  {
    // sine waveform
    for (int i = 0; i < 40; i++)
    {
      float angle = i * 9;
      int x = 25 + sin(angle * 3.14159 / 180) * 7;
      u8g2.drawPixel(5 + i, x);
      // u8g2.drawStr(30, 10, ": Sine");
    }
  }
  else if (knob2rotation == 1)
  {
    // triangle waveform
    u8g2.drawLine(5, 27, 15, 17);
    u8g2.drawLine(15, 17, 25, 27);
    u8g2.drawLine(25, 27, 35, 17);
    u8g2.drawLine(35, 17, 45, 27);
    // u8g2.drawStr(90, 10, "Triangle");
  }
  else if (knob2rotation == 2)
  {
    // sawtooth waveform
    u8g2.drawLine(5, 27, 15, 17);
    u8g2.drawLine(15, 17, 15, 27);
    u8g2.drawLine(15, 27, 25, 17);
    u8g2.drawLine(25, 17, 25, 27);
    u8g2.drawLine(25, 27, 35, 17);
    // u8g2.drawStr(90, 10, "Sawtooth");
  }
  else if (knob2rotation == 3)
  {
    // pulse (square) waveform
    u8g2.drawLine(5, 27, 10, 27);
    u8g2.drawLine(10, 27, 10, 17);
    u8g2.drawLine(10, 17, 15, 17);
    u8g2.drawLine(15, 17, 15, 27);
    u8g2.drawLine(15, 27, 20, 27);
    u8g2.drawLine(20, 27, 20, 17);
    u8g2.drawLine(20, 17, 25, 17);
    u8g2.drawLine(25, 17, 25, 27);
    u8g2.drawLine(25, 27, 30, 27);
    // u8g2.drawStr(90, 10, "Pulse");
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
    u8g2.clearBuffer();                   // clear the internal memory
    u8g2.setFont(u8g2_font_profont10_tf); // choose a suitable font

    // only show main volume on first keyboard
    if (keyboardIndex == 0)
    {
      uint8_t knob3rotation = global_knob3;
      uint8_t knob2rotation = global_knob2;

      // Knob 3 (volume)
      //u8g2.drawStr(80, 10, "Vol"); // write something to the internal memory
      u8g2.setCursor(70,10);
      u8g2.print(vibStep, DEC);

      // Knob 2 (waveform)
      u8g2.drawStr(5, 10, "Waveform"); // write something to the internal memory
      drawWaveform(knob2rotation);

  

      // note showing
      u8g2.drawStr(80, 30, "Note"); // write something to the internal memory
      // u8g2.drawStr(120, 30, notes[note]);

      u8g2.setCursor(105, 30);
      xSemaphoreTake(notesArrayMutex, portMAX_DELAY);
      u8g2.print(g_ss, HEX);
      xSemaphoreGive(notesArrayMutex);

      // note showing
      // u8g2.drawStr(2, 30, notes[note]);
    }else if(keyboardIndex == 1){
      u8g2.drawStr(5,10,"NON");
      u8g2.drawStr(40,10,"PAN");
      u8g2.drawStr(75,10,"NON");
      u8g2.drawStr(110,10,"NON");
      drawKnobLevel(5, global_knob4);
      drawKnobLevel(40, global_knob5);
      drawKnobLevel(75, global_knob6);
      drawKnobLevel(110, global_knob7);
    }
    else{
      u8g2.drawStr(5,10,"ATK");
      u8g2.drawStr(40,10,"DEC");
      u8g2.drawStr(75,10,"SUS");
      u8g2.drawStr(110,10,"REL");
      drawKnobLevel(5, global_knob8);
      drawKnobLevel(40, global_knob9);
      drawKnobLevel(75, global_knob10);
      drawKnobLevel(110, global_knob11);
    }

    u8g2.print(global_knob11, DEC);
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

  uint8_t knob2keymatrix, knob3keymatrix, knob1keymatrix, knob0keymatrix;
  uint16_t toAnd, keys;
  bool pressed;
  uint8_t TX_Message[8] = {0};
  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // use mutex to access keyarray
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    for (int i = 0; i < 7; ++i)
    {
      setRow(i);
      digitalWrite(REN_PIN, 1); // Enable selected row
      delayMicroseconds(3);
      keyArray[i] = readCols();
      digitalWrite(REN_PIN, 0);
    }
    xSemaphoreGive(keyArrayMutex);

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    keys = (keyArray[2] << 8) + (keyArray[1] << 4) + keyArray[0];


    //knob key matrices
    knob3keymatrix = keyArray[3] & 0x03;
    knob2keymatrix = (keyArray[3] & 0x0C) >> 2;
    knob1keymatrix = keyArray[4] & 0x03;
    knob0keymatrix = (keyArray[4] & 0x0C) >> 2;
    xSemaphoreGive(keyArrayMutex);
    local_knob3.update_rotation(knob3keymatrix);
    local_knob2.update_rotation(knob2keymatrix);
    local_knob1.update_rotation(knob1keymatrix);
    local_knob0.update_rotation(knob0keymatrix);
    
    if(keyboardIndex ==0){
      global_knob0 = local_knob0.get_rotation_atomic();
      global_knob1 = local_knob1.get_rotation_atomic();
      global_knob2 = local_knob2.get_rotation_atomic();
      global_knob3 = local_knob3.get_rotation_atomic();
    }
    else if(keyboardIndex ==1){
      global_knob4 = local_knob0.get_rotation_atomic();
      global_knob5 = local_knob1.get_rotation_atomic();
      global_knob6 = local_knob2.get_rotation_atomic();
      global_knob7 = local_knob3.get_rotation_atomic();
    }
    else{
      global_knob8 = local_knob0.get_rotation_atomic();
      global_knob9 = local_knob1.get_rotation_atomic();
      global_knob10 = local_knob2.get_rotation_atomic();
      global_knob11 = local_knob3.get_rotation_atomic();
    }
    

    // note_states represents a 12-bit state of all notes
    uint16_t note_states = 0;
    uint16_t toAnd = 1;
    bool pressed = false;

    for (int i = 0; i < 12; ++i)
    {
      if (!((keys & toAnd) & 0xfff))
      {
        //envelope
        if (!envActive[i]){
          envActive[i] = true;
          startEnvelopeTask(i);
        }
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
    uint8_t temp_knob0, temp_knob1, temp_knob2, temp_knob3;
    temp_knob0 = local_knob0.get_rotation_atomic();
    temp_knob1 = local_knob1.get_rotation_atomic();
    temp_knob2 = local_knob2.get_rotation_atomic();
    temp_knob3 = local_knob3.get_rotation_atomic();
    TX_Message[4] = temp_knob0 & 0xf;
    TX_Message[5] = temp_knob1 & 0xf;
    TX_Message[6] = temp_knob2 & 0xf;
    TX_Message[7] = temp_knob3 & 0xf;
    if (numberOfKeyboards > 1)
    {
      CAN_TX(0x123, TX_Message);
    }

    // Put in mutex?
   

    __atomic_store_n(&bendStep, 2 * 8080 * (512 - analogRead(JOYX_PIN)), __ATOMIC_RELAXED);

    xSemaphoreTake(notesArrayMutex, portMAX_DELAY);
    g_note_states[keyboardIndex] = note_states; // need to protect this at some point!!
    g_ss = (uint64_t)g_note_states[0] + ((uint64_t)g_note_states[1] << 12) + ((uint64_t)g_note_states[2] << 24);
    xSemaphoreGive(notesArrayMutex);
  }
}

void handShake()
{

  uint32_t startup = millis();
  uint32_t current = startup;
  while (current < startup + 3000) // for 4 seconds
  {
    current = millis();
    // use mutex to access keyarray
    for (int i = 0; i < 7; ++i)
    {
      setRow(i);
      if (i == 5 || i == 6)
      {
        digitalWrite(OUT_PIN, 1);
      }
      // Set value to latch in DFF
      digitalWrite(REN_PIN, 1); // Enable selected row
      delayMicroseconds(3);
      keyArray[i] = readCols();
      digitalWrite(REN_PIN, 0);
    }
    if (!(keyArray[5] & 0b1000))
    {
      west = true;
    }
    if (!(keyArray[6] & 0b1000))
    {
      east = true;
    }
  }
  // transmit 1 if west and east. else transmit 0 or 2.
  uint8_t TX_Message[8] = {0};
  if (west && east)
  {
    TX_Message[0] = 1;
    CAN_TX(0x123, TX_Message);
    delay(500);
    CAN_TX(0x123, TX_Message); // send msg again in case of startup error or something
    numberOfKeyboards = 3;
    keyboardIndex = 1;
    return;
  }

  // TX_Message[0] = 2;
  uint8_t RX_Message[8] = {0};
  uint32_t ID;
  uint32_t start = millis();
  uint32_t current2 = start;

  while (current2 < (start + 3000))
  {
    current2 = millis();
    if (CAN_CheckRXLevel() > 0)
    {
      CAN_RX(ID, RX_Message);
      if (RX_Message[0] == 1)
      {
        numberOfKeyboards = 3;
        if (east)
        {
          keyboardIndex = 0;
        }
        else if (west)
        {
          keyboardIndex = 2;
        }
        return;
      }
    }
  }

  if (east || west)
  {
    numberOfKeyboards = 2;
    if (east)
    {
      keyboardIndex = 0;
    }
    else if (west)
    {
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
      256,                      /* Stack size in words, not bytes */
      NULL,                     /* Parameter passed into the task */
      5,                        /* Task priority */
      &sampleGenerationHandle); /* Pointer to store the task handle */

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
      scanKeysTask,     /* Function that implements the task */
      "scanKeys",       /* Text name for the task */
      64,               /* Stack size in words, not bytes */
      NULL,             /* Parameter passed into the task */
      4,                /* Task priority */
      &scanKeysHandle); /* Pointer to store the task handle */

  TaskHandle_t scanOtherBoards = NULL;
  xTaskCreate(
      scanOtherBoardsTask, /* Function that implements the task */
      "scanOtherBoards",   /* Text name for the task */
      64,                  /* Stack size in words, not bytes */
      NULL,                /* Parameter passed into the task */
      3,                   /* Task priority */
      &scanKeysHandle);    /* Pointer to store the task handle */

  TaskHandle_t updateDisplayHandle = NULL;
  xTaskCreate(
      updateDisplayTask,     /* Function that implements the task */
      "updateDisplay",       /* Text name for the task */
      64,                    /* Stack size in words, not bytes */
      NULL,                  /* Parameter passed into the task */
      1,                     /* Task priority */
      &updateDisplayHandle); /* Pointer to store the task handle */

  TaskHandle_t setVibStep = NULL;
  xTaskCreate(
      setVibStepTask,     /* Function that implements the task */
      "setVibrato",       /* Text name for the task */
      64,                    /* Stack size in words, not bytes */
      NULL,                  /* Parameter passed into the task */
      2,                     /* Task priority */
      &setVibStep); /* Pointer to store the task handle */

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


  // Initialise CAN
  CAN_Init(false);
  setCANFilter(0x123, 0x7ff);
  CAN_Start();
  handShake();
  // setting local knob limits
  local_knob3.set_limits(0, 8);
  local_knob2.set_limits(0, 8);
  local_knob1.set_limits(0, 8);
  if(keyboardIndex == 0){
    local_knob0.set_limits(-3,3);
  }else{
    local_knob0.set_limits(0,8);
  }
  
  Serial.println("finished handshaking");
  vTaskStartScheduler();
}

void loop()
{
}
