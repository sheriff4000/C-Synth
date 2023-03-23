#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include "../include/knob.hh"
#include <ES_CAN.h>

// #define DISABLE_THREADS 1
// #define DISABLE_ISRs 1

// #define TEST_SETVIBSTEPTASK 1
// #define TEST_SAMPLEGENERATIONTASK 1
// #define TEST_SCANOTHERBOARDSTASK 1
// #define TEST_UPDATEDISPLAYTASK 1
// #define TEST_SCANKEYSTASK 1
// #define TEST_PLAYLOOPTASK 1
// #define TEST_RECORDLOOPTASK 1
// #define TEST_METRONOMETASK 1
// #define TEST_SAMPLEISR

// mutex to handle synchronization bug
SemaphoreHandle_t keyArrayMutex;
SemaphoreHandle_t notesArrayMutex;
SemaphoreHandle_t sampleBufferSemaphore;

//  Pin definitions
//  Row select and enable
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

// sine wave lookup table
float_t sin_lut[1024] = {0, 0.00613588, 0.0122715, 0.0184067, 0.0245412, 0.0306748, 0.0368072, 0.0429383, 0.0490677, 0.0551952, 0.0613207, 0.0674439, 0.0735646, 0.0796824, 0.0857973, 0.091909, 0.0980171, 0.104122, 0.110222, 0.116319, 0.122411, 0.128498, 0.134581, 0.140658, 0.14673, 0.152797, 0.158858, 0.164913, 0.170962, 0.177004, 0.18304, 0.189069, 0.19509, 0.201105, 0.207111, 0.21311, 0.219101, 0.225084, 0.231058, 0.237024, 0.24298, 0.248928, 0.254866, 0.260794, 0.266713, 0.272621, 0.27852, 0.284408, 0.290285, 0.296151, 0.302006, 0.30785, 0.313682, 0.319502, 0.32531, 0.331106, 0.33689, 0.342661, 0.348419, 0.354164, 0.359895, 0.365613, 0.371317, 0.377007, 0.382683, 0.388345, 0.393992, 0.399624, 0.405241, 0.410843, 0.41643, 0.422, 0.427555, 0.433094, 0.438616, 0.444122, 0.449611, 0.455084, 0.460539, 0.465976, 0.471397, 0.476799, 0.482184, 0.48755, 0.492898, 0.498228, 0.503538, 0.50883, 0.514103, 0.519356, 0.52459, 0.529804, 0.534998, 0.540171, 0.545325, 0.550458, 0.55557, 0.560662, 0.565732, 0.570781, 0.575808, 0.580814, 0.585798, 0.59076, 0.595699, 0.600616, 0.605511, 0.610383, 0.615232, 0.620057, 0.624859, 0.629638, 0.634393, 0.639124, 0.643832, 0.648514, 0.653173, 0.657807, 0.662416, 0.667, 0.671559, 0.676093, 0.680601, 0.685084, 0.689541, 0.693971, 0.698376, 0.702755, 0.707107, 0.711432, 0.715731, 0.720003, 0.724247, 0.728464, 0.732654, 0.736817, 0.740951, 0.745058, 0.749136, 0.753187, 0.757209, 0.761202, 0.765167, 0.769103, 0.77301, 0.776888, 0.780737, 0.784557, 0.788346, 0.792107, 0.795837, 0.799537, 0.803208, 0.806848, 0.810457, 0.814036, 0.817585, 0.821103, 0.824589, 0.828045, 0.83147, 0.834863, 0.838225, 0.841555, 0.844854, 0.84812, 0.851355, 0.854558, 0.857729, 0.860867, 0.863973, 0.867046, 0.870087, 0.873095, 0.87607, 0.879012, 0.881921, 0.884797, 0.88764, 0.890449, 0.893224, 0.895966, 0.898674, 0.901349, 0.903989, 0.906596, 0.909168, 0.911706, 0.91421, 0.916679, 0.919114, 0.921514, 0.92388, 0.92621, 0.928506, 0.930767, 0.932993, 0.935184, 0.937339, 0.939459, 0.941544, 0.943593, 0.945607, 0.947586, 0.949528, 0.951435, 0.953306, 0.955141, 0.95694, 0.958703, 0.960431, 0.962121, 0.963776, 0.965394, 0.966976, 0.968522, 0.970031, 0.971504, 0.97294, 0.974339, 0.975702, 0.977028, 0.978317, 0.97957, 0.980785, 0.981964, 0.983105, 0.98421, 0.985278, 0.986308, 0.987301, 0.988258, 0.989177, 0.990058, 0.990903, 0.99171, 0.99248, 0.993212, 0.993907, 0.994565, 0.995185, 0.995767, 0.996313, 0.99682, 0.99729, 0.997723, 0.998118, 0.998476, 0.998795, 0.999078, 0.999322, 0.999529, 0.999699, 0.999831, 0.999925, 0.999981, 1, 0.999981, 0.999925, 0.999831, 0.999699, 0.999529, 0.999322, 0.999078, 0.998795, 0.998476, 0.998118, 0.997723, 0.99729, 0.99682, 0.996313, 0.995767, 0.995185, 0.994565, 0.993907, 0.993212, 0.99248, 0.99171, 0.990903, 0.990058, 0.989177, 0.988258, 0.987301, 0.986308, 0.985278, 0.98421, 0.983105, 0.981964, 0.980785, 0.97957, 0.978317, 0.977028, 0.975702, 0.974339, 0.97294, 0.971504, 0.970031, 0.968522, 0.966976, 0.965394, 0.963776, 0.962121, 0.960431, 0.958703, 0.95694, 0.955141, 0.953306, 0.951435, 0.949528, 0.947586, 0.945607, 0.943593, 0.941544, 0.939459, 0.937339, 0.935184, 0.932993, 0.930767, 0.928506, 0.92621, 0.92388, 0.921514, 0.919114, 0.916679, 0.91421, 0.911706, 0.909168, 0.906596, 0.903989, 0.901349, 0.898674, 0.895966, 0.893224, 0.890449, 0.88764, 0.884797, 0.881921, 0.879012, 0.87607, 0.873095, 0.870087, 0.867046, 0.863973, 0.860867, 0.857729, 0.854558, 0.851355, 0.84812, 0.844854, 0.841555, 0.838225, 0.834863, 0.83147, 0.828045, 0.824589, 0.821103, 0.817585, 0.814036, 0.810457, 0.806848, 0.803208, 0.799537, 0.795837, 0.792107, 0.788346, 0.784557, 0.780737, 0.776888, 0.77301, 0.769103, 0.765167, 0.761202, 0.757209, 0.753187, 0.749136, 0.745058, 0.740951, 0.736817, 0.732654, 0.728464, 0.724247, 0.720003, 0.715731, 0.711432, 0.707107, 0.702755, 0.698376, 0.693971, 0.689541, 0.685084, 0.680601, 0.676093, 0.671559, 0.667, 0.662416, 0.657807, 0.653173, 0.648514, 0.643832, 0.639124, 0.634393, 0.629638, 0.624859, 0.620057, 0.615232, 0.610383, 0.605511, 0.600616, 0.595699, 0.59076, 0.585798, 0.580814, 0.575808, 0.570781, 0.565732, 0.560662, 0.55557, 0.550458, 0.545325, 0.540171, 0.534998, 0.529804, 0.52459, 0.519356, 0.514103, 0.50883, 0.503538, 0.498228, 0.492898, 0.48755, 0.482184, 0.476799, 0.471397, 0.465976, 0.460539, 0.455084, 0.449611, 0.444122, 0.438616, 0.433094, 0.427555, 0.422, 0.41643, 0.410843, 0.405241, 0.399624, 0.393992, 0.388345, 0.382683, 0.377007, 0.371317, 0.365613, 0.359895, 0.354164, 0.348419, 0.342661, 0.33689, 0.331106, 0.32531, 0.319502, 0.313682, 0.30785, 0.302006, 0.296151, 0.290285, 0.284408, 0.27852, 0.272621, 0.266713, 0.260794, 0.254866, 0.248928, 0.24298, 0.237024, 0.231058, 0.225084, 0.219101, 0.21311, 0.207111, 0.201105, 0.19509, 0.189069, 0.18304, 0.177004, 0.170962, 0.164913, 0.158858, 0.152797, 0.14673, 0.140658, 0.134581, 0.128498, 0.122411, 0.116319, 0.110222, 0.104122, 0.0980171, 0.091909, 0.0857973, 0.0796824, 0.0735646, 0.0674439, 0.0613207, 0.0551952, 0.0490677, 0.0429383, 0.0368072, 0.0306748, 0.0245412, 0.0184067, 0.0122715, 0.00613588, 0, -0.00613588, -0.0122715, -0.0184067, -0.0245412, -0.0306748, -0.0368072, -0.0429383, -0.0490677, -0.0551952, -0.0613207, -0.0674439, -0.0735646, -0.0796824, -0.0857973, -0.091909, -0.0980171, -0.104122, -0.110222, -0.116319, -0.122411, -0.128498, -0.134581, -0.140658, -0.14673, -0.152797, -0.158858, -0.164913, -0.170962, -0.177004, -0.18304, -0.189069, -0.19509, -0.201105, -0.207111, -0.21311, -0.219101, -0.225084, -0.231058, -0.237024, -0.24298, -0.248928, -0.254866, -0.260794, -0.266713, -0.272621, -0.27852, -0.284408, -0.290285, -0.296151, -0.302006, -0.30785, -0.313682, -0.319502, -0.32531, -0.331106, -0.33689, -0.342661, -0.348419, -0.354164, -0.359895, -0.365613, -0.371317, -0.377007, -0.382683, -0.388345, -0.393992, -0.399624, -0.405241, -0.410843, -0.41643, -0.422, -0.427555, -0.433094, -0.438616, -0.444122, -0.449611, -0.455084, -0.460539, -0.465976, -0.471397, -0.476799, -0.482184, -0.48755, -0.492898, -0.498228, -0.503538, -0.50883, -0.514103, -0.519356, -0.52459, -0.529804, -0.534998, -0.540171, -0.545325, -0.550458, -0.55557, -0.560662, -0.565732, -0.570781, -0.575808, -0.580814, -0.585798, -0.59076, -0.595699, -0.600616, -0.605511, -0.610383, -0.615232, -0.620057, -0.624859, -0.629638, -0.634393, -0.639124, -0.643832, -0.648514, -0.653173, -0.657807, -0.662416, -0.667, -0.671559, -0.676093, -0.680601, -0.685084, -0.689541, -0.693971, -0.698376, -0.702755, -0.707107, -0.711432, -0.715731, -0.720003, -0.724247, -0.728464, -0.732654, -0.736817, -0.740951, -0.745058, -0.749136, -0.753187, -0.757209, -0.761202, -0.765167, -0.769103, -0.77301, -0.776888, -0.780737, -0.784557, -0.788346, -0.792107, -0.795837, -0.799537, -0.803208, -0.806848, -0.810457, -0.814036, -0.817585, -0.821103, -0.824589, -0.828045, -0.83147, -0.834863, -0.838225, -0.841555, -0.844854, -0.84812, -0.851355, -0.854558, -0.857729, -0.860867, -0.863973, -0.867046, -0.870087, -0.873095, -0.87607, -0.879012, -0.881921, -0.884797, -0.88764, -0.890449, -0.893224, -0.895966, -0.898674, -0.901349, -0.903989, -0.906596, -0.909168, -0.911706, -0.91421, -0.916679, -0.919114, -0.921514, -0.92388, -0.92621, -0.928506, -0.930767, -0.932993, -0.935184, -0.937339, -0.939459, -0.941544, -0.943593, -0.945607, -0.947586, -0.949528, -0.951435, -0.953306, -0.955141, -0.95694, -0.958703, -0.960431, -0.962121, -0.963776, -0.965394, -0.966976, -0.968522, -0.970031, -0.971504, -0.97294, -0.974339, -0.975702, -0.977028, -0.978317, -0.97957, -0.980785, -0.981964, -0.983105, -0.98421, -0.985278, -0.986308, -0.987301, -0.988258, -0.989177, -0.990058, -0.990903, -0.99171, -0.99248, -0.993212, -0.993907, -0.994565, -0.995185, -0.995767, -0.996313, -0.99682, -0.99729, -0.997723, -0.998118, -0.998476, -0.998795, -0.999078, -0.999322, -0.999529, -0.999699, -0.999831, -0.999925, -0.999981, -1, -0.999981, -0.999925, -0.999831, -0.999699, -0.999529, -0.999322, -0.999078, -0.998795, -0.998476, -0.998118, -0.997723, -0.99729, -0.99682, -0.996313, -0.995767, -0.995185, -0.994565, -0.993907, -0.993212, -0.99248, -0.99171, -0.990903, -0.990058, -0.989177, -0.988258, -0.987301, -0.986308, -0.985278, -0.98421, -0.983105, -0.981964, -0.980785, -0.97957, -0.978317, -0.977028, -0.975702, -0.974339, -0.97294, -0.971504, -0.970031, -0.968522, -0.966976, -0.965394, -0.963776, -0.962121, -0.960431, -0.958703, -0.95694, -0.955141, -0.953306, -0.951435, -0.949528, -0.947586, -0.945607, -0.943593, -0.941544, -0.939459, -0.937339, -0.935184, -0.932993, -0.930767, -0.928506, -0.92621, -0.92388, -0.921514, -0.919114, -0.916679, -0.91421, -0.911706, -0.909168, -0.906596, -0.903989, -0.901349, -0.898674, -0.895966, -0.893224, -0.890449, -0.88764, -0.884797, -0.881921, -0.879012, -0.87607, -0.873095, -0.870087, -0.867046, -0.863973, -0.860867, -0.857729, -0.854558, -0.851355, -0.84812, -0.844854, -0.841555, -0.838225, -0.834863, -0.83147, -0.828045, -0.824589, -0.821103, -0.817585, -0.814036, -0.810457, -0.806848, -0.803208, -0.799537, -0.795837, -0.792107, -0.788346, -0.784557, -0.780737, -0.776888, -0.77301, -0.769103, -0.765167, -0.761202, -0.757209, -0.753187, -0.749136, -0.745058, -0.740951, -0.736817, -0.732654, -0.728464, -0.724247, -0.720003, -0.715731, -0.711432, -0.707107, -0.702755, -0.698376, -0.693971, -0.689541, -0.685084, -0.680601, -0.676093, -0.671559, -0.667, -0.662416, -0.657807, -0.653173, -0.648514, -0.643832, -0.639124, -0.634393, -0.629638, -0.624859, -0.620057, -0.615232, -0.610383, -0.605511, -0.600616, -0.595699, -0.59076, -0.585798, -0.580814, -0.575808, -0.570781, -0.565732, -0.560662, -0.55557, -0.550458, -0.545325, -0.540171, -0.534998, -0.529804, -0.52459, -0.519356, -0.514103, -0.50883, -0.503538, -0.498228, -0.492898, -0.48755, -0.482184, -0.476799, -0.471397, -0.465976, -0.460539, -0.455084, -0.449611, -0.444122, -0.438616, -0.433094, -0.427555, -0.422, -0.41643, -0.410843, -0.405241, -0.399624, -0.393992, -0.388345, -0.382683, -0.377007, -0.371317, -0.365613, -0.359895, -0.354164, -0.348419, -0.342661, -0.33689, -0.331106, -0.32531, -0.319502, -0.313682, -0.30785, -0.302006, -0.296151, -0.290285, -0.284408, -0.27852, -0.272621, -0.266713, -0.260794, -0.254866, -0.248928, -0.24298, -0.237024, -0.231058, -0.225084, -0.219101, -0.21311, -0.207111, -0.201105, -0.19509, -0.189069, -0.18304, -0.177004, -0.170962, -0.164913, -0.158858, -0.152797, -0.14673, -0.140658, -0.134581, -0.128498, -0.122411, -0.116319, -0.110222, -0.104122, -0.0980171, -0.091909, -0.0857973, -0.0796824, -0.0735646, -0.0674439, -0.0613207, -0.0551952, -0.0490677, -0.0429383, -0.0368072, -0.0306748, -0.0245412, -0.0184067, -0.0122715, -0.00613588};

// Array of 3, twelve bit numbers for each keyboard
volatile uint16_t g_note_states[3];
volatile uint64_t g_ss;

// loopy loop
volatile uint16_t recorded_g_note_states[200][3];
volatile int number_of_used_slots;
volatile bool loop_play = false;
volatile bool loop_record = false;
volatile bool loopPlaying;
volatile int loopIndex;
volatile int endLoopIndex;

/// notes that are being played
const char *notes[13] = {"C", "C#", "D", "Eb", "E", "F", "F#", "G", "G#", "A", "Bb", "B", " "};

const char *knobTitles[12] = {"OCT", "NONE", "WAVE", "VOL", "PAN", "REC", "PLAY", "NONE", "ATTACK", "DECAY", "SUSTAIN", "NONE"};
volatile uint8_t note;

// bendy bendy
volatile uint32_t bendStep;

// metronomey
volatile bool metronomeBeep;

// vibby vibby
volatile int32_t vibStep;

// envylopey
volatile float noteMult[36] = {0.};
volatile bool envActive[36] = {false};

// local knobs
Knob local_knob0, local_knob1, local_knob2, local_knob3;

// global knobs
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

const int SAMPLE_BUFFER_SIZE = 128;
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

void setVibStepTask(void *pvParameters)
{
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // vib things
  float vibStepSize = 2049870.0 / 2.0; // 1366580 * 2;

  long int joyChange;
  float abs_joyChange;
  int i = 0;
  float t;
  float freq;
  float tempVibStep;

  while (1)
  {

// skip if testing so that task doesn't block
#ifndef TEST_SETVIBSTEPTASK
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
#endif

    joyChange = 512 - analogRead(JOYY_PIN);
    abs_joyChange = (joyChange < 0) ? -1.0 * joyChange : joyChange;

#ifndef TEST_SETVIBSTEPTASK
    if (abs_joyChange < 100)
#else
    if (0)
#endif
    {
      __atomic_store_n(&vibStep, 0, __ATOMIC_RELAXED);
      continue;
    }

    freq = 2500.0 * (abs_joyChange / 512.0);

    t = ((float)i) / 22000.0;

    tempVibStep = vibStepSize * sin(2 * 3.14 * freq * t);
    __atomic_store_n(&vibStep, (int32_t)tempVibStep, __ATOMIC_RELAXED);

    if (i == 22000)
    {
      i = 0;
    }
    i++;

#ifdef TEST_SETVIBSTEPTASK
    break;
#endif
  }
}

void keyPressExecution(void *pvParameters)
{
  // Note pressed
  uint8_t noteIdx = (int)pvParameters;

  // Envelope parameters
  uint32_t attack = 100 * __atomic_load_n(&global_knob7, __ATOMIC_RELAXED);
  uint32_t decay = 50 * __atomic_load_n(&global_knob8, __ATOMIC_RELAXED);
  float sustain = 1;
  uint32_t release = 100 * 50;

  // Measurements
  float voutMult = 0.0;
  int startTime = millis();
  int currentTime = startTime;
  float atkDif;

  // Safe handling of g_note_states
  uint16_t g_note_temp;
  xSemaphoreTake(notesArrayMutex, portMAX_DELAY);
  envActive[noteIdx] = true;
  noteMult[noteIdx] = voutMult;
  g_note_temp = g_note_states[noteIdx / 12];
  xSemaphoreGive(notesArrayMutex);

  // Starting attack
  while ((currentTime < startTime + attack) && ((g_note_temp & (1 << (noteIdx % 12))) != 0))
  {
    currentTime = millis();
    xSemaphoreTake(notesArrayMutex, portMAX_DELAY);
    g_note_temp = g_note_states[noteIdx / 12];
    atkDif = ((float)(currentTime - startTime) / (float)attack);
    voutMult = atkDif;
    noteMult[noteIdx] = voutMult;
    xSemaphoreGive(notesArrayMutex);
  }

  xSemaphoreTake(notesArrayMutex, portMAX_DELAY);
  envActive[noteIdx] = false;
  noteMult[noteIdx] = 1;
  xSemaphoreGive(notesArrayMutex);

  startTime = millis();
  currentTime = startTime;

  // Starting decay
  while ((currentTime < startTime + decay) && ((g_note_temp & (1 << (noteIdx % 12))) != 0))
  {
    currentTime = millis();
    xSemaphoreTake(notesArrayMutex, portMAX_DELAY);
    envActive[noteIdx] = true;
    g_note_temp = g_note_states[noteIdx / 12];
    voutMult = 1. - ((1.0 - sustain) * ((float)(currentTime - startTime) / (float)decay));
    noteMult[noteIdx] = voutMult;
    xSemaphoreGive(notesArrayMutex);
  }

  voutMult = sustain;
  xSemaphoreTake(notesArrayMutex, portMAX_DELAY);
  noteMult[noteIdx] = voutMult;
  g_note_temp = g_note_states[noteIdx / 12];
  envActive[noteIdx] = false;
  xSemaphoreGive(notesArrayMutex);

  // Starting sustain
  while ((g_note_temp & (1 << (noteIdx % 12))) != 0)
  {
    xSemaphoreTake(notesArrayMutex, portMAX_DELAY);
    envActive[noteIdx] = true;
    g_note_temp = g_note_states[noteIdx / 12];
    xSemaphoreGive(notesArrayMutex);
  }

  xSemaphoreTake(notesArrayMutex, portMAX_DELAY);
  envActive[noteIdx] = false;
  noteMult[noteIdx] = 0;
  xSemaphoreGive(notesArrayMutex);

  vTaskDelete(NULL);
}

// Create a envelope tast every time a key is pressed
void startEnvelopeTask(int noteIdx)
{
  TaskHandle_t envelopeTask = NULL;
  xTaskCreate(keyPressExecution, "enveloper", 256, (void *)noteIdx, 1, &envelopeTask);
}

// Sample interrupt (with double buffer)
void sampleISR()
{
  static uint32_t readCtr = 0;

#ifndef TEST_SAMPLEISR
  if (readCtr == SAMPLE_BUFFER_SIZE)
#else
  if (1)
#endif
  {
    readCtr = 0;
    writeBuffer1 = !writeBuffer1;
#ifndef TEST_SAMPLEISR
    xSemaphoreGiveFromISR(sampleBufferSemaphore, NULL);
#endif
  }

  if (writeBuffer1)
    analogWrite(OUTR_PIN, sampleBuffer0[readCtr++]);
  else
    analogWrite(OUTR_PIN, sampleBuffer1[readCtr++]);
}

// Generating sound task
void sampleGenerationTask(void *pvParameters)
{
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
#ifndef TEST_SAMPLEGENERATIONTASK
    xSemaphoreTake(sampleBufferSemaphore, portMAX_DELAY);
#endif

    // Adding to the buffer
    for (uint32_t writeCtr = 0; writeCtr < SAMPLE_BUFFER_SIZE; writeCtr++)
    {

      Vout = 0;
      volume = __atomic_load_n(&global_knob3, __ATOMIC_RELAXED);
      pan = (float)__atomic_load_n(&global_knob4, __ATOMIC_RELAXED) / 8.0;

      int8_t wave_type = __atomic_load_n(&global_knob2, __ATOMIC_RELAXED);
      int8_t octave_shift = __atomic_load_n(&global_knob0, __ATOMIC_RELAXED);
      bool pos_shift = (octave_shift > 0) ? true : false;

      // TODO: is bendStep a safe read? Atomic it?
      uint32_t bend = __atomic_load_n(&bendStep, __ATOMIC_RELAXED);
      int32_t vib = __atomic_load_n(&vibStep, __ATOMIC_RELAXED);
      int32_t stepOffset = bend + vib;

      xSemaphoreTake(notesArrayMutex, portMAX_DELAY);
      ss = g_ss;

// sine wave
#ifndef TEST_SAMPLEGENERATIONTASK
      if (wave_type == 0)
#else
      if (1)
#endif
      {
        float_t angle;
        for (int i = 0; i < 12; ++i)
        {
// Lower keyboard
#ifndef TEST_SAMPLEGENERATIONTASK
          if (ss & 1)
#else
          if (1)
#endif
          {
            lower_phases0[i] += ((pos_shift ? (stepSizes[i] << (octave_shift - 1)) : (stepSizes[i] >> (1 - octave_shift)))) + stepOffset;
            Vout += (sin_lut[lower_phases0[i] >> 22]) * 128 * (float)noteMult[i];
          }

// Middle keyboard
#ifndef TEST_SAMPLEGENERATIONTASK
          if (ss & 0x1000)
#else
          if (1)
#endif
          {
            middle_phases0[i] += ((pos_shift ? (stepSizes[i] << (octave_shift)) : (stepSizes[i] >> (-octave_shift)))) + stepOffset;
            Vout += sin_lut[middle_phases0[i] >> 22] * 128 * (float)noteMult[i + 12];
          }

// Upper keyboard
#ifndef TEST_SAMPLEGENERATIONTASK
          if (ss & 0x1000000)
#else
          if (1)
#endif
          {
            upper_phases3[i] += ((pos_shift ? (stepSizes[i] << (1 + octave_shift)) : ((stepSizes[i] << 1) >> (-octave_shift)))) + stepOffset;
            Vout += sin_lut[upper_phases0[i] >> 22] * 128 * (float)noteMult[i + 24];
          }

          ss = ss >> 1;
        }
      }
      // triangle wave
      else if (wave_type == 1)
      {
        for (int i = 0; i < 12; ++i)
        {
          // Lower keyboard
          if (ss & 1)
          {
            lower_phases1[i] += (pos_shift ? (stepSizes[i] << (octave_shift - 1)) : (stepSizes[i] >> (1 - octave_shift))) + stepOffset;
            Vout += (((int)(lower_phases1[i] >> 31) & 1) ? -(int)(lower_phases1[i] >> 24) + 128 : (int)(lower_phases1[i] >> 24)) * noteMult[i] - 128;
            Vout += 64;
          }

          // Middle keyboard
          if (ss & 0x1000)
          {
            middle_phases1[i] += (pos_shift ? (stepSizes[i] << (octave_shift)) : (stepSizes[i] >> (-octave_shift))) + stepOffset;
            Vout += (((int)(middle_phases1[i] >> 31) & 1) ? -(int)(middle_phases1[i] >> 24) + 128 : (int)(middle_phases1[i] >> 24)) * noteMult[i + 12] - 128;
            Vout += 64;
          }

          // Upper keyboard
          if (ss & 0x1000000)
          {
            upper_phases1[i] += (pos_shift ? (stepSizes[i] << (1 + octave_shift)) : ((stepSizes[i] << 1) >> (-octave_shift))) + stepOffset;
            Vout += (((int)(upper_phases1[i] >> 31) & 1) ? -(int)(upper_phases1[i] >> 24) + 128 : (int)(upper_phases1[i] >> 24)) * noteMult[i + 24] - 128;
            Vout += 64;
          }

          ss = ss >> 1;
        }
      }
      // sawtooth
      else if (wave_type == 2)
      {
        for (int i = 0; i < 12; ++i)
        {
          // Lower keyboard
          if (ss & 1)
          {
            lower_phases2[i] += (pos_shift ? (stepSizes[i] << (octave_shift - 1)) : (stepSizes[i] >> (1 - octave_shift))) + stepOffset;
            Vout += ((int)(lower_phases2[i] >> 24) - 128) * noteMult[i];
          }

          // Middle keyboard
          if (ss & 0x1000)
          {
            middle_phases2[i] += (pos_shift ? (stepSizes[i] << (octave_shift)) : (stepSizes[i] >> (-octave_shift))) + stepOffset;
            Vout += ((int)(middle_phases2[i] >> 24) - 128) * noteMult[i + 12];
          }

          // Upper keyboard
          if (ss & 0x1000000)
          {
            upper_phases2[i] += (pos_shift ? (stepSizes[i] << (1 + octave_shift)) : ((stepSizes[i] << 1) >> (-octave_shift))) + stepOffset;
            Vout += ((int)(upper_phases2[i] >> 24) - 128) * noteMult[i + 24];
          }

          ss = ss >> 1;
        }
      }
      // pulse wave
      else if (wave_type == 3)
      {
        for (int i = 0; i < 12; ++i)
        {
          // Lower keyboard
          if (ss & 1)
          {
            lower_phases3[i] += (pos_shift ? (stepSizes[i] << (octave_shift - 1)) : (stepSizes[i] >> (1 - octave_shift))) + stepOffset;
            Vout += ((((lower_phases3[i] >> 31) & 1) ? -255 : 255) >> 2) * noteMult[i];
          }

          // Middle keyboard
          if (ss & 0x1000)
          {
            middle_phases3[i] += (pos_shift ? (stepSizes[i] << (octave_shift)) : (stepSizes[i] >> (-octave_shift))) + stepOffset;
            Vout += ((((middle_phases3[i] >> 31) & 1) ? -255 : 255) >> 2) * noteMult[i + 12];
          }

          // Upper keyboard
          if (ss & 0x1000000)
          {
            upper_phases3[i] += (pos_shift ? (stepSizes[i] << (1 + octave_shift)) : ((stepSizes[i] << 1) >> (-octave_shift))) + stepOffset;
            Vout += ((((upper_phases3[i] >> 31) & 1) ? -255 : 255) >> 2) * noteMult[i + 24];
          }

          ss = ss >> 1;
        }
      }
      xSemaphoreGive(notesArrayMutex);

      // Volume shifting
      Vout >>= (8 - volume);

      // Vout capping
      if (Vout > 127)
      {
        Vout = 127;
      }
      else if (Vout < -128)
      {
        Vout = -128;
      }

// Panning (sound moving to/from different keyboard speaker)
// Only applicable for more than 1 keyboard
#ifndef TEST_SAMPLEGENERATIONTASK
      if (numberOfKeyboards == 2)
#else
      if (1)
#endif
      {
        if (keyboardIndex == 0)
        {
          Vout = Vout * (1 - pan);
        }
        else
        {
          Vout = Vout * pan;
        }
      }

#ifndef TEST_SAMPLEGENERATIONTASK
      if (numberOfKeyboards == 3)
#else
      if (0)
#endif
      {
        if (keyboardIndex == 0)
        {
          Vout = Vout * (1 - pan);
        }
        else if (keyboardIndex == 2)
        {
          Vout = Vout * pan;
        }
      }

      // Write to the buffer
      if (writeBuffer1)
        sampleBuffer1[writeCtr] = Vout + 128;
      else
        sampleBuffer0[writeCtr] = Vout + 128;
    }

#ifdef TEST_SAMPLEGENERATIONTASK
    break;
#endif
  }
}

// CAN stuff
void scanOtherBoardsTask(void *pvParameters)
{
  // Timing for the task
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // CAN Message Variables
  uint32_t ID;
  uint8_t RX_Message[8] = {0};

  while (1)
  {
#ifndef TEST_SCANOTHERBOARDSTASK
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
#endif

#ifndef TEST_SCANOTHERBOARDSTASK
    while (CAN_CheckRXLevel())
#else
    if (1)
#endif
    {
      CAN_RX(ID, RX_Message);
      uint16_t newNotes0, newNotes1, newNotes2;

      xSemaphoreTake(notesArrayMutex, portMAX_DELAY);
      g_note_states[RX_Message[0]] = ((RX_Message[3] & 0xf) << 8) + ((RX_Message[2] & 0xf) << 4) + (RX_Message[1] & 0xf);

      // Sending looping message to other boards
      if (keyboardIndex != 1)
      {
        __atomic_store_n(&loop_record, RX_Message[6], __ATOMIC_RELAXED);
        __atomic_store_n(&loop_play, RX_Message[7], __ATOMIC_RELAXED);
      }
      xSemaphoreGive(notesArrayMutex);

      // Knob values received from other boards
      int8_t tempknob0, tempknob1, tempknob2, tempknob3;
      tempknob0 = RX_Message[4] & 0xF;
      tempknob1 = (RX_Message[4] & 0xF0) >> 4;
      tempknob2 = RX_Message[5] & 0xF;
      tempknob3 = (RX_Message[5] & 0xF0) >> 4;

      if (RX_Message[0] == 0)
      {
        __atomic_store_n(&global_knob0, tempknob0, __ATOMIC_RELAXED);
        __atomic_store_n(&global_knob1, tempknob1, __ATOMIC_RELAXED);
        __atomic_store_n(&global_knob2, tempknob2, __ATOMIC_RELAXED);
        __atomic_store_n(&global_knob3, tempknob3, __ATOMIC_RELAXED);
      }
      if (RX_Message[0] == 1)
      {
        __atomic_store_n(&global_knob4, tempknob0, __ATOMIC_RELAXED);
        __atomic_store_n(&global_knob5, tempknob1, __ATOMIC_RELAXED);
        __atomic_store_n(&global_knob6, tempknob2, __ATOMIC_RELAXED);
        __atomic_store_n(&global_knob7, tempknob3, __ATOMIC_RELAXED);
      }
      else
      {
        __atomic_store_n(&global_knob8, tempknob0, __ATOMIC_RELAXED);
        __atomic_store_n(&global_knob9, tempknob1, __ATOMIC_RELAXED);
        __atomic_store_n(&global_knob10, tempknob2, __ATOMIC_RELAXED);
        __atomic_store_n(&global_knob11, tempknob3, __ATOMIC_RELAXED);
      }

      // Loop running
      xSemaphoreTake(notesArrayMutex, portMAX_DELAY);

#ifndef TEST_SCANOTHERBOARDSTASK
      if (loopPlaying && (keyboardIndex == 1))
#else
      if (1)
#endif
      {
        newNotes0 = (recorded_g_note_states[loopIndex][0]) & (~g_note_states[0]);
        newNotes1 = (recorded_g_note_states[loopIndex][1]) & (~g_note_states[1]);
        newNotes2 = (recorded_g_note_states[loopIndex][2]) & (~g_note_states[2]);

        int s = 1;
        for (int i = 0; i < 12; i++)
        {
#ifndef TEST_SCANOTHERBOARDSTASK
          if ((newNotes0 & s) != 0)
#else
          if (1)
#endif
          {
            envActive[i] = true;
            startEnvelopeTask(i);
          }

#ifndef TEST_SCANOTHERBOARDSTASK
          if ((newNotes1 & s) != 0)
#else
          if (1)
#endif
          {
            envActive[i + 12] = true;
            startEnvelopeTask(i + 12);
          }
#ifndef TEST_SCANOTHERBOARDSTASK
          if ((newNotes2 & s) != 0)
#else
          if (1)
#endif
          {
            envActive[i + 24] = true;
            startEnvelopeTask(i + 24);
          }

          s <<= 1;
        }
        g_note_states[0] |= recorded_g_note_states[loopIndex][0];
        g_note_states[1] |= recorded_g_note_states[loopIndex][1];
        g_note_states[2] |= recorded_g_note_states[loopIndex][2];
      }
      xSemaphoreGive(notesArrayMutex);
    }

#ifdef TEST_SCANOTHERBOARDSTASK
    break;
#endif
  }
}

// Display helper functions (knob bar)
void drawKnobLevel(int xCoordinate, uint8_t knob_value)
{
  for (int i = 0; i < knob_value; i++)
  {
    u8g2.drawLine(xCoordinate, 30 - 2 * i, xCoordinate + 15, 30 - 2 * i);
    u8g2.drawLine(xCoordinate, 29 - 2 * i, xCoordinate + 15, 29 - 2 * i);
    drawKnobLevel(5, __atomic_load_n(&global_knob0, __ATOMIC_RELAXED));
  }
}

// Display helper functions (Waveform)
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
    }
  }
  else if (knob2rotation == 1)
  {
    // triangle waveform
    u8g2.drawLine(5, 27, 15, 17);
    u8g2.drawLine(15, 17, 25, 27);
    u8g2.drawLine(25, 27, 35, 17);
    u8g2.drawLine(35, 17, 45, 27);
  }
  else if (knob2rotation == 2)
  {
    // sawtooth waveform
    u8g2.drawLine(5, 27, 15, 17);
    u8g2.drawLine(15, 17, 15, 27);
    u8g2.drawLine(15, 27, 25, 17);
    u8g2.drawLine(25, 17, 25, 27);
    u8g2.drawLine(25, 27, 35, 17);
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
  }
}

// Display helper functions (record button)
void recordButton()
{
  u8g2.drawCircle(47, 22, 5);
  u8g2.drawFilledEllipse(47, 22, 2.5, 2.5);
}

// Display helper functions (Play button)
void playButton()
{
  u8g2.drawTriangle(80, 25, 80, 15, 85, 20);
}

// Display helper functions (metronome square)
void metronomeDraw()
{
  u8g2.drawBox(115, 22.5, 10, 10);
}

// Updating the display task
void updateDisplayTask(void *pvParameters)
{
  int noteToDisplay;
  // Timing for the task
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  // infinite loop for this task
  while (1)
  {
#ifndef TEST_UPDATEDISPLAYTASK
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
#endif

    // Update display
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_profont10_tf);

// First keyboard display: Wavetype, volume, notepressed
#ifndef TEST_UPDATEDISPLAYTASK
    if (keyboardIndex == 0)
#else
    if (1)
#endif
    {
      uint8_t knob3rotation = __atomic_load_n(&global_knob3, __ATOMIC_RELAXED);
      uint8_t knob2rotation = __atomic_load_n(&global_knob2, __ATOMIC_RELAXED);

      // Knobs
      u8g2.drawStr(80, 10, "Volume"); // write something to the internal memory
      u8g2.setCursor(115, 10);
      u8g2.print(knob3rotation);

      // Knob 2 (waveform)
      u8g2.drawStr(5, 10, "Waveform"); // write something to the internal memory
#ifdef TEST_UPDATEDISPLAYTASK
      drawWaveform(0); // sine waveform
#else
      drawWaveform(knob2rotation);
#endif
    noteToDisplay = 12;
    xSemaphoreTake(notesArrayMutex, portMAX_DELAY);
    int s = 1;
    for(int i = 0; i < 12; i++){
      if((g_note_states[0] & s) || (g_note_states[1] & s) || (g_note_states[2] & s) ){
        note = i;
      }
      s <<= 1;
    }
    xSemaphoreGive(notesArrayMutex);
      // note showing
      //u8g2.drawStr(80, 30, "Note"); // write something to the internal memory // make back to 30
      u8g2.drawStr(60, 20, notes[note]);
    }
    // Second keyboard display: Panning, Record(record loop), Play (play loop), Attack level
    else if (keyboardIndex == 1)
    {
      u8g2.drawStr(5, 10, "PAN");
      u8g2.drawStr(40, 10, "REC");
      u8g2.drawStr(75, 10, "PLAY");
      u8g2.drawStr(110, 10, "ATK");

      uint8_t knob4rotation = __atomic_load_n(&global_knob4, __ATOMIC_RELAXED);
      uint8_t knob7rotation = __atomic_load_n(&global_knob7, __ATOMIC_RELAXED);
      drawKnobLevel(5, knob4rotation);
      drawKnobLevel(110, knob7rotation);

      // Displaying appropriate symbols based on record/play
      if (__atomic_load_n(&loop_record, __ATOMIC_RELAXED))
      {
        recordButton();
      }
      if (__atomic_load_n(&loop_play, __ATOMIC_RELAXED))
      {
        playButton();
      }
    }
    // Third keyboard display: Decay, Sustain, Tempo , Metronome display
    else
    {
      u8g2.drawStr(5, 10, "DEC");
      u8g2.drawStr(40, 10, "SUS");
      u8g2.drawStr(75, 10, "TMP");
      u8g2.drawStr(110, 10, "MET");

      // Knob levels for Decay/Sustain
      uint8_t knob8rotation = local_knob0.get_rotation();
      uint8_t knob9rotation = local_knob1.get_rotation();
      drawKnobLevel(5, knob8rotation);
      drawKnobLevel(40, knob9rotation);

      // setting tempo
      u8g2.setCursor(75, 30);
      u8g2.print(40 + 15 * __atomic_load_n(&global_knob10, __ATOMIC_RELAXED));
      // metronome square
      if (__atomic_load_n(&metronomeBeep, __ATOMIC_RELAXED))
      {
        metronomeDraw();
      }
    }
    u8g2.sendBuffer(); // transfer internal memory to the display

    // Toggle LED
    digitalToggle(LED_BUILTIN);

#ifdef TEST_UPDATEDISPLAYTASK
    break;
#endif
  }
}

// Scannign keys task
void scanKeysTask(void *pvParameters)
{
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // variables for decoding the knobs
  uint8_t knob3keymatrix, knob2keymatrix, knob1keymatrix, knob0keymatrix;
  uint16_t toAnd, keys;
  bool pressed;
  uint8_t TX_Message[8] = {0};

  while (1)
  {
#ifndef TEST_SCANKEYSTASK
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
#endif

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

    keys = (keyArray[2] << 8) + (keyArray[1] << 4) + keyArray[0];

#ifndef TEST_SCANKEYSTASK
    if (keyboardIndex == 1)
#else
    if (1)
#endif
    {
      __atomic_store_n(&loop_play, !(keyArray[5] & 1), __ATOMIC_RELAXED);
      __atomic_store_n(&loop_record, !(keyArray[6] & 2), __ATOMIC_RELAXED);
    }

    // knob key matrices
    knob3keymatrix = keyArray[3] & 0x03;
    knob2keymatrix = (keyArray[3] & 0x0C) >> 2;
    knob1keymatrix = keyArray[4] & 0x03;
    knob0keymatrix = (keyArray[4] & 0x0C) >> 2;
    xSemaphoreGive(keyArrayMutex);

    // update_rotation is an atomic update
    local_knob3.update_rotation(knob3keymatrix);
    local_knob2.update_rotation(knob2keymatrix);
    local_knob1.update_rotation(knob1keymatrix);
    local_knob0.update_rotation(knob0keymatrix);

    // assigning knobs appropriately for multiple keyboards
    if (keyboardIndex == 0)
    {
      __atomic_store_n(&global_knob0, local_knob0.get_rotation_atomic(), __ATOMIC_RELAXED);
      __atomic_store_n(&global_knob1, local_knob1.get_rotation_atomic(), __ATOMIC_RELAXED);
      __atomic_store_n(&global_knob2, local_knob2.get_rotation_atomic(), __ATOMIC_RELAXED);
      __atomic_store_n(&global_knob3, local_knob3.get_rotation_atomic(), __ATOMIC_RELAXED);
    }
    else if (keyboardIndex == 1)
    {
      __atomic_store_n(&global_knob4, local_knob0.get_rotation_atomic(), __ATOMIC_RELAXED);
      __atomic_store_n(&global_knob5, local_knob1.get_rotation_atomic(), __ATOMIC_RELAXED);
      __atomic_store_n(&global_knob6, local_knob2.get_rotation_atomic(), __ATOMIC_RELAXED);
      __atomic_store_n(&global_knob7, local_knob3.get_rotation_atomic(), __ATOMIC_RELAXED);
    }
    else
    {
      __atomic_store_n(&global_knob8, local_knob0.get_rotation_atomic(), __ATOMIC_RELAXED);
      __atomic_store_n(&global_knob9, local_knob1.get_rotation_atomic(), __ATOMIC_RELAXED);
      __atomic_store_n(&global_knob10, local_knob2.get_rotation_atomic(), __ATOMIC_RELAXED);
      __atomic_store_n(&global_knob11, local_knob3.get_rotation_atomic(), __ATOMIC_RELAXED);
    }

    // note_states represents a 12-bit state of all notes
    uint16_t note_states = 0;
    toAnd = 1;
    bool pressed = false;

    for (int i = 0; i < 12; ++i)
    {
#ifndef TEST_SCANKEYSTASK
      if (!((keys & toAnd) & 0xfff))
#else
      if (1)
#endif
      {
        // note pressed
        note_states += toAnd;
        note = i;
        pressed = true;
      }
      toAnd = toAnd << 1;
    }

    int msg_states = note_states;

    // Transmit messages to other boards
    TX_Message[0] = keyboardIndex;
    TX_Message[1] = msg_states & 0xf;
    msg_states = msg_states >> 4;
    TX_Message[2] = msg_states & 0xf;
    msg_states = msg_states >> 4;
    TX_Message[3] = msg_states & 0xf;

    // Sending knob messages
    uint8_t temp_knob0, temp_knob1, temp_knob2, temp_knob3;
    temp_knob0 = local_knob0.get_rotation_atomic();
    temp_knob1 = local_knob1.get_rotation_atomic();
    temp_knob2 = local_knob2.get_rotation_atomic();
    temp_knob3 = local_knob3.get_rotation_atomic();
    TX_Message[4] = ((temp_knob1 & 0xf) << 4) + (temp_knob0 & 0xf);
    TX_Message[5] = ((temp_knob3 & 0xf) << 4) + (temp_knob2 & 0xf);

// Sending loop information
#ifndef TEST_SCANKEYSTASK
    if (keyboardIndex == 1)
#else
    if (1)
#endif
    {
      TX_Message[6] = __atomic_load_n(&loop_record, __ATOMIC_RELAXED);
      TX_Message[7] = __atomic_load_n(&loop_play, __ATOMIC_RELAXED);
    }

#ifndef TEST_SCANKEYSTASK
    if (numberOfKeyboards > 1)
#else
    if (1)
#endif
    {
      CAN_TX(0x123, TX_Message);
    }

    __atomic_store_n(&bendStep, 2 * 8080 * (512 - analogRead(JOYX_PIN)), __ATOMIC_RELAXED);

    uint16_t newNotes0, newNotes1, newNotes2;

    xSemaphoreTake(notesArrayMutex, portMAX_DELAY);
    g_note_states[keyboardIndex] = note_states;

    // Envelope handling
    for (int i = 0; i < 12; i++)
    {
// Lower board envelope
#ifndef TEST_SCANKEYSTASK
      if (!envActive[i] && ((g_note_states[0] & (1 << i)) != 0))
#else
      if (1)
#endif
      {
        envActive[i] = true;
        startEnvelopeTask(i);
      }

// Middle board envelope
#ifndef TEST_SCANKEYSTASK
      if (!envActive[i + 12] && ((g_note_states[1] & (1 << i)) != 0))
#else
      if (1)
#endif
      {
        envActive[i + 12] = true;
        startEnvelopeTask(i + 12);
      }

// Upper board envelope
#ifndef TEST_SCANKEYSTASK
      if (!envActive[i + 24] && ((g_note_states[2] & (1 << i)) != 0))
#else
      if (1)
#endif
      {
        envActive[i + 24] = true;
        startEnvelopeTask(i + 24);
      }
    }

// Further loop handling
#ifndef TEST_SCANKEYSTASK
    if (__atomic_load_n(&loopPlaying, __ATOMIC_RELAXED) && (keyboardIndex == 1))
#else
    if (1)
#endif
    {
      newNotes0 = (recorded_g_note_states[loopIndex][0]) & (~g_note_states[0]);
      newNotes1 = (recorded_g_note_states[loopIndex][1]) & (~g_note_states[1]);
      newNotes2 = (recorded_g_note_states[loopIndex][2]) & (~g_note_states[2]);

      int s = 1;
      for (int i = 0; i < 12; i++)
      {
#ifndef TEST_SCANKEYSTASK
        if ((newNotes0 & s) != 0)
#else
        if (1)
#endif
        {
          envActive[i] = true;
          startEnvelopeTask(i);
        }

#ifndef TEST_SCANKEYSTASK
        if ((newNotes1 & s) != 0)
#else
        if (1)
#endif
        {
          envActive[i + 12] = true;
          startEnvelopeTask(i + 12);
        }

#ifndef TEST_SCANKEYSTASK
        if ((newNotes2 & s) != 0)
#else
        if (1)
#endif
        {
          envActive[i + 24] = true;
          startEnvelopeTask(i + 24);
        }
        s <<= 1;
      }

      g_note_states[0] |= recorded_g_note_states[loopIndex][0];
      g_note_states[1] |= recorded_g_note_states[loopIndex][1];
      g_note_states[2] |= recorded_g_note_states[loopIndex][2];
    }

    g_ss = (uint64_t)g_note_states[0] + ((uint64_t)g_note_states[1] << 12) + ((uint64_t)g_note_states[2] << 24);

    xSemaphoreGive(notesArrayMutex);

#ifdef TEST_SCANKEYSTASK
    break;
#endif
  }
}

// Looping (play) task
void playLoopTask(void *pvParameters)
{
  const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  __atomic_store_n(&loopPlaying, false, __ATOMIC_RELAXED);

  int currentIndexPlaying = 0;
  loopIndex = 0;
  bool button_pressed;

  while (1)
  {
#ifndef TEST_PLAYLOOPTASK
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
#endif

#ifndef TEST_PLAYLOOPTASK
    if (!__atomic_load_n(&loop_record, __ATOMIC_RELAXED))
#else
    if (1)
#endif
    {
      button_pressed = __atomic_load_n(&loop_play, __ATOMIC_RELAXED);
    }
    else
    {
      button_pressed = false;
    }

#ifndef TEST_PLAYLOOPTASK
    if (!button_pressed)
#else
    if (0)
#endif
    {
      currentIndexPlaying = 0;
      __atomic_store_n(&loopPlaying, false, __ATOMIC_RELAXED);
    }
    else
    {
      __atomic_store_n(&loopPlaying, true, __ATOMIC_RELAXED);
      loopIndex = currentIndexPlaying;
      currentIndexPlaying++;
      if (currentIndexPlaying >= __atomic_load_n(&endLoopIndex, __ATOMIC_RELAXED))
      {
        currentIndexPlaying = 0;
      }
    }

#ifdef TEST_PLAYLOOPTASK
    break;
#endif
  }
}

// Looping (record) task
void recordLoopTask(void *pvParameters)
{
  const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  int currentIndexRecording = 0;
  bool button_pressed;
  endLoopIndex = 200;
  while (1)
  {
#ifndef TEST_RECORDLOOPTASK
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
#endif

    button_pressed = __atomic_load_n(&loop_record, __ATOMIC_RELAXED);

#ifndef TEST_RECORDLOOPTASK
    if (!button_pressed)
#else
    if (0)
#endif
    {
      currentIndexRecording = 0;
    }
    else
    {
      xSemaphoreTake(notesArrayMutex, portMAX_DELAY);

#ifndef TEST_RECORDLOOPTASK
      if (currentIndexRecording == 0)
#else
      if (1)
#endif
      {
        for (int i = 0; i < 100; ++i)
        {
          recorded_g_note_states[i][0] = 0;
          recorded_g_note_states[i][1] = 0;
          recorded_g_note_states[i][2] = 0;
        }
      }
      if (currentIndexRecording < 200)
      {
        endLoopIndex = currentIndexRecording;
        recorded_g_note_states[currentIndexRecording][0] = g_note_states[0];
        recorded_g_note_states[currentIndexRecording][1] = g_note_states[1];
        recorded_g_note_states[currentIndexRecording][2] = g_note_states[2];
        currentIndexRecording += 1;
      }
      xSemaphoreGive(notesArrayMutex);
    }

#ifdef TEST_RECORDLOOPTASK
    break;
#endif
  }
}

// Mentronome task
void metronomeTask(void *pvParameters)
{
#ifndef TEST_METRONOMETASK
  if (keyboardIndex != 2)
#else
  if (0)
#endif
  {
    vTaskDelete(NULL);
  }

  __atomic_store_n(&metronomeBeep, false, __ATOMIC_RELAXED);
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  int numberOfCycles = 15;
  int currentCycle = 0;
  while (1)
  {
#ifndef TEST_METRONOMETASK
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
#endif
    numberOfCycles = (600) / (40 + 15 * __atomic_load_n(&global_knob10, __ATOMIC_RELAXED));
    currentCycle += 1;

#ifndef TEST_METRONOMETASK
    if (currentCycle == numberOfCycles - 3)
#else
    if (0)
#endif
    {
      __atomic_store_n(&metronomeBeep, true, __ATOMIC_RELAXED);
    }

#ifndef TEST_METRONOMETASK
    if (currentCycle >= numberOfCycles)
#else
    if (1)
#endif
    {
      __atomic_store_n(&metronomeBeep, false, __ATOMIC_RELAXED);
      currentCycle = 0;
    }

#ifdef TEST_METRONOMETASK
    break;
#endif
  }
}

void handShake()
{
  // for three seconds, check for neighbouring keyboards
  uint32_t startup = millis();
  while (millis() < startup + 3000)
  {
    for (int i = 0; i < 7; ++i)
    {
      setRow(i);
      // Set value to latch in DFF
      if (i == 5 || i == 6)
      {
        digitalWrite(OUT_PIN, 1);
      }

      digitalWrite(REN_PIN, 1);
      delayMicroseconds(3);
      keyArray[i] = readCols();
      digitalWrite(REN_PIN, 0);
    }

    // detect neighbouring keyboards
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

  uint8_t RX_Message[8] = {0};
  uint32_t ID;
  uint32_t start = millis();

  while (millis() < (start + 3000))
  {
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
#ifndef DISABLE_THREADS
  TaskHandle_t sampleGenerationHandle = NULL;
  xTaskCreate(sampleGenerationTask, "sampleGeneration", 256, NULL, 8, &sampleGenerationHandle);

  TaskHandle_t loopRecordHandle = NULL;
  xTaskCreate(recordLoopTask, "recordLoop", 256, NULL, 2, &loopRecordHandle);

  TaskHandle_t loopPlayHandle = NULL;
  xTaskCreate(playLoopTask, "playLoop", 256, NULL, 3, &loopPlayHandle);

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(scanKeysTask, "scanKeys", 128, NULL, 7, &scanKeysHandle);

  TaskHandle_t scanOtherBoards = NULL;
  xTaskCreate(scanOtherBoardsTask, "scanOtherBoards", 64, NULL, 6, &scanKeysHandle);

  TaskHandle_t updateDisplayHandle = NULL;
  xTaskCreate(updateDisplayTask, "updateDisplay", 64, NULL, 1, &updateDisplayHandle);

  TaskHandle_t setVibStep = NULL;
  xTaskCreate(setVibStepTask, "setVibrato", 64, NULL, 4, &setVibStep);

  TaskHandle_t metronome = NULL;
  xTaskCreate(metronomeTask, "metronome", 64, NULL, 5, &metronome);
#endif

  keyArrayMutex = xSemaphoreCreateMutex();
  notesArrayMutex = xSemaphoreCreateMutex();
  sampleBufferSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(sampleBufferSemaphore);

  // Timer
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);

  // Interrupt to execute the note playing
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);

#ifndef DISABLE_ISRs
  sampleTimer->attachInterrupt(sampleISR);
#endif
  sampleTimer->resume();

  // Initialise CAN
  CAN_Init(false);
  setCANFilter(0x123, 0x7ff);
  CAN_Start();
  handShake();

  // setting local knob limits
  local_knob3.set_limits(0, 8);
  
  local_knob1.set_limits(0, 8);
  if (keyboardIndex == 0){
    local_knob0.set_limits(-3, 3);
    local_knob2.set_limits(0, 3);
  }
  else{
    local_knob0.set_limits(0, 8);
    local_knob2.set_limits(0, 8);
  }

#ifdef TEST_SETVIBSTEPTASK
  uint32_t startTime = micros();
  for (int iter = 0; iter < 32; iter++)
  {
    setVibStepTask(NULL);
  }
  Serial.println(micros() - startTime);
  while (1)
    ;
#endif

#ifdef TEST_SAMPLEGENERATIONTASK
  uint32_t startTime = micros();
  for (int iter = 0; iter < 32; iter++)
  {
    sampleGenerationTask(NULL);
  }
  Serial.println(micros() - startTime);
  while (1)
    ;
#endif

#ifdef TEST_SCANOTHERBOARDSTASK
  uint32_t startTime = micros();
  for (int iter = 0; iter < 32; iter++)
  {
    scanOtherBoardsTask(NULL);
  }
  Serial.println(micros() - startTime);
  while (1)
    ;
#endif

#ifdef TEST_UPDATEDISPLAYTASK
  uint32_t startTime = micros();
  for (int iter = 0; iter < 32; iter++)
  {
    updateDisplayTask(NULL);
  }
  Serial.println(micros() - startTime);
  while (1)
    ;
#endif

#ifdef TEST_SCANKEYSTASK
  uint32_t startTime = micros();
  for (int iter = 0; iter < 32; iter++)
  {
    scanKeysTask(NULL);
  }
  Serial.println(micros() - startTime);
  while (1)
    ;
#endif

#ifdef TEST_PLAYLOOPTASK
  uint32_t startTime = micros();
  for (int iter = 0; iter < 32; iter++)
  {
    playLoopTask(NULL);
  }
  Serial.println(micros() - startTime);
  while (1)
    ;
#endif

#ifdef TEST_RECORDLOOPTASK
  uint32_t startTime = micros();
  for (int iter = 0; iter < 32; iter++)
  {
    recordLoopTask(NULL);
  }
  Serial.println(micros() - startTime);
  while (1)
    ;
#endif

#ifdef TEST_METRONOMETASK
  uint32_t startTime = micros();
  for (int iter = 0; iter < 32; iter++)
  {
    metronomeTask(NULL);
  }
  Serial.println(micros() - startTime);
  while (1)
    ;
#endif

#ifdef TEST_SAMPLEISR
  uint32_t startTime = micros();
  for (int iter = 0; iter < 32; iter++)
  {
    sampleISR();
  }
  Serial.println(micros() - startTime);
  while (1)
    ;
#endif

  vTaskStartScheduler();
}

void loop()
{
}