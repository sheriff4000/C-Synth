# Real time system analysis

## Minimum initiation interval and measured maximum execution time of each task

|         Task         | Initiation Interval (ms) | Maximum Execution time(us) |
| :------------------: | ------------------------ | :------------------------: |
| sampleGenerationTask | 5.818                    |          2462.25           |
|     scanKeysTask     | 20                       |         1315.1875          |
|   scanOtherBoards    | 100                      |         18380.8125         |
|    setVibStepTask    | 20                       |          165.375           |
|  updateDisplayTask   | 100                      |         17141.0312         |
|    metronomeTask     | 100                      |          0.46875           |
|     playLoopTask     | 50                       |          0.53125           |
|    recordLoopTask    | 50                       |           17.25            |
|      sampleISR       | 0.045455                 |           9.0625           |

## Critical analysis of rate monotonic scheduler

Critical analysis can be performed to see if a specific schedule will work. The latency ($L_n$) of the lowest-priority task ($\tau_n$) is x at the worst-instance in time. The following formula can be used to calculate whether the schedule will work:

$\Huge L_n = \sum_{i=1}[\frac{\tau_n}{\tau_i}]\cdot T_i \le \tau_n$

where $\tau_i$ is the initiation interval and $T_i$ is the execution time for task for task $i$.

$L_n$ was found to be (excluding the sampleISR) to be $85.27 \le 100$. 

## Total CPU utilisation

The CPU utilisation can be calculated through the following equation:

$\Huge CPU  Utilisation = \sum_{i=1}[\frac{T_i}{\tau_i}]$

The measured CPU utilisation under these tests was calculated using the results obtained in Section [## Minimum initiation interval and measured maximum execution time of each task]. The CPU Utilisation is 85.23%.

## Shared data structures and methods used to guarantee safe access and synchronisation

Below are a discuss the methods used of key data structures in the firmware:

**volatile uint16_t g_note_states[3];**

This array is used by each of the keyboards to store the value of all of the notes of the 3 connected keyboards. Each element stores a 12 bit number (the least significant 12 bits), with each bit representing the state of each key (1 means pressed, 0 means not pressed). This data structure is written to in scanOtherBoardsTask() and scanKeysTask(), and is read in multiple functions to produce sound. In order to protect this data type, a mutex is used, where a function must take the mutex assigned to note protection before accessing the data structure.

\***\*Knob local_knob0, local_knob1, local_knob2, local_knob3;**\*\*\*\*\*

the knob class is used to update and read the values of the knobs of each keyboard safely.

The value of each knob is an integer.

the value of a knob is updated by calling local_knob.update_rotation(keymatrix);

This function uses an atomic store to ensure that no other functions access the value of the knob while updating its value

The value of each knob is loaded by calling local_knob.get_rotation();

This function uses an atomic load to ensure that no other functions update the value of the knob while its value is being fetched.

## Analysis of inter-task dependencies that show any possibility of deadlock

The following lists show the dependencies on shared/global data structures for each task:

- sampleGenerationTask
  - Read
    - global knobs 0, 2, 3, 4
    - bendStep
    - vibStep
    - keyboardIndex & numberOfKeyboards
    - sampleBuffer 0 & 1
    - noteMult
- sampleISR
  - Read
    - sampleBuffer 0 and 1
- scanKeysTask

  - Read
    - recorded_g_note_states
    - keyboardIndex
    - envActive
    - loopPlaying
    - loopIndex
  - Write
    - envActive
    - g*note* states
    - global knobs
    - bendStep

- scanOtherBoardsTask

  - Read
    - envActive
    - recorded_g_note_states
    - loop_record and loop_play
    - loopPlaying
  - Write
    - global knobs
    - envActive
    - g_note_states

- updateDisplayTask

  - Read
    - keyboardIndex
    - global knobs
    - loop_record and loop_play

- recordLoopTask

  - Read
    - g_note_states
  - Write
    - recorded_g_note_states

- playLoopTask

  - Read
    - loop_record and loop_play
  - Write
    - loopPlaying
    - loopIndex

- setVibStepTask

  - Write
    - vibStep

- metronomeTask

  - Write
    - metronomeBeep

- keyPressExecution
  - Read - g_note_states
    -Write - envActive - noteMult
