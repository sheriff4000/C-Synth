# Real time system analysis 

## Characterisation of each task with minimum initiation interval and measured maximum execution time

## Critical analysis of rate monotonic scheduler
 - show all deadlines are met under worst case conditions
 
## Execution time

|         Task         | Initiation Interval (ms)| Maximum Execution time |
|:--------------------:|-------------------------|:----------------------:|
| sampleGenerationTask |        5.818            |     8                  |
|     scanKeysTask     |         -               |     7                  |
| scanOtherBoards      |        20               | 6                      |
| setVibStepTask       |       20                | 5                      |
|   updateDisplayTask  |       100               |      4                 |
| metronomeTask        |       -                 | 3                             |
| playLoopTask         |       50                | 2                             |
|    recordLoopTask    |      50                 |     1                         |
| keyPressExecution    | Thread                  |     1                        |
|       sampleISR      | Interrupt               |  22000Hz               |

## Total CPU utilisation

## Shared data structures and methods used to guarantee safe access and synchronisation
 - The latency of the lowest-priority task ($t_n$) is 

## Analysis of inter-task dependencies that show any possibility of deadlock

The following lists show the  dependencies on shared/global data structures for each task as well as how any deadlocks are avoided
### sampleGenerationTask
    - Read
        - global knobs 0, 2, 3, 4 
        - bendStep 
        - vibStep 
        - keyboardIndex & numberOfKeyboards 
        - sampleBuffer 0 & 1 
        - noteMult 
    the task only has read dependencies so pre


### sampleISR
    - Read
        - sampleBuffer 0 and 1 
    

### scanKeysTask
    - Read
        - recorded_g_note_states 
        - keyboardIndex 
        - envActive 
        - loopPlaying 
        - loopIndex 
    - Write 
        - envActive 
        - g_note_ states 
        - global knobs 
        - bendStep

### scanOtherBoardsTask
    - Read
        - loop_record and loop_play 
        - loopPlaying 
        - recorded_g_note_states 
        - loopIndex 
        - envActive
    - Write
        - global knobs 
        - g_note_ states 
        - envActive

### updateDisplayTask
    - Read
        - keyboardIndex 
        - global knobs 
        - loop_record and loop_play 

### recordLoopTask
    - Read
        - g_note_states 
        - loop_record 
    - Write
        - recorded_g_note_states 

### playLoopTask 
    - Read
        - loop_record and loop_play 
    - Write
        - loopPlaying 
        - loopIndex 

### setVibStepTask
    - Write
        - vibStep 

### metronomeTask
    - Write
        - metronomeBeep 
### keyPressExecution
    - Read
        - global knobs 
        - g_note_states 
    - Write
        - envActive 
        - noteMult 

