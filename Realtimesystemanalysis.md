# Real time system analysis 

## Minimum initiation interval and measured maximum execution time of each task

|         Task         | Initiation Interval (ms)| Maximum Execution time(us) |
|:--------------------:|-------------------------|:----------------------:|
| sampleGenerationTask |        5.818            |     2462.25                |
|     scanKeysTask     |         -               |       1315.1875                |
| scanOtherBoards      |        20               |      18380.8125                   |
| setVibStepTask       |       20                |      165.375                 |
|   updateDisplayTask  |       100               |      17141.0312                 |
| metronomeTask        |       -                 | 0.46875                             |
| playLoopTask         |       50                | 0.53125                             |
|    recordLoopTask    |      50                 |     17.25                         |
|       sampleISR      | Interrupt               |  22000Hz               |

## Critical analysis of rate monotonic scheduler
Critical analysis can be performed to see if a specific schedule will work. The latency ($L_n$) of the lowest-priority task ($\tau_n$) is x at the worst-instance in time. The following formula can be used to calculate whether the schedule will work: 

$\Huge L_n = \sum_{i=1}[\frac{\tau_n}{\tau_i}]\cdot T_i \le \tau_n$

where \tau_i is the initiation interval and $T_i$ is the execution time for task for task $i$.  

## Total CPU utilisation

The CPU utilisation can be calculated through the following equation:

 $\Huge CPU  Utilisation = \sum_{i=1}[\frac{\tau_i}{T_i}]$

The measured CPU utilisation under these tests was calculated using the results obtained in Section [## Minimum initiation interval and measured maximum execution time of each task]. The CPU Utilisation is x.


## Shared data structures and methods used to guarantee safe access and synchronisation
 
 

## Analysis of inter-task dependencies that show any possibility of deadlock

The following lists show the  dependencies on shared/global data structures for each task:

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
        - g_note_ states 
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
    - Read
        - g_note_states
    -Write
        - envActive
        - noteMult

