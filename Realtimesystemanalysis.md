# Real time system analysis 

## Characterisation of each task with minimum initiation interval and measured maximum execution time

## Critical analysis of rate monotonic scheduler
 - show all deadlines are met under worst case conditions
 
## Execution time

## Total CPU utilisation

## Shared data structures and methods used to guarantee safe access and synchronisation
- Knob class


## Analysis of inter-task dependencies that show any possibility of deadlock

### sampleGenerationTask dependencies
    - Read
        - global knobs 0, 2, 3, 4 
        - bendStep 
        - vibStep 
        - keyboardIndex & numberOfKeyboards 
        - sampleBuffer 0 & 1 
        - noteMult 
    the task only has read dependencies so


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

