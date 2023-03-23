# Real time system analysis 

## Characterisation of each task with minimum initiation interval and measured maximum execution time

## Critical analysis of rate monotonic scheduler
 - show all deadlines are met under worst case conditions
 
## Execution time

## Total CPU utilisation

## Shared data structures and methods used to guarantee safe access and synchronisation
- Knob class


## Analysis of inter-task dependencies that show any possibility of deadlock

- sampleGenerationTask dependencies
    - global knobs 0, 2, 3, 4
    - bendStep
    - vibStep
    - keyboardIndex & numberOfKeyboards
    - sampleBuffer 0 & 1


- sampleISR - interrupt dependencies
    - sampleBuffer 0 and 1

- scanKeysTask
    - g_note_ states
    - global knobs
    - recorded_g_note_states
    - keyboardIndex
    - envActive
    - loopPlaying

- scanOtherBoardsTask
    - loop_record and loop_play
    - global knobs
    - loopPlaying
    - recorded_g_note_states
    - g_note_ states
    - envActive

- updateDisplayTask
    - keyboardIndex
    - global knobs
    - loop_record and loop_play

- recordLoopTask
    - recorded_g_note_states
    - g_note_states
- playLoopTask
    - loopPlaying
    - loop_record and loop_play
    - loopIndex

- setVibStepTask
    -  vibStep

- metronomeTask
    - metronomeBeep

