# Real time system analysis 

## Characterisation of each task with minimum initiation interval and measured maximum execution time

## Critical analysis of rate monotonic scheduler
 - show all deadlines are met under worst case conditions
 
## Execution time

## Total CPU utilisation

## Shared data structures and methods used to guarantee safe access and synchronisation
- Knob class


## Analysis of inter-task dependencies that show any possibility of deadlock

Grouping variables based on their function, the following groups arise:
    - Handshaking variables
    - Keyboard defining variables
    - Note state variables
    - Loop variables
    - Step offset variables
    - Envelope variables
    - Global knobs

- Handshaking variables
    - Only used in the handShake(), to initialise 

- Keyboard defining variables
    - Used in most functions (e.g. scanKeys, updateDisplay, sampleGeneration) but it is only defined in setup, so no inter-task dependencies
- Note state variables 
    - Constantly read and written to so large risk of dependencies. To access, we use the semaphore noteArrayMutex in keyPressExecution, scanOtherBoards and scanKeysTask. None of these semaphore sections (take then give) have any waiting conditions that could cause deadlock.
- Loop variables
    - 
- Step offset variables
    -
- Envelope variables
    -
- Global knobs
    -

