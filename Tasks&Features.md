# Tasks & Features

## Advanced features 

- Polyphony
  - Multiplie chords can be played together   
- Pitch bend
  - (description of pitch bend)   
- Vibrato
  - (description)   
- Global stereo speakers
  - All speakers output current Vout
- Panning of speakers
  - Move the location of speaker output to different speakers when connected 
- Envelope
  - Attack, Sustain, Decay 
- Multiple output waveforms (sine, triangle, sawtooth, pulse)
  - Different waveforms output to Vout 
- Looping
  - Current melody recorded and can be played on loop until desired 
- Aesthetic graphical user interface
  - Easy to use user interface displaying all knob values 

## Tasks (thread/interrupt)

******sampleGenerationTask******

This task takes the current state of each key, as well as all of the values of the current knobs and uses these values to generate the Vout values. These values are written into a buffer called sampleBuffer This task is implemented using a thread. This task has the highest priority of all other tasks, as it has the highest frequency and the task of producing sound is the most important feature of the synthesiser.

********sampleISR - interrupt********

This function takes the Vout values written into the sampleBuffer and writes them to the analog output pin which creates the sound. This task is implemented as an interrupt, because it is run at an extremely fast frequency and has priority over any other task performed by the system, so must be able to stop the execution of the other tasks at any point.

************************scanKeysTask************************

This function reads the state of each key in the keyboard, as well as the values of each knob. It then updates the state variables keeping track of each of these values, as well as transmits the state of the keyboard to the other connected keyboards via CAN. This function is implemented as a task which is timed to 

******scanOtherBoardsTask******

This task uses CAN to check for messages sent from other boards in the keyboard. Each me

**************************updateDisplayTask**************************
