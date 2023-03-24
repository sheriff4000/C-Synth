# Tasks & Features

## Advanced features 

- Polyphony
  - Multiplie chords can be played together   
- Pitch bend
  - Joystick on the left keyboard can execute pitch bend (slighly raise or lower the pitch of notes being played) by moving right or left
- Vibrato
  - Create vibrato (small pitch bend oscillations) with varying frequency. This frequency is controlled by moving the keyboard of the left keyboard up/down
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
Overview: 

|         Task         | Interrupt/Thread | Priority | Stack size |
|:--------------------:|------------------|:--------:|------------|
| sampleGenerationTask | Thread           |     8    | 256        |
|     scanKeysTask     | Thread           |     7    | 128        |
| scanOtherBoards      | Thread           | 6        | 64         |
| setVibStepTask       | Thread           | 5        | 64         |
|   updateDisplayTask  | Thread           |     4    | 64         |
| metronomeTask        | Thread           | 3        | 64         |
| playLoopTask         | Thread           | 2        | 256        |
|    recordLoopTask    | Thread           |     1    | 256        |
| startEnvelopeTask    | Thread           | 1        | 256        |
|       sampleISR      | Interrupt        |  22000Hz |            |

**sampleGenerationTask**

This task takes the current state of each key, as well as all of the values of the current knobs and uses these values to generate the Vout values. These values are written into a buffer called sampleBuffer This task is implemented using a thread. This task has the highest priority of all other tasks, as it has the highest frequency and the task of producing sound is the most important feature of the synthesiser.

**scanKeysTask**

This function reads the state of each key in the keyboard, as well as the values of each knob. It then updates the state variables keeping track of each of these values, as well as transmits the state of the keyboard (the state of each key and the value of each knob) to the other connected keyboards via CAN. 

**scanOtherBoardsTask**

This task uses CAN to check for messages sent from other boards in the keyboard. These messages contain the state of the keyboard transmitting the message (the state of each key and the value of each of its knobs), as well as the index (0 is left, 1 is middle, 2 is right) of the keyboard transmitting the message. These received messages are used by each keyboard are used to maintain a constant global state - this stores the state of each key in all of the keyboards, as well as all of the knob values.

**setVibStepTask**

This task constantly scans the joystick of the left keyboard to calculate an offset that varies sinusoidally. The frequency of the sinusoid is varied by the vertical value of the joystick. This offset is applied to the Vout values to be played, creating a vibrato effect.
This task constantly scans the joystick to calculate an offset (added to the step size for the accumulator) that varies sinusoidally.

**updateDisplayTask**

This task produces a graphical display on each keyboard. For the leftmost keyboard, it displays information showing the current wave form, the volume of the keyboard, and the note being played. The other keyboards display the meaning of each knob and a graphical representation of the current state of each knob. This function is implemented as a task with low priority, since the graphical display does not need to update instantaneously - any task which produces sound is much more important.

**sampleISR - interrupt**

This function takes the Vout values written into the sampleBuffer and writes them to the analog output pin which creates the sound. This task is implemented as an interrupt, because it is run at an extremely fast frequency and has priority over any other task performed by the system, so must be able to stop the execution of the other tasks at any point.


**keyPressExecution**
keyPressExecution is activated in order for a note to be played, it increments and decrements a multiplier used to set the amplitude of a note's signal.


**recordLoopTask**

This task is used for our looping function - an advanced feature. Once global knob 5 is being held, the task stores the value of each key every 50ms, and stores these note states in a 2D array. This allows a sequence of notes being played to be stored, able to be played back in the playLoopTask.


**playLoopTask**

This task is used to play the notes that were saved during recordLoopTask in order, at the same speed as recorded, when global knob 6 is being held. Once the playback reaches the end of the saved sequence, the sequence plays back from the start. The task is implemented as a task which runs at the same frequency as recordLoopTask, to allow the playback of the recorded sound to be at the same speed as the recording.



**metronomeTask**
 
The metronome task maintains a steadily ticking metronome. It does this by looping every 100ms and 'beeping' at regular intervals. This 'beep' is implemented by setting a global boolean value to be high for a short amount of time at regular intervals. The gap between each interval is set in the task by reading the value of global_knob10, which sets the tempo of the metronome. This function is implemented as a task.