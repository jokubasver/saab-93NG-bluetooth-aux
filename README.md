# saab-93NG-bluetooth-aux
A module to augment the Aux-In functionality of SAAB 93NG's (2002/3+), to add Bluetooth media playback and steering wheel media control support.

# Features
- Bluetooth music playback through the wired aux port
  - Bluetooth Auto-Reconnect feature (when entering the car, or after signal loss)
  - Play audio immediately after connecting/reconnecting to phone
  - Auto-switching to Aux input mode by reading SID data (with a help of a CAN transceiver, tested with MCP2551)
- HFP (Hands Free Profile) Audio support
  - Uses a custom Arduino ESP32 v2.0.3 core that brings in support for HFP audio
  - Switches automatically from A2DP to HFP audio when in call
  - Supports CVSD and mSBC codecs
  - I2S microphone support - tested with INMP441
- Steering wheel button controls
  - Play / Pause
  - Next / Previous
  - Voice Assistant
  - Accept incoming phone call 
  - Hang up active phone call 

# First prototype!
<img src="https://i.imgur.com/RzdHh4H.jpg" width="315" height="auto">

# Prototype parts
 - ESP32 NodeMCU Board (30-pin variant)
 - Adafruit I2S to Stereo DAC (UDA1334)
 - MCP2515 CAN-bus to SPI interface module

# Project Layout
- /saab_bluetooth_aux_kit
  - This is the PCB which all the modules will solder onto, for easy assembly.
- /esp32_a2dp_reciever  
  - PIO project, the software flashed to the ESP32 module to provide bluetooth audio -> I2S functionality.
  - Also handles CAN bus messages, for steering wheel Play/Pause/Next/Previous
  
# Notes for building your own (by [jokubasver](https://github.com/jokubasver))
- INMP441 microphone pin connections to ESP32:
  - SCK - pin 16
  - WS - pin 17
  - SD - pin 21
  - L/R - GND
  - Microphone support is experimental and quite buggy.

- I-Bus connections:
  - CAN H connects to LS GMLAN1 (green wire, it can be found on the ICM connector pin 1, or solder directly to ICM connector PCB pad).
  - CAN L connects to GND
  - Connect a 4.7k resistor between CAN H and CAN L
  - If connection is unreliable, or if the ESP32 freezes - try leaving CAN L floating and disconnect the resistor.

- For Auto Aux switching, you will need to build an extra circuit:
  - <img width="1426" height="1630" alt="image" src="https://github.com/user-attachments/assets/c49c2320-3353-4fbe-ac44-fa4defc95590" />
  - This is the circuit I used on my prototype, though it's probably better to use an SN65HVD230DR connected to 3.3V instead of 5V.
  - C D Bus is the communication channel between the ICM and SID - it is UART over CAN.
  - ESP32 looks for the "AUX" string in the SID display - if Bluetooth is connected, CD button presses are sent via I-Bus until the SID displays "AUX".


- Instead of using 12V, the EHU 5V line can be used.
  - It turns off after ~10-15 seconds after locking the car with the remote. 
  - 5V wakes up as soon as you unlock the car.
  - This requires the use of an isolated 5V-5V DC-DC converter (see below)

- 5V, GND, AUX Left, AUX right can be soldered on the EHU PCB. All pads are clearly marked:
- <img width="1200" height="1600" alt="128476431-3756da6e-83ff-4f5b-8169-b65d09f547a4" src="https://github.com/user-attachments/assets/d1efcf97-d306-4dd2-bf78-013945a0ac96" />
- <img width="1200" height="1600" alt="128475600-fccb96eb-5737-41b3-a692-3691a5302de4" src="https://github.com/user-attachments/assets/d70ec4a9-9552-4132-be53-cf4eb4a897f5" />

- Using EHU 5V will cause some audible Bluetooth interference. 
  - Use an isolated 5V-5V DC-DC converter to eliminate it (CC3-0505SF-E would be pin-compatible and would fit nicely on the bluetooth aux kit PCB)
  - I am personally using AM1SS-0505SJZ together with capacitors and inductors that are shown in the datasheet's example schematic. 

- **Warning on EHU 5V line**
  - I have experienced massive current draw from the battery when using this 5V line previously, while experimenting with a generic Bluetooth Aux module. I tried it again with this project and everything is fine.
  - My guess is that my first experiment had a ground loop, and this either caused current to flow through the ground loop, draining the battery, or it confused the EHU and prevented it to go into sleep.
  - Therefore, I highly suggest you use an isolated DC-DC converter (whether you are using 12V, or 5V from the EHU)
