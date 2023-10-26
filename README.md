# DRSSTC Midi Multi Tone Controller
This project is the repository for a dual tone, dual Tesla coil midi controller. Includes Schematics, code and design files for the Housing used.

# Design 
This controller is designed to interface with Dual Resonant Solid State Tesla Coils (DRSSTC) that utilise Fibre optic activated control boards based upon the Steve Ward design. 
I have tested this design with two Tesla coils that utilise the LoneOceans UD2.7 Controller. 
This device connects to a computer via a USB-C cable and has an LCD display for the setup and parameterisation of the Tesla coil operation.
Using free software it received MIDI commands from the PC, and then activates the Tesla coils with variable ON durations (with a settable maximum) to play musical notes with variable volumes. 
Its designed to be Low-cost and built using standard components, such as the STM32F411 Blackpill development boards.
It is also designed to be as safe as possible, with two mechanical switches being available to break fibre optic operation.
The one switch is a momentary switch, which ensures that a user must hold onto the switch for continued operation.

# Safety Warning
Working and operating Telsa coils is dangerous.
The Fibre Optic systems should not be relied upon to ensure saftey of the Tesla coils. 
At all time a means of Turning off the Tesla coils should be present and tested.
While all efforts have been taken to design this Controller as well as possible, it cannot be guaranteed.
I personally test the controller thoroughly after new code is uploaded to ensure that the maximum on-time is obeyed.

# Features
This controller can:

Operate two Tesla coils at once. 
Provide each Tesla coil with Dual Tones.
Has a settable maximum ON-Time per Tesla coil.
Utilises variable on times, to emulate variable tone volume. 
Ensures an off time between on pulses, to allow the resonant system to discharge.
Has a fixed frequency mode. 
Uses a USB-C cable for communication and power.

# Required Software
In order to operate this controller the following software is recommended.
LoopMidi by Tobias Erichsen https://www.tobias-erichsen.de/software/loopmidi.html
Hairless Midi https://github.com/projectgus/hairless-midiserial

LoopMidi creates a locallised Midi system through which midi signals can be rerouted.
Hairless Midi takes these midi Signals and sends it out a Serial port. 

The STM32F411 is then configured as a Virtual Serial Port to receive these commands. 

# Idemnification.
Every effort has been taken to ensure that this device and software operates as intended and as safe as possible.  
However I cannot account for variability in the components other people use for example.
I also cannot guarantee a lack of my own stupidity. 
Hence I do not accept any responsibility for any harm or damage that might occur directly or indirectly due to this device.
Reproduce this project at your own risk.
By copying/working on this repository you acknowledge that you take this risk on willingly.
