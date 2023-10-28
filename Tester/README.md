# DRSSTC Midi Control Tester
Testing a MIDI file, to make sure that it would sound decent on a set of Tesla coils is difficult.
Specifically when one downloads a Midi file off the internet and does not know if the certain channels work well in a limited frequency pulse pattern.
So I took apart an old set of computer speakers and reused the speakers to build a small tester.

![FrontPanel]([https://github.com/JarrenLange/TeslaMidiMultiToneController/blob/main/Tester/Images/TesterFront.jpg])

# Design 
The design uses a TSC426 Mosfet driver (that I easily had on hand) the nominally HI signal from the Fibre optic receivers is buffered.
Simple as this means that the speakers should generate a single pulse of air pressure, similair to a Tesla arc for each pulse.
This helps to test the entire MIDI chain, before the Tesla coils are turned on.
