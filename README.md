# el_commutador

a simple design for an active coaxial commutator with SMA connectors. used for tangle-free freely-moving neuronal recordings in small animals using ulca miniscopes or neuropixels probes. this design mainly differs from the design sold in the open ephys store in the follwowing aspects:
* mechanical parts and electronic controller are separate devices and connected with a cable
* use of a hollow-shaft stepper instead of a gear system decreases mechanical complexity and size of the device
* more simplistic and lower-priced design, but without any housing

# assembling the commutator

the file 'bom.xlsx' contains all necessary components. the file 'assembly.pdf' outlines the assembly of the hardware. the total cost of all components for a single commutator is ca. 300 euros.

# running the commutator

load the script 'motor_teensy.ino' onto the teensy 4.0 using arduino software with teensyduino software. the file 'el_commutador.py' contains the function 'run_commutator' to launch a GUI instance for a commutator. for now we feed real-time head direction information into the system by reading the head orientation .csv-file as it is written by the miniscope recording software. in the future other sources of head direction data can be implemented to be fed in the GUI. manual operation is also possible.
