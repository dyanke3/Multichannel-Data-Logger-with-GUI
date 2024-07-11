# Summary
The device shown below was made as a final year project for my studies. Main goal was to design a system, which could be used to monitor a fixed environment with a capability to log the data in a storage peripheral. In the end, after a few revisions, a functional multichannel logging device was achieved, here's a list of what it can do:
* Logger supports up to 8 sensors (channels) connected at once, in this case O<sub>2</sub>, CO<sub>2</sub> and temperature sensors were present.
* Different interface sensors can be integrated into the system, including I<sup>2</sup>C, UART (Receive only) and analog signal sensing components.
* The Logger is able to automatically identify the connected peripherals and determine the slot, to which they are connected. 
* A microSD card slot was succesfully integrated into the device, thus it can write data of each sensor to _.txt_ files. Time stamp is also included, as the system uses STM32's internal RTC for date and time tracking.
* Ability to choose 5 different data logging periods: 1sec., 10sec., 30sec., 1min. and 5min. 
* Device is fully portable and uses a 3500mAh LiPo battery as a power source. System is designed to be charged via USB-C port and includes a modern solution from [Texas Instruments](https://www.ti.com/lit/ug/slvuby2a/slvuby2a.pdf?ts=1707079337468&ref_url=https%253A%252F%252Fwww.google.com%252F). USB-PD controller and charger IC's provide various internal protection circuits and allow to safely use high-voltage USB-C chargers, because of USB-PD communication configuration.
* A Nextion display (HMI) was used for GUI development, allowing user to operate the device with a touch screen.

![IMG_9431](https://github.com/dyanke3/Multichannel-Data-Logger-with-GUI/assets/170525314/58cd7c52-1b8a-4618-a0b1-d3f28c3bf7cf)

## Hardware
The system is powered by a STM32F1 series MCU and its I<sup>2</sup>C interface is expanded using a 8 channel I<sup>2</sup>C Multiplexer. CO<sub>2</sub> sensor is fully controlled via I<sup>2</sup>C and identifying of O<sub>2</sub> (UART) and temperature (Analog) sensors was also achieved because of this interface, as tbe design of these sensors integrate EEPROM memory with a single byte ID written in it. All of the channels also connect with a analog 8:1 Multiplexer (separate pin), whose output then flows to SPDT Analog Switch, which is connected to MCU's internal ADC and UART channel pins respectively. For example, in case of identifying a NTC Thermistor circuitry, MCU is able to address needed outputs of these analog IC's and establish a connection between ADC and output of the Thermistor. Channel connectors are RJ12, as they are cheap, have 6 contacts and ability to easily make your own wires. USB-C charging design include combination of TPS25750 and BQ25792 IC's, which allow to use various Power-Delivery chargers and the desired charging voltage is configurable, also it let's the Logger to safely operate from battery power. Battery voltage from the BQ25792 output is then connected to a 3.3V LDO and a 5V DC-DC Boost Converter, providing power for peripherals and display. PCB design in 3D view is shown below:

![image](https://github.com/dyanke3/Multichannel-Data-Logger-with-GUI/assets/170525314/7175eb4d-cc74-4f40-9903-cf95f59386f9)

![image](https://github.com/dyanke3/Multichannel-Data-Logger-with-GUI/assets/170525314/0e2a31dc-ac39-46d9-a99f-c38c65462419)

Board measurements were alligned to match the Nextion NX8048T050 display, but it caused a lot of unused space, eventually it gave the PCB an "L" shape. Free space was used for smaller break-out boards of the sensors and later provided a great spot to fit the battery. For USB-PD IC tracing, signal Polygon Pours were used as these chips can support higher currents, overall, the PCB included Via Stitching to provide shorter return paths for all of the signals. There was an attempt to design a USB differential pair on this 2-layer board, unfortunately it turned out not to be functional as Data pins were mismatched 🤦‍♂️.

## Software

## Enclosure Design (Autodesk Fusion)
Casing of the device is rather simple, it has cutouts for ports/slots and the button, PCB is slighty elevated from the bottom in order to provide room to slide battery underneath it a bit. Cylinder shaped spacers were also designed to fit in the same enclosure, as they separate the display from the board and hold the screen itself (smaller diameter goes into enclosure holes). Everything holds intact because of the display bezel, which presses the internal parts and is fixed with a screw from both sides. 

![Screenshot 2024-07-11 192657](https://github.com/dyanke3/Multichannel-Data-Logger-with-GUI/assets/170525314/f2e8a286-947e-434e-b6b2-dc38e4024dd9)

![Screenshot 2024-07-11 193210](https://github.com/dyanke3/Multichannel-Data-Logger-with-GUI/assets/170525314/3177ca6a-b7b5-4d93-8a06-99bcc2b2397d)

