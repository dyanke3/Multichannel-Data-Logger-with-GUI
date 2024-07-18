# Summary
The device shown below was made as a final year project for my studies. Main goal was to design a system, which could be used to monitor a fixed environment with a capability to log the data in a storage peripheral. After a few revisions, a functional multichannel logging device was finally achieved, here's a list of things it can do:
* Logger supports up to 8 sensors (channels) connected at once, in this case [O<sub>2</sub>](https://gaslab.com/products/oxygen-sensor-luminox-lox-o2), [CO<sub>2</sub>](https://sensirion.com/products/catalog/STC31) and temperature sensors were present.
* Different interface sensors can be integrated into the system, including I<sup>2</sup>C, UART (Receive only) and analog signal sensing components.
* The Logger is able to automatically identify the connected peripherals and determine the slot, to which they are connected. 
* A microSD card slot was succesfully integrated into the device, thus it can write data of each sensor to _.txt_ files. Time stamp is also included, as the system uses STM32's internal RTC for date and time tracking.
* Ability to choose 5 different data logging periods: 1sec., 10sec., 30sec., 1min. and 5min. 
* Device is fully portable and uses a 3500mAh LiPo battery as a power source. System is designed to be charged via USB-C port and includes a modern solution from [Texas Instruments](https://www.ti.com/lit/ug/slvuby2a/slvuby2a.pdf?ts=1707079337468&ref_url=https%253A%252F%252Fwww.google.com%252F). USB-PD controller and charger IC's provide various internal protection circuits and allow to safely use high-voltage USB-C chargers, because of USB-PD communication configuration.
* A Nextion display (HMI) was used for GUI development, allowing user to operate the device with a touch screen.

![IMG_9431](https://github.com/dyanke3/Multichannel-Data-Logger-with-GUI/assets/170525314/58cd7c52-1b8a-4618-a0b1-d3f28c3bf7cf)

## Hardware
The system is powered by a STM32F1 series MCU and its I<sup>2</sup>C interface is expanded using a 8 channel I<sup>2</sup>C Multiplexer. CO<sub>2</sub> sensor is fully controlled via I<sup>2</sup>C and identification of O<sub>2</sub> (UART) and temperature (Analog) sensors was also achieved because of this interface, as the design of these sensors integrate EEPROM memory with a single byte ID written in it. All of the channels are also connected to a analog 8:1 Multiplexer (separate pin), its output then goes into a SPDT Analog Switch, which is connected to MCU's internal ADC and UART channel pins respectively. For example, in case of identifying a NTC thermistor circuitry, MCU is able to address needed outputs of these analog IC's and establish a connection between ADC and output of the Thermistor. Channel connectors are RJ12, as they are cheap, contain 6 contacts and the ability to easily make your own wires. USB-C charging design include the combination of TPS25750 and BQ25792 IC's, which allow to use various Power-Delivery chargers and configure the desired charging voltage, also it let's the Logger to safely operate from battery power. Battery voltage from the BQ25792 output is then connected to a 3.3V LDO and a 5V DC-DC Boost Converter, providing power for peripherals and display. PCB design in 3D view is shown below:

![image](https://github.com/dyanke3/Multichannel-Data-Logger-with-GUI/assets/170525314/7175eb4d-cc74-4f40-9903-cf95f59386f9)

![image](https://github.com/dyanke3/Multichannel-Data-Logger-with-GUI/assets/170525314/0e2a31dc-ac39-46d9-a99f-c38c65462419)

Board measurements were alligned to match the Nextion NX8048T050 display, but it caused a lot of unused space, eventually it gave the PCB an "L" shape. Free space was used for smaller break-out boards of the sensors and later provided a great spot to fit the battery. For USB-PD IC tracing, signal Polygon Pours were used as these chips can support higher currents, overall, the PCB included Via Stitching to provide shorter return paths for all of the signals. There was an attempt to design a USB differential pair on this 2-layer board, unfortunately it turned out not to be functional as Data pins were mismatched 🤦‍♂️. Also, CO<sub>2</sub> sensor was likely damaged during soldering and unfortunately gives out false readings, although it reacts to changes in concentration with ~15% systematic error.

## Software
Starting from GUI, Nextion provides a great [tool](https://nextion.tech/editor_guide/) to design it. In the editor, it is possible to add text boxes, buttons, check boxes and the interface itself is updated through a microSD slot on the display, which is really convenient. Display itself can send/receive UART messages, thus allowing to communicate with an external MCU. In Nextion's software there is a possibility to trigger events upon a button press or a checkbox selection, thus it can print out a UART message (i.e. 0xB1 would start the acquisition), which is received by STM32 using the UART interrupt command. Text transmission to the display is achieved via special formatting of the UART TX message, which uses ID's of the text boxes and three 0xFF bytes at the end of the buffer.

Timer callback function is used for the main algorithm: it varies between I<sup>2</sup>C Multiplexer channels every 1 second and firstly tries to read out EEPROM data. Two _if_ statements exist for O<sub>2</sub> and thermistor sensor identification and upon reading a correct byte on one of the selected channels, the program proceeds with further processing. For example, reading out an EEPROM value of _0x01_ would indicate the O<sub>2</sub> sensor, then the same channel on Analog MUX would be selected, Analog Switch would open the path to UART_RX pin and the message processing function would be called. CO<sub>2</sub> on the other hand is basically identified by disabling sensor's CRC and checking, whether this I<sup>2</sup>C command gets a timeout or not. If disabling CRC was successful, further I<sup>2</sup>C command sequence is proceeded and in case **none** of these statements are met, channel prints out "Empty".

A separate Timer was used for periodic file appending and its interrupt period differs by modifying the timer prescaler values in real-time. These modifications are done corresponding to input from the user: checkboxes transmit different bytes, thus triggering an UART interrupt and by processing this data, a _HAL_TIM_SET_PRESCALER_ command is called respectively. For example, _0x01_ would set a 1sec. logging period, _0x02_ - 10sec. and so on. A separate menu in the GUI is shown below:

<p align="center">
  <img src="https://github.com/user-attachments/assets/e291db1b-19c6-4dd4-970d-caa87b29ba1f" />
</p>

The device had no intention for high-speed microSD data writing, accordingly, SPI interface was implemented both from HW and SW side. To integrate this storage peripheral with a STM32 system, _FATFS_ Middleware was configured in STM32CubeMX and this [tutorial](https://controllerstech.com/sd-card-using-spi-in-stm32/) was followed specifically for SPI interface realization. The library provides the ability to operate in append mode, so _.txt_ files can be created on an empty microSD card only once and will be appended each time logging is present. In order to prevent blank data logging into files or a mismatch, a flag _SD_i[Port]_ was implemented in the main algorithm, in case of a detected sensor it is set to a certain value:
* SD_i[Port] == 1 - O<sub>2</sub>;
* SD_i[Port] == 2 - Temp;
* SD_i[Port] == 3 - CO<sub>2</sub>.
* 
If a I<sup>2</sup>C timeout is triggered, this flag is set to 0 by the respective channel. These values are checked in the separate timer callback function by quickly cycling through all of the channels and in case the statement is met, a corresponding file is opened and a buffer with sensor data and RTC time stamp is written into it. Screenshot below shows the result of data logging functionality:

![image](https://github.com/user-attachments/assets/7c7547f9-2da8-4a97-ab0e-dae25e2dd4ec)

## Enclosure Design (Autodesk Fusion)
Casing of the device is rather simple, it has cutouts for ports/slots, LED's and the button, PCB is slighty elevated from the bottom in order to provide room to slide battery underneath it a bit. Cylinder shaped spacers were also designed to fit in the same enclosure, as they separate the display from the board and hold the screen itself (smaller diameter goes into enclosure holes). Everything holds intact because of the display bezel, which presses the internal parts and is fixed with a screw from both sides. 

![Screenshot 2024-07-11 192657](https://github.com/dyanke3/Multichannel-Data-Logger-with-GUI/assets/170525314/f2e8a286-947e-434e-b6b2-dc38e4024dd9)

![Screenshot 2024-07-11 193210](https://github.com/dyanke3/Multichannel-Data-Logger-with-GUI/assets/170525314/3177ca6a-b7b5-4d93-8a06-99bcc2b2397d)

