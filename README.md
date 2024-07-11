# Summary
The device shown below was made as a final year project for my studies. Main goal was to design a system, which could be used to monitor a fixed environment with a capability to log the data in a storage peripheral. In the end, after a few revisions, a functional multichannel logging device was achieved, here's a list of what it can do:
* Logger supports up to 8 sensors (channels) connected at once, in this case O<sub>2</sub>, CO<sub>2</sub> and temperature sensors were present.
* Different interface sensors can be integrated into the system, including I<sup>2</sup>C, UART (Receive only) and analog signal sensing components.
* The Logger is able to automatically identify the connected peripherals and determine the slot, to which they are connected. 
* A microSD card slot was succesfully integrated into the device, thus it can write data of each sensor to _.txt_ files. Time stamp is also present, as the system uses STM32's internal RTC for date and time tracking.
* Ability to choose 5 different data logging periods: 1sec., 10sec., 30sec., 1min. and 5min. 
* Device is fully portable and uses a 3500mAh LiPo battery as a power source. System is designed to be charged via USB-C port and includes a modern solution from [Texas Instruments](https://www.ti.com/lit/ug/slvuby2a/slvuby2a.pdf?ts=1707079337468&ref_url=https%253A%252F%252Fwww.google.com%252F). USB-PD controller and charger IC's provide various internal protection circuits and allow to safely use high-voltage USB-C chargers, because of USB-PD communication configuration.
* A Nextion display (HMI) was used for GUI development, allowing user to operate the device with a touch screen.

![IMG_9431](https://github.com/dyanke3/Multichannel-Data-Logger-with-GUI/assets/170525314/a62c3150-88f7-4dbc-83b7-69e279075845)

![IMG_9437](https://github.com/dyanke3/Multichannel-Data-Logger-with-GUI/assets/170525314/fe225082-6c8d-4ec3-b638-49810d9c14f5)

## Hardware
The system is powered by a STM32F1 series MCU and its I<sup>2</sup>C interface is expanded using a 8 channel I<sup>2</sup>C Multiplexer. CO<sub>2</sub> sensor is fully controlled via I<sup>2</sup>C and identifying of O<sub>2</sub> (UART) and temperature (Analog) sensors was also achieved because of this interface, as tbe design of these sensors integrate EEPROM memory with a single byte ID written in it. All of the channels also connect with a analog 8:1 Multiplexer (separate pin), whose output then flows to SPDT Analog Switch, which is connected to MCU's internal ADC and UART channel pins respectively. For example, in case of identifying a NTC Thermistor circuitry, MCU is able to address needed outputs of these analog IC's and establish a connection between ADC and output of the Thermistor. USB-C charging design include combination of TPS25750 and BQ25792 IC's 
