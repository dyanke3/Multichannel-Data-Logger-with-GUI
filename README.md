# Summary
The device shown below was made as a final year project for my studies. Main goal was to design a system, which could be used to monitor a fixed environment with a capability to log the data in a storage peripheral. In the end, after a few revisions, a functional multichannel logging device was achieved, here's a list of what it can do:
* Logger supports up to 8 sensors (channels) connected at once, in this case O<sup>2</sup>, CO<sup>2</sup> and temperature sensors were used.
* Different interface sensors can be integrated into the system, including I<sup>2</sup>C, UART (Receive only) and analog signal sensing components.
* The Logger is able to automatically identify the connected peripherals and determine the slot, to which they are connected. 
* A microSD card slot was succesfully integrated into the device, thus it can write data of each sensor to _.txt_ files. Time stamp is also present, as the system uses STM32's internal RTC for date and time tracking.
* Ability to choose 5 different data logging periods: 1sec., 10sec., 30sec., 1min. and 5min. respectively.
* Device is fully portable and uses a 3500mAh LiPo battery as a power source. System is designed to be charged via USB-C port and includes a modern solution from Texas Instruments. USB-PD controller and charger IC's provide various internal protection circuits and allow to safely use high-voltage USB-C chargers, because of USB-PD communication configuration.
* A Nextion display (HMI) was used for GUI development, allowing user to operate the device through a touch screen.
