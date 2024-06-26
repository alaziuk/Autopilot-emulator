# Autopilot emulator

Emulates behavior of autopilot for drones. In this example it's communicating through UART and simulates functionality of sending information of reference speed to ESC controlling BLDC Motor (from 0% to 100%).

There's another project that works as a controller for this emulator. It's a gui application - [UART Communicator](https://github.com/alaziuk/UART-Commuincator).

It's a STM32CubeIDE project for NUCLEO-F767ZI.

The project uses interrupts to listen to a 5-byte message. The message begins with "AT" and ends with "xyz", where:
* x is the value of the hundreds place
* y is the value of the tens place
* z is the value of the units (or ones) place

The value is capped at 100, because it's a percentage representation for speed. Meaning that if input is > 100, it's considered a 100.

It currently have PWM communication at 200 Hz. The duty for speed control is currently: 0% speed - 10% duty to 100% - 90% duty.

### What's next?
It's going to be expanded to have CAN communication.
