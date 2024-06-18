# Autopilot emulator

Emulates behavior of autopilot for drones. In this example it's communicating through UART and simulates functionality of sending information of reference speed to ESC controlling BLDC Motor (from 0% to 100%).

It's a STM32CubeIDE project for NUCLEO-F767ZL.

## Usage
The project uses interrupts to listen to a 5-byte message. The message begins with "AT" and ends with "xyz", where:
* x is the value of the hundreds place
* y is the value of the tens place
* z is the value of the units (or ones) place

The value is capped at 100, because it's a percentage representation for speed. Meaning that if input is > 100, it's considered a 100.

