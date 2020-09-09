# Tachometer prototype, ATmega328P microcontroller
This is a program to drive an analog mechanical tachometer with a bipolar stepper motor.
ATmega328P reads a pulse input from a signal generator and moves the pointer of the tachometer.
The system also uses a potentiometer for fine-tuning the needle of the tachometer.
To drive the motor, a L293D half-H driver is used.

## Main parts used
- ATmega328P microcontroller
- L293D half-H driver
- ADAFRUIT ADA2424 Automotive Gauge Stepper Motor

## Schematic

## How it works
Atmel Studio 7 is used to program the ATmega328P (Arduino Uno) and it is programmed in C.
The microcontroller reads a square wave pulse input coming from a signal generator to pin PB0. The signal generator is there
just to simulate, for example a cars speed sensor. L293D half-H driver is used
to drive the motor, L293D ouputs are connected to stepper motor windings. Logic uses +5VDC and the motor +9VDC.
The frequency limits in the program are set to: min 300Hz and max 2000Hz, signal frequency changes linearly between
these values. The basic idea behind moving the tachometer is that the input signal pulses are being counted in 100ms
cycles, raising the value of pulsecounter on every falling edge of the pulse. The pulse count in this time window tells the
input signal frequency. 


