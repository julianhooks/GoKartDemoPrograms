### ThrottleByWire Demo Code
This code should contain test/demonstrations which verify that all hardware on the TBW board functions correctly.
#### Scope/Goals
1) Demonstrate ability to control the drive motors via the throttle pedal
2) Demonstrate ability to read the position of the throttle
3) Demonstrate ability to transmit/receive on the CAN bus
#### throttle Reading
The potentiometer comes in on pin **CN4-9 - PA4**  from J2.
This value is used to output a PWM signal to the motor driver
#### Motor Driver Board
Vesc 75/300 MKIV

https://trampaboards.com/the-most-powerful-vesc-yet-a-vesc-with-300a-75v-perfect-for-electric-vehicles-electric-boats-electric-hydrofoils-industrial-applications-p-25831.html 

uses Vesc tool to configure (see ZIP)
https://vesc-project.com/vesc_tool 

configured to read a standard RC PWM signal (50Hz with 1-2ms pulses)

#### CAN Functionality

**Pin to pin connections:**
CAN Tx - CN3-5 - PA12 - CAN1Tx
CAN Rx - CN3-13 - PA13 - CAN1Rx

_Read and transmit some CAN packet._

