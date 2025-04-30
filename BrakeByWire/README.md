### BrakeByWire Demo Code
This code should contain test/demonstrations which verify that all hardware on the BBW board functions correctly.
#### Scope/Goals
1) Demonstrate ability to control the linear actuator
2) Demonstrate ability to read the position of the linear actuator via potentiometer input (Connector J5)
3) Demonstrate ability to read brake pressure via pressure sensor (Connector J6)
4) Demonstrate ability to transmit/receive on the CAN bus
#### Acuator Control
The [linear actuator](https://www.progressiveautomations.com/products/linear-actuator-with-potentiometer) is controlled by a [JZ2407DB-A](https://www.amazon.com/AITIAO-Controller-Regulator-Industrial-Optocoupler/dp/B0B5TWRP9R) dual H-Bridge circuit board. This board directly controls an h-bridge via connector J4. J4's pinouts are: 
1) 3.3V
2) h-bridge PWM input - CN3-12 - PA8
3) h-bridge input a - CN4-6 - PA7
4) h-bridge input b - CN4-7 - PA6
5) GND
The allowed PWM frequencies for this board are **0-10kHz**

_Demonstrating acuator control requires varying the PWM duty cycle and h-bridge direction to show a change in acuator position, speed, and direction._
#### Position Reading
The potentiometer comes in on pin **CN4-9 - PA4**  from J5.

_Demonstrating position reading requires reading and communicating this voltage. Including a voltage to position function could be nice as well, but not required._
#### Pressure Reading
The [pressure sensor](https://www.digikey.com/en/products/detail/te-connectivity-measurement-specialties/M3021-000005-500PG/125131) is connected to the brake hydraulic line. The output of the sensor is read at **CN4-8 - PA5** from J6.

_Demonstrating pressure reading requires requires reading and communicating this voltage. A voltage to pressure function could be nice but is not required._
#### CAN Functionality

**Pin to pin connections:**
CAN Tx - CN3-5 - PA12 - CAN1Tx
CAN Rx - CN3-13 - PA13 - CAN1Rx

_Read and transmit some CAN packet._

