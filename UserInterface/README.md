i### UserInterface Demo Code
This code should contain test/demonstrations which verify that all hardware on the USBW board functions correctly.
#### Scope/Goals:
1) Demonstrate ability to write to the screen
2) Demonstrate ability to read from the switches
3) Demonstrate ability to transmit/receive on the CAN bus
#### Screen
The screen communicates over I2C.
**Pin to pin connections:**
SCL - CN3-1 - PA9 - I2C1-SCL
SDA - CN3-2 - PA10 - I2C1-SDA

The library which controls this is included with the code

_Write text to the screen._
#### Switches
**Pin to pin connections:**
S2 - CN4-9 - PA4
S3 - CN4-10 - PA3

_Read the switches._
#### CAN Functionality

**Pin to pin connections:**
CAN Tx - CN3-5 - PA12 - CAN1Tx
CAN Rx - CN3-13 - PA13 - CAN1Rx

_Read and transmit some CAN packet._

