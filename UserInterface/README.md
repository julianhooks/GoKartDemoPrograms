### UserInterface Demo Code
This code should contain test/demonstrations which verify that all hardware on the USBW board functions correctly.
#### Scope/Goals:
1) Demonstrate ability to write to the screen
2) Demonstrate ability to read from the switches
3) Demonstrate ability to transmit/receive on the CAN bus
#### Screen
The screen communicates over I2C. **Find I2C bus pins.**

_Write text to the screen._
#### Switches
**What pins are the switches on.**

_Read the switches._
#### CAN Functionality

**Pin to pin connections:**
CAN Tx - CN3-5 - PA12 - CAN1Tx
CAN Rx - CN3-13 - PA13 - CAN1Rx

_Read and transmit some CAN packet._

