### LowerSteerByWire Demo Code
This code should contain test/demonstrations which verify that all hardware on the LSBW board functions correctly.
#### Scope/Goals:
1) Demonstrate ability to control the LSBW motor via the VESC6 ESC
2) Demonstrate ability to read the angle sensor
3) Demonstrate ability to transmit/receive on the CAN bus
#### Motor Control
The motor is directly controlled by a VESC6 ESC, which the LSBW board communicates to via UART. Refer to VESC documentation and libraries for what should be sent and received (Docs)[https://vedder.se/2015/10/communicating-with-the-vesc-using-UART/] (example)[https://github.com/vedderb/bldc_uart_comm_stm32f4_discovery].

**Pin to pin connections:**
Tx - CN3-1 - PA9
Rx - CN3-2 - PA10

_Send command to VSEC6 to turn steering motor in each direction._
#### Angle Sensor
The angle sensor is a (AS5047P-TS_EK_AB)[https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/1212/AS5047P-TS_EK_AB.pdf] magnet angle sensor. It communicates over SPI to the Nucleo, but only 1 way, so there's no MOSI pin on the Nucleo. 

**Pin to pin connections:**
MISO - CN4-7 - PA6
MOSI - NC 
CLK - CN4-8 - PA5
CS - CN4-9 - PA4

_Read from the SPI bus_
#### CAN Functionality

**Pin to pin connections:**
CAN Tx - CN3-5 - PA12 - CAN1Tx
CAN Rx - CN3-13 - PA13 - CAN1Rx

_Read and transmit some CAN packet._



