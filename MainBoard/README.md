### Main Board Demo COde: CANBus check
#### Hardware Notes
Spektrum RC receiver is on USART 2 in receive only mode, baud is 115200.

USART 6 is pulled out to header J8

USART 3 is for debug purposes and is hooked into ST-Link, baud is 115200. Use a serial monitor to read messages from the main controller.

Still using CAN1

I2C1 is pulled out to header J7

Ethernet PHY is not currently set up but is functional.

#### Software Notes
Definitely worth taking a look at / reworking the message ID enumerations in the future to ease in message filtering. For now they're fine, but if we wanted to add debug messages that we could ignore later or have some boards sort messages by ID, there may be better enumerations than current.

CANBus debug stuff https://electronics.stackexchange.com/questions/353005/can-initialization-timeout-error-in-stm32f4
