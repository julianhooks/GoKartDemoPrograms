### Example Code for CANBUS
Demonstrates the conventions and hardware setup for CANBUS
the MAIN board uses the exact same setup.
You can find more details in the code

#### Data Formatting
Each packet of data is sent an 8 element array of unsigned 8-bit numbers
data is formatted as to hold 3 16-bit numbers in big endian formatting
 [0] data 1 MSB
 [1] data 1 LSB
 [2] data 2 MSB
 [3] data 2 LSB
 [4] data 3 MSB
 [5] data 3 LSB
 [6] (empty)
 [7] Identifier (such as 0x100 for MAIN)
 
not every packet will have all data filled, but identifier should always be allocated

#### Board ID's
identifier ID for each board (every board should have the exact same values)
must be between 0-255
static uint32_t MAIN = 100;
static uint32_t TEST = 101;
static uint32_t UI = 102;