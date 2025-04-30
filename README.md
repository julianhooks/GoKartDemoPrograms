### Pressure Sensors
The power supply to the pressure tranducers should be 12 volts, but they are 3.3V supplied. The part numbers on the transducers are misprinted (Should be 3041, not 3021).

### DB-9 Errors
The DB-9 cables for CANBus are not wired to standard (From )
> pin 2: CAN-Low (CAN−)
> pin 3: GND (ground)
> pin 7: CAN-High (CAN+)
> pin 9: CAN V+ (5V)

But rather:
> pin 1: CAN-High (CAN+)
> pin 2: CAN-Low (CAN−)
> pin 4: CAN V+ (5V)
> pin 5: GND (ground)

###