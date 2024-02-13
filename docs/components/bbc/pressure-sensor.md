# Pressure Sensor

We control the Brake-By-Cable system based on the error of desired
pressure in the master cylinder (connected to all 4 brakes on the
g-wagon). To read the pressure in the master cylinder, we added a
[pressure sensor] to the brake lines.

## Sensor Links

[Datasheet]

[Manual]

## Current Operating Conditions

### Voltage Input

The current-based sensor needs at least +17V to operate properly, to
generate 17V from the node power 12V line, using a boost converter to
step the voltage up.

NOTE: During testing, we found that as the supply voltage changes
(between 17-20V), the output of the sensor (the pressure reading) does
not change.

#### Boost Circuit

Currently using [MC34063 Converter] to step 12V to 17V. (see datasheet:
9.2.2 Step-Up Converter Application)

- R1: 3.4k Ohm
- R2: 47k Ohm
- RSC: 0.43 Ohm
- CO: 330 uF
- CT: 140 pF
- L: 300 uH
- Also implementing 1.0uH & 100uF LC filter on Vout

### Reading The Output

The sensor outputs 4-20mA, currently running over 220 Ohm resistor.

[datasheet]: https://www.instrumart.com/assets/Keller-Preciseline-datasheet.pdf
[manual]: https://www.instrumart.com/assets/Keller-transmitter-manual.pdf
[mc34063 converter]: https://www.ti.com/lit/ds/symlink/mc34063a.pdf?ts=1707659498761&ref_url=https%253A%252F%252Fwww.mouser.it%252F
[pressure sensor]: https://www.instrumart.com/products/34150/keller-preciseline-high-accuracy-pressure-transmitter
