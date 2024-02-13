# Motor Control

To control the motor at the heart of the brake-by-cable system, we chose
a high power motor controller. The motor controller measures the current
going through the motor so that we can monitor the health of the motor.

## Selected Motor Controller

[Pololu/2991 Datasheet]

This motor controller is rated for 17A continuous draw, and its small
footprint allows it to easily fit on the node board hat.

### Added Capacitance

- Added 100uF capacitor to voltage input

### Motor Polarity

- **OUTA** (Motor Black Wire)
- **OUTB** (Motor Red Wire)

### Signals

- **PWM**: Pulse Width Modulation Input
- **DIR**: HIGH current flows OUTA->OUTB (motor pulls on pedal), LOW
  current flows OUTB-> OUTA (motor releases pedal)
- **SLP**: (default HIGH) Inverted sleep input, drive LOW to stop motor
- **FLT**: (with pull-up resistor) driven LOW when fault has occurred

### Current Sense

The **CS** (current sense) pin on the motor controller is fed into the
ADC (analog to digital converter) on the node board. The output voltage
is about 20 mV/A plus a 50 mV offset.

[pololu/2991 datasheet]: https://www.pololu.com/product/2991
