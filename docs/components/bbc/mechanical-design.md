# Mechanical Design

We designed the Brake-By-Cable system to fit inside the g-wagon: in the
limited space underneath the master cylinder, between the differential
and the steering linkage.

## Design Constraints

- Generate 100 lbs (45 kg) of force to fully displace the brake pedal in
  0.5 seconds.
- Human driver can press on pedal at anytime (system does not interfere
  with normal driving)
- When power is cut (in e-stop), the brake pedal is released (the
  parking brake will stop the car)

## Design Criteria

- Minimize modifications to car
- Robust enough to last for years with minimal maintenance
- Weather proof (minimize rust)
- Safe (no pinch-points)

## Chosen Design

After many design iterations, involving several methods of satisfying
the design constraints, we chose this a first-class lever to achieve the
required force on the pedal, and a DC brushed motor with enough power to
rotate fast enough to fully actuate the pedal in under 0.5 seconds.

### Some Numbers:

- Cable connecting pedal to bottom of lever under 45 kg tension
- 3:1 (15 cm to 5 cm) lever means cable connected to top of lever under
  15 kg tension
- Large pulley attached to motor has 4 cm radius (25 cm circumference)
- Lever displaces 60 degrees (15 cm of cable wraps around pulley
  attached to motor)
- Pedal displaces 5 cm

### Motor Requirements:

- 15 kg force 2 cm from center, motor needs 30kg\*cm torque
- To fully displace pedal, motor must spin pulley of 4 cm radius to pull
  15 cm of cable in in 0.5 seconds. Motor needs to be able to spin at
  least 300 rpm at 30kg\*cm torque.

## Parts List

- [DC Brushed Motor]
- [Steel Cable]
- [Copper Compression Sleeves]
- [1/4" Steel Plate]
- 4x M5 bolts (for fastening motor to steel plate)

#### Large Pulley

- Pulley machined from Steel
- [12 mm Shaft Coupler]
- 4x M4 Bolts, Washers, Nuts (for fastening coupler to pulley)
- [Large Washer for Cable Termination]

#### Smaller Pulleys

- Pulley machined from Aluminum
- 3x [M5 Shoulder Bolt]
- 3x [6 mm Bearing]

#### Center of Lever

- [M6 Shoulder Bolt]
- [8 mm Bearing]
- M6 Nut, Washer (for securing shoulder bolt to steel plate)

<!-- ## Special Thanks

Special thanks to Max, Sinisa, Mike, & Doug for assisting machining,
fabricating, and assembling the brake-by-cable system. -->

[1/4" steel plate]: https://www.mcmaster.com/6544K25/
[12 mm shaft coupler]: https://www.amazon.com/Coupling-Connector-Coupler-Accessory-Fittings/dp/B0833438LC/ref=sr_1_10?keywords=12mm%2Bmotor%2Bcoupling&sr=8-10&th=1
[6 mm bearing]: https://www.mcmaster.com/6659K674/
[8 mm bearing]: https://www.mcmaster.com/6659K676/
[copper compression sleeves]: https://www.mcmaster.com/3897T23/
[dc brushed motor]: https://docs.rs-online.com/d589/A700000007082193.pdf
[large washer for cable termination]: https://www.mcmaster.com/94630A111/
[m5 shoulder bolt]: https://www.mcmaster.com/92981A108/
[m6 shoulder bolt]: https://www.mcmaster.com/92981A202/
[steel cable]: https://www.mcmaster.com/3450T26/
