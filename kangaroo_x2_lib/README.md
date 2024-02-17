# Kangaroo Communication library

This library provides a serial communication interface with Kangaroo x2.
The library has been created by adapting the code of "[Arduino Library for Kangaroo Packet Serial - v 1.0.6](https://www.dimensionengineering.com/info/arduino)".

## Differential drive

### Calculate units by encoder lines 

* Wheel radius: `r` -> radius of the wheels
* Wheel distance: `B` -> distance between the middle of the wheels
* Gear ration: `R` (`R:1`) -> motor axis to wheel axis ratio
* Encoder lines: `L` -> specification of the encoder (not quadrature)

**Drive units (forwarding):** `F = 2 * π * r` -> length of a full wheel turn
* C++ code: `drive.units(F, R*L);`

**Turn units: `T = R*L*pi*B/F`** -> number of encoder lines counted to turn of 360°
* C++ code: `turn.units(360, T);`

The function `calculateDiffDriveUnits` in `tools.hpp` automatically applies these formulas to setup the controller.


