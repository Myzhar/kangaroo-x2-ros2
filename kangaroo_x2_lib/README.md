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

Example:

``` C++
  // ----> Units setup calculation
  float radius = 90.0f;
  float baseline = 320.0f;
  uint32_t enc_lines = 100;
  float gear_ratio = 18.33333333333333333333f;
  uint32_t out_d_dist, out_d_lines, out_t_lines;
  calculateDiffDriveUnits(radius, baseline, enc_lines, gear_ratio, out_d_dist,
                          out_d_lines, out_t_lines);

  std::cout << "Robot Configuration: " << std::endl;
  std::cout << " * Wheel radius: " << radius << " mm" << std::endl;
  std::cout << " * Wheel distance: " << baseline << " mm" << std::endl;
  std::cout << " * Encoder lines: " << enc_lines << std::endl;
  std::cout << " * Gear ratio: " << gear_ratio << ":1" << std::endl;

  std::cout << std::endl;

  std::cout << "Kangaroo x2 Configuration: " << std::endl;
  std::cout << " * D, UNITS: " << out_d_dist << " mm = " << out_d_lines
            << " lines" << std::endl;
  std::cout << " * T, UNITS: " << 360 << "° = " << out_t_lines << " lines"
            << std::endl;
  // <---- Units setup calculation
```

Output:
``` bash
Robot Configuration: 
 * Wheel radius: 90 mm
 * Wheel distance: 320 mm
 * Encoder lines: 100
 * Gear ratio: 18.3333:1

Kangaroo x2 Configuration: 
 * D, UNITS: 565 mm = 1833 lines
 * T, UNITS: 360° = 3259 lines
```


