# Feetech C++ Library
This repository contains a C++ library for interfacing with Feetech STS model servos from a computer.

## Intended setup
This library was intended to be used with a Feetech STS or SCS model servo, connected to a computer (can be a PC or a SBC, in the case of robotics applications) via Feetech's [FE-URT1 board](https://www.feetechrc.com/FE-URT1-C001.html). While it can be used as a pure C++ library, this module was initially intended to support a ROS2 software stack for robotics.

## Dependencies
The library is dependent on Boost. To install on Linux:
```
sudo apt-get install libboost-all-dev
```

## How to use
Simply clone the library into your project and include the header file. This will give you access to all the public functions.

The `src/example.cpp` file contains an example moving the servo. It can be compiled by calling the convenience script `compile_example.sh` and executed with `./example`.

You need to follow Feetech's instructions on setting the right ID (default value 1) and Baud rate on the servo. These must correspond with the values passed to the driver.


## Credits
This library was heavily inspired by [matthieuvigne's Arduino implementation](https://github.com/matthieuvigne/STS_servos).

This adaptation was made by 
- Anton Bredenbeck ([@antbre](https://github.com/antbre), Delft University of Technology)
- Martijn Brummelhuis ([@mbrummelhuis](https://github.com/mbrummelhuis), Delft University of Technology)

## License
MIT license