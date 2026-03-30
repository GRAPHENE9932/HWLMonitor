# HWLMonitor

"Image here"

[![CMake on a single platform](https://github.com/GRAPHENE9932/HWLMonitor/actions/workflows/cmake-single-platform.yml/badge.svg)](https://github.com/GRAPHENE9932/HWLMonitor/actions/workflows/cmake-single-platform.yml)

A USB battery-powered device for input lag measurement.

Features:
- Illuminance and color temperature measurement.
- Input/output latency measurement with a fast photodiode and the device acting
like a USB mouse.
- Measurement of PWM amplitude and frequency of screens and ambient lights.

## Schematics

"Image here"

## Building the firmware

### Prerequisites

- An st-link programmer
- `arm-none-eabi` toolchain
- `st-flash` installed

### Building process

```
git clone https://github.com/GRAPHENE9932/HWLMonitor.git
cd HWLMonitor
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

### Uploading to the hardware

After successfully building the firmware and connecting device's `NRST`,
`SWDIO`, `GND` and `SWCLK` pins to the corresponding pins on your st-link, the
firmware can be uploaded with the `make upload` command.
