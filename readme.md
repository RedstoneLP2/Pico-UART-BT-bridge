# Pico UART BT bridge
## What?
Use of a Raspberry Pico W as a RFCOMM serial bridge
## Why?
I had one lying around, wanted to [add bluetooth to my Multimeter](https://www.mikrocontroller.net/articles/Multimeter_PDM-300-C2_Analyse) and thought it might alsoy be useful for someone else.
## How to build?
- Install the [Pico-SDK](https://github.com/raspberrypi/pico-sdk)
- Download this Repo
- `mkdir build && cd build`
- `cmake ..`
- `make -j${nproc}`