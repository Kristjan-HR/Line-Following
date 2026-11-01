# Line Following

Implementation of the Lab 10 program for the *Cyber‑Physical Systems* course at Reykjavík University.

## Prerequisites
The code relies on several development tools and libraries. Install the required packages before building:

```bash
sudo apt update
sudo apt install build-essential liblgpio-dev i2c-tools libi2c-dev libspi-dev
```

## Build
A small `Makefile` is included in the repository.

- Run **`make`** to build the `robot` executable.  
- Run **`make clean`** to remove the binary.

## License
The project is released under the MIT license. See the [LICENSE](LICENSE) file.  
It incorporates the **TCS34725 driver**, which is also MIT‑licensed. See the [LICENSE‑DRIVER](LICENSE-DRIVER) file for the driver's full license text.

## Attribution
The driver was written by Marcel Kyas.  
Source repository: <https://github.com/mkyas/cpsy-tcs34725>

