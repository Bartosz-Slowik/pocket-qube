# Pocket Qube on STM32U545RE template

## Software dependencies
If you are using Nix with flakes enabled you can just run `nix develop`,
it will put you into the shell with all software dependencies included.

Otherwise you need to install:
- a relatively recent version of Rust
- probe-rs: e.g. through `cargo binstall probe-rs-tools`

## Local generated files
Git ignores Rust build output, local environment secrets, and the monitor's
captured photos (`monitor/images/`), logs (`monitor/logs/`), and map tile cache
(`monitor/tiles/`). Sample images in `assets/` and Cargo lockfiles remain tracked.
Ignore rules do not remove files that are already tracked by Git.

## Linux udev rules
To make flashing STM not require root permissions you can add the following rules to udev. They allow reading and writing to STLINK devices for all users.
```
# STLINK
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", MODE="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3754", MODE="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374e", MODE="0666"
```

## Running code and tests
Provided you have everything installed, you can just run the following commands to compile, flash and execute code on STM32U545RE:
```
cargo run --release # To run in release mode
cargo run           # To run in debug mode
```

## BMP388 note
The board schematic already includes an Adafruit BMP388 sensor. The current firmware logs the intended SPI pin mapping from [pocketQube_connections.txt](pocketQube_connections.txt):
- SCK is on PB6 (NET_3)
- SDI is on PB7 (NET_19)
- The exported schematic leaves the SDO and CS lines unconnected to the STM32, so full sensor readout will require a hardware-side wiring update or a revised board export.

Tests in `test/` directory can be run with:
```
cargo test --release
cargo test
```
Similarly to `cargo run`, this will compile, flash and execute test code. Between each test the microcontroller is restarted.









