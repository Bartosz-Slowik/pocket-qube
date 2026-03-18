# Pocket Qube on STM32U545RE template

## Software dependencies
If you are using Nix with flakes enabled you can just run `nix develop`,
it will put you into the shell with all software dependencies included.

Otherwise you need to install:
- a relatively recent version of Rust
- probe-rs: e.g. through `cargo binstall probe-rs-tools`

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

Tests in `test/` directory can be run with:
```
cargo test --release
cargo test
```
Similarly to `cargo run`, this will compile, flash and execute test code. Between each test the microcontroller is restarted.










