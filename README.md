# Pololu Tic Rust Driver
[![Lib.rs Version](https://img.shields.io/crates/v/pololu_tic?style=for-the-badge&logo=rust&label=lib.rs&color=%23a68bfc)](https://lib.rs/crates/pololu_tic)
[![docs.rs](https://img.shields.io/docsrs/pololu_tic?style=for-the-badge)](https://docs.rs/pololu_tic/)
![GitHub Actions Workflow Status](https://img.shields.io/github/actions/workflow/status/unl-rocketry/pololu_tic-rs/rust_build-test.yml?style=for-the-badge)

A Rust [`embedded-hal`](https://docs.rs/embedded-hal/latest/embedded_hal/)
library for controlling the [Pololu Tic](https://www.pololu.com/tic) series of
stepper motor drivers. It supports the same devices the
[official Arduino driver](https://github.com/pololu/tic-arduino) does, namely
the `T500`, `T834`, `T825`, `T249`, and `36v4`.

Currently, this driver supports the I²C, Serial, and USB control modes which
the Tic devices support. This driver only supports `embedded-hal >= 1.0`.

> [!IMPORTANT]
> The Tic devices utilize I²C clock-stretching, which can cause timeouts and
> errors on some chips. Ensure you increase the I²C timeout delay on your chip
> if you are getting timeout errors while using this library.

> [!WARNING]
> USB support is **untested**, **experimental** and the interface may change at
> **any time**.

## Feature Flags
This library has a few feature flags to enable or disable support for different
interfaces.

 - `i2c` (default): Enables support for the I²C interface.
 - `serial`: Enables support for the UART Serial interface.
 - `usb`: Enables the USB interface support using `nusb`, and implies the `std` feature.
 - `std`: Enables `std` support, which enables traits and conversions in a few libraries.

## License
This library is licensed under the MIT and Apache 2.0 permissive open-source
licenses. Please review the terms of these licenses to decide how to incorporate
this library into your projects.

## Example
A basic example of using this library to set up and control a Tic36v4 is as
follows. Ensure you replace `<i2c_bus>` with your platform's `embedded_hal`
I²C interface.

```rust
use pololu_tic::{TicBase, TicI2C, TicProduct};

let mut tic = pololu_tic::TicI2C::new_with_address(
    <i2c_bus>,
    TicProduct::Tic36v4,
    14
);

tic.set_target_velocity(2000000);

loop {
    tic.reset_command_timeout();
}
```
