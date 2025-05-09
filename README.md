# Pololu Tic Rust Driver
A Rust [`embedded-hal`](https://docs.rs/embedded-hal/latest/embedded_hal/) library for controlling the 
[Pololu Tic](https://www.pololu.com/tic) series of stepper motor drivers. It supports the same devices 
the [official Arduino driver](https://github.com/pololu/tic-arduino) does, namely the `T500`, `T834`, 
`T825`, `T249`, and `36v4`.

Currently, this driver only supports I²C, and not the Serial control mode which the Tic devices support. 
Serial control is planned for the future. This driver also only supports `embedded-hal >= 1.0`.

> [!IMPORTANT]
> The Tic devices utilize I²C clock-stretching, which can cause timeouts and errors on some chips. Ensure
> you increase the I²C timeout delay on your chip if you are getting timeout errors while using this library.

## License
This library is licensed under the MIT and Apache 2.0 permissive open-source licenses. Please review the terms 
of these licenses to decide how to incorporate this library into your projects.

## Example
A basic example of using this library to set up and control a Tic36v4 is as follows. Ensure you replace
`<i2c_bus>` with your platform's `embedded_hal` I²C interface.
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
