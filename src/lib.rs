//! # Pololu Tic Rust Driver
//! A Rust crate for controlling the [Pololu Tic](https://www.pololu.com/tic) series
//! of stepper motor drivers. It supports the same devices the
//! [official Arduino driver](https://github.com/pololu/tic-arduino) does, namely
//! the `T500`, `T834`, `T825`, `T249`, and `36v4`, along with the USB control mode
//! that the [pololu-tic-software](https://github.com/pololu/pololu-tic-software)
//! program supports.
//!
//! This driver by default will work in `no-std` enviroments like microcontrollers,
//! but is perfectly suitable for using on other computers with the `usb` feature
//! enabled.
//!
//! Currently, this driver supports the [I²C](I2c), [Serial], and [USB](usb) control modes which
//! the Tic devices support. This driver only supports `embedded-hal >= 1.0`.
//!
//! ## Feature Flags
//! This library has a few feature flags to enable or disable support for different
//! interfaces.
//!
//!  - `i2c` (default): Enables support for the [I²C interface](I2c).
//!  - `serial`: Enables support for the [UART Serial interface](Serial).
//!  - `usb`: Enables support for the [USB interface](usb) using `nusb`, and implies the `std` feature.
//!  - `std`: Enables `std` support, which enables traits and conversions in a few libraries.
//!
//! ## License
//! This library is dual-licensed under the MIT and Apache 2.0 permissive
//! open-source licenses. Please review the terms of these licenses to decide
//! how to incorporate this library into your projects.
//!
//! ## Examples
//! More examples can be found in the struct documentation for each interface
//! type.
//!
//! A basic example of using this library to set up and control a Tic36v4 over I²C is as
//! follows. Ensure you replace `<i2c_bus>` and `<delay>` with your platform's `embedded_hal`
//! I²C interface and delay implementations.
//!
//! ```rust,ignore
//! use pololu_tic::{TicBase, TicI2C, Product};
//!
//! let mut tic = pololu_tic::TicI2C::new_with_address(
//!     <i2c_bus>,
//!     Product::Tic36v4,
//!     <delay>,
//!     14
//! );
//!
//! tic.set_target_velocity(2000000);
//!
//! loop {
//!     tic.reset_command_timeout();
//! }
//! ```

#![deny(unsafe_code)]
#![warn(missing_docs)]
#![no_std]

#[cfg(feature = "std")]
extern crate std;

mod base;
pub mod variables;

mod backends {
    #[cfg(feature = "i2c")]
    pub mod i2c;
    #[cfg(feature = "serial")]
    pub mod serial;
    #[cfg(feature = "usb")]
    pub mod usb;
}

#[macro_use]
extern crate num_derive;

#[doc(inline)]
#[cfg(feature = "i2c")]
pub use backends::i2c::I2c;

#[doc(inline)]
#[cfg(feature = "serial")]
pub use backends::serial::Serial;

#[doc(inline)]
#[cfg(feature = "usb")]
pub use backends::usb::Usb;

#[doc(inline)]
#[cfg(feature = "usb")]
pub use backends::usb;

#[doc(inline)]
pub use base::TicBase;

const TIC_03A_CURRENT_TABLE: [u16; 33] = [
    0, 1, 174, 343, 495, 634, 762, 880, 990, 1092, 1189, 1281, 1368, 1452, 1532, 1611, 1687, 1762,
    1835, 1909, 1982, 2056, 2131, 2207, 2285, 2366, 2451, 2540, 2634, 2734, 2843, 2962, 3093,
];

/// Represents a [`None`] value for some specific commands.
pub const TIC_INPUT_NULL: u16 = 0xFFFF;

/// The type of Tic driver that is being represented.
#[derive(Debug, FromPrimitive, ToPrimitive, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Product {
    /// An unknown driver. Used if not provided, as the default.
    Unknown = 0,
    /// [Tic T825](https://www.pololu.com/product/3130)
    T825 = 1,
    /// [Tic T834](https://www.pololu.com/product/3132)
    T834 = 2,
    /// [Tic T500](https://www.pololu.com/product/3134)
    T500 = 3,
    /// Not available on the website.
    N825 = 4,
    /// [Tic T249](https://www.pololu.com/product/3138)
    T249 = 5,
    /// [Tic 36v4](https://www.pololu.com/product/3140)
    Tic36v4 = 6,
}

/// This constant is used by the library to convert between milliamps and the
/// Tic's native current unit, which is 32 mA.  This is only valid for the Tic
/// T825 and Tic T834.
const TIC_CURRENT_UNITS: u8 = 32;

/// This constant is used by the library to convert between milliamps and the
/// Tic T249 native current unit, which is 40 mA.
const TIC_T249_CURRENT_UNITS: u8 = 40;

/// This enum defines the Tic's error bits. See the
/// [**Error handling**](https://www.pololu.com/docs/0J71/5.4) section of
/// the Tic user's guide for more information about what these errors mean.
///
/// See [`TicBase::error_status()`] and [`TicBase::errors_occurred()`].
#[derive(FromPrimitive, ToPrimitive, Debug)]
pub enum Error {
    /// The "Intentionally de-energized" error bit is 0 when the Tic starts up.
    /// It can be set with the "De-energize" command and cleared with the
    /// "Energize" command.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/5.4#cond-int-deenergized)
    IntentionallyDeenergized = 0,
    /// Indicates that motor driver IC on the Tic has detected a problem and
    /// reported it to the main microcontroller
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/5.4#cond-motor-driver-error)
    MotorDriverError = 1,
    /// Indicates that the voltage on VIN has dropped well below the minimum
    /// operating voltage.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/5.4#cond-low-vin)
    LowVin = 2,
    /// Indicates that one of the pins configured as a kill switch is in its active state.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/5.4#cond-kill-switch)
    KillSwitch = 3,
    /// Indicates that the Tic’s main input signal is not valid, so it cannot be
    /// used to set the target position or velocity.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/5.4#cond-required-input-invalid)
    RequiredInputInvalid = 4,
    /// Indicates that something went wrong with the I²C or TTL serial
    /// communication.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/5.4#cond-serial-error)
    SerialError = 5,
    /// Whenever the Tic’s control mode is “Serial / I²C / USB” and that time
    /// exceeds the timeout period, the Tic sets the “Command timeout” error
    /// bit.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/5.4#cond-serial-error)
    CommandTimeout = 6,
    /// The Tic’s safe start feature helps to avoid unexpectedly powering the
    /// motor in speed control modes and in “Serial / I²C / USB” mode.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/5.4#cond-safe-start)
    SafeStartViolation = 7,
    /// The Tic reports an “ERR line high” error if it is not driving its ERR
    /// pin high and the digital reading on the ERR pin input is high.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/5.4#cond-err-high)
    ErrLineHigh = 8,
    /// The Tic receives a TTL serial byte that does not have a valid stop bit.
    ///
    /// Related to the [`Self::SerialError`] error.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/5.4#cond-serial-error)
    SerialFraming = 16,
    /// The Tic receives a TTL serial byte at a time when its hardware or software buffers are not able to hold the byte, and the byte is lost. This should not happen under normal conditions.
    ///
    /// Related to the [`Self::SerialError`] error.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/5.4#cond-serial-error)
    RxOverrun = 17,
    /// The Tic receives a TTL serial byte with its most significant bit set (which starts a new command) while it is still waiting for data bytes from the previous command.
    ///
    /// Related to the [`Self::SerialError`] error.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/5.4#cond-serial-error)
    Format = 18,
    /// The Tic receives an incorrect CRC byte at the end of a command.
    ///
    /// Related to the [`Self::SerialError`] error.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/5.4#cond-serial-error)
    Crc = 19,
    /// The Tic has detected the encoder is missing some counts.
    ///
    /// [Read more](<https://www.pololu.com/docs/0J71/4.12#:~:text=As you do this, look at the "Encoder Skip" count>)
    EncoderSkip = 20,
}

/// The generic error type for anything in this crate.
#[derive(thiserror::Error, Debug)]
#[non_exhaustive]
pub enum HandlerError {
    /// An error internal to the driver.
    #[error("the driver experienced an internal error")]
    InternalError(Error),

    /// An I²C communication error.
    #[cfg(feature = "i2c")]
    #[error("the i2c communication experienced an error")]
    I2cError(embedded_hal::i2c::ErrorKind),

    /// A stream communication error.
    ///
    /// Most likely applicable to Serial.
    #[cfg(feature = "serial")]
    #[error("the serial communication experienced an error")]
    StreamError(embedded_io::ErrorKind),

    /// An `nusb` internal error.
    #[cfg(feature = "usb")]
    #[error("internal nusb error")]
    NusbError(nusb::Error),

    /// A USB transfer error.
    #[cfg(feature = "usb")]
    #[error("the USB communication experienced a transfer error")]
    UsbTransferError(nusb::transfer::TransferError),

    /// USB invalid device error.
    #[cfg(feature = "usb")]
    #[error("the target USB device is invalid")]
    UsbInvalidDevice(u16),

    /// Attempted to use a device before initalization.
    #[error("the device is not yet initalized")]
    NotInitalized,

    /// Generic parsing error.
    #[error("the value could not be parsed")]
    ParseError,
}

/// The Tic command codes which are used for its serial, I2C, and USB interface.
///
/// These codes are used by the library and you should not need to use them. They would be useful for creating a new Tic driver backend, so they are documented here.
///
/// [Read more](https://www.pololu.com/docs/0J71/8)
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq, Clone, Copy)]
pub enum Command {
    /// Sets the target position of the Tic, in microsteps.
    ///
    /// If the control mode is set to Serial / I²C / USB, the Tic will start moving the motor to reach the target position. If the control mode is something other than Serial / I²C / USB, this command will be silently ignored.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/8#cmd-set-target-position)
    SetTargetPosition = 0xE0,

    /// Sets the target velocity of the Tic, in microsteps per 10,000 seconds.
    ///
    /// If the control mode is set to Serial / I²C / USB, the Tic will start accelerating or decelerating the motor to reach the target velocity. If the control mode is something other than Serial / I²C / USB, this command will be silently ignored.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/8#cmd-set-target-velocity)
    SetTargetVelocity = 0xE3,

    /// Stops the motor abruptly without respecting the deceleration limit and sets the “Current position” variable, which represents what position the Tic currently thinks the motor is in.
    ///
    /// Besides stopping the motor and setting the current position, this command also clears the [`Flags::PositionUncertain`](variables::Flags::PositionUncertain) flag, sets the input state to [`InputState::Halt`](variables::InputState::Halt), and clears the “input after scaling” variable.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/8#cmd-halt-and-set-position)
    HaltAndSetPosition = 0xEC,

    /// Stops the motor abruptly without respecting the deceleration limit.
    ///
    /// Besides stopping the motor, this command also sets the [`Flags::PositionUncertain`](variables::Flags::PositionUncertain) flag (because the abrupt stop might cause steps to be missed), sets the input state to [`InputState::Halt`](variables::InputState::Halt), and clears the “input after scaling” variable.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/8#cmd-halt-and-hold)
    HaltAndHold = 0x89,

    /// Starts the Tic’s homing procedure as described in [**Section 5.6 - Homing**](https://www.pololu.com/docs/0J71/5.6).
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/8#cmd-go-home)
    GoHome = 0x97,

    /// If the command timeout is enabled, this command resets it and prevents the [`Error::CommandTimeout`] error from happening for some time.
    ///
    /// See [**Section 5.4 - Error handling**](https://www.pololu.com/docs/0J71/5.4) of the Tic user's guide for more information about the command timeout.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/8#cmd-reset-command-timeout)
    ResetCommandTimeout = 0x8C,

    /// Causes the Tic to de-energize the stepper motor coils by disabling its stepper motor driver.
    ///
    /// The motor will stop moving and consuming power. This command sets the [`Flags::PositionUncertain`][variables::Flags::PositionUncertain] flag (because the Tic is no longer in control of the motor’s position); the Tic will also set the [`Error::IntentionallyDeenergized`] error, turn on its red LED, and drive its ERR line high.
    ///
    /// [`Command::Energize`] will undo the effect of this command (except it will leave the [`Flags::PositionUncertain`][variables::Flags::PositionUncertain] flag set) and could make the system start up again.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/8#cmd-deenergize)
    Deenergize = 0x86,

    /// A request for the Tic to energize the stepper motor coils by enabling its stepper motor driver. The Tic will clear the [`Error::IntentionallyDeenergized`] error. If there are no other errors, this allows the system to start up.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/8#cmd-energize)
    Energize = 0x85,

    /// Causes the [`Error::SafeStartViolation`] error to be cleared for 200 ms. If there are no other errors, this allows the system to start up. If the control mode is not Serial / I²C / USB, then this command is silently ignored.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/8#cmd-exit-safe-start)
    ExitSafeStart = 0x83,

    /// If safe start is enabled and the control mode is Serial / I²C / USB, RC speed, analog speed, or encoder speed, this command causes the Tic to stop the motor (using the configured soft error response behavior) and set its [`Error::SafeStartViolation`] error bit.
    ///
    /// If safe start is disabled, or if the Tic is not in one of the listed modes, this command will cause a brief interruption in motor control (during which the soft error response behavior will be triggered) but otherwise have no effect.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/8#cmd-enter-safe-start)
    EnterSafeStart = 0x8F,

    /// This command makes the Tic forget most parts of its current state.
    ///
    /// Specifically, it does the following:
    /// - Reloads all settings from the Tic’s non-volatile memory and discards any temporary changes to the settings previously made with serial commands (this applies to the step mode, current limit, decay mode, max speed, starting speed, max acceleration, and max deceleration settings)
    /// - Abruptly halts the motor
    /// - Resets the motor driver
    /// - Sets the Tic’s operation state to [`OperationState::Reset`][variables::OperationState::Reset]
    /// - Clears the last movement command and the current position
    /// - Clears the encoder position
    /// - Clears the serial and [`Error::CommandTimeout`] errors and the [`TicBase::errors_occurred`] bits
    /// - Enters safe start if configured to do so
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/8#cmd-reset)
    Reset = 0xB0,

    /// Attempts to clear a motor driver error, which is an over-current or over-temperature fault reported by the Tic’s motor driver.
    ///
    /// If the “Automatically clear driver errors” setting is enabled (the default), the Tic will automatically clear a driver error and it is not necessary to send this command. Otherwise, this command must be sent to clear the driver error before the Tic can continue controlling the motor. See [**Section 5.4 - Error handling**](https://www.pololu.com/docs/0J71/5.4) for more information about driver errors.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/8#cmd-clear-driver-error)
    ClearDriverError = 0x8A,

    /// Temporarily sets the Tic’s maximum allowed motor speed in units of steps per 10,000 seconds.
    ///
    /// The provided value will override the corresponding setting from the Tic’s non-volatile memory until the next [`Command::Reset`] (or [`UsbCommand::Reinitalize`][crate::usb::UsbCommand::Reinitalize]) command or full microcontroller reset.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/8#cmd-set-max-speed)
    SetSpeedMax = 0xE6,

    /// Temporarily sets the Tic’s starting speed in units of steps per 10,000 seconds.
    ///
    /// This is the maximum speed at which instant acceleration and deceleration are allowed; see Section 5.1 for more information. The provided value will override the corresponding setting from the Tic’s non-volatile memory until the next [`Command::Reset`] (or [`UsbCommand::Reinitalize`][crate::usb::UsbCommand::Reinitalize]) command or full microcontroller reset.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/8#cmd-clear-driver-error)
    SetStartingSpeed = 0xE5,

    /// Temporarily sets the Tic’s maximum allowed motor acceleration in units of steps per second per 100 seconds. The provided value will override the corresponding setting from the Tic’s non-volatile memory until the next [`Command::Reset`] (or [`UsbCommand::Reinitalize`][crate::usb::UsbCommand::Reinitalize]) command or full microcontroller reset.
    ///
    /// If the provided value is between 0 and 99, it is treated as 100.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/8#cmd-set-max-acceleration)
    SetAccelMax = 0xEA,

    /// Temporarily sets the Tic’s maximum allowed motor deceleration in units of steps per second per 100 seconds. The provided value will override the corresponding setting from the Tic’s non-volatile memory until the next [`Command::Reset`] (or [`UsbCommand::Reinitalize`][crate::usb::UsbCommand::Reinitalize]) command or full microcontroller reset.
    ///
    /// If the provided value is 0, then the max deceleration will be set equal to the current max acceleration value. If the provided value is between 1 and 99, it is treated as 100.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/8#cmd-set-max-deceleration)
    SetDecelMax = 0xE9,

    /// Temporarily sets the step mode (also known as microstepping mode) of the driver on the Tic, which defines how many microsteps correspond to one full step.
    ///
    /// The provided value will override the corresponding setting from the Tic’s non-volatile memory until the next [`Command::Reset`] (or [`UsbCommand::Reinitalize`][crate::usb::UsbCommand::Reinitalize]) command or full microcontroller reset.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/8#cmd-set-step-mode)
    SetStepMode = 0x94,

    /// Temporarily sets the stepper motor coil current limit of the driver on the Tic. The provided value will override the corresponding setting from the Tic’s non-volatile memory until the next [`Command::Reset`] (or [`UsbCommand::Reinitalize`][crate::usb::UsbCommand::Reinitalize]) command or full microcontroller reset.
    ///
    /// See the description of the “Current limit” setting in [**Section 7 - Setting reference**](https://www.pololu.com/docs/0J71/6) for information about the units and allowed ranges for the 7-bit current limit argument for each different type of Tic.
    ///
    /// Note: This crate automatically converts between the proper units when using [TicBase::current_limit] and [TicBase::set_current_limit], so doing any conversions is not necessary unless implementing a new Tic type.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/8#cmd-set-current-limit)
    SetCurrentLimit = 0x91,

    /// Temporarily sets the decay mode of the driver on the Tic.
    ///
    /// For more information about the decay mode, see [**Section 7 - Setting reference**](https://www.pololu.com/docs/0J71/6) and the driver datasheet. The provided value will override the corresponding setting from the Tic’s non-volatile memory until the next [`Command::Reset`] (or [`UsbCommand::Reinitalize`][crate::usb::UsbCommand::Reinitalize]) command or full microcontroller reset. If the command contains an unrecognized decay mode, the Tic will use decay mode 0. Although the decay mode on the Tic T500 and Tic T249 is not configurable, those Tics still accept this command.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/8#cmd-set-decay-mode)
    SetDecayMode = 0x92,

    /// Only valid for the Tic T249. It temporarily changes one of the configuration options of the Active Gain Control (AGC). The provided value will override the corresponding setting from the Tic’s non-volatile memory until the next [`Command::Reset`] (or [`UsbCommand::Reinitalize`][crate::usb::UsbCommand::Reinitalize]) command or full microcontroller reset.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/8#cmd-set-agc-option)
    SetAgcOption = 0x98,

    /// Reads a block of data from the Tic’s variables; the block starts from the specified offset and can have a variable length. See [**Section 7 - Variable reference**](https://www.pololu.com/docs/0J71/7) for the offset and type of each variable.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/8#cmd-get-variable)
    GetVariable = 0xA1,

    /// This command is identical to [`Command::GetVariable`], except that it also clears the “Errors occurred” variable at the same time. The intended use of this command is to both read and clear the “Errors occurred” variable so that whenever you see a bit set in that variable, you know it indicates an error that occurred since the last “Get variable and clear errors occurred” command.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/8#cmd-get-variable-and-clear-errors-occurred)
    GetVariableAndClearErrorsOccurred = 0xA2,

    /// Reads a block of data from the Tic’s settings (stored in non-volatile memory); the block starts from the specified offset and can have a variable length. While some of the Tic’s settings can be overridden temporarily using other commands documented in this section, the settings that this command accesses are stored in non-volatile memory and can only be changed using [`UsbCommand::SetSetting`][crate::usb::UsbCommand::SetSetting]. See [**Section 7 - Setting reference**](https://www.pololu.com/docs/0J71/6) for the offset and type of each setting.
    ///
    /// [Read more](https://www.pololu.com/docs/0J71/8#cmd-get-setting)
    GetSetting = 0xA8,
}
