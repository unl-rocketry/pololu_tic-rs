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
//! Currently, this driver supports the I²C, Serial, and USB control modes which
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
//! A basic example of using this library to set up and control a Tic36v4 is as
//! follows. Ensure you replace `<i2c_bus>` with your platform's `embedded_hal`
//! I²C interface.
//!
//! ```rust,ignore
//! use pololu_tic::{TicBase, TicI2C, Product};
//!
//! let mut tic = pololu_tic::TicI2C::new_with_address(
//!     <i2c_bus>,
//!     Product::Tic36v4,
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
#![no_std]

#[cfg(feature = "std")]
extern crate std;

mod base;

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

pub const TIC_INPUT_NULL: u16 = 0xFFFF;

/// The type of Tic driver that is being represented.
#[derive(FromPrimitive, ToPrimitive, Clone, Copy, PartialEq, Eq)]
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
    SerialFraming = 16,
    RxOverrun = 17,
    Format = 18,
    Crc = 19,
    EncoderSkip = 20,
}

/// The generic error type for anything in this crate.
#[derive(thiserror::Error, Debug)]
#[non_exhaustive]
pub enum HandlerError {
    #[error("the driver experienced an internal error")]
    InternalError(Error),

    #[cfg(feature = "i2c")]
    #[error("the i2c communication experienced an error")]
    I2cError(embedded_hal::i2c::ErrorKind),

    #[cfg(feature = "serial")]
    #[error("the serial communication experienced an error")]
    StreamError(embedded_io::ErrorKind),

    #[cfg(feature = "usb")]
    #[error("internal nusb error")]
    NusbError(nusb::Error),

    #[cfg(feature = "usb")]
    #[error("the USB communication experienced a transfer error")]
    UsbTransferError(nusb::transfer::TransferError),

    #[cfg(feature = "usb")]
    #[error("the target USB device is invalid")]
    UsbInvalidDevice(u16),

    #[error("the device is not yet initalized")]
    NotInitalized,

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
    /// [Read more](https://www.pololu.com/docs/0J71/8#cmd-set-target-position)
    SetTargetPosition = 0xE0,
    /// Sets the target velocity of the Tic, in microsteps per 10,000 seconds.
    ///
    /// [Read more]()
    SetTargetVelocity = 0xE3,
    /// Stops the motor abruptly without respecting the deceleration limit and sets the “Current position” variable, which represents what position the Tic currently thinks the motor is in. Besides stopping the motor and setting the current position, this command also clears the [`Flags::PositionUncertain`] flag, sets the input state to [`InputState::Halt`], and clears the “input after scaling” variable.
    ///
    /// [Read more]()
    HaltAndSetPosition = 0xEC,
    /// Stops the motor abruptly without respecting the deceleration limit. Besides stopping the motor, this command also sets the [`Flags::PositionUncertain`] flag (because the abrupt stop might cause steps to be missed), sets the input state to [`InputState::Halt`], and clears the “input after scaling” variable.
    ///
    /// [Read more]()
    HaltAndHold = 0x89,
    /// Starts the Tic’s homing procedure.
    ///
    /// [Read more]()
    GoHome = 0x97,
    /// If the command timeout is enabled, this command resets it and prevents the [`Error::CommandTimeout`] error from happening for some time. See [**Section 5.4 - Error handling**](https://www.pololu.com/docs/0J71/5.4) of the Tic user's guide for more information about the command timeout.
    ///
    /// [Read more]()
    ResetCommandTimeout = 0x8C,
    /// Causes the Tic to de-energize the stepper motor coils by disabling its stepper motor driver. The motor will stop moving and consuming power. This command sets the [`Flags::PositionUncertain`] flag (because the Tic is no longer in control of the motor’s position); the Tic will also set the [`Error::IntentionallyDeenergized`] error, turn on its red LED, and drive its ERR line high.
    ///
    /// [`Command::Energize`] will undo the effect of this command (except it will leave the [`Flags::PositionUncertain`] flag set) and could make the system start up again.
    ///
    /// [Read more]()
    Deenergize = 0x86,
    /// A request for the Tic to energize the stepper motor coils by enabling its stepper motor driver. The Tic will clear the [`Error::IntentionallyDeenergized`] error. If there are no other errors, this allows the system to start up.
    ///
    /// [Read more]()
    Energize = 0x85,
    /// Causes the [`Error::SafeStartViolation`] error to be cleared for 200 ms. If there are no other errors, this allows the system to start up. If the control mode is not Serial / I²C / USB, then this command is silently ignored.
    ///
    /// [Read more]()
    ExitSafeStart = 0x83,
    /// If safe start is enabled and the control mode is Serial / I²C / USB, RC speed, analog speed, or encoder speed, this command causes the Tic to stop the motor (using the configured soft error response behavior) and set its [`Error::SafeStartViolation`] error bit. If safe start is disabled, or if the Tic is not in one of the listed modes, this command will cause a brief interruption in motor control (during which the soft error response behavior will be triggered) but otherwise have no effect.
    ///
    /// [Read more]()
    EnterSafeStart = 0x8F,
    /// This command makes the Tic forget most parts of its current state. Specifically, it does the following:
    /// - Reloads all settings from the Tic’s non-volatile memory and discards any temporary changes to the settings previously made with serial commands (this applies to the step mode, current limit, decay mode, max speed, starting speed, max acceleration, and max deceleration settings)
    /// - Abruptly halts the motor
    /// - Resets the motor driver
    /// - Sets the Tic’s operation state to [`OperationState::Reset`]
    /// - Clears the last movement command and the current position
    /// - Clears the encoder position
    /// - Clears the serial and [`Error::CommandTimeout`] errors and the [`TicBase::errors_occurred`] bits
    /// - Enters safe start if configured to do so
    ///
    /// [Read more]()
    Reset = 0xB0,
    ClearDriverError = 0x8A,
    SetSpeedMax = 0xE6,
    SetStartingSpeed = 0xE5,
    SetAccelMax = 0xEA,
    SetDecelMax = 0xE9,
    SetStepMode = 0x94,
    /// Temporarily sets the stepper motor coil current limit of the driver on the Tic. The provided value will override the corresponding setting from the Tic’s non-volatile memory until the next Reset (or Reinitialize) command or full microcontroller reset.
    ///
    /// [Read more]()
    SetCurrentLimit = 0x91,
    SetDecayMode = 0x92,
    SetAgcOption = 0x98,
    GetVariable = 0xA1,
    GetVariableAndClearErrorsOccurred = 0xA2,
    GetSetting = 0xA8,
}

/// The possible operation states for the Tic.
///
/// See [`TicBase::operation_state()`].
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
pub enum OperationState {
    Reset = 0,
    Deenergized = 2,
    SoftError = 4,
    WaitingForErrLine = 6,
    StartingUp = 8,
    Normal = 10,
}

/// The possible planning modes for the Tic's step generation code.
///
/// See [`TicBase::planning_mode()`].
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
pub enum PlanningMode {
    Off = 0,
    TargetPosition = 1,
    TargetVelocity = 2,
}

/// The possible causes of a full microcontroller reset for the Tic.
///
/// See [`TicBase::device_reset_cause()`].
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
pub enum ResetCause {
    PowerUp = 0,
    Brownout = 1,
    ResetLine = 2,
    Watchdog = 4,
    Software = 8,
    StackOverflow = 16,
    StackUnderflow = 32,
}

/// The possible decay modes.
///
/// See [`TicBase::decay_mode()`] and [`TicBase::set_decay_mode()`].
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
pub enum DecayMode {
    /// This specifies "Mixed" decay mode on the Tic T825
    /// and "Mixed 50%" on the Tic T824.
    Mixed = 0,

    /// This specifies "Slow" decay mode.
    Slow = 1,

    /// This specifies "Fast" decay mode.
    Fast = 2,

    /// This specifies "Mixed 25%" decay mode on the Tic T824
    /// and is the same as [`DecayMode::Mixed`] on the Tic T825.
    Mixed25 = 3,

    /// This specifies "Mixed 75%" decay mode on the Tic T824
    /// and is the same as [`DecayMode::Mixed`] on the Tic T825.
    Mixed75 = 4,
}

/// The possible step modes.
///
/// See [`TicBase::step_mode()`] and [`TicBase::set_step_mode()`].
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
pub enum StepMode {
    Full = 0,
    Half = 1,
    Microstep4 = 2,
    Microstep8 = 3,
    Microstep16 = 4,
    Microstep32 = 5,
    Microstep2_100p = 6,
    Microstep64 = 7,
    Microstep128 = 8,
    Microstep256 = 9,
}

/// Possible AGC modes.
///
/// See [`TicBase::set_agc_mode()`] and [`TicBase::agc_mode()`].
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
pub enum AgcMode {
    Off = 0,
    On = 1,
    ActiveOff = 2,
}

/// Possible AGC bottom current limit percentages.
///
/// See [`TicBase::set_agc_bottom_current_limit()`] and [`TicBase::agc_bottom_current_limit()`].
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
pub enum AgcBottomCurrentLimit {
    P45 = 0,
    P50 = 1,
    P55 = 2,
    P60 = 3,
    P65 = 4,
    P70 = 5,
    P75 = 6,
    P80 = 7,
}

/// Possible AGC current boost steps values.
///
/// See [`TicBase::set_agc_current_boost_steps()`] and [`TicBase::agc_current_boost_steps()`].
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
pub enum AgcCurrentBoostSteps {
    S5 = 0,
    S7 = 1,
    S9 = 2,
    S11 = 3,
}

/// Possible AGC frequency limit values.
///
/// See [`TicBase::set_agc_frequency_limit()`] and [`TicBase::agc_frequency_limit()`].
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
pub enum AgcFrequencyLimit {
    Off = 0,
    F225Hz = 1,
    F450Hz = 2,
    F675Hz = 3,
}

/// The Tic's control pins.
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
pub enum ControlPin {
    SCL = 0,
    SDA = 1,
    TX = 2,
    RX = 3,
    RC = 4,
}

/// The Tic's pin states.
///
/// See [`TicBase::pin_state()`].
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
pub enum PinState {
    HighImpedance = 0,
    InputPullUp = 1,
    OutputLow = 2,
    OutputHigh = 3,
}

/// The possible states of the Tic's main input.
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
pub enum InputState {
    /// The input is not ready yet.  More samples are needed, or a command has not
    /// been received yet.
    NotReady = 0,

    /// The input is invalid.
    Invalid = 1,

    /// The input is valid and is telling the Tic to halt the motor.
    Halt = 2,

    /// The input is valid and is telling the Tic to go to a target position,
    /// which you can get with [`TicBase::input_after_scaling()`].
    Position = 3,

    /// The input is valid and is telling the Tic to go to a target velocity,
    /// which you can get with [`TicBase::input_after_scaling()`].
    Velocity = 4,
}

/// The bits in the Tic's Misc Flags 1 register.
///
/// You should not need to use this directly. See [`TicBase::is_energized()`] and
/// [`TicBase::is_position_uncertain()`].
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
pub enum Flags {
    Energized = 0,
    PositionUncertain = 1,
    ForwardLimitActive = 2,
    ReverseLimitActive = 3,
    HomingActive = 4,
}

/// Possible motor driver errors for the Tic T249.
///
/// See [`TicBase::last_motor_driver_error()`].
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
pub enum MotorDriverError {
    None = 0,
    OverCurrent = 1,
    OverTemperature = 2,
}

/// The bits in the "Last HP driver errors" variable.
///
/// See [`TicBase::last_hp_driver_errors()`].
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
pub enum HpDriverError {
    OverTemperature = 0,
    OverCurrentA = 1,
    OverCurrentB = 2,
    PreDriverFaultA = 3,
    PreDriverFaultB = 4,
    UnderVoltage = 5,
    Verify = 7,
}
