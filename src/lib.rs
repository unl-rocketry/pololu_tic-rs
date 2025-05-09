//! # Pololu Tic Driver
//! A platform agnostic Rust driver for the
//! [Pololu Tic motor driver](https://www.pololu.com/tic) boards.
//! Currently this library only supports the `I2C` interface of the boards,
//! but serial support is planned.
//!
//! This library supports the same boards the equivalent
//! [Arduino library](https://github.com/pololu/tic-arduino/) supports,
//! currently the `T500`, `T834`, `T825`, `T249`, `36v4` are supported.
//!
//! ## Example
//! A basic example of using this library to set up and control a Tic36v4 is as follows. Ensure you replace
//! `<i2c_bus>` with your platform's `embedded_hal` I²C interface.
//! ```rust,ignore
//! use pololu_tic::{TicBase, TicI2C, TicProduct};
//!
//! let mut tic = pololu_tic::TicI2C::new_with_address(
//!     <i2c_bus>,
//!     TicProduct::Tic36v4,
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

mod base;
mod i2c;
mod serial;

#[macro_use]
extern crate num_derive;

#[doc(inline)]
pub use i2c::TicI2C;

#[doc(inline)]
pub use serial::TicSerial;

#[doc(inline)]
pub use base::TicBase;

const TIC_03A_CURRENT_TABLE: [u16; 33] = [
    0, 1, 174, 343, 495, 634, 762, 880, 990, 1092, 1189, 1281, 1368, 1452, 1532, 1611, 1687, 1762,
    1835, 1909, 1982, 2056, 2131, 2207, 2285, 2366, 2451, 2540, 2634, 2734, 2843, 2962, 3093,
];

/// The type of Tic driver that is being represented.
#[derive(FromPrimitive, ToPrimitive, Clone, Copy, PartialEq, Eq)]
pub enum TicProduct {
    /// An unknown driver. Used if not provided, as the default.
    Unknown = 0,
    /// [Tic T825](https://www.pololu.com/product/3130)
    T825 = 1,
    /// [Tic T834](https://www.pololu.com/product/3132)
    T834 = 2,
    /// [Tic T500](https://www.pololu.com/product/3134)
    T500 = 3,
    /// [Tic T249](https://www.pololu.com/product/3138)
    T249 = 4,
    /// [Tic T825](https://www.pololu.com/product/3130)
    Tic36v4 = 5,
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
pub enum TicError {
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
pub enum TicHandlerError {
    #[error("the driver experienced an internal error")]
    InternalError(TicError),

    #[error("the i2c connection experienced an error")]
    I2cError(embedded_hal::i2c::ErrorKind),

    #[error("the i2c connection experienced an error")]
    StreamError(embedded_io::ErrorKind),

    #[error("the value could not be parsed")]
    ParseError,
}

impl From<embedded_hal::i2c::ErrorKind> for TicHandlerError {
    fn from(err: embedded_hal::i2c::ErrorKind) -> Self {
        Self::I2cError(err)
    }
}

impl From<embedded_io::ErrorKind> for TicHandlerError {
    fn from(err: embedded_io::ErrorKind) -> Self {
        Self::StreamError(err)
    }
}

/// The Tic command codes which are used for its serial, I2C, and USB interface.
/// These codes are used by the library and you should not need to use them.
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq, Clone, Copy)]
pub enum TicCommand {
    SetTargetPosition = 0xE0,
    SetTargetVelocity = 0xE3,
    HaltAndSetPosition = 0xEC,
    HaltAndHold = 0x89,
    GoHome = 0x97,
    ResetCommandTimeout = 0x8C,
    Deenergize = 0x86,
    Energize = 0x85,
    ExitSafeStart = 0x83,
    EnterSafeStart = 0x8F,
    Reset = 0xB0,
    ClearDriverError = 0x8A,
    SetSpeedMax = 0xE6,
    SetStartingSpeed = 0xE5,
    SetAccelMax = 0xEA,
    SetDecelMax = 0xE9,
    SetStepMode = 0x94,
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
pub enum TicOperationState {
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
pub enum TicPlanningMode {
    Off = 0,
    TargetPosition = 1,
    TargetVelocity = 2,
}

/// The possible causes of a full microcontroller reset for the Tic.
///
/// See [`TicBase::device_reset_cause()`].
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
pub enum TicReset {
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
pub enum TicDecayMode {
    /// This specifies "Mixed" decay mode on the Tic T825
    /// and "Mixed 50%" on the Tic T824.
    Mixed = 0,

    /// This specifies "Slow" decay mode.
    Slow = 1,

    /// This specifies "Fast" decay mode.
    Fast = 2,

    /// This specifies "Mixed 25%" decay mode on the Tic T824
    /// and is the same as TicDecayMode::Mixed on the Tic T825.
    Mixed25 = 3,

    /// This specifies "Mixed 75%" decay mode on the Tic T824
    /// and is the same as TicDecayMode::Mixed on the Tic T825.
    Mixed75 = 4,
}

/// The possible step modes.
///
/// See [`TicBase::step_mode()`] and [`TicBase::set_step_mode()`].
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
pub enum TicStepMode {
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
pub enum TicAgcMode {
    Off = 0,
    On = 1,
    ActiveOff = 2,
}

/// Possible AGC buttom current limit percentages.
///
/// See [`TicBase::set_agc_bottom_current_limit()`] and [`TicBase::agc_bottom_current_limit()`].
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
pub enum TicAgcBottomCurrentLimit {
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
pub enum TicAgcCurrentBoostSteps {
    S5 = 0,
    S7 = 1,
    S9 = 2,
    S11 = 3,
}

/// Possible AGC frequency limit values.
///
/// See [`TicBase::set_agc_frequency_limit()`] and [`TicBase::agc_frequency_limit()`].
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
pub enum TicAgcFrequencyLimit {
    Off = 0,
    F225Hz = 1,
    F450Hz = 2,
    F675Hz = 3,
}

/// The Tic's control pins.
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
pub enum TicPin {
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
pub enum TicPinState {
    HighImpedance = 0,
    InputPullUp = 1,
    OutputLow = 2,
    OutputHigh = 3,
}

/// The possible states of the Tic's main input.
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
pub enum TicInputState {
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

/// The bits in the Tic's Misc Flags 1 register.  You should
/// not need to use this directly. See [`TicBase::is_energized()`] and [`TicBase::is_position_uncertain()`].
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
pub enum TicMiscFlags1 {
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
pub enum TicMotorDriverError {
    None = 0,
    OverCurrent = 1,
    OverTemperature = 2,
}

/// The bits in the "Last HP driver errors" variable.
///
/// See [`TicBase::last_hp_driver_errors()`].
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
pub enum TicHpDriverError {
    OverTemperature = 0,
    OverCurrentA = 1,
    OverCurrentB = 2,
    PreDriverFaultA = 3,
    PreDriverFaultB = 4,
    UnderVoltage = 5,
    Verify = 7,
}
