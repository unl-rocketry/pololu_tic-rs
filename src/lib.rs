//! # Pololu Tic Driver
//! A platform agnostic Rust driver for the
//! [Pololu Tic motor driver](https://www.pololu.com/tic) boards.
//! Currently this library only supports the `I2C` interface of the boards,
//! but serial support is planned.
//!
//! This library supports the same boards the equivalent
//! [Arduino library](https://github.com/pololu/tic-arduino/) supports,
//! currently the `T500`, `T834`, `T825`, `T249`, `36v4` are supported.

#![deny(unsafe_code)]
#![no_std]

pub mod base;
pub mod i2c;

#[macro_use]
extern crate num_derive;

#[doc(inline)]
pub use i2c::TicI2C;

const TIC_03A_CURRENT_TABLE: [u16; 33] = [
    0, 1, 174, 343, 495, 634, 762, 880, 990, 1092, 1189, 1281, 1368, 1452, 1532, 1611, 1687, 1762,
    1835, 1909, 1982, 2056, 2131, 2207, 2285, 2366, 2451, 2540, 2634, 2734, 2843, 2962, 3093,
];

/// The type of Tic driver that is being represented.
#[derive(FromPrimitive, ToPrimitive)]
#[derive(Clone, Copy, PartialEq, Eq)]
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

/// This is used to represent a null or missing value for some of the Tic's
/// 16-bit input variables.
const TIC_INPUT_NULL: u16 = 0xFFFF;

/// This enum defines the Tic's error bits. See the
/// "[Error handling](https://www.pololu.com/docs/0J71/5.4)" section of the Tic
/// user's guide for more information about what these errors mean.
///
/// See [`base::TicBase::error_status()`] and [`base::TicBase::errors_occurred()`].
#[derive(FromPrimitive, ToPrimitive)]
pub enum TicError {
    IntentionallyDeenergized = 0,
    MotorDriverError = 1,
    LowVin = 2,
    KillSwitch = 3,
    RequiredInputInvalid = 4,
    SerialError = 5,
    CommandTimeout = 6,
    SafeStartViolation = 7,
    ErrLineHigh = 8,
    SerialFraming = 16,
    RxOverrun = 17,
    Format = 18,
    Crc = 19,
    EncoderSkip = 20,

    UnknownError = 0xFF,
}

/// This enum defines the Tic command codes which are used for its serial, I2C,
/// and USB interface.  These codes are used by the library and you should not
/// need to use them.
#[derive(FromPrimitive, ToPrimitive)]
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

/// This enum defines the possible operation states for the Tic.
///
/// See TicBase::getOperationState().
#[derive(FromPrimitive, ToPrimitive)]
pub enum TicOperationState {
    Reset = 0,
    Deenergized = 2,
    SoftError = 4,
    WaitingForErrLine = 6,
    StartingUp = 8,
    Normal = 10,
}

/// This enum defines the possible planning modes for the Tic's step generation
/// code.
///
/// See TicBase::getPlanningMode().
#[derive(FromPrimitive, ToPrimitive)]
pub enum TicPlanningMode {
    Off = 0,
    TargetPosition = 1,
    TargetVelocity = 2,
}

/// This enum defines the possible causes of a full microcontroller reset for
/// the Tic.
///
/// See TicBase::getDeviceReset().
#[derive(FromPrimitive, ToPrimitive)]
pub enum TicReset {
    PowerUp = 0,
    Brownout = 1,
    ResetLine = 2,
    Watchdog = 4,
    Software = 8,
    StackOverflow = 16,
    StackUnderflow = 32,
}

/// This enum defines the possible decay modes.
///
/// See TicBase::getDecayMode() and TicBase::setDecayMode().
#[derive(FromPrimitive, ToPrimitive)]
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

/// This enum defines the possible step modes.
///
/// See TicBase::getStepMode() and TicBase::setStepMode().
#[derive(FromPrimitive, ToPrimitive)]
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

/// This enum defines possible AGC modes.
///
/// See TicBase::setAgcMode() and TicBase::getAgcMode().
#[derive(FromPrimitive, ToPrimitive)]
pub enum TicAgcMode {
    Off = 0,
    On = 1,
    ActiveOff = 2,
}

/// This enum defines possible AGC buttom current limit percentages.
///
/// See TicBase::setAgcBottomCurrentLimit() and
/// TicBase:getAgcBottomCurrentLimit().
#[derive(FromPrimitive, ToPrimitive)]
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

/// This enum defines possible AGC current boost steps values.
///
/// See TicBase::setAgcCurrentBoostSteps() and
/// TicBase::getAgcCurrentBoostSteps().
#[derive(FromPrimitive, ToPrimitive)]
pub enum TicAgcCurrentBoostSteps {
    S5 = 0,
    S7 = 1,
    S9 = 2,
    S11 = 3,
}

/// This enuam defines possible AGC frequency limit values.
///
/// See TicBase::setAgcFrequencyLimit() and TicBase::getAgcFrequencyLimit().
#[derive(FromPrimitive, ToPrimitive)]
pub enum TicAgcFrequencyLimit {
    Off = 0,
    F225Hz = 1,
    F450Hz = 2,
    F675Hz = 3,
}

/// This enum defines the Tic's control pins.
#[derive(FromPrimitive, ToPrimitive)]
pub enum TicPin {
    SCL = 0,
    SDA = 1,
    TX = 2,
    RX = 3,
    RC = 4,
}

/// This enum defines the Tic's pin states.
///
/// See TicBase::getPinState().
#[derive(FromPrimitive, ToPrimitive)]
pub enum TicPinState {
    HighImpedance = 0,
    InputPullUp = 1,
    OutputLow = 2,
    OutputHigh = 3,
}

/// This enum defines the possible states of the Tic's main input.
#[derive(FromPrimitive, ToPrimitive)]
pub enum TicInputState {
    /// The input is not ready yet.  More samples are needed, or a command has not
    /// been received yet.
    NotReady = 0,

    /// The input is invalid.
    Invalid = 1,

    /// The input is valid and is telling the Tic to halt the motor.
    Halt = 2,

    /// The input is valid and is telling the Tic to go to a target position,
    /// which you can get with TicBase::getInputAfterScaling().
    Position = 3,

    /// The input is valid and is telling the Tic to go to a target velocity,
    /// which you can get with TicBase::getInputAfterScaling().
    Velocity = 4,
}

/// This enum defines the bits in the Tic's Misc Flags 1 register.  You should
/// not need to use this directly.  See TicBase::getEnergized() and
/// TicBase::getPositionUncertain().
#[derive(FromPrimitive, ToPrimitive)]
pub enum TicMiscFlags1 {
    Energized = 0,
    PositionUncertain = 1,
    ForwardLimitActive = 2,
    ReverseLimitActive = 3,
    HomingActive = 4,
}

/// This enum defines possible motor driver errors for the Tic T249.
///
/// See TicBase::getLastMotorDriverError().
#[derive(FromPrimitive, ToPrimitive)]
pub enum TicMotorDriverError {
    None = 0,
    OverCurrent = 1,
    OverTemperature = 2,
}

/// This enum defines the bits in the "Last HP driver errors" variable.
///
/// See TicBase::getLastHpDriverErrors().
#[derive(FromPrimitive, ToPrimitive)]
pub enum TicHpDriverError {
    OverTemperature = 0,
    OverCurrentA = 1,
    OverCurrentB = 2,
    PreDriverFaultA = 3,
    PreDriverFaultB = 4,
    UnderVoltage = 5,
    Verify = 7,
}
