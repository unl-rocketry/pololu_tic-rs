//! Enums and items used by the Tic to perform functionality.

#[allow(unused_imports)]
use crate::TicBase;

/// The possible operation states for the Tic.
///
/// See [`TicBase::operation_state()`].
///
/// <style>
/// div.docblock table {
///     width: 100%;
///     max-width: 600px;
/// }
///
/// div.docblock table td {
///     width: 50%;
/// }
/// </style>
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
pub enum OperationState {
    /// | Conditions | Effects |
    /// |---|---|
    /// | • Motor driver error<br>• Low VIN | • De-energize<br>• Drive ERR line high<br>• Reset driver<br>• Clear encoder count |
    Reset = 0,
    /// | Conditions | Effects |
    /// |---|---|
    /// | • Intentionally de-energized | • De-energize<br>• Drive ERR line high |
    Deenergized = 2,
    /// | Conditions | Effects |
    /// |---|---|
    /// | • Kill switch active<br>• Required input invalid<br>• Serial error<br>• Command timeout<br>• Safe start violation | • Drive ERR line high<br>• Soft error response |
    SoftError = 4,
    /// | Conditions | Effects |
    /// |---|---|
    /// | • Err line high | • Soft error response |
    WaitingForErrLine = 6,
    /// | Conditions | Effects |
    /// |---|---|
    /// | • RC/analog input not ready<br>• Coil current stabilizing | • Energize |
    StartingUp = 8,
    /// | Conditions | Effects |
    /// |---|---|
    /// | • (None of the above) | • Energize<br>• Obey input<br>• Learn position |
    Normal = 10,
}

/// The possible planning modes for the Tic's step generation code.
///
/// See [`TicBase::planning_mode()`].
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
#[allow(missing_docs)]
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
    /// The Tic has reset after power up.
    PowerUp = 0,
    /// The microcontroller lost power due to low voltage.
    Brownout = 1,
    /// The `RST` pin was pulled low.
    ResetLine = 2,
    /// The watchdog timed out and reset. Should never happen;
    /// could indicate a firmware bug.
    Watchdog = 4,
    /// Software reset (by firmware upgrade process).
    Software = 8,
    /// Stack overflow. Should never happen;
    /// could indicate a firmware bug.
    StackOverflow = 16,
    /// Stack underflow. Should never happen;
    /// could indicate a firmware bug.
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
#[allow(missing_docs)]
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
#[allow(missing_docs)]
pub enum AgcMode {
    Off = 0,
    On = 1,
    ActiveOff = 2,
}

/// Possible AGC bottom current limit percentages.
///
/// See [`TicBase::set_agc_bottom_current_limit()`] and [`TicBase::agc_bottom_current_limit()`].
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
#[allow(missing_docs)]
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
    /// 5 steps.
    S5 = 0,
    /// 7 steps.
    S7 = 1,
    /// 9 steps.
    S9 = 2,
    /// 11 steps.
    S11 = 3,
}

/// Possible AGC frequency limit values.
///
/// See [`TicBase::set_agc_frequency_limit()`] and [`TicBase::agc_frequency_limit()`].
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
#[allow(missing_docs)]
pub enum AgcFrequencyLimit {
    Off = 0,
    F225Hz = 1,
    F450Hz = 2,
    F675Hz = 3,
}

/// The Tic's control pins.
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
#[allow(missing_docs)]
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
#[allow(missing_docs)]
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
#[non_exhaustive]
pub enum Flags {
    /// The Tic’s motor outputs are enabled and if a stepper motor is properly connected, its coils are energized (i.e. electrical current is flowing).
    Energized = 0,
    /// The Tic has not received external confirmation that the value of its “current position” variable is correct (see [**Section 5.4**](https://www.pololu.com/docs/0J71/5.4)).
    PositionUncertain = 1,
    /// One of the forward limit switches is active.
    ForwardLimitActive = 2,
    /// One of the reverse limit switches is active.
    ReverseLimitActive = 3,
    /// The Tic’s homing procedure is running.
    HomingActive = 4,
}

/// Possible motor driver errors for the Tic T249.
///
/// See [`TicBase::last_motor_driver_error()`].
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
#[allow(missing_docs)]
pub enum MotorDriverError {
    None = 0,
    OverCurrent = 1,
    OverTemperature = 2,
}

/// The bits in the "Last HP driver errors" variable.
///
/// See [`TicBase::last_hp_driver_errors()`].
#[derive(FromPrimitive, ToPrimitive, Debug, PartialEq, Eq)]
#[allow(missing_docs)]
pub enum HpDriverError {
    OverTemperature = 0,
    OverCurrentA = 1,
    OverCurrentB = 2,
    PreDriverFaultA = 3,
    PreDriverFaultB = 4,
    UnderVoltage = 5,
    Verify = 7,
}
