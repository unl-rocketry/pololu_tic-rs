//! Base functions

use num_traits::FromPrimitive as _;

use crate::{
    AgcBottomCurrentLimit, AgcCurrentBoostSteps, AgcFrequencyLimit, AgcMode,
    Command, DecayMode, HandlerError, InputState, Flags, MotorDriverError,
    OperationState, ControlPin, PinState, PlanningMode, Product, ResetCause, StepMode,
    TIC_03A_CURRENT_TABLE, TIC_CURRENT_UNITS, TIC_T249_CURRENT_UNITS,
};

pub mod communication {
    use crate::{Command, HandlerError};

    pub trait TicCommunication {
        fn command_quick(&mut self, cmd: Command) -> Result<(), HandlerError>;
        fn command_w32(&mut self, cmd: Command, val: u32) -> Result<(), HandlerError>;
        fn command_w7(&mut self, cmd: Command, val: u8) -> Result<(), HandlerError>;
        fn block_read(
            &mut self,
            cmd: Command,
            offset: u8,
            buffer: &mut [u8],
        ) -> Result<(), HandlerError>;

        fn get_var8(&mut self, offset: u8) -> Result<u8, HandlerError> {
            let mut result = [0u8; 1];
            self.block_read(Command::GetVariable, offset, &mut result)?;
            Ok(result[0])
        }

        fn get_var16(&mut self, offset: u8) -> Result<u16, HandlerError> {
            let mut buffer = [0u8; 2];
            self.block_read(Command::GetVariable, offset, &mut buffer)?;
            Ok(u16::from_le_bytes(buffer))
        }

        fn get_var32(&mut self, offset: u8) -> Result<u32, HandlerError> {
            let mut buffer = [0u8; 4];
            self.block_read(Command::GetVariable, offset, &mut buffer)?;
            Ok(u32::from_le_bytes(buffer))
        }
    }
}

/// Base functions implemented by all Tic devices.
///
/// This trait builds off a common interface to produce all the controls that
/// the Tic devices supply.
pub trait TicBase: communication::TicCommunication {
    /// Gets the current Tic product
    fn product(&self) -> Product;

    fn delay(&mut self, delay: core::time::Duration);

    /// Sets the target position of the Tic, in microsteps.
    ///
    /// This function sends a "Set target position" to the Tic.  If the Control
    /// mode is set to Serial/I2C/USB, the Tic will start moving the motor to
    /// reach the target position.  If the control mode is something other than
    /// Serial, this command will be silently ignored.
    ///
    /// See also [`Self::target_position()`].
    fn set_target_position(&mut self, position: i32) -> Result<(), HandlerError> {
        self.command_w32(Command::SetTargetPosition, position as u32)
    }

    /// Sets the target velocity of the Tic, in microsteps per 10000 seconds.
    ///
    /// This function sends a "Set target velocity" command to the Tic.  If the
    /// Control mode is set to Serial/I2C/USB, the Tic will start accelerating or
    /// decelerating to reach the target velocity.
    ///
    /// If the control mode is something other than Serial, this command will be
    /// silently ignored.
    ///
    /// See also [`Self::target_velocity()`].
    fn set_target_velocity(&mut self, velocity: i32) -> Result<(), HandlerError> {
        self.command_w32(Command::SetTargetVelocity, velocity as u32)
    }

    /// Stops the motor abruptly without respecting the deceleration limit and
    /// sets the "Current position" variable, which represents where the Tic
    /// currently thinks the motor's output is.
    ///
    /// This function sends a "Halt and set position" command to the Tic.  Besides
    /// stopping the motor and setting the current position, this command also
    /// clears the "Postion uncertain" flag, sets the "Input state" to "halt", and
    /// clears the "Input after scaling" variable.
    ///
    /// If the control mode is something other than Serial, this command will
    /// be silently ignored.
    fn halt_and_set_position(&mut self, position: i32) -> Result<(), HandlerError> {
        self.command_w32(Command::HaltAndSetPosition, position as u32)
    }

    /// Stops the motor abruptly without respecting the deceleration limit.
    ///
    /// This function sends a "Halt and hold" command to the Tic.  Besides stopping
    /// the motor, this command also sets the "Position uncertain" flag (because
    /// the abrupt stop might cause steps to be missed), sets the "Input state" to
    /// "halt", and clears the "Input after scaling" variable.
    ///
    /// If the control mode is something other than Serial/I2C/USB, ths
    /// command will be silently ignored.
    ///
    /// See also [`Self::deenergize()`].
    fn halt_and_hold(&mut self) -> Result<(), HandlerError> {
        self.command_quick(Command::HaltAndHold)
    }

    /// Tells the Tic to start its homing procedure in the reverse direction.
    ///
    /// See the "Homing" section of the Tic user's guide for details.
    ///
    /// See also [`Self::go_home_forward()`].
    fn go_home_reverse(&mut self) -> Result<(), HandlerError> {
        self.command_w7(Command::GoHome, 0)
    }

    /// Tells the Tic to start its homing procedure in the forward direction.
    ///
    /// See the "Homing" section of the Tic user's guide for details.
    ///
    /// See also [`Self::go_home_reverse()`].
    fn go_home_forward(&mut self) -> Result<(), HandlerError> {
        self.command_w7(Command::GoHome, 1)
    }

    /// Prevents the "Command timeout" error from happening for some time.
    ///
    /// This function sends a "Reset command timeout" command to the Tic.
    fn reset_command_timeout(&mut self) -> Result<(), HandlerError> {
        self.command_quick(Command::ResetCommandTimeout)
    }

    /// De-energizes the stepper motor coils.
    ///
    /// This function sends a De-energize command to the Tic, causing it to disable
    /// its stepper motor driver.  The motor will stop moving and consuming power.
    /// The Tic will set the "Intentionally de-energized" error bit, turn on its
    /// red LED, and drive its ERR line high.  This command also sets the
    /// "Position uncertain" flag (because the Tic is no longer in control of the
    /// motor's position).
    ///
    /// Note that the Energize command, which can be sent with energize(), will
    /// undo the effect of this command (except it will leave the "Position
    /// uncertain" flag set) and could make the system start up again.
    ///
    /// See also [`Self::halt_and_hold()`].
    fn deenergize(&mut self) -> Result<(), HandlerError> {
        self.command_quick(Command::Deenergize)
    }

    /// Sends the Energize command.
    ///
    /// This function sends an Energize command to the Tic, clearing the
    /// "Intentionally de-energized" error bit.  If there are no other errors,
    /// this allows the system to start up.
    fn energize(&mut self) -> Result<(), HandlerError> {
        self.command_quick(Command::Energize)
    }

    /// Sends the "Exit safe start" command.
    ///
    /// In Serial/I2C/USB control mode, this command causes the safe start
    /// violation error to be cleared for 200 ms.  If there are no other errors,
    /// this allows the system to start up.
    fn exit_safe_start(&mut self) -> Result<(), HandlerError> {
        self.command_quick(Command::ExitSafeStart)
    }

    /// Sends the "Enter safe start" command.
    ///
    /// This command has no effect if safe-start is disabled in the Tic's settings.
    ///
    /// In Serial/I2C/USB control mode, this command causes the Tic to stop the
    /// motor and set its safe start violation error bit.  An "Exit safe start"
    /// command is required before the Tic will move the motor again.
    ///
    /// See the Tic user's guide for information about what this command does in
    /// the other control modes.
    fn enter_safe_start(&mut self) -> Result<(), HandlerError> {
        self.command_quick(Command::EnterSafeStart)
    }

    /// Sends the Reset command.
    ///
    /// This command makes the Tic forget most parts of its current state.  For
    /// more information, see the Tic user's guide.
    fn reset(&mut self) -> Result<(), HandlerError> {
        self.command_quick(Command::Reset)?;

        // The Tic's serial and I2C interfaces will be unreliable for a brief period
        // after the Tic receives the Reset command, so we delay 10 ms here.
        self.delay(core::time::Duration::from_millis(10));

        Ok(())
    }

    /// Attempts to clear a motor driver error.
    ///
    /// This function sends a "Clear driver error" command to the Tic.  For more
    /// information, see the Tic user's guide.
    fn clear_driver_error(&mut self) -> Result<(), HandlerError> {
        self.command_quick(Command::ClearDriverError)
    }

    /// Temporarily sets the maximum speed, in units of steps per 10000 seconds.
    ///
    /// This function sends a "Set max speed" command to the Tic.  For more
    /// information, see the Tic user's guide.
    ///
    /// See also [`Self::max_speed()`].
    fn set_max_speed(&mut self, speed: u32) -> Result<(), HandlerError> {
        self.command_w32(Command::SetSpeedMax, speed)
    }

    /// Temporarily sets the starting speed, in units of steps per 10000 seconds.
    ///
    /// This function sends a "Set starting speed" command to the Tic.  For more
    /// information, see the Tic user's guide.
    ///
    /// See also [`Self::starting_speed()`].
    fn set_starting_speed(&mut self, speed: u32) -> Result<(), HandlerError> {
        self.command_w32(Command::SetStartingSpeed, speed)
    }

    /// Temporarily sets the maximum acceleration, in units of steps per second
    /// per 100 seconds.
    ///
    /// This function sends a "Set max acceleration" command to the Tic.  For more
    /// information, see the Tic user's guide.
    ///
    /// See also [`Self::max_accel()`].
    fn set_max_accel(&mut self, accel: u32) -> Result<(), HandlerError> {
        self.command_w32(Command::SetAccelMax, accel)
    }

    /// Temporarily sets the maximum deceleration, in units of steps per second
    /// per 100 seconds.
    ///
    /// This function sends a "Set max deceleration" command to the Tic.  For more
    /// information, see the Tic user's guide.
    ///
    /// See also [`Self::max_decel()`].
    fn set_max_decel(&mut self, decel: u32) -> Result<(), HandlerError> {
        self.command_w32(Command::SetDecelMax, decel)
    }

    /// Temporarily sets the stepper motor's step mode, which defines how many
    /// microsteps correspond to one full step.
    ///
    /// This function sends a "Set step mode" command to the Tic.  For more
    /// information, see the Tic user's guide.
    ///
    /// See also [`Self::step_mode()`].
    fn set_step_mode(&mut self, mode: StepMode) -> Result<(), HandlerError> {
        self.command_w7(Command::SetStepMode, mode as u8)
    }

    /// Temporarily sets the stepper motor driver's decay mode.
    ///
    /// The decay modes are documented in the Tic user's guide.
    ///
    /// See also [`Self::decay_mode()`].
    fn set_decay_mode(&mut self, mode: DecayMode) -> Result<(), HandlerError> {
        self.command_w7(Command::SetDecayMode, mode as u8)
    }

    /// Temporarily sets the AGC mode.
    ///
    /// This is only valid for the Tic T249.
    ///
    /// See also [`Self::agc_mode()`].
    fn set_agc_mode(&mut self, mode: AgcMode) -> Result<(), HandlerError> {
        self.command_w7(Command::SetAgcOption, (mode as u8) & 0xF)
    }

    /// Temporarily sets the AGC bottom current limit.
    ///
    /// This is only valid for the Tic T249.
    ///
    /// See also [`Self::agc_bottom_current_limit()`].
    fn set_agc_bottom_current_limit(
        &mut self,
        limit: AgcBottomCurrentLimit,
    ) -> Result<(), HandlerError> {
        self.command_w7(Command::SetAgcOption, 0x10 | ((limit as u8) & 0xF))
    }

    /// Temporarily sets the AGC current boost steps.
    ///
    /// This is only valid for the Tic T249.
    ///
    /// See also [`Self::agc_current_boost_steps()`].
    fn set_agc_current_boost_steps(
        &mut self,
        steps: AgcCurrentBoostSteps,
    ) -> Result<(), HandlerError> {
        self.command_w7(Command::SetAgcOption, 0x20 | ((steps as u8) & 0xF))
    }

    /// Temporarily sets the AGC frequency limit.
    ///
    /// This is only valid for the Tic T249.
    ///
    /// See also getAgcFrequencyLimit().
    fn set_agc_frequency_limit(
        &mut self,
        limit: AgcFrequencyLimit,
    ) -> Result<(), HandlerError> {
        self.command_w7(Command::SetAgcOption, 0x30 | ((limit as u8) & 0xF))
    }

    /// Gets the Tic's current operation state, which indicates whether it is
    /// operating normally or in an error state.
    ///
    /// For more information, see the "Error handling" section of the Tic user's
    /// guide.
    fn operation_state(&mut self) -> Result<OperationState, HandlerError> {
        OperationState::from_u8(self.get_var8(VarOffset::OperationState as u8)?)
            .ok_or(HandlerError::ParseError)
    }

    /// Returns true if the motor driver is energized (trying to send current to
    /// its outputs).
    fn is_energized(&mut self) -> Result<bool, HandlerError> {
        Ok(
            (self.get_var8(VarOffset::MiscFlags1 as u8)? >> Flags::Energized as u8 & 1)
                != 0,
        )
    }

    /// Gets a flag that indicates whether there has been external confirmation that
    /// the value of the Tic's "Current position" variable is correct.
    ///
    /// For more information, see the "Error handling" section of the Tic user's
    /// guide.
    fn is_position_uncertain(&mut self) -> Result<bool, HandlerError> {
        Ok(
            (self.get_var8(VarOffset::MiscFlags1 as u8)? >> Flags::PositionUncertain as u8
                & 1)
                != 0,
        )
    }

    /// Returns true if one of the forward limit switches is active.
    fn is_forward_limit_active(&mut self) -> Result<bool, HandlerError> {
        Ok(
            (self.get_var8(VarOffset::MiscFlags1 as u8)?
                >> Flags::ForwardLimitActive as u8
                & 1)
                != 0,
        )
    }

    /// Returns true if one of the reverse limit switches is active.
    fn is_reverse_limit_active(&mut self) -> Result<bool, HandlerError> {
        Ok(
            (self.get_var8(VarOffset::MiscFlags1 as u8)?
                >> Flags::ReverseLimitActive as u8
                & 1)
                != 0,
        )
    }

    /// Returns true if the Tic's homing procedure is running.
    fn is_homing_active(&mut self) -> Result<bool, HandlerError> {
        Ok(
            (self.get_var8(VarOffset::MiscFlags1 as u8)? >> Flags::HomingActive as u8 & 1)
                != 0,
        )
    }

    /// Gets the errors that are currently stopping the motor.
    ///
    /// Each bit in the returned register represents a different error.  The bits
    /// are defined in ::TicError enum.
    fn error_status(&mut self) -> Result<u16, HandlerError> {
        self.get_var16(VarOffset::ErrorStatus as u8)
    }

    /// Gets the errors that have occurred since the last time this function was called.
    ///
    /// Note that the Tic Control Center constantly clears the bits in this
    /// register, so if you are running the Tic Control Center then you will not
    /// be able to reliably detect errors with this function.
    ///
    /// Each bit in the returned register represents a different error.  The bits
    /// are defined in the [`crate::Error`] enum.
    fn errors_occurred(&mut self) -> Result<u32, HandlerError> {
        let mut result = [0u8; 4];
        self.block_read(
            Command::GetVariableAndClearErrorsOccurred,
            VarOffset::ErrorsOccurred as u8,
            &mut result,
        )?;
        Ok(u32::from_le_bytes(result))
    }

    /// Returns the current planning mode for the Tic's step generation code.
    ///
    /// This tells us whether the Tic is sending steps, and if it is sending
    /// steps, tells us whether it is in Target Position or Target Velocity mode.
    fn planning_mode(&mut self) -> Result<PlanningMode, HandlerError> {
        PlanningMode::from_u8(self.get_var8(VarOffset::PlanningMode as u8)?)
            .ok_or(HandlerError::ParseError)
    }

    /// Gets the target position, in microsteps.
    ///
    /// This is only relevant if the planning mode from getPlanningMode() is
    /// [`PlanningMode::TargetPosition`].
    ///
    /// See also [`Self::set_target_position()`].
    fn target_position(&mut self) -> Result<i32, HandlerError> {
        Ok(self.get_var32(VarOffset::TargetPosition as u8)? as i32)
    }

    /// Gets the target velocity, in microsteps per 10000 seconds.
    ///
    /// This is only relevant if the planning mode from [`Self::planning_mode()`] is
    /// [`PlanningMode::TargetVelocity`].
    ///
    /// See also [`Self::set_target_velocity()`].
    fn target_velocity(&mut self) -> Result<i32, HandlerError> {
        Ok(self.get_var32(VarOffset::TargetVelocity as u8)? as i32)
    }

    /// Gets the current maximum speed, in microsteps per 10000 seconds.
    ///
    /// This is the current value, which could differ from the value in the Tic's
    /// settings.
    ///
    /// See also [`Self::set_max_speed()`].
    fn max_speed(&mut self) -> Result<u32, HandlerError> {
        self.get_var32(VarOffset::SpeedMax as u8)
    }

    /// Gets the starting speed in microsteps per 10000 seconds.
    ///
    /// This is the current value, which could differ from the value in the
    /// Tic's settings.
    ///
    /// See also [`Self::set_starting_speed()`].
    fn starting_speed(&mut self) -> Result<u32, HandlerError> {
        self.get_var32(VarOffset::StartingSpeed as u8)
    }

    /// Gets the maximum acceleration, in microsteps per second per 100 seconds.
    ///
    /// This is the current value, which could differ from the value in the Tic's
    /// settings.
    ///
    /// See also [`Self::set_max_accel()`].
    fn max_accel(&mut self) -> Result<u32, HandlerError> {
        self.get_var32(VarOffset::AccelMax as u8)
    }

    /// Gets the maximum deceleration, in microsteps per second per 100 seconds.
    ///
    /// This is the current value, which could differ from the value in the Tic's
    /// settings.
    ///
    /// See also [`Self::set_max_decel()`].
    fn max_decel(&mut self) -> Result<u32, HandlerError> {
        self.get_var32(VarOffset::DecelMax as u8)
    }

    /// Gets the current position of the stepper motor, in microsteps.
    ///
    /// Note that this just tracks steps that the Tic has commanded the stepper
    /// driver to take; it could be different from the actual position of the
    /// motor for various reasons.
    ///
    /// For an example of how to use this this, see the SerialPositionControl
    /// example or the I2CPositionControl exmaple.
    ///
    /// See also [`Self::halt_and_set_position()`].
    fn current_position(&mut self) -> Result<i32, HandlerError> {
        Ok(self.get_var32(VarOffset::CurrentPosition as u8)? as i32)
    }

    /// Gets the current velocity of the stepper motor, in microsteps per 10000
    /// seconds.
    ///
    /// Note that this is just the velocity used in the Tic's step planning
    /// algorithms, and it might not correspond to the actual velocity of the
    /// motor for various reasons.
    fn current_velocity(&mut self) -> Result<i32, HandlerError> {
        Ok(self.get_var32(VarOffset::CurrentVelocity as u8)? as i32)
    }

    /// Gets the acting target position, in microsteps.
    ///
    /// This is a variable used in the Tic's target position step planning
    /// algorithm, and it could be invalid while the motor is stopped.
    ///
    /// This is mainly intended for getting insight into how the Tic's algorithms
    /// work or troubleshooting issues, and most people should not use this.
    fn acting_target_position(&mut self) -> Result<u32, HandlerError> {
        self.get_var32(VarOffset::ActingTargetPosition as u8)
    }

    /// Gets the time since the last step, in timer ticks.
    ///
    /// Each timer tick represents one third of a microsecond.  The Tic only
    /// updates this variable every 5 milliseconds or so, and it could be invalid
    /// while the motor is stopped.
    ///
    /// This is mainly intended for getting insight into how the Tic's algorithms
    /// work or troubleshooting issues, and most people should not use this.
    fn time_since_last_step(&mut self) -> Result<u32, HandlerError> {
        self.get_var32(VarOffset::TimeSinceLastStep as u8)
    }

    /// Gets the cause of the controller's last full microcontroller reset.
    ///
    /// The Reset command ([`Self::reset()`]) does not affect this variable.
    fn device_reset_cause(&mut self) -> Result<ResetCause, HandlerError> {
        ResetCause::from_u8(self.get_var8(VarOffset::DeviceReset as u8)?)
            .ok_or(HandlerError::ParseError)
    }

    /// Gets the current measurement of the VIN voltage, in millivolts.
    fn vin_voltage(&mut self) -> Result<u16, HandlerError> {
        self.get_var16(VarOffset::VinVoltage as u8)
    }

    /// Gets the time since the last full reset of the Tic's microcontroller, in
    /// milliseconds.
    ///
    /// A Reset command ([`Self::reset()`]) does not count.
    fn up_time(&mut self) -> Result<u32, HandlerError> {
        self.get_var32(VarOffset::UpTime as u8)
    }

    /// Gets the raw encoder count measured from the Tic's RX and TX lines.
    fn encoder_position(&mut self) -> Result<i32, HandlerError> {
        Ok(self.get_var32(VarOffset::EncoderPosition as u8)? as i32)
    }

    /// Gets the raw pulse width measured on the Tic's RC input, in units of
    /// twelfths of a microsecond.
    ///
    /// Returns TicInputNull if the RC input is missing or invalid.
    fn rc_pulse_width(&mut self) -> Result<u16, HandlerError> {
        self.get_var16(VarOffset::RCPulseWidth as u8)
    }

    /// Gets the analog reading from the specified pin.
    ///
    /// The reading is left-justified, so 0xFFFF represents a voltage equal to the
    /// Tic's 5V pin (approximately 4.8 V).
    ///
    /// Returns TicInputNull if the analog reading is disabled or not ready.
    fn analog_reading(&mut self, pin: ControlPin) -> Result<u16, HandlerError> {
        let offset = VarOffset::AnalogReadingSCL as u8 + 2 * pin as u8;
        self.get_var16(offset)
    }

    /// Gets a digital reading from the specified pin.
    ///
    /// Returns `true` for high and `false` for low.
    fn digital_reading(&mut self, pin: ControlPin) -> Result<bool, HandlerError> {
        let readings = self.get_var8(VarOffset::DigitalReadings as u8)?;
        Ok(((readings >> pin as u8) & 1) != 0)
    }

    /// Gets the current state of a pin, i.e. what kind of input or output it is.
    ///
    /// Note that the state might be misleading if the pin is being used as a
    /// serial or I2C pin.
    fn pin_state(&mut self, pin: ControlPin) -> Result<PinState, HandlerError> {
        let states = self.get_var8(VarOffset::PinStates as u8)?;
        PinState::from_u8(states >> (2 * pin as u8) & 0b11).ok_or(HandlerError::ParseError)
    }

    /// Gets the current step mode of the stepper motor.
    fn step_mode(&mut self) -> Result<StepMode, HandlerError> {
        StepMode::from_u8(self.get_var8(VarOffset::StepMode as u8)?)
            .ok_or(HandlerError::ParseError)
    }

    /// Gets the current decay mode of the stepper motor driver.
    ///
    /// See [`Self::set_decay_mode()`].
    fn decay_mode(&mut self) -> Result<DecayMode, HandlerError> {
        DecayMode::from_u8(self.get_var8(VarOffset::DecayMode as u8)?)
            .ok_or(HandlerError::ParseError)
    }

    /// Gets the current state of the Tic's main input.
    ///
    /// See [`InputState`] for more information.
    fn input_state(&mut self) -> Result<InputState, HandlerError> {
        InputState::from_u8(self.get_var8(VarOffset::InputState as u8)?)
            .ok_or(HandlerError::ParseError)
    }

    /// Gets a variable used in the process that converts raw RC and analog values
    /// into a motor position or speed.  This is mainly for debugging your input
    /// scaling settings in an RC or analog mode.
    ///
    /// A value of TicInputNull means the input value is not available.
    fn input_after_averaging(&mut self) -> Result<u16, HandlerError> {
        self.get_var16(VarOffset::InputAfterAveraging as u8)
    }

    /// Gets a variable used in the process that converts raw RC and analog values
    /// into a motor position or speed.  This is mainly for debugging your input
    /// scaling settings in an RC or analog mode.
    ///
    /// A value of TicInputNull means the input value is not available.
    fn input_after_hysteresis(&mut self) -> Result<u16, HandlerError> {
        self.get_var16(VarOffset::InputAfterHysteresis as u8)
    }

    /// Gets the value of the Tic's main input after scaling has been applied.
    ///
    /// If the input is valid, this number is the target position or target
    /// velocity specified by the input.
    ///
    /// See also [`Self::input_state()`].
    fn input_after_scaling(&mut self) -> Result<i32, HandlerError> {
        Ok(self.get_var32(VarOffset::InputAfterScaling as u8)? as i32)
    }

    /// Gets the cause of the last motor driver error.
    ///
    /// This is only valid for the Tic T249.
    fn last_motor_driver_error(&mut self) -> Result<MotorDriverError, HandlerError> {
        MotorDriverError::from_u8(self.get_var8(VarOffset::LastMotorDriverError as u8)?)
            .ok_or(HandlerError::ParseError)
    }

    /// Gets the AGC mode.
    ///
    /// This is only valid for the Tic T249.
    ///
    /// See also [`Self::set_agc_mode()`].
    fn agc_mode(&mut self) -> Result<AgcMode, HandlerError> {
        AgcMode::from_u8(self.get_var8(VarOffset::AgcMode as u8)?)
            .ok_or(HandlerError::ParseError)
    }

    /// Gets the AGC bottom current limit.
    ///
    /// This is only valid for the Tic T249.
    ///
    /// See also [`Self::set_agc_bottom_current_limit()`].
    fn agc_bottom_current_limit(&mut self) -> Result<AgcBottomCurrentLimit, HandlerError> {
        AgcBottomCurrentLimit::from_u8(self.get_var8(VarOffset::AgcBottomCurrentLimit as u8)?)
            .ok_or(HandlerError::ParseError)
    }

    /// Gets the AGC current boost steps.
    ///
    /// This is only valid for the Tic T249.
    ///
    /// See also [`Self::set_agc_current_boost_steps()`].
    fn agc_current_boost_steps(&mut self) -> Result<AgcCurrentBoostSteps, HandlerError> {
        AgcCurrentBoostSteps::from_u8(self.get_var8(VarOffset::AgcCurrentBoostSteps as u8)?)
            .ok_or(HandlerError::ParseError)
    }

    /// Gets the AGC frequency limit.
    ///
    /// This is only valid for the Tic T249.
    ///
    /// See also [`Self::set_agc_frequency_limit()`].
    fn agc_frequency_limit(&mut self) -> Result<AgcFrequencyLimit, HandlerError> {
        AgcFrequencyLimit::from_u8(self.get_var8(VarOffset::AgcFrequencyLimit as u8)?)
            .ok_or(HandlerError::ParseError)
    }

    /// Gets the "Last HP driver errors" variable.
    ///
    /// Each bit in this register represents an error.  If the bit is 1, the
    /// error was one of the causes of the Tic's last motor driver error.
    ///
    /// This is only valid for the Tic 36v4.
    fn last_hp_driver_errors(&mut self) -> Result<u8, HandlerError> {
        self.get_var8(VarOffset::LastHpDriverErrors as u8)
    }

    /// Gets a contiguous block of settings from the Tic's EEPROM.
    ///
    /// The maximum length that can be fetched is 15 bytes.
    ///
    /// This library does not attempt to interpret the settings and say what they
    /// mean.  If you are interested in how the settings are encoded in the Tic's
    /// EEPROM, see the "Settings reference" section of the Tic user's guide.
    fn get_setting(&mut self, offset: u8, buffer: &mut [u8]) -> Result<(), HandlerError> {
        self.block_read(Command::GetSetting, offset, buffer)
    }

    /// Temporarily sets the stepper motor coil current limit in milliamps.  If
    /// the desired current limit is not available, this function uses the closest
    /// current limit that is lower than the desired one.
    ///
    /// This function sends a "Set current limit" command to the Tic.  For more
    /// information about this command and how to choose a good current limit, see
    /// the Tic user's guide.
    ///
    /// See also [`Self::current_limit()`].
    fn set_current_limit(&mut self, limit: u16) -> Result<(), HandlerError> {
        let mut code = 0;

        if self.product() == Product::T500 {
            for (i, value) in TIC_03A_CURRENT_TABLE.iter().enumerate() {
                if *value <= limit {
                    code = i as u32;
                } else {
                    break;
                }
            }
        } else if self.product() == Product::T249 {
            code = limit as u32 / TIC_T249_CURRENT_UNITS as u32;
        } else if self.product() == Product::Tic36v4 {
            if limit < 72 {
                code = 0;
            } else if limit >= 9095 {
                code = 127
            } else {
                code = (limit as u32 * 768 - 55000 / 2) / 55000;
                if code < 127 && ((55000 * (code + 1) + 384) / 768) <= limit as u32 {
                    code += 1;
                }
            }
        } else {
            code = limit as u32 / TIC_CURRENT_UNITS as u32;
        }

        self.command_w7(Command::SetCurrentLimit, code as u8)?;

        Ok(())
    }

    /// Gets the stepper motor coil current limit in milliamps.
    ///
    /// This is the value being used now, which could differ from the value in the
    /// Tic's settings.
    ///
    /// See also [`Self::set_current_limit()`].
    fn current_limit(&mut self) -> Result<u16, HandlerError> {
        let mut code = self.get_var8(VarOffset::CurrentLimit as u8)? as u16;
        Ok(if self.product() == Product::T500 {
            if code > 32 {
                code = 32;
            }
            TIC_03A_CURRENT_TABLE[code as usize]
        } else if self.product() == Product::T249 {
            code * TIC_T249_CURRENT_UNITS as u16
        } else if self.product() == Product::Tic36v4 {
            (55000 * code + 384) / 768
        } else {
            code * TIC_CURRENT_UNITS as u16
        })
    }
}

enum VarOffset {
    OperationState = 0x00,       // uint8_t
    MiscFlags1 = 0x01,           // uint8_t
    ErrorStatus = 0x02,          // uint16_t
    ErrorsOccurred = 0x04,       // uint32_t
    PlanningMode = 0x09,         // uint8_t
    TargetPosition = 0x0A,       // int32_t
    TargetVelocity = 0x0E,       // int32_t
    StartingSpeed = 0x12,        // uint32_t
    SpeedMax = 0x16,             // uint32_t
    DecelMax = 0x1A,             // uint32_t
    AccelMax = 0x1E,             // uint32_t
    CurrentPosition = 0x22,      // int32_t
    CurrentVelocity = 0x26,      // int32_t
    ActingTargetPosition = 0x2A, // int32_t
    TimeSinceLastStep = 0x2E,    // uint32_t
    DeviceReset = 0x32,          // uint8_t
    VinVoltage = 0x33,           // uint16_t
    UpTime = 0x35,               // uint32_t
    EncoderPosition = 0x39,      // int32_t
    RCPulseWidth = 0x3D,         // uint16_t
    AnalogReadingSCL = 0x3F,     // uint16_t
    //AnalogReadingSDA = 0x41,      // uint16_t
    //AnalogReadingTX = 0x43,       // uint16_t
    //AnalogReadingRX = 0x45,       // uint16_t
    DigitalReadings = 0x47,       // uint8_t
    PinStates = 0x48,             // uint8_t
    StepMode = 0x49,              // uint8_t
    CurrentLimit = 0x4A,          // uint8_t
    DecayMode = 0x4B,             // uint8_t
    InputState = 0x4C,            // uint8_t
    InputAfterAveraging = 0x4D,   // uint16_t
    InputAfterHysteresis = 0x4F,  // uint16_t
    InputAfterScaling = 0x51,     // uint16_t
    LastMotorDriverError = 0x55,  // uint8_t
    AgcMode = 0x56,               // uint8_t
    AgcBottomCurrentLimit = 0x57, // uint8_t
    AgcCurrentBoostSteps = 0x58,  // uint8_t
    AgcFrequencyLimit = 0x59,     // uint8_t
    LastHpDriverErrors = 0xFF,    // uint8_t
}
