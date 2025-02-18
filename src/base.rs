use num_traits::FromPrimitive as _;

use crate::{TicAgcBottomCurrentLimit, TicAgcCurrentBoostSteps, TicAgcFrequencyLimit, TicAgcMode, TicCommand, TicDecayMode, TicError, TicMiscFlags1, TicOperationState, TicPlanningMode, TicProduct, TicStepMode};

pub trait TicBase {
    fn command_quick(&mut self, cmd: TicCommand);
    fn command_w32(&mut self, cmd: TicCommand, val: u32);
    fn command_w7(&mut self, cmd: TicCommand, val: u8);
    fn get_segment(&mut self, cmd: TicCommand, offset: u8, length: u8, buffer: &mut [u8]);

    fn get_var8(&mut self, offset: u8) -> u8 {
        let mut result = [0u8; 2];
        self.get_segment(TicCommand::GetVariable, offset, 1, &mut result);
        result[0]
    }

    fn get_var16(&mut self, offset: u8) -> u16 {
        let mut buffer = [0u8; 2];
        self.get_segment(TicCommand::GetVariable, offset, 2, &mut buffer);
        u16::from_le_bytes(buffer)
    }

    fn get_var32(&mut self, offset: u8) -> u32 {
        let mut buffer = [0u8; 4];
        self.get_segment(TicCommand::GetVariable, offset, 4, &mut buffer);
        u32::from_le_bytes(buffer)
    }

    /// You can use this function to specify what type of Tic you are using.
    ///
    /// Example usage (pick one of the following):
    /// ```
    /// tic.setProduct(TicProduct::T500);
    /// tic.setProduct(TicProduct::T834);
    /// tic.setProduct(TicProduct::T825);
    /// tic.setProduct(TicProduct::T249);
    /// tic.setProduct(TicProduct::Tic36v4);
    /// ```
    ///
    /// This changes the behavior of the [`setCurrentLimit()`] function.
    fn set_product(&mut self, product: TicProduct);

    /// Sets the target position of the Tic, in microsteps.
    ///
    /// Example usage:
    /// ```
    /// tic.setTargetPosition(100);
    /// ```
    ///
    /// This function sends a "Set target position" to the Tic.  If the Control
    /// mode is set to Serial/I2C/USB, the Tic will start moving the motor to
    /// reach the target position.  If the control mode is something other than
    /// Serial, this command will be silently ignored.
    ///
    /// See also [`getTargetPosition()`].
    fn set_target_position(&mut self, position: i32) {
        self.command_w32(TicCommand::SetTargetPosition, position as u32);
    }

    /// Sets the target velocity of the Tic, in microsteps per 10000 seconds.
    ///
    /// Example usage:
    /// ```
    /// tic.setTargetVelocity(-1800000);  // -180 steps per second
    /// ```
    ///
    /// This function sends a "Set target velocity" command to the Tic.  If the
    /// Control mode is set to Serial/I2C/USB, the Tic will start accelerating or
    /// decelerating to reach the target velocity.
    ///
    /// If the control mode is something other than Serial, this command will be
    /// silently ignored.
    ///
    /// See also getTargetVelocity().
    fn set_target_velocity(&mut self, velocity: i32) {
        self.command_w32(TicCommand::SetTargetVelocity, velocity as u32);
    }

    /// Stops the motor abruptly without respecting the deceleration limit and
    /// sets the "Current position" variable, which represents where the Tic
    /// currently thinks the motor's output is.
    ///
    /// Example usage:
    /// ```
    /// tic.haltAndSetPosition(0);
    /// ```
    ///
    /// This function sends a "Halt and set position" command to the Tic.  Besides
    /// stopping the motor and setting the current position, this command also
    /// clears the "Postion uncertain" flag, sets the "Input state" to "halt", and
    /// clears the "Input after scaling" variable.
    ///
    /// If the control mode is something other than Serial, this command will
    /// be silently ignored.
    fn halt_and_set_position(&mut self, position: i32) {
        self.command_w32(TicCommand::HaltAndSetPosition, position as u32);
    }

    /// Stops the motor abruptly without respecting the deceleration limit.
    ///
    /// Example usage:
    /// ```
    /// tic.haltAndHold();
    /// ```
    ///
    /// This function sends a "Halt and hold" command to the Tic.  Besides stopping
    /// the motor, this command also sets the "Position uncertain" flag (because
    /// the abrupt stop might cause steps to be missed), sets the "Input state" to
    /// "halt", and clears the "Input after scaling" variable.
    ///
    /// If the control mode is something other than Serial/I2C/USB, ths
    /// command will be silently ignored.
    ///
    /// See also deenergize().
    fn halt_and_hold(&mut self) {
        self.command_quick(TicCommand::HaltAndHold);
    }

    /// Tells the Tic to start its homing procedure in the reverse direction.
    ///
    /// See the "Homing" section of the Tic user's guide for details.
    ///
    /// See also goHomeForward().
    fn go_home_reverse(&mut self) {
        self.command_w7(TicCommand::GoHome, 0);
    }

    /// Tells the Tic to start its homing procedure in the forward direction.
    ///
    /// See the "Homing" section of the Tic user's guide for details.
    ///
    /// See also goHomeReverse().
    fn go_home_forward(&mut self) {
        self.command_w7(TicCommand::GoHome, 1);
    }

    /// Prevents the "Command timeout" error from happening for some time.
    ///
    /// Example usage:
    /// ```
    /// tic.resetCommandTimeout();
    /// ```
    ///
    /// This function sends a "Reset command timeout" command to the Tic.
    fn reset_command_timeout(&mut self) {
        self.command_quick(TicCommand::ResetCommandTimeout);
    }

    /// De-energizes the stepper motor coils.
    ///
    /// Example usage:
    /// ```
    /// tic.deenergize();
    /// ```
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
    /// See also haltAndHold().
    fn deenergize(&mut self) {
        self.command_quick(TicCommand::Deenergize);
    }

    /// Sends the Energize command.
    ///
    /// Example usage:
    /// ```
    /// tic.energize();
    /// ```
    ///
    /// This function sends an Energize command to the Tic, clearing the
    /// "Intentionally de-energized" error bit.  If there are no other errors,
    /// this allows the system to start up.
    fn energize(&mut self) {
        self.command_quick(TicCommand::Energize);
    }

    /// Sends the "Exit safe start" command.
    ///
    /// Example usage:
    /// ```
    /// tic.exitSafeStart();
    /// ```
    ///
    /// In Serial/I2C/USB control mode, this command causes the safe start
    /// violation error to be cleared for 200 ms.  If there are no other errors,
    /// this allows the system to start up.
    fn exit_safe_start(&mut self) {
        self.command_quick(TicCommand::ExitSafeStart);
    }

    /// Sends the "Enter safe start" command.
    ///
    /// Example usage:
    /// ```
    /// tic.enterSafeStart();
    /// ```
    ///
    /// This command has no effect if safe-start is disabled in the Tic's settings.
    ///
    /// In Serial/I2C/USB control mode, this command causes the Tic to stop the
    /// motor and set its safe start violation error bit.  An "Exit safe start"
    /// command is required before the Tic will move the motor again.
    ///
    /// See the Tic user's guide for information about what this command does in
    /// the other control modes.
    fn enter_safe_start(&mut self) {
        self.command_quick(TicCommand::EnterSafeStart);
    }

    /// Sends the Reset command.
    ///
    /// Example usage:
    /// ```
    /// tic.reset();
    /// ```
    ///
    /// This command makes the Tic forget most parts of its current state.  For
    /// more information, see the Tic user's guide.
    fn reset(&mut self) {
        self.command_quick(TicCommand::Reset);

        // The Tic's serial and I2C interfaces will be unreliable for a brief period
        // after the Tic receives the Reset command, so we delay 10 ms here.
        //delay(10);
    }

    /// Attempts to clear a motor driver error.
    ///
    /// Example usage:
    /// ```
    /// tic.clearDriverError();
    /// ```
    ///
    /// This function sends a "Clear driver error" command to the Tic.  For more
    /// information, see the Tic user's guide.
    fn clear_driver_error(&mut self) {
        self.command_quick(TicCommand::ClearDriverError);
    }

    /// Temporarily sets the maximum speed, in units of steps per 10000 seconds.
    ///
    /// Example usage:
    /// ```
    /// tic.setMaxSpeed(5550000);  // 555 steps per second
    /// ```
    ///
    /// This function sends a "Set max speed" command to the Tic.  For more
    /// information, see the Tic user's guide.
    ///
    /// See also getMaxSpeed().
    fn set_max_speed(&mut self, speed: u32) {
        self.command_w32(TicCommand::SetSpeedMax, speed);
    }

    /// Temporarily sets the starting speed, in units of steps per 10000 seconds.
    ///
    /// Example usage:
    /// ```
    /// tic.setStartingSpeed(500000);  // 50 steps per second
    /// ```
    ///
    /// This function sends a "Set starting speed" command to the Tic.  For more
    /// information, see the Tic user's guide.
    ///
    /// See also getStartingSpeed().
    fn set_starting_speed(&mut self, speed: u32) {
        self.command_w32(TicCommand::SetStartingSpeed, speed);
    }

    /// Temporarily sets the maximum acceleration, in units of steps per second
    /// per 100 seconds.
    ///
    /// Example usage:
    /// ```
    /// tic.setMaxAccel(10000);  // 100 steps per second per second
    /// ```
    ///
    /// This function sends a "Set max acceleration" command to the Tic.  For more
    /// information, see the Tic user's guide.
    ///
    /// See also getMaxAccel().
    fn set_max_accel(&mut self, accel: u32) {
        self.command_w32(TicCommand::SetAccelMax, accel);
    }

    /// Temporarily sets the maximum deceleration, in units of steps per second
    /// per 100 seconds.
    ///
    /// Example usage:
    /// ```
    /// tic.setMaxDecel(10000);  // 100 steps per second per second
    /// ```
    ///
    /// This function sends a "Set max deceleration" command to the Tic.  For more
    /// information, see the Tic user's guide.
    ///
    /// See also getMaxDecel().
    fn set_max_decel(&mut self, decel: u32) {
        self.command_w32(TicCommand::SetDecelMax, decel);
    }

    /// Temporarily sets the stepper motor's step mode, which defines how many
    /// microsteps correspond to one full step.
    ///
    /// Example usage:
    /// ```
    /// tic.setStepMode(TicStepMode::Microstep8);
    /// ```
    ///
    /// This function sends a "Set step mode" command to the Tic.  For more
    /// information, see the Tic user's guide.
    ///
    /// See also getStepMode().
    fn set_step_mode(&mut self, mode: TicStepMode) {
        self.command_w7(TicCommand::SetStepMode, mode as u8);
    }

    /// Temporarily sets the stepper motor coil current limit in milliamps.  If
    /// the desired current limit is not available, this function uses the closest
    /// current limit that is lower than the desired one.
    ///
    /// When converting the current limit from milliamps to a code to send to the
    /// Tic, this function needs to know what kind of Tic you are using.  By
    /// default, this function assumes you are using a Tic T825 or Tic T834.  If
    /// you are using a different kind of Tic, we recommend calling setProduct()
    /// some time before calling setCurrentLimit().
    ///
    /// Example usage:
    /// ```
    /// tic.setCurrentLimit(500);  // 500 mA
    /// ```
    ///
    /// This function sends a "Set current limit" command to the Tic.  For more
    /// information about this command and how to choose a good current limit, see
    /// the Tic user's guide.
    ///
    /// See also getCurrentLimit().
    fn setCurrentLimit(limit: u16);

    /// Temporarily sets the stepper motor driver's decay mode.
    ///
    /// Example usage:
    /// ```
    /// tic.setDecayMode(TicDecayMode::Slow);
    /// ```
    ///
    /// The decay modes are documented in the Tic user's guide.
    ///
    /// See also getDecayMode().
    fn set_decay_mode(&mut self, mode: TicDecayMode) {
        self.command_w7(TicCommand::SetDecayMode, mode as u8);
    }

    /// Temporarily sets the AGC mode.
    ///
    /// This is only valid for the Tic T249.
    ///
    /// See also getAgcMode().
    fn set_agc_mode(&mut self, mode: TicAgcMode) {
        self.command_w7(TicCommand::SetAgcOption, (mode as u8) & 0xF);
    }

    /// Temporarily sets the AGC bottom current limit.
    ///
    /// This is only valid for the Tic T249.
    ///
    /// See also getAgcBottomCurrentLimit().
    fn set_agc_bottom_current_limit(&mut self, limit: TicAgcBottomCurrentLimit) {
        self.command_w7(TicCommand::SetAgcOption, 0x10 | ((limit as u8) & 0xF));
    }

    /// Temporarily sets the AGC current boost steps.
    ///
    /// This is only valid for the Tic T249.
    ///
    /// See also getAgcCurrentBoostSteps().
    fn set_agc_current_boost_steps(&mut self, steps: TicAgcCurrentBoostSteps) {
        self.command_w7(TicCommand::SetAgcOption, 0x20 | ((steps as u8) & 0xF));
    }

    /// Temporarily sets the AGC frequency limit.
    ///
    /// This is only valid for the Tic T249.
    ///
    /// See also getAgcFrequencyLimit().
    fn set_agc_frequency_limit(&mut self, limit: TicAgcFrequencyLimit) {
        self.command_w7(TicCommand::SetAgcOption, 0x30 | ((limit as u8) & 0xF));
    }

    /// Gets the Tic's current operation state, which indicates whether it is
    /// operating normally or in an error state.
    ///
    /// Example usage:
    /// ```
    /// if (tic.getOperationState() != TicOperationState::Normal)
    /// {
    ///   // There is an error, or the Tic is starting up.
    /// }
    /// ```
    ///
    /// For more information, see the "Error handling" section of the Tic user's
    /// guide.
    fn get_operation_state(&mut self) -> Result<TicOperationState, TicError> {
        TicOperationState::from_u8(
            self.get_var8(VarOffset::OperationState as u8)
        ).ok_or(TicError::SerialError)
    }

    /// Returns true if the motor driver is energized (trying to send current to
    /// its outputs).
    fn get_energized(&mut self) -> bool {
        (
            self.get_var8(VarOffset::MiscFlags1 as u8) >>
            TicMiscFlags1::Energized as u8 & 1
        ) != 0
    }

    /// Gets a flag that indicates whether there has been external confirmation that
    /// the value of the Tic's "Current position" variable is correct.
    ///
    /// For more information, see the "Error handling" section of the Tic user's
    /// guide.
    fn get_position_uncertain(&mut self) -> bool {
        (
            self.get_var8(VarOffset::MiscFlags1 as u8) >>
            TicMiscFlags1::PositionUncertain as u8 & 1
        ) != 0
    }

    /// Returns true if one of the forward limit switches is active.
    fn get_forward_limit_active(&mut self) -> bool {
        (
            self.get_var8(VarOffset::MiscFlags1 as u8) >>
            TicMiscFlags1::ForwardLimitActive as u8 & 1
        ) != 0
    }

    /// Returns true if one of the reverse limit switches is active.
    fn get_reverse_limit_active(&mut self) -> bool {
        (
            self.get_var8(VarOffset::MiscFlags1 as u8) >>
            TicMiscFlags1::ReverseLimitActive as u8 & 1
        ) != 0
    }

    /// Returns true if the Tic's homing procedure is running.
    fn get_homing_active(&mut self) -> bool {
        (
            self.get_var8(VarOffset::MiscFlags1 as u8) >>
            TicMiscFlags1::HomingActive as u8 & 1
        ) != 0
    }

    /// Gets the errors that are currently stopping the motor.
    ///
    /// Each bit in the returned register represents a different error.  The bits
    /// are defined in ::TicError enum.
    ///
    /// Example usage:
    /// ```
    /// uint16_t errors = tic.getErrorStatus();
    /// if (errors & (1 << (uint8_t)TicError::LowVin))
    /// {
    ///   // handle loss of power
    /// }
    /// ```
    fn get_error_status(&mut self) -> u16 {
        return self.get_var16(VarOffset::ErrorStatus as u8);
    }

    /// Gets the errors that have occurred since the last time this function was called.
    ///
    /// Note that the Tic Control Center constantly clears the bits in this
    /// register, so if you are running the Tic Control Center then you will not
    /// be able to reliably detect errors with this function.
    ///
    /// Each bit in the returned register represents a different error.  The bits
    /// are defined in ::TicError enum.
    ///
    /// Example usage:
    /// ```
    /// uint32_t errors = tic.getErrorsOccurred();
    /// if (errors & (1 << (uint8_t)TicError::MotorDriverError))
    /// {
    ///   // handle a motor driver error
    /// }
    /// ```
    fn get_errors_occurred(&mut self) -> u32 {
        let mut result = [0u8; 4];
        self.get_segment(
            TicCommand::GetVariableAndClearErrorsOccurred,
            VarOffset::ErrorsOccurred as u8,
            4,
            &mut result
        );
        u32::from_le_bytes(result)
    }

    /// Returns the current planning mode for the Tic's step generation code.
    ///
    /// This tells us whether the Tic is sending steps, and if it is sending
    /// steps, tells us whether it is in Target Position or Target Velocity mode.
    ///
    /// Example usage:
    /// ```
    /// if (tic.getPlanningMode() == TicPlanningMode::TargetPosition)
    /// {
    ///     // The Tic is moving the stepper motor to a target position, or has
    ///     // already reached it and is at rest.
    /// }
    /// ```
    fn get_planning_mode(&mut self) -> TicPlanningMode {
        TicPlanningMode::from_u8(self.get_var8(VarOffset::PlanningMode as u8))
    }

    /// Gets the target position, in microsteps.
    ///
    /// This is only relevant if the planning mode from getPlanningMode() is
    /// TicPlanningMode::Position.
    ///
    /// See also setTargetPosition().
    int32_t getTargetPosition()
    {
        return getVar32(VarOffset::TargetPosition);
    }

    /// Gets the target velocity, in microsteps per 10000 seconds.
    ///
    /// This is only relevant if the planning mode from getPlanningMode() is
    /// TicPlanningMode::Velocity.
    ///
    /// See also setTargetVelocity().
    int32_t getTargetVelocity()
    {
        return getVar32(VarOffset::TargetVelocity);
    }

    /// Gets the current maximum speed, in microsteps per 10000 seconds.
    ///
    /// This is the current value, which could differ from the value in the Tic's
    /// settings.
    ///
    /// See also setMaxSpeed().
    uint32_t getMaxSpeed()
    {
        return getVar32(VarOffset::SpeedMax);
    }

    /// Gets the starting speed in microsteps per 10000 seconds.
    ///
    /// This is the current value, which could differ from the value in the
    /// Tic's settings.
    ///
    /// Example usage:
    /// ```
    /// uint32_t startingSpeed = tic.getStartingSpeed();
    /// ```
    ///
    /// See also setStartingSpeed().
    uint32_t getStartingSpeed()
    {
        return getVar32(VarOffset::StartingSpeed);
    }

    /// Gets the maximum acceleration, in microsteps per second per 100 seconds.
    ///
    /// This is the current value, which could differ from the value in the Tic's
    /// settings.
    ///
    /// Example usage:
    /// ```
    /// uint32_t accelMax = tic.getMaxAccel();
    /// ```
    ///
    /// See also setMaxAccel().
    uint32_t getMaxAccel()
    {
        return getVar32(VarOffset::AccelMax);
    }

    /// Gets the maximum deceleration, in microsteps per second per 100 seconds.
    ///
    /// This is the current value, which could differ from the value in the Tic's
    /// settings.
    ///
    /// Example usage:
    /// ```
    /// uint32_t decelMax = tic.getMaxDecel();
    /// ```
    ///
    /// See also setMaxDecel().
    uint32_t getMaxDecel()
    {
        return getVar32(VarOffset::DecelMax);
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
    /// See also haltAndSetPosition().
    int32_t getCurrentPosition()
    {
        return getVar32(VarOffset::CurrentPosition);
    }

    /// Gets the current velocity of the stepper motor, in microsteps per 10000
    /// seconds.
    ///
    /// Note that this is just the velocity used in the Tic's step planning
    /// algorithms, and it might not correspond to the actual velocity of the
    /// motor for various reasons.
    ///
    /// Example usage:
    /// ```
    /// int32_t velocity = tic.getCurrentVelocity();
    /// ```
    int32_t getCurrentVelocity()
    {
        return getVar32(VarOffset::CurrentVelocity);
    }

    /// Gets the acting target position, in microsteps.
    ///
    /// This is a variable used in the Tic's target position step planning
    /// algorithm, and it could be invalid while the motor is stopped.
    ///
    /// This is mainly intended for getting insight into how the Tic's algorithms
    /// work or troubleshooting issues, and most people should not use this.
    uint32_t getActingTargetPosition()
    {
        return getVar32(VarOffset::ActingTargetPosition);
    }

    /// Gets the time since the last step, in timer ticks.
    ///
    /// Each timer tick represents one third of a microsecond.  The Tic only
    /// updates this variable every 5 milliseconds or so, and it could be invalid
    /// while the motor is stopped.
    ///
    /// This is mainly intended for getting insight into how the Tic's algorithms
    /// work or troubleshooting issues, and most people should not use this.
    uint32_t getTimeSinceLastStep()
    {
        return getVar32(VarOffset::TimeSinceLastStep);
    }

    /// Gets the cause of the controller's last full microcontroller reset.
    ///
    /// Example usage:
    /// ```
    /// if (tic.getDeviceReset() == TicReset::Brownout)
    /// {
    ///   // There was a brownout reset; the power supply could not keep up.
    /// }
    /// ```
    ///
    /// The Reset command (reset()) does not affect this variable.
    TicReset getDeviceReset()
    {
        return (TicReset)self.get_var8(VarOffset::DeviceReset);
    }

    /// Gets the current measurement of the VIN voltage, in millivolts.
    ///
    /// Example usage:
    /// ```
    /// uint16_t power = tic.getVinVoltage();
    /// ```
    uint16_t getVinVoltage()
    {
        return self.get_var16(VarOffset::VinVoltage);
    }

    /// Gets the time since the last full reset of the Tic's microcontroller, in
    /// milliseconds.
    ///
    /// Example usage:
    /// ```
    /// uint32_t upTime = tic.getUpTime();
    /// ```
    ///
    /// A Reset command (reset())does not count.
    uint32_t getUpTime()
    {
        return getVar32(VarOffset::UpTime);
    }

    /// Gets the raw encoder count measured from the Tic's RX and TX lines.
    ///
    /// Example usage:
    /// ```
    /// int32_t encoderPosition = getEncoderPosition();
    /// ```
    int32_t getEncoderPosition()
    {
        return getVar32(VarOffset::EncoderPosition);
    }

    /// Gets the raw pulse width measured on the Tic's RC input, in units of
    /// twelfths of a microsecond.
    ///
    /// Returns TicInputNull if the RC input is missing or invalid.
    ///
    /// Example usage:
    /// ```
    /// uint16_t pulseWidth = tic.getRCPulseWidth();
    /// if (pulseWidth != TicInputNull && pulseWidth > 18000)
    /// {
    ///   // Pulse width is greater than 1500 microseconds.
    /// }
    /// ```
    uint16_t getRCPulseWidth()
    {
        return self.get_var16(VarOffset::RCPulseWidth);
    }

    /// Gets the analog reading from the specified pin.
    ///
    /// The reading is left-justified, so 0xFFFF represents a voltage equal to the
    /// Tic's 5V pin (approximately 4.8 V).
    ///
    /// Returns TicInputNull if the analog reading is disabled or not ready.
    ///
    /// Example usage:
    /// ```
    /// uint16_t reading = getAnalogReading(TicPin::SDA);
    /// if (reading != TicInputNull && reading < 32768)
    /// {
    ///   // The reading is less than about 2.4 V.
    /// }
    /// ```
    uint16_t getAnalogReading(TicPin pin)
    {
        uint8_t offset = VarOffset::AnalogReadingSCL + 2 * (uint8_t)pin;
        return self.get_var16(offset);
    }

    /// Gets a digital reading from the specified pin.
    ///
    /// Returns `true` for high and `false` for low.
    ///
    /// Example usage:
    /// ```
    /// if (tic.getDigitalReading(TicPin::RC))
    /// {
    ///   // Something is driving the RC pin high.
    /// }
    /// ```
    bool getDigitalReading(TicPin pin)
    {
        uint8_t readings = self.get_var8(VarOffset::DigitalReadings);
        return (readings >> (uint8_t)pin) & 1;
    }

    /// Gets the current state of a pin, i.e. what kind of input or output it is.
    ///
    /// Note that the state might be misleading if the pin is being used as a
    /// serial or I2C pin.
    ///
    /// Example usage:
    ///
    /// ```
    /// if (tic.getPinState(TicPin::SCL) == TicPinState::OutputHigh)
    /// {
    ///   // SCL is driving high.
    /// }
    /// ```
    TicPinState getPinState(TicPin pin)
    {
        uint8_t states = self.get_var8(VarOffset::PinStates);
        return (TicPinState)(states >> (2 * (uint8_t)pin) & 0b11);
    }

    /// Gets the current step mode of the stepper motor.
    ///
    /// Example usage:
    /// ```
    /// if (tic.getStepMode() == TicStepMode::Microstep8)
    /// {
    ///   // The Tic is currently using 1/8 microsteps.
    /// }
    /// ```
    TicStepMode getStepMode()
    {
        return (TicStepMode)self.get_var8(VarOffset::StepMode);
    }

    /// Gets the stepper motor coil current limit in milliamps.
    ///
    /// This is the value being used now, which could differ from the value in the
    /// Tic's settings.
    ///
    /// Example usage:
    /// ```
    /// uint16_t current = tic.getCurrentLimit();
    /// ```
    ///
    /// By default, this function assumes you are using a Tic T825 or Tic T834.
    /// If you are using a different kind of Tic, we recommend calling
    /// setProduct() some time before calling getCurrentLimit().
    ///
    /// See also setCurrentLimit().
    uint16_t getCurrentLimit();

    /// Gets the current decay mode of the stepper motor driver.
    ///
    /// Example usage:
    /// ```
    /// if (tic.getDecayMode() == TicDecayMode::Slow)
    /// {
    ///   // The Tic is in slow decay mode.
    /// }
    /// ```
    ///
    /// See setDecayMode().
    TicDecayMode getDecayMode()
    {
        return (TicDecayMode)self.get_var8(VarOffset::DecayMode);
    }

    /// Gets the current state of the Tic's main input.
    ///
    /// Example usage:
    /// ```
    /// if (tic.getInputState() == TicInputState::Position)
    /// {
    ///   // The Tic's input is specifying a target position.
    /// }
    /// ```
    ///
    /// See TicInputState for more information.
    TicInputState getInputState()
    {
        return (TicInputState)self.get_var8(VarOffset::InputState);
    }

    /// Gets a variable used in the process that converts raw RC and analog values
    /// into a motor position or speed.  This is mainly for debugging your input
    /// scaling settings in an RC or analog mode.
    ///
    /// A value of TicInputNull means the input value is not available.
    uint16_t getInputAfterAveraging()
    {
        return self.get_var16(VarOffset::InputAfterAveraging);
    }

    /// Gets a variable used in the process that converts raw RC and analog values
    /// into a motor position or speed.  This is mainly for debugging your input
    /// scaling settings in an RC or analog mode.
    ///
    /// A value of TicInputNull means the input value is not available.
    uint16_t getInputAfterHysteresis()
    {
        return self.get_var16(VarOffset::InputAfterHysteresis);
    }

    /// Gets the value of the Tic's main input after scaling has been applied.
    ///
    /// If the input is valid, this number is the target position or target
    /// velocity specified by the input.
    ///
    /// Example usage:
    /// ```
    /// if (tic.getInputAfter
    /// ```
    ///
    /// See also getInputState().
    int32_t getInputAfterScaling()
    {
        return getVar32(VarOffset::InputAfterScaling);
    }

    /// Gets the cause of the last motor driver error.
    ///
    /// This is only valid for the Tic T249.
    TicMotorDriverError getLastMotorDriverError()
    {
        return (TicMotorDriverError)self.get_var8(VarOffset::LastMotorDriverError);
    }

    /// Gets the AGC mode.
    ///
    /// This is only valid for the Tic T249.
    ///
    /// See also setAgcMode().
    TicAgcMode getAgcMode()
    {
        return (TicAgcMode)self.get_var8(VarOffset::AgcMode);
    }

    /// Gets the AGC bottom current limit.
    ///
    /// This is only valid for the Tic T249.
    ///
    /// See also setAgcBottomCurrentLimit().
    TicAgcBottomCurrentLimit getAgcBottomCurrentLimit()
    {
        return (TicAgcBottomCurrentLimit)self.get_var8(VarOffset::AgcBottomCurrentLimit);
    }

    /// Gets the AGC current boost steps.
    ///
    /// This is only valid for the Tic T249.
    ///
    /// See also setAgcCurrentBoostSteps().
    TicAgcCurrentBoostSteps getAgcCurrentBoostSteps()
    {
        return (TicAgcCurrentBoostSteps)self.get_var8(VarOffset::AgcCurrentBoostSteps);
    }

    /// Gets the AGC frequency limit.
    ///
    /// This is only valid for the Tic T249.
    ///
    /// See also setAgcFrequencyLimit().
    TicAgcFrequencyLimit getAgcFrequencyLimit()
    {
        return (TicAgcFrequencyLimit)self.get_var8(VarOffset::AgcFrequencyLimit);
    }

    /// Gets the "Last HP driver errors" variable.
    ///
    /// Each bit in this register represents an error.  If the bit is 1, the
    /// error was one of the causes of the Tic's last motor driver error.
    ///
    /// This is only valid for the Tic 36v4.
    uint8_t getLastHpDriverErrors()
    {
        return self.get_var8(VarOffset::LastHpDriverErrors);
    }

    /// Gets a contiguous block of settings from the Tic's EEPROM.
    ///
    /// The maximum length that can be fetched is 15 bytes.
    ///
    /// Example usage:
    /// ```
    /// // Get the Tic's serial device number.
    /// uint8_t deviceNumber;
    /// tic.getSetting(7, 1, &deviceNumber);
    /// ```
    ///
    /// This library does not attempt to interpret the settings and say what they
    /// mean.  If you are interested in how the settings are encoded in the Tic's
    /// EEPROM, see the "Settings reference" section of the Tic user's guide.
    void getSetting(uint8_t offset, uint8_t length, uint8_t * buffer)
    {
        self.get_segment(TicCommand::GetSetting, offset, length, buffer);
    }

    /// Returns 0 if the last communication with the device was successful, and
    /// non-zero if there was an error.
    uint8_t getLastError()
    {
        return _lastError;
    }

    /// Zero if the last communication with the device was successful, non-zero
    /// otherwise.
    uint8_t _lastError = 0;
}

enum VarOffset {
    OperationState        = 0x00, // uint8_t
    MiscFlags1            = 0x01, // uint8_t
    ErrorStatus           = 0x02, // uint16_t
    ErrorsOccurred        = 0x04, // uint32_t
    PlanningMode          = 0x09, // uint8_t
    TargetPosition        = 0x0A, // int32_t
    TargetVelocity        = 0x0E, // int32_t
    StartingSpeed         = 0x12, // uint32_t
    SpeedMax              = 0x16, // uint32_t
    DecelMax              = 0x1A, // uint32_t
    AccelMax              = 0x1E, // uint32_t
    CurrentPosition       = 0x22, // int32_t
    CurrentVelocity       = 0x26, // int32_t
    ActingTargetPosition  = 0x2A, // int32_t
    TimeSinceLastStep     = 0x2E, // uint32_t
    DeviceReset           = 0x32, // uint8_t
    VinVoltage            = 0x33, // uint16_t
    UpTime                = 0x35, // uint32_t
    EncoderPosition       = 0x39, // int32_t
    RCPulseWidth          = 0x3D, // uint16_t
    AnalogReadingSCL      = 0x3F, // uint16_t
    AnalogReadingSDA      = 0x41, // uint16_t
    AnalogReadingTX       = 0x43, // uint16_t
    AnalogReadingRX       = 0x45, // uint16_t
    DigitalReadings       = 0x47, // uint8_t
    PinStates             = 0x48, // uint8_t
    StepMode              = 0x49, // uint8_t
    CurrentLimit          = 0x4A, // uint8_t
    DecayMode             = 0x4B, // uint8_t
    InputState            = 0x4C, // uint8_t
    InputAfterAveraging   = 0x4D, // uint16_t
    InputAfterHysteresis  = 0x4F, // uint16_t
    InputAfterScaling     = 0x51, // uint16_t
    LastMotorDriverError  = 0x55, // uint8_t
    AgcMode               = 0x56, // uint8_t
    AgcBottomCurrentLimit = 0x57, // uint8_t
    AgcCurrentBoostSteps  = 0x58, // uint8_t
    AgcFrequencyLimit     = 0x59, // uint8_t
    LastHpDriverErrors    = 0xFF, // uint8_t
}
