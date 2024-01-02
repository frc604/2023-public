package frc.quixlib.motorcontrol;

interface QuixMotorControllerWithEncoder {
  /** Returns the CAN ID of the device. */
  public int getDeviceID();

  /**
   * Configures the motor. Should be called on construction or when recovering from power loss or
   * fault.
   */
  public void setConfiguration();

  // ==================== Motor Setters ====================

  /** Sets percent output from -1.0 to 1.0. */
  public void setPercentOutput(double percent);

  /** Sets voltage output. */
  public void setVoltageOutput(double voltage);

  /** Closed-loop position mode. Position setpoint defined in MechanismRatio units. */
  public void setPositionSetpoint(int slot, double setpoint);

  /**
   * Closed-loop position mode with feed-forward. Position setpoint defined in MechanismRatio units.
   */
  public void setPositionSetpoint(int slot, double setpoint, double feedforwardVolts);

  /** Closed-loop velocity mode. Velocity setpoint defined in MechanismRatio units. */
  public void setVelocitySetpoint(int slot, double setpoint);

  /**
   * Closed-loop velocity mode with feed-forward. Velocity setpoint defined in MechanismRatio units.
   */
  public void setVelocitySetpoint(int slot, double setpoint, double feedforwardVolts);

  // ==================== Motor Getters ====================

  /** Returns the percent output (-1.0 to 1.0) as reported by the device. */
  public double getPercentOutput();

  /**
   * Returns the percent output (-1.0 to 1.0) as reported by the device. Sign corresponds to the
   * physical direction of the motor. Useful for simulation.
   */
  public double getPhysicalPercentOutput();

  /** Returns the stator current draw in amps. */
  public double getStatorCurrent();

  /** Returns whether the motor is inverted. */
  public boolean getInverted();

  // ==================== Sensor Setters ====================

  /** Sets the sensor zero-point to the current position. */
  public void zeroSensorPosition();

  /** Sets the sensor position to the given value. Uses MechanismRatio units. */
  public void setSensorPosition(double pos);

  // ==================== Sensor Getters ====================

  /** Returns the sensor position. Uses MechanismRatio units. */
  public double getSensorPosition();

  /** Returns the sensor velocity. Uses MechanismRatio units. */
  public double getSensorVelocity();

  // ==================== Unit Conversions ====================

  /** Returns the MechaismRatio. */
  public MechanismRatio getMechanismRatio();

  /** Convert MechanismRatio position to native sensor position. */
  public double toNativeSensorPosition(double pos);

  /** Convert from native sensor position. Returns position in MechanismRatio units. */
  public double fromNativeSensorPosition(double pos);

  /** Convert MechanismRatio velocity to native sensor velocity. */
  public double toNativeSensorVelocity(double vel);

  /** Convert from native sensor velocity. Returns velocity in MechanismRatio units. */
  public double fromNativeSensorVelocity(double vel);

  // ==================== Simulation ====================

  /** Sets the simulated angular position and velocity of the sensor in mechanism units. */
  public void setSimSensorPositionAndVelocity(double pos, double vel, double dt, MechanismRatio mr);

  /**
   * Sets the simulated angular velocity of the sensor in mechanism units. Also sets the simulated
   * position.
   */
  public void setSimSensorVelocity(double vel, double dt, MechanismRatio mr);
}
