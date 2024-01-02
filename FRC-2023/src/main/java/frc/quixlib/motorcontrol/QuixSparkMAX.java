package frc.quixlib.motorcontrol;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.quixlib.devices.CANDeviceID;

public class QuixSparkMAX implements QuixMotorControllerWithEncoder, AutoCloseable {
  private final CANSparkMax m_controller;
  private final RelativeEncoder m_encoder;
  private final SparkMaxPIDController m_pid;
  private final MechanismRatio m_ratio;
  private final QuixSparkMAXConfiguration m_config;

  public static class QuixSparkMAXConfiguration {
    private IdleMode IDLE_MODE = IdleMode.kCoast;
    private boolean ENABLE_VOLTAGE_COMPENSATION = true;
    private double VOLTAGE_COMPENSATION_SATURATION = 12.0; // V
    private boolean INVERTED = false;
    private boolean ENABLE_CURRENT_LIMIT = true;
    private int CURRENT_LIMIT = 40; // A

    public QuixSparkMAXConfiguration setBrakeMode() {
      IDLE_MODE = IdleMode.kBrake;
      return this;
    }

    public QuixSparkMAXConfiguration setInverted(final boolean inverted) {
      INVERTED = inverted;
      return this;
    }

    public QuixSparkMAXConfiguration setCurrentLimit(final int amps) {
      CURRENT_LIMIT = amps;
      return this;
    }
  }

  public static QuixSparkMAXConfiguration makeDefaultConfig() {
    return new QuixSparkMAXConfiguration();
  }

  /** Default constructor */
  public QuixSparkMAX(final CANDeviceID canID, final MechanismRatio ratio) {
    this(canID, ratio, makeDefaultConfig());
  }

  public QuixSparkMAX(final CANDeviceID canID, final MechanismRatio ratio, final boolean inverted) {
    this(canID, ratio, makeDefaultConfig().setInverted(inverted));
  }

  /** Follower constructor */
  public QuixSparkMAX(final CANDeviceID canID, final QuixSparkMAX leader) {
    this(canID, new MechanismRatio()); // Mechanism ratio for a follower is not used.
    m_controller.follow(leader.m_controller);
  }

  /** Constructor with full configuration */
  public QuixSparkMAX(
      final CANDeviceID canID, final MechanismRatio ratio, final QuixSparkMAXConfiguration config) {
    if (canID.CANbusName != CANDeviceID.kRIOCANbusName) {
      throw new RuntimeException("SparkMAX must be on the RIO CAN bus.");
    }

    m_controller = new CANSparkMax(canID.deviceNumber, MotorType.kBrushless);
    m_encoder = m_controller.getEncoder();
    m_pid = m_controller.getPIDController();
    m_ratio = ratio;
    m_config = config;
    setConfiguration();
  }

  public void setConfiguration() {
    // Set motor controller configuration.
    m_controller.restoreFactoryDefaults();
    m_controller.setIdleMode(m_config.IDLE_MODE);
    if (m_config.ENABLE_VOLTAGE_COMPENSATION) {
      m_controller.enableVoltageCompensation(m_config.VOLTAGE_COMPENSATION_SATURATION);
    } else {
      m_controller.disableVoltageCompensation();
    }
    m_controller.setInverted(m_config.INVERTED);
    if (m_config.ENABLE_CURRENT_LIMIT) {
      m_controller.setSmartCurrentLimit(m_config.CURRENT_LIMIT);
    }

    // Reduce rate of unnecessary CAN frames.
    m_controller.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    m_controller.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    m_controller.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    m_controller.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);

    // Write all settings to flash to be extra safe.
    // Flash has limited write cycles, so only do this on FMS.
    if (DriverStation.isFMSAttached()) {
      m_controller.burnFlash();
    }
  }

  public void close() {
    m_controller.close();
  }

  public int getDeviceID() {
    return m_controller.getDeviceId();
  }

  public void setPercentOutput(final double percent) {
    m_controller.set(percent);
  }

  public void setVoltageOutput(final double voltage) {
    m_controller.setVoltage(voltage);
  }

  public void setPIDConfig(final int slot, final PIDConfig config) {
    // TODO: convert gains to spark units for ease of tuning.
    m_pid.setP(config.kP, slot);
    m_pid.setI(config.kI, slot);
    m_pid.setD(config.kD, slot);
    m_pid.setFF(config.kF, slot);
    // TODO: burn flash?
  }

  public void setPositionSetpoint(final int slot, final double setpoint) {
    setPositionSetpoint(slot, setpoint, 0.0);
  }

  public void setPositionSetpoint(
      final int slot, final double setpoint, final double feedforwardVolts) {
    m_pid.setReference(
        toNativeSensorPosition(setpoint),
        CANSparkMax.ControlType.kPosition,
        slot,
        feedforwardVolts,
        SparkMaxPIDController.ArbFFUnits.kVoltage);
  }

  public void setVelocitySetpoint(final int slot, final double setpoint) {
    setVelocitySetpoint(slot, setpoint, 0.0);
  }

  public void setVelocitySetpoint(
      final int slot, final double setpoint, final double feedforwardVolts) {
    m_pid.setReference(
        toNativeSensorVelocity(setpoint),
        CANSparkMax.ControlType.kVelocity,
        slot,
        feedforwardVolts,
        SparkMaxPIDController.ArbFFUnits.kVoltage);
  }

  public double getMaxVoltage() {
    return m_config.ENABLE_VOLTAGE_COMPENSATION
        ? m_config.VOLTAGE_COMPENSATION_SATURATION
        : RobotController.getBatteryVoltage();
  }

  public double getPercentOutput() {
    return m_controller.getAppliedOutput();
  }

  public double getPhysicalPercentOutput() {
    return (getInverted() ? -1.0 : 1.0) * m_controller.getAppliedOutput();
  }

  public double getVoltageOutput() {
    return m_controller.getBusVoltage() * m_controller.getAppliedOutput();
  }

  public double getStatorCurrent() {
    return m_controller.getOutputCurrent();
  }

  public boolean getInverted() {
    return m_controller.getInverted();
  }

  public void zeroSensorPosition() {
    setSensorPosition(0.0);
  }

  public void setSensorPosition(final double pos) {
    m_encoder.setPosition(toNativeSensorPosition(pos));
  }

  public double getSensorPosition() {
    return fromNativeSensorPosition(m_encoder.getPosition());
  }

  public double getSensorVelocity() {
    return fromNativeSensorVelocity(m_encoder.getVelocity());
  }

  public MechanismRatio getMechanismRatio() {
    return m_ratio;
  }

  public double toNativeSensorPosition(final double pos) {
    final double motorRadians = m_ratio.mechanismPositionToSensorRadians(pos);
    // Native position is revolutions.
    return motorRadians / (2.0 * Math.PI);
  }

  public double fromNativeSensorPosition(final double pos) {
    return pos / toNativeSensorPosition(1.0);
  }

  public double toNativeSensorVelocity(final double vel) {
    // Native velocity is revolutions per minute.
    return toNativeSensorPosition(vel) * 60.0;
  }

  public double fromNativeSensorVelocity(final double vel) {
    return vel / toNativeSensorVelocity(1.0);
  }

  public void setSimSensorPositionAndVelocity(
      final double pos, final double vel, final double dt, final MechanismRatio mr) {
    // TODO: REV has its own simulation. Figure out how to use it.
    return;
  }

  public void setSimSensorVelocity(final double vel, final double dt, final MechanismRatio mr) {
    // TODO: REV has its own simulation. Figure out how to use it.
    return;
  }
}
