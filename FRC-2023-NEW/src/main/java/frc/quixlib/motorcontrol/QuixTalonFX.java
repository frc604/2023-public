package frc.quixlib.motorcontrol;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.quixlib.devices.CANDeviceID;
import frc.quixlib.devices.QuixStatusSignal;
import frc.quixlib.phoenix.PhoenixUtil;
import frc.robot.Robot;
import java.util.function.Function;

public class QuixTalonFX implements QuixMotorControllerWithEncoder, AutoCloseable {
  private static final double kCANTimeoutS = 0.1; // s
  private static final double kDefaultUpdateFreqHz = 100.0;
  private final CANDeviceID m_canID;
  private final TalonFX m_controller;
  private final TalonFXSimState m_simState;
  private final MechanismRatio m_ratio;
  private final QuixTalonFXConfiguration m_config;

  private final DutyCycleOut m_dutyCycleControl = new DutyCycleOut(0);
  private final VoltageOut m_voltageControl = new VoltageOut(0);
  private final VelocityVoltage m_velocityControl = new VelocityVoltage(0);
  private final PositionVoltage m_positionControl = new PositionVoltage(0);

  private final QuixStatusSignal m_percentOutputSignal;
  private final QuixStatusSignal m_sensorPositionSignal;
  private final QuixStatusSignal m_sensorVelocitySignal;

  public static class QuixTalonFXConfiguration {
    private NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
    private boolean INVERTED = false;
    private double SUPPLY_CURRENT_LIMIT = 40.0; // A
    private double STATOR_CURRENT_LIMIT = 40.0; // A
    private boolean FWD_SOFT_LIMIT_ENABLED = false;
    private double FWD_SOFT_LIMIT = 0.0; // In MechanismRatio units
    private boolean REV_SOFT_LIMIT_ENABLED = false;
    private double REV_SOFT_LIMIT = 0.0; // In MechanismRatio units
    private PIDConfig slot0Config = new PIDConfig();
    private PIDConfig slot1Config = new PIDConfig();
    private PIDConfig slot2Config = new PIDConfig();
    private double sensorUpdateFrequencyHz = 100.0;

    public QuixTalonFXConfiguration setBrakeMode() {
      NEUTRAL_MODE = NeutralModeValue.Brake;
      return this;
    }

    public QuixTalonFXConfiguration setInverted(final boolean inverted) {
      INVERTED = inverted;
      return this;
    }

    public QuixTalonFXConfiguration setStatorCurrentLimit(final double amps) {
      STATOR_CURRENT_LIMIT = amps;
      return this;
    }

    public QuixTalonFXConfiguration setSupplyCurrentLimit(final double amps) {
      SUPPLY_CURRENT_LIMIT = amps;
      return this;
    }

    public QuixTalonFXConfiguration setForwardSoftLimit(final double pos) {
      FWD_SOFT_LIMIT_ENABLED = true;
      FWD_SOFT_LIMIT = pos;
      return this;
    }

    public QuixTalonFXConfiguration setReverseSoftLimit(final double pos) {
      REV_SOFT_LIMIT_ENABLED = true;
      REV_SOFT_LIMIT = pos;
      return this;
    }

    public QuixTalonFXConfiguration setPIDConfig(final int slot, final PIDConfig config) {
      switch (slot) {
        case 0:
          slot0Config = config;
          break;
        case 1:
          slot1Config = config;
          break;
        case 2:
          slot2Config = config;
          break;
        default:
          throw new RuntimeException("Invalid PID slot " + slot);
      }
      return this;
    }

    public QuixTalonFXConfiguration setSensorUpdateFrequency(final double freqHz) {
      sensorUpdateFrequencyHz = freqHz;
      return this;
    }

    public TalonFXConfiguration toTalonFXConfiguration(
        final Function<Double, Double> toNativeSensorPosition) {
      final TalonFXConfiguration config = new TalonFXConfiguration();
      config.MotorOutput.NeutralMode = NEUTRAL_MODE;
      config.MotorOutput.Inverted =
          INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
      config.MotorOutput.DutyCycleNeutralDeadband = 0.0;

      if (Robot.isReal()) {
        // TODO: Figure out why stator current limits break simulation.
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
      }
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
      config.CurrentLimits.SupplyCurrentThreshold = SUPPLY_CURRENT_LIMIT;
      config.CurrentLimits.SupplyTimeThreshold = 0.1; // s

      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = FWD_SOFT_LIMIT_ENABLED;
      config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
          toNativeSensorPosition.apply(FWD_SOFT_LIMIT);
      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = REV_SOFT_LIMIT_ENABLED;
      config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
          toNativeSensorPosition.apply(REV_SOFT_LIMIT);

      config.Voltage.SupplyVoltageTimeConstant = 0.0;
      config.Voltage.PeakForwardVoltage = 16.0;
      config.Voltage.PeakReverseVoltage = 16.0;

      config.Slot0 = slot0Config.fillCTRE(new Slot0Configs());
      config.Slot1 = slot1Config.fillCTRE(new Slot1Configs());
      config.Slot2 = slot2Config.fillCTRE(new Slot2Configs());
      return config;
    }
  }

  public static QuixTalonFXConfiguration makeDefaultConfig() {
    return new QuixTalonFXConfiguration();
  }

  /** Default constructor */
  public QuixTalonFX(final CANDeviceID canID, final MechanismRatio ratio) {
    this(canID, ratio, makeDefaultConfig());
  }

  public QuixTalonFX(final CANDeviceID canID, final MechanismRatio ratio, final boolean inverted) {
    this(canID, ratio, makeDefaultConfig().setInverted(inverted));
  }

  /** Follower constructor */
  public QuixTalonFX(final CANDeviceID canID, QuixTalonFX leader, boolean opposeLeader) {
    this(canID, new MechanismRatio()); // Mechanism ratio for a follower is not used.
    m_controller.setControl(new Follower(leader.getDeviceID(), opposeLeader));
  }

  /** Constructor with full configuration */
  public QuixTalonFX(
      final CANDeviceID canID, final MechanismRatio ratio, final QuixTalonFXConfiguration config) {
    m_canID = canID;
    m_controller = new TalonFX(canID.deviceNumber, canID.CANbusName);
    m_simState = m_controller.getSimState();
    m_ratio = ratio;
    m_config = config;

    m_percentOutputSignal = new QuixStatusSignal(m_controller.getDutyCycle());
    m_sensorPositionSignal =
        new QuixStatusSignal(m_controller.getRotorPosition(), this::fromNativeSensorPosition);
    m_sensorVelocitySignal =
        new QuixStatusSignal(m_controller.getRotorVelocity(), this::fromNativeSensorVelocity);

    // Clear reset flag.
    m_controller.hasResetOccurred();

    setConfiguration();

    // Check if unlicensed.
    if (m_controller.getStickyFault_UnlicensedFeatureInUse().getValue()) {
      throw new RuntimeException("Talon FX " + m_canID + " is unlicensed!");
    }
  }

  public void setConfiguration() {
    // Set motor controller configuration.
    final TalonFXConfiguration config =
        m_config.toTalonFXConfiguration(this::toNativeSensorPosition);
    PhoenixUtil.retryUntilSuccess(
        () -> m_controller.getConfigurator().apply(config, kCANTimeoutS),
        () -> {
          TalonFXConfiguration readConfig = new TalonFXConfiguration();
          m_controller.getConfigurator().refresh(readConfig, kCANTimeoutS);
          return PhoenixUtil.TalonFXConfigsEqual(config, readConfig);
        },
        "TalonFX " + m_canID + ": applyConfiguration");

    // Set update frequencies.
    PhoenixUtil.retryUntilSuccess(
        () ->
            m_percentOutputSignal.setUpdateFrequency(
                m_config.sensorUpdateFrequencyHz, kCANTimeoutS),
        () -> m_percentOutputSignal.getAppliedUpdateFrequency() == m_config.sensorUpdateFrequencyHz,
        "TalonFX " + m_canID + ": m_percentOutputSignal.setUpdateFrequency()");
    PhoenixUtil.retryUntilSuccess(
        () ->
            m_sensorPositionSignal.setUpdateFrequency(
                m_config.sensorUpdateFrequencyHz, kCANTimeoutS),
        () ->
            m_sensorPositionSignal.getAppliedUpdateFrequency() == m_config.sensorUpdateFrequencyHz,
        "TalonFX " + m_canID + ": m_sensorPositionSignal.setUpdateFrequency()");
    PhoenixUtil.retryUntilSuccess(
        () ->
            m_sensorVelocitySignal.setUpdateFrequency(
                m_config.sensorUpdateFrequencyHz, kCANTimeoutS),
        () ->
            m_sensorVelocitySignal.getAppliedUpdateFrequency() == m_config.sensorUpdateFrequencyHz,
        "TalonFX " + m_canID + ": m_sensorVelocitySignal.setUpdateFrequency()");

    // Disable all signals that have not been explicitly defined.
    PhoenixUtil.retryUntilSuccess(
        () -> m_controller.optimizeBusUtilization(kCANTimeoutS),
        "TalonFX " + m_canID + ": optimizeBusUtilization");

    // Block until we get valid signals.
    PhoenixUtil.retryUntilSuccess(
        () ->
            QuixStatusSignal.waitForAll(
                kCANTimeoutS,
                m_percentOutputSignal,
                m_sensorPositionSignal,
                m_sensorVelocitySignal),
        "TalonFX " + m_canID + ": waitForAll()");
  }

  public boolean checkFaultsAndReconfigureIfNecessary() {
    // TODO: Log other faults.
    if (m_controller.hasResetOccurred()) {
      DriverStation.reportError("TalonFX " + m_canID + ": reset occured", false);
      setConfiguration();
      return true;
    }
    return false;
  }

  // Expose status signals for timesync
  public QuixStatusSignal percentOutputSignal() {
    return m_percentOutputSignal;
  }

  public QuixStatusSignal sensorPositionSignal() {
    return m_sensorPositionSignal;
  }

  public QuixStatusSignal sensorVelocitySignal() {
    return m_sensorVelocitySignal;
  }

  public void close() {
    m_controller.close();
  }

  public int getDeviceID() {
    return m_controller.getDeviceID();
  }

  public void setCurrentLimit(final double amps) {
    m_config.SUPPLY_CURRENT_LIMIT = amps;
    m_config.STATOR_CURRENT_LIMIT = amps;

    // TODO: Consider a shorter non-blocking timeout
    m_controller
        .getConfigurator()
        .apply(
            m_config.toTalonFXConfiguration(this::toNativeSensorPosition).CurrentLimits,
            kCANTimeoutS);
  }

  public void setPercentOutput(final double percent) {
    m_dutyCycleControl.Output = percent;
    m_controller.setControl(m_dutyCycleControl);
  }

  public void setVoltageOutput(final double voltage) {
    setVoltageOutput(voltage, false);
  }

  public void setVoltageOutput(final double voltage, final boolean synchronous) {
    m_voltageControl.UpdateFreqHz = synchronous ? 0.0 : kDefaultUpdateFreqHz;
    m_voltageControl.Output = voltage;
    m_controller.setControl(m_voltageControl);
  }

  public void setPositionSetpoint(final int slot, final double setpoint) {
    setPositionSetpoint(slot, setpoint, 0.0);
  }

  public void setPositionSetpoint(
      final int slot, final double setpoint, final boolean synchronous) {
    setPositionSetpoint(slot, setpoint, 0.0, synchronous);
  }

  public void setPositionSetpoint(
      final int slot, final double setpoint, final double feedforwardVolts) {
    setPositionSetpoint(slot, setpoint, feedforwardVolts, false);
  }

  public void setPositionSetpoint(
      final int slot,
      final double setpoint,
      final double feedforwardVolts,
      final boolean synchronous) {
    m_positionControl.UpdateFreqHz = synchronous ? 0.0 : kDefaultUpdateFreqHz;
    m_positionControl.Slot = slot;
    m_positionControl.Position = toNativeSensorPosition(setpoint);
    m_positionControl.FeedForward = feedforwardVolts;
    m_controller.setControl(m_positionControl);
  }

  public void setVelocitySetpoint(final int slot, final double setpoint) {
    setVelocitySetpoint(slot, setpoint, 0.0);
  }

  public void setVelocitySetpoint(
      final int slot, final double setpoint, final double feedforwardVolts) {
    setVelocitySetpoint(slot, setpoint, feedforwardVolts, false);
  }

  public void setVelocitySetpoint(
      final int slot,
      final double setpoint,
      final double feedforwardVolts,
      final boolean synchronous) {
    m_velocityControl.UpdateFreqHz = synchronous ? 0.0 : kDefaultUpdateFreqHz;
    m_velocityControl.Slot = slot;
    m_velocityControl.Velocity = toNativeSensorVelocity(setpoint);
    m_velocityControl.FeedForward = feedforwardVolts;
    m_controller.setControl(m_velocityControl);
  }

  public double getPercentOutput() {
    m_percentOutputSignal.refresh();
    return m_percentOutputSignal.getValue();
  }

  public double getPhysicalPercentOutput() {
    return (getInverted() ? -1.0 : 1.0) * getPercentOutput();
  }

  public double getSupplyCurrent() {
    return m_controller.getSupplyCurrent().getValue();
  }

  public double getStatorCurrent() {
    return m_controller.getStatorCurrent().getValue();
  }

  public boolean getInverted() {
    // This assumes that the config has been properly applied.
    return m_config.INVERTED;
  }

  public void zeroSensorPosition() {
    setSensorPosition(0.0);
  }

  public void setSensorPosition(final double pos) {
    // TODO: Handle zero offset internally.
    m_controller.setPosition(toNativeSensorPosition(pos));
  }

  public double getSensorPosition() {
    m_sensorPositionSignal.refresh();
    return m_sensorPositionSignal.getValue();
  }

  public double getSensorVelocity() {
    m_sensorVelocitySignal.refresh();
    return m_sensorVelocitySignal.getValue();
  }

  public MechanismRatio getMechanismRatio() {
    return m_ratio;
  }

  public double toNativeSensorPosition(final double pos) {
    return toNativeSensorPosition(pos, m_ratio);
  }

  public static double toNativeSensorPosition(final double pos, final MechanismRatio mr) {
    // Native position is rotations. There is 1 rotation per revolution (lol).
    return mr.mechanismPositionToSensorRadians(pos) / (2.0 * Math.PI);
  }

  public double fromNativeSensorPosition(final double pos) {
    return pos / toNativeSensorPosition(1.0);
  }

  public double toNativeSensorVelocity(final double vel) {
    return toNativeSensorVelocity(vel, m_ratio);
  }

  public static double toNativeSensorVelocity(final double vel, final MechanismRatio mr) {
    // Native velocity is rotations per second.
    return toNativeSensorPosition(vel, mr);
  }

  public double fromNativeSensorVelocity(final double vel) {
    return vel / toNativeSensorVelocity(1.0);
  }

  public void setSimSensorPositionAndVelocity(
      final double pos, final double vel, final double dt, final MechanismRatio mr) {
    // Convert position into Falcon rotations.
    final double rotations = toNativeSensorPosition(pos, mr);
    // Convert velocity into Falcon rotations per second.
    final double rotationsPerSecond = toNativeSensorVelocity(vel, mr);
    // Simulated hardware is never inverted, so flip signs accordingly.
    final double sign = getInverted() ? -1.0 : 1.0;
    m_simState.setRotorVelocity(sign * rotationsPerSecond);
    m_simState.setRawRotorPosition(sign * rotations);
  }

  public void setSimSensorVelocity(final double vel, final double dt, final MechanismRatio mr) {
    // Convert velocity into Falcon rotations per second.
    final double rotationsPerSecond = toNativeSensorVelocity(vel, mr);
    // Simulated hardware is never inverted, so flip signs accordingly.
    final double sign = getInverted() ? -1.0 : 1.0;
    m_simState.setRotorVelocity(sign * rotationsPerSecond);
    m_simState.addRotorPosition(sign * rotationsPerSecond * dt);
  }
}
