package frc.quixlib.devices;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import frc.quixlib.phoenix.PhoenixUtil;

public class QuixPigeon2 implements QuixIMU {
  private static final double kCANTimeoutS = 0.1; // s
  private final CANDeviceID m_canID;
  private final Pigeon2 m_pigeon;
  private final Pigeon2SimState m_simState;
  private final QuixPigeon2Configuration m_config;

  private final QuixStatusSignal m_rollSignal;
  private final QuixStatusSignal m_pitchSignal;
  private final QuixStatusSignal m_yawSignal;
  private final QuixStatusSignal m_rollRateSignal;
  private final QuixStatusSignal m_pitchRateSignal;
  private final QuixStatusSignal m_yawRateSignal;

  private double continuousYawOffset = 0.0;

  public static class QuixPigeon2Configuration {
    private double mountPoseRoll = 0.0; // rads
    private double mountPosePitch = 0.0; // rads
    private double mountPoseYaw = 0.0; // rads
    private double gyroTrimX = 0.0; // rads
    private double gyroTrimY = 0.0; // rads
    private double gyroTrimZ = 0.0; // rads

    private double updateFrequencyHz = 100.0;

    public QuixPigeon2Configuration setGyroTrimZ(final double rads) {
      gyroTrimZ = rads;
      return this;
    }

    public QuixPigeon2Configuration setUpdateFrequency(final double freqHz) {
      updateFrequencyHz = freqHz;
      return this;
    }

    public Pigeon2Configuration toPigeon2Configuration() {
      Pigeon2Configuration config = new Pigeon2Configuration();

      config.MountPose.MountPoseRoll = Math.toDegrees(mountPoseYaw);
      config.MountPose.MountPosePitch = Math.toDegrees(mountPosePitch);
      config.MountPose.MountPoseYaw = Math.toDegrees(mountPoseRoll);

      config.GyroTrim.GyroScalarX = Math.toDegrees(gyroTrimX);
      config.GyroTrim.GyroScalarY = Math.toDegrees(gyroTrimY);
      config.GyroTrim.GyroScalarZ = Math.toDegrees(gyroTrimZ);

      return config;
    }
  }

  public static QuixPigeon2Configuration makeDefaultConfig() {
    return new QuixPigeon2Configuration();
  }

  /** Default constructor */
  public QuixPigeon2(final CANDeviceID canID) {
    this(canID, makeDefaultConfig());
  }

  /** Constructor with full configuration */
  public QuixPigeon2(final CANDeviceID canID, final QuixPigeon2Configuration config) {
    m_canID = canID;
    m_pigeon = new Pigeon2(canID.deviceNumber, canID.CANbusName);
    m_simState = m_pigeon.getSimState();
    m_config = config;

    m_rollSignal = new QuixStatusSignal(m_pigeon.getRoll().clone(), Math::toRadians);
    m_pitchSignal = new QuixStatusSignal(m_pigeon.getPitch().clone(), Math::toRadians);
    m_yawSignal =
        new QuixStatusSignal(
            m_pigeon.getYaw().clone(),
            (Double value) -> {
              return Math.toRadians(value) - continuousYawOffset;
            });
    m_rollRateSignal =
        new QuixStatusSignal(m_pigeon.getAngularVelocityX().clone(), Math::toRadians);
    m_pitchRateSignal =
        new QuixStatusSignal(m_pigeon.getAngularVelocityY().clone(), Math::toRadians);
    m_yawRateSignal = new QuixStatusSignal(m_pigeon.getAngularVelocityZ().clone(), Math::toRadians);

    setConfiguration();

    // Check if unlicensed.
    if (m_pigeon.getStickyFault_UnlicensedFeatureInUse().getValue()) {
      throw new RuntimeException("Pigeon2 " + m_canID + " is unlicensed!");
    }
  }

  public void setConfiguration() {
    // Set configuration.
    Pigeon2Configuration config = m_config.toPigeon2Configuration();
    PhoenixUtil.retryUntilSuccess(
        () -> m_pigeon.getConfigurator().apply(config, kCANTimeoutS),
        () -> {
          Pigeon2Configuration readConfig = new Pigeon2Configuration();
          m_pigeon.getConfigurator().refresh(readConfig, kCANTimeoutS);
          return PhoenixUtil.Pigeon2ConfigsEqual(config, readConfig);
        },
        "Pigeon2 " + m_canID + ": applyConfiguration");

    // Set update frequencies.
    PhoenixUtil.retryUntilSuccess(
        () -> m_rollSignal.setUpdateFrequency(m_config.updateFrequencyHz, kCANTimeoutS),
        () -> m_rollSignal.getAppliedUpdateFrequency() == m_config.updateFrequencyHz,
        "Pigeon2 " + m_canID + ": m_rollSignal.setUpdateFrequency()");
    PhoenixUtil.retryUntilSuccess(
        () -> m_pitchSignal.setUpdateFrequency(m_config.updateFrequencyHz, kCANTimeoutS),
        () -> m_pitchSignal.getAppliedUpdateFrequency() == m_config.updateFrequencyHz,
        "Pigeon2 " + m_canID + ": m_pitchSignal.setUpdateFrequency()");
    PhoenixUtil.retryUntilSuccess(
        () -> m_yawSignal.setUpdateFrequency(m_config.updateFrequencyHz, kCANTimeoutS),
        () -> m_yawSignal.getAppliedUpdateFrequency() == m_config.updateFrequencyHz,
        "Pigeon2 " + m_canID + ": m_yawSignal.setUpdateFrequency()");
    PhoenixUtil.retryUntilSuccess(
        () -> m_rollRateSignal.setUpdateFrequency(m_config.updateFrequencyHz, kCANTimeoutS),
        () -> m_rollRateSignal.getAppliedUpdateFrequency() == m_config.updateFrequencyHz,
        "Pigeon2 " + m_canID + ": m_rollRateSignal.setUpdateFrequency()");
    PhoenixUtil.retryUntilSuccess(
        () -> m_pitchRateSignal.setUpdateFrequency(m_config.updateFrequencyHz, kCANTimeoutS),
        () -> m_pitchRateSignal.getAppliedUpdateFrequency() == m_config.updateFrequencyHz,
        "Pigeon2 " + m_canID + ": m_pitchRateSignal.setUpdateFrequency()");
    PhoenixUtil.retryUntilSuccess(
        () -> m_yawRateSignal.setUpdateFrequency(m_config.updateFrequencyHz, kCANTimeoutS),
        () -> m_yawRateSignal.getAppliedUpdateFrequency() == m_config.updateFrequencyHz,
        "Pigeon2 " + m_canID + ": m_yawRateSignal.setUpdateFrequency()");

    // Disable all signals that have not been explicitly defined.
    PhoenixUtil.retryUntilSuccess(
        () -> m_pigeon.optimizeBusUtilization(kCANTimeoutS),
        "Pigeon2 " + m_canID + ": optimizeBusUtilization");

    // Block until we get valid signals.
    PhoenixUtil.retryUntilSuccess(
        () ->
            QuixStatusSignal.waitForAll(
                kCANTimeoutS,
                m_rollSignal,
                m_pitchSignal,
                m_yawSignal,
                m_rollRateSignal,
                m_pitchRateSignal,
                m_yawRateSignal),
        "Pigeon2 " + m_canID + ": waitForAll()");
  }

  // Expose status signals for timesync
  public QuixStatusSignal rollSignal() {
    return m_rollSignal;
  }

  public QuixStatusSignal pitchSignal() {
    return m_pitchSignal;
  }

  public QuixStatusSignal continuousYawSignal() {
    return m_yawSignal;
  }

  public QuixStatusSignal rollRateSignal() {
    return m_rollRateSignal;
  }

  public QuixStatusSignal pitchRateSignal() {
    return m_pitchRateSignal;
  }

  public QuixStatusSignal yawRateSignal() {
    return m_yawRateSignal;
  }

  public void zeroContinuousYaw() {
    setContinuousYaw(0.0);
  }

  public void setContinuousYaw(final double rad) {
    continuousYawOffset += getContinuousYaw() - rad;
  }

  public double getRoll() {
    m_rollSignal.refresh();
    return m_rollSignal.getValue();
  }

  public double getPitch() {
    m_pitchSignal.refresh();
    return m_pitchSignal.getValue();
  }

  public double getContinuousYaw() {
    m_yawSignal.refresh();
    return m_yawSignal.getValue();
  }

  public double getRollRate() {
    m_rollRateSignal.refresh();
    return m_rollRateSignal.getValue();
  }

  public double getPitchRate() {
    m_pitchRateSignal.refresh();
    return m_pitchRateSignal.getValue();
  }

  public double getYawRate() {
    m_yawRateSignal.refresh();
    return m_yawRateSignal.getValue();
  }

  public void setSimContinuousYaw(final double rad) {
    m_simState.setRawYaw(Math.toDegrees(rad));
  }
}
