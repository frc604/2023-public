package frc.quixlib.devices;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.wpilibj.DriverStation;
import frc.quixlib.motorcontrol.MechanismRatio;

public class QuixCANCoder implements QuixAbsoluteEncoder {
  private static final int kCANTimeoutMs = 100; // ms
  private final CANCoder m_cancoder;
  private final CANCoderSimCollection m_simCollection;
  private final MechanismRatio m_ratio;

  public QuixCANCoder(final CANDeviceID canID, final MechanismRatio ratio) {
    m_cancoder = new CANCoder(canID.deviceNumber, canID.CANbusName);
    m_simCollection = m_cancoder.getSimCollection();
    m_ratio = ratio;

    // Set configuration.
    reportError(m_cancoder.configFactoryDefault(kCANTimeoutMs), "configFactoryDefault");
    reportError(
        m_cancoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360),
        "configAbsoluteSensorRange");
    reportError(
        m_cancoder.configSensorInitializationStrategy(
            SensorInitializationStrategy.BootToAbsolutePosition),
        "configSensorInitializationStrategy");
    reportError(
        m_cancoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100),
        "CANCoderStatusFrame");
    reportError(
        m_cancoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 1000),
        "CANCoderStatusFrame");
  }

  public void zero() {
    setPosition(0.0);
  }

  public void setPosition(final double pos) {
    m_cancoder.setPosition(toNativeSensorPosition(pos));
  }

  public double getPosition() {
    return fromNativeSensorPosition(m_cancoder.getPosition());
  }

  public double getAbsPosition() {
    return fromNativeSensorPosition(m_cancoder.getAbsolutePosition());
  }

  public double getVelocity() {
    return fromNativeSensorPosition(m_cancoder.getVelocity());
  }

  public double toNativeSensorPosition(final double pos) {
    return Math.toDegrees(m_ratio.mechanismPositionToSensorRadians(pos));
  }

  public double fromNativeSensorPosition(final double pos) {
    return pos / toNativeSensorPosition(1.0);
  }

  public double toNativeSensorVelocity(final double vel) {
    return toNativeSensorPosition(vel);
  }

  public double fromNativeSensorVelocity(final double vel) {
    return vel / toNativeSensorVelocity(1.0);
  }

  public void setSimSensorVelocity(final double vel, final double dt) {
    final double degreesPerSecond = toNativeSensorVelocity(vel);
    // Convert degrees per second into 12-bit units per 100ms.
    final double unitsPer100ms = degreesPerSecond * 4096 * 0.1 / 360.0;
    m_simCollection.setVelocity((int) unitsPer100ms);
    m_simCollection.addPosition((int) (unitsPer100ms * 10.0 * dt));
  }

  private void reportError(final ErrorCode errorCode, final String message) {
    if (errorCode != ErrorCode.OK) {
      DriverStation.reportError(message + ": " + errorCode, /*printTrace=*/ false);
    }
  }
}
