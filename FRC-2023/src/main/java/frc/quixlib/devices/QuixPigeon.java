package frc.quixlib.devices;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import edu.wpi.first.wpilibj.DriverStation;

public class QuixPigeon implements QuixIMU {
  private static final int kCANTimeoutMs = 100; // ms
  private final PigeonIMU pigeon;
  private final BasePigeonSimCollection sim_collection;
  private final double[] xyz_dps = new double[3]; // Roll/Pitch/Yaw rates in degrees per second
  private double continuousYawOffset = 0.0;

  public QuixPigeon(final CANDeviceID canID) {
    if (canID.CANbusName != CANDeviceID.kRIOCANbusName) {
      throw new RuntimeException("QuixPigeon must be on the RIO CAN bus.");
    }

    pigeon = new PigeonIMU(canID.deviceNumber);
    sim_collection = pigeon.getSimCollection();
    pigeon.configFactoryDefault();

    // Reduce rate of unnecessary CAN frames.
    // https://docs.ctre-phoenix.com/en/stable/ch18_CommonAPI.html#pigeon-imu
    reportError(
        pigeon.setStatusFramePeriod(
            PigeonIMU_StatusFrame.CondStatus_1_General, 1000, kCANTimeoutMs),
        "setStatusFramePeriod");
    reportError(
        pigeon.setStatusFramePeriod(
            PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 10, kCANTimeoutMs),
        "setStatusFramePeriod");
    reportError(
        pigeon.setStatusFramePeriod(
            PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 10, kCANTimeoutMs),
        "setStatusFramePeriod");
    reportError(
        pigeon.setStatusFramePeriod(
            PigeonIMU_StatusFrame.CondStatus_11_GyroAccum, 20, kCANTimeoutMs),
        "setStatusFramePeriod");
    reportError(
        pigeon.setStatusFramePeriod(
            PigeonIMU_StatusFrame.CondStatus_2_GeneralCompass, 1000, kCANTimeoutMs),
        "setStatusFramePeriod");
    reportError(
        pigeon.setStatusFramePeriod(
            PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, 1000, kCANTimeoutMs),
        "setStatusFramePeriod");
    reportError(
        pigeon.setStatusFramePeriod(
            PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, 1000, kCANTimeoutMs),
        "setStatusFramePeriod");
    reportError(
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.RawStatus_4_Mag, 1000, kCANTimeoutMs),
        "setStatusFramePeriod");
    reportError(
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 1000, kCANTimeoutMs),
        "setStatusFramePeriod");
    reportError(
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_4_Mag, 1000, kCANTimeoutMs),
        "setStatusFramePeriod");
    reportError(
        pigeon.setStatusFramePeriod(
            PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 1000, kCANTimeoutMs),
        "setStatusFramePeriod");
  }

  public void zeroContinuousYaw() {
    setContinuousYaw(0.0);
  }

  public void setContinuousYaw(final double rad) {
    continuousYawOffset = Math.toRadians(pigeon.getYaw()) - rad;
  }

  public double getRoll() {
    return pigeon.getRoll();
  }

  public double getPitch() {
    return pigeon.getPitch();
  }

  public double getContinuousYaw() {
    return Math.toRadians(pigeon.getYaw()) - continuousYawOffset;
  }

  public double getRollRate() {
    pigeon.getRawGyro(xyz_dps);
    return Math.toRadians(xyz_dps[0]);
  }

  public double getPitchRate() {
    pigeon.getRawGyro(xyz_dps);
    return Math.toRadians(xyz_dps[1]);
  }

  public double getYawRate() {
    pigeon.getRawGyro(xyz_dps);
    return Math.toRadians(xyz_dps[2]);
  }

  public void setSimContinuousYaw(final double rad) {
    sim_collection.setRawHeading(Math.toDegrees(rad));
  }

  private void reportError(final ErrorCode errorCode, final String message) {
    if (errorCode != ErrorCode.OK) {
      DriverStation.reportError(message + ": " + errorCode, /*printTrace=*/ false);
    }
  }
}
