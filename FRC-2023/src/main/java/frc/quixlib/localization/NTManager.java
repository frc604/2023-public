package frc.quixlib.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import frc.quixlib.vision.Fiducial;
import java.util.EnumSet;
import java.util.function.Consumer;

public class NTManager {
  private final NetworkTable m_localizerTable;
  // Robot to DS odometry measurements
  private final DoubleArrayPublisher m_odomPub;
  // Robot to DS vision measurements
  private final DoubleArrayPublisher m_visionPub;
  // Robot to DS targets
  private final DoubleArrayPublisher m_targetPub;

  public NTManager(final Consumer<NTPoseEstimate> newEstimateListener) {
    final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    m_localizerTable = inst.getTable("localizer");
    m_odomPub =
        m_localizerTable.getDoubleArrayTopic("odometry").publish(PubSubOption.sendAll(true));
    m_visionPub =
        m_localizerTable.getDoubleArrayTopic("vision").publish(PubSubOption.sendAll(true));
    m_targetPub =
        m_localizerTable.getDoubleArrayTopic("targets").publish(PubSubOption.sendAll(true));

    // Setup listener for when the estimate is updated.
    final var estimatesSub =
        m_localizerTable
            .getDoubleArrayTopic("estimates")
            .subscribe(new double[] {}, PubSubOption.sendAll(true));
    inst.addListener(
        estimatesSub,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> {
          newEstimateListener.accept(
              new NTPoseEstimate(
                  (int) event.valueData.value.getDoubleArray()[0],
                  new Pose2d(
                      event.valueData.value.getDoubleArray()[1],
                      event.valueData.value.getDoubleArray()[2],
                      new Rotation2d(event.valueData.value.getDoubleArray()[3])),
                  event.valueData.value.getDoubleArray()[4] == 1.0 ? true : false));
        });
  }

  public void publishOdometry(final NTOdometryMeasurement odometry) {
    m_odomPub.set(odometry.toArray());
  }

  public void publishVision(final NTVisionMeasurement vision) {
    m_visionPub.set(vision.toArray());
  }

  public void publishTargets(final Fiducial[] targets) {
    final int kDataLength = 8;
    final double[] data = new double[kDataLength * targets.length];
    for (int i = 0; i < targets.length; i++) {
      data[kDataLength * i] = targets[i].id();
      data[kDataLength * i + 1] = targets[i].getX();
      data[kDataLength * i + 2] = targets[i].getY();
      data[kDataLength * i + 3] = targets[i].getZ();
      data[kDataLength * i + 4] = targets[i].getXRot();
      data[kDataLength * i + 5] = targets[i].getYRot();
      data[kDataLength * i + 6] = targets[i].getZRot();
      data[kDataLength * i + 7] = targets[i].getSize();
    }
    m_targetPub.set(data);
  }
}
