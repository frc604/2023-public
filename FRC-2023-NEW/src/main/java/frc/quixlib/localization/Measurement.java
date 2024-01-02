package frc.quixlib.localization;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;

public class Measurement {
  // Odom measurement
  private final Pose2d m_pose;
  private final Pose2d m_poseSigmas; // TODO: Actually use this

  // Vision measurements
  private final ArrayList<Integer> m_cameraIDs;
  private final ArrayList<Integer> m_targetIDs;
  // Fiducial corner ID is valid only for AprilTags. -1 denotes the center. IDs are numbered as
  // follows:
  // * -> +X  3 ----- 2
  // * |      |       |
  // * V      |       |
  // * +Y     0 ----- 1
  private final ArrayList<Integer> m_fiducialCornerIDs;
  private final ArrayList<Pair<Double, Double>> m_bearingsAndElevations;
  private final ArrayList<Pair<Double, Double>> m_beSigmas; // TODO: Actually use this

  public Measurement(final Pose2d pose) {
    m_pose = pose;
    m_poseSigmas = new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(0.1)); // TODO: Don't hardcode
    m_cameraIDs = new ArrayList<>();
    m_targetIDs = new ArrayList<>();
    m_fiducialCornerIDs = new ArrayList<>();
    m_bearingsAndElevations = new ArrayList<>();
    m_beSigmas = new ArrayList<>();
  }

  public void addVisionMeasurement(
      final int cameraID, final int targetID, final Pair<Double, Double> measurement) {
    addVisionMeasurement(cameraID, targetID, -1, measurement);
  }

  public void addVisionMeasurement(
      final int cameraID,
      final int targetID,
      final int fiducialCornerID,
      final Pair<Double, Double> measurement) {
    m_cameraIDs.add(cameraID);
    m_targetIDs.add(targetID);
    m_fiducialCornerIDs.add(fiducialCornerID);
    m_bearingsAndElevations.add(measurement);
    m_beSigmas.add(new Pair<>(0.1, 0.1)); // TODO: Don't hardcode
  }

  public double[] toArray(final int id) {
    final int kOdomDataLength = 6;
    final int kVisionDataLength = 7;
    final double[] data = new double[1 + kOdomDataLength + kVisionDataLength * m_cameraIDs.size()];
    data[0] = (double) id;
    data[1] = m_pose.getX();
    data[2] = m_pose.getY();
    data[3] = m_pose.getRotation().getRadians();
    data[4] = m_poseSigmas.getX();
    data[5] = m_poseSigmas.getY();
    data[6] = m_poseSigmas.getRotation().getRadians();
    for (int i = 0; i < m_cameraIDs.size(); i++) {
      data[kOdomDataLength + kVisionDataLength * i + 1] = (double) m_cameraIDs.get(i);
      data[kOdomDataLength + kVisionDataLength * i + 2] = (double) m_targetIDs.get(i);
      data[kOdomDataLength + kVisionDataLength * i + 3] = (double) m_fiducialCornerIDs.get(i);
      data[kOdomDataLength + kVisionDataLength * i + 4] = m_bearingsAndElevations.get(i).getFirst();
      data[kOdomDataLength + kVisionDataLength * i + 5] =
          m_bearingsAndElevations.get(i).getSecond();
      data[kOdomDataLength + kVisionDataLength * i + 6] = m_beSigmas.get(i).getFirst();
      data[kOdomDataLength + kVisionDataLength * i + 7] = m_beSigmas.get(i).getSecond();
    }
    return data;
  }
}
