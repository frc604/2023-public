package frc.quixlib.localization;

import edu.wpi.first.math.geometry.Pose2d;

public class NTOdometryMeasurement {
  private int m_id; // This can't be final because the ID gets set afterwards.
  private final Pose2d m_pose;
  private final Pose2d m_sigmas;

  public NTOdometryMeasurement(final int id, final Pose2d pose, final Pose2d sigmas) {
    m_id = id;
    m_pose = pose;
    m_sigmas = sigmas;
  }

  public void setId(final int id) {
    m_id = id;
  }

  public double[] toArray() {
    return new double[] {
      (double) m_id,
      m_pose.getX(),
      m_pose.getY(),
      m_pose.getRotation().getRadians(),
      m_sigmas.getX(),
      m_sigmas.getY(),
      m_sigmas.getRotation().getRadians()
    };
  }
}
