package frc.quixlib.localization;

import edu.wpi.first.math.geometry.Pose2d;

public class NTPoseEstimate {
  private final int m_id;
  private final Pose2d m_pose;
  private final boolean m_hasVision;

  public NTPoseEstimate(final int id, final Pose2d pose, final boolean hasVision) {
    m_id = id;
    m_pose = pose;
    m_hasVision = hasVision;
  }

  public int getID() {
    return m_id;
  }

  public Pose2d getPose() {
    return m_pose;
  }

  public boolean hasVision() {
    return m_hasVision;
  }
}
