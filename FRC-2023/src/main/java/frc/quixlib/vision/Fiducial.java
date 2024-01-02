package frc.quixlib.vision;

import edu.wpi.first.math.geometry.Pose3d;

public class Fiducial {
  private final Type m_type;
  // An ID of -1 indicates this is an unlabeled fiducial (e.g. retroreflective tape)
  private final int m_id;
  private final Pose3d m_pose;
  private final double m_size;

  public static enum Type {
    RETROREFLECTIVE,
    APRILTAG,
  }

  public Fiducial(final Type type, final int id, final Pose3d pose, final double size) {
    m_type = type;
    m_id = id;
    m_pose = pose;
    m_size = size;
  }

  public Type getType() {
    return m_type;
  }

  public int id() {
    return m_id;
  }

  public Pose3d getPose() {
    return m_pose;
  }

  public double getX() {
    return m_pose.getX();
  }

  public double getY() {
    return m_pose.getY();
  }

  public double getZ() {
    return m_pose.getZ();
  }

  public double getXRot() {
    return m_pose.getRotation().getX();
  }

  public double getYRot() {
    return m_pose.getRotation().getY();
  }

  public double getZRot() {
    return m_pose.getRotation().getZ();
  }

  public double getSize() {
    return m_size;
  }
}
