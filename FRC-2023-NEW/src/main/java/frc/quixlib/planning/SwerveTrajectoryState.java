package frc.quixlib.planning;

import edu.wpi.first.math.geometry.Pose2d;

public class SwerveTrajectoryState {
  public final Pose2d pose;
  public final double vx;
  public final double vy;
  public final double vTheta;

  public SwerveTrajectoryState(
      final Pose2d pose, final double vx, final double vy, final double vTheta) {
    this.pose = pose;
    this.vx = vx;
    this.vy = vy;
    this.vTheta = vTheta;
  }
}
