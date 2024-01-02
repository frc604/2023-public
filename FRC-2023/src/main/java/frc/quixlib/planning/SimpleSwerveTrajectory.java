package frc.quixlib.planning;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.quixlib.math.MathUtils;
import frc.quixlib.swerve.FieldSpeeds;

public class SimpleSwerveTrajectory {
  private final TrapezoidProfile m_xTrap;
  private final TrapezoidProfile m_yTrap;
  private final TrapezoidProfile m_thetaTrap;

  /**
   * Naive acceleration-limited trajectory starting at startPose and startSpeeds and ends at
   * targetPose with zero velocity.
   */
  public SimpleSwerveTrajectory(
      final Pose2d startPose,
      final FieldSpeeds startSpeeds,
      final Pose2d targetPose,
      final Constraints xyConstraints,
      final Constraints thetaConstraints) {
    final Rotation2d startRotation = startPose.getRotation();

    m_xTrap =
        new TrapezoidProfile(
            xyConstraints,
            new State(targetPose.getX(), 0.0),
            new State(startPose.getX(), startSpeeds.vxMetersPerSecond));
    m_yTrap =
        new TrapezoidProfile(
            xyConstraints,
            new State(targetPose.getY(), 0.0),
            new State(startPose.getY(), startSpeeds.vyMetersPerSecond));
    m_thetaTrap =
        new TrapezoidProfile(
            thetaConstraints,
            new State(
                MathUtils.placeInScope(
                    targetPose.getRotation().getRadians(), startRotation.getRadians()),
                0.0),
            new State(startRotation.getRadians(), startSpeeds.omegaRadiansPerSecond));
  }

  /** Returns the trajectory state at time t. */
  public SwerveTrajectoryState getState(final double t) {
    final var xState = m_xTrap.calculate(t);
    final var yState = m_yTrap.calculate(t);
    final var thetaState = m_thetaTrap.calculate(t);
    return new SwerveTrajectoryState(
        new Pose2d(xState.position, yState.position, new Rotation2d(thetaState.position)),
        xState.velocity,
        yState.velocity,
        thetaState.velocity);
  }

  public double totalTime() {
    return Math.max(Math.max(m_xTrap.totalTime(), m_yTrap.totalTime()), m_thetaTrap.totalTime());
  }
}
