package frc.quixlib.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.quixlib.math.MathUtils;
import java.util.Arrays;
import java.util.Collections;

/** Generates limited-scrub swerve module states based on physical feasibility. */
public class QuixSwerveModuleSetpointGenerator {
  private final SwerveDriveKinematics m_kinematics;
  private final double m_maxDriveSpeed; // m/s
  private final double m_maxSpeedDiff; // m/s/s
  private final double m_maxAngleDiff; // rad/s

  public QuixSwerveModuleSetpointGenerator(
      final SwerveDriveKinematics kinematics,
      final double maxDriveSpeed,
      final double maxModuleAcceleration,
      final double maxModuleSteeringRate) {
    m_kinematics = kinematics;
    m_maxDriveSpeed = maxDriveSpeed;
    m_maxSpeedDiff = maxModuleAcceleration * TimedRobot.kDefaultPeriod;
    m_maxAngleDiff = maxModuleSteeringRate * TimedRobot.kDefaultPeriod;
  }

  /**
   * Returns module states that result in ChassisSpeeds as close to |desiredChassisSpeeds| as
   * possible without exceeding |allowedScrub|.
   *
   * @param desiredChassisSpeeds Chassis speeds to try to achieve while staying in scrub and
   *     feasibility limits.
   * @param allowedScrub Maximum scrub permissible in m/s.
   */
  public SwerveSetpoint getFeasibleSetpoint(
      final SwerveSetpoint previousSetpoint,
      final ChassisSpeeds desiredChassisSpeeds,
      final double allowedScrub) {
    final var desiredSetpoint = getDesaturatedSwerveSetpoint(desiredChassisSpeeds);

    // Binary search between linearly interpolated current and desired chassis speeds that doesn't
    // exceed feasibility limits or the desired amount of module scrub.
    double lowerBound = 0.01; // Non-zero to ensure we always make some progress.
    double upperBound = 1.0;

    // Optimistically use upper bound as setpoint.
    SwerveSetpoint bestSetpoint =
        optimizeWithSlewRateLimit(
            getDesaturatedSwerveSetpoint(
                    interpolateChassisSpeeds(
                        previousSetpoint.chassisSpeeds, desiredSetpoint.chassisSpeeds, upperBound))
                .moduleStates,
            previousSetpoint.moduleStates,
            m_maxSpeedDiff,
            m_maxAngleDiff);
    final var upperBoundScrubs = computeModuleScrubs(bestSetpoint.moduleStates);
    final double maxUpperBoundScrub = Collections.max(Arrays.asList(upperBoundScrubs));
    if (maxUpperBoundScrub <= allowedScrub) {
      return bestSetpoint;
    }

    // Begin binary search.
    for (int n = 0; n < 10; n++) {
      // Alpha of 1.0 means to fully use the desired chassis speeds..
      // 0.0 means to fully use the current chassis speeds.
      final double alpha = (lowerBound + upperBound) * 0.5;
      final SwerveSetpoint testSetpoint =
          optimizeWithSlewRateLimit(
              getDesaturatedSwerveSetpoint(
                      interpolateChassisSpeeds(
                          previousSetpoint.chassisSpeeds, desiredSetpoint.chassisSpeeds, alpha))
                  .moduleStates,
              previousSetpoint.moduleStates,
              m_maxSpeedDiff,
              m_maxAngleDiff);

      final var scrubs = computeModuleScrubs(testSetpoint.moduleStates);
      final double maxScrub = Collections.max(Arrays.asList(scrubs));
      if (maxScrub > allowedScrub) {
        upperBound = alpha;
      } else {
        lowerBound = alpha;
        bestSetpoint = testSetpoint;
      }
    }
    return bestSetpoint;
  }

  /** Returns the SwerveSetpoint for the given |chassisSpeeds| after desaturating wheel speeds. */
  private SwerveSetpoint getDesaturatedSwerveSetpoint(final ChassisSpeeds chassisSpeeds) {
    final SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, m_maxDriveSpeed);
    return new SwerveSetpoint(m_kinematics.toChassisSpeeds(states), states);
  }

  private static ChassisSpeeds interpolateChassisSpeeds(
      final ChassisSpeeds a, final ChassisSpeeds b, final double alpha) {
    return new ChassisSpeeds(
        (1 - alpha) * a.vxMetersPerSecond + alpha * b.vxMetersPerSecond,
        (1 - alpha) * a.vyMetersPerSecond + alpha * b.vyMetersPerSecond,
        (1 - alpha) * a.omegaRadiansPerSecond + alpha * b.omegaRadiansPerSecond);
  }

  private SwerveSetpoint optimizeWithSlewRateLimit(
      final SwerveModuleState[] targetStates,
      final SwerveModuleState[] currentStates,
      final double maxSpeedDiff,
      final double maxAngleDiff) {
    final SwerveModuleState[] optimizedStates = new SwerveModuleState[targetStates.length];
    for (int i = 0; i < targetStates.length; i++) {
      // Optimize first to ensure steering rate differences are minimized.
      optimizedStates[i] = optimizeModule(targetStates[i], currentStates[i]);

      // Apply acceleration limit.
      final double speedDiff =
          optimizedStates[i].speedMetersPerSecond - currentStates[i].speedMetersPerSecond;
      if (speedDiff > maxSpeedDiff) {
        optimizedStates[i].speedMetersPerSecond =
            currentStates[i].speedMetersPerSecond + maxSpeedDiff;
      }
      if (speedDiff < -maxSpeedDiff) {
        optimizedStates[i].speedMetersPerSecond =
            currentStates[i].speedMetersPerSecond - maxSpeedDiff;
      }

      // Apply steering rate limit.
      final double angleDiff =
          optimizedStates[i].angle.getRadians() - currentStates[i].angle.getRadians();
      if (angleDiff > maxAngleDiff) {
        optimizedStates[i].angle =
            new Rotation2d(currentStates[i].angle.getRadians() + maxAngleDiff);
      }
      if (angleDiff < -maxAngleDiff) {
        optimizedStates[i].angle =
            new Rotation2d(currentStates[i].angle.getRadians() - maxAngleDiff);
      }

      // Re-optimize final result.
      optimizedStates[i] = optimizeModule(optimizedStates[i], currentStates[i]);
    }
    return new SwerveSetpoint(m_kinematics.toChassisSpeeds(optimizedStates), optimizedStates);
  }

  /** Returns the amount of scrub for each module relative to overall chassis movement. */
  public Double[] computeModuleScrubs(final SwerveModuleState[] states) {
    final var idealStates = m_kinematics.toSwerveModuleStates(m_kinematics.toChassisSpeeds(states));
    final Double[] scrubs = new Double[states.length];
    for (int i = 0; i < states.length; i++) {
      var cart1 =
          MathUtils.pol2cart(
              idealStates[i].speedMetersPerSecond, idealStates[i].angle.getRadians());
      var idealVelocityVec =
          Matrix.mat(Nat.N2(), Nat.N1()).fill(cart1.getFirst(), cart1.getSecond());

      var cart2 = MathUtils.pol2cart(states[i].speedMetersPerSecond, states[i].angle.getRadians());
      var actualVelocityVec =
          Matrix.mat(Nat.N2(), Nat.N1()).fill(cart2.getFirst(), cart2.getSecond());

      scrubs[i] = idealVelocityVec.minus(actualVelocityVec).normF();
    }
    return scrubs;
  }

  /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. Places the result within [-pi, pi) of the current
   * state. Additionally, keeps the same steering angle if the speed zero.
   *
   * @param desiredState The desired state.
   * @param currentState The current state.
   */
  private static SwerveModuleState optimizeModule(
      final SwerveModuleState desiredState, final SwerveModuleState currentState) {
    double targetAngle =
        MathUtils.placeInScope(desiredState.angle.getRadians(), currentState.angle.getRadians());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentState.angle.getRadians();
    if (Math.abs(delta) > 0.5 * Math.PI) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 0.5 * Math.PI ? (targetAngle -= Math.PI) : (targetAngle += Math.PI);
    }
    // Don't steer unnecessarily at singularity.
    if (targetSpeed == 0.0) {
      targetAngle = currentState.angle.getRadians();
    }
    return new SwerveModuleState(targetSpeed, new Rotation2d(targetAngle));
  }
}
