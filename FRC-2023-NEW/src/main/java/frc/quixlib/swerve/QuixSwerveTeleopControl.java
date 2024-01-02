package frc.quixlib.swerve;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class QuixSwerveTeleopControl {
  private final SlewRateLimiter m_xVelRateLimiter;
  private final SlewRateLimiter m_yVelRateLimiter;
  private final SlewRateLimiter m_angularRateLimiter;
  private final double m_stickDeadband;
  private final double m_maxDriveSpeed;
  private final double m_maxAngularVelocity;
  private final boolean m_squared;

  /**
   * Helper to compute drive velocities from joystick values.
   *
   * @param linearSlewRate m/s/s
   * @param angularSlewRate rad/s/s
   * @param stickDeadband joystick deadband as a percentage (0 to 1.0)
   * @param maxDriveSpeed m/s/s
   * @param maxAngularVelocity rad/s/s
   * @param squared whether or not to square the deadbanded joystick
   */
  public QuixSwerveTeleopControl(
      final double linearSlewRate,
      final double angularSlewRate,
      final double stickDeadband,
      final double maxDriveSpeed,
      final double maxAngularVelocity,
      final boolean squared) {
    m_xVelRateLimiter = new SlewRateLimiter(linearSlewRate);
    m_yVelRateLimiter = new SlewRateLimiter(linearSlewRate);
    m_angularRateLimiter = new SlewRateLimiter(angularSlewRate);
    m_stickDeadband = stickDeadband;
    m_maxDriveSpeed = maxDriveSpeed;
    m_maxAngularVelocity = maxAngularVelocity;
    m_squared = squared;
  }

  public class DriveVelocities {
    public final double xVel; // m/s
    public final double yVel; // m/s
    public final double thetaVel; // rad/s

    public DriveVelocities(final double xVel, final double yVel, final double thetaVel) {
      this.xVel = xVel;
      this.yVel = yVel;
      this.thetaVel = thetaVel;
    }
  }

  /**
   * Compute the translation and rotation velocities given joystick axis inputs.
   *
   * @param xAxis Joystick axis that corresponds to the forward axis of the robot.
   * @param yAxis Joystick axis that corresponds to the left axis of the robot.
   * @param rAxis Joystick axis that corresponds to the CCW rotation of the robot.
   */
  public DriveVelocities getDriveVelocitiesFromJoysticks(double xAxis, double yAxis, double rAxis) {

    /* Deadbands */
    final var deadbandedXY = applyDeadbandOnXYAndScale(xAxis, yAxis, m_stickDeadband, m_squared);
    xAxis = deadbandedXY.getFirst();
    yAxis = deadbandedXY.getSecond();
    rAxis = applyDeadband(rAxis, m_stickDeadband);

    if (m_squared) {
      // xy scaling handled in applyDeadbandOnXYAndScale
      rAxis = signedSquare(rAxis);
    }

    // Convert joystick axes to velocities.
    final double xVel = xAxis * m_maxDriveSpeed;
    final double yVel = yAxis * m_maxDriveSpeed;
    final double angularVel = rAxis * m_maxAngularVelocity;

    // Apply slew rates on velocities.
    return new DriveVelocities(
        m_xVelRateLimiter.calculate(xVel),
        m_yVelRateLimiter.calculate(yVel),
        m_angularRateLimiter.calculate(angularVel));
  }

  /**
   * Apply deadband in joystick polar coordinates instead of on the raw joystick values. Scale the
   * polar value.
   */
  private static Pair<Double, Double> applyDeadbandOnXYAndScale(
      final double x, final double y, final double deadband, final boolean squared) {
    final double polarR = Math.hypot(x, y);
    if (polarR == 0.0) {
      return new Pair<>(0.0, 0.0);
    }
    final double deadbandedR = applyDeadband(polarR, deadband);
    final double scaledCoefficient = (squared ? deadbandedR * deadbandedR : deadbandedR) / polarR;
    return new Pair<>(x * scaledCoefficient, y * scaledCoefficient);
  }

  /**
   * Apply deadband. Ramps linearly from 0 to 1 from (deadband, 1) and 0 to -1 from (-deadband, -1).
   */
  public static double applyDeadband(final double value, final double deadband) {
    if (Math.abs(value) < deadband) {
      return 0.0;
    } else {
      final double signedOne = Math.signum(value);
      return ((value - signedOne) / (1.0 - deadband)) + signedOne;
    }
  }

  private static double signedSquare(final double value) {
    return Math.signum(value) * value * value;
  }
}
