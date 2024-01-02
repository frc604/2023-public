package frc.quixlib.math;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

public class MathUtils {
  private static final double kEps = 1E-9;

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, kEps);
  }

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static Pair<Double, Double> cart2pol(final double x, final double y) {
    final double r = Math.sqrt(x * x + y * y);
    final double theta = Math.atan2(y, x);
    return new Pair<>(r, theta);
  }

  public static Pair<Double, Double> pol2cart(final double r, final double theta) {
    final double x = r * Math.cos(theta);
    final double y = r * Math.sin(theta);
    return new Pair<>(x, y);
  }

  /** Constrains an angle to be within [-pi, pi). */
  public static double constrainAngleNegPiToPi(final double angle) {
    double x = (angle + Math.PI) % (2.0 * Math.PI);
    if (x < 0.0) {
      x += 2.0 * Math.PI;
    }
    return x - Math.PI;
  }

  /** Returns |angle| placed within within [-pi, pi) of |referenceAngle|. */
  public static double placeInScope(final double angle, final double referenceAngle) {
    return referenceAngle + constrainAngleNegPiToPi(angle - referenceAngle);
  }

  /**
   * Obtain a new Pose2d from a (constant curvature) velocity. See:
   * https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp. Borrowed from 254:
   * https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/lib/geometry/Pose2d.java
   */
  public static Pose2d exp(final Twist2d delta) {
    final double sin_theta = Math.sin(delta.dtheta);
    final double cos_theta = Math.cos(delta.dtheta);
    final double s =
        Math.abs(delta.dtheta) < kEps
            ? 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta
            : sin_theta / delta.dtheta;
    final double c =
        Math.abs(delta.dtheta) < kEps ? 0.5 * delta.dtheta : (1.0 - cos_theta) / delta.dtheta;
    return new Pose2d(
        new Translation2d(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
        new Rotation2d(cos_theta, sin_theta));
  }

  /**
   * Logical inverse of the above. Borrowed from 254:
   * https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/lib/geometry/Pose2d.java
   */
  public static Twist2d log(final Pose2d transform) {
    final double dtheta = transform.getRotation().getRadians();
    final double half_dtheta = 0.5 * dtheta;
    final double cos_minus_one = Math.cos(transform.getRotation().getRadians()) - 1.0;
    final double halftheta_by_tan_of_halfdtheta =
        Math.abs(cos_minus_one) < kEps
            ? 1.0 - 1.0 / 12.0 * dtheta * dtheta
            : -(half_dtheta * Math.sin(transform.getRotation().getRadians())) / cos_minus_one;
    final Translation2d translation_part =
        transform
            .getTranslation()
            .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
    return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
  }
}
