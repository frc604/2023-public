package frc.quixlib.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;

public class MathUtils {
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

  /** Rotates a vector by a given angle. */
  public static Matrix<N2, N1> rotateVector(final Matrix<N2, N1> vector, final double theta) {
    final double x1 = vector.get(0, 0);
    final double y1 = vector.get(1, 0);
    final double c = Math.cos(theta);
    final double s = Math.sin(theta);
    final double x2 = c * x1 - s * y1;
    final double y2 = s * x1 + c * y1;
    return Matrix.mat(Nat.N2(), Nat.N1()).fill(x2, y2);
  }

  public static double extractTFMatrixZ(final Matrix<N4, N4> matrix) {
    return matrix.get(2, 3);
  }

  public static double extractTFMatrixYRot(final Matrix<N4, N4> matrix) {
    return Math.atan2(
        -matrix.get(2, 0),
        Math.sqrt(matrix.get(2, 1) * matrix.get(2, 1) + matrix.get(2, 2) * matrix.get(2, 2)));
  }

  public static Matrix<N4, N4> pose2DToZRotTFMatrix(Pose2d pose) {
    return makeZRotTFMatrix(pose.getX(), pose.getY(), 0.0, pose.getRotation().getRadians());
  }

  public static Pose2d zRotTFMatrixToPose(final Matrix<N4, N4> zRotTFMatrix) {
    return new Pose2d(
        zRotTFMatrix.get(0, 3),
        zRotTFMatrix.get(1, 3),
        new Rotation2d(Math.atan2(zRotTFMatrix.get(1, 0), zRotTFMatrix.get(0, 0))));
  }

  public static Matrix<N3, N1> getPointInFrame(Matrix<N3, N1> point, final Matrix<N4, N4> frame) {
    final Matrix<N4, N1> pointMod =
        Matrix.mat(Nat.N4(), Nat.N1()).fill(point.get(0, 0), point.get(1, 0), point.get(2, 0), 1);
    return frame.inv().times(pointMod).block(Nat.N3(), Nat.N1(), 0, 0);
  }

  public static Matrix<N4, N4> makeYRotTFMatrix(
      final double x, final double y, final double z, final double theta) {
    return Matrix.mat(Nat.N4(), Nat.N4())
        .fill(
            Math.cos(theta),
            0,
            Math.sin(theta),
            x,
            0,
            1,
            0,
            y,
            -Math.sin(theta),
            0,
            Math.cos(theta),
            z,
            0,
            0,
            0,
            1);
  }

  public static Matrix<N4, N4> makeZRotTFMatrix(
      final double x, final double y, final double z, final double theta) {
    return Matrix.mat(Nat.N4(), Nat.N4())
        .fill(
            Math.cos(theta),
            -Math.sin(theta),
            0,
            x,
            Math.sin(theta),
            Math.cos(theta),
            0,
            y,
            0,
            0,
            1,
            z,
            0,
            0,
            0,
            1);
  }

  private static final double kEps = 1E-9;

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
