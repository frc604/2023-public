package frc.quixlib.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class CameraMathUtils {
  /**
   * Converts cartesian coordinates in camera frame to bearing/elevation that would be returned by a
   * pinhole camera model. X-forward, Y-left, Z-up.
   */
  public static Pair<Double, Double> cart2BEPinhole(final Matrix<N3, N1> cartesian) {
    final double x = cartesian.get(0, 0);
    final double y = cartesian.get(1, 0);
    final double z = cartesian.get(2, 0);

    final double bearing = Math.atan2(y, x);
    final double elevation = Math.atan2(z, x);

    // Negate bearing because camera returns +bearing to the right.
    return new Pair<>(-bearing, elevation);
  }

  /**
   * Returns the camera-frame vector given the bearing and elevation in a pinhole model. X-forward,
   * Y-left, Z-up.
   */
  public static Matrix<N3, N1> pinholeBE2Cart(final double bearing, final double elevation) {
    // Negate bearing because camera +bearing is to the right.
    final double x = 1.0;
    final double y = x * Math.tan(-bearing);
    final double z = x * Math.tan(elevation);
    return Matrix.mat(Nat.N3(), Nat.N1()).fill(x, y, z);
  }

  /**
   * Converts cartesian coordinates in camera frame to bearing/elevation in a spherical coordinate
   * system. X-forward, Y-left, Z-up.
   */
  public static Pair<Double, Double> cart2BESph(final Matrix<N3, N1> cartesian) {
    final double x = cartesian.get(0, 0);
    final double y = cartesian.get(1, 0);
    final double z = cartesian.get(2, 0);

    final double bearing = Math.atan2(y, x);
    final double elevation = Math.atan2(z, Math.sqrt(x * x + y * y));

    return new Pair<>(bearing, elevation);
  }

  /**
   * Converts bearing/elevation in a spherical coordinate system to cartesian coordinates.
   * X-forward, Y-left, Z-up.
   */
  public static Matrix<N3, N1> beSph2Cart(final double bearing, final double elevation) {
    final double x = Math.cos(bearing) * Math.cos(elevation);
    final double y = Math.sin(bearing) * Math.cos(elevation);
    final double z = Math.sin(elevation);

    return Matrix.mat(Nat.N3(), Nat.N1()).fill(x, y, z);
  }

  /**
   * Converts X/Y pixels to yaw/pitch radians. +X right, +Y down. Estimtes camera center and focal
   * length from image size and FOV.
   */
  public static Pair<Double, Double> XYToYawPitchWithHeightAndFOV(
      final double x,
      final double y,
      final double imageWidth,
      final double imageHeight,
      final double fovWidth,
      final double fovHeight) {
    final double centerX = (imageWidth / 2.0) - 0.5;
    final double centerY = (imageHeight / 2.0) - 0.5;
    final double horizontalFocalLength = imageWidth / (2.0 * Math.tan(fovWidth / 2.0));
    final double verticalFocalLength = imageHeight / (2.0 * Math.tan(fovHeight / 2.0));
    return XYToYawPitch(x, y, centerX, centerY, horizontalFocalLength, verticalFocalLength);
  }

  /** Converts X/Y pixels to yaw/pitch radians. +X right, +Y down. */
  public static Pair<Double, Double> XYToYawPitch(
      final double x,
      final double y,
      final double centerX,
      final double centerY,
      final double horizontalFocalLength,
      final double verticalFocalLength) {
    return new Pair<>(
        Math.atan((x - centerX) / horizontalFocalLength),
        -Math.atan((y - centerY) / verticalFocalLength));
  }

  /** Converts yaw/pitch radians to X/Y pixels. +X right, +Y down. */
  public static Pair<Double, Double> yawPitchToXY(
      final double yaw,
      final double pitch,
      final double imageWidth,
      final double imageHeight,
      final double fovWidth,
      final double fovHeight) {
    final double centerX = (imageWidth / 2.0) - 0.5;
    final double centerY = (imageHeight / 2.0) - 0.5;
    final double horizontalFocalLength = imageWidth / (2.0 * Math.tan(fovWidth / 2.0));
    final double verticalFocalLength = imageHeight / (2.0 * Math.tan(fovHeight / 2.0));
    return new Pair<>(
        (Math.tan(yaw) * horizontalFocalLength) + centerX,
        (Math.tan(-pitch) * verticalFocalLength) + centerY);
  }
}
