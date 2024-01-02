package frc.quixlib.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import frc.quixlib.math.CameraMathUtils;
import java.util.ArrayList;
import org.ejml.simple.SimpleMatrix;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.photonvision.targeting.PhotonTrackedTarget;

/** A data class for a target measurement. */
public class Target {
  private final int m_id;
  private final double m_yaw;
  private final double m_pitch;
  private final double m_area;
  private final double m_skew;
  private final ArrayList<Pair<Double, Double>> m_corners;

  /**
   * Instantiates a new target.
   *
   * @param id ID of the target
   * @param yaw the yaw of the target
   * @param pitch the pitch of the target
   * @param area the area of the target
   * @param skew the skew of the target
   */
  public Target(
      final int id, final double yaw, final double pitch, final double area, final double skew) {
    this(id, yaw, pitch, area, skew, new ArrayList<>());
  }

  /**
   * Instantiates a new target.
   *
   * @param id ID of the target
   * @param yaw the yaw of the target
   * @param pitch the pitch of the target
   * @param area the area of the target
   * @param skew the skew of the target
   * @param corners the corners of the target as a list of pairs of yaw and pitch
   */
  public Target(
      final int id,
      final double yaw,
      final double pitch,
      final double area,
      final double skew,
      final ArrayList<Pair<Double, Double>> corners) {
    m_id = id;
    m_yaw = yaw;
    m_pitch = pitch;
    m_area = area;
    m_skew = skew;
    m_corners = corners;
  }

  public Target() {
    this(-1, 0.0, 0.0, 0.0, 0.0, new ArrayList<>());
  }

  public static Target fromPhotonTrackedTargetWithoutCalibration(
      final PhotonTrackedTarget target,
      final double imageWidth,
      final double imageHeight,
      final double fovWidth,
      final double fovHeight) {
    final ArrayList<Pair<Double, Double>> corners = new ArrayList<>();
    for (final var corner : target.getDetectedCorners()) {
      final var yawPitch =
          CameraMathUtils.XYToYawPitchWithHeightAndFOV(
              corner.x, corner.y, imageWidth, imageHeight, fovWidth, fovHeight);
      corners.add(
          new Pair<>(Math.toDegrees(yawPitch.getFirst()), Math.toDegrees(yawPitch.getSecond())));
    }
    return new Target(
        target.getFiducialId(),
        target.getYaw(),
        target.getPitch(),
        target.getArea(),
        target.getSkew(),
        corners);
  }

  public static Target fromPhotonTrackedTargetWithCalibration(
      final PhotonTrackedTarget target,
      final Matrix<N3, N3> cameraMatrix,
      final Matrix<N5, N1> distCoeffs) {
    final Point[] corners = new Point[target.getDetectedCorners().size()];
    for (int i = 0; i < target.getDetectedCorners().size(); i++) {
      corners[i] =
          new Point(target.getDetectedCorners().get(i).x, target.getDetectedCorners().get(i).y);
    }

    final MatOfPoint2f tempIn = new MatOfPoint2f(corners);
    final MatOfPoint2f tempOut = new MatOfPoint2f();
    Calib3d.undistortImagePoints(
        tempIn,
        tempOut,
        matrixToMat(cameraMatrix.getStorage()),
        matrixToMat(distCoeffs.getStorage()));

    final ArrayList<Pair<Double, Double>> undistoredCorners = new ArrayList<>();
    for (final var corner : tempOut.toArray()) {
      final var yawPitch =
          CameraMathUtils.XYToYawPitch(
              corner.x,
              corner.y,
              cameraMatrix.get(0, 2),
              cameraMatrix.get(1, 2),
              cameraMatrix.get(0, 0),
              cameraMatrix.get(1, 1));
      undistoredCorners.add(
          new Pair<>(Math.toDegrees(yawPitch.getFirst()), Math.toDegrees(yawPitch.getSecond())));
    }
    return new Target(
        target.getFiducialId(),
        target.getYaw(),
        target.getPitch(),
        target.getArea(),
        target.getSkew(),
        undistoredCorners);
  }

  private static MatOfDouble matrixToMat(SimpleMatrix matrix) {
    var mat = new Mat(matrix.numRows(), matrix.numCols(), CvType.CV_64F);
    mat.put(0, 0, matrix.getDDRM().getData());
    var wrappedMat = new MatOfDouble();
    mat.convertTo(wrappedMat, CvType.CV_64F);
    mat.release();
    return wrappedMat;
  }

  /**
   * Returns the bearing and elevation of the target in spherical coodinates centered on the camera
   * frame.
   */
  public Pair<Double, Double> getSphericalBE() {
    final Matrix<N3, N1> camFramePoint =
        CameraMathUtils.pinholeBE2Cart(Math.toRadians(m_yaw), Math.toRadians(m_pitch));
    return CameraMathUtils.cart2BESph(camFramePoint);
  }

  /**
   * Returns the bearing and elevation of the specified target corner in spherical coodinates
   * centered on the camera frame.
   */
  public Pair<Double, Double> getCornerSphericalBE(final int cornerID) {
    final var corner = m_corners.get(cornerID);
    final Matrix<N3, N1> camFramePoint =
        CameraMathUtils.pinholeBE2Cart(
            Math.toRadians(corner.getFirst()), Math.toRadians(corner.getSecond()));
    return CameraMathUtils.cart2BESph(camFramePoint);
  }

  /**
   * Gets ID.
   *
   * @return the ID
   */
  public int getID() {
    return m_id;
  }

  /**
   * Gets yaw in degrees.
   *
   * @return the yaw
   */
  public double getYaw() {
    return m_yaw;
  }

  /**
   * Gets pitch in degrees.
   *
   * @return the pitch
   */
  public double getPitch() {
    return m_pitch;
  }

  /**
   * Gets area.
   *
   * @return the area
   */
  public double getArea() {
    return m_area;
  }

  /**
   * Gets skew.
   *
   * @return the skew
   */
  public double getSkew() {
    return m_skew;
  }

  /**
   * Gets yaw and pitch of corners in degrees. For fiducials, the order is known and is always
   * counter-clock wise around the tag, like so:
   *
   * <p>spotless:off
   * -> +X  3 ----- 2
   * |      |       |
   * V      |       |
   * +Y     0 ----- 1
   * spotless:on
   *
   * @return the corners
   */
  public ArrayList<Pair<Double, Double>> getCorners() {
    return m_corners;
  }
}
