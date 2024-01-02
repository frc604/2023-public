package frc.quixlib.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.util.Units;
import frc.quixlib.math.CameraMathUtils;
import frc.quixlib.math.MathUtils;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public interface QuixVisionCamera {
  /** Returns the latest measurement. */
  public PipelineVisionPacket getLatestMeasurement();

  /** Select the active pipeline index. */
  public void setPipelineIndex(int index);

  /** Get the active pipeline config. */
  public PipelineConfig getPipelineConfig();

  /** Returns the robot-to-camera transform. */
  public Matrix<N4, N4> getTransform();

  /** Returns the type of fiducials this camera is tracking. */
  public Fiducial.Type getFiducialType();

  /**
   * Simulates vision by updating NetworkTables based on the 3D transform of the camera in field
   * coordinates.
   */
  public void updateSim(Matrix<N4, N4> fieldCameraT, Fiducial[] targets);

  // Simulation parameters
  public static final double aprilTagDetectionSimDistance = Units.feetToMeters(15); // m
  public static final double retroReflectiveDetectionSimDistance = Units.feetToMeters(25); // m

  /** Simulates tracked retroreflective and AprilTag targets. */
  public static ArrayList<PhotonTrackedTarget> simulateTrackedTargets(
      Matrix<N4, N4> fieldCameraT,
      Fiducial[] fiducials,
      double imageWidth,
      double imageHeight,
      double fovHeight,
      double fovWidth) {
    ArrayList<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
    for (final var fiducial : fiducials) {
      // Only "detect" fiducial within range.
      double dx = fieldCameraT.get(0, 3) - fiducial.getX();
      double dy = fieldCameraT.get(1, 3) - fiducial.getY();
      double dz = fieldCameraT.get(2, 3) - fiducial.getZ();
      double distToFiducial = Math.sqrt(dx * dx + dy * dy + dz * dz);
      if (distToFiducial
          > (fiducial.getType() == Fiducial.Type.APRILTAG
              ? aprilTagDetectionSimDistance
              : retroReflectiveDetectionSimDistance)) {
        continue;
      }

      // Only "detect" AprilTags if the camera is +/- 45 degrees from the front.
      if (fiducial.getType() == Fiducial.Type.APRILTAG) {
        var camPose = MathUtils.zRotTFMatrixToPose(fieldCameraT);
        Rotation2d cameraRotation =
            new Rotation2d(camPose.getX() - fiducial.getX(), camPose.getY() - fiducial.getY());
        double diff =
            MathUtils.constrainAngleNegPiToPi(cameraRotation.getRadians() - fiducial.getZRot());
        if (Math.abs(diff) > Math.PI * 0.25) {
          continue;
        }
      }

      var pinholeBE = getBEPinholeFromPose(fiducial.getPose(), fieldCameraT);
      double bearing = pinholeBE.getFirst();
      double elevation = pinholeBE.getSecond();
      boolean targetVisible =
          Math.abs(bearing) < fovWidth * 0.5 && Math.abs(elevation) < fovHeight * 0.5;
      if (!targetVisible) {
        continue;
      }

      // Simulate AprilTag corners.
      List<TargetCorner> detectedCorners;
      if (fiducial.getType() == Fiducial.Type.APRILTAG) {
        final var botLeftBE =
            getBEPinholeFromPose(
                fiducial
                    .getPose()
                    .transformBy(
                        new Transform3d(
                            new Translation3d(
                                0.0, -0.5 * fiducial.getSize(), -0.5 * fiducial.getSize()),
                            new Rotation3d())),
                fieldCameraT);
        final var botRightBE =
            getBEPinholeFromPose(
                fiducial
                    .getPose()
                    .transformBy(
                        new Transform3d(
                            new Translation3d(
                                0.0, 0.5 * fiducial.getSize(), -0.5 * fiducial.getSize()),
                            new Rotation3d())),
                fieldCameraT);
        final var topRightBE =
            getBEPinholeFromPose(
                fiducial
                    .getPose()
                    .transformBy(
                        new Transform3d(
                            new Translation3d(
                                0.0, 0.5 * fiducial.getSize(), 0.5 * fiducial.getSize()),
                            new Rotation3d())),
                fieldCameraT);
        final var topLeftBE =
            getBEPinholeFromPose(
                fiducial
                    .getPose()
                    .transformBy(
                        new Transform3d(
                            new Translation3d(
                                0.0, -0.5 * fiducial.getSize(), 0.5 * fiducial.getSize()),
                            new Rotation3d())),
                fieldCameraT);
        detectedCorners =
            List.of(
                BEPinhole2TargetCorner(botLeftBE, imageWidth, imageHeight, fovWidth, fovHeight),
                BEPinhole2TargetCorner(botRightBE, imageWidth, imageHeight, fovWidth, fovHeight),
                BEPinhole2TargetCorner(topRightBE, imageWidth, imageHeight, fovWidth, fovHeight),
                BEPinhole2TargetCorner(topLeftBE, imageWidth, imageHeight, fovWidth, fovHeight));
      } else {
        detectedCorners =
            List.of(
                new TargetCorner(0, 0), new TargetCorner(0, 0),
                new TargetCorner(0, 0), new TargetCorner(0, 0));
      }

      var target =
          new PhotonTrackedTarget(
              Math.toDegrees(bearing),
              Math.toDegrees(elevation),
              0,
              0,
              fiducial.id(),
              new Transform3d(),
              new Transform3d(),
              0.0,
              List.of(
                  new TargetCorner(0, 0), new TargetCorner(0, 0),
                  new TargetCorner(0, 0), new TargetCorner(0, 0)),
              detectedCorners);
      targets.add(target);
    }
    return targets;
  }

  private static Pair<Double, Double> getBEPinholeFromPose(
      Pose3d fieldPose, Matrix<N4, N4> fieldCameraT) {
    return CameraMathUtils.cart2BEPinhole(
        MathUtils.getPointInFrame(
            Matrix.mat(Nat.N3(), Nat.N1())
                .fill(fieldPose.getX(), fieldPose.getY(), fieldPose.getZ()),
            fieldCameraT));
  }

  private static TargetCorner BEPinhole2TargetCorner(
      final Pair<Double, Double> be,
      final double imageWidth,
      final double imageHeight,
      final double fovWidth,
      final double fovHeight) {
    final var xy =
        CameraMathUtils.yawPitchToXY(
            be.getFirst(), be.getSecond(), imageWidth, imageHeight, fovWidth, fovHeight);
    return new TargetCorner(xy.getFirst(), xy.getSecond());
  }
}
