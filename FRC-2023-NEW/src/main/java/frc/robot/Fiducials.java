package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.quixlib.vision.Fiducial;
import java.util.Arrays;

public class Fiducials {
  private static final boolean isNASA = false; // Whether this is 1868's field at NASA.

  private static final double aprilTagSize = Units.inchesToMeters(6.5); // m
  public static final Fiducial redGridRightAprilTag =
      new Fiducial(
          Fiducial.Type.APRILTAG,
          1,
          new Pose3d(
              Units.inchesToMeters(610.77),
              Units.inchesToMeters(42.19),
              Units.inchesToMeters(18.22),
              new Rotation3d(0.0, 0.0, Math.PI)),
          aprilTagSize);
  public static final Fiducial redGridMiddleAprilTag =
      new Fiducial(
          Fiducial.Type.APRILTAG,
          2,
          new Pose3d(
              Units.inchesToMeters(610.77),
              Units.inchesToMeters(108.19),
              Units.inchesToMeters(18.22),
              new Rotation3d(0.0, 0.0, Math.PI)),
          aprilTagSize);
  public static final Fiducial redGridLeftAprilTag =
      new Fiducial(
          Fiducial.Type.APRILTAG,
          3,
          new Pose3d(
              Units.inchesToMeters(610.77),
              Units.inchesToMeters(174.19),
              Units.inchesToMeters(18.22),
              new Rotation3d(0.0, 0.0, Math.PI)),
          aprilTagSize);
  public static final Fiducial blueFeederStation =
      new Fiducial(
          Fiducial.Type.APRILTAG,
          4,
          new Pose3d(
              Units.inchesToMeters(636.96),
              Units.inchesToMeters(265.74),
              Units.inchesToMeters(27.38),
              new Rotation3d(0.0, 0.0, Math.PI)),
          aprilTagSize);
  public static final Fiducial redFeederStation =
      new Fiducial(
          Fiducial.Type.APRILTAG,
          5,
          new Pose3d(
              Units.inchesToMeters(14.25),
              Units.inchesToMeters(isNASA ? 235.24 : 265.74),
              Units.inchesToMeters(27.38),
              new Rotation3d(0.0, 0.0, 0.0)),
          aprilTagSize);
  public static final Fiducial blueGridRightAprilTag =
      new Fiducial(
          Fiducial.Type.APRILTAG,
          6,
          new Pose3d(
              Units.inchesToMeters(isNASA ? 72.50 : 40.45),
              Units.inchesToMeters(isNASA ? 143.69 : 174.19),
              Units.inchesToMeters(18.22),
              new Rotation3d(0.0, 0.0, 0.0)),
          aprilTagSize);
  public static final Fiducial blueGridMiddleAprilTag =
      new Fiducial(
          Fiducial.Type.APRILTAG,
          7,
          new Pose3d(
              Units.inchesToMeters(isNASA ? 72.50 : 40.45),
              Units.inchesToMeters(isNASA ? 77.69 : 108.19),
              Units.inchesToMeters(18.22),
              new Rotation3d(0.0, 0.0, 0.0)),
          aprilTagSize);
  public static final Fiducial blueGridLeftAprilTag =
      new Fiducial(
          Fiducial.Type.APRILTAG,
          8,
          new Pose3d(
              Units.inchesToMeters(isNASA ? 72.50 : 40.45),
              Units.inchesToMeters(isNASA ? 11.69 : 42.19),
              Units.inchesToMeters(18.22),
              new Rotation3d(0.0, 0.0, 0.0)),
          aprilTagSize);

  private static final Fiducial[] getRetroreflectiveFiducialsInGridFromAprilTagPose(Pose3d pose) {
    return new Fiducial[] {
      // Left front
      new Fiducial(
          Fiducial.Type.RETROREFLECTIVE,
          -1,
          pose.transformBy(
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(-8.91),
                      Units.inchesToMeters(-22.0),
                      Units.inchesToMeters(5.69)),
                  new Rotation3d())),
          0.0),
      // Right front
      new Fiducial(
          Fiducial.Type.RETROREFLECTIVE,
          -1,
          pose.transformBy(
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(-8.91),
                      Units.inchesToMeters(22.0),
                      Units.inchesToMeters(5.69)),
                  new Rotation3d())),
          0.0),
      // Left back
      new Fiducial(
          Fiducial.Type.RETROREFLECTIVE,
          -1,
          pose.transformBy(
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(-25.90),
                      Units.inchesToMeters(-22.0),
                      Units.inchesToMeters(22.66)),
                  new Rotation3d())),
          0.0),
      // Right back
      new Fiducial(
          Fiducial.Type.RETROREFLECTIVE,
          -1,
          pose.transformBy(
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(-25.90),
                      Units.inchesToMeters(22.0),
                      Units.inchesToMeters(22.66)),
                  new Rotation3d())),
          0.0)
    };
  }

  private static final Fiducial[] redGridRightRetroreflective =
      getRetroreflectiveFiducialsInGridFromAprilTagPose(redGridRightAprilTag.getPose());
  private static final Fiducial[] redGridMiddleRetroreflective =
      getRetroreflectiveFiducialsInGridFromAprilTagPose(redGridMiddleAprilTag.getPose());
  private static final Fiducial[] redGridLeftRetroreflective =
      getRetroreflectiveFiducialsInGridFromAprilTagPose(redGridLeftAprilTag.getPose());
  private static final Fiducial[] blueGridRightRetroreflective =
      getRetroreflectiveFiducialsInGridFromAprilTagPose(blueGridRightAprilTag.getPose());
  private static final Fiducial[] blueGridMiddleRetroreflective =
      getRetroreflectiveFiducialsInGridFromAprilTagPose(blueGridMiddleAprilTag.getPose());
  private static final Fiducial[] blueGridLeftRetroreflective =
      getRetroreflectiveFiducialsInGridFromAprilTagPose(blueGridLeftAprilTag.getPose());

  public static final Fiducial[] fiducials =
      new Fiducial[] {
        redGridRightAprilTag,
        redGridMiddleAprilTag,
        redGridLeftAprilTag,
        blueFeederStation,
        redFeederStation,
        blueGridRightAprilTag,
        blueGridMiddleAprilTag,
        blueGridLeftAprilTag,
        redGridRightRetroreflective[0],
        redGridRightRetroreflective[1],
        redGridRightRetroreflective[2],
        redGridRightRetroreflective[3],
        redGridMiddleRetroreflective[0],
        redGridMiddleRetroreflective[1],
        redGridMiddleRetroreflective[2],
        redGridMiddleRetroreflective[3],
        redGridLeftRetroreflective[0],
        redGridLeftRetroreflective[1],
        redGridLeftRetroreflective[2],
        redGridLeftRetroreflective[3],
        blueGridRightRetroreflective[0],
        blueGridRightRetroreflective[1],
        blueGridRightRetroreflective[2],
        blueGridRightRetroreflective[3],
        blueGridMiddleRetroreflective[0],
        blueGridMiddleRetroreflective[1],
        blueGridMiddleRetroreflective[2],
        blueGridMiddleRetroreflective[3],
        blueGridLeftRetroreflective[0],
        blueGridLeftRetroreflective[1],
        blueGridLeftRetroreflective[2],
        blueGridLeftRetroreflective[3],
      };

  public static final Fiducial[] aprilTagFiducials =
      Arrays.stream(fiducials)
          .filter(f -> f.getType() == Fiducial.Type.APRILTAG)
          .toArray(Fiducial[]::new);

  public static final Fiducial[] retroreflectiveFiducials =
      Arrays.stream(fiducials)
          .filter(f -> f.getType() == Fiducial.Type.RETROREFLECTIVE)
          .toArray(Fiducial[]::new);
}
