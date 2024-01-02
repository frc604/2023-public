// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Fiducials;

public class ScoringSelector extends SubsystemBase {
  private static final double kTopXOffset = Units.inchesToMeters(-25.93);
  private static final double kMidXOffset = Units.inchesToMeters(-8.91);
  private static final double kBottomXOffset = Units.inchesToMeters(4.45);

  private static final double kSideYOffset = Units.inchesToMeters(22.0);

  private static final double kTopZLeftRightOffset = Units.inchesToMeters(27.85);
  private static final double kTopZMidOffset = Units.inchesToMeters(17.35);
  private static final double kMidZLeftRightOffset = Units.inchesToMeters(15.88);
  private static final double kMidZMidOffset = Units.inchesToMeters(5.38);

  private static final double kBottomZOffset = Units.inchesToMeters(-17.91);

  private static final double kChassisXOffset = Units.inchesToMeters(42.0);

  // Left-to-right from the driver station perspective is right-to-left from the field persepctive.
  private static final Pose3d[] redAprilTagPoses =
      new Pose3d[] {
        Fiducials.redGridRightAprilTag.getPose(), // Grid ID 0
        Fiducials.redGridMiddleAprilTag.getPose(), // Grid ID 1
        Fiducials.redGridLeftAprilTag.getPose(), // Grid ID 2
      };
  private static final Pose3d[] blueAprilTagPoses =
      new Pose3d[] {
        Fiducials.blueGridRightAprilTag.getPose(), // Grid ID 0
        Fiducials.blueGridMiddleAprilTag.getPose(), // Grid ID 1
        Fiducials.blueGridLeftAprilTag.getPose(), // Grid ID 2
      };

  // Keep track of last values so we don't re-compute if nothing changed.
  private int m_lastGridID = -1;
  private int m_lastNodeID = -1;
  private Boolean m_lastIsRed = null;

  public int getNodeID() {
    return m_lastNodeID;
  }

  // Subscribe to scoring location changes.
  final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  final IntegerArraySubscriber scoringLocationSub =
      inst.getTable("scoring-selector")
          .getIntegerArrayTopic("loc")
          .subscribe(new long[] {0, 6}, PubSubOption.sendAll(true)); // Default to a high cone
  // Publish scoring locations (for auto).
  final IntegerArrayPublisher scoringLocationPub =
      inst.getTable("scoring-selector")
          .getIntegerArrayTopic("loc")
          .publish(PubSubOption.sendAll(true));

  // For viz.
  private final Field2d m_fieldViz;

  private Pose2d m_scoringChassisPose = new Pose2d();
  private Translation2d m_scoringLocation = new Translation2d();

  public ScoringSelector(Field2d fieldViz) {
    m_fieldViz = fieldViz;
    // Show scheduler status in SmartDashboard.
    SmartDashboard.putData(this);
  }

  public Pose2d getScoringChassisPose() {
    return m_scoringChassisPose;
  }

  public Translation2d getScoringLocation() {
    return m_scoringLocation;
  }

  /** 0 is low, 1 is mid, 2 is high. */
  public int getLevel() {
    return m_lastNodeID / 3;
  }

  public boolean isCubeLocation() {
    return m_lastNodeID == 4 || m_lastNodeID == 7;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateScoringLocation();
    m_fieldViz.getObject("Scoring Chassis Pose").setPose(getScoringChassisPose());
    m_fieldViz
        .getObject("Scoring Location")
        .setPose(new Pose2d(getScoringLocation(), new Rotation2d()));
  }

  public void setScoringLocation(final int grid_id, final int node_id) {
    scoringLocationPub.set(new long[] {grid_id, node_id});
  }

  /**
   * Update scoring location based on grid and node IDs.
   *
   * <p>Grids are numbered 0, 1, and 2 and correspond to the left, middle, and right grids when
   * viewed from the alliance station perspective.
   *
   * <p>Nodes are numbered 0-9 where 0-2 are bottom nodes, 3-5 are middle are top nodes.
   *
   * <p>All nodes within a given height are left-to-right.
   */
  private void updateScoringLocation() {
    final long[] loc = scoringLocationSub.get();
    if (loc.length != 2) {
      return;
    }
    final int gridID = (int) loc[0];
    final int nodeID = (int) loc[1];

    final var alliance = DriverStation.getAlliance();
    final boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;

    // Bounds check.
    if (gridID < 0 || gridID > 2 || nodeID < 0 || nodeID > 8) {
      return;
    }

    // Don't re-calculate if nothing changed.
    if (m_lastIsRed != null
        && gridID == m_lastGridID
        && nodeID == m_lastNodeID
        && isRed == m_lastIsRed) {
      return;
    }
    m_lastGridID = gridID;
    m_lastNodeID = nodeID;
    m_lastIsRed = isRed;

    final Pose3d aprilTagPose = isRed ? redAprilTagPoses[gridID] : blueAprilTagPoses[gridID];

    switch (nodeID) {
      case 0:
      case 3:
      case 6:
        {
          m_scoringChassisPose = getRightScoringChassisPose(aprilTagPose);
          break;
        }
      case 1:
      case 4:
      case 7:
        {
          m_scoringChassisPose = getCenterScoringChassisPose(aprilTagPose);
          break;
        }
      case 2:
      case 5:
      case 8:
        {
          m_scoringChassisPose = getLeftScoringChassisPose(aprilTagPose);
          break;
        }
      default:
        {
          break;
        }
    }

    switch (nodeID) {
        // Bottom Row
      case 0:
        {
          m_scoringLocation = getBottomRightTargetPose(aprilTagPose).toPose2d().getTranslation();
          break;
        }
      case 1:
        {
          m_scoringLocation = getBottomMidTargetPose(aprilTagPose).toPose2d().getTranslation();
          break;
        }
      case 2:
        {
          m_scoringLocation = getBottomLeftTargetPose(aprilTagPose).toPose2d().getTranslation();
          break;
        }
        // Middle Row
      case 3:
        {
          m_scoringLocation = getMidRightTargetPose(aprilTagPose).toPose2d().getTranslation();
          break;
        }
      case 4:
        {
          m_scoringLocation = getMidMidTargetPose(aprilTagPose).toPose2d().getTranslation();
          break;
        }
      case 5:
        {
          m_scoringLocation = getMidLeftTargetPose(aprilTagPose).toPose2d().getTranslation();
          break;
        }
        // Top Row
      case 6:
        {
          m_scoringLocation = getTopRightTargetPose(aprilTagPose).toPose2d().getTranslation();
          break;
        }
      case 7:
        {
          m_scoringLocation = getTopMidTargetPose(aprilTagPose).toPose2d().getTranslation();
          break;
        }
      case 8:
        {
          m_scoringLocation = getTopLeftTargetPose(aprilTagPose).toPose2d().getTranslation();
          break;
        }
      default:
        {
          break;
        }
    }
  }

  private Pose2d getRightScoringChassisPose(Pose3d aprilTagPose) {
    return aprilTagPose
        .transformBy(
            new Transform3d(
                new Translation3d(kChassisXOffset, kSideYOffset, 0.0),
                new Rotation3d(0.0, 0.0, Math.PI)))
        .toPose2d();
  }

  private Pose2d getCenterScoringChassisPose(Pose3d aprilTagPose) {
    return aprilTagPose
        .transformBy(
            new Transform3d(
                new Translation3d(kChassisXOffset, 0.0, 0.0), new Rotation3d(0.0, 0.0, Math.PI)))
        .toPose2d();
  }

  private Pose2d getLeftScoringChassisPose(Pose3d aprilTagPose) {
    return aprilTagPose
        .transformBy(
            new Transform3d(
                new Translation3d(kChassisXOffset, -kSideYOffset, 0.0),
                new Rotation3d(0.0, 0.0, Math.PI)))
        .toPose2d();
  }

  private Pose3d getTopLeftTargetPose(Pose3d aprilTagPose) {
    return aprilTagPose.transformBy(
        new Transform3d(
            new Translation3d(
                kTopXOffset, // x
                -kSideYOffset, // y
                kTopZLeftRightOffset), // z
            new Rotation3d()));
  }

  private Pose3d getTopMidTargetPose(Pose3d aprilTagPose) {
    return aprilTagPose.transformBy(
        new Transform3d(
            new Translation3d(
                kTopXOffset, // x
                0, // y
                kTopZMidOffset), // z
            new Rotation3d()));
  }

  private Pose3d getTopRightTargetPose(Pose3d aprilTagPose) {
    return aprilTagPose.transformBy(
        new Transform3d(
            new Translation3d(
                kTopXOffset, // x
                kSideYOffset, // y
                kTopZLeftRightOffset), // z
            new Rotation3d()));
  }

  private Pose3d getMidLeftTargetPose(Pose3d aprilTagPose) {
    return aprilTagPose.transformBy(
        new Transform3d(
            new Translation3d(
                kMidXOffset, // x
                -kSideYOffset, // y
                kMidZLeftRightOffset), // z
            new Rotation3d()));
  }

  private Pose3d getMidMidTargetPose(Pose3d aprilTagPose) {
    return aprilTagPose.transformBy(
        new Transform3d(
            new Translation3d(
                kMidXOffset, // x
                0, // y
                kMidZMidOffset), // z
            new Rotation3d()));
  }

  private Pose3d getMidRightTargetPose(Pose3d aprilTagPose) {
    return aprilTagPose.transformBy(
        new Transform3d(
            new Translation3d(
                kMidXOffset, // x
                kSideYOffset, // y
                kMidZLeftRightOffset), // z
            new Rotation3d()));
  }

  private Pose3d getBottomLeftTargetPose(Pose3d aprilTagPose) {
    return aprilTagPose.transformBy(
        new Transform3d(
            new Translation3d(
                kBottomXOffset, // x
                -kSideYOffset, // y
                kBottomZOffset), // z
            new Rotation3d()));
  }

  private Pose3d getBottomMidTargetPose(Pose3d aprilTagPose) {
    return aprilTagPose.transformBy(
        new Transform3d(
            new Translation3d(
                kBottomXOffset, // x
                0, // y
                kBottomZOffset), // z
            new Rotation3d()));
  }

  private Pose3d getBottomRightTargetPose(Pose3d aprilTagPose) {
    return aprilTagPose.transformBy(
        new Transform3d(
            new Translation3d(
                kBottomXOffset, // x
                kSideYOffset, // y
                kBottomZOffset), // z
            new Rotation3d()));
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  // --- END STUFF FOR SIMULATION ---
}
