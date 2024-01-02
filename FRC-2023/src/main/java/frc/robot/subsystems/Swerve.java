// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.quixlib.devices.QuixIMU;
import frc.quixlib.math.MathUtils;
import frc.quixlib.swerve.QuixSwerve;
import frc.quixlib.swerve.QuixSwerveModule;
import frc.quixlib.swerve.QuixSwerveModuleFactory;
import frc.quixlib.vision.Fiducial;
import frc.quixlib.vision.QuixVisionCamera;
import frc.robot.Constants;
import frc.robot.Fiducials;
import java.util.ArrayList;

public class Swerve extends QuixSwerve {
  private final ArrayList<QuixVisionCamera> m_cameras;

  public Swerve(QuixIMU imu, ArrayList<QuixVisionCamera> cameras, Field2d fieldViz) {
    super(
        imu,
        cameras,
        Constants.Swerve.maxDriveSpeed,
        Constants.Swerve.maxModuleAcceleration,
        Constants.Swerve.maxModuleSteeringRate,
        Constants.Swerve.driveController,
        Fiducials.fiducials,
        fieldViz);
    m_cameras = cameras;
  }

  protected QuixSwerveModule[] createModules() {
    QuixSwerveModuleFactory swerveModuleFactory =
        new QuixSwerveModuleFactory(
            Constants.Swerve.wheelCircumference,
            Constants.Swerve.driveRatio,
            Constants.Swerve.steeringRatio,
            Constants.Swerve.drivePIDConfig,
            Constants.Swerve.driveFeedforward,
            Constants.Swerve.steeringPIDConfig);
    QuixSwerveModule[] modules = {
      swerveModuleFactory.createModule(
              Constants.Swerve.FrontLeft.modulePosition,
              Constants.Swerve.FrontLeft.driveMotorID,
              Constants.Swerve.FrontLeft.steeringMotorID,
              Constants.Swerve.FrontLeft.canCoderID,
              Constants.Swerve.FrontLeft.absEncoderOffsetRad),
          swerveModuleFactory.createModule(
              Constants.Swerve.RearLeft.modulePosition,
              Constants.Swerve.RearLeft.driveMotorID,
              Constants.Swerve.RearLeft.steeringMotorID,
              Constants.Swerve.RearLeft.canCoderID,
              Constants.Swerve.RearLeft.absEncoderOffsetRad),
      swerveModuleFactory.createModule(
              Constants.Swerve.RearRight.modulePosition,
              Constants.Swerve.RearRight.driveMotorID,
              Constants.Swerve.RearRight.steeringMotorID,
              Constants.Swerve.RearRight.canCoderID,
              Constants.Swerve.RearRight.absEncoderOffsetRad),
          swerveModuleFactory.createModule(
              Constants.Swerve.FrontRight.modulePosition,
              Constants.Swerve.FrontRight.driveMotorID,
              Constants.Swerve.FrontRight.steeringMotorID,
              Constants.Swerve.FrontRight.canCoderID,
              Constants.Swerve.FrontRight.absEncoderOffsetRad),
    };
    return modules;
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  public void simulateVision(Pose2d simPose) {
    // Update camera based on sim pose.
    final Matrix<N4, N4> fieldToRobot =
        MathUtils.makeZRotTFMatrix(
            simPose.getX(), simPose.getY(), 0.0, simPose.getRotation().getRadians());
    for (var camera : m_cameras) {
      final Matrix<N4, N4> fieldToCamera = fieldToRobot.times(camera.getTransform());
      camera.updateSim(
          fieldToCamera,
          camera.getFiducialType() == Fiducial.Type.RETROREFLECTIVE
              ? Fiducials.retroreflectiveFiducials
              : Fiducials.aprilTagFiducials);
    }
  }
  // --- END STUFF FOR SIMULATION ---
}
