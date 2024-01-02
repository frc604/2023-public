// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.quixlib.planning.SimpleSwerveTrajectory;
import frc.quixlib.planning.SwerveTrajectoryState;
import frc.robot.Constants;
import frc.robot.subsystems.ScoringSelector;
import frc.robot.subsystems.Swerve;

public class DriveToPose extends CommandBase {
  private final Swerve m_swerve;
  private final ScoringSelector m_scoringSelectorSubsystem;
  private SimpleSwerveTrajectory m_traj;
  private final Timer m_timer = new Timer();
  private Pose2d m_targetPose;

  public DriveToPose(Swerve swerve, ScoringSelector scoringSelectorSubsystem) {
    m_swerve = swerve;
    m_scoringSelectorSubsystem = scoringSelectorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetPose = m_scoringSelectorSubsystem.getScoringChassisPose();
    m_traj =
        new SimpleSwerveTrajectory(
            m_swerve.getPose(),
            m_swerve.getFieldSpeeds(),
            m_targetPose,
            new Constraints(
                Constants.Swerve.maxDriveSpeed * 0.25,
                Constants.Swerve.maxDriveAcceleration * 0.25),
            new Constraints(
                Constants.Swerve.maxAngularVelocity * 0.25,
                Constants.Swerve.maxAngularAcceleration * 0.25));
    m_timer.start();
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final SwerveTrajectoryState state = m_traj.getState(m_timer.get());
    m_swerve.driveToPose(
        state.pose, state.vx, state.vy, state.vTheta, Constants.Swerve.teleopScrubLimit);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > m_traj.totalTime();
  }
}
