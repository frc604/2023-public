// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.quixlib.math.MathUtils;
import frc.robot.Constants;
import frc.robot.subsystems.ScoringSelector;
import frc.robot.subsystems.Swerve;

public class TurnToTarget extends CommandBase {
  private final Swerve m_swerve;
  private final ScoringSelector m_scoringSelectorSubsystem;
  private final Timer m_stationaryTimer = new Timer();
  private static final double kAngleOffset = Math.toRadians(-3.0);

  public TurnToTarget(Swerve swerve, ScoringSelector scoringSelectorSubsystem) {
    m_swerve = swerve;
    m_scoringSelectorSubsystem = scoringSelectorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_stationaryTimer.start();
    m_stationaryTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final Translation2d target = m_scoringSelectorSubsystem.getScoringLocation();
    final double targetAngle =
        MathUtils.placeInScope(
            Math.atan2(
                    target.getY() - m_swerve.getPose().getY(),
                    target.getX() - m_swerve.getPose().getX())
                + kAngleOffset,
            m_swerve.getPose().getRotation().getRadians());

    m_swerve.turnToAngle(new Rotation2d(targetAngle), 0.0, Constants.Swerve.autoScrubLimit);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.driveOpenLoop(0, 0, 0, false, Constants.Swerve.autoScrubLimit);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    final Translation2d target = m_scoringSelectorSubsystem.getScoringLocation();
    final double targetAngle =
        MathUtils.placeInScope(
            Math.atan2(
                    target.getY() - m_swerve.getPose().getY(),
                    target.getX() - m_swerve.getPose().getX())
                + kAngleOffset,
            m_swerve.getPose().getRotation().getRadians());

    final boolean withinTolerance =
        Math.abs(
                MathUtils.constrainAngleNegPiToPi(
                    m_swerve.getPose().getRotation().getRadians() - targetAngle))
            <= Units.degreesToRadians(0.5);

    final double kSettlingTime = 0.1; // s
    if (!withinTolerance) {
      m_stationaryTimer.reset();
      return false;
    } else if (withinTolerance && m_stationaryTimer.get() > kSettlingTime) {
      return true;
    }
    return false;
  }
}
