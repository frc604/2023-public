// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.quixlib.devices.QuixPigeon2;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class FinalBangBangBalance extends Command {
  private final Swerve m_swerve;
  private MedianFilter m_pitchRateFilter = new MedianFilter(5);
  private final QuixPigeon2 m_imu;
  private final Timer m_timer = new Timer();
  private final double target = 3.5; // deg
  private final double vel = 0.10; // m/s
  private double m_robotPitchZeroOffset;

  public FinalBangBangBalance(Swerve swerve, QuixPigeon2 imu, double robotPitchZeroOffset) {
    m_swerve = swerve;
    m_imu = imu;
    m_robotPitchZeroOffset = robotPitchZeroOffset;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    m_pitchRateFilter.reset();
    m_timer.start();
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double robotPitch =
        m_pitchRateFilter.calculate(-m_imu.getRoll() - m_robotPitchZeroOffset);

    if (robotPitch > Math.toRadians(target)) {
      m_swerve.driveClosedLoop(-vel, 0, 0, false, Constants.Swerve.teleopScrubLimit);
    } else if (robotPitch < -Math.toRadians(target)) {
      m_swerve.driveClosedLoop(vel, 0, 0, false, Constants.Swerve.teleopScrubLimit);
    } else {
      m_swerve.stopWithX();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
