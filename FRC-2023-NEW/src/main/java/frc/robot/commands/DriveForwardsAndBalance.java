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

public class DriveForwardsAndBalance extends Command {
  private final QuixPigeon2 m_imu;
  private final Swerve m_swerve;
  private final boolean m_reversed;
  private MedianFilter m_pitchRateFilter = new MedianFilter(5);
  private final Timer m_timer = new Timer();
  private double m_robotPitchZeroOffset;

  public DriveForwardsAndBalance(
      QuixPigeon2 imu, Swerve swerve, boolean reversed, double robotPitchZeroOffset) {
    m_imu = imu;
    m_swerve = swerve;
    m_reversed = reversed;
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
    m_swerve.driveClosedLoop(
        0.5 * (m_reversed ? -1.0 : 1.0), 0, 0, false, Constants.Swerve.teleopScrubLimit);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.driveOpenLoop(0.0, 0, 0, false, Constants.Swerve.teleopScrubLimit);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Positive robot pitch is negative IMU roll.
    final double robotPitch =
        m_pitchRateFilter.calculate(-m_imu.getRoll() - m_robotPitchZeroOffset);
    if (m_reversed) {
      return robotPitch < Math.toRadians(9.5) || m_timer.get() > 10.0;
    } else {
      return robotPitch > Math.toRadians(-9.5) || m_timer.get() > 10.0;
    }
  }
}
