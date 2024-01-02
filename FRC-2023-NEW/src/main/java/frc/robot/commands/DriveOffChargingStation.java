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

public class DriveOffChargingStation extends Command {
  private final Swerve m_swerve;
  private final QuixPigeon2 m_imu;
  private final Timer m_timer = new Timer();
  private MedianFilter m_pitchFilter = new MedianFilter(10);
  private final double kDeadband = Math.toRadians(2.0);
  private double m_robotPitchZeroOffset;

  public DriveOffChargingStation(Swerve swerve, QuixPigeon2 imu, double robotPitchZeroOffset) {
    m_swerve = swerve;
    m_imu = imu;
    m_robotPitchZeroOffset = robotPitchZeroOffset;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    m_pitchFilter.reset();
    m_timer.start();
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerve.driveClosedLoop(-1.0, 0, 0, false, Constants.Swerve.teleopScrubLimit);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    final double absPitch =
        m_pitchFilter.calculate(Math.abs(-m_imu.getRoll() - m_robotPitchZeroOffset));
    System.out.println(Math.toDegrees(absPitch));
    return absPitch < kDeadband; // || m_timer.get() > 5.0;
  }
}
