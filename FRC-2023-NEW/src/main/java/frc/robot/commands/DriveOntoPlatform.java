// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveOntoPlatform extends Command {
  private final Swerve m_swerve;
  private final boolean m_reversed;
  private final Timer m_timer = new Timer();

  public DriveOntoPlatform(Swerve swerve, boolean reversed) {
    m_swerve = swerve;
    m_reversed = reversed;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    m_timer.start();
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerve.driveClosedLoop(
        1.75 * (m_reversed ? -1.0 : 1.0), 0, 0, false, Constants.Swerve.teleopScrubLimit);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.driveOpenLoop(0.0, 0, 0, false, Constants.Swerve.teleopScrubLimit);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    final double kTime = 1.75; // s
    return m_timer.get() > kTime;
  }
}
