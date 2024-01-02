// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveBackSlightly extends Command {
  private final Swerve m_swerve;
  private final boolean m_reversed;
  private final Timer m_timer = new Timer();

  public DriveBackSlightly(Swerve swerve, boolean reversed) {
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
    if (m_timer.get() < 0.2) {
      m_swerve.driveClosedLoop(
          -0.5 * (m_reversed ? -1.0 : 1.0), 0, 0, false, Constants.Swerve.teleopScrubLimit);
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
    return m_timer.get() > 0.25;
  }
}
