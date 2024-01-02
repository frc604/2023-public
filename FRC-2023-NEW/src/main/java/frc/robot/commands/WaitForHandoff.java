// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;

public class WaitForHandoff extends Command {
  private final Gripper m_gripper;

  public WaitForHandoff(Gripper gripper) {
    m_gripper = gripper;
    // Use addRequirements() here to declare subsystem dependencies.
    // We don't require the gripper we are just reading its piece state.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_gripper.hasPiece();
  }
}
