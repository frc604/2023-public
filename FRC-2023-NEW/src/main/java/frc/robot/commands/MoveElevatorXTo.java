// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevators;

public class MoveElevatorXTo extends Command {
  private final Elevators m_elevatorSubsystem;
  private final double m_pos;

  public MoveElevatorXTo(Elevators elevatorSubsystem, double pos) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_pos = pos;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSubsystem.moveXToPos(
        Math.min(Math.max(m_pos, Constants.Elevators.xMinExtend), Constants.Elevators.xMaxExtend));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevatorSubsystem.isXMotionFinished();
  }
}
