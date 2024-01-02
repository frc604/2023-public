// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevators;
import frc.robot.subsystems.Intake;

public class ZeroSelectedAxes extends Command {
  private final Intake m_intake;
  private final Elevators m_elevator;
  private final Arm m_arm;

  public ZeroSelectedAxes(Intake intake, Elevators elevator, Arm arm) {
    m_intake = intake;
    m_elevator = elevator;
    m_arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, elevator, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.zeroArm();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.zeroSensorPosition();
    m_arm.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
