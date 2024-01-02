// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class Reorient extends Command {
  private final Intake m_intakeSubsystem;

  public Reorient(Intake intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_intakeSubsystem.isConeTipOut()) {
      m_intakeSubsystem.moveSliderTo(Constants.Intake.Slider.coneOutSliderMeasureExtension);
      m_intakeSubsystem.moveReorienterTo(Constants.Intake.Reorienter.coneOutReorienterAngle);
    } else {
      m_intakeSubsystem.moveSliderTo(Constants.Intake.Slider.coneInSliderMeasureExtension);
      m_intakeSubsystem.moveReorienterTo(Constants.Intake.Reorienter.coneInReorienterAngle);
    }
    m_intakeSubsystem.moveArmTo(Constants.Intake.IntakeArm.minArmAngle);
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
    return m_intakeSubsystem.isSliderMotionFinished()
        && m_intakeSubsystem.isArmMotionFinished()
        && m_intakeSubsystem.isReorienterMotionFinished();
  }
}
