// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class DetectPieceOrientation extends Command {
  private final Intake m_intakeSubsystem;

  public DetectPieceOrientation(Intake intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.moveArmTo(Constants.Intake.IntakeArm.minArmAngle);
    m_intakeSubsystem.moveSliderTo(Constants.Intake.Slider.detectConeSliderExtension);
    m_intakeSubsystem.moveReorienterTo(Constants.Intake.Reorienter.detectConeReorienterAngle);
    m_intakeSubsystem.setIsConeTipOut(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intakeSubsystem.getIntakeBeamBreakTriggered()) {
      m_intakeSubsystem.setIsConeTipOut(true);
    }
  }

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
