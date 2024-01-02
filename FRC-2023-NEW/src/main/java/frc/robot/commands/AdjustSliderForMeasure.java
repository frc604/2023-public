// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class AdjustSliderForMeasure extends Command {
  private final Intake m_intakeSubsystem;

  public AdjustSliderForMeasure(Intake intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    // We don't require the intake because we are just reading the sensor.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.moveSliderTo(
        m_intakeSubsystem.isConeTipOut()
            ? Constants.Intake.Slider.coneOutSliderMeasureExtension
            : Constants.Intake.Slider.coneInSliderMeasureExtension);
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
    return m_intakeSubsystem.isSliderMotionFinished();
  }
}
