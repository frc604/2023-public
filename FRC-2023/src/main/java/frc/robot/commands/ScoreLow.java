// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class ScoreLow extends CommandBase {
  private final Intake m_intakeSubsystem;
  private final Timer m_rollerTimer = new Timer();

  public ScoreLow(Intake intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_rollerTimer.start();
    m_intakeSubsystem.moveSliderTo(Constants.Intake.Slider.scoreLowSliderExtension);
    m_intakeSubsystem.moveArmTo(Constants.Intake.IntakeArm.scoreLowArmAngle);
    m_intakeSubsystem.moveReorienterTo(Constants.Intake.Reorienter.scoreLowReorienterAngle);
    m_intakeSubsystem.spinRollerSlow();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intakeSubsystem.isSliderMotionFinished()
        && m_intakeSubsystem.isArmMotionFinished()
        && m_intakeSubsystem.isReorienterMotionFinished()) {
      m_intakeSubsystem.score();
    } else {
      m_rollerTimer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_rollerTimer.get() > 1.0;
  }
}
