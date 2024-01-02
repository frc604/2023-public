// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeDeploy extends Command {
  private final XboxController m_xboxController; // May be null.
  private final Intake m_intakeSubsystem;

  public IntakeDeploy(XboxController xboxController, Intake intakeSubsystem) {
    m_xboxController = xboxController;
    m_intakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.moveSliderTo(Constants.Intake.Slider.maxSliderExtension);
    m_intakeSubsystem.moveArmTo(Constants.Intake.IntakeArm.deployArmAngle);
    m_intakeSubsystem.moveReorienterTo(Constants.Intake.Reorienter.deployReorienterAngle);
    m_intakeSubsystem.startRollerSpin();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intakeSubsystem.isRollerStalled() && m_xboxController != null) {
      m_xboxController.setRumble(RumbleType.kBothRumble, 1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.spinRollerSlow();
    if (m_xboxController != null) {
      m_xboxController.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
