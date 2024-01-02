// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevators;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;

public class LowerElevatorForHandoff extends CommandBase {
  private final Intake m_intakeSubsystem;
  private final Elevators m_elevatorSubsystem;
  private final Arm m_armSubsystem;
  private final Gripper m_gripperSubsystem;

  public LowerElevatorForHandoff(
      Intake intakeSubsystem,
      Elevators elevatorSubsystem,
      Arm armSubsytem,
      Gripper gripperSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    m_elevatorSubsystem = elevatorSubsystem;
    m_armSubsystem = armSubsytem;
    m_gripperSubsystem = gripperSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem, armSubsytem, gripperSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.moveSliderTo(
        m_intakeSubsystem.isConeTipOut()
            ? Constants.Intake.Slider.coneOutSliderHandoffExtension
            : Constants.Intake.Slider.coneInSliderHandoffExtension);
    m_elevatorSubsystem.moveZToPos(Constants.Elevators.zHandoffHeight, true);
    m_armSubsystem.increaseCurrentLimit();
    m_armSubsystem.moveArmTo(Constants.Arm.minArmAngle);
    m_gripperSubsystem.handoff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.decreaseCurrentLimit();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intakeSubsystem.isSliderMotionFinished()
        && m_elevatorSubsystem.isZMotionFinished()
        && m_armSubsystem.isArmMotionFinished()
        && m_gripperSubsystem.isRollerStopped();
  }
}
