// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevators;
import frc.robot.subsystems.Intake;

public class AdjustElevatorXForHandoff extends CommandBase {
  private final Intake m_intakeSubsystem;
  private final Elevators m_elevatorSubsystem;
  private double piecePos;

  public AdjustElevatorXForHandoff(Intake intakeSubsystem, Elevators elevatorSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    m_elevatorSubsystem = elevatorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    // We don't require the intake because we are just reading the sensor.
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    piecePos = m_intakeSubsystem.getIntakeBeamBreakFilteredDistance();

    m_elevatorSubsystem.moveXToPos(
        Math.max(
            Constants.Elevators.xMinExtend,
            Math.min(
                Constants.Elevators.xMaxExtend, piecePos + Constants.Elevators.xHandoffPosOffset)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double newPiecePos = m_intakeSubsystem.getIntakeBeamBreakFilteredDistance();

    if (Math.abs(newPiecePos - piecePos) > Constants.Elevators.xHandoffBuffer) {
      piecePos = newPiecePos;
      m_elevatorSubsystem.moveXToPos(
          Math.max(
              Constants.Elevators.xMinExtend,
              Math.min(
                  Constants.Elevators.xMaxExtend,
                  piecePos + Constants.Elevators.xHandoffPosOffset)));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevatorSubsystem.isXMotionFinished();
  }
}
