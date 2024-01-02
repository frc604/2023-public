// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevators;
import frc.robot.subsystems.ScoringSelector;

public class ExtendArmToScore extends Command {
  private final Elevators m_elevatorSubsystem;
  private final Arm m_armSubsystem;
  private final ScoringSelector m_scoringSelector;
  private final boolean m_isAuto;

  public ExtendArmToScore(
      Elevators elevatorSubsystem, Arm armSubsytem, ScoringSelector scoringSelector) {
    this(elevatorSubsystem, armSubsytem, scoringSelector, false);
  }

  public ExtendArmToScore(
      Elevators elevatorSubsystem,
      Arm armSubsytem,
      ScoringSelector scoringSelector,
      boolean isAuto) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_armSubsystem = armSubsytem;
    m_scoringSelector = scoringSelector;
    m_isAuto = isAuto;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem, armSubsytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (m_scoringSelector.getLevel()) {
        // Never score low with arm. If we choose low treat it as mid.
      case 0:
      case 1:
        {
          m_elevatorSubsystem.moveZToPos(
              m_scoringSelector.isCubeLocation()
                  ? Constants.Elevators.zScoreMidCubeHeight
                  : Constants.Elevators.zScoreMidConeHeight);
          m_elevatorSubsystem.moveXToPos(Constants.Elevators.xScoreMidExtend);
          m_armSubsystem.moveArmTo(Constants.Arm.maxArmAngle);
          break;
        }
      case 2:
      default:
        {
          m_elevatorSubsystem.moveZToPos(
              m_scoringSelector.isCubeLocation()
                  ? Constants.Elevators.zScoreHighCubeHeight
                  : Constants.Elevators.zScoreHighConeHeight);
          m_elevatorSubsystem.moveXToPos(
              m_isAuto
                  ? Constants.Elevators.xScoreHighExtendAuto
                  : Constants.Elevators.xScoreHighExtend);
          m_armSubsystem.moveArmTo(Constants.Arm.maxArmAngle);
          break;
        }
    }
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
    final boolean armFinished =
        m_scoringSelector.isCubeLocation()
            ? m_armSubsystem.isArmMotionFinishedForCube()
            : m_armSubsystem.isArmMotionFinished();
    return m_elevatorSubsystem.isZMotionFinished()
        && m_elevatorSubsystem.isXMotionFinished()
        && armFinished;
  }
}
