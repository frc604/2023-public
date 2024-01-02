// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevators;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.ScoringSelector;

public class ScoreWithGripper extends CommandBase {
  private final Gripper m_gripperSubsystem;
  private final Elevators m_elevatorSubsystem;
  private final ScoringSelector m_scoringSelector;
  private final Timer m_timer = new Timer();
  private boolean m_isScoringFinished;

  public ScoreWithGripper(
      Gripper gripperSubsystem, Elevators elevatorSubsystem, ScoringSelector scoringSelector) {
    m_gripperSubsystem = gripperSubsystem;
    m_elevatorSubsystem = elevatorSubsystem;
    m_scoringSelector = scoringSelector;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gripperSubsystem, elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    m_timer.reset();
    m_isScoringFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_gripperSubsystem.score();

    // Prevent arm from getting stuck behind pegs after scoring
    final double gripperRunTime =
        m_scoringSelector.isCubeLocation()
            ? Constants.Gripper.gripperScoreCubeRunTime
            : Constants.Gripper.gripperScoreConeRunTime;
    if (m_timer.get() > gripperRunTime) {
      switch (m_scoringSelector.getLevel()) {
        case 0:
        case 1:
          {
            if (!m_scoringSelector.isCubeLocation()) {
              m_elevatorSubsystem.moveZToPos(
                  Constants.Elevators.zScoreMidConeHeight
                      + Constants.Elevators.zAfterScoreConeOffset);
            }
            m_isScoringFinished = true;
            break;
          }
        case 2:
        default:
          {
            if (!m_scoringSelector.isCubeLocation()) {
              m_elevatorSubsystem.moveZToPos(
                  Constants.Elevators.zScoreHighConeHeight
                      + Constants.Elevators.zAfterScoreConeOffset);
            }
            m_isScoringFinished = true;
            break;
          }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gripperSubsystem.stopSpin();
    m_gripperSubsystem.setHasPiece(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isScoringFinished && m_elevatorSubsystem.isZMotionFinished();
  }
}
