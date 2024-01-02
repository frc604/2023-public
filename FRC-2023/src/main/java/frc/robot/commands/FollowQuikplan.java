// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.quixlib.devices.QuixPigeon2;
import frc.quixlib.swerve.QuikPlanSwerveTrajectoryReader;
import frc.quixlib.swerve.QuikPlanSwerveTrajectoryReader.QuikPlanAction;
import frc.quixlib.swerve.QuikPlanSwerveTrajectoryReader.QuikplanTrajectoryState;
import frc.quixlib.wpilib.LazySequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevators;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ScoringSelector;
import frc.robot.subsystems.Swerve;
import java.util.ArrayList;
import java.util.HashSet;

public class FollowQuikplan extends CommandBase {
  private final boolean kDebugPrint = false;

  private final QuikPlanSwerveTrajectoryReader m_reader;
  private final ScoringSelector m_scoringSelector;
  private final QuixPigeon2 m_imu;
  private final Swerve m_swerve;
  private final Intake m_intake;
  private final Elevators m_elevator;
  private final Arm m_arm;
  private final Gripper m_gripper;
  private final Timer m_timer = new Timer();
  private boolean m_timerStopped = false;
  private final HashSet<Double> m_scheduledActionTimes = new HashSet<>();
  private final ArrayList<Command> m_commands = new ArrayList<>();
  private boolean m_firstScore = true;
  private double m_robotPitchZeroOffset = 0.0;

  private ArrayList<Double> m_times = new ArrayList<>();
  private ArrayList<Double> m_xErrors = new ArrayList<>();
  private ArrayList<Double> m_yErrors = new ArrayList<>();
  private ArrayList<Double> m_thetaErrors = new ArrayList<>();

  public FollowQuikplan(
      QuikPlanSwerveTrajectoryReader reader,
      ScoringSelector scoringSelector,
      QuixPigeon2 imu,
      Swerve swerve,
      Intake intake,
      Elevators elevator,
      Arm arm,
      Gripper gripper) {
    m_reader = reader;
    m_scoringSelector = scoringSelector;
    m_imu = imu;
    m_swerve = swerve;
    m_intake = intake;
    m_elevator = elevator;
    m_arm = arm;
    m_gripper = gripper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.resetPose(m_reader.getInitialPose());
    m_timer.reset();
    m_timer.start();
    m_firstScore = true;
    m_robotPitchZeroOffset = -m_imu.getRoll(); // Negative IMU roll is robot pitch
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double curTime = m_timer.get();
    final QuikplanTrajectoryState targetState = m_reader.getState(curTime);
    final var actionEntry = m_reader.getAction(curTime);
    final double actionTime = actionEntry == null ? 0.0 : actionEntry.getKey();
    final QuikPlanAction action = actionEntry == null ? null : actionEntry.getValue();

    // Schedule the action at this timestep if it has not already been scheduled.
    if (actionEntry != null && !m_scheduledActionTimes.contains(actionTime)) {
      m_scheduledActionTimes.add(actionTime);

      // Note: These must match the ActionTypes defined in quikplan.py
      switch (action.actionType) {
        case 1: // SCORE_PIECE
          m_timerStopped = true;
          // Grid and node IDs in Quikplan are defined from the blue perspective.
          // Mirror them for red.
          if (DriverStation.getAlliance() == Alliance.Blue) {
            m_scoringSelector.setScoringLocation(action.gridID, action.nodeID);
          } else {
            final int quotient = action.nodeID / 3;
            final int remainder = action.nodeID % 3;
            m_scoringSelector.setScoringLocation(2 - action.gridID, 3 * quotient + 2 - remainder);
          }
          schedule(
              new LazySequentialCommandGroup(
                  // Take over from the trajectory, so stop the timer.
                  new InstantCommand(() -> m_timer.stop()),
                  // Make sure we are gripping the first piece.
                  m_firstScore ? new RunGripperHold(m_gripper) : new NullCommand(),
                  // Final pose refinement. Don't run on first piece.
                  // m_firstScore ? new NullCommand() : new DriveToPose(m_swerve,
                  // m_scoringSelector),
                  // m_firstScore ? new NullCommand() : new TurnToTarget(m_swerve,
                  // m_scoringSelector),
                  m_firstScore
                      ? new LazySequentialCommandGroup(
                          new ExtendArmToScore(m_elevator, m_arm, m_scoringSelector, true),
                          new ScoreWithGripper(m_gripper, m_elevator, m_scoringSelector))
                      :
                      // Wait for handoff then score.
                      new LazySequentialCommandGroup(
                          new WaitForHandoff(m_gripper),
                          new ExtendArmToScore(m_elevator, m_arm, m_scoringSelector, true),
                          new ScoreWithGripper(m_gripper, m_elevator, m_scoringSelector)),
                  // We can start driving while retracting the arm/elevator.
                  new InstantCommand(
                      () -> {
                        m_timer.start();
                        m_timerStopped = false;
                      }),
                  new RetractArmAfterScoring(m_elevator, m_arm),
                  new LowerElevatorAfterScoring(m_elevator)));
          m_firstScore = false;
          break;
        case 2: // DEPLOY_INTAKE
          schedule(new IntakeDeploy(null, m_intake));
          break;
        case 3: // RETRACT_INTAKE and perform handoff
          schedule(
              new LazySequentialCommandGroup(
                  new InstantCommand(() -> m_intake.setIsConeTipOut(true)),
                  new Reorient(m_intake),
                  new AdjustSliderForMeasure(m_intake),
                  new AdjustElevatorXForHandoff(m_intake, m_elevator),
                  // Perform handoff and extend to score.
                  new LazySequentialCommandGroup(
                      new LowerElevatorForHandoff(m_intake, m_elevator, m_arm, m_gripper),
                      new HandoffPiece(m_intake, m_elevator, m_arm, m_gripper),
                      new MoveToPrescore(m_elevator, m_arm))));
          break;
        case 4: // BALANCE
          m_timerStopped = true;
          schedule(
              new LazySequentialCommandGroup(
                  // Take over from the trajectory, so stop the timer.
                  new InstantCommand(() -> m_timer.stop()),
                  new DriveOntoPlatform(m_swerve, false),
                  new DriveForwardsAndBalance(m_imu, m_swerve, false, m_robotPitchZeroOffset),
                  new DriveBackSlightly(m_swerve, false),
                  new FinalBangBangBalance(m_swerve, m_imu, m_robotPitchZeroOffset)));
          break;
        case 5: // BALANCE BACKWARDS
          m_timerStopped = true;
          schedule(
              new LazySequentialCommandGroup(
                  // Take over from the trajectory, so stop the timer.
                  new InstantCommand(() -> m_timer.stop()),
                  new DriveOntoPlatform(m_swerve, true),
                  new DriveForwardsAndBalance(m_imu, m_swerve, true, m_robotPitchZeroOffset),
                  new DriveBackSlightly(m_swerve, true),
                  new FinalBangBangBalance(m_swerve, m_imu, m_robotPitchZeroOffset)));
          break;
        case 6: // POOP CHUTE
          m_timerStopped = true;
          schedule(
              new LazySequentialCommandGroup(
                  // Take over from the trajectory, so stop the timer.
                  new InstantCommand(() -> m_timer.stop()),
                  new ScoreWithPoopChute(m_intake),
                  new InstantCommand(
                      () -> {
                        m_timer.start();
                        m_timerStopped = false;
                      })));
          break;
        case 7: // DRIVE OFF CHARGING STATION AND BALANCE REVERSE
          m_timerStopped = true;
          schedule(
              new LazySequentialCommandGroup(
                  // Take over from the trajectory, so stop the timer.
                  new InstantCommand(() -> m_timer.stop()),
                  new DriveOntoPlatform(m_swerve, true),
                  new DriveOffChargingStation(m_swerve, m_imu, m_robotPitchZeroOffset),
                  new DriveOntoPlatform(m_swerve, false),
                  new DriveForwardsAndBalance(m_imu, m_swerve, false, m_robotPitchZeroOffset),
                  new DriveBackSlightly(m_swerve, false),
                  new FinalBangBangBalance(m_swerve, m_imu, m_robotPitchZeroOffset)));
        default:
          break;
      }
    }

    if (!m_timerStopped) {
      // We explicitly schedule this as a command to deal with DriveToPose and
      // TurnToTarget gaining control of the swerve sybsystem.
      schedule(
          new InstantCommand(
              () -> {
                final Pose2d poseError =
                    m_swerve.driveToPose(
                        targetState.pose,
                        targetState.xVel,
                        targetState.yVel,
                        targetState.thetaVel,
                        Constants.Swerve.autoScrubLimit);
                if (kDebugPrint) {
                  m_times.add(m_timer.get());
                  m_xErrors.add(poseError.getX());
                  m_yErrors.add(poseError.getY());
                  m_thetaErrors.add(poseError.getRotation().getRadians());
                }
              },
              m_swerve));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_swerve.stop();
    if (kDebugPrint) {
      for (int i = 0; i < m_times.size(); i++) {
        System.out.println(
            m_times.get(i)
                + ","
                + m_xErrors.get(i)
                + ","
                + m_yErrors.get(i)
                + ","
                + m_thetaErrors.get(i));
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    final double kEndTimeExtension = 1.0; // s
    return m_timer.hasElapsed(m_reader.getTotalTime() + kEndTimeExtension);
  }

  void schedule(Command c) {
    // Keep track of all non-InstantCommands so we can cancel them later if necessary.
    if (!(c instanceof InstantCommand)) {
      m_commands.add(c);
    }
    c.schedule();
  }

  @Override
  public void cancel() {
    super.cancel();
    for (Command command : m_commands) {
      command.cancel();
    }
  }
}
