// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.quixlib.devices.QuixPigeon2;
import frc.quixlib.swerve.QuikPlanSwerveTrajectoryReader;
import frc.quixlib.vision.PhotonVisionCamera;
import frc.quixlib.vision.QuixVisionCamera;
import frc.robot.commands.AdjustElevatorXForHandoff;
import frc.robot.commands.AdjustSliderForMeasure;
import frc.robot.commands.DetectPieceOrientation;
import frc.robot.commands.ExtendArmToScore;
import frc.robot.commands.FollowQuikplan;
import frc.robot.commands.HandoffPiece;
import frc.robot.commands.IntakeDeploy;
import frc.robot.commands.IntakeStow;
import frc.robot.commands.LowerElevatorAfterScoring;
import frc.robot.commands.LowerElevatorForHandoff;
import frc.robot.commands.MoveElevatorXZTo;
import frc.robot.commands.MoveElevatorZTo;
import frc.robot.commands.NullCommand;
import frc.robot.commands.PrepareForHandoff;
import frc.robot.commands.Reorient;
import frc.robot.commands.RetractArmAfterScoring;
import frc.robot.commands.ScoreLow;
import frc.robot.commands.ScoreWithGripper;
import frc.robot.commands.TeleopSwerveCommand;
import frc.robot.commands.ZeroSelectedAxes;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevators;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.ScoringSelector;
import frc.robot.subsystems.Swerve;
import java.util.ArrayList;
import java.util.Arrays;

public class RobotContainer {
  // Controllers
  public final XboxController driverXbox = new XboxController(0); // Placeholder Controller

  // Driver Buttons
  private final JoystickButton buttonY =
      new JoystickButton(driverXbox, XboxController.Button.kY.value);
  private final JoystickButton buttonA =
      new JoystickButton(driverXbox, XboxController.Button.kA.value);
  private final JoystickButton buttonX =
      new JoystickButton(driverXbox, XboxController.Button.kX.value);
  private final JoystickButton buttonB =
      new JoystickButton(driverXbox, XboxController.Button.kB.value);
  private final Trigger rightTrigger = new Trigger(() -> driverXbox.getRightTriggerAxis() > 0.2);
  private final Trigger leftTrigger = new Trigger(() -> driverXbox.getLeftTriggerAxis() > 0.2);
  private final JoystickButton bumperLeft =
      new JoystickButton(driverXbox, XboxController.Button.kLeftBumper.value);
  private final JoystickButton bumperRight =
      new JoystickButton(driverXbox, XboxController.Button.kRightBumper.value);
  private final JoystickButton buttonBack =
      new JoystickButton(driverXbox, XboxController.Button.kBack.value);
  private final JoystickButton buttonStart =
      new JoystickButton(driverXbox, XboxController.Button.kStart.value);

  // Simulation
  private final Field2d fieldViz = new Field2d();

  // Sensors
  private final QuixPigeon2 imu = new QuixPigeon2(Constants.pigeonID);
  private final ArrayList<QuixVisionCamera> cameras =
      new ArrayList<>(
          Arrays.asList(
              new PhotonVisionCamera(
                  "photonvision-leftcam",
                  Constants.Cameras.LeftCam.robotToCameraT,
                  Constants.Cameras.LeftCam.fovWidth,
                  Constants.Cameras.LeftCam.fovHeight,
                  Constants.Cameras.LeftCam.pipelineConfigs),
              new PhotonVisionCamera(
                  "photonvision-rightcam",
                  Constants.Cameras.RightCam.robotToCameraT,
                  Constants.Cameras.RightCam.fovWidth,
                  Constants.Cameras.RightCam.fovHeight,
                  Constants.Cameras.RightCam.pipelineConfigs)));

  // Subsystems
  private final ScoringSelector scoringSelector = new ScoringSelector(fieldViz);
  private final Swerve swerve = new Swerve(imu, cameras, fieldViz);
  private final Mechanism2d simViz = new Mechanism2d(100, 100);
  private final Intake intakeSubsystem = new Intake();
  private final Elevators elevatorsSubsystem = new Elevators();
  private final Arm armSubsystem = new Arm();
  private final Gripper gripperSubsystem = new Gripper();
  private final LEDs leds = new LEDs(intakeSubsystem, scoringSelector);

  // Misc.
  private final QuikPlanSwerveTrajectoryReader trajectoryReader =
      new QuikPlanSwerveTrajectoryReader(fieldViz);

  public RobotContainer() {
    // Default commands
    swerve.setDefaultCommand(new TeleopSwerveCommand(swerve, driverXbox, true, true));

    configureBindings();

    // Visualize in SmartDashboard.
    SmartDashboard.putData("Field", fieldViz);
    SmartDashboard.putData("Robot Viz", simViz);
  }

  /** */
  private void configureBindings() {
    // Gyro reset
    buttonY.onTrue(
        new InstantCommand(
            () ->
                swerve.setContinuousYaw(
                    DriverStation.getAlliance() == Alliance.Blue ? 0.0 : Math.PI),
            swerve));

    // Intake
    rightTrigger.onTrue(
        new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  gripperSubsystem.setHasPiece(false);
                }),
            new MoveElevatorZTo(elevatorsSubsystem, Constants.Elevators.zIntakeClearanceHeight),
            new IntakeDeploy(driverXbox, intakeSubsystem)));
    rightTrigger.onFalse(
        new SequentialCommandGroup(
            // Reorient
            new MoveElevatorZTo(elevatorsSubsystem, Constants.Elevators.zIntakeClearanceHeight),
            new ParallelCommandGroup(
                new PrepareForHandoff(elevatorsSubsystem, armSubsystem),
                new SequentialCommandGroup(
                    new ConditionalCommand(
                        // Coming from intake, run full detect & reorient.
                        new DetectPieceOrientation(intakeSubsystem),
                        // Already stowed, force reorient without detection.
                        new InstantCommand(
                            () -> intakeSubsystem.setIsConeTipOut(!intakeSubsystem.isConeTipOut())),
                        intakeSubsystem::isCloserToDeployThanStow),
                    new Reorient(intakeSubsystem))),
            // Prepare for handoff
            new AdjustSliderForMeasure(intakeSubsystem),
            new AdjustElevatorXForHandoff(intakeSubsystem, elevatorsSubsystem)));

    // Score low.
    buttonA.onTrue(
        new SequentialCommandGroup(
            new MoveElevatorZTo(elevatorsSubsystem, Constants.Elevators.zScoreLowClearanceHeight),
            new ScoreLow(intakeSubsystem),
            new IntakeStow(intakeSubsystem),
            new MoveElevatorZTo(elevatorsSubsystem, Constants.Elevators.zIntakeClearanceHeight)));

    // // Drive to pose without scoring
    // leftTrigger.whileTrue(new DriveToPose(swerve, scoringSelector));

    // Manual extend
    buttonX.onTrue(
        new SequentialCommandGroup(
            // Finish handoff if not holding a piece
            new ConditionalCommand(
                new NullCommand(),
                new SequentialCommandGroup(
                    new AdjustSliderForMeasure(intakeSubsystem),
                    new AdjustElevatorXForHandoff(
                        intakeSubsystem, elevatorsSubsystem), // TODO: maybe delete
                    new LowerElevatorForHandoff(
                        intakeSubsystem, elevatorsSubsystem, armSubsystem, gripperSubsystem),
                    new HandoffPiece(
                        intakeSubsystem, elevatorsSubsystem, armSubsystem, gripperSubsystem)),
                gripperSubsystem::hasPiece),
            new SequentialCommandGroup(
                // Retract X slider before moving to the scoring position if the arm is already
                // extended to prevent getting stuck on poles.
                new ConditionalCommand(
                    new MoveElevatorXZTo(
                        elevatorsSubsystem,
                        Constants.Elevators.xMinExtend,
                        Constants.Elevators.zScoreHighConeHeight),
                    new NullCommand(),
                    armSubsystem::isCloserToScoreThanStowPosition),
                new ExtendArmToScore(elevatorsSubsystem, armSubsystem, scoringSelector),
                new IntakeStow(intakeSubsystem))));

    // Manual score
    bumperRight.onTrue(
        new SequentialCommandGroup(
            new ScoreWithGripper(gripperSubsystem, elevatorsSubsystem, scoringSelector),
            new RetractArmAfterScoring(elevatorsSubsystem, armSubsystem),
            new LowerElevatorAfterScoring(elevatorsSubsystem)));

    // Poop
    bumperLeft.onTrue(new IntakeStow(intakeSubsystem));

    // Manual re-zero
    buttonStart.whileTrue(new ZeroSelectedAxes(intakeSubsystem, elevatorsSubsystem, armSubsystem));

    // // For testing
    // buttonB.whileTrue(
    //     new SequentialCommandGroup(
    //         new DriveOntoPlatform(swerve, true),
    //         new DriveForwardsAndBalance(imu, swerve, true),
    //         new DriveBackSlightly(swerve, true),
    //         new FinalBangBangBalance(swerve, imu)));
  }

  public Command getAutonomousCommand() {
    return new FollowQuikplan(
        trajectoryReader,
        scoringSelector,
        imu,
        swerve,
        intakeSubsystem,
        elevatorsSubsystem,
        armSubsystem,
        gripperSubsystem);
  }

  /** Runs when the robot is disabled. */
  public void disabledPeriodic() {
    trajectoryReader.loadSelectedFile();
    if (Robot.isSimulation()) {
      swerve.resetSimPose(trajectoryReader.getInitialPose());
    }
  }
}
