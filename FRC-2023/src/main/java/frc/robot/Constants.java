package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.quixlib.devices.CANDeviceID;
import frc.quixlib.math.MathUtils;
import frc.quixlib.motorcontrol.MechanismRatio;
import frc.quixlib.motorcontrol.PIDConfig;
import frc.quixlib.swerve.QuixSwerveController;
import frc.quixlib.vision.Fiducial;
import frc.quixlib.vision.PipelineConfig;

public class Constants {
  public static final String kCanivoreName = "canivore";
  public static final CANDeviceID pigeonID = new CANDeviceID(0);

  public static final class Intake {
    public static final int intakeBeamBreakInputChannel = 0;

    public static final class Slider {
      public static final CANDeviceID sliderMotorID = new CANDeviceID(8, kCanivoreName);
      public static final boolean sliderMotorInverted = false;

      public static final double sliderSprocketPitchDiameter =
          Units.inchesToMeters(1.432); // 18t #25
      public static final double sliderSprocketCircumference =
          Math.PI * sliderSprocketPitchDiameter;
      public static final MechanismRatio sliderRatio =
          new MechanismRatio(12.0, 48.0, sliderSprocketCircumference);

      public static final double minSliderExtension = Units.inchesToMeters(0.5); // half inch buffer
      public static final double maxSliderExtension =
          Units.inchesToMeters(18.375); // half inch buffer
      public static final double stowSliderExtension = Units.inchesToMeters(2.0);
      public static final double detectConeSliderExtension = Units.inchesToMeters(16.5);

      // For measuring
      public static final double coneOutSliderMeasureExtension = Units.inchesToMeters(3.0);
      public static final double coneInSliderMeasureExtension = Units.inchesToMeters(10.0);

      // For handoff
      public static final double coneOutSliderHandoffExtension = Units.inchesToMeters(5.0);
      public static final double coneInSliderHandoffExtension = Units.inchesToMeters(10.0);

      // Handoff clearance
      public static final double coneOutSliderHandoffClearanceExtension = Units.inchesToMeters(3.0);
      public static final double coneInSliderHandoffClearanceExtension = Units.inchesToMeters(12.0);

      public static final double scoreLowSliderExtension = Units.inchesToMeters(8.0);

      public static final double sliderZeroingPower = -0.05;
      public static final double sliderStallSpeed = 0.1; // m/s
      public static final double sliderStallTime = 0.1; // s

      public static final ElevatorFeedforward sliderFF = new ElevatorFeedforward(0, 0.0, 5.0, 0.0);
      public static final PIDConfig sliderPIDConfig = new PIDConfig(0.1, 0.0, 0.0);
      public static final Constraints sliderTrapConstraints = new Constraints(1.0, 3.0);

      public static final double simSliderMass = Units.lbsToKilograms(21.0);
    }

    public static final class IntakeArm {
      public static final CANDeviceID armMotorID = new CANDeviceID(9, kCanivoreName);
      public static final boolean armMotorInverted = true;

      public static final MechanismRatio armRatio = new MechanismRatio(1.0, 32.0);

      public static final double minArmAngle = 0.0;
      public static final double maxArmAngle = Math.toRadians(210.0);
      public static final double deployArmAngle = Math.toRadians(200.0);
      public static final double scoreLowArmAngle = Math.toRadians(150.0);
      public static final double armZeroingPower = -0.05;
      public static final double armStallSpeed = 0.25 * Math.PI; // rad/s TODO: make real
      public static final double armStallTime = 0.1; // s

      public static final ArmFeedforward armFF = new ArmFeedforward(0, 0.3, 0.55);
      public static final PIDConfig armPIDConfig = new PIDConfig(0.1, 0.0, 0.0);
      public static final Constraints armTrapConstraints =
          new Constraints(2 * Math.PI, 10 * Math.PI);

      public static final double simArmLength = Units.inchesToMeters(4.0);
      public static final double simArmMass = Units.lbsToKilograms(12.0);
    }

    public static final class Reorienter {
      public static final CANDeviceID reorienterMotorID = new CANDeviceID(10, kCanivoreName);
      public static final boolean reorienterMotorInverted = true;

      public static final MechanismRatio reorienterRatio = new MechanismRatio(1.0, 32.0);

      public static final double minReorienterAngle = Math.toRadians(0.0);
      public static final double maxReorienterAngle = Math.toRadians(300.0);

      public static final double deployReorienterAngle = Math.toRadians(150);
      public static final double detectConeReorienterAngle = Math.toRadians(145);
      public static final double coneOutReorienterAngle = Math.toRadians(10.0);
      public static final double coneInReorienterAngle = Math.toRadians(170.0);
      public static final double scoreLowReorienterAngle = Math.toRadians(180);

      public static final double reorienterZeroingPower = -0.05;
      public static final double reorienterStallSpeed = 0.25 * Math.PI; // rad/s TODO: make real
      public static final double reorienterStallTime = 0.1; // s

      public static final ArmFeedforward reorienterFF = new ArmFeedforward(0, 0, 0.55);
      public static final PIDConfig reorienterPIDConfig = new PIDConfig(0.1, 0.0, 0.0);

      public static final Constraints reorienterTrapConstraints =
          new Constraints(4 * Math.PI, 16 * Math.PI);

      public static final double simArmLength = Units.inchesToMeters(2.0);
      public static final double simArmMass = Units.lbsToKilograms(4.0);
    }

    public static final class Roller {
      public static final CANDeviceID rollerMotorID = new CANDeviceID(11, kCanivoreName);
      public static final boolean rollerMotorInverted = true;

      public static final MechanismRatio rollerRatio = new MechanismRatio(8.0, 40.0);

      public static final double rollerIntakePower = 0.50;
      public static final double rollerScoreLowPower = 0.80;
      public static final double rollerHandoffPower = 0.20;
      public static final double rollerScorePower = 1.00;
      public static final double rollerHoldPower = 0.50;

      public static final double rollerStallSpeed = Math.PI * 0.125; // rad/s
      public static final double rollerStallTime = 0.1; // seconds
    }
  }

  public static final class Elevators {
    // Z: Up/Down
    public static final CANDeviceID zMotorID = new CANDeviceID(12, kCanivoreName);
    public static final boolean zMotorInverted = true;

    public static final double zSprocketPitchDiameter = Units.inchesToMeters(1.432); // 18T #25
    public static final double zSprocketCircumference = Math.PI * zSprocketPitchDiameter;
    public static final MechanismRatio zMotorRatio =
        new MechanismRatio(1.0, 16.0, zSprocketCircumference);

    public static final double zMinHeight = Units.inchesToMeters(0.0); // m
    public static final double zMaxHeight = Units.inchesToMeters(24.0); // m
    public static final double zIntakeClearanceHeight = Units.inchesToMeters(10.0); // m
    public static final double zScoreLowClearanceHeight = Units.inchesToMeters(16.0); // m
    public static final double zPrepareHandoffHeight = Units.inchesToMeters(10.0); // m
    public static final double zHandoffHeight = Units.inchesToMeters(0.0); // m
    public static final double zAfterHandoffHeight = Units.inchesToMeters(6.0); // m
    public static final double zScoreMidConeHeight = Units.inchesToMeters(4.0); // m
    public static final double zScoreMidCubeHeight = Units.inchesToMeters(8.0); // m
    public static final double zScoreHighConeHeight = Units.inchesToMeters(17.0); // m
    public static final double zScoreHighCubeHeight = Units.inchesToMeters(20.0); // m
    public static final double zAfterScoreConeOffset = Units.inchesToMeters(1.5);
    public static final double zZeroingPower = -0.05;
    public static final double zStallSpeed = 0.1; // m/s
    public static final double zStallTime = 0.1; // s

    public static final ElevatorFeedforward zFF = new ElevatorFeedforward(0.25, 0.1, 15.8);
    public static final PIDConfig zPositionPIDConfig = new PIDConfig(0.01, 0.0, 0.0);
    public static final double zTrapConstraintsAccl = 3.0; // m/s/s
    public static final Constraints zTrapConstraints = new Constraints(1.0, zTrapConstraintsAccl);
    public static final Constraints zSlowTrapConstraints =
        new Constraints(0.75, zTrapConstraintsAccl);

    public static final double zSimCarriageMass = Units.lbsToKilograms(13.0); // kg

    // X: Extend/Retract
    public static final CANDeviceID xMotorID = new CANDeviceID(13, kCanivoreName);
    public static final boolean xMotorInverted = true;

    public static final double xSprocketPitchDiameter = Units.inchesToMeters(1.592); // 20T #25
    public static final double xSprocketCircumference = Math.PI * xSprocketPitchDiameter;
    public static final MechanismRatio xMotorRatio =
        new MechanismRatio(12.0, 36.0, xSprocketCircumference); // kg

    public static final double xMinExtend = Units.inchesToMeters(1.0); // m
    public static final double xMaxExtend = Units.inchesToMeters(16.0); // m
    public static final double xMidPos = 0.5 * (xMinExtend + xMaxExtend); // m
    public static final double xHandoffPosOffset = Units.inchesToMeters(-7.0); // m
    public static final double xScoreMidExtend = Units.inchesToMeters(1.0); // m
    public static final double xScoreHighExtend = Units.inchesToMeters(16.0); // m
    public static final double xScoreHighExtendAuto = Units.inchesToMeters(14.0); // m

    public static final double xZeroingPower = -0.05;
    public static final double xStallSpeed = 0.1; // m/s
    public static final double xStallTime = 0.1; // s
    public static final double xHandoffBuffer = Units.inchesToMeters(1);

    public static final ElevatorFeedforward xFF = new ElevatorFeedforward(0.0, 0.0, 3.0);
    public static final PIDConfig xPositionPIDConfig = new PIDConfig(0.1, 0.0, 0.0);
    public static final Constraints xTrapConstraints = new Constraints(1.0, 3.0);

    public static final double xSimCarriageMass = Units.lbsToKilograms(5.0);
  }

  public static final class Arm {
    public static final CANDeviceID armMotorID = new CANDeviceID(14, kCanivoreName);
    public static final boolean armMotorInverted = false;

    public static final double armZeroingPower = -0.15;

    public static final MechanismRatio armMotorRatio = new MechanismRatio(1.0, 192.0);

    // Handoff even more negative than min angle to make sure the arm is not floppy.
    public static final double minArmAngle = Math.toRadians(-15.0); // rad
    public static final double maxArmAngle = Math.toRadians(55.0); // rad
    public static final double stowArmAngle = Math.toRadians(0.0); // rad

    public static final double kG = 0.25;
    public static final double kV = 4.0;
    public static final PIDConfig armPIDConfig = new PIDConfig(0.1, 0.0, 0.0);
    public static final double armTrapConstraintsAccl = 0.75 * Math.PI; // rad/s/s
    public static final Constraints armTrapConstraints =
        new Constraints(0.4 * Math.PI, armTrapConstraintsAccl);
    public static final Constraints armSlowTrapConstraints =
        new Constraints(0.3 * Math.PI, armTrapConstraintsAccl);

    // The real arm has much more complicatedsimMaxArmAngle dynamics. These are just some numbers
    // that work in sim.
    public static final double simArmLength = Units.inchesToMeters(24.0);
    public static final double simArmMass = Units.lbsToKilograms(1.0);
    // Simulate hard stops
    public static final double simMinArmAngle = minArmAngle + Math.toRadians(5.0);
    public static final double simMaxArmAngle = maxArmAngle - Math.toRadians(5.0);
  }

  public static final class Gripper {
    public static final CANDeviceID gripperMotor = new CANDeviceID(15, kCanivoreName);
    public static final boolean gripperMotorInvert = true;

    public static final MechanismRatio rollerRatio = new MechanismRatio(12.0, 36.0);

    public static final double holdPower = 0.5;
    public static final double handoffPower = 0.5;
    public static final double scorePower = 1.0;

    public static final double gripperScoreConeRunTime = 1.0; // s
    public static final double gripperScoreCubeRunTime = 0.75; // s

    // Approximations for simulation.
    public static final double rollerMOI = 0.005; // Moment of inertia (kg * m^2)
  }

  public static final class LEDs {
    public static final int ledPort = 0; // TODO: make real
    public static final int ledCount = 10; // Amount of leds used
  }

  public static final class Cameras {
    public static final class LeftCam {
      public static final double fovWidth = Math.toRadians(70); // rad
      public static final double fovHeight = Math.toRadians(47.5); // rad
      public static final Matrix<N4, N4> robotToCameraT =
          MathUtils.makeZRotTFMatrix(
              Units.inchesToMeters(13.70),
              Units.inchesToMeters(13.06),
              Units.inchesToMeters(12.14),
              Units.degreesToRadians(40.0));
      public static final PipelineConfig[] pipelineConfigs =
          new PipelineConfig[] {
            new PipelineConfig(Fiducial.Type.APRILTAG, 1280, 800),
          };
    }

    public static final class RightCam {
      public static final double fovWidth = Math.toRadians(70); // rad
      public static final double fovHeight = Math.toRadians(47.5); // rad
      public static final Matrix<N4, N4> robotToCameraT =
          MathUtils.makeZRotTFMatrix(
              Units.inchesToMeters(13.70),
              Units.inchesToMeters(-13.06),
              Units.inchesToMeters(12.14),
              Units.degreesToRadians(-40.0));
      public static final PipelineConfig[] pipelineConfigs =
          new PipelineConfig[] {
            new PipelineConfig(Fiducial.Type.APRILTAG, 1280, 800),
          };
    }
  }

  public static final class Swerve {
    public static final double maxDriveSpeed = 4.5; // m/s
    public static final double maxDriveAcceleration = 8.0; // m/s/s
    public static final double maxAngularVelocity = Math.PI * 2.0; // rad/s
    public static final double maxAngularAcceleration = Math.PI * 4.0; // rad/s/s
    public static final double trackWidth = Units.inchesToMeters(23.75);
    public static final double wheelBase = Units.inchesToMeters(23.75);
    public static final double wheelDiameter = Units.inchesToMeters(3.91);
    public static final double wheelCircumference = wheelDiameter * Math.PI;
    public static final MechanismRatio driveRatio =
        new MechanismRatio(1.0, 6.75, wheelCircumference);
    public static final MechanismRatio steeringRatio = new MechanismRatio(1.0, 150.0 / 7.0);
    public static final PIDConfig drivePIDConfig = new PIDConfig(0.2, 0.0, 0.0);
    public static final SimpleMotorFeedforward driveFeedforward =
        new SimpleMotorFeedforward(0.2, 2.4, 0.16343);
    public static final PIDConfig steeringPIDConfig = new PIDConfig(0.2, 0.0, 0.0);

    /* Swerve Teleop Slew Rate Limits */
    public static final double linearSlewRate = 10.0; // m/s/s
    public static final double angularSlewRate = 20.0; // rad/s/s
    public static final double stickDeadband = 0.15;

    /* Module Slew Rate Limits */
    // Max module acceleration can be high since drive wheels can be backdriven.
    public static final double maxModuleAcceleration = 1000.0; // m/s/s
    public static final double maxModuleSteeringRate = 4.0 * Math.PI; // rad/s/s

    /* Allowable scrub */
    public static final double autoScrubLimit = 0.25; // m/s
    public static final double teleopScrubLimit = 0.25; // m/s

    public static final QuixSwerveController driveController =
        new QuixSwerveController(
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(2.0, 0.0, 0.5));

    /* For balancing on charging station */
    public static final double considerOvertippedPitch = Math.toRadians(5.0); // rad
    public static final double pitchRateThreshold = 0.12; // rad/s

    /* Front Left Module - Module 0 */
    public static final class FrontLeft {
      public static final CANDeviceID driveMotorID = new CANDeviceID(0);
      public static final CANDeviceID steeringMotorID = new CANDeviceID(1);
      public static final CANDeviceID canCoderID = new CANDeviceID(0);
      public static final Translation2d modulePosition =
          new Translation2d(wheelBase / 2.0, trackWidth / 2.0);
      public static final double absEncoderOffsetRad = Math.toRadians(276.328);
    }

    /* Rear Left Module - Module 1 */
    public static final class RearLeft {
      public static final CANDeviceID driveMotorID = new CANDeviceID(2);
      public static final CANDeviceID steeringMotorID = new CANDeviceID(3);
      public static final CANDeviceID canCoderID = new CANDeviceID(1);
      public static final Translation2d modulePosition =
          new Translation2d(-wheelBase / 2.0, trackWidth / 2.0);
      public static final double absEncoderOffsetRad = Math.toRadians(7.295);
    }

    /* Rear Right Module - Module 2 */
    public static final class RearRight {
      public static final CANDeviceID driveMotorID = new CANDeviceID(4);
      public static final CANDeviceID steeringMotorID = new CANDeviceID(5);
      public static final CANDeviceID canCoderID = new CANDeviceID(2);
      public static final Translation2d modulePosition =
          new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0);
      public static final double absEncoderOffsetRad = Math.toRadians(63.721);
    }

    /* Front Right Module - Module 3 */
    public static final class FrontRight {
      public static final CANDeviceID driveMotorID = new CANDeviceID(6);
      public static final CANDeviceID steeringMotorID = new CANDeviceID(7);
      public static final CANDeviceID canCoderID = new CANDeviceID(3);
      public static final Translation2d modulePosition =
          new Translation2d(wheelBase / 2.0, -trackWidth / 2.0);
      public static final double absEncoderOffsetRad = Math.toRadians(79.541);
    }
  }

  // Placeholder values
  public static final class Example {
    public static final CANDeviceID motorID = new CANDeviceID(0);
    public static final MechanismRatio motorRatio = new MechanismRatio(0, 0, 0);
  }
}
