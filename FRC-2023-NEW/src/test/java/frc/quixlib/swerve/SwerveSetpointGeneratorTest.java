package frc.quixlib.swerve;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import org.junit.jupiter.api.*;

public class SwerveSetpointGeneratorTest {
  // Based on 254's test.
  private static final double kEps = 1e-5;

  private static final double kRobotSide = 0.5; // m
  private static final Translation2d[] kModulePositions = { // Front left
    new Translation2d(kRobotSide / 2.0, kRobotSide / 2.0),
    // Front right
    new Translation2d(kRobotSide / 2.0, -kRobotSide / 2.0),
    // Back left
    new Translation2d(-kRobotSide / 2.0, kRobotSide / 2.0),
    // Back right
    new Translation2d(-kRobotSide / 2.0, -kRobotSide / 2.0)
  };

  private static final SwerveDriveKinematics kKinematics =
      new SwerveDriveKinematics(kModulePositions);
  private static final double kMaxDriveVelocity = 5.0; // m/s
  private static final double kMaxDriveAcceleration = 10.0; // m/s^2
  private static final double kMaxSteeringVelocity = Math.toRadians(1500.0); // rad/s

  private static final SwerveSetpointGenerator.KinematicLimits kKinematicLimits =
      new SwerveSetpointGenerator.KinematicLimits();

  static {
    kKinematicLimits.kMaxDriveVelocity = kMaxDriveVelocity;
    kKinematicLimits.kMaxDriveAcceleration = kMaxDriveAcceleration;
    kKinematicLimits.kMaxSteeringVelocity = kMaxSteeringVelocity;
  }

  private static final double kMaxSteeringVelocityError = Math.toRadians(2.0); // rad/s
  private static final double kMaxAccelerationError = 0.1; // m/s^2

  public void SatisfiesConstraints(SwerveSetpoint prev, SwerveSetpoint next) {
    for (int i = 0; i < prev.moduleStates.length; ++i) {
      final var prevModule = prev.moduleStates[i];
      final var nextModule = next.moduleStates[i];
      double diffRotation = nextModule.angle.getRadians() - prevModule.angle.getRadians();
      assertTrue(Math.abs(diffRotation) < kMaxSteeringVelocity + kMaxSteeringVelocityError);
      assertTrue(Math.abs(nextModule.speedMetersPerSecond) <= kMaxDriveVelocity);
      assertTrue(
          Math.abs(nextModule.speedMetersPerSecond - prevModule.speedMetersPerSecond)
                  / TimedRobot.kDefaultPeriod
              <= kMaxDriveAcceleration + kMaxAccelerationError);
    }
  }

  public SwerveSetpoint driveToGoal(
      SwerveSetpoint prevSetpoint, ChassisSpeeds goal, SwerveSetpointGenerator generator) {
    // System.out.println("Driving to goal state " + goal);
    //     System.out.println("Initial state: " + prevSetpoint);
    while (Math.abs(prevSetpoint.chassisSpeeds.vxMetersPerSecond - goal.vxMetersPerSecond) > kEps
        || Math.abs(prevSetpoint.chassisSpeeds.vyMetersPerSecond - goal.vyMetersPerSecond) > kEps
        || Math.abs(prevSetpoint.chassisSpeeds.omegaRadiansPerSecond - goal.omegaRadiansPerSecond)
            > kEps) {
      var newsetpoint =
          generator.generateSetpoint(
              kKinematicLimits, prevSetpoint, goal, TimedRobot.kDefaultPeriod);
      // System.out.println(newsetpoint);
      SatisfiesConstraints(prevSetpoint, newsetpoint);
      prevSetpoint = newsetpoint;
    }
    return prevSetpoint;
  }

  @Test
  public void testGenerateSetpoint() {
    SwerveModuleState[] initialStates = {
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState()
    };
    SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), initialStates);

    var generator = new SwerveSetpointGenerator(kKinematics, kModulePositions);

    var goalSpeeds = new ChassisSpeeds(0.0, 0.0, 1.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator);

    goalSpeeds = new ChassisSpeeds(0.0, 0.0, -1.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator);

    goalSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator);

    goalSpeeds = new ChassisSpeeds(1.0, 0.0, 0.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator);

    goalSpeeds = new ChassisSpeeds(0.0, 1.0, 0.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator);

    goalSpeeds = new ChassisSpeeds(0.1, -1.0, 0.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator);

    goalSpeeds = new ChassisSpeeds(1.0, -0.5, 0.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator);

    goalSpeeds = new ChassisSpeeds(1.0, 0.4, 0.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator);
  }
}
