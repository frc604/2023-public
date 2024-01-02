// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.robot.ArmLookup;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private final QuixTalonFX m_armMotor =
      new QuixTalonFX(
          Constants.Arm.armMotorID,
          Constants.Arm.armMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Arm.armMotorInverted)
              .setBrakeMode()
              .setSupplyCurrentLimit(20.0)
              .setStatorCurrentLimit(30.0)
              .setForwardSoftLimit(Constants.Arm.maxArmAngle));

  private static final int kArmPositionSlot = 0;

  private TrapezoidProfile m_armProfile;
  private State m_armState = new State(m_armMotor.getSensorPosition(), 0.0);
  private final Timer m_armTrapTimer = new Timer();

  public Arm() {
    m_armMotor.setPIDConfig(kArmPositionSlot, Constants.Arm.armPIDConfig);

    m_armTrapTimer.start();
    // Show scheduler status in SmartDashboard.
    SmartDashboard.putData(this);
  }

  private ControlMode m_armMotorControlMode = ControlMode.STOP;

  private enum ControlMode {
    ZEROING,
    CLOSED_LOOP_POSITION,
    STOP,
  }

  public boolean isArmMotionFinished() {
    return m_armTrapTimer.get() > m_armProfile.totalTime();
  }

  public boolean isArmMotionFinishedForCube() {
    // We don't care about the arm position as much for cubes, so we can be sloppy with when the arm
    // is finished.
    final double kCloseEnoughTimeBuffer = 0.5; // s
    return m_armTrapTimer.get() + kCloseEnoughTimeBuffer > m_armProfile.totalTime();
  }

  public void moveArmTo(double pos) {
    m_armMotorControlMode = ControlMode.CLOSED_LOOP_POSITION;
    m_armProfile =
        new TrapezoidProfile(Constants.Arm.armTrapConstraints, new State(pos, 0), m_armState);
    m_armTrapTimer.reset();
  }

  public void moveArmToSlow(double pos) {
    m_armMotorControlMode = ControlMode.CLOSED_LOOP_POSITION;
    m_armProfile =
        new TrapezoidProfile(Constants.Arm.armSlowTrapConstraints, new State(pos, 0), m_armState);
    m_armTrapTimer.reset();
  }

  public void stopArm() {
    m_armMotorControlMode = ControlMode.STOP;
  }

  public void zeroArm() {
    m_armMotor.setConfiguration();
    m_armMotor.setPIDConfig(kArmPositionSlot, Constants.Arm.armPIDConfig);
    m_armMotorControlMode = ControlMode.ZEROING;
  }

  public void zeroSensorPosition() {
    m_armMotor.setSensorPosition(Math.toRadians(0.0));
  }

  public void increaseCurrentLimit() {
    m_armMotor.setCurrentLimit(40.0);
  }

  public void decreaseCurrentLimit() {
    m_armMotor.setCurrentLimit(30.0);
  }

  public boolean isCloserToScoreThanStowPosition() {
    final double armPos = m_armMotor.getSensorPosition();
    return Math.abs(armPos - Constants.Arm.maxArmAngle)
        < Math.abs(armPos - Constants.Arm.minArmAngle);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      // Update state to sensor state when disabled to prevent jumps on enable.
      m_armState = new State(m_armMotor.getSensorPosition(), 0.0);
      m_armMotorControlMode = ControlMode.STOP;
    }

    // This method will be called once per scheduler run
    switch (m_armMotorControlMode) {
      case ZEROING:
        {
          m_armMotor.setPercentOutput(Constants.Arm.armZeroingPower);
          break;
        }
      case CLOSED_LOOP_POSITION:
        {
          m_armState = m_armProfile.calculate(m_armTrapTimer.get());
          final double gravityFF = Constants.Arm.kG * ArmLookup.dydtheta.get(m_armState.position);
          final double velocityFF = Constants.Arm.kV * m_armState.velocity;
          m_armMotor.setPositionSetpoint(
              kArmPositionSlot, m_armState.position, gravityFF + velocityFF);
          break;
        }
      case STOP:
      default:
        {
          m_armMotor.setPercentOutput(0.0);
          break;
        }
    }
    SmartDashboard.putNumber("Arm Motor: Current Position", m_armMotor.getSensorPosition());
    SmartDashboard.putNumber("Arm Motor: Current Velocity", m_armMotor.getSensorVelocity());
    SmartDashboard.putNumber("Arm Motor: Target Position", m_armState.position);
    SmartDashboard.putNumber("Arm Motor: Target Velocity", m_armState.velocity);
    SmartDashboard.putNumber("Arm Motor: Current (Amps)", m_armMotor.getStatorCurrent());
    SmartDashboard.putNumber("Arm Motor: Power", m_armMotor.getPercentOutput());
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(1),
          Constants.Arm.armMotorRatio.reduction(),
          SingleJointedArmSim.estimateMOI(Constants.Arm.simArmLength, Constants.Arm.simArmMass),
          Constants.Arm.simArmLength,
          Constants.Arm.simMinArmAngle,
          Constants.Arm.simMaxArmAngle,
          true // Simulate gravity
          );

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_armSim.setInput(m_armMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_armSim.update(TimedRobot.kDefaultPeriod);
    m_armMotor.setSimSensorPositionAndVelocity(
        m_armSim.getAngleRads(),
        m_armSim.getVelocityRadPerSec(),
        TimedRobot.kDefaultPeriod,
        Constants.Arm.armMotorRatio);
  }
  // --- END STUFF FOR SIMULATION ---
}
