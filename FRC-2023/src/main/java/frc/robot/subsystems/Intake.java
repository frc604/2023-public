// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.robot.Constants;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
  private final QuixTalonFX m_sliderMotor =
      new QuixTalonFX(
          Constants.Intake.Slider.sliderMotorID,
          Constants.Intake.Slider.sliderRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Intake.Slider.sliderMotorInverted)
              .setForwardSoftLimit(Constants.Intake.Slider.maxSliderExtension)
              .setReverseSoftLimit(Constants.Intake.Slider.minSliderExtension));
  private final QuixTalonFX m_armMotor =
      new QuixTalonFX(
          Constants.Intake.IntakeArm.armMotorID,
          Constants.Intake.IntakeArm.armRatio,
          QuixTalonFX.makeDefaultConfig()
              .setBrakeMode()
              .setInverted(Constants.Intake.IntakeArm.armMotorInverted)
              .setForwardSoftLimit(Constants.Intake.IntakeArm.deployArmAngle)
              .setReverseSoftLimit(Constants.Intake.IntakeArm.minArmAngle));
  private final QuixTalonFX m_reorienterMotor =
      new QuixTalonFX(
          Constants.Intake.Reorienter.reorienterMotorID,
          Constants.Intake.Reorienter.reorienterRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(
                  Constants.Intake.Reorienter.reorienterMotorInverted) // TODO: verify realness
              .setForwardSoftLimit(Constants.Intake.Reorienter.maxReorienterAngle)
              .setReverseSoftLimit(Constants.Intake.Reorienter.minReorienterAngle));
  private final QuixTalonFX m_rollerMotor =
      new QuixTalonFX(
          Constants.Intake.Roller.rollerMotorID,
          Constants.Intake.Roller.rollerRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Intake.Roller.rollerMotorInverted)); // TODO: verify realness

  private ControlMode m_sliderMotorControlMode = ControlMode.STOP;
  private ControlMode m_armMotorControlMode = ControlMode.STOP;
  private ControlMode m_reorienterMotorControlMode = ControlMode.STOP;

  private enum ControlMode {
    ZEROING,
    CLOSED_LOOP_POSITION,
    STOP,
  }

  private final DutyCycle m_intakeBeamBreakDutyCycle =
      new DutyCycle(new DigitalInput(Constants.Intake.intakeBeamBreakInputChannel));
  private final MedianFilter m_intakeDistanceFilter = new MedianFilter(3);
  private double m_filteredDistance = 0.0;

  private TrapezoidProfile m_sliderProfile;
  private State m_sliderState = new State(m_sliderMotor.getSensorPosition(), 0.0);
  private final Timer m_sliderTimer = new Timer();

  private TrapezoidProfile m_armProfile;
  private State m_armState = new State(m_armMotor.getSensorPosition(), 0.0);
  private final Timer m_armTimer = new Timer();

  private TrapezoidProfile m_reorienterProfile;
  private State m_reorienterState = new State(m_reorienterMotor.getSensorPosition(), 0.0);
  private final Timer m_reorienterTimer = new Timer();

  private final Timer m_rollerTimer = new Timer();

  private final int kSliderMotorPositionSlot = 0;
  private final int kArmMotorAngleSlot = 0;
  private final int kReorienterMotorAngleSlot = 0;

  private boolean m_isConeTipOut = false;
  private boolean m_isDistanceSensorHealthy = true;

  public Intake() {
    m_sliderMotor.setPIDConfig(kSliderMotorPositionSlot, Constants.Intake.Slider.sliderPIDConfig);
    m_armMotor.setPIDConfig(kArmMotorAngleSlot, Constants.Intake.IntakeArm.armPIDConfig);
    m_reorienterMotor.setPIDConfig(
        kReorienterMotorAngleSlot, Constants.Intake.Reorienter.reorienterPIDConfig);

    m_sliderProfile =
        new TrapezoidProfile(
            Constants.Intake.Slider.sliderTrapConstraints, new State(0.0, 0.0), m_sliderState);
    m_sliderTimer.start();

    m_armProfile =
        new TrapezoidProfile(
            Constants.Intake.IntakeArm.armTrapConstraints, new State(0.0, 0.0), m_armState);
    m_armTimer.start();

    m_reorienterProfile =
        new TrapezoidProfile(
            Constants.Intake.Reorienter.reorienterTrapConstraints,
            new State(0.0, 0.0),
            m_reorienterState);
    m_reorienterTimer.start();

    m_rollerTimer.start();

    // Show scheduler status in SmartDashboard.
    SmartDashboard.putData(this);

    // Setup viz.
    SmartDashboard.putData("Intake Viz", m_simViz);
  }

  public void moveSliderTo(double pos) {
    m_sliderMotorControlMode = ControlMode.CLOSED_LOOP_POSITION;
    m_sliderProfile =
        new TrapezoidProfile(
            Constants.Intake.Slider.sliderTrapConstraints, new State(pos, 0), m_sliderState);
    m_sliderTimer.reset();
  }

  public void moveArmTo(double pos) {
    m_armMotorControlMode = ControlMode.CLOSED_LOOP_POSITION;
    m_armProfile =
        new TrapezoidProfile(
            Constants.Intake.IntakeArm.armTrapConstraints, new State(pos, 0), m_armState);
    m_armTimer.reset();
  }

  public void moveReorienterTo(double angle) {
    m_reorienterMotorControlMode = ControlMode.CLOSED_LOOP_POSITION;
    m_reorienterProfile =
        new TrapezoidProfile(
            Constants.Intake.Reorienter.reorienterTrapConstraints,
            new State(angle, 0),
            m_reorienterState);
    m_reorienterTimer.reset();
  }

  public void stopSlider() {
    m_reorienterMotorControlMode = ControlMode.STOP;
  }

  public void stopArm() {
    m_reorienterMotorControlMode = ControlMode.STOP;
  }

  public void stopReorienter() {
    m_reorienterMotorControlMode = ControlMode.STOP;
  }

  public void zeroSlider() {
    m_reorienterMotorControlMode = ControlMode.ZEROING;
  }

  public void zeroArm() {
    m_reorienterMotorControlMode = ControlMode.ZEROING;
  }

  public void zeroReorienter() {
    m_reorienterMotorControlMode = ControlMode.ZEROING;
  }

  public void startRollerSpin() {
    m_rollerMotor.setCurrentLimit(30.0);
    m_rollerMotor.setPercentOutput(Constants.Intake.Roller.rollerIntakePower);
    m_rollerTimer.reset();
  }

  public void score() {
    m_rollerMotor.setCurrentLimit(60.0);
    m_rollerMotor.setPercentOutput(-Constants.Intake.Roller.rollerScoreLowPower);
    m_rollerTimer.reset();
  }

  public void spinRollerHandoffRelease() {
    m_rollerMotor.setCurrentLimit(40.0);
    m_rollerMotor.setPercentOutput(-Constants.Intake.Roller.rollerHandoffPower);
    m_rollerTimer.reset();
  }

  public void spinRollerSlow() {
    m_rollerMotor.setCurrentLimit(30.0);
    m_rollerMotor.setPercentOutput(Constants.Intake.Roller.rollerHoldPower);
    m_rollerTimer.reset();
  }

  public void stopRoller() {
    m_rollerMotor.setCurrentLimit(20.0);
    m_rollerMotor.setPercentOutput(0.0);
    m_rollerTimer.reset();
  }

  public boolean isSliderMotionFinished() {
    return m_sliderTimer.get() > m_sliderProfile.totalTime();
  }

  public boolean isArmMotionFinished() {
    return m_armTimer.get() > m_armProfile.totalTime();
  }

  public boolean isReorienterMotionFinished() {
    return m_reorienterTimer.get() > m_reorienterProfile.totalTime();
  }

  public boolean isRollerStalled() {
    if (Math.abs(m_rollerMotor.getSensorVelocity()) > Constants.Intake.Roller.rollerStallSpeed) {
      m_rollerTimer.reset();
      return false;
    }
    return m_rollerTimer.get() > Constants.Intake.Roller.rollerStallTime;
  }

  public boolean getIntakeBeamBreakTriggered() {
    return getIntakeBeamBreakFilteredDistance() < Units.inchesToMeters(25.0);
  }

  private double getIntakeBeamBreakDistance() {
    final double kMinDist = 0.01; // m
    final double kMaxDist = 1.0; // m
    final double kMinDutyCycle = 0.25;
    final double kMaxDutyCycle = 0.75;
    final double kDutyCycleEps = 0.01;
    final String kSensorHealthyKey = "Distance sensor healthy";

    final double dutyCycle = m_intakeBeamBreakDutyCycle.getOutput();
    if (Robot.isReal() && (dutyCycle < kMinDutyCycle || dutyCycle > kMaxDutyCycle)) {
      if (dutyCycle < kDutyCycleEps && m_isDistanceSensorHealthy) {
        // Sensor malfunction.
        System.out.println("Distance sensor malfunction: " + Timer.getFPGATimestamp());
        SmartDashboard.putBoolean(kSensorHealthyKey, false);
        m_isDistanceSensorHealthy = false;
      }
      return -1.0; // Error
    }

    if (!m_isDistanceSensorHealthy) {
      System.out.println("Distance sensor recovered: " + Timer.getFPGATimestamp());
      m_isDistanceSensorHealthy = true;
    }
    SmartDashboard.putBoolean(kSensorHealthyKey, true);
    final double fraction = (dutyCycle - kMinDutyCycle) / (kMaxDutyCycle - kMinDutyCycle);
    return kMinDist + fraction * (kMaxDist - kMinDist);
  }

  public double getIntakeBeamBreakFilteredDistance() {
    return m_filteredDistance;
  }

  public void setIsConeTipOut(boolean isConeTipOut) {
    m_isConeTipOut = isConeTipOut;
  }

  public boolean isConeTipOut() {
    return m_isConeTipOut;
  }

  public boolean isCloserToDeployThanStow() {
    final double armPos = m_armMotor.getSensorPosition();
    return Math.abs(armPos - Constants.Intake.IntakeArm.deployArmAngle)
        < Math.abs(armPos - Constants.Intake.IntakeArm.minArmAngle);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      // Update state to sensor state when disabled to prevent jumps on enable.
      m_sliderState = new State(m_sliderMotor.getSensorPosition(), 0.0);
      m_armState = new State(m_armMotor.getSensorPosition(), 0.0);
      m_reorienterState = new State(m_reorienterMotor.getSensorPosition(), 0.0);
    }

    final double rawBeamBreakDist = getIntakeBeamBreakDistance();
    if (rawBeamBreakDist > 0.0) {
      m_filteredDistance = m_intakeDistanceFilter.calculate(rawBeamBreakDist);
    }

    // Handle slider motor controls
    switch (m_sliderMotorControlMode) {
      case ZEROING:
        {
          m_sliderMotor.setPercentOutput(Constants.Intake.Slider.sliderZeroingPower);

          if (Math.abs(m_sliderMotor.getSensorVelocity())
              > Constants.Intake.Slider.sliderStallSpeed) {
            m_sliderTimer.reset();
          } else if (m_sliderTimer.get() > Constants.Intake.Slider.sliderStallTime) {
            m_sliderMotor.zeroSensorPosition();
            m_sliderMotor.setPercentOutput(0.0);
            m_sliderMotorControlMode = ControlMode.STOP;
          }
          break;
        }
      case CLOSED_LOOP_POSITION:
        {
          m_sliderState = m_sliderProfile.calculate(m_sliderTimer.get());
          m_sliderMotor.setPositionSetpoint(
              kSliderMotorPositionSlot,
              m_sliderState.position,
              Constants.Intake.Slider.sliderFF.calculate(m_sliderState.velocity));
          break;
        }
      case STOP:
      default:
        {
          m_sliderMotor.setPercentOutput(0.0);
          break;
        }
    }

    // Handle arm motor control
    switch (m_armMotorControlMode) {
      case ZEROING:
        {
          m_armMotor.setPercentOutput(Constants.Intake.IntakeArm.armZeroingPower);

          if (Math.abs(m_armMotor.getSensorVelocity()) > Constants.Intake.IntakeArm.armStallSpeed) {
            m_armTimer.reset();
          } else if (m_armTimer.get() > Constants.Intake.IntakeArm.armStallTime) {
            m_armMotor.zeroSensorPosition();
            m_armMotor.setPercentOutput(0.0);
            m_armMotorControlMode = ControlMode.STOP;
          }
          break;
        }
      case CLOSED_LOOP_POSITION:
        {
          m_armState = m_armProfile.calculate(m_armTimer.get());
          m_armMotor.setPositionSetpoint(
              kArmMotorAngleSlot,
              m_armState.position,
              Constants.Intake.IntakeArm.armFF.calculate(m_armState.position, m_armState.velocity));
          break;
        }
      case STOP:
      default:
        {
          m_armMotor.setPercentOutput(0.0);
          break;
        }
    }

    // Handle reorienter motor control
    switch (m_reorienterMotorControlMode) {
      case ZEROING:
        {
          m_reorienterMotor.setPercentOutput(Constants.Intake.Reorienter.reorienterZeroingPower);

          if (Math.abs(m_reorienterMotor.getSensorVelocity())
              > Constants.Intake.Reorienter.reorienterStallSpeed) {
            m_reorienterTimer.reset();
          } else if (m_reorienterTimer.get() > Constants.Intake.Reorienter.reorienterStallTime) {
            m_reorienterMotor.zeroSensorPosition();
            m_reorienterMotor.setPercentOutput(0.0);
            m_reorienterMotorControlMode = ControlMode.STOP;
          }
          break;
        }
      case CLOSED_LOOP_POSITION:
        {
          m_reorienterState = m_reorienterProfile.calculate(m_reorienterTimer.get());
          m_reorienterMotor.setPositionSetpoint(
              kReorienterMotorAngleSlot,
              m_reorienterState.position,
              Constants.Intake.Reorienter.reorienterFF.calculate(
                  m_reorienterState.position, m_reorienterState.velocity));
          break;
        }
      case STOP:
      default:
        {
          m_reorienterMotor.setPercentOutput(0.0);
          break;
        }
    }

    SmartDashboard.putNumber("Intake Slider: Current Position", m_sliderMotor.getSensorPosition());
    SmartDashboard.putNumber("Intake Slider: Current Velocity", m_sliderMotor.getSensorVelocity());
    SmartDashboard.putNumber("Intake Slider: Target Position", m_sliderState.position);
    SmartDashboard.putNumber("Intake Slider: Target Velocity", m_sliderState.velocity);

    SmartDashboard.putNumber("Intake Arm: Current Position", m_armMotor.getSensorPosition());
    SmartDashboard.putNumber("Intake Arm: Current Velocity", m_armMotor.getSensorVelocity());
    SmartDashboard.putNumber("Intake Arm: Target Position", m_armState.position);
    SmartDashboard.putNumber("Intake Arm: Target Velocity", m_armState.velocity);

    SmartDashboard.putNumber(
        "Intake Reorienter: Current Position", m_reorienterMotor.getSensorPosition());
    SmartDashboard.putNumber(
        "Intake Reorienter: Current Velocity", m_reorienterMotor.getSensorVelocity());
    SmartDashboard.putNumber("Intake Reorienter: Target Position", m_reorienterState.position);
    SmartDashboard.putNumber("Intake Reorienter: Target Velocity", m_reorienterState.velocity);

    SmartDashboard.putBoolean("Beam Break Triggered", getIntakeBeamBreakTriggered());
    SmartDashboard.putNumber("Beam Break Distance", getIntakeBeamBreakDistance());
    SmartDashboard.putNumber("Beam Break Filtered Distance", getIntakeBeamBreakFilteredDistance());
    SmartDashboard.putNumber("Intake Roller: Current", m_rollerMotor.getStatorCurrent());
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  // Simulation parameters
  private final ElevatorSim m_sliderSim =
      new ElevatorSim(
          DCMotor.getFalcon500(1),
          Constants.Intake.Slider.sliderRatio.reduction(),
          Constants.Intake.Slider.simSliderMass,
          Constants.Intake.Slider.sliderSprocketPitchDiameter * 0.5,
          Constants.Intake.Slider.minSliderExtension,
          Constants.Intake.Slider.maxSliderExtension,
          false);

  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(1),
          Constants.Intake.IntakeArm.armRatio.reduction(),
          SingleJointedArmSim.estimateMOI(
              Constants.Intake.IntakeArm.simArmLength, Constants.Intake.IntakeArm.simArmMass),
          Constants.Intake.IntakeArm.simArmLength,
          Constants.Intake.IntakeArm.minArmAngle,
          Constants.Intake.IntakeArm.maxArmAngle,
          true // Simulate gravity
          );

  private final SingleJointedArmSim m_reorienterSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(1),
          Constants.Intake.Reorienter.reorienterRatio.reduction(),
          SingleJointedArmSim.estimateMOI(
              Constants.Intake.Reorienter.simArmLength, Constants.Intake.Reorienter.simArmMass),
          Constants.Intake.Reorienter.simArmLength,
          Constants.Intake.Reorienter.minReorienterAngle,
          Constants.Intake.Reorienter.maxReorienterAngle,
          true // Simulate gravity
          );

  // Visualization
  private final double kSliderXOffset = 10.0;
  private final double kSliderHeight = 40.0;
  private final double kArmPivotHeight = kSliderHeight + 10.0;
  private final double kArmLength = 20.0;

  private final Mechanism2d m_simViz = new Mechanism2d(150, 100);
  private final MechanismRoot2d m_sliderRoot =
      m_simViz.getRoot("Slider Root", kSliderXOffset, kSliderHeight);
  private final MechanismLigament2d m_slider =
      m_sliderRoot.append(
          new MechanismLigament2d("Slider", 10, 0, 10, new Color8Bit(Color.kYellow)));
  private MechanismRoot2d m_armPivot = m_simViz.getRoot("Deploy Arm Pivot", 0, kArmPivotHeight);
  private MechanismLigament2d m_arm =
      m_armPivot.append(
          new MechanismLigament2d(
              "Intake Deploy Arm",
              kArmLength,
              Units.radiansToDegrees(m_armSim.getAngleRads()),
              10,
              new Color8Bit(Color.kSilver)));
  private MechanismRoot2d m_reorienterPivot =
      m_simViz.getRoot("Reorienter Pivot", 0, kArmPivotHeight);
  private MechanismLigament2d m_reorienter =
      m_reorienterPivot.append(
          new MechanismLigament2d(
              "Intake Reorienter",
              10,
              Units.radiansToDegrees(m_reorienterSim.getAngleRads()),
              5,
              new Color8Bit(Color.kRed)));

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_sliderSim.setInput(m_sliderMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_sliderSim.update(TimedRobot.kDefaultPeriod);
    m_sliderMotor.setSimSensorPositionAndVelocity(
        m_sliderSim.getPositionMeters(),
        m_sliderSim.getVelocityMetersPerSecond(),
        TimedRobot.kDefaultPeriod,
        Constants.Intake.Slider.sliderRatio);

    m_armSim.setInput(m_armMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_armSim.update(TimedRobot.kDefaultPeriod);
    m_armMotor.setSimSensorPositionAndVelocity(
        m_armSim.getAngleRads(),
        m_armSim.getVelocityRadPerSec(),
        TimedRobot.kDefaultPeriod,
        Constants.Intake.IntakeArm.armRatio);

    m_reorienterSim.setInput(
        m_reorienterMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_reorienterSim.update(TimedRobot.kDefaultPeriod);
    m_reorienterMotor.setSimSensorPositionAndVelocity(
        m_reorienterSim.getAngleRads(),
        m_reorienterSim.getVelocityRadPerSec(),
        TimedRobot.kDefaultPeriod,
        Constants.Intake.Reorienter.reorienterRatio);

    // Update slider viz.
    final double kExtensionPerMeter = 50.0 / Constants.Intake.Slider.maxSliderExtension;
    final double sliderLength =
        (Units.inchesToMeters(12.0) + m_sliderSim.getPositionMeters()) * kExtensionPerMeter;
    m_slider.setLength(sliderLength);

    // Update the arm viz.
    m_armPivot.setPosition(kSliderXOffset + sliderLength, kArmPivotHeight);
    final double armAngle = Math.PI - m_armSim.getAngleRads();
    m_arm.setAngle(Math.toDegrees(armAngle));

    // Update the reorienter viz.
    m_reorienterPivot.setPosition(
        kSliderXOffset + sliderLength + kArmLength * Math.cos(armAngle),
        kArmPivotHeight + kArmLength * Math.sin(armAngle));
    final double reorienterAngle = m_reorienterSim.getAngleRads();
    m_reorienter.setAngle(Math.toDegrees(-0.5 * Math.PI + armAngle + reorienterAngle));
  }
  // --- END STUFF FOR SIMULATION ---
}
