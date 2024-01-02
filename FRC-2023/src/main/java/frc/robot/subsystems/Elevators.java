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
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.robot.Constants;

public class Elevators extends SubsystemBase {
  private final QuixTalonFX m_zMotor =
      new QuixTalonFX(
          Constants.Elevators.zMotorID,
          Constants.Elevators.zMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setBrakeMode()
              .setInverted(Constants.Elevators.zMotorInverted)
              .setForwardSoftLimit(Constants.Elevators.zMaxHeight)
              .setReverseSoftLimit(Constants.Elevators.zMinHeight));
  private ControlMode m_zMotorControlMode = ControlMode.STOP;

  private final QuixTalonFX m_xMotor =
      new QuixTalonFX(
          Constants.Elevators.xMotorID,
          Constants.Elevators.xMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setBrakeMode()
              .setInverted(Constants.Elevators.xMotorInverted)
              .setSupplyCurrentLimit(20.0)
              .setStatorCurrentLimit(40.0)
              .setForwardSoftLimit(Constants.Elevators.xMaxExtend)
              .setReverseSoftLimit(Constants.Elevators.xMinExtend));
  private ControlMode m_xMotorControlMode = ControlMode.STOP;

  private static final int kZPositionSlot = 0;
  private static final int kXPositionSlot = 0;

  private TrapezoidProfile m_zProfile;
  private TrapezoidProfile m_xProfile;

  private final Timer m_zTraptimer = new Timer();
  private final Timer m_xTrapTimer = new Timer();
  private final Timer m_zZeroTimer = new Timer();
  private final Timer m_xZeroTimer = new Timer();

  private State m_zState = new State(m_zMotor.getSensorPosition(), 0.0);
  private State m_xState = new State(m_xMotor.getSensorPosition(), 0.0);

  private enum ControlMode {
    ZEROING,
    CLOSED_LOOP_POSITION,
    STOP,
  }

  public Elevators() {
    m_zMotor.setPIDConfig(kZPositionSlot, Constants.Elevators.zPositionPIDConfig);
    m_xMotor.setPIDConfig(kXPositionSlot, Constants.Elevators.xPositionPIDConfig);

    m_zProfile =
        new TrapezoidProfile(Constants.Elevators.zTrapConstraints, new State(0.0, 0.0), m_zState);
    m_zTraptimer.start();

    m_xProfile =
        new TrapezoidProfile(Constants.Elevators.xTrapConstraints, new State(0.0, 0.0), m_xState);
    m_xTrapTimer.start();

    m_zZeroTimer.start();
    m_xZeroTimer.start();

    // Show scheduler status in SmartDashboard.
    SmartDashboard.putData(this);

    // Setup viz.
    SmartDashboard.putData("Elevator Viz", m_simViz);
  }

  public void moveZToPos(double pos) {
    moveZToPos(pos, false);
  }

  public void moveZToPos(double pos, boolean slow) {
    m_zMotorControlMode = ControlMode.CLOSED_LOOP_POSITION;
    m_zProfile =
        new TrapezoidProfile(
            slow ? Constants.Elevators.zSlowTrapConstraints : Constants.Elevators.zTrapConstraints,
            new State(pos, 0.0),
            m_zState);
    m_zTraptimer.reset();
  }

  public void moveXToPos(double pos) {
    m_xMotorControlMode = ControlMode.CLOSED_LOOP_POSITION;
    m_xProfile =
        new TrapezoidProfile(Constants.Elevators.xTrapConstraints, new State(pos, 0.0), m_xState);
    m_xTrapTimer.reset();
  }

  public boolean isZMotionFinished() {
    return m_zTraptimer.get() > m_zProfile.totalTime();
  }

  public boolean isXMotionFinished() {
    return m_xTrapTimer.get() > m_xProfile.totalTime();
  }

  public void stopZ() {
    m_zMotorControlMode = ControlMode.STOP;
  }

  public void stopX() {
    m_xMotorControlMode = ControlMode.STOP;
  }

  public void zeroZ() {
    m_zZeroTimer.reset();
    m_zMotorControlMode = ControlMode.ZEROING;
  }

  public void zeroX() {
    m_xZeroTimer.reset();
    m_xMotorControlMode = ControlMode.ZEROING;
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      // Update state to sensor state when disabled to prevent jumps on enable.
      m_zState = new State(m_zMotor.getSensorPosition(), 0.0);
      m_xState = new State(m_xMotor.getSensorPosition(), 0.0);
    }

    // Handle Z motor control
    switch (m_zMotorControlMode) {
      case ZEROING:
        {
          m_zMotor.setPercentOutput(Constants.Elevators.zZeroingPower);

          if (Math.abs(m_zMotor.getSensorVelocity()) > Constants.Elevators.zStallSpeed) {
            m_zZeroTimer.reset();
          } else if (m_zZeroTimer.get() > Constants.Elevators.zStallTime) {
            m_zMotor.zeroSensorPosition();
            m_zMotor.setPercentOutput(0.0);
            m_zMotorControlMode = ControlMode.STOP;
          }
          break;
        }
      case CLOSED_LOOP_POSITION:
        {
          m_zState = m_zProfile.calculate(m_zTraptimer.get());
          m_zMotor.setPositionSetpoint(
              kZPositionSlot,
              m_zState.position,
              Constants.Elevators.zFF.calculate(m_zState.velocity));
          break;
        }
      case STOP:
      default:
        {
          m_zMotor.setPercentOutput(0.0);
          break;
        }
    }

    // Handle X motor control
    switch (m_xMotorControlMode) {
      case ZEROING:
        {
          m_xMotor.setPercentOutput(Constants.Elevators.xZeroingPower);

          if (Math.abs(m_xMotor.getSensorVelocity()) > Constants.Elevators.xStallSpeed) {
            m_xZeroTimer.reset();
          } else if (m_xZeroTimer.get() > Constants.Elevators.xStallTime) {
            m_xMotor.zeroSensorPosition();
            m_xMotor.setPercentOutput(0.0);
            m_xMotorControlMode = ControlMode.STOP;
          }
          break;
        }
      case CLOSED_LOOP_POSITION:
        {
          m_xState = m_xProfile.calculate(m_xTrapTimer.get());
          m_xMotor.setPositionSetpoint(
              kXPositionSlot,
              m_xState.position,
              Constants.Elevators.xFF.calculate(m_xState.velocity));
          break;
        }
      case STOP:
      default:
        {
          m_xMotor.setPercentOutput(0.0);
          break;
        }
    }

    SmartDashboard.putNumber("Z Motor: Stator Current", m_zMotor.getStatorCurrent());
    SmartDashboard.putNumber("Z Motor: Current Position", m_zMotor.getSensorPosition());
    SmartDashboard.putNumber("Z Motor: Current Velocity", m_zMotor.getSensorVelocity());
    SmartDashboard.putNumber("Z Motor: Target Position", m_zState.position);
    SmartDashboard.putNumber("Z Motor: Target Velocity", m_zState.velocity);

    SmartDashboard.putNumber("X Motor: Current Position", m_xMotor.getSensorPosition());
    SmartDashboard.putNumber("X Motor: Current Velocity", m_xMotor.getSensorVelocity());
    SmartDashboard.putNumber("X Motor: Target Position", m_xState.position);
    SmartDashboard.putNumber("X Motor: Target Velocity", m_xState.velocity);
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  // Simulation parameters
  private final ElevatorSim m_zSim =
      new ElevatorSim(
          DCMotor.getFalcon500(1),
          Constants.Elevators.zMotorRatio.reduction(),
          Constants.Elevators.zSimCarriageMass,
          Constants.Elevators.zSprocketPitchDiameter * 0.5,
          Constants.Elevators.zMinHeight,
          Constants.Elevators.zMaxHeight,
          true);
  private final ElevatorSim m_xSim =
      new ElevatorSim(
          DCMotor.getFalcon500(1),
          Constants.Elevators.xMotorRatio.reduction(),
          Constants.Elevators.xSimCarriageMass,
          Constants.Elevators.xSprocketPitchDiameter * 0.5,
          Constants.Elevators.xMinExtend,
          Constants.Elevators.xMaxExtend,
          false);

  // Visualization
  private final Mechanism2d m_simViz = new Mechanism2d(100, 100);

  private MechanismRoot2d m_zRoot = m_simViz.getRoot("Z Root", 10, 10);
  private MechanismLigament2d m_z =
      m_zRoot.append(new MechanismLigament2d("Z", 80, 0, 10, new Color8Bit(Color.kYellow)));

  private MechanismRoot2d m_xRoot = m_simViz.getRoot("X Root", 20, 10);
  private MechanismLigament2d m_x =
      m_xRoot.append(new MechanismLigament2d("Z", 10, 0, 10, new Color8Bit(Color.kRed)));

  // --- BEGIN STUFF FOR SIMULATION ---
  @Override
  public void simulationPeriodic() {
    m_xSim.setInput(m_xMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_xSim.update(TimedRobot.kDefaultPeriod);
    m_xMotor.setSimSensorPositionAndVelocity(
        m_xSim.getPositionMeters(),
        m_xSim.getVelocityMetersPerSecond(),
        TimedRobot.kDefaultPeriod,
        Constants.Elevators.xMotorRatio);

    m_zSim.setInput(m_zMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_zSim.update(TimedRobot.kDefaultPeriod);
    m_zMotor.setSimSensorPositionAndVelocity(
        m_zSim.getPositionMeters(),
        m_zSim.getVelocityMetersPerSecond(),
        TimedRobot.kDefaultPeriod,
        Constants.Elevators.zMotorRatio);

    // Update Z viz.
    final double kHeightPerMeter = 80 / Constants.Elevators.zMaxHeight;
    m_zRoot.setPosition(10.0, m_zSim.getPositionMeters() * kHeightPerMeter);

    // Update X viz.
    final double kExtensionPerMeter = 80 / Constants.Elevators.xMaxExtend;
    m_xRoot.setPosition(
        10.0 + m_xSim.getPositionMeters() * kExtensionPerMeter,
        10.0 + m_zSim.getPositionMeters() * kHeightPerMeter);
  }
  // --- END STUFF FOR SIMULATION ---
}
