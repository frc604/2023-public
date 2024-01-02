// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.robot.Constants;

public class Gripper extends SubsystemBase {
  private static final QuixTalonFX m_gripperMotor =
      new QuixTalonFX(
          Constants.Gripper.gripperMotor,
          Constants.Gripper.rollerRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Gripper.gripperMotorInvert)
              .setBrakeMode()
              .setSupplyCurrentLimit(15.0)
              .setStatorCurrentLimit(40.0));
  private boolean m_hasPiece = false;

  public Gripper() {
    // Show scheduler status in SmartDashboard.
    SmartDashboard.putData(this);

    // Visualize sim in SmartDashboard.
    setupViz();
  }

  public void setHasPiece(boolean hasPiece) {
    m_hasPiece = hasPiece;
  }

  /**
   * Whether or not the gripper is holding a piece. Note that this is determined via logic not
   * sensors.
   */
  public boolean hasPiece() {
    return m_hasPiece;
  }

  public void handoff() {
    m_gripperMotor.setPercentOutput(-Constants.Gripper.handoffPower);
  }

  public void hold() {
    m_gripperMotor.setPercentOutput(-Constants.Gripper.holdPower);
  }

  public void score() {
    m_gripperMotor.setPercentOutput(Constants.Gripper.scorePower);
  }

  public void stopSpin() {
    m_gripperMotor.setPercentOutput(0.0);
  }

  public boolean isRollerStopped() {
    final double kConsideredStopVel = 1.0; // rad/s
    return Math.abs(m_gripperMotor.getSensorVelocity()) < kConsideredStopVel;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Gripper: Current Velocity", m_gripperMotor.getSensorVelocity());
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private static final FlywheelSim m_rollerSim =
      new FlywheelSim(
          DCMotor.getFalcon500(1),
          m_gripperMotor.getMechanismRatio().reduction(),
          Constants.Gripper.rollerMOI);

  // Visualization
  private final Mechanism2d m_simViz = new Mechanism2d(100, 100);
  private MechanismRoot2d m_rollerPivot;
  private MechanismLigament2d m_rollerLigament;

  private void setupViz() {
    m_rollerPivot = m_simViz.getRoot("GripperPivot", 30, 50);
    m_rollerLigament =
        m_rollerPivot.append(
            new MechanismLigament2d("Roller", 5, 90, 10, new Color8Bit(Color.kBlue)));
    SmartDashboard.putData("Gripper Viz", m_simViz);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    // // Simulate launcher rollers and update sensors.
    // m_rollerSim.setInput(
    //     m_gripperMotor.getPhysicalPercentOutput() * RobotController.getBatteryVoltage());
    // m_rollerSim.setInput(
    //     m_rightMotor.getPhysicalPercentOutput() * RobotController.getBatteryVoltage());
    // m_rollerSim.update(TimedRobot.kDefaultPeriod);
    // m_rollerSim.update(TimedRobot.kDefaultPeriod);
    // m_gripperMotor.setSimSensorVelocity(
    //     (m_gripperMotor.getInverted() ? -1.0 : 1.0) * m_rollerSim.getAngularVelocityRadPerSec(),
    //     TimedRobot.kDefaultPeriod,
    //     Constants.Gripper.rollerRatio);
    // m_rightMotor.setSimSensorVelocity(
    //     (m_rightMotor.getInverted() ? -1.0 : 1.0) * m_rollerSim.getAngularVelocityRadPerSec(),
    //     TimedRobot.kDefaultPeriod,
    //     Constants.Gripper.rollerRatio);

    // // Update the viz based on the simulated flywheel velocity.
    // final double kVizScale = 0.2; // Scale viz velocity to make high velocities look better.
    // final double dAngleFront =
    //     Math.toDegrees(m_rollerSim.getAngularVelocityRadPerSec() * TimedRobot.kDefaultPeriod);
    // m_rollerLigament.setAngle(m_rollerLigament.getAngle() + dAngleFront * kVizScale);
    // final double dAngleRear =
    //     Math.toDegrees(m_rollerSim.getAngularVelocityRadPerSec() * TimedRobot.kDefaultPeriod);
    // m_rightRollerLigament.setAngle(m_rightRollerLigament.getAngle() + dAngleRear * kVizScale);
  }
  // --- END STUFF FOR SIMULATION ---
}
