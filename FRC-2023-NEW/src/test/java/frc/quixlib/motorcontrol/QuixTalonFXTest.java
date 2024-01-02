package frc.quixlib.motorcontrol;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.quixlib.devices.CANDeviceID;
import frc.quixlib.devices.QuixStatusSignal;
import org.junit.jupiter.api.*;

public class QuixTalonFXTest implements AutoCloseable {
  static final double kTol = 5e-2;
  static final double kCANDelayTime = 0.5; // Extra long because sim can run slow.

  QuixTalonFX controller;
  QuixTalonFX controller_follower;
  QuixTalonFX controller_inverted;
  QuixTalonFX controller_with_ratio;
  QuixTalonFX controller_with_ratio_inverted;

  static final int kSlotZero = 0;
  static final int kSlotPositive = 1;
  static final int kSlotNegative = 2;

  @BeforeEach
  public void setup() {
    assert HAL.initialize(500, 0);

    // Default controller and follower.
    controller =
        new QuixTalonFX(
            new CANDeviceID(0),
            new MechanismRatio(),
            QuixTalonFX.makeDefaultConfig()
                .setPIDConfig(kSlotZero, new PIDConfig(0.0, 0.0, 0.0))
                .setPIDConfig(kSlotPositive, new PIDConfig(1.0, 0.0, 0.0))
                .setPIDConfig(kSlotNegative, new PIDConfig(-1.0, 0.0, 0.0)));
    controller_follower = new QuixTalonFX(new CANDeviceID(1), controller, true);

    // Inverted controller.
    controller_inverted = new QuixTalonFX(new CANDeviceID(2), new MechanismRatio(), true);

    // Controller with ratio.
    controller_with_ratio =
        new QuixTalonFX(
            new CANDeviceID(3),
            new MechanismRatio(1.0, 2.0),
            QuixTalonFX.makeDefaultConfig()
                .setPIDConfig(kSlotZero, new PIDConfig(0.0, 0.0, 0.0))
                .setPIDConfig(kSlotPositive, new PIDConfig(1.0, 0.0, 0.0))
                .setPIDConfig(kSlotNegative, new PIDConfig(-1.0, 0.0, 0.0)));
    controller_with_ratio_inverted =
        new QuixTalonFX(new CANDeviceID(4), new MechanismRatio(1.0, 2.0), true);

    // Enable the robot
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    // Delay ~100ms so the devices can start up and enable
    Timer.delay(0.500);
  }

  @Override
  public void close() {
    controller.close();
    controller_follower.close();
    controller_inverted.close();
    controller_with_ratio.close();
    controller_with_ratio_inverted.close();
  }

  @AfterEach
  public void shutdown() throws Exception {
    close();
  }

  @Test
  public void testUnitConversions() {
    // To native sensor position
    assertEquals(0.0, controller.toNativeSensorPosition(0.0), kTol);
    assertEquals(1.0, controller.toNativeSensorPosition(2.0 * Math.PI), kTol);
    assertEquals(-1.0, controller.toNativeSensorPosition(-2.0 * Math.PI), kTol);

    assertEquals(0.0, controller_with_ratio.toNativeSensorPosition(0.0), kTol);
    assertEquals(2.0, controller_with_ratio.toNativeSensorPosition(2.0 * Math.PI), kTol);
    assertEquals(-2.0, controller_with_ratio.toNativeSensorPosition(-2.0 * Math.PI), kTol);

    // From native sensor position
    assertEquals(0.0, controller.fromNativeSensorPosition(0.0), kTol);
    assertEquals(2.0 * Math.PI, controller.fromNativeSensorPosition(1.0), kTol);
    assertEquals(-2.0 * Math.PI, controller.fromNativeSensorPosition(-1.0), kTol);

    assertEquals(0.0, controller_with_ratio.fromNativeSensorPosition(0.0), kTol);
    assertEquals(2.0 * Math.PI, controller_with_ratio.fromNativeSensorPosition(2.0), kTol);
    assertEquals(-2.0 * Math.PI, controller_with_ratio.fromNativeSensorPosition(-2.0), kTol);

    // To native sensor velocity
    assertEquals(0.0, controller.toNativeSensorVelocity(0.0), kTol);
    assertEquals(1.0, controller.toNativeSensorVelocity(2.0 * Math.PI), kTol);
    assertEquals(-1.0, controller.toNativeSensorVelocity(-2.0 * Math.PI), kTol);

    assertEquals(0.0, controller_with_ratio.toNativeSensorVelocity(0.0), kTol);
    assertEquals(2.0, controller_with_ratio.toNativeSensorVelocity(2.0 * Math.PI), kTol);
    assertEquals(-2.0, controller_with_ratio.toNativeSensorVelocity(-2.0 * Math.PI), kTol);

    // From native sensor velocity
    assertEquals(0.0, controller.fromNativeSensorVelocity(0.0), kTol);
    assertEquals(2.0 * Math.PI, controller.fromNativeSensorVelocity(1.0), kTol);
    assertEquals(-2.0 * Math.PI, controller.fromNativeSensorVelocity(-1.0), kTol);

    assertEquals(0.0, controller_with_ratio.fromNativeSensorVelocity(0.0), kTol);
    assertEquals(2.0 * Math.PI, controller_with_ratio.fromNativeSensorVelocity(2.0), kTol);
    assertEquals(-2.0 * Math.PI, controller_with_ratio.fromNativeSensorVelocity(-2.0), kTol);
  }

  @Test
  public void testGetInverted() {
    assertFalse(controller.getInverted());
    assertFalse(controller_follower.getInverted());
    assertTrue(controller_inverted.getInverted());
    assertFalse(controller_with_ratio.getInverted());
    assertTrue(controller_with_ratio_inverted.getInverted());
  }

  // @Test
  // public void testSetGetPercentOutput() {
  //   // Zero
  //   controller.setPercentOutput(0.0);
  //   Timer.delay(0.020);
  //   controller.percentOutputSignal().waitForUpdate(kCANDelayTime);
  //   assertEquals(controller.getPercentOutput(), 0.0, kTol);
  //   assertEquals(controller_follower.getPercentOutput(), 0.0, kTol);

  //   // Positive value
  //   controller.setPercentOutput(0.3);
  //   Timer.delay(0.020);
  //   controller.percentOutputSignal().waitForUpdate(kCANDelayTime);
  //   assertEquals(controller.getPercentOutput(), 0.3, kTol);
  //   assertEquals(controller_follower.getPercentOutput(), 0.3, kTol);

  //   // Negative value
  //   controller.setPercentOutput(-0.5);
  //   Timer.delay(0.020);
  //   controller.percentOutputSignal().waitForUpdate(kCANDelayTime);
  //   assertEquals(controller.getPercentOutput(), -0.5, kTol);
  //   assertEquals(controller_follower.getPercentOutput(), -0.5, kTol);

  //   // > 1.0
  //   controller.setPercentOutput(1.5);
  //   Timer.delay(0.020);
  //   controller.percentOutputSignal().waitForUpdate(kCANDelayTime);
  //   assertEquals(controller.getPercentOutput(), 1.0, kTol);
  //   assertEquals(controller_follower.getPercentOutput(), 1.0, kTol);

  //   // < -1.0
  //   controller.setPercentOutput(-10.0);
  //   Timer.delay(0.020);
  //   controller.percentOutputSignal().waitForUpdate(kCANDelayTime);
  //   assertEquals(controller.getPercentOutput(), -1.0, kTol);
  //   assertEquals(controller_follower.getPercentOutput(), -1.0, kTol);
  // }

  // @Test
  // public void testSetGetPercentOutputInverted() {
  //   // Zero
  //   controller_inverted.setPercentOutput(0.0);
  //   Timer.delay(0.020);
  //   controller_inverted.percentOutputSignal().waitForUpdate(kCANDelayTime);
  //   assertEquals(controller_inverted.getPercentOutput(), 0.0, kTol);

  //   // Positive value
  //   controller_inverted.setPercentOutput(0.3);
  //   Timer.delay(0.020);
  //   controller_inverted.percentOutputSignal().waitForUpdate(kCANDelayTime);
  //   assertEquals(controller_inverted.getPercentOutput(), 0.3, kTol);

  //   // Negative value
  //   controller_inverted.setPercentOutput(-0.5);
  //   Timer.delay(0.020);
  //   controller_inverted.percentOutputSignal().waitForUpdate(kCANDelayTime);
  //   assertEquals(controller_inverted.getPercentOutput(), -0.5, kTol);

  //   // > 1.0
  //   controller_inverted.setPercentOutput(1.5);
  //   Timer.delay(0.020);
  //   assertEquals(controller_inverted.getPercentOutput(), 1.0, kTol);

  //   // < -1.0
  //   controller_inverted.setPercentOutput(-10.0);
  //   Timer.delay(0.020);
  //   controller_inverted.percentOutputSignal().waitForUpdate(kCANDelayTime);
  //   assertEquals(controller_inverted.getPercentOutput(), -1.0, kTol);
  // }

  @Test
  public void testEncoderSetPosition() {
    // Set positive position
    controller.setSensorPosition(1.0);
    controller_inverted.setSensorPosition(1.0);
    controller_with_ratio.setSensorPosition(1.0);
    QuixStatusSignal.waitForAll(
        kCANDelayTime,
        controller.sensorPositionSignal(),
        controller_inverted.sensorPositionSignal(),
        controller_with_ratio.sensorPositionSignal());
    assertEquals(controller.getSensorPosition(), 1.0, kTol);
    assertEquals(controller_inverted.getSensorPosition(), 1.0, kTol);
    assertEquals(controller_with_ratio.getSensorPosition(), 1.0, kTol);

    // Set negative position
    controller.setSensorPosition(-15.0);
    controller_inverted.setSensorPosition(-15.0);
    controller_with_ratio.setSensorPosition(-15.0);
    QuixStatusSignal.waitForAll(
        kCANDelayTime,
        controller.sensorPositionSignal(),
        controller_inverted.sensorPositionSignal(),
        controller_with_ratio.sensorPositionSignal());
    assertEquals(controller.getSensorPosition(), -15.0, kTol);
    assertEquals(controller_inverted.getSensorPosition(), -15.0, kTol);
    assertEquals(controller_with_ratio.getSensorPosition(), -15.0, kTol);

    // Set zero
    controller.zeroSensorPosition();
    controller_inverted.zeroSensorPosition();
    controller_with_ratio.zeroSensorPosition();
    QuixStatusSignal.waitForAll(
        kCANDelayTime,
        controller.sensorPositionSignal(),
        controller_inverted.sensorPositionSignal(),
        controller_with_ratio.sensorPositionSignal());
    assertEquals(controller.getSensorPosition(), 0.0, kTol);
    assertEquals(controller_inverted.getSensorPosition(), 0.0, kTol);
    assertEquals(controller_with_ratio.getSensorPosition(), 0.0, kTol);
  }

  @Test
  public void testSimSensor() {
    controller.setSensorPosition(10.0);
    controller_with_ratio.setSensorPosition(10.0);

    // Positive speed
    controller.setSimSensorVelocity(100.0, 0.02, controller.getMechanismRatio());
    controller_with_ratio.setSimSensorVelocity(
        100.0, 0.02, controller_with_ratio.getMechanismRatio());
    QuixStatusSignal.waitForAll(
        kCANDelayTime,
        controller.sensorPositionSignal(),
        controller.sensorVelocitySignal(),
        controller_with_ratio.sensorPositionSignal(),
        controller_with_ratio.sensorVelocitySignal());
    assertEquals(controller.getSensorVelocity(), 100.0, kTol);
    assertEquals(controller.getSensorPosition(), 12.0, kTol);
    assertEquals(controller_with_ratio.getSensorVelocity(), 100.0, kTol);
    assertEquals(controller_with_ratio.getSensorPosition(), 12.0, kTol);

    // Negative speed
    controller.setSimSensorVelocity(-100.0, 0.02, controller.getMechanismRatio());
    controller_with_ratio.setSimSensorVelocity(
        -100.0, 0.02, controller_with_ratio.getMechanismRatio());
    QuixStatusSignal.waitForAll(
        kCANDelayTime,
        controller.sensorPositionSignal(),
        controller.sensorVelocitySignal(),
        controller_with_ratio.sensorPositionSignal(),
        controller_with_ratio.sensorVelocitySignal());
    assertEquals(controller.getSensorVelocity(), -100.0, kTol);
    assertEquals(controller.getSensorPosition(), 10.0, kTol);
    assertEquals(controller_with_ratio.getSensorVelocity(), -100.0, kTol);
    assertEquals(controller_with_ratio.getSensorPosition(), 10.0, kTol);
  }

  @Test
  public void testSimSensorInverted() {
    controller_inverted.setSensorPosition(10.0);
    controller_with_ratio_inverted.setSensorPosition(10.0);

    // Positive speed
    controller_inverted.setSimSensorVelocity(100.0, 0.02, controller_inverted.getMechanismRatio());
    controller_with_ratio_inverted.setSimSensorVelocity(
        100.0, 0.02, controller_with_ratio_inverted.getMechanismRatio());
    QuixStatusSignal.waitForAll(
        kCANDelayTime,
        controller_inverted.sensorPositionSignal(),
        controller_inverted.sensorVelocitySignal(),
        controller_with_ratio_inverted.sensorPositionSignal(),
        controller_with_ratio_inverted.sensorVelocitySignal());
    assertEquals(controller_inverted.getSensorVelocity(), 100.0, kTol);
    assertEquals(controller_inverted.getSensorPosition(), 12.0, kTol);
    assertEquals(controller_with_ratio_inverted.getSensorVelocity(), 100.0, kTol);
    assertEquals(controller_with_ratio_inverted.getSensorPosition(), 12.0, kTol);

    // Negative speed
    controller_inverted.setSimSensorVelocity(-100.0, 0.02, controller_inverted.getMechanismRatio());
    controller_with_ratio_inverted.setSimSensorVelocity(
        -100.0, 0.02, controller_with_ratio_inverted.getMechanismRatio());
    QuixStatusSignal.waitForAll(
        kCANDelayTime,
        controller_inverted.sensorPositionSignal(),
        controller_inverted.sensorVelocitySignal(),
        controller_with_ratio_inverted.sensorPositionSignal(),
        controller_with_ratio_inverted.sensorVelocitySignal());
    assertEquals(controller_inverted.getSensorVelocity(), -100.0, kTol);
    assertEquals(controller_inverted.getSensorPosition(), 10.0, kTol);
    assertEquals(controller_with_ratio_inverted.getSensorVelocity(), -100.0, kTol);
    assertEquals(controller_with_ratio_inverted.getSensorPosition(), 10.0, kTol);
  }

  // @Test
  // public void testPositionControl() {
  //   double kMaxVoltage = 12.0;

  //   // Zero gains
  //   controller.setPositionSetpoint(kSlotZero, 100.0);
  //   controller_with_ratio.setPositionSetpoint(kSlotZero, 100.0);
  //   Timer.delay(0.020);
  //   QuixStatusSignal.waitForAll(
  //       kCANDelayTime,
  //       controller.percentOutputSignal(),
  //       controller_with_ratio.percentOutputSignal());
  //   assertEquals(controller.getPercentOutput(), 0.0, kTol);
  //   assertEquals(controller_with_ratio.getPercentOutput(), 0.0, kTol);

  //   // Feed-forward only
  //   controller.setPositionSetpoint(kSlotZero, 100.0, 0.5 * kMaxVoltage);
  //   controller_with_ratio.setPositionSetpoint(kSlotZero, 100.0, 0.5 * kMaxVoltage);
  //   Timer.delay(0.020);
  //   QuixStatusSignal.waitForAll(
  //       kCANDelayTime,
  //       controller.percentOutputSignal(),
  //       controller_with_ratio.percentOutputSignal());
  //   assertEquals(controller.getPercentOutput(), 0.5, kTol);
  //   assertEquals(controller_with_ratio.getPercentOutput(), 0.5, kTol);

  //   // Positive kP, positive setpoint
  //   controller.setPositionSetpoint(kSlotPositive, 100.0);
  //   controller_with_ratio.setPositionSetpoint(kSlotPositive, 100.0);
  //   Timer.delay(0.020);
  //   QuixStatusSignal.waitForAll(
  //       kCANDelayTime,
  //       controller.percentOutputSignal(),
  //       controller_with_ratio.percentOutputSignal());
  //   assertTrue(controller.getPercentOutput() > 0.0);
  //   assertTrue(controller_with_ratio.getPercentOutput() > 0.0);

  //   // Update position so error is zero.
  //   controller.setSensorPosition(100.0);
  //   controller_with_ratio.setSensorPosition(100.0);
  //   Timer.delay(0.020);
  //   QuixStatusSignal.waitForAll(
  //       kCANDelayTime,
  //       controller.percentOutputSignal(),
  //       controller_with_ratio.percentOutputSignal());
  //   assertEquals(controller.getPercentOutput(), 0.0, kTol);
  //   assertEquals(controller_with_ratio.getPercentOutput(), 0.0, kTol);

  //   // Reset position.
  //   controller.setSensorPosition(100.0);
  //   controller_with_ratio.setSensorPosition(100.0);

  //   // Positive kP, negative setpoint
  //   controller.setPositionSetpoint(kSlotPositive, -100.0);
  //   controller_with_ratio.setPositionSetpoint(kSlotPositive, -100.0);
  //   Timer.delay(0.020);
  //   QuixStatusSignal.waitForAll(
  //       kCANDelayTime,
  //       controller.percentOutputSignal(),
  //       controller_with_ratio.percentOutputSignal());
  //   assertTrue(controller.getPercentOutput() < 0.0);
  //   assertTrue(controller_with_ratio.getPercentOutput() < 0.0);

  //   // Negative kP, positive setpoint should result in no output.
  //   controller.setPositionSetpoint(kSlotNegative, 100.0);
  //   controller_with_ratio.setPositionSetpoint(kSlotNegative, 100.0);
  //   Timer.delay(0.020);
  //   QuixStatusSignal.waitForAll(
  //       kCANDelayTime,
  //       controller.percentOutputSignal(),
  //       controller_with_ratio.percentOutputSignal());
  //   assertEquals(controller.getPercentOutput(), 0.0, kTol);
  //   assertEquals(controller_with_ratio.getPercentOutput(), 0.0, kTol);

  //   // Negative kP, negative setpoint should result in no output.
  //   controller.setPositionSetpoint(kSlotNegative, -100.0);
  //   controller_with_ratio.setPositionSetpoint(kSlotNegative, -100.0);
  //   Timer.delay(0.020);
  //   QuixStatusSignal.waitForAll(
  //       kCANDelayTime,
  //       controller.percentOutputSignal(),
  //       controller_with_ratio.percentOutputSignal());
  //   assertEquals(controller.getPercentOutput(), 0.0, kTol);
  //   assertEquals(controller_with_ratio.getPercentOutput(), 0.0, kTol);
  // }

  // @Test
  // public void testSoftLimits() {
  //   final double kFwdLimit = 10.0;
  //   final double kRevLimit = -10.0;

  //   final QuixTalonFX motorWithLimit =
  //       new QuixTalonFX(
  //           new CANDeviceID(5),
  //           new MechanismRatio(1.0, 2.0),
  //           QuixTalonFX.makeDefaultConfig()
  //               .setForwardSoftLimit(kFwdLimit)
  //               .setReverseSoftLimit(kRevLimit));
  //   final QuixTalonFX motorWithoutLimit =
  //       new QuixTalonFX(new CANDeviceID(6), new MechanismRatio(1.0, 2.0));

  //   // Within soft limits. Fwd/rev should both work.
  //   motorWithLimit.setPercentOutput(0.5);
  //   motorWithoutLimit.setPercentOutput(0.5);
  //   Thread.sleep(200); // Wait for CAN delay
  //   assertEquals(0.5, motorWithLimit.getPercentOutput(), kTol);
  //   assertEquals(0.5, motorWithoutLimit.getPercentOutput(), kTol);

  //   motorWithLimit.setPercentOutput(-0.5);
  //   motorWithoutLimit.setPercentOutput(-0.5);
  //   Thread.sleep(200); // Wait for CAN delay
  //   assertEquals(-0.5, motorWithLimit.getPercentOutput(), kTol);
  //   assertEquals(-0.5, motorWithoutLimit.getPercentOutput(), kTol);

  //   // Just within fwd limit. Fwd/rev should both work.
  //   motorWithLimit.setSensorPosition(kFwdLimit - 0.1);
  //   motorWithoutLimit.setSensorPosition(kFwdLimit - 0.1);
  //   Thread.sleep(200); // Wait for CAN delay

  //   motorWithLimit.setPercentOutput(0.5);
  //   motorWithoutLimit.setPercentOutput(0.5);
  //   Thread.sleep(200); // Wait for CAN delay
  //   assertEquals(0.5, motorWithLimit.getPercentOutput(), kTol);
  //   assertEquals(0.5, motorWithoutLimit.getPercentOutput(), kTol);

  //   motorWithLimit.setPercentOutput(-0.5);
  //   motorWithoutLimit.setPercentOutput(-0.5);
  //   Thread.sleep(200); // Wait for CAN delay
  //   assertEquals(-0.5, motorWithLimit.getPercentOutput(), kTol);
  //   assertEquals(-0.5, motorWithoutLimit.getPercentOutput(), kTol);

  //   // Just within rev limit. Fwd/rev should both work.
  //   motorWithLimit.setSensorPosition(kRevLimit + 0.1);
  //   motorWithoutLimit.setSensorPosition(kRevLimit + 0.1);
  //   Thread.sleep(200); // Wait for CAN delay

  //   motorWithLimit.setPercentOutput(0.5);
  //   motorWithoutLimit.setPercentOutput(0.5);
  //   Thread.sleep(200); // Wait for CAN delay
  //   assertEquals(0.5, motorWithLimit.getPercentOutput(), kTol);
  //   assertEquals(0.5, motorWithoutLimit.getPercentOutput(), kTol);

  //   motorWithLimit.setPercentOutput(-0.5);
  //   motorWithoutLimit.setPercentOutput(-0.5);
  //   Thread.sleep(200); // Wait for CAN delay
  //   assertEquals(-0.5, motorWithLimit.getPercentOutput(), kTol);
  //   assertEquals(-0.5, motorWithoutLimit.getPercentOutput(), kTol);

  //   // Beyond forward soft limit. Fwd should be limited to zero.
  //   motorWithLimit.setSensorPosition(kFwdLimit + 0.1);
  //   motorWithoutLimit.setSensorPosition(kFwdLimit + 0.1);
  //   Thread.sleep(200); // Wait for CAN delay

  //   motorWithLimit.setPercentOutput(0.5);
  //   motorWithoutLimit.setPercentOutput(0.5);
  //   Thread.sleep(200); // Wait for CAN delay
  //   assertEquals(0.0, motorWithLimit.getPercentOutput(), kTol);
  //   assertEquals(0.5, motorWithoutLimit.getPercentOutput(), kTol);

  //   motorWithLimit.setPercentOutput(-0.5);
  //   motorWithoutLimit.setPercentOutput(-0.5);
  //   Thread.sleep(200); // Wait for CAN delay
  //   assertEquals(-0.5, motorWithLimit.getPercentOutput(), kTol);
  //   assertEquals(-0.5, motorWithoutLimit.getPercentOutput(), kTol);

  //   // Beyind reverse soft limit. Rev should be limited to zero.
  //   motorWithLimit.setSensorPosition(kRevLimit - 0.1);
  //   motorWithoutLimit.setSensorPosition(kRevLimit - 0.1);
  //   Thread.sleep(200); // Wait for CAN delay

  //   motorWithLimit.setPercentOutput(0.5);
  //   motorWithoutLimit.setPercentOutput(0.5);
  //   Thread.sleep(200); // Wait for CAN delay
  //   assertEquals(0.5, motorWithLimit.getPercentOutput(), kTol);
  //   assertEquals(0.5, motorWithoutLimit.getPercentOutput(), kTol);

  //   motorWithLimit.setPercentOutput(-0.5);
  //   motorWithoutLimit.setPercentOutput(-0.5);
  //   Thread.sleep(200); // Wait for CAN delay
  //   assertEquals(0.0, motorWithLimit.getPercentOutput(), kTol);
  //   assertEquals(-0.5, motorWithoutLimit.getPercentOutput(), kTol);
  // }
}
