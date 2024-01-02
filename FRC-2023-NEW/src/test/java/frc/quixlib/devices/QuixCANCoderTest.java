package frc.quixlib.devices;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.Timer;
import frc.quixlib.motorcontrol.MechanismRatio;
import org.junit.jupiter.api.*;

public class QuixCANCoderTest {
  static final double kTol = 5e-2;
  static final double kCANDelayTime = 0.5; // Extra long because sim can run slow.

  @Test
  public void testBasic() {
    QuixCANCoder cancoder = new QuixCANCoder(new CANDeviceID(0), new MechanismRatio());

    cancoder.zero(); // CANCoder sim starts out non-zero.
    Timer.delay(0.020);
    QuixStatusSignal.waitForAll(
        kCANDelayTime, cancoder.positionSignal(), cancoder.absolutePositionSignal());
    assertEquals(0.0, cancoder.getPosition(), kTol);
    assertEquals(0.0, cancoder.getAbsPosition(), kTol);
  }

  @Test
  public void testSim() {
    QuixCANCoder cancoder = new QuixCANCoder(new CANDeviceID(1), new MechanismRatio());

    cancoder.zero(); // CANCoder sim starts out non-zero.
    Timer.delay(0.020);
    QuixStatusSignal.waitForAll(
        kCANDelayTime, cancoder.positionSignal(), cancoder.absolutePositionSignal());
    assertEquals(0.0, cancoder.getPosition(), kTol);
    assertEquals(0.0, cancoder.getAbsPosition(), kTol);

    cancoder.setSimSensorVelocity(12.3, 0.1);
    Timer.delay(0.020);
    QuixStatusSignal.waitForAll(
        kCANDelayTime,
        cancoder.positionSignal(),
        cancoder.absolutePositionSignal(),
        cancoder.velocitySignal());
    assertEquals(1.23, cancoder.getPosition(), kTol);
    assertEquals(1.23, cancoder.getAbsPosition(), kTol);
    assertEquals(12.3, cancoder.getVelocity(), kTol);

    cancoder.setSimSensorVelocity(10.0, 0.1);
    Timer.delay(0.020);
    QuixStatusSignal.waitForAll(
        kCANDelayTime,
        cancoder.positionSignal(),
        cancoder.absolutePositionSignal(),
        cancoder.velocitySignal());
    assertEquals(2.23, cancoder.getPosition(), kTol);
    assertEquals(2.23, cancoder.getAbsPosition(), kTol);
    assertEquals(10.0, cancoder.getVelocity(), kTol);

    cancoder.setSimSensorVelocity(5.0, 1.0);
    Timer.delay(0.020);
    QuixStatusSignal.waitForAll(
        kCANDelayTime,
        cancoder.positionSignal(),
        cancoder.absolutePositionSignal(),
        cancoder.velocitySignal());
    assertEquals(7.23, cancoder.getPosition(), kTol);
    assertEquals(7.23 % (2.0 * Math.PI), cancoder.getAbsPosition(), kTol);
    assertEquals(5.0, cancoder.getVelocity(), kTol);

    cancoder.setSimSensorVelocity(-1.0, 0.1);
    Timer.delay(0.020);
    QuixStatusSignal.waitForAll(
        kCANDelayTime,
        cancoder.positionSignal(),
        cancoder.absolutePositionSignal(),
        cancoder.velocitySignal());
    assertEquals(7.13, cancoder.getPosition(), kTol);
    assertEquals(7.13 % (2.0 * Math.PI), cancoder.getAbsPosition(), kTol);
    assertEquals(-1.0, cancoder.getVelocity(), kTol);
  }

  @Test
  public void testSetPosition() {
    QuixCANCoder cancoder = new QuixCANCoder(new CANDeviceID(2), new MechanismRatio());

    cancoder.zero(); // CANCoder sim starts out non-zero.
    Timer.delay(0.020);
    QuixStatusSignal.waitForAll(
        kCANDelayTime, cancoder.positionSignal(), cancoder.absolutePositionSignal());
    assertEquals(0.0, cancoder.getPosition(), kTol);
    assertEquals(0.0, cancoder.getAbsPosition(), kTol);

    cancoder.setPosition(12.3);
    Timer.delay(0.020);
    QuixStatusSignal.waitForAll(
        kCANDelayTime, cancoder.positionSignal(), cancoder.absolutePositionSignal());
    assertEquals(12.3, cancoder.getPosition(), kTol);
    assertEquals(12.3 % (2.0 * Math.PI), cancoder.getAbsPosition(), kTol);

    cancoder.zero();
    Timer.delay(0.020);
    QuixStatusSignal.waitForAll(
        kCANDelayTime, cancoder.positionSignal(), cancoder.absolutePositionSignal());
    assertEquals(0.0, cancoder.getAbsPosition(), kTol);
    assertEquals(0.0, cancoder.getAbsPosition(), kTol);
  }

  @Test
  public void testRefresh() {
    QuixCANCoder cancoder = new QuixCANCoder(new CANDeviceID(3), new MechanismRatio());

    cancoder.zero(); // CANCoder sim starts out non-zero.
    Timer.delay(0.020);
    QuixStatusSignal.waitForAll(
        kCANDelayTime, cancoder.positionSignal(), cancoder.absolutePositionSignal());
    assertEquals(0.0, cancoder.getPosition(), kTol);
    assertEquals(0.0, cancoder.getAbsPosition(), kTol);

    cancoder.setPosition(1.0);
    // Even though the position has been set, the new signal hasn't been received yet.
    assertEquals(0.0, cancoder.getPosition(), kTol);
    assertEquals(0.0, cancoder.getAbsPosition(), kTol);

    // After the signal should have updated, if we don't refresh we don't see the new value.
    Timer.delay(kCANDelayTime);
    assertEquals(0.0, cancoder.positionSignal().getValue(), kTol);
    assertEquals(0.0, cancoder.absolutePositionSignal().getValue(), kTol);

    // Performing a refresh, we finally see the updated value.
    QuixStatusSignal.refreshAll(cancoder.positionSignal(), cancoder.absolutePositionSignal());
    assertEquals(1.0, cancoder.positionSignal().getValue(), kTol);
    assertEquals(1.0, cancoder.absolutePositionSignal().getValue(), kTol);

    cancoder.setPosition(2.0);
    // Even though the position has been set, the new signal hasn't been received yet.
    assertEquals(1.0, cancoder.getPosition(), kTol);
    assertEquals(1.0, cancoder.getAbsPosition(), kTol);

    // After the signal should have updated, if we don't refresh we don't see the new value.
    Timer.delay(kCANDelayTime);
    assertEquals(1.0, cancoder.positionSignal().getValue(), kTol);
    assertEquals(1.0, cancoder.absolutePositionSignal().getValue(), kTol);

    // Performing a refresh, we finally see the updated value.
    // Uses implicit refresh.
    assertEquals(2.0, cancoder.getPosition(), kTol);
    assertEquals(2.0, cancoder.getAbsPosition(), kTol);
  }
}
