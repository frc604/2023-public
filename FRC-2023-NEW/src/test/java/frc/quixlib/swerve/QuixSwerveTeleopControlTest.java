package frc.quixlib.swerve;

import static org.junit.jupiter.api.Assertions.*;

import frc.quixlib.math.MathUtils;
import frc.quixlib.swerve.QuixSwerveTeleopControl.DriveVelocities;
import org.junit.jupiter.api.*;

public class QuixSwerveTeleopControlTest {
  static final double kTol = 1e-6;

  @Test
  public void testDeadband() {
    final double kDeadband = 0.15;
    // Zero is still zero
    assertEquals(0.0, QuixSwerveTeleopControl.applyDeadband(0.0, kDeadband), kTol);
    // One is still one
    assertEquals(1.0, QuixSwerveTeleopControl.applyDeadband(1.0, kDeadband), kTol);
    // Output at deadband is zero
    assertEquals(0.0, QuixSwerveTeleopControl.applyDeadband(0.15, kDeadband), kTol);
    // Check other values
    assertEquals(0.25, QuixSwerveTeleopControl.applyDeadband(0.3625, kDeadband), kTol);
    assertEquals(0.5, QuixSwerveTeleopControl.applyDeadband(0.575, kDeadband), kTol);
    assertEquals(0.75, QuixSwerveTeleopControl.applyDeadband(0.7875, kDeadband), kTol);
  }

  private void assertSpeedScalingIsCorrect(
      final double r, final double theta, final boolean squared) {
    final double kMaxSpeed = 4.0;
    final QuixSwerveTeleopControl control =
        new QuixSwerveTeleopControl(
            Double.POSITIVE_INFINITY,
            Double.POSITIVE_INFINITY,
            0.0,
            kMaxSpeed,
            2.0 * Math.PI,
            squared);

    final var xy = MathUtils.pol2cart(r, theta);
    final DriveVelocities driveVel =
        control.getDriveVelocitiesFromJoysticks(xy.getFirst(), xy.getSecond(), 0.0);
    assertEquals(
        squared ? r * r * kMaxSpeed : r * kMaxSpeed,
        Math.sqrt(driveVel.xVel * driveVel.xVel + driveVel.yVel * driveVel.yVel),
        kTol);
  }

  @Test
  public void testSquaredScaling() {
    // 0 power, full power, and other values in all quadrants and axis aligned angles
    for (double r = 0.0; r <= 1.0; r += 0.25) {
      for (double theta = 0.0; theta < 2 * Math.PI; theta += Math.PI * 0.25) {
        assertSpeedScalingIsCorrect(r, theta, true);
      }
    }
  }

  @Test
  public void testLinearScaling() {
    // 0 power, full power, and other values in all quadrants and axis aligned angles
    for (double r = 0.0; r <= 1.0; r += 0.25) {
      for (double theta = 0.0; theta < 2 * Math.PI; theta += Math.PI * 0.25) {
        assertSpeedScalingIsCorrect(r, theta, false);
      }
    }
  }
}
