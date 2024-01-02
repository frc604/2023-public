package frc.quixlib.wpilib;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.*;

public class Rotation2dTest {
  private final double kTol = 1e-6;

  @Test
  public void testRotation2d() {
    // Test to ensure WPILib doesn't break our assumptions about Rotation2d.
    // We assume that values do not get wrapped even if they are out of the range [-pi, pi).
    // This was broken once before (but then reverted) here:
    // https://github.com/wpilibsuite/allwpilib/pull/4611
    Rotation2d r = new Rotation2d(15.5 * Math.PI);
    assertEquals(15.5 * Math.PI, r.getRadians(), kTol);

    // Values get wrapped after performing an operation (even if the operation is just adding zero).
    assertEquals(-0.5 * Math.PI, r.plus(new Rotation2d()).getRadians(), kTol);
  }
}
