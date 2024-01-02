package frc.quixlib.swerve;

public class FieldSpeeds {
  /** Represents +X velocity w.r.t the field frame of reference. */
  public final double vxMetersPerSecond;

  /** Represents +Y velocity w.r.t the field frame of reference. */
  public final double vyMetersPerSecond;

  /** Represents the angular velocity of the field frame. (CCW is +) */
  public final double omegaRadiansPerSecond;

  /** Constructs a FieldSpeeds with zeros for dx, dy, and theta. */
  public FieldSpeeds() {
    this(0.0, 0.0, 0.0);
  }

  /**
   * Constructs a FieldSpeeds object.
   *
   * @param vxMetersPerSecond +X velocity.
   * @param vyMetersPerSecond +Y velocity.
   * @param omegaRadiansPerSecond Angular velocity.
   */
  public FieldSpeeds(
      final double vxMetersPerSecond,
      final double vyMetersPerSecond,
      final double omegaRadiansPerSecond) {
    this.vxMetersPerSecond = vxMetersPerSecond;
    this.vyMetersPerSecond = vyMetersPerSecond;
    this.omegaRadiansPerSecond = omegaRadiansPerSecond;
  }
}
