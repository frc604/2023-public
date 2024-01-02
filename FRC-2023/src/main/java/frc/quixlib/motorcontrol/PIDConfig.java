package frc.quixlib.motorcontrol;

public class PIDConfig {
  public final double kP;
  public final double kI;
  public final double kD;
  public final double kF;

  public PIDConfig(final double kP, final double kI, final double kD) {
    this(kP, kI, kD, 0.0);
  }

  public PIDConfig(final double kP, final double kI, final double kD, final double kF) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kF = kF;
  }
}
