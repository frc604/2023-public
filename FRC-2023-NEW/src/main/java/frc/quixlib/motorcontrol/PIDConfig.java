package frc.quixlib.motorcontrol;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;

public class PIDConfig {
  // This matches CTRE configs
  public final double kP;
  public final double kI;
  public final double kD;
  public final double kS;
  public final double kV;
  public final double kA;
  public final double kG;

  public PIDConfig() {
    this(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  public PIDConfig(final double kP, final double kI, final double kD) {
    this(kP, kI, kD, 0.0, 0.0, 0.0, 0.0);
  }

  public PIDConfig(
      final double kP,
      final double kI,
      final double kD,
      final double kS,
      final double kV,
      final double kA,
      final double kG) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    this.kG = kG;
  }

  public Slot0Configs fillCTRE(Slot0Configs conf) {
    conf.kP = kP;
    conf.kI = kI;
    conf.kD = kD;
    conf.kS = kS;
    conf.kV = kV;
    conf.kA = kA;
    conf.kG = kG;
    return conf;
  }

  public Slot1Configs fillCTRE(Slot1Configs conf) {
    conf.kP = kP;
    conf.kI = kI;
    conf.kD = kD;
    conf.kS = kS;
    conf.kV = kV;
    conf.kA = kA;
    conf.kG = kG;
    return conf;
  }

  public Slot2Configs fillCTRE(Slot2Configs conf) {
    conf.kP = kP;
    conf.kI = kI;
    conf.kD = kD;
    conf.kS = kS;
    conf.kV = kV;
    conf.kA = kA;
    conf.kG = kG;
    return conf;
  }
}
