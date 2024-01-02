package frc.quixlib.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveSetpoint {
  public final ChassisSpeeds chassisSpeeds;
  public SwerveModuleState[] moduleStates;

  public SwerveSetpoint(final ChassisSpeeds chassisSpeeds, final SwerveModuleState[] moduleStates) {
    this.chassisSpeeds = chassisSpeeds;
    this.moduleStates = moduleStates;
  }

  @Override
  public String toString() {
    String ret = chassisSpeeds.toString() + "\n";
    for (int i = 0; i < moduleStates.length; ++i) {
      ret += "  " + moduleStates[i].toString() + "\n";
    }
    return ret;
  }
}
