package frc.quixlib.motorcontrol;

/** Defines a ratio and distance per rotation between motor and mechanism. */
public class MechanismRatio {
  private static final double kDefaultDistancePerRotation = 2.0 * Math.PI; // Default to radians;

  private final double m_driving;
  private final double m_driven;
  private final double m_distancePerRotation;

  public MechanismRatio() {
    this(1.0, 1.0);
  }

  public MechanismRatio(double drivingTeeth, double drivenTeeth) {
    this(drivingTeeth, drivenTeeth, kDefaultDistancePerRotation);
  }

  public MechanismRatio(double drivingTeeth, double drivenTeeth, double distancePerRotation) {
    m_driving = drivingTeeth;
    m_driven = drivenTeeth;
    m_distancePerRotation = distancePerRotation;
  }

  /**
   * Returns the mechanism ratio defined as: driven / driving. Numbers greater than 1 represent
   * reductions.
   */
  public double reduction() {
    return m_driven / m_driving;
  }

  /** Returns the distance per rotation of the driven gear. */
  public double distancePerRotation() {
    return m_distancePerRotation;
  }

  /** Returns the mechanism position for the given sensor position in radians. */
  public double sensorRadiansToMechanismPosition(double sensorRadians) {
    final double distancePerRadian = m_distancePerRotation / (2.0 * Math.PI);
    return sensorRadians * distancePerRadian / reduction();
  }

  /** Returns the sensor position in radians for the given mechanism position. */
  public double mechanismPositionToSensorRadians(double mechanismPos) {
    return mechanismPos / sensorRadiansToMechanismPosition(1.0);
  }
}
