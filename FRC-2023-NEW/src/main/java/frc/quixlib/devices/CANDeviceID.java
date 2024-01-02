package frc.quixlib.devices;

public class CANDeviceID {
  public static final String kRIOCANbusName = "rio";

  public final int deviceNumber;
  public final String CANbusName;

  public CANDeviceID(final int deviceNumber, final String CANbusName) {
    this.deviceNumber = deviceNumber;
    this.CANbusName = CANbusName;
  }

  public CANDeviceID(final int deviceNumber) {
    this(deviceNumber, kRIOCANbusName);
  }

  public String toString() {
    return "[" + CANbusName + ":" + deviceNumber + "]";
  }
}
