package frc.quixlib.localization;

import edu.wpi.first.math.Pair;
import java.util.ArrayList;

public class NTVisionMeasurement {
  private int m_id; // This can't be final because the ID gets set afterwards.
  private final ArrayList<Integer> m_cameraIDs;
  private final ArrayList<Integer> m_targetIDs;
  // Fiducial corner ID is valid only for AprilTags. -1 denotes the center. IDs are numbered as
  // follows:
  // * -> +X  3 ----- 2
  // * |      |       |
  // * V      |       |
  // * +Y     0 ----- 1
  private final ArrayList<Integer> m_fiducialCornerIDs;
  private final ArrayList<Pair<Double, Double>> m_measurements;
  private final ArrayList<Pair<Double, Double>> m_sigmas;

  public NTVisionMeasurement(final int id) {
    this.m_id = id;
    this.m_cameraIDs = new ArrayList<>();
    this.m_targetIDs = new ArrayList<>();
    this.m_fiducialCornerIDs = new ArrayList<>();
    this.m_measurements = new ArrayList<>();
    this.m_sigmas = new ArrayList<>();
  }

  public void setId(final int id) {
    this.m_id = id;
  }

  public void addMeasurement(
      final int cameraID,
      final int targetID,
      final Pair<Double, Double> measurement,
      final Pair<Double, Double> sigma) {
    addMeasurement(cameraID, targetID, -1, measurement, sigma);
  }

  public void addMeasurement(
      final int cameraID,
      final int targetID,
      final int fiducialCornerID,
      final Pair<Double, Double> measurement,
      final Pair<Double, Double> sigma) {
    m_cameraIDs.add(cameraID);
    m_targetIDs.add(targetID);
    m_fiducialCornerIDs.add(fiducialCornerID);
    m_measurements.add(measurement);
    m_sigmas.add(sigma);
  }

  public double[] toArray() {
    final int kDataLength = 7;
    final double[] data = new double[1 + kDataLength * m_measurements.size()];
    data[0] = (double) m_id;
    for (int i = 0; i < m_measurements.size(); i++) {
      data[kDataLength * i + 1] = (double) m_cameraIDs.get(i); // camera ID
      data[kDataLength * i + 2] = (double) m_targetIDs.get(i); // target ID
      data[kDataLength * i + 3] = (double) m_fiducialCornerIDs.get(i); // fiducial corner ID
      data[kDataLength * i + 4] = m_measurements.get(i).getFirst(); // bearing
      data[kDataLength * i + 5] = m_measurements.get(i).getSecond(); // elevation
      data[kDataLength * i + 6] = m_sigmas.get(i).getFirst(); // bearing sigma
      data[kDataLength * i + 7] = m_sigmas.get(i).getSecond(); // elevation sigma
    }
    return data;
  }
}
