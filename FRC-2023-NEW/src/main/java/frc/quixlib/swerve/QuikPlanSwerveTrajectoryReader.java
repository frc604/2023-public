package frc.quixlib.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map.Entry;
import java.util.concurrent.ConcurrentSkipListMap;
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class QuikPlanSwerveTrajectoryReader {
  private final List<PolynomialSplineFunction> m_interpolatedStateData = new ArrayList<>();
  private final ConcurrentSkipListMap<Double, QuikPlanAction> m_timeToActionMap =
      new ConcurrentSkipListMap<>();
  private final LinearInterpolator m_interpolator = new LinearInterpolator();
  private final SendableChooser<File> m_chooser = new SendableChooser<>();
  private final Field2d m_fieldViz;

  private File m_selectedFile = null;
  private double m_totalTime = 0.0;

  public QuikPlanSwerveTrajectoryReader(final Field2d fieldViz) {
    m_fieldViz = fieldViz;

    // Read trajectory files
    final List<File> files = new ArrayList<>();
    try {
      Files.walk(Filesystem.getDeployDirectory().toPath())
          .map(Path::toFile)
          .filter((file) -> file.getName().endsWith(".csv"))
          .forEach(files::add);
    } catch (IOException e) {
      e.printStackTrace();
    }

    // Add option for each file sorted by file name
    files.sort(null);
    for (File file : files) {
      m_chooser.addOption(file.getName(), file);
    }

    // Select and load first file by default
    if (files.size() > 0) {
      final File firstFile = files.get(0);
      m_chooser.setDefaultOption(firstFile.getName(), firstFile);
      SmartDashboard.putData(m_chooser);
      loadSelectedFile();
    }
  }

  /** Loads the selected file if it has changed. */
  public void loadSelectedFile() {
    final File selectedFile = m_chooser.getSelected();
    if (selectedFile == m_selectedFile) {
      return;
    }
    loadTrajectory(selectedFile);
    m_selectedFile = selectedFile;
    SmartDashboard.putString("Selected Trajectory", selectedFile.getName());
  }

  private void loadTrajectory(final File file) {
    clearLoadedTrajectory();

    // Temp data.
    final ArrayList<Double> timeData = new ArrayList<>();
    final ArrayList<ArrayList<Double>> stateData = new ArrayList<ArrayList<Double>>();
    for (int i = 0; i < CSVState.values().length; i++) {
      stateData.add(new ArrayList<Double>());
    }

    // Read time and state data.
    try (final BufferedReader br = new BufferedReader(new FileReader(file))) {
      String line;
      while ((line = br.readLine()) != null) {
        if (line.isEmpty()) {
          continue;
        }
        final double[] data =
            Arrays.stream(line.split(",")).mapToDouble(Double::parseDouble).toArray();
        timeData.add(data[0]);
        for (int i = 1; i < 7; i++) {
          stateData.get(i - 1).add(data[i]);
        }
        final int actionType = (int) data[7];
        if (actionType != 0) { // Only store non-null actions.
          m_timeToActionMap.put(
              data[0], new QuikPlanAction(actionType, (int) data[8], (int) data[9]));
        }
      }
    } catch (Exception e) {
      e.printStackTrace();
      return;
    }

    // Save interpolated state data.
    for (final List<Double> dataList : stateData) {
      m_interpolatedStateData.add(
          m_interpolator.interpolate(
              timeData.stream().mapToDouble(d -> d).toArray(),
              dataList.stream().mapToDouble(d -> d).toArray()));
    }
    m_totalTime = timeData.get(timeData.size() - 1);

    // Update viz
    final double kNumSamples = 100;
    final var vizPoses = new ArrayList<Pose2d>();
    for (int i = 0; i < kNumSamples; i++) {
      double t = getTotalTime() * i / kNumSamples;
      vizPoses.add(getState(t).pose);
    }
    m_fieldViz.getObject("traj").setPoses(vizPoses);
  }

  private void clearLoadedTrajectory() {
    m_totalTime = 0.0;
    m_interpolatedStateData.clear();
    m_timeToActionMap.clear();
  }

  private List<Double> getInterpolatedCSVState(final double time) {
    final List<Double> state = new ArrayList<>();
    for (PolynomialSplineFunction psf : m_interpolatedStateData) {
      state.add(psf.value(Math.min(time, getTotalTime())));
    }
    return state;
  }

  public QuikplanTrajectoryState getState(final double time) {
    final var state = getInterpolatedCSVState(time);
    return new QuikplanTrajectoryState(
        new Pose2d(
            state.get(CSVState.x.index),
            state.get(CSVState.y.index),
            new Rotation2d(state.get(CSVState.theta.index))),
        state.get(CSVState.dx.index),
        state.get(CSVState.dy.index),
        state.get(CSVState.dTheta.index));
  }

  public Pose2d getInitialPose() {
    return getState(0.0).pose;
  }

  public double getTotalTime() {
    return m_totalTime;
  }

  /** Returns the latest action less than or equal to the given time. */
  public Entry<Double, QuikPlanAction> getAction(final double time) {
    return m_timeToActionMap.floorEntry(time);
  }

  private enum CSVState {
    // Common across years
    x(0),
    y(1),
    theta(2),
    dx(3),
    dy(4),
    dTheta(5);

    public final int index;

    private CSVState(int index) {
      this.index = index;
    }
  }

  public class QuikplanTrajectoryState {
    public final Pose2d pose;
    public final double xVel;
    public final double yVel;
    public final double thetaVel;

    public QuikplanTrajectoryState(
        final Pose2d pose, final double xVel, final double yVel, final double thetaVel) {
      this.pose = pose;
      this.xVel = xVel;
      this.yVel = yVel;
      this.thetaVel = thetaVel;
    }
  }

  public class QuikPlanAction {
    public final int actionType;
    public final int gridID;
    public final int nodeID;

    public QuikPlanAction(final int actionType, final int gridID, final int nodeID) {
      this.actionType = actionType;
      this.gridID = gridID;
      this.nodeID = nodeID;
    }
  }
}
