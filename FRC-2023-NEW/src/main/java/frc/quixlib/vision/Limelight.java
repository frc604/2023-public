package frc.quixlib.vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.Timer;
import frc.quixlib.math.CameraMathUtils;
import java.util.ArrayList;
import java.util.List;
import org.json.JSONArray;
import org.json.JSONObject;
import org.photonvision.simulation.PhotonCameraSim;

public class Limelight implements QuixVisionCamera {
  private final NetworkTable m_limelightTable;
  private final Transform3d m_transform;
  private final double m_fovWidth;
  private final double m_fovHeight;
  private final PipelineConfig[] m_pipelineConfigs;

  // Pub/Sub
  private final IntegerPublisher m_pipelinePub;
  private final IntegerPublisher m_tvPub;
  private final IntegerPublisher m_tidPub;
  private final DoublePublisher m_txPub;
  private final DoublePublisher m_tyPub;
  private final DoublePublisher m_tlPub;
  private final StringPublisher m_jsonPub;

  private final IntegerSubscriber m_pipelineSub;
  private final IntegerSubscriber m_tvSub;
  private final IntegerSubscriber m_tidSub;
  private final DoubleSubscriber m_txSub;
  private final DoubleSubscriber m_tySub;
  private final DoubleSubscriber m_taSub;
  private final DoubleSubscriber m_tsSub;
  private final DoubleSubscriber m_tlSub;
  private final StringSubscriber m_jsonSub;

  // Magic number from LL website.
  private static final double kImageCaptureLatencyMs = 11.0; // ms

  public Limelight(
      final String tableName,
      final Transform3d transform,
      final double fovWidth,
      final double fovHeight,
      final PipelineConfig[] pipielineConfigs) {
    m_limelightTable = NetworkTableInstance.getDefault().getTable(tableName);
    m_limelightTable
        .getIntegerTopic("stream")
        .publish()
        .set(2); // USB webcam with primary camera in PIP.
    m_transform = transform;
    m_fovWidth = fovWidth;
    m_fovHeight = fovHeight;
    m_pipelineConfigs = pipielineConfigs;

    // Setup topic pub/sub
    m_pipelinePub = m_limelightTable.getIntegerTopic("pipeline").publish();
    m_tvPub = m_limelightTable.getIntegerTopic("tv").publish();
    m_tidPub = m_limelightTable.getIntegerTopic("tid").publish();
    m_txPub = m_limelightTable.getDoubleTopic("tx").publish();
    m_tyPub = m_limelightTable.getDoubleTopic("ty").publish();
    m_tlPub = m_limelightTable.getDoubleTopic("tl").publish();
    m_jsonPub = m_limelightTable.getStringTopic("json").publish();

    m_pipelineSub = m_limelightTable.getIntegerTopic("pipeline").subscribe(0);
    m_tvSub = m_limelightTable.getIntegerTopic("tv").subscribe(0);
    m_tidSub = m_limelightTable.getIntegerTopic("tid").subscribe(-1);
    m_txSub = m_limelightTable.getDoubleTopic("tx").subscribe(0.0);
    m_tySub = m_limelightTable.getDoubleTopic("ty").subscribe(0.0);
    m_taSub = m_limelightTable.getDoubleTopic("ta").subscribe(0.0);
    m_tsSub = m_limelightTable.getDoubleTopic("ts").subscribe(0.0);
    m_tlSub = m_limelightTable.getDoubleTopic("tl").subscribe(0.0);
    m_jsonSub = m_limelightTable.getStringTopic("json").subscribe("");

    setPipelineIndex(0);
  }

  public PhotonCameraSim getCameraSim() {
    // TODO: Use PhotonVision to sim Limelight
    return null;
  }

  public void setPipelineIndex(final int index) {
    if (index > m_pipelineConfigs.length) {
      System.out.println("Invalid pipeline index: " + index);
      return;
    }
    m_pipelinePub.set(index);
  }

  public PipelineConfig getPipelineConfig() {
    return m_pipelineConfigs[(int) m_pipelineSub.get()];
  }

  public Transform3d getTransform() {
    return m_transform;
  }

  public Fiducial.Type getFiducialType() {
    return getPipelineConfig().fiducialType;
  }

  public PipelineVisionPacket getLatestMeasurement() {
    if (m_tvSub.get() == 0) {
      // No targets.
      return new PipelineVisionPacket(false, new Target(), new ArrayList<Target>(), -1);
    }

    final PipelineConfig pipelineConfig = getPipelineConfig();

    // Populate all targets from JSON.
    final List<Target> targets = new ArrayList<>();
    final JSONObject json = new JSONObject(m_jsonSub.get());
    final JSONObject results = json.getJSONObject("Results");
    final JSONArray retro = results.getJSONArray("Retro");
    final JSONArray fiducial = results.getJSONArray("Fiducial");
    for (int i = 0; i < retro.length(); i++) {
      targets.add(
          new Target(
              -1,
              retro.getJSONObject(i).getDouble("tx"),
              retro.getJSONObject(i).getDouble("ty"),
              retro.getJSONObject(i).getDouble("ta"),
              0.0));
    }
    for (int i = 0; i < fiducial.length(); i++) {
      final ArrayList<Pair<Double, Double>> corners = new ArrayList<>();
      for (int j = 0; j < fiducial.getJSONObject(i).getJSONArray("pts").length(); j++) {
        final JSONObject pt = fiducial.getJSONObject(i).getJSONArray("pts").getJSONObject(j);
        final var yawPitch =
            CameraMathUtils.XYToYawPitchWithHeightAndFOV(
                pt.getDouble("x"),
                pt.getDouble("y"),
                pipelineConfig.imageWidth,
                pipelineConfig.imageHeight,
                m_fovWidth,
                m_fovHeight);
        corners.add(
            new Pair<>(Math.toDegrees(yawPitch.getFirst()), Math.toDegrees(yawPitch.getSecond())));
      }

      targets.add(
          new Target(
              fiducial.getJSONObject(i).getInt("fid"),
              fiducial.getJSONObject(i).getDouble("tx"),
              fiducial.getJSONObject(i).getDouble("ty"),
              fiducial.getJSONObject(i).getDouble("ta"),
              0.0,
              corners));
    }

    return new PipelineVisionPacket(
        true,
        new Target(
            (int) m_tidSub.get(), m_txSub.get(), m_tySub.get(), m_taSub.get(), m_tsSub.get()),
        targets,
        Timer.getFPGATimestamp() - (m_tlSub.get() + kImageCaptureLatencyMs) * 1e-3);
  }

  /*
  public void updateSim(final Pose3d cameraPose, final Fiducial[] fiducials) {
    final PipelineConfig pipelineConfig = getPipelineConfig();
    final ArrayList<PhotonTrackedTarget> targets =
        QuixVisionCamera.simulateTrackedTargets(
            cameraPose,
            fiducials,
            pipelineConfig.imageWidth,
            pipelineConfig.imageHeight,
            m_fovHeight,
            m_fovWidth);

    // Determine best target
    double bestDistance = Double.POSITIVE_INFINITY;
    PhotonTrackedTarget bestTarget = null;
    for (final var target : targets) {
      final double dist =
          Math.sqrt(target.getYaw() * target.getYaw() + target.getPitch() * target.getPitch());
      if (dist >= bestDistance) {
        continue;
      }
      bestDistance = dist;
      bestTarget = target;
    }

    // Populate NetworkTables based on best target.
    m_tvPub.set(targets.isEmpty() ? 0 : 1);
    // Simulation has zero latency, so we need to cancel out the image capture latency.
    m_tlPub.set(-kImageCaptureLatencyMs);
    if (bestTarget != null) {
      m_tidPub.set(bestTarget.getFiducialId());
      m_txPub.set(bestTarget.getYaw());
      m_tyPub.set(bestTarget.getPitch());
    }

    // Populate NetworkTables json dump.
    final JSONArray retro = new JSONArray();
    final JSONArray fiducial = new JSONArray();
    for (final var target : targets) {
      final JSONObject t = new JSONObject();
      t.put("tx", target.getYaw());
      t.put("ty", target.getPitch());
      t.put("ta", target.getArea());

      if (target.getFiducialId() == -1) {
        retro.put(t);
      } else {
        t.put("fid", target.getFiducialId());
        // AprilTag corners
        JSONArray cornerPoints = new JSONArray();
        for (var corner : target.getDetectedCorners()) {
          JSONObject pt = new JSONObject();
          pt.put("x", corner.x);
          pt.put("y", corner.y);
          cornerPoints.put(pt);
        }
        t.put("pts", cornerPoints);
        fiducial.put(t);
      }
    }

    final JSONObject results = new JSONObject();
    results.put("v", targets.isEmpty() ? 0 : 1);
    // Simulation has zero latency, so we need to cancel out the image capture latency.
    results.put("tl", -kImageCaptureLatencyMs);
    results.put("Retro", retro);
    results.put("Fiducial", fiducial);

    final JSONObject json = new JSONObject();
    json.put("Results", results);
    m_jsonPub.set(json.toString());
  }
   */
}
