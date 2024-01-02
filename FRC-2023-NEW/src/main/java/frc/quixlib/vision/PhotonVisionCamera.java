package frc.quixlib.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Robot;
import java.util.ArrayList;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

public class PhotonVisionCamera implements QuixVisionCamera {
  private final PhotonCamera m_camera;
  private final PhotonCameraSim m_sim;
  private final Transform3d m_transform;
  private final double m_fovWidth;
  private final double m_fovHeight;
  private final PipelineConfig[] m_pipelineConfigs;

  // Magic number from LL website.
  // TODO: Figure out why sim is "early" by one period.
  // TODO: Figure out why this is negative.
  private static final double kImageCaptureLatencySeconds =
      Robot.isSimulation() ? -TimedRobot.kDefaultPeriod : -0.02;

  public PhotonVisionCamera(
      final String cameraName,
      final Transform3d transform,
      final double fovWidth,
      final double fovHeight,
      final PipelineConfig[] pipelineConfigs) {
    m_camera = new PhotonCamera(cameraName);
    m_transform = transform;
    m_fovWidth = fovWidth;
    m_fovHeight = fovHeight;
    m_pipelineConfigs = pipelineConfigs;

    setPipelineIndex(0);

    // Setup sim
    final SimCameraProperties props = new SimCameraProperties();
    // TODO: Sim more than one pipeline.
    props.setCalibration(
        pipelineConfigs[0].imageWidth,
        pipelineConfigs[0].imageHeight,
        pipelineConfigs[0].camIntrinsics,
        pipelineConfigs[0].distCoeffs);
    props.setCalibError(0.25, 0.08);
    props.setFPS(20.0);
    props.setAvgLatencyMs(35.0);
    props.setLatencyStdDevMs(5.0);
    m_sim = new PhotonCameraSim(m_camera, props);
    m_sim.enableDrawWireframe(true);
  }

  public PhotonCameraSim getCameraSim() {
    return m_sim;
  }

  public void setPipelineIndex(final int index) {
    if (index > m_pipelineConfigs.length) {
      System.out.println("Invalid pipeline index: " + index);
      return;
    }
    m_camera.setPipelineIndex(index);
  }

  public PipelineConfig getPipelineConfig() {
    return m_pipelineConfigs[m_camera.getPipelineIndex()];
  }

  public Transform3d getTransform() {
    return m_transform;
  }

  public Fiducial.Type getFiducialType() {
    return getPipelineConfig().fiducialType;
  }

  public PipelineVisionPacket getLatestMeasurement() {
    final var result = m_camera.getLatestResult();
    final boolean hasTargets = result.hasTargets();
    if (!hasTargets) {
      return new PipelineVisionPacket(false, null, null, -1);
    }

    final PipelineConfig pipelineConfig = getPipelineConfig();
    // Attempt to use the camera matrix and distortion coefficients if they are available, otherwise
    // fall back to estimate based on image size and FOV.
    final boolean hasCalibration =
        m_camera.getCameraMatrix().isPresent() && m_camera.getDistCoeffs().isPresent();
    final Target bestTarget =
        hasCalibration
            ? Target.fromPhotonTrackedTargetWithCalibration(
                result.getBestTarget(),
                m_camera.getCameraMatrix().get(),
                m_camera.getDistCoeffs().get())
            : Target.fromPhotonTrackedTargetWithoutCalibration(
                result.getBestTarget(),
                pipelineConfig.imageWidth,
                pipelineConfig.imageHeight,
                m_fovWidth,
                m_fovHeight);
    final ArrayList<Target> targets = new ArrayList<Target>();
    for (var t : result.getTargets()) {
      targets.add(
          hasCalibration
              ? Target.fromPhotonTrackedTargetWithCalibration(
                  t, m_camera.getCameraMatrix().get(), m_camera.getDistCoeffs().get())
              : Target.fromPhotonTrackedTargetWithoutCalibration(
                  t,
                  pipelineConfig.imageWidth,
                  pipelineConfig.imageHeight,
                  m_fovWidth,
                  m_fovHeight));
    }

    return new PipelineVisionPacket(
        hasTargets,
        bestTarget,
        targets,
        result.getTimestampSeconds() - kImageCaptureLatencySeconds);
  }
}
