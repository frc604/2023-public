package frc.quixlib.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Robot;
import java.util.ArrayList;
import org.photonvision.PhotonCamera;
import org.photonvision.SimPhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionCamera implements QuixVisionCamera {
  private final PhotonCamera m_camera;
  private final SimPhotonCamera m_sim;
  private final Matrix<N4, N4> m_transform;
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
      final Matrix<N4, N4> transform,
      final double fovWidth,
      final double fovHeight,
      final PipelineConfig[] pipielineConfigs) {
    m_camera = new PhotonCamera(cameraName);
    m_sim = new SimPhotonCamera(cameraName);
    m_transform = transform;
    m_fovWidth = fovWidth;
    m_fovHeight = fovHeight;
    m_pipelineConfigs = pipielineConfigs;

    setPipelineIndex(0);
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

  public Matrix<N4, N4> getTransform() {
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

  public void updateSim(final Matrix<N4, N4> fieldCameraT, final Fiducial[] fiducials) {
    final PipelineConfig pipelineConfig = getPipelineConfig();
    final ArrayList<PhotonTrackedTarget> targets =
        QuixVisionCamera.simulateTrackedTargets(
            fieldCameraT,
            fiducials,
            pipelineConfig.imageWidth,
            pipelineConfig.imageHeight,
            m_fovHeight,
            m_fovWidth);
    m_sim.submitProcessedFrame(0.0, targets);
  }
}
