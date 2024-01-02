package frc.quixlib.vision;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.simulation.PhotonCameraSim;

public interface QuixVisionCamera {
  /** Returns the simulated camera object. */
  public PhotonCameraSim getCameraSim();

  /** Returns the latest measurement. */
  public PipelineVisionPacket getLatestMeasurement();

  /** Select the active pipeline index. */
  public void setPipelineIndex(int index);

  /** Get the active pipeline config. */
  public PipelineConfig getPipelineConfig();

  /** Returns the robot-to-camera transform. */
  public Transform3d getTransform();

  /** Returns the type of fiducials this camera is tracking. */
  public Fiducial.Type getFiducialType();
}
