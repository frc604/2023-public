package frc.quixlib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Fiducials;
import java.util.ArrayList;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

public class QuixVisionSim {
  private final VisionSystemSim m_visionSim = new VisionSystemSim("main");

  public QuixVisionSim(final ArrayList<QuixVisionCamera> cameras) {
    for (var camera : cameras) {
      for (var fiducial : Fiducials.aprilTagFiducials) {
        m_visionSim.addVisionTargets(
            "apriltag",
            new VisionTargetSim(fiducial.getPose(), TargetModel.kAprilTag16h5, fiducial.id()));
      }
      m_visionSim.addCamera(camera.getCameraSim(), camera.getTransform());
    }
  }

  public void resetSimPose(final Pose2d pose) {
    m_visionSim.resetRobotPose(pose);
  }

  public void updatePose(final Pose2d pose) {
    m_visionSim.update(pose);
  }

  public Field2d getSimField() {
    return m_visionSim.getDebugField();
  }
}
