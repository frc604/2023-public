package frc.quixlib.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;

/** Describes a given vision camera pipeline configuration. */
public class PipelineConfig {
  public final Fiducial.Type fiducialType;
  public final int imageWidth; // pixels
  public final int imageHeight; // pixels
  public final Matrix<N3, N3> camIntrinsics; // used for sim only
  public final Matrix<N5, N1> distCoeffs; // used for sim only

  public PipelineConfig(
      final Fiducial.Type fiducialType,
      final int imageWidth,
      final int imageHeight,
      final Matrix<N3, N3> camIntrinsics,
      final Matrix<N5, N1> distCoeffs) {
    this.fiducialType = fiducialType;
    this.imageWidth = imageWidth;
    this.imageHeight = imageHeight;
    this.camIntrinsics = camIntrinsics;
    this.distCoeffs = distCoeffs;
  }
}
