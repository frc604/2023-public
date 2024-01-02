package frc.quixlib.vision;

/** Describes a given vision camera pipeline configuration. */
public class PipelineConfig {
  public final Fiducial.Type fiducialType;
  public final int imageWidth;
  public final int imageHeight;

  public PipelineConfig(
      final Fiducial.Type fiducialType, final int imageWidth, final int imageHeight) {
    this.fiducialType = fiducialType;
    this.imageWidth = imageWidth;
    this.imageHeight = imageHeight;
  }
}
