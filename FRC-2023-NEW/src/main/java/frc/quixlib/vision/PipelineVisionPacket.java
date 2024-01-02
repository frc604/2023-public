package frc.quixlib.vision;

import java.util.List;

/** A data class for a pipeline packet. */
public class PipelineVisionPacket {
  private final boolean m_hasTargets;
  private final Target m_bestTarget;
  private final List<Target> m_targets;
  private final double m_captureTimestamp;

  public PipelineVisionPacket(
      final boolean hasTargets,
      final Target bestTarget,
      final List<Target> targets,
      final double captureTimestamp) {
    m_hasTargets = hasTargets;
    m_bestTarget = bestTarget;
    m_targets = targets;
    m_captureTimestamp = captureTimestamp;
  }

  /**
   * If the vision packet has valid targets.
   *
   * @return if targets are found.
   */
  public boolean hasTargets() {
    return m_hasTargets;
  }

  /**
   * Gets best target.
   *
   * @return the best target.
   */
  public Target getBestTarget() {
    return m_bestTarget;
  }

  /**
   * Gets targets.
   *
   * @return the targets.
   */
  public List<Target> getTargets() {
    return m_targets;
  }

  /**
   * Gets the capture timestamp in seconds. -1 if the result has no timestamp set.
   *
   * @return the latency in seconds.
   */
  public double getCaptureTimestamp() {
    return m_captureTimestamp;
  }
}
