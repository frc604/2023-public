package frc.quixlib.devices;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Timestamp;
import java.util.function.Function;

/** A wrapper aound Phoenix6's StatusSignal that handles unit conversions. */
public class QuixStatusSignal {
  private final StatusSignal<Double> m_statusSignal;
  private final Function<Double, Double> m_fromNativeUnits;

  public QuixStatusSignal(final StatusSignal<Double> statusSignal) {
    m_statusSignal = statusSignal.clone();
    m_fromNativeUnits = (Double value) -> value;
  }

  public QuixStatusSignal(
      final StatusSignal<Double> statusSignal, final Function<Double, Double> fromNativeUnits) {
    m_statusSignal = statusSignal.clone();
    m_fromNativeUnits = fromNativeUnits;
  }

  public StatusCode setUpdateFrequency(double frequencyHz, double timeoutSeconds) {
    return m_statusSignal.setUpdateFrequency(frequencyHz, timeoutSeconds);
  }

  public double getAppliedUpdateFrequency() {
    return m_statusSignal.getAppliedUpdateFrequency();
  }

  public void refresh() {
    m_statusSignal.refresh();
  }

  public void waitForUpdate(final double timeoutSec) {
    m_statusSignal.waitForUpdate(timeoutSec);
  }

  public double getValue() {
    return m_fromNativeUnits.apply(m_statusSignal.getValue());
  }

  public Timestamp getTimestamp() {
    return m_statusSignal.getTimestamp();
  }

  public static StatusCode refreshAll(final QuixStatusSignal... quixSignals) {
    return waitForAll(0.0, quixSignals);
  }

  public static StatusCode waitForAll(
      final double timeoutSec, final QuixStatusSignal... quixSignals) {
    final BaseStatusSignal[] signals = new BaseStatusSignal[quixSignals.length];
    for (int i = 0; i < quixSignals.length; i++) {
      signals[i] = quixSignals[i].m_statusSignal;
    }
    return BaseStatusSignal.waitForAll(timeoutSec, signals);
  }

  public static double getLatencyCompensatedValue(
      final QuixStatusSignal signal, final QuixStatusSignal signalSlope) {
    return signal.getValue() + (signalSlope.getValue() * signal.getTimestamp().getLatency());
  }
}
