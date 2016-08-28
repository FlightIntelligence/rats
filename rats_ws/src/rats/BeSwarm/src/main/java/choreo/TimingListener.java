package choreo;

/**
 * Listener interface for timing events.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
interface TimingListener {
  /**
   * Notify this listener of an event ocurring at the specified timing.
   *
   * @param timeInSeconds The time this event occurred.
   */
  void notifyNewTiming(double timeInSeconds);
}
