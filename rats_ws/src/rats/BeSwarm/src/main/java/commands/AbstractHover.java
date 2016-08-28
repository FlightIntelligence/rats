package commands;

import time.TimeProvider;

import static com.google.common.base.Preconditions.checkArgument;

/**
 * AbstractHover command.
 *
 * @author Hoang Tung Dinh
 */
// TODO: Refactor this class to use the optical flow sensor only
public abstract class AbstractHover implements Command {
  private final double durationInSeconds;
  private final TimeProvider timeProvider;

  protected AbstractHover(double durationInSeconds, TimeProvider timeProvider) {
    checkArgument(
        durationInSeconds > 0,
        String.format("Duration must be a positive value, but it is %f", durationInSeconds));
    this.durationInSeconds = durationInSeconds;
    this.timeProvider = timeProvider;
  }

  protected final double getDurationInSeconds() {
    return durationInSeconds;
  }

  protected final TimeProvider getTimeProvider() {
    return timeProvider;
  }
}
