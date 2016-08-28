package commands.tumsimcommands;

import commands.AbstractParrotHover;
import control.localization.StateEstimator;
import services.Velocity4dService;
import time.TimeProvider;

/**
 * AbstractHover command for the ArDrone in the Tum simulator.
 *
 * @author Hoang Tung Dinh
 */
public final class TumSimHover extends AbstractParrotHover {
  private TumSimHover(
      double durationInSeconds,
      TimeProvider timeProvider,
      Velocity4dService velocity4dService,
      StateEstimator stateEstimator) {
    super(durationInSeconds, timeProvider, velocity4dService, stateEstimator);
  }

  public static TumSimHover create(
      double durationInSeconds,
      TimeProvider timeProvider,
      Velocity4dService velocity4dService,
      StateEstimator stateEstimator) {
    return new TumSimHover(durationInSeconds, timeProvider, velocity4dService, stateEstimator);
  }
}
