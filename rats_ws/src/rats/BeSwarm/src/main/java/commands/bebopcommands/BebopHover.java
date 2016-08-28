package commands.bebopcommands;

import commands.AbstractParrotHover;
import control.localization.StateEstimator;
import services.Velocity4dService;
import time.TimeProvider;

/** @author Hoang Tung Dinh */
public final class BebopHover extends AbstractParrotHover {
  private BebopHover(
      double durationInSeconds,
      TimeProvider timeProvider,
      Velocity4dService velocity4dService,
      StateEstimator stateEstimator) {
    super(durationInSeconds, timeProvider, velocity4dService, stateEstimator);
  }

  public static BebopHover create(
      double durationInSeconds,
      TimeProvider timeProvider,
      Velocity4dService velocity4dService,
      StateEstimator stateEstimator) {
    return new BebopHover(durationInSeconds, timeProvider, velocity4dService, stateEstimator);
  }
}
