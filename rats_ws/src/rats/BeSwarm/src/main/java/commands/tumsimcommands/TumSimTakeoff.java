package commands.tumsimcommands;

import commands.AbstractParrotTakeOff;
import services.FlyingStateService;
import services.ResetService;
import services.TakeOffService;
import services.rossubscribers.FlyingState;

/**
 * Take off command for drones in Tum simulator.
 *
 * @author Hoang Tung Dinh
 */
public final class TumSimTakeoff extends AbstractParrotTakeOff {
  private TumSimTakeoff(
      TakeOffService takeOffService,
      FlyingStateService flyingStateService,
      ResetService resetService) {
    super(takeOffService, flyingStateService, resetService);
  }

  /**
   * Creates an instance of the {@link TumSimTakeoff} command.
   *
   * @param takeOffService the take off service
   * @param flyingStateService the flying state service
   * @param resetService the reset service
   * @return an instance of the {@link TumSimTakeoff} command
   */
  public static TumSimTakeoff create(
      TakeOffService takeOffService,
      FlyingStateService flyingStateService,
      ResetService resetService) {
    return new TumSimTakeoff(takeOffService, flyingStateService, resetService);
  }

  @Override
  protected boolean isInHoveringState(FlyingState currentFlyingState) {
    return currentFlyingState == FlyingState.FLYING;
  }
}
