package commands.bebopcommands;

import commands.AbstractParrotTakeOff;
import services.FlyingStateService;
import services.ResetService;
import services.TakeOffService;
import services.rossubscribers.FlyingState;

/**
 * Take off command for bebop drone.
 *
 * @author Hoang Tung Dinh
 */
public final class BebopTakeOff extends AbstractParrotTakeOff {
  private BebopTakeOff(
      TakeOffService takeOffService,
      FlyingStateService flyingStateService,
      ResetService resetService) {
    super(takeOffService, flyingStateService, resetService);
  }

  /**
   * Creates an instance of the {@link BebopTakeOff} command.
   *
   * @param takeOffService the take off service
   * @param flyingStateService the flying state service
   * @param resetService the reset service
   * @return an instance of the {@link BebopTakeOff} command
   */
  public static BebopTakeOff create(
      TakeOffService takeOffService,
      FlyingStateService flyingStateService,
      ResetService resetService) {
    return new BebopTakeOff(takeOffService, flyingStateService, resetService);
  }

  @Override
  protected boolean isInHoveringState(FlyingState currentFlyingState) {
    return currentFlyingState == FlyingState.HOVERING;
  }
}
