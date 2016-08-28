package commands.bebopcommands;

import commands.AbstractParrotLand;
import services.FlyingStateService;
import services.LandService;

/**
 * Land command for Bebop.
 *
 * @author Hoang Tung Dinh
 */
public final class BebopLand extends AbstractParrotLand {
  private BebopLand(LandService landService, FlyingStateService flyingStateService) {
    super(landService, flyingStateService);
  }

  /**
   * Creates a land command.
   *
   * @param landService the land service of the drone
   * @param flyingStateService the flying state service of the drone
   * @return a land command
   */
  public static BebopLand create(LandService landService, FlyingStateService flyingStateService) {
    return new BebopLand(landService, flyingStateService);
  }
}
