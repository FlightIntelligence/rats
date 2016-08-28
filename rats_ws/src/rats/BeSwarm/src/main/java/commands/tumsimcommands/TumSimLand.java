package commands.tumsimcommands;

import commands.AbstractParrotLand;
import services.FlyingStateService;
import services.LandService;

/**
 * Land command for drones in Tum simulator.
 *
 * @author Hoang Tung Dinh
 */
public final class TumSimLand extends AbstractParrotLand {
  private TumSimLand(LandService landService, FlyingStateService flyingStateService) {
    super(landService, flyingStateService);
  }

  /**
   * Creates a land command.
   *
   * @param landService the land service of the drone
   * @param flyingStateService the flying state service of the drone
   * @return a land command
   */
  public static TumSimLand create(LandService landService, FlyingStateService flyingStateService) {
    return new TumSimLand(landService, flyingStateService);
  }
}
