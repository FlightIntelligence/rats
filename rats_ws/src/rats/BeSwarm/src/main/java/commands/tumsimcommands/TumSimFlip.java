package commands.tumsimcommands;

import commands.AbstractParrotFlip;
import control.dto.FlipDirection;
import services.FlipService;

/**
 * Flip command for drones in Tum simulator.
 *
 * @author Hoang Tung Dinh
 */
public final class TumSimFlip extends AbstractParrotFlip {
  private TumSimFlip(FlipService flipService, FlipDirection flipDirection) {
    super(flipService, flipDirection);
  }

  /**
   * Create a flipping command.
   *
   * @param flipService the flip publisher
   * @param flipDirection the flipping flipDirection
   * @return an instance of the flipping command
   */
  public static TumSimFlip create(FlipService flipService, FlipDirection flipDirection) {
    return new TumSimFlip(flipService, flipDirection);
  }
}
