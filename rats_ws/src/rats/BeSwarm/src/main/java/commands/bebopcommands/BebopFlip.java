package commands.bebopcommands;

import commands.AbstractParrotFlip;
import control.dto.FlipDirection;
import services.FlipService;

/**
 * Flip command for bebop drone.
 *
 * @author Hoang Tung Dinh
 */
public final class BebopFlip extends AbstractParrotFlip {
  private BebopFlip(FlipService flipService, FlipDirection flipDirection) {
    super(flipService, flipDirection);
  }

  /**
   * Create a flipping command.
   *
   * @param flipService the flip publisher
   * @param flipDirection the flipping flipDirection
   * @return an instance of the flipping command
   */
  public static BebopFlip create(FlipService flipService, FlipDirection flipDirection) {
    return new BebopFlip(flipService, flipDirection);
  }
}
