package commands;

import control.dto.FlipDirection;
import services.FlipService;

/**
 * Flip command for parrot drones.
 *
 * @author Hoang Tung Dinh
 */
public abstract class AbstractParrotFlip implements Command {

  private final FlipService flipService;
  private final FlipDirection flipDirection;

  protected AbstractParrotFlip(FlipService flipService, FlipDirection flipDirection) {
    this.flipService = flipService;
    this.flipDirection = flipDirection;
  }

  @Override
  public void execute() {
    flipService.sendFlipMessage(flipDirection);
  }
}
