package commands.cratescommands;

import commands.AbstractHover;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import services.HoverService;
import time.TimeProvider;

/**
 * AbstractHover command for drones in Crates simulator.
 *
 * @author Hoang Tung Dinh
 */
public final class CratesHover extends AbstractHover {

  private static final Logger logger = LoggerFactory.getLogger(CratesHover.class);
  private final HoverService hoverService;

  private CratesHover(
      double durationInSeconds, TimeProvider timeProvider, HoverService hoverService) {
    super(durationInSeconds, timeProvider);
    this.hoverService = hoverService;
  }

  /**
   * Creates an instance of {@link CratesHover}.
   *
   * @param durationInSeconds the duration that the drone will hover
   * @param timeProvider the time provider
   * @param hoverService the hover service
   * @return an instance of {@link CratesHover}
   */
  public static CratesHover create(
      double durationInSeconds, TimeProvider timeProvider, HoverService hoverService) {
    return new CratesHover(durationInSeconds, timeProvider, hoverService);
  }

  @Override
  public void execute() {
    logger.debug("Execute Crates hover command.");
    hoverService.sendHoverMessage();
  }
}
