package commands;

import control.dto.BodyFrameVelocity;
import control.dto.Velocity;
import control.localization.StateEstimator;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import services.Velocity4dService;
import time.TimeProvider;

import java.util.concurrent.TimeUnit;

/**
 * AbstractHover command for parrot drones.
 *
 * @author Hoang Tung Dinh
 */
public abstract class AbstractParrotHover extends AbstractHover {

  private static final Logger logger = LoggerFactory.getLogger(AbstractParrotHover.class);

  private final Velocity4dService velocity4dService;

  protected AbstractParrotHover(
      double durationInSeconds,
      TimeProvider timeProvider,
      Velocity4dService velocity4dService,
      StateEstimator stateEstimator) {
    super(durationInSeconds, timeProvider);
    this.velocity4dService = velocity4dService;
  }

  @Override
  public final void execute() {
    logger.debug("Execute AbstractParrotHover command.");
    final BodyFrameVelocity bodyFrameVelocity = Velocity.createZeroVelocity();
    velocity4dService.sendBodyFrameVelocity(bodyFrameVelocity);
    // TODO: test the drone to see if we need the HOVER flying state feedback here

    try {
      TimeUnit.MILLISECONDS.sleep((long) (getDurationInSeconds() * 1000));
    } catch (InterruptedException e) {
      logger.debug("AbstractParrotHover command is interrupted.", e);
      Thread.currentThread().interrupt();
    }
  }
}
