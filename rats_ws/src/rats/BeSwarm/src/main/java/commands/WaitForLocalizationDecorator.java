package commands;

import com.google.common.base.Optional;
import control.dto.DroneStateStamped;
import control.localization.StateEstimator;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.annotation.Nullable;
import java.util.concurrent.TimeUnit;

/**
 * Waits until receiving a valid pose and then executes a {@link Command}
 *
 * @author Hoang Tung Dinh
 */
public final class WaitForLocalizationDecorator implements Command {

  private static final Logger logger = LoggerFactory.getLogger(WaitForLocalizationDecorator.class);
  private static final int SLEEP_DURATION_IN_MILLISECONDS = 50;
  private final StateEstimator stateEstimator;
  private final Command command;
  @Nullable private DroneStateStamped lastReceivedPose;

  private WaitForLocalizationDecorator(StateEstimator stateEstimator, Command command) {
    this.stateEstimator = stateEstimator;
    this.command = command;
  }

  /**
   * Creates an instance of this class.
   *
   * @param stateEstimator the state estimator of the drone
   * @param command the command to be decorated
   * @return a decorated command which will wait until receiving a valid pose from the {@code
   *     stateEstimator} and the execute the {@code command}
   */
  public static WaitForLocalizationDecorator create(
      StateEstimator stateEstimator, Command command) {
    return new WaitForLocalizationDecorator(stateEstimator, command);
  }

  @Override
  public void execute() {
    logger.debug("Start waiting for localization.");

    while (true) {
      final Optional<DroneStateStamped> droneStateStampedOptional =
          stateEstimator.getCurrentState();

      if (droneStateStampedOptional.isPresent()) {
        final DroneStateStamped droneStateStamped = droneStateStampedOptional.get();
        if (lastReceivedPose == null) {
          lastReceivedPose = droneStateStamped;
        } else {
          if (droneStateStamped.getTimeStampInSeconds()
              != lastReceivedPose.getTimeStampInSeconds()) {
            break;
          }
        }
      }

      try {
        TimeUnit.MILLISECONDS.sleep(SLEEP_DURATION_IN_MILLISECONDS);
      } catch (InterruptedException e) {
        logger.debug("Sleep is interrupted in land command.", e);
        Thread.currentThread().interrupt();
      }
    }

    command.execute();
  }
}
