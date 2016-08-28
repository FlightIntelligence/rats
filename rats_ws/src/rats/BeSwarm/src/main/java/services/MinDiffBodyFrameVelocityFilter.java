package services;

import static com.google.common.base.Preconditions.checkArgument;

/**
 * A decorator filter for the {@link control.dto.BodyFrameVelocity} of a {@link Velocity4dService}.
 * The filter is to remove small noises when sending the velocity to the drone. If the difference
 * between two consecutive velocity values is insignificant, the second velocity value will be set
 * to the first velocity value.
 *
 * @author Hoang Tung Dinh
 */
public final class MinDiffBodyFrameVelocityFilter extends AbstractBodyFrameVelocityFilter {

  private final double minimumDifference;

  private MinDiffBodyFrameVelocityFilter(
      Velocity4dService velocity4dService, double minimumDifference) {
    super(velocity4dService);
    checkArgument(minimumDifference > 0, "minimumDifference must be positive.");
    this.minimumDifference = minimumDifference;
  }

  /**
   * Creates an instance of {@link MinDiffBodyFrameVelocityFilter}.
   *
   * @param velocity4dService the velocity service to be filtered
   * @param minimumDifference the minimum difference between two consecutive velocity values
   * @return an instance of {@link control.dto.BodyFrameVelocity}
   */
  public static MinDiffBodyFrameVelocityFilter create(
      Velocity4dService velocity4dService, double minimumDifference) {
    return new MinDiffBodyFrameVelocityFilter(velocity4dService, minimumDifference);
  }

  @Override
  double filter(double lastVelocity, double currentVelocity) {
    if (currentVelocity > lastVelocity - minimumDifference
        && currentVelocity < lastVelocity + minimumDifference) {
      return lastVelocity;
    } else {
      return currentVelocity;
    }
  }
}
