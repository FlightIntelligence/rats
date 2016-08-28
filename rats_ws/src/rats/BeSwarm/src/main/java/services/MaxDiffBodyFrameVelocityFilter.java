package services;

import static com.google.common.base.Preconditions.checkArgument;

/**
 * A decorator filter for the {@link control.dto.BodyFrameVelocity} of a {@link Velocity4dService}.
 * The filter is to ensure that the difference between two consecutive velocity values can only be
 * at most {@code maximumDifference}. If the next velocity is different from the current velocity by
 * more than {@code maximumDifference}, the next velocity will be set to: current velocity +/-
 * {@code maximumDifference}.
 *
 * @author Hoang Tung Dinh
 */
public final class MaxDiffBodyFrameVelocityFilter extends AbstractBodyFrameVelocityFilter {

  private final double maximumDifference;

  private MaxDiffBodyFrameVelocityFilter(
      Velocity4dService velocity4dService, double maximumDifference) {
    super(velocity4dService);
    checkArgument(maximumDifference > 0, "maximumDifference must be positive.");
    this.maximumDifference = maximumDifference;
  }

  /**
   * Creates an instance of {@link MaxDiffBodyFrameVelocityFilter}.
   *
   * @param velocity4dService the velocity service to be filtered
   * @param maximumDifference the maximum difference between two consecutive velocity service
   * @return an instance of {@link MaxDiffBodyFrameVelocityFilter}
   */
  public static MaxDiffBodyFrameVelocityFilter create(
      Velocity4dService velocity4dService, double maximumDifference) {
    return new MaxDiffBodyFrameVelocityFilter(velocity4dService, maximumDifference);
  }

  @Override
  double filter(double lastVelocity, double currentVelocity) {
    if (currentVelocity > lastVelocity + maximumDifference) {
      return lastVelocity + maximumDifference;
    } else if (currentVelocity < lastVelocity - maximumDifference) {
      return lastVelocity - maximumDifference;
    } else {
      return currentVelocity;
    }
  }
}
