package services;

import static com.google.common.base.Preconditions.checkArgument;

/**
 * A cascade filter for a {@link Velocity4dService}. The filter ensures that: (1) If the two
 * consecutive velocity values are different by less than {@code delta}, the next velocity will be
 * set to the current velocity. (2) If the two consecutive velocity values are different by more
 * than {@code delta}, the next velocity will be set to (current velocity +/- delta).
 *
 * @author Hoang Tung Dinh
 */
public final class CascadeBodyFrameVelocityFilter extends AbstractBodyFrameVelocityFilter {

  private final double delta;

  private CascadeBodyFrameVelocityFilter(Velocity4dService velocity4dService, double delta) {
    super(velocity4dService);
    checkArgument(delta > 0, "delta must be positive.");
    this.delta = delta;
  }

  /**
   * Creates an instance of {@link CascadeBodyFrameVelocityFilter}.
   *
   * @param velocity4dService the velocity service to be filtered
   * @param delta the filtering value. The filter ensures that: (1) If the two consecutive velocity
   *     values are different by less than {@code delta}, the next velocity will be set to the
   *     current velocity. (2) If the two consecutive velocity values are different by more than
   *     {@code delta}, the next velocity will be set to (current velocity +/- delta)
   * @return an instance of {@link CascadeBodyFrameVelocityFilter}
   */
  public static CascadeBodyFrameVelocityFilter create(
      Velocity4dService velocity4dService, double delta) {
    return new CascadeBodyFrameVelocityFilter(velocity4dService, delta);
  }

  @Override
  double filter(double lastVelocity, double currentVelocity) {
    double filteredVelocity;
    if (currentVelocity > lastVelocity - delta && currentVelocity < lastVelocity + delta) {
      filteredVelocity = lastVelocity;
    } else {
      filteredVelocity = currentVelocity;
    }

    if (filteredVelocity > lastVelocity + delta) {
      filteredVelocity = lastVelocity + delta;
    } else if (filteredVelocity < lastVelocity - delta) {
      filteredVelocity = lastVelocity - delta;
    }

    return filteredVelocity;
  }
}
