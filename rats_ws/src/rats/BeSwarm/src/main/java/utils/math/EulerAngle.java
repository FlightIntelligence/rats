package utils.math;

import com.google.auto.value.AutoValue;

/**
 * A value class which stores the euler angle in three dimensions.
 *
 * @author Hoang Tung Dinh
 */
@AutoValue
public abstract class EulerAngle {

  EulerAngle() {}

  /**
   * Gets the builder of this class.
   *
   * @return a builder instance
   */
  public static Builder builder() {
    return new AutoValue_EulerAngle.Builder();
  }

  /**
   * Computes the distance between two angles. The distance will be in range [-pi, pi]. The distance
   * is negative if the {@code secondAngle} is on the left of the {@code firstAngle}.
   *
   * @param firstAngle the first angle
   * @param secondAngle the second angle
   * @return the distance between the two angles
   */
  public static double computeAngleDistance(double firstAngle, double secondAngle) {
    double distance = secondAngle - firstAngle;

    while (distance < -Math.PI) {
      distance += 2 * Math.PI;
    }

    while (distance > Math.PI) {
      distance -= 2 * Math.PI;
    }

    return distance;
  }

  /**
   * Gets the angle of the X rotation.
   *
   * @return the angle of the X rotation
   */
  public abstract double angleX();

  /**
   * Gets the angle of the Y rotation.
   *
   * @return the angle of the Y rotation
   */
  public abstract double angleY();

  /**
   * Gets the angle of the Z rotation.
   *
   * @return the angle of the Z rotation
   */
  public abstract double angleZ();

  /** The builder of the {@link EulerAngle} value class. */
  @AutoValue.Builder
  public abstract static class Builder {
    /**
     * Sets the angle of the X rotation.
     *
     * @param value the value to set
     * @return a reference to this Builder
     */
    public abstract Builder setAngleX(double value);

    /**
     * Sets the angle of the Y rotation.
     *
     * @param value the value to set
     * @return a reference to this Builder
     */
    public abstract Builder setAngleY(double value);

    /**
     * Sets the angle of the Z rotation.
     *
     * @param value the value to set
     * @return a reference to this Builder
     */
    public abstract Builder setAngleZ(double value);

    /**
     * Builds an {@link EulerAngle} instance.
     *
     * @return an {@link EulerAngle} instance
     */
    public abstract EulerAngle build();
  }
}
