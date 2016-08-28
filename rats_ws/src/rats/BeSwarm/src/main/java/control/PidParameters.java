package control;

import com.google.auto.value.AutoValue;

/**
 * A value class which stores parameters for a pid controller.
 *
 * @author Hoang Tung Dinh
 */
@AutoValue
public abstract class PidParameters {

  /**
   * {@link Builder#kp}, {@link Builder#kd()}, {@link Builder#ki} are mandatory. All other
   * parameters are optional.
   *
   * @return a builder
   */
  public static Builder builder() {
    return new AutoValue_PidParameters.Builder()
        .setLagTimeInSeconds(0)
        .setMinVelocity(-Double.MAX_VALUE)
        .setMaxVelocity(Double.MAX_VALUE)
        .setMinIntegralError(-Double.MAX_VALUE)
        .setMaxIntegralError(Double.MAX_VALUE);
  }

  /**
   * Gets the kp.
   *
   * @return the kp
   */
  public abstract double kp();

  /**
   * Gets the kd.
   *
   * @return the kd
   */
  public abstract double kd();

  /**
   * Gets the ki.
   *
   * @return the ki
   */
  public abstract double ki();

  /**
   * Gets the lag time in seconds.
   *
   * @return the lag time in seconds
   */
  public abstract double lagTimeInSeconds();

  /**
   * Gets the minimum velocity.
   *
   * @return the minimum velocity
   */
  public abstract double minVelocity();

  /**
   * Gets the maximum velocity.
   *
   * @return the maximum velocity
   */
  public abstract double maxVelocity();

  /**
   * Gets the minimum integral error.
   *
   * @return the minimum integral error
   */
  public abstract double minIntegralError();

  /**
   * Gets the maximum integral error.
   *
   * @return the maximum integral error
   */
  public abstract double maxIntegralError();

  /** Builds an {@link PidParameters} instance. */
  @AutoValue.Builder
  public abstract static class Builder {
    /**
     * Set the kp.
     *
     * @return a reference to this Builder
     */
    public abstract Builder setKp(double value);

    /**
     * Set the kd.
     *
     * @return a reference to this Builder
     */
    public abstract Builder setKd(double value);

    /**
     * Set the ki.
     *
     * @return a reference to this Builder
     */
    public abstract Builder setKi(double value);

    /**
     * Set the kp.
     *
     * @return a reference to this Builder
     */
    public abstract Builder setLagTimeInSeconds(double value);

    /**
     * Set the lag time in seconds.
     *
     * @return a reference to this Builder
     */
    public abstract Builder setMinVelocity(double value);

    /**
     * Set the maximum velocity.
     *
     * @return a reference to this Builder
     */
    public abstract Builder setMaxVelocity(double value);

    /**
     * Set the minimum integral error.
     *
     * @return a reference to this Builder
     */
    public abstract Builder setMinIntegralError(double value);

    /**
     * Set the maximum integral error.
     *
     * @return a reference to this Builder
     */
    public abstract Builder setMaxIntegralError(double value);

    /**
     * Builds a {@link PidParameters} instance.
     *
     * @return a built {@link PidParameters} instance
     */
    public abstract PidParameters build();
  }
}
