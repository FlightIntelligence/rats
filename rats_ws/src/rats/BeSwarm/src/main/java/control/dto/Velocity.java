package control.dto;

import com.google.auto.value.AutoValue;
import geometry_msgs.Twist;

/** @author Hoang Tung Dinh */
@AutoValue
public abstract class Velocity implements InertialFrameVelocity, BodyFrameVelocity {
  protected Velocity() {}

  /**
   * Gets a builder of this class.
   *
   * @return a builder instance
   */
  public static Builder builder() {
    return new AutoValue_Velocity.Builder();
  }

  /**
   * Creates a zero velocity.
   *
   * @return a velocity with all components equal zero
   */
  public static Velocity createZeroVelocity() {
    return builder().setLinearX(0).setLinearY(0).setLinearZ(0).setAngularZ(0).build();
  }

  /**
   * Converts a Twist velocity (given in NED coordinates) a {@link Velocity} instance.
   *
   * @param twist the twist velocity
   * @return a {@link Velocity} instance.
   */
  public static Velocity createLocalVelocityFrom(Twist twist) {
    final double twistX = twist.getLinear().getX();
    final double twistY = twist.getLinear().getY();
    final double twistZ = twist.getLinear().getZ();
    final double twistAngularZ = twist.getAngular().getZ();

    return builder()
        .setLinearX(twistX)
        .setLinearY(twistY)
        .setLinearZ(twistZ)
        .setAngularZ(twistAngularZ)
        .build();
  }

  @Override
  public abstract double linearX();

  @Override
  public abstract double linearY();

  @Override
  public abstract double linearZ();

  @Override
  public abstract double angularZ();

  /** Builds a {@link Velocity} instance. */
  @AutoValue.Builder
  public abstract static class Builder {

    /**
     * Sets the velocity in the x coordinate.
     *
     * @param value the value of the velocity in the x coordinate
     * @return a reference to this Builder
     */
    public abstract Builder setLinearX(double value);

    /**
     * Sets the velocity in the y coordinate.
     *
     * @param value the value of the velocity in the y coordinate
     * @return a reference to this Builder
     */
    public abstract Builder setLinearY(double value);

    /**
     * Sets the velocity in the z coordinate.
     *
     * @param value the value of the velocity in the z coordinate
     * @return a reference to this Builder
     */
    public abstract Builder setLinearZ(double value);

    /**
     * Sets the velocity of the Z rotation (the yaw).
     *
     * @param value the value of the velocity of the Z rotation (the yaw)
     * @return a reference to this Builder
     */
    public abstract Builder setAngularZ(double value);

    /**
     * Builds a {@link Velocity} instance.
     *
     * @return a {@link Velocity} instance
     */
    public abstract Velocity build();
  }
}
