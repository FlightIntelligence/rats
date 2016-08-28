package control.dto;

import com.google.auto.value.AutoValue;
import geometry_msgs.PoseStamped;
import utils.math.Transformations;

/**
 * A value class which stores the pose of the drone.
 *
 * @author Hoang Tung Dinh
 * @author mhct
 */
@AutoValue
public abstract class Pose {
  /** Default epsilon used for pose comparison. */
  private static final double EPS = 0.001;

  Pose() {}

  /**
   * Gets a builder of this class.
   *
   * @return a builder instance
   */
  public static Builder builder() {
    return new AutoValue_Pose.Builder();
  }

  /**
   * Creates a {@link Pose} instance from a {@link PoseStamped} instance.
   *
   * @param poseStamped the pose stamped used to create the pose
   * @return a {@link Pose} instance equivalent to the {@code poseStamped}
   */
  public static Pose create(PoseStamped poseStamped) {
    return Pose.builder()
        .setX(poseStamped.getPose().getPosition().getX())
        .setY(poseStamped.getPose().getPosition().getY())
        .setZ(poseStamped.getPose().getPosition().getZ())
        .setYaw(
            Transformations.quaternionToEulerAngle(poseStamped.getPose().getOrientation()).angleZ())
        .build();
  }

  /**
   * Gets a zero pose.
   *
   * @return a pose with all components equal to zero
   */
  public static Pose createZeroPose() {
    return Pose.builder().setX(0).setY(0).setZ(0).setYaw(0).build();
  }

  /**
   * Checks whether two poses are similar with in a tolerance.
   *
   * @param p1 the first pose
   * @param p2 the second pose
   * @return true if two poses are similar within a tolerance
   */
  public static boolean areSamePoseWithinEps(Pose p1, Pose p2) {
    return StrictMath.abs(p1.x() - p2.x()) < EPS
        && StrictMath.abs(p1.y() - p2.y()) < EPS
        && StrictMath.abs(p1.z() - p2.z()) < EPS
        && StrictMath.abs(p1.yaw() - p2.yaw()) < EPS;
  }

  /**
   * Gets the x position.
   *
   * @return the x position
   */
  public abstract double x();

  /**
   * Gets the y position.
   *
   * @return the y position
   */
  public abstract double y();

  /**
   * Gets the z position.
   *
   * @return the z position
   */
  public abstract double z();

  /**
   * Gets the yaw position.
   *
   * @return the yaw position
   */
  public abstract double yaw();

  /** Builds a {@link Pose} instance. */
  @AutoValue.Builder
  public abstract static class Builder {
    /**
     * Sets the x position.
     *
     * @return a reference to this Builder
     */
    public abstract Builder setX(double value);

    /**
     * Sets the y position.
     *
     * @return a reference to this Builder
     */
    public abstract Builder setY(double value);

    /**
     * Sets the z position.
     *
     * @return a reference to this Builder
     */
    public abstract Builder setZ(double value);

    /**
     * Sets the yaw position.
     *
     * @return a reference to this Builder
     */
    public abstract Builder setYaw(double value);

    /**
     * Builds a {@link Pose} instance.
     *
     * @return a built {@link Pose} instance
     */
    public abstract Pose build();
  }
}
