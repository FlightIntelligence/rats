package services.parrot;

import control.dto.BodyFrameVelocity;
import control.dto.InertialFrameVelocity;
import control.dto.Pose;
import control.dto.Velocity;
import geometry_msgs.Twist;
import org.ros.node.topic.Publisher;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import services.Velocity4dService;
import time.TimeProvider;
import utils.math.Transformations;

import static com.google.common.base.Preconditions.checkArgument;
import static com.google.common.base.Preconditions.checkNotNull;

/**
 * A facade which provides an interface to publish velocity (a.k.a. piloting) message. There are
 * minimum and maximum values for each dimension of the velocity. If the input in one dimension of
 * the velocity is out of the min/max range, the dimension will be set to the min/max value.
 *
 * @author Hoang Tung Dinh
 */
final class ParrotVelocity4dService implements Velocity4dService {
  private static final Logger velocityLogger =
      LoggerFactory.getLogger(ParrotVelocity4dService.class.getName() + ".vel");

  private final Publisher<Twist> publisher;
  private final TimeProvider timeProvider;

  private final double minLinearX;
  private final double minLinearY;
  private final double minLinearZ;
  private final double minAngularZ;

  private final double maxLinearX;
  private final double maxLinearY;
  private final double maxLinearZ;
  private final double maxAngularZ;

  private ParrotVelocity4dService(Builder builder) {
    publisher = builder.publisher;
    timeProvider = builder.timeProvider;
    minLinearX = builder.minLinearX;
    maxLinearX = builder.maxLinearX;
    minLinearY = builder.minLinearY;
    maxLinearY = builder.maxLinearY;
    minLinearZ = builder.minLinearZ;
    maxLinearZ = builder.maxLinearZ;
    minAngularZ = builder.minAngularZ;
    maxAngularZ = builder.maxAngularZ;
    checkArgument(
        publisher.getTopicName().toString().endsWith("/cmd_vel"),
        "Topic name must be [namespace]/cmd_vel");
    checkArgument(minLinearX <= maxLinearX);
    checkArgument(minLinearY <= maxLinearY);
    checkArgument(minLinearZ <= maxLinearZ);
  }

  private static double getRefinedValue(double value, double minValue, double maxValue) {
    double refinedValue = value;

    if (value > maxValue) {
      refinedValue = maxValue;
    } else if (refinedValue < minValue) {
      refinedValue = minValue;
    }

    return refinedValue;
  }

  /**
   * The {@link Builder#publisher(Publisher)} is mandatory. All min and max values are optional.
   *
   * @return a builder instance of this facade
   */
  public static Builder builder() {
    return new Builder()
        .minLinearX(-Double.MAX_VALUE)
        .minLinearY(-Double.MAX_VALUE)
        .minLinearZ(-Double.MAX_VALUE)
        .minAngularZ(-Double.MAX_VALUE)
        .maxLinearX(Double.MAX_VALUE)
        .maxLinearY(Double.MAX_VALUE)
        .maxLinearZ(Double.MAX_VALUE)
        .maxAngularZ(Double.MAX_VALUE);
  }

  @Override
  public void sendInertialFrameVelocity(InertialFrameVelocity inertialFrameVelocity, Pose pose) {
    final BodyFrameVelocity bodyFrameVelocity =
        Transformations.inertialFrameVelocityToBodyFrameVelocity(inertialFrameVelocity, pose);

    sendBodyFrameVelocity(bodyFrameVelocity);
  }

  @Override
  public void sendBodyFrameVelocity(BodyFrameVelocity bodyFrameVelocity) {
    final BodyFrameVelocity refinedVelocity = getRefinedVelocity(bodyFrameVelocity);

    velocityLogger.trace(
        "{} {} {} {} {}",
        timeProvider.getCurrentTimeSeconds(),
        refinedVelocity.linearX(),
        refinedVelocity.linearY(),
        refinedVelocity.linearZ(),
        refinedVelocity.angularZ());

    publisher.publish(newTwistMessage(refinedVelocity));
  }

  private Twist newTwistMessage(BodyFrameVelocity refinedVelocity) {
    final Twist twist = publisher.newMessage();
    twist.getLinear().setX(refinedVelocity.linearX());
    twist.getLinear().setY(refinedVelocity.linearY());
    twist.getLinear().setZ(refinedVelocity.linearZ());
    twist.getAngular().setZ(refinedVelocity.angularZ());

    return twist;
  }

  private BodyFrameVelocity getRefinedVelocity(BodyFrameVelocity velocity) {
    return Velocity.builder()
        .setLinearX(getRefinedLinearX(velocity.linearX()))
        .setLinearY(getRefinedLinearY(velocity.linearY()))
        .setLinearZ(getRefinedLinearZ(velocity.linearZ()))
        .setAngularZ(getRefinedAngularZ(velocity.angularZ()))
        .build();
  }

  private double getRefinedLinearX(double linearX) {
    return getRefinedValue(linearX, minLinearX, maxLinearX);
  }

  private double getRefinedLinearY(double linearY) {
    return getRefinedValue(linearY, minLinearY, maxLinearY);
  }

  private double getRefinedLinearZ(double linearZ) {
    return getRefinedValue(linearZ, minLinearZ, maxLinearZ);
  }

  private double getRefinedAngularZ(double angularZ) {
    return getRefinedValue(angularZ, minAngularZ, maxAngularZ);
  }

  /** {@code VelocityPublisher} builder static inner class. */
  public static final class Builder {
    private Publisher<Twist> publisher;
    private TimeProvider timeProvider;

    private Double minLinearX;
    private Double minLinearY;
    private Double minLinearZ;
    private Double minAngularZ;

    private Double maxLinearX;
    private Double maxLinearY;
    private Double maxLinearZ;
    private Double maxAngularZ;

    private Builder() {}

    /**
     * Sets the {@code publisher} and returns a reference to this Builder so that the methods can be
     * chained together.
     *
     * @param val the {@code publisher} to set
     * @return a reference to this Builder
     */
    public Builder publisher(Publisher<Twist> val) {
      publisher = val;
      return this;
    }

    /**
     * Sets the {@code timeProvider} and returns a reference to this Builder so that the methods can
     * be chained together.
     *
     * @param val the {@code timeProvider} to set
     * @return a reference to this Builder
     */
    public Builder timeProvider(TimeProvider val) {
      timeProvider = val;
      return this;
    }

    /**
     * Sets the {@code minLinearX} and returns a reference to this Builder so that the methods can
     * be chained together.
     *
     * @param val the {@code minLinearX} to set
     * @return a reference to this Builder
     */
    public Builder minLinearX(double val) {
      minLinearX = val;
      return this;
    }

    /**
     * Sets the {@code maxLinearX} and returns a reference to this Builder so that the methods can
     * be chained together.
     *
     * @param val the {@code maxLinearX} to set
     * @return a reference to this Builder
     */
    public Builder maxLinearX(double val) {
      maxLinearX = val;
      return this;
    }

    /**
     * Sets the {@code minLinearY} and returns a reference to this Builder so that the methods can
     * be chained together.
     *
     * @param val the {@code minLinearY} to set
     * @return a reference to this Builder
     */
    public Builder minLinearY(double val) {
      minLinearY = val;
      return this;
    }

    /**
     * Sets the {@code maxLinearY} and returns a reference to this Builder so that the methods can
     * be chained together.
     *
     * @param val the {@code maxLinearY} to set
     * @return a reference to this Builder
     */
    public Builder maxLinearY(double val) {
      maxLinearY = val;
      return this;
    }

    /**
     * Sets the {@code minLinearZ} and returns a reference to this Builder so that the methods can
     * be chained together.
     *
     * @param val the {@code minLinearZ} to set
     * @return a reference to this Builder
     */
    public Builder minLinearZ(double val) {
      minLinearZ = val;
      return this;
    }

    /**
     * Sets the {@code maxLinearZ} and returns a reference to this Builder so that the methods can
     * be chained together.
     *
     * @param val the {@code maxLinearZ} to set
     * @return a reference to this Builder
     */
    public Builder maxLinearZ(double val) {
      maxLinearZ = val;
      return this;
    }

    /**
     * Sets the {@code minAngularZ} and returns a reference to this Builder so that the methods can
     * be chained together.
     *
     * @param val the {@code minAngularZ} to set
     * @return a reference to this Builder
     */
    public Builder minAngularZ(double val) {
      minAngularZ = val;
      return this;
    }

    /**
     * Sets the {@code maxAngularZ} and returns a reference to this Builder so that the methods can
     * be chained together.
     *
     * @param val the {@code maxAngularZ} to set
     * @return a reference to this Builder
     */
    public Builder maxAngularZ(double val) {
      maxAngularZ = val;
      return this;
    }

    /**
     * Returns a {@code VelocityPublisher} built from the parameters previously set.
     *
     * @return a {@code VelocityPublisher} built with parameters of this {@code
     *     VelocityPublisher.Builder}
     */
    public ParrotVelocity4dService build() {
      checkNotNull(publisher, "missing publisher");
      checkNotNull(timeProvider, "missing timeProvider");
      checkNotNull(minLinearX, "missing minLinearX");
      checkNotNull(minLinearY, "missing minLinearY");
      checkNotNull(minLinearZ, "missing minLinearZ");
      checkNotNull(minAngularZ, "missing minAngularZ");
      checkNotNull(maxLinearX, "missing maxLinearX");
      checkNotNull(maxLinearY, "missing maxLinearY");
      checkNotNull(maxLinearZ, "missing maxLinearZ");
      checkNotNull(maxAngularZ, "missing maxAngularZ");
      return new ParrotVelocity4dService(this);
    }
  }
}
