package applications.trajectory;

import applications.trajectory.geom.point.Point3D;
import applications.trajectory.geom.point.Point4D;
import control.Trajectory1d;
import control.Trajectory4d;

import static com.google.common.base.Preconditions.checkArgument;

/**
 * A 4D motion primitive for circle motion in 3 dimensions. This circle is defined by a displacement
 * parameter for relocating the origin, a radius, a frequency and an angle of the circle towards the
 * xy-plane. This angle represents the angle between the xy-plane and the plane in which the circle
 * movement is performed.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
final class CircleTrajectory4D extends PeriodicTrajectory implements Trajectory4d {
  private final Point3D location;
  private final CircleTrajectory2D xycircle;
  private final double scaleFactor;
  private final Trajectory1d angularMotion;

  private CircleTrajectory4D(
      Point3D location, double phase, double radius, double frequency, double planeAngle) {
    this(
        location,
        phase,
        radius,
        frequency,
        planeAngle,
        new ConstantVelocityAngularTrajectory1D(frequency, phase));
  }

  private CircleTrajectory4D(
      Point3D location,
      double phase,
      double radius,
      double frequency,
      double planeAngle,
      Trajectory1d yawTrajectory) {
    super(phase, Point4D.from(location, 0), radius, frequency);
    this.location = location;
    this.scaleFactor = StrictMath.sin(planeAngle);
    this.xycircle =
        CircleTrajectory2D.builder()
            .setRadius(radius)
            .setFrequency(frequency)
            .setOrigin(location)
            .setPhase(phase)
            .build();
    this.angularMotion = yawTrajectory;
  }

  private CircleTrajectory4D(
      Point3D location,
      double phase,
      double radius,
      double frequency,
      double planeAngle,
      double constantYawAngle) {
    this(
        location,
        phase,
        radius,
        frequency,
        planeAngle,
        new LinearTrajectory1D(constantYawAngle, 0));
  }

  static Builder builder() {
    return new Builder();
  }

  @Override
  public double getDesiredPositionX(double timeInSeconds) {
    return xycircle.getDesiredPositionAbscissa(timeInSeconds);
  }

  @Override
  public double getDesiredPositionY(double timeInSeconds) {
    return (1 - scaleFactor)
            * (xycircle.getDesiredPositionOrdinate(timeInSeconds) - location.getY())
        + location.getY();
  }

  @Override
  public double getDesiredPositionZ(double timeInSeconds) {
    return scaleFactor * (xycircle.getDesiredPositionOrdinate(timeInSeconds) - location.getY())
        + location.getZ();
  }

  @Override
  public double getDesiredAngleZ(double timeInSeconds) {
    return angularMotion.getDesiredPosition(timeInSeconds);
  }

  @Override
  public String toString() {
    return "CircleTrajectory4D{"
        + "origin="
        + location
        + " frequency="
        + getFrequency()
        + ", radius="
        + getRadius()
        + ", planeAngle="
        + StrictMath.atan(scaleFactor)
        + '}';
  }

  /** Builder class for 4D circle trajectories. */
  public static final class Builder {
    private Point3D location;
    private double radius;
    private double frequency;
    private double planeAngle;
    private double phase;
    private boolean angularMovement;
    private double yawDirection;

    private Builder() {
      location = Point3D.origin();
      radius = 1;
      frequency = 0.05;
      angularMovement = true;
    }

    /**
     * Default value is Point3D.origin()
     *
     * @param location The origin of the circle.
     * @return this builder
     */
    public Builder setLocation(Point3D location) {
      this.location = location;
      return this;
    }

    /**
     * Default radius = 1.
     *
     * @param radius The radius of the circle.
     * @return this builder
     */
    public Builder setRadius(double radius) {
      this.radius = radius;
      return this;
    }

    /**
     * Default frequency = 0.05.
     *
     * @param frequency The frequency of the circle.
     * @return this builder
     */
    public Builder setFrequency(double frequency) {
      this.frequency = frequency;
      return this;
    }

    /**
     * The plane angle.
     *
     * @param planeAngle The angle of the trajectory plane with the z-plane.
     * @return this builder
     */
    public Builder setPlaneAngle(double planeAngle) {
      this.planeAngle = planeAngle;
      return this;
    }

    /**
     * Default value = 0.
     *
     * @param phase the phase displacement of the movement.
     * @return this builder
     */
    public Builder setPhase(double phase) {
      this.phase = phase;
      return this;
    }

    /**
     * @param yawDirection The orientation of the drone to hold constant for this trajectory.
     * @return this builder instance.
     */
    public Builder fixYawAt(double yawDirection) {
      checkArgument(Math.abs(yawDirection) < Math.PI * 2);
      this.angularMovement = false;
      this.yawDirection = yawDirection;
      return this;
    }

    /** @return return a CircleTrajectory instance configured by this builder object. */
    public CircleTrajectory4D build() {
      if (angularMovement) {
        return new CircleTrajectory4D(location, phase, radius, frequency, planeAngle);
      }
      return new CircleTrajectory4D(location, phase, radius, frequency, planeAngle, yawDirection);
    }
  }
}
