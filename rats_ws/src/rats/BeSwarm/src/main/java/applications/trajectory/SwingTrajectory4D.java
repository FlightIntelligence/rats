package applications.trajectory;

import applications.trajectory.geom.point.Point4D;
import control.Trajectory1d;
import control.Trajectory2d;
import control.Trajectory4d;

/**
 * Swing trajectory in 3D space as a 4D trajectory.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
class SwingTrajectory4D extends PeriodicTrajectory implements Trajectory4d {
  private final Trajectory2d swing;
  private final double yFactor;
  private final double xFactor;
  private final Trajectory1d angularMotion;

  SwingTrajectory4D(
      Point4D origin, double phase, double xzPlaneAngle, double radius, double frequency) {
    super(phase, origin, radius, frequency);
    xFactor = StrictMath.cos(xzPlaneAngle);
    yFactor = StrictMath.sin(xzPlaneAngle);
    this.swing =
        PendulumTrajectory2D.builder()
            .setRadius(radius)
            .setFrequency(frequency)
            .setOrigin(origin)
            .build();
    //keep constant yaw: use ConstantVelocityAngularTrajectory1D(0, 0)
    this.angularMotion = new LinearTrajectory1D(origin.getAngle(), 0);
  }

  static Builder builder() {
    return new Builder();
  }

  @Override
  public double getDesiredPositionX(double timeInSeconds) {
    return xFactor * swing.getDesiredPositionAbscissa(timeInSeconds);
  }

  @Override
  public double getDesiredPositionY(double timeInSeconds) {
    return yFactor * swing.getDesiredPositionAbscissa(timeInSeconds)
        + this.getLinearDisplacement().getY();
  }

  @Override
  public double getDesiredPositionZ(double timeInSeconds) {
    return swing.getDesiredPositionOrdinate(timeInSeconds);
  }

  @Override
  public double getDesiredAngleZ(double timeInSeconds) {
    return angularMotion.getDesiredPosition(timeInSeconds);
  }

  @Override
  public String toString() {
    return "CircleTrajectory4D{"
        + "origin="
        + getLinearDisplacement()
        + " frequency="
        + getFrequency()
        + ", radius="
        + getRadius()
        + ", xzPlaneAngle="
        + StrictMath.acos(xFactor)
        + '}';
  }

  /** Builder for 4D swing trajectories. */
  public static class Builder {
    private Point4D origin;
    private double xzPlaneAngle;
    private double radius;
    private double frequency;
    private double phase;

    Builder() {
      origin = Point4D.origin();
    }
    /**
     * Default value is Point3D.origin()
     *
     * @param location The origin of the circle.
     * @return this builder
     */
    public Builder setOrigin(Point4D origin) {
      this.origin = origin;
      return this;
    }

    /**
     * The plane angle.
     *
     * @param planeAngle The angle of the trajectory plane with the xz-plane.
     * @return this builder
     */
    public Builder setXzPlaneAngle(double xzPlaneAngle) {
      this.xzPlaneAngle = xzPlaneAngle;
      return this;
    }

    /**
     * Default radius = 0.
     *
     * @param radius The radius of the circle.
     * @return this builder
     */
    public Builder setRadius(double radius) {
      this.radius = radius;
      return this;
    }

    /**
     * Default frequency = 0.
     *
     * @param frequency The frequency of the circle.
     * @return this builder
     */
    public Builder setFrequency(double frequency) {
      this.frequency = frequency;
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

    /** @return A new trajectory instance that represents a swing motion. */
    public SwingTrajectory4D build() {
      return new SwingTrajectory4D(origin, phase, xzPlaneAngle, radius, frequency);
    }
  }
}
