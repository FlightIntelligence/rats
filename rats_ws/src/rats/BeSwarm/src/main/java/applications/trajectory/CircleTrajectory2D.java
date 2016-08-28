package applications.trajectory;

import applications.trajectory.geom.point.Point3D;
import applications.trajectory.geom.point.Point4D;
import control.Trajectory2d;

import static com.google.common.base.Preconditions.checkArgument;

/**
 * A circular trajectory in 2 dimensions of motion specified in a frequency (How many revolutions
 * per second) and a radius. Created by Kristof Coninx.
 */
final class CircleTrajectory2D extends PeriodicTrajectory implements Trajectory2d {
  private final double freq2pi;

  /**
   * Constructor
   *
   * @param radius The radius of the circle.
   * @param frequency The frequency f (amount of revolutions per second). Equals 1/period.
   * @param origin The origin point around which to form the trajectory. Represents a linear
   *     displacement.
   * @param clockwise Turn right hand if true;
   */
  private CircleTrajectory2D(
      double radius, double frequency, Point3D origin, double phase, boolean clockwise) {
    super(phase, Point4D.from(origin, 0), radius, frequency);
    this.freq2pi = frequency * TWOPI * (clockwise ? 1 : -1);
    double rfreq2pi = frequency * radius * TWOPI * (clockwise ? 1 : -1);
    checkArgument(
        Math.abs(rfreq2pi) < MAX_ABSOLUTE_VELOCITY,
        "Absolute speed should not be larger than "
            + "MAX_ABSOLUTE_VELOCITY,"
            + " which is: "
            + MAX_ABSOLUTE_VELOCITY);
  }

  static Builder builder() {
    return new Builder();
  }

  @Override
  public double getDesiredPositionAbscissa(double timeInSeconds) {
    final double currentTime = getRelativeTime(timeInSeconds);
    return getLinearDisplacement().getX()
        + getRadius() * StrictMath.cos(freq2pi * currentTime + getPhaseDisplacement());
  }

  @Override
  public double getDesiredPositionOrdinate(double timeInSeconds) {
    final double currentTime = getRelativeTime(timeInSeconds);
    return getLinearDisplacement().getY()
        + getRadius() * StrictMath.sin(freq2pi * currentTime + getPhaseDisplacement());
  }

  static class Builder {
    private double radius;
    private double frequency;
    private Point3D origin;
    private boolean clockwise;
    private double phase;

    Builder() {
      radius = 1;
      frequency = 5;
      origin = Point3D.origin();
      clockwise = true;
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
     * Default frequency = 5.
     *
     * @param frequency The frequency of the circle.
     * @return this builder
     */
    public Builder setFrequency(double frequency) {
      this.frequency = frequency;
      return this;
    }

    /**
     * Default value is Point3D.origin()
     *
     * @param origin The origin of the circle.
     * @return this builder
     */
    public Builder setOrigin(Point3D origin) {
      this.origin = origin;
      return this;
    }

    /**
     * Default value = true.
     *
     * @param clockwise whether this movement is clockwise or not.
     * @return this builder
     */
    public Builder setClockwise(boolean clockwise) {
      this.clockwise = clockwise;
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

    /** @return an instance of a circle Trajectory in 2 dimensions. */
    public CircleTrajectory2D build() {
      return new CircleTrajectory2D(radius, frequency, origin, phase, clockwise);
    }
  }
}
