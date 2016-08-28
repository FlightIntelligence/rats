package applications.trajectory;

import applications.trajectory.geom.point.Point4D;
import control.Trajectory1d;
import control.Trajectory2d;

import static com.google.common.base.Preconditions.checkArgument;

/**
 * A Pendulum trajectory in 2 dimensions of motion specified in a frequency (How many revolutions
 * per second) and a radius (the length of the virtual pendulum string). The pendulum trajectory is
 * a half circle but with with modeled transition from kinetic to potential energy and back.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
public final class PendulumTrajectory2D extends PeriodicTrajectory implements Trajectory2d {
  private final Trajectory1d linearMovement;
  private final Trajectory1d pendulumOrdinate;

  /**
   * Constructor
   *
   * @param radius The length of the virtual pendulum string (or radius).
   * @param frequency The frequency f (amount of revolutions per second). Equals 1/period.
   */
  private PendulumTrajectory2D(double radius, double frequency, Point4D origin, double phase) {
    super(HALFPI * 3 + phase, origin, radius, frequency);
    this.linearMovement = new PendulumSwingTrajectory1D(origin, radius, frequency, phase);
    checkArgument(
        Math.abs(radius * frequency) < MAX_ABSOLUTE_VELOCITY / PISQUARED,
        "Absolute speed should not be larger than "
            + "MAX_ABSOLUTE_VELOCITY,"
            + " which is: "
            + MAX_ABSOLUTE_VELOCITY);
    this.pendulumOrdinate = new PendulumOrdinate();
  }

  /** @return Builder for 2D pendulum trajectories. */
  public static Builder builder() {
    return new Builder();
  }

  @Override
  public double getDesiredPositionAbscissa(double timeInSeconds) {
    return this.linearMovement.getDesiredPosition(timeInSeconds);
  }

  @Override
  public double getDesiredPositionOrdinate(double timeInSeconds) {
    return this.pendulumOrdinate.getDesiredPosition(timeInSeconds);
  }

  /** Builder class for 2D pendulum trajectories. */
  public static final class Builder {
    private double radius;
    private double frequency;
    private Point4D origin;
    private double phase;

    private Builder() {
      origin = Point4D.origin();
      radius = 1;
      frequency = 5;
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
     * Default value is Point4D.origin()
     *
     * @param origin The origin of the circle.
     * @return this builder
     */
    public Builder setOrigin(Point4D origin) {
      this.origin = origin;
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

    /** @return The 2d Pendulum motion instance. */
    public PendulumTrajectory2D build() {
      return new PendulumTrajectory2D(radius, frequency, origin, phase);
    }
  }

  private class PendulumOrdinate implements Trajectory1d {
    @Override
    public double getDesiredPosition(double timeInSeconds) {
      setStartTime(timeInSeconds);

      final double currentTime = timeInSeconds - getStartTime();
      return getLinearDisplacement().getZ()
          + getRadius()
              * StrictMath.sin(
                  TrajectoryUtils.pendulumAngleFromTime(currentTime, getFrequency())
                      + getPhaseDisplacement());
    }
  }
}
