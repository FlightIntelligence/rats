package applications.trajectory;

import applications.trajectory.geom.point.Point4D;

/**
 * Abstract class for periodic trajectory commonalities.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
public abstract class PeriodicTrajectory extends BasicTrajectory {

  protected static final double TWOPI = Math.PI * 2;
  protected static final double HALFPI = Math.PI / 2;
  protected static final double PISQUARED = Math.PI * Math.PI;
  protected static final double TWOPISQUARED = 2 * PISQUARED;
  private final double radius;
  private final double frequency;
  private final double phaseDisplacement;

  protected PeriodicTrajectory() {
    this(0, Point4D.origin(), 1, 1);
  }

  protected PeriodicTrajectory(
      double phase, Point4D displacement, double radius, double frequency) {
    super(displacement);
    this.phaseDisplacement = phase;
    this.radius = radius;
    this.frequency = frequency;
  }

  protected PeriodicTrajectory(double phase) {
    this(phase, Point4D.origin(), 1, 1);
  }

  /** @return Displacement in phase in radians. */
  protected double getPhaseDisplacement() {
    return phaseDisplacement;
  }

  public double getRadius() {
    return radius;
  }

  public double getFrequency() {
    return frequency;
  }

  @Override
  public String toString() {
    return "PeriodicTrajectory{" + "frequency=" + frequency + ", radius=" + radius + '}';
  }
}
