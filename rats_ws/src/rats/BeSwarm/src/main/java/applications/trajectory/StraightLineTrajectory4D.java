package applications.trajectory;

import applications.trajectory.geom.point.Point3D;
import applications.trajectory.geom.point.Point4D;
import control.FiniteTrajectory4d;
import control.Trajectory4d;

import static com.google.common.base.Preconditions.checkArgument;

/**
 * Trajectory represent a straight line in space between two given points at a given speed. Once the
 * destination point has been reached, the trajectory enforces to hold position at the destination
 * point. The optional parameter velocityCutoffTimePercentage represents the percentage of the
 * tragjectory ( in time) to perform at the given velocity. The default value is 1, representing the
 * trajectory will reach its destination with a positive velocity in the direction of travel. This
 * will cause overshooting behavior. Choose a value < 1 to trigger the controller to start braking
 * sooner.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
class StraightLineTrajectory4D extends BasicTrajectory implements FiniteTrajectory4d {
  private final Point4D srcpoint;
  private final Point4D targetpoint;
  private final double velocity;
  private final Trajectory4d moveTraj;
  private final Trajectory4d holdTraj;
  private final double endTime;
  private final double totalDistance;
  private Trajectory4d currentTraj;

  StraightLineTrajectory4D(Point4D srcpoint, Point4D targetpoint, double velocity) {
    this(srcpoint, targetpoint, velocity, 1);
  }

  StraightLineTrajectory4D(
      Point4D srcpoint, Point4D targetpoint, double velocity, double velocityCutoffTimePercentage) {
    this.srcpoint = srcpoint;
    this.targetpoint = targetpoint;
    this.velocity = velocity;
    checkArgument(velocity > 0, "The provided velocity should be strictly greater than 0.");
    checkArgument(
        velocityCutoffTimePercentage <= 1 && velocityCutoffTimePercentage > 0,
        "Velocity cutoff percentage should represent a percantage between 0 and 1.");
    checkArgument(
        velocity <= BasicTrajectory.MAX_ABSOLUTE_VELOCITY,
        "The provided velocity should be smaller than BasicTrajectory" + ".MAX_ABSOLUTE_VELOCITY");
    Point4D diff = Point4D.minus(targetpoint, srcpoint);
    this.totalDistance = Point3D.project(diff).norm();
    double speed = velocity;
    checkArgument(totalDistance > 0, "Distance to travel cannot be zero.");
    this.endTime = totalDistance / speed;
    Point4D speedComponent =
        Point4D.create(
            velocity * (diff.getX() / totalDistance),
            velocity * (diff.getY() / totalDistance),
            velocity * (diff.getZ() / totalDistance),
            diff.getAngle() / endTime);
    this.holdTraj = new HoldPositionTrajectory4D(targetpoint);
    this.moveTraj =
        new HoldPositionForwarder(srcpoint, speedComponent, endTime * velocityCutoffTimePercentage);
    this.currentTraj = moveTraj;
  }

  @Override
  public double getDesiredPositionX(double timeInSeconds) {
    final double currentTime = getRelativeTime(timeInSeconds);
    return getCurrentTrajectory().getDesiredPositionX(currentTime);
  }

  @Override
  public double getDesiredPositionY(double timeInSeconds) {
    final double currentTime = getRelativeTime(timeInSeconds);
    return getCurrentTrajectory().getDesiredPositionY(currentTime);
  }

  @Override
  public double getDesiredPositionZ(double timeInSeconds) {
    final double currentTime = getRelativeTime(timeInSeconds);
    return getCurrentTrajectory().getDesiredPositionZ(currentTime);
  }

  @Override
  public double getDesiredAngleZ(double timeInSeconds) {
    final double currentTime = getRelativeTime(timeInSeconds);
    return getCurrentTrajectory().getDesiredAngleZ(currentTime);
  }

  protected Trajectory4d getCurrentTrajectory() {
    return currentTraj;
  }

  @Override
  public String toString() {
    return "StraightLineTrajectory4D{"
        + "velocity="
        + getVelocity()
        + ", src point="
        + getSrcpoint()
        + ", target point="
        + getTargetpoint()
        + '}';
  }

  public double getVelocity() {
    return velocity;
  }

  public Point4D getSrcpoint() {
    return srcpoint;
  }

  public Point4D getTargetpoint() {
    return targetpoint;
  }

  @Override
  public double getTrajectoryDuration() {
    return this.endTime;
  }

  public final double getTotalDistance() {
    return totalDistance;
  }

  private class HoldPositionForwarder extends Trajectory4DForwardingDecorator {
    private final double endTime;

    HoldPositionForwarder(Point4D srcComp, Point4D speedComp, double endTime) {
      super(new LinearTrajectory4D(srcComp, speedComp));
      this.endTime = endTime;
    }

    @Override
    protected void positionDelegate(double timeInSeconds) {
      if (timeInSeconds >= endTime) {
        setHoldPosition(true);
      }
    }

    private void setHoldPosition(boolean shouldHold) {
      if (shouldHold) {
        currentTraj = holdTraj;
      } else {
        currentTraj = moveTraj;
      }
    }
  }
}
