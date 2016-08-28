package applications.trajectory;

import applications.trajectory.geom.point.Point4D;
import control.Trajectory4d;

/** @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be> */
class HoldPositionTrajectory4D implements Trajectory4d {
  private final Point4D targetPoint;

  HoldPositionTrajectory4D(Point4D targetpoint) {
    this.targetPoint = targetpoint;
  }

  @Override
  public double getDesiredPositionX(double timeInSeconds) {
    return targetPoint.getX();
  }

  @Override
  public double getDesiredPositionY(double timeInSeconds) {
    return targetPoint.getY();
  }

  @Override
  public double getDesiredPositionZ(double timeInSeconds) {
    return targetPoint.getZ();
  }

  @Override
  public double getDesiredAngleZ(double timeInSeconds) {
    return targetPoint.getAngle();
  }

  @Override
  public String toString() {
    return "HoldPositionTrajectory4D{" + "target point=" + targetPoint + '}';
  }
}
