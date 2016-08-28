package applications.trajectory;

import control.Trajectory1d;

/**
 * A linear trajectory in one dimension.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
class LinearTrajectory1D extends BasicTrajectory implements Trajectory1d {
  private final double startComp;
  private final double speedComp;

  LinearTrajectory1D(double startComponent, double speedComponent) {
    this.startComp = startComponent;
    this.speedComp = speedComponent;
  }

  @Override
  public double getDesiredPosition(double timeInSeconds) {
    setStartTime(timeInSeconds);
    final double currentTime = timeInSeconds - getStartTime();
    return startComp + speedComp * currentTime;
  }
}
