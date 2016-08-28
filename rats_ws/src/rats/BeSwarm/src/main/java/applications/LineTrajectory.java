package applications;

import control.Trajectory1d;
import control.Trajectory4d;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/** @author mhct */
public final class LineTrajectory implements Trajectory4d {

  private static final Logger logger = LoggerFactory.getLogger(LineTrajectory.class);

  private final Trajectory1d trajectoryLinearX;
  private final Trajectory1d trajectoryLinearY;
  private final Trajectory1d trajectoryLinearZ;

  private final Trajectory1d trajectoryAngularZ = new ZeroTrajectory();

  private double startTime = -1;

  private LineTrajectory(double flightDuration, double length) {
    this.trajectoryLinearX = new ZeroTrajectory();
    this.trajectoryLinearY = new TrajectoryLinearY(flightDuration, length);
    this.trajectoryLinearZ = new ZeroTrajectory();
  }

  /**
   * Creates a line trajectory.
   *
   * @param flightDuration the duration of the trajectory
   * @param length the length of the trajectory
   * @return a line trajectory instance
   */
  public static LineTrajectory create(double flightDuration, double length) {
    return new LineTrajectory(flightDuration, length);
  }

  @Override
  public double getDesiredPositionX(double timeInSeconds) {
    return trajectoryLinearX.getDesiredPosition(timeInSeconds);
  }

  @Override
  public double getDesiredPositionY(double timeInSeconds) {
    return trajectoryLinearY.getDesiredPosition(timeInSeconds);
  }

  @Override
  public double getDesiredPositionZ(double timeInSeconds) {
    return trajectoryLinearZ.getDesiredPosition(timeInSeconds);
  }

  @Override
  public double getDesiredAngleZ(double timeInSeconds) {
    return trajectoryAngularZ.getDesiredPosition(timeInSeconds);
  }

  private final class TrajectoryLinearY implements Trajectory1d {

    private final double flightDuration;
    private final double length;

    public TrajectoryLinearY(double flightDuration, double length) {
      this.flightDuration = flightDuration;
      this.length = length;
    }

    @Override
    public double getDesiredPosition(double timeInSeconds) {
      if (startTime < 0) {
        startTime = timeInSeconds;
      }
      final double currentTime = timeInSeconds - startTime;

      if (currentTime >= flightDuration) {
        return -length;
      } else {
        double position = 0 - (currentTime % flightDuration) * length / flightDuration;
        if (logger.isInfoEnabled()) {
          logger.info("Desired position" + position);
        }

        return position;
      }
    }
  }

  private final class ZeroTrajectory implements Trajectory1d {

    private ZeroTrajectory() {}

    @Override
    public double getDesiredPosition(double timeInSeconds) {
      if (startTime < 0) {
        startTime = timeInSeconds;
      }

      return 0.0;
    }
  }
}
