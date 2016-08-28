package commands;

import com.google.common.base.Optional;
import commands.schedulers.PeriodicTaskRunner;
import control.Trajectory4d;
import control.dto.DroneStateStamped;
import control.localization.StateEstimator;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import time.TimeProvider;

/**
 * Follow trajectory command.
 *
 * @author Hoang Tung Dinh
 */
public abstract class AbstractFollowTrajectory implements Command {

  private static final Logger logger = LoggerFactory.getLogger(AbstractFollowTrajectory.class);
  private static final Logger poseLogger =
      LoggerFactory.getLogger(AbstractFollowTrajectory.class.getName() + ".poselogger");
  private static final Logger velocityLogger =
      LoggerFactory.getLogger(AbstractFollowTrajectory.class.getName() + ".velocitylogger");

  private final StateEstimator stateEstimator;
  private final Trajectory4d trajectory4d;
  private final double durationInSeconds;
  private final double controlRateInSeconds;
  private final double droneStateLifeDurationInSeconds;
  private final TimeProvider timeProvider;

  protected AbstractFollowTrajectory(
      StateEstimator stateEstimator,
      Trajectory4d trajectory4d,
      double durationInSeconds,
      double controlRateInSeconds,
      double droneStateLifeDurationInSeconds,
      TimeProvider timeProvider) {
    this.stateEstimator = stateEstimator;
    this.trajectory4d = trajectory4d;
    this.durationInSeconds = durationInSeconds;
    this.controlRateInSeconds = controlRateInSeconds;
    this.droneStateLifeDurationInSeconds = droneStateLifeDurationInSeconds;
    this.timeProvider = timeProvider;
  }

  @Override
  public final void execute() {
    logger.debug("Execute follow trajectory command: {}", trajectory4d);
    final Runnable controlLoop = createControlLoop();
    PeriodicTaskRunner.run(controlLoop, controlRateInSeconds, durationInSeconds);
  }

  protected abstract AbstractControlLoop createControlLoop();

  protected abstract class AbstractControlLoop implements Runnable {
    private final double startTimeInSeconds;
    private final int stateLifeDurationInNumberOfControlLoops;
    // assigned to 0
    private int counter;
    private double lastTimeStamp = Double.MIN_VALUE;

    protected AbstractControlLoop() {
      this.startTimeInSeconds = timeProvider.getCurrentTimeSeconds();
      this.stateLifeDurationInNumberOfControlLoops =
          (int) Math.ceil(droneStateLifeDurationInSeconds / controlRateInSeconds);
    }

    @Override
    public void run() {
      logger.trace("Start a control loop.");
      final Optional<DroneStateStamped> currentState = stateEstimator.getCurrentState();
      if (!currentState.isPresent()) {
        logger.trace("Cannot get state. Haven't sent any velocity.");
        return;
      }

      setCounter(currentState.get());

      if (counter >= stateLifeDurationInNumberOfControlLoops) {
        logger.debug("Pose is outdated. Stop sending velocity.");
      } else {
        logger.trace("Got pose and velocity. Start computing the next velocity response.");
        final double currentTimeInSeconds =
            timeProvider.getCurrentTimeSeconds() - startTimeInSeconds;
        computeAndSendResponse(currentTimeInSeconds, currentState.get());
        logDroneState(currentState.get(), currentTimeInSeconds);
      }
    }

    protected abstract void computeAndSendResponse(
        double currentTimeInSeconds, DroneStateStamped currentState);

    private void logDroneState(DroneStateStamped currentState, double currentTimeInSeconds) {
      final double systemTimeInSeconds = timeProvider.getCurrentTimeSeconds();
      poseLogger.trace(
          "{} {} {} {} {} {} {} {} {}",
          systemTimeInSeconds,
          currentState.pose().x(),
          currentState.pose().y(),
          currentState.pose().z(),
          currentState.pose().yaw(),
          trajectory4d.getDesiredPositionX(currentTimeInSeconds),
          trajectory4d.getDesiredPositionY(currentTimeInSeconds),
          trajectory4d.getDesiredPositionZ(currentTimeInSeconds),
          trajectory4d.getDesiredAngleZ(currentTimeInSeconds));

      final double deltaTimeInSeconds = 0.1;
      final double desiredVelocityX =
          (trajectory4d.getDesiredPositionX(currentTimeInSeconds + deltaTimeInSeconds)
                  - trajectory4d.getDesiredPositionX(currentTimeInSeconds))
              / deltaTimeInSeconds;
      final double desiredVelocityY =
          (trajectory4d.getDesiredPositionY(currentTimeInSeconds + deltaTimeInSeconds)
                  - trajectory4d.getDesiredPositionY(currentTimeInSeconds))
              / deltaTimeInSeconds;
      final double desiredVelocityZ =
          (trajectory4d.getDesiredPositionZ(currentTimeInSeconds + deltaTimeInSeconds)
                  - trajectory4d.getDesiredPositionZ(currentTimeInSeconds))
              / deltaTimeInSeconds;
      final double desiredVelocityYaw =
          (trajectory4d.getDesiredAngleZ(currentTimeInSeconds + deltaTimeInSeconds)
                  - trajectory4d.getDesiredAngleZ(currentTimeInSeconds))
              / deltaTimeInSeconds;

      velocityLogger.trace(
          "{} {} {} {} {} {} {} {} {}",
          systemTimeInSeconds,
          currentState.inertialFrameVelocity().linearX(),
          currentState.inertialFrameVelocity().linearY(),
          currentState.inertialFrameVelocity().linearZ(),
          currentState.inertialFrameVelocity().angularZ(),
          desiredVelocityX,
          desiredVelocityY,
          desiredVelocityZ,
          desiredVelocityYaw);
    }

    private void setCounter(DroneStateStamped currentState) {
      final double currentTimeStamp = currentState.getTimeStampInSeconds();
      if (currentTimeStamp == lastTimeStamp) {
        counter++;
      } else {
        counter = 0;
        lastTimeStamp = currentTimeStamp;
      }
    }
  }

  protected abstract static class AbstractBuilder<T extends AbstractBuilder> {

    private static final double DEFAULT_CONTROL_RATE_IN_SECONDS = 0.05;
    private static final double DEFAULT_DRONE_STATE_LIFE_DURATION_IN_SECONDS = 0.1;

    protected StateEstimator stateEstimator;
    protected Trajectory4d trajectory4d;
    protected Double durationInSeconds;
    protected Double controlRateInSeconds;
    protected Double droneStateLifeDurationInSeconds;
    protected TimeProvider timeProvider;

    protected AbstractBuilder() {
      controlRateInSeconds = DEFAULT_CONTROL_RATE_IN_SECONDS;
      droneStateLifeDurationInSeconds = DEFAULT_DRONE_STATE_LIFE_DURATION_IN_SECONDS;
    }

    protected abstract T self();

    /**
     * Sets the {@code stateEstimator} and returns a reference to this Builder so that the methods
     * can be chained together.
     *
     * @param val the {@code stateEstimator} to set
     * @return a reference to this Builder
     */
    public T withStateEstimator(StateEstimator val) {
      stateEstimator = val;
      return self();
    }

    /**
     * Sets the {@code trajectory4d} and returns a reference to this Builder so that the methods can
     * be chained together.
     *
     * @param val the {@code trajectory4d} to set
     * @return a reference to this Builder
     */
    public T withTrajectory4d(Trajectory4d val) {
      trajectory4d = val;
      return self();
    }

    /**
     * Sets the {@code durationInSeconds} and returns a reference to this Builder so that the
     * methods can be chained together.
     *
     * @param val the {@code durationInSeconds} to set
     * @return a reference to this Builder
     */
    public T withDurationInSeconds(double val) {
      durationInSeconds = val;
      return self();
    }

    /**
     * Sets the {@code controlRateInSeconds} and returns a reference to this Builder so that the
     * methods can be chained together.
     *
     * @param val the {@code controlRateInSeconds} to set
     * @return a reference to this Builder
     */
    public T withControlRateInSeconds(double val) {
      controlRateInSeconds = val;
      return self();
    }

    /**
     * Sets the {@code droneStateLifeDurationInSeconds} and returns a reference to this Builder so
     * that the methods can be chained together.
     *
     * @param val the {@code droneStateLifeDurationInSeconds} to set
     * @return a reference to this Builder
     */
    public T withDroneStateLifeDurationInSeconds(double val) {
      droneStateLifeDurationInSeconds = val;
      return self();
    }

    /**
     * Sets the {@code timeProvider} and returns a reference to this Builder so that the methods can
     * be chained together.
     *
     * @param val the {@code timeProvider} to set
     * @return a reference to this Builder
     */
    public T withTimeProvider(TimeProvider val) {
      timeProvider = val;
      return self();
    }
  }
}
