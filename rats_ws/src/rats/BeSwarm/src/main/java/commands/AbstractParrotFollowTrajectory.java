package commands;

import control.DefaultPidParameters;
import control.PidController4d;
import control.PidParameters;
import control.Trajectory4d;
import control.dto.DroneStateStamped;
import control.dto.InertialFrameVelocity;
import control.localization.StateEstimator;
import services.Velocity4dService;
import time.TimeProvider;

import static com.google.common.base.Preconditions.checkNotNull;

/**
 * Abstract follow trajectory command for parrot drones.
 *
 * @author Hoang Tung Dinh
 */
public abstract class AbstractParrotFollowTrajectory extends AbstractFollowTrajectory {

  private final PidController4d pidController4d;
  private final Velocity4dService velocity4dService;

  protected AbstractParrotFollowTrajectory(
      StateEstimator stateEstimator,
      Trajectory4d trajectory4d,
      double durationInSeconds,
      double controlRateInSeconds,
      double droneStateLifeDurationInSeconds,
      TimeProvider timeProvider,
      PidController4d pidController4d,
      Velocity4dService velocity4dService) {
    super(
        stateEstimator,
        trajectory4d,
        durationInSeconds,
        controlRateInSeconds,
        droneStateLifeDurationInSeconds,
        timeProvider);

    this.pidController4d = pidController4d;
    this.velocity4dService = velocity4dService;
  }

  @Override
  protected final AbstractFollowTrajectory.AbstractControlLoop createControlLoop() {
    return new ControlLoop();
  }

  private final class ControlLoop extends AbstractFollowTrajectory.AbstractControlLoop {

    private ControlLoop() {}

    @Override
    protected void computeAndSendResponse(
        double currentTimeInSeconds, DroneStateStamped currentState) {
      final InertialFrameVelocity nextVelocity =
          pidController4d.compute(
              currentState.pose(), currentState.inertialFrameVelocity(), currentTimeInSeconds);
      velocity4dService.sendInertialFrameVelocity(nextVelocity, currentState.pose());
    }
  }

  /** {@code AbstractParrotFollowTrajectory} builder static inner class. */
  public abstract static class ParrotBuilder<T extends ParrotBuilder<T>>
      extends AbstractFollowTrajectory.AbstractBuilder<T> {

    protected PidParameters pidLinearXParameters;
    protected PidParameters pidLinearYParameters;
    protected PidParameters pidLinearZParameters;
    protected PidParameters pidAngularZParameters;
    protected Velocity4dService velocity4dService;

    protected ParrotBuilder() {
      pidLinearXParameters = DefaultPidParameters.LINEAR_X.getParameters();
      pidLinearYParameters = DefaultPidParameters.LINEAR_Y.getParameters();
      pidLinearZParameters = DefaultPidParameters.LINEAR_Z.getParameters();
      pidAngularZParameters = DefaultPidParameters.ANGULAR_Z.getParameters();
    }

    /**
     * Sets the {@code pidLinearXParameters} and returns a reference to this Builder so that the
     * methods can be chained together.
     *
     * @param val the {@code pidLinearXParameters} to set
     * @return a reference to this Builder
     */
    public T withPidLinearXParameters(PidParameters val) {
      pidLinearXParameters = val;
      return self();
    }

    /**
     * Sets the {@code pidLinearYParameters} and returns a reference to this Builder so that the
     * methods can be chained together.
     *
     * @param val the {@code pidLinearYParameters} to set
     * @return a reference to this Builder
     */
    public T withPidLinearYParameters(PidParameters val) {
      pidLinearYParameters = val;
      return self();
    }

    /**
     * Sets the {@code pidLinearZParameters} and returns a reference to this Builder so that the
     * methods can be chained together.
     *
     * @param val the {@code pidLinearZParameters} to set
     * @return a reference to this Builder
     */
    public T withPidLinearZParameters(PidParameters val) {
      pidLinearZParameters = val;
      return self();
    }

    /**
     * Sets the {@code pidAngularZParameters} and returns a reference to this Builder so that the
     * methods can be chained together.
     *
     * @param val the {@code pidAngularZParameters} to set
     * @return a reference to this Builder
     */
    public T withPidAngularZParameters(PidParameters val) {
      pidAngularZParameters = val;
      return self();
    }

    /**
     * Sets the {@code velocity4dService} and returns a reference to this Builder so that the
     * methods can be chained together.
     *
     * @param val the {@code velocity4dService} to set
     * @return a reference to this Builder
     */
    public T withVelocity4dService(Velocity4dService val) {
      velocity4dService = val;
      return self();
    }

    protected void checkMissingParameters() {
      checkNotNull(trajectory4d);
      checkNotNull(durationInSeconds);
      checkNotNull(pidLinearXParameters);
      checkNotNull(pidLinearYParameters);
      checkNotNull(pidLinearZParameters);
      checkNotNull(pidAngularZParameters);
      checkNotNull(controlRateInSeconds);
      checkNotNull(droneStateLifeDurationInSeconds);
      checkNotNull(stateEstimator);
      checkNotNull(velocity4dService);
      checkNotNull(timeProvider);
    }

    protected PidController4d createPidController4d() {
      return PidController4d.builder()
          .trajectory4d(trajectory4d)
          .linearXParameters(pidLinearXParameters)
          .linearYParameters(pidLinearYParameters)
          .linearZParameters(pidLinearZParameters)
          .angularZParameters(pidAngularZParameters)
          .build();
    }
  }
}
