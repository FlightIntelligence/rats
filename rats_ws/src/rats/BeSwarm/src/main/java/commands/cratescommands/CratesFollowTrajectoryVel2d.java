package commands.cratescommands;

import applications.trajectory.TrajectoryUtils;
import commands.AbstractFollowTrajectory;
import control.PidController1d;
import control.PidParameters;
import control.Trajectory1d;
import control.Trajectory4d;
import control.dto.DroneStateStamped;
import control.localization.StateEstimator;
import services.Velocity2dService;
import time.TimeProvider;

import static com.google.common.base.Preconditions.checkNotNull;

/**
 * Follow trajectory command for Crates's drones. This command uses 2d velocity to control the
 * drones. That is, the command will send the velocity in x and y in the inertial frame and the
 * desired z position and yaw position.
 *
 * @author Hoang Tung Dinh
 */
public final class CratesFollowTrajectoryVel2d extends AbstractFollowTrajectory {

  private final Velocity2dService velocity2dService;
  private final PidController1d pidControllerLinearX;
  private final PidController1d pidControllerLinearY;
  private final Trajectory1d trajectoryLinearZ;
  private final Trajectory1d trajectoryAngularZ;

  private CratesFollowTrajectoryVel2d(
      StateEstimator stateEstimator,
      Trajectory4d trajectory4d,
      double durationInSeconds,
      double controlRateInSeconds,
      double droneStateLifeDurationInSeconds,
      TimeProvider timeProvider,
      Velocity2dService velocity2dService,
      PidController1d pidControllerLinearX,
      PidController1d pidControllerLinearY) {
    super(
        stateEstimator,
        trajectory4d,
        durationInSeconds,
        controlRateInSeconds,
        droneStateLifeDurationInSeconds,
        timeProvider);
    this.velocity2dService = velocity2dService;
    this.pidControllerLinearX = pidControllerLinearX;
    this.pidControllerLinearY = pidControllerLinearY;
    trajectoryLinearZ = TrajectoryUtils.getTrajectoryLinearZ(trajectory4d);
    trajectoryAngularZ = TrajectoryUtils.getTrajectoryAngularZ(trajectory4d);
  }

  /**
   * Creates a builder for {@link CratesFollowTrajectoryVel2d}.
   *
   * @return a builder for {@link CratesFollowTrajectoryVel2d}
   */
  public static Builder builder() {
    return new Builder();
  }

  @Override
  protected AbstractControlLoop createControlLoop() {
    return new ControlLoop();
  }

  private final class ControlLoop extends AbstractControlLoop {

    @Override
    protected void computeAndSendResponse(
        double currentTimeInSeconds, DroneStateStamped currentState) {
      final double nextVelocityX =
          pidControllerLinearX.compute(
              currentState.pose().x(),
              currentState.inertialFrameVelocity().linearX(),
              currentTimeInSeconds);
      final double nextVelocityY =
          pidControllerLinearY.compute(
              currentState.pose().y(),
              currentState.inertialFrameVelocity().linearY(),
              currentTimeInSeconds);

      velocity2dService.sendVelocityHeightMessage(
          nextVelocityX,
          nextVelocityY,
          trajectoryLinearZ.getDesiredPosition(currentTimeInSeconds),
          trajectoryAngularZ.getDesiredPosition(currentTimeInSeconds));
    }
  }

  /** {@code CratesFollowTrajectoryVel2d} builder static inner class. */
  public static final class Builder extends AbstractFollowTrajectory.AbstractBuilder<Builder> {
    private Velocity2dService velocity2dService;
    private PidParameters pidLinearXParameters;
    private PidParameters pidLinearYParameters;

    private Builder() {}

    @Override
    protected Builder self() {
      return this;
    }

    /**
     * Sets the {@code velocity2dService} and returns a reference to this Builder so that the
     * methods can be chained together.
     *
     * @param val the {@code velocity2dService} to set
     * @return a reference to this Builder
     */
    public Builder withVelocity2dService(Velocity2dService val) {
      velocity2dService = val;
      return this;
    }

    /**
     * Sets the {@code pidLinearXParameters} and returns a reference to this Builder so that the
     * methods can be chained together.
     *
     * @param val the {@code pidLinearXParameters} to set
     * @return a reference to this Builder
     */
    public Builder withPidLinearXParameters(PidParameters val) {
      pidLinearXParameters = val;
      return this;
    }

    /**
     * Sets the {@code pidLinearYParameters} and returns a reference to this Builder so that the
     * methods can be chained together.
     *
     * @param val the {@code pidLinearYParameters} to set
     * @return a reference to this Builder
     */
    public Builder withPidLinearYParameters(PidParameters val) {
      pidLinearYParameters = val;
      return this;
    }

    /**
     * Returns a {@code CratesFollowTrajectoryVel2d} built from the parameters previously set.
     *
     * @return a {@code CratesFollowTrajectoryVel2d} built with parameters of this {@code
     *     CratesFollowTrajectoryVel2d.Builder}
     */
    public CratesFollowTrajectoryVel2d build() {
      checkNotNull(trajectory4d);
      checkNotNull(durationInSeconds);
      checkNotNull(pidLinearXParameters);
      checkNotNull(pidLinearYParameters);
      checkNotNull(controlRateInSeconds);
      checkNotNull(droneStateLifeDurationInSeconds);
      checkNotNull(stateEstimator);
      checkNotNull(velocity2dService);
      checkNotNull(timeProvider);

      final PidController1d pidLinearX =
          PidController1d.create(
              pidLinearXParameters, TrajectoryUtils.getTrajectoryLinearX(trajectory4d));
      final PidController1d pidLinearY =
          PidController1d.create(
              pidLinearYParameters, TrajectoryUtils.getTrajectoryLinearY(trajectory4d));

      return new CratesFollowTrajectoryVel2d(
          stateEstimator,
          trajectory4d,
          durationInSeconds,
          controlRateInSeconds,
          droneStateLifeDurationInSeconds,
          timeProvider,
          velocity2dService,
          pidLinearX,
          pidLinearY);
    }
  }
}
