package commands.cratescommands;

import applications.trajectory.TrajectoryUtils;
import commands.AbstractFollowTrajectory;
import control.DefaultPidParameters;
import control.PidController1d;
import control.PidParameters;
import control.Trajectory1d;
import control.Trajectory4d;
import control.dto.DroneStateStamped;
import control.localization.StateEstimator;
import services.Velocity3dService;
import time.TimeProvider;

import static com.google.common.base.Preconditions.checkNotNull;

/**
 * Follow trajectory command for Crates's drones. This command uses 3d velocity to control the
 * drones. That is, the command will sends velocity in x, y, and z in the inertial frame and the
 * desired yaw angle (NOT the yaw velocity) to the drone in each control loop.
 *
 * @author Hoang Tung Dinh
 */
public final class CratesFollowTrajectoryVel3d extends AbstractFollowTrajectory {

  private final Velocity3dService velocity3dService;
  private final PidController1d pidControllerLinearX;
  private final PidController1d pidControllerLinearY;
  private final PidController1d pidControllerLinearZ;
  private final Trajectory1d trajectoryAngularZ;

  private CratesFollowTrajectoryVel3d(
      StateEstimator stateEstimator,
      final Trajectory4d trajectory4d,
      double durationInSeconds,
      double controlRateInSeconds,
      double droneStateLifeDurationInSeconds,
      TimeProvider timeProvider,
      Velocity3dService velocity3dService,
      PidController1d pidControllerLinearX,
      PidController1d pidControllerLinearY,
      PidController1d pidControllerLinearZ) {
    super(
        stateEstimator,
        trajectory4d,
        durationInSeconds,
        controlRateInSeconds,
        droneStateLifeDurationInSeconds,
        timeProvider);
    this.velocity3dService = velocity3dService;
    this.pidControllerLinearX = pidControllerLinearX;
    this.pidControllerLinearY = pidControllerLinearY;
    this.pidControllerLinearZ = pidControllerLinearZ;
    trajectoryAngularZ = TrajectoryUtils.getTrajectoryAngularZ(trajectory4d);
  }

  /**
   * Creates a builder for {@link CratesFollowTrajectoryVel3d}.
   *
   * @return a builder for {@link CratesFollowTrajectoryVel3d}
   */
  public static Builder builder() {
    return new Builder();
  }

  @Override
  protected AbstractFollowTrajectory.AbstractControlLoop createControlLoop() {
    return new ControlLoop();
  }

  private final class ControlLoop extends AbstractFollowTrajectory.AbstractControlLoop {

    private ControlLoop() {}

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
      final double nextVelocityZ =
          pidControllerLinearZ.compute(
              currentState.pose().z(),
              currentState.inertialFrameVelocity().linearZ(),
              currentTimeInSeconds);

      velocity3dService.sendVelocity3dMessage(
          nextVelocityX,
          nextVelocityY,
          nextVelocityZ,
          trajectoryAngularZ.getDesiredPosition(currentTimeInSeconds));
    }
  }

  /** {@code CratesFollowTrajectoryVel3d} builder static inner class. */
  public static final class Builder extends AbstractFollowTrajectory.AbstractBuilder<Builder> {
    private PidParameters pidLinearXParameters;
    private PidParameters pidLinearYParameters;
    private PidParameters pidLinearZParameters;
    private Velocity3dService velocity3dService;

    private Builder() {
      pidLinearXParameters = DefaultPidParameters.LINEAR_X.getParameters();
      pidLinearYParameters = DefaultPidParameters.LINEAR_Y.getParameters();
      pidLinearZParameters = DefaultPidParameters.LINEAR_Z.getParameters();
    }

    @Override
    protected Builder self() {
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
     * Sets the {@code pidLinearZParameters} and returns a reference to this Builder so that the
     * methods can be chained together.
     *
     * @param val the {@code pidLinearZParameters} to set
     * @return a reference to this Builder
     */
    public Builder withPidLinearZParameters(PidParameters val) {
      pidLinearZParameters = val;
      return this;
    }

    /**
     * Sets the {@code velocity3dService} and returns a reference to this Builder so that the
     * methods can be chained together.
     *
     * @param val the {@code velocity3dService} to set
     * @return a reference to this Builder
     */
    public Builder withVelocity3dService(Velocity3dService val) {
      velocity3dService = val;
      return this;
    }

    /**
     * Returns a {@code CratesFollowTrajectoryVel3d} built from the parameters previously set.
     *
     * @return a {@code CratesFollowTrajectoryVel3d} built with parameters of this {@code
     *     CratesFollowTrajectoryVel3d.Builder}
     */
    public CratesFollowTrajectoryVel3d build() {
      checkNotNull(trajectory4d);
      checkNotNull(durationInSeconds);
      checkNotNull(pidLinearXParameters);
      checkNotNull(pidLinearYParameters);
      checkNotNull(pidLinearZParameters);
      checkNotNull(controlRateInSeconds);
      checkNotNull(droneStateLifeDurationInSeconds);
      checkNotNull(stateEstimator);
      checkNotNull(velocity3dService);
      checkNotNull(timeProvider);

      final PidController1d pidLinearX =
          PidController1d.create(
              pidLinearXParameters, TrajectoryUtils.getTrajectoryLinearX(trajectory4d));
      final PidController1d pidLinearY =
          PidController1d.create(
              pidLinearYParameters, TrajectoryUtils.getTrajectoryLinearY(trajectory4d));
      final PidController1d pidLinearZ =
          PidController1d.create(
              pidLinearZParameters, TrajectoryUtils.getTrajectoryLinearZ(trajectory4d));

      return new CratesFollowTrajectoryVel3d(
          stateEstimator,
          trajectory4d,
          durationInSeconds,
          controlRateInSeconds,
          droneStateLifeDurationInSeconds,
          timeProvider,
          velocity3dService,
          pidLinearX,
          pidLinearY,
          pidLinearZ);
    }
  }
}
