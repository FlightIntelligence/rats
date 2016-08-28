package control.dto;

import com.google.auto.value.AutoValue;

/** @author Hoang Tung Dinh */
@AutoValue
public abstract class DroneStateStamped {

  DroneStateStamped() {}

  /**
   * Creates a drone state stamped with time.
   *
   * @param pose the pose of the drone
   * @param inertialFrameVelocity the velocity of the drone in the inertial frame
   * @param timeStampInSeconds the time stamp of the state
   * @return a state of the drone stamped with time
   */
  public static DroneStateStamped create(
      Pose pose, InertialFrameVelocity inertialFrameVelocity, double timeStampInSeconds) {
    return new AutoValue_DroneStateStamped(pose, inertialFrameVelocity, timeStampInSeconds);
  }

  /**
   * Gets the pose of the drone.
   *
   * @return the pose of the drone
   */
  public abstract Pose pose();

  /**
   * Gets the velocity of the drone in the inertial frame.
   *
   * @return the velocity of the drone in the inertial frame
   */
  public abstract InertialFrameVelocity inertialFrameVelocity();

  /**
   * Gets the time stamp of the state.
   *
   * @return the time stamp of the state
   */
  public abstract double getTimeStampInSeconds();
}
