package services;

import control.dto.BodyFrameVelocity;
import control.dto.InertialFrameVelocity;
import control.dto.Pose;

/**
 * Represents velocity 4d service.
 *
 * @author Hoang Tung Dinh
 */
public interface Velocity4dService {
  /**
   * Sends an inertial frame velocity message to the drone. The velocity includes four components,
   * including three velocity components in the x-y-z coordinate and one velocity component for the
   * yaw. This method requires the current pose of the drone. The pose is to transform the velocity
   * in the inertial frame to the velocity in the body frame.
   *
   * @param inertialFrameVelocity the desired velocity in the inertial frame
   * @param pose the current pose of the drone
   */
  void sendInertialFrameVelocity(InertialFrameVelocity inertialFrameVelocity, Pose pose);

  /**
   * Sends a body frame velocity to the drone.
   *
   * @param bodyFrameVelocity the desired velocity in the body frame
   */
  void sendBodyFrameVelocity(BodyFrameVelocity bodyFrameVelocity);
}
