package services;

/**
 * Represents velocity 3d service.
 *
 * @author Hoang Tung Dinh
 */
public interface Velocity3dService {
  /**
   * Sends a velocity-3d message. This method requires a 3-component velocity in x-y-z coordinate
   * and the desired angular-z position (the yaw) as the arguments.
   *
   * @param inertialFrameVelocityX the x-component velocity in the inertial frame
   * @param inertialFrameVelocityY the y-component velocity in the inertial frame
   * @param inertialFrameVelocityZ the z-component velocity in the inertial frame
   * @param angularPositionZ the desired angular-z position (the yaw)
   */
  void sendVelocity3dMessage(
      double inertialFrameVelocityX,
      double inertialFrameVelocityY,
      double inertialFrameVelocityZ,
      double angularPositionZ);
}
