package services;

/**
 * Represents velocity 2d service.
 *
 * @author Hoang Tung Dinh
 */
public interface Velocity2dService {
  /**
   * Sends a velocity-2d message to the drone. This method requires the velocity in x-y plane, the
   * desired z-axis position (the altitude) and the desired angular-z position (the yaw).
   *
   * @param inertialFrameVelocityX the x-component velocity in the inertial frame
   * @param inertialFrameVelocityY the y-component velocity in the inertial frame
   * @param linearPositionZ the desired z position (altitude)
   * @param angularPositionZ the desired angular-z position (yaw)
   */
  void sendVelocityHeightMessage(
      double inertialFrameVelocityX,
      double inertialFrameVelocityY,
      double linearPositionZ,
      double angularPositionZ);
}
