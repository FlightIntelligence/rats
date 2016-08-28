package control.dto;

/** @author Hoang Tung Dinh */
interface VelocityDto {
  /**
   * Gets the velocity in the x coordinate.
   *
   * @return the velocity in the x coordinate
   */
  double linearX();

  /**
   * Gets the velocity in the y coordinate.
   *
   * @return the velocity in the y coordinate
   */
  double linearY();

  /**
   * Gets the velocity in the z coordinate.
   *
   * @return the velocity in the z coordinate
   */
  double linearZ();

  /**
   * Gets the velocity in the z rotation (the yaw).
   *
   * @return the velocity in the z rotation (the yaw)
   */
  double angularZ();
}
