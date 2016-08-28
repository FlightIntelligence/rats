package control;

/** @author Hoang Tung Dinh */
public interface Trajectory4d {
  /**
   * @param timeInSeconds the point in time to get the position for.
   * @return The desired value of the position relative to the x axis. for the given point in time.
   */
  double getDesiredPositionX(double timeInSeconds);

  /**
   * @param timeInSeconds the point in time to get the position for.
   * @return The desired value of the position relative to the y axis. for the given point in time.
   */
  double getDesiredPositionY(double timeInSeconds);

  /**
   * @param timeInSeconds the point in time to get the position for.
   * @return The desired value of the position relative to the z axis. for the given point in time.
   */
  double getDesiredPositionZ(double timeInSeconds);

  /**
   * This method returns the angle of the yaw of the drone. The yaw is defined according to the
   * right hand rule w.r.t. the z axis. The yaw equals to zero when the orientation of the drone is
   * [x=1, y=0, z=0]. For an illustration, See <a
   * href="https://en.wikipedia.org/wiki/Euler_angles">this wikipedia page</a>. The yaw angle is in
   * radians, ranging from -infinite to infinite.
   *
   * @param timeInSeconds the point in time to get the position for.
   * @return the desired position of the yaw
   */
  double getDesiredAngleZ(double timeInSeconds);
}
