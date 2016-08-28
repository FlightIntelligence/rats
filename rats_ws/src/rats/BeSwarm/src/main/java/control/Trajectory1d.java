package control;

/**
 * Representation of the trajectory function.
 *
 * @author Hoang Tung Dinh
 */
public interface Trajectory1d {
  /**
   * Gets the desired position of the drone.
   *
   * @param timeInSeconds the point in time to get the position for.
   * @return The desired value of the position in the dimenstion specified by this object for the
   *     given point in time.
   */
  double getDesiredPosition(double timeInSeconds);
}
