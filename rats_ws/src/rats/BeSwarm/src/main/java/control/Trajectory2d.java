package control;

/**
 * 2D Trajectory representing a trajectory in 2 planes of motion. Default plane of motion is the
 * (x,y)-plane but the (x,z), and the (y,z) plane motion trajectories can also be modelled under
 * this interface.
 *
 * @author Kristof Coninx
 */
public interface Trajectory2d {

  /**
   * @param timeInSeconds the point in time to get the position for.
   * @return The desired value of the position relative to the abscissa for the given point in time.
   */
  double getDesiredPositionAbscissa(double timeInSeconds);

  /**
   * @param timeInSeconds the point in time to get the position for.
   * @return The desired value of the position relative to the ordinate. for the given point in
   *     time.
   */
  double getDesiredPositionOrdinate(double timeInSeconds);
}
