package control;

/** @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be> */
public interface FiniteTrajectory4d extends Trajectory4d {
  /**
   * @return The estimated point in time after consuming this trajectory, that the trajectory will
   *     not perform any more movement.
   */
  double getTrajectoryDuration();
}
