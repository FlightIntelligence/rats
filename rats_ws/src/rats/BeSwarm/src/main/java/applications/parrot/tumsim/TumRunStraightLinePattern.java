package applications.parrot.tumsim;

import applications.trajectory.Trajectories;
import applications.trajectory.geom.point.Point4D;
import control.FiniteTrajectory4d;

/**
 * This class is for running the simulation with the AR drone in the Tum simulator.
 *
 * @author Hoang Tung Dinh
 * @see <a href="https://github.com/dougvk/tum_simulator">The simulator</a>
 */
public final class TumRunStraightLinePattern extends AbstractTumSimulatorExample {

  /** Default Constructor. */
  public TumRunStraightLinePattern() {
    super("TumRunStraightLineTrajectory");
  }

  @Override
  public FiniteTrajectory4d getConcreteTrajectory() {
    return Trajectories.newStraightLineTrajectory(
        Point4D.create(0, 0, 1, 0), Point4D.create(2, 0, 1, 0), 0.2);
  }
}
