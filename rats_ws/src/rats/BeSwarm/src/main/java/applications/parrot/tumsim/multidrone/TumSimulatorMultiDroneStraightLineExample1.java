package applications.parrot.tumsim.multidrone;

import applications.parrot.tumsim.AbstractTumSimulatorExample;
import applications.trajectory.Trajectories;
import applications.trajectory.geom.point.Point4D;
import control.FiniteTrajectory4d;

/**
 * Example trajectory base class with displement parameter for multiple drone instantiations. This
 * example can be used for the pingpong room.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 * @see <a href="https://github.com/dougvk/tum_simulator">The simulator</a>
 */
public class TumSimulatorMultiDroneStraightLineExample1 extends AbstractTumSimulatorExample {

  private static final int XCOORD_START = 0;
  private static final int YCOORD_START = 0;
  private static final int ZCOORD_START = 1;
  private static final int ANGLE_START = 0;
  private static final int XCOORD_END = 0;
  private static final int YCOORD_END = -4;
  private static final int ZCOORD_END = 1;
  private static final int ANGLE_END = 0;
  private static final int VELOCITY = 1;
  private final double displacement;

  /** Default Constructor. */
  public TumSimulatorMultiDroneStraightLineExample1() {
    this(0);
  }

  /** @param displacement the displaement in the x direction for multiple drones. */
  public TumSimulatorMultiDroneStraightLineExample1(double displacement) {
    super("TumRunStraightLineTrajectory1");
    this.displacement = displacement;
  }

  @Override
  public FiniteTrajectory4d getConcreteTrajectory() {
    return Trajectories.newStraightLineTrajectory(
        Point4D.create(XCOORD_START + displacement, YCOORD_START, ZCOORD_START, ANGLE_START),
        Point4D.create(XCOORD_END + displacement, YCOORD_END, ZCOORD_END, ANGLE_END),
        VELOCITY);
  }
}
