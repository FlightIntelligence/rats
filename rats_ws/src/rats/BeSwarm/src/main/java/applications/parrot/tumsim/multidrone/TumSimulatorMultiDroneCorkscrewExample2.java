package applications.parrot.tumsim.multidrone;

/**
 * Example trajectory for a corkscrew motion with possible parametrization for multiple drone
 * instantiation. Place drone at (2,0,1)
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
public class TumSimulatorMultiDroneCorkscrewExample2
    extends TumSimulatorMultiDroneCorkscrewExample1 {
  /** Default constructor with PI phase displacement. */
  public TumSimulatorMultiDroneCorkscrewExample2() {
    super(Math.PI);
  }
}
