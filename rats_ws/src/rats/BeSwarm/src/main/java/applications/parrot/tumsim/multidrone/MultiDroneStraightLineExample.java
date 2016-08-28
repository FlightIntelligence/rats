package applications.parrot.tumsim.multidrone;

/**
 * Example file for providing trajectories for 2 drones with 2 implementation of corkscrew
 * trajectories around one another. these trajectories keep a minimum distance of 1 m at all times.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
public class MultiDroneStraightLineExample extends AbstractMultiDroneExample {

  /** Default constructor. */
  public MultiDroneStraightLineExample() {
    super(
        new TumSimulatorMultiDroneStraightLineExample1(),
        new TumSimulatorMultiDroneStraightLineExample2());
  }
}
