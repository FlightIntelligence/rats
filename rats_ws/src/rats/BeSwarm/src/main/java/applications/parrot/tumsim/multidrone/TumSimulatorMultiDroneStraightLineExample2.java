package applications.parrot.tumsim.multidrone;

/**
 * Example trajectory with displement parameter for multiple drone instantiations. This example can
 * be used for the pingpong room.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 * @see <a href="https://github.com/dougvk/tum_simulator">The simulator</a>
 */
public final class TumSimulatorMultiDroneStraightLineExample2
    extends TumSimulatorMultiDroneStraightLineExample1 {
  private static final double DISPLACEMENT = 2;

  /** Default Constructor. */
  public TumSimulatorMultiDroneStraightLineExample2() {
    super(DISPLACEMENT);
  }
}
