package applications.parrot.tumsim;

import applications.LineTrajectory;
import choreo.Choreography;
import control.FiniteTrajectory4d;

/**
 * This class is for running the simulation with the AR drone in the Tum simulator.
 *
 * @author Hoang Tung Dinh
 * @see <a href="https://github.com/dougvk/tum_simulator">The simulator</a>
 */
public final class TumRunSimpleLinePattern extends AbstractTumSimulatorExample {

  /** Default Constructor. */
  public TumRunSimpleLinePattern() {
    super("TumRunSimpleLinePattern");
  }

  @Override
  public FiniteTrajectory4d getConcreteTrajectory() {
    final double flightDuration = 100;
    return Choreography.builder()
        .withTrajectory(LineTrajectory.create(flightDuration, 2.0))
        .forTime(flightDuration)
        .build();
  }
}
