package applications.parrot.tumsim;

import applications.TrajectoriesForTesting;
import control.FiniteTrajectory4d;

/** @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be> */
public class TumSimulatorPendulumExample extends AbstractTumSimulatorExample {

  /** Default Constructor. */
  public TumSimulatorPendulumExample() {
    super("TumRunPendulumTrajectory");
  }

  @Override
  public FiniteTrajectory4d getConcreteTrajectory() {
    return TrajectoriesForTesting.getSlowIndoorPendulum();
  }
}
