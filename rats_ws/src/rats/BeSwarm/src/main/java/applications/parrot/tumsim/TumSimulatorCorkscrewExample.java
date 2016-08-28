package applications.parrot.tumsim;

import applications.TrajectoriesForTesting;
import control.FiniteTrajectory4d;

/** @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be> */
public class TumSimulatorCorkscrewExample extends AbstractTumSimulatorExample {

  /** Default constructor. */
  public TumSimulatorCorkscrewExample() {
    super("TumRunCorkscrewTrajectory");
  }

  @Override
  public FiniteTrajectory4d getConcreteTrajectory() {
    return TrajectoriesForTesting.getCorkscrew();
  }
}
