package applications.parrot.bebop;

import applications.TrajectoriesForTesting;
import control.FiniteTrajectory4d;

/** @author Hoang Tung Dinh */
public class BebopIndoorPendulumExample extends AbstractBebopExample {

  /** Default constructor. */
  public BebopIndoorPendulumExample() {
    super("BebopIndoorPendulumExample");
  }

  @Override
  public FiniteTrajectory4d getConcreteTrajectory() {
    return TrajectoriesForTesting.getSlowIndoorPendulum();
  }
}
