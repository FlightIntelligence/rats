package applications.parrot.bebop;

import applications.TrajectoriesForTesting;
import control.FiniteTrajectory4d;

/** @author Hoang Tung Dinh */
public class BebopSlowCircleExample extends AbstractBebopExample {

  /** Default constructor. */
  public BebopSlowCircleExample() {
    super("BebopSlowCircleExample");
  }

  @Override
  public FiniteTrajectory4d getConcreteTrajectory() {
    return TrajectoriesForTesting.getSlowCircle();
  }
}
