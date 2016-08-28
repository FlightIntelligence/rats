package applications.parrot.bebop;

import applications.TrajectoriesForTesting;
import control.FiniteTrajectory4d;

/** @author Hoang Tung Dinh */
public class BebopLargeCircleExample extends AbstractBebopExample {

  /** Default constructor. */
  public BebopLargeCircleExample() {
    super("BebopLargeCircleExample");
  }

  @Override
  public FiniteTrajectory4d getConcreteTrajectory() {
    return TrajectoriesForTesting.getLargeCircle();
  }
}
