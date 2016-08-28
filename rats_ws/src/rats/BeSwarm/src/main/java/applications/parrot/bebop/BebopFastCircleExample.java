package applications.parrot.bebop;

import applications.TrajectoriesForTesting;
import control.FiniteTrajectory4d;

/** @author Hoang Tung Dinh */
public class BebopFastCircleExample extends AbstractBebopExample {

  /** Default constructor. */
  public BebopFastCircleExample() {
    super("BebopFastCircleExample");
  }

  @Override
  public FiniteTrajectory4d getConcreteTrajectory() {
    return TrajectoriesForTesting.getFastCircle();
  }
}
