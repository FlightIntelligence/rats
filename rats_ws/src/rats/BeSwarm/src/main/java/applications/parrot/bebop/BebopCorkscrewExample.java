package applications.parrot.bebop;

import applications.TrajectoriesForTesting;
import control.FiniteTrajectory4d;

/**
 * An implementation of {@link AbstractBebopExample} with the {@link
 * TrajectoriesForTesting#getCorkscrew()} trajectory.
 *
 * @author Hoang Tung Dinh
 */
public class BebopCorkscrewExample extends AbstractBebopExample {

  /** Default constructor. */
  public BebopCorkscrewExample() {
    super("BebopCorkscrewExample");
  }

  @Override
  public FiniteTrajectory4d getConcreteTrajectory() {
    return TrajectoriesForTesting.getCorkscrew();
  }
}
