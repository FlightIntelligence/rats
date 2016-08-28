package applications.trajectory;

import java.util.List;

/**
 * MultiTrajectoryServer instances offer multiple trajectory server instances.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
public interface MultiTrajectoryServer {

  /** @return A list of trajectory server instances. */
  List<TrajectoryServer> getAllDifferentTrajectories();
}
