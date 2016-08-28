package applications.parrot.tumsim.multidrone;

import applications.trajectory.MultiTrajectoryServer;
import applications.trajectory.TrajectoryServer;
import com.google.common.collect.Lists;

import java.util.Arrays;
import java.util.List;

/**
 * Example file for providing trajectories for 2 drones with 2 implementation of corkscrew
 * trajectories around one another. these trajectories keep a minimum distance of 1 m at all times.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
public abstract class AbstractMultiDroneExample implements MultiTrajectoryServer {

  private final List<TrajectoryServer> droneServers;

  /**
   * Default constructor.
   *
   * @param trajectories The trajectories to serve up for the different drones.
   */
  public AbstractMultiDroneExample(TrajectoryServer... trajectories) {
    droneServers = Lists.newArrayList();
    droneServers.addAll(Arrays.asList(trajectories));
  }

  @Override
  public List<TrajectoryServer> getAllDifferentTrajectories() {
    return Lists.newArrayList(droneServers);
  }
}
