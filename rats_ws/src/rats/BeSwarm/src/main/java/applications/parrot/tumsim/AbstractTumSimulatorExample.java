package applications.parrot.tumsim;

import applications.trajectory.TrajectoryServer;
import control.FiniteTrajectory4d;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

/**
 * Abstract example trajectory class. Extend this class for easy implementation of new example
 * trajectories.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
public abstract class AbstractTumSimulatorExample extends AbstractNodeMain
    implements TrajectoryServer {
  private final String nodeName;

  protected AbstractTumSimulatorExample(String nodeName) {
    this.nodeName = nodeName;
  }

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of(nodeName);
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {
    final FiniteTrajectory4d trajectory = getConcreteTrajectory();
    final TumExampleFlightFacade flight = TumExampleFlightFacade.create(trajectory, connectedNode);
    flight.fly();
  }
}
