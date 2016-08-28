package time;

import org.ros.node.ConnectedNode;

/**
 * A {@link TimeProvider}'s instance using ROS timer.
 *
 * @author Hoang Tung Dinh
 */
public final class RosTime implements TimeProvider {
  private final ConnectedNode connectedNode;

  private RosTime(ConnectedNode connectedNode) {
    this.connectedNode = connectedNode;
  }

  /**
   * Creates a {@link RosTime}'s instance.
   *
   * @param connectedNode the ROS node
   * @return a {@link RosTime}'s instance
   */
  public static RosTime create(ConnectedNode connectedNode) {
    return new RosTime(connectedNode);
  }

  @Override
  public long getCurrentTimeNanoSeconds() {
    return connectedNode.getCurrentTime().totalNsecs();
  }

  @Override
  public double getCurrentTimeSeconds() {
    return connectedNode.getCurrentTime().toSeconds();
  }
}
