package services.parrot;

import org.ros.node.ConnectedNode;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import services.CommonServiceFactory;
import services.LandService;
import services.ResetService;
import services.TakeOffService;
import services.Velocity4dService;
import std_msgs.Empty;

/** @author Hoang Tung Dinh */
public abstract class ParrotServiceFactory implements CommonServiceFactory {

  private static final Logger logger = LoggerFactory.getLogger(ParrotServiceFactory.class);

  private final ConnectedNode connectedNode;
  private final String droneName;

  ParrotServiceFactory(ConnectedNode connectedNode, String droneName) {
    this.connectedNode = connectedNode;
    this.droneName = droneName;
  }

  @Override
  public final TakeOffService createTakeOffService() {
    final String topicName = "/" + droneName + "/takeoff";
    final TakeOffService takeOffService =
        ParrotTakeOffService.create(connectedNode.<Empty>newPublisher(topicName, Empty._TYPE));
    logger.info("Take off service connected to {}", topicName);
    return takeOffService;
  }

  @Override
  public final LandService createLandService() {
    final String topicName = "/" + droneName + "/land";
    final LandService landService =
        ParrotLandService.create(connectedNode.<Empty>newPublisher(topicName, Empty._TYPE));
    logger.info("AbstractParrotLand service connected to {}", topicName);
    return landService;
  }

  /**
   * Creates the reset service for a parrot drone.
   * @return a {@link ResetService}
   */
  public final ResetService createResetService() {
    final String topicName = "/" + droneName + "/reset";
    return BebopResetService.create(connectedNode.<Empty>newPublisher(topicName, Empty._TYPE));
  }

  /**
   * Creates a 4d-velocity service for a parrot drone.
   *
   * @return a {@link Velocity4dService} instance for a parrot drone
   */
  public abstract Velocity4dService createVelocity4dService();

  ConnectedNode getConnectedNode() {
    return connectedNode;
  }

  String getDroneName() {
    return droneName;
  }
}
