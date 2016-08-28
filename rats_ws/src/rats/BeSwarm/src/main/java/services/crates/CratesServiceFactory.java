package services.crates;

import hal_quadrotor.Hover;
import hal_quadrotor.HoverRequest;
import hal_quadrotor.HoverResponse;
import hal_quadrotor.Land;
import hal_quadrotor.LandRequest;
import hal_quadrotor.LandResponse;
import hal_quadrotor.State;
import hal_quadrotor.Takeoff;
import hal_quadrotor.TakeoffRequest;
import hal_quadrotor.TakeoffResponse;
import hal_quadrotor.Velocity;
import hal_quadrotor.VelocityHeight;
import hal_quadrotor.VelocityHeightRequest;
import hal_quadrotor.VelocityHeightResponse;
import hal_quadrotor.VelocityRequest;
import hal_quadrotor.VelocityResponse;
import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import services.CommonServiceFactory;
import services.FlyingStateService;
import services.HoverService;
import services.LandService;
import services.TakeOffService;
import services.Velocity2dService;
import services.Velocity3dService;
import services.rossubscribers.MessagesSubscriberService;
import time.RosTime;

/** @author Hoang Tung Dinh */
public final class CratesServiceFactory implements CommonServiceFactory {

  private static final Logger logger = LoggerFactory.getLogger(CratesServiceFactory.class);
  private static final String SERVICE_NOT_FOUND = "Service not found.";

  private final String droneName;
  private final String modelName;
  private final ConnectedNode connectedNode;
  private final String namePrefix;

  private CratesServiceFactory(String droneName, String modelName, ConnectedNode connectedNode) {
    this.droneName = droneName;
    this.modelName = modelName;
    this.connectedNode = connectedNode;
    this.namePrefix = "/hal/quadrotor/" + modelName + "/" + droneName + "/";
  }

  /**
   * Creates a service factory for the Crates simulator.
   *
   * @param droneName the name of the drone
   * @param modelName the model of the drone
   * @param connectedNode the connected ros node
   * @return a service factory for the drone
   */
  public static CratesServiceFactory create(
      String droneName, String modelName, ConnectedNode connectedNode) {
    return new CratesServiceFactory(droneName, modelName, connectedNode);
  }

  @Override
  public TakeOffService createTakeOffService() {
    try {
      return CratesTakeOffService.create(
          connectedNode.<TakeoffRequest, TakeoffResponse>newServiceClient(
              namePrefix + "controller/AbstractParrotTakeOff", Takeoff._TYPE));
    } catch (ServiceNotFoundException e) {
      logger.info(
          "Take off service not found. Drone: {}. Model: {}. Exception: {}",
          droneName,
          modelName,
          e);
      throw new IllegalStateException(
          String.format("Take off service not found. Drone: %s. Model: %s", droneName, modelName));
    }
  }

  @Override
  public LandService createLandService() {
    try {
      return CratesLandService.create(
          connectedNode.<LandRequest, LandResponse>newServiceClient(
              namePrefix + "controller/AbstractParrotLand", Land._TYPE));
    } catch (ServiceNotFoundException e) {
      logger.debug(SERVICE_NOT_FOUND, e);
      throw new IllegalStateException(
          String.format("AbstractParrotLand service not found. Drone: %s. Model: %s", droneName, modelName));
    }
  }

  @Override
  public FlyingStateService createFlyingStateService() {
    final String topicName = namePrefix + "Truth";
    final MessagesSubscriberService<State> flyingStateSubscriber =
        MessagesSubscriberService.create(
            connectedNode.<State>newSubscriber(topicName, State._TYPE),
            RosTime.create(connectedNode));
    return CratesFlyingStateService.create(flyingStateSubscriber);
  }

  /**
   * Creates a velocity3d service for the drone.
   *
   * @return a velocity3d service for the drone
   */
  public Velocity3dService createVelocity3dService() {
    try {
      return CratesVelocity3dService.create(
          connectedNode.<VelocityRequest, VelocityResponse>newServiceClient(
              namePrefix + "controller/Velocity", Velocity._TYPE));
    } catch (ServiceNotFoundException e) {
      logger.debug(SERVICE_NOT_FOUND, e);
      throw new IllegalStateException(
          String.format("Velocity service not found. Drone: %s. Model: %s", droneName, modelName));
    }
  }

  /**
   * Creates a velocity2d service for the drone.
   *
   * @return a velocity2d service for the drone
   */
  public Velocity2dService createVelocity2dService() {
    try {
      return CratesVelocity2dService.create(
          connectedNode.<VelocityHeightRequest, VelocityHeightResponse>newServiceClient(
              namePrefix + "controller/VelocityHeight", VelocityHeight._TYPE));
    } catch (ServiceNotFoundException e) {
      logger.debug(SERVICE_NOT_FOUND, e);
      throw new IllegalStateException(
          String.format(
              "Velocity height service not found. Drone: %s. Model: %s", droneName, modelName));
    }
  }

  /**
   * Creates a {@link HoverService} for the drone.
   *
   * @return an instance of {@link HoverService}
   */
  public HoverService createHoverService() {
    try {
      return CratesHoverService.create(
          connectedNode.<HoverRequest, HoverResponse>newServiceClient(
              namePrefix + "controller/AbstractHover", Hover._TYPE));
    } catch (ServiceNotFoundException e) {
      logger.debug(SERVICE_NOT_FOUND, e);
      throw new IllegalStateException(
          String.format("AbstractHover service not found. Drone: %s. Model: %s", droneName, modelName));
    }
  }
}
