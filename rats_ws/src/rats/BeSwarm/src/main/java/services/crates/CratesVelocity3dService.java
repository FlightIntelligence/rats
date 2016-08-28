package services.crates;

import hal_quadrotor.VelocityRequest;
import hal_quadrotor.VelocityResponse;
import org.ros.node.service.ServiceClient;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import services.Velocity3dService;

/** @author Hoang Tung Dinh */
final class CratesVelocity3dService implements Velocity3dService {
  private static final Logger logger = LoggerFactory.getLogger(CratesVelocity3dService.class);
  private final ServiceClient<VelocityRequest, VelocityResponse> srvVelocity;

  private CratesVelocity3dService(ServiceClient<VelocityRequest, VelocityResponse> srvVelocity) {
    this.srvVelocity = srvVelocity;
  }

  /**
   * Create a 3d-velocity service for a drone in the Crates simulator.
   *
   * @param srvVelocity the service client connected to the {@code Velocity} rosservice of the drone
   * @return a 3d-velocity service
   */
  public static CratesVelocity3dService create(
      ServiceClient<VelocityRequest, VelocityResponse> srvVelocity) {
    return new CratesVelocity3dService(srvVelocity);
  }

  @Override
  public void sendVelocity3dMessage(
      double inertialFrameVelocityX,
      double inertialFrameVelocityY,
      double inertialFrameVelocityZ,
      double angularPositionZ) {
    final VelocityRequest velocityRequest = srvVelocity.newMessage();
    velocityRequest.setDx(inertialFrameVelocityX);
    velocityRequest.setDy(inertialFrameVelocityY);
    velocityRequest.setDz(inertialFrameVelocityZ);
    velocityRequest.setYaw(angularPositionZ);
    logger.trace(
        "Sending 3d velocity: [velX={} velY={} velZ={} posYaw={}]",
        inertialFrameVelocityX,
        inertialFrameVelocityY,
        inertialFrameVelocityZ,
        angularPositionZ);
    CratesUtilities.sendRequest(srvVelocity, velocityRequest);
  }
}
