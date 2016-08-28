package services.crates;

import hal_quadrotor.LandRequest;
import hal_quadrotor.LandResponse;
import org.ros.node.service.ServiceClient;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import services.LandService;

/**
 * AbstractParrotLand service for drones in Crates simulator.
 *
 * @author Hoang Tung Dinh
 */
final class CratesLandService implements LandService {
  private static final Logger logger = LoggerFactory.getLogger(CratesLandService.class);
  private final ServiceClient<LandRequest, LandResponse> srvLand;

  private CratesLandService(ServiceClient<LandRequest, LandResponse> srvLand) {
    this.srvLand = srvLand;
  }

  /**
   * Creates an instance of {@link CratesLandService}.
   *
   * @param srvLand the ros service client for landing
   * @return an instance of {@link CratesLandService}
   */
  public static CratesLandService create(ServiceClient<LandRequest, LandResponse> srvLand) {
    return new CratesLandService(srvLand);
  }

  @Override
  public void sendLandingMessage() {
    final LandRequest landRequest = srvLand.newMessage();
    logger.debug("Send landing messages.");
    CratesUtilities.sendRequest(srvLand, landRequest);
  }
}
