package services.crates;

import hal_quadrotor.HoverRequest;
import hal_quadrotor.HoverResponse;
import org.ros.node.service.ServiceClient;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import services.HoverService;

/**
 * AbstractHover service for drones in Crates simulator.
 *
 * @author Hoang Tung Dinh
 */
final class CratesHoverService implements HoverService {

  private static final Logger logger = LoggerFactory.getLogger(CratesHoverService.class);
  private final ServiceClient<HoverRequest, HoverResponse> srvHover;

  private CratesHoverService(ServiceClient<HoverRequest, HoverResponse> srvHover) {
    this.srvHover = srvHover;
  }

  /**
   * Creates an instance of {@link CratesHoverService}.
   *
   * @param srvHover the ros service client for hovering
   * @return an instance of {@link CratesHoverService}
   */
  public static CratesHoverService create(ServiceClient<HoverRequest, HoverResponse> srvHover) {
    return new CratesHoverService(srvHover);
  }

  @Override
  public void sendHoverMessage() {
    final HoverRequest hoverRequest = srvHover.newMessage();
    logger.debug("Send hover request");
    CratesUtilities.sendRequest(srvHover, hoverRequest);
  }
}
