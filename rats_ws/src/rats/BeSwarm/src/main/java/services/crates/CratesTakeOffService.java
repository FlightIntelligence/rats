package services.crates;

import com.google.common.annotations.VisibleForTesting;
import hal_quadrotor.TakeoffRequest;
import hal_quadrotor.TakeoffResponse;
import org.ros.node.service.ServiceClient;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import services.TakeOffService;

/** @author Hoang Tung Dinh */
final class CratesTakeOffService implements TakeOffService {
  @VisibleForTesting static final double DEFAULT_TAKE_OFF_ALTITUDE = 5;
  private static final Logger logger = LoggerFactory.getLogger(CratesTakeOffService.class);
  private final ServiceClient<TakeoffRequest, TakeoffResponse> srvTakeOff;

  private CratesTakeOffService(ServiceClient<TakeoffRequest, TakeoffResponse> srvTakeOff) {
    this.srvTakeOff = srvTakeOff;
  }

  /**
   * Creates a take off service for a drone in the crates simulator.
   *
   * @param srvTakeOff the service client connected to the take of rosservice
   * @return a take off service
   */
  public static CratesTakeOffService create(
      ServiceClient<TakeoffRequest, TakeoffResponse> srvTakeOff) {
    return new CratesTakeOffService(srvTakeOff);
  }

  @Override
  public void sendTakingOffMessage() {
    logger.debug("Send taking off messages.");
    final TakeoffRequest takeoffRequest = srvTakeOff.newMessage();
    takeoffRequest.setAltitude(DEFAULT_TAKE_OFF_ALTITUDE);
    CratesUtilities.sendRequest(srvTakeOff, takeoffRequest);
  }
}
