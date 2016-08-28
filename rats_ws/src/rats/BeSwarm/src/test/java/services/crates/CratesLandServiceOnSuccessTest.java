package services.crates;

import hal_quadrotor.LandResponse;
import org.ros.node.service.ServiceResponseListener;

import static org.mockito.Mockito.mock;

/** @author Hoang Tung Dinh */
public class CratesLandServiceOnSuccessTest extends CratesLandServiceTest {
  @Override
  void responseToMessage(ServiceResponseListener<LandResponse> serviceResponseListener) {
    serviceResponseListener.onSuccess(mock(LandResponse.class));
  }
}
