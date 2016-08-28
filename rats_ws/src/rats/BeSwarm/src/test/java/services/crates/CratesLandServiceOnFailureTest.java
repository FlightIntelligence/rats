package services.crates;

import hal_quadrotor.LandResponse;
import org.ros.exception.RemoteException;
import org.ros.node.service.ServiceResponseListener;

import static org.mockito.Mockito.mock;

/** @author Hoang Tung Dinh */
public class CratesLandServiceOnFailureTest extends CratesLandServiceTest {
  @Override
  void responseToMessage(ServiceResponseListener<LandResponse> serviceResponseListener) {
    serviceResponseListener.onFailure(mock(RemoteException.class));
  }
}
