package services.crates;

import hal_quadrotor.TakeoffResponse;
import org.ros.exception.RemoteException;
import org.ros.node.service.ServiceResponseListener;

import static org.mockito.Mockito.mock;

/** @author Hoang Tung Dinh */
public class CratesTakeOffServiceOnDefaultAltitudeOnFailureTest
    extends CratesTakeOffServiceOnDefaultAltitudeTest {
  @Override
  void responseToMessage(ServiceResponseListener<TakeoffResponse> serviceResponseListener) {
    serviceResponseListener.onFailure(mock(RemoteException.class));
  }
}
