package services.crates;

import hal_quadrotor.TakeoffRequest;
import hal_quadrotor.TakeoffResponse;
import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import services.TakeOffService;

import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

import static com.google.common.truth.Truth.assertThat;
import static org.mockito.Mockito.RETURNS_DEEP_STUBS;
import static org.mockito.Mockito.atLeastOnce;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

/** @author Hoang Tung Dinh */
public abstract class CratesTakeOffServiceTest {

  private ServiceClient<TakeoffRequest, TakeoffResponse> serviceClient;
  private ArgumentCaptor<ServiceResponseListener> serviceResponseListenerArgumentCaptor;
  private ArgumentCaptor<TakeoffRequest> takeoffRequestArgumentCaptor;
  private TakeOffService cratesTakeOffService;

  abstract Future<?> sendMessage(TakeOffService cratesTakeOffService);

  abstract void responseToMessage(ServiceResponseListener<TakeoffResponse> serviceResponseListener);

  abstract void checkCorrectSentAltitude(TakeoffRequest takeoffRequest);

  @Before
  public void setUp() {
    serviceClient = mock(ServiceClient.class, RETURNS_DEEP_STUBS);
    when(serviceClient.newMessage()).thenReturn(mock(TakeoffRequest.class));
    serviceResponseListenerArgumentCaptor = ArgumentCaptor.forClass(ServiceResponseListener.class);
    takeoffRequestArgumentCaptor = ArgumentCaptor.forClass(TakeoffRequest.class);
    cratesTakeOffService = CratesTakeOffService.create(serviceClient);
  }

  @Test
  public void testSendDefaultTakeOffMessage() throws InterruptedException {
    final Future<?> future = sendMessage(cratesTakeOffService);
    TimeUnit.MILLISECONDS.sleep(300);

    final TakeoffRequest sentTakeoffRequest = getTakeoffRequest(future);
    checkCorrectSentAltitude(sentTakeoffRequest);

    final ServiceResponseListener<TakeoffResponse> serviceResponseListener =
        serviceResponseListenerArgumentCaptor.getValue();
    responseToMessage(serviceResponseListener);

    TimeUnit.MILLISECONDS.sleep(50);
    assertThat(future.isDone()).isTrue();
  }

  private TakeoffRequest getTakeoffRequest(Future<?> future) throws InterruptedException {
    checkMessageSentButNotDone(future);
    return takeoffRequestArgumentCaptor.getValue();
  }

  private void checkMessageSentButNotDone(Future<?> future) {
    assertThat(future.isDone()).isFalse();
    verify(serviceClient, atLeastOnce())
        .call(
            takeoffRequestArgumentCaptor.capture(),
            serviceResponseListenerArgumentCaptor.capture());
  }
}
