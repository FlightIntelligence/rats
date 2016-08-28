package services.crates;

import hal_quadrotor.LandRequest;
import hal_quadrotor.LandResponse;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import services.LandService;

import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

import static com.google.common.truth.Truth.assertThat;
import static org.mockito.Mockito.any;
import static org.mockito.Mockito.atLeastOnce;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

/** @author Hoang Tung Dinh */
public abstract class CratesLandServiceTest {

  private ServiceClient<LandRequest, LandResponse> serviceClient;
  private ArgumentCaptor<ServiceResponseListener> argumentCaptor;
  private LandService cratesLandService;
  private Future<?> future;

  private static void checkServiceCalledAndIsWaiting(
      ServiceClient<LandRequest, LandResponse> serviceClient,
      ArgumentCaptor<ServiceResponseListener> argumentCaptor,
      Future<?> future) {
    verify(serviceClient, atLeastOnce()).call(any(LandRequest.class), argumentCaptor.capture());
    assertThat(future.isDone()).isFalse();
  }

  abstract void responseToMessage(ServiceResponseListener<LandResponse> serviceResponseListener);

  @Before
  public void setUp() throws InterruptedException {
    serviceClient = mock(ServiceClient.class);
    argumentCaptor = ArgumentCaptor.forClass(ServiceResponseListener.class);

    cratesLandService = CratesLandService.create(serviceClient);
    future =
        Executors.newSingleThreadExecutor()
            .submit(
                new Runnable() {
                  @Override
                  public void run() {
                    cratesLandService.sendLandingMessage();
                  }
                });

    TimeUnit.MILLISECONDS.sleep(300);
  }

  @After
  public void tearDown() {
    future.cancel(true);
  }

  @Test
  public void testSendLandMessage() throws InterruptedException {
    checkServiceCalledAndIsWaiting(serviceClient, argumentCaptor, future);

    final ServiceResponseListener<LandResponse> serviceResponseListener = argumentCaptor.getValue();
    responseToMessage(serviceResponseListener);

    TimeUnit.MILLISECONDS.sleep(50);
    assertThat(future.isDone()).isTrue();
  }
}
