package services.crates;

import hal_quadrotor.VelocityRequest;
import hal_quadrotor.VelocityResponse;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import services.Velocity3dService;

import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

import static com.google.common.truth.Truth.assertThat;
import static org.mockito.Mockito.RETURNS_DEEP_STUBS;
import static org.mockito.Mockito.atLeastOnce;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

/** @author Hoang Tung Dinh */
public abstract class CratesVelocity3dServiceTest {

  private static void checkCorrectVelocitySent(
      double velX,
      double velY,
      double velZ,
      double posYaw,
      ArgumentCaptor<VelocityRequest> velocityRequestArgumentCaptor) {
    final VelocityRequest velocityRequest = velocityRequestArgumentCaptor.getValue();
    verify(velocityRequest).setDx(velX);
    verify(velocityRequest).setDy(velY);
    verify(velocityRequest).setDz(velZ);
    verify(velocityRequest).setYaw(posYaw);
  }

  abstract void responseToMessage(
      ServiceResponseListener<VelocityResponse> serviceResponseListener);

  @Test
  public void testSendVelocityMessage() throws InterruptedException {
    final ServiceClient<VelocityRequest, VelocityResponse> serviceClient =
        mock(ServiceClient.class, RETURNS_DEEP_STUBS);
    when(serviceClient.newMessage()).thenReturn(mock(VelocityRequest.class));

    final Velocity3dService cratesVelocity3dService = CratesVelocity3dService.create(serviceClient);
    final double velX = 1;
    final double velY = 2;
    final double velZ = 3;
    final double posYaw = -1;

    final Future<?> future =
        checkSendingMessageBeforeReponse(cratesVelocity3dService, velX, velY, velZ, posYaw);

    final ArgumentCaptor<VelocityRequest> velocityRequestArgumentCaptor =
        ArgumentCaptor.forClass(VelocityRequest.class);
    final ArgumentCaptor<ServiceResponseListener> serviceResponseListenerArgumentCaptor =
        ArgumentCaptor.forClass(ServiceResponseListener.class);
    verify(serviceClient, atLeastOnce())
        .call(
            velocityRequestArgumentCaptor.capture(),
            serviceResponseListenerArgumentCaptor.capture());

    checkCorrectVelocitySent(velX, velY, velZ, posYaw, velocityRequestArgumentCaptor);
    checkResponseToMessage(future, serviceResponseListenerArgumentCaptor);
  }

  private Future<?> checkSendingMessageBeforeReponse(
      final Velocity3dService velocity3dService,
      final double velX,
      final double velY,
      final double velZ,
      final double posYaw)
      throws InterruptedException {
    final Future<?> future =
        Executors.newSingleThreadExecutor()
            .submit(
                new Runnable() {
                  @Override
                  public void run() {
                    velocity3dService.sendVelocity3dMessage(velX, velY, velZ, posYaw);
                  }
                });

    TimeUnit.MILLISECONDS.sleep(300);

    assertThat(future.isDone()).isFalse();
    return future;
  }

  private void checkResponseToMessage(
      Future<?> future,
      ArgumentCaptor<ServiceResponseListener> serviceResponseListenerArgumentCaptor)
      throws InterruptedException {
    final ServiceResponseListener<VelocityResponse> serviceResponseListener =
        serviceResponseListenerArgumentCaptor.getValue();
    responseToMessage(serviceResponseListener);

    TimeUnit.MILLISECONDS.sleep(50);
    assertThat(future.isDone()).isTrue();
  }
}
