package services.crates;

import hal_quadrotor.TakeoffRequest;
import services.TakeOffService;

import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import static org.mockito.Mockito.verify;

/** @author Hoang Tung Dinh */
public abstract class CratesTakeOffServiceOnDefaultAltitudeTest extends CratesTakeOffServiceTest {
  @Override
  Future<?> sendMessage(final TakeOffService cratesTakeOffService) {
    return Executors.newSingleThreadExecutor()
        .submit(
            new Runnable() {
              @Override
              public void run() {
                cratesTakeOffService.sendTakingOffMessage();
              }
            });
  }

  @Override
  void checkCorrectSentAltitude(TakeoffRequest takeoffRequest) {
    verify(takeoffRequest).setAltitude(CratesTakeOffService.DEFAULT_TAKE_OFF_ALTITUDE);
  }
}
