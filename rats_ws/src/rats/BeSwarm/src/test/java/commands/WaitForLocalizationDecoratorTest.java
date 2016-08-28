package commands;

import com.google.common.base.Optional;
import control.dto.DroneStateStamped;
import control.localization.StateEstimator;
import org.junit.Test;

import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

import static org.mockito.Mockito.RETURNS_DEEP_STUBS;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

/** @author Hoang Tung Dinh */
public class WaitForLocalizationDecoratorTest {

  @Test
  public void testWaitForLocalization() throws InterruptedException {
    final Command toBeExecutedCommand = mock(Command.class);
    final StateEstimator stateEstimator = mock(StateEstimator.class);

    final Command waitForLocalization =
        WaitForLocalizationDecorator.create(stateEstimator, toBeExecutedCommand);

    when(stateEstimator.getCurrentState()).thenReturn(Optional.<DroneStateStamped>absent());

    final Future<?> future =
        Executors.newSingleThreadExecutor()
            .submit(
                new Runnable() {
                  @Override
                  public void run() {
                    waitForLocalization.execute();
                  }
                });

    TimeUnit.MILLISECONDS.sleep(100);
    verify(toBeExecutedCommand, never()).execute();

    final DroneStateStamped firstState = mock(DroneStateStamped.class, RETURNS_DEEP_STUBS);
    when(firstState.getTimeStampInSeconds()).thenReturn(10d);

    when(stateEstimator.getCurrentState()).thenReturn(Optional.of(firstState));
    TimeUnit.MILLISECONDS.sleep(100);
    verify(toBeExecutedCommand, never()).execute();

    final DroneStateStamped secondState = mock(DroneStateStamped.class, RETURNS_DEEP_STUBS);
    when(secondState.getTimeStampInSeconds()).thenReturn(11d);

    when(stateEstimator.getCurrentState()).thenReturn(Optional.of(secondState));
    TimeUnit.MILLISECONDS.sleep(100);
    verify(toBeExecutedCommand).execute();

    future.cancel(true);
  }
}
