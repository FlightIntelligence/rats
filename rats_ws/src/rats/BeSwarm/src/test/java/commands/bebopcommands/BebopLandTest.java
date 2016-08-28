package commands.bebopcommands;

import com.google.common.base.Optional;
import commands.Command;
import junitparams.JUnitParamsRunner;
import junitparams.Parameters;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import services.FlyingStateService;
import services.LandService;
import services.rossubscribers.FlyingState;

import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

import static com.google.common.truth.Truth.assertThat;
import static org.mockito.Mockito.atLeast;
import static org.mockito.Mockito.atMost;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.reset;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

/**
 * Tests for {@link BebopLand}.
 *
 * @author Hoang Tung Dinh
 */
@RunWith(JUnitParamsRunner.class)
public class BebopLandTest {

  private LandService landService;
  private FlyingStateService flyingStateService;
  private Future<?> future;

  @Before
  public void setUp() throws InterruptedException {
    landService = mock(LandService.class);
    flyingStateService = mock(FlyingStateService.class);
    final Command landCommand = BebopLand.create(landService, flyingStateService);

    when(flyingStateService.getCurrentFlyingState()).thenReturn(Optional.<FlyingState>absent());

    future =
        Executors.newSingleThreadExecutor()
            .submit(
                new Runnable() {
                  @Override
                  public void run() {
                    landCommand.execute();
                  }
                });
  }

  @After
  public void tearDown() {
    future.cancel(true);
  }

  @Test
  public void testExecute_noStateReceived() throws InterruptedException {
    reset(landService);
    TimeUnit.MILLISECONDS.sleep(200);
    verify(landService, atLeast(2)).sendLandingMessage();
    assertThat(future.isDone()).isFalse();
  }

  @Test
  public void testExecute_landingStateReceived() throws InterruptedException {
    when(flyingStateService.getCurrentFlyingState()).thenReturn(Optional.of(FlyingState.LANDING));
    reset(landService);
    TimeUnit.MILLISECONDS.sleep(200);
    verify(landService, atMost(1)).sendLandingMessage();
    assertThat(future.isDone()).isFalse();
  }

  @Test
  public void testExecute_landedStateReceived() throws InterruptedException {
    when(flyingStateService.getCurrentFlyingState()).thenReturn(Optional.of(FlyingState.LANDED));
    reset(landService);
    TimeUnit.MILLISECONDS.sleep(200);
    verify(landService, atMost(1)).sendLandingMessage();
    assertThat(future.isDone()).isTrue();
  }

  @Test
  @Parameters(method = "flyingStateValues")
  public void testExecute_otherStatesReceived(Optional<FlyingState> flyingStateOptional)
      throws InterruptedException {
    when(flyingStateService.getCurrentFlyingState()).thenReturn(flyingStateOptional);
    reset(landService);
    TimeUnit.MILLISECONDS.sleep(200);
    verify(landService, atLeast(2)).sendLandingMessage();
    assertThat(future.isDone()).isFalse();
  }

  private Object[] flyingStateValues() {
    return new Object[] {
      new Object[] {Optional.of(FlyingState.TAKING_OFF)},
      new Object[] {Optional.of(FlyingState.HOVERING)},
      new Object[] {Optional.of(FlyingState.FLYING)},
      new Object[] {Optional.of(FlyingState.EMERGENCY)},
      new Object[] {Optional.of(FlyingState.USER_TAKEOFF)},
      new Object[] {Optional.of(FlyingState.UNKNOWN)}
    };
  }
}
