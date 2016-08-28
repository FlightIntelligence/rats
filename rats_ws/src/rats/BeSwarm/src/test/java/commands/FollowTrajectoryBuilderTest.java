package commands;

import commands.bebopcommands.BebopFollowTrajectory;
import control.PidParameters;
import control.Trajectory4d;
import control.localization.StateEstimator;
import org.junit.Before;
import org.junit.Test;
import services.Velocity4dService;
import utils.TestUtils;

import static org.mockito.Mockito.RETURNS_MOCKS;
import static org.mockito.Mockito.mock;

/**
 * Tests for {@link BebopFollowTrajectory}'s builder.
 *
 * @author Hoang Tung Dinh
 */
public class FollowTrajectoryBuilderTest extends AbstractBuilderTest {

  private Trajectory4d trajectory4d;

  @Override
  @Before
  public void setUp() {
    super.setUp();
    trajectory4d = mock(Trajectory4d.class, RETURNS_MOCKS);
  }

  @Override
  void createAndExecuteCommand(ArgumentHolder argumentHolder) {
    final Command followTrajectory =
        BebopFollowTrajectory.builder()
            .withVelocity4dService(argumentHolder.velocityService())
            .withStateEstimator(argumentHolder.stateEstimator())
            .withTimeProvider(argumentHolder.timeProvider())
            .withTrajectory4d(trajectory4d)
            .withDurationInSeconds(argumentHolder.durationInSeconds())
            .withPidLinearXParameters(argumentHolder.pidLinearX())
            .withPidLinearYParameters(argumentHolder.pidLinearY())
            .withPidLinearZParameters(argumentHolder.pidLinearZ())
            .withPidAngularZParameters(argumentHolder.pidAngularZ())
            .build();

    followTrajectory.execute();
  }

  @Override
  void checkCorrectExtraMethodsCalled() {
    TestUtils.verifyTrajectoryCalled(trajectory4d);
  }

  @Test(expected = NullPointerException.class)
  public void testMissingTimeProvider() {
    BebopFollowTrajectory.builder()
        .withVelocity4dService(mock(Velocity4dService.class))
        .withStateEstimator(mock(StateEstimator.class))
        .withTrajectory4d(mock(Trajectory4d.class))
        .withDurationInSeconds(1)
        .withPidLinearXParameters(mock(PidParameters.class))
        .withPidLinearYParameters(mock(PidParameters.class))
        .withPidLinearZParameters(mock(PidParameters.class))
        .withPidAngularZParameters(mock(PidParameters.class))
        .build();
  }
}
