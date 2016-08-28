package commands;

import com.google.auto.value.AutoValue;
import com.google.common.base.Optional;
import control.PidParameters;
import control.dto.DroneStateStamped;
import control.dto.InertialFrameVelocity;
import control.dto.Pose;
import control.localization.StateEstimator;
import org.junit.Before;
import org.junit.Test;
import services.Velocity4dService;
import time.TimeProvider;

import static org.mockito.Matchers.any;
import static org.mockito.Mockito.RETURNS_MOCKS;
import static org.mockito.Mockito.atLeastOnce;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

/** @author Hoang Tung Dinh */
public abstract class AbstractBuilderTest {

  private static final double DURATION_IN_SECONDS = 0.5;
  private Velocity4dService velocity4dService;
  private StateEstimator stateEstimator;
  private PidParameters pidLinearX;
  private PidParameters pidLinearY;
  private PidParameters pidLinearZ;
  private PidParameters pidAngularZ;
  private TimeProvider timeProvider;

  private static void checkCorrectPidParametersCalled(PidParameters pidParameters) {
    verify(pidParameters, atLeastOnce()).lagTimeInSeconds();
    verify(pidParameters, atLeastOnce()).kp();
    verify(pidParameters, atLeastOnce()).ki();
    verify(pidParameters, atLeastOnce()).kd();
    verify(pidParameters, atLeastOnce()).maxIntegralError();
    verify(pidParameters, atLeastOnce()).minIntegralError();
    verify(pidParameters, atLeastOnce()).maxVelocity();
    verify(pidParameters, atLeastOnce()).minVelocity();
  }

  private static void checkCorrectServicesCalled(
      Velocity4dService velocity4dService,
      StateEstimator stateEstimator,
      TimeProvider timeProvider) {
    verify(velocity4dService, atLeastOnce())
        .sendInertialFrameVelocity(any(InertialFrameVelocity.class), any(Pose.class));
    verify(stateEstimator, atLeastOnce()).getCurrentState();
    verify(timeProvider, atLeastOnce()).getCurrentTimeSeconds();
  }

  abstract void createAndExecuteCommand(ArgumentHolder argumentHolder);

  abstract void checkCorrectExtraMethodsCalled();

  @Before
  public void setUp() {
    velocity4dService = mock(Velocity4dService.class, RETURNS_MOCKS);
    stateEstimator = mock(StateEstimator.class, RETURNS_MOCKS);
    when(stateEstimator.getCurrentState())
        .thenReturn(Optional.of(mock(DroneStateStamped.class, RETURNS_MOCKS)));

    pidLinearX = mock(PidParameters.class, RETURNS_MOCKS);
    pidLinearY = mock(PidParameters.class, RETURNS_MOCKS);
    pidLinearZ = mock(PidParameters.class, RETURNS_MOCKS);
    pidAngularZ = mock(PidParameters.class, RETURNS_MOCKS);
    timeProvider = mock(TimeProvider.class);
  }

  @Test
  public void testCorrectArgumentsCalled() {
    final ArgumentHolder argumentHolder =
        ArgumentHolder.builder()
            .velocityService(velocity4dService)
            .stateEstimator(stateEstimator)
            .pidLinearX(pidLinearX)
            .pidLinearY(pidLinearY)
            .pidLinearZ(pidLinearZ)
            .pidAngularZ(pidAngularZ)
            .durationInSeconds(DURATION_IN_SECONDS)
            .timeProvider(timeProvider)
            .build();

    createAndExecuteCommand(argumentHolder);
    checkCorrectExtraMethodsCalled();

    checkCorrectServicesCalled(velocity4dService, stateEstimator, timeProvider);
    checkCorrectPidParametersCalled(pidLinearX);
    checkCorrectPidParametersCalled(pidLinearY);
    checkCorrectPidParametersCalled(pidLinearZ);
    checkCorrectPidParametersCalled(pidAngularZ);
  }

  @AutoValue
  abstract static class ArgumentHolder {
    public static Builder builder() {
      return new AutoValue_AbstractBuilderTest_ArgumentHolder.Builder();
    }

    abstract Velocity4dService velocityService();

    abstract StateEstimator stateEstimator();

    abstract PidParameters pidLinearX();

    abstract PidParameters pidLinearY();

    abstract PidParameters pidLinearZ();

    abstract PidParameters pidAngularZ();

    abstract double durationInSeconds();

    abstract TimeProvider timeProvider();

    @AutoValue.Builder
    public abstract static class Builder {
      public abstract Builder velocityService(Velocity4dService value);

      public abstract Builder stateEstimator(StateEstimator value);

      public abstract Builder pidLinearX(PidParameters value);

      public abstract Builder pidLinearY(PidParameters value);

      public abstract Builder pidLinearZ(PidParameters value);

      public abstract Builder pidAngularZ(PidParameters value);

      public abstract Builder durationInSeconds(double value);

      public abstract Builder timeProvider(TimeProvider value);

      public abstract ArgumentHolder build();
    }
  }
}
