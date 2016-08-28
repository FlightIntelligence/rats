package services.crates;

import com.google.common.collect.ImmutableMap;
import hal_quadrotor.State;
import services.FlyingStateService;
import services.FlyingStateServiceTest;
import services.rossubscribers.FlyingState;
import services.rossubscribers.MessagesSubscriberService;

import static org.mockito.Mockito.RETURNS_MOCKS;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

/** @author Hoang Tung Dinh */
public class CratesFlyingStateServiceTest extends FlyingStateServiceTest<String, State> {
  @Override
  public ImmutableMap<String, FlyingState> getFlyingStateMap() {
    return ImmutableMap.<String, FlyingState>builder()
        .put("AnglesHeight", FlyingState.FLYING)
        .put("Emergency", FlyingState.EMERGENCY)
        .put("AbstractHover", FlyingState.HOVERING)
        .put("Idle", FlyingState.LANDED)
        .put("AbstractParrotLand", FlyingState.LANDING)
        .put("AbstractParrotTakeOff", FlyingState.TAKING_OFF)
        .put("VelocityHeight", FlyingState.FLYING)
        .put("Velocity", FlyingState.FLYING)
        .put("Waypoint", FlyingState.FLYING)
        .build();
  }

  @Override
  public FlyingStateService createFlyingStateService(
      MessagesSubscriberService<State> messagesSubscriberService) {
    return CratesFlyingStateService.create(messagesSubscriberService);
  }

  @Override
  public State createMockStateMessage(String newStateCode) {
    final State state = mock(State.class, RETURNS_MOCKS);
    when(state.getController()).thenReturn(newStateCode);
    return state;
  }
}
