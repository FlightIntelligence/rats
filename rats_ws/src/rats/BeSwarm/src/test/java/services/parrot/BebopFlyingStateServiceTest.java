package services.parrot;

import bebop_msgs.Ardrone3PilotingStateFlyingStateChanged;
import com.google.common.collect.ImmutableMap;
import services.FlyingStateService;
import services.FlyingStateServiceTest;
import services.rossubscribers.FlyingState;
import services.rossubscribers.MessagesSubscriberService;

import static org.mockito.Mockito.RETURNS_MOCKS;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

/** @author Hoang Tung Dinh */
public class BebopFlyingStateServiceTest
    extends FlyingStateServiceTest<Byte, Ardrone3PilotingStateFlyingStateChanged> {
  @Override
  public ImmutableMap<Byte, FlyingState> getFlyingStateMap() {
    return ImmutableMap.<Byte, FlyingState>builder()
        .put((byte) 0, FlyingState.LANDED)
        .put((byte) 1, FlyingState.TAKING_OFF)
        .put((byte) 2, FlyingState.HOVERING)
        .put((byte) 3, FlyingState.FLYING)
        .put((byte) 4, FlyingState.LANDING)
        .put((byte) 5, FlyingState.EMERGENCY)
        .put((byte) 6, FlyingState.USER_TAKEOFF)
        .build();
  }

  @Override
  public FlyingStateService createFlyingStateService(
      MessagesSubscriberService<Ardrone3PilotingStateFlyingStateChanged>
          messagesSubscriberService) {
    return BebopFlyingStateService.create(messagesSubscriberService);
  }

  @Override
  public Ardrone3PilotingStateFlyingStateChanged createMockStateMessage(Byte newStateCode) {
    final Ardrone3PilotingStateFlyingStateChanged state =
        mock(Ardrone3PilotingStateFlyingStateChanged.class, RETURNS_MOCKS);
    when(state.getState()).thenReturn(newStateCode);
    return state;
  }
}
