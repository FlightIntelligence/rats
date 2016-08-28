package services.parrot;

import ardrone_autonomy.Navdata;
import com.google.common.collect.ImmutableMap;
import services.FlyingStateService;
import services.FlyingStateServiceTest;
import services.rossubscribers.FlyingState;
import services.rossubscribers.MessagesSubscriberService;

import static org.mockito.Mockito.RETURNS_MOCKS;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

/** @author Hoang Tung Dinh */
public class TumSimFlyingStateServiceTest extends FlyingStateServiceTest<Integer, Navdata> {
  @Override
  public ImmutableMap<Integer, FlyingState> getFlyingStateMap() {
    return ImmutableMap.<Integer, FlyingState>builder()
        .put(0, FlyingState.UNKNOWN)
        .put(1, FlyingState.LANDED)
        .put(2, FlyingState.LANDED)
        .put(3, FlyingState.FLYING)
        .put(4, FlyingState.HOVERING)
        .put(5, FlyingState.UNKNOWN)
        .put(6, FlyingState.TAKING_OFF)
        .put(7, FlyingState.FLYING)
        .put(8, FlyingState.LANDING)
        .put(9, FlyingState.UNKNOWN)
        .build();
  }

  @Override
  public FlyingStateService createFlyingStateService(
      MessagesSubscriberService<Navdata> messagesSubscriberService) {
    return TumSimFlyingStateService.create(messagesSubscriberService);
  }

  @Override
  public Navdata createMockStateMessage(Integer newStateCode) {
    final Navdata state = mock(Navdata.class, RETURNS_MOCKS);
    when(state.getState()).thenReturn(newStateCode);
    return state;
  }
}
