package services;

import com.google.common.collect.ImmutableMap;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import org.ros.internal.message.Message;
import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;
import services.rossubscribers.FlyingState;
import services.rossubscribers.MessagesSubscriberService;
import time.TimeProvider;

import java.util.Map;

import static com.google.common.truth.Truth.assertThat;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

/** @author Hoang Tung Dinh */
public abstract class FlyingStateServiceTest<T, U extends Message> {

  public abstract ImmutableMap<T, FlyingState> getFlyingStateMap();

  public abstract FlyingStateService createFlyingStateService(
      MessagesSubscriberService<U> messagesSubscriberService);

  public abstract U createMockStateMessage(T newStateCode);

  @Test
  public void testUpdateFlyingState() {
    final Subscriber<U> subscriber = mock(Subscriber.class);
    final MessagesSubscriberService<U> flyingStateSubscriber =
        MessagesSubscriberService.create(subscriber, mock(TimeProvider.class));

    final ArgumentCaptor<MessageListener> argumentCaptor =
        ArgumentCaptor.forClass(MessageListener.class);
    verify(subscriber).addMessageListener(argumentCaptor.capture());
    final MessageListener<U> messageListener = argumentCaptor.getValue();

    final FlyingStateService flyingStateService = createFlyingStateService(flyingStateSubscriber);
    assertThat(flyingStateService.getCurrentFlyingState()).isAbsent();

    final ImmutableMap<T, FlyingState> flyingStateMap = getFlyingStateMap();

    for (final Map.Entry<T, FlyingState> entry : flyingStateMap.entrySet()) {
      checkUpdateNewState(messageListener, flyingStateService, entry.getKey(), entry.getValue());
      // this is for the case receiving the same state
      checkUpdateNewState(messageListener, flyingStateService, entry.getKey(), entry.getValue());
    }
  }

  private void checkUpdateNewState(
      MessageListener<U> messageListener,
      FlyingStateService bebopFlyingStateService,
      T newStateCode,
      FlyingState newState) {
    final U state = createMockStateMessage(newStateCode);
    messageListener.onNewMessage(state);
    assertThat(bebopFlyingStateService.getCurrentFlyingState()).hasValue(newState);
  }
}
