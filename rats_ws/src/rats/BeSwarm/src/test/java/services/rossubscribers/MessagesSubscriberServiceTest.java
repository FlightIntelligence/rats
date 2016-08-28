package services.rossubscribers;

import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import org.ros.internal.message.Message;
import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;
import time.TimeProvider;

import static com.google.common.truth.Truth.assertThat;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

/** @author Hoang Tung Dinh */
public abstract class MessagesSubscriberServiceTest<T extends Message> {

  private Subscriber<T> subscriber;

  abstract Subscriber<T> createSubscriber();

  abstract T createNewMessage(String messageName);

  @Before
  public void setUp() {
    subscriber = createSubscriber();
  }

  @Test
  public void testWithQueueSizeOfOne() {
    final MessagesSubscriberService<T> messagesSubscriberService =
        MessagesSubscriberService.create(subscriber, mock(TimeProvider.class));
    final ArgumentCaptor<MessageListener> argumentCaptor = getMessageListenerArgumentCaptor();
    testAddListenerAndAbsentMostRecentMessage(messagesSubscriberService, argumentCaptor);
    testAddOneAndGetMostRecentMessage(messagesSubscriberService, argumentCaptor);
  }

  @Test
  public void testWithQueueSizeOfThree() {
    final MessagesSubscriberService<T> messagesSubscriberService =
        MessagesSubscriberService.create(subscriber, 3, mock(TimeProvider.class));
    final ArgumentCaptor<MessageListener> argumentCaptor = getMessageListenerArgumentCaptor();
    testAddListenerAndAbsentMostRecentMessage(messagesSubscriberService, argumentCaptor);
    testAddOneAndGetMostRecentMessage(messagesSubscriberService, argumentCaptor);
    testAddFourMessages(messagesSubscriberService, argumentCaptor);
  }

  private void testAddFourMessages(
      MessagesSubscriberService<T> messagesSubscriberService,
      ArgumentCaptor<MessageListener> argumentCaptor) {
    final Message firstMessage = createNewMessage("first message");
    final Message secondMessage = createNewMessage("second message");
    final Message thirdMessage = createNewMessage("third message");
    final Message fourthMessage = createNewMessage("fourth message");
    argumentCaptor.getValue().onNewMessage(firstMessage);
    argumentCaptor.getValue().onNewMessage(secondMessage);
    argumentCaptor.getValue().onNewMessage(thirdMessage);
    argumentCaptor.getValue().onNewMessage(fourthMessage);
    assertThat(messagesSubscriberService.getMostRecentMessage()).hasValue(fourthMessage);
    assertThat(messagesSubscriberService.getMessageQueue())
        .containsExactly(secondMessage, thirdMessage, fourthMessage)
        .inOrder();
  }

  private void testAddListenerAndAbsentMostRecentMessage(
      MessagesSubscriberService<T> messagesSubscriberService,
      ArgumentCaptor<MessageListener> argumentCaptor) {
    verify(subscriber).addMessageListener(argumentCaptor.capture());
    assertThat(messagesSubscriberService.getMostRecentMessage()).isAbsent();
  }

  private ArgumentCaptor<MessageListener> getMessageListenerArgumentCaptor() {
    return ArgumentCaptor.forClass(MessageListener.class);
  }

  private void testAddOneAndGetMostRecentMessage(
      MessagesSubscriberService<T> messagesSubscriberService,
      ArgumentCaptor<MessageListener> argumentCaptor) {
    final Message newMessage = createNewMessage("message");
    argumentCaptor.getValue().onNewMessage(newMessage);
    assertThat(messagesSubscriberService.getMostRecentMessage()).hasValue(newMessage);
  }
}
