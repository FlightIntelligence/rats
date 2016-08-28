package services.parrot;

import org.junit.Test;
import org.mockito.ArgumentCaptor;
import org.ros.internal.message.Message;
import org.ros.node.topic.Publisher;

import static com.google.common.truth.Truth.assertThat;
import static org.mockito.Mockito.RETURNS_DEEP_STUBS;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

/**
 * Abstract class providing a test for simple parrot services.
 *
 * @author Hoang Tung Dinh
 */
public abstract class AbstractParrotServiceTest<T extends Message> {

  abstract String createTopicName();

  abstract T createNewMessage();

  abstract void createServiceAndSendMessage(Publisher<T> publisher);

  abstract ArgumentCaptor<T> createArgumentCaptor();

  @Test
  public void testSendMessage() {
    final Publisher<T> publisher = mock(Publisher.class, RETURNS_DEEP_STUBS);
    when(publisher.getTopicName().toString()).thenReturn(createTopicName());

    final T message = createNewMessage();
    when(publisher.newMessage()).thenReturn(message);

    createServiceAndSendMessage(publisher);

    final ArgumentCaptor<T> argumentCaptor = createArgumentCaptor();
    verify(publisher).publish(argumentCaptor.capture());
    assertThat(argumentCaptor.getValue()).isEqualTo(message);
  }
}
