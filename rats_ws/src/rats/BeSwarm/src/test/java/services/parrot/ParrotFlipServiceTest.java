package services.parrot;

import control.dto.FlipDirection;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import org.ros.node.topic.Publisher;
import services.FlipService;
import std_msgs.UInt8;

import static com.google.common.truth.Truth.assertThat;
import static org.mockito.Mockito.RETURNS_DEEP_STUBS;
import static org.mockito.Mockito.atLeastOnce;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

/** @author Hoang Tung Dinh */
public class ParrotFlipServiceTest {

  @Test
  public void testSendFlipMessage() {
    final Publisher<UInt8> publisher = mock(Publisher.class, RETURNS_DEEP_STUBS);
    when(publisher.getTopicName().toString()).thenReturn("/bebop/flip");

    final FlipService parrotFlipService = ParrotFlipService.create(publisher);

    for (final FlipDirection flipDirection : FlipDirection.values()) {
      final UInt8 message = mock(UInt8.class);
      when(publisher.newMessage()).thenReturn(message);

      parrotFlipService.sendFlipMessage(flipDirection);

      final ArgumentCaptor<Byte> messageArgumentCaptor = ArgumentCaptor.forClass(Byte.class);
      verify(message).setData(messageArgumentCaptor.capture());
      assertThat(messageArgumentCaptor.getValue()).isEqualTo(flipDirection.getCode());

      final ArgumentCaptor<UInt8> publisherArgumentCaptor = ArgumentCaptor.forClass(UInt8.class);
      verify(publisher, atLeastOnce()).publish(publisherArgumentCaptor.capture());
      assertThat(publisherArgumentCaptor.getValue()).isEqualTo(message);
    }
  }
}
