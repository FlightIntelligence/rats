package services.parrot;

import org.mockito.ArgumentCaptor;
import org.ros.node.topic.Publisher;
import services.ResetService;
import std_msgs.Empty;

import static org.mockito.Mockito.mock;

/** @author Hoang Tung Dinh */
public class BebopResetServiceTest extends AbstractParrotServiceTest<Empty> {

  @Override
  String createTopicName() {
    return "/bebop/sendResetMessage";
  }

  @Override
  Empty createNewMessage() {
    return mock(Empty.class);
  }

  @Override
  void createServiceAndSendMessage(Publisher<Empty> publisher) {
    final ResetService resetService = BebopResetService.create(publisher);
    resetService.sendResetMessage();
  }

  @Override
  ArgumentCaptor<Empty> createArgumentCaptor() {
    return ArgumentCaptor.forClass(Empty.class);
  }
}
