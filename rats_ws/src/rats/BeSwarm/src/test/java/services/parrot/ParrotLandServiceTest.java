package services.parrot;

import org.mockito.ArgumentCaptor;
import org.ros.node.topic.Publisher;
import services.LandService;
import std_msgs.Empty;

import static org.mockito.Mockito.mock;

/** @author Hoang Tung Dinh */
public class ParrotLandServiceTest extends AbstractParrotServiceTest<Empty> {

  @Override
  String createTopicName() {
    return "/bebop/land";
  }

  @Override
  Empty createNewMessage() {
    return mock(Empty.class);
  }

  @Override
  void createServiceAndSendMessage(Publisher<Empty> publisher) {
    final LandService landService = ParrotLandService.create(publisher);
    landService.sendLandingMessage();
  }

  @Override
  ArgumentCaptor<Empty> createArgumentCaptor() {
    return ArgumentCaptor.forClass(Empty.class);
  }
}
