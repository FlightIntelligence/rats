package services.rossubscribers;

import bebop_msgs.Ardrone3PilotingStateFlyingStateChanged;
import org.ros.node.topic.Subscriber;

import static org.mockito.Mockito.RETURNS_MOCKS;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.withSettings;

/** @author Hoang Tung Dinh */
public class FlyingStateMessagesSubscriberServiceTest
    extends MessagesSubscriberServiceTest<Ardrone3PilotingStateFlyingStateChanged> {

  @Override
  Subscriber<Ardrone3PilotingStateFlyingStateChanged> createSubscriber() {
    return mock(Subscriber.class);
  }

  @Override
  Ardrone3PilotingStateFlyingStateChanged createNewMessage(String messageName) {
    return mock(
        Ardrone3PilotingStateFlyingStateChanged.class,
        withSettings().defaultAnswer(RETURNS_MOCKS).name(messageName));
  }
}
