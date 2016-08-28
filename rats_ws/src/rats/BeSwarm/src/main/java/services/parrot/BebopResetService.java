package services.parrot;

import org.ros.node.topic.Publisher;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import services.ResetService;
import std_msgs.Empty;

/**
 * Bebop service to sendResetMessage the bebop drone.
 *
 * @author Hoang Tung Dinh
 */
final class BebopResetService implements ResetService {

  private static final Logger logger = LoggerFactory.getLogger(BebopResetService.class);
  private final Publisher<Empty> publisher;

  private BebopResetService(Publisher<Empty> publisher) {
    this.publisher = publisher;
  }

  /**
   * Creates a {@link BebopResetService} instance.
   *
   * @param publisher the publisher associated to the sendResetMessage ros topic
   * @return a {@link BebopResetService} instance
   */
  public static BebopResetService create(Publisher<Empty> publisher) {
    return new BebopResetService(publisher);
  }

  @Override
  public void sendResetMessage() {
    final Empty empty = publisher.newMessage();
    publisher.publish(empty);
    logger.debug("Sent a sendResetMessage message to {}.", publisher.getTopicName());
  }
}
