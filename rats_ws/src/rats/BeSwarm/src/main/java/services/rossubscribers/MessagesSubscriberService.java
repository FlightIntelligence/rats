package services.rossubscribers;

import com.google.common.base.Optional;
import com.google.common.collect.EvictingQueue;
import com.google.common.collect.Queues;
import org.ros.internal.message.Message;
import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import time.TimeProvider;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedList;
import java.util.Queue;

import static com.google.common.base.Preconditions.checkArgument;

/**
 * Message subscriber service listening to messages from a ROS topic and stored them in an {@link
 * EvictingQueue}.
 *
 * @param <T> the type of the messages
 * @author mhct
 */
public class MessagesSubscriberService<T extends Message> {

  private static final Logger logger = LoggerFactory.getLogger(MessagesSubscriberService.class);
  private static final int DEFAULT_MESSAGE_QUEUE_SIZE = 1;
  private final MessagesListener<T> messagesListener;

  protected MessagesSubscriberService(
      Subscriber<T> subscriber, int maxMessageQueueSize, TimeProvider timeProvider) {
    checkArgument(
        maxMessageQueueSize >= 1,
        String.format("Queue size must be at least 1, but it is %d.", maxMessageQueueSize));
    this.messagesListener = MessagesListener.create(maxMessageQueueSize, timeProvider);
    subscriber.addMessageListener(messagesListener);
  }

  /**
   * Creates an instance of this class with the queue size of one.
   *
   * @param subscriber the rostopic subscriber
   * @param timeProvider the time provider
   * @param <U> the type of the messages
   * @return an instance of this class
   */
  public static <U extends Message> MessagesSubscriberService<U> create(
      Subscriber<U> subscriber, TimeProvider timeProvider) {
    return new MessagesSubscriberService<>(subscriber, DEFAULT_MESSAGE_QUEUE_SIZE, timeProvider);
  }

  /**
   * Creates an instance of this class.
   *
   * @param subscriber the rostopic subscriber
   * @param maxMessageQueueSize the maximum queue of most recent messages
   * @param timeProvider the time provider
   * @param <U> the type of the messages
   * @return an instance of this class
   */
  public static <U extends Message> MessagesSubscriberService<U> create(
      Subscriber<U> subscriber, int maxMessageQueueSize, TimeProvider timeProvider) {
    return new MessagesSubscriberService<>(subscriber, maxMessageQueueSize, timeProvider);
  }

  /** Returns the most recent message received. */
  public Optional<T> getMostRecentMessage() {
    return messagesListener.getMostRecentMessage();
  }

  /** Returns the queue of most recent messages. */
  public Queue<T> getMessageQueue() {
    return messagesListener.getMessageQueue();
  }

  /**
   * Registers a message observer.
   *
   * @param messageObserver the message observer to be registered
   */
  public void registerMessageObserver(MessageObserver<T> messageObserver) {
    messagesListener.registerMessageObserver(messageObserver);
  }

  /**
   * Removed a message observer.
   *
   * @param messageObserver the message observer to be removed
   */
  public void removeMessageObserver(MessageObserver<T> messageObserver) {
    messagesListener.removeMessageObserver(messageObserver);
  }

  private static final class MessagesListener<K extends Message> implements MessageListener<K> {
    private final Collection<MessageObserver<K>> messageObservers;
    private final Queue<K> messageQueue;
    @Nullable private K mostRecentMessage;
    private final TimeProvider timeProvider;

    private MessagesListener(int maxQueueSize, TimeProvider timeProvider) {
      messageQueue = Queues.synchronizedQueue(EvictingQueue.<K>create(maxQueueSize));
      messageObservers = new ArrayList<>();
      this.timeProvider = timeProvider;
    }

    /**
     * Creates an instance of this class.
     *
     * @param maxQueueSize the maximum size of the queue storing most recent messages
     * @param timeProvider the time provider
     * @param <U> the type of the message
     * @return an instance of this class
     */
    public static <U extends Message> MessagesListener<U> create(
        int maxQueueSize, TimeProvider timeProvider) {
      return new MessagesListener<>(maxQueueSize, timeProvider);
    }

    @Override
    public void onNewMessage(K newMessage) {
      logger.trace(
          "{} {}", timeProvider.getCurrentTimeSeconds(), newMessage.toRawMessage().getType());
      messageQueue.add(newMessage);
      mostRecentMessage = newMessage;
      notifyMessageObservers(newMessage);
    }

    private void notifyMessageObservers(K t) {
      for (final MessageObserver<K> msgObs : messageObservers) {
        msgObs.onNewMessage(t);
      }
    }

    /**
     * Registers a message observer.
     *
     * @param messageObserver the message observer to be registered
     */
    public void registerMessageObserver(MessageObserver<K> messageObserver) {
      messageObservers.add(messageObserver);
    }

    /**
     * Removed a message observer.
     *
     * @param messageObserver the message observer to be removed
     */
    public void removeMessageObserver(MessageObserver<K> messageObserver) {
      messageObservers.remove(messageObserver);
    }

    Optional<K> getMostRecentMessage() {
      if (mostRecentMessage == null) {
        return Optional.absent();
      } else {
        return Optional.of(mostRecentMessage);
      }
    }

    Queue<K> getMessageQueue() {
      return new LinkedList<>(messageQueue);
    }
  }
}
