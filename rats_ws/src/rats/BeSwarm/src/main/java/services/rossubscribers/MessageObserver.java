package services.rossubscribers;

import org.ros.internal.message.Message;

/**
 * @param <T> the type of the messages
 * @author Hoang Tung Dinh
 */
public interface MessageObserver<T extends Message> {
  /**
   * Notifies when there is a new message.
   *
   * @param message the new message
   */
  void onNewMessage(T message);
}
