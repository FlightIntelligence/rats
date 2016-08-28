package services;

/**
 * Take off service.
 *
 * @author Hoang Tung Dinh
 */
public interface TakeOffService {
  /** Send a message to require the drone to take off. */
  void sendTakingOffMessage();
}
