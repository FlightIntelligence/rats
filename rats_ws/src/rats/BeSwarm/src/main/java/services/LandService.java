package services;

/**
 * Represents the land service.
 *
 * @author Hoang Tung Dinh
 */
public interface LandService {
  /** Sends a message to require the drone to land. */
  void sendLandingMessage();
}
