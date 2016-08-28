package services;

import control.dto.FlipDirection;

/**
 * Represents the flip service.
 *
 * @author Hoang Tung Dinh
 */
public interface FlipService {
  /**
   * Sends a message to require the drone to flip in a specific direction.
   *
   * @param flipDirection the desired flip direction
   */
  void sendFlipMessage(FlipDirection flipDirection);
}
