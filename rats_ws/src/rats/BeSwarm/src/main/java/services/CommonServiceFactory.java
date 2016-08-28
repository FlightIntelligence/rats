package services;

/**
 * Contains common services supported by all drones.
 *
 * @author Hoang Tung Dinh
 */
public interface CommonServiceFactory {
  /**
   * Creates the take off service.
   *
   * @return a take off service instance
   */
  TakeOffService createTakeOffService();

  /**
   * Creates the land service.
   *
   * @return a land service instance
   */
  LandService createLandService();

  /**
   * Creates the flying state service.
   *
   * @return a flying state service instance
   */
  FlyingStateService createFlyingStateService();
}
