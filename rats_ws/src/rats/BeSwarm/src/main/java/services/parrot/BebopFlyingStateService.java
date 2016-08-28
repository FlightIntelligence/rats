package services.parrot;

import bebop_msgs.Ardrone3PilotingStateFlyingStateChanged;
import com.google.common.collect.ImmutableMap;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import services.AbstractFlyingStateService;
import services.rossubscribers.FlyingState;
import services.rossubscribers.MessagesSubscriberService;

/** @author Hoang Tung Dinh */
public final class BebopFlyingStateService
    extends AbstractFlyingStateService<Ardrone3PilotingStateFlyingStateChanged> {

  private static final Logger logger = LoggerFactory.getLogger(BebopFlyingStateService.class);
  private static final ImmutableMap<Byte, FlyingState> FLYING_STATE_MAP =
      ImmutableMap.<Byte, FlyingState>builder()
          .put((byte) 0, FlyingState.LANDED)
          .put((byte) 1, FlyingState.TAKING_OFF)
          .put((byte) 2, FlyingState.HOVERING)
          .put((byte) 3, FlyingState.FLYING)
          .put((byte) 4, FlyingState.LANDING)
          .put((byte) 5, FlyingState.EMERGENCY)
          .put((byte) 6, FlyingState.USER_TAKEOFF)
          .build();

  private BebopFlyingStateService() {}

  /**
   * Creates the flying state service for the bebop drone.
   *
   * @param flyingStateSubscriber the message subscriber of the flying state changed of bebop
   * @return the flying state service
   */
  public static BebopFlyingStateService create(
      MessagesSubscriberService<Ardrone3PilotingStateFlyingStateChanged> flyingStateSubscriber) {
    final BebopFlyingStateService bebopFlyingStateService = new BebopFlyingStateService();
    flyingStateSubscriber.registerMessageObserver(bebopFlyingStateService);
    return bebopFlyingStateService;
  }

  @Override
  public void onNewMessage(Ardrone3PilotingStateFlyingStateChanged message) {
    setCurrentFlyingState(FLYING_STATE_MAP.get(message.getState()));
    logger.info("Current flying state: {}", getCurrentFlyingState().get().getStateName());
  }
}
