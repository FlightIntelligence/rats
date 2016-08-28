package services.parrot;

import ardrone_autonomy.Navdata;
import com.google.common.collect.ImmutableMap;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import services.AbstractFlyingStateService;
import services.rossubscribers.FlyingState;
import services.rossubscribers.MessagesSubscriberService;

import java.util.Map;

/** @author Hoang Tung Dinh */
final class TumSimFlyingStateService extends AbstractFlyingStateService<Navdata> {

  private static final Logger logger = LoggerFactory.getLogger(TumSimFlyingStateService.class);
  private static final Map<Integer, ArDroneFlyingState> FLYING_STATE_MAP =
      ImmutableMap.<Integer, ArDroneFlyingState>builder()
          .put(0, ArDroneFlyingState.UNKNOWN)
          .put(1, ArDroneFlyingState.INITED)
          .put(2, ArDroneFlyingState.LANDED)
          .put(3, ArDroneFlyingState.FLYING)
          .put(4, ArDroneFlyingState.HOVERING)
          .put(5, ArDroneFlyingState.TEST)
          .put(6, ArDroneFlyingState.TAKING_OFF)
          .put(7, ArDroneFlyingState.FLYING)
          .put(8, ArDroneFlyingState.LANDING)
          .put(9, ArDroneFlyingState.LOOPING)
          .build();

  private TumSimFlyingStateService() {}

  /**
   * Creates a flying state service for the Tum simulator.
   *
   * @param flyingStateSubscriber the subscriber to the rostopic that provides the drone's state
   * @return a flying state service instance
   */
  public static TumSimFlyingStateService create(
      MessagesSubscriberService<Navdata> flyingStateSubscriber) {
    final TumSimFlyingStateService tumSimFlyingStateService = new TumSimFlyingStateService();
    flyingStateSubscriber.registerMessageObserver(tumSimFlyingStateService);
    return tumSimFlyingStateService;
  }

  @Override
  public void onNewMessage(Navdata message) {
    setCurrentFlyingState(FLYING_STATE_MAP.get(message.getState()).getConvertedFlyingState());
    logger.trace("Current flying state: {}", getCurrentFlyingState().get().getStateName());
  }

  private enum ArDroneFlyingState {
    UNKNOWN("Unknown") {
      @Override
      FlyingState getConvertedFlyingState() {
        return FlyingState.UNKNOWN;
      }
    },

    INITED("Inited") {
      @Override
      FlyingState getConvertedFlyingState() {
        return FlyingState.LANDED;
      }
    },

    LANDED("Landed") {
      @Override
      FlyingState getConvertedFlyingState() {
        return FlyingState.LANDED;
      }
    },

    FLYING("Flying") {
      @Override
      FlyingState getConvertedFlyingState() {
        return FlyingState.FLYING;
      }
    },

    HOVERING("Hovering") {
      @Override
      FlyingState getConvertedFlyingState() {
        return FlyingState.HOVERING;
      }
    },

    TEST("Test") {
      @Override
      FlyingState getConvertedFlyingState() {
        return FlyingState.UNKNOWN;
      }
    },

    TAKING_OFF("Taking off") {
      @Override
      FlyingState getConvertedFlyingState() {
        return FlyingState.TAKING_OFF;
      }
    },

    LANDING("Landing") {
      @Override
      FlyingState getConvertedFlyingState() {
        return FlyingState.LANDING;
      }
    },

    LOOPING("Looping") {
      @Override
      FlyingState getConvertedFlyingState() {
        return FlyingState.UNKNOWN;
      }
    };

    private final String stateName;

    ArDroneFlyingState(String stateName) {
      this.stateName = stateName;
    }

    public String getStateName() {
      return stateName;
    }

    abstract FlyingState getConvertedFlyingState();
  }
}
