package services.crates;

import com.google.common.base.Optional;
import com.google.common.collect.ImmutableMap;
import hal_quadrotor.State;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import services.AbstractFlyingStateService;
import services.rossubscribers.FlyingState;
import services.rossubscribers.MessagesSubscriberService;

import java.util.Map;

/** @author Hoang Tung Dinh */
final class CratesFlyingStateService extends AbstractFlyingStateService<State> {

  private static final Logger logger = LoggerFactory.getLogger(CratesFlyingStateService.class);
  private static final Map<String, CratesFlyingState> FLYING_STATE_MAP =
      ImmutableMap.<String, CratesFlyingState>builder()
          .put("AnglesHeight", CratesFlyingState.ANGLES_HEIGHT)
          .put("Emergency", CratesFlyingState.EMERGENCY)
          .put("AbstractHover", CratesFlyingState.HOVER)
          .put("Idle", CratesFlyingState.IDLE)
          .put("AbstractParrotLand", CratesFlyingState.LAND)
          .put("AbstractParrotTakeOff", CratesFlyingState.TAKE_OFF)
          .put("VelocityHeight", CratesFlyingState.VELOCITY_HEIGHT)
          .put("Velocity", CratesFlyingState.VELOCITY)
          .put("Waypoint", CratesFlyingState.WAYPOINT)
          .build();

  private CratesFlyingStateService() {}

  /**
   * Creates the flying state service for a drone in the Crates simulator.
   *
   * @param flyingStateSubscriber the subscriber to a state topic of the drone
   * @return an flying state service instance
   */
  public static CratesFlyingStateService create(
      MessagesSubscriberService<State> flyingStateSubscriber) {
    final CratesFlyingStateService cratesFlyingStateService = new CratesFlyingStateService();
    flyingStateSubscriber.registerMessageObserver(cratesFlyingStateService);
    return cratesFlyingStateService;
  }

  @Override
  public void onNewMessage(State message) {
    final String controllerStateName = message.getController();
    final Optional<FlyingState> currentFlyingState = getCurrentFlyingState();
    if (!currentFlyingState.isPresent()
        || !currentFlyingState.get().getStateName().equals(controllerStateName)) {
      final FlyingState newFlyingState =
          FLYING_STATE_MAP.get(controllerStateName).getConvertedFlyingState();
      setCurrentFlyingState(newFlyingState);
      logger.trace(
          "Current crates state: {}. Current standard flying state: {}.",
          controllerStateName,
          newFlyingState.getStateName());
    }
  }

  private enum CratesFlyingState {
    ANGLES_HEIGHT("AnglesHeight") {
      @Override
      FlyingState getConvertedFlyingState() {
        return FlyingState.FLYING;
      }
    },

    EMERGENCY("Emergency") {
      @Override
      FlyingState getConvertedFlyingState() {
        return FlyingState.EMERGENCY;
      }
    },

    HOVER("AbstractHover") {
      @Override
      FlyingState getConvertedFlyingState() {
        return FlyingState.HOVERING;
      }
    },

    IDLE("Idle") {
      @Override
      FlyingState getConvertedFlyingState() {
        return FlyingState.LANDED;
      }
    },

    LAND("AbstractParrotLand") {
      @Override
      FlyingState getConvertedFlyingState() {
        return FlyingState.LANDING;
      }
    },

    TAKE_OFF("AbstractParrotTakeOff") {
      @Override
      FlyingState getConvertedFlyingState() {
        return FlyingState.TAKING_OFF;
      }
    },

    VELOCITY_HEIGHT("VelocityHeight") {
      @Override
      FlyingState getConvertedFlyingState() {
        return FlyingState.FLYING;
      }
    },

    VELOCITY("Velocity") {
      @Override
      FlyingState getConvertedFlyingState() {
        return FlyingState.FLYING;
      }
    },

    WAYPOINT("Waypoint") {
      @Override
      FlyingState getConvertedFlyingState() {
        return FlyingState.FLYING;
      }
    };

    private final String stateName;

    CratesFlyingState(String stateName) {
      this.stateName = stateName;
    }

    public String getStateName() {
      return stateName;
    }

    abstract FlyingState getConvertedFlyingState();
  }
}
