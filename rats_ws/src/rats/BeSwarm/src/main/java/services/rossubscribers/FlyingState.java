package services.rossubscribers;

/** @author Hoang Tung Dinh */
public enum FlyingState {
  UNKNOWN("Unknown"),
  LANDED("Landed"),
  TAKING_OFF("Taking off"),
  HOVERING("Hovering"),
  FLYING("Flying"),
  LANDING("Landing"),
  EMERGENCY("Emergency"),
  USER_TAKEOFF("User take off");

  private final String stateName;

  FlyingState(String stateName) {
    this.stateName = stateName;
  }

  public String getStateName() {
    return stateName;
  }
}
