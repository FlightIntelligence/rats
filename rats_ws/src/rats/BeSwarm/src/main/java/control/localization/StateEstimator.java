package control.localization;

import com.google.common.base.Optional;
import control.dto.DroneStateStamped;

/** @author Hoang Tung Dinh */
public interface StateEstimator {
  /**
   * Gets the current estimated state.
   *
   * @return the current estimated state of the drone, could be absent
   */
  Optional<DroneStateStamped> getCurrentState();
}
