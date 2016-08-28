package services;

import com.google.common.base.Optional;
import org.ros.internal.message.Message;
import services.rossubscribers.FlyingState;
import services.rossubscribers.MessageObserver;

import java.util.concurrent.atomic.AtomicReference;

/**
 * @param <T> the type of the state message
 * @author Hoang Tung Dinh
 */
public abstract class AbstractFlyingStateService<T extends Message>
    implements FlyingStateService, MessageObserver<T> {

  private final AtomicReference<FlyingState> currentFlyingState = new AtomicReference<>();

  protected AbstractFlyingStateService() {}

  @Override
  public final Optional<FlyingState> getCurrentFlyingState() {
    final FlyingState flyingState = currentFlyingState.get();
    if (flyingState == null) {
      return Optional.absent();
    } else {
      return Optional.of(flyingState);
    }
  }

  protected void setCurrentFlyingState(FlyingState currentFlyingState) {
    this.currentFlyingState.set(currentFlyingState);
  }
}
