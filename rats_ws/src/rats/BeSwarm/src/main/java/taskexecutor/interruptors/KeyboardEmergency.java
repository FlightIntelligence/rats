package taskexecutor.interruptors;

import com.google.common.annotations.VisibleForTesting;
import keyboard.Key;
import services.rossubscribers.MessageObserver;
import taskexecutor.EmergencyNotifier;
import taskexecutor.Task;
import taskexecutor.TaskExecutor;

import java.util.ArrayList;
import java.util.Collection;

/** @author Hoang Tung Dinh */
public final class KeyboardEmergency implements EmergencyNotifier, MessageObserver<Key> {
  /** The code of the emergency key. */
  @VisibleForTesting static final short EMERGENCY_KEY = Key.KEY_x;

  private final Task task;
  private final Collection<TaskExecutor> taskExecutors = new ArrayList<>();

  private KeyboardEmergency(Task task) {
    this.task = task;
  }

  /**
   * Creates a keyboard emergency notifier. It will notify all the subscribed {@link TaskExecutor}
   * when key "x" is pressed.
   *
   * @param task the emergency task to be executed when key "x" is pressed
   * @return a keyboard emergency notifier
   */
  public static KeyboardEmergency create(Task task) {
    return new KeyboardEmergency(task);
  }

  @Override
  public void registerTaskExecutor(TaskExecutor taskExecutor) {
    taskExecutors.add(taskExecutor);
  }

  @Override
  public void removeTaskExecutor(TaskExecutor taskExecutor) {
    taskExecutors.remove(taskExecutor);
  }

  @Override
  public void onNewMessage(Key message) {
    if (message.getCode() == EMERGENCY_KEY) {
      for (final TaskExecutor taskExecutor : taskExecutors) {
        taskExecutor.submitTask(task);
      }
    }
  }
}
