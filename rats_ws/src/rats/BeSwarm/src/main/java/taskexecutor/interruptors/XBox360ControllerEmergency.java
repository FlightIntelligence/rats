package taskexecutor.interruptors;

import sensor_msgs.Joy;
import services.rossubscribers.MessageObserver;
import taskexecutor.EmergencyNotifier;
import taskexecutor.Task;
import taskexecutor.TaskExecutor;

import java.util.ArrayList;
import java.util.Collection;

/**
 * A xBox 360 controller listener who notifies the drone to do a task when RB + A are pressed.
 *
 * @author Hoang Tung Dinh
 */
public final class XBox360ControllerEmergency implements EmergencyNotifier, MessageObserver<Joy> {
  private final Task task;
  private final Collection<TaskExecutor> taskExecutors = new ArrayList<>();
  private static final int RB_CODE = 5;
  private static final int A_CODE = 0;

  private XBox360ControllerEmergency(Task task) {
    this.task = task;
  }

  public static XBox360ControllerEmergency create(Task task) {
    return new XBox360ControllerEmergency(task);
  }

  @Override
  public void onNewMessage(Joy message) {
    final int[] buttons = message.getButtons();
    // check if buttons are pressed
    if (buttons[RB_CODE] == 1 && buttons[A_CODE] == 1) {
      for (final TaskExecutor taskExecutor : taskExecutors) {
        taskExecutor.submitTask(task);
      }
    }
  }

  @Override
  public void registerTaskExecutor(TaskExecutor taskExecutor) {
    taskExecutors.add(taskExecutor);
  }

  @Override
  public void removeTaskExecutor(TaskExecutor taskExecutor) {
    taskExecutors.remove(taskExecutor);
  }
}
