package taskexecutor.interruptors;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import taskexecutor.EmergencyNotifier;
import taskexecutor.Task;
import taskexecutor.TaskExecutor;

import java.util.ArrayList;
import java.util.Collection;

import static java.util.concurrent.TimeUnit.SECONDS;

/** @author Hoang Tung Dinh */
public final class SimpleEmergencyAfterOneMinute implements EmergencyNotifier {

  private static final Logger logger = LoggerFactory.getLogger(SimpleEmergencyAfterOneMinute.class);

  private final Task task;
  private final Collection<TaskExecutor> taskExecutors = new ArrayList<>();

  private SimpleEmergencyAfterOneMinute(Task task) {
    this.task = task;
  }

  /**
   * Creates an emergency notifier that will notify an emergency situation after one minute.
   *
   * @param task the emergency task to be executed in the emergency situation.
   * @return an emergency notifier instance
   */
  public static SimpleEmergencyAfterOneMinute create(Task task) {
    return new SimpleEmergencyAfterOneMinute(task);
  }

  /**
   * Starts waiting for one minutes and then submits the emergency task to all subscribed {@link
   * TaskExecutor}.
   */
  public void run() {
    try {
      SECONDS.sleep(60);
      for (final TaskExecutor taskExecutor : taskExecutors) {
        taskExecutor.submitTask(task);
      }
    } catch (InterruptedException e) {
      if (logger.isDebugEnabled()) {
        logger.debug(
            "Waiting until sending emergency notification is interrupted. No "
                + "emergency notification will be sent.",
            e);
      }
      Thread.currentThread().interrupt();
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
