package taskexecutor;

/**
 * Notifies of the emergency situation to {@link TaskExecutor}s.
 *
 * @author Hoang Tung Dinh
 */
public interface EmergencyNotifier {
  /**
   * Registers the {@code taskExecutor} for listening to the emergency notification.
   *
   * @param taskExecutor the task executor to be registered
   */
  void registerTaskExecutor(TaskExecutor taskExecutor);

  /**
   * Removes a {@code taskExecutor} from listening to the emergency notification.
   *
   * @param taskExecutor the task executor to be removed
   */
  void removeTaskExecutor(TaskExecutor taskExecutor);
}
