package taskexecutor;

/** @author Hoang Tung Dinh */
public interface TaskExecutor {
  /**
   * Submits a new task to the task executor.
   *
   * @param newTask the new task to be submitted
   * @return a notification whether the task is accepted or rejected
   */
  Status submitTask(Task newTask);

  /**
   * Represents the status of a task after it was submitted. The task can be accepted and executed
   * or it can be rejected.
   */
  enum Status {
    ACCEPTED,
    REJECTED
  }
}
