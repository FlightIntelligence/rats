package taskexecutor;

/** @author Hoang Tung Dinh */
public enum TaskType {
  FIRST_ORDER_EMERGENCY(0),
  SECOND_ORDER_EMERGENCY(1),
  NORMAL_TASK(2);

  private final int taskPriority;

  TaskType(int taskPriority) {
    this.taskPriority = taskPriority;
  }

  /**
   * Checks whether this task type has higher priority than another task type.
   *
   * @param taskType the other task type
   * @return true if this task type has higher priority than the other task type
   */
  public boolean hasHigherPriorityThan(TaskType taskType) {
    return taskPriority < taskType.taskPriority;
  }
}
