package taskexecutor;

import com.google.common.collect.ImmutableList;
import commands.Command;

/** @author Hoang Tung Dinh */
public final class Task {
  private final ImmutableList<Command> commands;
  private final TaskType taskType;

  private Task(ImmutableList<Command> commands, TaskType taskType) {
    this.commands = commands;
    this.taskType = taskType;
  }

  /**
   * Creates a task contains all the commands in order.
   *
   * @param commands the commands of the created task
   * @param taskType the type of the created task
   * @return a task containing all the commands in order
   */
  public static Task create(ImmutableList<Command> commands, TaskType taskType) {
    return new Task(commands, taskType);
  }

  /**
   * Creates a task contains all the commands in order.
   *
   * @param taskType the type of the created task
   * @param commands the commands of the created task
   * @return a task containing all the commands in order
   */
  public static Task create(TaskType taskType, Command... commands) {
    ImmutableList<Command> commandList = ImmutableList.<Command>copyOf(commands);
    return new Task(commandList, taskType);
  }

  public ImmutableList<Command> getCommands() {
    return commands;
  }

  /**
   * Checks whether this task has higher priority than another task.
   *
   * @param otherTask the other task
   * @return true if this task has higher priority than the other task
   */
  public boolean hasHigherPriority(Task otherTask) {
    return taskType.hasHigherPriorityThan(otherTask.taskType);
  }
}
