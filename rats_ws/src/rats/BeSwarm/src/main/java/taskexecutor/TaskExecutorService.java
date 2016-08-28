package taskexecutor;

import commands.Command;

import javax.annotation.Nullable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import static com.google.common.base.Preconditions.checkNotNull;

/** @author Hoang Tung Dinh */
public final class TaskExecutorService implements TaskExecutor {
  @Nullable private Task task;
  @Nullable private Future<?> future;

  private TaskExecutorService() {}

  /**
   * Gets an instace of this class.
   *
   * @return a builder instance
   */
  public static TaskExecutorService create() {
    return new TaskExecutorService();
  }

  private void runTask() {
    checkNotNull(task, "Task must not be null when this method is called");
    final ExecutorService executorService = Executors.newSingleThreadExecutor();
    final Runnable runnable = new RunCommands(task.getCommands());
    future = executorService.submit(runnable);
  }

  @Override
  public Status submitTask(Task newTask) {
    if (task == null || newTask.hasHigherPriority(task)) {
      if (future != null) {
        future.cancel(true);
      }
      task = newTask;
      runTask();
      return Status.ACCEPTED;
    }

    return Status.REJECTED;
  }

  private static final class RunCommands implements Runnable {
    private final Iterable<Command> commands;

    private RunCommands(Iterable<Command> commands) {
      this.commands = commands;
    }

    @Override
    public void run() {
      for (final Command command : commands) {
        command.execute();
      }
    }
  }
}
