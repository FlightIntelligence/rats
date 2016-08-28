package commands.schedulers;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

/** @author Hoang Tung Dinh */
public final class PeriodicTaskRunner {

  private static final Logger logger = LoggerFactory.getLogger(PeriodicTaskRunner.class);

  private PeriodicTaskRunner() {}

  /**
   * Runs a java task. Note that this task is a {@link Runnable} object, and is different from the
   * {@link taskexecutor.Task} which defines a list of drone's commands.
   *
   * @param task the {@link Runnable} task
   * @param rateInSeconds the rate of executing this task
   * @param durationInSeconds the duration in which the task will be executed
   */
  public static void run(Runnable task, double rateInSeconds, double durationInSeconds) {
    final long initialDelay = 0;
    final long rateInMilliSeconds = (long) (rateInSeconds * 1000);

    final Future<?> future =
        Executors.newSingleThreadScheduledExecutor()
            .scheduleAtFixedRate(task, initialDelay, rateInMilliSeconds, TimeUnit.MILLISECONDS);

    final long durationInMilliSeconds = (long) (durationInSeconds * 1000);

    try {
      future.get(durationInMilliSeconds, TimeUnit.MILLISECONDS);
    } catch (InterruptedException ignored) {
      logger.debug("The executing task is interrupted. Stop executing the task.");
      // the task is cancelled if time ran out or the current thread is interrupted while
      // waiting.
      future.cancel(true);
      Thread.currentThread().interrupt();
    } catch (TimeoutException ignored) {
      logger.debug("The executing task is run out of time. Stop executing the task");
      future.cancel(true);
    } catch (ExecutionException e) {
      logger.debug("An execution exception occurs.", e);
    }
  }
}
