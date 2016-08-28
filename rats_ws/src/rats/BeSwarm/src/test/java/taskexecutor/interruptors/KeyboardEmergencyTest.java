package taskexecutor.interruptors;

import com.google.common.collect.ImmutableList;
import commands.Command;
import keyboard.Key;
import org.junit.Before;
import org.junit.Test;
import taskexecutor.Task;
import taskexecutor.TaskExecutor;
import taskexecutor.TaskType;

import static org.mockito.Matchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

/** @author Hoang Tung Dinh */
public class KeyboardEmergencyTest {

  private final Task task =
      Task.create(ImmutableList.<Command>of(), TaskType.FIRST_ORDER_EMERGENCY);
  private KeyboardEmergency keyboardEmergency;
  private TaskExecutor firstTaskExecutor;
  private TaskExecutor secondTaskExecutor;

  @Before
  public void setUp() {
    keyboardEmergency = KeyboardEmergency.create(task);
    firstTaskExecutor = mock(TaskExecutor.class);
    secondTaskExecutor = mock(TaskExecutor.class);
    keyboardEmergency.registerTaskExecutor(firstTaskExecutor);
    keyboardEmergency.registerTaskExecutor(secondTaskExecutor);
  }

  @Test
  public void testNotEmergencyKeyPressed() {
    final Key key = mock(Key.class);
    when(key.getCode()).thenReturn(Key.KEY_0);
    keyboardEmergency.onNewMessage(key);
    verify(firstTaskExecutor, never()).submitTask(any(Task.class));
    verify(secondTaskExecutor, never()).submitTask(any(Task.class));
  }

  @Test
  public void testEmergencyKeyPressed() {
    final Key key = mock(Key.class);
    when(key.getCode()).thenReturn(KeyboardEmergency.EMERGENCY_KEY);
    keyboardEmergency.onNewMessage(key);
    verify(firstTaskExecutor).submitTask(task);
    verify(secondTaskExecutor).submitTask(task);
  }

  @Test
  public void testRemoveTaskExecutor() {
    keyboardEmergency.removeTaskExecutor(firstTaskExecutor);
    final Key key = mock(Key.class);
    when(key.getCode()).thenReturn(KeyboardEmergency.EMERGENCY_KEY);
    keyboardEmergency.onNewMessage(key);
    verify(firstTaskExecutor, never()).submitTask(any(Task.class));
    verify(secondTaskExecutor).submitTask(task);
  }
}
