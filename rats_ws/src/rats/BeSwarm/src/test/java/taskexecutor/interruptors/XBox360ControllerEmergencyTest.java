package taskexecutor.interruptors;

import com.google.common.collect.ImmutableList;
import commands.Command;
import junitparams.JUnitParamsRunner;
import junitparams.Parameters;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import sensor_msgs.Joy;
import taskexecutor.Task;
import taskexecutor.TaskExecutor;
import taskexecutor.TaskType;

import static org.mockito.Mockito.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

/**
 * Tests for {@link XBox360ControllerEmergency}'s methods.
 *
 * @author Hoang Tung Dinh
 */
@RunWith(JUnitParamsRunner.class)
public class XBox360ControllerEmergencyTest {

  private final Task task =
      Task.create(ImmutableList.<Command>of(), TaskType.FIRST_ORDER_EMERGENCY);
  private XBox360ControllerEmergency xBox360ControllerEmergency;
  private TaskExecutor firstTaskExecutor;
  private TaskExecutor secondTaskExecutor;

  @Before
  public void setUp() {
    xBox360ControllerEmergency = XBox360ControllerEmergency.create(task);
    firstTaskExecutor = mock(TaskExecutor.class);
    secondTaskExecutor = mock(TaskExecutor.class);
    xBox360ControllerEmergency.registerTaskExecutor(firstTaskExecutor);
    xBox360ControllerEmergency.registerTaskExecutor(secondTaskExecutor);
  }

  @Test
  @Parameters(method = "buttonValues")
  public void testButtonPressed(int[] buttons, ButtonState buttonState) {
    final Joy joyMessage = mock(Joy.class);
    when(joyMessage.getButtons()).thenReturn(buttons);
    xBox360ControllerEmergency.onNewMessage(joyMessage);
    if (buttonState == ButtonState.NOT_EMERGENCY_COMBINATION) {
      verify(firstTaskExecutor, never()).submitTask(any(Task.class));
      verify(secondTaskExecutor, never()).submitTask(any(Task.class));
    } else {
      verify(firstTaskExecutor).submitTask(any(Task.class));
      verify(secondTaskExecutor).submitTask(any(Task.class));
    }
  }

  private Object[] buttonValues() {
    return new Object[] {
      new Object[] {
        new int[] {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, ButtonState.NOT_EMERGENCY_COMBINATION
      },
      new Object[] {
        new int[] {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, ButtonState.NOT_EMERGENCY_COMBINATION
      },
      new Object[] {
        new int[] {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0}, ButtonState.NOT_EMERGENCY_COMBINATION
      },
      new Object[] {
        new int[] {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0}, ButtonState.IS_EMERGENCY_COMBINATION
      },
      new Object[] {
        new int[] {1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0}, ButtonState.IS_EMERGENCY_COMBINATION
      },
      new Object[] {
        new int[] {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0}, ButtonState.IS_EMERGENCY_COMBINATION
      },
      new Object[] {
        new int[] {1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0}, ButtonState.IS_EMERGENCY_COMBINATION
      }
    };
  }

  private enum ButtonState {
    IS_EMERGENCY_COMBINATION,
    NOT_EMERGENCY_COMBINATION
  }
}
