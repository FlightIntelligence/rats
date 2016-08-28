package applications;

import keyboard.Key;
import org.ros.node.ConnectedNode;
import sensor_msgs.Joy;
import services.rossubscribers.MessagesSubscriberService;
import taskexecutor.Task;
import taskexecutor.TaskExecutor;
import taskexecutor.TaskExecutorService;
import taskexecutor.interruptors.KeyboardEmergency;
import taskexecutor.interruptors.XBox360ControllerEmergency;
import time.RosTime;

/**
 * This class illustrates an example flight. The drone will execute a fly {@link Task} while
 * listening to xbox emergency button and keyboard emergency button.
 *
 * @author Hoang Tung Dinh
 */
public final class ExampleFlight {

  private final ConnectedNode connectedNode;
  private final Task flyTask;
  private final Task emergencyTask;

  private ExampleFlight(ConnectedNode connectedNode, Task flyTask, Task emergencyTask) {
    this.connectedNode = connectedNode;
    this.flyTask = flyTask;
    this.emergencyTask = emergencyTask;
  }

  /**
   * Creates an instance of {@link ExampleFlight}.
   *
   * @param connectedNode the connected node
   * @param flyTask the fly task
   * @param emergencyTask the emergency task
   * @return an instance of {@link ExampleFlight}
   */
  public static ExampleFlight create(
      ConnectedNode connectedNode, Task flyTask, Task emergencyTask) {
    return new ExampleFlight(connectedNode, flyTask, emergencyTask);
  }

  /** Starts flying. */
  public void fly() {
    // task to execute in case of emergency
    final KeyboardEmergency keyboardEmergencyNotifier =
        createKeyboardEmergencyNotifier(emergencyTask);
    final XBox360ControllerEmergency xBox360ControllerEmergency =
        createXBox360ControllerEmergency(emergencyTask);

    final TaskExecutor taskExecutor = TaskExecutorService.create();
    keyboardEmergencyNotifier.registerTaskExecutor(taskExecutor);
    xBox360ControllerEmergency.registerTaskExecutor(taskExecutor);

    // start fly task
    taskExecutor.submitTask(flyTask);
  }

  private KeyboardEmergency createKeyboardEmergencyNotifier(Task emergencyTask) {
    final MessagesSubscriberService<Key> keyboardSubscriber =
        MessagesSubscriberService.create(
            connectedNode.<Key>newSubscriber("/keyboard/keydown", Key._TYPE),
            RosTime.create(connectedNode));
    final KeyboardEmergency keyboardEmergency = KeyboardEmergency.create(emergencyTask);
    keyboardSubscriber.registerMessageObserver(keyboardEmergency);
    return keyboardEmergency;
  }

  private XBox360ControllerEmergency createXBox360ControllerEmergency(Task emergencyTask) {
    final MessagesSubscriberService<Joy> joystickSubscriber =
        MessagesSubscriberService.create(
            connectedNode.<Joy>newSubscriber("/bebop/joy", Joy._TYPE),
            RosTime.create(connectedNode));
    final XBox360ControllerEmergency xBox360ControllerEmergency =
        XBox360ControllerEmergency.create(emergencyTask);
    joystickSubscriber.registerMessageObserver(xBox360ControllerEmergency);
    return xBox360ControllerEmergency;
  }
}
