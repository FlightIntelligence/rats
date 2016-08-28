package operationaltesting;

import com.google.common.collect.ImmutableList;
import commands.Command;
import commands.bebopcommands.BebopLand;
import commands.bebopcommands.BebopTakeOff;
import commands.schedulers.PeriodicTaskRunner;
import control.dto.InertialFrameVelocity;
import control.dto.Pose;
import control.dto.Velocity;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import sensor_msgs.Joy;
import services.FlyingStateService;
import services.LandService;
import services.ResetService;
import services.TakeOffService;
import services.Velocity4dService;
import services.parrot.BebopServiceFactory;
import services.rossubscribers.MessagesSubscriberService;
import taskexecutor.Task;
import taskexecutor.TaskExecutor;
import taskexecutor.TaskExecutorService;
import taskexecutor.TaskType;
import taskexecutor.interruptors.XBox360ControllerEmergency;
import time.RosTime;

/**
 * Operational test for the xbox emergency button when flying with bebop. The runner commands the
 * drone to take off, then waits for five seconds, then starts sending zero velocity for each 50ms.
 * After five seconds, the user presses the emergency land button. The drone is expected to land
 * after that.
 *
 * @author Hoang Tung Dinh
 */
public final class EmergencyButtonOT extends AbstractNodeMain {

  private static final Logger logger = LoggerFactory.getLogger(EmergencyButtonOT.class);
  private static final String DRONE_NAME = "bebop";

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("EmergencyButtonOT");
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {
    final String controllerStatus =
        connectedNode.getParameterTree().getString("/beswarm/ot/xboxcontroller");

    final TaskExecutor taskExecutor = TaskExecutorService.create();

    final BebopServiceFactory bebopServiceFactory =
        BebopServiceFactory.create(connectedNode, DRONE_NAME);
    final FlyingStateService flyingStateService = bebopServiceFactory.createFlyingStateService();
    final TakeOffService takeOffService = bebopServiceFactory.createTakeOffService();
    final ResetService resetService = bebopServiceFactory.createResetService();
    final LandService landService = bebopServiceFactory.createLandService();
    final Velocity4dService velocity4dService = bebopServiceFactory.createVelocity4dService();

    if ("enable".equals(controllerStatus)) {
      final XBox360ControllerEmergency xBox360ControllerEmergency =
          createXBoxControllerEmergency(connectedNode, flyingStateService, landService);
      xBox360ControllerEmergency.registerTaskExecutor(taskExecutor);
    }

    // start flying
    final Task flyTask =
        createFlyTask(flyingStateService, takeOffService, resetService, velocity4dService);
    taskExecutor.submitTask(flyTask);
  }

  private XBox360ControllerEmergency createXBoxControllerEmergency(
      ConnectedNode connectedNode, FlyingStateService flyingStateService, LandService landService) {
    final Command land = BebopLand.create(landService, flyingStateService);
    final Task emergencyLandingTask =
        Task.create(ImmutableList.of(land), TaskType.FIRST_ORDER_EMERGENCY);

    final MessagesSubscriberService<Joy> joystickSubscriber =
        MessagesSubscriberService.create(
            connectedNode.<Joy>newSubscriber("/bebop/joy", Joy._TYPE),
            RosTime.create(connectedNode));
    final XBox360ControllerEmergency xBox360ControllerEmergency =
        XBox360ControllerEmergency.create(emergencyLandingTask);
    joystickSubscriber.registerMessageObserver(xBox360ControllerEmergency);
    return xBox360ControllerEmergency;
  }

  private Task createFlyTask(
      FlyingStateService flyingStateService,
      TakeOffService takeOffService,
      ResetService resetService,
      final Velocity4dService velocity4dService) {
    final Command takeOff = BebopTakeOff.create(takeOffService, flyingStateService, resetService);
    final Command sendZeroVelocity =
        new Command() {
          @Override
          public void execute() {
            logger.debug("Executing SendZeroVelocity command.");
            final Runnable sendZeroVelocityTask =
                new Runnable() {
                  @Override
                  public void run() {
                    final InertialFrameVelocity inertialFrameVelocity =
                        Velocity.builder()
                            .setLinearX(0)
                            .setLinearY(0)
                            .setLinearZ(0.01)
                            .setAngularZ(0)
                            .build();
                    final Pose pose = Pose.createZeroPose();
                    velocity4dService.sendInertialFrameVelocity(inertialFrameVelocity, pose);
                  }
                };

            PeriodicTaskRunner.run(sendZeroVelocityTask, 0.05, 120);
          }
        };

    return Task.create(ImmutableList.of(takeOff, sendZeroVelocity), TaskType.NORMAL_TASK);
  }
}
