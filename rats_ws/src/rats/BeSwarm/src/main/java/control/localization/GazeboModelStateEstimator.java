package control.localization;

import com.google.common.base.Optional;
import control.dto.DroneStateStamped;
import control.dto.InertialFrameVelocity;
import control.dto.Pose;
import control.dto.Velocity;
import gazebo_msgs.ModelStates;
import geometry_msgs.Point;
import geometry_msgs.Quaternion;
import geometry_msgs.Twist;
import services.rossubscribers.MessagesSubscriberService;
import time.TimeProvider;
import utils.math.Transformations;

/** @author Hoang Tung Dinh */
public final class GazeboModelStateEstimator implements StateEstimator {
  private final MessagesSubscriberService<ModelStates> modelStateSubscriber;
  private final String modelName;
  private final TimeProvider timeProvider;

  private GazeboModelStateEstimator(
      MessagesSubscriberService<ModelStates> modelStateSubscriber,
      String modelName,
      TimeProvider timeProvider) {
    this.modelStateSubscriber = modelStateSubscriber;
    this.modelName = modelName;
    this.timeProvider = timeProvider;
  }

  /**
   * Creates a state estimator that use the ModelStates topics in Gazebo to get the current state of
   * the drone.
   *
   * @param modelStateSubscriber the rostopic subscriber to a topic publishing model state messages
   * @param modelName the name of the drone model. A model state topic in Gazebo contains the states
   *     of all models in the simulation environment, while the drone is one of those models.
   * @param timeProvider the time provider
   * @return a state estimator instance
   */
  public static GazeboModelStateEstimator create(
      MessagesSubscriberService<ModelStates> modelStateSubscriber,
      String modelName,
      TimeProvider timeProvider) {
    return new GazeboModelStateEstimator(modelStateSubscriber, modelName, timeProvider);
  }

  private static InertialFrameVelocity getInertialFrameVelocity(
      ModelStates modelStates, int index) {
    final Twist gazeboTwist = modelStates.getTwist().get(index);
    return Velocity.builder()
        .setLinearX(gazeboTwist.getLinear().getX())
        .setLinearY(gazeboTwist.getLinear().getY())
        .setLinearZ(gazeboTwist.getLinear().getZ())
        .setAngularZ(gazeboTwist.getAngular().getZ())
        .build();
  }

  private static Pose getDronePose(ModelStates modelStates, int index) {
    final geometry_msgs.Pose gazeboPose = modelStates.getPose().get(index);
    return convertGazeboPoseToDronePose(gazeboPose);
  }

  private static Pose convertGazeboPoseToDronePose(geometry_msgs.Pose gazeboPose) {
    final Point currentPoint = gazeboPose.getPosition();
    final Quaternion currentOrientation = gazeboPose.getOrientation();
    final double currentYaw = Transformations.quaternionToEulerAngle(currentOrientation).angleZ();
    return Pose.builder()
        .setX(currentPoint.getX())
        .setY(currentPoint.getY())
        .setZ(currentPoint.getZ())
        .setYaw(currentYaw)
        .build();
  }

  @Override
  public Optional<DroneStateStamped> getCurrentState() {
    final Optional<ModelStates> modelStateOptional = modelStateSubscriber.getMostRecentMessage();
    if (!modelStateOptional.isPresent()) {
      return Optional.absent();
    }

    final ModelStates modelStates = modelStateOptional.get();
    final int index = modelStates.getName().indexOf(modelName);
    if (index == -1) {
      return Optional.absent();
    }

    final Pose pose = getDronePose(modelStates, index);
    final InertialFrameVelocity inertialFrameVelocity =
        getInertialFrameVelocity(modelStates, index);

    final double timeStampInSeconds = timeProvider.getCurrentTimeSeconds();
    return Optional.of(DroneStateStamped.create(pose, inertialFrameVelocity, timeStampInSeconds));
  }
}
