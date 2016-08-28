package operationaltesting;

import com.google.common.base.Optional;
import control.dto.DroneStateStamped;
import control.dto.InertialFrameVelocity;
import control.localization.BebopStateEstimatorWithPoseStamped;
import control.localization.BebopStateEstimatorWithPoseStampedAndOdom;
import control.localization.StateEstimator;
import geometry_msgs.PoseStamped;
import geometry_msgs.Twist;
import nav_msgs.Odometry;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import services.rossubscribers.MessagesSubscriberService;
import time.RosTime;

import java.util.concurrent.TimeUnit;

/**
 * Operational test for {@link StateEstimator}'s implementations. This test is to compare the
 * velocity got from odometry and the velocity got from ArMarker.
 *
 * @author Hoang Tung Dinh
 */
public final class StateEstimatorOT extends AbstractNodeMain {

  private static final Logger logger = LoggerFactory.getLogger(StateEstimatorOT.class);
  private static final Logger arMarkerVelLogger =
      LoggerFactory.getLogger(StateEstimatorOT.class.getName() + ".velocity.armarker");
  private static final Logger odomVelLogger =
      LoggerFactory.getLogger(StateEstimatorOT.class.getName() + ".velocity.odom");
  private static final String DRONE_NAME = "bebop";

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("StateEstimatorOT");
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {
    final MessagesSubscriberService<PoseStamped> poseSubscriber = getPoseSubscriber(connectedNode);
    final MessagesSubscriberService<Odometry> odomSubscriber = getOdometrySubscriber(connectedNode);

    final StateEstimator stateEstimatorWithArMarker =
        BebopStateEstimatorWithPoseStamped.create(poseSubscriber, 3);
    final StateEstimator stateEstimatorWithArMarkerAndOdom =
        BebopStateEstimatorWithPoseStampedAndOdom.create(poseSubscriber, odomSubscriber);

    final Publisher<Twist> velocityFromArMarker =
        connectedNode.newPublisher("/beswarm/armarker_velocity/cmd_vel", Twist._TYPE);
    final Publisher<Twist> velocityFromOdom =
        connectedNode.newPublisher("/beswarm/odom_velocity/cmd_vel", Twist._TYPE);

    final Runnable dataRecorder =
        new Runnable() {
          @Override
          public void run() {
            final Optional<DroneStateStamped> stateArMarker =
                stateEstimatorWithArMarker.getCurrentState();
            final Optional<DroneStateStamped> stateArMarkerAndOdom =
                stateEstimatorWithArMarkerAndOdom.getCurrentState();

            if (stateArMarker.isPresent()) {
              final InertialFrameVelocity velocity = stateArMarker.get().inertialFrameVelocity();
              velocityFromArMarker.publish(createTwistMessage(velocity, velocityFromArMarker));
              arMarkerVelLogger.info(
                  "{} {} {} {} {}",
                  connectedNode.getCurrentTime().toSeconds(),
                  velocity.linearX(),
                  velocity.linearY(),
                  velocity.linearZ(),
                  velocity.angularZ());
            }

            if (stateArMarkerAndOdom.isPresent()) {
              final InertialFrameVelocity velocity =
                  stateArMarkerAndOdom.get().inertialFrameVelocity();
              velocityFromOdom.publish(createTwistMessage(velocity, velocityFromOdom));
              odomVelLogger.info(
                  "{} {} {} {} {}",
                  connectedNode.getCurrentTime().toSeconds(),
                  velocity.linearX(),
                  velocity.linearY(),
                  velocity.linearZ(),
                  velocity.angularZ());
            }
          }
        };

    connectedNode
        .getScheduledExecutorService()
        .scheduleAtFixedRate(dataRecorder, 0, 20, TimeUnit.MILLISECONDS);
  }

  private static Twist createTwistMessage(
      InertialFrameVelocity velocity, Publisher<Twist> publisher) {
    final Twist twist = publisher.newMessage();
    twist.getLinear().setX(velocity.linearX());
    twist.getLinear().setY(velocity.linearY());
    twist.getLinear().setZ(velocity.linearZ());
    twist.getAngular().setZ(velocity.angularZ());
    return twist;
  }

  private static MessagesSubscriberService<PoseStamped> getPoseSubscriber(
      ConnectedNode connectedNode) {
    final String poseTopic = "/arlocros/pose";
    logger.info("Subscribed to {} for getting pose.", poseTopic);
    return MessagesSubscriberService.create(
        connectedNode.<PoseStamped>newSubscriber(poseTopic, PoseStamped._TYPE),
        RosTime.create(connectedNode));
  }

  private static MessagesSubscriberService<Odometry> getOdometrySubscriber(
      ConnectedNode connectedNode) {
    final String odometryTopic = "/" + DRONE_NAME + "/odom";
    logger.info("Subscribed to {} for getting odometry", odometryTopic);
    return MessagesSubscriberService.create(
        connectedNode.<Odometry>newSubscriber(odometryTopic, Odometry._TYPE),
        RosTime.create(connectedNode));
  }
}
