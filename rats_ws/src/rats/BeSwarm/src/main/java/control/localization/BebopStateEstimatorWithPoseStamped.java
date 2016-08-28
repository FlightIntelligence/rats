package control.localization;

import com.google.common.base.Optional;
import com.google.common.collect.EvictingQueue;
import control.dto.DroneStateStamped;
import control.dto.InertialFrameVelocity;
import control.dto.Pose;
import control.dto.Velocity;
import geometry_msgs.PoseStamped;
import services.rossubscribers.MessagesSubscriberService;

import javax.annotation.Nullable;
import java.util.Collection;

/**
 * A localization that gets both pose and velocity from ArMarker.
 *
 * @author Hoang Tung Dinh
 */
public final class BebopStateEstimatorWithPoseStamped implements StateEstimator {

  private final MessagesSubscriberService<PoseStamped> poseSubscriber;
  private final EvictingQueue<InertialFrameVelocity> velocityQueue;
  @Nullable private PoseStamped lastPoseStamped;

  private BebopStateEstimatorWithPoseStamped(
      MessagesSubscriberService<PoseStamped> poseSubscriber, int numOfVelocitiesToAverage) {
    this.poseSubscriber = poseSubscriber;
    velocityQueue = EvictingQueue.create(numOfVelocitiesToAverage);
  }

  public static BebopStateEstimatorWithPoseStamped create(
      MessagesSubscriberService<PoseStamped> poseSubscriber, int numOfVelocitiesToAverage) {
    return new BebopStateEstimatorWithPoseStamped(poseSubscriber, numOfVelocitiesToAverage);
  }

  @Override
  public Optional<DroneStateStamped> getCurrentState() {
    final Optional<PoseStamped> poseStamped = poseSubscriber.getMostRecentMessage();

    // if there is no pose available yet
    if (!poseStamped.isPresent()) {
      return Optional.absent();
    }

    // if there is no pose stored yet
    if (lastPoseStamped == null) {
      lastPoseStamped = poseStamped.get();
      return Optional.absent();
    }

    // if there is at least one pose stored and the pose now is different from the most recent pose
    if (!lastPoseStamped.getHeader().getStamp().equals(poseStamped.get().getHeader().getStamp())) {
      // compute the velocity between two most recent pose
      final double timeDelta =
          poseStamped.get().getHeader().getStamp().toSeconds()
              - lastPoseStamped.getHeader().getStamp().toSeconds();
      final Pose mostRecentPose = Pose.create(poseStamped.get());
      final Pose secondMostRecentPose = Pose.create(lastPoseStamped);
      final InertialFrameVelocity inertialFrameVelocity =
          getVelocity(mostRecentPose, secondMostRecentPose, timeDelta);
      // put the velocity into the queue
      velocityQueue.add(inertialFrameVelocity);
      lastPoseStamped = poseStamped.get();
    }

    if (velocityQueue.remainingCapacity() == 0) {
      final InertialFrameVelocity currentVelocity = getAverageVelocity(velocityQueue);
      return Optional.of(
          DroneStateStamped.create(
              Pose.create(lastPoseStamped),
              currentVelocity,
              lastPoseStamped.getHeader().getStamp().toSeconds()));
    } else {
      return Optional.absent();
    }
  }

  private static InertialFrameVelocity getAverageVelocity(
      Collection<InertialFrameVelocity> velocities) {
    double linearX = 0;
    double linearY = 0;
    double linearZ = 0;
    double angularZ = 0;

    for (final InertialFrameVelocity velocity : velocities) {
      linearX += velocity.linearX();
      linearY += velocity.linearY();
      linearZ += velocity.linearZ();
      angularZ += velocity.angularZ();
    }

    final int numberOfVelocities = velocities.size();

    return Velocity.builder()
        .setLinearX(linearX / numberOfVelocities)
        .setLinearY(linearY / numberOfVelocities)
        .setLinearZ(linearZ / numberOfVelocities)
        .setAngularZ(angularZ / numberOfVelocities)
        .build();
  }

  private static InertialFrameVelocity getVelocity(
      Pose mostRecentPose, Pose secondMostRecentPose, double timeDelta) {
    final double velLinearX = (mostRecentPose.x() - secondMostRecentPose.x()) / timeDelta;
    final double velLinearY = (mostRecentPose.y() - secondMostRecentPose.y()) / timeDelta;
    final double velLinearZ = (mostRecentPose.z() - secondMostRecentPose.z()) / timeDelta;
    final double velAngularZ = (mostRecentPose.yaw() - secondMostRecentPose.yaw()) / timeDelta;
    return Velocity.builder()
        .setLinearX(velLinearX)
        .setLinearY(velLinearY)
        .setLinearZ(velLinearZ)
        .setAngularZ(velAngularZ)
        .build();
  }
}
