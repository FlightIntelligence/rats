package applications;

import com.google.common.base.Optional;
import geometry_msgs.PoseStamped;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import services.rossubscribers.MessagesSubscriberService;
import time.RosTime;

/** Print the poses published to the {@code /arlocros/pose} topic. */
public class PrintPose extends AbstractNodeMain {

  private static final Logger logger = LoggerFactory.getLogger(PrintPose.class);
  private static final String POSE_TOPIC = "/arlocros/pose";

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("PosePrinter");
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {
    MessagesSubscriberService<PoseStamped> poseSubscriber =
        MessagesSubscriberService.<PoseStamped>create(
            connectedNode.<PoseStamped>newSubscriber(POSE_TOPIC, PoseStamped._TYPE), RosTime.create(connectedNode));

    try {
      int i = 0;
      while (true) {
        Optional<PoseStamped> pose = poseSubscriber.getMostRecentMessage();
        if (pose.isPresent()) {
          double x = pose.get().getPose().getPosition().getX();
          double y = pose.get().getPose().getPosition().getY();
          double z = pose.get().getPose().getPosition().getZ();
          logger.info("{}, {}, {}", x, y, z);
        } else if (i % 100 == 0) {
          logger.info("No pose available");
        }
        i++;
        Thread.sleep(100);
      }
    } catch (InterruptedException e) {
      logger.debug("The thread is interrupted.", e);
      Thread.currentThread().interrupt();
    }
  }
}
