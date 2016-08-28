package control.localization;

import com.google.common.base.Optional;
import control.dto.BodyFrameVelocity;
import control.dto.DroneStateStamped;
import control.dto.InertialFrameVelocity;
import control.dto.Pose;
import geometry_msgs.PoseStamped;
import junitparams.JUnitParamsRunner;
import junitparams.Parameters;
import nav_msgs.Odometry;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import services.rossubscribers.MessagesSubscriberService;
import utils.TestUtils;

import static com.google.common.truth.Truth.assertThat;
import static org.mockito.Mockito.RETURNS_DEEP_STUBS;
import static org.mockito.Mockito.RETURNS_MOCKS;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

/** @author Hoang Tung Dinh */
@RunWith(JUnitParamsRunner.class)
public class BebopStateEstimatorWithPoseStampedAndOdomTest {

  private MessagesSubscriberService<PoseStamped> poseSubscriber;
  private MessagesSubscriberService<Odometry> odometrySubscriber;
  private StateEstimator bebopStateEstimator;

  private static Odometry createMockOdometry(BodyFrameVelocity bodyFrameVelocity) {
    final Odometry odometry = mock(Odometry.class, RETURNS_DEEP_STUBS);
    when(odometry.getTwist().getTwist().getLinear().getX()).thenReturn(bodyFrameVelocity.linearX());
    when(odometry.getTwist().getTwist().getLinear().getY()).thenReturn(bodyFrameVelocity.linearY());
    when(odometry.getTwist().getTwist().getLinear().getZ()).thenReturn(bodyFrameVelocity.linearZ());
    when(odometry.getTwist().getTwist().getAngular().getZ())
        .thenReturn(bodyFrameVelocity.angularZ());
    return odometry;
  }

  private static PoseStamped createMockPoseStamped(
      Pose pose, QuaternionAngle quaternionAngle, double timeStampInSeconds) {
    final PoseStamped poseStamped = mock(PoseStamped.class, RETURNS_DEEP_STUBS);
    when(poseStamped.getPose().getPosition().getX()).thenReturn(pose.x());
    when(poseStamped.getPose().getPosition().getY()).thenReturn(pose.y());
    when(poseStamped.getPose().getPosition().getZ()).thenReturn(pose.z());
    when(poseStamped.getPose().getOrientation().getW()).thenReturn(quaternionAngle.w());
    when(poseStamped.getPose().getOrientation().getX()).thenReturn(quaternionAngle.x());
    when(poseStamped.getPose().getOrientation().getY()).thenReturn(quaternionAngle.y());
    when(poseStamped.getPose().getOrientation().getZ()).thenReturn(quaternionAngle.z());
    when(poseStamped.getHeader().getStamp().toSeconds()).thenReturn(timeStampInSeconds);
    return poseStamped;
  }

  @Before
  public void setUp() {
    poseSubscriber = mock(MessagesSubscriberService.class);
    odometrySubscriber = mock(MessagesSubscriberService.class);
    bebopStateEstimator =
        BebopStateEstimatorWithPoseStampedAndOdom.create(poseSubscriber, odometrySubscriber);
  }

  @Test
  public void testGetCurrentState_noPoseReceived() {
    when(poseSubscriber.getMostRecentMessage()).thenReturn(Optional.<PoseStamped>absent());
    when(odometrySubscriber.getMostRecentMessage()).thenReturn(Optional.of(mock(Odometry.class)));
    assertThat(bebopStateEstimator.getCurrentState()).isAbsent();
  }

  @Test
  public void testGetCurrentState_noVelocityReceived() {
    when(poseSubscriber.getMostRecentMessage())
        .thenReturn(Optional.of(mock(PoseStamped.class, RETURNS_MOCKS)));
    when(odometrySubscriber.getMostRecentMessage()).thenReturn(Optional.<Odometry>absent());
    assertThat(bebopStateEstimator.getCurrentState()).isAbsent();
  }

  @Test
  @Parameters(source = DroneStateProvider.class)
  public void testGetCurrentState_withPoseAndVelocity(
      Pose pose,
      BodyFrameVelocity bodyFrameVelocity,
      InertialFrameVelocity inertialFrameVelocity,
      QuaternionAngle quaternionAngle) {
    final double timeStampInSeconds = 0.5;
    final PoseStamped poseStamped =
        createMockPoseStamped(pose, quaternionAngle, timeStampInSeconds);
    final Odometry odometry = createMockOdometry(bodyFrameVelocity);

    when(poseSubscriber.getMostRecentMessage()).thenReturn(Optional.of(poseStamped));
    when(odometrySubscriber.getMostRecentMessage()).thenReturn(Optional.of(odometry));

    final DroneStateStamped droneStateStamped =
        DroneStateStamped.create(pose, inertialFrameVelocity, timeStampInSeconds);

    TestUtils.assertPoseEqual(pose, droneStateStamped.pose());
    TestUtils.assertVelocityEqual(inertialFrameVelocity, droneStateStamped.inertialFrameVelocity());
    assertThat(timeStampInSeconds).isWithin(0.000001).of(droneStateStamped.getTimeStampInSeconds());
  }
}
