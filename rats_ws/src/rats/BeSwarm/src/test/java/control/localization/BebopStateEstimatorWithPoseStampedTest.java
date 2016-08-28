package control.localization;

import com.google.common.base.Optional;
import control.dto.DroneStateStamped;
import geometry_msgs.PoseStamped;
import geometry_msgs.Quaternion;
import org.junit.Test;
import services.rossubscribers.MessagesSubscriberService;

import static com.google.common.truth.Truth.assertThat;
import static org.mockito.Mockito.RETURNS_DEEP_STUBS;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

/**
 * Tests for {@link BebopStateEstimatorWithPoseStamped}
 *
 * @author Hoang Tung Dinh
 */
public class BebopStateEstimatorWithPoseStampedTest {

  private static final double DELTA = 0.000001;

  @Test
  public void testGetCurrentState() {
    final MessagesSubscriberService<PoseStamped> poseSubscriber =
        mock(MessagesSubscriberService.class);
    final StateEstimator stateEstimator =
        BebopStateEstimatorWithPoseStamped.create(poseSubscriber, 1);

    // when there is no pose message yet
    when(poseSubscriber.getMostRecentMessage()).thenReturn(Optional.<PoseStamped>absent());
    assertThat(stateEstimator.getCurrentState()).isAbsent();
    assertThat(stateEstimator.getCurrentState()).isAbsent();
    assertThat(stateEstimator.getCurrentState()).isAbsent();

    // when there is only one pose message
    final PoseStamped firstPose = mock(PoseStamped.class, RETURNS_DEEP_STUBS);
    when(firstPose.getHeader().getStamp().toSeconds()).thenReturn(1.0);
    when(firstPose.getPose().getPosition().getX()).thenReturn(0.0);
    when(firstPose.getPose().getPosition().getY()).thenReturn(0.0);
    when(firstPose.getPose().getPosition().getZ()).thenReturn(0.0);
    when(firstPose.getPose().getOrientation())
        .thenReturn(mock(Quaternion.class, RETURNS_DEEP_STUBS));

    when(poseSubscriber.getMostRecentMessage()).thenReturn(Optional.of(firstPose));
    assertThat(stateEstimator.getCurrentState()).isAbsent();
    assertThat(stateEstimator.getCurrentState()).isAbsent();
    assertThat(stateEstimator.getCurrentState()).isAbsent();

    // when there are two pose messages
    final PoseStamped secondPose = mock(PoseStamped.class, RETURNS_DEEP_STUBS);
    when(secondPose.getHeader().getStamp().toSeconds()).thenReturn(2.0);
    when(secondPose.getPose().getPosition().getX()).thenReturn(1.0);
    when(secondPose.getPose().getPosition().getY()).thenReturn(2.0);
    when(secondPose.getPose().getPosition().getZ()).thenReturn(3.0);
    when(secondPose.getPose().getOrientation())
        .thenReturn(mock(Quaternion.class, RETURNS_DEEP_STUBS));

    when(poseSubscriber.getMostRecentMessage()).thenReturn(Optional.of(secondPose));
    testCorrectFirstState(stateEstimator);
    // test twice
    testCorrectFirstState(stateEstimator);

    // when there are three pose messages
    final PoseStamped thirdPose = mock(PoseStamped.class, RETURNS_DEEP_STUBS);
    when(thirdPose.getHeader().getStamp().toSeconds()).thenReturn(3.0);
    when(thirdPose.getPose().getPosition().getX()).thenReturn(0.0);
    when(thirdPose.getPose().getPosition().getY()).thenReturn(0.0);
    when(thirdPose.getPose().getPosition().getZ()).thenReturn(0.0);
    when(thirdPose.getPose().getOrientation())
        .thenReturn(mock(Quaternion.class, RETURNS_DEEP_STUBS));

    when(poseSubscriber.getMostRecentMessage()).thenReturn(Optional.of(thirdPose));
    testCorrectSecondState(stateEstimator);
    // test twice
    testCorrectSecondState(stateEstimator);
  }

  private static void testCorrectSecondState(StateEstimator stateEstimator) {
    final Optional<DroneStateStamped> secondState = stateEstimator.getCurrentState();
    assertThat(secondState).isPresent();
    assertThat(secondState.get().getTimeStampInSeconds()).isWithin(DELTA).of(3.0);
    assertThat(secondState.get().pose().x()).isWithin(DELTA).of(0.0);
    assertThat(secondState.get().pose().y()).isWithin(DELTA).of(0.0);
    assertThat(secondState.get().pose().z()).isWithin(DELTA).of(0.0);
    assertThat(secondState.get().inertialFrameVelocity().linearX()).isWithin(DELTA).of(-1.0);
    assertThat(secondState.get().inertialFrameVelocity().linearY()).isWithin(DELTA).of(-2.0);
    assertThat(secondState.get().inertialFrameVelocity().linearZ()).isWithin(DELTA).of(-3.0);
  }

  private static void testCorrectFirstState(StateEstimator stateEstimator) {
    final Optional<DroneStateStamped> firstState = stateEstimator.getCurrentState();
    assertThat(firstState).isPresent();
    assertThat(firstState.get().getTimeStampInSeconds()).isWithin(DELTA).of(2.0);
    assertThat(firstState.get().pose().x()).isWithin(DELTA).of(1.0);
    assertThat(firstState.get().pose().y()).isWithin(DELTA).of(2.0);
    assertThat(firstState.get().pose().z()).isWithin(DELTA).of(3.0);
    assertThat(firstState.get().inertialFrameVelocity().linearX()).isWithin(DELTA).of(1.0);
    assertThat(firstState.get().inertialFrameVelocity().linearY()).isWithin(DELTA).of(2.0);
    assertThat(firstState.get().inertialFrameVelocity().linearZ()).isWithin(DELTA).of(3.0);
  }
}
