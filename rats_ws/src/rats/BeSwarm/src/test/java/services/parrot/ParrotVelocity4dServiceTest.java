package services.parrot;

import control.dto.BodyFrameVelocity;
import control.dto.InertialFrameVelocity;
import control.dto.Pose;
import control.dto.Velocity;
import geometry_msgs.Twist;
import geometry_msgs.Vector3;
import junitparams.JUnitParamsRunner;
import junitparams.Parameters;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.ArgumentCaptor;
import org.ros.node.topic.Publisher;
import services.Velocity4dService;
import time.TimeProvider;
import utils.TestUtils;
import utils.math.VelocityProvider;
import utils.math.VelocityProviderWithThreshold;

import javax.annotation.Nullable;

import static com.google.common.truth.Truth.assertThat;
import static org.mockito.Mockito.RETURNS_DEEP_STUBS;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

/** @author Hoang Tung Dinh */
@RunWith(JUnitParamsRunner.class)
public class ParrotVelocity4dServiceTest {

  @Nullable private Publisher<Twist> publisher;
  @Nullable private Twist twist;
  @Nullable private Vector3 linear;
  @Nullable private Vector3 angular;

  @Before
  public void setUp() {
    publisher = mock(Publisher.class, RETURNS_DEEP_STUBS);
    when(publisher.getTopicName().toString()).thenReturn("/bebop/cmd_vel");

    twist = mock(Twist.class, RETURNS_DEEP_STUBS);
    when(publisher.newMessage()).thenReturn(twist);

    linear = mock(Vector3.class);
    angular = mock(Vector3.class);
    when(twist.getLinear()).thenReturn(linear);
    when(twist.getAngular()).thenReturn(angular);
  }

  @After
  public void tearDown() {
    publisher = null;
    twist = null;
    linear = null;
    angular = null;
  }

  @Test
  @Parameters(source = VelocityProvider.class)
  public void testSendVelocityMessageWithoutThreshold(
      Pose pose, BodyFrameVelocity bodyFrameVelocity, InertialFrameVelocity inertialFrameVelocity) {
    final Velocity4dService parrotVelocity4dService =
        ParrotVelocity4dService.builder()
            .publisher(publisher)
            .timeProvider(mock(TimeProvider.class))
            .build();
    parrotVelocity4dService.sendInertialFrameVelocity(inertialFrameVelocity, pose);
    checkMessageSetUp(bodyFrameVelocity, linear, angular);
    checkCorrectTwistMessageSent(publisher, twist);
  }

  @Test
  @Parameters(source = VelocityProviderWithThreshold.class)
  public void testSendVelocityMessageWithThreshold(
      Pose pose, BodyFrameVelocity bodyFrameVelocity, InertialFrameVelocity inertialFrameVelocity) {
    final Velocity4dService parrotVelocity4dService =
        ParrotVelocity4dService.builder()
            .publisher(publisher)
            .timeProvider(mock(TimeProvider.class))
            .minLinearX(-0.25)
            .minLinearY(-0.25)
            .minLinearZ(-0.25)
            .minAngularZ(-0.25)
            .maxLinearX(0.25)
            .maxLinearY(0.25)
            .maxLinearZ(0.25)
            .maxAngularZ(0.25)
            .build();
    parrotVelocity4dService.sendInertialFrameVelocity(inertialFrameVelocity, pose);
    checkMessageSetUp(bodyFrameVelocity, linear, angular);
    checkCorrectTwistMessageSent(publisher, twist);
  }

  private void checkCorrectTwistMessageSent(Publisher<Twist> publisher, Twist twist) {
    final ArgumentCaptor<Twist> twistArgumentCaptor = ArgumentCaptor.forClass(Twist.class);
    verify(publisher).publish(twistArgumentCaptor.capture());
    assertThat(twistArgumentCaptor.getValue()).isEqualTo(twist);
  }

  private void checkMessageSetUp(
      BodyFrameVelocity bodyFrameVelocity, Vector3 linear, Vector3 angular) {
    final ArgumentCaptor<Double> argumentCaptor = ArgumentCaptor.forClass(Double.class);

    verify(linear).setX(argumentCaptor.capture());
    final double linearX = argumentCaptor.getValue();

    verify(linear).setY(argumentCaptor.capture());
    final double linearY = argumentCaptor.getValue();

    verify(linear).setZ(argumentCaptor.capture());
    final double linearZ = argumentCaptor.getValue();

    verify(angular).setZ(argumentCaptor.capture());
    final double angularZ = argumentCaptor.getValue();

    final BodyFrameVelocity sentBodyFrameVelocity =
        Velocity.builder()
            .setLinearX(linearX)
            .setLinearY(linearY)
            .setLinearZ(linearZ)
            .setAngularZ(angularZ)
            .build();
    TestUtils.assertVelocityEqual(bodyFrameVelocity, sentBodyFrameVelocity);
  }
}
