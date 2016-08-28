package utils;

import control.Trajectory4d;
import control.dto.BodyFrameVelocity;
import control.dto.InertialFrameVelocity;
import control.dto.Pose;

import static com.google.common.truth.Truth.assertThat;
import static org.mockito.Matchers.anyDouble;
import static org.mockito.Mockito.atLeastOnce;
import static org.mockito.Mockito.verify;

/**
 * Some test utilities for comparing double values and verifying methods called.
 *
 * @author Hoang Tung Dinh
 */
public final class TestUtils {
  private static final double DELTA = 0.01;

  public static void assertVelocityEqual(InertialFrameVelocity vel1, InertialFrameVelocity vel2) {
    assertThat(vel1.linearX()).isWithin(DELTA).of(vel2.linearX());
    assertThat(vel1.linearY()).isWithin(DELTA).of(vel2.linearY());
    assertThat(vel1.linearZ()).isWithin(DELTA).of(vel2.linearZ());
    assertThat(vel1.angularZ()).isWithin(DELTA).of(vel2.angularZ());
  }

  public static void assertVelocityEqual(BodyFrameVelocity vel1, BodyFrameVelocity vel2) {
    assertThat(vel1.linearX()).isWithin(DELTA).of(vel2.linearX());
    assertThat(vel1.linearY()).isWithin(DELTA).of(vel2.linearY());
    assertThat(vel1.linearZ()).isWithin(DELTA).of(vel2.linearZ());
    assertThat(vel1.angularZ()).isWithin(DELTA).of(vel2.angularZ());
  }

  public static void assertPoseEqual(Pose pose1, Pose pose2) {
    assertThat(pose1.x()).isWithin(DELTA).of(pose2.x());
    assertThat(pose1.y()).isWithin(DELTA).of(pose2.y());
    assertThat(pose1.z()).isWithin(DELTA).of(pose2.z());
    assertThat(pose1.yaw()).isWithin(DELTA).of(pose2.yaw());
  }

  public static void verifyTrajectoryCalled(Trajectory4d trajectory4d) {
    verify(trajectory4d, atLeastOnce()).getDesiredPositionX(anyDouble());
    verify(trajectory4d, atLeastOnce()).getDesiredPositionY(anyDouble());
    verify(trajectory4d, atLeastOnce()).getDesiredPositionZ(anyDouble());
    verify(trajectory4d, atLeastOnce()).getDesiredAngleZ(anyDouble());
  }
}
