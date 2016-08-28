package utils.math;

import control.dto.BodyFrameVelocity;
import control.dto.InertialFrameVelocity;
import control.dto.Pose;
import control.dto.Velocity;

/** @author Hoang Tung Dinh */
public final class VelocityProviderWithThreshold {

  private VelocityProviderWithThreshold() {}

  public static Object[] provideVelocityValuesWithThresholdOneFouth() {
    final Pose pose1 = Pose.builder().setX(0).setY(0).setZ(10).setYaw(0.7853981633974483).build();
    final BodyFrameVelocity bVel1 =
        Velocity.builder().setLinearX(0.25).setLinearY(0).setLinearZ(0).setAngularZ(0).build();
    final InertialFrameVelocity iVel1 =
        Velocity.builder().setLinearX(0.707).setLinearY(0.707).setLinearZ(0).setAngularZ(0).build();

    final Pose pose2 = Pose.builder().setX(0).setY(0).setZ(10).setYaw(0).build();
    final BodyFrameVelocity bVel2 =
        Velocity.builder()
            .setLinearX(0.25)
            .setLinearY(0.25)
            .setLinearZ(0.25)
            .setAngularZ(0.25)
            .build();
    final InertialFrameVelocity iVel2 =
        Velocity.builder().setLinearX(1).setLinearY(2).setLinearZ(3).setAngularZ(4).build();

    final Pose pose3 = Pose.builder().setX(0).setY(0).setZ(10).setYaw(1).build();
    final BodyFrameVelocity bVel3 =
        Velocity.builder()
            .setLinearX(0.25)
            .setLinearY(0.25)
            .setLinearZ(0.25)
            .setAngularZ(0.25)
            .build();
    final InertialFrameVelocity iVel3 =
        Velocity.builder().setLinearX(-1.14).setLinearY(1.922).setLinearZ(3).setAngularZ(4).build();
    return new Object[] {
      new Object[] {pose1, bVel1, iVel1},
      new Object[] {pose2, bVel2, iVel2},
      new Object[] {pose3, bVel3, iVel3}
    };
  }
}
