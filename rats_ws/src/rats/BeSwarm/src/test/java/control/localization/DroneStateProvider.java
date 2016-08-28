package control.localization;

import control.dto.BodyFrameVelocity;
import control.dto.InertialFrameVelocity;
import control.dto.Pose;
import control.dto.Velocity;

/** @author Hoang Tung Dinh */
public final class DroneStateProvider {

  private DroneStateProvider() {}

  public static Object[] provideDroneStateValues() {
    final Pose pose1 = Pose.builder().setX(0).setY(0).setZ(10).setYaw(0.7853981633974483).build();
    final BodyFrameVelocity bVel1 =
        Velocity.builder().setLinearX(1).setLinearY(0).setLinearZ(0).setAngularZ(0).build();
    final InertialFrameVelocity iVel1 =
        Velocity.builder().setLinearX(0.707).setLinearY(0.707).setLinearZ(0).setAngularZ(0).build();
    final QuaternionAngle quAngle1 =
        QuaternionAngle.builder().setW(0.92388).setX(0).setY(0).setZ(0.38268).build();

    final Pose pose2 = Pose.builder().setX(0).setY(0).setZ(10).setYaw(0).build();
    final BodyFrameVelocity bVel2 =
        Velocity.builder().setLinearX(1).setLinearY(2).setLinearZ(3).setAngularZ(4).build();
    final InertialFrameVelocity iVel2 =
        Velocity.builder().setLinearX(1).setLinearY(2).setLinearZ(3).setAngularZ(4).build();
    final QuaternionAngle quAngle2 =
        QuaternionAngle.builder().setW(1).setX(0).setY(0).setZ(0).build();

    final Pose pose3 = Pose.builder().setX(0).setY(0).setZ(10).setYaw(1).build();
    final BodyFrameVelocity bVel3 =
        Velocity.builder().setLinearX(1).setLinearY(2).setLinearZ(3).setAngularZ(4).build();
    final InertialFrameVelocity iVel3 =
        Velocity.builder().setLinearX(-1.14).setLinearY(1.922).setLinearZ(3).setAngularZ(4).build();
    final QuaternionAngle quAngle3 =
        QuaternionAngle.builder().setW(0.87758).setX(0).setY(0).setZ(0.47943).build();

    return new Object[] {
      new Object[] {pose1, bVel1, iVel1, quAngle1},
      new Object[] {pose2, bVel2, iVel2, quAngle2},
      new Object[] {pose3, bVel3, iVel3, quAngle3}
    };
  }
}
