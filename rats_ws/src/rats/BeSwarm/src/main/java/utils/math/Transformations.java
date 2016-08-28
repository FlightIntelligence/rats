package utils.math;

import applications.trajectory.geom.point.Point3D;
import applications.trajectory.geom.point.Point4D;
import control.dto.BodyFrameVelocity;
import control.dto.InertialFrameVelocity;
import control.dto.Pose;
import control.dto.Velocity;
import geometry_msgs.Quaternion;
import org.ejml.simple.SimpleMatrix;

/** @author Hoang Tung Dinh */
public final class Transformations {

  private Transformations() {}

  /**
   * Compute euler angle from quarternion angle. The resulting angles are always in range [-pi, pi]
   *
   * @param quaternion the angle in quaternion representation
   * @return the angle in euler representation.
   * @see <a
   *     href="https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles">Equations</a>
   */
  public static EulerAngle quaternionToEulerAngle(Quaternion quaternion) {
    final double q0 = quaternion.getW();
    final double q1 = quaternion.getX();
    final double q2 = quaternion.getY();
    final double q3 = quaternion.getZ();

    final double eulerX = StrictMath.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
    final double eulerY = StrictMath.asin(2 * (q0 * q2 - q3 * q1));
    final double eulerZ = StrictMath.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));

    return EulerAngle.builder().setAngleX(eulerX).setAngleY(eulerY).setAngleZ(eulerZ).build();
  }

  /**
   * Transforms the velocity in the inertial frame to the velocity in the body frame.
   *
   * @param inertialFrameVelocity the velocity in the inertial frame
   * @param pose the pose associated with the velocity in the inertial frame
   * @return the velocity in the body frame
   */
  public static BodyFrameVelocity inertialFrameVelocityToBodyFrameVelocity(
      InertialFrameVelocity inertialFrameVelocity, Pose pose) {
    // same linearZ
    final double linearZ = inertialFrameVelocity.linearZ();
    // same angularZ
    final double angularZ = inertialFrameVelocity.angularZ();

    final double theta = -pose.yaw();
    final double sin = StrictMath.sin(theta);
    final double cos = StrictMath.cos(theta);

    final double linearX =
        inertialFrameVelocity.linearX() * cos - inertialFrameVelocity.linearY() * sin;
    final double linearY =
        inertialFrameVelocity.linearX() * sin + inertialFrameVelocity.linearY() * cos;

    return Velocity.builder()
        .setLinearX(linearX)
        .setLinearY(linearY)
        .setLinearZ(linearZ)
        .setAngularZ(angularZ)
        .build();
  }

  /**
   * Transforms the velocity in the body frame to the velocity in the inertial frame.
   *
   * @param bodyFrameVelocity the velocity in the body frame
   * @param pose the pose associated with the velocity
   * @return the velocity in the inertial frame
   */
  public static InertialFrameVelocity bodyFrameVelocityToInertialFrameVelocity(
      BodyFrameVelocity bodyFrameVelocity, Pose pose) {
    // same linearZ
    final double linearZ = bodyFrameVelocity.linearZ();
    // same angularZ
    final double angularZ = bodyFrameVelocity.angularZ();

    final double theta = pose.yaw();

    final double sin = StrictMath.sin(theta);
    final double cos = StrictMath.cos(theta);

    final double linearX = bodyFrameVelocity.linearX() * cos - bodyFrameVelocity.linearY() * sin;
    final double linearY = bodyFrameVelocity.linearX() * sin + bodyFrameVelocity.linearY() * cos;

    return Velocity.builder()
        .setLinearX(linearX)
        .setLinearY(linearY)
        .setLinearZ(linearZ)
        .setAngularZ(angularZ)
        .build();
  }

  /**
   * Translates a 3D point along the x-, y- and z-axes.
   *
   * @param point the point to be translated
   * @param dx the distance to move the point along the X axis
   * @param dy the distance to move the point along the Y axis
   * @param dz the distance to move the point along the Z axis
   * @return a new 3D point represent the initial {@code point} after the translation
   */
  public static Point3D translate(Point3D point, double dx, double dy, double dz) {
    return Point3D.create(point.getX() + dx, point.getY() + dy, point.getZ() + dz);
  }

  /**
   * Translates a 4D point along the x-, y- and z-axes and yaw angle.
   *
   * @param point the point to be translated
   * @param dx the distance to move the point along the X axis
   * @param dy the distance to move the point along the Y axis
   * @param dz the distance to move the point along the Z axis
   * @param dangle th distance to move the angle of this point.
   * @return a new 4D point represent the initial {@code point} after the translation
   */
  public static Point4D translate(Point4D point, double dx, double dy, double dz, double dangle) {
    return Point4D.create(
        point.getX() + dx, point.getY() + dy, point.getZ() + dz, point.getAngle() + dangle);
  }

  /**
   * Rotates a point using Euler rotation. This method uses the extrinsic rotation. The order of the
   * rotation is defined by {@code rotationOrder}.
   *
   * @param point the point to be rotated in 3D coordinate
   * @param rotationAngleX the rotation angle about the x-axis according to the right hand rule
   * @param rotationAngleY the rotation angle about the y-axis according to the right hand rule
   * @param rotationAngleZ the rotation angle about the z-axis according to the right hand rule
   * @param rotationOrder the rotation order of the transform
   * @return a new 3D point representing the rotated point
   * @see <a href="https://en.wikipedia.org/wiki/Euler_angles">Euler angles and Euler rotations</a>
   * @see <a href="https://en.wikipedia.org/wiki/Rotation_matrix">Rotation matrix</a>
   */
  public static Point3D rotate(
      Point3D point,
      double rotationAngleX,
      double rotationAngleY,
      double rotationAngleZ,
      RotationOrder rotationOrder) {
    // This method invokes some matrix multiplications and it could be a computational
    // bottleneck. Currently,
    // three rotation matrices are computed separately and then multiplied all together after
    // that. This allows
    // the rotation order to be changed later. However, if the rotation order is decided and
    // fixed, all matrix
    // multiplications can be hardcoded manually, which can improve the computational
    // efficiency.
    final SimpleMatrix rotationMatrixX = getRotationMatrixX(rotationAngleX);
    final SimpleMatrix rotationMatrixY = getRotationMatrixY(rotationAngleY);
    final SimpleMatrix rotationMatrixZ = getRotationMatrixZ(rotationAngleZ);

    final SimpleMatrix rotationMatrix =
        rotationOrder.get3dRotationMatrix(rotationMatrixX, rotationMatrixY, rotationMatrixZ);
    final SimpleMatrix originalPoint =
        new SimpleMatrix(new double[][] {{point.getX()}, {point.getY()}, {point.getZ()}});
    final SimpleMatrix rotatedPoint = rotationMatrix.mult(originalPoint);
    return Point3D.create(rotatedPoint.get(0, 0), rotatedPoint.get(1, 0), rotatedPoint.get(2, 0));
  }

  /**
   * Reverses the rotation having been done by {@link Transformations#rotate(Point3D, double,
   * double, double, RotationOrder)}.
   *
   * @param rotatedPoint the point to be rotated in 3D coordinate
   * @param rotationAngleX the initial rotation angle about the x-axis
   * @param rotationAngleY the initial rotation angle about the y-axis
   * @param rotationAngleZ the initial rotation angle about the z-axis
   * @param initialRotationOrder the initial rotation order
   * @return the initial point before the rotation {@link Transformations#rotate(Point3D, double,
   *     double, double, RotationOrder)}
   */
  public static Point3D reverseRotation(
      Point3D rotatedPoint,
      double rotationAngleX,
      double rotationAngleY,
      double rotationAngleZ,
      RotationOrder initialRotationOrder) {
    return rotate(
        rotatedPoint,
        -rotationAngleX,
        -rotationAngleY,
        -rotationAngleZ,
        initialRotationOrder.getInverseOrder());
  }

  private static SimpleMatrix getRotationMatrixZ(double rotationAngle) {
    final double sinZ = StrictMath.sin(rotationAngle);
    final double cosZ = StrictMath.cos(rotationAngle);
    return new SimpleMatrix(new double[][] {{cosZ, -sinZ, 0}, {sinZ, cosZ, 0}, {0, 0, 1}});
  }

  private static SimpleMatrix getRotationMatrixY(double rotationAngle) {
    final double sinY = StrictMath.sin(rotationAngle);
    final double cosY = StrictMath.cos(rotationAngle);
    return new SimpleMatrix(new double[][] {{cosY, 0, sinY}, {0, 1, 0}, {-sinY, 0, cosY}});
  }

  private static SimpleMatrix getRotationMatrixX(double rotationAngle) {
    final double sinX = StrictMath.sin(rotationAngle);
    final double cosX = StrictMath.cos(rotationAngle);
    return new SimpleMatrix(new double[][] {{1, 0, 0}, {0, cosX, -sinX}, {0, sinX, cosX}});
  }
}
