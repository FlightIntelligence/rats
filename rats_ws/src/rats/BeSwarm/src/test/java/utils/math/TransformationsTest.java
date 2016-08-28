package utils.math;

import applications.trajectory.geom.point.Point3D;
import control.dto.BodyFrameVelocity;
import control.dto.InertialFrameVelocity;
import control.dto.Pose;
import geometry_msgs.Quaternion;
import junitparams.JUnitParamsRunner;
import junitparams.Parameters;
import org.junit.Test;
import org.junit.runner.RunWith;
import utils.TestUtils;

import static com.google.common.truth.Truth.assertThat;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

/** @author Hoang Tung Dinh */
@RunWith(JUnitParamsRunner.class)
public class TransformationsTest {

  private static final double DELTA = 0.01;

  private static void assertPoint3DEqual(Point3D p0, Point3D p1) {
    assertThat(p0.getX()).isWithin(DELTA).of(p1.getX());
    assertThat(p0.getY()).isWithin(DELTA).of(p1.getY());
    assertThat(p0.getZ()).isWithin(DELTA).of(p1.getZ());
  }

  private Object[] eulerAndCorrespondingQuaternions() {
    return new Object[] {
      new Object[] {1, 0, 0, 0, 0, 0, 0},
      new Object[] {0.7071, 0, 0.7071, 0, 0, 1.5707963267948966, 0},
      new Object[] {0.7071, 0, -0.7071, 0, 0, -1.5707963267948966, 0}
    };
  }

  @Test
  @Parameters(method = "eulerAndCorrespondingQuaternions")
  public void testQuaternionToEulerAngle(
      double quaternionW,
      double quaternionX,
      double quaternionY,
      double quaternionZ,
      double eulerX,
      double eulerY,
      double eulerZ) {
    final Quaternion mockQuaternion = mock(Quaternion.class);
    when(mockQuaternion.getW()).thenReturn(quaternionW);
    when(mockQuaternion.getX()).thenReturn(quaternionX);
    when(mockQuaternion.getY()).thenReturn(quaternionY);
    when(mockQuaternion.getZ()).thenReturn(quaternionZ);

    final EulerAngle eulerAngle = Transformations.quaternionToEulerAngle(mockQuaternion);

    assertThat(eulerAngle.angleX()).isWithin(DELTA).of(eulerX);
    assertThat(eulerAngle.angleY()).isWithin(DELTA).of(eulerY);
    assertThat(eulerAngle.angleZ()).isWithin(DELTA).of(eulerZ);
  }

  @Test
  @Parameters(source = VelocityProvider.class)
  public void testBodyFrameVelocityToInertialFrameVelocity(
      Pose pose, BodyFrameVelocity bodyFrameVelocity, InertialFrameVelocity inertialFrameVelocity) {
    TestUtils.assertVelocityEqual(
        Transformations.bodyFrameVelocityToInertialFrameVelocity(bodyFrameVelocity, pose),
        inertialFrameVelocity);
  }

  @Test
  @Parameters(source = VelocityProvider.class)
  public void testInertialFrameVelocityToBodyFrameVelocity(
      Pose pose, BodyFrameVelocity bodyFrameVelocity, InertialFrameVelocity inertialFrameVelocity) {
    TestUtils.assertVelocityEqual(
        Transformations.inertialFrameVelocityToBodyFrameVelocity(inertialFrameVelocity, pose),
        bodyFrameVelocity);
  }

  @Test
  public void testTranslate() {
    final Point3D point3D = Point3D.create(1, 2.5, 3.2);
    final Point3D translatedPoint = Transformations.translate(point3D, -3.2, -10.3, 20);
    final Point3D desiredTranslatedPoint = Point3D.create(-2.2, -7.8, 23.2);
    assertPoint3DEqual(translatedPoint, desiredTranslatedPoint);
  }

  private Object[] rotationPointValues() {
    return new Object[] {
      new Object[] {Point3D.create(1, 0, 0), 0, 0, StrictMath.PI, Point3D.create(-1, 0, 0)},
      new Object[] {Point3D.create(1, 0, 0), 0, StrictMath.PI / 2, 0, Point3D.create(0, 0, -1)},
      new Object[] {Point3D.create(0, 0, 1), StrictMath.PI / 2, 0, 0, Point3D.create(0, -1, 0)},
      new Object[] {
        Point3D.create(0, 1, 0),
        StrictMath.PI / 2,
        StrictMath.PI / 2,
        StrictMath.PI / 2,
        Point3D.create(0, 1, 0)
      },
      new Object[] {
        Point3D.create(1, 1, 0),
        StrictMath.PI / 2,
        StrictMath.PI / 2,
        StrictMath.PI / 2,
        Point3D.create(0, 1, -1)
      }
    };
  }

  @Test
  @Parameters(method = "rotationPointValues")
  public void testRotateXYZ(
      Point3D initialPoint,
      double rotationAngleX,
      double rotationAngleY,
      double rotationAngleZ,
      Point3D rotatedPoint) {
    final Point3D p =
        Transformations.rotate(
            initialPoint, rotationAngleX, rotationAngleY, rotationAngleZ, RotationOrder.XYZ);
    assertPoint3DEqual(p, rotatedPoint);
  }

  @Test
  @Parameters(method = "rotationPointValues")
  public void testReverseRotationXYZ(
      Point3D initialPoint,
      double rotationAngleX,
      double rotationAngleY,
      double rotationAngleZ,
      Point3D rotatedPoint) {
    final Point3D p =
        Transformations.reverseRotation(
            rotatedPoint, rotationAngleX, rotationAngleY, rotationAngleZ, RotationOrder.XYZ);
    assertPoint3DEqual(p, initialPoint);
  }
}
