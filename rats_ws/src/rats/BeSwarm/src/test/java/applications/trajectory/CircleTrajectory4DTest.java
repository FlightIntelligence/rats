package applications.trajectory;

import applications.trajectory.geom.point.Point3D;
import com.google.common.collect.Lists;
import control.Trajectory4d;
import org.junit.Before;
import org.junit.Test;

import java.util.List;

import static applications.trajectory.TestUtils.assertBounds;

/** @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be> */
public class CircleTrajectory4DTest {

  private Trajectory4d target;
  private Trajectory4d targetPlaneShift;
  private Point3D origin = Point3D.create(0, 0, 5);
  private double radius = 1d;
  private double frequency = 0.1;
  private double planeshift = Math.PI / 6d;

  @Before
  public void setUp() {
    target = Trajectories.newCircleTrajectory4D(origin, radius, frequency, 0);
    //TODO test this as well.
    targetPlaneShift = Trajectories.newCircleTrajectory4D(origin, radius, frequency, planeshift);
  }

  @Test
  public void getTrajectoryLinearXTestBounds() throws Exception {
    List<Double> l = Lists.newArrayList();
    for (int i = 0; i < 1000; i++) {
      l.add(target.getDesiredPositionX(i / 10d));
    }
    assertBounds(l, origin.getX() - radius, origin.getX() + radius);
  }

  @Test
  public void getTrajectoryLinearYTestBounds() throws Exception {
    List<Double> l = Lists.newArrayList();
    for (int i = 0; i < 1000; i++) {
      l.add(target.getDesiredPositionY(i / 10d));
    }
    assertBounds(l, origin.getY() - radius, origin.getY() + radius);
  }

  @Test
  public void getTrajectoryLinearZTestBounds() throws Exception {
    List<Double> l = Lists.newArrayList();
    for (int i = 0; i < 1000; i++) {
      l.add(target.getDesiredPositionZ(i / 10d));
    }
    assertBounds(
        l,
        origin.getZ() - StrictMath.tan(planeshift) * radius,
        origin.getZ() + StrictMath.tan(planeshift) * radius);
  }

  @Test
  public void getTrajectoryAngularZTestBounds() throws Exception {
    List<Double> l = Lists.newArrayList();
    for (int i = 0; i < 1000; i++) {
      l.add(target.getDesiredPositionZ(i / 10d));
    }
    assertBounds(l, 0, Math.PI * 2);
  }

  @Test
  public void testCircleTranslocation() {
    double centerx = 15;
    double centery = 10;
    double centerz = 20;
    double radius = 1;
    target =
        Trajectories.newCircleTrajectory4D(
            Point3D.create(centerx, centery, centerz), radius, 0.1, 0);
    List<Double> lx = Lists.newArrayList();
    List<Double> ly = Lists.newArrayList();
    List<Double> lz = Lists.newArrayList();

    for (int i = 0; i < 1000; i++) {
      lx.add(target.getDesiredPositionX(i / 10d));
      ly.add(target.getDesiredPositionY(i / 10d));
      lz.add(target.getDesiredPositionZ(i / 10d));
    }
    assertBounds(lx, centerx - radius, centerx + radius);
    assertBounds(ly, centery - radius, centery + radius);
    assertBounds(lz, centerz, centerz);
  }

  @Test
  public void testPlaneAngle() {
    double centerx = 15;
    double centery = 10;
    double centerz = 20;
    List<Double> lz = Lists.newArrayList();
    target =
        Trajectories.newCircleTrajectory4D(
            Point3D.create(centerx, centery, centerz), radius, frequency, Math.PI / 2);
    for (int i = 0; i < 1000; i++) {
      lz.add(target.getDesiredPositionZ(i / 10d));
    }
    assertBounds(lz, centerz - radius, centerz + radius);
  }

  @Test
  public void testNoAngularMovement() {
    testAngularMovement(0);
  }

  private void testAngularMovement(double angle) {
    double centerx = 15;
    double centery = 10;
    double centerz = 20;
    double orientation = angle;
    List<Double> lz = Lists.newArrayList();
    target =
        Trajectories.newConstantYawCircleTrajectory4D(
            Point3D.create(centerx, centery, centerz), radius, frequency, 0, orientation);
    for (int i = 0; i < 1000; i++) {
      lz.add(target.getDesiredAngleZ(i / 10d));
    }
    assertBounds(lz, orientation, orientation);
  }

  @Test
  public void testConcreteAngularMovement() {
    testAngularMovement(Math.PI / 2);
  }

  @Test(expected = IllegalArgumentException.class)
  public void testCheckArgWithin2PI() {
    double centerx = 15;
    double centery = 10;
    double centerz = 20;
    double orientation = Math.PI * 2 + 5;
    target =
        Trajectories.newConstantYawCircleTrajectory4D(origin, radius, frequency, 0, orientation);
  }
}
