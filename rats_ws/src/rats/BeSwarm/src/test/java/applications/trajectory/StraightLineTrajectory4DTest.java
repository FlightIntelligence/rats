package applications.trajectory;

import applications.trajectory.geom.point.Point4D;
import control.Trajectory4d;
import org.junit.Before;
import org.junit.Test;

import static applications.trajectory.TestUtils.EPSILON;
import static applications.trajectory.TestUtils.getVelocityX;
import static applications.trajectory.TestUtils.getVelocityY;
import static applications.trajectory.TestUtils.getVelocityZ;
import static org.junit.Assert.assertEquals;

/** @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be> */
public class StraightLineTrajectory4DTest {
  protected final double speed = 1;
  protected Point4D before;
  protected Point4D after;
  protected Point4D afterNotOrigin;
  protected Trajectory4d target;
  protected Trajectory4d target2;

  @Before
  public void setUp() {
    before = Point4D.create(0, 0, 0, 0);
    afterNotOrigin = Point4D.create(10, 10, 10, 0);
    after = Point4D.create(10, 0, 0, Math.PI / 2);
    createTargets();
    init();
  }

  protected void createTargets() {
    target = Trajectories.newStraightLineTrajectory(before, after, speed);
    target2 = Trajectories.newStraightLineTrajectory(before, afterNotOrigin, speed);
  }

  private void init() {
    target.getDesiredPositionX(0);
    target.getDesiredPositionY(0);
    target.getDesiredPositionZ(0);
    target.getDesiredAngleZ(0);
    target2.getDesiredPositionX(0);
    target2.getDesiredPositionY(0);
    target2.getDesiredPositionZ(0);
    target2.getDesiredAngleZ(0);
  }

  @Test
  public void getTrajectoryLinearX() throws Exception {
    assertEquals(5, target.getDesiredPositionX(5), 0);
    assertEquals(10, target.getDesiredPositionX(10), 0);

    double t = 5;
    testPartialDistanceCovered(t);
    t = 10;
    testPartialDistanceCovered(t);
  }

  private void testPartialDistanceCovered(double t) {
    double toCalc = 10d / Math.sqrt(300d) * t;
    assertEquals(toCalc, target2.getDesiredPositionX(t), 0.01);
  }

  @Test
  public void getTrajectoryLinearXVelocity() {
    assertEquals(1, getVelocityX(target, 5), EPSILON);
  }

  @Test
  public void getTrajectoryLinearXTestHoldAtEnd() throws Exception {
    target.getDesiredPositionX(15);
    assertEquals(10, target.getDesiredPositionX(15), EPSILON);
  }

  @Test
  public void getTrajectoryLinearY() throws Exception {
    assertEquals(0, target.getDesiredPositionY(5), EPSILON);
    assertEquals(0, target.getDesiredPositionY(10), EPSILON);

    double t = 5;
    double toCalc = 10d / Math.sqrt(300d) * t;
    assertEquals(toCalc, target2.getDesiredPositionY(t), EPSILON);
    t = 10;
    toCalc = 10d / Math.sqrt(300d) * t;
    assertEquals(toCalc, target2.getDesiredPositionY(t), EPSILON);
  }

  @Test
  public void getTrajectoryLinearYVelocity() {
    assertEquals(0, getVelocityY(target, 5), 0);
  }

  @Test
  public void getTrajectoryLinearZ() throws Exception {
    assertEquals(0, target.getDesiredPositionZ(5), 0);
    assertEquals(0, target.getDesiredPositionZ(10), 0);

    double t = 5;
    double toCalc = 10d / Math.sqrt(300d) * t;
    assertEquals(toCalc, target2.getDesiredPositionZ(t), 0.01);
    t = 10;
    toCalc = 10d / Math.sqrt(300d) * t;
    assertEquals(toCalc, target2.getDesiredPositionZ(t), 0.01);
  }

  @Test
  public void getTrajectoryLinearZVelocity() {
    assertEquals(0, getVelocityZ(target, 5), 0);
  }

  @Test
  public void getTrajectoryAngularZ() throws Exception {
    assertEquals(Math.PI / 4, target.getDesiredAngleZ(5), 0);
    assertEquals(Math.PI / 2, target.getDesiredAngleZ(10), 0);
  }

  @Test
  public void testTrajectoryProgression() throws Exception {
    double duration = 10492;
    target = Trajectories.newStraightLineTrajectory(before, after, speed);
    target2 = Trajectories.newStraightLineTrajectory(before, afterNotOrigin, speed);
    assertEquals(0, target.getDesiredPositionX(duration + 0), 0);
    assertEquals(5, target.getDesiredPositionX(duration + 5), 0);
    assertEquals(10, target.getDesiredPositionX(duration + 10), 0);

    double t = 0;
    testPartialDistanceCovered(t);
    t = 5;
    testPartialDistanceCovered(t);
    t = 10;
    testPartialDistanceCovered(t);
  }
}
