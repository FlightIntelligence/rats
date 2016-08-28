package choreo;

import applications.trajectory.Trajectories;
import applications.trajectory.geom.point.Point3D;
import applications.trajectory.geom.point.Point4D;
import control.Trajectory4d;
import org.junit.Before;
import org.junit.Test;

import static applications.trajectory.TestUtils.verifyTrajectoryPos4D;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

/** @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be> */
public class ChoreographyTest {

  private Choreography choreotarget;
  private double duration = 20d;
  private Trajectory4d pathTrajectory;
  private Trajectory4d holdTrajectory;
  private Point4D point;
  private double radius = 0.2d;
  private double frequency = 0.1d;

  @Before
  public void setUp() {
    point = Point4D.create(5, 5, 5, 2);
    holdTrajectory = Trajectories.newHoldPositionTrajectory(point);
    pathTrajectory = Trajectories.newCircleTrajectory4D(Point3D.origin(), radius, frequency, 0);
    this.choreotarget =
        Choreography.builder()
            .withTrajectory(holdTrajectory)
            .forTime(duration)
            .withTrajectory(pathTrajectory)
            .forTime(duration)
            .build();
  }

  @Test
  public void testTwoSegmentTrajectoryChoreo() {
    verifyTrajectoryPos4D(choreotarget, 2, Point4D.create(5, 5, 5, 2));
    //First invocation past duration still get's old point. all following
    // trigger change in segment for first call.
    choreotarget.getDesiredPositionX(0d + duration);
    verifyTrajectoryPos4D(choreotarget, (1 / frequency) + duration, Point4D.create(radius, 0, 0, 0));
  }

  @Test
  public void testComplexExample() {
    Trajectory4d first =
        Trajectories.newStraightLineTrajectory(
            Point4D.create(0, 0, 1.5, 0), Point4D.create(5, 5, 5, 0), 0.6);
    Trajectory4d second =
        Trajectories.newCircleTrajectory4D(Point3D.create(4, 5, 5), 1, 0.10, Math.PI / 4);
    Trajectory4d third = Trajectories.newHoldPositionTrajectory(Point4D.create(1, 1, 2, 0));
    Choreography choreo =
        Choreography.builder()
            .withTrajectory(first)
            .forTime(20)
            .withTrajectory(second)
            .forTime(40)
            .withTrajectory(third)
            .forTime(30)
            .build();
    assertNotEquals(0, choreo.getDesiredPositionX(1));
    assertNotEquals(0, choreo.getDesiredPositionY(1));
    assertNotEquals(0, choreo.getDesiredPositionZ(1));
    assertEquals(0, choreo.getDesiredAngleZ(1), 0);
  }

  @Test
  public void testWithRealStartTimes() {
    double timeShift = 380;
    choreotarget.getDesiredPositionX(timeShift);
    choreotarget.getDesiredPositionY(timeShift);
    choreotarget.getDesiredPositionZ(timeShift);

    verifyTrajectoryPos4D(choreotarget, 2, Point4D.create(5, 5, 5, 2));
    //First invocation past duration still get's old point. all following
    // trigger change in segment for first call.
    choreotarget.getDesiredPositionX(timeShift + 0d + duration);
    verifyTrajectoryPos4D(
        choreotarget, (1 / frequency) + timeShift + duration, Point4D.create(radius, 0, 0, 0));
  }

  @Test
  public void testGetDuration() {
    assertEquals(2 * duration, choreotarget.getTrajectoryDuration(), 0);
  }

  @Test
  public void testFiniteTrajectory() {
    choreotarget =
        Choreography.builder()
            .withTrajectory(
                Trajectories.newStraightLineTrajectory(
                    Point4D.origin(), Point4D.create(1, 0, 0, 0), 0.5))
            .build();
    assertEquals(2, choreotarget.getTrajectoryDuration(), 0);

    choreotarget =
        Choreography.builder()
            .withTrajectory(
                Trajectories.newStraightLineTrajectory(
                    Point4D.origin(), Point4D.create(1, 0, 0, 0), 0.5))
            .forTime(5)
            .build();
    assertEquals(5, choreotarget.getTrajectoryDuration(), 0);
    choreotarget =
        Choreography.builder()
            .withTrajectory(
                Trajectories.newStraightLineTrajectory(
                    Point4D.origin(), Point4D.create(1, 0, 0, 0), 0.5))
            .withTrajectory(Trajectories.newExamplePendulumSwingTrajectory())
            .forTime(5)
            .build();
    assertEquals(7, choreotarget.getTrajectoryDuration(), 0);

    choreotarget =
        Choreography.builder()
            .withTrajectory(
                Trajectories.newStraightLineTrajectory(
                    Point4D.origin(), Point4D.create(1, 0, 0, 0), 0.5))
            .withTrajectory(Trajectories.newExamplePendulumSwingTrajectory())
            .forTime(2)
            .withTrajectory(Trajectories.newExampleCircleTrajectory4D())
            .forTime(10)
            .build();
    assertEquals(14, choreotarget.getTrajectoryDuration(), 0);
  }
}
