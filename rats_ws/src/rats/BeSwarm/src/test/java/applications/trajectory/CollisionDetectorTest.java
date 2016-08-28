package applications.trajectory;

import applications.trajectory.geom.point.Point3D;
import applications.trajectory.geom.point.Point4D;
import choreo.Choreography;
import com.google.common.collect.Lists;
import control.FiniteTrajectory4d;
import org.junit.Before;
import org.junit.Test;

import java.util.List;

import static applications.trajectory.CollisionDetector.Collision;
import static junit.framework.TestCase.assertTrue;
import static org.junit.Assert.assertFalse;

/** @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be> */
public class CollisionDetectorTest {

  private FiniteTrajectory4d holdPos;
  private FiniteTrajectory4d lin1;
  private FiniteTrajectory4d circ1;
  private FiniteTrajectory4d circ1_phase;
  private FiniteTrajectory4d circ2;
  private FiniteTrajectory4d cork1;
  private FiniteTrajectory4d cork1_phase;
  private double EPS = 0.01;

  @Before
  public void setUp() {
    this.holdPos =
        Choreography.builder()
            .withTrajectory(Trajectories.newHoldPositionTrajectory(Point4D.create(5, 0, 5, 0)))
            .forTime(10)
            .build();
    this.lin1 =
        Trajectories.newStraightLineTrajectory(
            Point4D.create(0, 0, 5, 0), Point4D.create(5, 0, 5, 0), 0.5);
    double freq = 0.1;
    this.circ1 =
        Choreography.builder()
            .withTrajectory(
                Trajectories.circleTrajectoryBuilder()
                    .setRadius(1)
                    .setLocation(Point3D.create(1, 1, 5))
                    .setFrequency(freq)
                    .build())
            .forTime(100)
            .build();
    this.circ1_phase =
        Choreography.builder()
            .withTrajectory(
                Trajectories.circleTrajectoryBuilder()
                    .setRadius(1)
                    .setLocation(Point3D.create(1, 1, 5))
                    .setFrequency(freq)
                    .setPhase(Math.PI)
                    .build())
            .forTime(100)
            .build();
    this.circ2 =
        Choreography.builder()
            .withTrajectory(
                Trajectories.circleTrajectoryBuilder()
                    .setRadius(1)
                    .setLocation(Point3D.create(1, 1, 6))
                    .setFrequency(freq)
                    .build())
            .forTime(100)
            .build();

    FiniteTrajectory4d cork =
        Trajectories.newCorkscrewTrajectory(
            Point4D.create(1, 1, 1, 0), Point3D.create(10, 10, 10), 0.5, 1, 0.10, 0);
    this.cork1 =
        Choreography.builder()
            .withTrajectory(cork)
            .forTime(cork.getTrajectoryDuration() - EPS)
            .build();

    cork =
        Trajectories.newCorkscrewTrajectory(
            Point4D.create(1, 1, 1, 0), Point3D.create(10, 10, 10), 0.5, 1, 0.10, Math.PI);
    this.cork1_phase =
        Choreography.builder()
            .withTrajectory(cork)
            .forTime(cork.getTrajectoryDuration() - EPS)
            .build();
  }

  @Test
  public void testCollisionDetectAtPoint() {
    List<Collision> collisions =
        new CollisionDetector(Lists.newArrayList(holdPos, lin1), 1).findCollisions();
    assertTrue(isCollisionPresentFor(9.9, collisions));
    assertTrue(isCollisionPresentFor(8.5, collisions));
    assertTrue(isCollisionPresentFor(8.1, collisions));
    assertFalse(isCollisionPresentFor(8.0, collisions));
    assertFalse(isCollisionPresentFor(7.9, collisions));
    assertFalse(isCollisionPresentFor(7.8, collisions));
  }

  private boolean isCollisionPresentFor(double timeT, List<CollisionDetector.Collision> coll) {
    for (Collision c : coll) {
      if (Math.abs(c.getTimePoint() - timeT) < TestUtils.EPSILON) {
        return true;
      }
    }
    return false;
  }

  @Test
  public void testNoCollisionDetectedWPhaseShift() {
    List<Collision> collisions =
        new CollisionDetector(Lists.newArrayList(circ1, circ1_phase), 1).findCollisions();
    assertTrue(collisions.isEmpty());
  }

  @Test
  public void testNoCollisionDetectedWCoordShift() {
    List<Collision> collisions =
        new CollisionDetector(Lists.newArrayList(circ1, circ2), 1).findCollisions();
    assertTrue(collisions.isEmpty());
  }

  @Test
  public void testCorkscrew() {
    List<Collision> collisions =
        new CollisionDetector(Lists.newArrayList(cork1, cork1_phase), 1).findCollisions();
    assertTrue("Found " + collisions.size() + " collisions: " + collisions, collisions.isEmpty());
  }

  @Test
  public void testNonConnectingSegments() {

    Choreography.BuildableStepBuilder t1Builder =
        Choreography.builder()
            .withTrajectory(Trajectories.newHoldPositionTrajectory(Point4D.create(1, 0, 0, 0)))
            .forTime(5)
            .withTrajectory(Trajectories.newHoldPositionTrajectory(Point4D.create(-1, 0, 0, 0)))
            .forTime(5);

    FiniteTrajectory4d t1 = t1Builder.build();

    Choreography.BuildableStepBuilder t2Builder =
        Choreography.builder()
            .withTrajectory(Trajectories.newHoldPositionTrajectory(Point4D.create(-1, 0, 0, 0)))
            .forTime(5)
            .withTrajectory(Trajectories.newHoldPositionTrajectory(Point4D.create(1, 0, 0, 0)))
            .forTime(5);

    FiniteTrajectory4d t2 = t2Builder.build();

    List<Collision> collisions =
        new CollisionDetector(Lists.newArrayList(t1, t2), 1).findCollisions();
    assertFalse("Found " + collisions.size() + " collisions: " + collisions, collisions.isEmpty());
  }
}
