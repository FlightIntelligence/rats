package applications.trajectory;

import applications.trajectory.geom.point.Point4D;
import control.FiniteTrajectory4d;
import control.Trajectory1d;
import control.Trajectory4d;

import java.util.Collections;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be> */
//TODO merge this util class with utils.TestUtil
public final class TestUtils {
  public static final double EPSILON = 0.001;
  public static final double DELTA = EPSILON;

  private TestUtils() {}

  public static void assertBounds(List<Double> results, double min, double max) {
    for (Double d : results) {
      assertTrue(Collections.min(results) + EPSILON >= min - EPSILON);
      assertTrue(Collections.max(results) - EPSILON <= max + EPSILON);
    }
  }

  public static void verifyPositionFrequencyRadiusRelation(
      double frequency, double radius, Trajectory1d target) {
    for (double i = 0; i < 30; i += 1 / frequency) {
      assertEquals(radius, target.getDesiredPosition(i), 0.01);
    }
  }

  public static void verifyVelocityFrequencyRadiusRelation(double frequency, Trajectory1d target) {
    for (double i = 0; i < 30; i += 1 / frequency) {
      assertEquals(0, getVelocity(target, i), 0.01);
    }
  }

  public static double getVelocity(Trajectory1d trajectory, double t) {
    return (trajectory.getDesiredPosition(t + DELTA) - trajectory.getDesiredPosition(t)) / DELTA;
  }

  public static void testSpeedBounds(Trajectory1d target, double maxspeed) {
    for (double i = 0; i < 30; i += 2) {
      assertTrue(Math.abs(getVelocity(target, i)) < maxspeed);
    }
  }

  public static double getVelocityX(Trajectory4d trajectory, double t) {
    return (trajectory.getDesiredPositionX(t + DELTA) - trajectory.getDesiredPositionX(t)) / DELTA;
  }

  public static double getVelocityY(Trajectory4d trajectory, double t) {
    return (trajectory.getDesiredPositionY(t + DELTA) - trajectory.getDesiredPositionY(t)) / DELTA;
  }

  public static double getVelocityZ(Trajectory4d trajectory, double t) {
    return (trajectory.getDesiredPositionZ(t + DELTA) - trajectory.getDesiredPositionZ(t)) / DELTA;
  }

  public static double getAngularVelocity(Trajectory4d trajectory, double t) {
    return (trajectory.getDesiredAngleZ(t + DELTA) - trajectory.getDesiredAngleZ(t)) / DELTA;
  }

  public static void verifyTrajectoryPos4D(Trajectory4d traj, double time, Point4D target) {
    assertEquals(target.getX(), traj.getDesiredPositionX(time), EPSILON);
    assertEquals(target.getY(), traj.getDesiredPositionY(time), EPSILON);
    assertEquals(target.getZ(), traj.getDesiredPositionZ(time), EPSILON);
    assertEquals(target.getAngle(), traj.getDesiredAngleZ(time), EPSILON);
  }

  public static void verifyTrajectoryCollisions(
      List<FiniteTrajectory4d> trajectories, double minimumDistance) {
    CollisionDetector detector = new CollisionDetector(trajectories, minimumDistance);
  }

  public static void verifyTrajectoryCollisions(List<FiniteTrajectory4d> trajectories) {
    CollisionDetector detector = new CollisionDetector(trajectories);
    verifyForCollisionsOccuring(detector);
  }

  private static void verifyForCollisionsOccuring(CollisionDetector detector) {
    List<CollisionDetector.Collision> collisions = detector.findCollisions();
    assertTrue("Found " + collisions.size() + " collisions: " + collisions, collisions.isEmpty());
  }
}
