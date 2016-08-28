package applications.trajectory;

import applications.trajectory.geom.LineSegment;
import applications.trajectory.geom.point.Point3D;
import com.google.auto.value.AutoValue;
import com.google.common.base.Optional;
import com.google.common.collect.Lists;
import control.FiniteTrajectory4d;
import control.Trajectory4d;

import java.util.Collection;
import java.util.List;

import static applications.trajectory.TrajectoryUtils.sampleTrajectory;
import static applications.trajectory.geom.point.Point3D.dot;
import static applications.trajectory.geom.point.Point3D.minus;
import static applications.trajectory.geom.point.Point3D.plus;
import static applications.trajectory.geom.point.Point3D.project;
import static applications.trajectory.geom.point.Point3D.scale;

/** @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be> */
public class CollisionDetector {
  private static final double DEFAULT_MINIMUM_DISTANCE = 1;
  private static final double DEFAULT_TIME_DELTA = 0.01;
  private static final double EPS = 1.0E-8;
  private static final CollisionTester SINGLE_SAMPLE_DISTANCE_BASED_COLLISION =
      new CollisionTester() {

        @Override
        public Optional<Collision> isCollision(
            double t, FiniteTrajectory4d first, FiniteTrajectory4d second, double minimumDistance) {
          Point3D firstPoint = project(sampleTrajectory(first, t));
          Point3D secondPoint = project(sampleTrajectory(second, t));
          double actualDistance = Point3D.distance(firstPoint, secondPoint);
          if (actualDistance < minimumDistance - TestUtils.EPSILON) {
            return Optional.of(Collision.create(t, actualDistance, first, second));
          }
          return Optional.absent();
        }
      };
  private static final CollisionTester TWO_SAMPLE_LINE_DISTANCE_BASED_COLLISION =
      new CollisionTester() {

        @Override
        public Optional<Collision> isCollision(
            double t, FiniteTrajectory4d first, FiniteTrajectory4d second, double minimumDistance) {
          Point3D firstPointT1 = project(sampleTrajectory(first, t));
          Point3D secondPointT1 = project(sampleTrajectory(first, t + DEFAULT_TIME_DELTA));
          Point3D firstPointT2 = project(sampleTrajectory(second, t));
          Point3D secondPointT2 = project(sampleTrajectory(second, t + DEFAULT_TIME_DELTA));

          LineSegment firstSeg = LineSegment.create(firstPointT1, secondPointT1);
          LineSegment secondSeg = LineSegment.create(firstPointT2, secondPointT2);
          double distance = distance(firstSeg, secondSeg);

          double boundary =
              minimumDistance
                  - DEFAULT_TIME_DELTA * (BasicTrajectory.MAX_ABSOLUTE_VELOCITY)
                  - TestUtils.EPSILON;
          if (distance < boundary) {
            return Optional.of(Collision.create(t, distance, first, second));
          }
          return Optional.absent();
        }
      };

  //implementation from http://geomalgorithms.com/a07-_distance.html#dist3D_Segment_to_Segment()
  private static double distance(LineSegment firstSeg, LineSegment secondSeg) {
    Point3D u = firstSeg.getSlope();
    Point3D v = secondSeg.getSlope();
    Point3D w = minus(firstSeg.getStartPoint(), secondSeg.getStartPoint());
    double a = dot(u, u);
    double b = dot(u, v);
    double c = dot(v, v);
    double d = dot(u, w);
    double e = dot(v, w);
    double D = a * c - b * b;
    double sc, sN, sD = D;
    double tc, tN, tD = D;

    if (D < TestUtils.EPSILON) {
      sN = 0D;
      sD = 1D;
      tN = e;
      tD = c;
    } else {
      sN = (b * e - c * d);
      tN = (a * e - b * d);
      if (sN < 0D) {
        sN = 0D;
        tN = e;
        tD = c;
      } else if (sN > sD) {
        sN = sD;
        tN = e + b;
        tD = c;
      }
    }

    if (tN < 0D) {
      tN = 0D;
      if (-d < 0D) {
        sN = 0D;
      } else if (-d > a) {
        sN = sD;
      } else {
        sN = -d;
        sD = a;
      }
    } else if (tN > tD) {
      tN = tD;
      if ((-d + b) < 0D) {
        sN = 0D;
      } else if ((-d + b > a)) {
        sN = sD;
      } else {
        sN = (-d + b);
        sD = a;
      }
    }
    sc = Math.abs(sN) < TestUtils.EPSILON ? 0.0 : sN / sD;
    tc = Math.abs(tN) < TestUtils.EPSILON ? 0.0 : tN / tD;

    Point3D dP = plus(w, minus(scale(u, sc), scale(v, tc)));
    return dP.norm();
  }

  private final List<FiniteTrajectory4d> trajectories;
  private final double minimumDistance;

  public CollisionDetector(List<FiniteTrajectory4d> trajectories) {
    this(trajectories, applications.trajectory.CollisionDetector.DEFAULT_MINIMUM_DISTANCE);
  }

  public CollisionDetector(List<FiniteTrajectory4d> trajectories, double minimumDistance) {
    this.trajectories = Lists.newArrayList(trajectories);
    this.minimumDistance = minimumDistance;
  }

  public List<Collision> findCollisions() {
    return sampleForCollisions(TWO_SAMPLE_LINE_DISTANCE_BASED_COLLISION);
  }

  private List<Collision> sampleForCollisions(CollisionTester tester) {
    List<Collision> collisions = Lists.newArrayList();
    double finalTimePoint =
        applications.trajectory.CollisionDetector.findLastTimePoint(trajectories);
    for (double t = 0;
        t < finalTimePoint;
        t += applications.trajectory.CollisionDetector.DEFAULT_TIME_DELTA) {
      collisions.addAll(getCollisionsAtTime(t, tester));
    }
    return collisions;
  }

  static double findLastTimePoint(List<FiniteTrajectory4d> trajectories) {
    double maxTime = 0;
    for (FiniteTrajectory4d trajectory : trajectories) {
      if (trajectory.getTrajectoryDuration() > maxTime) {
        maxTime = trajectory.getTrajectoryDuration();
      }
    }
    return maxTime;
  }

  private Collection<Collision> getCollisionsAtTime(double t, CollisionTester tester) {
    List<Collision> collT = Lists.newArrayList();
    for (int i = 0; i < trajectories.size(); i++) {
      for (int j = i + 1; j < trajectories.size(); j++) {
        Optional<Collision> possibleColl =
            tester.isCollision(t, trajectories.get(i), trajectories.get(j), minimumDistance);
        if (possibleColl.isPresent()) {
          collT.add(possibleColl.get());
        }
      }
    }
    return collT;
  }

  @AutoValue
  public abstract static class Collision {
    public static Collision create(
        double time, double distance, Trajectory4d first, Trajectory4d second) {
      return new AutoValue_CollisionDetector_Collision(time, distance, first, second);
    }

    public abstract double getTimePoint();

    public abstract double getActualDistance();

    public abstract Trajectory4d getFirstCollidingTrajectory();

    public abstract Trajectory4d getSecondCollidingTrajectory();
  }

  private interface CollisionTester {
    /**
     * @param t The time point to sample.
     * @param first The first trajectory.
     * @param second The second trajectory.
     * @param minimumDistance The minimum allowed distance between two trajectories.
     * @return true if a collision occurs.
     */
    Optional<Collision> isCollision(
        double t, FiniteTrajectory4d first, FiniteTrajectory4d second, double minimumDistance);
  }
}
