package applications;

import applications.trajectory.Trajectories;
import applications.trajectory.geom.point.Point3D;
import applications.trajectory.geom.point.Point4D;
import choreo.Choreography;
import control.FiniteTrajectory4d;
import control.Trajectory4d;

/** @author Hoang Tung Dinh */
public final class TrajectoriesForTesting {

  private TrajectoriesForTesting() {}

  /** This trajectory has been tested and worked well. */
  public static FiniteTrajectory4d getSlowCircle() {
    Trajectory4d second =
        Trajectories.circleTrajectoryBuilder()
            .setLocation(Point3D.create(1, -2, 1.5))
            .setRadius(0.5)
            .setFrequency(0.05)
            .fixYawAt(-Math.PI / 2)
            .build();
    return Choreography.builder().withTrajectory(second).forTime(120).build();
  }

  /**
   * This trajectory has been tested and worked well with pid params 0.3 1.0 for x y, 0.3 0.6 for z
   * and 0.3 0.4 for yaw.
   */
  public static FiniteTrajectory4d getFastCircle() {
    Trajectory4d second =
        Trajectories.circleTrajectoryBuilder()
            .setLocation(Point3D.create(1, -2, 1.5))
            .setRadius(0.5)
            .setFrequency(0.2)
            .fixYawAt(-Math.PI / 2)
            .build();
    return Choreography.builder().withTrajectory(second).forTime(120).build();
  }

  public static FiniteTrajectory4d getLargeCircle() {
    Trajectory4d second =
        Trajectories.circleTrajectoryBuilder()
            .setLocation(Point3D.create(1.5, -2.5, 1.5))
            .setRadius(1.5)
            .setFrequency(0.1)
            .fixYawAt(-Math.PI / 2)
            .build();
    return Choreography.builder().withTrajectory(second).forTime(120).build();
  }

  /**
   * This trajectory can work with pid params: 0.5 0.8 for x, 0.2 1.0 for y, 1.0 0.3 for z, 0.3 0.4
   * for yaw
   */
  public static FiniteTrajectory4d getSlowIndoorPendulum() {
    double yawAngle = -Math.PI / 2;
    Trajectory4d target1 =
        Trajectories.swingTrajectoryBuilder()
            .setOrigin(Point4D.create(1.5, -2, 2.0, yawAngle))
            .setFrequency(0.06)
            .setRadius(1.5)
            .build();
    return Choreography.builder().withTrajectory(target1).forTime(120).build();
  }

  public static FiniteTrajectory4d getCorkscrew() {
    double orientation = -Math.PI / 2;
    double radius = 0.5;
    double frequency = 0.2;
    double velocity = 0.1;
    Point4D start = Point4D.create(1, 0, 1, orientation);
    Point3D end = Point3D.create(2.5, -3.0, 1.5);
    Trajectory4d init = Trajectories.newHoldPositionTrajectory(start);
    FiniteTrajectory4d first =
        Trajectories.newCorkscrewTrajectory(start, end, velocity, radius, frequency, 0);
    Trajectory4d inter = Trajectories.newHoldPositionTrajectory(Point4D.from(end, orientation));
    return Choreography.builder()
        .withTrajectory(init)
        .forTime(4)
        .withTrajectory(first)
        .forTime(first.getTrajectoryDuration() + 2)
        .withTrajectory(inter)
        .forTime(5)
        .build();
  }
}
