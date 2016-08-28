package applications.parrot.tumsim;

import applications.trajectory.Trajectories;
import applications.trajectory.geom.point.Point3D;
import applications.trajectory.geom.point.Point4D;
import choreo.Choreography;
import control.FiniteTrajectory4d;
import control.Trajectory4d;

import static applications.trajectory.Trajectories.newStraightLineTrajectory;

/** @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be> */
public class TumSimulatorComplexExample extends AbstractTumSimulatorExample {

  /** Default Constructor. */
  public TumSimulatorComplexExample() {
    super("TumRunComplexExampleTrajectory");
  }

  @Override
  public FiniteTrajectory4d getConcreteTrajectory() {
    Trajectory4d init = Trajectories.newHoldPositionTrajectory(Point4D.create(0, 0, 1, 0));
    FiniteTrajectory4d first =
        newStraightLineTrajectory(
            Point4D.create(0, 0, 1, 0), Point4D.create(1.5, -3.0, 1.5, 0), 0.1);
    Trajectory4d inter = Trajectories.newHoldPositionTrajectory(Point4D.create(1.5, -3.0, 1.5, 0));
    Trajectory4d second =
        Trajectories.newCircleTrajectory4D(Point3D.create(1.0, -3.0, 1.5), 0.5, 0.05, Math.PI / 8);
    Trajectory4d third = Trajectories.newHoldPositionTrajectory(Point4D.create(1.5, -3.5, 1.5, 0));
    Trajectory4d fourth = Trajectories.newHoldPositionTrajectory(Point4D.create(1.5, -3.5, 1.0, 0));
    return Choreography.builder()
        .withTrajectory(init)
        .forTime(4)
        .withTrajectory(first)
        .forTime(first.getTrajectoryDuration() + 2)
        .withTrajectory(inter)
        .forTime(5)
        .withTrajectory(second)
        .forTime(40)
        .withTrajectory(third)
        .forTime(10)
        .withTrajectory(fourth)
        .forTime(5)
        .build();
  }
}
