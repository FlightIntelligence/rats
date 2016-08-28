package applications.trajectory;

import applications.trajectory.geom.point.Point4D;
import com.google.common.collect.Lists;
import control.FiniteTrajectory4d;
import org.junit.Before;
import org.junit.Test;

import java.util.List;

import static org.junit.Assert.assertEquals;

/** @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be> */
public class ZDropTrajectoryTest {

  private Point4D before;
  private Point4D after;
  private FiniteTrajectory4d target;
  private double negDrop = 2;
  private double freq = 3;
  private double speed = 1;
  private double bounddelta = 0.23;

  @Before
  public void setUp() {
    before = Point4D.create(0, 0, 0, 0);
    after = Point4D.create(10, 0, 0, Math.PI / 2);
    target = Trajectories.newZDropLineTrajectory(before, after, speed, freq, negDrop);
    init();
  }

  private void init() {
    target.getDesiredPositionX(0);
    target.getDesiredPositionY(0);
    target.getDesiredPositionZ(0);
    target.getDesiredAngleZ(0);
  }

  @Test
  public void testDropRateAfter() {
    before = Point4D.create(0, 0, 10, 0);
    after = Point4D.create(10, 0, 10, Math.PI / 2);
    target = Trajectories.newZDropLineTrajectory(before, after, speed, freq, negDrop);
    init();
    cycle2(freq);
    target.getDesiredPositionZ(10);
    testParamDropRate(target.getTrajectoryDuration() + 1, target.getTrajectoryDuration() + 20, 0);
  }

  private void cycle2(double n) {
    double t = target.getTrajectoryDuration();
    for (int i = 1; i <= 2 * n; i++) {
      target.getDesiredPositionZ(t);
    }
  }

  private void testParamDropRate(double start, double duration, double expected) {
    List<Double> zresults = Lists.newArrayList();
    for (double i = start; i < duration; i += 0.1d) {
      zresults.add(target.getDesiredPositionZ(i));
    }
    int count = countOccurrence(zresults, after.getZ() - negDrop + bounddelta);
    assertEquals(expected, count, 0);
  }

  private int countOccurrence(List<Double> zresults, double value) {
    int count = 0;
    boolean bound = false;
    boolean counted = false;
    for (double d : zresults) {
      if (d <= value) {
        if (!counted) {
          count += 1;
          counted = true;
        }
      } else {
        counted = false;
      }
    }
    return count;
  }

  @Test
  public void testDropRateHighZ() {
    before = Point4D.create(0, 0, 10, 0);
    after = Point4D.create(10, 0, 10, Math.PI / 2);
    target = Trajectories.newZDropLineTrajectory(before, after, speed, freq, negDrop);
    init();
    testParamDropRate(0, target.getTrajectoryDuration(), freq);
  }

  @Test
  public void testComplexCase() {
    before = Point4D.create(0, 0, 10, 0);
    after = Point4D.create(0, 15, 10, 0);
    speed = 0.5d;
    freq = 4;
    negDrop = 2;
    target = Trajectories.newZDropLineTrajectory(before, after, speed, freq, negDrop);
    init();
    testParamDropRate(0, target.getTrajectoryDuration() * 2, freq);
  }
}
