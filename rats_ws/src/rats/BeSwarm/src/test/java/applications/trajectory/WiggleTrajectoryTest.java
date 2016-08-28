package applications.trajectory;

import applications.trajectory.geom.point.Point4D;
import com.google.common.collect.Lists;
import control.FiniteTrajectory4d;
import org.junit.Before;
import org.junit.Test;

import java.util.List;

/** @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be> */
public class WiggleTrajectoryTest {
  private static final double CENTERX = 10;
  private static final double CENTERY = 10;
  private static final double CENTERZ = 10;

  private double distance;
  private double angle;
  private FiniteTrajectory4d wiggle;

  @Before
  public void setUp() {
    this.distance = 1;
    int times = 3;
    this.angle = Math.PI / 2;
    Point4D p = Point4D.create(CENTERX, CENTERY, CENTERZ, angle);
    this.wiggle = new WiggleTrajectory(p, times, distance);
  }

  @Test
  public void testWiggle() {
    List<Double> samplesX = Lists.newArrayList();
    List<Double> samplesY = Lists.newArrayList();
    List<Double> samplesZ = Lists.newArrayList();
    List<Double> samplesAngle = Lists.newArrayList();

    for (double i = 0.0; i < wiggle.getTrajectoryDuration(); i += 0.1) {
      samplesX.add(wiggle.getDesiredPositionX(i));
      samplesY.add(wiggle.getDesiredPositionY(i));
      samplesZ.add(wiggle.getDesiredPositionZ(i));
      samplesAngle.add(wiggle.getDesiredAngleZ(i));
    }

    TestUtils.assertBounds(samplesX, CENTERX - 0, CENTERX + 0);
    TestUtils.assertBounds(samplesY, CENTERY - distance, CENTERY + distance);
    TestUtils.assertBounds(samplesZ, CENTERZ - 0, CENTERZ + 0);
    TestUtils.assertBounds(samplesAngle, angle, angle);
  }
}
