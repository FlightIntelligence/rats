package applications.trajectory;

import org.junit.Test;

import static applications.trajectory.TestUtils.EPSILON;
import static applications.trajectory.TestUtils.getVelocityX;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

/** @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be> */
public class StraightLineWithSmoothBrakingTrajectoryTest extends StraightLineTrajectory4DTest {

  private double brakingMark = 0.8d;

  @Override
  protected void createTargets() {
    target =
        Trajectories.newStraightLineWithSmoothBrakingTrajectory(before, after, speed, brakingMark);
    target2 =
        Trajectories.newStraightLineWithSmoothBrakingTrajectory(
            before, afterNotOrigin, speed, brakingMark);
  }

  @Test
  public void testTargetVelocityCutoffPoint() {
    double brakePoint = after.getX() * brakingMark;
    getVelocityX(target, brakePoint - 0.5);
    assertNotEquals(0, getVelocityX(target, brakePoint - 0.5));
    getVelocityX(target, brakePoint + 0.5);
    assertEquals(0, getVelocityX(target, brakePoint + 0.5), EPSILON);
  }
}
