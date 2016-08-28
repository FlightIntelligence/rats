package applications.trajectory;

import control.Trajectory1d;
import org.junit.Test;

import static applications.trajectory.TestUtils.verifyPositionFrequencyRadiusRelation;
import static applications.trajectory.TestUtils.testSpeedBounds;
import static applications.trajectory.TestUtils.verifyVelocityFrequencyRadiusRelation;

/** @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be> */
public abstract class Periodic1DTest {
  protected final double lowFreq = 1 / 10;
  protected final double highFreq = 1.5;
  protected final double radius = 0.065;
  protected final double phase = 0;
  protected Trajectory1d highFrequencyCircle;
  protected Trajectory1d lowFrequencyCircle;

  @Test
  public void getTrajectoryPositionTestFrequencyAndRadiusRelation() throws Exception {
    verifyPositionFrequencyRadiusRelation(highFreq, radius, highFrequencyCircle);
    verifyPositionFrequencyRadiusRelation(lowFreq, radius, lowFrequencyCircle);
  }

  @Test
  public void getTrajectoryVelocityTestFrequencyAndRadiusRelation() {
    testSpeedBounds(highFrequencyCircle, BasicTrajectory.MAX_ABSOLUTE_VELOCITY);
    verifyVelocityFrequencyRadiusRelation(highFreq, highFrequencyCircle);

    testSpeedBounds(lowFrequencyCircle, BasicTrajectory.MAX_ABSOLUTE_VELOCITY);
    verifyVelocityFrequencyRadiusRelation(lowFreq, lowFrequencyCircle);
  }
}
