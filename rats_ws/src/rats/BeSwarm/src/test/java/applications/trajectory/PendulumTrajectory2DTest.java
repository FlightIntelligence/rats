package applications.trajectory;

import org.junit.Before;
import org.junit.Test;

/** @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be> */
public class PendulumTrajectory2DTest extends Periodic2DTest {

  @Before
  public void setUp() throws Exception {
    highFrequencyCircle =
        PendulumTrajectory2D.builder().setRadius(radius).setFrequency(highFreq).build();
    lowFrequencyCircle =
        PendulumTrajectory2D.builder().setRadius(radius).setFrequency(lowFreq).build();
  }

  @Test(expected = IllegalArgumentException.class)
  public void testConstructorTooHighSpeedRate() {
    PendulumTrajectory2D.builder().setRadius(1).setFrequency(1).build();
  }
}
