package applications.trajectory;

import org.junit.Before;
import org.junit.Test;

/** @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be> */
public class CircleTrajectory2DTest extends Periodic2DTest {

  @Before
  public void setUp() throws Exception {
    highFrequencyCircle =
        CircleTrajectory2D.builder().setFrequency(highFreq).setRadius(radius).build();
    lowFrequencyCircle =
        CircleTrajectory2D.builder().setFrequency(lowFreq).setRadius(radius).build();
  }

  @Test(expected = IllegalArgumentException.class)
  public void testConstructorTooHighSpeedRate() {
    CircleTrajectory2D.builder().setFrequency(1).setRadius(1).build();
  }
}
