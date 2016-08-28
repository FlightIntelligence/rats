package utils.math;

import junitparams.JUnitParamsRunner;
import junitparams.Parameters;
import org.junit.Test;
import org.junit.runner.RunWith;

import static com.google.common.truth.Truth.assertThat;

/**
 * Tests for {@link EulerAngle#computeAngleDistance(double, double)}'s methods.
 *
 * @author Hoang Tung Dinh
 */
@RunWith(JUnitParamsRunner.class)
public class EulerAngleTest {
  private static final double DELTA = 0.01;
  private static final double PI = Math.PI;

  private Object[] angleDistanceValues() {
    return new Object[] {
      new Object[] {PI, -PI, 0},
      new Object[] {1, -1, -2},
      new Object[] {-1, 1, 2},
      new Object[] {-2 * PI / 3, 2 * PI / 3, -2 * PI / 3},
      new Object[] {2 * PI / 3, -2 * PI / 3, 2 * PI / 3},
      new Object[] {PI / 3, -PI / 3, -2 * PI / 3},
      new Object[] {-PI / 3, PI / 3, 2 * PI / 3}
    };
  }

  @Test
  @Parameters(method = "angleDistanceValues")
  public void testComputeAngleDistance(double firstAngle, double secondAngle, double distance) {
    assertThat(EulerAngle.computeAngleDistance(firstAngle, secondAngle))
        .isWithin(DELTA)
        .of(distance);
  }
}
