package control;

import org.junit.Test;

import static com.google.common.truth.Truth.assertThat;

/** @author Hoang Tung Dinh */
public class PidController1dTest {

  private static final double DELTA = 0.000001;

  @Test
  public void testCompute() {
    final PidParameters pidParameters =
        PidParameters.builder().setKp(0.05).setKd(1).setKi(0).build();
    final Trajectory1d trajectory1d =
        new Trajectory1d() {
          @Override
          public double getDesiredPosition(double timeInSeconds) {
            return 10;
          }
        };
    final PidController1d pidController1d = PidController1d.create(pidParameters, trajectory1d);

    assertThat(pidController1d.compute(0, 0, 0)).isWithin(DELTA).of(0.5);
    assertThat(pidController1d.compute(0.5, 0.5, 0)).isWithin(DELTA).of(-0.025);
  }
}
