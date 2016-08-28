package services;

import control.dto.BodyFrameVelocity;
import control.dto.Velocity;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import utils.TestUtils;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

/**
 * Tests for {@link MinDiffBodyFrameVelocityFilter}.
 *
 * @author Hoang Tung Dinh
 */
public class MinDiffBodyFrameVelocityFilterTest {
  @Test(expected = IllegalArgumentException.class)
  public void testIllegalMinDifference() {
    MinDiffBodyFrameVelocityFilter.create(mock(Velocity4dService.class), -0.1);
  }

  @Test
  public void testFilter() {
    final Velocity4dService velocity4dService = mock(Velocity4dService.class);
    final Velocity4dService velocityFilter =
        MinDiffBodyFrameVelocityFilter.create(velocity4dService, 0.1);

    final ArgumentCaptor<BodyFrameVelocity> argumentCaptor =
        ArgumentCaptor.forClass(BodyFrameVelocity.class);

    final BodyFrameVelocity v0 =
        Velocity.builder().setLinearX(1).setLinearY(-1).setLinearZ(0).setAngularZ(0.5).build();
    velocityFilter.sendBodyFrameVelocity(v0);
    verify(velocity4dService, times(1)).sendBodyFrameVelocity(argumentCaptor.capture());
    TestUtils.assertVelocityEqual(v0, argumentCaptor.getValue());

    final BodyFrameVelocity v1 =
        Velocity.builder()
            .setLinearX(0.5)
            .setLinearY(-0.5)
            .setLinearZ(0.3)
            .setAngularZ(0.2)
            .build();
    velocityFilter.sendBodyFrameVelocity(v1);
    verify(velocity4dService, times(2)).sendBodyFrameVelocity(argumentCaptor.capture());
    TestUtils.assertVelocityEqual(v1, argumentCaptor.getValue());

    final BodyFrameVelocity v2 =
        Velocity.builder()
            .setLinearX(0.55)
            .setLinearY(-0.45)
            .setLinearZ(0.35)
            .setAngularZ(0.15)
            .build();
    velocityFilter.sendBodyFrameVelocity(v2);
    verify(velocity4dService, times(3)).sendBodyFrameVelocity(argumentCaptor.capture());
    TestUtils.assertVelocityEqual(v1, argumentCaptor.getValue());
  }
}
