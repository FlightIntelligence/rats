package services;

import control.dto.BodyFrameVelocity;
import control.dto.InertialFrameVelocity;
import control.dto.Pose;
import control.dto.Velocity;

import javax.annotation.Nullable;

/**
 * An abstract decorator filter for the {@link BodyFrameVelocity} of a {@link Velocity4dService}.
 *
 * @author Hoang Tung Dinh
 */
abstract class AbstractBodyFrameVelocityFilter implements Velocity4dService {

  private final Velocity4dService velocity4dService;
  @Nullable private BodyFrameVelocity lastBodyFrameVelocity;

  AbstractBodyFrameVelocityFilter(Velocity4dService velocity4dService) {
    this.velocity4dService = velocity4dService;
  }

  @Override
  public void sendInertialFrameVelocity(InertialFrameVelocity inertialFrameVelocity, Pose pose) {
    velocity4dService.sendInertialFrameVelocity(inertialFrameVelocity, pose);
  }

  @Override
  public void sendBodyFrameVelocity(BodyFrameVelocity bodyFrameVelocity) {
    if (lastBodyFrameVelocity == null) {
      lastBodyFrameVelocity = bodyFrameVelocity;
      velocity4dService.sendBodyFrameVelocity(bodyFrameVelocity);
    } else {
      final BodyFrameVelocity filteredVelocity = filter(lastBodyFrameVelocity, bodyFrameVelocity);
      velocity4dService.sendBodyFrameVelocity(filteredVelocity);
      lastBodyFrameVelocity = filteredVelocity;
    }
  }

  private BodyFrameVelocity filter(
      BodyFrameVelocity lastVelocity, BodyFrameVelocity currentVelocity) {
    return Velocity.builder()
        .setLinearX(filter(lastVelocity.linearX(), currentVelocity.linearX()))
        .setLinearY(filter(lastVelocity.linearY(), currentVelocity.linearY()))
        .setLinearZ(filter(lastVelocity.linearZ(), currentVelocity.linearZ()))
        .setAngularZ(filter(lastVelocity.angularZ(), currentVelocity.angularZ()))
        .build();
  }

  abstract double filter(double lastVelocity, double currentVelocity);
}
