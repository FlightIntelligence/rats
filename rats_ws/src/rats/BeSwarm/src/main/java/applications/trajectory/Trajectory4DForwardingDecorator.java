package applications.trajectory;

import control.Trajectory4d;

/**
 * Forwarding decorator for trajectory4D instances with inner-trajectory4D hooks.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
public abstract class Trajectory4DForwardingDecorator implements Trajectory4d {
  private final Trajectory4d target;

  /**
   * Public constructor
   *
   * @param target The target trajectory to wrap.
   */
  public Trajectory4DForwardingDecorator(Trajectory4d target) {
    this.target = target;
  }

  @Override
  public double getDesiredPositionX(double timeInSeconds) {
    positionDelegate(timeInSeconds);
    return target.getDesiredPositionX(timeInSeconds);
  }

  /**
   * This delegate method will be called before every call to getDesiredPosition()
   *
   * @param timeInSeconds the time argument.
   */
  protected abstract void positionDelegate(double timeInSeconds);

  @Override
  public double getDesiredPositionY(double timeInSeconds) {
    positionDelegate(timeInSeconds);
    return target.getDesiredPositionY(timeInSeconds);
  }

  @Override
  public double getDesiredPositionZ(double timeInSeconds) {
    positionDelegate(timeInSeconds);
    return target.getDesiredPositionZ(timeInSeconds);
  }

  @Override
  public double getDesiredAngleZ(double timeInSeconds) {
    positionDelegate(timeInSeconds);
    return target.getDesiredAngleZ(timeInSeconds);
  }
}
