package applications.trajectory;

import control.Trajectory1d;

/**
 * Forwarding decorator for trajectory1D instances with inner-trajectory1D hooks.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
public abstract class Trajectory1DForwardingDecorator implements Trajectory1d {
  private final Trajectory1d target;

  /**
   * Public constructor
   *
   * @param target The target trajectory to wrap.
   */
  public Trajectory1DForwardingDecorator(Trajectory1d target) {
    this.target = target;
  }

  @Override
  public double getDesiredPosition(double timeInSeconds) {
    positionDelegate(timeInSeconds);
    return target.getDesiredPosition(timeInSeconds);
  }

  /**
   * This delegate method will be called before every call to getDesiredPosition()
   *
   * @param timeInSeconds the time argument.
   */
  protected abstract void positionDelegate(double timeInSeconds);
}
