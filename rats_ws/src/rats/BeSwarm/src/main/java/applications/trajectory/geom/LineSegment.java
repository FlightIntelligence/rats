package applications.trajectory.geom;

import applications.trajectory.geom.point.Point3D;
import com.google.auto.value.AutoValue;

/**
 * Parametric line segment (l) representing a straight line segment between two points in 3D space.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
@AutoValue
public abstract class LineSegment {
  LineSegment() {}

  /** @return The starting point (p1) of the segment l. */
  public abstract Point3D getStartPoint();

  /** @return The end point (p2) of the segment l. */
  public abstract Point3D getEndPoint();

  /**
   * @return The slope s of the segment between two points (p1, p2) where the line through those
   *     points can be written as l(t) = p1 + s*t and for t = 0: l(t) = p1, t = 1: l(t) = p2.
   */
  public Point3D getSlope() {
    return Point3D.minus(getEndPoint(), getStartPoint());
  }

  /**
   * Factory method.
   *
   * @param start The start point.
   * @param end The end point.
   * @return a value object representing a line segment.
   */
  public static LineSegment create(Point3D start, Point3D end) {
    return new AutoValue_LineSegment(start, end);
  }
}
