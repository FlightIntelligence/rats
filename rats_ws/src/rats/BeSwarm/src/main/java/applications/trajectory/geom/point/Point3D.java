package applications.trajectory.geom.point;

import com.google.auto.value.AutoValue;

/**
 * Point class for grouping a point in 4d space with angular orientation.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
@AutoValue
public abstract class Point3D {
  Point3D() {}

  /** @return A coordinate instance representing (0,0,0,0). */
  public static Point3D origin() {
    return new AutoValue_Point3D(0, 0, 0);
  }

  /**
   * @param arg The point4D to project onto the primary 3 dimensions.
   * @return
   */
  public static Point3D project(Point4D arg) {
    return new AutoValue_Point3D(arg.getX(), arg.getY(), arg.getZ());
  }

  /**
   * Gets the euclidean distance between two points.
   *
   * @param p0 the first point
   * @param p1 the second point
   * @return the euclidean distance between {@code p0} and {@code p1}
   */
  public static double distance(Point3D p0, Point3D p1) {
    return StrictMath.sqrt(
        (p0.getX() - p1.getX()) * (p0.getX() - p1.getX())
            + (p0.getY() - p1.getY()) * (p0.getY() - p1.getY())
            + (p0.getZ() - p1.getZ()) * (p0.getZ() - p1.getZ()));
  }

  /** @return The X coordinate. */
  public abstract double getX();

  /** @return The Y coordinate. */
  public abstract double getY();

  /** @return The Z coordinate. */
  public abstract double getZ();

  /**
   * @param targetPoint the destination of the vector.
   * @param sourcePoint the source of the vector.
   * @return A point instance representing the difference or distance between the given points.
   */
  public static Point3D minus(Point3D targetPoint, Point3D sourcePoint) {
    return create(
        targetPoint.getX() - sourcePoint.getX(),
        targetPoint.getY() - sourcePoint.getY(),
        targetPoint.getZ() - sourcePoint.getZ());
  }

  /**
   * Create a new point object
   *
   * @param x the x coordinate.
   * @param y the Y coordinate.
   * @param z the Z coordinate.
   * @return A value class representing the given coordinates.
   */
  public static Point3D create(double x, double y, double z) {
    return new AutoValue_Point3D(x, y, z);
  }

  /**
   * @param targetPoint the destination of the vector.
   * @param sourcePoint the source of the vector.
   * @return A point instance representing the sum of the given points.
   */
  public static Point3D plus(Point3D targetPoint, Point3D sourcePoint) {
    return create(
        targetPoint.getX() + sourcePoint.getX(),
        targetPoint.getY() + sourcePoint.getY(),
        targetPoint.getZ() + sourcePoint.getZ());
  }

  /**
   * @param targetPoint the target point.
   * @param scale the scale factor.
   * @return A Point3D instance representing the scalar multiplication of the given point with the
   *     given scale factor.
   */
  public static Point3D scale(Point3D targetPoint, double scale) {
    return create(
        targetPoint.getX() * scale, targetPoint.getY() * scale, targetPoint.getZ() * scale);
  }

  /** @return The norm of this point p as |p| = sqrt(x*x + y*y + z*z). */
  public double norm() {
    return Math.sqrt(dot(this, this));
  }

  /**
   * @param targetPoint the first point.
   * @param sourcePoint the second point .
   * @return A double representing the dot product of the given points.
   */
  public static double dot(Point3D targetPoint, Point3D sourcePoint) {
    return targetPoint.getX() * sourcePoint.getX()
        + targetPoint.getY() * sourcePoint.getY()
        + targetPoint.getZ() * sourcePoint.getZ();
  }
}
