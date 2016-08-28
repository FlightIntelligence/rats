package applications.trajectory.geom.point;

import com.google.auto.value.AutoValue;
import control.dto.Pose;

import static applications.trajectory.geom.point.Point3D.project;

/**
 * Point class for grouping a point in 4d space with angular orientation.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
@AutoValue
public abstract class Point4D {
  Point4D() {}

  /** @return A coordinate instance representing (0,0,0,0). */
  public static Point4D origin() {
    return new AutoValue_Point4D(0, 0, 0, 0);
  }

  /**
   * @param arg the 3d point to use for information about x,y,z coordinates.
   * @param angle the concrete angle as a 4th dimension value.
   * @return A 4D point created from a 3D point and a specified angle.
   */
  public static Point4D from(Point3D arg, double angle) {
    return new AutoValue_Point4D(arg.getX(), arg.getY(), arg.getZ(), angle);
  }

  /**
   * @param arg the 3d point to use for information about x,y,z coordinates.
   * @return A 4D point created from a 3D point and a 0 angle.
   */
  public static Point4D from(Point3D arg) {
    return new AutoValue_Point4D(arg.getX(), arg.getY(), arg.getZ(), 0);
  }

  /**
   * @param pose the pose to use to create the point 4D
   * @return a 4D point created from a pose
   */
  public static Point4D from(Pose pose) {
    return new AutoValue_Point4D(pose.x(), pose.y(), pose.z(), pose.yaw());
  }

  /**
   * Gets the euclidean distance between two points.
   *
   * @param p0 the first point
   * @param p1 the second point
   * @return the euclidean distance between {@code p0} and {@code p1}
   */
  public static double distance(Point4D p0, Point4D p1) {
    return Point3D.distance(project(p0), project(p1));
  }

  /**
   * @param targetPoint the destination of the vector.
   * @return A point instance representing the difference or distance between the given point and
   *     this point.
   */
  public Point4D minus(Point4D targetPoint) {
    return minus(this, targetPoint);
  }

  /**
   * @param targetPoint the destination of the vector.
   * @param sourcePoint the source of the vector.
   * @return A point instance representing the difference or distance between the given points.
   */
  public static Point4D minus(Point4D targetPoint, Point4D sourcePoint) {
    return create(
        targetPoint.getX() - sourcePoint.getX(),
        targetPoint.getY() - sourcePoint.getY(),
        targetPoint.getZ() - sourcePoint.getZ(),
        targetPoint.getAngle() - sourcePoint.getAngle());
  }

  /**
   * Create a new point object
   *
   * @param x the x coordinate.
   * @param y the Y coordinate.
   * @param z the Z coordinate.
   * @param angle the yaw angle.
   * @return A value class representing the given coordinates.
   */
  public static Point4D create(double x, double y, double z, double angle) {
    return new AutoValue_Point4D(x, y, z, angle);
  }

  /** @return The X coordinate. */
  public abstract double getX();

  /** @return The Y coordinate. */
  public abstract double getY();

  /** @return The Z coordinate. */
  public abstract double getZ();

  /** @return The yaw angle orientation. */
  public abstract double getAngle();

  /**
   * @param targetPoint the destination of the vector.
   * @return A point instance representing the sum of the given points.
   */
  public Point4D plus(Point4D targetPoint) {
    return plus(this, targetPoint);
  }

  /**
   * @param targetPoint the destination of the vector.
   * @param sourcePoint the source of the vector.
   * @return A point instance representing the sum of the given points.
   */
  public static Point4D plus(Point4D targetPoint, Point4D sourcePoint) {
    return create(
        targetPoint.getX() + sourcePoint.getX(),
        targetPoint.getY() + sourcePoint.getY(),
        targetPoint.getZ() + sourcePoint.getZ(),
        targetPoint.getAngle() + sourcePoint.getAngle());
  }
}
