package control.localization;

import com.google.auto.value.AutoValue;

/** @author Hoang Tung Dinh */
@AutoValue
abstract class QuaternionAngle {

  QuaternionAngle() {}

  /**
   * Gets a builder of this class.
   *
   * @return a builder instance
   */
  public static Builder builder() {
    return new AutoValue_QuaternionAngle.Builder();
  }

  abstract double w();

  abstract double x();

  abstract double y();

  abstract double z();

  /** Builds a {@link QuaternionAngle} instance. */
  @AutoValue.Builder
  public abstract static class Builder {
    abstract Builder setW(double value);

    abstract Builder setX(double value);

    abstract Builder setY(double value);

    abstract Builder setZ(double value);

    abstract QuaternionAngle build();
  }
}
