package utils.math;

import org.ejml.simple.SimpleMatrix;

/** @author Hoang Tung Dinh */
public enum RotationOrder {
  XYZ {
    @Override
    SimpleMatrix get3dRotationMatrix(
        SimpleMatrix rotationMatrixX, SimpleMatrix rotationMatrixY, SimpleMatrix rotationMatrixZ) {
      return rotationMatrixZ.mult(rotationMatrixY).mult(rotationMatrixX);
    }
  },

  ZYX {
    @Override
    SimpleMatrix get3dRotationMatrix(
        SimpleMatrix rotationMatrixX, SimpleMatrix rotationMatrixY, SimpleMatrix rotationMatrixZ) {
      return rotationMatrixX.mult(rotationMatrixY).mult(rotationMatrixZ);
    }
  };

  abstract SimpleMatrix get3dRotationMatrix(
      SimpleMatrix rotationMatrixX, SimpleMatrix rotationMatrixY, SimpleMatrix rotationMatrixZ);

  RotationOrder getInverseOrder() {
    switch (this) {
      case XYZ:
        return ZYX;
      case ZYX:
        return XYZ;
      default:
        throw new IllegalStateException("The inverse is not defined for the input order.");
    }
  }
}
