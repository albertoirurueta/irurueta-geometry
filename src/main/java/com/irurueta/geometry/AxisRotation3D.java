/*
 * Copyright (C) 2012 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.irurueta.geometry;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;

import java.io.Serializable;

/**
 * This class defines the amount of rotation for 3D points or planes.
 * Rotation is defined internally as axis coordinates and rotation angle,
 * following Rodrigues formulas.
 * This class is based in:
 * <a href="http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/code/sfrotation_java.htm">
 *     http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/code/sfrotation_java.htm
 * </a>
 */
public class AxisRotation3D extends Rotation3D implements Serializable {

    /**
     * Number of parameters defining a rotation axis.
     */
    public static final int AXIS_PARAMS = 3;

    /**
     * Constant defining machine precision.
     */
    public static final double EPS = 1e-12;

    /**
     * x element of axis angle.
     */
    private double axisX = 0.0;

    /**
     * y element of axis angle.
     */
    private double axisY = 0.0;

    /**
     * z element of axis angle.
     */
    private double axisZ = 1.0;

    /**
     * angle element of axis angle.
     */
    private double theta = 0.0;

    /**
     * Constructor which allows initial value to be supplied as axis and angle.
     * For better accuracy, axis values should be normalized.
     *
     * @param axisX x dimension of normalized axis.
     * @param axisY y dimension of normalized axis.
     * @param axisZ z dimension of normalized axis.
     * @param theta angle in radians.
     */
    public AxisRotation3D(final double axisX, final double axisY, final double axisZ, final double theta) {
        setAxisAndRotation(axisX, axisY, axisZ, theta);
    }

    /**
     * Constructor where array of axis values and rotation angle are provided
     * For better accuracy, axis values should be normalized.
     *
     * @param axis  Array containing x,y and z values of axis.
     * @param theta rotation angle in radians.
     * @throws IllegalArgumentException if provided axis length is not 3.
     */
    public AxisRotation3D(final double[] axis, final double theta) {
        setAxisAndRotation(axis, theta);
    }

    /**
     * Copy constructor.
     *
     * @param rotation instance to copy.
     */
    public AxisRotation3D(final AxisRotation3D rotation) {
        fromRotation(rotation);
    }

    /**
     * Copy constructor.
     *
     * @param rot Converts and copies provided rotation instance.
     */
    public AxisRotation3D(final Rotation3D rot) {
        try {
            fromMatrix(rot.asInhomogeneousMatrix());
        } catch (final InvalidRotationMatrixException ignore) {
            // never happens
        }
    }

    /**
     * Empty constructor.
     */
    public AxisRotation3D() {
    }

    /**
     * Returns type of this rotation.
     *
     * @return Type of this rotation.
     */
    @Override
    public Rotation3DType getType() {
        return Rotation3DType.AXIS_ROTATION3D;
    }

    /**
     * Sets rotation of this instance by copying provided rotation.
     *
     * @param rot Rotation to be copied.
     */
    @Override
    public final void fromRotation(final AxisRotation3D rot) {
        if (rot != null) {
            axisX = rot.axisX;
            axisY = rot.axisY;
            axisZ = rot.axisZ;
            theta = rot.theta;
        } else {
            axisX = 0.0;
            axisY = 0.0;
            axisZ = 1.0;
            theta = 0.0;
        }
    }

    /**
     * Sets rotation axis of this instance while preserving the rotation angle.
     * Once set, points will rotate around provided axis.
     *
     * @param axisX X coordinate of rotation axis.
     * @param axisY Y coordinate of rotation axis.
     * @param axisZ Z coordinate of rotation axis.
     */
    public void setAxis(final double axisX, final double axisY, final double axisZ) {
        theta = Math.sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);
        if (theta == 0.0) {
            this.axisX = 1;
            this.axisY = this.axisZ = 0.0;
            return;
        }
        this.axisX = axisX / theta;
        this.axisY = axisY / theta;
        this.axisZ = axisZ / theta;
    }

    /**
     * Sets the axis and rotation of this instance.
     * Once set, points will rotate around provided axis an amount equal to
     * provided rotation angle in radians.
     * Note: to avoid numerical instabilities and improve accuracy, axis
     * coordinates should be normalized (e.g. norm equal to 1).
     *
     * @param axisX X coordinate of rotation axis.
     * @param axisY Y coordinate of rotation axis.
     * @param axisZ Z coordinate of rotation axis.
     * @param theta Amount of rotation in radians.
     */
    @Override
    public final void setAxisAndRotation(
            final double axisX, final double axisY, final double axisZ, final double theta) {
        this.axisX = axisX;
        this.axisY = axisY;
        this.axisZ = axisZ;
        this.theta = theta;
    }

    /**
     * Returns X coordinate of rotation axis.
     *
     * @return X coordinate of rotation axis.
     */
    public double getAxisX() {
        return axisX;
    }

    /**
     * Returns Y coordinate of rotation axis.
     *
     * @return Y coordinate of rotation axis.
     */
    public double getAxisY() {
        return axisY;
    }

    /**
     * Returns Z coordinate of rotation axis.
     *
     * @return Z coordinate of rotation axis.
     */
    public double getAxisZ() {
        return axisZ;
    }

    /**
     * Returns rotation axis corresponding to this instance.
     * Result is stored in provided axis array, which must have length 3
     *
     * @param axis Array where axis coordinates will be stored.
     * @throws IllegalArgumentException Raised if provided array does not have
     *                                  length 3.
     */
    @Override
    public void rotationAxis(final double[] axis) {
        if (axis.length != INHOM_COORDS) {
            throw new IllegalArgumentException();
        }

        axis[0] = axisX;
        axis[1] = axisY;
        axis[2] = axisZ;
    }

    /**
     * Returns rotation amount or angle in radians around the rotation axis
     * associated to this instance.
     *
     * @return Rotation angle in radians.
     */
    @Override
    public double getRotationAngle() {
        return theta;
    }

    /**
     * Returns a 3D rotation which is inverse to this instance.
     * In other words, the combination of this rotation with its inverse
     * produces no change.
     *
     * @return Inverse 3D rotation.
     */
    @Override
    public AxisRotation3D inverseRotationAndReturnNew() {
        final var rot = new AxisRotation3D();
        inverseRotation(rot);
        return rot;
    }

    /**
     * Sets into provided MatrixRotation3D instance a rotation inverse to this
     * instance.
     * The combination of this rotation with its inverse produces no change.
     *
     * @param result Instance where inverse rotation will be set.
     */
    public void inverseRotation(final AxisRotation3D result) {
        // copy this rotation into result
        result.fromRotation(this);
        // reverse angle
        result.theta = -theta;
    }

    /**
     * Sets into provided MatrixRotation3D instance a rotation inverse to this
     * instance.
     * The combination of this rotation with its inverse produces no change.
     *
     * @param result Instance where inverse rotation will be set.
     */
    @Override
    public void inverseRotation(final Rotation3D result) {
        if (result instanceof AxisRotation3D axixResult) {
            inverseRotation(axixResult);

        } else if (result instanceof MatrixRotation3D) {
            final var rot = new AxisRotation3D();
            inverseRotation(rot);
            try {
                result.fromMatrix(rot.asInhomogeneousMatrix());
            } catch (final InvalidRotationMatrixException ignore) {
                // never happens
            }
        }
    }

    /**
     * Reverses the rotation of this instance.
     */
    @Override
    public void inverseRotation() {
        inverseRotation(this);
    }

    /**
     * Returns this 3D rotation instance expressed as a 3x3 inhomogeneous
     * matrix.
     * This is equivalent to call getInternalMatrix().
     *
     * @return Rotation matrix expressed in inhomogeneous coordinates.
     */
    @Override
    public Matrix asInhomogeneousMatrix() {
        Matrix result = null;
        try {
            result = new Matrix(INHOM_COORDS, INHOM_COORDS);
            asInhomogeneousMatrix(result);
        } catch (final WrongSizeException ignore) {
            // never happens
        }

        return result;
    }

    /**
     * Sets into provided Matrix instance this 3D rotation expressed as a
     * 3x3 inhomogeneous matrix.
     *
     * @param result Matrix where rotation will be set.
     * @throws IllegalArgumentException Raised if provided instance does not
     *                                  have size 3x3.
     */
    @Override
    public void asInhomogeneousMatrix(final Matrix result) {
        if (result.getRows() != INHOM_COORDS || result.getColumns() != INHOM_COORDS) {
            throw new IllegalArgumentException();
        }

        final var c = Math.cos(theta);
        final var s = Math.sin(theta);
        final var t = 1.0 - c;
        // normalize axis
        final var magnitude = Math.sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);
        if (magnitude > EPS) {
            // normalize only if axis norm is large enough to avoid numerical
            // instability
            axisX /= magnitude;
            axisY /= magnitude;
            axisZ /= magnitude;
        }

        result.setElementAt(0, 0, c + axisX * axisX * t);
        result.setElementAt(1, 1, c + axisY * axisY * t);
        result.setElementAt(2, 2, c + axisZ * axisZ * t);


        var tmp1 = axisX * axisY * t;
        var tmp2 = axisZ * s;
        result.setElementAt(1, 0, tmp1 + tmp2);
        result.setElementAt(0, 1, tmp1 - tmp2);
        tmp1 = axisX * axisZ * t;
        tmp2 = axisY * s;
        result.setElementAt(2, 0, tmp1 - tmp2);
        result.setElementAt(0, 2, tmp1 + tmp2);
        tmp1 = axisY * axisZ * t;
        tmp2 = axisX * s;
        result.setElementAt(2, 1, tmp1 + tmp2);
        result.setElementAt(1, 2, tmp1 - tmp2);
    }

    /**
     * Returns this 3D rotation instance expressed as a 4x4 homogeneous matrix.
     *
     * @return Rotation matrix expressed in homogeneous coordinates.
     */
    @Override
    public Matrix asHomogeneousMatrix() {
        Matrix result = null;
        try {
            result = Matrix.identity(HOM_COORDS, HOM_COORDS);
            // sets into 3x3 top-left sub-matrix the internal matrix of this
            // instance, the remaining part will continue to be the identity
            result.setSubmatrix(0, 0, INHOM_COORDS - 1,
                    INHOM_COORDS - 1, asInhomogeneousMatrix());
        } catch (final WrongSizeException ignore) {
            // never happens
        }
        return result;
    }

    /**
     * Sets into provided Matrix instance this 3D rotation expressed as a
     * 4x4 homogeneous matrix.
     *
     * @param result Matrix where rotation will be set.
     * @throws IllegalArgumentException Raised if provided instance does not
     *                                  have size 4x4.
     */
    @Override
    public void asHomogeneousMatrix(final Matrix result) {
        if (result.getRows() != HOM_COORDS || result.getColumns() != HOM_COORDS) {
            throw new IllegalArgumentException();
        }

        result.initialize(0.0);
        // sets into 3x3 top-left sub-matrix the internal matrix of this instance
        result.setSubmatrix(0, 0, INHOM_COORDS - 1,
                INHOM_COORDS - 1, asInhomogeneousMatrix());
        // set las element to 1.0 (to be like the identity
        result.setElementAt(HOM_COORDS - 1, HOM_COORDS - 1, 1.0);
    }

    /**
     * Sets amount of rotation from provided inhomogeneous rotation matrix.
     * Provided matrix must be orthogonal (i.e. squared, non-singular, it's
     * transpose must be its inverse) and must have determinant equal to 1.
     * Provided matrix must also have size 3x3.
     *
     * @param m         Provided rotation matrix.
     * @param threshold Threshold to determine whether matrix is orthonormal.
     * @throws InvalidRotationMatrixException Raised if provided matrix is not
     *                                        valid (has wrong size, or it is not orthonormal).
     * @throws IllegalArgumentException       Raised if provided threshold is negative
     *                                        {@link #isValidRotationMatrix(Matrix)}
     */
    @Override
    public void fromInhomogeneousMatrix(final Matrix m, final double threshold) throws InvalidRotationMatrixException {

        if (m.getRows() != INHOM_COORDS || m.getColumns() != INHOM_COORDS) {
            throw new InvalidRotationMatrixException();
        }
        if (!Rotation3D.isValidRotationMatrix(m, threshold)) {
            throw new InvalidRotationMatrixException();
        }

        double angle;
        double x;
        double y;
        // variables for result
        double z;
        // margin to allow for rounding errors
        var epsilon = 0.01;
        // margin to distinguish between 0 and 180 degrees
        var epsilon2 = 0.1;

        if ((Math.abs(m.getElementAt(0, 1) - m.getElementAt(1, 0)) < epsilon) &&
                (Math.abs(m.getElementAt(0, 2) - m.getElementAt(2, 0)) < epsilon) &&
                (Math.abs(m.getElementAt(1, 2) - m.getElementAt(2, 1)) < epsilon)) {
            // singularity found
            // first check for identity matrix which must have +1 for all terms
            // in leading diagonal and zero in other terms
            if ((Math.abs(m.getElementAt(0, 1) + m.getElementAt(1, 0)) < epsilon2)
                    && (Math.abs(m.getElementAt(0, 2) + m.getElementAt(2, 0)) < epsilon2)
                    && (Math.abs(m.getElementAt(1, 2) + m.getElementAt(2, 1)) < epsilon2)
                    && (Math.abs(m.getElementAt(0, 0) + m.getElementAt(1, 1)
                    + m.getElementAt(2, 2) - 3.0) < epsilon2)) {
                // this singularity is identity matrix so angle = 0
                setAxisAndRotation(0.0, 0.0, 1.0, 0.0);  // zero angle,
                // arbitrary axis
                return;
            }
            // otherwise this singularity is angle = 180
            angle = Math.PI;
            final var xx = (m.getElementAt(0, 0) + 1.0) / 2.0;
            final var yy = (m.getElementAt(1, 1) + 1.0) / 2.0;
            final var zz = (m.getElementAt(2, 2) + 1.0) / 2.0;
            final var xy = (m.getElementAt(0, 1) + m.getElementAt(1, 0)) / 4.0;
            final var xz = (m.getElementAt(0, 2) + m.getElementAt(2, 0)) / 4.0;
            final var yz = (m.getElementAt(1, 2) + m.getElementAt(2, 1)) / 4.0;
            if ((xx > yy) && (xx > zz)) {
                // m.getElementAt(0, 0) is the
                // largest diagonal term
                if (xx < epsilon) {
                    x = 0.0;
                    y = Math.sqrt(2.0) / 2.0;
                    z = Math.sqrt(2.0) / 2.0;
                } else {
                    x = Math.sqrt(xx);
                    y = xy / x;
                    z = xz / x;
                }
            } else if (yy > zz) {
                // m.getElementAt(1, 1) is the largest
                // diagonal term
                if (yy < epsilon) {
                    x = Math.sqrt(2.0) / 2.0;
                    y = 0.0;
                    z = Math.sqrt(2.0) / 2.0;
                } else {
                    y = Math.sqrt(yy);
                    x = xy / y;
                    z = yz / y;
                }
            } else {
                // m.getElementAt(2, 2) is the largest diagonal term so
                // base result on this
                if (zz < epsilon) {
                    x = Math.sqrt(2.0) / 2.0;
                    y = Math.sqrt(2.0) / 2.0;
                    z = 0.0;
                } else {
                    z = Math.sqrt(zz);
                    x = xz / z;
                    y = yz / z;
                }
            }
            setAxisAndRotation(x, y, z, angle);
            // return 180 deg rotation
            return;
        }

        // as we have reached here there are no singularities, so we can handle
        // normally
        var s = Math.sqrt((m.getElementAt(2, 1) - m.getElementAt(1, 2))
                * (m.getElementAt(2, 1) - m.getElementAt(1, 2))
                + (m.getElementAt(0, 2) - m.getElementAt(2, 0))
                * (m.getElementAt(0, 2) - m.getElementAt(2, 0))
                + (m.getElementAt(1, 0) - m.getElementAt(0, 1))
                * (m.getElementAt(1, 0) - m.getElementAt(0, 1))); // used to
        // normalise
        if (Math.abs(s) < 0.001) {
            s = 1.0;
        }
        // prevent divide by zero, should not happen if matrix is orthogonal and
        // should be caught by singularity test above, but I've left it in just
        // in case
        theta = Math.acos((m.getElementAt(0, 0) + m.getElementAt(1, 1)
                + m.getElementAt(2, 2) - 1.0) / 2.0);
        axisX = (m.getElementAt(2, 1) - m.getElementAt(1, 2)) / s;
        axisY = (m.getElementAt(0, 2) - m.getElementAt(2, 0)) / s;
        axisZ = (m.getElementAt(1, 0) - m.getElementAt(0, 1)) / s;
    }

    /**
     * Sets amount of rotation from provided homogeneous rotation matrix.
     * Provided matrix must be orthogonal (i.e. squared, non-singular, it's
     * transpose must be its inverse) and must have determinant equal to 1.
     * Provided matrix must also have size 4x4, and its last row and column must
     * be zero, except for element in last row and column which must be 1.
     *
     * @param m         Provided rotation matrix.
     * @param threshold Threshold to determine whether matrix is orthonormal.
     * @throws InvalidRotationMatrixException Raised if provided matrix is not
     *                                        valid (has wrong size, or it is not orthonormal).
     * @throws IllegalArgumentException       Raised if provided threshold is
     *                                        negative.
     *                                        {@link #isValidRotationMatrix(Matrix)}.
     */
    @SuppressWarnings("DuplicatedCode")
    @Override
    public void fromHomogeneousMatrix(final Matrix m, final double threshold) throws InvalidRotationMatrixException {
        if (m.getRows() != HOM_COORDS || m.getColumns() != HOM_COORDS) {
            throw new InvalidRotationMatrixException();
        }
        if (!Rotation3D.isValidRotationMatrix(m, threshold)) {
            throw new InvalidRotationMatrixException();
        }
        if (Math.abs(m.getElementAt(3, 0)) > threshold
                || Math.abs(m.getElementAt(3, 1)) > threshold
                || Math.abs(m.getElementAt(3, 2)) > threshold
                || Math.abs(m.getElementAt(0, 3)) > threshold
                || Math.abs(m.getElementAt(1, 3)) > threshold
                || Math.abs(m.getElementAt(2, 3)) > threshold
                || Math.abs(m.getElementAt(3, 3) - 1.0) > threshold) {
            throw new InvalidRotationMatrixException();
        }

        fromInhomogeneousMatrix(m.getSubmatrix(0, 0, INHOM_COORDS - 1,
                INHOM_COORDS - 1), threshold);
    }

    /**
     * Rotates a 3D point using the origin of coordinates as the axis of
     * rotation.
     * Point will be rotated by the amount of rotation contained in this
     * instance.
     *
     * @param inputPoint  Input point to be rotated.
     * @param resultPoint Rotated point.
     */
    @Override
    public void rotate(final Point3D inputPoint, final Point3D resultPoint) {
        final var s = Math.sin(theta / 2.0);
        final var xh = axisX * s;
        final var yh = axisY * s;
        final var zh = axisZ * s;
        final var wh = Math.cos(theta / 2.0);

        final var inhomX = inputPoint.getInhomX();
        final var inhomY = inputPoint.getInhomY();
        final var inhomZ = inputPoint.getInhomZ();

        final var resultX = wh * wh * inhomX + 2.0 * yh * wh * inhomZ - 2.0 * zh * wh * inhomY + xh * xh * inhomX
                + 2.0 * yh * xh * inhomY + 2 * zh * xh * inhomZ - zh * zh * inhomX - yh * yh * inhomX;
        final var resultY = 2.0 * xh * yh * inhomX + yh * yh * inhomY + 2.0 * zh * yh * inhomZ
                + 2.0 * wh * zh * inhomX - zh * zh * inhomY + wh * wh * inhomY - 2 * xh * wh * inhomZ
                - xh * xh * inhomY;
        final var resultZ = 2.0 * xh * zh * inhomX + 2.0 * yh * zh * inhomY + zh * zh * inhomZ
                - 2.0 * wh * yh * inhomX - yh * yh * inhomZ + 2.0 * wh * xh * inhomY - xh * xh * inhomZ
                + wh * wh * inhomZ;
        resultPoint.setInhomogeneousCoordinates(resultX, resultY, resultZ);
    }

    /**
     * Combines provided rotation with this rotation and returns the result as
     * a new MatrixRotation3D instance.
     *
     * @param rotation Input rotation to be combined.
     * @return Combined rotation, which is equal to the multiplication of the
     * internal matrix of provided rotation with the internal matrix of this
     * instance.
     */
    public AxisRotation3D combineAndReturnNew(final AxisRotation3D rotation) {
        final var result = new AxisRotation3D();
        combine(this, rotation, result);
        return result;
    }

    /**
     * Combines provided rotation with this rotation and returns the result as
     * a new MatrixRotation3D instance.
     *
     * @param rotation Input rotation to be combined.
     * @return Combined rotation, which is equal to the multiplication of the
     * internal matrix of provided rotation with the internal matrix of this
     * instance.
     */
    @Override
    public Rotation3D combineAndReturnNew(final Rotation3D rotation) {
        if (rotation instanceof AxisRotation3D axisRotation) {
            return combineAndReturnNew(axisRotation);
        } else {
            return combineAndReturnNew(new AxisRotation3D(rotation));
        }
    }

    /**
     * Combines provided rotation into this rotation resulting in the
     * multiplication of the internal matrices of both rotations.
     *
     * @param rotation Input rotation to be combined.
     */
    public void combine(final AxisRotation3D rotation) {
        combine(this, rotation, this);
    }

    /**
     * Combines provided rotation into this rotation resulting in the
     * multiplication of the internal matrices of both rotations.
     *
     * @param rotation Input rotation to be combined.
     */
    @Override
    public void combine(final Rotation3D rotation) {
        if (rotation instanceof AxisRotation3D axisRotation) {
            combine(axisRotation);
        } else {
            combine(new AxisRotation3D(rotation));
        }
    }

    /**
     * Combines the rotation of instances rot1 and rot1 into provided result
     * instance.
     *
     * @param rot1   1st input rotation.
     * @param rot2   2nd input rotation.
     * @param result Combined rotation, which is equal to the multiplication of
     *               the internal matrix of provided rotation with the internal matrix of this
     *               instance.
     */
    public static void combine(final AxisRotation3D rot1, final AxisRotation3D rot2, final AxisRotation3D result) {

        final var m1 = rot1.asInhomogeneousMatrix();
        final var m2 = rot2.asInhomogeneousMatrix();
        try {
            m1.multiply(m2);
            result.fromMatrix(m1);
        } catch (final InvalidRotationMatrixException | WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Sets values of this rotation from a quaternion.
     *
     * @param q a quaternion to set values from.
     */
    @Override
    public void fromRotation(final Quaternion q) {
        q.toAxisRotation(this);
    }

    /**
     * Converts this 3D rotation into a matrix rotation storing the result
     * into provided instance.
     *
     * @param result instance where result wil be stored.
     */
    @Override
    public void toMatrixRotation(final MatrixRotation3D result) {
        result.setAxisAndRotation(new double[]{axisX, axisY, axisZ}, theta);
    }

    /**
     * Converts this 3D rotation into an axis rotation storing the result into
     * provided instance.
     *
     * @param result instance where result will be stored.
     */
    @Override
    public void toAxisRotation(final AxisRotation3D result) {
        result.fromRotation(this);
    }

    /**
     * Converts this 3D rotation into a quaternion storing the result into
     * provided instance.
     *
     * @param result instance where result will be stored.
     */
    @Override
    public void toQuaternion(final Quaternion result) {
        result.setFromAxisAndRotation(axisX, axisY, axisZ, theta);
    }
}
