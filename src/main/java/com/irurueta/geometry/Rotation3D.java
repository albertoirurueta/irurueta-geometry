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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;

import java.io.Serializable;

/**
 * Abstract class representing a rotation in 3D space.
 * Subclasses of this class will implement the interface of this class.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class Rotation3D implements Serializable {

    /**
     * Constant defining threshold to determine whether a matrix is orthogonal
     * or not and has determinant equal to 1. Rotation matrices must fulfill
     * those requirements.
     */
    public static final double DEFAULT_VALID_THRESHOLD = 1e-12;

    /**
     * Constant defining minimum allowed threshold.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Constant defining number of inhomogeneous 3D coordinates.
     */
    public static final int INHOM_COORDS = 3;

    /**
     * Constant defining number of homogeneous 3D coordinates.
     */
    public static final int HOM_COORDS = 4;

    /**
     * Constant defining default type if none is provided.
     */
    public static final Rotation3DType DEFAULT_TYPE = Rotation3DType.QUATERNION;

    /**
     * Default threshold to determine if two instances are equal.
     */
    public static final double DEFAULT_COMPARISON_THRESHOLD = 1e-9;

    /**
     * Constant defining minimum allowed comparison threshold.
     */
    private static final double MIN_COMPARISON_THRESHOLD = 0.0;

    /**
     * Empty constructor.
     */
    protected Rotation3D() {
    }

    /**
     * Returns type of this rotation.
     *
     * @return Type of this rotation.
     */
    public abstract Rotation3DType getType();

    /**
     * Sets the axis and rotation of this instance.
     * Once set, points will rotate around provided axis an amount equal to
     * provided rotation angle in radians.
     * Note: to avoid numerical instabilities and improve accuracy, axis
     * coordinates should be normalized (e.g. norm equal to 1).
     *
     * @param axis  Array of length 3 containing axis coordinates.
     * @param theta Amount of rotation in radians.
     * @throws IllegalArgumentException Raised if provided axis array does not
     *                                  have length 3.
     */
    public final void setAxisAndRotation(final double[] axis, final double theta) {
        if (axis.length != INHOM_COORDS) {
            throw new IllegalArgumentException();
        }

        setAxisAndRotation(axis[0], axis[1], axis[2], theta);
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
    public abstract void setAxisAndRotation(
            final double axisX, final double axisY, final double axisZ, final double theta);

    /**
     * Returns rotation axis corresponding to this instance as a new array
     * containing axis coordinates.
     *
     * @return Rotation axis coordinates.
     * @throws RotationException Raised if numerical instabilities happen.
     */
    public double[] getRotationAxis() throws RotationException {
        final double[] axis = new double[INHOM_COORDS];
        rotationAxis(axis);
        return axis;
    }

    /**
     * Returns rotation axis corresponding to this instance.
     * Result is stored in provided axis array, which must have length 3.
     *
     * @param axis Array where axis coordinates will be stored.
     * @throws IllegalArgumentException Raised if provided array does not have
     *                                  length 3.
     * @throws RotationException        Raised if numerical instabilities happen.
     */
    public abstract void rotationAxis(final double[] axis)
            throws RotationException;

    /**
     * Returns rotation amount or angle in radians around the rotation axis
     * associated to this instance.
     *
     * @return Rotation angle in radians.
     * @throws RotationException Raised if numerical instabilities happen.
     *                           Because internal matrix will always be well-defined (orthogonal and
     *                           determinant equal to 1), this exception will rarely happen.
     */
    public abstract double getRotationAngle() throws RotationException;

    /**
     * Returns a 3D rotation which is inverse to this instance.
     * In other words, the combination of this rotation with its inverse
     * produces no change.
     *
     * @return Inverse 3D rotation.
     */
    public abstract Rotation3D inverseRotationAndReturnNew();

    /**
     * Sets into provided Rotation3D instance a rotation inverse to this
     * instance.
     * The combination of this rotation with its inverse produces no change.
     *
     * @param result Instance where inverse rotation will be set.
     */
    public abstract void inverseRotation(final Rotation3D result);

    /**
     * Reverses the rotation of this instance.
     */
    public abstract void inverseRotation();

    /**
     * Returns this 3D rotation instance expressed as a 3x3 inhomogeneous
     * matrix.
     * This is equivalent to call getInternalMatrix().
     *
     * @return Rotation matrix expressed in inhomogeneous coordinates.
     */
    public abstract Matrix asInhomogeneousMatrix();

    /**
     * Sets into provided Matrix instance this 3D rotation expressed as a
     * 3x3 inhomogeneous matrix.
     *
     * @param result Matrix where rotation will be set.
     * @throws IllegalArgumentException Raised if provided instance does not
     *                                  have size 3x3.
     */
    public abstract void asInhomogeneousMatrix(final Matrix result);

    /**
     * Returns this 3D rotation instance expressed as a 4x4 homogeneous matrix.
     *
     * @return Rotation matrix expressed in homogeneous coordinates.
     */
    public abstract Matrix asHomogeneousMatrix();

    /**
     * Sets into provided Matrix instance this 3D rotation expressed as a
     * 4x4 homogeneous matrix.
     *
     * @param result Matrix where rotation will be set.
     * @throws IllegalArgumentException Raised if provided instance does not
     *                                  have size 4x4.
     */
    public abstract void asHomogeneousMatrix(final Matrix result);

    /**
     * Sets amount of rotation from provided rotation matrix.
     * Provided matrix must be orthogonal (i.e. squared, non-singular, it's
     * transpose must be its inverse) and must have determinant equal to 1.
     * Provided matrix can be expressed in either inhomogeneous (3x3) or
     * homogeneous (4x4) coordinates.
     *
     * @param m         Provided rotation matrix.
     * @param threshold Threshold to determine whether matrix is orthonormal.
     * @throws InvalidRotationMatrixException Raised if provided matrix is not
     *                                        valid (has wrong size, or it is not orthonormal).
     * @throws IllegalArgumentException       Raised if provided threshold is
     *                                        negative.
     *                                        {@link #isValidRotationMatrix(Matrix)}
     */
    public final void fromMatrix(final Matrix m, final double threshold)
            throws InvalidRotationMatrixException {
        if (m.getRows() == INHOM_COORDS &&
                m.getColumns() == INHOM_COORDS) {
            // inhomogeneous matrix
            fromInhomogeneousMatrix(m, threshold);
        } else if (m.getRows() == HOM_COORDS &&
                m.getColumns() == HOM_COORDS) {
            // homogeneous matrix
            fromHomogeneousMatrix(m, threshold);
        } else {
            throw new InvalidRotationMatrixException();
        }
    }

    /**
     * Sets amount of rotation from provided rotation matrix.
     * Provided matrix must be orthogonal (i.e. squared, non-singular, it's
     * transpose must be its inverse) and must have determinant equal to 1.
     * Provided matrix can be expressed in either inhomogeneous (3x3) or
     * homogeneous (4x4) coordinates.
     * Because threshold is not provided it is used DEFAULT_VALID_THRESHOLD
     * instead.
     *
     * @param m Provided rotation matrix.
     * @throws InvalidRotationMatrixException Raised if provided matrix is not
     *                                        valid (has wrong size, or it is not orthonormal).
     *                                        {@link #isValidRotationMatrix(Matrix)}
     */
    public final void fromMatrix(final Matrix m)
            throws InvalidRotationMatrixException {
        fromMatrix(m, DEFAULT_VALID_THRESHOLD);
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
     * @throws IllegalArgumentException       Raised if provided threshold is
     *                                        negative.
     *                                        {@link #isValidRotationMatrix(Matrix)}
     */
    public abstract void fromInhomogeneousMatrix(final Matrix m, final double threshold)
            throws InvalidRotationMatrixException;

    /**
     * Sets amount of rotation from provided inhomogeneous rotation matrix.
     * Provided matrix must be orthogonal (i.e. squared, non-singular, it's
     * transpose must be its inverse) and must have determinant equal to 1.
     * Provided matrix must also have size 3x3.
     * Because threshold is not provided it is used DEFAULT_VALID_THRESHOLD
     * instead.
     *
     * @param m Provided rotation matrix.
     * @throws InvalidRotationMatrixException Raised if provided matrix is not
     *                                        valid (has wrong size, or it is not orthonormal).
     *                                        {@link #isValidRotationMatrix(Matrix)}
     */
    public void fromInhomogeneousMatrix(final Matrix m)
            throws InvalidRotationMatrixException {
        fromInhomogeneousMatrix(m, DEFAULT_VALID_THRESHOLD);
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
     *                                        {@link #isValidRotationMatrix(Matrix)}
     */
    public abstract void fromHomogeneousMatrix(final Matrix m, final double threshold)
            throws InvalidRotationMatrixException;

    /**
     * Sets amount of rotation from provided homogeneous rotation matrix.
     * Provided matrix must be orthogonal (i.e. squared, non-singular), its
     * transpose must be its inverse and must have determinant equal to 1.
     * Provided matrix must also have size exe, and its last row and column must
     * be zero, except for element in last row and column which must be 1
     * Because threshold is not provided it is used DEFAULT_VALID_THRESHOLD
     * instead.
     *
     * @param m Provided rotation matrix.
     * @throws InvalidRotationMatrixException Raised if provided matrix is not
     *                                        valid (has wrong size, or it is not orthonormal).
     *                                        {@link #isValidRotationMatrix(Matrix)}
     */
    public void fromHomogeneousMatrix(final Matrix m)
            throws InvalidRotationMatrixException {
        fromHomogeneousMatrix(m, DEFAULT_VALID_THRESHOLD);
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
    public abstract void rotate(final Point3D inputPoint, final Point3D resultPoint);

    /**
     * Returns a 3D point containing a rotated version of provided point.
     * Point will be rotated using the origin of the coordinates as the axis of
     * rotation.
     * Point will be rotated by the amount of rotation contained in this
     * instance.
     *
     * @param point Point to be rotated.
     * @return Rotated point.
     */
    public Point3D rotate(final Point3D point) {
        final Point3D result = new HomogeneousPoint3D();
        rotate(point, result);
        return result;
    }

    /**
     * Rotates a plane using the origin of coordinates as the axis of rotation.
     * Plane will be rotated by the amount of rotation contained in this
     * instance.
     *
     * @param inputPlane  Input plane to be rotated.
     * @param resultPlane plane where result is stored.
     */
    public void rotate(final Plane inputPlane, final Plane resultPlane) {
        try {
            final Matrix r = asHomogeneousMatrix();
            // because of the duality theorem:
            // P'*M = 0 --> P*R^-1*R*M = 0 --> P2' = P'*R^-1 and M2 = R*M
            // where P2 and M2 are rotated plane and point, however rotated
            // plane uses the inverse rotation, which is the transposed matrix
            // Hence P2' = P' * R', and by undoing the transposition
            // P2 = (P' * R')' = R'' * P'' = R * P

            final Matrix p = new Matrix(Plane.PLANE_NUMBER_PARAMS, 1);

            // to increase accuracy
            inputPlane.normalize();
            p.setElementAt(0, 0, inputPlane.getA());
            p.setElementAt(1, 0, inputPlane.getB());
            p.setElementAt(2, 0, inputPlane.getC());
            p.setElementAt(3, 0, inputPlane.getD());

            // Rotated plane below is R * P
            r.multiply(p);

            resultPlane.setParameters(r.getElementAt(0, 0),
                    r.getElementAt(1, 0), r.getElementAt(2, 0),
                    r.getElementAt(3, 0));
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Returns a plane containing a rotated version of provided plane.
     * Plane will be rotated using the origin of the coordinates as the axis of
     * rotation.
     * Plane will be rotated by the amount of rotation contained in this
     * instance.
     *
     * @param plane Plane to be rotated.
     * @return Rotated plane.
     */
    public Plane rotate(final Plane plane) {
        final Plane result = new Plane();
        rotate(plane, result);
        return result;
    }

    /**
     * Returns boolean indicating whether provided matrix is a valid matrix for
     * a rotation.
     * Rotation matrices must be orthogonal and must have determinant equal to
     * 1.
     *
     * @param m         Input matrix to be checked.
     * @param threshold Threshold to determine whether matrix is orthogonal and
     *                  whether determinant is one.
     * @return True if matrix is valid, false otherwise.
     * @throws IllegalArgumentException Raised if provided threshold is
     *                                  negative.
     */
    public static boolean isValidRotationMatrix(final Matrix m, final double threshold) {
        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        try {
            return Utils.isOrthogonal(m, threshold) &&
                    Math.abs(Math.abs(Utils.det(m)) - 1.0) < threshold;
        } catch (final AlgebraException e) {
            return false;
        }
    }

    /**
     * Returns boolean indicating whether provided matrix is a valid matrix for
     * a rotation.
     * Rotation matrices must be orthogonal and must have determinant equal to 1
     * Because threshold is not provided, it is used DEFAULT_VALID_THRESHOLD
     * instead.
     *
     * @param m Input matrix to be checked.
     * @return True if matrix is valid, false otherwise.
     * @throws IllegalArgumentException Raised if provided threshold is
     *                                  negative.
     */
    public static boolean isValidRotationMatrix(final Matrix m) {
        return isValidRotationMatrix(m, DEFAULT_VALID_THRESHOLD);
    }

    /**
     * Combines provided rotation with this rotation and returns the result as
     * a new Rotation3D instance.
     *
     * @param rotation Input rotation to be combined.
     * @return Combined rotation, which is equal to the multiplication of the
     * internal matrix of provided rotation with the internal matrix of this
     * instance.
     */
    public abstract Rotation3D combineAndReturnNew(final Rotation3D rotation);

    /**
     * Combines provided rotation into this rotation resulting in the
     * multiplication of the internal matrices of both rotations.
     *
     * @param rotation Input rotation to be combined.
     */
    public abstract void combine(final Rotation3D rotation);

    /**
     * Factory method.
     * Creates a rotation that has no effect on geometric objects using default
     * type.
     *
     * @return A 3D rotation.
     */
    public static Rotation3D create() {
        return create(DEFAULT_TYPE);
    }

    /**
     * Factory method.
     * Creates a rotation that has no effect on geometric objects using provided
     * type.
     *
     * @param type Rotation type.
     * @return A 3D rotation.
     */
    public static Rotation3D create(final Rotation3DType type) {
        switch (type) {
            case AXIS_ROTATION3D:
                return new AxisRotation3D();
            case MATRIX_ROTATION3D:
                return new MatrixRotation3D();
            case QUATERNION:
            default:
                return new Quaternion();
        }
    }

    /**
     * Factory method.
     * Creates a 3D rotation using provided axis and rotation angle.
     * Note: to increase accuracy axis coordinates should be normalized.
     *
     * @param axis  Array containing rotation axis coordinates.
     * @param theta Rotation angle around axis expressed in radians.
     * @return A 3D rotation instance.
     * @throws IllegalArgumentException Raised if provided axis array does not
     *                                  have length 3.
     */
    public static Rotation3D create(final double[] axis, final double theta) {
        return create(axis, theta, DEFAULT_TYPE);
    }

    /**
     * Factory method.
     * Creates a 3D rotation using provided axis, rotation angle and rotation
     * type.
     * Note: to increase accuracy axis coordinates should be normalized.
     *
     * @param axis  Array containing rotation axis coordinates.
     * @param theta Rotation angle around axis expressed in radians.
     * @param type  Rotation type.
     * @return A 3D rotation instance.
     * @throws IllegalArgumentException Raised if provided axis array does not
     *                                  have length 3.
     */
    public static Rotation3D create(final double[] axis, final double theta,
                                    final Rotation3DType type) {
        switch (type) {
            case AXIS_ROTATION3D:
                return new AxisRotation3D(axis, theta);
            case MATRIX_ROTATION3D:
                return new MatrixRotation3D(axis, theta);
            case QUATERNION:
            default:
                return new Quaternion(axis, theta);
        }
    }

    /**
     * Factory method.
     * Creates a 3D rotation using provided axis coordinates and rotation angle.
     * Note: to increase accuracy axis coordinates should be normalized.
     *
     * @param axisX X coordinate of axis.
     * @param axisY Y coordinate of axis.
     * @param axisZ Z coordinate of axis.
     * @param theta Rotation angle around axis expressed in radians.
     * @return A 3D rotation instance.
     */
    public static Rotation3D create(final double axisX, final double axisY, final double axisZ,
                                    final double theta) {
        return create(axisX, axisY, axisZ, theta, DEFAULT_TYPE);
    }

    /**
     * Factory method.
     * Creates a 3D rotation using provided axis coordinates, rotation angle and
     * rotation type.
     * Note: to increase accuracy axis coordinates should be normalized.
     *
     * @param axisX X coordinate of axis.
     * @param axisY Y coordinate of axis.
     * @param axisZ Z coordinate of axis.
     * @param theta Rotation angle around axis expressed in radians.
     * @param type  Rotation type.
     * @return A 3D rotation instance.
     */
    public static Rotation3D create(final double axisX, final double axisY, final double axisZ,
                                    final double theta, final Rotation3DType type) {
        switch (type) {
            case AXIS_ROTATION3D:
                return new AxisRotation3D(axisX, axisY, axisZ, theta);
            case MATRIX_ROTATION3D:
                return new MatrixRotation3D(axisX, axisY, axisZ, theta);
            case QUATERNION:
            default:
                final Quaternion result = new Quaternion();
                result.setAxisAndRotation(axisX, axisY, axisZ, theta);
                return result;
        }
    }

    /**
     * Determines if two Rotation3D instances are equal up to provided threshold
     * or not (i.e. have the same rotation).
     *
     * @param other     other rotation to compare.
     * @param threshold threshold to determine if they are equal.
     * @return true if they are equal, false otherwise.
     * @throws IllegalArgumentException if threshold is negative.
     * @throws RotationException        if rotation angle or axis cannot be determined.
     */
    public boolean equals(final Rotation3D other, final double threshold)
            throws RotationException {

        if (threshold < MIN_COMPARISON_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        final double[] thisAxis = getRotationAxis();
        final double thisAngle = getRotationAngle();
        final double[] otherAxis = other.getRotationAxis();
        final double otherAngle = other.getRotationAngle();

        final double cosAngle = ArrayUtils.dotProduct(thisAxis, otherAxis);
        if (cosAngle >= 0.0) {
            // axis have same direction
            final double diffX = thisAxis[0] - otherAxis[0];
            final double diffY = thisAxis[1] - otherAxis[1];
            final double diffZ = thisAxis[2] - otherAxis[2];
            final double sqrNormDiff = diffX * diffX + diffY * diffY + diffZ * diffZ;

            if (sqrNormDiff > threshold) {
                // axes are not equal
                return false;
            }

            // compare difference of angles
            return Math.abs(thisAngle - otherAngle) <= threshold ||
                    Math.abs(thisAngle - otherAngle - 2 * Math.PI) <= threshold;
        } else {
            // axis might be reversed, hence also angle is reversed
            final double sumX = thisAxis[0] + otherAxis[0];
            final double sumY = thisAxis[1] + otherAxis[1];
            final double sumZ = thisAxis[2] + otherAxis[2];
            final double sqrNormSum = sumX * sumX + sumY * sumY + sumZ * sumZ;

            if (sqrNormSum > threshold) {
                // axes are not equal
                return false;
            }

            // compare sum of angles (because rotation angle is also reversed)
            return Math.abs(thisAngle + otherAngle) <= threshold ||
                    Math.abs(thisAngle + otherAngle - 2 * Math.PI) <= threshold;
        }
    }

    /**
     * Determines if two Rotation3D instances are equal or not (i.e. have the
     * same rotation).
     *
     * @param other other object to compare.
     * @return true if they are equal, false otherwise.
     */
    public boolean equals(final Rotation3D other) {
        try {
            return equals(other, DEFAULT_COMPARISON_THRESHOLD);
        } catch (final RotationException e) {
            return false;
        }
    }

    /**
     * Determines if two Rotation3D instances are equal or not (i.e. have the
     * same rotation).
     *
     * @param obj other object to compare.
     * @return true if they are equal, false otherwise.
     */
    @Override
    public boolean equals(final Object obj) {
        if (obj == this) {
            return true;
        }
        if (!(obj instanceof Rotation3D)) {
            return false;
        }

        return equals((Rotation3D) obj);
    }

    /**
     * Hash code to compare instances.
     *
     * @return hash code to compare instances.
     */
    @Override
    public int hashCode() {
        return 5;
    }

    /**
     * Sets values of this rotation from a 3D matrix rotation.
     *
     * @param rot 3D matrix rotation to set values from.
     */
    public void fromRotation(final MatrixRotation3D rot) {
        try {
            fromInhomogeneousMatrix(rot.internalMatrix);
        } catch (final InvalidRotationMatrixException ignore) {
            // never thrown
        }
    }

    /**
     * Sets values of this rotation from a 3D axis rotation.
     *
     * @param rot an axis rotation to set values from.
     */
    public void fromRotation(final AxisRotation3D rot) {
        setAxisAndRotation(rot.getAxisX(), rot.getAxisY(), rot.getAxisZ(),
                rot.getRotationAngle());
    }

    /**
     * Sets values of this rotation from a quaternion.
     *
     * @param q a quaternion to set values from.
     */
    public abstract void fromRotation(final Quaternion q);

    /**
     * Sets values of this rotation from another rotation.
     *
     * @param rot a 3D rotation to set values from.
     * @throws IllegalArgumentException if provided rotation type is
     *                                  not supported. Only {@link Rotation3DType#AXIS_ROTATION3D},
     *                                  {@link Rotation3DType#MATRIX_ROTATION3D} and
     *                                  {@link Rotation3DType#QUATERNION} are supported.
     */
    public void fromRotation(final Rotation3D rot) {
        switch (rot.getType()) {
            case AXIS_ROTATION3D:
                fromRotation((AxisRotation3D) rot);
                break;
            case MATRIX_ROTATION3D:
                fromRotation((MatrixRotation3D) rot);
                break;
            case QUATERNION:
                fromRotation((Quaternion) rot);
                break;
            default:
                throw new IllegalArgumentException();
        }
    }

    /**
     * Converts this 3D rotation into a matrix rotation storing the result
     * into provided instance.
     *
     * @param result instance where result wil be stored.
     */
    public void toMatrixRotation(final MatrixRotation3D result) {
        result.fromRotation(this);
    }

    /**
     * Converts this 3D rotation into a matrix rotation and returns the result
     * as a new instance.
     *
     * @return a new 3D matrix rotation equivalent to this rotation.
     */
    public MatrixRotation3D toMatrixRotation() {
        final MatrixRotation3D r = new MatrixRotation3D();
        toMatrixRotation(r);
        return r;
    }

    /**
     * Converts this 3D rotation into an axis rotation storing the result into
     * provided instance.
     *
     * @param result instance where result will be stored.
     */
    public void toAxisRotation(final AxisRotation3D result) {
        result.fromRotation(this);
    }

    /**
     * Converts this 3D rotation into an axis rotation and returns the result
     * as a new instance.
     *
     * @return a new axis rotation equivalent to this rotation.
     */
    public AxisRotation3D toAxisRotation() {
        final AxisRotation3D r = new AxisRotation3D();
        toAxisRotation(r);
        return r;
    }

    /**
     * Converts this 3D rotation into a quaternion storing the result into
     * provided instance.
     *
     * @param result instance where result will be stored.
     */
    public void toQuaternion(final Quaternion result) {
        result.fromRotation(this);
    }

    /**
     * Converts this 3D rotation into a quaternion and returns the result as a
     * new instance.
     *
     * @return a new quaternion equivalent to this rotation.
     */
    public Quaternion toQuaternion() {
        final Quaternion q = new Quaternion();
        toQuaternion(q);
        return q;
    }
}
