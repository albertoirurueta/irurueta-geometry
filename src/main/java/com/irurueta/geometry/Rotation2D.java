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
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;

import java.io.Serializable;
import java.util.Objects;

/**
 * This class defines the amount of rotation for 2D points or lines.
 */
@SuppressWarnings("DuplicatedCode")
public class Rotation2D implements Serializable {

    /**
     * Constant defining the number of rows on a 2D rotation matrix expressed
     * in inhomogeneous coordinates.
     */
    public static final int ROTATION2D_INHOM_MATRIX_ROWS = 2;

    /**
     * Constant defining the number of columns on a 2D rotation matrix expressed
     * in inhomogeneous coordinates.
     */
    public static final int ROTATION2D_INHOM_MATRIX_COLS = 2;

    /**
     * Constant defining the number of rows on a 2D rotation matrix expressed
     * in homogeneous coordinates.
     */
    public static final int ROTATION2D_HOM_MATRIX_ROWS = 3;

    /**
     * Constant defining the number of columns on a 2D rotation matrix expressed
     * in homogeneous coordinates.
     */
    public static final int ROTATION2D_HOM_MATRIX_COLS = 3;

    /**
     * Constant defining threshold to determine whether a matrix is orthogonal
     * or not and has determinant equal to 1. Rotation matrices must fulfill
     * those requirements.
     */
    public static final double MATRIX_VALID_THRESHOLD = 1e-9;

    /**
     * Constant defining minimum allowed threshold.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Default threshold to determine if two instances are equal.
     */
    public static final double DEFAULT_COMPARISON_THRESHOLD = 1e-9;

    /**
     * Constant defining minimum allowed comparison threshold.
     */
    public static final double MIN_COMPARISON_THRESHOLD = 0.0;

    /**
     * Private member containing amount of rotation expressed in radians.
     */
    private double theta;

    /**
     * Empty Constructor.
     * Initializes rotation to zero radians.
     */
    public Rotation2D() {
        theta = 0.0;
    }

    /**
     * Copy constructor.
     * Copies provided rotation into this instance.
     *
     * @param rotation Instance to be copied.
     */
    public Rotation2D(final Rotation2D rotation) {
        theta = rotation.theta;
    }

    /**
     * Constructor.
     * Creates a 2D rotation using provided matrix.
     * Provided matrix can be expressed in either homogeneous or inhomogeneous
     * coordinates, and it must also be orthogonal and having determinant equal
     * to 1.
     * The threshold to determine whether provided matrix is orthonormal will
     * be MATRIX_VALID_THRESHOLD.
     *
     * @param m Matrix to create rotation from.
     * @throws InvalidRotationMatrixException Raised if provided matrix is not
     *                                        valid (its size is wrong, or it is not orthonormal).
     * @see #isValidRotationMatrix(Matrix)
     */
    public Rotation2D(final Matrix m) throws InvalidRotationMatrixException {
        fromMatrix(m);
    }

    /**
     * Constructor.
     * Creates a 2D rotation using provided matrix.
     * Provided matrix can be expressed in either homogeneous or inhomogeneous
     * coordinates, and it must also be orthogonal up to provided threshold, and
     * must have determinant equal to 1.
     *
     * @param m         Matrix to create rotation from.
     * @param threshold Threshold to determine whether matrix is orthonormal.
     * @throws InvalidRotationMatrixException Raised if provided matrix is not
     *                                        valid (its size is wrong, or it is not orthonormal).
     * @throws IllegalArgumentException       Raised if provided threshold is negative
     * @see #isValidRotationMatrix(Matrix)
     */
    public Rotation2D(final Matrix m, final double threshold) throws InvalidRotationMatrixException {
        fromMatrix(m, threshold);
    }

    /**
     * Constructor.
     * Creates a 2D rotation using provided rotation value expressed in radians
     *
     * @param theta Rotation amount expressed in radians.
     */
    public Rotation2D(final double theta) {
        this.theta = theta;
    }

    /**
     * Returns rotation amount expressed in radians.
     *
     * @return Rotation amount expressed in radians.
     */
    public double getTheta() {
        return theta;
    }

    /**
     * Sets rotation amount expressed in radians.
     *
     * @param theta rotation angle.
     */
    public void setTheta(final double theta) {
        this.theta = theta;
    }

    /**
     * Returns a 2D rotation which is inverse to this instance.
     * In other words, the combination of this rotation with its inverse
     * produces no change.
     *
     * @return Inverse 2D rotation.
     */
    public Rotation2D inverseRotation() {
        final var result = new Rotation2D();
        inverseRotation(result);
        return result;
    }

    /**
     * Sets into provided Rotation2D instance a rotation inverse to this
     * instance.
     * The combination of this rotation with its inverse produces no change.
     *
     * @param result Instance where inverse rotation will be set.
     */
    public void inverseRotation(final Rotation2D result) {
        result.setTheta(-theta);
    }

    /**
     * Returns this 2D rotation instance expressed as a 2x2 inhomogeneous matrix.
     *
     * @return Rotation matrix expressed in inhomogeneous coordinates.
     */
    public Matrix asInhomogeneousMatrix() {
        Matrix result = null;
        try {
            result = new Matrix(ROTATION2D_INHOM_MATRIX_ROWS, ROTATION2D_INHOM_MATRIX_COLS);
            asInhomogeneousMatrix(result);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
        return result;
    }

    /**
     * Sets into provided Matrix instance this 2D rotation expressed as a
     * 2x2 inhomogeneous matrix.
     *
     * @param result Matrix where rotation will be set.
     * @throws IllegalArgumentException Raised if provided instance does not
     *                                  have size 2x2.
     */
    public void asInhomogeneousMatrix(final Matrix result) {
        if (result.getRows() != ROTATION2D_INHOM_MATRIX_ROWS || result.getColumns() != ROTATION2D_INHOM_MATRIX_COLS) {
            throw new IllegalArgumentException();
        }

        // set result
        final var sinTheta = Math.sin(theta);
        final var cosTheta = Math.cos(theta);
        result.setElementAt(0, 0, cosTheta);
        result.setElementAt(1, 0, sinTheta);
        result.setElementAt(0, 1, -sinTheta);
        result.setElementAt(1, 1, cosTheta);
    }

    /**
     * Returns this 2D rotation instance expressed as a 3x3 homogeneous matrix.
     *
     * @return Rotation matrix expressed in homogeneous coordinates.
     */
    public Matrix asHomogeneousMatrix() {
        Matrix result = null;
        try {
            result = new Matrix(ROTATION2D_HOM_MATRIX_ROWS, ROTATION2D_HOM_MATRIX_COLS);
            asHomogeneousMatrix(result);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
        return result;
    }

    /**
     * Sets into provided Matrix instance this 2D rotation expressed as a
     * 3x3 homogeneous matrix.
     *
     * @param result Matrix where rotation will be set.
     * @throws IllegalArgumentException Raised if provided instance does not
     *                                  have size 3x3.
     */
    public void asHomogeneousMatrix(final Matrix result) {
        if (result.getRows() != ROTATION2D_HOM_MATRIX_ROWS || result.getColumns() != ROTATION2D_HOM_MATRIX_COLS) {
            throw new IllegalArgumentException();
        }

        // set result
        final var sinTheta = Math.sin(theta);
        final var cosTheta = Math.cos(theta);
        result.setElementAt(0, 0, cosTheta);
        result.setElementAt(1, 0, sinTheta);
        result.setElementAt(2, 0, 0.0);
        result.setElementAt(0, 1, -sinTheta);
        result.setElementAt(1, 1, cosTheta);
        result.setElementAt(2, 1, 0.0);
        result.setElementAt(0, 2, 0.0);
        result.setElementAt(1, 2, 0.0);
        result.setElementAt(2, 2, 1.0);
    }

    /**
     * Sets amount of rotation from provided rotation matrix.
     * Provided matrix must be orthogonal (i.e. squared, non-singular, it's
     * transpose must be its inverse) and must have determinant equal to 1.
     * Provided matrix can be expressed in either inhomogeneous (2x2) or
     * homogeneous (3x3) coordinates.
     *
     * @param m         Provided rotation matrix.
     * @param threshold Threshold to determine whether matrix is orthonormal.
     * @throws InvalidRotationMatrixException Raised if provided matrix is not
     *                                        valid (has wrong size, or it is not orthonormal).
     * @throws IllegalArgumentException       Raised if provided threshold is negative
     * @see #isValidRotationMatrix(Matrix)
     */
    public final void fromMatrix(final Matrix m, final double threshold) throws InvalidRotationMatrixException {
        if (m.getRows() == ROTATION2D_INHOM_MATRIX_ROWS && m.getColumns() == ROTATION2D_INHOM_MATRIX_COLS) {
            // inhomogeneous matrix
            fromInhomogeneousMatrix(m, threshold);
        } else if (m.getRows() == ROTATION2D_HOM_MATRIX_ROWS && m.getColumns() == ROTATION2D_HOM_MATRIX_COLS) {
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
     * Provided matrix can be expressed in either inhomogeneous (2x2) or
     * homogeneous (3x3) coordinates.
     * Because threshold is not provided it is used MATRIX_VALID_THRESHOLD
     * instead.
     *
     * @param m Provided rotation matrix.
     * @throws InvalidRotationMatrixException Raised if provided matrix is not
     *                                        valid (has wrong size, or it is not orthonormal).
     * @see #isValidRotationMatrix(Matrix)
     */
    public final void fromMatrix(final Matrix m) throws InvalidRotationMatrixException {
        fromMatrix(m, MATRIX_VALID_THRESHOLD);
    }

    /**
     * Sets amount of rotation from provided inhomogeneous rotation matrix.
     * Provided matrix must be orthogonal (i.e. squared, non-singular, it's
     * transpose must be its inverse) and must have determinant equal to 1.
     * Provided matrix must also have size 2x2.
     *
     * @param m         Provided rotation matrix.
     * @param threshold Threshold to determine whether matrix is orthonormal.
     * @throws InvalidRotationMatrixException Raised if provided matrix is not
     *                                        valid (has wrong size, or it is not orthonormal).
     * @throws IllegalArgumentException       Raised if provided threshold is negative.
     * @see #isValidRotationMatrix(Matrix)
     */
    public void fromInhomogeneousMatrix(final Matrix m, final double threshold) throws InvalidRotationMatrixException {
        if (m.getRows() != ROTATION2D_INHOM_MATRIX_ROWS || m.getColumns() != ROTATION2D_INHOM_MATRIX_COLS) {
            throw new InvalidRotationMatrixException();
        }
        if (!isValidRotationMatrix(m, threshold)) {
            throw new InvalidRotationMatrixException();
        }

        final var cosTheta = m.getElementAt(0, 0);
        final var sinTheta = m.getElementAt(1, 0);

        // estimated theta will be in the range -pi, pi.
        theta = Math.atan2(sinTheta, cosTheta);
    }

    /**
     * Sets amount of rotation from provided inhomogeneous rotation matrix.
     * Provided matrix must be orthogonal (i.e. squared, non-singular, it's
     * transpose must be its inverse) and must have determinant equal to 1.
     * Provided matrix must also have size 2x2.
     * Because threshold is not provided it is used MATRIX_VALID_THRESHOLD
     * instead.
     *
     * @param m Provided rotation matrix.
     * @throws InvalidRotationMatrixException Raised if provided matrix is not
     *                                        valid (has wrong size, or it is not orthonormal).
     * @see #isValidRotationMatrix(Matrix)
     */
    public void fromInhomogeneousMatrix(final Matrix m) throws InvalidRotationMatrixException {
        fromInhomogeneousMatrix(m, MATRIX_VALID_THRESHOLD);
    }

    /**
     * Sets amount of rotation from provided homogeneous rotation matrix.
     * Provided matrix must be orthogonal (i.e. squared, non-singular, it's
     * transpose must be its inverse) and must have determinant equal to 1.
     * Provided matrix must also have size 3x3, and its last row and column must
     * be zero, except for element in last row and column which must be 1.
     *
     * @param m         Provided rotation matrix.
     * @param threshold Threshold to determine whether matrix is orthonormal.
     * @throws InvalidRotationMatrixException Raised if provided matrix is not
     *                                        valid (has wrong size, or it is not orthonormal).
     * @throws IllegalArgumentException       Raised if provided threshold is negative.
     * @see #isValidRotationMatrix(Matrix)
     */
    public void fromHomogeneousMatrix(final Matrix m, final double threshold) throws InvalidRotationMatrixException {
        if (m.getRows() != ROTATION2D_HOM_MATRIX_ROWS || m.getColumns() != ROTATION2D_HOM_MATRIX_COLS) {
            throw new InvalidRotationMatrixException();
        }
        if (!isValidRotationMatrix(m, threshold)) {
            throw new InvalidRotationMatrixException();
        }
        if (Math.abs(m.getElementAt(2, 0)) > threshold
                || Math.abs(m.getElementAt(2, 1)) > threshold
                || Math.abs(m.getElementAt(0, 2)) > threshold
                || Math.abs(m.getElementAt(1, 2)) > threshold
                || Math.abs(m.getElementAt(2, 2) - 1.0) > threshold) {
            throw new InvalidRotationMatrixException();
        }

        final var cosTheta = m.getElementAt(0, 0);
        final var sinTheta = m.getElementAt(1, 0);

        // estimated theta will be in the range -pi, pi.
        theta = Math.atan2(sinTheta, cosTheta);
    }

    /**
     * Sets amount of rotation from provided homogeneous rotation matrix.
     * Provided matrix must be orthogonal (i.e. squared, non-singular), its
     * transpose must be its inverse and must have determinant equal to 1.
     * Provided matrix must also have size 3x3, and its last row and column must
     * be zero, except for element in last row and column which must be 1
     * Because threshold is not provided it is used MATRIX_VALID_THRESHOLD
     * instead.
     *
     * @param m Provided rotation matrix.
     * @throws InvalidRotationMatrixException Raised if provided matrix is not
     *                                        valid (has wrong size, or it is not orthonormal).
     * @see #isValidRotationMatrix(Matrix)
     */
    public void fromHomogeneousMatrix(final Matrix m) throws InvalidRotationMatrixException {
        fromHomogeneousMatrix(m, MATRIX_VALID_THRESHOLD);
    }

    /**
     * Rotates a 2D point using the origin of coordinates as the axis of
     * rotation.
     * Point will be rotated by the amount of rotation contained in this
     * instance.
     *
     * @param inputPoint  Input point to be rotated.
     * @param resultPoint Rotated point.
     */
    public void rotate(final Point2D inputPoint, final Point2D resultPoint) {
        try {
            final var r = asHomogeneousMatrix();
            final var p = new Matrix(Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);

            // to increase accuracy
            inputPoint.normalize();
            p.setElementAt(0, 0, inputPoint.getHomX());
            p.setElementAt(1, 0, inputPoint.getHomY());
            p.setElementAt(2, 0, inputPoint.getHomW());

            // Rotated point below is R * p
            r.multiply(p);

            resultPoint.setHomogeneousCoordinates(r.getElementAt(0, 0), r.getElementAt(1, 0),
                    r.getElementAt(2, 0));
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Returns a 2D point containing a rotated version of provided point.
     * Point will be rotated using the origin of the coordinates as the axis of
     * rotation.
     * Point will be rotated by the amount of rotation contained in this
     * instance.
     *
     * @param point Point to be rotated.
     * @return Rotated point.
     */
    public Point2D rotate(final Point2D point) {
        final var result = new HomogeneousPoint2D();
        rotate(point, result);
        return result;
    }

    /**
     * Rotates a line using the origin of coordinates as the axis of rotation.
     * Line2D will be rotated by the amount of rotation contained in this
     * instance.
     *
     * @param inputLine  Input line to be rotated.
     * @param resultLine Rotated line.
     */
    public void rotate(final Line2D inputLine, final Line2D resultLine) {
        try {
            final var r = asHomogeneousMatrix();
            // because of the duality theorem:
            // l'*m = 0 --> l*R^-1*R*m = 0 --> l2' = l'*R^-1 and m2 = R*m
            // where l2 and m2 are rotated line and point, however rotated
            // line uses the inverse rotation, which is the transposed matrix
            // Hence l2' = l' * R', and by undoing the transposition
            // l2 = (l' * R')' = R'' * l'' = R * l

            final var l = new Matrix(Line2D.LINE_NUMBER_PARAMS, 1);

            // to increase accuracy
            inputLine.normalize();
            l.setElementAt(0, 0, inputLine.getA());
            l.setElementAt(1, 0, inputLine.getB());
            l.setElementAt(2, 0, inputLine.getC());

            // Rotated line below is R * l
            r.multiply(l);

            resultLine.setParameters(r.getElementAt(0, 0), r.getElementAt(1, 0),
                    r.getElementAt(2, 0));
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Returns a line containing a rotated version of provided line.
     * Line2D will be rotated using the origin of the coordinates as the axis of
     * rotation.
     * Line2D will be rotated by the amount of rotation contained in this
     * instance.
     *
     * @param line Line2D to be rotated.
     * @return Rotated line.
     */
    public Line2D rotate(final Line2D line) {
        final var result = new Line2D();
        rotate(line, result);
        return result;
    }

    /**
     * Returns boolean indicating whether provided matrix is a valid matrix for
     * a rotation.
     * Rotation matrices must be orthogonal and must have determinant equal to 1.
     *
     * @param m         Input matrix to be checked.
     * @param threshold Threshold to determine whether matrix is orthogonal and
     *                  whether determinant is one.
     * @return True if matrix is valid, false otherwise.
     * @throws IllegalArgumentException Raised if provided threshold is negative.
     */
    public static boolean isValidRotationMatrix(final Matrix m, final double threshold) {
        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        try {
            return Utils.isOrthogonal(m, threshold) && (Math.abs(Utils.det(m)) - 1.0) < threshold;
        } catch (final AlgebraException e) {
            return false;
        }
    }

    /**
     * Returns boolean indicating whether provided matrix is a valid matrix for
     * a rotation.
     * Rotation matrices must be orthogonal and must have determinant equal to 1
     * Because threshold is not provided, it is used MATRIX_VALID_THRESHOLD
     * instead.
     *
     * @param m Input matrix to be checked.
     * @return True if matrix is valid, false otherwise.
     * @throws IllegalArgumentException Raised if provided threshold is negative.
     */
    public static boolean isValidRotationMatrix(final Matrix m) {
        return isValidRotationMatrix(m, MATRIX_VALID_THRESHOLD);
    }

    /**
     * Combines provided rotation with this rotation and returns the result as
     * a new Rotation2D instance.
     *
     * @param rotation Input rotation to be combined.
     * @return Combined rotation, which is equal to the sum of provided rotation
     * with this rotation.
     */
    public Rotation2D combineAndReturnNew(final Rotation2D rotation) {
        final var result = new Rotation2D();
        combine(this, rotation, result);
        return result;
    }

    /**
     * Combines provided rotation into this rotation resulting in the sum of
     * both rotations.
     *
     * @param rotation Input rotation to be combined.
     */
    public void combine(final Rotation2D rotation) {
        combine(this, rotation, this);
    }

    /**
     * Combines the rotation of instances rot1 and rot1 into provided result
     * instance.
     *
     * @param rot1   1st input rotation.
     * @param rot2   2nd input rotation.
     * @param result Combined rotation, which is equal to the sum of provided
     *               input rotations.
     */
    public static void combine(final Rotation2D rot1, final Rotation2D rot2, final Rotation2D result) {
        result.theta = rot1.theta + rot2.theta;
    }

    /**
     * Determines if two Rotation2D instances are equal up to provided threshold
     * or not (i.e. have the same rotation).
     *
     * @param other     other rotation to compare.
     * @param threshold threshold to determine if they are equal.
     * @return true if they are equal, false otherwise.
     * @throws IllegalArgumentException if threshold is negative.
     */
    public boolean equals(final Rotation2D other, final double threshold) {
        if (threshold < MIN_COMPARISON_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        return Math.abs(other.theta - theta) <= threshold;
    }

    /**
     * Determines if two Rotation2D instances are equal or not (i.e. have the
     * same rotation).
     *
     * @param other other object to compare.
     * @return true if they are equal, false otherwise.
     */
    public boolean equals(final Rotation2D other) {
        return equals(other, DEFAULT_COMPARISON_THRESHOLD);
    }

    /**
     * Determines if two Rotation2D instances are equal or not (i.e. have the
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
        if (!(obj instanceof Rotation2D)) {
            return false;
        }

        return equals((Rotation2D) obj);
    }

    /**
     * Hash code to compare instances.
     *
     * @return hash code to compare instances.
     */
    @Override
    public int hashCode() {
        return Objects.hash(theta);
    }
}
