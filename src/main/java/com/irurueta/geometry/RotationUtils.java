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

/**
 * Utility methods related to rotations.
 */
@SuppressWarnings("DuplicatedCode")
public class RotationUtils {

    /**
     * Number of components of angular rates (angular speed).
     */
    public static final int N_ANGULAR_RATES = 3;

    /**
     * Size of skew symmetric matrix omega.
     */
    public static final int SKEW_MATRIX_SIZE = 4;

    /**
     * Constructor.
     */
    private RotationUtils() {
    }

    /**
     * Skew symmetric matrix omega from angular rates vector w.
     *
     * @param w1     angular rate from x-axis.
     * @param w2     angular rate from y-axis.
     * @param w3     angular rate from z-axis.
     * @param result instance where result will be stored.
     * @throws IllegalArgumentException if provided matrix is not 4x4.
     * @see <a href="https://github.com/joansola/slamtb">w2omega.m at https://github.com/joansola/slamtb</a>
     */
    public static void w2omega(
            final double w1, final double w2, final double w3, final Matrix result) {
        if (result.getRows() != SKEW_MATRIX_SIZE ||
                result.getColumns() != SKEW_MATRIX_SIZE) {
            throw new IllegalArgumentException("result must be 4x4");
        }

        result.setElementAt(0, 0, 0.0);
        result.setElementAt(1, 0, w1);
        result.setElementAt(2, 0, w2);
        result.setElementAt(3, 0, w3);

        result.setElementAt(0, 1, -w1);
        result.setElementAt(1, 1, 0.0);
        result.setElementAt(2, 1, -w3);
        result.setElementAt(3, 1, w2);

        result.setElementAt(0, 2, -w2);
        result.setElementAt(1, 2, w3);
        result.setElementAt(2, 2, 0.0);
        result.setElementAt(3, 2, -w1);

        result.setElementAt(0, 3, -w3);
        result.setElementAt(1, 3, -w2);
        result.setElementAt(2, 3, w1);
        result.setElementAt(3, 3, 0.0);
    }

    /**
     * Skew symmetric matrix omega from angular rates vector w.
     *
     * @param w      array containing angular rates. Must have length 3.
     * @param result instance where result will be stored.
     * @throws IllegalArgumentException if provided matrix is not 4x4 or array w
     *                                  does not have length 3.
     * @see <a href="https://github.com/joansola/slamtb">w2omega.m at https://github.com/joansola/slamtb</a>
     */
    public static void w2omega(final double[] w, final Matrix result) {
        if (w.length != N_ANGULAR_RATES) {
            throw new IllegalArgumentException("w must have length 3");
        }
        w2omega(w[0], w[1], w[2], result);
    }

    /**
     * Skew symmetric matrix omega from angular rates vector w.
     *
     * @param w1 angular rate from x-axis.
     * @param w2 angular rate from y-axis.
     * @param w3 angular rate from z-axis.
     * @return a new skew symmetric matrix omega.
     * @see <a href="https://github.com/joansola/slamtb">w2omega.m at https://github.com/joansola/slamtb</a>
     */
    public static Matrix w2omega(
            final double w1, final double w2, final double w3) {
        Matrix result = null;
        try {
            result = new Matrix(SKEW_MATRIX_SIZE, SKEW_MATRIX_SIZE);
            w2omega(w1, w2, w3, result);
        } catch (final WrongSizeException ignore) {
            // never happens
        }

        return result;
    }

    /**
     * Skew symmetric matrix omega from angular rates vector w.
     *
     * @param w array containing angular rates. Must have length 3.
     * @return a new skew symmetric matrix omega.
     * @see <a href="https://github.com/joansola/slamtb">w2omega.m at https://github.com/joansola/slamtb</a>
     */
    public static Matrix w2omega(final double[] w) {
        if (w.length != N_ANGULAR_RATES) {
            throw new IllegalArgumentException("w must have length 3");
        }
        return w2omega(w[0], w[1], w[2]);
    }

    /**
     * Converts provided quaternion into Pi matrix.
     * Given a quaternion q = [a b c d]', and the angular rates vector
     * w = [p q r]' and omega = w2omega(w) a skew symmetric matrix, then the PI
     * matrix is the Jacobian respect to w, expressed as: PI = omega(w)*q.
     *
     * @param quaternion a quaternion.
     * @param result     matrix where resulting pi matrix is stored.
     * @throws IllegalArgumentException if provided result matrix is not 4x3.
     * @see <a href="https://github.com/joansola/slamtb">q2Pi.m at https://github.com/joansola/slamtb</a>
     */
    public static void quaternionToPiMatrix(
            final Quaternion quaternion,
            final Matrix result) {
        if (result.getRows() != Quaternion.N_PARAMS ||
                result.getColumns() != Quaternion.N_ANGLES) {
            throw new IllegalArgumentException("result matrix must be 4x3");
        }

        final double a = quaternion.getA();
        final double b = quaternion.getB();
        final double c = quaternion.getC();
        final double d = quaternion.getD();

        result.setElementAt(0, 0, -b);
        result.setElementAt(1, 0, a);
        result.setElementAt(2, 0, d);
        result.setElementAt(3, 0, -c);

        result.setElementAt(0, 1, -c);
        result.setElementAt(1, 1, -d);
        result.setElementAt(2, 1, a);
        result.setElementAt(3, 1, b);

        result.setElementAt(0, 2, -d);
        result.setElementAt(1, 2, c);
        result.setElementAt(2, 2, -b);
        result.setElementAt(3, 2, a);
    }

    /**
     * Converts provided quaternion into Pi matrix.
     * Given a quaternion q = [a b c d]', and the angular rates vector
     * w = [p q r]' and omega = w2omega(w) a skew symmetric matrix, then the PI
     * matrix is the Jacobian respect to w, expressed as: PI = omega(w)*q.
     *
     * @param quaternion a quaternion.
     * @return pi matrix.
     * @see <a href="https://github.com/joansola/slamtb">q2Pi.m at https://github.com/joansola/slamtb</a>
     */
    public static Matrix quaternionToPiMatrix(final Quaternion quaternion) {
        Matrix m = null;
        try {
            m = new Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES);
            quaternionToPiMatrix(quaternion, m);
        } catch (final WrongSizeException ignore) {
            // never thrown
        }
        return m;
    }

    /**
     * Converts provided quaternion into conjugated Pi matrix.
     * Given a quaternion q = [a b c d]', then the conjugated Pi matrix is the
     * Pi matrix of the conjugated quaternion [a -b -c -d]'.
     *
     * @param quaternion a quaternion.
     * @param result     matrix where resulting conjugated pi matrix is stored.
     * @throws IllegalArgumentException if provided result matrix is not 4x3.
     * @see <a href="https://github.com/joansola/slamtb">pi2pc.m at https://github.com/joansola/slamtb</a>
     */
    public static void quaternionToConjugatedPiMatrix(
            final Quaternion quaternion, final Matrix result) {
        quaternionToPiMatrix(quaternion.conjugateAndReturnNew(), result);
    }

    /**
     * Converts provided quaternion into conjugated Pi matrix.
     * Given a quaternion q = [a b c d]', then the conjugated Pi matrix is the
     * Pi matrix of the conjugated quaternion [a -b -c -d]'.
     *
     * @param quaternion a quaternion.
     * @return a matrix containing the conjugated pi matrix.
     * @see <a href="https://github.com/joansola/slamtb">pi2pc.m at https://github.com/joansola/slamtb</a>
     */
    public static Matrix quaternionToConjugatedPiMatrix(
            final Quaternion quaternion) {
        Matrix m = null;
        try {
            m = new Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES);
            quaternionToConjugatedPiMatrix(quaternion, m);
        } catch (final WrongSizeException ignore) {
            // never thrown
        }
        return m;
    }

    /**
     * Computes conjugated pi matrix from pi matrix.
     *
     * @param pi     pi matrix used as input.
     * @param result instance where conjugated pi matrix is stored.
     * @throws IllegalArgumentException if provided matrices are not 4x3.
     * @see <a href="https://github.com/joansola/slamtb">pi2pc.m at https://github.com/joansola/slamtb</a>
     */
    public static void piMatrixToConjugatedPiMatrix(
            final Matrix pi, final Matrix result) {
        if (pi.getRows() != Quaternion.N_PARAMS ||
                pi.getColumns() != Quaternion.N_ANGLES) {
            throw new IllegalArgumentException("pi must be 4x3");
        }
        if (result.getRows() != Quaternion.N_PARAMS ||
                result.getColumns() != Quaternion.N_ANGLES) {
            throw new IllegalArgumentException("result must be 4x3");
        }

        result.copyFrom(pi);

        result.setElementAtIndex(0, -pi.getElementAtIndex(0));
        result.setElementAtIndex(2, -pi.getElementAtIndex(2));
        result.setElementAtIndex(3, -pi.getElementAtIndex(3));
        result.setElementAtIndex(4, -pi.getElementAtIndex(4));
        result.setElementAtIndex(5, -pi.getElementAtIndex(5));
        result.setElementAtIndex(7, -pi.getElementAtIndex(7));
        result.setElementAtIndex(8, -pi.getElementAtIndex(8));
        result.setElementAtIndex(9, -pi.getElementAtIndex(9));
        result.setElementAtIndex(10, -pi.getElementAtIndex(10));
    }

    /**
     * Computes conjugated pi matrix from pi matrix.
     *
     * @param pi pi matrix used as input.
     * @return conjugated pi matrix.
     * @throws IllegalArgumentException if provided input matrix is not 4x3.
     * @see <a href="https://github.com/joansola/slamtb">pi2pc.m at https://github.com/joansola/slamtb</a>
     */
    public static Matrix piMatrixToConjugatedPiMatrix(final Matrix pi) {
        Matrix m = null;
        try {
            m = new Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES);
            piMatrixToConjugatedPiMatrix(pi, m);
        } catch (final WrongSizeException ignore) {
            // never thrown
        }
        return m;
    }

    /**
     * Rotates a point by using the rotation matrix obtained from a quaternion.
     *
     * @param q         quaternion containing rotation information.
     * @param point     array containing inhomogeneous 3D coordinates of a point to
     *                  be rotated.
     * @param result    array containing result of rotation.
     * @param jacobianQ jacobian wrt of quaternion. Must be 3x4.
     * @param jacobianP jacobian wrt of point. Must be 3x3.
     * @throws IllegalArgumentException if provided arrays of points or result
     *                                  don't have length 3, or if jacobian of quaternions is not 3x4 (if
     *                                  provided), or if jacobian of point is not 3x3 (if provided).
     * @see <a href="https://github.com/joansola/slamtb">Rp.m at https://github.com/joansola/slamtb</a>
     */
    public static void rotationMatrixTimesVector(
            final Quaternion q, final double[] point,
            final double[] result, final Matrix jacobianQ, final Matrix jacobianP) {
        if (point.length != Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            throw new IllegalArgumentException("point must have length 3");
        }
        if (result.length != Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            throw new IllegalArgumentException("result must have length 3");
        }
        if (jacobianQ != null && (jacobianQ.getRows() != Quaternion.N_ANGLES ||
                jacobianQ.getColumns() != Quaternion.N_PARAMS)) {
            throw new IllegalArgumentException(
                    "jacobian wrt of quaternion must be 3x4");
        }
        if (jacobianP != null &&
                (jacobianP.getRows() != MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS ||
                        jacobianP.getColumns() != MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS)) {
            throw new IllegalArgumentException(
                    "jacobian wrt of point must be 3x3");
        }

        try {
            final Matrix r = new Matrix(MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
                    MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS);
            q.toMatrixRotation(r);

            if (jacobianP != null) {
                jacobianP.copyFrom(r);
            }
            // multiply rotation matrix by point
            final Matrix p = Matrix.newFromArray(point, true);
            r.multiply(p);

            // copy to result
            r.getSubmatrixAsArray(0, 0, r.getRows() - 1,
                    r.getColumns() - 1, result);

            if (jacobianQ != null) {
                final double a = q.getA();
                final double b = q.getB();
                final double c = q.getC();
                final double d = q.getD();

                final double x = point[0];
                final double y = point[1];
                final double z = point[2];

                final double axdycz = 2.0 * (a * x - d * y + c * z);
                final double bxcydz = 2.0 * (b * x + c * y + d * z);
                final double cxbyaz = 2.0 * (c * x - b * y - a * z);
                final double dxaybz = 2.0 * (d * x + a * y - b * z);

                jacobianQ.setElementAt(0, 0, axdycz);
                jacobianQ.setElementAt(1, 0, dxaybz);
                jacobianQ.setElementAt(2, 0, -cxbyaz);

                jacobianQ.setElementAt(0, 1, bxcydz);
                jacobianQ.setElementAt(1, 1, cxbyaz);
                jacobianQ.setElementAt(2, 1, dxaybz);

                jacobianQ.setElementAt(0, 2, -cxbyaz);
                jacobianQ.setElementAt(1, 2, bxcydz);
                jacobianQ.setElementAt(2, 2, -axdycz);

                jacobianQ.setElementAt(0, 3, -dxaybz);
                jacobianQ.setElementAt(1, 3, axdycz);
                jacobianQ.setElementAt(2, 3, bxcydz);
            }
        } catch (final WrongSizeException ignore) {
            // never thrown
        }
    }

    /**
     * Rotates a point by using the rotation matrix obtained from a quaternion.
     *
     * @param q         quaternion containing rotation information.
     * @param point     array containing inhomogeneous 3D coordinates of a point to
     *                  be rotated.
     * @param jacobianQ jacobian wrt of quaternion. Must be 3x4.
     * @param jacobianP jacobian wrt of point. Must be 3x3.
     * @return array containing result of rotation as 3D inhomogeneous
     * coordinates.
     * @throws IllegalArgumentException if provided point array doesn't have
     *                                  length 3, or if jacobian of quaternions is not 3x4 (if provided), or if
     *                                  jacobian of point is not 3x3 (if provided).
     * @see <a href="https://github.com/joansola/slamtb">Rp.m at https://github.com/joansola/slamtb</a>
     */
    public static double[] rotationMatrixTimesVector(
            final Quaternion q, final double[] point,
            final Matrix jacobianQ, final Matrix jacobianP) {
        final double[] result =
                new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        rotationMatrixTimesVector(q, point, result, jacobianQ, jacobianP);
        return result;
    }

    /**
     * Rotates a point by using the rotation matrix obtained from a quaternion.
     *
     * @param q      quaternion containing rotation information.
     * @param point  array containing inhomogeneous 3D coordinates of a point to
     *               be rotated.
     * @param result array containing result of rotation.
     * @throws IllegalArgumentException if provided arrays of points or result
     *                                  don't have length 3.
     * @see <a href="https://github.com/joansola/slamtb">Rp.m at https://github.com/joansola/slamtb</a>
     */
    public static void rotationMatrixTimesVector(
            final Quaternion q, final double[] point,
            final double[] result) {
        rotationMatrixTimesVector(q, point, result, null, null);
    }

    /**
     * Rotates a point by using the rotation matrix obtained from a quaternion.
     *
     * @param q     quaternion containing rotation information.
     * @param point array containing inhomogeneous 3D coordinates of a point to
     *              be rotated.
     * @return array containing result of rotation as 3D inhomogeneous
     * coordinates.
     * @throws IllegalArgumentException if provided point array doesn't have
     *                                  length 3.
     * @see <a href="https://github.com/joansola/slamtb">Rp.m at https://github.com/joansola/slamtb</a>
     */
    public static double[] rotationMatrixTimesVector(
            final Quaternion q, final double[] point) {
        return rotationMatrixTimesVector(q, point, null, null);
    }

    /**
     * Rotates a 3D point by using the rotation matrix obtained from a
     * quaternion.
     *
     * @param q         quaternion containing rotation information.
     * @param point     3D point to be rotated.
     * @param result    result of rotation.
     * @param jacobianQ jacobian wrt of quaternion. Must be 3x4.
     * @param jacobianP jacobian wrt of point. Must be 3x3.
     * @throws IllegalArgumentException if jacobian of quaternions is not 3x4
     *                                  (if provided), or if jacobian of point is not 3x3 (if provided).
     * @see <a href="https://github.com/joansola/slamtb">Rp.m at https://github.com/joansola/slamtb</a>
     */
    public static void rotationMatrixTimesVector(
            final Quaternion q, final Point3D point, final Point3D result,
            final Matrix jacobianQ, final Matrix jacobianP) {
        final double[] coords = new double[]{
                point.getInhomX(), point.getInhomY(), point.getInhomZ()};
        final double[] rp = rotationMatrixTimesVector(q, coords, jacobianQ,
                jacobianP);
        result.setInhomogeneousCoordinates(rp[0], rp[1], rp[2]);
    }

    /**
     * Rotates a 3D point by using the rotation matrix obtained from a
     * quaternion.
     *
     * @param q         quaternion containing rotation information.
     * @param point     3D point to be rotated.
     * @param jacobianQ jacobian wrt of quaternion. Must be 3x4.
     * @param jacobianP jacobian wrt of point. Must be 3x3.
     * @return result of rotation.
     * @throws IllegalArgumentException if jacobian of quaternions is not 3x4.
     *                                  (if provided), of if jacobian of point is not 3x3 (if provided).
     * @see <a href="https://github.com/joansola/slamtb">Rp.m at https://github.com/joansola/slamtb</a>
     */
    public static Point3D rotationMatrixTimesVector(
            final Quaternion q, final Point3D point,
            final Matrix jacobianQ, final Matrix jacobianP) {
        final Point3D result = Point3D.create();
        rotationMatrixTimesVector(q, point, result, jacobianQ, jacobianP);
        return result;
    }

    /**
     * Rotates a 3D point by using the rotation matrix obtained from a
     * quaternion.
     *
     * @param q      quaternion containing rotation information.
     * @param point  3D point to be rotated.
     * @param result result of rotation.
     * @see <a href="https://github.com/joansola/slamtb">Rp.m at https://github.com/joansola/slamtb</a>
     */
    public static void rotationMatrixTimesVector(
            final Quaternion q, final Point3D point, final Point3D result) {
        rotationMatrixTimesVector(q, point, result, null, null);
    }

    /**
     * Rotates a 3D point by using the rotation matrix obtained from a
     * quaternion.
     *
     * @param q     quaternion containing rotation information.
     * @param point 3D point to be rotated.
     * @return result of rotation.
     * @see <a href="https://github.com/joansola/slamtb">Rp.m at https://github.com/joansola/slamtb</a>
     */
    public static Point3D rotationMatrixTimesVector(
            final Quaternion q, final Point3D point) {
        return rotationMatrixTimesVector(q, point, null, null);
    }

    /**
     * Rotates a point by the inverse rotation obtained from a quaternion.
     *
     * @param q         quaternion containing rotation information.
     * @param point     array containing inhomogeneous 3D coordinates of a point to
     *                  be rotated.
     * @param result    array containing result of rotation.
     * @param jacobianQ jacobian wrt of quaternion. Must be 3x4.
     * @param jacobianP jacobian wrt of point. Must be 3x3.
     * @throws IllegalArgumentException if provided arrays of points or result
     *                                  don't have length 3, or if jacobian of quaternions is not 3x4 (if
     *                                  provided), or if jacobian of point is not 3x3 (if provided).
     * @see <a href="https://github.com/joansola/slamtb">Rtp.m at https://github.com/joansola/slamtb</a>
     */
    public static void transposedRotationMatrixTimesVector(
            final Quaternion q, final double[] point, final double[] result,
            final Matrix jacobianQ, final Matrix jacobianP) {
        if (point.length != Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            throw new IllegalArgumentException("point must have length 3");
        }
        if (result.length != Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            throw new IllegalArgumentException("result must have length 3");
        }
        if (jacobianQ != null && (jacobianQ.getRows() != Quaternion.N_ANGLES ||
                jacobianQ.getColumns() != Quaternion.N_PARAMS)) {
            throw new IllegalArgumentException(
                    "jacobian wrt of quaternion must be 3x4");
        }
        if (jacobianP != null &&
                (jacobianP.getRows() != MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS ||
                        jacobianP.getColumns() != MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS)) {
            throw new IllegalArgumentException(
                    "jacobian wrt of point must be 3x3");
        }

        try {
            final Matrix rt = new Matrix(MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
                    MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS);
            q.toMatrixRotation(rt);
            // transpose rotation
            // the transpose of rotation is its inverse
            rt.transpose();

            if (jacobianP != null) {
                jacobianP.copyFrom(rt);
            }
            // multiply rotation matrix by point
            final Matrix p = Matrix.newFromArray(point, true);
            rt.multiply(p);

            // copy to result
            rt.getSubmatrixAsArray(0, 0, rt.getRows() - 1,
                    rt.getColumns() - 1, result);

            if (jacobianQ != null) {
                final double a = q.getA();
                final double b = q.getB();
                final double c = q.getC();
                final double d = q.getD();

                final double x = p.getElementAtIndex(0);
                final double y = p.getElementAtIndex(1);
                final double z = p.getElementAtIndex(2);

                final double axdycz = 2.0 * (a * x + d * y - c * z);
                final double bxcydz = 2.0 * (b * x + c * y + d * z);
                final double cxbyaz = 2.0 * (c * x - b * y + a * z);
                final double dxaybz = 2.0 * (d * x - a * y - b * z);

                jacobianQ.setElementAt(0, 0, axdycz);
                jacobianQ.setElementAt(1, 0, -dxaybz);
                jacobianQ.setElementAt(2, 0, cxbyaz);

                jacobianQ.setElementAt(0, 1, bxcydz);
                jacobianQ.setElementAt(1, 1, cxbyaz);
                jacobianQ.setElementAt(2, 1, dxaybz);

                jacobianQ.setElementAt(0, 2, -cxbyaz);
                jacobianQ.setElementAt(1, 2, bxcydz);
                jacobianQ.setElementAt(2, 2, axdycz);

                jacobianQ.setElementAt(0, 3, -dxaybz);
                jacobianQ.setElementAt(1, 3, -axdycz);
                jacobianQ.setElementAt(2, 3, bxcydz);
            }
        } catch (final WrongSizeException ignore) {
            // never thrown
        }
    }

    /**
     * Rotates a point by the inverse rotation obtained from a quaternion.
     *
     * @param q         quaternion containing rotation information.
     * @param point     array containing inhomogeneous 3D coordinates of a point to
     *                  be rotated.
     * @param jacobianQ jacobian wrt of quaternion. Must be 3x4.
     * @param jacobianP jacobian wrt of point. Must be 3x3.
     * @return result of rotation.
     * @throws IllegalArgumentException if provided arrays of points doesn't
     *                                  have length 3, or if jacobian of quaternions is not 3x4 (if provided), or
     *                                  if jacobian of point is not 3x3 (if provided).
     * @see <a href="https://github.com/joansola/slamtb">Rtp.m at https://github.com/joansola/slamtb</a>
     */
    public static double[] transposedRotationMatrixTimesVector(
            final Quaternion q, final double[] point,
            final Matrix jacobianQ, final Matrix jacobianP) {
        final double[] result =
                new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        transposedRotationMatrixTimesVector(q, point, result, jacobianQ,
                jacobianP);
        return result;
    }

    /**
     * Rotates a point by the inverse rotation obtained from a quaternion.
     *
     * @param q      quaternion containing rotation information.
     * @param point  array containing inhomogeneous 3D coordinates of a point to
     *               be rotated.
     * @param result result of rotation.
     * @throws IllegalArgumentException if provided arrays of points doesn't
     *                                  have length 3.
     * @see <a href="https://github.com/joansola/slamtb">Rtp.m at https://github.com/joansola/slamtb</a>
     */
    public static void transposedRotationMatrixTimesVector(
            final Quaternion q, final double[] point, final double[] result) {
        transposedRotationMatrixTimesVector(q, point, result, null, null);
    }

    /**
     * Rotates a point by the inverse rotation obtained from a quaternion.
     *
     * @param q     quaternion containing rotation information.
     * @param point array containing inhomogeneous 3D coordinates of a point to
     *              be rotated.
     * @return array containing result of rotation as 3D inhomogeneous
     * coordinates.
     * @throws IllegalArgumentException if provided point array doesn't have
     *                                  length 3.
     * @see <a href="https://github.com/joansola/slamtb">Rtp.m at https://github.com/joansola/slamtb</a>
     */
    public static double[] transposedRotationMatrixTimesVector(
            final Quaternion q, final double[] point) {
        return transposedRotationMatrixTimesVector(q, point, null, null);
    }

    /**
     * Rotates a 3D point by the inverse rotation obtained from a quaternion.
     *
     * @param q         quaternion containing rotation information.
     * @param point     array containing inhomogeneous 3D coordinates of a point to
     *                  be rotated.
     * @param result    result of rotation.
     * @param jacobianQ jacobian wrt of quaternion. Must be 3x4.
     * @param jacobianP jacobian wrt of point. Must be 3x3.
     * @throws IllegalArgumentException if jacobian of quaternions is not 3x4
     *                                  (if provided), or if jacobian of point is not 3x3 (if provided).
     * @see <a href="https://github.com/joansola/slamtb">Rtp.m at https://github.com/joansola/slamtb</a>
     */
    public static void transposedRotationMatrixTimesVector(
            final Quaternion q, final Point3D point, final Point3D result,
            final Matrix jacobianQ, final Matrix jacobianP) {
        final double[] coords = new double[]{
                point.getInhomX(), point.getInhomY(), point.getInhomZ()};
        final double[] rp = transposedRotationMatrixTimesVector(q, coords, jacobianQ,
                jacobianP);
        result.setInhomogeneousCoordinates(rp[0], rp[1], rp[2]);
    }

    /**
     * Rotates a 3D point by the inverse rotation obtained from a quaternion.
     *
     * @param q         quaternion containing rotation information.
     * @param point     array containing inhomogeneous 3D coordinates of a point to
     *                  be rotated.
     * @param jacobianQ jacobian wrt of quaternion. Must be 3x4.
     * @param jacobianP jacobian wrt of point. Must be 3x3.
     * @return result of rotation.
     * @throws IllegalArgumentException if jacobian of quaternions is not 3x4
     *                                  (if provided), or if jacobian of point is not 3x3 (if provided).
     * @see <a href="https://github.com/joansola/slamtb">Rtp.m at https://github.com/joansola/slamtb</a>
     */
    public static Point3D transposedRotationMatrixTimesVector(
            final Quaternion q, final Point3D point,
            final Matrix jacobianQ, final Matrix jacobianP) {
        final Point3D result = Point3D.create();
        transposedRotationMatrixTimesVector(q, point, result, jacobianQ,
                jacobianP);
        return result;
    }

    /**
     * Rotates a 3D point by the inverse rotation obtained from a quaternion.
     *
     * @param q      quaternion containing rotation information.
     * @param point  3D point to be rotated.
     * @param result result of rotation.
     * @see <a href="https://github.com/joansola/slamtb">Rtp.m at https://github.com/joansola/slamtb</a>
     */
    public static void transposedRotationMatrixTimesVector(
            final Quaternion q, final Point3D point, final Point3D result) {
        transposedRotationMatrixTimesVector(q, point, result, null, null);
    }

    /**
     * Rotates a 3D point by the inverse rotation obtained from a quaternion.
     *
     * @param q     quaternion containing rotation information.
     * @param point 3D point to be rotated.
     * @return result of rotation.
     * @see <a href="https://github.com/joansola/slamtb">Rtp.m at https://github.com/joansola/slamtb</a>
     */
    public static Point3D transposedRotationMatrixTimesVector(
            final Quaternion q, final Point3D point) {
        return transposedRotationMatrixTimesVector(q, point, null, null);
    }

    /**
     * Rotation to convert camera body to camera sensor.
     *
     * @param result instance where resulting rotation will be stored.
     */
    public static void cameraBodyToCameraSensorRotation(
            final MatrixRotation3D result) {
        Quaternion.eulerToMatrixRotation(-Math.PI / 2.0, 0, -Math.PI / 2.0,
                result);
    }

    /**
     * Rotation to convert camera body to camera sensor.
     *
     * @return a new instance containing rotation.
     */
    public static MatrixRotation3D cameraBodyToCameraSensorRotation() {
        final MatrixRotation3D result = new MatrixRotation3D();
        cameraBodyToCameraSensorRotation(result);
        return result;
    }

    /**
     * Transform quaternion Gaussian to Euler Gaussian.
     * The Gaussian quaternion is provided as N(q, Q) where q is the quaternion
     * mean and Q is the quaternion covariance, and returns an Euler angles
     * Gaussian as N(e,E) where e is the Euler angles mean and E is the Euler
     * angles covariance.
     *
     * @param q                    mean quaternion to be transformed.
     * @param quaternionCovariance quaternion covariance to be transformed.
     * @param angles               obtained mean Euler angles.
     * @param anglesCovariance     obtained Euler covariance.
     * @throws IllegalArgumentException if provided quaternion covariance is
     *                                  not 4x4
     * @see <a href="https://github.com/joansola/slamtb">q2eG.m at https://github.com/joansola/slamtb</a>
     */
    public static void quaternionToEulerGaussian(
            final Quaternion q, final Matrix quaternionCovariance, final double[] angles,
            final Matrix anglesCovariance) {
        if (quaternionCovariance.getRows() != Quaternion.N_PARAMS ||
                quaternionCovariance.getColumns() != Quaternion.N_PARAMS) {
            throw new IllegalArgumentException(
                    "quaternion covariance must be 4x4");
        }

        try {
            final Matrix eq = new Matrix(Quaternion.N_ANGLES, Quaternion.N_PARAMS);
            q.toEulerAngles(angles, eq);

            // jacobian = eq * Q * eq', where is provided covariance
            anglesCovariance.copyFrom(eq);
            anglesCovariance.multiply(quaternionCovariance);
            anglesCovariance.multiply(eq.transposeAndReturnNew());
        } catch (final WrongSizeException ignore) {
            // never thrown
        }
    }

    /**
     * Obtains the skew symmetric matrix from angular rates vector.
     *
     * @param angularRates a vector containing the 3 components (x,y,z) of
     *                     angular rates (rad/s).
     * @param result       a skew symmetric matrix.
     * @throws IllegalArgumentException if provided array of angular rates does
     *                                  not have length 3 or if provided result matrix is not 4x4.
     */
    public static void angularRatesToSkew(
            final double[] angularRates, final Matrix result) {
        if (angularRates.length != Quaternion.N_ANGLES) {
            throw new IllegalArgumentException(
                    "angular rates must have length 3");
        }
        if (result.getRows() != Quaternion.N_PARAMS ||
                result.getColumns() != Quaternion.N_PARAMS) {
            throw new IllegalArgumentException("result matrix must be 4x4");
        }

        result.setElementAt(0, 0, 0.0);
        result.setElementAt(1, 0, angularRates[0]);
        result.setElementAt(2, 0, angularRates[1]);
        result.setElementAt(3, 0, angularRates[2]);

        result.setElementAt(0, 1, -angularRates[0]);
        result.setElementAt(1, 1, 0.0);
        result.setElementAt(2, 1, -angularRates[2]);
        result.setElementAt(3, 1, angularRates[1]);

        result.setElementAt(0, 2, -angularRates[1]);
        result.setElementAt(1, 2, angularRates[2]);
        result.setElementAt(2, 2, 0.0);
        result.setElementAt(3, 2, -angularRates[0]);

        result.setElementAt(0, 3, -angularRates[2]);
        result.setElementAt(1, 3, -angularRates[1]);
        result.setElementAt(2, 3, angularRates[0]);
        result.setElementAt(3, 3, 0.0);
    }

    /**
     * Obtains the skew symmetric matrix from angular rates vector.
     *
     * @param angularRates a vector containing the 3 components (x,y,z) of
     *                     angular rates (rad/s).
     * @return a new instance containing the skew symmetric matrix
     * @throws IllegalArgumentException if provided array of angular rates does
     *                                  not have length 3.
     */
    public static Matrix angularRatesToSkew(final double[] angularRates) {
        Matrix result = null;
        try {
            result = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            angularRatesToSkew(angularRates, result);
        } catch (final WrongSizeException ignore) {
            // never thrown
        }
        return result;
    }

}
