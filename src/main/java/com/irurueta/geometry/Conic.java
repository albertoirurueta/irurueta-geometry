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
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.algebra.WrongSizeException;

import java.io.Serializable;


/**
 * This class contains the implementation of a conic.
 */
@SuppressWarnings("DuplicatedCode")
public class Conic extends BaseConic implements Serializable {

    /**
     * Constructor.
     */
    public Conic() {
        super();
    }

    /**
     * Constructor of this class. This constructor accepts every parameter
     * describing a conic (parameters a, b, c, d, e, f).
     *
     * @param a Parameter A of the conic.
     * @param b Parameter B of the conic.
     * @param c Parameter C of the conic.
     * @param d Parameter D of the conic.
     * @param e Parameter E of the conic.
     * @param f Parameter F of the conic.
     */
    public Conic(final double a, final double b, final double c, final double d, final double e, final double f) {
        super(a, b, c, d, e, f);
    }

    /**
     * This method sets the matrix used to describe a conic.
     * This matrix must be 3x3 and symmetric.
     *
     * @param m 3x3 Matrix describing the conic.
     * @throws IllegalArgumentException    Raised when the size of the matrix is
     *                                     not 3x3.
     * @throws NonSymmetricMatrixException Raised when the conic matrix is not
     *                                     symmetric.
     */
    public Conic(final Matrix m) throws NonSymmetricMatrixException {
        super(m);
    }

    /**
     * Creates conic where provided points are contained (are locus).
     *
     * @param point1 1st point.
     * @param point2 2nd point.
     * @param point3 3rd point.
     * @param point4 4th point.
     * @param point5 5th point.
     * @throws CoincidentPointsException Raised if points are coincident or
     *                                   produce a degenerated configuration.
     */
    public Conic(final Point2D point1, final Point2D point2, final Point2D point3, final Point2D point4,
                 final Point2D point5) throws CoincidentPointsException {
        setParametersFromPoints(point1, point2, point3, point4, point5);
    }

    /**
     * Checks if the given point is locus (lies within) this conic.
     *
     * @param point     Point to be checked.
     * @param threshold Threshold of distance to determine whether the
     *                  point is locus of the conic or not. Threshold might be needed because of
     *                  machine precision issues. If not provided DEFAULT_LOCUS_THRESHOLD will be
     *                  used instead.
     * @return True if the point lies within this conic, false otherwise.
     * @throws IllegalArgumentException Raised if threshold is negative.
     */
    public boolean isLocus(final Point2D point, final double threshold) {
        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        try {
            normalize();
            final var c = asMatrix();
            final var homPoint = new Matrix(Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            point.normalize();
            homPoint.setElementAt(0, 0, point.getHomX());
            homPoint.setElementAt(1, 0, point.getHomY());
            homPoint.setElementAt(2, 0, point.getHomW());
            final var locusMatrix = homPoint.transposeAndReturnNew();
            locusMatrix.multiply(c);
            locusMatrix.multiply(homPoint);

            return Math.abs(locusMatrix.getElementAt(0, 0)) < threshold;
        } catch (final WrongSizeException ignore) {
            return false;
        }
    }

    /**
     * Checks if the given point is locus (lies within) this conic.
     *
     * @param point Point to be checked.
     * @return True if the point lies within this conic, false otherwise.
     * @see #isLocus(Point2D, double)
     */
    public boolean isLocus(final Point2D point) {
        return isLocus(point, DEFAULT_LOCUS_THRESHOLD);
    }

    /**
     * Computes the angle between two 2D points using this conic as a geometry
     * base.
     *
     * @param pointA First point.
     * @param pointB Second point.
     * @return Angle between provided points given in radians..
     */
    public double angleBetweenPoints(final Point2D pointA, final Point2D pointB) {
        try {
            // retrieve conic as matrix
            normalize();
            final var c = asMatrix();
            final var transHomPointA = new Matrix(1, Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH);
            pointA.normalize();
            transHomPointA.setElementAt(0, 0, pointA.getHomX());
            transHomPointA.setElementAt(0, 1, pointA.getHomY());
            transHomPointA.setElementAt(0, 2, pointA.getHomW());

            final var tmp = transHomPointA.multiplyAndReturnNew(c);
            tmp.multiply(transHomPointA.transposeAndReturnNew()); //This is 
            // homPointA' * C * homPointA

            final var normA = tmp.getElementAt(0, 0);

            final var homPointB = new Matrix(Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            pointB.normalize();
            homPointB.setElementAt(0, 0, pointB.getHomX());
            homPointB.setElementAt(1, 0, pointB.getHomY());
            homPointB.setElementAt(2, 0, pointB.getHomW());

            homPointB.transpose(tmp);
            tmp.multiply(c);
            tmp.multiply(homPointB);

            final var normB = tmp.getElementAt(0, 0);

            transHomPointA.multiply(c);
            transHomPointA.multiply(homPointB);
            // This is homPointA' * C * homPointB

            final var angleNumerator = transHomPointA.getElementAt(0, 0);

            final var cosTheta = angleNumerator / Math.sqrt(normA * normB);
            return Math.acos(cosTheta);
        } catch (final WrongSizeException ignore) {
            // This will never happen
            return 0.0;
        }
    }

    /**
     * Checks if two points are perpendicular in the geometry base generated by
     * this conic.
     *
     * @param pointA    First point.
     * @param pointB    Second point.
     * @param threshold Threshold to determine whether the points are
     *                  perpendicular or not. If the dot product between provided points and this
     *                  conic is greater than provided threshold, then points won't be assumed to
     *                  be perpendicular. Threshold is provided because of machine precision
     *                  limits, if not provided DEFAULT_PERPENDICULAR_THRESHOLD will be used
     *                  instead.
     * @return True if points are perpendicular, false otherwise.
     * @throws IllegalArgumentException Raised if threshold is negative.
     */
    public boolean arePerpendicularPoints(final Point2D pointA, final Point2D pointB, final double threshold) {
        try {
            // retrieve conic as matrix
            final var transHomPointA = new Matrix(1, Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH);
            pointA.normalize();
            transHomPointA.setElementAt(0, 0, pointA.getHomX());
            transHomPointA.setElementAt(0, 1, pointA.getHomY());
            transHomPointA.setElementAt(0, 2, pointA.getHomW());

            final var homPointB = new Matrix(Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            pointB.normalize();
            homPointB.setElementAt(0, 0, pointB.getHomX());
            homPointB.setElementAt(1, 0, pointB.getHomY());
            homPointB.setElementAt(2, 0, pointB.getHomW());

            normalize();
            final var c = asMatrix();
            transHomPointA.multiply(c);
            transHomPointA.multiply(homPointB);
            // This is homPointA' * C * homPointB

            final var perpend = transHomPointA.getElementAt(0, 0);

            return Math.abs(perpend) < threshold;
        } catch (final WrongSizeException ignore) {
            // This will never happen
            return false;
        }
    }

    /**
     * Sets the values of the dual conic corresponding to this conic instance
     * into provided dualConic instance.
     * The dual conic is equal to the inverse of the conic matrix.
     *
     * @param dualConic Dual conic instance where the values of the dual conic
     *                  of this conic instance will be stored.
     * @throws DualConicNotAvailableException Raised if the dual conic does not
     *                                        exist because this conic instance is degenerate (its inverse cannot be
     *                                        computed).
     */
    public void dualConic(final DualConic dualConic) throws DualConicNotAvailableException {

        final var conicMatrix = asMatrix();
        try {
            final var invMatrix = com.irurueta.algebra.Utils.inverse(conicMatrix);

            // ensure that resulting matrix after inversion is symmetric
            // by computing the mean of off-diagonal elements
            final var a = invMatrix.getElementAt(0, 0);
            final var b = 0.5 * (invMatrix.getElementAt(0, 1) + invMatrix.getElementAt(1, 0));
            final var c = invMatrix.getElementAt(1, 1);
            final var d = 0.5 * (invMatrix.getElementAt(0, 2) + invMatrix.getElementAt(2, 0));
            final var e = 0.5 * (invMatrix.getElementAt(1, 2) + invMatrix.getElementAt(2, 1));
            final var f = invMatrix.getElementAt(2, 2);
            dualConic.setParameters(a, b, c, d, e, f);
        } catch (final AlgebraException e) {
            throw new DualConicNotAvailableException(e);
        }
    }

    /**
     * Computes the dual conic of this conic.
     * The dual conic is equal to the inverse of the conic matrix.
     *
     * @return A new DualConic corresponding to the dual conic of this instance.
     * @throws DualConicNotAvailableException Raised if the dual conic does not
     *                                        exist because this conic instance is degenerate (its inverse cannot be
     *                                        computed).
     */
    public DualConic getDualConic() throws DualConicNotAvailableException {
        final var dualConic = new DualConic();
        dualConic(dualConic);
        return dualConic;
    }

    /**
     * Returns the ConicType of this conic.
     *
     * @return A ConicType describing the type of this conic. It can be
     * one of the following: ELLIPSE_CONIC_TYPE, CIRCLE_CONIC_TYPE,
     * PARABOLA_CONIC_TYPE, HYPERBOLA_CONIC_TYPE and
     * RECTANGULAR_HYPERBOLA_CONIC_TYPE.
     */
    public ConicType getConicType() {
        // computes and evaluates the following expression: b^2 - 4ac
        final var expression = (b * b) - (a * c);

        if (expression < 0) {
            if (a == c && b == 0) {
                return ConicType.CIRCLE_CONIC_TYPE;
            } else {
                return ConicType.ELLIPSE_CONIC_TYPE;
            }
        } else if (expression == 0) {
            return ConicType.PARABOLA_CONIC_TYPE;
        } else {
            // expression > 0
            if ((a + c) == 0) {
                return ConicType.RECTANGULAR_HYPERBOLA_CONIC_TYPE;
            } else {
                return ConicType.HYPERBOLA_CONIC_TYPE;
            }
        }
    }

    /**
     * Sets parameters of this conic so that provided points lie within it (are
     * locus).
     *
     * @param point1 1st point.
     * @param point2 2nd point.
     * @param point3 3rd point.
     * @param point4 4th point.
     * @param point5 5th point.
     * @throws CoincidentPointsException Raised if points are coincident or
     *                                   produce a degenerated configuration.
     */
    public final void setParametersFromPoints(
            final Point2D point1, final Point2D point2, final Point2D point3, final Point2D point4,
            final Point2D point5) throws CoincidentPointsException {

        // normalize points to increase accuracy
        point1.normalize();
        point2.normalize();
        point3.normalize();
        point4.normalize();
        point5.normalize();

        try {
            // each point belonging to a conic follows equation:
            // p' * C * p = 0 ==>
            // x^2 + y^2 + w^2 + 2*x*y + 2*x*w + 2*y*w = 0
            final var m = new Matrix(5, 6);
            var x = point1.getHomX();
            var y = point1.getHomY();
            var w = point1.getHomW();
            m.setElementAt(0, 0, x * x);
            m.setElementAt(0, 1, 2.0 * x * y);
            m.setElementAt(0, 2, y * y);
            m.setElementAt(0, 3, 2.0 * x * w);
            m.setElementAt(0, 4, 2.0 * y * w);
            m.setElementAt(0, 5, w * w);
            x = point2.getHomX();
            y = point2.getHomY();
            w = point2.getHomW();
            m.setElementAt(1, 0, x * x);
            m.setElementAt(1, 1, 2.0 * x * y);
            m.setElementAt(1, 2, y * y);
            m.setElementAt(1, 3, 2.0 * x * w);
            m.setElementAt(1, 4, 2.0 * y * w);
            m.setElementAt(1, 5, w * w);
            x = point3.getHomX();
            y = point3.getHomY();
            w = point3.getHomW();
            m.setElementAt(2, 0, x * x);
            m.setElementAt(2, 1, 2.0 * x * y);
            m.setElementAt(2, 2, y * y);
            m.setElementAt(2, 3, 2.0 * x * w);
            m.setElementAt(2, 4, 2.0 * y * w);
            m.setElementAt(2, 5, w * w);
            x = point4.getHomX();
            y = point4.getHomY();
            w = point4.getHomW();
            m.setElementAt(3, 0, x * x);
            m.setElementAt(3, 1, 2.0 * x * y);
            m.setElementAt(3, 2, y * y);
            m.setElementAt(3, 3, 2.0 * x * w);
            m.setElementAt(3, 4, 2.0 * y * w);
            m.setElementAt(3, 5, w * w);
            x = point5.getHomX();
            y = point5.getHomY();
            w = point5.getHomW();
            m.setElementAt(4, 0, x * x);
            m.setElementAt(4, 1, 2.0 * x * y);
            m.setElementAt(4, 2, y * y);
            m.setElementAt(4, 3, 2.0 * x * w);
            m.setElementAt(4, 4, 2.0 * y * w);
            m.setElementAt(4, 5, w * w);

            // normalize each row to increase accuracy
            final var row = new double[6];
            double rowNorm;

            for (var j = 0; j < 5; j++) {
                m.getSubmatrixAsArray(j, 0, j, 5, row);
                rowNorm = com.irurueta.algebra.Utils.normF(row);
                for (var i = 0; i < 6; i++) {
                    m.setElementAt(j, i, m.getElementAt(j, i) / rowNorm);
                }
            }

            final var decomposer = new SingularValueDecomposer(m);
            decomposer.decompose();

            if (decomposer.getRank() < 5) {
                throw new CoincidentPointsException();
            }

            // the right null-space of m contains the parameters a, b, c, d, e ,f
            // of the conic
            final var v = decomposer.getV();

            final var a = v.getElementAt(0, 5);
            final var b = v.getElementAt(1, 5);
            final var c = v.getElementAt(2, 5);
            final var d = v.getElementAt(3, 5);
            final var e = v.getElementAt(4, 5);
            final var f = v.getElementAt(5, 5);

            setParameters(a, b, c, d, e, f);
        } catch (final AlgebraException ex) {
            throw new CoincidentPointsException(ex);
        }
    }

    /**
     * Returns a line tangent to this conic at provided point. Provided point
     * must be locus of this conic, otherwise a NotLocusException will be thrown.
     *
     * @param point a locus point of this conic.
     * @return A 2D line tangent to this conic at provided point.
     * @throws NotLocusException if provided point is not locus of this conic up
     *                           to DEFAULT_LOCUS_THRESHOLD.
     */
    public Line2D getTangentLineAt(final Point2D point) throws NotLocusException {
        final var line = new Line2D();
        tangentLineAt(point, line, DEFAULT_LOCUS_THRESHOLD);
        return line;
    }

    /**
     * Computes a line tangent to this conic at provided point. Provided point
     * must be locus of this conic, otherwise a NotLocusException will be thrown.
     *
     * @param point     a locus point of this conic.
     * @param line      instance of a 2D line where result will be stored.
     * @param threshold threshold to determine if provided point is locus.
     * @throws NotLocusException        if provided point is not locus of this conic up
     *                                  to provided threshold.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public void tangentLineAt(final Point2D point, final Line2D line, final double threshold) throws NotLocusException {
        if (!isLocus(point, threshold)) {
            throw new NotLocusException();
        }

        point.normalize();
        normalize();

        final var c = asMatrix();

        try {
            final var p = new Matrix(Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            p.setElementAt(0, 0, point.getHomX());
            p.setElementAt(1, 0, point.getHomY());
            p.setElementAt(2, 0, point.getHomW());

            c.multiply(p);
        } catch (final WrongSizeException ignore) {
            // never happens
        }

        line.setParameters(c.getElementAt(0, 0), c.getElementAt(1, 0),
                c.getElementAt(2, 0));
    }

    /**
     * Creates a canonical instance of the absolute conic in the metric stratum.
     * The absolute conic in the metric stratum is the intersection of the
     * absolute quadric with the plane at the infinity.
     * Both the absolute conic and the dual absolute conic define orthogonality
     * in the metric stratum, and in a purely metric stratum (i.e. when camera
     * is correctly calibrated), their canonical value is equal to the identity.
     *
     * @return a canonical instance of the absolute conic.
     */
    public static Conic createCanonicalAbsoluteConic() {
        return new Conic(1.0, 0.0, 1.0, 0.0, 0.0, 1.0);
    }

    //TODO: shortest distance of point to conic
    //TODO: closest point to conic
    //TODO: intersection of Line2D with Conic results in two points (page 9 PHD report.pdf)
}
