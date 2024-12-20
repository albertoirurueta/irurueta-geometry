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
import com.irurueta.algebra.SingularValueDecomposer;

import java.io.Serializable;
import java.util.Objects;

/**
 * Class defining a plane.
 * Planes can be expressed using the following expression:
 * A * x + B * y + C * z + D = 0
 */
@SuppressWarnings("DuplicatedCode")
public class Plane implements Serializable {

    /**
     * Constant defining the size of the vector that contains plane parameters.
     */
    public static final int PLANE_NUMBER_PARAMS = 4;

    /**
     * Constant defining the distance threshold to determine whether a point
     * lays inside (is locus) this plane or not.
     */
    public static final double DEFAULT_LOCUS_THRESHOLD = 1e-12;

    /**
     * Minimum allowed threshold.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Defines the threshold used when comparing two values.
     */
    public static final double DEFAULT_COMPARISON_THRESHOLD = 1e-10;

    /**
     * Machine precision.
     */
    private static final double PRECISION = 1e-12;

    /**
     * Constant defining error threshold, which is a small value close to
     * machine precision.
     */
    private static final double DEFAULT_ERROR_THRESHOLD = 1e-12;

    /**
     * Constant defining the size of vector that define the direction of a plane.
     */
    private static final int INHOM_VECTOR_SIZE = 3;

    /**
     * Parameter A of a plane.
     */
    private double a;

    /**
     * Parameter B of a plane.
     */
    private double b;

    /**
     * Parameter C of a plane.
     */
    private double c;

    /**
     * Parameter D of a plane.
     */
    private double d;

    /**
     * Defines whether the plane is already normalized or not.
     */
    private boolean normalized;

    /**
     * Default constructor of this class.
     */
    public Plane() {
        a = b = c = d = 0.0;
        normalized = false;
    }

    /**
     * Constructor.
     * This constructor accepts every parameter describing a plane in its
     * homogeneous form:
     * Ax + By + Cz + D = 0
     *
     * @param a Parameter A of this plane.
     * @param b Parameter B of this plane.
     * @param c Parameter C of this plane.
     * @param d Parameter D of this plane.
     */
    public Plane(final double a, final double b, final double c, final double d) {
        setParameters(a, b, c, d);
    }

    /**
     * Constructor.
     * This constructor accepts an array containing all the parameters (a, b, c,
     * d) describing a plane.
     *
     * @param array Array containing plane parameters.
     * @throws IllegalArgumentException Raised if length of array is not 4.
     */
    public Plane(final double[] array) {
        setParameters(array);
    }

    /**
     * Constructor.
     * This constructor accepts three 3D points and computes the plane
     * parameters so that the plane passes through provided points (they are
     * locus).
     *
     * @param pointA First 3D point to compute the plane.
     * @param pointB Second 3D point to compute the plane.
     * @param pointC Third 3D point to compute the plane.
     * @throws ColinearPointsException Raised if provided points lay in a line
     *                                 preventing a single plane to be estimated. This happens in degenerate
     *                                 configurations where points are co-linear and an infinite set of planes
     *                                 pass through them.
     */
    public Plane(final Point3D pointA, final Point3D pointB, final Point3D pointC) throws ColinearPointsException {
        setParametersFromThreePoints(pointA, pointB, pointC);
    }

    /**
     * Constructor.
     *
     * @param point   Point laying inside the plane.
     * @param vectorA First vector laying in the plane.
     * @param vectorB Second vector laying in the plane.
     * @throws IllegalArgumentException Raised if vectors length is not 3.
     * @throws ParallelVectorsException Raised if provided vectors are parallel.
     */
    public Plane(final Point3D point, final double[] vectorA, final double[] vectorB) throws ParallelVectorsException {
        setParametersFrom1PointAnd2Vectors(point, vectorA, vectorB);
    }


    /**
     * Constructor of a plane from one point and its director vector.
     *
     * @param point  Point laying inside the plane.
     * @param vector Director vector.
     * @throws IllegalArgumentException Raised if vector length is not 3.
     */
    public Plane(final Point3D point, final double[] vector) {
        setParametersFromPointAndDirectorVector(point, vector);
    }

    /**
     * Returns parameter A of this plane.
     *
     * @return Parameter A of this plane.
     */
    public double getA() {
        return a;
    }

    /**
     * Returns parameter B of this plane.
     *
     * @return Parameter B of this plane.
     */
    public double getB() {
        return b;
    }

    /**
     * Returns parameter C of this plane.
     *
     * @return Parameter C of this plane.
     */
    public double getC() {
        return c;
    }

    /**
     * Returns parameter D of this plane.
     *
     * @return Parameter D of this plane.
     */
    public double getD() {
        return d;
    }

    /**
     * Sets parameters of this plane.
     *
     * @param a Parameter A of this plane.
     * @param b Parameter B of this plane.
     * @param c Parameter C of this plane.
     * @param d Parameter D of this plane.
     */
    public final void setParameters(final double a, final double b, final double c, final double d) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
        normalized = false;
    }

    /**
     * Sets parameters of this plane.
     *
     * @param array Array containing parameters of this plane.
     * @throws IllegalArgumentException Raised if provided array does not have length equal to 4.
     */
    public final void setParameters(final double[] array) {
        if (array.length != PLANE_NUMBER_PARAMS) {
            throw new IllegalArgumentException();
        }

        a = array[0];
        b = array[1];
        c = array[2];
        d = array[3];
        normalized = false;
    }

    /**
     * Computes and sets plane parameters using provided 3D points.
     * A plane can be defined from just 3 points.
     *
     * @param pointA 1st point.
     * @param pointB 2nd point.
     * @param pointC 3rd point.
     * @throws ColinearPointsException if provided points are in a co-linear or degenerate configuration.
     */
    public final void setParametersFromThreePoints(final Point3D pointA, final Point3D pointB, final Point3D pointC)
            throws ColinearPointsException {

        // normalize points to increase accuracy
        pointA.normalize();
        pointB.normalize();
        pointC.normalize();

        // we use 3 points to find one plane
        try {
            // set homogeneous coordinates of each point on each row of the matrix
            final var m = new Matrix(3, PLANE_NUMBER_PARAMS);

            m.setElementAt(0, 0, pointA.getHomX());
            m.setElementAt(0, 1, pointA.getHomY());
            m.setElementAt(0, 2, pointA.getHomZ());
            m.setElementAt(0, 3, pointA.getHomW());

            m.setElementAt(1, 0, pointB.getHomX());
            m.setElementAt(1, 1, pointB.getHomY());
            m.setElementAt(1, 2, pointB.getHomZ());
            m.setElementAt(1, 3, pointB.getHomW());

            m.setElementAt(2, 0, pointC.getHomX());
            m.setElementAt(2, 1, pointC.getHomY());
            m.setElementAt(2, 2, pointC.getHomZ());
            m.setElementAt(2, 3, pointC.getHomW());

            final var decomposer = new SingularValueDecomposer(m);
            decomposer.decompose();

            if (decomposer.getRank() < 3) {
                // points where co-linear, and so the null-space of those 3 points
                // has dimension greater than one (a pencil of planes instead
                // of just one plane can be defined)
                throw new ColinearPointsException();
            }

            // V is a 4x4 orthonormal matrix
            final var mV = decomposer.getV();

            // last column of V will contain the right null-space of m, which is
            // the plane where provided points belong to.
            a = mV.getElementAt(0, 3);
            b = mV.getElementAt(1, 3);
            c = mV.getElementAt(2, 3);
            d = mV.getElementAt(3, 3);

            // Because V is orthonormal, its columns have norm equal to 1 and
            // there is no need to normalize this plane to increase accuracy
            normalized = true;

        } catch (final AlgebraException e) {
            // should only fail if decomposition fails for numerical reasons
            throw new ColinearPointsException(e);
        }
    }

    /**
     * Determines if provided points are co-linear or have a degenerate
     * configuration. If returned value is true, then such points cannot be used
     * to estimate a plane.
     *
     * @param pointA 1st plane.
     * @param pointB 2nd plane.
     * @param pointC 3rd plane.
     * @return true if provided points co-linear, false otherwise.
     */
    public static boolean areColinearPoints(final Point3D pointA, final Point3D pointB, final Point3D pointC) {
        // normalize points to increase accuracy
        pointA.normalize();
        pointB.normalize();
        pointC.normalize();

        // we use 3 points to find one plane
        try {
            // set homogeneous coordinates of each point on each row of the matrix
            final var m = new Matrix(3, PLANE_NUMBER_PARAMS);

            m.setElementAt(0, 0, pointA.getHomX());
            m.setElementAt(0, 1, pointA.getHomY());
            m.setElementAt(0, 2, pointA.getHomZ());
            m.setElementAt(0, 3, pointA.getHomW());

            m.setElementAt(1, 0, pointB.getHomX());
            m.setElementAt(1, 1, pointB.getHomY());
            m.setElementAt(1, 2, pointB.getHomZ());
            m.setElementAt(1, 3, pointB.getHomW());

            m.setElementAt(2, 0, pointC.getHomX());
            m.setElementAt(2, 1, pointC.getHomY());
            m.setElementAt(2, 2, pointC.getHomZ());
            m.setElementAt(2, 3, pointC.getHomW());

            final var decomposer = new SingularValueDecomposer(m);
            decomposer.decompose();

            // if points were co-linear, their null-space has dimension greater
            // than one (a pencil of planes instead of just one plane can be
            // defined)
            return (decomposer.getRank() < 3);
        } catch (final AlgebraException e) {
            return true;
        }
    }

    /**
     * Sets parameter A of this plane.
     *
     * @param a Parameter A.
     */
    public void setA(final double a) {
        this.a = a;
        normalized = false;
    }

    /**
     * Sets parameter B of this plane.
     *
     * @param b Parameter B.
     */
    public void setB(final double b) {
        this.b = b;
        normalized = false;
    }

    /**
     * Sets parameter C of this plane.
     *
     * @param c Parameter C.
     */
    public void setC(final double c) {
        this.c = c;
        normalized = false;
    }

    /**
     * Sets parameter D of this plane.
     *
     * @param d Parameter D.
     */
    public void setD(final double d) {
        this.d = d;
        normalized = false;
    }

    /**
     * Sets the parameters of a plane from one point and two vectors.
     *
     * @param point   Point laying inside the plane.
     * @param vectorA First vector laying in the plane.
     * @param vectorB Second vector laying in the plane.
     * @throws IllegalArgumentException Raised if vectors length is not 3.
     * @throws ParallelVectorsException Raised if provided vectors are parallel.
     */
    public final void setParametersFrom1PointAnd2Vectors(
            final Point3D point, final double[] vectorA, final double[] vectorB) throws ParallelVectorsException {

        if (vectorA.length != INHOM_VECTOR_SIZE || vectorB.length != INHOM_VECTOR_SIZE) {
            throw new IllegalArgumentException();
        }

        // normalize vectors to increase accuracy (we make a copy to avoid
        // changing provided arrays)
        var norm = com.irurueta.algebra.Utils.normF(vectorA);
        final var vA = ArrayUtils.multiplyByScalarAndReturnNew(vectorA, 1.0 / norm);
        norm = com.irurueta.algebra.Utils.normF(vectorB);
        final var vB = ArrayUtils.multiplyByScalarAndReturnNew(vectorB, 1.0 / norm);

        try {
            final var cross = com.irurueta.algebra.Utils.crossProduct(vA, vB);

            // check if resulting vector from cross product is too small (vectors
            // are almost parallel, and machine precision might worsen things)
            if (Math.abs(cross[0]) < DEFAULT_ERROR_THRESHOLD && Math.abs(cross[1]) < DEFAULT_ERROR_THRESHOLD
                    && Math.abs(cross[2]) < DEFAULT_ERROR_THRESHOLD) {
                throw new ParallelVectorsException();
            }

            // the point and the two vectors will define a plane computing the
            // cross product of the two vectors gives the values for (a,b,c),
            // which it is its director vector, but d is still unknown.
            // Given a point (xp, yp, zp, wp) and forcing this expression
            // point'*plane = 0 results in
            // a*xp + b*xy + c*xz + d*wp = 0
            // and solving
            // d = -(a*xp + b*xy + c*xz) / wp -> the plane is fully defined
            setParametersFromPointAndDirectorVector(point, cross);
        } catch (final AlgebraException e) {
            throw new ParallelVectorsException(e);
        }
    }

    /**
     * Sets parameters of a plane from one point and its director vector.
     *
     * @param point  Point laying inside the plane.
     * @param vector Director vector.
     * @throws IllegalArgumentException Raised if vector length is not 3.
     */
    public final void setParametersFromPointAndDirectorVector(final Point3D point, final double[] vector) {

        if (vector.length != INHOM_VECTOR_SIZE) {
            throw new IllegalArgumentException();
        }

        // normalize point to increase accuracy
        point.normalize();

        a = vector[0];
        b = vector[1];
        c = vector[2];

        d = -(a * point.getHomX() + b * point.getHomY() + c * point.getHomZ()) / point.getHomW();

        normalized = false;
    }

    /**
     * Check if provided point is locus (lays into) of the plane.
     *
     * @param point Point to be checked.
     * @return True if point is locus of this plane, false otherwise.
     */
    public boolean isLocus(final Point3D point) {
        return isLocus(point, DEFAULT_LOCUS_THRESHOLD);
    }

    /**
     * Check if provided point is locus (lays into) of the plane.
     *
     * @param point     Point to be checked.
     * @param threshold Threshold (non-negative small value) to decide if a
     *                  point is locus of this plane.
     * @return True if point is locus of this plane, false otherwise.
     * @throws IllegalArgumentException Raised if threshold is negative.
     */
    public boolean isLocus(final Point3D point, final double threshold) {
        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        // make dot product of homogeneous coordinates with plane
        // m = [x, y, z, w], P = [a, b, c, d], then
        // x * a + y * b + z * c + w * d must be very small to be locus

        point.normalize();
        normalize();

        final var dotProd = point.getHomX() * a + point.getHomY() * b + point.getHomZ() * c + point.getHomW() * d;

        return Math.abs(dotProd) < threshold;
    }

    /**
     * Distance between a plane and a 3D point. Returned distance equals to the
     * Euclidean distance between this plane and provided point but having sign.
     * Sign indicates whether point is at one side or the other of the plane.
     *
     * @param point Point whose distance to this line will be computed.
     * @return Distance between this line and provided point.
     */
    public double signedDistance(final Point3D point) {
        point.normalize();
        normalize();

        // numerator is the dot product of point and line
        final var num = point.getHomX() * a + point.getHomY() * b + point.getHomZ() * c + point.getHomW() * d;

        final var den = Math.sqrt(a * a + b * b + c * c) * point.getHomW();

        return num / den;
    }

    /**
     * Returns the point belonging to this line closest to provided point, which
     * will be located at signedDistance(Point2D) from this line.
     * If provided point belong to this line, then the same point will be
     * returned as a result.
     *
     * @param point Point to be checked.
     * @return Closest point.
     */
    public Point3D getClosestPoint(final Point3D point) {
        return getClosestPoint(point, DEFAULT_LOCUS_THRESHOLD);
    }

    /**
     * Returns the point belonging to this line closest to provided point, which
     * will be located at signedDistance(Point2D) from this line.
     * If provided point belong to this line, then the same point will be
     * returned as a result.
     *
     * @param point     Point to be checked.
     * @param threshold Threshold to determine whether point is locus of line or
     *                  not.
     * @return Closest point.
     * @throws IllegalArgumentException Raised if threshold is negative.
     */
    public Point3D getClosestPoint(final Point3D point, final double threshold) {
        final var result = Point3D.create();
        closestPoint(point, result, threshold);
        return result;
    }

    /**
     * Computes the point belonging to this plane closest to provided point,
     * which will be located at signedDistance(Point3D) from this plane.
     * If provided point belongs to this plane, then the same point will be
     * returned as a result.
     *
     * @param point  Point to be checked.
     * @param result Instance where the closest point will be stored.
     */
    public void closestPoint(final Point3D point, final Point3D result) {
        closestPoint(point, result, DEFAULT_LOCUS_THRESHOLD);
    }

    /**
     * Computes the point belonging to this plane closest to provided point,
     * which will be located at signedDistance(Point3D) from this plane.
     * If provided point belongs to this plane, then the same point will be
     * returned as a result.
     *
     * @param point     Point to be checked.
     * @param result    Instance where the closest point will be stored.
     * @param threshold threshold to determine whether a point is locus of
     *                  this plane.
     * @throws IllegalArgumentException Raised if threshold is negative.
     */
    public void closestPoint(final Point3D point, final Point3D result, final double threshold) {
        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        // normalize point to increase accuracy
        point.normalize();

        if (isLocus(point, threshold)) {
            // if point belongs to line, then it is returned as result
            result.setCoordinates(point);
            return;
        }

        // move point in director vector direction until it belongs to this plane
        // (point.getInhomX() + mA * amount) * mA + (point.getInhomY() +
        // mB * amount) * mB + (point.getInhomZ + mC * amount) * mC + mD = 0

        final var amount = -(point.getHomX() * a + point.getHomY() * b + point.getHomZ() * c + point.getHomW() * d)
                / (point.getHomW() * (a * a + b * b + c * c));
        result.setHomogeneousCoordinates(point.getHomX() + a * amount * point.getHomW(),
                point.getHomY() + b * amount * point.getHomW(),
                point.getHomZ() + c * amount * point.getHomW(),
                point.getHomW());
        result.normalize();
    }

    /**
     * Returns parameters of this plane as an array containing [a, b, c, d].
     *
     * @return Array containing all the parameters that describe this plane.
     */
    public double[] asArray() {
        final var array = new double[PLANE_NUMBER_PARAMS];
        asArray(array);
        return array;
    }

    /**
     * Stores the parameters of this plane in provided array as [a, b, c, d].
     *
     * @param array Array where parameters of this plane will be stored.
     * @throws IllegalArgumentException Raised if provided array doesn't have
     *                                  length 4.
     */
    public void asArray(final double[] array) {
        if (array.length != PLANE_NUMBER_PARAMS) {
            throw new IllegalArgumentException();
        }

        array[0] = a;
        array[1] = b;
        array[2] = c;
        array[3] = d;
    }

    /**
     * Normalizes the parameters of this line to increase the accuracy of some
     * computations.
     */
    public void normalize() {
        if (!normalized) {
            final var norm = Math.sqrt(a * a + b * b + c * c + d * d);

            if (norm > PRECISION) {
                a /= norm;
                b /= norm;
                c /= norm;
                d /= norm;

                normalized = true;
            }
        }
    }

    /**
     * Returns boolean indicating whether this plane has already been
     * normalized.
     *
     * @return True if this plane is normalized, false otherwise.
     */
    public boolean isNormalized() {
        return normalized;
    }

    /**
     * Returns director vector of this plane.
     *
     * @return Director vector of this plane.
     */
    public double[] getDirectorVector() {
        final var out = new double[INHOM_VECTOR_SIZE];
        directorVector(out);
        return out;
    }

    /**
     * Computes director vector of this plane and stores the result in provided
     * array.
     *
     * @param directorVector Array containing director vector.
     * @throws IllegalArgumentException Raised if provided array does not have
     *                                  length 3.
     */
    public void directorVector(final double[] directorVector) {
        if (directorVector.length != INHOM_VECTOR_SIZE) {
            throw new IllegalArgumentException();
        }

        directorVector[0] = a;
        directorVector[1] = b;
        directorVector[2] = c;
    }

    /**
     * Computes and returns the intersection point between this plane and the
     * other 2 provided planes.
     *
     * @param otherPlane1 other plane 1.
     * @param otherPlane2 other plane 2.
     * @return point where the three planes intersect.
     * @throws NoIntersectionException if the three planes do not intersect in
     *                                 a single point.
     */
    public Point3D getIntersection(final Plane otherPlane1, final Plane otherPlane2) throws NoIntersectionException {
        final var result = Point3D.create();
        intersection(otherPlane1, otherPlane2, result);
        return result;
    }

    /**
     * Computes the intersection point between this plane and the other 2
     * provided planes.
     *
     * @param otherPlane1 other plane 1.
     * @param otherPlane2 other plane 2.
     * @param result      point where the intersection will be stored.
     * @throws NoIntersectionException if the three planes do not intersect in
     *                                 a single point.
     */
    public void intersection(final Plane otherPlane1, final Plane otherPlane2, final Point3D result)
            throws NoIntersectionException {

        // normalize planes to increase accuracy
        normalize();
        otherPlane1.normalize();
        otherPlane2.normalize();

        // set matrix where each row contains the parameters of the plane
        try {
            final var m = new Matrix(3, 4);
            m.setElementAt(0, 0, a);
            m.setElementAt(0, 1, b);
            m.setElementAt(0, 2, c);
            m.setElementAt(0, 3, d);

            m.setElementAt(1, 0, otherPlane1.getA());
            m.setElementAt(1, 1, otherPlane1.getB());
            m.setElementAt(1, 2, otherPlane1.getC());
            m.setElementAt(1, 3, otherPlane1.getD());

            m.setElementAt(2, 0, otherPlane2.getA());
            m.setElementAt(2, 1, otherPlane2.getB());
            m.setElementAt(2, 2, otherPlane2.getC());
            m.setElementAt(2, 3, otherPlane2.getD());

            // If planes are not parallel, then matrix has rank 3, and its right
            // null-space is equal to their intersection.
            final var decomposer = new SingularValueDecomposer(m);
            decomposer.decompose();

            // planes are parallel
            if (decomposer.getRank() < 3) {
                throw new NoIntersectionException();
            }

            final var v = decomposer.getV();

            // last column of V contains the right null-space of m, which is the
            // intersection of lines expressed in homogeneous coordinates.
            // because column is already normalized by SVD decomposition, point
            // will also be normalized
            result.setHomogeneousCoordinates(v.getElementAt(0, 3), v.getElementAt(1, 3),
                    v.getElementAt(2, 3), v.getElementAt(3, 3));
        } catch (final AlgebraException e) {
            // lines are numerically unstable
            throw new NoIntersectionException(e);
        }
    }

    /**
     * Computes the dot product between the parameters A, B, C, D of this plane
     * and the ones of provided plane.
     * This method normalizes both planes to compute dot product.
     *
     * @param plane plane to compute dot product with.
     * @return dot product value.
     */
    public double dotProduct(final Plane plane) {
        normalize();
        plane.normalize();
        return a * plane.a + b * plane.b + c * plane.c + d * plane.d;
    }

    /**
     * Checks if the plane described by this instance equals provided plane
     * up to provided threshold.
     *
     * @param plane     plane to be compared to.
     * @param threshold threshold grade of tolerance to determine whether the
     *                  planes are equal or not. It is used because due to machine precision,
     *                  the values might not be exactly equal (if not provided
     *                  DEFAULT_COMPARISON_THRESHOLD is used).
     * @return true if current plane and provided one are the same, false
     * otherwise.
     * @throws IllegalArgumentException if threshold is negative.
     */
    public boolean equals(final Plane plane, final double threshold) {

        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        normalize();
        plane.normalize();

        return (1.0 - Math.abs(dotProduct(plane))) <= threshold;
    }

    /**
     * Checks if the plane described by this instance equals provided plane
     * up to default comparison threshold.
     *
     * @param plane plane to be compared to.
     * @return true if current plane and provided one are the same, false
     * otherwise.
     */
    public boolean equals(final Plane plane) {
        return equals(plane, DEFAULT_COMPARISON_THRESHOLD);
    }

    /**
     * Checks if provided object equals current plane.
     *
     * @param obj object to compare.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(final Object obj) {
        if (!(obj instanceof Plane plane)) {
            return false;
        }
        if (obj == this) {
            return true;
        }

        return equals(plane);
    }

    /**
     * Returns hash code value. This is only defined to keep the compiler happy.
     * This method must be overridden in subclasses of this class.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(a, b, c, d);
    }

    /**
     * Creates a new instance of a plane located the canonical infinity.
     * The canonical infinity corresponds to all 3D points located at infinity
     * (i.e. M = (X,Y,Z,W = 0), hence P = (A = 0,B = 0,C = 0, W = 1))
     *
     * @return a new instance of a plane located at the canonical infinity.
     */
    public static Plane createCanonicalPlaneAtInfinity() {
        final var p = new Plane();
        setAsCanonicalPlaneAtInfinity(p);
        return p;
    }

    /**
     * Sets provided plane into the canonical infinity.
     * The canonical infinity corresponds to all 3D points located at infinity
     * (i.e. M = (X,Y,Z,W = 0), hence P = (A = 0,B = 0,C = 0, W = 1))
     *
     * @param plane plane to be set at infinity.
     */
    public static void setAsCanonicalPlaneAtInfinity(final Plane plane) {
        plane.a = plane.b = plane.c = 0.0;
        plane.d = 1.0;
        plane.normalized = true;
    }
}
