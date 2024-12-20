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
import com.irurueta.algebra.Utils;

import java.io.Serializable;

/**
 * This class defines a lines in 3D space.
 * A line in 3D space is defined as the intersection of two non-parallel planes.
 */
public class Line3D implements Serializable {

    /**
     * Positive threshold determine whether points lay inside (is locus) of a
     * given line or not.
     */
    public static final double DEFAULT_LOCUS_THRESHOLD = 1e-12;

    /**
     * Minimum allowed threshold
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Constant defining the size of vector that define the direction of a line
     */
    private static final int INHOM_VECTOR_SIZE = 3;

    /**
     * 1st plane forming this 3D line.
     */
    private Plane plane1;

    /**
     * 2nd plane forming this 3D line.
     */
    private Plane plane2;

    /**
     * Constructor.
     * Sets planes intersecting into this 3D line.
     *
     * @param plane1 1st plane.
     * @param plane2 2nd plane.
     * @throws CoincidentPlanesException Raised if provided planes are
     *                                   coincident and hence never intersect. Notice that parallel planes are
     *                                   not coincident and they intersect at infinity.
     */
    public Line3D(final Plane plane1, final Plane plane2) throws CoincidentPlanesException {
        setPlanes(plane1, plane2);
    }

    /**
     * Constructor.
     * Builds a 3D line passing through provided 2 points.
     *
     * @param point1 1st point.
     * @param point2 2nd point.
     * @throws CoincidentPointsException Raised if provided points are
     *                                   considered to be equal.
     */
    public Line3D(final Point3D point1, final Point3D point2) throws CoincidentPointsException {
        setPlanesFromPoints(point1, point2);
    }

    /**
     * Determines whether provided planes are coincident.
     * Two planes are considered coincident if they are equal up to scale.
     *
     * @param plane1 1st plane.
     * @param plane2 2nd plane.
     * @return True if planes are coincident, false otherwise.
     */
    public static boolean areCoincidentPlanes(final Plane plane1, final Plane plane2) {
        // normalize planes to increase accuracy
        plane1.normalize();
        plane2.normalize();

        try {
            final var m = new Matrix(2, Plane.PLANE_NUMBER_PARAMS);
            m.setElementAt(0, 0, plane1.getA());
            m.setElementAt(0, 1, plane1.getB());
            m.setElementAt(0, 2, plane1.getC());
            m.setElementAt(0, 3, plane1.getD());

            m.setElementAt(1, 0, plane2.getA());
            m.setElementAt(1, 1, plane2.getB());
            m.setElementAt(1, 2, plane2.getC());
            m.setElementAt(1, 3, plane2.getD());

            // check that matrix rank is 2 (and its nullity is also 2), in that
            // case its right null-space has dimension two, which is linear
            // combination of two 3D points (a line in 3D), and line is defined.
            // Note that rank cannot be greater than 2 because matrix m has only
            // 2 rows.
            // If rank is smaller than 2, then the 2 planes are parallel, and the
            // null-space has at least dimension 3, which is a perpendicular
            // plane or the whole space (Depending whether nullity is 3 or 4)
            return Utils.rank(m) < 2;
        } catch (final AlgebraException e) {
            // if for numerical reasons it cannot be determined whether planes
            // are parallel, it will be assumed the worst case, which is that
            // planes are parallel, which is something common for very large
            // values or planes close at infinity
            return true;
        }
    }

    /**
     * Sets intersecting planes for this 3D line.
     * The intersection of provided planes will determine this 3D line.
     *
     * @param plane1 1st plane.
     * @param plane2 2nd plane.
     * @throws CoincidentPlanesException Raised if provided planes are
     *                                   coincident. Notice that parallel planes are not coincident and they
     *                                   intersect at infinity.
     */
    public final void setPlanes(final Plane plane1, final Plane plane2) throws CoincidentPlanesException {
        if (areCoincidentPlanes(plane1, plane2)) {
            throw new CoincidentPlanesException();
        }

        this.plane1 = plane1;
        this.plane2 = plane2;
    }

    /**
     * Sets planes of this 3D line so that it passes through provided points.
     *
     * @param point1 1st point.
     * @param point2 2nd point.
     * @throws CoincidentPointsException Raised if provided points are equal
     *                                   and hence a line cannot be determined.
     */
    public final void setPlanesFromPoints(final Point3D point1, final Point3D point2) throws CoincidentPointsException {
        // build matrix containing director vector on a row
        try {
            final var m = new Matrix(1, INHOM_VECTOR_SIZE);
            m.setElementAt(0, 0, point2.getInhomX() - point1.getInhomX());
            m.setElementAt(0, 1, point2.getInhomY() - point1.getInhomY());
            m.setElementAt(0, 2, point2.getInhomZ() - point1.getInhomZ());

            // m matrix will have rank 1 and nullity 2. The null-space will be
            // formed by two vectors perpendicular to the director vector
            final var decomposer = new SingularValueDecomposer(m);
            decomposer.decompose();

            // check that points are not coincident, and hence all the values of
            // matrix m are not zero
            if (decomposer.getRank() < 1) {
                throw new CoincidentPointsException();
            }

            // last two columns of V contains director vectors of plane1 and
            // plane2
            final var v = decomposer.getV();

            final var directorVector1 = v.getSubmatrixAsArray(0, 1, 2, 1);
            final var directorVector2 = v.getSubmatrixAsArray(0, 2, 2, 2);

            plane1 = new Plane(point1, directorVector1);
            plane2 = new Plane(point1, directorVector2);
        } catch (final AlgebraException e) {
            throw new CoincidentPointsException(e);
        }
    }

    /**
     * Returns 1st plane determining this 3D line.
     * The intersection of plane1 and plane2 determines this 3D line.
     *
     * @return 1st plane.
     */
    public Plane getPlane1() {
        return plane1;
    }

    /**
     * Returns 2nd plane determining this 3D line.
     * The intersection of plane1 and plane2 determines this 3D line.
     *
     * @return 2nd plane.
     */
    public Plane getPlane2() {
        return plane2;
    }

    /**
     * Determines if provided point is locus of this 3D line up to provided
     * threshold.
     *
     * @param point     Point to be checked.
     * @param threshold Threshold to determine if provided point is locus or
     *                  not. This should usually be a small value.
     * @return True if provided point belongs to this 3D line, false otherwise.
     * @throws IllegalArgumentException Raised if provided threshold is negative.
     */
    public boolean isLocus(final Point3D point, final double threshold) {
        return plane1.isLocus(point, threshold) && plane2.isLocus(point, threshold);
    }

    /**
     * Raised if provided point is locus of this 3D line.
     *
     * @param point Point to be checked.
     * @return True if provided point belongs to this 3D line, false otherwise.
     */
    public boolean isLocus(final Point3D point) {
        return isLocus(point, DEFAULT_LOCUS_THRESHOLD);
    }

    /**
     * Returns shortest distance of provided point to this 3D line.
     * The shortest distance is obtained in perpendicular direction of this
     * line.
     *
     * @param point Point to be checked.
     * @return Shortest distance of provided point to this 3D line.
     */
    public double getDistance(final Point3D point) {
        final var closestPoint = getClosestPoint(point);
        return point.distanceTo(closestPoint);
    }

    /**
     * Returns closest point belonging to this 3D line respect provided point.
     *
     * @param point Point to be checked.
     * @return Closest point belonging to this 3D line respect provided point.
     */
    public Point3D getClosestPoint(final Point3D point) {
        return getClosestPoint(point, DEFAULT_LOCUS_THRESHOLD);
    }

    /**
     * Returns closest point belonging to this 3D line respect provided point
     * up to provided threshold.
     *
     * @param point     Point to be checked.
     * @param threshold Threshold to determine the closest point.
     * @return Closest point belonging to this 3D line respect provided point.
     * @throws IllegalArgumentException Raised if provided threshold is negative.
     */
    public Point3D getClosestPoint(final Point3D point, final double threshold) {
        final var result = Point3D.create();
        closestPoint(point, result, threshold);
        return result;
    }

    /**
     * Computes closest point belonging to this 3D line respect provided point
     * and stores the result in provided instance.
     *
     * @param point  Point to be checked.
     * @param result Instance where computed point will be stored.
     */
    public void closestPoint(final Point3D point, final Point3D result) {
        closestPoint(point, result, DEFAULT_LOCUS_THRESHOLD);
    }

    /**
     * Computes closest point belonging to this 3D line respect provided point
     * up to provided threshold and stores the result in provided instance.
     *
     * @param point     Point to be checked.
     * @param result    Instance where computed point will be stored.
     * @param threshold Threshold to determine the closest point.
     * @throws IllegalArgumentException Raised if provided threshold is negative.
     */
    public void closestPoint(final Point3D point, final Point3D result, final double threshold) {
        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        // normalize to increase accuracy
        point.normalize();

        // compute director vector perpendicular to director vectors of plane1
        // and plane2, and use it along provided point to set a 3rd plane.
        // Using plane1, plane2 and the 3rd plane, we can get their intersection
        // to obtain a point which will be locus of this line3 and will be
        // located at shortest distance of provided point to this line3.

        // This is a plane having as director vector this line 3D and passing
        // through provided point
        final var p = new Plane(point, getDirection());
        try {
            intersection(p, result);
        } catch (final NoIntersectionException ignore) {
            // never happens
        }
    }

    /**
     * Normalize the planes forming this 3D line.
     */
    public void normalize() {
        plane1.normalize();
        plane2.normalize();
    }

    /**
     * Determines whether the planes forming this 3D line are normalized or not.
     *
     * @return True if planes forming this 3D line are normalized, false
     * otherwise.
     */
    public boolean isNormalized() {
        return plane1.isNormalized() && plane2.isNormalized();
    }

    /**
     * Returns array containing vector that indicates the direction of this 3D
     * line.
     *
     * @return Returns vector indicating the direction of this 3D line.
     */
    public double[] getDirection() {
        try {
            return Utils.crossProduct(plane1.getDirectorVector(), plane2.getDirectorVector());
        } catch (final AlgebraException ignore) {
            return null;
        }
    }

    /**
     * Returns point where provided point intersects this 3D line.
     *
     * @param plane Plane to intersect this 3D line.
     * @return Point where provided point intersects this 3D line.
     * @throws NoIntersectionException Raised if provided plane does not
     *                                 intersect this 3D line.
     */
    public Point3D getIntersection(final Plane plane) throws NoIntersectionException {
        final var result = Point3D.create();
        intersection(plane, result);
        return result;
    }

    /**
     * Computes point where provided point intersects this 3D line and stores
     * the result in provided instance.
     *
     * @param plane  Plane to intersect this 3D line.
     * @param result Instance where computes point will be stored.
     * @throws NoIntersectionException Raised if provided plane does not
     *                                 intersect this 3D line.
     */
    public void intersection(final Plane plane, final Point3D result) throws NoIntersectionException {
        // use plane1, plane2 and provided plane to find an intersection
        plane.intersection(plane1, plane2, result);
    }
}
