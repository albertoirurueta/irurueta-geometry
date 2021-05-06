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

import java.io.Serializable;

/**
 * This class defines a circle.
 */
public class Circle implements Serializable {

    /**
     * Constant defining minimum allowed radius.
     */
    public static final double MIN_RADIUS = 0.0;

    /**
     * Constant defining default threshold value used when none is provided.
     */
    public static final double DEFAULT_THRESHOLD = 1e-9;

    /**
     * Constant defining minimum allowed threshold.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Constant defining machine precision.
     */
    public static final double EPS = 1e-12;

    /**
     * Center of circle.
     */
    private Point2D mCenter;

    /**
     * Radius of circle.
     */
    private double mRadius;

    /**
     * Empty constructor.
     * Creates circle located at space origin (0,0) with radius 1.0.
     */
    public Circle() {
        mCenter = Point2D.create();
        mRadius = 1.0;
    }

    /**
     * Constructor.
     * Sets center and radius of circle.
     *
     * @param center Center of circle.
     * @param radius Radius of circle.
     * @throws IllegalArgumentException Raised if provided radius is negative.
     */
    public Circle(final Point2D center, final double radius) {
        setCenterAndRadius(center, radius);
    }

    /**
     * Constructor.
     * Computes a circle by using three points that must belong to its locus.
     *
     * @param point1 point 1.
     * @param point2 point 2.
     * @param point3 point 3.
     * @throws ColinearPointsException if provided set of points are coincident
     *                                 or co-linear (form a single line). In such cases a singularity occurs
     *                                 since a circle having an infinite radius would be required to contain all
     *                                 three points in its locus.
     */
    public Circle(final Point2D point1, final Point2D point2, final Point2D point3)
            throws ColinearPointsException {
        setParametersFromPoints(point1, point2, point3);
    }

    /**
     * Constructor.
     * Computes a circle from a valid conic corresponding to a circle.
     *
     * @param conic a conic to create a circle from.
     * @throws IllegalArgumentException if provided conic is not a circle.
     */
    public Circle(final Conic conic) {
        setFromConic(conic);
    }

    /**
     * Returns center of circle.
     *
     * @return Center of circle.
     */
    public Point2D getCenter() {
        return mCenter;
    }

    /**
     * Sets center of circle.
     *
     * @param center Center of circle.
     * @throws NullPointerException Raised if provided center is null.
     */
    public void setCenter(final Point2D center) {
        if (center == null) {
            throw new NullPointerException();
        }
        mCenter = center;
    }

    /**
     * Returns radius of circle.
     *
     * @return Radius of circle.
     */
    public double getRadius() {
        return mRadius;
    }

    /**
     * Sets radius of circle.
     *
     * @param radius Radius of circle.
     * @throws IllegalArgumentException Raised if provided radius is negative.
     */
    public void setRadius(final double radius) {
        if (radius < MIN_RADIUS) {
            throw new IllegalArgumentException();
        }

        mRadius = radius;
    }

    /**
     * Sets center and radius of this circle.
     *
     * @param center Center to be set.
     * @param radius Radius to be set.
     * @throws IllegalArgumentException Raised if provided radius is negative.
     * @throws NullPointerException     Raised if provided center is null.
     */
    public final void setCenterAndRadius(final Point2D center, final double radius) {
        setRadius(radius);
        setCenter(center);
    }

    /**
     * Sets parameters of a circle by using three points that must belong to its
     * locus.
     *
     * @param point1 point 1.
     * @param point2 point 2.
     * @param point3 point 3.
     * @throws ColinearPointsException if provided set of points are coincident
     *                                 or co-linear (form a single line). In such cases a singularity occurs
     *                                 since a circle having an infinite radius would be required to contain all
     *                                 three points in its locus.
     */
    public final void setParametersFromPoints(
            final Point2D point1, final Point2D point2,
            final Point2D point3) throws ColinearPointsException {

        // normalize points to increase accuracy
        point1.normalize();
        point2.normalize();
        point3.normalize();

        try {
            final Matrix m = new Matrix(3, 3);
            final double[] b = new double[3];

            // 1st point
            double x = point1.getHomX();
            double y = point1.getHomY();
            double w = point1.getHomW();
            m.setElementAt(0, 0, 2.0 * x * w);
            m.setElementAt(0, 1, 2.0 * y * w);
            m.setElementAt(0, 2, w * w);
            b[0] = -x * x - y * y;

            // 2nd point
            x = point2.getHomX();
            y = point2.getHomY();
            w = point2.getHomW();
            m.setElementAt(1, 0, 2.0 * x * w);
            m.setElementAt(1, 1, 2.0 * y * w);
            m.setElementAt(1, 2, w * w);
            b[1] = -x * x - y * y;

            // 3rd point
            x = point3.getHomX();
            y = point3.getHomY();
            w = point3.getHomW();
            m.setElementAt(2, 0, 2.0 * x * w);
            m.setElementAt(2, 1, 2.0 * y * w);
            m.setElementAt(2, 2, w * w);
            b[2] = -x * x - y * y;

            // normalize each row to increase accuracy
            final double[] row = new double[3];
            double rowNorm;

            for (int j = 0; j < 3; j++) {
                m.getSubmatrixAsArray(j, 0, j, 2, row);
                rowNorm = com.irurueta.algebra.Utils.normF(row);
                for (int i = 0; i < 3; i++) {
                    m.setElementAt(j, i, m.getElementAt(j, i) / rowNorm);
                }
                b[j] /= rowNorm;
            }

            final double[] params = com.irurueta.algebra.Utils.solve(m, b);

            // d = -cx
            final double d = params[0];
            // e = -cy
            final double e = params[1];
            // f = cx^2 + cy^2 - R^2
            final double f = params[2];

            // compute center
            final double inhomCx = -d;
            final double inhomCy = -e;
            final Point2D center = new InhomogeneousPoint2D(inhomCx, inhomCy);

            // compute radius
            final double radius = Math.sqrt(inhomCx * inhomCx +
                    inhomCy * inhomCy - f);

            setCenterAndRadius(center, radius);
        } catch (final AlgebraException e) {
            throw new ColinearPointsException(e);
        }
    }

    /**
     * Returns area of a circle having provided radius.
     *
     * @param radius Radius of a circle.
     * @return Area of a circle having provided radius.
     * @throws IllegalArgumentException Raised if provided radius is negative.
     */
    public static double area(final double radius) {
        if (radius < MIN_RADIUS) {
            throw new IllegalArgumentException();
        }
        return Math.PI * radius * radius;
    }

    /**
     * Returns area of this circle.
     *
     * @return Area of this circle.
     */
    public double getArea() {
        return area(mRadius);
    }

    /**
     * Returns perimeter of a circle having provided radius.
     *
     * @param radius Radius of a circle.
     * @return Perimeter of a circle having provided radius.
     * @throws IllegalArgumentException Raised if provided radius is negative.
     */
    public static double perimeter(final double radius) {
        if (radius < MIN_RADIUS) {
            throw new IllegalArgumentException();
        }
        return 2.0 * Math.PI * radius;
    }

    /**
     * Returns perimeter of this circle.
     *
     * @return Perimeter of this circle.
     */
    public double getPerimeter() {
        return perimeter(mRadius);
    }

    /**
     * Gets curvature for provided radius.
     * Curvature of a circle is always the reciprocal of its radius.
     *
     * @param radius radius of a circle.
     * @return curvature of a circle.
     */
    public static double curvature(final double radius) {
        return 1.0 / radius;
    }

    /**
     * Gets curvature of radius at provided point.
     * Curvature of a circle is always the reciprocal of its radius.
     *
     * @return curvature of circle.
     */
    public double getCurvature() {
        return curvature(mRadius);
    }

    /**
     * Determines if provided point is inside this circle or not up to a certain
     * threshold.
     * If provided threshold is positive, the circle behaves as if it was a
     * larger circle increased by threshold amount, if provided threshold is
     * negative, the circle behaves as if it was a smaller circle decreased by
     * threshold amount in radius.
     *
     * @param point     Point to be checked.
     * @param threshold Threshold to determine if point is inside or not.
     * @return True if point is considered to be inside this circle, false
     * otherwise.
     */
    public boolean isInside(final Point2D point, final double threshold) {
        return point.distanceTo(mCenter) - threshold <= mRadius;
    }

    /**
     * Determines if provided point is inside this circle or not.
     *
     * @param point Point to be checked.
     * @return True if point is considered to be inside this circle, false
     * otherwise.
     */
    public boolean isInside(final Point2D point) {
        return isInside(point, 0.0);
    }

    /**
     * Returns distance from provided point to the closest point located in the
     * circle boundary.
     * Returned distance will be negative when point is inside of circle, and
     * positive otherwise.
     *
     * @param point Point to be checked.
     * @return Distance from point to circle boundary.
     */
    public double getSignedDistance(final Point2D point) {
        return signedDistance(this, point);
    }

    /**
     * Returns distance from provided point to the closest point located in
     * provided circle boundary.
     * Returned distance will be negative when point is inside of circle, and
     * positive otherwise.
     *
     * @param circle A circle.
     * @param point  Point to be checked.
     * @return Distance from point to provided circle boundary.
     */
    public static double signedDistance(final Circle circle, final Point2D point) {
        return point.distanceTo(circle.getCenter()) - circle.getRadius();
    }

    /**
     * Returns distance from provided point to the closest point located in the
     * circle boundary.
     *
     * @param point Point to be checked.
     * @return Distance from point to circle boundary.
     */
    public double getDistance(final Point2D point) {
        return Math.abs(getSignedDistance(point));
    }

    /**
     * Returns distance from provided point to the closest point located in
     * provided circle boundary.
     *
     * @param circle A circle.
     * @param point  Point to be checked.
     * @return Distance from point to provided circle boundary.
     */
    public static double distance(final Circle circle, final Point2D point) {
        return Math.abs(signedDistance(circle, point));
    }

    /**
     * Returns closest point to provided point that is located in this circle
     * boundary.
     *
     * @param point A point to be checked.
     * @return Closest point laying in circle boundary.
     * @throws UndefinedPointException Raised if provided point is at circle
     *                                 center or very close to it.
     */
    public Point2D getClosestPoint(final Point2D point)
            throws UndefinedPointException {
        final Point2D result = Point2D.create();
        closestPoint(point, result);
        return result;
    }

    /**
     * Computes closest point to provided point that is located in this circle
     * boundary and stores the result in provided result instance.
     *
     * @param point  A point to be checked.
     * @param result Instance where result will be stored.
     * @throws UndefinedPointException Raised if provided point is at circle
     *                                 center or very close to ti.
     */
    public void closestPoint(final Point2D point, final Point2D result)
            throws UndefinedPointException {

        double directionX = point.getInhomX() - mCenter.getInhomX();
        double directionY = point.getInhomY() - mCenter.getInhomY();
        // normalize direction and multiply by radius to set result as locus of
        // circle
        final double norm = Math.sqrt(directionX * directionX +
                directionY * directionY);

        // check if point is at center or very close to center, in that case the
        // closest point cannot be found (would be all points of a circle)
        if (norm < EPS) {
            throw new UndefinedPointException();
        }

        directionX *= mRadius / norm;
        directionY *= mRadius / norm;

        result.setInhomogeneousCoordinates(mCenter.getInhomX() + directionX,
                mCenter.getInhomY() + directionY);
    }

    /**
     * Determines whether provided point lies at circle boundary or not up to
     * a certain threshold.
     *
     * @param point     Point to be checked.
     * @param threshold A small threshold to determine whether point lies at
     *                  circle boundary.
     * @return True if point lies at circle boundary, false otherwise.
     * @throws IllegalArgumentException Raised if provided threshold is
     *                                  negative.
     */
    public boolean isLocus(final Point2D point, final double threshold) {
        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        return Math.abs(point.distanceTo(mCenter) - mRadius) <= threshold;
    }

    /**
     * Determines whether provided point lies at circle boundary or not.
     *
     * @param point Point to be checked.
     * @return True if point lies at circle boundary, false otherwise.
     */
    public boolean isLocus(final Point2D point) {
        return isLocus(point, DEFAULT_THRESHOLD);
    }

    /**
     * Returns a line tangent to this circle at provided point. Provided point
     * must be locus of this circle, otherwise a NotLocusException will be
     * thrown.
     *
     * @param point a locus point of this circle.
     * @return a 2D line tangent to this circle at provided point.
     * @throws NotLocusException if provided point is not locus of this circle
     *                           up to DEFAULT_THRESHOLD.
     */
    public Line2D getTangentLineAt(final Point2D point) throws NotLocusException {
        return getTangentLineAt(point, DEFAULT_THRESHOLD);
    }

    /**
     * Returns a line tangent to this circle at provided point. Provided point
     * must be locus of this circle, otherwise a NotLocusException will be
     * thrown.
     *
     * @param point     a locus point of this circle.
     * @param threshold threshold to determine if provided point is locus.
     * @return a 2D line tangent to this circle at provided point.
     * @throws NotLocusException        if provided point is not locus of this circle
     *                                  up to provided threshold.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public Line2D getTangentLineAt(final Point2D point, final double threshold)
            throws NotLocusException {

        final Line2D line = new Line2D();
        tangentLineAt(point, line, threshold);
        return line;
    }

    /**
     * Computes a line tangent to this circle at provided point. Provided point
     * must be locus of this circle, otherwise a NotLocusException will be
     * thrown.
     *
     * @param point     a locus point of this circle.
     * @param line      instance of a 2D line where result will be stored.
     * @param threshold threshold to determine if provided point is locus.
     * @throws NotLocusException        if provided point is not locus of this circle
     *                                  up to provided threshold.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public void tangentLineAt(final Point2D point, final Line2D line,
                              final double threshold) throws NotLocusException {

        if (!isLocus(point, threshold)) {
            throw new NotLocusException();
        }

        point.normalize();
        mCenter.normalize();

        // C =   [1  0   d]
        //       [0  1   e]
        //       [d  e   f]

        // Hence line is l = C * p, where C is the circle conic and p is a
        // point in the locus of the circle

        final double homX = point.getHomX();
        final double homY = point.getHomY();
        final double homW = point.getHomW();
        final double cx = mCenter.getInhomX();
        final double cy = mCenter.getInhomY();
        final double conicD = -cx;
        final double conicE = -cy;
        final double conicF = cx * cx + cy * cy - mRadius * mRadius;

        final double lineA = homX + conicD * homW;
        final double lineB = homY + conicE * homW;
        final double lineC = conicD * homX + conicE * homY + conicF * homW;
        line.setParameters(lineA, lineB, lineC);
    }

    /**
     * Converts this circle into a conic.
     * Conics are a more general representation of circles.
     *
     * @return A conic representing this circle.
     */
    public Conic toConic() {
        mCenter.normalize();
        // use inhomogeneous center coordinates
        final double cx = mCenter.getInhomX();
        final double cy = mCenter.getInhomY();

        final double a = 1.0;
        final double b = 0.0;
        final double c = 1.0;
        final double d = -cx;
        final double e = -cy;
        final double f = cx * cx + cy * cy - mRadius * mRadius;

        return new Conic(a, b, c, d, e, f);
    }

    /**
     * Set parameters of this circle from a valid conic corresponding to a
     * circle.
     *
     * @param conic conic to set parameters from.
     * @throws IllegalArgumentException if provided conic is not a circle.
     */
    public final void setFromConic(final Conic conic) {
        if (conic.getConicType() != ConicType.CIRCLE_CONIC_TYPE) {
            throw new IllegalArgumentException();
        }

        conic.normalize();

        final double a = conic.getA();
        // normalize parameters so that a = 1.0, d = -cx, e = -cy and
        // f = cx^ + cy^2 - r^
        final double normD = conic.getD() / a;
        final double normE = conic.getE() / a;
        final double normF = conic.getF() / a;

        final double cx = -normD;
        final double cy = -normE;
        final double radius = Math.sqrt(cx * cx + cy * cy - normF);

        mCenter = new InhomogeneousPoint2D(cx, cy);
        mRadius = radius;
    }
}
