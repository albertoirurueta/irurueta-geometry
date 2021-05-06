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

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * This class defines a triangle in the 2D space.
 */
@SuppressWarnings("DuplicatedCode")
public class Triangle2D implements Serializable {

    /**
     * Default threshold value. Thresholds are used to determine whether a point
     * lies inside the triangle or not, or if it's locus or not, etc.
     */
    public static final double DEFAULT_THRESHOLD = 1e-9;

    /**
     * Minimum allowed threshold value.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Constant defining number of vertices on a triangle.
     */
    public static final int NUM_VERTICES = 3;

    /**
     * 1st vertex of this triangle.
     */
    private Point2D mVertex1;

    /**
     * 2nd vertex of this triangle.
     */
    private Point2D mVertex2;

    /**
     * 3rd vertex of this triangle.
     */
    private Point2D mVertex3;

    /**
     * Constructor.
     *
     * @param vertex1 1st vertex.
     * @param vertex2 2nd vertex.
     * @param vertex3 3rd vertex.
     * @throws NullPointerException Raised if any of the vertices is null.
     */
    public Triangle2D(final Point2D vertex1, final Point2D vertex2, final Point2D vertex3) {
        setVertices(vertex1, vertex2, vertex3);
    }

    /**
     * Returns 1st vertex of this triangle.
     *
     * @return 1st vertex.
     */
    public Point2D getVertex1() {
        return mVertex1;
    }

    /**
     * Sets 1st vertex of this triangle.
     *
     * @param vertex1 1st vertex.
     * @throws NullPointerException Raised if provided vertex is null.
     */
    public void setVertex1(final Point2D vertex1) {
        if (vertex1 == null) {
            throw new NullPointerException();
        }
        this.mVertex1 = vertex1;
    }

    /**
     * Returns 2nd vertex of this triangle.
     *
     * @return 2nd vertex.
     */
    public Point2D getVertex2() {
        return mVertex2;
    }

    /**
     * Sets 2nd vertex of this triangle.
     *
     * @param vertex2 2nd vertex.
     * @throws NullPointerException Raised if provided vertex is null.
     */
    public void setVertex2(final Point2D vertex2) {
        if (vertex2 == null) {
            throw new NullPointerException();
        }
        this.mVertex2 = vertex2;
    }

    /**
     * Returns 3rd vertex of this triangle.
     *
     * @return 3rd vertex.
     */
    public Point2D getVertex3() {
        return mVertex3;
    }

    /**
     * Sets 3rd vertex of this triangle.
     *
     * @param vertex3 3rd vertex.
     * @throws NullPointerException Raised if provided vertex is null.
     */
    public void setVertex3(final Point2D vertex3) {
        if (vertex3 == null) {
            throw new NullPointerException();
        }
        this.mVertex3 = vertex3;
    }

    /**
     * Returns vertices of this triangle as a list of points.
     *
     * @return Vertices of this triangle.
     */
    public List<Point2D> getVertices() {
        final List<Point2D> vertices = new ArrayList<>(NUM_VERTICES);
        vertices(vertices);
        return vertices;
    }

    /**
     * Stores vertices of this triangle in provided list. Note that content of
     * list will be cleared before storing this triangle's vertices.
     *
     * @param result list where vertices will be stored.
     */
    public void vertices(final List<Point2D> result) {
        result.clear();
        result.add(mVertex1);
        result.add(mVertex2);
        result.add(mVertex3);
    }

    /**
     * Sets all vertices of this triangle.
     *
     * @param vertex1 1st vertex.
     * @param vertex2 2nd vertex.
     * @param vertex3 3rd vertex.
     * @throws NullPointerException Raised if any of the vertices is null.
     */
    public final void setVertices(
            final Point2D vertex1, final Point2D vertex2, final Point2D vertex3) {
        if (vertex1 == null || vertex2 == null || vertex3 == null) {
            throw new NullPointerException();
        }

        mVertex1 = vertex1;
        mVertex2 = vertex2;
        mVertex3 = vertex3;
    }

    /**
     * Returns area of provided triangle with sign. If vertices are defined
     * clockwise area is positive, otherwise returned area is negative.
     *
     * @param triangle Triangle to be evaluated.
     * @return Area of triangle with sign. Positive sign indicates that vertices
     * are clockwise, negative sign indicates that vertices are counterclockwise.
     */
    public static double signedArea(final Triangle2D triangle) {
        return signedArea(triangle.getVertex1(), triangle.getVertex2(),
                triangle.getVertex3());
    }

    /**
     * Returns area with sign of the triangle formed by provided vertices. If
     * vertices are defined clockwise area is negative, otherwise returned area
     * is positive.
     *
     * @param vertex1 1st vertex of a triangle.
     * @param vertex2 2nd vertex of a triangle.
     * @param vertex3 3rd vertex of a triangle.
     * @return Area of a triangle with sign. Negative sign indicates that
     * vertices are clockwise, positive sign indicates that vertices are
     * counterclockwise.
     */
    public static double signedArea(
            final Point2D vertex1, final Point2D vertex2, final Point2D vertex3) {
        // The signed area of a triangle is half the determinant of its vectors,
        // or half the modulus of the cross product of its vectors

        // Hence, having the vectors of the triangle defined as:
        // v1 = vertex2 - vertex1, and v2 = vertex3 - vertex1, then:
        final double p1x = vertex1.getInhomX();
        final double p1y = vertex1.getInhomY();

        final double x1 = vertex2.getInhomX() - p1x;
        final double y1 = vertex2.getInhomY() - p1y;

        final double x2 = vertex3.getInhomX() - p1x;
        final double y2 = vertex3.getInhomY() - p1y;

        // Considering the matrix:
        // [x1   x2]
        // [y1   y2]
        // Then half its determinant or half the cross product of its column
        // vectors is:
        return 0.5 * (x1 * y2 - x2 * y1);
    }

    /**
     * Returns area of this triangle with sign. If vertices are defined
     * clockwise area is positive, otherwise returned area is negative.
     *
     * @return Area of this triangle with sign. Positive sign indicates that
     * vertices are clockwise, negative sign indicates that vertices are
     * counterclockwise.
     */
    public double getSignedArea() {
        return signedArea(mVertex1, mVertex2, mVertex3);
    }

    /**
     * Returns area of provided triangle.
     *
     * @param triangle Triangle to be checked.
     * @return Area of triangle.
     */
    public static double area(final Triangle2D triangle) {
        return area(triangle.getVertex1(), triangle.getVertex2(),
                triangle.getVertex3());
    }

    /**
     * Returns area of a triangle formed by provided vertices.
     *
     * @param vertex1 1st vertex of a triangle.
     * @param vertex2 2nd vertex of a triangle.
     * @param vertex3 3rd vertex of a triangle.
     * @return Area of a triangle.
     */
    public static double area(
            final Point2D vertex1, final Point2D vertex2, final Point2D vertex3) {
        return Math.abs(signedArea(vertex1, vertex2, vertex3));
    }

    /**
     * Returns area of this triangle.
     *
     * @return Area of this triangle.
     */
    public double getArea() {
        return area(mVertex1, mVertex2, mVertex3);
    }

    /**
     * Determines whether vertices of this triangle are considered to be
     * colinear. Points are considered to be colinear when area of triangle is
     * very small.
     *
     * @return True if vertices are colinear, false otherwise.
     */
    public boolean areVerticesColinear() {
        return areVerticesColinear(DEFAULT_THRESHOLD);
    }

    /**
     * Determines whether vertices of this triangle are considered to be
     * colinear up to certain threshold. Points are considered to be colinear
     * when are of triangle is very small.
     *
     * @param threshold Threshold to determine whether vertices are colinear.
     *                  Vertices will be colinear when area of triangle is smaller than provided
     *                  threshold.
     * @return True if vertices are colinear, false otherwise.
     * @throws IllegalArgumentException Raised if provided threshold is negative.
     */
    public boolean areVerticesColinear(final double threshold) {

        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        return getArea() <= threshold;
    }

    /**
     * Returns perimeter of provided triangle.
     *
     * @param triangle Perimeter of provided triangle.
     * @return Perimeter of provided triangle.
     */
    public static double perimeter(final Triangle2D triangle) {
        return perimeter(triangle.getVertex1(), triangle.getVertex2(),
                triangle.getVertex3());
    }

    /**
     * Returns perimeter of triangle formed by provided vertices.
     *
     * @param vertex1 1st vertex of a triangle.
     * @param vertex2 2nd vertex of a triangle.
     * @param vertex3 3rd vertex of a triangle.
     * @return Perimeter of a triangle.
     */
    public static double perimeter(
            final Point2D vertex1, final Point2D vertex2, final Point2D vertex3) {
        return vertex1.distanceTo(vertex2) + vertex2.distanceTo(vertex3) +
                vertex3.distanceTo(vertex1);
    }

    /**
     * Returns perimeter of this triangle.
     *
     * @return Perimeter of this triangle.
     */
    public double getPerimeter() {
        return perimeter(this);
    }

    /**
     * Indicates whether provided point lies inside this triangle or not.
     *
     * @param point Point to be checked.
     * @return True if point lies inside this triangle, false otherwise.
     */
    public boolean isInside(final Point2D point) {
        return isInside(point, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point lies inside this triangle or not up to
     * a certain threshold.
     *
     * @param point     Point to be checked.
     * @param threshold Threshold to determine whether point is inside this
     *                  triangle, or not. This should usually be a small value.
     * @return True if point lies inside this triangle, false otherwise.
     * @throws IllegalArgumentException Raised if provided threshold is negative.
     */
    public boolean isInside(final Point2D point, final double threshold) {
        return isInside(mVertex1, mVertex2, mVertex3, point, threshold);
    }

    /**
     * Indicates whether provided point lies inside provided triangle or not.
     *
     * @param triangle A triangle.
     * @param point    Point to be checked.
     * @return True if point lies inside provided triangle, false otherwise.
     */
    public static boolean isInside(final Triangle2D triangle, final Point2D point) {
        return isInside(triangle, point, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point lies inside provided triangle or not up
     * to a certain threshold.
     *
     * @param triangle  A triangle.
     * @param point     Point to be checked.
     * @param threshold Threshold to determine whether point is inside this
     *                  triangle, or not. This should usually be a small value.
     * @return True if point lies inside this triangle, false otherwise.
     * @throws IllegalArgumentException Raised if provided threshold is negative.
     */
    public static boolean isInside(
            final Triangle2D triangle, final Point2D point, final double threshold) {
        return isInside(triangle.getVertex1(), triangle.getVertex2(),
                triangle.getVertex3(), point, threshold);
    }

    /**
     * Indicates whether provided point lies inside a triangle formed by
     * provided vertices or not.
     *
     * @param vertex1 1st vertex of a triangle.
     * @param vertex2 2nd vertex of a triangle.
     * @param vertex3 3rd vertex of a triangle.
     * @param point   Point to be checked.
     * @return True if point lies inside triangle formed by provided vertices,
     * false otherwise.
     */
    public static boolean isInside(
            final Point2D vertex1, final Point2D vertex2,
            final Point2D vertex3, final Point2D point) {
        return isInside(vertex1, vertex2, vertex3, point, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point lies inside a triangle formed by
     * provided vertices or not up to a certain threshold.
     *
     * @param vertex1   1st vertex of a triangle.
     * @param vertex2   2nd vertex of a triangle.
     * @param vertex3   3rd vertex of a triangle.
     * @param point     Point to be checked.
     * @param threshold Threshold to determine whether point is inside the
     *                  triangle formed by provided vertices or not. This should usually be a
     *                  small value.
     * @return True if point lies inside triangle formed by provided vertices,
     * false otherwise.
     * @throws IllegalArgumentException Raised if provided threshold is negative.
     */
    public static boolean isInside(
            final Point2D vertex1, final Point2D vertex2,
            final Point2D vertex3, final Point2D point, final double threshold) {
        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        // given triangle ABC made by vectors:
        // ab = p2 - p1, and ac = p3 - p1

        // If point (x, y) lies within triangle ABC, then we have 3 sub-triangles
        // ApB, BpC and ApC made of points:
        // ApB: mVertex1, point, mVertex2
        // BpC: mVertex2, point, mVertex3
        // ApC: mVertex3, point, mVertex1

        // The point will lie inside triangle ABC if the sum of the areas of the
        // 3 sub-triangles ApB, BpC and ApC equals the area of triangle ABC (up to
        // certain accuracy to account for numerical precision)

        // Then the areas of triangles are:
        final double areaABC = area(vertex1, vertex2, vertex3);

        final double areaApB = area(vertex1, point, vertex2);
        final double areaBpC = area(vertex2, point, vertex3);
        final double areaApC = area(vertex3, point, vertex1);

        return Math.abs(areaApB + areaBpC + areaApC - areaABC) <= threshold;
    }

    /**
     * Returns center of this triangle, which is the result of averaging its
     * vertices.
     *
     * @return Center of this triangle.
     */
    public Point2D getCenter() {
        final Point2D result = Point2D.create();
        center(result);
        return result;
    }

    /**
     * Computes the center of this triangle and stores the result in provided
     * point. The center of this triangle is computed as the average of its
     * vertices.
     *
     * @param result Point instance where center will be stored.
     */
    public void center(final Point2D result) {
        center(mVertex1, mVertex2, mVertex3, result);
    }

    /**
     * Computes the center of a triangle formed by provided vertices.
     * The center is computed as the average of the three vertices.
     *
     * @param vertex1 1st vertex of a triangle.
     * @param vertex2 2nd vertex of a triangle.
     * @param vertex3 3rd vertex of a triangle.
     * @return Center of a triangle formed by provided vertices.
     */
    public static Point2D center(
            final Point2D vertex1, final Point2D vertex2, final Point2D vertex3) {
        final Point2D result = Point2D.create();
        center(vertex1, vertex2, vertex3, result);
        return result;
    }

    /**
     * Computes the center of provided triangle.
     * The center is computed as the average of the vertices of provided
     * triangle.
     *
     * @param t A triangle.
     * @return Center of provided triangle.
     */
    public static Point2D center(final Triangle2D t) {
        return center(t.getVertex1(), t.getVertex2(), t.getVertex3());
    }

    /**
     * Computes the center of a triangle formed by provided vertices and stores
     * the result in provided result point.
     * The center is computed as the average of provided vertices.
     *
     * @param vertex1 1st vertex of a triangle.
     * @param vertex2 2nd vertex of a triangle.
     * @param vertex3 3rd vertex of a triangle.
     * @param result  Point instance where center will be stored.
     */
    public static void center(
            final Point2D vertex1, final Point2D vertex2, final Point2D vertex3,
            final Point2D result) {

        final double x = (vertex1.getInhomX() + vertex2.getInhomX() + vertex3.getInhomX()) /
                3.0;
        final double y = (vertex1.getInhomY() + vertex2.getInhomY() + vertex3.getInhomY()) /
                3.0;

        result.setInhomogeneousCoordinates(x, y);
    }

    /**
     * Computes the center of provided triangle and stores the result in
     * provided result point.
     * The center is computed as the average of the vertices of provided
     * triangle.
     *
     * @param t      A triangle.
     * @param result Point instance where center will be stored.
     */
    public static void center(final Triangle2D t, final Point2D result) {
        center(t.getVertex1(), t.getVertex2(), t.getVertex3(), result);
    }

    /**
     * Computes the shortest distance from a given point to the boundaries of
     * this triangle, considering its boundaries as lines with a finite length
     * Distance is computed up to triangle boundary, no matter if point lies
     * inside the triangle or not.
     *
     * @param point Point to be checked.
     * @return Shortest distance to this triangle.
     */
    public double getShortestDistance(final Point2D point) {
        return shortestDistance(this, point);
    }

    /**
     * Computes the shortest distance from a given point to the boundaries of
     * provided triangle, considering its boundaries as lines with a finite
     * length.
     * Distance is computed up to triangle boundary, no matter if point lies
     * inside the triangle or not.
     *
     * @param triangle A triangle.
     * @param point    Point to be checked.
     * @return Shortest distance to this triangle.
     */
    public static double shortestDistance(
            final Triangle2D triangle, final Point2D point) {
        return shortestDistance(triangle.getVertex1(), triangle.getVertex2(),
                triangle.getVertex3(), point);
    }

    // shortest distance to the sides of the triangle, no matter if the point
    // lies inside the triangle or not

    /**
     * Computes the shortest distance from a given point to the boundaries of
     * a triangle formed by provided vertices, where those boundaries are
     * considered to be lines with a finite length.
     * Distance is computed up to triangle boundary, no matter if point lies
     * inside the triangle or not.
     *
     * @param vertex1 1st vertex of a triangle.
     * @param vertex2 2nd vertex of a triangle.
     * @param vertex3 3rd vertex of a triangle.
     * @param point   Point to be checked.
     * @return Shortest distance to the triangle formed by provided vertices.
     */
    public static double shortestDistance(
            final Point2D vertex1, final Point2D vertex2,
            final Point2D vertex3, final Point2D point) {

        // normalize points to increase accuracy
        vertex1.normalize();
        vertex2.normalize();
        vertex3.normalize();
        point.normalize();

        double bestDist;
        double dist;

        final Line2D line = new Line2D();
        line.setParametersFromPairOfPoints(vertex1, vertex2);
        // to increase accuracy
        line.normalize();
        if (line.isLocus(point)) {
            if (point.isBetween(vertex1, vertex2)) {
                return 0.0;
            } else {
                // point is outside the triangle and
                // point belongs to the line forming this side of the triangle,
                // hence the closest vertex of this line will be the shortest
                // distance
                bestDist = vertex1.distanceTo(point);
                dist = vertex2.distanceTo(point);
                if (dist < bestDist) {
                    bestDist = dist;
                }

                return bestDist;
            }
        }

        // point does not belong to the first line
        bestDist = Math.abs(line.signedDistance(point));

        // try on second side of the triangle
        line.setParametersFromPairOfPoints(vertex1, vertex3);
        // to increase accuracy
        line.normalize();
        if (line.isLocus(point)) {
            if (point.isBetween(vertex1, vertex3)) {
                return 0.0;
            } else {
                // point belongs to the line forming this side of the triangle,
                // hence the closest vertex of this line will be the shortest
                // distance
                bestDist = vertex1.distanceTo(point);
                dist = vertex3.distanceTo(point);
                if (dist < bestDist) {
                    bestDist = dist;
                }

                return bestDist;
            }
        }

        // point does not belong to the first or second line
        dist = Math.abs(line.signedDistance(point));

        // check if second line is closest to first line
        if (dist < bestDist) {
            bestDist = dist;
        }

        // try on third side of the triangle
        line.setParametersFromPairOfPoints(vertex2, vertex3);
        // to increase accuracy
        line.normalize();
        if (line.isLocus(point)) {
            if (point.isBetween(vertex2, vertex3)) {
                return 0.0;
            } else {
                // point belongs to the line forming this side of the triangle,
                // hence the closest vertex of this line will be the shortest
                // distance
                bestDist = vertex2.distanceTo(point);
                dist = vertex3.distanceTo(point);
                if (dist < bestDist) {
                    bestDist = dist;
                }

                return bestDist;
            }
        }

        // point does not belong to any line forming a side of the triangle
        dist = Math.abs(line.signedDistance(point));

        // check if distance to third line is the shortest
        if (dist < bestDist) {
            bestDist = dist;
        }

        return bestDist;
    }

    /**
     * Returns the point which is locus of this triangle closest to provided
     * point.
     *
     * @param point Point to be checked.
     * @return Closest point laying in this triangle boundaries.
     */
    public Point2D getClosestPoint(final Point2D point) {
        return getClosestPoint(point, DEFAULT_THRESHOLD);
    }

    /**
     * Returns the point which is locus of this triangle (up to a certain
     * threshold) closest to provided point.
     *
     * @param point     Point to be checked.
     * @param threshold Threshold to determine when a point is locus of this
     *                  triangle or not.
     * @return Closest point laying in this triangle boundaries.
     * @throws IllegalArgumentException Raised if provided threshold is negative.
     */
    public Point2D getClosestPoint(final Point2D point, final double threshold) {
        final Point2D result = Point2D.create();
        closestPoint(point, result, threshold);
        return result;
    }

    /**
     * Computes the point which is locus of this triangle closest to provided
     * point and stores the result in provided result point.
     *
     * @param point  Point to be checked.
     * @param result Point where result will be stored.
     */
    public void closestPoint(final Point2D point, final Point2D result) {
        closestPoint(point, result, DEFAULT_THRESHOLD);
    }

    /**
     * Computes the point which is locus of this triangle (up to a certain
     * threshold) closest to provided point and stores the result in provided
     * result point.
     *
     * @param point     Point to be checked.
     * @param result    Point where result will be stored.
     * @param threshold Threshold to determine when a point is locus of this
     *                  triangle or not.
     * @throws IllegalArgumentException Raised if provided threshold is negative.
     */
    public void closestPoint(final Point2D point, final Point2D result,
                             final double threshold) {
        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        // normalize vertices and point to increase accuracy
        mVertex1.normalize();
        mVertex2.normalize();
        mVertex3.normalize();
        point.normalize();

        final Line2D line1 = new Line2D(mVertex1, mVertex2);
        // to increase accuracy
        line1.normalize();
        if (line1.isLocus(point)) {
            if (point.isBetween(mVertex1, mVertex2)) {
                // point is on this side of the triangle, so point must be the
                // result
                result.setCoordinates(point);
            } else {
                // point belongs to the line forming this side of the triangle,
                // hence the closest vertex of this line will be the closest
                // point to the triangle
                final double dist1 = mVertex1.distanceTo(point);
                final double dist2 = mVertex2.distanceTo(point);
                if (dist1 < dist2) {
                    result.setCoordinates(mVertex1);
                } else {
                    result.setCoordinates(mVertex2);
                }
            }
            return;
        }

        // try on second side of the triangle
        final Line2D line2 = new Line2D(mVertex1, mVertex3);
        // to increase accuracy
        line2.normalize();
        if (line2.isLocus(point)) {
            if (point.isBetween(mVertex1, mVertex3)) {
                // point is on this side of the triangle, so point must be the
                // result
                result.setCoordinates(point);
            } else {
                // point belongs to the line forming this side of the triangle,
                // hence the closest vertex of this line will be the closest
                // point to the triangle
                final double dist1 = mVertex1.distanceTo(point);
                final double dist3 = mVertex3.distanceTo(point);
                if (dist1 < dist3) {
                    result.setCoordinates(mVertex1);
                } else {
                    result.setCoordinates(mVertex3);
                }
            }
            return;
        }


        // try on third side of the triangle
        final Line2D line3 = new Line2D(mVertex2, mVertex3);
        // to increase accuracy
        line3.normalize();
        if (line3.isLocus(point)) {
            if (point.isBetween(mVertex2, mVertex3)) {
                // point is on this side of the triangle, so point must be the
                // result
                result.setCoordinates(point);
            } else {
                // point belongs to the line forming this side of the triangle,
                // hence the closest vertex of this line will be the closest
                // point to the triangle
                final double dist2 = mVertex2.distanceTo(point);
                final double dist3 = mVertex3.distanceTo(point);
                if (dist2 < dist3) {
                    result.setCoordinates(mVertex2);
                } else {
                    result.setCoordinates(mVertex3);
                }
            }
            return;
        }

        // point does not belong to any line forming a side of the triangle
        // so we find the closest point for each side
        final Point2D closest1 = line1.getClosestPoint(point, threshold);
        // to increase accuracy
        closest1.normalize();
        final Point2D closest2 = line2.getClosestPoint(point, threshold);
        // to increase accuracy
        closest2.normalize();
        final Point2D closest3 = line3.getClosestPoint(point, threshold);
        // to increase accuracy
        closest3.normalize();

        // check if points lie within sides of triangle
        final boolean between1 = closest1.isBetween(mVertex1, mVertex2);
        final boolean between2 = closest2.isBetween(mVertex1, mVertex3);
        final boolean between3 = closest3.isBetween(mVertex2, mVertex3);

        final double distClosest1 = closest1.distanceTo(point);
        final double distClosest2 = closest2.distanceTo(point);
        final double distClosest3 = closest3.distanceTo(point);

        final double distVertex1 = mVertex1.distanceTo(point);
        final double distVertex2 = mVertex2.distanceTo(point);
        final double distVertex3 = mVertex3.distanceTo(point);


        if (between1 && !between2 && !between3) {
            // choose closest1 or opposite vertex (vertex3)
            if (distClosest1 < distVertex3) {
                result.setCoordinates(closest1);
            } else {
                result.setCoordinates(mVertex3);
            }
        } else if (!between1 && between2 && !between3) {
            // choose closest2 or opposite vertex (vertex2)
            if (distClosest2 < distVertex2) {
                result.setCoordinates(closest2);
            } else {
                result.setCoordinates(mVertex2);
            }
        } else if (!between1 && !between2 && between3) {
            // choose closest3 or opposite vertex (vertex1)
            if (distClosest3 < distVertex1) {
                result.setCoordinates(closest3);
            } else {
                result.setCoordinates(mVertex1);
            }
        } else if (between1 && between2 && !between3) {
            // determine if closest1 or closest2
            if (distClosest1 < distClosest2) {
                result.setCoordinates(closest1);
            } else {
                result.setCoordinates(closest2);
            }
        } else if (!between1 && between2) {
            // and between3

            // determine if closest2 or closest3
            if (distClosest2 < distClosest3) {
                result.setCoordinates(closest2);
            } else {
                result.setCoordinates(closest3);
            }
        } else if (between1 && !between2) {
            // and between3

            //determine if closest1 or closest3
            if (distClosest1 < distClosest3) {
                result.setCoordinates(closest1);
            } else {
                result.setCoordinates(closest3);
            }
        } else if (between1) {
            // and between2 and between3

            //determine if closest1, closest2 or closest3
            if (distClosest1 < distClosest2 && distClosest1 < distClosest3) {
                // pick closest1
                result.setCoordinates(closest1);
            } else if (distClosest2 < distClosest1 &&
                    distClosest2 < distClosest3) {
                // pick closest2
                result.setCoordinates(closest2);
            } else {
                // pick closest3
                result.setCoordinates(closest3);
            }
        } else {
            // all closest points are outside vertex limits, so we pick the
            // closest vertex

            if (distVertex1 < distVertex2 && distVertex1 < distVertex3) {
                // pick vertex1
                result.setCoordinates(mVertex1);
            } else if (distVertex2 < distVertex1 && distVertex2 < distVertex3) {
                // pick vertex2
                result.setCoordinates(mVertex2);
            } else {
                // pick vertex3
                result.setCoordinates(mVertex3);
            }
        }
    }

    /**
     * Returns boolean indicating if provided point is locus of this triangle
     * (i.e. lies within this triangle boundaries) up to a certain threshold.
     *
     * @param point     Point to be checked.
     * @param threshold Threshold to determine if point is locus or not. This
     *                  should usually be a small value.
     * @return True if provided point is locus, false otherwise.
     * @throws IllegalArgumentException Raised if provided threshold is negative.
     */
    public boolean isLocus(final Point2D point, final double threshold) {
        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        return point.isBetween(mVertex1, mVertex2, threshold) ||
                point.isBetween(mVertex1, mVertex3, threshold) ||
                point.isBetween(mVertex2, mVertex3, threshold);
    }

    /**
     * Returns boolean indicating if provided point is locus of this triangle
     * (i.e. lies within this triangle boundaries).
     *
     * @param point Point to be checked.
     * @return True if provided point is locus, false otherwise.
     */
    public boolean isLocus(final Point2D point) {
        return isLocus(point, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether the vertices of this triangle are provided in clockwise
     * order or not.
     * Vertices of a triangle are in clockwise order when the triangle's signed
     * area is negative.
     *
     * @param threshold Threshold to determine if vertices are clockwise or not.
     *                  This should usually be 0.0 and there is no restriction in sign
     * @return True if vertices of this triangle are in clockwise order, false
     * if they are in counterclockwise order.
     */
    public boolean areVerticesClockwise(final double threshold) {
        return getSignedArea() < threshold;
    }

    /**
     * Indicates whether the vertices of this triangle are provided in clockwise
     * order or not.
     *
     * @return True if vertices of this triangle are in clockwise order, false
     * if they are in counterclockwise order.
     */
    public boolean areVerticesClockwise() {
        return areVerticesClockwise(0.0); // default threshold to check sign
    }

    //TODO: compute surrounding circle and inner circle
}
