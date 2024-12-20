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
 * This class defines a triangle in the 3D space.
 */
@SuppressWarnings("DuplicatedCode")
public class Triangle3D implements Serializable {
    /**
     * Default threshold value. Thresholds are used to determine whether a point
     * lies inside the triangle or not, or if its locus or not, etc.
     */
    public static final double DEFAULT_THRESHOLD = 1e-9;

    /**
     * Minimum allowed threshold value
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Constant defining number of coordinates
     */
    public static final int INHOM_COORDS = 3;

    /**
     * Constant defining number of vertices on a triangle
     */
    public static final int NUM_VERTICES = 3;

    /**
     * 1st vertex of this triangle.
     */
    private Point3D vertex1;

    /**
     * 2nd vertex of this triangle.
     */
    private Point3D vertex2;

    /**
     * 3rd vertex of this triangle.
     */
    private Point3D vertex3;

    /**
     * Constructor.
     *
     * @param vertex1 1st vertex.
     * @param vertex2 2nd vertex.
     * @param vertex3 3rd vertex.
     * @throws NullPointerException Raised if any of the vertices is null.
     */
    public Triangle3D(final Point3D vertex1, final Point3D vertex2, final Point3D vertex3) {
        setVertices(vertex1, vertex2, vertex3);
    }

    /**
     * Returns 1st vertex of this triangle.
     *
     * @return 1st vertex.
     */
    public Point3D getVertex1() {
        return vertex1;
    }

    /**
     * Sets 1st vertex of this triangle.
     *
     * @param vertex1 1st vertex.
     * @throws NullPointerException Raised if provided vertex is null.
     */
    public void setVertex1(final Point3D vertex1) {
        if (vertex1 == null) {
            throw new NullPointerException();
        }
        this.vertex1 = vertex1;
    }

    /**
     * Returns 2nd vertex of this triangle.
     *
     * @return 2nd vertex.
     */
    public Point3D getVertex2() {
        return vertex2;
    }

    /**
     * Sets 2nd vertex of this triangle.
     *
     * @param vertex2 2nd vertex.
     * @throws NullPointerException Raised if provided vertex is null.
     */
    public void setVertex2(final Point3D vertex2) {
        if (vertex2 == null) {
            throw new NullPointerException();
        }
        this.vertex2 = vertex2;
    }

    /**
     * Returns 3rd vertex of this triangle.
     *
     * @return 3rd vertex.
     */
    public Point3D getVertex3() {
        return vertex3;
    }

    /**
     * Sets 3rd vertex of this triangle.
     *
     * @param vertex3 3rd vertex.
     * @throws NullPointerException Raised if provided vertex is null.
     */
    public void setVertex3(final Point3D vertex3) {
        if (vertex3 == null) {
            throw new NullPointerException();
        }
        this.vertex3 = vertex3;
    }

    /**
     * Returns vertices of this triangle as a list of points.
     *
     * @return Vertices of this triangle.
     */
    public List<Point3D> getVertices() {
        final var vertices = new ArrayList<Point3D>(NUM_VERTICES);
        vertices(vertices);
        return vertices;
    }

    /**
     * Stores vertices of this triangle in provided list. Note that content of
     * list will be cleared before storing this triangle's vertices.
     *
     * @param result list where vertices will be stored.
     */
    public void vertices(final List<Point3D> result) {
        result.clear();
        result.add(vertex1);
        result.add(vertex2);
        result.add(vertex3);
    }

    /**
     * Sets all vertices of this triangle.
     *
     * @param vertex1 1st vertex.
     * @param vertex2 2nd vertex.
     * @param vertex3 3rd vertex.
     * @throws NullPointerException Raised if any of the vertices is null.
     */
    public final void setVertices(final Point3D vertex1, final Point3D vertex2, final Point3D vertex3) {
        if (vertex1 == null || vertex2 == null || vertex3 == null) {
            throw new NullPointerException();
        }

        this.vertex1 = vertex1;
        this.vertex2 = vertex2;
        this.vertex3 = vertex3;
    }

    /**
     * Returns area of provided triangle.
     *
     * @param triangle Triangle to be evaluated.
     * @return Area of triangle.
     */
    public static double area(final Triangle3D triangle) {
        return area(triangle.getVertex1(), triangle.getVertex2(), triangle.getVertex3());
    }

    /**
     * Returns area of the triangle formed by provided vertices.
     *
     * @param vertex1 1st vertex of a triangle.
     * @param vertex2 2nd vertex of a triangle.
     * @param vertex3 3rd vertex of a triangle.
     * @return Area of a triangle.
     */
    public static double area(final Point3D vertex1, final Point3D vertex2, final Point3D vertex3) {
        // The signed area of a triangle is half the determinant of its vectors,
        // or half the modulus of the cross product of its vectors

        // Hence, having the vectors of the triangle defined as:
        // v1 = vertex2 - vertex1, and v2 = vertex3 - vertex1, then:
        final var inhomX1 = vertex1.getInhomX();
        final var inhomY1 = vertex1.getInhomY();
        final var inhomZ1 = vertex1.getInhomZ();

        // given triangle ABC made by vectors ab and ac
        final var abX = vertex2.getInhomX() - inhomX1;
        final var abY = vertex2.getInhomY() - inhomY1;
        final var abZ = vertex2.getInhomZ() - inhomZ1;

        final var acX = vertex3.getInhomX() - inhomX1;
        final var acY = vertex3.getInhomY() - inhomY1;
        final var acZ = vertex3.getInhomZ() - inhomZ1;

        // the area of ABC is half the modulus of the cross product of
        // vectors ab and ac
        final var crossX = abY * acZ - abZ * acY;
        final var crossY = abZ * acX - abX * acZ;
        final var crossZ = abX * acY - abY * acX;

        return 0.5 * Math.sqrt(crossX * crossX + crossY * crossY + crossZ * crossZ);
    }

    /**
     * Returns area of this triangle.
     *
     * @return Area of this triangle.
     */
    public double getArea() {
        return area(vertex1, vertex2, vertex3);
    }

    /**
     * Determines whether vertices of this triangle are considered to be
     * co-linear. Points are considered to be colinear when area of triangle is
     * very small.
     *
     * @return True if vertices are colinear, false otherwise.
     */
    public boolean areVerticesColinear() {
        return areVerticesColinear(DEFAULT_THRESHOLD);
    }

    /**
     * Determines whether vertices of this triangle are considered to be
     * co-linear up to certain threshold. Points are considered to be colinear
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
    public static double perimeter(final Triangle3D triangle) {
        return perimeter(triangle.getVertex1(), triangle.getVertex2(), triangle.getVertex3());
    }

    /**
     * Returns perimeter of triangle formed by provided vertices.
     *
     * @param vertex1 1st vertex of a triangle.
     * @param vertex2 2nd vertex of a triangle.
     * @param vertex3 3rd vertex of a triangle.
     * @return Perimeter of a triangle.
     */
    public static double perimeter(final Point3D vertex1, final Point3D vertex2, final Point3D vertex3) {
        return vertex1.distanceTo(vertex2) + vertex2.distanceTo(vertex3) + vertex3.distanceTo(vertex1);
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
     * To lie inside point must be on the same plane formed by this triangle and
     * within triangle boundaries.
     *
     * @param point Point to be checked.
     * @return True if point lies inside this triangle, false otherwise.
     */
    public boolean isInside(final Point3D point) {
        return isInside(point, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point lies inside this triangle or not up to
     * a certain threshold.
     * To lie inside point must be on the same plane formed by this triangle and
     * within triangle boundaries up to a certain threshold.
     *
     * @param point     Point to be checked.
     * @param threshold Threshold to determine whether point is inside this
     *                  triangle, or not. This should usually be a small value.
     * @return True if point lies inside this triangle, false otherwise.
     * @throws IllegalArgumentException Raised if provided threshold is negative.
     */
    public boolean isInside(final Point3D point, final double threshold) {
        return isInside(vertex1, vertex2, vertex3, point, threshold);
    }

    /**
     * Indicates whether provided point lies inside provided triangle or not
     * To lie inside point must be on the same plane formed by provided triangle
     * and within triangle boundaries.
     *
     * @param triangle A triangle.
     * @param point    Point to be checked.
     * @return True if point lies inside provided triangle, false otherwise.
     */
    public static boolean isInside(final Triangle3D triangle, final Point3D point) {
        return isInside(triangle, point, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point lies inside provided triangle or not up
     * to a certain threshold.
     * To lie inside point must be on the same plane formed by provided triangle
     * and within triangle boundaries up to a certain threshold.
     *
     * @param triangle  A triangle.
     * @param point     Point to be checked.
     * @param threshold Threshold to determine whether point is inside this
     *                  triangle, or not. This should usually be a small value.
     * @return True if point lies inside this triangle, false otherwise.
     * @throws IllegalArgumentException Raised if provided threshold is negative.
     */
    public static boolean isInside(final Triangle3D triangle, final Point3D point, final double threshold) {
        return isInside(triangle.getVertex1(), triangle.getVertex2(), triangle.getVertex3(), point, threshold);
    }

    /**
     * Indicates whether provided point lies inside a triangle formed by
     * provided vertices or not.
     * To lie inside point must be on the same plane formed by that triangle and
     * within its boundaries.
     *
     * @param vertex1 1st vertex of a triangle.
     * @param vertex2 2nd vertex of a triangle.
     * @param vertex3 3rd vertex of a triangle.
     * @param point   Point to be checked.
     * @return True if point lies inside triangle formed by provided vertices,
     * false otherwise.
     */
    public static boolean isInside(
            final Point3D vertex1, final Point3D vertex2, final Point3D vertex3, final Point3D point) {
        return isInside(vertex1, vertex2, vertex3, point, DEFAULT_THRESHOLD);
    }

    /**
     * Indicates whether provided point lies inside a triangle formed by
     * provided vertices or not up to a certain threshold.
     * To lie inside point must be on the same plane formed by that triangle and
     * within its boundaries up to a certain threshold.
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
            final Point3D vertex1, final Point3D vertex2,
            final Point3D vertex3, final Point3D point, final double threshold) {
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
        final var areaABC = area(vertex1, vertex2, vertex3);

        final var areaApB = area(vertex1, point, vertex2);
        final var areaBpC = area(vertex2, point, vertex3);
        final var areaApC = area(vertex3, point, vertex1);

        return Math.abs(areaApB + areaBpC + areaApC - areaABC) <= threshold;
    }

    /**
     * Returns the plane formed by the vertices of this triangle.
     * The difference between a plane and a 3D triangle is that a triangle has
     * its boundaries defined, whereas a plane extends up to the infinity.
     *
     * @return A plane.
     * @throws ColinearPointsException Raised if vertices of this triangle are
     *                                 co-linear (triangle has area equal or very close to 0.0).
     */
    public Plane toPlane() throws ColinearPointsException {
        return new Plane(vertex1, vertex2, vertex3);
    }

    /**
     * Computes the plane formed by the vertices of this triangle and stores the
     * result into provided Plane instance.
     * The difference between a plane and a 3D triangle is that a triangle has
     * its boundaries defined, whereas a plane extends up to the infinity.
     *
     * @param result Instance where resulting plane will be stored.
     * @throws ColinearPointsException Raised if vertices of this triangle are
     *                                 co-linear (triangle has area equal or very close to 0.0).
     */
    public void toPlane(final Plane result) throws ColinearPointsException {
        result.setParametersFromThreePoints(vertex1, vertex2, vertex3);
    }

    /**
     * Returns center of this triangle, which is the result of averaging its
     * vertices.
     *
     * @return Center of this triangle.
     */
    public Point3D getCenter() {
        final var result = Point3D.create();
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
    public void center(final Point3D result) {
        center(vertex1, vertex2, vertex3, result);
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
    public static Point3D center(final Point3D vertex1, final Point3D vertex2, final Point3D vertex3) {
        final var result = Point3D.create();
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
    public static Point3D center(final Triangle3D t) {
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
            final Point3D vertex1, final Point3D vertex2, final Point3D vertex3, final Point3D result) {

        final var x = (vertex1.getInhomX() + vertex2.getInhomX() + vertex3.getInhomX()) / 3.0;
        final var y = (vertex1.getInhomY() + vertex2.getInhomY() + vertex3.getInhomY()) / 3.0;
        final var z = (vertex1.getInhomZ() + vertex2.getInhomZ() + vertex3.getInhomZ()) / 3.0;

        result.setInhomogeneousCoordinates(x, y, z);
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
    public static void center(final Triangle3D t, final Point3D result) {
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
    public double getShortestDistance(final Point3D point) {
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
    public static double shortestDistance(final Triangle3D triangle, final Point3D point) {
        return shortestDistance(triangle.getVertex1(), triangle.getVertex2(), triangle.getVertex3(), point);
    }

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
            final Point3D vertex1, final Point3D vertex2, final Point3D vertex3, final Point3D point) {

        // normalize points to increase accuracy
        vertex1.normalize();
        vertex2.normalize();
        vertex3.normalize();
        point.normalize();

        var bestDist = Double.MAX_VALUE;
        var dist = Double.MAX_VALUE;

        Line3D line = null;
        try {
            line = new Line3D(vertex1, vertex2);
            //to increase accuracy
            line.normalize();
            if (line.isLocus(point)) {
                if (point.isBetween(vertex1, vertex2)) {
                    return 0.0;
                } else {
                    // point is outside the triangle and
                    // point belongs to the line forming this side of the
                    // triangle, hence the closest vertex of this line will be
                    // the shortest distance
                    bestDist = vertex1.distanceTo(point);
                    dist = vertex2.distanceTo(point);
                    if (dist < bestDist) {
                        bestDist = dist;
                    }

                    return bestDist;
                }
            }

            // point does not belong to the first line
            bestDist = Math.abs(line.getDistance(point));
        } catch (final CoincidentPointsException e) {
            if (point.equals(vertex1)) {
                return vertex1.distanceTo(point);
            }
            if (point.equals(vertex2)) {
                return vertex2.distanceTo(point);
            }
        }

        if (line == null) {
            return bestDist;
        }

        // try on second side of the triangle
        try {
            line.setPlanesFromPoints(vertex1, vertex3);
            // to increase accuracy
            line.normalize();
            if (line.isLocus(point)) {
                if (point.isBetween(vertex1, vertex3)) {
                    return 0.0;
                } else {
                    // point belongs to the line forming this side of the
                    // triangle, hence the closest vertex of this line will be
                    // the shortest distance
                    bestDist = vertex1.distanceTo(point);
                    dist = vertex3.distanceTo(point);
                    if (dist < bestDist) {
                        bestDist = dist;
                    }

                    return bestDist;
                }
            }

            // point does not belong to the first or second line
            dist = Math.abs(line.getDistance(point));
        } catch (final CoincidentPointsException e) {
            if (point.equals(vertex1)) {
                return vertex1.distanceTo(point);
            }
            if (point.equals(vertex3)) {
                return vertex3.distanceTo(point);
            }
        }

        // check if second line is closest to first line
        if (dist < bestDist) {
            bestDist = dist;
        }

        // try on third side of the triangle
        try {
            line.setPlanesFromPoints(vertex2, vertex3);
            // to increase accuracy
            line.normalize();
            if (line.isLocus(point)) {
                if (point.isBetween(vertex2, vertex3)) {
                    return 0.0;
                } else {
                    // point belongs to the line forming this side of the
                    // triangle, hence the closest vertex of this line will be
                    // the shortest distance
                    bestDist = vertex2.distanceTo(point);
                    dist = vertex3.distanceTo(point);
                    if (dist < bestDist) {
                        bestDist = dist;
                    }

                    return bestDist;
                }
            }

            // point does not belong to any line forming a side of the triangle
            dist = Math.abs(line.getDistance(point));
        } catch (final CoincidentPointsException e) {
            if (point.equals(vertex2)) {
                return vertex2.distanceTo(point);
            }
            if (point.equals(vertex3)) {
                return vertex3.distanceTo(point);
            }
        }

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
    public Point3D getClosestPoint(final Point3D point) {
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
    public Point3D getClosestPoint(final Point3D point, final double threshold) {
        final var result = Point3D.create();
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
    public void closestPoint(final Point3D point, final Point3D result) {
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
    public void closestPoint(final Point3D point, final Point3D result, final double threshold) {
        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        // normalize vertices and point to increase accuracy
        vertex1.normalize();
        vertex2.normalize();
        vertex3.normalize();
        point.normalize();

        Line3D line1;
        Line3D line2;
        Line3D line3;
        try {
            line1 = new Line3D(vertex1, vertex2);
            // to increase accuracy
            line1.normalize();
            if (line1.isLocus(point)) {
                if (point.isBetween(vertex1, vertex2)) {
                    // point is on this side of the triangle, so point must
                    // be the result
                    result.setCoordinates(point);
                } else {
                    // point belongs to the line forming this side of the
                    // triangle, hence the closest vertex of this line will be
                    // the closest point to the triangle
                    final var dist1 = vertex1.distanceTo(point);
                    final var dist2 = vertex2.distanceTo(point);
                    if (dist1 < dist2) {
                        result.setCoordinates(vertex1);
                    } else {
                        result.setCoordinates(vertex2);
                    }
                }
                return;
            }
        } catch (final CoincidentPointsException e) {
            if (point.equals(vertex1) || point.equals(vertex2)) {
                result.setCoordinates(point);
            }
            return;
        }


        // try on second side of the triangle
        try {
            line2 = new Line3D(vertex1, vertex3);
            // to increase accuracy
            line2.normalize();
            if (line2.isLocus(point)) {
                if (point.isBetween(vertex1, vertex3)) {
                    // point is on this side of the triangle, so point must be
                    // the result
                    result.setCoordinates(point);
                } else {
                    // point belongs to the line forming this side of the
                    // triangle, hence the closest vertex of this line will be
                    // the closest point to the triangle
                    final var dist1 = vertex1.distanceTo(point);
                    final var dist3 = vertex3.distanceTo(point);
                    if (dist1 < dist3) {
                        result.setCoordinates(vertex1);
                    } else {
                        result.setCoordinates(vertex3);
                    }
                }
                return;
            }
        } catch (final CoincidentPointsException e) {
            if (point.equals(vertex1) || point.equals(vertex3)) {
                result.setCoordinates(point);
            }
            return;
        }


        // try on third side of the triangle
        try {
            line3 = new Line3D(vertex2, vertex3);
            // to increase accuracy
            line3.normalize();
            if (line3.isLocus(point)) {
                if (point.isBetween(vertex2, vertex3)) {
                    // point is on this side of the triangle, so point must be
                    // the result
                    result.setCoordinates(point);
                } else {
                    // point belongs to the line forming this side of the
                    // triangle, hence the closest vertex of this line will be
                    // the closest point to the triangle
                    final var dist2 = vertex2.distanceTo(point);
                    final var dist3 = vertex3.distanceTo(point);
                    if (dist2 < dist3) {
                        result.setCoordinates(vertex2);
                    } else {
                        result.setCoordinates(vertex3);
                    }
                }
                return;
            }
        } catch (final CoincidentPointsException e) {
            if (point.equals(vertex2) || point.equals(vertex3)) {
                result.setCoordinates(point);
            }
            return;
        }


        // point does not belong to any line forming a side of the triangle,
        // so we find the closest point for each side
        Point3D closest1;
        Point3D closest2;
        Point3D closest3;

        closest1 = line1.getClosestPoint(point, threshold);
        // to increase accuracy
        closest1.normalize();

        closest2 = line2.getClosestPoint(point, threshold);
        // to increase accuracy
        closest2.normalize();

        closest3 = line3.getClosestPoint(point, threshold);
        // to increase accuracy
        closest3.normalize();

        // check if points lie within sides of triangle
        final var between1 = closest1.isBetween(vertex1, vertex2);
        final var between2 = closest2.isBetween(vertex1, vertex3);
        final var between3 = closest3.isBetween(vertex2, vertex3);

        final var distClosest1 = closest1.distanceTo(point);
        final var distClosest2 = closest2.distanceTo(point);
        final var distClosest3 = closest3.distanceTo(point);

        final var distVertex1 = vertex1.distanceTo(point);
        final var distVertex2 = vertex2.distanceTo(point);
        final var distVertex3 = vertex3.distanceTo(point);

        if (between1 && !between2 && !between3) {
            // choose closest1 or opposite vertex (vertex3)
            if (distClosest1 < distVertex3) {
                result.setCoordinates(closest1);
            } else {
                result.setCoordinates(vertex3);
            }
        } else if (!between1 && between2 && !between3) {
            // choose closest2 or opposite vertex (vertex2)
            if (distClosest2 < distVertex2) {
                result.setCoordinates(closest2);
            } else {
                result.setCoordinates(vertex2);
            }
        } else if (!between1 && !between2 && between3) {
            // choose closest3 or opposite vertex (vertex1)
            if (distClosest3 < distVertex1) {
                result.setCoordinates(closest3);
            } else {
                result.setCoordinates(vertex1);
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

            // determine if closest1 or closest3
            if (distClosest1 < distClosest3) {
                result.setCoordinates(closest1);
            } else {
                result.setCoordinates(closest3);
            }
        } else if (between1) {
            // and between2 and between3

            // determine if closest1, closest2 or closest3
            if (distClosest1 < distClosest2 && distClosest1 < distClosest3) {
                // pick closest1
                result.setCoordinates(closest1);
            } else if (distClosest2 < distClosest1 && distClosest2 < distClosest3) {
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
                result.setCoordinates(vertex1);
            } else if (distVertex2 < distVertex1 && distVertex2 < distVertex3) {
                // pick vertex2
                result.setCoordinates(vertex2);
            } else {
                // pick vertex3
                result.setCoordinates(vertex3);
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
    public boolean isLocus(final Point3D point, final double threshold) {
        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        return point.isBetween(vertex1, vertex2, threshold) || point.isBetween(vertex1, vertex3, threshold)
                || point.isBetween(vertex2, vertex3, threshold);
    }

    /**
     * Returns boolean indicating if provided point is locus of this triangle
     * (i.e. lies within this triangle boundaries).
     *
     * @param point Point to be checked.
     * @return True if provided point is locus, false otherwise.
     */
    public boolean isLocus(final Point3D point) {
        return isLocus(point, DEFAULT_THRESHOLD);
    }


    /**
     * Returns array containing orientation of this 3D triangle.
     *
     * @return Array containing orientation of this 3D triangle.
     * @throws CoincidentPointsException Raised if vertices of this triangle
     *                                   are too close to each other.
     */
    public double[] getOrientation() throws CoincidentPointsException {
        return orientation(this);
    }

    /**
     * Returns array containing orientation of this 3D triangle.
     *
     * @param threshold Threshold to determine whether vertices of this triangle
     *                  are coincident or not.
     * @return Array containing orientation of this 3D triangle.
     * @throws IllegalArgumentException  Raised if provided threshold is negative.
     * @throws CoincidentPointsException Raised if vertices of this triangle
     *                                   are too close to each other.
     */
    public double[] getOrientation(final double threshold) throws CoincidentPointsException {
        return orientation(this, threshold);
    }

    /**
     * Computes orientation of this 3D triangle and stores the result in
     * provided array.
     *
     * @param result Array where orientation is stored.
     * @throws IllegalArgumentException  Raised if provided array does not have
     *                                   length 3.
     * @throws CoincidentPointsException Raised if vertices of this triangle
     *                                   are too close to each other.
     */
    public void orientation(final double[] result) throws CoincidentPointsException {
        orientation(this, result);
    }

    /**
     * Computes orientation of this 3D triangle and stores the result in
     * provided array.
     *
     * @param result    Array where orientation is stored.
     * @param threshold Threshold to determine whether vertices of this triangle
     *                  are coincident or not.
     * @throws IllegalArgumentException  Raised if provided threshold is negative
     *                                   or if provided array does not have length 3.
     * @throws CoincidentPointsException Raised if vertices of this triangle
     *                                   are too close to each other.
     */
    public void orientation(final double[] result, final double threshold) throws CoincidentPointsException {
        orientation(this, result, threshold);
    }

    /**
     * Returns orientation of provided 3D triangle.
     *
     * @param triangle A triangle.
     * @return Array containing orientation of provided 3D triangle.
     * @throws CoincidentPointsException Raised if vertices of provided triangle
     *                                   are too close to each other.
     */
    public static double[] orientation(final Triangle3D triangle) throws CoincidentPointsException {
        return orientation(triangle.getVertex1(), triangle.getVertex2(), triangle.getVertex3());
    }

    /**
     * Returns orientation of provided 3D triangle.
     *
     * @param triangle  A triangle.
     * @param threshold Threshold to determine whether vertices of provided
     *                  triangle are coincident or not.
     * @return Array containing orientation of provided 3D triangle.
     * @throws IllegalArgumentException  Raised if provided threshold is negative.
     * @throws CoincidentPointsException Raised if vertices of this triangle are
     *                                   too close to each other.
     */
    public static double[] orientation(final Triangle3D triangle, final double threshold)
            throws CoincidentPointsException {
        return orientation(triangle.getVertex1(), triangle.getVertex2(),
                triangle.getVertex3(), threshold);
    }

    /**
     * Computes orientation of provided 3D triangle and stores the result in
     * provided array.
     *
     * @param triangle A triangle.
     * @param result   Array where orientation is stored.
     * @throws IllegalArgumentException  Raised if provided array does not have
     *                                   length 3.
     * @throws CoincidentPointsException Raised if vertices of provided triangle
     *                                   are too close to each other.
     */
    public static void orientation(final Triangle3D triangle, final double[] result) throws CoincidentPointsException {
        orientation(triangle.getVertex1(), triangle.getVertex2(), triangle.getVertex3(), result);
    }

    /**
     * Computes orientation of provided 3D triangle and stores the result in
     * provided array.
     *
     * @param triangle  A triangle.
     * @param result    Array where orientation is stored.
     * @param threshold Threshold to determine whether vertices of provided
     *                  triangle are coincident or not.
     * @throws IllegalArgumentException  Raised if provided threshold is negative
     *                                   or if array does not have length 3.
     * @throws CoincidentPointsException Raised if vertices of provided triangle
     *                                   are too close to each other.
     */
    public static void orientation(
            final Triangle3D triangle, final double[] result, final double threshold) throws CoincidentPointsException {
        orientation(triangle.getVertex1(), triangle.getVertex2(), triangle.getVertex3(), result, threshold);
    }

    /**
     * Returns orientation of 3D triangle formed by provided vertices.
     *
     * @param vertex1 1st vertex.
     * @param vertex2 2nd vertex.
     * @param vertex3 3rd vertex.
     * @return Array containing triangle orientation.
     * @throws CoincidentPointsException Raised if provided vertices are too
     *                                   close to each other.
     */
    public static double[] orientation(
            final Point3D vertex1, final Point3D vertex2, final Point3D vertex3) throws CoincidentPointsException {
        return orientation(vertex1, vertex2, vertex3, DEFAULT_THRESHOLD);
    }

    /**
     * Returns orientation of 3D triangle formed by provided vertices.
     *
     * @param vertex1   1st vertex.
     * @param vertex2   2nd vertex.
     * @param vertex3   3rd vertex.
     * @param threshold Threshold to determine whether provided vertices are
     *                  coincident or not.
     * @return Array containing triangle orientation.
     * @throws IllegalArgumentException  Raised if provided threshold is negative.
     * @throws CoincidentPointsException Raised if provided vertices are too
     *                                   close to each other.
     */
    public static double[] orientation(
            final Point3D vertex1, final Point3D vertex2, final Point3D vertex3, final double threshold)
            throws CoincidentPointsException {
        final var result = new double[INHOM_COORDS];
        orientation(vertex1, vertex2, vertex3, result, threshold);
        return result;
    }

    /**
     * Computes orientation of 3D triangle formed by provided vertices and
     * stores the result in provided array.
     *
     * @param vertex1 1st vertex.
     * @param vertex2 2nd vertex.
     * @param vertex3 3rd vertex.
     * @param result  Array where triangle orientation is stored.
     * @throws IllegalArgumentException  Raised if provided array does not have
     *                                   length 3.
     * @throws CoincidentPointsException Raised if provided vertices are too
     *                                   close to each other.
     */
    public static void orientation(
            final Point3D vertex1, final Point3D vertex2, final Point3D vertex3, final double[] result)
            throws CoincidentPointsException {
        orientation(vertex1, vertex2, vertex3, result, DEFAULT_THRESHOLD);
    }

    /**
     * Computes orientation of 3D triangle formed by provided vertices and
     * stores the result in provided array.
     *
     * @param vertex1   1st vertex.
     * @param vertex2   2nd vertex.
     * @param vertex3   3rd vertex.
     * @param result    Array where triangle orientation is stored.
     * @param threshold Threshold to determine whether provided vertices are
     *                  coincident or not.
     * @throws IllegalArgumentException  Raised if threshold is negative or if
     *                                   provided array does not have length 3.
     * @throws CoincidentPointsException Raised if provided vertices are too
     *                                   close to each other.
     */
    public static void orientation(
            final Point3D vertex1, final Point3D vertex2, final Point3D vertex3, final double[] result,
            final double threshold) throws CoincidentPointsException {

        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        if (result.length != INHOM_COORDS) {
            throw new IllegalArgumentException();
        }

        final var inhomX1 = vertex1.getInhomX();
        final var inhomY1 = vertex1.getInhomY();
        final var inhomZ1 = vertex1.getInhomZ();

        final var inhomX2 = vertex2.getInhomX();
        final var inhomY2 = vertex2.getInhomY();
        final var inhomZ2 = vertex2.getInhomZ();

        final var inhomX3 = vertex3.getInhomX();
        final var inhomY3 = vertex3.getInhomY();
        final var inhomZ3 = vertex3.getInhomZ();

        // given triangle ABC made by vectors ab and ac
        final var abX = inhomX2 - inhomX1;
        final var abY = inhomY2 - inhomY1;
        final var abZ = inhomZ2 - inhomZ1;

        final var acX = inhomX3 - inhomX1;
        final var acY = inhomY3 - inhomY1;
        final var acZ = inhomZ3 - inhomZ1;

        // the area of ABC is half the modulus of the cross product of
        // vectors ab and ac
        var crossX = abY * acZ - abZ * acY;
        var crossY = abZ * acX - abX * acZ;
        var crossZ = abX * acY - abY * acX;
        // normalize orientation vector
        var norm = Math.sqrt(crossX * crossX + crossY * crossY + crossZ * crossZ);

        if (norm < threshold) {
            throw new CoincidentPointsException();
        }

        crossX /= norm;
        crossY /= norm;
        crossZ /= norm;

        result[0] = crossX;
        result[1] = crossY;
        result[2] = crossZ;
    }

    /**
     * Returns the angle formed by the two provided triangles, assuming that
     * each triangle forms a plane.
     *
     * @param triangle1 1st triangle.
     * @param triangle2 2nd triangle.
     * @return Angle formed by the two provided triangles expressed in radians.
     * @throws CoincidentPointsException Raised if vertices in a triangle are
     *                                   too close. This usually indicates numerical instability or triangle
     *                                   degeneracy.
     */
    public static double getAngleBetweenTriangles(final Triangle3D triangle1, final Triangle3D triangle2)
            throws CoincidentPointsException {
        return getAngleBetweenTriangles(triangle1.getOrientation(), triangle2.getOrientation());
    }

    /**
     * Internal method to compute the angle between two triangles using the
     * vectors containing the director vector of their corresponding planes
     * (i.e. their orientations).
     *
     * @param orientation1 Orientation of 1st triangle.
     * @param orientation2 Orientation of 2nd triangle.
     * @return Angle formed by the two triangles expressed in radians.
     * @throws IllegalArgumentException Raised if provided orientation arrays
     *                                  don't have length 3.
     */
    private static double getAngleBetweenTriangles(
            final double[] orientation1, final double[] orientation2) {
        if (orientation1.length != INHOM_COORDS ||
                orientation2.length != INHOM_COORDS) {
            throw new IllegalArgumentException();
        }

        final var x1 = orientation1[0];
        final var y1 = orientation1[1];
        final var z1 = orientation1[2];

        final var x2 = orientation2[0];
        final var y2 = orientation2[1];
        final var z2 = orientation2[2];

        final var norm1 = Math.sqrt(x1 * x1 + y1 * y1 + z1 * z1);
        final var norm2 = Math.sqrt(x2 * x2 + y2 * y2 + z2 * z2);

        final var dotProduct = (x1 * x2 + y1 * y2 + z1 * z2) / (norm1 * norm2);

        return Math.acos(dotProduct);
    }
}
