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

import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Utils;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.io.IOException;
import java.util.*;

import static org.junit.Assert.*;

public class Polygon3DTest {

    private static final int MIN_SIDES = 6;
    private static final int MAX_SIDES = 12;

    private static final double MIN_RADIUS = 1.0;
    private static final double MAX_RADIUS = 10.0;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double ABSOLUTE_ERROR = 1e-8;
    private static final double EPS = 1e-9;

    private static final int INHOM_COORDS = 3;

    private static final int TIMES = 100;

    @Test
    public void testConstants() {
        assertEquals(1e-9, Polygon3D.DEFAULT_THRESHOLD, 0.0);
        assertEquals(0.0, Polygon3D.MIN_THRESHOLD, 0.0);
        assertEquals(3, Polygon3D.MIN_VERTICES);
        assertEquals(3, Polygon3D.INHOM_COORDS);
        assertEquals(TriangulatorMethod.VAN_GOGH_TRIANGULATOR, Polygon3D.DEFAULT_TRIANGULATOR_METHOD);
    }

    @Test
    public void testConstructor() throws NotEnoughVerticesException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);

        // build vertices list
        final List<Point3D> vertices = buildPolygonVertices(sides, radius, theta);

        // build polygon
        Polygon3D polygon = new Polygon3D(vertices);

        // check correctness
        assertFalse(polygon.isTriangulated());
        assertEquals(polygon.getVertices(), vertices);
        assertEquals(Polygon3D.DEFAULT_TRIANGULATOR_METHOD, polygon.getTriangulatorMethod());

        final Iterator<Point3D> iterator1 = polygon.getVertices().iterator();
        final Iterator<Point3D> iterator2 = vertices.iterator();

        Point3D vertex1;
        Point3D vertex2;
        while (iterator1.hasNext() && iterator2.hasNext()) {
            vertex1 = iterator1.next();
            vertex2 = iterator2.next();
            assertTrue(vertex1.equals(vertex2, ABSOLUTE_ERROR));
        }

        // Force NotEnoughVerticesException
        vertex1 = vertices.get(0);
        vertex2 = vertices.get(1);
        vertices.clear();
        vertices.add(vertex1);
        vertices.add(vertex2);
        polygon = null;
        try {
            polygon = new Polygon3D(vertices);
            fail("NotEnoughVerticesException expected but not thrown");
        } catch (final NotEnoughVerticesException ignore) {
        }
        assertNull(polygon);
    }

    @Test
    public void testGetSetTriangulatorMethod() throws NotEnoughVerticesException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);

        // build vertices list
        final List<Point3D> vertices = buildPolygonVertices(sides, radius, theta);

        // build polygon
        final Polygon3D polygon = new Polygon3D(vertices);

        // check correctness
        assertEquals(Polygon3D.DEFAULT_TRIANGULATOR_METHOD, polygon.getTriangulatorMethod());

        // set new method
        polygon.setTriangulatorMethod(TriangulatorMethod.VAN_GOGH_TRIANGULATOR);
        // check correctness
        assertEquals(TriangulatorMethod.VAN_GOGH_TRIANGULATOR, polygon.getTriangulatorMethod());
    }

    @Test
    public void testGetSetVertices1() throws NotEnoughVerticesException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES);

        // build vertices list
        final List<Point3D> vertices = buildPolygonVertices(sides, radius, theta);

        // build polygon vertices
        final Polygon3D polygon = new Polygon3D(vertices);
        assertEquals(vertices, polygon.getVertices());

        // build new vertices
        sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        final List<Point3D> vertices2 = buildPolygonVertices(sides, radius, theta);

        polygon.setVertices(vertices2);
        assertEquals(vertices2, polygon.getVertices());
    }

    @Test
    public void testGetSetVertices2() throws NotEnoughVerticesException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES);

        // build vertices list
        final List<Point3D> vertices = new LinkedList<>(buildPolygonVertices(sides, radius, theta));

        // build polygon vertices
        final Polygon3D polygon = new Polygon3D(vertices);
        assertEquals(vertices, polygon.getVertices());

        // build new vertices
        sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        final List<Point3D> vertices2 = new LinkedList<>(buildPolygonVertices(sides, radius, theta));

        polygon.setVertices(vertices2);
        assertEquals(vertices2, polygon.getVertices());
    }

    @Test
    public void testArea() throws NotEnoughVerticesException {
        // X: -1, Y: 0, Z: 5 (W: 1)
        final Point3D v1 = new InhomogeneousPoint3D(-1.0, 0.0, 5.0);
        // X: 1, Y: 0, Z: 5 (W: 1)
        final Point3D v2 = new InhomogeneousPoint3D(1.0, 0.0, 5.0);
        // X: 2, Y: 1, Z: 5 (W: 1)
        final Point3D v3 = new InhomogeneousPoint3D(2.0, 1.0, 5.0);
        // X: 0, Y: 1, Z: 5 (W: 1)
        final Point3D v4 = new InhomogeneousPoint3D(0.0, 1.0, 5.0);

        final List<Point3D> vertices = new ArrayList<>();
        vertices.add(v1);
        vertices.add(v2);
        vertices.add(v3);
        vertices.add(v4);

        final Polygon3D polygon = new Polygon3D(vertices);
        assertEquals(2.0, polygon.getArea(), ABSOLUTE_ERROR);
    }

    @Test
    public void testTriangulateIsTriangulatedAndArea()
            throws NotEnoughVerticesException, TriangulatorException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);

        // build vertices list
        final List<Point3D> vertices = buildPolygonVertices(sides, radius, theta);

        final Polygon3D polygon = new Polygon3D(vertices);

        final VanGoghTriangulator3D triangulator = new VanGoghTriangulator3D();

        // expected area is the sum of the areas of the triangles forming this
        // polygon
        double area = 0.0;
        for (final Triangle3D triangle : triangulator.triangulate(vertices)) {
            area += triangle.getArea();
        }

        assertFalse(polygon.isTriangulated());
        final List<Triangle3D> triangles = polygon.getTriangles();
        assertTrue(polygon.isTriangulated());

        double areaTriangles = 0.0;
        for (final Triangle3D triangle : triangles) {
            areaTriangles += triangle.getArea();
        }

        assertEquals(area, polygon.getArea(), ABSOLUTE_ERROR);
        assertEquals(areaTriangles, polygon.getArea(), ABSOLUTE_ERROR);

        // triangulate again
        polygon.triangulate();

        assertTrue(polygon.isTriangulated());
    }

    @Test
    public void testPerimeter() throws NotEnoughVerticesException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);

        // build vertices list
        final List<Point3D> vertices = buildPolygonVertices(sides, radius, theta);

        final Polygon3D polygon = new Polygon3D(vertices);

        final Iterator<Point3D> iterator = vertices.iterator();
        Point3D prevVertex = iterator.next();
        Point3D curVertex;
        double perimeter = 0.0;
        while (iterator.hasNext()) {
            curVertex = iterator.next();
            perimeter += curVertex.distanceTo(prevVertex);
            prevVertex = curVertex;
        }

        // compare distance of last vertex with first one
        perimeter += prevVertex.distanceTo(vertices.get(0));

        assertEquals(perimeter, polygon.getPerimeter(), ABSOLUTE_ERROR);

        // Test for a triangle
        sides = 3;
        final List<Point3D> vertices2 = buildPolygonVertices(sides, radius, theta);
        final Triangle3D triangle = new Triangle3D(vertices2.get(0), vertices2.get(1), vertices2.get(2));
        polygon.setVertices(vertices2);
        assertEquals(triangle.getPerimeter(), polygon.getPerimeter(), ABSOLUTE_ERROR);
    }

    @Test
    public void testIsInside() throws NotEnoughVerticesException,
            TriangulatorException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        final double phi = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);

        // build vertices list
        final List<Point3D> vertices = buildPolygonVertices(sides, radius, phi);

        final Polygon3D polygon = new Polygon3D(vertices);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        // create point inside of vertex

        final Point3D inside = new InhomogeneousPoint3D(
                radius / 2.0 * Math.cos(theta) * Math.cos(phi),
                radius / 2.0 * Math.sin(theta) * Math.cos(phi),
                radius * Math.sin(phi));

        // create point outside of vertex
        final Point3D outside = new InhomogeneousPoint3D(
                2.0 * radius * Math.cos(theta) * Math.cos(phi),
                2.0 * radius * Math.sin(theta) * Math.cos(phi),
                radius * Math.sin(phi));

        assertTrue(polygon.isInside(inside));
        assertTrue(polygon.isInside(inside, ABSOLUTE_ERROR));
        assertFalse(polygon.isInside(outside));
        assertFalse(polygon.isInside(outside, ABSOLUTE_ERROR));

        // check that vertices are inside
        for (final Point3D vertex : vertices) {
            assertTrue(polygon.isInside(vertex));
            assertTrue(polygon.isInside(vertex, ABSOLUTE_ERROR));
        }

        // Force IllegalArgumentException
        try {
            polygon.isInside(inside, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testCenter() throws NotEnoughVerticesException {
        // Generate random vertices
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);

        // build vertices list
        final List<Point3D> vertices = buildPolygonVertices(sides, radius, theta);

        double inhomX = 0.0;
        double inhomY = 0.0;
        double inhomZ = 0.0;
        for (final Point3D vertex : vertices) {
            inhomX += vertex.getInhomX() / (double) sides;
            inhomY += vertex.getInhomY() / (double) sides;
            inhomZ += vertex.getInhomZ() / (double) sides;
        }

        final Point3D center = new InhomogeneousPoint3D(inhomX, inhomY, inhomZ);

        final Polygon3D polygon = new Polygon3D(vertices);

        assertTrue(center.equals(polygon.getCenter(), ABSOLUTE_ERROR));
        final Point3D center2 = Point3D.create();
        polygon.center(center2);
        assertTrue(center.equals(center2, ABSOLUTE_ERROR));
    }

    @Test
    public void testIsLocusGetShortestDistanceAndClosestPoint()
            throws NotEnoughVerticesException, TriangulatorException,
            CoincidentPointsException {

        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
            final double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
            double dist = randomizer.nextDouble(2.0 * ABSOLUTE_ERROR, radius / 2.0);
            final double phi = 0.0;

            final double theta = randomizer.nextDouble(0.0,
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            int n = (int) (theta / (2 * Math.PI) * (double) sides);

            final List<Point3D> vertices = buildPolygonVertices(sides, radius, phi);

            final Point3D point1 = vertices.get(n);
            n = (n + 1) % sides;
            final Point3D point2 = vertices.get(n);

            final Line3D line = new Line3D(point1, point2);
            line.normalize();

            final Line2D line2D = new Line2D(
                    new InhomogeneousPoint2D(point1.getInhomX(), point1.getInhomY()),
                    new InhomogeneousPoint2D(point2.getInhomX(), point2.getInhomY()));
            // to increase accuracy
            line2D.normalize();
            assertEquals(point1.getInhomZ(), point2.getInhomZ(), ABSOLUTE_ERROR);

            final double[] direction = line2D.getDirectorVector();
            final double norm = Utils.normF(direction);
            ArrayUtils.multiplyByScalar(direction, 1.0 / norm, direction);

            // find point laying on the line between polygon vertices
            final Point3D testPoint = new InhomogeneousPoint3D(
                    radius * Math.cos(theta) * Math.cos(phi),
                    radius * Math.sin(theta) * Math.cos(phi),
                    radius * Math.sin(phi));
            // point below is locus of polygon
            final Point3D locusPoint = line.getClosestPoint(testPoint);
            // locus point is between point1 and point2
            assertTrue(locusPoint.isBetween(point1, point2));

            // move point a little bit away from locus
            final Point3D notLocusPoint = new InhomogeneousPoint3D(
                    locusPoint.getInhomX() + dist * direction[0],
                    locusPoint.getInhomY() + dist * direction[1],
                    locusPoint.getInhomZ());
            // ensure that notLocusPoint lies outside of polygon
            final Polygon3D polygon = new Polygon3D(vertices);

            if (polygon.isInside(notLocusPoint)) {
                // change sign of dist to move point in opposite direction
                dist *= -1.0;
                notLocusPoint.setInhomogeneousCoordinates(
                        locusPoint.getInhomX() + dist * direction[0],
                        locusPoint.getInhomY() + dist * direction[1],
                        locusPoint.getInhomZ());
            }
            assertFalse(polygon.isInside(notLocusPoint));

            // check that notLocusPoint is at distance dist from line
            assertEquals(line.getDistance(notLocusPoint), Math.abs(dist), ABSOLUTE_ERROR);

            assertTrue(polygon.isLocus(locusPoint));
            assertTrue(polygon.isLocus(locusPoint, ABSOLUTE_ERROR));

            // because point is locus, the shortest distance is zero and it is the
            // closest point
            assertEquals(0.0, polygon.getShortestDistance(locusPoint), ABSOLUTE_ERROR);
            assertTrue(polygon.getClosestPoint(locusPoint).equals(locusPoint, ABSOLUTE_ERROR));
            final Point3D closestPoint = Point3D.create();
            polygon.closestPoint(locusPoint, closestPoint);
            assertTrue(closestPoint.equals(locusPoint, ABSOLUTE_ERROR));
            assertTrue(closestPoint.equals(locusPoint, ABSOLUTE_ERROR));

            assertFalse(polygon.isLocus(notLocusPoint));
            assertFalse(polygon.isLocus(notLocusPoint, EPS));

            // not locus point is at distance dist from polygon
            assertEquals(Math.abs(dist), polygon.getShortestDistance(notLocusPoint), ABSOLUTE_ERROR);

            // and the closest point to polygon is locusPoint
            assertTrue(polygon.getClosestPoint(notLocusPoint).equals(
                    new InhomogeneousPoint3D(locusPoint), ABSOLUTE_ERROR));

            polygon.closestPoint(notLocusPoint, closestPoint);
            assertTrue(closestPoint.equals(new InhomogeneousPoint3D(locusPoint), ABSOLUTE_ERROR));
            assertTrue(closestPoint.equals(new InhomogeneousPoint3D(locusPoint), ABSOLUTE_ERROR));

            // with a large enough threshold, not locus point is considered as locus
            assertTrue(polygon.isLocus(notLocusPoint, radius * radius));

            // all vertices of polygon are also locus
            for (final Point3D vertex : vertices) {
                assertTrue(polygon.isLocus(vertex));
                assertTrue(polygon.isLocus(vertex, ABSOLUTE_ERROR));
                // because vertices are locus, shortest distance is 0.0 and it is
                // the closest point
                assertEquals(0.0, polygon.getShortestDistance(vertex), ABSOLUTE_ERROR);
                assertTrue(polygon.getClosestPoint(vertex).equals(vertex, ABSOLUTE_ERROR));
                polygon.closestPoint(vertex, closestPoint);
                assertTrue(closestPoint.equals(vertex, ABSOLUTE_ERROR));
                assertTrue(closestPoint.equals(vertex, ABSOLUTE_ERROR));
            }

            // Force IllegalArgumentException
            try {
                polygon.isLocus(locusPoint, -ABSOLUTE_ERROR);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
        }
    }

    @Test
    public void testOrientation() throws NotEnoughVerticesException,
            CoincidentPointsException, ColinearPointsException {

        // Test 1st for known values

        // X: -1, Y: 0, Z: 5 (W: 1)
        final Point3D v1 = new InhomogeneousPoint3D(-1.0, 0.0, 5.0);
        // X: 1, Y: 0, Z: 5 (W: 1)
        final Point3D v2 = new InhomogeneousPoint3D(1.0, 0.0, 5.0);
        // X: 2, Y: 1, Z: 5 (W: 1)
        final Point3D v3 = new InhomogeneousPoint3D(2.0, 1.0, 5.0);
        // X: 0, Y: 1, Z: 5 (W: 1)
        final Point3D v4 = new InhomogeneousPoint3D(0.0, 1.0, 5.0);

        List<Point3D> vertices = new ArrayList<>();
        vertices.add(v1);
        vertices.add(v2);
        vertices.add(v3);
        vertices.add(v4);

        Polygon3D polygon = new Polygon3D(vertices);

        double[] expectedOrientation = new double[INHOM_COORDS];
        expectedOrientation[0] = 0.0;
        expectedOrientation[1] = 0.0;
        expectedOrientation[2] = 1.0;

        double[] orientation = polygon.getOrientation();
        assertArrayEquals(expectedOrientation, orientation, ABSOLUTE_ERROR);
        orientation = polygon.getOrientation(ABSOLUTE_ERROR);
        assertArrayEquals(expectedOrientation, orientation, ABSOLUTE_ERROR);
        orientation = new double[INHOM_COORDS];
        polygon.orientation(orientation);
        assertArrayEquals(expectedOrientation, orientation, ABSOLUTE_ERROR);
        polygon.orientation(orientation, ABSOLUTE_ERROR);
        assertArrayEquals(expectedOrientation, orientation, ABSOLUTE_ERROR);
        orientation = Polygon3D.orientation(vertices);
        assertArrayEquals(expectedOrientation, orientation, ABSOLUTE_ERROR);
        orientation = Polygon3D.orientation(vertices, ABSOLUTE_ERROR);
        assertArrayEquals(expectedOrientation, orientation, ABSOLUTE_ERROR);
        orientation = Polygon3D.orientation(polygon);
        assertArrayEquals(expectedOrientation, orientation, ABSOLUTE_ERROR);
        orientation = Polygon3D.orientation(polygon, ABSOLUTE_ERROR);
        assertArrayEquals(expectedOrientation, orientation, ABSOLUTE_ERROR);
        orientation = new double[INHOM_COORDS];
        Polygon3D.orientation(vertices, orientation);
        assertArrayEquals(expectedOrientation, orientation, ABSOLUTE_ERROR);
        Polygon3D.orientation(vertices, orientation, ABSOLUTE_ERROR);
        assertArrayEquals(expectedOrientation, orientation, ABSOLUTE_ERROR);
        Polygon3D.orientation(polygon, orientation);
        assertArrayEquals(expectedOrientation, orientation, ABSOLUTE_ERROR);
        Polygon3D.orientation(polygon, orientation, ABSOLUTE_ERROR);
        assertArrayEquals(expectedOrientation, orientation, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            polygon.getOrientation(-ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            polygon.orientation(orientation, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Polygon3D.orientation(vertices, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Polygon3D.orientation(polygon, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Polygon3D.orientation(vertices, orientation, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Polygon3D.orientation(polygon, orientation, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        orientation = new double[INHOM_COORDS + 1];
        try {
            polygon.orientation(orientation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            polygon.orientation(orientation, ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Polygon3D.orientation(vertices, orientation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Polygon3D.orientation(vertices, orientation, ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Polygon3D.orientation(polygon, orientation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Polygon3D.orientation(polygon, orientation, ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Force CoincidentPointsException
        vertices.clear();
        vertices.add(v1);
        vertices.add(v1);
        vertices.add(v1);
        polygon = new Polygon3D(vertices);
        orientation = new double[INHOM_COORDS];
        try {
            polygon.getOrientation();
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            polygon.getOrientation(ABSOLUTE_ERROR);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            polygon.orientation(orientation);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            polygon.orientation(orientation, ABSOLUTE_ERROR);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            Polygon3D.orientation(vertices);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            Polygon3D.orientation(vertices, ABSOLUTE_ERROR);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            Polygon3D.orientation(polygon);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            Polygon3D.orientation(polygon, ABSOLUTE_ERROR);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            Polygon3D.orientation(vertices, orientation);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            Polygon3D.orientation(vertices, orientation, ABSOLUTE_ERROR);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            Polygon3D.orientation(polygon, orientation);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            Polygon3D.orientation(polygon, orientation, ABSOLUTE_ERROR);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }

        // now test with random values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES);

        // build vertices list
        vertices = buildPolygonVertices(sides, radius, theta);

        polygon = new Polygon3D(vertices);

        // build plane from 3 vertices
        final Plane plane = new Plane(vertices.get(0), vertices.get(1), vertices.get(2));
        expectedOrientation = plane.getDirectorVector();
        // normalize it
        final double norm = Utils.normF(expectedOrientation);
        ArrayUtils.multiplyByScalar(expectedOrientation, 1.0 / norm, expectedOrientation);

        // check correctness
        orientation = polygon.getOrientation();

        assertArrayEquals(absArray(orientation), absArray(expectedOrientation), ABSOLUTE_ERROR);
        orientation = polygon.getOrientation(ABSOLUTE_ERROR);
        assertArrayEquals(absArray(orientation), absArray(expectedOrientation), ABSOLUTE_ERROR);
        orientation = new double[INHOM_COORDS];
        polygon.orientation(orientation);
        assertArrayEquals(absArray(orientation), absArray(expectedOrientation), ABSOLUTE_ERROR);
        polygon.orientation(orientation, ABSOLUTE_ERROR);
        assertArrayEquals(absArray(orientation), absArray(expectedOrientation), ABSOLUTE_ERROR);
        orientation = Polygon3D.orientation(vertices);
        assertArrayEquals(absArray(orientation), absArray(expectedOrientation), ABSOLUTE_ERROR);
        orientation = Polygon3D.orientation(vertices, ABSOLUTE_ERROR);
        assertArrayEquals(absArray(orientation), absArray(expectedOrientation), ABSOLUTE_ERROR);
        orientation = Polygon3D.orientation(polygon);
        assertArrayEquals(absArray(orientation), absArray(expectedOrientation), ABSOLUTE_ERROR);
        orientation = Polygon3D.orientation(polygon, ABSOLUTE_ERROR);
        assertArrayEquals(absArray(orientation), absArray(expectedOrientation), ABSOLUTE_ERROR);
        orientation = new double[INHOM_COORDS];
        Polygon3D.orientation(vertices, orientation);
        assertArrayEquals(absArray(orientation), absArray(expectedOrientation), ABSOLUTE_ERROR);
        Polygon3D.orientation(vertices, orientation, ABSOLUTE_ERROR);
        assertArrayEquals(absArray(orientation), absArray(expectedOrientation), ABSOLUTE_ERROR);
        Polygon3D.orientation(polygon, orientation);
        assertArrayEquals(absArray(orientation), absArray(expectedOrientation), ABSOLUTE_ERROR);
        Polygon3D.orientation(polygon, orientation, ABSOLUTE_ERROR);
        assertArrayEquals(absArray(orientation), absArray(expectedOrientation), ABSOLUTE_ERROR);
    }

    @Test
    public void testAngleBetweenPolygons() throws NotEnoughVerticesException,
            CoincidentPointsException {
        // X: -1, Y: 0, Z: 5 (W: 1)
        final Point3D v1 = new InhomogeneousPoint3D(-1.0, 0.0, 5.0);
        // X: 1, Y: 0, Z: 5 (W: 1)
        final Point3D v2 = new InhomogeneousPoint3D(1.0, 0.0, 5.0);
        // X: 2, Y: 1, Z: 5 (W: 1)
        final Point3D v3 = new InhomogeneousPoint3D(2.0, 1.0, 5.0);
        // X: 0, Y: 1, Z: 5 (W: 1)
        final Point3D v4 = new InhomogeneousPoint3D(0.0, 1.0, 5.0);

        // parallelepiped
        final List<Point3D> vertices1 = new ArrayList<>();
        final List<Point3D> vertices2 = new ArrayList<>();
        vertices1.add(v1);
        vertices1.add(v2);
        vertices1.add(v3);
        vertices1.add(v4);

        vertices2.add(v4);
        vertices2.add(v3);
        vertices2.add(v2);
        vertices2.add(v1);

        final Polygon3D polygon1 = new Polygon3D(vertices1);
        // because order of vertices is reversed, orientation will have opposite
        // direction
        final Polygon3D polygon2 = new Polygon3D(vertices2);

        double angle = Polygon3D.getAngleBetweenPolygons(vertices1, vertices2);
        assertEquals(Math.PI, angle, ABSOLUTE_ERROR);
        angle = Polygon3D.getAngleBetweenPolygons(vertices1, vertices2, ABSOLUTE_ERROR);
        assertEquals(Math.PI, angle, ABSOLUTE_ERROR);

        angle = Polygon3D.getAngleBetweenPolygons(polygon1, polygon2);
        assertEquals(Math.PI, angle, ABSOLUTE_ERROR);
        angle = Polygon3D.getAngleBetweenPolygons(polygon1, polygon2, ABSOLUTE_ERROR);
        assertEquals(Math.PI, angle, ABSOLUTE_ERROR);

        // trying with same polygon will return an angle of zero
        angle = Polygon3D.getAngleBetweenPolygons(vertices1, vertices1);
        assertEquals(0.0, angle, ABSOLUTE_ERROR);
        angle = Polygon3D.getAngleBetweenPolygons(vertices1, vertices1, ABSOLUTE_ERROR);
        assertEquals(0.0, angle, ABSOLUTE_ERROR);

        angle = Polygon3D.getAngleBetweenPolygons(polygon1, polygon1);
        assertEquals(0.0, angle, ABSOLUTE_ERROR);
        angle = Polygon3D.getAngleBetweenPolygons(polygon1, polygon1, ABSOLUTE_ERROR);
        assertEquals(0.0, angle, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            Polygon3D.getAngleBetweenPolygons(vertices1, vertices2, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Polygon3D.getAngleBetweenPolygons(polygon1, polygon2, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Force CoincidentPointsException
        vertices1.clear();
        vertices1.add(v1);
        vertices1.add(v1);
        vertices1.add(v1);

        vertices2.clear();
        vertices2.add(v2);
        vertices2.add(v2);
        vertices2.add(v2);

        polygon1.setVertices(vertices1);
        polygon2.setVertices(vertices2);
        try {
            Polygon3D.getAngleBetweenPolygons(vertices1, vertices2);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            Polygon3D.getAngleBetweenPolygons(vertices1, vertices2, ABSOLUTE_ERROR);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            Polygon3D.getAngleBetweenPolygons(polygon1, polygon2);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            Polygon3D.getAngleBetweenPolygons(polygon1, polygon2, ABSOLUTE_ERROR);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
    }

    @Test
    public void testSerializeDeserialize() throws NotEnoughVerticesException,
            TriangulatorException, IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);

        // build vertices list
        final List<Point3D> vertices = buildPolygonVertices(sides, radius, theta);

        // build polygon
        Polygon3D polygon1 = new Polygon3D(vertices);
        polygon1.triangulate();

        assertSame(vertices, polygon1.getVertices());
        assertFalse(polygon1.getTriangles().isEmpty());

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(polygon1);
        final Polygon3D polygon2 = SerializationHelper.deserialize(bytes);

        assertNotSame(polygon1, polygon2);
        assertEquals(polygon1.getVertices(), polygon2.getVertices());
        assertNotSame(polygon1.getVertices(), polygon2.getVertices());
        assertEquals(polygon1.getTriangles().size(), polygon2.getTriangles().size());
    }

    private List<Point3D> buildPolygonVertices(
            final int sides, final double radius,
            final double theta) {
        final List<Point3D> vertices = new ArrayList<>(sides);
        Point3D vertex;
        for (int i = 0; i < sides; i++) {
            final double angle = (double) i / (double) sides * 2.0 * Math.PI;
            vertex = new InhomogeneousPoint3D(
                    radius * Math.cos(angle) * Math.cos(theta),
                    radius * Math.sin(angle) * Math.cos(theta),
                    radius * Math.sin(theta));
            vertices.add(vertex);
        }
        return vertices;
    }

    private double[] absArray(final double[] array) {
        final double[] out = new double[array.length];
        for (int i = 0; i < array.length; i++) {
            out[i] = Math.abs(array[i]);
        }
        return out;
    }
}
