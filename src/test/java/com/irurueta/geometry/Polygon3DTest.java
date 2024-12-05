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
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class Polygon3DTest {

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
    void testConstants() {
        assertEquals(1e-9, Polygon3D.DEFAULT_THRESHOLD, 0.0);
        assertEquals(0.0, Polygon3D.MIN_THRESHOLD, 0.0);
        assertEquals(3, Polygon3D.MIN_VERTICES);
        assertEquals(3, Polygon3D.INHOM_COORDS);
        assertEquals(TriangulatorMethod.VAN_GOGH_TRIANGULATOR, Polygon3D.DEFAULT_TRIANGULATOR_METHOD);
    }

    @Test
    void testConstructor() throws NotEnoughVerticesException {
        final var randomizer = new UniformRandomizer();
        final var sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final var radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        final var theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);

        // build vertices list
        final var vertices = buildPolygonVertices(sides, radius, theta);

        // build polygon
        var polygon = new Polygon3D(vertices);

        // check correctness
        assertFalse(polygon.isTriangulated());
        assertEquals(polygon.getVertices(), vertices);
        assertEquals(Polygon3D.DEFAULT_TRIANGULATOR_METHOD, polygon.getTriangulatorMethod());

        final var iterator1 = polygon.getVertices().iterator();
        final var iterator2 = vertices.iterator();

        while (iterator1.hasNext() && iterator2.hasNext()) {
            final var vertex1 = iterator1.next();
            final var vertex2 = iterator2.next();
            assertTrue(vertex1.equals(vertex2, ABSOLUTE_ERROR));
        }

        // Force NotEnoughVerticesException
        final var vertex1 = vertices.get(0);
        final var vertex2 = vertices.get(1);
        vertices.clear();
        vertices.add(vertex1);
        vertices.add(vertex2);
        assertThrows(NotEnoughVerticesException.class, () -> new Polygon3D(vertices));
    }

    @Test
    void testGetSetTriangulatorMethod() throws NotEnoughVerticesException {
        final var randomizer = new UniformRandomizer();
        final var sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final var radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        final var theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);

        // build vertices list
        final var vertices = buildPolygonVertices(sides, radius, theta);

        // build polygon
        final var polygon = new Polygon3D(vertices);

        // check correctness
        assertEquals(Polygon3D.DEFAULT_TRIANGULATOR_METHOD, polygon.getTriangulatorMethod());

        // set new method
        polygon.setTriangulatorMethod(TriangulatorMethod.VAN_GOGH_TRIANGULATOR);
        // check correctness
        assertEquals(TriangulatorMethod.VAN_GOGH_TRIANGULATOR, polygon.getTriangulatorMethod());
    }

    @Test
    void testGetSetVertices1() throws NotEnoughVerticesException {
        final var randomizer = new UniformRandomizer();
        var sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        var radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // build vertices list
        final var vertices = buildPolygonVertices(sides, radius, theta);

        // build polygon vertices
        final var polygon = new Polygon3D(vertices);
        assertEquals(vertices, polygon.getVertices());

        // build new vertices
        sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        final var vertices2 = buildPolygonVertices(sides, radius, theta);

        polygon.setVertices(vertices2);
        assertEquals(vertices2, polygon.getVertices());
    }

    @Test
    void testGetSetVertices2() throws NotEnoughVerticesException {
        final var randomizer = new UniformRandomizer();
        var sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        var radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // build vertices list
        final var vertices = new LinkedList<Point3D>(buildPolygonVertices(sides, radius, theta));

        // build polygon vertices
        final var polygon = new Polygon3D(vertices);
        assertEquals(vertices, polygon.getVertices());

        // build new vertices
        sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        final var vertices2 = new LinkedList<Point3D>(buildPolygonVertices(sides, radius, theta));

        polygon.setVertices(vertices2);
        assertEquals(vertices2, polygon.getVertices());
    }

    @Test
    void testArea() throws NotEnoughVerticesException {
        // X: -1, Y: 0, Z: 5 (W: 1)
        final var v1 = new InhomogeneousPoint3D(-1.0, 0.0, 5.0);
        // X: 1, Y: 0, Z: 5 (W: 1)
        final var v2 = new InhomogeneousPoint3D(1.0, 0.0, 5.0);
        // X: 2, Y: 1, Z: 5 (W: 1)
        final var v3 = new InhomogeneousPoint3D(2.0, 1.0, 5.0);
        // X: 0, Y: 1, Z: 5 (W: 1)
        final var v4 = new InhomogeneousPoint3D(0.0, 1.0, 5.0);

        final var vertices = new ArrayList<Point3D>();
        vertices.add(v1);
        vertices.add(v2);
        vertices.add(v3);
        vertices.add(v4);

        final var polygon = new Polygon3D(vertices);
        assertEquals(2.0, polygon.getArea(), ABSOLUTE_ERROR);
    }

    @Test
    void testTriangulateIsTriangulatedAndArea() throws NotEnoughVerticesException, TriangulatorException {
        final var randomizer = new UniformRandomizer();
        final var sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final var radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        final var theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);

        // build vertices list
        final var vertices = buildPolygonVertices(sides, radius, theta);

        final var polygon = new Polygon3D(vertices);

        final var triangulator = new VanGoghTriangulator3D();

        // expected area is the sum of the areas of the triangles forming this
        // polygon
        var area = 0.0;
        for (final var triangle : triangulator.triangulate(vertices)) {
            area += triangle.getArea();
        }

        assertFalse(polygon.isTriangulated());
        final var triangles = polygon.getTriangles();
        assertTrue(polygon.isTriangulated());

        var areaTriangles = 0.0;
        for (final var triangle : triangles) {
            areaTriangles += triangle.getArea();
        }

        assertEquals(area, polygon.getArea(), ABSOLUTE_ERROR);
        assertEquals(areaTriangles, polygon.getArea(), ABSOLUTE_ERROR);

        // triangulate again
        polygon.triangulate();

        assertTrue(polygon.isTriangulated());
    }

    @Test
    void testPerimeter() throws NotEnoughVerticesException {
        final var randomizer = new UniformRandomizer();
        var sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final var radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        final var theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);

        // build vertices list
        final var vertices = buildPolygonVertices(sides, radius, theta);

        final var polygon = new Polygon3D(vertices);

        final var iterator = vertices.iterator();
        var prevVertex = iterator.next();
        var perimeter = 0.0;
        while (iterator.hasNext()) {
            final var curVertex = iterator.next();
            perimeter += curVertex.distanceTo(prevVertex);
            prevVertex = curVertex;
        }

        // compare distance of last vertex with first one
        perimeter += prevVertex.distanceTo(vertices.get(0));

        assertEquals(perimeter, polygon.getPerimeter(), ABSOLUTE_ERROR);

        // Test for a triangle
        sides = 3;
        final var vertices2 = buildPolygonVertices(sides, radius, theta);
        final var triangle = new Triangle3D(vertices2.get(0), vertices2.get(1), vertices2.get(2));
        polygon.setVertices(vertices2);
        assertEquals(triangle.getPerimeter(), polygon.getPerimeter(), ABSOLUTE_ERROR);
    }

    @Test
    void testIsInside() throws NotEnoughVerticesException, TriangulatorException {

        final var randomizer = new UniformRandomizer();
        final var sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final var radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        final var phi = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // build vertices list
        final var vertices = buildPolygonVertices(sides, radius, phi);

        final var polygon = new Polygon3D(vertices);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        // create point inside of vertex

        final var inside = new InhomogeneousPoint3D(
                radius / 2.0 * Math.cos(theta) * Math.cos(phi),
                radius / 2.0 * Math.sin(theta) * Math.cos(phi),
                radius * Math.sin(phi));

        // create point outside of vertex
        final var outside = new InhomogeneousPoint3D(
                2.0 * radius * Math.cos(theta) * Math.cos(phi),
                2.0 * radius * Math.sin(theta) * Math.cos(phi),
                radius * Math.sin(phi));

        assertTrue(polygon.isInside(inside));
        assertTrue(polygon.isInside(inside, ABSOLUTE_ERROR));
        assertFalse(polygon.isInside(outside));
        assertFalse(polygon.isInside(outside, ABSOLUTE_ERROR));

        // check that vertices are inside
        for (final var vertex : vertices) {
            assertTrue(polygon.isInside(vertex));
            assertTrue(polygon.isInside(vertex, ABSOLUTE_ERROR));
        }

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> polygon.isInside(inside, -ABSOLUTE_ERROR));
    }

    @Test
    void testCenter() throws NotEnoughVerticesException {
        // Generate random vertices
        final var randomizer = new UniformRandomizer();
        final var sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final var radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        final var theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);

        // build vertices list
        final var vertices = buildPolygonVertices(sides, radius, theta);

        var inhomX = 0.0;
        var inhomY = 0.0;
        var inhomZ = 0.0;
        for (final var vertex : vertices) {
            inhomX += vertex.getInhomX() / (double) sides;
            inhomY += vertex.getInhomY() / (double) sides;
            inhomZ += vertex.getInhomZ() / (double) sides;
        }

        final var center = new InhomogeneousPoint3D(inhomX, inhomY, inhomZ);

        final var polygon = new Polygon3D(vertices);

        assertTrue(center.equals(polygon.getCenter(), ABSOLUTE_ERROR));
        final var center2 = Point3D.create();
        polygon.center(center2);
        assertTrue(center.equals(center2, ABSOLUTE_ERROR));
    }

    @Test
    void testIsLocusGetShortestDistanceAndClosestPoint() throws NotEnoughVerticesException, TriangulatorException,
            CoincidentPointsException {

        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
            final var radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
            var dist = randomizer.nextDouble(2.0 * ABSOLUTE_ERROR, radius / 2.0);
            final var phi = 0.0;

            final var theta = Math.toRadians(randomizer.nextDouble(0.0, 2.0 * MAX_ANGLE_DEGREES));

            var n = (int) (theta / (2 * Math.PI) * (double) sides);

            final var vertices = buildPolygonVertices(sides, radius, phi);

            final var point1 = vertices.get(n);
            n = (n + 1) % sides;
            final var point2 = vertices.get(n);

            final var line = new Line3D(point1, point2);
            line.normalize();

            final var line2D = new Line2D(
                    new InhomogeneousPoint2D(point1.getInhomX(), point1.getInhomY()),
                    new InhomogeneousPoint2D(point2.getInhomX(), point2.getInhomY()));
            // to increase accuracy
            line2D.normalize();
            assertEquals(point1.getInhomZ(), point2.getInhomZ(), ABSOLUTE_ERROR);

            final var direction = line2D.getDirectorVector();
            final var norm = Utils.normF(direction);
            ArrayUtils.multiplyByScalar(direction, 1.0 / norm, direction);

            // find point laying on the line between polygon vertices
            final var testPoint = new InhomogeneousPoint3D(
                    radius * Math.cos(theta) * Math.cos(phi),
                    radius * Math.sin(theta) * Math.cos(phi),
                    radius * Math.sin(phi));
            // point below is locus of polygon
            final var locusPoint = line.getClosestPoint(testPoint);
            // locus point is between point1 and point2
            assertTrue(locusPoint.isBetween(point1, point2));

            // move point a little bit away from locus
            final var notLocusPoint = new InhomogeneousPoint3D(
                    locusPoint.getInhomX() + dist * direction[0],
                    locusPoint.getInhomY() + dist * direction[1],
                    locusPoint.getInhomZ());
            // ensure that notLocusPoint lies outside of polygon
            final var polygon = new Polygon3D(vertices);

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

            // because point is locus, the shortest distance is zero and it is the closest point
            assertEquals(0.0, polygon.getShortestDistance(locusPoint), ABSOLUTE_ERROR);
            assertTrue(polygon.getClosestPoint(locusPoint).equals(locusPoint, ABSOLUTE_ERROR));
            final var closestPoint = Point3D.create();
            polygon.closestPoint(locusPoint, closestPoint);
            assertTrue(closestPoint.equals(locusPoint, ABSOLUTE_ERROR));
            assertTrue(closestPoint.equals(locusPoint, ABSOLUTE_ERROR));

            assertFalse(polygon.isLocus(notLocusPoint));
            assertFalse(polygon.isLocus(notLocusPoint, EPS));

            // not locus point is at distance dist from polygon
            assertEquals(Math.abs(dist), polygon.getShortestDistance(notLocusPoint), ABSOLUTE_ERROR);

            // and the closest point to polygon is locusPoint
            assertTrue(polygon.getClosestPoint(notLocusPoint).equals(new InhomogeneousPoint3D(locusPoint),
                    ABSOLUTE_ERROR));

            polygon.closestPoint(notLocusPoint, closestPoint);
            assertTrue(closestPoint.equals(new InhomogeneousPoint3D(locusPoint), ABSOLUTE_ERROR));
            assertTrue(closestPoint.equals(new InhomogeneousPoint3D(locusPoint), ABSOLUTE_ERROR));

            // with a large enough threshold, not locus point is considered as locus
            assertTrue(polygon.isLocus(notLocusPoint, radius * radius));

            // all vertices of polygon are also locus
            for (final var vertex : vertices) {
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
            assertThrows(IllegalArgumentException.class, () -> polygon.isLocus(locusPoint, -ABSOLUTE_ERROR));
        }
    }

    @Test
    void testOrientation() throws NotEnoughVerticesException, CoincidentPointsException, ColinearPointsException {

        // Test 1st for known values

        // X: -1, Y: 0, Z: 5 (W: 1)
        final var v1 = new InhomogeneousPoint3D(-1.0, 0.0, 5.0);
        // X: 1, Y: 0, Z: 5 (W: 1)
        final var v2 = new InhomogeneousPoint3D(1.0, 0.0, 5.0);
        // X: 2, Y: 1, Z: 5 (W: 1)
        final var v3 = new InhomogeneousPoint3D(2.0, 1.0, 5.0);
        // X: 0, Y: 1, Z: 5 (W: 1)
        final var v4 = new InhomogeneousPoint3D(0.0, 1.0, 5.0);

        final var vertices1 = new ArrayList<Point3D>();
        vertices1.add(v1);
        vertices1.add(v2);
        vertices1.add(v3);
        vertices1.add(v4);

        final var polygon1 = new Polygon3D(vertices1);

        var expectedOrientation = new double[INHOM_COORDS];
        expectedOrientation[0] = 0.0;
        expectedOrientation[1] = 0.0;
        expectedOrientation[2] = 1.0;

        final var orientation1 = polygon1.getOrientation();
        assertArrayEquals(expectedOrientation, orientation1, ABSOLUTE_ERROR);
        final var orientation2 = polygon1.getOrientation(ABSOLUTE_ERROR);
        assertArrayEquals(expectedOrientation, orientation2, ABSOLUTE_ERROR);
        final var orientation3 = new double[INHOM_COORDS];
        polygon1.orientation(orientation3);
        assertArrayEquals(expectedOrientation, orientation3, ABSOLUTE_ERROR);
        polygon1.orientation(orientation3, ABSOLUTE_ERROR);
        assertArrayEquals(expectedOrientation, orientation3, ABSOLUTE_ERROR);
        final var orientation4 = Polygon3D.orientation(vertices1);
        assertArrayEquals(expectedOrientation, orientation4, ABSOLUTE_ERROR);
        final var orientation5 = Polygon3D.orientation(vertices1, ABSOLUTE_ERROR);
        assertArrayEquals(expectedOrientation, orientation5, ABSOLUTE_ERROR);
        final var orientation6 = Polygon3D.orientation(polygon1);
        assertArrayEquals(expectedOrientation, orientation6, ABSOLUTE_ERROR);
        final var orientation7 = Polygon3D.orientation(polygon1, ABSOLUTE_ERROR);
        assertArrayEquals(expectedOrientation, orientation7, ABSOLUTE_ERROR);
        final var orientation8 = new double[INHOM_COORDS];
        Polygon3D.orientation(vertices1, orientation8);
        assertArrayEquals(expectedOrientation, orientation8, ABSOLUTE_ERROR);
        Polygon3D.orientation(vertices1, orientation8, ABSOLUTE_ERROR);
        assertArrayEquals(expectedOrientation, orientation8, ABSOLUTE_ERROR);
        Polygon3D.orientation(polygon1, orientation8);
        assertArrayEquals(expectedOrientation, orientation8, ABSOLUTE_ERROR);
        Polygon3D.orientation(polygon1, orientation8, ABSOLUTE_ERROR);
        assertArrayEquals(expectedOrientation, orientation8, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> polygon1.getOrientation(-ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class, () -> polygon1.orientation(orientation8, -ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class, () -> Polygon3D.orientation(vertices1, -ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class, () -> Polygon3D.orientation(polygon1, -ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class,
                () -> Polygon3D.orientation(vertices1, orientation8, -ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class,
                () -> Polygon3D.orientation(polygon1, orientation8, -ABSOLUTE_ERROR));

        final var orientation9 = new double[INHOM_COORDS + 1];
        assertThrows(IllegalArgumentException.class, () -> polygon1.orientation(orientation9));
        assertThrows(IllegalArgumentException.class, () -> polygon1.orientation(orientation9, ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class, () -> Polygon3D.orientation(vertices1, orientation9));
        assertThrows(IllegalArgumentException.class,
                () -> Polygon3D.orientation(vertices1, orientation9, ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class, () -> Polygon3D.orientation(polygon1, orientation9));
        assertThrows(IllegalArgumentException.class,
                () -> Polygon3D.orientation(polygon1, orientation9, ABSOLUTE_ERROR));

        // Force CoincidentPointsException
        vertices1.clear();
        vertices1.add(v1);
        vertices1.add(v1);
        vertices1.add(v1);
        final var polygon2 = new Polygon3D(vertices1);
        final var orientation10 = new double[INHOM_COORDS];
        assertThrows(CoincidentPointsException.class, polygon2::getOrientation);
        assertThrows(CoincidentPointsException.class, () -> polygon2.getOrientation(ABSOLUTE_ERROR));
        assertThrows(CoincidentPointsException.class, () -> polygon2.orientation(orientation10));
        assertThrows(CoincidentPointsException.class, () -> polygon2.orientation(orientation10, ABSOLUTE_ERROR));
        assertThrows(CoincidentPointsException.class, () -> Polygon3D.orientation(vertices1));
        assertThrows(CoincidentPointsException.class, () -> Polygon3D.orientation(vertices1, ABSOLUTE_ERROR));
        assertThrows(CoincidentPointsException.class, () -> Polygon3D.orientation(polygon2));
        assertThrows(CoincidentPointsException.class, () -> Polygon3D.orientation(polygon2, ABSOLUTE_ERROR));
        assertThrows(CoincidentPointsException.class, () -> Polygon3D.orientation(vertices1, orientation10));
        assertThrows(CoincidentPointsException.class,
                () -> Polygon3D.orientation(vertices1, orientation10, ABSOLUTE_ERROR));
        assertThrows(CoincidentPointsException.class, () -> Polygon3D.orientation(polygon2, orientation10));
        assertThrows(CoincidentPointsException.class,
                () -> Polygon3D.orientation(polygon2, orientation10, ABSOLUTE_ERROR));

        // now test with random values
        final var randomizer = new UniformRandomizer();
        final var sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final var radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        final var theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);

        // build vertices list
        final var vertices2 = buildPolygonVertices(sides, radius, theta);

        final var polygon3 = new Polygon3D(vertices2);

        // build plane from 3 vertices
        final var plane = new Plane(vertices2.get(0), vertices2.get(1), vertices2.get(2));
        expectedOrientation = plane.getDirectorVector();
        // normalize it
        final var norm = Utils.normF(expectedOrientation);
        ArrayUtils.multiplyByScalar(expectedOrientation, 1.0 / norm, expectedOrientation);

        // check correctness
        final var orientation11 = polygon3.getOrientation();

        assertArrayEquals(absArray(orientation11), absArray(expectedOrientation), ABSOLUTE_ERROR);
        final var orientation12 = polygon3.getOrientation(ABSOLUTE_ERROR);
        assertArrayEquals(absArray(orientation12), absArray(expectedOrientation), ABSOLUTE_ERROR);
        final var orientation13 = new double[INHOM_COORDS];
        polygon3.orientation(orientation13);
        assertArrayEquals(absArray(orientation13), absArray(expectedOrientation), ABSOLUTE_ERROR);
        polygon3.orientation(orientation13, ABSOLUTE_ERROR);
        assertArrayEquals(absArray(orientation13), absArray(expectedOrientation), ABSOLUTE_ERROR);
        final var orientation14 = Polygon3D.orientation(vertices2);
        assertArrayEquals(absArray(orientation14), absArray(expectedOrientation), ABSOLUTE_ERROR);
        final var orientation15 = Polygon3D.orientation(vertices2, ABSOLUTE_ERROR);
        assertArrayEquals(absArray(orientation15), absArray(expectedOrientation), ABSOLUTE_ERROR);
        final var orientation16 = Polygon3D.orientation(polygon3);
        assertArrayEquals(absArray(orientation16), absArray(expectedOrientation), ABSOLUTE_ERROR);
        final var orientation17 = Polygon3D.orientation(polygon3, ABSOLUTE_ERROR);
        assertArrayEquals(absArray(orientation17), absArray(expectedOrientation), ABSOLUTE_ERROR);
        final var orientation18 = new double[INHOM_COORDS];
        Polygon3D.orientation(vertices2, orientation18);
        assertArrayEquals(absArray(orientation18), absArray(expectedOrientation), ABSOLUTE_ERROR);
        Polygon3D.orientation(vertices2, orientation18, ABSOLUTE_ERROR);
        assertArrayEquals(absArray(orientation18), absArray(expectedOrientation), ABSOLUTE_ERROR);
        Polygon3D.orientation(polygon3, orientation18);
        assertArrayEquals(absArray(orientation18), absArray(expectedOrientation), ABSOLUTE_ERROR);
        Polygon3D.orientation(polygon3, orientation18, ABSOLUTE_ERROR);
        assertArrayEquals(absArray(orientation18), absArray(expectedOrientation), ABSOLUTE_ERROR);
    }

    @Test
    void testAngleBetweenPolygons() throws NotEnoughVerticesException, CoincidentPointsException {
        // X: -1, Y: 0, Z: 5 (W: 1)
        final var v1 = new InhomogeneousPoint3D(-1.0, 0.0, 5.0);
        // X: 1, Y: 0, Z: 5 (W: 1)
        final var v2 = new InhomogeneousPoint3D(1.0, 0.0, 5.0);
        // X: 2, Y: 1, Z: 5 (W: 1)
        final var v3 = new InhomogeneousPoint3D(2.0, 1.0, 5.0);
        // X: 0, Y: 1, Z: 5 (W: 1)
        final var v4 = new InhomogeneousPoint3D(0.0, 1.0, 5.0);

        // parallelepiped
        final var vertices1 = new ArrayList<Point3D>();
        final var vertices2 = new ArrayList<Point3D>();
        vertices1.add(v1);
        vertices1.add(v2);
        vertices1.add(v3);
        vertices1.add(v4);

        vertices2.add(v4);
        vertices2.add(v3);
        vertices2.add(v2);
        vertices2.add(v1);

        final var polygon1 = new Polygon3D(vertices1);
        // because order of vertices is reversed, orientation will have opposite
        // direction
        final var polygon2 = new Polygon3D(vertices2);

        var angle = Polygon3D.getAngleBetweenPolygons(vertices1, vertices2);
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
        assertThrows(IllegalArgumentException.class,
                () -> Polygon3D.getAngleBetweenPolygons(vertices1, vertices2, -ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class,
                () -> Polygon3D.getAngleBetweenPolygons(polygon1, polygon2, -ABSOLUTE_ERROR));

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
        assertThrows(CoincidentPointsException.class, () -> Polygon3D.getAngleBetweenPolygons(vertices1, vertices2));
        assertThrows(CoincidentPointsException.class,
                () -> Polygon3D.getAngleBetweenPolygons(vertices1, vertices2, ABSOLUTE_ERROR));
        assertThrows(CoincidentPointsException.class, () -> Polygon3D.getAngleBetweenPolygons(polygon1, polygon2));
        assertThrows(CoincidentPointsException.class,
                () -> Polygon3D.getAngleBetweenPolygons(polygon1, polygon2, ABSOLUTE_ERROR));
    }

    @Test
    void testSerializeDeserialize() throws NotEnoughVerticesException, TriangulatorException, IOException,
            ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final var radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        final var theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);

        // build vertices list
        final var vertices = buildPolygonVertices(sides, radius, theta);

        // build polygon
        var polygon1 = new Polygon3D(vertices);
        polygon1.triangulate();

        assertSame(vertices, polygon1.getVertices());
        assertFalse(polygon1.getTriangles().isEmpty());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(polygon1);
        final var polygon2 = SerializationHelper.<Polygon3D>deserialize(bytes);

        assertNotSame(polygon1, polygon2);
        assertEquals(polygon1.getVertices(), polygon2.getVertices());
        assertNotSame(polygon1.getVertices(), polygon2.getVertices());
        assertEquals(polygon1.getTriangles().size(), polygon2.getTriangles().size());
    }

    private static List<Point3D> buildPolygonVertices(
            final int sides, final double radius, final double theta) {
        final var vertices = new ArrayList<Point3D>(sides);
        for (var i = 0; i < sides; i++) {
            final var angle = (double) i / (double) sides * 2.0 * Math.PI;
            final var vertex = new InhomogeneousPoint3D(
                    radius * Math.cos(angle) * Math.cos(theta),
                    radius * Math.sin(angle) * Math.cos(theta),
                    radius * Math.sin(theta));
            vertices.add(vertex);
        }
        return vertices;
    }

    private static double[] absArray(final double[] array) {
        final var out = new double[array.length];
        for (var i = 0; i < array.length; i++) {
            out[i] = Math.abs(array[i]);
        }
        return out;
    }
}
