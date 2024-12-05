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
import java.util.*;

import static org.junit.jupiter.api.Assertions.*;

class Polygon2DTest {

    private static final int MIN_SIDES = 6;
    private static final int MAX_SIDES = 12;

    private static final double MIN_RADIUS = 1.0;
    private static final double MAX_RADIUS = 10.0;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final int TIMES = 100;

    @Test
    void testConstants() {
        assertEquals(1e-9, Polygon2D.DEFAULT_THRESHOLD, 0.0);
        assertEquals(0.0, Polygon2D.MIN_THRESHOLD, 0.0);
        assertEquals(3, Polygon2D.MIN_VERTICES);
        assertEquals(TriangulatorMethod.VAN_GOGH_TRIANGULATOR, Polygon2D.DEFAULT_TRIANGULATOR_METHOD);
    }

    @Test
    void testConstructor() throws NotEnoughVerticesException {
        final var randomizer = new UniformRandomizer();
        final var sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final var radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        // build vertices list
        final List<Point2D> vertices = buildPolygonVertices(sides, radius);

        // build polygon
        var polygon = new Polygon2D(vertices);

        // check correctness
        assertFalse(polygon.isTriangulated());
        assertEquals(polygon.getVertices(), vertices);
        assertEquals(Polygon2D.DEFAULT_TRIANGULATOR_METHOD, polygon.getTriangulatorMethod());

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
        assertThrows(NotEnoughVerticesException.class, () -> new Polygon2D(vertices));
    }

    @Test
    void testGetSetTriangulatorMethod() throws NotEnoughVerticesException {
        final var randomizer = new UniformRandomizer();
        final var sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final var radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        // build vertices list
        final var vertices = buildPolygonVertices(sides, radius);

        // build polygon
        final var polygon = new Polygon2D(vertices);

        // check correctness
        assertEquals(Polygon2D.DEFAULT_TRIANGULATOR_METHOD, polygon.getTriangulatorMethod());

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

        // build vertices list
        final var vertices = buildPolygonVertices(sides, radius);

        // build polygon vertices
        final var polygon = new Polygon2D(vertices);
        assertEquals(vertices, polygon.getVertices());

        // build new vertices
        sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        final var vertices2 = buildPolygonVertices(sides, radius);

        polygon.setVertices(vertices2);
        assertEquals(vertices2, polygon.getVertices());
    }

    @Test
    void testGetSetVertices2() throws NotEnoughVerticesException {
        final var randomizer = new UniformRandomizer();
        var sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        var radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        // build vertices list
        final var vertices = new LinkedList<>(buildPolygonVertices(sides, radius));

        // build polygon vertices
        final var polygon = new Polygon2D(vertices);
        assertEquals(polygon.getVertices(), vertices);

        // build new vertices
        sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        final var vertices2 = new LinkedList<>(buildPolygonVertices(sides, radius));

        polygon.setVertices(vertices2);
        assertEquals(vertices2, polygon.getVertices());
    }


    @Test
    void testTriangulateIsTriangulatedAndArea() throws NotEnoughVerticesException, TriangulatorException {
        final var randomizer = new UniformRandomizer();
        final var sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final var radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        // build vertices list
        final var vertices = buildPolygonVertices(sides, radius);

        final var polygon = new Polygon2D(vertices);

        // polygon is regular, hence its area will be:
        final var area = 0.5 * sides * radius * radius * Math.sin(2.0 * Math.PI / (double) sides);

        assertFalse(polygon.isTriangulated());
        final var triangles = polygon.getTriangles();
        assertTrue(polygon.isTriangulated());

        var signedAreaTriangles = 0.0;
        for (final var triangle : triangles) {
            signedAreaTriangles += triangle.getSignedArea();
        }
        final var areaTriangles = Math.abs(signedAreaTriangles);

        assertEquals(area, polygon.getArea(), ABSOLUTE_ERROR);
        assertEquals(areaTriangles, polygon.getArea(), ABSOLUTE_ERROR);
        assertEquals(signedAreaTriangles, polygon.getSignedArea(), ABSOLUTE_ERROR);

        // triangulate again
        polygon.triangulate();

        assertTrue(polygon.isTriangulated());
    }

    @Test
    void testGetAreaSignedAreaAndAreVerticesClockwise() throws NotEnoughVerticesException {
        final var randomizer = new UniformRandomizer();
        var sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final var radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        // build vertices list
        final var vertices = buildPolygonVertices(sides, radius);

        final var polygon = new Polygon2D(vertices);

        // polygon is regular, hence its area will be:
        var area = 0.5 * sides * radius * radius * Math.sin(2.0 * Math.PI / (double) sides);
        // because vertices are defined counterclockwise, signed area will be
        // positive
        assertEquals(area, polygon.getSignedArea(), ABSOLUTE_ERROR);
        assertEquals(area, polygon.getArea(), ABSOLUTE_ERROR);
        assertFalse(polygon.areVerticesClockwise());
        assertFalse(polygon.areVerticesClockwise(0.0));

        // if we reverse the order of vertices
        Collections.reverse(vertices);

        polygon.setVertices(vertices);
        // now vertices are defined clockwise, and signed area will be negative
        assertEquals(-area, polygon.getSignedArea(), ABSOLUTE_ERROR);
        assertEquals(area, polygon.getArea(), ABSOLUTE_ERROR);
        assertTrue(polygon.areVerticesClockwise());
        assertTrue(polygon.areVerticesClockwise(0.0));

        // Test for a triangle
        sides = 3;
        final var vertices2 = buildPolygonVertices(sides, radius);
        final var triangle = new Triangle2D(vertices2.get(0), vertices2.get(1), vertices2.get(2));
        polygon.setVertices(vertices2);
        area = 0.5 * sides * radius * radius * Math.sin(2.0 * Math.PI / (double) sides);

        // check correctness
        assertEquals(area, triangle.getSignedArea(), ABSOLUTE_ERROR);
        assertEquals(area, polygon.getSignedArea(), ABSOLUTE_ERROR);
        assertFalse(polygon.areVerticesClockwise());
        assertFalse(polygon.areVerticesClockwise(0.0));

        // and if we revere order of vertices...
        Collections.reverse(vertices2);
        polygon.setVertices(vertices2);
        assertEquals(-area, polygon.getSignedArea(), ABSOLUTE_ERROR);
        assertTrue(polygon.areVerticesClockwise());
        assertTrue(polygon.areVerticesClockwise(0.0));
    }

    @Test
    void testPerimeter() throws NotEnoughVerticesException {
        final var randomizer = new UniformRandomizer();
        var sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final var radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        // build vertices list
        final var vertices = buildPolygonVertices(sides, radius);

        final var polygon = new Polygon2D(vertices);

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
        final var vertices2 = buildPolygonVertices(sides, radius);
        final var triangle = new Triangle2D(vertices2.get(0), vertices2.get(1), vertices2.get(2));
        polygon.setVertices(vertices2);
        assertEquals(triangle.getPerimeter(), polygon.getPerimeter(), ABSOLUTE_ERROR);
    }

    @Test
    void testIsInside() throws NotEnoughVerticesException, TriangulatorException {

        final var randomizer = new UniformRandomizer();
        final var sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final var radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        // build vertices list
        final var vertices = buildPolygonVertices(sides, radius);

        final var polygon = new Polygon2D(vertices);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        // create point inside of vertex
        final var inside = new InhomogeneousPoint2D(
                radius / 2.0 * Math.cos(theta),
                radius / 2.0 * Math.sin(theta));

        // create point outside of vertex
        final var outside = new InhomogeneousPoint2D(
                2.0 * radius * Math.cos(theta),
                2.0 * radius * Math.sin(theta));

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

        // build vertices list
        final var vertices = buildPolygonVertices(sides, radius);

        var inhomX = 0.0;
        var inhomY = 0.0;
        for (final var vertex : vertices) {
            inhomX += vertex.getInhomX() / (double) sides;
            inhomY += vertex.getInhomY() / (double) sides;
        }

        final var center = new InhomogeneousPoint2D(inhomX, inhomY);

        final var polygon = new Polygon2D(vertices);

        assertTrue(center.equals(polygon.getCenter(), ABSOLUTE_ERROR));
        final var center2 = Point2D.create();
        polygon.center(center2);
        assertTrue(center.equals(center2, ABSOLUTE_ERROR));
    }

    @Test
    void testIsLocusGetShortestDistanceAndClosestPoint() throws NotEnoughVerticesException, TriangulatorException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
            final var radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
            var dist = randomizer.nextDouble(2.0 * ABSOLUTE_ERROR, radius / 2.0);

            final var theta = Math.toRadians(randomizer.nextDouble(0.0, 2.0 * MAX_ANGLE_DEGREES));

            var n = (int) (theta / (2 * Math.PI) * (double) sides);

            final var vertices = buildPolygonVertices(sides, radius);

            final var point1 = vertices.get(n);
            n = (n + 1) % sides;
            final var point2 = vertices.get(n);

            final var line = new Line2D(point1, point2);
            line.normalize();

            final var directorVector = line.getDirectorVector();
            final var norm = Utils.normF(directorVector);
            ArrayUtils.multiplyByScalar(directorVector, 1.0 / norm, directorVector);

            // find point laying on the line between polygon vertices
            final var testPoint = new InhomogeneousPoint2D(radius * Math.cos(theta), radius * Math.sin(theta));
            // point below is locus of polygon
            final var locusPoint = line.getClosestPoint(testPoint);
            // locus point is between point1 and point2
            assertTrue(locusPoint.isBetween(point1, point2));

            // move point a little bit away from locus
            final var notLocusPoint = new InhomogeneousPoint2D(
                    locusPoint.getInhomX() + dist * directorVector[0],
                    locusPoint.getInhomY() + dist * directorVector[1]);
            // ensure that notLocusPoint lies outside of polygon
            final var polygon = new Polygon2D(vertices);

            if (polygon.isInside(notLocusPoint)) {
                // change sign of dist to move point in opposite direction
                dist *= -1.0;
                notLocusPoint.setInhomogeneousCoordinates(
                        locusPoint.getInhomX() + dist * directorVector[0],
                        locusPoint.getInhomY() + dist * directorVector[1]);
            }
            assertFalse(polygon.isInside(notLocusPoint));

            // check that notLocusPoint is at distance dist from line
            assertEquals(Math.abs(dist), Math.abs(line.signedDistance(notLocusPoint)), ABSOLUTE_ERROR);

            assertTrue(polygon.isLocus(locusPoint));
            assertTrue(polygon.isLocus(locusPoint, ABSOLUTE_ERROR));

            // because point is locus, the shortest distance is zero and it is
            // the closest point
            assertEquals(0.0, polygon.getShortestDistance(locusPoint), ABSOLUTE_ERROR);
            assertTrue(polygon.getClosestPoint(locusPoint).equals(locusPoint, ABSOLUTE_ERROR));
            final var closestPoint = Point2D.create();
            polygon.closestPoint(locusPoint, closestPoint);
            assertTrue(closestPoint.equals(locusPoint, ABSOLUTE_ERROR));
            assertTrue(closestPoint.equals(locusPoint, ABSOLUTE_ERROR));

            assertFalse(polygon.isLocus(notLocusPoint));
            if (polygon.isLocus(notLocusPoint, ABSOLUTE_ERROR)) {
                continue;
            }
            assertFalse(polygon.isLocus(notLocusPoint, ABSOLUTE_ERROR));

            // not locus point is at distance dist from polygon
            assertEquals(polygon.getShortestDistance(notLocusPoint), Math.abs(dist), ABSOLUTE_ERROR);
            // and the closest point to polygon is locusPoint
            assertTrue(polygon.getClosestPoint(notLocusPoint).equals(locusPoint, ABSOLUTE_ERROR));
            polygon.closestPoint(notLocusPoint, closestPoint);
            assertTrue(closestPoint.equals(locusPoint, ABSOLUTE_ERROR));
            assertTrue(closestPoint.equals(locusPoint, ABSOLUTE_ERROR));

            // with a large enough threshold, not locus point is considered as
            // locus
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

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testSerializeDeserialize() throws NotEnoughVerticesException, TriangulatorException, IOException,
            ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final var radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        // build vertices list
        final var vertices = buildPolygonVertices(sides, radius);

        final var polygon1 = new Polygon2D(vertices);
        polygon1.triangulate();

        assertSame(vertices, polygon1.getVertices());
        assertFalse(polygon1.getTriangles().isEmpty());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(polygon1);
        final var polygon2 = SerializationHelper.<Polygon2D>deserialize(bytes);

        assertNotSame(polygon1, polygon2);
        assertEquals(polygon1.getVertices(), polygon2.getVertices());
        assertNotSame(polygon1.getVertices(), polygon2.getVertices());
        assertEquals(polygon1.getTriangles().size(), polygon2.getTriangles().size());
    }

    private static List<Point2D> buildPolygonVertices(final int sides, final double radius) {
        final var vertices = new ArrayList<Point2D>(sides);
        for (var i = 0; i < sides; i++) {
            final var angle = (double) i / (double) sides * 2.0 * Math.PI;
            final var vertex = new InhomogeneousPoint2D(radius * Math.cos(angle), radius * Math.sin(angle));
            vertices.add(vertex);
        }
        return vertices;
    }
}
