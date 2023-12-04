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

public class Polygon2DTest {

    private static final int MIN_SIDES = 6;
    private static final int MAX_SIDES = 12;

    private static final double MIN_RADIUS = 1.0;
    private static final double MAX_RADIUS = 10.0;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final int TIMES = 100;

    @Test
    public void testConstants() {
        assertEquals(1e-9, Polygon2D.DEFAULT_THRESHOLD, 0.0);
        assertEquals(0.0, Polygon2D.MIN_THRESHOLD, 0.0);
        assertEquals(3, Polygon2D.MIN_VERTICES);
        assertEquals(TriangulatorMethod.VAN_GOGH_TRIANGULATOR,
                Polygon2D.DEFAULT_TRIANGULATOR_METHOD);
    }

    @Test
    public void testConstructor() throws NotEnoughVerticesException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        // build vertices list
        final List<Point2D> vertices = buildPolygonVertices(sides, radius);

        // build polygon
        Polygon2D polygon = new Polygon2D(vertices);

        // check correctness
        assertFalse(polygon.isTriangulated());
        assertEquals(polygon.getVertices(), vertices);
        assertEquals(Polygon2D.DEFAULT_TRIANGULATOR_METHOD,
                polygon.getTriangulatorMethod());

        final Iterator<Point2D> iterator1 = polygon.getVertices().iterator();
        final Iterator<Point2D> iterator2 = vertices.iterator();

        Point2D vertex1;
        Point2D vertex2;
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
            polygon = new Polygon2D(vertices);
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

        // build vertices list
        final List<Point2D> vertices = buildPolygonVertices(sides, radius);

        // build polygon
        final Polygon2D polygon = new Polygon2D(vertices);

        // check correctness
        assertEquals(Polygon2D.DEFAULT_TRIANGULATOR_METHOD,
                polygon.getTriangulatorMethod());

        // set new method
        polygon.setTriangulatorMethod(TriangulatorMethod.VAN_GOGH_TRIANGULATOR);
        // check correctness
        assertEquals(TriangulatorMethod.VAN_GOGH_TRIANGULATOR,
                polygon.getTriangulatorMethod());
    }

    @Test
    public void testGetSetVertices1() throws NotEnoughVerticesException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        // build vertices list
        final List<Point2D> vertices = buildPolygonVertices(sides, radius);

        // build polygon vertices
        final Polygon2D polygon = new Polygon2D(vertices);
        assertEquals(vertices, polygon.getVertices());

        // build new vertices
        sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        final List<Point2D> vertices2 = buildPolygonVertices(sides, radius);

        polygon.setVertices(vertices2);
        assertEquals(vertices2, polygon.getVertices());
    }

    @Test
    public void testGetSetVertices2() throws NotEnoughVerticesException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        // build vertices list
        final List<Point2D> vertices = new LinkedList<>(buildPolygonVertices(sides, radius));

        // build polygon vertices
        final Polygon2D polygon = new Polygon2D(vertices);
        assertEquals(polygon.getVertices(), vertices);

        // build new vertices
        sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        final List<Point2D> vertices2 = new LinkedList<>(buildPolygonVertices(sides, radius));

        polygon.setVertices(vertices2);
        assertEquals(vertices2, polygon.getVertices());
    }


    @Test
    public void testTriangulateIsTriangulatedAndArea()
            throws NotEnoughVerticesException, TriangulatorException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        // build vertices list
        final List<Point2D> vertices = buildPolygonVertices(sides, radius);

        final Polygon2D polygon = new Polygon2D(vertices);

        // polygon is regular, hence its area will be:
        final double area = 0.5 * sides * radius * radius * Math.sin(
                2.0 * Math.PI / (double) sides);

        assertFalse(polygon.isTriangulated());
        final List<Triangle2D> triangles = polygon.getTriangles();
        assertTrue(polygon.isTriangulated());

        final double areaTriangles;
        double signedAreaTriangles = 0.0;
        for (final Triangle2D triangle : triangles) {
            signedAreaTriangles += triangle.getSignedArea();
        }
        areaTriangles = Math.abs(signedAreaTriangles);

        assertEquals(area, polygon.getArea(), ABSOLUTE_ERROR);
        assertEquals(areaTriangles, polygon.getArea(), ABSOLUTE_ERROR);
        assertEquals(signedAreaTriangles, polygon.getSignedArea(), ABSOLUTE_ERROR);

        // triangulate again
        polygon.triangulate();

        assertTrue(polygon.isTriangulated());
    }

    @Test
    public void testGetAreaSignedAreaAndAreVerticesClockwise()
            throws NotEnoughVerticesException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        // build vertices list
        final List<Point2D> vertices = buildPolygonVertices(sides, radius);

        final Polygon2D polygon = new Polygon2D(vertices);

        // polygon is regular, hence its area will be:
        double area = 0.5 * sides * radius * radius * Math.sin(
                2.0 * Math.PI / (double) sides);
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
        final List<Point2D> vertices2 = buildPolygonVertices(sides, radius);
        final Triangle2D triangle = new Triangle2D(vertices2.get(0), vertices2.get(1), vertices2.get(2));
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
    public void testPerimeter() throws NotEnoughVerticesException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        // build vertices list
        final List<Point2D> vertices = buildPolygonVertices(sides, radius);

        final Polygon2D polygon = new Polygon2D(vertices);

        final Iterator<Point2D> iterator = vertices.iterator();
        Point2D prevVertex = iterator.next();
        Point2D curVertex;
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
        final List<Point2D> vertices2 = buildPolygonVertices(sides, radius);
        final Triangle2D triangle = new Triangle2D(vertices2.get(0), vertices2.get(1), vertices2.get(2));
        polygon.setVertices(vertices2);
        assertEquals(triangle.getPerimeter(), polygon.getPerimeter(), ABSOLUTE_ERROR);
    }

    @Test
    public void testIsInside() throws NotEnoughVerticesException,
            TriangulatorException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        // build vertices list
        final List<Point2D> vertices = buildPolygonVertices(sides, radius);

        final Polygon2D polygon = new Polygon2D(vertices);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        // create point inside of vertex
        final Point2D inside = new InhomogeneousPoint2D(
                radius / 2.0 * Math.cos(theta),
                radius / 2.0 * Math.sin(theta));

        // create point outside of vertex
        final Point2D outside = new InhomogeneousPoint2D(
                2.0 * radius * Math.cos(theta),
                2.0 * radius * Math.sin(theta));

        assertTrue(polygon.isInside(inside));
        assertTrue(polygon.isInside(inside, ABSOLUTE_ERROR));
        assertFalse(polygon.isInside(outside));
        assertFalse(polygon.isInside(outside, ABSOLUTE_ERROR));

        // check that vertices are inside
        for (final Point2D vertex : vertices) {
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

        // build vertices list
        final List<Point2D> vertices = buildPolygonVertices(sides, radius);

        double inhomX = 0.0;
        double inhomY = 0.0;
        for (final Point2D vertex : vertices) {
            inhomX += vertex.getInhomX() / (double) sides;
            inhomY += vertex.getInhomY() / (double) sides;
        }

        final Point2D center = new InhomogeneousPoint2D(inhomX, inhomY);

        final Polygon2D polygon = new Polygon2D(vertices);

        assertTrue(center.equals(polygon.getCenter(), ABSOLUTE_ERROR));
        final Point2D center2 = Point2D.create();
        polygon.center(center2);
        assertTrue(center.equals(center2, ABSOLUTE_ERROR));
    }

    @Test
    public void testIsLocusGetShortestDistanceAndClosestPoint()
            throws NotEnoughVerticesException, TriangulatorException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
            final double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
            double dist = randomizer.nextDouble(2.0 * ABSOLUTE_ERROR, radius / 2.0);

            final double theta = randomizer.nextDouble(0.0,
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            int n = (int) (theta / (2 * Math.PI) * (double) sides);

            final List<Point2D> vertices = buildPolygonVertices(sides, radius);

            final Point2D point1 = vertices.get(n);
            n = (n + 1) % sides;
            final Point2D point2 = vertices.get(n);

            final Line2D line = new Line2D(point1, point2);
            line.normalize();

            final double[] directorVector = line.getDirectorVector();
            final double norm = Utils.normF(directorVector);
            ArrayUtils.multiplyByScalar(directorVector, 1.0 / norm,
                    directorVector);

            // find point laying on the line between polygon vertices
            final Point2D testPoint = new InhomogeneousPoint2D(
                    radius * Math.cos(theta), radius * Math.sin(theta));
            // point below is locus of polygon
            final Point2D locusPoint = line.getClosestPoint(testPoint);
            // locus point is between point1 and point2
            assertTrue(locusPoint.isBetween(point1, point2));

            // move point a little bit away from locus
            final Point2D notLocusPoint = new InhomogeneousPoint2D(
                    locusPoint.getInhomX() + dist * directorVector[0],
                    locusPoint.getInhomY() + dist * directorVector[1]);
            // ensure that notLocusPoint lies outside of polygon
            final Polygon2D polygon = new Polygon2D(vertices);

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
            final Point2D closestPoint = Point2D.create();
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
            for (final Point2D vertex : vertices) {
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

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSerializeDeserialize() throws NotEnoughVerticesException,
            TriangulatorException, IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        // build vertices list
        final List<Point2D> vertices = buildPolygonVertices(sides, radius);

        final Polygon2D polygon1 = new Polygon2D(vertices);
        polygon1.triangulate();

        assertSame(vertices, polygon1.getVertices());
        assertFalse(polygon1.getTriangles().isEmpty());

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(polygon1);
        final Polygon2D polygon2 = SerializationHelper.deserialize(bytes);

        assertNotSame(polygon1, polygon2);
        assertEquals(polygon1.getVertices(), polygon2.getVertices());
        assertNotSame(polygon1.getVertices(), polygon2.getVertices());
        assertEquals(polygon1.getTriangles().size(),
                polygon2.getTriangles().size());
    }

    private List<Point2D> buildPolygonVertices(final int sides, final double radius) {
        final List<Point2D> vertices = new ArrayList<>(sides);
        Point2D vertex;
        for (int i = 0; i < sides; i++) {
            final double angle = (double) i / (double) sides * 2.0 * Math.PI;
            vertex = new InhomogeneousPoint2D(radius * Math.cos(angle),
                    radius * Math.sin(angle));
            vertices.add(vertex);
        }
        return vertices;
    }
}
