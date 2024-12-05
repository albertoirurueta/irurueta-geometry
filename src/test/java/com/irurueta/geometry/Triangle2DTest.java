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

import com.irurueta.algebra.DecomposerException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class Triangle2DTest {

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final int TIMES = 100;

    @Test
    void testConstants() {
        assertEquals(1e-9, Triangle2D.DEFAULT_THRESHOLD, 0.0);
        assertEquals(0.0, Triangle2D.MIN_THRESHOLD, 0.0);
        assertEquals(3, Triangle2D.NUM_VERTICES);
    }

    @Test
    void testConstructor() {
        final var randomizer = new UniformRandomizer();

        final var point1 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point2 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point3 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        var triangle = new Triangle2D(point1, point2, point3);

        assertEquals(triangle.getVertex1(), point1);
        assertEquals(triangle.getVertex2(), point2);
        assertEquals(triangle.getVertex3(), point3);

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new Triangle2D(null, point2, point3));
        assertThrows(NullPointerException.class, () -> new Triangle2D(point1, null, point3));
        assertThrows(NullPointerException.class, () -> new Triangle2D(point1, point2, null));
    }

    @Test
    void testGetSetVertex1() {
        final var randomizer = new UniformRandomizer();

        final var point1 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point2 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point3 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var triangle = new Triangle2D(point1, point2, point3);

        assertEquals(point1, triangle.getVertex1());
        assertEquals(point2, triangle.getVertex2());
        assertEquals(point3, triangle.getVertex3());

        // new vertex1
        final var vertex1 = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        triangle.setVertex1(vertex1);
        // check correctness
        assertEquals(vertex1, triangle.getVertex1());
        assertEquals(point2, triangle.getVertex2());
        assertEquals(point3, triangle.getVertex3());

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> triangle.setVertex1(null));
    }

    @Test
    void testGetSetVertex2() {
        final var randomizer = new UniformRandomizer();

        final var point1 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point2 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point3 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var triangle = new Triangle2D(point1, point2, point3);

        assertEquals(triangle.getVertex1(), point1);
        assertEquals(triangle.getVertex2(), point2);
        assertEquals(triangle.getVertex3(), point3);

        // new vertex1
        final var vertex2 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        triangle.setVertex2(vertex2);
        // check correctness
        assertEquals(point1, triangle.getVertex1());
        assertEquals(vertex2, triangle.getVertex2());
        assertEquals(point3, triangle.getVertex3());

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> triangle.setVertex2(null));
    }

    @Test
    void testGetSetVertex3() {
        final var randomizer = new UniformRandomizer();

        final var point1 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point2 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point3 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var triangle = new Triangle2D(point1, point2, point3);

        assertEquals(point1, triangle.getVertex1());
        assertEquals(point2, triangle.getVertex2());
        assertEquals(point3, triangle.getVertex3());

        // new vertex1
        final var vertex3 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        triangle.setVertex3(vertex3);
        // check correctness
        assertEquals(point1, triangle.getVertex1());
        assertEquals(point2, triangle.getVertex2());
        assertEquals(vertex3, triangle.getVertex3());

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> triangle.setVertex3(null));
    }

    @Test
    void testGetSetVertices() {
        final var randomizer = new UniformRandomizer();

        final var point1 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point2 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point3 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var triangle = new Triangle2D(point1, point2, point3);

        assertEquals(point1, triangle.getVertex1());
        assertEquals(point2, triangle.getVertex2());
        assertEquals(point3, triangle.getVertex3());

        final var point1b = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point2b = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point3b = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        // Set new vertices
        triangle.setVertices(point1b, point2b, point3b);

        // check correctness
        assertEquals(point1b, triangle.getVertex1());
        assertEquals(point2b, triangle.getVertex2());
        assertEquals(point3b, triangle.getVertex3());

        // get vertices as a list
        final var vertices = triangle.getVertices();
        final var vertices2 = new ArrayList<Point2D>();
        triangle.vertices(vertices2);

        assertEquals(Triangle2D.NUM_VERTICES, vertices.size());
        assertEquals(Triangle2D.NUM_VERTICES, vertices2.size());

        assertTrue(vertices.get(0).equals(point1b, ABSOLUTE_ERROR));
        assertTrue(vertices.get(1).equals(point2b, ABSOLUTE_ERROR));
        assertTrue(vertices.get(2).equals(point3b, ABSOLUTE_ERROR));

        assertTrue(vertices2.get(0).equals(point1b, ABSOLUTE_ERROR));
        assertTrue(vertices2.get(1).equals(point2b, ABSOLUTE_ERROR));
        assertTrue(vertices2.get(2).equals(point3b, ABSOLUTE_ERROR));

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> triangle.setVertices(null, point2, point3));
        assertThrows(NullPointerException.class, () -> triangle.setVertices(point1, null, point3));
        assertThrows(NullPointerException.class, () -> triangle.setVertices(point1, point2, null));
    }

    @Test
    void testAreaSignedAreaAndAreColinearPoints() throws WrongSizeException, DecomposerException {

        final var randomizer = new UniformRandomizer();
        final var base = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var height = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        // Test known and simple values
        var point1 = new InhomogeneousPoint2D(0.0, 0.0);
        var point2 = new InhomogeneousPoint2D(base, 0.0);
        var point3 = new InhomogeneousPoint2D(base / 2.0, height);

        var expectedArea = base * height / 2.0;

        final var triangle1 = new Triangle2D(point1, point2, point3);

        assertEquals(Math.abs(Triangle2D.signedArea(triangle1)), expectedArea, ABSOLUTE_ERROR);
        assertEquals(Math.abs(Triangle2D.signedArea(point1, point2, point3)), expectedArea, ABSOLUTE_ERROR);
        assertEquals(Math.abs(triangle1.getSignedArea()), expectedArea, ABSOLUTE_ERROR);

        assertEquals(Triangle2D.area(triangle1), expectedArea, ABSOLUTE_ERROR);
        assertEquals(Triangle2D.area(point1, point2, point3), expectedArea, ABSOLUTE_ERROR);
        assertEquals(triangle1.getArea(), expectedArea, ABSOLUTE_ERROR);

        if (expectedArea > Triangle2D.DEFAULT_THRESHOLD) {
            assertFalse(triangle1.areVerticesColinear());
        } else {
            assertTrue(triangle1.areVerticesColinear());
        }

        // if threshold is large enough, points will always be considered to be
        // co-linear
        assertTrue(triangle1.areVerticesColinear(expectedArea));


        // Test with random values
        point1 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point2 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point3 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var m = new Matrix(2, 2);
        // 1st column
        m.setElementAt(0, 0, point2.getInhomX() - point1.getInhomX());
        m.setElementAt(1, 0, point2.getInhomY() - point1.getInhomY());
        // 2nd columns
        m.setElementAt(0, 1, point3.getInhomX() - point1.getInhomX());
        m.setElementAt(1, 1, point3.getInhomY() - point1.getInhomY());

        // expected area
        final var expectedSignedArea = 0.5 * Utils.det(m);
        expectedArea = Math.abs(expectedSignedArea);

        final var triangle2 = new Triangle2D(point1, point2, point3);

        assertEquals(expectedSignedArea, Triangle2D.signedArea(triangle2), ABSOLUTE_ERROR);
        assertEquals(expectedSignedArea, Triangle2D.signedArea(point1, point2, point3), ABSOLUTE_ERROR);
        assertEquals(expectedSignedArea, triangle2.getSignedArea(), ABSOLUTE_ERROR);

        assertEquals(expectedArea, Triangle2D.area(triangle2), ABSOLUTE_ERROR);
        assertEquals(expectedArea, Triangle2D.area(point1, point2, point3), ABSOLUTE_ERROR);
        assertEquals(expectedArea, triangle2.getArea(), ABSOLUTE_ERROR);

        if (expectedArea > Triangle2D.DEFAULT_THRESHOLD) {
            assertFalse(triangle2.areVerticesColinear());
        } else {
            assertTrue(triangle2.areVerticesColinear());
        }

        // if threshold is large enough, points will always be considered to be
        // co-linear
        assertTrue(triangle2.areVerticesColinear(expectedArea + ABSOLUTE_ERROR));

        // If two points are coincident, then area must be zero or close to zero
        final var triangle3 = new Triangle2D(point1, point1, point2);
        assertEquals(0.0, Triangle2D.signedArea(triangle3), ABSOLUTE_ERROR);
        assertEquals(0.0, Triangle2D.signedArea(point1, point1, point2), ABSOLUTE_ERROR);
        assertEquals(0.0, triangle3.getSignedArea(), ABSOLUTE_ERROR);

        assertEquals(0.0, Triangle2D.area(triangle3), ABSOLUTE_ERROR);
        assertEquals(0.0, Triangle2D.area(point1, point1, point2), ABSOLUTE_ERROR);
        assertEquals(0.0, triangle3.getArea(), ABSOLUTE_ERROR);

        // because area is zero, then points are co-linear
        assertTrue(triangle3.areVerticesColinear());
        assertTrue(triangle3.areVerticesColinear(ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> triangle3.areVerticesColinear(-ABSOLUTE_ERROR));
    }

    @Test
    void testIsInsideShortestDistanceAndIsLocus() {
        for (var t = 0; t < TIMES; t++) {
            // Test for known values
            final var randomizer = new UniformRandomizer();
            double base;
            double height;
            double area;
            do {
                // we iterate until we ensure that triangle is not too small
                base = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                height = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                area = 0.5 * Math.abs(base * height);
            } while (area < 1.0);
            final var dist = randomizer.nextDouble(Triangle2D.DEFAULT_THRESHOLD, MAX_RANDOM_VALUE);

            // Test known and simple values
            final var point1 = new InhomogeneousPoint2D(0.0, 0.0);
            final var point2 = new InhomogeneousPoint2D(base, 0.0);
            final var point3 = new InhomogeneousPoint2D(base / 2.0, height);

            final var triangle = new Triangle2D(point1, point2, point3);
            var center = triangle.getCenter();

            // vertices and center lie inside the triangle
            assertTrue(triangle.isInside(point1));
            assertTrue(triangle.isInside(point2));
            assertTrue(triangle.isInside(point3));
            assertTrue(triangle.isInside(center));

            // test shortest distance
            assertEquals(0.0, triangle.getShortestDistance(point1), 0.0);
            assertEquals(0.0, triangle.getShortestDistance(point2), 0.0);
            assertEquals(0.0, triangle.getShortestDistance(point3), 0.0);
            var line1 = new Line2D(point1, point2);
            var line2 = new Line2D(point1, point3);
            var line3 = new Line2D(point2, point3);
            var dist1 = Math.abs(line1.signedDistance(center));
            var dist2 = Math.abs(line2.signedDistance(center));
            var dist3 = Math.abs(line3.signedDistance(center));
            var centerDist = Math.min(dist1, Math.min(dist2, dist3));
            assertEquals(triangle.getShortestDistance(center), centerDist, ABSOLUTE_ERROR);

            // test is locus (vertices will be locus, but not center)
            assertTrue(triangle.isLocus(point1));
            assertTrue(triangle.isLocus(point2));
            assertTrue(triangle.isLocus(point3));
            assertFalse(triangle.isLocus(center));

            assertTrue(Triangle2D.isInside(triangle, point1));
            assertTrue(Triangle2D.isInside(triangle, point2));
            assertTrue(Triangle2D.isInside(triangle, point3));
            assertTrue(Triangle2D.isInside(triangle, center));

            assertEquals(0.0, Triangle2D.shortestDistance(triangle, point1), 0.0);
            assertEquals(0.0, Triangle2D.shortestDistance(triangle, point2), 0.0);
            assertEquals(0.0, Triangle2D.shortestDistance(triangle, point3), 0.0);
            assertEquals(centerDist, Triangle2D.shortestDistance(triangle, center), ABSOLUTE_ERROR);

            assertTrue(Triangle2D.isInside(point1, point2, point3, point1));
            assertTrue(Triangle2D.isInside(point1, point2, point3, point2));
            assertTrue(Triangle2D.isInside(point1, point2, point3, point3));
            assertTrue(Triangle2D.isInside(point1, point2, point3, center));

            assertEquals(0.0, Triangle2D.shortestDistance(point1, point2, point3, point1), 0.0);
            assertEquals(0.0, Triangle2D.shortestDistance(point1, point2, point3, point2), 0.0);
            assertEquals(0.0, Triangle2D.shortestDistance(point1, point2, point3, point3), 0.0);
            assertEquals(centerDist, Triangle2D.shortestDistance(point1, point2, point3, center), ABSOLUTE_ERROR);

            // the same is true for a small threshold
            assertTrue(triangle.isInside(point1, 0.0));
            assertTrue(triangle.isInside(point2, 0.0));
            assertTrue(triangle.isInside(point3, 0.0));

            // check is locus with a small threshold
            assertTrue(triangle.isLocus(point1, 0.0));
            assertTrue(triangle.isLocus(point1, 0.0));
            assertTrue(triangle.isLocus(point1, 0.0));
            assertFalse(triangle.isLocus(center, 0.0));

            assertTrue(Triangle2D.isInside(triangle, point1, 0.0));
            assertTrue(Triangle2D.isInside(triangle, point2, 0.0));
            assertTrue(Triangle2D.isInside(triangle, point3, 0.0));

            assertTrue(Triangle2D.isInside(point1, point2, point3, point1, 0.0));
            assertTrue(Triangle2D.isInside(point1, point2, point3, point2, 0.0));
            assertTrue(Triangle2D.isInside(point1, point2, point3, point3, 0.0));

            // Force IllegalArgumentException
            assertThrows(IllegalArgumentException.class, () -> triangle.isInside(point1, -ABSOLUTE_ERROR));
            assertThrows(IllegalArgumentException.class, () -> Triangle2D.isInside(triangle, point1, -ABSOLUTE_ERROR));
            assertThrows(IllegalArgumentException.class, () -> Triangle2D.isInside(point1, point2, point3, point1,
                    -ABSOLUTE_ERROR));
            assertThrows(IllegalArgumentException.class, () -> triangle.isLocus(point3, -ABSOLUTE_ERROR));

            // Check point outside
            final var outside = new InhomogeneousPoint2D(-dist, 0.0);

            assertFalse(triangle.isInside(outside));

            // point outside is not locus
            assertFalse(triangle.isLocus(outside));

            assertFalse(Triangle2D.isInside(triangle, outside));

            assertFalse(Triangle2D.isInside(point1, point2, point3, outside));

            // the same is true for a small threshold, but point is considered to
            // be inside when setting large threshold
            assertFalse(triangle.isInside(outside, 0.0));
            assertTrue(triangle.isInside(outside, 3.0 * MAX_RANDOM_VALUE * MAX_RANDOM_VALUE));

            assertEquals(triangle.getShortestDistance(outside), dist, ABSOLUTE_ERROR);

            assertFalse(Triangle2D.isInside(triangle, outside, 0.0));
            assertTrue(Triangle2D.isInside(triangle, outside, 3.0 * MAX_RANDOM_VALUE * MAX_RANDOM_VALUE));

            // check locus with a small and large threshold
            assertFalse(triangle.isLocus(outside, 0.0));
            assertTrue(triangle.isLocus(outside, 3.0 * MAX_RANDOM_VALUE * MAX_RANDOM_VALUE));

            assertEquals(Triangle2D.shortestDistance(triangle, outside), dist, ABSOLUTE_ERROR);

            assertFalse(Triangle2D.isInside(point1, point2, point3, outside, 0.0));
            assertTrue(Triangle2D.isInside(point1, point2, point3, outside,
                    3.0 * MAX_RANDOM_VALUE * MAX_RANDOM_VALUE));

            assertEquals(dist, Triangle2D.shortestDistance(point1, point2, point3, outside), ABSOLUTE_ERROR);

            // Testing for a random triangle we can see that vertices and center
            // lie inside the triangle
            final var point4 = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var point5 = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var point6 = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

            triangle.setVertices(point4, point5, point6);
            center = triangle.getCenter();

            // vertices and center lie inside the triangle
            assertTrue(triangle.isInside(point4));
            assertTrue(triangle.isInside(point5));
            assertTrue(triangle.isInside(point6));
            assertTrue(triangle.isInside(center));

            //vertices are locus, but not center
            assertTrue(triangle.isLocus(point4, ABSOLUTE_ERROR));
            assertTrue(triangle.isLocus(point5, ABSOLUTE_ERROR));
            assertTrue(triangle.isLocus(point6, ABSOLUTE_ERROR));
            assertFalse(triangle.isLocus(center, ABSOLUTE_ERROR));

            assertEquals(0.0, triangle.getShortestDistance(point4), ABSOLUTE_ERROR);
            assertEquals(0.0, triangle.getShortestDistance(point5), ABSOLUTE_ERROR);
            assertEquals(0.0, triangle.getShortestDistance(point6), ABSOLUTE_ERROR);
            line1 = new Line2D(point4, point5);
            line2 = new Line2D(point4, point6);
            line3 = new Line2D(point5, point6);
            dist1 = Math.abs(line1.signedDistance(center));
            dist2 = Math.abs(line2.signedDistance(center));
            dist3 = Math.abs(line3.signedDistance(center));
            centerDist = Math.min(dist1, Math.min(dist2, dist3));
            assertEquals(centerDist, triangle.getShortestDistance(center), ABSOLUTE_ERROR);


            assertTrue(Triangle2D.isInside(triangle, point4));
            assertTrue(Triangle2D.isInside(triangle, point5));
            assertTrue(Triangle2D.isInside(triangle, point6));
            assertTrue(Triangle2D.isInside(triangle, center));

            assertEquals(0.0, Triangle2D.shortestDistance(triangle, point4), ABSOLUTE_ERROR);
            assertEquals(0.0, Triangle2D.shortestDistance(triangle, point5), ABSOLUTE_ERROR);
            assertEquals(0.0, Triangle2D.shortestDistance(triangle, point6), ABSOLUTE_ERROR);
            assertEquals(Triangle2D.shortestDistance(triangle, center), centerDist, ABSOLUTE_ERROR);

            assertTrue(Triangle2D.isInside(point4, point5, point6, point4));
            assertTrue(Triangle2D.isInside(point4, point5, point6, point5));
            assertTrue(Triangle2D.isInside(point4, point5, point6, point6));
            assertTrue(Triangle2D.isInside(point4, point5, point6, center));

            assertEquals(0.0, Triangle2D.shortestDistance(point4, point5, point6, point4), ABSOLUTE_ERROR);
            assertEquals(0.0, Triangle2D.shortestDistance(point4, point5, point6, point5), ABSOLUTE_ERROR);
            assertEquals(0.0, Triangle2D.shortestDistance(point4, point5, point6, point6), ABSOLUTE_ERROR);
            assertEquals(centerDist, Triangle2D.shortestDistance(point4, point5, point6, center), ABSOLUTE_ERROR);


            // the same is true for a small threshold
            assertTrue(triangle.isInside(point4, ABSOLUTE_ERROR));
            assertTrue(triangle.isInside(point5, ABSOLUTE_ERROR));
            assertTrue(triangle.isInside(point6, ABSOLUTE_ERROR));
            assertTrue(triangle.isInside(center, ABSOLUTE_ERROR));

            assertTrue(Triangle2D.isInside(triangle, point4, ABSOLUTE_ERROR));
            assertTrue(Triangle2D.isInside(triangle, point5, ABSOLUTE_ERROR));
            assertTrue(Triangle2D.isInside(triangle, point6, ABSOLUTE_ERROR));
            assertTrue(Triangle2D.isInside(triangle, center, ABSOLUTE_ERROR));

            assertTrue(Triangle2D.isInside(point4, point5, point6, point4, ABSOLUTE_ERROR));
            assertTrue(Triangle2D.isInside(point4, point5, point6, point5, ABSOLUTE_ERROR));
            assertTrue(Triangle2D.isInside(point4, point5, point6, point6, ABSOLUTE_ERROR));
            assertTrue(Triangle2D.isInside(point4, point5, point6, center, ABSOLUTE_ERROR));
        }
    }

    @Test
    void testCenter() {
        final var randomizer = new UniformRandomizer();

        final var point1 = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point2 = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point3 = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var triangle = new Triangle2D(point1, point2, point3);

        final var expectedCenter = new InhomogeneousPoint2D(
                (point1.getInhomX() + point2.getInhomX() + point3.getInhomX()) / 3.0,
                (point1.getInhomY() + point2.getInhomY() + point3.getInhomY()) / 3.0);

        assertTrue(triangle.getCenter().equals(expectedCenter, ABSOLUTE_ERROR));

        final var center = Point2D.create();
        triangle.center(center);
        assertTrue(center.equals(expectedCenter, ABSOLUTE_ERROR));

        assertTrue(expectedCenter.equals(Triangle2D.center(point1, point2, point3), ABSOLUTE_ERROR));
        assertTrue(expectedCenter.equals(Triangle2D.center(triangle), ABSOLUTE_ERROR));

        Triangle2D.center(point1, point2, point3, center);
        assertTrue(center.equals(expectedCenter, ABSOLUTE_ERROR));

        Triangle2D.center(triangle, center);
        assertTrue(center.equals(expectedCenter, ABSOLUTE_ERROR));
    }

    @Test
    void testPerimeter() {
        final var randomizer = new UniformRandomizer();

        final var point1 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point2 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point3 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var triangle = new Triangle2D(point1, point2, point3);

        final var perimeter = point1.distanceTo(point2) + point1.distanceTo(point3) + point3.distanceTo(point2);

        assertEquals(perimeter, triangle.getPerimeter(), ABSOLUTE_ERROR);
        assertEquals(perimeter, Triangle2D.perimeter(triangle), ABSOLUTE_ERROR);
        assertEquals(perimeter, Triangle2D.perimeter(point1, point2, point3), ABSOLUTE_ERROR);
    }

    @Test
    void testClosestPoint() {
        // Test for known values
        final var randomizer = new UniformRandomizer();
        final var base = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE));
        final var height = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE));

        // Test known and simple values
        final var point1 = new InhomogeneousPoint2D(0.0, 0.0);
        final var point2 = new InhomogeneousPoint2D(base, 0.0);
        final var point3 = new InhomogeneousPoint2D(base / 2.0, height);

        final var triangle = new Triangle2D(point1, point2, point3);

        // try for point1
        final var testPoint = Point2D.create();
        testPoint.setInhomogeneousCoordinates(-base, 0.0);
        assertTrue(triangle.getClosestPoint(testPoint).equals(point1, ABSOLUTE_ERROR));
        assertTrue(triangle.getClosestPoint(testPoint, ABSOLUTE_ERROR).equals(point1, ABSOLUTE_ERROR));

        final var closestPoint = Point2D.create();
        triangle.closestPoint(testPoint, closestPoint);
        assertTrue(closestPoint.equals(point1, ABSOLUTE_ERROR));
        triangle.closestPoint(testPoint, closestPoint, ABSOLUTE_ERROR);
        assertTrue(closestPoint.equals(point1, ABSOLUTE_ERROR));

        // try for point2
        testPoint.setInhomogeneousCoordinates(2.0 * base, 0.0);
        assertTrue(triangle.getClosestPoint(testPoint).equals(point2, ABSOLUTE_ERROR));
        assertTrue(triangle.getClosestPoint(testPoint, ABSOLUTE_ERROR).equals(point2, ABSOLUTE_ERROR));

        triangle.closestPoint(testPoint, closestPoint);
        assertTrue(closestPoint.equals(point2, ABSOLUTE_ERROR));
        triangle.closestPoint(testPoint, closestPoint, ABSOLUTE_ERROR);
        assertTrue(closestPoint.equals(point2, ABSOLUTE_ERROR));

        // try for point3
        testPoint.setInhomogeneousCoordinates(base / 2.0, 2.0 * height);
        assertTrue(triangle.getClosestPoint(testPoint).equals(point3, ABSOLUTE_ERROR));
        assertTrue(triangle.getClosestPoint(testPoint, ABSOLUTE_ERROR).equals(point3, ABSOLUTE_ERROR));

        triangle.closestPoint(testPoint, closestPoint);
        assertTrue(closestPoint.equals(point3, ABSOLUTE_ERROR));
        triangle.closestPoint(testPoint, closestPoint, ABSOLUTE_ERROR);
        assertTrue(closestPoint.equals(point3, ABSOLUTE_ERROR));

        // try for a point close to line1
        testPoint.setInhomogeneousCoordinates(base / 2.0, -height);
        final var basePoint = new InhomogeneousPoint2D(base / 2.0, 0.0);
        assertTrue(triangle.getClosestPoint(testPoint).equals(basePoint, ABSOLUTE_ERROR));
        assertTrue(triangle.getClosestPoint(testPoint, ABSOLUTE_ERROR).equals(basePoint, ABSOLUTE_ERROR));

        triangle.closestPoint(testPoint, closestPoint);
        assertTrue(closestPoint.equals(basePoint, ABSOLUTE_ERROR));
        triangle.closestPoint(testPoint, closestPoint, ABSOLUTE_ERROR);
        assertTrue(closestPoint.equals(basePoint, ABSOLUTE_ERROR));

        // try with test point on vertices

        // vertex1
        assertTrue(triangle.getClosestPoint(point1).equals(point1, ABSOLUTE_ERROR));
        assertTrue(triangle.getClosestPoint(point1, ABSOLUTE_ERROR).equals(point1, ABSOLUTE_ERROR));
        triangle.closestPoint(point1, closestPoint);
        assertTrue(closestPoint.equals(point1, ABSOLUTE_ERROR));
        triangle.closestPoint(point1, closestPoint, ABSOLUTE_ERROR);
        assertTrue(closestPoint.equals(point1, ABSOLUTE_ERROR));

        // vertex2
        assertTrue(triangle.getClosestPoint(point2).equals(point2, ABSOLUTE_ERROR));
        assertTrue(triangle.getClosestPoint(point2, ABSOLUTE_ERROR).equals(point2, ABSOLUTE_ERROR));
        triangle.closestPoint(point2, closestPoint);
        assertTrue(closestPoint.equals(point2, ABSOLUTE_ERROR));
        triangle.closestPoint(point2, closestPoint, ABSOLUTE_ERROR);
        assertTrue(closestPoint.equals(point2, ABSOLUTE_ERROR));

        // vertex3
        assertTrue(triangle.getClosestPoint(point3).equals(point3, ABSOLUTE_ERROR));
        assertTrue(triangle.getClosestPoint(point3, ABSOLUTE_ERROR).equals(point3, ABSOLUTE_ERROR));
        triangle.closestPoint(point3, closestPoint);
        assertTrue(closestPoint.equals(point3, ABSOLUTE_ERROR));
        triangle.closestPoint(point3, closestPoint, ABSOLUTE_ERROR);
        assertTrue(closestPoint.equals(point3, ABSOLUTE_ERROR));

        // and now try for center
        final var center = triangle.getCenter();
        center.normalize();

        final var line1 = new Line2D(point1, point2);
        final var line2 = new Line2D(point1, point3);
        final var line3 = new Line2D(point2, point3);

        line1.normalize();
        line2.normalize();
        line3.normalize();

        final var pointLine1 = line1.getClosestPoint(center);
        final var pointLine2 = line2.getClosestPoint(center);
        final var pointLine3 = line3.getClosestPoint(center);

        pointLine1.normalize();
        pointLine2.normalize();
        pointLine3.normalize();

        // pick the closest point among the three above to center
        final var dist1 = pointLine1.distanceTo(center);
        final var dist2 = pointLine2.distanceTo(center);
        final var dist3 = pointLine3.distanceTo(center);

        final var linePoint = Point2D.create();

        if (dist1 < dist2 && dist1 < dist3) {
            // pick pointLine1
            linePoint.setCoordinates(pointLine1);
        } else if (dist2 < dist1 && dist2 < dist3) {
            // pick pointLine2
            linePoint.setCoordinates(pointLine2);
        } else {
            // pick pointLine3
            linePoint.setCoordinates(pointLine3);
        }

        // check correctness
        assertTrue(triangle.getClosestPoint(center).equals(linePoint, ABSOLUTE_ERROR));
        assertTrue(triangle.getClosestPoint(center, ABSOLUTE_ERROR).equals(linePoint, ABSOLUTE_ERROR));
        triangle.closestPoint(center, closestPoint);
        assertTrue(closestPoint.equals(linePoint, ABSOLUTE_ERROR));
        triangle.closestPoint(center, closestPoint, ABSOLUTE_ERROR);
        assertTrue(closestPoint.equals(linePoint, ABSOLUTE_ERROR));
    }

    @Test
    void testAreVerticesClockwise() {

        final var randomizer = new UniformRandomizer();
        final var base = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var height = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        // Test known and simple values
        var point1 = new InhomogeneousPoint2D(0.0, 0.0);
        var point2 = new InhomogeneousPoint2D(base, 0.0);
        var point3 = new InhomogeneousPoint2D(base / 2.0, height);

        var triangle1 = new Triangle2D(point1, point2, point3);
        var triangle2 = new Triangle2D(point3, point2, point1);
        // we know that triangle1 is counterclockwise and triangle2 is clockwise
        assertFalse(triangle1.areVerticesClockwise());
        assertTrue(triangle2.areVerticesClockwise());
        assertFalse(triangle1.areVerticesClockwise(0.0));
        assertTrue(triangle2.areVerticesClockwise(0.0));

        // now try with random vertices
        point1 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point2 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point3 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        triangle1 = new Triangle2D(point1, point2, point3);
        final var signedArea = triangle1.getSignedArea();

        // create triangle with vertices in reversed order
        triangle2 = new Triangle2D(point3, point2, point1);

        if (signedArea > 0.0) {
            // points are clockwise
            assertFalse(triangle1.areVerticesClockwise());
            assertTrue(triangle2.areVerticesClockwise());
        } else {
            assertTrue(triangle1.areVerticesClockwise());
            assertFalse(triangle2.areVerticesClockwise());
        }
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();

        final var point1 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point2 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point3 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var triangle1 = new Triangle2D(point1, point2, point3);

        // check
        assertEquals(point1, triangle1.getVertex1());
        assertEquals(point2, triangle1.getVertex2());
        assertEquals(point3, triangle1.getVertex3());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(triangle1);
        final var triangle2 = SerializationHelper.<Triangle2D>deserialize(bytes);

        // check
        assertEquals(triangle1.getVertex1(), triangle2.getVertex1());
        assertEquals(triangle1.getVertex2(), triangle2.getVertex2());
        assertEquals(triangle1.getVertex3(), triangle2.getVertex3());
    }
}
