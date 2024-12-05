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
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class Triangle3DTest {

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final double LARGE_ABSOLUTE_ERROR = 1e-5;

    private static final int INHOM_COORDS = 3;

    @Test
    void testConstants() {
        assertEquals(1e-9, Triangle3D.DEFAULT_THRESHOLD, 0.0);
        assertEquals(0.0, Triangle3D.MIN_THRESHOLD, 0.0);
        assertEquals(3, Triangle3D.INHOM_COORDS);
        assertEquals(3, Triangle3D.NUM_VERTICES);
    }

    @Test
    void testConstructor() {
        final var randomizer = new UniformRandomizer();

        final var point1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point2 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point3 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        var triangle = new Triangle3D(point1, point2, point3);

        assertEquals(point1, triangle.getVertex1());
        assertEquals(point2, triangle.getVertex2());
        assertEquals(point3, triangle.getVertex3());

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new Triangle3D(null, point2, point3));
        assertThrows(NullPointerException.class, () -> new Triangle3D(point1, null, point3));
        assertThrows(NullPointerException.class, () -> new Triangle3D(point1, point2, null));
    }

    @Test
    void testGetSetVertex1() {
        final var randomizer = new UniformRandomizer();

        final var point1 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point2 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point3 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var triangle = new Triangle3D(point1, point2, point3);

        assertEquals(point1, triangle.getVertex1());
        assertEquals(point2, triangle.getVertex2());
        assertEquals(point3, triangle.getVertex3());

        // new vertex1
        final var vertex1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
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

        final var point1 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point2 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point3 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var triangle = new Triangle3D(point1, point2, point3);

        assertEquals(point1, triangle.getVertex1());
        assertEquals(point2, triangle.getVertex2());
        assertEquals(point3, triangle.getVertex3());

        // new vertex1
        final var vertex2 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
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

        final var point1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point2 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point3 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var triangle = new Triangle3D(point1, point2, point3);

        assertEquals(point1, triangle.getVertex1());
        assertEquals(point2, triangle.getVertex2());
        assertEquals(point3, triangle.getVertex3());

        // new vertex1
        final var vertex3 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
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

        final var point1 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point2 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point3 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var triangle = new Triangle3D(point1, point2, point3);

        assertEquals(point1, triangle.getVertex1());
        assertEquals(point2, triangle.getVertex2());
        assertEquals(point3, triangle.getVertex3());

        final var point1b = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point2b = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point3b = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
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
        final var vertices2 = new ArrayList<Point3D>();
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
    void testAreaAndColinearPoints() throws WrongSizeException {

        final var randomizer = new UniformRandomizer();
        final var base = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var height = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var zValue = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // Test known and simple values
        var point1 = new InhomogeneousPoint3D(0.0, 0.0, zValue);
        var point2 = new InhomogeneousPoint3D(base, 0.0, zValue);
        var point3 = new InhomogeneousPoint3D(base / 2.0, height, zValue);

        var expectedArea = base * height / 2.0;

        final var triangle1 = new Triangle3D(point1, point2, point3);

        assertEquals(expectedArea, Math.abs(Triangle3D.area(triangle1)), ABSOLUTE_ERROR);
        assertEquals(expectedArea, Math.abs(Triangle3D.area(point1, point2, point3)), ABSOLUTE_ERROR);
        assertEquals(expectedArea, Math.abs(triangle1.getArea()), ABSOLUTE_ERROR);

        if (expectedArea > Triangle2D.DEFAULT_THRESHOLD) {
            assertFalse(triangle1.areVerticesColinear());
        } else {
            assertTrue(triangle1.areVerticesColinear());
        }

        // Test with random values
        point1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point2 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point3 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        var m = new Matrix(2, 2);
        // 1st column
        m.setElementAt(0, 0, point2.getInhomX() - point1.getInhomX());
        m.setElementAt(1, 0, point2.getInhomY() - point1.getInhomY());
        // 2nd columns
        m.setElementAt(0, 1, point3.getInhomX() - point1.getInhomX());
        m.setElementAt(1, 1, point3.getInhomY() - point1.getInhomY());

        // expected area
        final var v1 = ArrayUtils.subtractAndReturnNew(point2.asArray(), point1.asArray());
        final var v2 = ArrayUtils.subtractAndReturnNew(point3.asArray(), point1.asArray());

        final var cross = Utils.crossProduct(v1, v2);
        expectedArea = 0.5 * Utils.normF(cross);

        final var triangle2 = new Triangle3D(point1, point2, point3);

        assertEquals(expectedArea, Triangle3D.area(triangle2), ABSOLUTE_ERROR);
        assertEquals(expectedArea, Triangle3D.area(point1, point2, point3), ABSOLUTE_ERROR);
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
        final var triangle3 = new Triangle3D(point1, point1, point2);
        assertEquals(0.0, Triangle3D.area(triangle3), ABSOLUTE_ERROR);
        assertEquals(0.0, Triangle3D.area(point1, point1, point2), ABSOLUTE_ERROR);
        assertEquals(0.0, triangle3.getArea(), ABSOLUTE_ERROR);

        // because area is zero, then points are co-linear
        assertTrue(triangle3.areVerticesColinear());
        assertTrue(triangle3.areVerticesColinear(ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> triangle3.areVerticesColinear(-ABSOLUTE_ERROR));
    }

    @Test
    void testIsInsideShortestDistanceAndIsLocus() throws CoincidentPointsException {
        // Test for known values
        final var randomizer = new UniformRandomizer();
        final var base = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var height = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var zValue = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var dist = randomizer.nextDouble(Triangle2D.DEFAULT_THRESHOLD, MAX_RANDOM_VALUE);

        // Test known and simple values
        final var point1 = new InhomogeneousPoint3D(0.0, 0.0, zValue);
        final var point2 = new InhomogeneousPoint3D(base, 0.0, zValue);
        final var point3 = new InhomogeneousPoint3D(base / 2.0, height, zValue);

        final var triangle = new Triangle3D(point1, point2, point3);
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
        var line1 = new Line3D(point1, point2);
        var line2 = new Line3D(point1, point3);
        var line3 = new Line3D(point2, point3);
        var dist1 = line1.getDistance(center);
        var dist2 = line2.getDistance(center);
        var dist3 = line3.getDistance(center);
        var centerDist = Math.min(dist1, Math.min(dist2, dist3));
        assertEquals(triangle.getShortestDistance(center), centerDist, ABSOLUTE_ERROR);

        // test is locus (vertices will be locus, but not center)
        assertTrue(triangle.isLocus(point1));
        assertTrue(triangle.isLocus(point2));
        assertTrue(triangle.isLocus(point3));
        assertFalse(triangle.isLocus(center, LARGE_ABSOLUTE_ERROR));

        assertTrue(Triangle3D.isInside(triangle, point1));
        assertTrue(Triangle3D.isInside(triangle, point2));
        assertTrue(Triangle3D.isInside(triangle, point3));
        assertTrue(Triangle3D.isInside(triangle, center));

        assertEquals(0.0, Triangle3D.shortestDistance(triangle, point1), 0.0);
        assertEquals(0.0, Triangle3D.shortestDistance(triangle, point2), 0.0);
        assertEquals(0.0, Triangle3D.shortestDistance(triangle, point3), 0.0);
        assertEquals(Triangle3D.shortestDistance(triangle, center), centerDist, ABSOLUTE_ERROR);

        assertTrue(Triangle3D.isInside(point1, point2, point3, point1));
        assertTrue(Triangle3D.isInside(point1, point2, point3, point2));
        assertTrue(Triangle3D.isInside(point1, point2, point3, point3));
        assertTrue(Triangle3D.isInside(point1, point2, point3, center));

        assertEquals(0.0, Triangle3D.shortestDistance(point1, point2, point3, point1), 0.0);
        assertEquals(0.0, Triangle3D.shortestDistance(point1, point2, point3, point2), 0.0);
        assertEquals(0.0, Triangle3D.shortestDistance(point1, point2, point3, point3), 0.0);
        assertEquals(centerDist, Triangle3D.shortestDistance(point1, point2, point3, center), ABSOLUTE_ERROR);

        // the same is true for a small threshold
        assertTrue(triangle.isInside(point1, 0.0));
        assertTrue(triangle.isInside(point2, 0.0));
        assertTrue(triangle.isInside(point3, 0.0));

        // check is locus with a small threshold
        assertTrue(triangle.isLocus(point1, 0.0));
        assertTrue(triangle.isLocus(point1, 0.0));
        assertTrue(triangle.isLocus(point1, 0.0));
        assertFalse(triangle.isLocus(center, 0.0));

        assertTrue(Triangle3D.isInside(triangle, point1, 0.0));
        assertTrue(Triangle3D.isInside(triangle, point2, 0.0));
        assertTrue(Triangle3D.isInside(triangle, point3, 0.0));

        assertTrue(Triangle3D.isInside(point1, point2, point3, point1, 0.0));
        assertTrue(Triangle3D.isInside(point1, point2, point3, point2, 0.0));
        assertTrue(Triangle3D.isInside(point1, point2, point3, point3, 0.0));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> triangle.isInside(point1, -ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class, () -> Triangle3D.isInside(triangle, point1, -ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class,
                () -> Triangle3D.isInside(point1, point2, point3, point1, -ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class, () -> triangle.isLocus(point3, -ABSOLUTE_ERROR));

        // Check point outside
        final var outside = new InhomogeneousPoint3D(-dist, 0.0, zValue);

        assertFalse(triangle.isInside(outside));

        // point outside is not locus
        assertFalse(triangle.isLocus(outside));

        assertFalse(Triangle3D.isInside(triangle, outside));

        assertFalse(Triangle3D.isInside(point1, point2, point3, outside));

        // the same is true for a small threshold, but point is considered to
        // be inside when setting large threshold
        assertFalse(triangle.isInside(outside, 0.0));
        assertTrue(triangle.isInside(outside, 3.0 * MAX_RANDOM_VALUE * MAX_RANDOM_VALUE));

        assertEquals(triangle.getShortestDistance(outside), dist, ABSOLUTE_ERROR);

        assertFalse(Triangle3D.isInside(triangle, outside, 0.0));
        assertTrue(Triangle3D.isInside(triangle, outside, 3.0 * MAX_RANDOM_VALUE * MAX_RANDOM_VALUE));

        // check locus with a small and large threshold
        assertFalse(triangle.isLocus(outside, 0.0));
        assertTrue(triangle.isLocus(outside, 3.0 * MAX_RANDOM_VALUE * MAX_RANDOM_VALUE));

        assertEquals(Triangle3D.shortestDistance(triangle, outside), dist, ABSOLUTE_ERROR);

        assertFalse(Triangle3D.isInside(point1, point2, point3, outside, 0.0));
        assertTrue(Triangle3D.isInside(point1, point2, point3, outside,
                3.0 * MAX_RANDOM_VALUE * MAX_RANDOM_VALUE));

        assertEquals(Triangle3D.shortestDistance(point1, point2, point3, outside), dist, ABSOLUTE_ERROR);

        // Testing for a random triangle we can see that vertices and center
        // lie inside the triangle
        final var point4 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point5 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point6 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        triangle.setVertices(point4, point5, point6);
        center = triangle.getCenter();

        // vertices and center lie inside the triangle
        assertTrue(triangle.isInside(point4));
        assertTrue(triangle.isInside(point5));
        assertTrue(triangle.isInside(point6));
        assertTrue(triangle.isInside(center));

        // vertices are locus, but not center
        assertTrue(triangle.isLocus(point4, ABSOLUTE_ERROR));
        assertTrue(triangle.isLocus(point5, ABSOLUTE_ERROR));
        assertTrue(triangle.isLocus(point6, ABSOLUTE_ERROR));
        assertFalse(triangle.isLocus(center, ABSOLUTE_ERROR));

        assertEquals(0.0, triangle.getShortestDistance(point4), ABSOLUTE_ERROR);
        assertEquals(0.0, triangle.getShortestDistance(point5), ABSOLUTE_ERROR);
        assertEquals(0.0, triangle.getShortestDistance(point6), ABSOLUTE_ERROR);
        line1 = new Line3D(point4, point5);
        line2 = new Line3D(point4, point6);
        line3 = new Line3D(point5, point6);
        dist1 = line1.getDistance(center);
        dist2 = line2.getDistance(center);
        dist3 = line3.getDistance(center);
        centerDist = Math.min(dist1, Math.min(dist2, dist3));
        assertEquals(centerDist, triangle.getShortestDistance(center), ABSOLUTE_ERROR);

        assertTrue(Triangle3D.isInside(triangle, point4));
        assertTrue(Triangle3D.isInside(triangle, point5));
        assertTrue(Triangle3D.isInside(triangle, point6));
        assertTrue(Triangle3D.isInside(triangle, center));

        assertEquals(0.0, Triangle3D.shortestDistance(triangle, point4), ABSOLUTE_ERROR);
        assertEquals(0.0, Triangle3D.shortestDistance(triangle, point5), ABSOLUTE_ERROR);
        assertEquals(0.0, Triangle3D.shortestDistance(triangle, point6), ABSOLUTE_ERROR);
        assertEquals(centerDist, Triangle3D.shortestDistance(triangle, center), ABSOLUTE_ERROR);

        assertTrue(Triangle3D.isInside(point4, point5, point6, point4));
        assertTrue(Triangle3D.isInside(point4, point5, point6, point5));
        assertTrue(Triangle3D.isInside(point4, point5, point6, point6));
        assertTrue(Triangle3D.isInside(point4, point5, point6, center));

        assertEquals(0.0, Triangle3D.shortestDistance(point4, point5, point6, point4), ABSOLUTE_ERROR);
        assertEquals(0.0, Triangle3D.shortestDistance(point4, point5, point6, point5), ABSOLUTE_ERROR);
        assertEquals(0.0, Triangle3D.shortestDistance(point4, point5, point6, point6), ABSOLUTE_ERROR);
        assertEquals(centerDist, Triangle3D.shortestDistance(point4, point5, point6, center), ABSOLUTE_ERROR);

        // the same is true for a small threshold
        assertTrue(triangle.isInside(point4, ABSOLUTE_ERROR));
        assertTrue(triangle.isInside(point5, ABSOLUTE_ERROR));
        assertTrue(triangle.isInside(point6, ABSOLUTE_ERROR));
        assertTrue(triangle.isInside(center, ABSOLUTE_ERROR));

        assertTrue(Triangle3D.isInside(triangle, point4, ABSOLUTE_ERROR));
        assertTrue(Triangle3D.isInside(triangle, point5, ABSOLUTE_ERROR));
        assertTrue(Triangle3D.isInside(triangle, point6, ABSOLUTE_ERROR));
        assertTrue(Triangle3D.isInside(triangle, center, ABSOLUTE_ERROR));

        assertTrue(Triangle3D.isInside(point4, point5, point6, point4, ABSOLUTE_ERROR));
        assertTrue(Triangle3D.isInside(point4, point5, point6, point5, ABSOLUTE_ERROR));
        assertTrue(Triangle3D.isInside(point4, point5, point6, point6, ABSOLUTE_ERROR));
        assertTrue(Triangle3D.isInside(point4, point5, point6, center, ABSOLUTE_ERROR));
    }

    @Test
    void testCenter() {
        final var randomizer = new UniformRandomizer();

        final var point1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point2 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point3 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var triangle = new Triangle3D(point1, point2, point3);

        final var expectedCenter = new InhomogeneousPoint3D(
                (point1.getInhomX() + point2.getInhomX() + point3.getInhomX()) / 3.0,
                (point1.getInhomY() + point2.getInhomY() + point3.getInhomY()) / 3.0,
                (point1.getInhomZ() + point2.getInhomZ() + point3.getInhomZ()) / 3.0);

        assertTrue(triangle.getCenter().equals(expectedCenter, ABSOLUTE_ERROR));

        final var center = Point3D.create();
        triangle.center(center);
        assertTrue(center.equals(expectedCenter, ABSOLUTE_ERROR));

        assertTrue(expectedCenter.equals(Triangle3D.center(point1, point2, point3), ABSOLUTE_ERROR));
        assertTrue(expectedCenter.equals(Triangle3D.center(triangle), ABSOLUTE_ERROR));

        Triangle3D.center(point1, point2, point3, center);
        assertTrue(center.equals(expectedCenter, ABSOLUTE_ERROR));

        Triangle3D.center(triangle, center);
        assertTrue(center.equals(expectedCenter, ABSOLUTE_ERROR));
    }

    @Test
    void testPerimeter() {
        final var randomizer = new UniformRandomizer();

        final var point1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point2 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point3 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var triangle = new Triangle3D(point1, point2, point3);

        final var perimeter = point1.distanceTo(point2) + point1.distanceTo(point3) + point3.distanceTo(point2);

        assertEquals(perimeter, triangle.getPerimeter(), ABSOLUTE_ERROR);
        assertEquals(perimeter, Triangle3D.perimeter(triangle), ABSOLUTE_ERROR);
        assertEquals(perimeter, Triangle3D.perimeter(point1, point2, point3), ABSOLUTE_ERROR);
    }

    @Test
    void testClosestPoint() throws CoincidentPointsException {
        // Test for known values
        final var randomizer = new UniformRandomizer();
        final var base = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE));
        final var height = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE));
        final var zValue = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // Test known and simple values
        final var point1 = new InhomogeneousPoint3D(0.0, 0.0, zValue);
        final var point2 = new InhomogeneousPoint3D(base, 0.0, zValue);
        final var point3 = new InhomogeneousPoint3D(base / 2.0, height, zValue);

        final var triangle = new Triangle3D(point1, point2, point3);

        // try for point1
        final var testPoint = Point3D.create();
        testPoint.setInhomogeneousCoordinates(-base, 0.0, zValue);
        assertTrue(triangle.getClosestPoint(testPoint).equals(point1, ABSOLUTE_ERROR));
        assertTrue(triangle.getClosestPoint(testPoint, ABSOLUTE_ERROR).equals(point1, ABSOLUTE_ERROR));

        final var closestPoint = Point3D.create();
        triangle.closestPoint(testPoint, closestPoint);
        assertTrue(closestPoint.equals(point1, ABSOLUTE_ERROR));
        triangle.closestPoint(testPoint, closestPoint, ABSOLUTE_ERROR);
        assertTrue(closestPoint.equals(point1, ABSOLUTE_ERROR));

        // try for point2
        testPoint.setInhomogeneousCoordinates(2.0 * base, 0.0, zValue);
        assertTrue(triangle.getClosestPoint(testPoint).equals(point2, ABSOLUTE_ERROR));
        assertTrue(triangle.getClosestPoint(testPoint, ABSOLUTE_ERROR).equals(point2, ABSOLUTE_ERROR));

        triangle.closestPoint(testPoint, closestPoint);
        assertTrue(closestPoint.equals(point2, ABSOLUTE_ERROR));
        triangle.closestPoint(testPoint, closestPoint, ABSOLUTE_ERROR);
        assertTrue(closestPoint.equals(point2, ABSOLUTE_ERROR));

        // try for point3
        testPoint.setInhomogeneousCoordinates(base / 2.0, 2.0 * height, zValue);
        assertTrue(triangle.getClosestPoint(testPoint).equals(point3, ABSOLUTE_ERROR));
        assertTrue(triangle.getClosestPoint(testPoint, ABSOLUTE_ERROR).equals(point3, ABSOLUTE_ERROR));

        triangle.closestPoint(testPoint, closestPoint);
        assertTrue(closestPoint.equals(point3, ABSOLUTE_ERROR));
        triangle.closestPoint(testPoint, closestPoint, ABSOLUTE_ERROR);
        assertTrue(closestPoint.equals(point3, ABSOLUTE_ERROR));

        // try for a point close to line1
        testPoint.setInhomogeneousCoordinates(base / 2.0, -height, zValue);
        final var basePoint = new InhomogeneousPoint3D(base / 2.0, 0.0, zValue);
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

        final var line1 = new Line3D(point1, point2);
        final var line2 = new Line3D(point1, point3);
        final var line3 = new Line3D(point2, point3);

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

        final var linePoint = Point3D.create();

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
    void testOrientationAndToPlane() throws ColinearPointsException, CoincidentPointsException {

        final var randomizer = new UniformRandomizer();

        final var point1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point2 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point3 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var triangle1 = new Triangle3D(point1, point2, point3);

        // check that points are locus of triangle
        assertTrue(triangle1.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(triangle1.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(triangle1.isLocus(point3, ABSOLUTE_ERROR));

        final var plane1 = triangle1.toPlane();
        final var plane2 = new Plane();
        triangle1.toPlane(plane2);

        // check that vertices of triangle are locus of plane1 and plane2
        assertTrue(plane1.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(plane1.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(plane1.isLocus(point3, ABSOLUTE_ERROR));

        assertTrue(plane2.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(plane2.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(plane2.isLocus(point3, ABSOLUTE_ERROR));

        final var expectedOrientation = plane1.getDirectorVector();
        // normalize expected orientation
        final var norm = Utils.normF(expectedOrientation);
        ArrayUtils.multiplyByScalar(expectedOrientation, 1.0 / norm, expectedOrientation);

        final var orientation1 = triangle1.getOrientation();
        final var orientation2 = triangle1.getOrientation(ABSOLUTE_ERROR);
        final var orientation3 = Triangle3D.orientation(triangle1);
        final var orientation4 = Triangle3D.orientation(triangle1, ABSOLUTE_ERROR);
        final var orientation5 = Triangle3D.orientation(point1, point2, point3);
        final var orientation6 = Triangle3D.orientation(point1, point2, point3, ABSOLUTE_ERROR);
        final var orientation7 = new double[INHOM_COORDS];
        triangle1.orientation(orientation7);
        final var orientation8 = new double[INHOM_COORDS];
        triangle1.orientation(orientation8, ABSOLUTE_ERROR);
        final var orientation9 = new double[INHOM_COORDS];
        Triangle3D.orientation(triangle1, orientation9);
        final var orientation10 = new double[INHOM_COORDS];
        Triangle3D.orientation(triangle1, orientation10, ABSOLUTE_ERROR);
        final var orientation11 = new double[INHOM_COORDS];
        Triangle3D.orientation(point1, point2, point3, orientation11);
        final var orientation12 = new double[INHOM_COORDS];
        Triangle3D.orientation(point1, point2, point3, orientation12, ABSOLUTE_ERROR);

        if (Math.signum(expectedOrientation[0]) != Math.signum(orientation1[0])) {
            // change sign of expectedOrientation, because it is equal up to
            // sign
            ArrayUtils.multiplyByScalar(expectedOrientation, -1.0, expectedOrientation);
        }

        assertArrayEquals(expectedOrientation, orientation1, ABSOLUTE_ERROR);
        assertArrayEquals(expectedOrientation, orientation2, ABSOLUTE_ERROR);
        assertArrayEquals(expectedOrientation, orientation3, ABSOLUTE_ERROR);
        assertArrayEquals(expectedOrientation, orientation4, ABSOLUTE_ERROR);
        assertArrayEquals(expectedOrientation, orientation5, ABSOLUTE_ERROR);
        assertArrayEquals(expectedOrientation, orientation6, ABSOLUTE_ERROR);
        assertArrayEquals(expectedOrientation, orientation7, ABSOLUTE_ERROR);
        assertArrayEquals(expectedOrientation, orientation8, ABSOLUTE_ERROR);
        assertArrayEquals(expectedOrientation, orientation9, ABSOLUTE_ERROR);
        assertArrayEquals(expectedOrientation, orientation10, ABSOLUTE_ERROR);
        assertArrayEquals(expectedOrientation, orientation11, ABSOLUTE_ERROR);
        assertArrayEquals(expectedOrientation, orientation12, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> triangle1.getOrientation(-ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class, () -> Triangle3D.orientation(triangle1, -ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class, () -> Triangle3D.orientation(point1, point2, point3,
                -ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class, () -> triangle1.orientation(orientation8, -ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class, () -> Triangle3D.orientation(triangle1, orientation10,
                -ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class, () -> Triangle3D.orientation(point1, point2, point3, orientation12,
                -ABSOLUTE_ERROR));

        final var wrong = new double[INHOM_COORDS + 1];
        assertThrows(IllegalArgumentException.class, () -> triangle1.orientation(wrong));
        assertThrows(IllegalArgumentException.class, () -> triangle1.orientation(wrong, ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class, () -> Triangle3D.orientation(triangle1, wrong));
        assertThrows(IllegalArgumentException.class, () -> Triangle3D.orientation(triangle1, wrong, ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class, () -> Triangle3D.orientation(point1, point2, point3, wrong));
        assertThrows(IllegalArgumentException.class, () -> Triangle3D.orientation(point1, point2, point3, wrong,
                ABSOLUTE_ERROR));

        // Force CoincidentPointsException
        final var triangle2 = new Triangle3D(point1, point1, point2);
        assertThrows(CoincidentPointsException.class, triangle2::getOrientation);
        assertThrows(CoincidentPointsException.class, () -> triangle2.getOrientation(ABSOLUTE_ERROR));
        assertThrows(CoincidentPointsException.class, () -> Triangle3D.orientation(triangle2));
        assertThrows(CoincidentPointsException.class, () -> Triangle3D.orientation(triangle2, ABSOLUTE_ERROR));
        assertThrows(CoincidentPointsException.class, () -> Triangle3D.orientation(point1, point1, point2));
        assertThrows(CoincidentPointsException.class, () -> Triangle3D.orientation(point1, point1, point2,
                ABSOLUTE_ERROR));
        assertThrows(CoincidentPointsException.class, () -> triangle2.orientation(orientation7));
        assertThrows(CoincidentPointsException.class, () -> triangle2.orientation(orientation8, ABSOLUTE_ERROR));
        assertThrows(CoincidentPointsException.class, () -> Triangle3D.orientation(triangle2, orientation9));
        assertThrows(CoincidentPointsException.class, () -> Triangle3D.orientation(triangle2, orientation10,
                ABSOLUTE_ERROR));
        assertThrows(CoincidentPointsException.class, () -> Triangle3D.orientation(point1, point1, point2,
                orientation11));
        assertThrows(CoincidentPointsException.class, () -> Triangle3D.orientation(point1, point1, point2,
                orientation12, ABSOLUTE_ERROR));
    }

    @Test
    void testGetAngleBetweenTriangles() throws CoincidentPointsException {
        final var randomizer = new UniformRandomizer();

        final var point1a = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point2a = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point3a = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var point1b = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point2b = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point3b = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var triangleA = new Triangle3D(point1a, point2a, point3a);
        final var triangleB = new Triangle3D(point1b, point2b, point3b);

        final var orientationA = triangleA.getOrientation();
        final var orientationB = triangleB.getOrientation();

        final var normA = Utils.normF(orientationA);
        final var normB = Utils.normF(orientationB);

        // compute dot product between orientation vectors
        final var dotProduct = orientationA[0] * orientationB[0] +
                orientationA[1] * orientationB[1] +
                orientationA[2] * orientationB[2];
        final var cosAngle = dotProduct / (normA * normB);

        final var expectedAngle = Math.acos(cosAngle);

        assertEquals(Triangle3D.getAngleBetweenTriangles(triangleA, triangleB), expectedAngle, ABSOLUTE_ERROR);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();

        final var point1 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point2 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var point3 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var triangle1 = new Triangle3D(point1, point2, point3);

        // check
        assertEquals(point1, triangle1.getVertex1());
        assertEquals(point2, triangle1.getVertex2());
        assertEquals(point3, triangle1.getVertex3());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(triangle1);
        final var triangle2 = SerializationHelper.<Triangle3D>deserialize(bytes);

        // check
        assertEquals(triangle1.getVertex1(), triangle2.getVertex1());
        assertEquals(triangle1.getVertex2(), triangle2.getVertex2());
        assertEquals(triangle1.getVertex3(), triangle2.getVertex3());
    }
}
