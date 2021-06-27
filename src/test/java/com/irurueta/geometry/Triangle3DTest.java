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
import org.junit.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class Triangle3DTest {

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final double LARGE_ABSOLUTE_ERROR = 1e-5;

    private static final int INHOM_COORDS = 3;

    @Test
    public void testConstants() {
        assertEquals(1e-9, Triangle3D.DEFAULT_THRESHOLD, 0.0);
        assertEquals(0.0, Triangle3D.MIN_THRESHOLD, 0.0);
        assertEquals(3, Triangle3D.INHOM_COORDS);
        assertEquals(3, Triangle3D.NUM_VERTICES);
    }

    @Test
    public void testConstructor() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final Point3D point1 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D point2 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D point3 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        Triangle3D triangle = new Triangle3D(point1, point2, point3);

        assertEquals(triangle.getVertex1(), point1);
        assertEquals(triangle.getVertex2(), point2);
        assertEquals(triangle.getVertex3(), point3);

        // Force NullPointerException
        triangle = null;
        try {
            triangle = new Triangle3D(null, point2, point3);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
        try {
            triangle = new Triangle3D(point1, null, point3);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
        try {
            triangle = new Triangle3D(point1, point2, null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
        //noinspection ConstantConditions
        assertNull(triangle);
    }

    @Test
    public void testGetSetVertex1() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final Point3D point1 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D point2 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D point3 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final Triangle3D triangle = new Triangle3D(point1, point2, point3);

        assertEquals(triangle.getVertex1(), point1);
        assertEquals(triangle.getVertex2(), point2);
        assertEquals(triangle.getVertex3(), point3);

        // new vertex1
        final Point3D vertex1 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        triangle.setVertex1(vertex1);
        // check correctness
        assertEquals(triangle.getVertex1(), vertex1);
        assertEquals(triangle.getVertex2(), point2);
        assertEquals(triangle.getVertex3(), point3);

        // Force NullPointerException
        try {
            triangle.setVertex1(null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
    }

    @Test
    public void testGetSetVertex2() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final Point3D point1 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D point2 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D point3 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final Triangle3D triangle = new Triangle3D(point1, point2, point3);

        assertEquals(triangle.getVertex1(), point1);
        assertEquals(triangle.getVertex2(), point2);
        assertEquals(triangle.getVertex3(), point3);

        // new vertex1
        final Point3D vertex2 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        triangle.setVertex2(vertex2);
        // check correctness
        assertEquals(triangle.getVertex1(), point1);
        assertEquals(triangle.getVertex2(), vertex2);
        assertEquals(triangle.getVertex3(), point3);

        // Force NullPointerException
        try {
            triangle.setVertex2(null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
    }

    @Test
    public void testGetSetVertex3() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final Point3D point1 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D point2 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D point3 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final Triangle3D triangle = new Triangle3D(point1, point2, point3);

        assertEquals(triangle.getVertex1(), point1);
        assertEquals(triangle.getVertex2(), point2);
        assertEquals(triangle.getVertex3(), point3);

        // new vertex1
        final Point3D vertex3 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        triangle.setVertex3(vertex3);
        // check correctness
        assertEquals(triangle.getVertex1(), point1);
        assertEquals(triangle.getVertex2(), point2);
        assertEquals(triangle.getVertex3(), vertex3);

        // Force NullPointerException
        try {
            triangle.setVertex3(null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
    }

    @Test
    public void testGetSetVertices() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final Point3D point1 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D point2 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D point3 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final Triangle3D triangle = new Triangle3D(point1, point2, point3);

        assertEquals(triangle.getVertex1(), point1);
        assertEquals(triangle.getVertex2(), point2);
        assertEquals(triangle.getVertex3(), point3);

        final Point3D point1b = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D point2b = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D point3b = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        // Set new vertices
        triangle.setVertices(point1b, point2b, point3b);

        // check correctness
        assertEquals(triangle.getVertex1(), point1b);
        assertEquals(triangle.getVertex2(), point2b);
        assertEquals(triangle.getVertex3(), point3b);

        // get vertices as a list
        final List<Point3D> vertices = triangle.getVertices();
        final List<Point3D> vertices2 = new ArrayList<>();
        triangle.vertices(vertices2);

        assertEquals(vertices.size(), Triangle2D.NUM_VERTICES);
        assertEquals(vertices2.size(), Triangle2D.NUM_VERTICES);

        assertTrue(vertices.get(0).equals(point1b, ABSOLUTE_ERROR));
        assertTrue(vertices.get(1).equals(point2b, ABSOLUTE_ERROR));
        assertTrue(vertices.get(2).equals(point3b, ABSOLUTE_ERROR));

        assertTrue(vertices2.get(0).equals(point1b, ABSOLUTE_ERROR));
        assertTrue(vertices2.get(1).equals(point2b, ABSOLUTE_ERROR));
        assertTrue(vertices2.get(2).equals(point3b, ABSOLUTE_ERROR));

        // Force NullPointerException
        try {
            triangle.setVertices(null, point2, point3);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
        try {
            triangle.setVertices(point1, null, point3);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
        try {
            triangle.setVertices(point1, point2, null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
    }

    @Test
    public void testAreaAndColinearPoints() throws WrongSizeException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double base = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE));
        final double height = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE));
        final double zValue = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // Test known and simple values
        Point3D point1 = new InhomogeneousPoint3D(0.0, 0.0, zValue);
        Point3D point2 = new InhomogeneousPoint3D(base, 0.0, zValue);
        Point3D point3 = new InhomogeneousPoint3D(base / 2.0, height, zValue);

        double expectedArea = base * height / 2.0;

        Triangle3D triangle = new Triangle3D(point1, point2, point3);

        assertEquals(Math.abs(Triangle3D.area(triangle)), expectedArea,
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Triangle3D.area(point1, point2, point3)),
                expectedArea, ABSOLUTE_ERROR);
        assertEquals(Math.abs(triangle.getArea()), expectedArea,
                ABSOLUTE_ERROR);

        if (expectedArea > Triangle2D.DEFAULT_THRESHOLD) {
            assertFalse(triangle.areVerticesColinear());
        } else {
            assertTrue(triangle.areVerticesColinear());
        }

        // Test with random values
        point1 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point2 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point3 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        Matrix m = new Matrix(2, 2);
        // 1st column
        m.setElementAt(0, 0, point2.getInhomX() - point1.getInhomX());
        m.setElementAt(1, 0, point2.getInhomY() - point1.getInhomY());
        // 2nd columns
        m.setElementAt(0, 1, point3.getInhomX() - point1.getInhomX());
        m.setElementAt(1, 1, point3.getInhomY() - point1.getInhomY());

        // expected area
        final double[] v1 = ArrayUtils.subtractAndReturnNew(point2.asArray(),
                point1.asArray());
        final double[] v2 = ArrayUtils.subtractAndReturnNew(point3.asArray(),
                point1.asArray());

        final double[] cross = Utils.crossProduct(v1, v2);
        expectedArea = 0.5 * Utils.normF(cross);

        triangle = new Triangle3D(point1, point2, point3);

        assertEquals(Triangle3D.area(triangle), expectedArea,
                ABSOLUTE_ERROR);
        assertEquals(Triangle3D.area(point1, point2, point3),
                expectedArea, ABSOLUTE_ERROR);
        assertEquals(triangle.getArea(), expectedArea,
                ABSOLUTE_ERROR);

        if (expectedArea > Triangle2D.DEFAULT_THRESHOLD) {
            assertFalse(triangle.areVerticesColinear());
        } else {
            assertTrue(triangle.areVerticesColinear());
        }

        // if threshold is large enough, points will always be considered to be
        // co-linear
        assertTrue(triangle.areVerticesColinear(expectedArea + ABSOLUTE_ERROR));

        // If two points are coincident, then area must be zero or close to zero
        triangle = new Triangle3D(point1, point1, point2);
        assertEquals(Triangle3D.area(triangle), 0.0, ABSOLUTE_ERROR);
        assertEquals(Triangle3D.area(point1, point1, point2), 0.0,
                ABSOLUTE_ERROR);
        assertEquals(triangle.getArea(), 0.0, ABSOLUTE_ERROR);

        // because area is zero, then points are co-linear
        assertTrue(triangle.areVerticesColinear());
        assertTrue(triangle.areVerticesColinear(ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        try {
            triangle.areVerticesColinear(-ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsInsideShortestDistanceAndIsLocus()
            throws CoincidentPointsException {
        // Test for known values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double base = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE));
        final double height = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE));
        final double zValue = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double dist = randomizer.nextDouble(Triangle2D.DEFAULT_THRESHOLD,
                MAX_RANDOM_VALUE);

        // Test known and simple values
        Point3D point1 = new InhomogeneousPoint3D(0.0, 0.0, zValue);
        Point3D point2 = new InhomogeneousPoint3D(base, 0.0, zValue);
        Point3D point3 = new InhomogeneousPoint3D(base / 2.0, height,
                zValue);

        final Triangle3D triangle = new Triangle3D(point1, point2, point3);
        Point3D center = triangle.getCenter();

        // vertices and center lie inside the triangle
        assertTrue(triangle.isInside(point1));
        assertTrue(triangle.isInside(point2));
        assertTrue(triangle.isInside(point3));
        assertTrue(triangle.isInside(center));

        // test shortest distance
        assertEquals(triangle.getShortestDistance(point1), 0.0, 0.0);
        assertEquals(triangle.getShortestDistance(point2), 0.0, 0.0);
        assertEquals(triangle.getShortestDistance(point3), 0.0, 0.0);
        Line3D line1 = new Line3D(point1, point2);
        Line3D line2 = new Line3D(point1, point3);
        Line3D line3 = new Line3D(point2, point3);
        double dist1 = line1.getDistance(center);
        double dist2 = line2.getDistance(center);
        double dist3 = line3.getDistance(center);
        double centerDist = Math.min(dist1, Math.min(dist2, dist3));
        assertEquals(triangle.getShortestDistance(center), centerDist,
                ABSOLUTE_ERROR);

        // test is locus (vertices will be locus, but not center)
        assertTrue(triangle.isLocus(point1));
        assertTrue(triangle.isLocus(point2));
        assertTrue(triangle.isLocus(point3));
        assertFalse(triangle.isLocus(center, LARGE_ABSOLUTE_ERROR));

        assertTrue(Triangle3D.isInside(triangle, point1));
        assertTrue(Triangle3D.isInside(triangle, point2));
        assertTrue(Triangle3D.isInside(triangle, point3));
        assertTrue(Triangle3D.isInside(triangle, center));

        assertEquals(Triangle3D.shortestDistance(triangle, point1), 0.0,
                0.0);
        assertEquals(Triangle3D.shortestDistance(triangle, point2), 0.0,
                0.0);
        assertEquals(Triangle3D.shortestDistance(triangle, point3), 0.0,
                0.0);
        assertEquals(Triangle3D.shortestDistance(triangle, center),
                centerDist, ABSOLUTE_ERROR);

        assertTrue(Triangle3D.isInside(point1, point2, point3, point1));
        assertTrue(Triangle3D.isInside(point1, point2, point3, point2));
        assertTrue(Triangle3D.isInside(point1, point2, point3, point3));
        assertTrue(Triangle3D.isInside(point1, point2, point3, center));

        assertEquals(Triangle3D.shortestDistance(point1, point2, point3,
                point1), 0.0, 0.0);
        assertEquals(Triangle3D.shortestDistance(point1, point2, point3,
                point2), 0.0, 0.0);
        assertEquals(Triangle3D.shortestDistance(point1, point2, point3,
                point3), 0.0, 0.0);
        assertEquals(Triangle3D.shortestDistance(point1, point2, point3,
                center), centerDist, ABSOLUTE_ERROR);

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

        assertTrue(Triangle3D.isInside(point1, point2, point3, point1,
                0.0));
        assertTrue(Triangle3D.isInside(point1, point2, point3, point2,
                0.0));
        assertTrue(Triangle3D.isInside(point1, point2, point3, point3,
                0.0));

        // Force IllegalArgumentException
        try {
            triangle.isInside(point1, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Triangle3D.isInside(triangle, point1, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Triangle3D.isInside(point1, point2, point3, point1,
                    -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        try {
            triangle.isLocus(point3, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Check point outside
        final Point3D outside = new InhomogeneousPoint3D(-dist, 0.0, zValue);

        assertFalse(triangle.isInside(outside));

        // point outside is not locus
        assertFalse(triangle.isLocus(outside));

        assertFalse(Triangle3D.isInside(triangle, outside));

        assertFalse(Triangle3D.isInside(point1, point2, point3, outside));

        // the same is true for a small threshold, but point is considered to
        // be inside when setting large threshold
        assertFalse(triangle.isInside(outside, 0.0));
        assertTrue(triangle.isInside(outside,
                3.0 * MAX_RANDOM_VALUE * MAX_RANDOM_VALUE));

        assertEquals(triangle.getShortestDistance(outside), dist,
                ABSOLUTE_ERROR);

        assertFalse(Triangle3D.isInside(triangle, outside, 0.0));
        assertTrue(Triangle3D.isInside(triangle, outside,
                3.0 * MAX_RANDOM_VALUE * MAX_RANDOM_VALUE));

        // check locus with a small and large threshold
        assertFalse(triangle.isLocus(outside, 0.0));
        assertTrue(triangle.isLocus(outside,
                3.0 * MAX_RANDOM_VALUE * MAX_RANDOM_VALUE));

        assertEquals(Triangle3D.shortestDistance(triangle, outside), dist,
                ABSOLUTE_ERROR);

        assertFalse(Triangle3D.isInside(point1, point2, point3, outside,
                0.0));
        assertTrue(Triangle3D.isInside(point1, point2, point3, outside,
                3.0 * MAX_RANDOM_VALUE * MAX_RANDOM_VALUE));

        assertEquals(Triangle3D.shortestDistance(point1, point2, point3,
                outside), dist, ABSOLUTE_ERROR);

        // Testing for a random triangle we can see that vertices and center
        // lie inside the triangle
        point1 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point2 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point3 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        triangle.setVertices(point1, point2, point3);
        center = triangle.getCenter();

        // vertices and center lie inside the triangle
        assertTrue(triangle.isInside(point1));
        assertTrue(triangle.isInside(point2));
        assertTrue(triangle.isInside(point3));
        assertTrue(triangle.isInside(center));

        // vertices are locus, but not center
        assertTrue(triangle.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(triangle.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(triangle.isLocus(point3, ABSOLUTE_ERROR));
        assertFalse(triangle.isLocus(center, ABSOLUTE_ERROR));

        assertEquals(triangle.getShortestDistance(point1), 0.0,
                ABSOLUTE_ERROR);
        assertEquals(triangle.getShortestDistance(point2), 0.0,
                ABSOLUTE_ERROR);
        assertEquals(triangle.getShortestDistance(point3), 0.0,
                ABSOLUTE_ERROR);
        line1 = new Line3D(point1, point2);
        line2 = new Line3D(point1, point3);
        line3 = new Line3D(point2, point3);
        dist1 = line1.getDistance(center);
        dist2 = line2.getDistance(center);
        dist3 = line3.getDistance(center);
        centerDist = Math.min(dist1, Math.min(dist2, dist3));
        assertEquals(triangle.getShortestDistance(center), centerDist,
                ABSOLUTE_ERROR);

        assertTrue(Triangle3D.isInside(triangle, point1));
        assertTrue(Triangle3D.isInside(triangle, point2));
        assertTrue(Triangle3D.isInside(triangle, point3));
        assertTrue(Triangle3D.isInside(triangle, center));

        assertEquals(Triangle3D.shortestDistance(triangle, point1), 0.0,
                ABSOLUTE_ERROR);
        assertEquals(Triangle3D.shortestDistance(triangle, point2), 0.0,
                ABSOLUTE_ERROR);
        assertEquals(Triangle3D.shortestDistance(triangle, point3), 0.0,
                ABSOLUTE_ERROR);
        assertEquals(Triangle3D.shortestDistance(triangle, center),
                centerDist, ABSOLUTE_ERROR);

        assertTrue(Triangle3D.isInside(point1, point2, point3, point1));
        assertTrue(Triangle3D.isInside(point1, point2, point3, point2));
        assertTrue(Triangle3D.isInside(point1, point2, point3, point3));
        assertTrue(Triangle3D.isInside(point1, point2, point3, center));

        assertEquals(Triangle3D.shortestDistance(point1, point2, point3,
                point1), 0.0, ABSOLUTE_ERROR);
        assertEquals(Triangle3D.shortestDistance(point1, point2, point3,
                point2), 0.0, ABSOLUTE_ERROR);
        assertEquals(Triangle3D.shortestDistance(point1, point2, point3,
                point3), 0.0, ABSOLUTE_ERROR);
        assertEquals(Triangle3D.shortestDistance(point1, point2, point3,
                center), centerDist, ABSOLUTE_ERROR);

        // the same is true for a small threshold
        assertTrue(triangle.isInside(point1, ABSOLUTE_ERROR));
        assertTrue(triangle.isInside(point2, ABSOLUTE_ERROR));
        assertTrue(triangle.isInside(point3, ABSOLUTE_ERROR));
        assertTrue(triangle.isInside(center, ABSOLUTE_ERROR));

        assertTrue(Triangle3D.isInside(triangle, point1, ABSOLUTE_ERROR));
        assertTrue(Triangle3D.isInside(triangle, point2, ABSOLUTE_ERROR));
        assertTrue(Triangle3D.isInside(triangle, point3, ABSOLUTE_ERROR));
        assertTrue(Triangle3D.isInside(triangle, center, ABSOLUTE_ERROR));

        assertTrue(Triangle3D.isInside(point1, point2, point3, point1,
                ABSOLUTE_ERROR));
        assertTrue(Triangle3D.isInside(point1, point2, point3, point2,
                ABSOLUTE_ERROR));
        assertTrue(Triangle3D.isInside(point1, point2, point3, point3,
                ABSOLUTE_ERROR));
        assertTrue(Triangle3D.isInside(point1, point2, point3, center,
                ABSOLUTE_ERROR));
    }

    @Test
    public void testCenter() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final Point3D point1 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D point2 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D point3 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final Triangle3D triangle = new Triangle3D(point1, point2, point3);

        final Point3D expectedCenter = new InhomogeneousPoint3D((point1.getInhomX() +
                point2.getInhomX() + point3.getInhomX()) / 3.0,
                (point1.getInhomY() + point2.getInhomY() + point3.getInhomY()) /
                        3.0, (point1.getInhomZ() + point2.getInhomZ() +
                point3.getInhomZ()) / 3.0);

        assertTrue(triangle.getCenter().equals(expectedCenter, ABSOLUTE_ERROR));

        final Point3D center = Point3D.create();
        triangle.center(center);
        assertTrue(center.equals(expectedCenter, ABSOLUTE_ERROR));

        assertTrue(expectedCenter.equals(Triangle3D.center(point1, point2,
                point3), ABSOLUTE_ERROR));
        assertTrue(expectedCenter.equals(Triangle3D.center(triangle),
                ABSOLUTE_ERROR));

        Triangle3D.center(point1, point2, point3, center);
        assertTrue(center.equals(expectedCenter, ABSOLUTE_ERROR));

        Triangle3D.center(triangle, center);
        assertTrue(center.equals(expectedCenter, ABSOLUTE_ERROR));
    }

    @Test
    public void testPerimeter() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final Point3D point1 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D point2 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D point3 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final Triangle3D triangle = new Triangle3D(point1, point2, point3);

        final double perimeter = point1.distanceTo(point2) +
                point1.distanceTo(point3) + point3.distanceTo(point2);

        assertEquals(triangle.getPerimeter(), perimeter, ABSOLUTE_ERROR);
        assertEquals(Triangle3D.perimeter(triangle), perimeter, ABSOLUTE_ERROR);
        assertEquals(Triangle3D.perimeter(point1, point2, point3), perimeter,
                ABSOLUTE_ERROR);
    }

    @Test
    public void testClosestPoint() throws CoincidentPointsException {
        // Test for known values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double base = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0,
                MAX_RANDOM_VALUE));
        final double height = Math.abs(randomizer.nextDouble(
                MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE));
        final double zValue = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // Test known and simple values
        final Point3D point1 = new InhomogeneousPoint3D(0.0, 0.0, zValue);
        final Point3D point2 = new InhomogeneousPoint3D(base, 0.0, zValue);
        final Point3D point3 = new InhomogeneousPoint3D(base / 2.0, height,
                zValue);

        final Triangle3D triangle = new Triangle3D(point1, point2, point3);

        // try for point1
        final Point3D testPoint = Point3D.create();
        testPoint.setInhomogeneousCoordinates(-base, 0.0, zValue);
        assertTrue(triangle.getClosestPoint(testPoint).equals(point1,
                ABSOLUTE_ERROR));
        assertTrue(triangle.getClosestPoint(testPoint,
                ABSOLUTE_ERROR).equals(point1, ABSOLUTE_ERROR));

        final Point3D closestPoint = Point3D.create();
        triangle.closestPoint(testPoint, closestPoint);
        assertTrue(closestPoint.equals(point1, ABSOLUTE_ERROR));
        triangle.closestPoint(testPoint, closestPoint, ABSOLUTE_ERROR);
        assertTrue(closestPoint.equals(point1, ABSOLUTE_ERROR));

        // try for point2
        testPoint.setInhomogeneousCoordinates(2.0 * base, 0.0, zValue);
        assertTrue(triangle.getClosestPoint(testPoint).equals(point2,
                ABSOLUTE_ERROR));
        assertTrue(triangle.getClosestPoint(testPoint,
                ABSOLUTE_ERROR).equals(point2, ABSOLUTE_ERROR));

        triangle.closestPoint(testPoint, closestPoint);
        assertTrue(closestPoint.equals(point2, ABSOLUTE_ERROR));
        triangle.closestPoint(testPoint, closestPoint, ABSOLUTE_ERROR);
        assertTrue(closestPoint.equals(point2, ABSOLUTE_ERROR));

        // try for point3
        testPoint.setInhomogeneousCoordinates(base / 2.0, 2.0 * height,
                zValue);
        assertTrue(triangle.getClosestPoint(testPoint).equals(point3,
                ABSOLUTE_ERROR));
        assertTrue(triangle.getClosestPoint(testPoint,
                ABSOLUTE_ERROR).equals(point3, ABSOLUTE_ERROR));

        triangle.closestPoint(testPoint, closestPoint);
        assertTrue(closestPoint.equals(point3, ABSOLUTE_ERROR));
        triangle.closestPoint(testPoint, closestPoint, ABSOLUTE_ERROR);
        assertTrue(closestPoint.equals(point3, ABSOLUTE_ERROR));

        // try for a point close to line1
        testPoint.setInhomogeneousCoordinates(base / 2.0, -height, zValue);
        final Point3D basePoint = new InhomogeneousPoint3D(base / 2.0, 0.0,
                zValue);
        assertTrue(triangle.getClosestPoint(testPoint).equals(basePoint,
                ABSOLUTE_ERROR));
        assertTrue(triangle.getClosestPoint(testPoint,
                ABSOLUTE_ERROR).equals(basePoint, ABSOLUTE_ERROR));

        triangle.closestPoint(testPoint, closestPoint);
        assertTrue(closestPoint.equals(basePoint, ABSOLUTE_ERROR));
        triangle.closestPoint(testPoint, closestPoint, ABSOLUTE_ERROR);
        assertTrue(closestPoint.equals(basePoint, ABSOLUTE_ERROR));


        // try with test point on vertices

        // vertex1
        assertTrue(triangle.getClosestPoint(point1).equals(point1,
                ABSOLUTE_ERROR));
        assertTrue(triangle.getClosestPoint(point1, ABSOLUTE_ERROR).equals(
                point1, ABSOLUTE_ERROR));
        triangle.closestPoint(point1, closestPoint);
        assertTrue(closestPoint.equals(point1, ABSOLUTE_ERROR));
        triangle.closestPoint(point1, closestPoint, ABSOLUTE_ERROR);
        assertTrue(closestPoint.equals(point1, ABSOLUTE_ERROR));

        // vertex2
        assertTrue(triangle.getClosestPoint(point2).equals(point2,
                ABSOLUTE_ERROR));
        assertTrue(triangle.getClosestPoint(point2, ABSOLUTE_ERROR).equals(
                point2, ABSOLUTE_ERROR));
        triangle.closestPoint(point2, closestPoint);
        assertTrue(closestPoint.equals(point2, ABSOLUTE_ERROR));
        triangle.closestPoint(point2, closestPoint, ABSOLUTE_ERROR);
        assertTrue(closestPoint.equals(point2, ABSOLUTE_ERROR));

        // vertex3
        assertTrue(triangle.getClosestPoint(point3).equals(point3,
                ABSOLUTE_ERROR));
        assertTrue(triangle.getClosestPoint(point3, ABSOLUTE_ERROR).equals(
                point3, ABSOLUTE_ERROR));
        triangle.closestPoint(point3, closestPoint);
        assertTrue(closestPoint.equals(point3, ABSOLUTE_ERROR));
        triangle.closestPoint(point3, closestPoint, ABSOLUTE_ERROR);
        assertTrue(closestPoint.equals(point3, ABSOLUTE_ERROR));


        // and now try for center
        final Point3D center = triangle.getCenter();
        center.normalize();

        final Line3D line1 = new Line3D(point1, point2);
        final Line3D line2 = new Line3D(point1, point3);
        final Line3D line3 = new Line3D(point2, point3);

        line1.normalize();
        line2.normalize();
        line3.normalize();

        final Point3D pointLine1 = line1.getClosestPoint(center);
        final Point3D pointLine2 = line2.getClosestPoint(center);
        final Point3D pointLine3 = line3.getClosestPoint(center);

        pointLine1.normalize();
        pointLine2.normalize();
        pointLine3.normalize();

        // pick closest point among the three above to center
        final double dist1 = pointLine1.distanceTo(center);
        final double dist2 = pointLine2.distanceTo(center);
        final double dist3 = pointLine3.distanceTo(center);

        final Point3D linePoint = Point3D.create();

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
        assertTrue(triangle.getClosestPoint(center).equals(linePoint,
                ABSOLUTE_ERROR));
        assertTrue(triangle.getClosestPoint(center, ABSOLUTE_ERROR).equals(
                linePoint, ABSOLUTE_ERROR));
        triangle.closestPoint(center, closestPoint);
        assertTrue(closestPoint.equals(linePoint, ABSOLUTE_ERROR));
        triangle.closestPoint(center, closestPoint, ABSOLUTE_ERROR);
        assertTrue(closestPoint.equals(linePoint, ABSOLUTE_ERROR));
    }

    @Test
    public void testOrientationAndToPlane() throws ColinearPointsException,
            CoincidentPointsException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final Point3D point1 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D point2 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D point3 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        Triangle3D triangle = new Triangle3D(point1, point2, point3);

        // check that points are locus of triangle
        assertTrue(triangle.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(triangle.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(triangle.isLocus(point3, ABSOLUTE_ERROR));

        final Plane plane1 = triangle.toPlane();
        final Plane plane2 = new Plane();
        triangle.toPlane(plane2);

        // check that vertices of triangle are locus of plane1 and plane2
        assertTrue(plane1.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(plane1.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(plane1.isLocus(point3, ABSOLUTE_ERROR));

        assertTrue(plane2.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(plane2.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(plane2.isLocus(point3, ABSOLUTE_ERROR));

        final double[] expectedOrientation = plane1.getDirectorVector();
        // normalize expected orientation
        final double norm = Utils.normF(expectedOrientation);
        ArrayUtils.multiplyByScalar(expectedOrientation, 1.0 / norm,
                expectedOrientation);

        final double[] orientation1 = triangle.getOrientation();
        final double[] orientation2 = triangle.getOrientation(ABSOLUTE_ERROR);
        final double[] orientation3 = Triangle3D.orientation(triangle);
        final double[] orientation4 = Triangle3D.orientation(triangle,
                ABSOLUTE_ERROR);
        final double[] orientation5 = Triangle3D.orientation(point1, point2, point3);
        final double[] orientation6 = Triangle3D.orientation(point1, point2, point3,
                ABSOLUTE_ERROR);
        final double[] orientation7 = new double[INHOM_COORDS];
        triangle.orientation(orientation7);
        final double[] orientation8 = new double[INHOM_COORDS];
        triangle.orientation(orientation8, ABSOLUTE_ERROR);
        final double[] orientation9 = new double[INHOM_COORDS];
        Triangle3D.orientation(triangle, orientation9);
        final double[] orientation10 = new double[INHOM_COORDS];
        Triangle3D.orientation(triangle, orientation10, ABSOLUTE_ERROR);
        final double[] orientation11 = new double[INHOM_COORDS];
        Triangle3D.orientation(point1, point2, point3, orientation11);
        final double[] orientation12 = new double[INHOM_COORDS];
        Triangle3D.orientation(point1, point2, point3, orientation12,
                ABSOLUTE_ERROR);

        if (Math.signum(expectedOrientation[0]) != Math.signum(orientation1[0])) {
            // change sign of expectedOrientation, because it is equal up to
            // sign
            ArrayUtils.multiplyByScalar(expectedOrientation, -1.0,
                    expectedOrientation);
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
        try {
            triangle.getOrientation(-ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Triangle3D.orientation(triangle, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Triangle3D.orientation(point1, point2, point3, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangle.orientation(orientation8, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Triangle3D.orientation(triangle, orientation10, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Triangle3D.orientation(point1, point2, point3, orientation12,
                    -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        final double[] wrong = new double[INHOM_COORDS + 1];
        try {
            triangle.orientation(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangle.orientation(wrong, ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Triangle3D.orientation(triangle, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Triangle3D.orientation(triangle, wrong, ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Triangle3D.orientation(point1, point2, point3, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            Triangle3D.orientation(point1, point2, point3, wrong,
                    ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Force CoincidentPointsException
        triangle = new Triangle3D(point1, point1, point2);

        try {
            triangle.getOrientation();
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            triangle.getOrientation(ABSOLUTE_ERROR);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            Triangle3D.orientation(triangle);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            Triangle3D.orientation(triangle, ABSOLUTE_ERROR);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            Triangle3D.orientation(point1, point1, point2);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            Triangle3D.orientation(point1, point1, point2, ABSOLUTE_ERROR);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            triangle.orientation(orientation7);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            triangle.orientation(orientation8, ABSOLUTE_ERROR);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            Triangle3D.orientation(triangle, orientation9);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            Triangle3D.orientation(triangle, orientation10, ABSOLUTE_ERROR);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            Triangle3D.orientation(point1, point1, point2, orientation11);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        try {
            Triangle3D.orientation(point1, point1, point2, orientation12,
                    ABSOLUTE_ERROR);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
    }

    @Test
    public void testGetAngleBetweenTriangles() throws CoincidentPointsException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final Point3D point1a = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D point2a = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D point3a = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final Point3D point1b = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D point2b = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D point3b = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final Triangle3D triangleA = new Triangle3D(point1a, point2a, point3a);
        final Triangle3D triangleB = new Triangle3D(point1b, point2b, point3b);

        final double[] orientationA = triangleA.getOrientation();
        final double[] orientationB = triangleB.getOrientation();

        final double normA = Utils.normF(orientationA);
        final double normB = Utils.normF(orientationB);

        // compute dot product between orientation vectors
        final double dotProduct = orientationA[0] * orientationB[0] +
                orientationA[1] * orientationB[1] +
                orientationA[2] * orientationB[2];
        final double cosAngle = dotProduct / (normA * normB);

        final double expectedAngle = Math.acos(cosAngle);

        assertEquals(Triangle3D.getAngleBetweenTriangles(triangleA, triangleB),
                expectedAngle, ABSOLUTE_ERROR);
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final Point3D point1 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D point2 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D point3 = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final Triangle3D triangle1 = new Triangle3D(point1, point2, point3);

        // check
        assertEquals(triangle1.getVertex1(), point1);
        assertEquals(triangle1.getVertex2(), point2);
        assertEquals(triangle1.getVertex3(), point3);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(triangle1);
        final Triangle3D triangle2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(triangle1.getVertex1(), triangle2.getVertex1());
        assertEquals(triangle1.getVertex2(), triangle2.getVertex2());
        assertEquals(triangle1.getVertex3(), triangle2.getVertex3());
    }
}
