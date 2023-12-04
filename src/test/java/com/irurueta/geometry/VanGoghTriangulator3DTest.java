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

import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class VanGoghTriangulator3DTest {

    private static final double ABSOLUTE_ERROR = 1e-8;
    private static final double DEFAULT_ORIENTATION_THRESHOLD = Math.PI / 2.0;

    private static final double MIN_RANDOM_DEGREES = -180.0;
    private static final double MAX_RANDOM_DEGREES = 180.0;

    private static final int MIN_SIDES = 6;
    private static final int MAX_SIDES = 12;

    private static final double MIN_RADIUS = 1.0;
    private static final double MAX_RADIUS = 10.0;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    @Test
    public void testConstants() {
        assertEquals(3, Triangulator3D.MIN_VERTICES);
        assertEquals(TriangulatorMethod.VAN_GOGH_TRIANGULATOR, Triangulator3D.DEFAULT_TRIANGULATOR_METHOD);
        assertEquals(VanGoghTriangulator3D.DEFAULT_ORIENTATION_THRESHOLD, Math.PI / 2.0, 0.0);
        assertEquals(0.0, VanGoghTriangulator3D.MIN_THRESHOLD, 0.0);
        assertEquals(3, VanGoghTriangulator3D.INHOM_COORDS);
    }

    @Test
    public void testConstructorAndGetMethod() {
        final VanGoghTriangulator2D triangulator = new VanGoghTriangulator2D();
        assertNotNull(triangulator);

        // check method correctness
        assertEquals(TriangulatorMethod.VAN_GOGH_TRIANGULATOR, triangulator.getMethod());
    }

    @Test
    public void testIsPolygonOrientationReversed()
            throws NotEnoughVerticesException, CoincidentPointsException {
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

        // check that orientation is reversed
        assertTrue(VanGoghTriangulator3D.isPolygonOrientationReversed(vertices1, vertices2));
        assertTrue(VanGoghTriangulator3D.isPolygonOrientationReversed(vertices1, vertices2,
                DEFAULT_ORIENTATION_THRESHOLD));

        // trying with same polygon will return an angle of zero
        angle = Polygon3D.getAngleBetweenPolygons(vertices1, vertices1);
        assertEquals(0.0, angle, ABSOLUTE_ERROR);
        angle = Polygon3D.getAngleBetweenPolygons(vertices1, vertices1, ABSOLUTE_ERROR);
        assertEquals(0.0, angle, ABSOLUTE_ERROR);

        angle = Polygon3D.getAngleBetweenPolygons(polygon1, polygon1);
        assertEquals(0.0, angle, ABSOLUTE_ERROR);
        angle = Polygon3D.getAngleBetweenPolygons(polygon1, polygon1, ABSOLUTE_ERROR);
        assertEquals(0.0, angle, ABSOLUTE_ERROR);

        // now orientation is not reversed
        assertFalse(VanGoghTriangulator3D.isPolygonOrientationReversed(vertices1, vertices1));
        assertFalse(VanGoghTriangulator3D.isPolygonOrientationReversed(vertices1, vertices1,
                DEFAULT_ORIENTATION_THRESHOLD));
    }

    @Test
    public void testIsOrientationReversed() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double angle = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;
        if (Math.abs(angle) >
                VanGoghTriangulator3D.DEFAULT_ORIENTATION_THRESHOLD) {
            assertTrue(VanGoghTriangulator3D.isOrientationReversed(angle));
        } else {
            assertFalse(VanGoghTriangulator3D.isOrientationReversed(angle));
        }

        if (Math.abs(angle) > DEFAULT_ORIENTATION_THRESHOLD) {
            assertTrue(VanGoghTriangulator3D.isOrientationReversed(angle,
                    DEFAULT_ORIENTATION_THRESHOLD));
        } else {
            assertFalse(VanGoghTriangulator3D.isOrientationReversed(angle,
                    DEFAULT_ORIENTATION_THRESHOLD));
        }

        assertFalse(VanGoghTriangulator3D.isOrientationReversed(
                VanGoghTriangulator3D.DEFAULT_ORIENTATION_THRESHOLD));
        assertTrue(VanGoghTriangulator3D.isOrientationReversed(
                2.0 * VanGoghTriangulator3D.DEFAULT_ORIENTATION_THRESHOLD));

        assertFalse(VanGoghTriangulator3D.isOrientationReversed(
                DEFAULT_ORIENTATION_THRESHOLD, DEFAULT_ORIENTATION_THRESHOLD));
        assertTrue(VanGoghTriangulator3D.isOrientationReversed(
                2.0 * DEFAULT_ORIENTATION_THRESHOLD,
                DEFAULT_ORIENTATION_THRESHOLD));
    }

    @Test
    public void testIsEar() throws CoincidentPointsException {
        // X: -1, Y: 0, Z: 5 (W: 1)
        Point3D v1 = new InhomogeneousPoint3D(-1.0, 0.0, 5.0);
        // X: 1, Y: 0, Z: 5 (W: 1)
        Point3D v2 = new InhomogeneousPoint3D(1.0, 0.0, 5.0);
        // X: 2, Y: 1, Z: 5 (W: 1)
        Point3D v3 = new InhomogeneousPoint3D(2.0, 1.0, 5.0);
        // X: 0, Y: 1, Z: 5 (W: 1)
        Point3D v4 = new InhomogeneousPoint3D(0.0, 1.0, 5.0);

        // Triangle
        final List<Point3D> polygonVertices = new ArrayList<>();

        polygonVertices.add(v1);
        polygonVertices.add(v2);
        polygonVertices.add(v4);

        // Check that the triangle is an ear with itself, by using any three
        // consecutive vertices

        // 1st triangle
        final Triangle3D triangle = new Triangle3D(v1, v2, v4);

        assertTrue(VanGoghTriangulator3D.isEar(triangle, polygonVertices));

        // 2nd triangle
        triangle.setVertices(v2, v4, v1);
        assertTrue(VanGoghTriangulator3D.isEar(triangle, polygonVertices));

        // 3rd triangle
        triangle.setVertices(v4, v1, v2);
        assertTrue(VanGoghTriangulator3D.isEar(triangle, polygonVertices));

        // Parallelepiped
        polygonVertices.clear();
        polygonVertices.add(v1);
        polygonVertices.add(v2);
        polygonVertices.add(v3);
        polygonVertices.add(v4);

        // all consecutive vertices of parallelepiped are also ears

        // 1st triangle
        triangle.setVertices(v1, v2, v3);
        assertTrue(VanGoghTriangulator3D.isEar(triangle, polygonVertices));

        // 2nd triangle
        triangle.setVertices(v2, v3, v4);
        assertTrue(VanGoghTriangulator3D.isEar(triangle, polygonVertices));

        // 3rd triangle
        triangle.setVertices(v3, v4, v1);
        assertTrue(VanGoghTriangulator3D.isEar(triangle, polygonVertices));

        // 4th triangle
        triangle.setVertices(v4, v1, v2);
        assertTrue(VanGoghTriangulator3D.isEar(triangle, polygonVertices));

        // General polygon
        // X: -1, Y: 0, Z: 5 (W: 1)
        v1 = new InhomogeneousPoint3D(-1.0, 0.0, 5.0);
        // X: 0, Y: 0.5, Z: 5 (W: 1)
        v2 = new InhomogeneousPoint3D(0.0, 0.5, 5.0);
        // X: 1, Y: 0, Z: 5 (W: 1)
        v3 = new InhomogeneousPoint3D(1.0, 0.0, 5.0);
        // X: 2, Y: 1, Z: 5 (W: 1)
        v4 = new InhomogeneousPoint3D(2.0, 1.0, 5.0);
        // X: 0, Y: 1, Z: 5 (W: 1)
        final Point3D v5 = new InhomogeneousPoint3D(0.0, 1.0, 5.0);

        polygonVertices.clear();
        polygonVertices.add(v1);
        polygonVertices.add(v2);
        polygonVertices.add(v3);
        polygonVertices.add(v4);
        polygonVertices.add(v5);

        // 1st triangle
        triangle.setVertices(v1, v2, v3);

        // triangle is not an ear (orientation reversed)
        assertFalse(VanGoghTriangulator3D.isEar(triangle, polygonVertices));

        // 2nd triangle
        triangle.setVertices(v2, v3, v4);

        // triangle is an ear
        assertTrue(VanGoghTriangulator3D.isEar(triangle, polygonVertices));

        // 3rd triangle
        triangle.setVertices(v3, v4, v5);

        // triangle is an ear
        assertTrue(VanGoghTriangulator3D.isEar(triangle, polygonVertices));

        // 4th triangle
        triangle.setVertices(v4, v5, v1);

        // triangle is not an ear (point of polygon inside triangle)
        assertFalse(VanGoghTriangulator3D.isEar(triangle, polygonVertices));

        // 5th triangle
        triangle.setVertices(v5, v1, v2);

        // triangle is an ear
        assertTrue(VanGoghTriangulator3D.isEar(triangle, polygonVertices));
    }

    @Test
    public void testTriangulateKnownPolygon() throws TriangulatorException {
        // X: -1, Y: 0, Z: 5 (W: 1)
        Point3D v1 = new InhomogeneousPoint3D(-1.0, 0.0, 5.0);
        // X: 1, Y: 0, Z: 5 (W: 1)
        Point3D v2 = new InhomogeneousPoint3D(1.0, 0.0, 5.0);
        // X: 2, Y: 1, Z: 5 (W: 1)
        Point3D v3 = new InhomogeneousPoint3D(2.0, 1.0, 5.0);
        // X: 0, Y: 1, Z: 5 (W: 1)
        Point3D v4 = new InhomogeneousPoint3D(0.0, 1.0, 5.0);

        // Test for Triangle polygon
        final List<Point3D> polygonVertices = new ArrayList<>();

        polygonVertices.add(v1);
        polygonVertices.add(v2);
        polygonVertices.add(v4);

        final VanGoghTriangulator3D triangulator = new VanGoghTriangulator3D();
        List<Triangle3D> triangles = triangulator.triangulate(polygonVertices);

        // triangles contains a single triangle with vertices v1, v2 and v4
        assertEquals(1, triangles.size());
        final Triangle3D triangle = triangles.get(0);
        assertTrue(triangle.getVertex1().equals(v1, ABSOLUTE_ERROR));
        assertTrue(triangle.getVertex2().equals(v2, ABSOLUTE_ERROR));
        assertTrue(triangle.getVertex3().equals(v4, ABSOLUTE_ERROR));

        // Test for Parallelepiped polygon
        polygonVertices.clear();
        polygonVertices.add(v1);
        polygonVertices.add(v2);
        polygonVertices.add(v3);
        polygonVertices.add(v4);

        triangles = triangulator.triangulate(polygonVertices);

        // parallelepiped is divided into 2 triangles
        assertEquals(2, triangles.size());
        // 1st triangle is formed by v4, v1 and v2
        Triangle3D triangle1 = triangles.get(0);
        assertTrue(triangle1.getVertex1().equals(v4, ABSOLUTE_ERROR));
        assertTrue(triangle1.getVertex2().equals(v1, ABSOLUTE_ERROR));
        assertTrue(triangle1.getVertex3().equals(v2, ABSOLUTE_ERROR));

        // 2nd triangle is formed by v2, v3 and v4
        Triangle3D triangle2 = triangles.get(1);
        assertTrue(triangle2.getVertex1().equals(v2, ABSOLUTE_ERROR));
        assertTrue(triangle2.getVertex2().equals(v3, ABSOLUTE_ERROR));
        assertTrue(triangle2.getVertex3().equals(v4, ABSOLUTE_ERROR));

        // General polygon
        // X: -1, Y: 0, Z: 5 (W: 1)
        v1 = new InhomogeneousPoint3D(-1.0, 0.0, 5.0);
        // X: 0, Y: 0.5, Z: 5 (W: 1)
        v2 = new InhomogeneousPoint3D(0.0, 0.5, 5.0);
        // X: 1, Y: 0, Z: 5 (W: 1)
        v3 = new InhomogeneousPoint3D(1.0, 0.0, 5.0);
        // X: 2, Y: 1, Z: 5 (W: 1)
        v4 = new InhomogeneousPoint3D(2.0, 1.0, 5.0);
        // X: 0, Y: 1, Z: 5 (W: 1)
        final Point3D v5 = new InhomogeneousPoint3D(0.0, 1.0, 5.0);

        polygonVertices.clear();
        polygonVertices.add(v1);
        polygonVertices.add(v2);
        polygonVertices.add(v3);
        polygonVertices.add(v4);
        polygonVertices.add(v5);

        triangles = triangulator.triangulate(polygonVertices);

        // divided in 3 triangles
        assertEquals(3, triangles.size());
        // 1st triangle is formed by v4, v1 and v2
        triangle1 = triangles.get(0);
        assertTrue(triangle1.getVertex1().equals(v5, ABSOLUTE_ERROR));
        assertTrue(triangle1.getVertex2().equals(v1, ABSOLUTE_ERROR));
        assertTrue(triangle1.getVertex3().equals(v2, ABSOLUTE_ERROR));
        // 2nd triangle is formed by v5, v2 and v3
        triangle2 = triangles.get(1);
        assertTrue(triangle2.getVertex1().equals(v5, ABSOLUTE_ERROR));
        assertTrue(triangle2.getVertex2().equals(v2, ABSOLUTE_ERROR));
        assertTrue(triangle2.getVertex3().equals(v3, ABSOLUTE_ERROR));
        // 3rd triangle is formed by v3, v4 and v5
        final Triangle3D triangle3 = triangles.get(2);
        assertTrue(triangle3.getVertex1().equals(v3, ABSOLUTE_ERROR));
        assertTrue(triangle3.getVertex2().equals(v4, ABSOLUTE_ERROR));
        assertTrue(triangle3.getVertex3().equals(v5, ABSOLUTE_ERROR));
    }

    @Test
    public void testTriangulate() throws NotEnoughVerticesException, TriangulatorException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        final double phi = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES);

        // build vertices list
        final List<Point3D> vertices = buildPolygonVertices(sides, radius, phi);

        // build polygon
        final Polygon3D polygon = new Polygon3D(vertices);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        // create point inside of vertex
        final Point3D point1 = new InhomogeneousPoint3D(
                radius / 2.0 * Math.cos(theta) * Math.cos(phi),
                radius / 2.0 * Math.sin(theta) * Math.cos(phi),
                radius * Math.sin(phi));

        // create point outside of vertex
        final Point3D point2 = new InhomogeneousPoint3D(
                2.0 * radius * Math.cos(theta) * Math.cos(phi),
                2.0 * radius * Math.sin(theta) * Math.cos(phi),
                radius * Math.sin(phi));

        final VanGoghTriangulator3D triangulator = new VanGoghTriangulator3D();
        final List<Triangle3D> triangles1 = triangulator.triangulate(polygon);
        final List<Triangle3D> triangles2 = triangulator.triangulate(vertices);
        final List<int[]> indices = new ArrayList<>();
        final List<Triangle3D> triangles3 = triangulator.triangulate(vertices, indices);

        double area1 = 0.0;
        boolean inside1 = false;
        boolean inside2 = false;
        for (final Triangle3D triangle : triangles1) {
            area1 += triangle.getArea();
            if (triangle.isInside(point1)) {
                inside1 = true;
            }
            if (triangle.isInside(point2)) {
                inside2 = true;
            }
        }

        assertEquals(area1, polygon.getArea(), ABSOLUTE_ERROR);

        // check that point1 lies inside, but point2 does not
        assertTrue(polygon.isInside(point1));
        assertTrue(inside1);

        assertFalse(polygon.isInside(point2));
        assertFalse(inside2);

        double area2 = 0.0;
        inside1 = false;
        for (final Triangle3D triangle : triangles2) {
            area2 += triangle.getArea();
            if (triangle.isInside(point1)) {
                inside1 = true;
            }
            if (triangle.isInside(point2)) {
                inside2 = true;
            }
        }

        assertEquals(area2, polygon.getArea(), ABSOLUTE_ERROR);

        // check that point1 lies inside, but point2 does not
        assertTrue(polygon.isInside(point1));
        assertTrue(inside1);

        assertFalse(polygon.isInside(point2));
        assertFalse(inside2);

        double area3 = 0.0;
        inside1 = false;
        for (final Triangle3D triangle : triangles3) {
            area3 += triangle.getArea();
            if (triangle.isInside(point1)) {
                inside1 = true;
            }
            if (triangle.isInside(point2)) {
                inside2 = true;
            }
        }

        assertEquals(area3, polygon.getArea(), ABSOLUTE_ERROR);

        // check that point1 lies inside, but point2 does not
        assertTrue(polygon.isInside(point1));
        assertTrue(inside1);

        assertFalse(polygon.isInside(point2));
        assertFalse(inside2);

        assertFalse(indices.isEmpty());
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
}
