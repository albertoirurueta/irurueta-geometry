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

public class Triangulator3DTest {

    private static final int MIN_SIDES = 3;
    private static final int MAX_SIDES = 12;

    private static final double MIN_RADIUS = 1.0;
    private static final double MAX_RADIUS = 10.0;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    public void testConstants() {
        assertEquals(3, Triangulator3D.MIN_VERTICES);
        assertEquals(TriangulatorMethod.VAN_GOGH_TRIANGULATOR,
                Triangulator3D.DEFAULT_TRIANGULATOR_METHOD);
    }

    @Test
    public void testCreateAndGetMethod() {
        Triangulator3D triangulator = Triangulator3D.create();
        assertNotNull(triangulator);

        // check method correctness
        assertEquals(Triangulator3D.DEFAULT_TRIANGULATOR_METHOD,
                triangulator.getMethod());

        // create with method
        triangulator = Triangulator3D.create(TriangulatorMethod.VAN_GOGH_TRIANGULATOR);

        // check method correctness
        assertEquals(TriangulatorMethod.VAN_GOGH_TRIANGULATOR, triangulator.getMethod());
    }

    @Test
    public void testTriangulate() throws NotEnoughVerticesException,
            TriangulatorException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        final double phi = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);

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

        final Triangulator3D triangulator = Triangulator3D.create();
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
