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
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class Triangulator3DTest {

    private static final int MIN_SIDES = 3;
    private static final int MAX_SIDES = 12;

    private static final double MIN_RADIUS = 1.0;
    private static final double MAX_RADIUS = 10.0;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    void testConstants() {
        assertEquals(3, Triangulator3D.MIN_VERTICES);
        assertEquals(TriangulatorMethod.VAN_GOGH_TRIANGULATOR,
                Triangulator3D.DEFAULT_TRIANGULATOR_METHOD);
    }

    @Test
    void testCreateAndGetMethod() {
        var triangulator = Triangulator3D.create();
        assertNotNull(triangulator);

        // check method correctness
        assertEquals(Triangulator3D.DEFAULT_TRIANGULATOR_METHOD, triangulator.getMethod());

        // create with method
        triangulator = Triangulator3D.create(TriangulatorMethod.VAN_GOGH_TRIANGULATOR);

        // check method correctness
        assertEquals(TriangulatorMethod.VAN_GOGH_TRIANGULATOR, triangulator.getMethod());
    }

    @Test
    void testTriangulate() throws NotEnoughVerticesException, TriangulatorException {

        final var randomizer = new UniformRandomizer();
        final var sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final var radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        final var phi = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);

        // build vertices list
        final var vertices = buildPolygonVertices(sides, radius, phi);

        // build polygon
        final var polygon = new Polygon3D(vertices);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        // create point inside of vertex
        final var point1 = new InhomogeneousPoint3D(
                radius / 2.0 * Math.cos(theta) * Math.cos(phi),
                radius / 2.0 * Math.sin(theta) * Math.cos(phi),
                radius * Math.sin(phi));

        // create point outside of vertex
        final var point2 = new InhomogeneousPoint3D(
                2.0 * radius * Math.cos(theta) * Math.cos(phi),
                2.0 * radius * Math.sin(theta) * Math.cos(phi),
                radius * Math.sin(phi));

        final var triangulator = Triangulator3D.create();
        final var triangles1 = triangulator.triangulate(polygon);
        final var triangles2 = triangulator.triangulate(vertices);
        final var indices = new ArrayList<int[]>();
        final var triangles3 = triangulator.triangulate(vertices, indices);

        var area1 = 0.0;
        var inside1 = false;
        var inside2 = false;
        for (final var triangle : triangles1) {
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
        for (final var triangle : triangles2) {
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

        var area3 = 0.0;
        inside1 = false;
        for (final var triangle : triangles3) {
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
}
