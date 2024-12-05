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

class Triangulator2DTest {

    private static final int MIN_SIDES = 3;
    private static final int MAX_SIDES = 12;

    private static final double MIN_RADIUS = 1.0;
    private static final double MAX_RADIUS = 10.0;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    void testConstants() {
        assertEquals(3, Triangulator2D.MIN_VERTICES);
        assertEquals(TriangulatorMethod.VAN_GOGH_TRIANGULATOR, Triangulator2D.DEFAULT_TRIANGULATOR_METHOD);
    }

    @Test
    void testCreateAndGetMethod() {
        var triangulator = Triangulator2D.create();
        assertNotNull(triangulator);

        // check method correctness
        assertEquals(Triangulator2D.DEFAULT_TRIANGULATOR_METHOD, triangulator.getMethod());

        // create with method
        triangulator = Triangulator2D.create(TriangulatorMethod.VAN_GOGH_TRIANGULATOR);

        // check method correctness
        assertEquals(TriangulatorMethod.VAN_GOGH_TRIANGULATOR, triangulator.getMethod());
    }

    @Test
    void testTriangulate() throws NotEnoughVerticesException, TriangulatorException {

        final var randomizer = new UniformRandomizer();
        final var sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final var radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        // build vertices list
        final var vertices = buildPolygonVertices(sides, radius);

        // build polygon
        final var polygon = new Polygon2D(vertices);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        // create point inside of vertex
        final var point1 = new InhomogeneousPoint2D(
                radius / 2.0 * Math.cos(theta),
                radius / 2.0 * Math.sin(theta));

        // create point outside of vertex
        final var point2 = new InhomogeneousPoint2D(
                2.0 * radius * Math.cos(theta),
                2.0 * radius * Math.sin(theta));

        final var triangulator = Triangulator2D.create();
        final var triangles1 = triangulator.triangulate(polygon);
        final var triangles2 = triangulator.triangulate(vertices);

        var signedArea1 = 0.0;
        var area1 = 0.0;
        var inside1 = false;
        var inside2 = false;
        for (final var triangle : triangles1) {
            signedArea1 += triangle.getSignedArea();
            area1 += triangle.getArea();
            if (triangle.isInside(point1)) {
                inside1 = true;
            }
            if (triangle.isInside(point2)) {
                inside2 = true;
            }
        }

        assertEquals(signedArea1, polygon.getSignedArea(), ABSOLUTE_ERROR);
        assertEquals(area1, polygon.getArea(), ABSOLUTE_ERROR);

        // check that point1 lies inside, but point2 does not
        assertTrue(polygon.isInside(point1));
        assertTrue(inside1);

        assertFalse(polygon.isInside(point2));
        assertFalse(inside2);

        var signedArea2 = 0.0;
        var area2 = 0.0;
        inside1 = false;
        for (final var triangle : triangles2) {
            signedArea2 += triangle.getSignedArea();
            area2 += triangle.getArea();
            if (triangle.isInside(point1)) {
                inside1 = true;
            }
            if (triangle.isInside(point2)) {
                inside2 = true;
            }
        }

        assertEquals(signedArea2, polygon.getSignedArea(), ABSOLUTE_ERROR);
        assertEquals(area2, polygon.getArea(), ABSOLUTE_ERROR);

        // check that point1 lies inside, but point2 does not
        assertTrue(polygon.isInside(point1));
        assertTrue(inside1);

        assertFalse(polygon.isInside(point2));
        assertFalse(inside2);
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
