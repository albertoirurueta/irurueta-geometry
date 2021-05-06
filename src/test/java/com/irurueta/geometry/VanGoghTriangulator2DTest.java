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

public class VanGoghTriangulator2DTest {

    private static final int MIN_SIDES = 3;
    private static final int MAX_SIDES = 12;

    private static final double MIN_RADIUS = 1.0;
    private static final double MAX_RADIUS = 10.0;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    public void testConstructorAndGetMethod() {
        final VanGoghTriangulator2D triangulator = new VanGoghTriangulator2D();
        assertNotNull(triangulator);

        // check method correctness
        assertEquals(triangulator.getMethod(),
                TriangulatorMethod.VAN_GOGH_TRIANGULATOR);
    }

    @Test
    public void testTriangulate() throws NotEnoughVerticesException,
            TriangulatorException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        final double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);

        // build vertices list
        final List<Point2D> vertices = buildPolygonVertices(sides, radius);

        // build polygon
        final Polygon2D polygon = new Polygon2D(vertices);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        // create point inside of vertex
        final Point2D point1 = new InhomogeneousPoint2D(
                radius / 2.0 * Math.cos(theta),
                radius / 2.0 * Math.sin(theta));

        // create point outside of vertex
        final Point2D point2 = new InhomogeneousPoint2D(
                2.0 * radius * Math.cos(theta),
                2.0 * radius * Math.sin(theta));

        final VanGoghTriangulator2D triangulator = new VanGoghTriangulator2D();
        final List<Triangle2D> triangles1 = triangulator.triangulate(polygon);
        final List<Triangle2D> triangles2 = triangulator.triangulate(vertices);
        final List<int[]> indices = new ArrayList<>();
        final List<Triangle2D> triangles3 = triangulator.triangulate(vertices,
                indices);

        double signedArea1 = 0.0;
        double area1 = 0.0;
        boolean inside1 = false;
        boolean inside2 = false;
        for (final Triangle2D triangle : triangles1) {
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

        double signedArea2 = 0.0;
        double area2 = 0.0;
        inside1 = inside2 = false;
        for (final Triangle2D triangle : triangles2) {
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

        double area3 = 0.0;
        inside1 = inside2 = false;
        for (final Triangle2D triangle : triangles3) {
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

        assertTrue(indices.size() > 0);
    }

    private List<Point2D> buildPolygonVertices(
            final int sides, final double radius) {
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
