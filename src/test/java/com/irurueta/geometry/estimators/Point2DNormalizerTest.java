/*
 * Copyright (C) 2015 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.geometry.estimators;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class Point2DNormalizerTest {
    private static final int MIN_POINTS = 2;
    private static final int MAX_POINTS = 10;

    private static final double MIN_VALUE = -10.0;
    private static final double MAX_VALUE = 10.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int TIMES = 100;

    @Test
    void testConstants() {
        assertEquals(2, Point2DNormalizer.MIN_POINTS);
    }

    @Test
    void testConstructor() {
        final var randomizer = new UniformRandomizer();

        final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        final var points = new ArrayList<Point2D>();
        for (var i = 0; i < nPoints; i++) {
            final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var point = new InhomogeneousPoint2D(x, y);
            points.add(point);
        }

        var normalizer = new Point2DNormalizer(points);

        // check default values
        assertSame(points, normalizer.getPoints());
        assertTrue(normalizer.isReady());
        assertFalse(normalizer.isLocked());
        assertEquals(Double.MAX_VALUE, normalizer.getMinInhomX(), 0.0);
        assertEquals(Double.MAX_VALUE, normalizer.getMinInhomY(), 0.0);
        assertEquals(-Double.MAX_VALUE, normalizer.getMaxInhomX(), 0.0);
        assertEquals(-Double.MAX_VALUE, normalizer.getMaxInhomY(), 0.0);
        assertEquals(1.0, normalizer.getScaleX(), 0.0);
        assertEquals(1.0, normalizer.getScaleY(), 0.0);
        assertEquals(0.0, normalizer.getCentroidX(), 0.0);
        assertEquals(0.0, normalizer.getCentroidY(), 0.0);
        assertNull(normalizer.getTransformation());
        assertNull(normalizer.getInverseTransformation());
        assertFalse(normalizer.isResultAvailable());

        // Force IllegalArgumentException
        points.clear();
        assertFalse(normalizer.isReady());

        assertThrows(IllegalArgumentException.class, () -> new Point2DNormalizer(points));
    }

    @Test
    void testGetSetPoints() throws LockedException {
        final var randomizer = new UniformRandomizer();

        final var nPoints1 = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        final var points1 = new ArrayList<Point2D>();
        for (var i = 0; i < nPoints1; i++) {
            final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var point = new InhomogeneousPoint2D(x, y);
            points1.add(point);
        }

        final var nPoints2 = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        final var points2 = new ArrayList<Point2D>();
        for (var i = 0; i < nPoints2; i++) {
            final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var point = new InhomogeneousPoint2D(x, y);
            points2.add(point);
        }

        final var normalizer = new Point2DNormalizer(points1);

        // check default value
        assertSame(points1, normalizer.getPoints());

        // set new value
        normalizer.setPoints(points2);

        // check correctness
        assertSame(points2, normalizer.getPoints());
    }

    @Test
    void testCompute() throws NotReadyException, LockedException, WrongSizeException, NormalizerException {
        final var randomizer = new UniformRandomizer();

        for (var times = 0; times < TIMES; times++) {
            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var points = new ArrayList<Point2D>();
            var minX = Double.MAX_VALUE;
            var minY = Double.MAX_VALUE;
            var maxX = -Double.MAX_VALUE;
            var maxY = -Double.MAX_VALUE;
            for (var i = 0; i < nPoints; i++) {
                final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
                final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
                final var point = new InhomogeneousPoint2D(x, y);
                points.add(point);

                if (x < minX) {
                    minX = x;
                }
                if (y < minY) {
                    minY = y;
                }
                if (x > maxX) {
                    maxX = x;
                }
                if (y > maxY) {
                    maxY = y;
                }
            }

            var width = maxX - minX;
            var height = maxY - minY;
            var scaleX = 1.0 / width;
            var scaleY = 1.0 / height;
            var centroidX = (minX + maxX) / 2.0;
            var centroidY = (minY + maxY) / 2.0;

            final var normalizer = new Point2DNormalizer(points);

            assertTrue(normalizer.isReady());
            assertFalse(normalizer.isLocked());

            normalizer.compute();

            assertFalse(normalizer.isLocked());

            assertEquals(minX, normalizer.getMinInhomX(), 0.0);
            assertEquals(minY, normalizer.getMinInhomY(), 0.0);
            assertEquals(maxX, normalizer.getMaxInhomX(), 0.0);
            assertEquals(maxY, normalizer.getMaxInhomY(), 0.0);
            assertEquals(scaleX, normalizer.getScaleX(), 0.0);
            assertEquals(scaleY, normalizer.getScaleY(), 0.0);
            assertEquals(centroidX, normalizer.getCentroidX(), 0.0);
            assertEquals(centroidY, normalizer.getCentroidY(), 0.0);

            final var transformation = normalizer.getTransformation();

            final var invTransformation = normalizer.getInverseTransformation();

            // test that invTransformation is indeed the inverse transformation
            final var t = transformation.asMatrix();
            final var invT = invTransformation.asMatrix();

            final var identity = invT.multiplyAndReturnNew(t);
            final var norm = identity.getElementAt(0, 0);
            // normalize
            identity.multiplyByScalar(1.0 / norm);

            assertTrue(identity.equals(Matrix.identity(Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH,
                            Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH), ABSOLUTE_ERROR));

            // normalize points
            final var normPoints = transformation.transformPointsAndReturnNew(points);

            // compute centroid and scales
            minX = minY = Double.MAX_VALUE;
            maxX = maxY = -Double.MAX_VALUE;
            for (final var normPoint : normPoints) {
                final var x = normPoint.getInhomX();
                final var y = normPoint.getInhomY();

                if (x < minX) {
                    minX = x;
                }
                if (y < minY) {
                    minY = y;
                }
                if (x > maxX) {
                    maxX = x;
                }
                if (y > maxY) {
                    maxY = y;
                }
            }

            width = maxX - minX;
            height = maxY - minY;
            scaleX = 1.0 / width;
            scaleY = 1.0 / height;
            centroidX = (minX + maxX) / 2.0;
            centroidY = (minY + maxY) / 2.0;

            // check that points have been correctly normalized (scales = 1 and
            // centroid = [0, 0])
            assertEquals(1.0, width, ABSOLUTE_ERROR);
            assertEquals(1.0, height, ABSOLUTE_ERROR);
            assertEquals(1.0, scaleX, ABSOLUTE_ERROR);
            assertEquals(1.0, scaleY, ABSOLUTE_ERROR);
            assertEquals(0.0, centroidX, ABSOLUTE_ERROR);
            assertEquals(0.0, centroidY, ABSOLUTE_ERROR);

            // denormalize points and check that are equal to the original ones
            final var denomPoints = invTransformation.transformPointsAndReturnNew(normPoints);

            for (var i = 0; i < nPoints; i++) {
                assertEquals(0.0, points.get(i).distanceTo(denomPoints.get(i)), ABSOLUTE_ERROR);
            }
        }
    }
}
