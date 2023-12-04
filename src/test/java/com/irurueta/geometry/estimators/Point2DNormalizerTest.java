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
import com.irurueta.geometry.ProjectiveTransformation2D;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class Point2DNormalizerTest {
    private static final int MIN_POINTS = 2;
    private static final int MAX_POINTS = 10;

    private static final double MIN_VALUE = -10.0;
    private static final double MAX_VALUE = 10.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int TIMES = 100;

    @Test
    public void testConstants() {
        assertEquals(2, Point2DNormalizer.MIN_POINTS);
    }

    @Test
    public void testConstructor() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        final List<Point2D> points = new ArrayList<>();
        InhomogeneousPoint2D point;
        for (int i = 0; i < nPoints; i++) {
            final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            point = new InhomogeneousPoint2D(x, y);
            points.add(point);
        }

        Point2DNormalizer normalizer = new Point2DNormalizer(points);

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

        normalizer = null;
        try {
            normalizer = new Point2DNormalizer(points);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(normalizer);
    }

    @Test
    public void testGetSetPoints() throws LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final int nPoints1 = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        final List<Point2D> points1 = new ArrayList<>();
        InhomogeneousPoint2D point;
        for (int i = 0; i < nPoints1; i++) {
            final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            point = new InhomogeneousPoint2D(x, y);
            points1.add(point);
        }

        final int nPoints2 = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        final List<Point2D> points2 = new ArrayList<>();
        for (int i = 0; i < nPoints2; i++) {
            final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            point = new InhomogeneousPoint2D(x, y);
            points2.add(point);
        }

        final Point2DNormalizer normalizer = new Point2DNormalizer(points1);

        // check default value
        assertSame(points1, normalizer.getPoints());

        // set new value
        normalizer.setPoints(points2);

        // check correctness
        assertSame(points2, normalizer.getPoints());
    }

    @Test
    public void testCompute() throws NotReadyException, LockedException,
            WrongSizeException, NormalizerException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int times = 0; times < TIMES; times++) {
            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final List<Point2D> points = new ArrayList<>();
            InhomogeneousPoint2D point;
            double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE,
                    maxX = -Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
            for (int i = 0; i < nPoints; i++) {
                final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
                final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
                point = new InhomogeneousPoint2D(x, y);
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

            double width = maxX - minX;
            double height = maxY - minY;
            double scaleX = 1.0 / width;
            double scaleY = 1.0 / height;
            double centroidX = (minX + maxX) / 2.0;
            double centroidY = (minY + maxY) / 2.0;

            final Point2DNormalizer normalizer = new Point2DNormalizer(points);

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

            final ProjectiveTransformation2D transformation =
                    normalizer.getTransformation();

            final ProjectiveTransformation2D invTransformation =
                    normalizer.getInverseTransformation();

            // test that invTransformation is indeed the inverse transformation
            final Matrix t = transformation.asMatrix();
            final Matrix invT = invTransformation.asMatrix();

            final Matrix identity = invT.multiplyAndReturnNew(t);
            final double norm = identity.getElementAt(0, 0);
            // normalize
            identity.multiplyByScalar(1.0 / norm);

            assertTrue(identity.equals(
                    Matrix.identity(Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH,
                            Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH),
                    ABSOLUTE_ERROR));

            // normalize points
            final List<Point2D> normPoints = transformation.transformPointsAndReturnNew(points);

            // compute centroid and scales
            minX = minY = Double.MAX_VALUE;
            maxX = maxY = -Double.MAX_VALUE;
            for (final Point2D normPoint : normPoints) {
                final double x = normPoint.getInhomX();
                final double y = normPoint.getInhomY();

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
            final List<Point2D> denomPoints = invTransformation.transformPointsAndReturnNew(
                    normPoints);

            for (int i = 0; i < nPoints; i++) {
                assertEquals(0.0, points.get(i).distanceTo(denomPoints.get(i)),
                        ABSOLUTE_ERROR);
            }
        }
    }
}
