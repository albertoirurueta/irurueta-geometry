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
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.ProjectiveTransformation3D;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class Point3DNormalizerTest {
    private static final int MIN_POINTS = 2;
    private static final int MAX_POINTS = 10;

    private static final double MIN_VALUE = -10.0;
    private static final double MAX_VALUE = 10.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int TIMES = 100;

    @Test
    public void testConstants() {
        assertEquals(2, Point3DNormalizer.MIN_POINTS);
    }

    @Test
    public void testConstructor() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        final List<Point3D> points = new ArrayList<>();
        InhomogeneousPoint3D point;
        for (int i = 0; i < nPoints; i++) {
            final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            point = new InhomogeneousPoint3D(x, y, z);
            points.add(point);
        }

        Point3DNormalizer normalizer = new Point3DNormalizer(points);

        // check default values
        assertSame(normalizer.getPoints(), points);
        assertTrue(normalizer.isReady());
        assertFalse(normalizer.isLocked());
        assertEquals(Double.MAX_VALUE, normalizer.getMinInhomX(), 0.0);
        assertEquals(Double.MAX_VALUE, normalizer.getMinInhomY(), 0.0);
        assertEquals(Double.MAX_VALUE, normalizer.getMinInhomZ(), 0.0);
        assertEquals(-Double.MAX_VALUE, normalizer.getMaxInhomX(), 0.0);
        assertEquals(-Double.MAX_VALUE, normalizer.getMaxInhomY(), 0.0);
        assertEquals(-Double.MAX_VALUE, normalizer.getMaxInhomZ(), 0.0);
        assertEquals(1.0, normalizer.getScaleX(), 0.0);
        assertEquals(1.0, normalizer.getScaleY(), 0.0);
        assertEquals(1.0, normalizer.getScaleZ(), 0.0);
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
            normalizer = new Point3DNormalizer(points);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(normalizer);
    }

    @Test
    public void testGetSetPoints() throws LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final int nPoints1 = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        final List<Point3D> points1 = new ArrayList<>();
        InhomogeneousPoint3D point;
        for (int i = 0; i < nPoints1; i++) {
            final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            point = new InhomogeneousPoint3D(x, y, z);
            points1.add(point);
        }

        final int nPoints2 = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        final List<Point3D> points2 = new ArrayList<>();
        for (int i = 0; i < nPoints2; i++) {
            final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            point = new InhomogeneousPoint3D(x, y, z);
            points2.add(point);
        }

        final Point3DNormalizer normalizer = new Point3DNormalizer(points1);

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
            final List<Point3D> points = new ArrayList<>();
            InhomogeneousPoint3D point;
            double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE,
                    minZ = Double.MAX_VALUE, maxX = -Double.MAX_VALUE,
                    maxY = -Double.MAX_VALUE, maxZ = -Double.MAX_VALUE;
            for (int i = 0; i < nPoints; i++) {
                final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
                final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
                final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
                point = new InhomogeneousPoint3D(x, y, z);
                points.add(point);

                if (x < minX) {
                    minX = x;
                }
                if (y < minY) {
                    minY = y;
                }
                if (z < minZ) {
                    minZ = z;
                }
                if (x > maxX) {
                    maxX = x;
                }
                if (y > maxY) {
                    maxY = y;
                }
                if (z > maxZ) {
                    maxZ = z;
                }
            }

            double width = maxX - minX;
            double height = maxY - minY;
            double depth = maxZ - minZ;
            double scaleX = 1.0 / width;
            double scaleY = 1.0 / height;
            double scaleZ = 1.0 / depth;
            double centroidX = (minX + maxX) / 2.0;
            double centroidY = (minY + maxY) / 2.0;
            double centroidZ = (minZ + maxZ) / 2.0;

            final Point3DNormalizer normalizer = new Point3DNormalizer(points);

            assertTrue(normalizer.isReady());
            assertFalse(normalizer.isLocked());

            normalizer.compute();

            assertFalse(normalizer.isLocked());

            assertEquals(minX, normalizer.getMinInhomX(), 0.0);
            assertEquals(minY, normalizer.getMinInhomY(), 0.0);
            assertEquals(minZ, normalizer.getMinInhomZ(), 0.0);
            assertEquals(maxX, normalizer.getMaxInhomX(), 0.0);
            assertEquals(maxY, normalizer.getMaxInhomY(),0.0);
            assertEquals(maxZ, normalizer.getMaxInhomZ(), 0.0);
            assertEquals(scaleX, normalizer.getScaleX(), 0.0);
            assertEquals(scaleY, normalizer.getScaleY(), 0.0);
            assertEquals(scaleZ, normalizer.getScaleZ(), 0.0);
            assertEquals(centroidX, normalizer.getCentroidX(), 0.0);
            assertEquals(centroidY, normalizer.getCentroidY(), 0.0);
            assertEquals(centroidZ, normalizer.getCentroidZ(), 0.0);

            final ProjectiveTransformation3D transformation =
                    normalizer.getTransformation();

            final ProjectiveTransformation3D invTransformation =
                    normalizer.getInverseTransformation();

            // test that invTransformation is indeed the inverse transformation
            final Matrix t = transformation.asMatrix();
            final Matrix invT = invTransformation.asMatrix();

            final Matrix identity = invT.multiplyAndReturnNew(t);
            final double norm = identity.getElementAt(0, 0);
            // normalize
            identity.multiplyByScalar(1.0 / norm);

            assertTrue(identity.equals(Matrix.identity(
                    Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH,
                    Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH),
                    ABSOLUTE_ERROR));

            // normalize points
            final List<Point3D> normPoints = transformation.transformPointsAndReturnNew(points);

            // compute centroid and scales
            minX = minY = minZ = Double.MAX_VALUE;
            maxX = maxY = maxZ = -Double.MAX_VALUE;
            for (final Point3D normPoint : normPoints) {
                final double x = normPoint.getInhomX();
                final double y = normPoint.getInhomY();
                final double z = normPoint.getInhomZ();

                if (x < minX) {
                    minX = x;
                }
                if (y < minY) {
                    minY = y;
                }
                if (z < minZ) {
                    minZ = z;
                }
                if (x > maxX) {
                    maxX = x;
                }
                if (y > maxY) {
                    maxY = y;
                }
                if (z > maxZ) {
                    maxZ = z;
                }
            }

            width = maxX - minX;
            height = maxY - minY;
            depth = maxZ - minZ;
            scaleX = 1.0 / width;
            scaleY = 1.0 / height;
            scaleZ = 1.0 / depth;
            centroidX = (minX + maxX) / 2.0;
            centroidY = (minY + maxY) / 2.0;
            centroidZ = (minZ + maxZ) / 2.0;

            // check that points have been correctly normalized (scales = 1 and
            // centroid = [0,0,0])
            assertEquals(1.0, width, ABSOLUTE_ERROR);
            assertEquals(1.0, height, ABSOLUTE_ERROR);
            assertEquals(1.0, depth, ABSOLUTE_ERROR);
            assertEquals(1.0, scaleX, ABSOLUTE_ERROR);
            assertEquals(1.0, scaleY, ABSOLUTE_ERROR);
            assertEquals(1.0, scaleZ, ABSOLUTE_ERROR);
            assertEquals(0.0, centroidX, ABSOLUTE_ERROR);
            assertEquals(0.0, centroidY, ABSOLUTE_ERROR);
            assertEquals(0.0, centroidZ, ABSOLUTE_ERROR);

            // denormalize points and check that are equal to the original ones
            final List<Point3D> denomPoints = invTransformation.transformPointsAndReturnNew(
                    normPoints);

            for (int i = 0; i < nPoints; i++) {
                assertEquals(0.0, points.get(i).distanceTo(denomPoints.get(i)),
                        ABSOLUTE_ERROR);
            }
        }
    }
}
