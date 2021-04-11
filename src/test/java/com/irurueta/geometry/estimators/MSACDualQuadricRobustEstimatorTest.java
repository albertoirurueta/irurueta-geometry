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

import com.irurueta.geometry.DualQuadric;
import com.irurueta.geometry.DualQuadricNotAvailableException;
import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quadric;
import com.irurueta.geometry.Sphere;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class MSACDualQuadricRobustEstimatorTest implements
        DualQuadricRobustEstimatorListener {

    private static final int MIN_PLANES = 500;
    private static final int MAX_PLANES = 1000;

    private static final double MIN_RANDOM_POINT_VALUE = -1.0;
    private static final double MAX_RANDOM_POINT_VALUE = 1.0;

    private static final double STD_ERROR = 1.0;
    private static final int PERCENTAGE_OUTLIER = 20;

    private static final double THRESHOLD = 1e-7;
    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int TIMES = 100;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    public void testConstructor() {
        // test constructor without arguments
        MSACDualQuadricRobustEstimator estimator = new MSACDualQuadricRobustEstimator();

        // check correctness
        assertEquals(estimator.getThreshold(),
                RANSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // test constructor with planes
        final List<Plane> planes = new ArrayList<>();
        for (int i = 0; i < DualQuadricRobustEstimator.MINIMUM_SIZE; i++) {
            planes.add(new Plane());
        }

        estimator = new MSACDualQuadricRobustEstimator(planes);

        // check correctness
        assertEquals(estimator.getThreshold(),
                MSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        final List<Plane> emptyPlanes = new ArrayList<>();
        estimator = null;
        try {
            estimator = new MSACDualQuadricRobustEstimator(emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener
        final DualQuadricRobustEstimatorListener listener =
                new DualQuadricRobustEstimatorListener() {

                    @Override
                    public void onEstimateStart(final DualQuadricRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateEnd(final DualQuadricRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final DualQuadricRobustEstimator estimator, final int iteration) {
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final DualQuadricRobustEstimator estimator, final float progress) {
                    }
                };

        estimator = new MSACDualQuadricRobustEstimator(listener);

        // check correctness
        assertEquals(estimator.getThreshold(),
                MSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // test constructor with listener and points
        estimator = new MSACDualQuadricRobustEstimator(listener, planes);

        // check correctness
        assertEquals(estimator.getThreshold(),
                MSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new MSACDualQuadricRobustEstimator(listener,
                    emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetThreshold() throws LockedException {
        final MSACDualQuadricRobustEstimator estimator =
                new MSACDualQuadricRobustEstimator();

        // check default value
        assertEquals(estimator.getThreshold(),
                MSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD, 0.0);

        // set new value
        estimator.setThreshold(0.5);

        // check correctness
        assertEquals(estimator.getThreshold(), 0.5, 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final MSACDualQuadricRobustEstimator estimator =
                new MSACDualQuadricRobustEstimator();

        // check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());

        // set listener
        estimator.setListener(this);

        // check correctness
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final MSACDualQuadricRobustEstimator estimator =
                new MSACDualQuadricRobustEstimator();

        // check default value
        assertEquals(estimator.getProgressDelta(),
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check correctness
        assertEquals(estimator.getProgressDelta(), 0.5f, 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setProgressDelta(2.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetConfidence() throws LockedException {
        final MSACDualQuadricRobustEstimator estimator =
                new MSACDualQuadricRobustEstimator();

        // check default value
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);

        // set new value
        estimator.setConfidence(0.5);

        // check correctness
        assertEquals(estimator.getConfidence(), 0.5, 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMaxIterations() throws LockedException {
        final MSACDualQuadricRobustEstimator estimator =
                new MSACDualQuadricRobustEstimator();

        // check default value
        assertEquals(estimator.getMaxIterations(),
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);

        // set new value
        estimator.setMaxIterations(1);

        // check correctness
        assertEquals(estimator.getMaxIterations(), 1);

        // Force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetPlanes() throws LockedException {
        final MSACDualQuadricRobustEstimator estimator =
                new MSACDualQuadricRobustEstimator();

        // check default value
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());

        // set new value
        final List<Plane> planes = new ArrayList<>();
        for (int i = 0; i < DualQuadricRobustEstimator.MINIMUM_SIZE; i++) {
            planes.add(new Plane());
        }
        estimator.setPlanes(planes);

        // check correctness
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());

        // clearing list makes instance not ready
        planes.clear();

        assertFalse(estimator.isReady());

        // Force IllegalArgumentException
        final List<Plane> emptyPlanes = new ArrayList<>();
        try {
            estimator.setPlanes(emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final MSACDualQuadricRobustEstimator estimator =
                new MSACDualQuadricRobustEstimator();

        assertNull(estimator.getQualityScores());

        final double[] qualityScores =
                new double[DualQuadricRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertNull(estimator.getQualityScores());
    }

    @Test
    public void testEstimate() throws LockedException, NotReadyException,
            RobustEstimatorException, DualQuadricNotAvailableException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            // instantiate a random circle
            final Point3D center = new HomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE), 1.0);
            final double radius = Math.abs(randomizer.nextDouble(
                    MAX_RANDOM_POINT_VALUE / 2.0,
                    MAX_RANDOM_POINT_VALUE));

            final Sphere sphere = new Sphere(center, radius);
            final Quadric quadric = sphere.toQuadric();
            final DualQuadric dualQuadric = quadric.getDualQuadric();

            // compute planes in the dual quadric locus
            final int nPlanes = randomizer.nextInt(MIN_PLANES, MAX_PLANES);
            final int halfPoints = (int) Math.ceil((double) nPlanes / 2.0);
            final double theta = (double) nPlanes / 360.0 * Math.PI / 180.0;
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            final List<Plane> planes = new ArrayList<>();
            final List<Plane> planesWithError = new ArrayList<>();
            Point3D point;
            Plane plane, planeWithError;
            final double[] directorVector = new double[
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            for (int i = 0; i < nPlanes; i++) {
                double angle1 = 0.0, angle2 = 0.0;
                if (i < halfPoints) {
                    angle1 = theta * (double) i;
                } else {
                    angle2 = theta * (double) (i - halfPoints);
                }
                point = new HomogeneousPoint3D(
                        center.getInhomX() + radius * Math.cos(angle1) *
                                Math.cos(angle2),
                        center.getInhomY() + radius * Math.sin(angle1) *
                                Math.cos(angle2),
                        center.getInhomZ() + radius * Math.sin(angle2),
                        1.0);
                directorVector[0] = point.getInhomX() - center.getInhomX();
                directorVector[1] = point.getInhomY() - center.getInhomY();
                directorVector[2] = point.getInhomZ() - center.getInhomZ();

                plane = new Plane(point, directorVector);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final double errorA = errorRandomizer.nextDouble();
                    final double errorB = errorRandomizer.nextDouble();
                    final double errorC = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD());
                } else {
                    // inlier plane
                    planeWithError = plane;
                }

                planes.add(plane);
                planesWithError.add(planeWithError);

                // check that point without error is within quadric locus
                assertTrue(sphere.isLocus(point, ABSOLUTE_ERROR));
                assertTrue(quadric.isLocus(point, ABSOLUTE_ERROR));
                assertTrue(dualQuadric.isLocus(plane, ABSOLUTE_ERROR));
            }

            final MSACDualQuadricRobustEstimator estimator =
                    new MSACDualQuadricRobustEstimator(this, planesWithError);

            estimator.setThreshold(THRESHOLD);

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final DualQuadric dualQuadric2 = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by checking that all planes
            // are within the estimated conic locus
            for (final Plane p : planes) {
                assertTrue(dualQuadric2.isLocus(p, ABSOLUTE_ERROR));
            }
        }
    }

    @Override
    public void onEstimateStart(final DualQuadricRobustEstimator estimator) {
        estimateStart++;
        checkLocked((MSACDualQuadricRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final DualQuadricRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((MSACDualQuadricRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(final DualQuadricRobustEstimator estimator,
                                        final int iteration) {
        estimateNextIteration++;
        checkLocked((MSACDualQuadricRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(final DualQuadricRobustEstimator estimator,
                                         final float progress) {
        estimateProgressChange++;
        checkLocked((MSACDualQuadricRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void checkLocked(final MSACDualQuadricRobustEstimator estimator) {
        try {
            estimator.setThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setConfidence(0.5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setMaxIterations(5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setPlanes(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
        assertTrue(estimator.isLocked());
    }
}
