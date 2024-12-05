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

import com.irurueta.geometry.DualQuadricNotAvailableException;
import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Sphere;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class MSACDualQuadricRobustEstimatorTest implements DualQuadricRobustEstimatorListener {

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
    void testConstants() {
        assertEquals(1e-7, MSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(0.0, MSACDualQuadricRobustEstimator.MIN_THRESHOLD, 0.0);
    }

    @Test
    void testConstructor() {
        // test constructor without arguments
        var estimator = new MSACDualQuadricRobustEstimator();

        // check correctness
        assertEquals(RANSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // test constructor with planes
        final var planes = new ArrayList<Plane>();
        for (var i = 0; i < DualQuadricRobustEstimator.MINIMUM_SIZE; i++) {
            planes.add(new Plane());
        }

        estimator = new MSACDualQuadricRobustEstimator(planes);

        // check correctness
        assertEquals(MSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(planes, estimator.getPlanes());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        final var emptyPlanes = new ArrayList<Plane>();
        assertThrows(IllegalArgumentException.class, () -> new MSACDualQuadricRobustEstimator(emptyPlanes));

        // test constructor with listener
        final var listener = new DualQuadricRobustEstimatorListener() {

            @Override
            public void onEstimateStart(final DualQuadricRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateEnd(final DualQuadricRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateNextIteration(final DualQuadricRobustEstimator estimator, final int iteration) {
                // no action needed
            }

            @Override
            public void onEstimateProgressChange(final DualQuadricRobustEstimator estimator, final float progress) {
                // no action needed
            }
        };

        estimator = new MSACDualQuadricRobustEstimator(listener);

        // check correctness
        assertEquals(MSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // test constructor with listener and points
        estimator = new MSACDualQuadricRobustEstimator(listener, planes);

        // check correctness
        assertEquals(MSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(planes, estimator.getPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new MSACDualQuadricRobustEstimator(listener, emptyPlanes));
    }

    @Test
    void testGetSetThreshold() throws LockedException {
        final var estimator = new MSACDualQuadricRobustEstimator();

        // check default value
        assertEquals(MSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);

        // set new value
        estimator.setThreshold(0.5);

        // check correctness
        assertEquals(0.5, estimator.getThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setThreshold(0.0));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new MSACDualQuadricRobustEstimator();

        // check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());

        // set listener
        estimator.setListener(this);

        // check correctness
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
    }

    @Test
    void testGetSetProgressDelta() throws LockedException {
        final var estimator = new MSACDualQuadricRobustEstimator();

        // check default value
        assertEquals(DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check correctness
        assertEquals(0.5f, estimator.getProgressDelta(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setProgressDelta(-1.0f));
        assertThrows(IllegalArgumentException.class, () -> estimator.setProgressDelta(2.0f));
    }

    @Test
    void testGetSetConfidence() throws LockedException {
        final var estimator = new MSACDualQuadricRobustEstimator();

        // check default value
        assertEquals(DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);

        // set new value
        estimator.setConfidence(0.5);

        // check correctness
        assertEquals(0.5, estimator.getConfidence(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setConfidence(-1.0));
        assertThrows(IllegalArgumentException.class, () -> estimator.setConfidence(2.0));
    }

    @Test
    void testGetSetMaxIterations() throws LockedException {
        final var estimator = new MSACDualQuadricRobustEstimator();

        // check default value
        assertEquals(DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(1);

        // check correctness
        assertEquals(1, estimator.getMaxIterations());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setMaxIterations(0));
    }

    @Test
    void testGetSetPlanes() throws LockedException {
        final var estimator = new MSACDualQuadricRobustEstimator();

        // check default value
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());

        // set new value
        final var planes = new ArrayList<Plane>();
        for (var i = 0; i < DualQuadricRobustEstimator.MINIMUM_SIZE; i++) {
            planes.add(new Plane());
        }
        estimator.setPlanes(planes);

        // check correctness
        assertSame(planes, estimator.getPlanes());
        assertTrue(estimator.isReady());

        // clearing list makes instance not ready
        planes.clear();

        assertFalse(estimator.isReady());

        // Force IllegalArgumentException
        final var emptyPlanes = new ArrayList<Plane>();
        assertThrows(IllegalArgumentException.class, () -> estimator.setPlanes(emptyPlanes));
    }

    @Test
    void testGetSetQualityScores() throws LockedException {
        final var estimator = new MSACDualQuadricRobustEstimator();

        assertNull(estimator.getQualityScores());

        final var qualityScores = new double[DualQuadricRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertNull(estimator.getQualityScores());
    }

    @Test
    void testEstimate() throws LockedException, NotReadyException, RobustEstimatorException,
            DualQuadricNotAvailableException {

        final var randomizer = new UniformRandomizer();

        for (var t = 0; t < TIMES; t++) {
            // instantiate a random circle
            final var center = new HomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE), 1.0);
            final var radius = Math.abs(randomizer.nextDouble(
                    MAX_RANDOM_POINT_VALUE / 2.0, MAX_RANDOM_POINT_VALUE));

            final var sphere = new Sphere(center, radius);
            final var quadric = sphere.toQuadric();
            final var dualQuadric = quadric.getDualQuadric();

            // compute planes in the dual quadric locus
            final var nPlanes = randomizer.nextInt(MIN_PLANES, MAX_PLANES);
            final var halfPoints = (int) Math.ceil((double) nPlanes / 2.0);
            final var theta = (double) nPlanes / 360.0 * Math.PI / 180.0;
            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            final var planes = new ArrayList<Plane>();
            final var planesWithError = new ArrayList<Plane>();
            final var directorVector = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            for (var i = 0; i < nPlanes; i++) {
                var angle1 = 0.0;
                var angle2 = 0.0;
                if (i < halfPoints) {
                    angle1 = theta * (double) i;
                } else {
                    angle2 = theta * (double) (i - halfPoints);
                }
                final var point = new HomogeneousPoint3D(
                        center.getInhomX() + radius * Math.cos(angle1) * Math.cos(angle2),
                        center.getInhomY() + radius * Math.sin(angle1) * Math.cos(angle2),
                        center.getInhomZ() + radius * Math.sin(angle2),
                        1.0);
                directorVector[0] = point.getInhomX() - center.getInhomX();
                directorVector[1] = point.getInhomY() - center.getInhomY();
                directorVector[2] = point.getInhomZ() - center.getInhomZ();

                final var plane = new Plane(point, directorVector);

                Plane planeWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorA = errorRandomizer.nextDouble();
                    final var errorB = errorRandomizer.nextDouble();
                    final var errorC = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC, plane.getD());
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

            final var estimator = new MSACDualQuadricRobustEstimator(this, planesWithError);

            estimator.setThreshold(THRESHOLD);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var dualQuadric2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by checking that all planes
            // are within the estimated conic locus
            for (final var p : planes) {
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
    public void onEstimateNextIteration(final DualQuadricRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        checkLocked((MSACDualQuadricRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(final DualQuadricRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        checkLocked((MSACDualQuadricRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private static void checkLocked(final MSACDualQuadricRobustEstimator estimator) {
        assertThrows(LockedException.class, () -> estimator.setThreshold(0.5));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setProgressDelta(0.5f));
        assertThrows(LockedException.class, () -> estimator.setConfidence(0.5));
        assertThrows(LockedException.class, () -> estimator.setMaxIterations(5));
        assertThrows(LockedException.class, () -> estimator.setPlanes(null));
        assertThrows(LockedException.class, estimator::estimate);
        assertTrue(estimator.isLocked());
    }
}
