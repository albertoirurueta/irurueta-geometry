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

import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class MSACPlaneRobustEstimatorTest implements PlaneRobustEstimatorListener {

    private static final double MIN_RANDOM_VALUE = -1.0;
    private static final double MAX_RANDOM_VALUE = 1.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int MIN_POINTS = 500;
    private static final int MAX_POINTS = 1000;

    private static final double THRESHOLD = 1e-6;

    private static final double STD_ERROR = 100.0;

    private static final int PERCENTAGE_OUTLIER = 20;

    private static final int TIMES = 10;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    void testConstants() {
        assertEquals(1.0, MSACPlaneRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(0.0, MSACPlaneRobustEstimator.MIN_THRESHOLD, 0.0);
    }

    @Test
    void testConstructor() {
        // test constructor without arguments
        var estimator = new MSACPlaneRobustEstimator();

        // check correctness
        assertEquals(RANSACPlaneRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PlaneRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PlaneRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PlaneRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // test constructor with points
        final var points = new ArrayList<Point3D>();
        for (var i = 0; i < PlaneRobustEstimator.MINIMUM_SIZE; i++) {
            points.add(Point3D.create());
        }

        estimator = new MSACPlaneRobustEstimator(points);

        // check correctness
        assertEquals(MSACPlaneRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PlaneRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PlaneRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PlaneRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(points, estimator.getPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point3D>();
        assertThrows(IllegalArgumentException.class, () -> new MSACPlaneRobustEstimator(emptyPoints));

        // test constructor with listener
        final var listener = new PlaneRobustEstimatorListener() {

            @Override
            public void onEstimateStart(final PlaneRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateEnd(final PlaneRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateNextIteration(final PlaneRobustEstimator estimator, final int iteration) {
                // no action needed
            }

            @Override
            public void onEstimateProgressChange(final PlaneRobustEstimator estimator, final float progress) {
                // no action needed
            }
        };

        estimator = new MSACPlaneRobustEstimator(listener);

        // check correctness
        assertEquals(MSACPlaneRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PlaneRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PlaneRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PlaneRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // test constructor with listener and points
        estimator = new MSACPlaneRobustEstimator(listener, points);

        // check correctness
        assertEquals(MSACPlaneRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PlaneRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PlaneRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PlaneRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(points, estimator.getPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new MSACPlaneRobustEstimator(listener, emptyPoints));
    }

    @Test
    void testGetSetThreshold() throws LockedException {
        final var estimator = new MSACPlaneRobustEstimator();

        // check default value
        assertEquals(MSACPlaneRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);

        // set new value
        estimator.setThreshold(0.5);

        assertEquals(0.5, estimator.getThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setThreshold(0.0));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new MSACPlaneRobustEstimator();

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
        final var estimator = new MSACPlaneRobustEstimator();

        // check default value
        assertEquals(PlaneRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);

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
        final var estimator = new MSACPlaneRobustEstimator();

        // check default value
        assertEquals(PlaneRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);

        // set new value
        estimator.setConfidence(0.5f);

        // check correctness
        assertEquals(0.5, estimator.getConfidence(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setConfidence(-1.0));
        assertThrows(IllegalArgumentException.class, () -> estimator.setConfidence(2.0));
    }

    @Test
    void testGetSetMaxIterations() throws LockedException {
        final var estimator = new MSACPlaneRobustEstimator();

        // check default value
        assertEquals(PlaneRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(1);

        // check correctness
        assertEquals(1, estimator.getMaxIterations());

        // Fail IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setMaxIterations(0));
    }

    @Test
    void testGetSetPoints() throws LockedException {
        final var estimator = new MSACPlaneRobustEstimator();

        // check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());

        // set new value
        final var points = new ArrayList<Point3D>();
        for (var i = 0; i < PlaneRobustEstimator.MINIMUM_SIZE; i++) {
            points.add(Point3D.create());
        }
        estimator.setPoints(points);

        // check correctness
        assertSame(points, estimator.getPoints());
        assertTrue(estimator.isReady());

        // clearing list makes instance not ready
        points.clear();

        assertFalse(estimator.isReady());

        // Force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point3D>();
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(emptyPoints));
    }

    @Test
    void testGetSetQualityScores() throws LockedException {
        final var estimator = new MSACPlaneRobustEstimator();

        assertNull(estimator.getQualityScores());

        final var qualityScores = new double[PlaneRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertNull(estimator.getQualityScores());
    }

    @Test
    void testEstimate() throws LockedException, NotReadyException, RobustEstimatorException {

        final var randomizer = new UniformRandomizer();

        for (var t = 0; t < TIMES; t++) {
            final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var plane = new Plane(a, b, c, d);

            // compute random points passing through the line
            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            final var points = new ArrayList<Point3D>();
            final var pointsWithError = new ArrayList<Point3D>();
            for (var i = 0; i < nPoints; i++) {
                // get a random point belonging to the plane
                // (a*x + b*y + c*z + d*w = 0)
                // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                final double homX;
                final double homY;
                final var homW = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                if (Math.abs(b) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }
                final var point = new HomogeneousPoint3D(homX, homY, homZ, homW);

                Point3D pointWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var errorZ = errorRandomizer.nextDouble();
                    pointWithError = new HomogeneousPoint3D(
                            point.getHomX() + errorX * point.getHomW(),
                            point.getHomY() + errorY * point.getHomW(),
                            point.getHomZ() + errorZ * point.getHomW(),
                            point.getHomW());
                } else {
                    // inlier point
                    pointWithError = point;
                }

                points.add(point);
                pointsWithError.add(pointWithError);

                // check that point without error is locus of line
                assertTrue(plane.isLocus(point, ABSOLUTE_ERROR));
            }

            final var estimator = new MSACPlaneRobustEstimator(this, pointsWithError);

            estimator.setThreshold(THRESHOLD);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var plane2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by checking that all points without
            // error have estimated line as locus
            for (final var p : points) {
                assertTrue(plane2.isLocus(p, ABSOLUTE_ERROR));
            }

            // check that both lines are equal
            plane.normalize();
            plane2.normalize();
            assertTrue(plane.equals(plane2, ABSOLUTE_ERROR));
        }
    }

    @Override
    public void onEstimateStart(final PlaneRobustEstimator estimator) {
        estimateStart++;
        checkLocked((MSACPlaneRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final PlaneRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((MSACPlaneRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(final PlaneRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        checkLocked((MSACPlaneRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(final PlaneRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        checkLocked((MSACPlaneRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = estimateProgressChange = 0;
    }

    private static void checkLocked(final MSACPlaneRobustEstimator estimator) {
        assertThrows(LockedException.class, () -> estimator.setThreshold(0.5));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setProgressDelta(0.5f));
        assertThrows(LockedException.class, () -> estimator.setConfidence(0.5));
        assertThrows(LockedException.class, () -> estimator.setMaxIterations(5));
        assertThrows(LockedException.class, () -> estimator.setPoints(null));
        assertThrows(LockedException.class, estimator::estimate);
        assertTrue(estimator.isLocked());
    }
}
