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

import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class MSACLine2DRobustEstimatorTest implements
        Line2DRobustEstimatorListener {

    private static final double MIN_RANDOM_VALUE = -1.0;
    private static final double MAX_RANDOM_VALUE = 1.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int MIN_POINTS = 500;
    private static final int MAX_POINTS = 1000;

    private static final double THRESHOLD = 1.0;

    private static final double STD_ERROR = 100.0;

    private static final int PERCENTAGE_OUTLIER = 20;

    private static final int TIMES = 10;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    public void testConstants() {
        assertEquals(1, MSACLine2DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(0.0, MSACLine2DRobustEstimator.MIN_THRESHOLD, 0.0);
    }

    @Test
    public void testConstructor() {
        // test constructor without arguments
        MSACLine2DRobustEstimator estimator = new MSACLine2DRobustEstimator();

        // check correctness
        assertEquals(estimator.getThreshold(),
                MSACLine2DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Line2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Line2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Line2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // test constructor with points
        final List<Point2D> points = new ArrayList<>();
        for (int i = 0; i < Line2DRobustEstimator.MINIMUM_SIZE; i++) {
            points.add(Point2D.create());
        }

        estimator = new MSACLine2DRobustEstimator(points);

        // check correctness
        assertEquals(estimator.getThreshold(),
                MSACLine2DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Line2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Line2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Line2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        final List<Point2D> emptyPoints = new ArrayList<>();
        estimator = null;
        try {
            estimator = new MSACLine2DRobustEstimator(emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener
        final Line2DRobustEstimatorListener listener =
                new Line2DRobustEstimatorListener() {

                    @Override
                    public void onEstimateStart(final Line2DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateEnd(final Line2DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final Line2DRobustEstimator estimator, final int iteration) {
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final Line2DRobustEstimator estimator, final float progress) {
                    }
                };

        estimator = new MSACLine2DRobustEstimator(listener);

        // check correctness
        assertEquals(estimator.getThreshold(),
                MSACLine2DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Line2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Line2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Line2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // test constructor with listener and points
        estimator = new MSACLine2DRobustEstimator(listener, points);

        // check correctness
        assertEquals(estimator.getThreshold(),
                MSACLine2DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Line2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Line2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Line2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new MSACLine2DRobustEstimator(listener, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetThreshold() throws LockedException {
        final MSACLine2DRobustEstimator estimator =
                new MSACLine2DRobustEstimator();

        // check default value
        assertEquals(estimator.getThreshold(),
                MSACLine2DRobustEstimator.DEFAULT_THRESHOLD, 0.0);

        // set new value
        estimator.setThreshold(0.5);

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
        final MSACLine2DRobustEstimator estimator =
                new MSACLine2DRobustEstimator();

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
        final MSACLine2DRobustEstimator estimator =
                new MSACLine2DRobustEstimator();

        // check default value
        assertEquals(estimator.getProgressDelta(),
                Line2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);

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
        final MSACLine2DRobustEstimator estimator =
                new MSACLine2DRobustEstimator();

        // check default value
        assertEquals(estimator.getConfidence(),
                Line2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);

        // set new value
        estimator.setConfidence(0.5f);

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
        final MSACLine2DRobustEstimator estimator =
                new MSACLine2DRobustEstimator();

        // check default value
        assertEquals(estimator.getMaxIterations(),
                Line2DRobustEstimator.DEFAULT_MAX_ITERATIONS);

        // set new value
        estimator.setMaxIterations(1);

        // check correctness
        assertEquals(estimator.getMaxIterations(), 1);

        // Fail IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetPoints() throws LockedException {
        final MSACLine2DRobustEstimator estimator =
                new MSACLine2DRobustEstimator();

        // check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());

        // set new value
        final List<Point2D> points = new ArrayList<>();
        for (int i = 0; i < Line2DRobustEstimator.MINIMUM_SIZE; i++) {
            points.add(Point2D.create());
        }
        estimator.setPoints(points);

        // check correctness
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());

        // clearing list makes instance not ready
        points.clear();

        assertFalse(estimator.isReady());

        // Force IllegalArgumentException
        final List<Point2D> emptyPoints = new ArrayList<>();
        try {
            estimator.setPoints(emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final MSACLine2DRobustEstimator estimator =
                new MSACLine2DRobustEstimator();

        assertNull(estimator.getQualityScores());

        final double[] qualityScores = new double[
                Line2DRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertNull(estimator.getQualityScores());
    }

    @Test
    public void testEstimate() throws LockedException, NotReadyException,
            RobustEstimatorException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final double a = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final double b = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final double c = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final Line2D line = new Line2D(a, b, c);

            // compute random points passing through the line
            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            final List<Point2D> points = new ArrayList<>();
            final List<Point2D> pointsWithError = new ArrayList<>();
            Point2D point, pointWithError;
            for (int i = 0; i < nPoints; i++) {
                // get a random point belonging to the line (a*x + b*y + c*w = 0)
                // y = -(a*x + c*w)/b or x = -(b*y - c*w)/a
                final double homX;
                final double homY;
                final double homW = randomizer.nextDouble(MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                if (Math.abs(b) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homW) / b;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homW) / a;
                }
                point = new HomogeneousPoint2D(homX, homY, homW);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    pointWithError = new HomogeneousPoint2D(
                            point.getHomX() + errorX * point.getHomW(),
                            point.getHomY() + errorY * point.getHomW(),
                            point.getHomW());
                } else {
                    // inlier point
                    pointWithError = point;
                }

                points.add(point);
                pointsWithError.add(pointWithError);

                // check that point without error is locus of line
                assertTrue(line.isLocus(point, ABSOLUTE_ERROR));
            }

            final MSACLine2DRobustEstimator estimator =
                    new MSACLine2DRobustEstimator(this, pointsWithError);

            estimator.setThreshold(THRESHOLD);

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final Line2D line2 = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by checking that all points without
            // error have estimated line as locus
            for (final Point2D p : points) {
                assertTrue(line2.isLocus(p, 5.0 * ABSOLUTE_ERROR));
            }

            // check that both lines are equal
            line.normalize();
            line2.normalize();
            assertTrue(line.equals(line2, ABSOLUTE_ERROR));
        }
    }

    @Override
    public void onEstimateStart(final Line2DRobustEstimator estimator) {
        estimateStart++;
        checkLocked((MSACLine2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final Line2DRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((MSACLine2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(final Line2DRobustEstimator estimator,
                                        final int iteration) {
        estimateNextIteration++;
        checkLocked((MSACLine2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(final Line2DRobustEstimator estimator,
                                         final float progress) {
        estimateProgressChange++;
        checkLocked((MSACLine2DRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void checkLocked(final MSACLine2DRobustEstimator estimator) {
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
            estimator.setPoints(null);
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
