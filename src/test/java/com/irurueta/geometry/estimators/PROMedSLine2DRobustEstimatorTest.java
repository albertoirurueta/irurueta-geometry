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
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class PROMedSLine2DRobustEstimatorTest implements Line2DRobustEstimatorListener {

    private static final double MIN_RANDOM_VALUE = -1.0;
    private static final double MAX_RANDOM_VALUE = 1.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int MIN_POINTS = 500;
    private static final int MAX_POINTS = 1000;

    private static final double STOP_THRESHOLD = 1.0;

    private static final double MIN_SCORE_ERROR = -0.3;
    private static final double MAX_SCORE_ERROR = 0.3;

    private static final double STD_ERROR = 100.0;

    private static final int PERCENTAGE_OUTLIER = 20;

    private static final int TIMES = 10;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    void testConstants() {
        assertEquals(2, Line2DRobustEstimator.MINIMUM_SIZE);
        assertEquals(0.05f, Line2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0f);
        assertEquals(0.0f, Line2DRobustEstimator.MIN_PROGRESS_DELTA, 0.0f);
        assertEquals(1.0f, Line2DRobustEstimator.MAX_PROGRESS_DELTA, 0.0f);
        assertEquals(0.99, Line2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(5000, Line2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(0.0, Line2DRobustEstimator.MIN_CONFIDENCE, 0.0);
        assertEquals(1.0, Line2DRobustEstimator.MAX_CONFIDENCE, 0.0);
        assertEquals(1, Line2DRobustEstimator.MIN_ITERATIONS);
        assertEquals(RobustEstimatorMethod.PROMEDS, Line2DRobustEstimator.DEFAULT_ROBUST_METHOD);
        assertEquals(1e-3, PROMedSLine2DRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(0.0, PROMedSLine2DRobustEstimator.MIN_STOP_THRESHOLD, 0.0);
    }

    @Test
    void testConstructor() {
        // test constructor without arguments
        var estimator = new PROMedSLine2DRobustEstimator();

        // check correctness
        assertEquals(PROMedSLine2DRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(Line2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(Line2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(Line2DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // test constructor with points
        final var points = new ArrayList<Point2D>();
        for (var i = 0; i < Line2DRobustEstimator.MINIMUM_SIZE; i++) {
            points.add(Point2D.create());
        }

        estimator = new PROMedSLine2DRobustEstimator(points);

        // check correctness
        assertEquals(PROMedSLine2DRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(Line2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(Line2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(Line2DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(points, estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point2D>();
        assertThrows(IllegalArgumentException.class, () -> new PROMedSLine2DRobustEstimator(emptyPoints));

        // test constructor with listener
        final var listener = new Line2DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(final Line2DRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateEnd(final Line2DRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateNextIteration(final Line2DRobustEstimator estimator, final int iteration) {
                // no action needed
            }

            @Override
            public void onEstimateProgressChange(final Line2DRobustEstimator estimator, final float progress) {
                // no action needed
            }
        };

        estimator = new PROMedSLine2DRobustEstimator(listener);

        // check correctness
        assertEquals(PROMedSLine2DRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(Line2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(Line2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(Line2DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // test constructor with listener and points
        estimator = new PROMedSLine2DRobustEstimator(listener, points);

        // check correctness
        assertEquals(PROMedSLine2DRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(Line2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(Line2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(Line2DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(points, estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSLine2DRobustEstimator(listener, emptyPoints));

        // test constructor with quality scores
        final var qualityScores = new double[Line2DRobustEstimator.MINIMUM_SIZE];
        estimator = new PROMedSLine2DRobustEstimator(qualityScores);

        // check correctness
        assertEquals(PROMedSLine2DRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(Line2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(Line2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(Line2DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        final var emptyScores = new double[0];
        assertThrows(IllegalArgumentException.class, () -> new PROMedSLine2DRobustEstimator(emptyScores));

        // test constructor with points and scores
        estimator = new PROMedSLine2DRobustEstimator(points, qualityScores);

        // check correctness
        assertEquals(PROMedSLine2DRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(Line2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(Line2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(Line2DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(points, estimator.getPoints());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSLine2DRobustEstimator(emptyPoints, qualityScores));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSLine2DRobustEstimator(points, emptyScores));

        // test constructor with listener and quality scores
        estimator = new PROMedSLine2DRobustEstimator(listener, qualityScores);

        // check correctness
        assertEquals(PROMedSLine2DRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(Line2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(Line2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(Line2DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSLine2DRobustEstimator(listener, emptyScores));

        // test constructor with listener, points and quality scores
        estimator = new PROMedSLine2DRobustEstimator(listener, points, qualityScores);

        // check correctness
        assertEquals(PROMedSLine2DRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(Line2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(Line2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(Line2DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(points, estimator.getPoints());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSLine2DRobustEstimator(listener, emptyPoints,
                qualityScores));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSLine2DRobustEstimator(listener, points,
                emptyScores));
    }

    @Test
    void testGetSetThreshold() throws LockedException {
        final var estimator = new PROMedSLine2DRobustEstimator();

        // check default value
        assertEquals(PROMedSLine2DRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(), 0.0);

        // set new value
        estimator.setStopThreshold(0.5);

        assertEquals(0.5, estimator.getStopThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setStopThreshold(0.0));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new PROMedSLine2DRobustEstimator();

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
        final var estimator = new PROMedSLine2DRobustEstimator();

        // check default value
        assertEquals(Line2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);

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
        final var estimator = new PROMedSLine2DRobustEstimator();

        // check default value
        assertEquals(Line2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);

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
        final var estimator = new PROMedSLine2DRobustEstimator();

        // check default value
        assertEquals(Line2DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(1);

        // check correctness
        assertEquals(1, estimator.getMaxIterations());

        // Fail IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setMaxIterations(0));
    }

    @Test
    void testGetSetPoints() throws LockedException {
        final var estimator = new PROMedSLine2DRobustEstimator();

        // check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());

        // set new value
        final var points = new ArrayList<Point2D>();
        for (var i = 0; i < Line2DRobustEstimator.MINIMUM_SIZE; i++) {
            points.add(Point2D.create());
        }
        estimator.setPoints(points);

        // check correctness
        assertSame(points, estimator.getPoints());
        assertFalse(estimator.isReady());

        // if we set quality scores, then estimator becomes ready
        final var qualityScores = new double[points.size()];
        estimator.setQualityScores(qualityScores);

        assertTrue(estimator.isReady());

        // clearing list makes instance not ready
        points.clear();

        assertFalse(estimator.isReady());

        // Force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point2D>();
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(emptyPoints));
    }

    @Test
    void testGetSetQualityScores() throws LockedException {
        final var estimator = new PROMedSLine2DRobustEstimator();

        assertNull(estimator.getQualityScores());

        final var qualityScores = new double[Line2DRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        final var qualityScores2 = new double[1];
        assertThrows(IllegalArgumentException.class, () -> estimator.setQualityScores(qualityScores2));
    }

    @Test
    void testEstimate() throws LockedException, NotReadyException, RobustEstimatorException {

        final var randomizer = new UniformRandomizer();

        for (var t = 0; t < TIMES; t++) {
            final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var line = new Line2D(a, b, c);

            // compute random points passing through the line
            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var qualityScores = new double[nPoints];
            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            final var points = new ArrayList<Point2D>();
            final var pointsWithError = new ArrayList<Point2D>();
            for (var i = 0; i < nPoints; i++) {
                final var scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;

                // get a random point belonging to the line (a*x + b*y + c*w = 0)
                // y = -(a*x + c*w)/b or x = -(b*y - c*w)/a
                final double homX;
                final double homY;
                final var homW = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                if (Math.abs(b) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homW) / b;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homW) / a;
                }
                final var point = new HomogeneousPoint2D(homX, homY, homW);

                Point2D pointWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    pointWithError = new HomogeneousPoint2D(
                            point.getHomX() + errorX * point.getHomW(),
                            point.getHomY() + errorY * point.getHomW(),
                            point.getHomW());

                    final var error = Math.sqrt(errorX * errorX + errorY * errorY);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    // inlier point
                    pointWithError = point;
                }

                points.add(point);
                pointsWithError.add(pointWithError);

                // check that point without error is locus of line
                assertTrue(line.isLocus(point, ABSOLUTE_ERROR));
            }

            final var estimator = new PROMedSLine2DRobustEstimator(this, pointsWithError, qualityScores);

            estimator.setStopThreshold(STOP_THRESHOLD);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var line2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by checking that all points without
            // error have estimated line as locus
            for (final var p : points) {
                assertTrue(line2.isLocus(p, ABSOLUTE_ERROR));
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
        checkLocked((PROMedSLine2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final Line2DRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((PROMedSLine2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(final Line2DRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        checkLocked((PROMedSLine2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(final Line2DRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        checkLocked((PROMedSLine2DRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = estimateProgressChange = 0;
    }

    private static void checkLocked(final PROMedSLine2DRobustEstimator estimator) {
        assertThrows(LockedException.class, () -> estimator.setStopThreshold(0.5));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setProgressDelta(0.5f));
        assertThrows(LockedException.class, () -> estimator.setConfidence(0.5));
        assertThrows(LockedException.class, () -> estimator.setMaxIterations(5));
        assertThrows(LockedException.class, () -> estimator.setPoints(null));
        assertThrows(LockedException.class, estimator::estimate);
        assertTrue(estimator.isLocked());
    }
}
