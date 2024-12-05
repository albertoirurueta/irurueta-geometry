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

import com.irurueta.geometry.Circle;
import com.irurueta.geometry.DualConicNotAvailableException;
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

class LMedSDualConicRobustEstimatorTest implements DualConicRobustEstimatorListener {

    private static final int MIN_LINES = 500;
    private static final int MAX_LINES = 1000;

    private static final double MIN_RANDOM_POINT_VALUE = -1.0;
    private static final double MAX_RANDOM_POINT_VALUE = 1.0;

    private static final double STD_ERROR = 1.0;
    private static final int PERCENTAGE_OUTLIER = 20;

    private static final double STOP_THRESHOLD = 1e-9;
    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int TIMES = 100;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    void testConstants() {
        assertEquals(1e-9, LMedSDualConicRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(0.0, LMedSDualConicRobustEstimator.MIN_STOP_THRESHOLD, 0.0);
    }

    @Test
    void testConstructor() {
        // test constructor without arguments
        var estimator = new LMedSDualConicRobustEstimator();

        // check correctness
        assertEquals(LMedSDualConicRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualConicRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(DualConicRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(DualConicRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // test constructor with lines
        final var lines = new ArrayList<Line2D>();
        for (var i = 0; i < DualConicRobustEstimator.MINIMUM_SIZE; i++) {
            lines.add(new Line2D());
        }

        estimator = new LMedSDualConicRobustEstimator(lines);

        // check correctness
        assertEquals(LMedSDualConicRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualConicRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(DualConicRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(DualConicRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(lines, estimator.getLines());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        final var emptyLines = new ArrayList<Line2D>();
        assertThrows(IllegalArgumentException.class, () -> new LMedSDualConicRobustEstimator(emptyLines));

        // test constructor with listener
        final var listener = new DualConicRobustEstimatorListener() {

            @Override
            public void onEstimateStart(final DualConicRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateEnd(final DualConicRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateNextIteration(final DualConicRobustEstimator estimator, final int iteration) {
                // no action needed
            }

            @Override
            public void onEstimateProgressChange(final DualConicRobustEstimator estimator, final float progress) {
                // no action needed
            }
        };

        estimator = new LMedSDualConicRobustEstimator(listener);

        // check correctness
        assertEquals(LMedSDualConicRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualConicRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(DualConicRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(DualConicRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // test constructor with listener and points
        estimator = new LMedSDualConicRobustEstimator(listener, lines);

        // check correctness
        assertEquals(LMedSDualConicRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualConicRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(DualConicRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(DualConicRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(lines, estimator.getLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new LMedSDualConicRobustEstimator(listener, emptyLines));
    }

    @Test
    void testGetSetStopThreshold() throws LockedException {
        final var estimator = new LMedSDualConicRobustEstimator();

        // check default value
        assertEquals(LMedSDualConicRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(), 0.0);

        // set new value
        estimator.setStopThreshold(0.5);

        // check correctness
        assertEquals(0.5, estimator.getStopThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setStopThreshold(0.0));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new LMedSDualConicRobustEstimator();

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
        final var estimator = new LMedSDualConicRobustEstimator();

        // check default value
        assertEquals(DualConicRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);

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
        final var estimator = new LMedSDualConicRobustEstimator();

        // check default value
        assertEquals(DualConicRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);

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
        final var estimator = new LMedSDualConicRobustEstimator();

        // check default value
        assertEquals(DualConicRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(1);

        // check correctness
        assertEquals(1, estimator.getMaxIterations());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setMaxIterations(0));
    }

    @Test
    void testGetSetLines() throws LockedException {
        final var estimator = new LMedSDualConicRobustEstimator();

        // check default value
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());

        // set new value
        final var lines = new ArrayList<Line2D>();
        for (var i = 0; i < DualConicRobustEstimator.MINIMUM_SIZE; i++) {
            lines.add(new Line2D());
        }
        estimator.setLines(lines);

        // check correctness
        assertSame(lines, estimator.getLines());
        assertTrue(estimator.isReady());

        // clearing list makes instance not ready
        lines.clear();

        assertFalse(estimator.isReady());

        // Force IllegalArgumentException
        final var emptyLines = new ArrayList<Line2D>();
        assertThrows(IllegalArgumentException.class, () -> estimator.setLines(emptyLines));
    }

    @Test
    void testGetSetQualityScores() throws LockedException {
        final var estimator = new LMedSDualConicRobustEstimator();

        assertNull(estimator.getQualityScores());

        final var qualityScores = new double[DualConicRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertNull(estimator.getQualityScores());
    }

    @Test
    void testEstimate() throws LockedException, NotReadyException, RobustEstimatorException,
            DualConicNotAvailableException {

        final var randomizer = new UniformRandomizer();

        for (var t = 0; t < TIMES; t++) {
            // instantiate a random circle
            final var center = new HomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE), 1.0);
            final var radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_POINT_VALUE / 2.0, MAX_RANDOM_POINT_VALUE));

            final var circle = new Circle(center, radius);
            final var conic = circle.toConic();
            final var dualConic = conic.getDualConic();

            // compute lines in the dual conic locus
            final var nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);
            final var theta = Math.toRadians((double) nLines / 360.0);
            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            final var lines = new ArrayList<Line2D>();
            final var linesWithError = new ArrayList<Line2D>();
            final var directorVector = new double[Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            for (var i = 0; i < nLines; i++) {
                final var angle = theta * (double) i;
                final var point = new HomogeneousPoint2D(
                        center.getInhomX() + radius * Math.cos(angle),
                        center.getInhomY() + radius * Math.sin(angle), 1.0);
                directorVector[0] = point.getInhomX() - center.getInhomX();
                directorVector[1] = point.getInhomY() - center.getInhomY();

                final var line = new Line2D(point, directorVector);
                Line2D lineWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorA = errorRandomizer.nextDouble();
                    final var errorB = errorRandomizer.nextDouble();
                    lineWithError = new Line2D(line.getA() + errorA, line.getB() + errorB, line.getC());
                } else {
                    // inlier point
                    lineWithError = line;
                }

                lines.add(line);
                linesWithError.add(lineWithError);

                // check that point without error is within conic locus
                assertTrue(circle.isLocus(point, ABSOLUTE_ERROR));
                assertTrue(conic.isLocus(point, ABSOLUTE_ERROR));
                assertTrue(dualConic.isLocus(line, ABSOLUTE_ERROR));
            }

            final var estimator = new LMedSDualConicRobustEstimator(this, linesWithError);

            estimator.setStopThreshold(STOP_THRESHOLD);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var dualConic2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by checking that all points
            // are within the estimated conic locus
            for (final var p : lines) {
                assertTrue(dualConic2.isLocus(p, ABSOLUTE_ERROR));
            }
        }
    }

    @Override
    public void onEstimateStart(final DualConicRobustEstimator estimator) {
        estimateStart++;
        checkLocked((LMedSDualConicRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final DualConicRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((LMedSDualConicRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(final DualConicRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        checkLocked((LMedSDualConicRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(final DualConicRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        checkLocked((LMedSDualConicRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = estimateProgressChange = 0;
    }

    private static void checkLocked(final LMedSDualConicRobustEstimator estimator) {
        assertThrows(LockedException.class, () -> estimator.setStopThreshold(0.5));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setProgressDelta(0.5f));
        assertThrows(LockedException.class, () -> estimator.setConfidence(0.5));
        assertThrows(LockedException.class, () -> estimator.setMaxIterations(5));
        assertThrows(LockedException.class, () -> estimator.setLines(null));
        assertThrows(LockedException.class, estimator::estimate);
    }
}
