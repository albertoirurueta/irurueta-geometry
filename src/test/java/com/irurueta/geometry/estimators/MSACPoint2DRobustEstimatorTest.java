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

import com.irurueta.geometry.CoordinatesType;
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

class MSACPoint2DRobustEstimatorTest implements Point2DRobustEstimatorListener {

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double ABSOLUTE_ERROR = 1e-5;

    private static final int MIN_LINES = 500;
    private static final int MAX_LINES = 1000;

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
        assertEquals(1.0, MSACPoint2DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(0.0, MSACPoint2DRobustEstimator.MIN_THRESHOLD, 0.0);
    }

    @Test
    void testConstructor() {
        // test constructor without arguments
        var estimator = new MSACPoint2DRobustEstimator();

        // check correctness
        assertEquals(RANSACPoint2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(Point2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(Point3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES, estimator.getRefinementCoordinatesType());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test constructor with lines
        final var lines = new ArrayList<Line2D>();
        for (var i = 0; i < Point2DRobustEstimator.MINIMUM_SIZE; i++) {
            lines.add(new Line2D());
        }

        estimator = new MSACPoint2DRobustEstimator(lines);

        // check correctness
        assertEquals(MSACPoint2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(Point2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(lines, estimator.getLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(Point3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES, estimator.getRefinementCoordinatesType());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final var emptyLines = new ArrayList<Line2D>();
        assertThrows(IllegalArgumentException.class, () -> new MSACPoint2DRobustEstimator(emptyLines));

        // test constructor with listener
        final var listener = new Point2DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(final Point2DRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateEnd(final Point2DRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateNextIteration(final Point2DRobustEstimator estimator, final int iteration) {
                // no action needed
            }

            @Override
            public void onEstimateProgressChange(final Point2DRobustEstimator estimator, final float progress) {
                // no action needed
            }
        };

        estimator = new MSACPoint2DRobustEstimator(listener);

        // check correctness
        assertEquals(MSACPoint2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(Point2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(Point3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES, estimator.getRefinementCoordinatesType());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test constructor with listener and lines
        estimator = new MSACPoint2DRobustEstimator(listener, lines);

        // check correctness
        assertEquals(RANSACPoint2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(Point2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(lines, estimator.getLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(Point3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES, estimator.getRefinementCoordinatesType());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new MSACPoint2DRobustEstimator(listener, emptyLines));
    }

    @Test
    void testGetSetThreshold() throws LockedException {
        final var estimator = new MSACPoint2DRobustEstimator();

        // check default value
        assertEquals(MSACPoint2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);

        // set new value
        estimator.setThreshold(0.5);

        assertEquals(0.5, estimator.getThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setThreshold(0.0));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new MSACPoint2DRobustEstimator();

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
        final var estimator = new MSACPoint2DRobustEstimator();

        // check default value
        assertEquals(Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);

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
        final var estimator = new MSACPoint2DRobustEstimator();

        // check default value
        assertEquals(Point2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);

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
        final var estimator = new MSACPoint2DRobustEstimator();

        // check default value
        assertEquals(Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(1);

        // check correctness
        assertEquals(1, estimator.getMaxIterations());

        // Fail IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setMaxIterations(0));
    }

    @Test
    void testGetSetLines() throws LockedException {
        final var estimator = new MSACPoint2DRobustEstimator();

        // check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());

        // set new value
        final var lines = new ArrayList<Line2D>();
        for (var i = 0; i < Point2DRobustEstimator.MINIMUM_SIZE; i++) {
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
        final var estimator = new MSACPoint2DRobustEstimator();

        assertNull(estimator.getQualityScores());

        final var qualityScores = new double[Point2DRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertNull(estimator.getQualityScores());
    }

    @Test
    void testIsSetResultRefined() throws LockedException {
        final var estimator = new MSACPoint2DRobustEstimator();

        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check correctness
        assertFalse(estimator.isResultRefined());
    }

    @Test
    void testGetSetRefinementCoordinatesType() throws LockedException {
        final var estimator = new MSACPoint2DRobustEstimator();

        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES, estimator.getRefinementCoordinatesType());

        // set new value
        estimator.setRefinementCoordinatesType(CoordinatesType.HOMOGENEOUS_COORDINATES);

        // check correctness
        assertEquals(CoordinatesType.HOMOGENEOUS_COORDINATES, estimator.getRefinementCoordinatesType());
    }

    @Test
    void testIsSetCovarianceKept() throws LockedException {
        final var estimator = new MSACPoint2DRobustEstimator();

        assertFalse(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(true);

        // check correctness
        assertTrue(estimator.isCovarianceKept());
    }

    @Test
    void testEstimateWithoutRefinement() throws LockedException, NotReadyException, RobustEstimatorException {
        final var randomizer = new UniformRandomizer();

        for (var t = 0; t < TIMES; t++) {
            final var point = new HomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    1.0);

            // compute random lines passing through the point
            final var nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);
            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            final var lines = new ArrayList<Line2D>();
            final var linesWithError = new ArrayList<Line2D>();
            for (var i = 0; i < nLines; i++) {
                // get another point (far enough to compute a line)
                Point2D anotherPoint;
                do {
                    anotherPoint = new HomogeneousPoint2D(
                            randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                            randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 1.0);
                } while (anotherPoint.distanceTo(point) < STD_ERROR);

                final var line = new Line2D(point, anotherPoint);

                Line2D lineWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // line is outlier
                    final var errorA = errorRandomizer.nextDouble();
                    final var errorB = errorRandomizer.nextDouble();
                    final var errorC = errorRandomizer.nextDouble();
                    lineWithError = new Line2D(line.getA() + errorA, line.getB() + errorB,
                            line.getC() + errorC);
                } else {
                    // inlier line
                    lineWithError = line;
                }

                lines.add(line);
                linesWithError.add(lineWithError);

                // check that point is locus of line without error
                assertTrue(line.isLocus(point, ABSOLUTE_ERROR));
            }

            final var estimator = new MSACPoint2DRobustEstimator(this, linesWithError);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var point2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by checking that all lines without
            // error have estimated point as locus
            for (final var l : lines) {
                assertTrue(l.isLocus(point2, ABSOLUTE_ERROR));
            }

            // check that both points are equal
            assertEquals(0.0, point.distanceTo(point2), ABSOLUTE_ERROR);
        }
    }

    @Test
    void testEstimateWithInhomogeneousRefinement() throws LockedException, NotReadyException, RobustEstimatorException {

        final var randomizer = new UniformRandomizer();

        for (var t = 0; t < TIMES; t++) {
            final var point = new HomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    1.0);

            // compute random lines passing through the point
            final var nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);
            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            final var lines = new ArrayList<Line2D>();
            final var linesWithError = new ArrayList<Line2D>();
            for (var i = 0; i < nLines; i++) {
                // get another point (far enough to compute a line)
                Point2D anotherPoint;
                do {
                    anotherPoint = new HomogeneousPoint2D(
                            randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                            randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 1.0);
                } while (anotherPoint.distanceTo(point) < STD_ERROR);

                final var line = new Line2D(point, anotherPoint);

                Line2D lineWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // line is outlier
                    final var errorA = errorRandomizer.nextDouble();
                    final var errorB = errorRandomizer.nextDouble();
                    final var errorC = errorRandomizer.nextDouble();
                    lineWithError = new Line2D(line.getA() + errorA, line.getB() + errorB,
                            line.getC() + errorC);
                } else {
                    // inlier line
                    lineWithError = line;
                }

                lines.add(line);
                linesWithError.add(lineWithError);

                // check that point is locus of line without error
                assertTrue(line.isLocus(point, ABSOLUTE_ERROR));
            }

            final var estimator = new MSACPoint2DRobustEstimator(this, linesWithError);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            estimator.setRefinementCoordinatesType(CoordinatesType.INHOMOGENEOUS_COORDINATES);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var point2 = estimator.estimate();

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNotNull(estimator.getCovariance());
            assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getCovariance().getRows());
            assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getCovariance().getColumns());

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by checking that all lines without
            // error have estimated point as locus
            for (final var l : lines) {
                assertTrue(l.isLocus(point2, ABSOLUTE_ERROR));
            }

            // check that both points are equal
            assertEquals(0.0, point.distanceTo(point2), ABSOLUTE_ERROR);
        }
    }

    @Test
    void testEstimateWithHomogeneousRefinement() throws LockedException, NotReadyException, RobustEstimatorException {

        final var randomizer = new UniformRandomizer();

        for (int t = 0; t < TIMES; t++) {
            final var point = new HomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    1.0);

            // compute random lines passing through the point
            final var nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);
            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            final var lines = new ArrayList<Line2D>();
            final var linesWithError = new ArrayList<Line2D>();
            for (var i = 0; i < nLines; i++) {
                // get another point (far enough to compute a line)
                Point2D anotherPoint;
                do {
                    anotherPoint = new HomogeneousPoint2D(
                            randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                            randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 1.0);
                } while (anotherPoint.distanceTo(point) < STD_ERROR);

                final var line = new Line2D(point, anotherPoint);

                Line2D lineWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // line is outlier
                    final var errorA = errorRandomizer.nextDouble();
                    final var errorB = errorRandomizer.nextDouble();
                    final var errorC = errorRandomizer.nextDouble();
                    lineWithError = new Line2D(line.getA() + errorA, line.getB() + errorB,
                            line.getC() + errorC);
                } else {
                    // inlier line
                    lineWithError = line;
                }

                lines.add(line);
                linesWithError.add(lineWithError);

                // check that point is locus of line without error
                assertTrue(line.isLocus(point, ABSOLUTE_ERROR));
            }

            final var estimator = new MSACPoint2DRobustEstimator(this, linesWithError);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            estimator.setRefinementCoordinatesType(CoordinatesType.HOMOGENEOUS_COORDINATES);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var point2 = estimator.estimate();

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNotNull(estimator.getCovariance());
            assertEquals(Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, estimator.getCovariance().getRows());
            assertEquals(Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, estimator.getCovariance().getColumns());

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by checking that all lines without
            // error have estimated point as locus
            for (final var l : lines) {
                assertTrue(l.isLocus(point2, ABSOLUTE_ERROR));
            }

            // check that both points are equal
            assertEquals(0.0, point.distanceTo(point2), ABSOLUTE_ERROR);
        }
    }

    @Override
    public void onEstimateStart(final Point2DRobustEstimator estimator) {
        estimateStart++;
        checkLocked((MSACPoint2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final Point2DRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((MSACPoint2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(final Point2DRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        checkLocked((MSACPoint2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(final Point2DRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        checkLocked((MSACPoint2DRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = estimateProgressChange = 0;
    }

    private static void checkLocked(final MSACPoint2DRobustEstimator estimator) {
        assertThrows(LockedException.class, () -> estimator.setThreshold(0.5));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setProgressDelta(0.5f));
        assertThrows(LockedException.class, () -> estimator.setConfidence(0.5));
        assertThrows(LockedException.class, () -> estimator.setMaxIterations(5));
        assertThrows(LockedException.class, () -> estimator.setLines(null));
        assertThrows(LockedException.class, estimator::estimate);
    }
}
