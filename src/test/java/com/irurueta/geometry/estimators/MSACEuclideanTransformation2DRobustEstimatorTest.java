/*
 * Copyright (C) 2017 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.geometry.EuclideanTransformation2D;
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Rotation2D;
import com.irurueta.geometry.Utils;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class MSACEuclideanTransformation2DRobustEstimatorTest implements EuclideanTransformation2DRobustEstimatorListener {

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_ANGLE_DEGREES = -90.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double MIN_TRANSLATION = -100.0;
    private static final double MAX_TRANSLATION = 100.0;

    private static final double THRESHOLD = 1.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final double STD_ERROR = 100.0;

    private static final int TIMES = 10;

    private static final int MIN_POINTS = 500;
    private static final int MAX_POINTS = 1000;

    private static final int PERCENTAGE_OUTLIER = 20;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    void testConstants() {
        assertEquals(1.0, MSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(0.0, MSACEuclideanTransformation2DRobustEstimator.MIN_THRESHOLD, 0.0);
    }

    @Test
    void testConstructor() {
        // test constructor without arguments
        var estimator = new MSACEuclideanTransformation2DRobustEstimator();

        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test constructor with points
        final var inputPoints = new ArrayList<Point2D>();
        final var outputPoints = new ArrayList<Point2D>();
        for (var i = 0; i < EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }

        estimator = new MSACEuclideanTransformation2DRobustEstimator(inputPoints, outputPoints);

        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final var pointsEmpty = new ArrayList<Point2D>();
        // not enough points
        assertThrows(IllegalArgumentException.class, () -> new MSACEuclideanTransformation2DRobustEstimator(pointsEmpty,
                pointsEmpty));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> new MSACEuclideanTransformation2DRobustEstimator(inputPoints,
                pointsEmpty));

        // test constructor with listener
        estimator = new MSACEuclideanTransformation2DRobustEstimator(this);

        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test constructor with listener and points
        estimator = new MSACEuclideanTransformation2DRobustEstimator(this, inputPoints, outputPoints);

        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        // not enough points
        assertThrows(IllegalArgumentException.class, () -> new MSACEuclideanTransformation2DRobustEstimator(
                this, pointsEmpty, pointsEmpty));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> new MSACEuclideanTransformation2DRobustEstimator(
                this, inputPoints, pointsEmpty));

        // test constructor without arguments and weak minimum points
        estimator = new MSACEuclideanTransformation2DRobustEstimator(true);

        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test constructor with points
        final var inputPoints2 = new ArrayList<Point2D>();
        final var outputPoints2 = new ArrayList<Point2D>();
        for (var i = 0; i < EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE; i++) {
            inputPoints2.add(Point2D.create());
            outputPoints2.add(Point2D.create());
        }

        estimator = new MSACEuclideanTransformation2DRobustEstimator(inputPoints2, outputPoints2,
                true);

        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertSame(inputPoints2, estimator.getInputPoints());
        assertSame(outputPoints2, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        // not enough points
        assertThrows(IllegalArgumentException.class, () -> new MSACEuclideanTransformation2DRobustEstimator(pointsEmpty,
                pointsEmpty, true));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> new MSACEuclideanTransformation2DRobustEstimator(
                inputPoints2, pointsEmpty, true));

        // test constructor with listener
        estimator = new MSACEuclideanTransformation2DRobustEstimator(this, true);

        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test constructor with listener and points
        estimator = new MSACEuclideanTransformation2DRobustEstimator(this, inputPoints2, outputPoints2,
                true);

        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertSame(inputPoints2, estimator.getInputPoints());
        assertSame(outputPoints2, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        // not enough points
        assertThrows(IllegalArgumentException.class, () -> new MSACEuclideanTransformation2DRobustEstimator(
                this, pointsEmpty, pointsEmpty));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> new MSACEuclideanTransformation2DRobustEstimator(
                this, inputPoints, pointsEmpty));
    }

    @Test
    void testGetSetThreshold() throws LockedException {
        final var estimator = new MSACEuclideanTransformation2DRobustEstimator();

        // check default value
        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);

        // set new value
        estimator.setThreshold(0.5);

        // check correctness
        assertEquals(0.5, estimator.getThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setThreshold(0.0));
    }

    @Test
    void testGetSetConfidence() throws LockedException {
        final var estimator = new MSACEuclideanTransformation2DRobustEstimator();

        // check default value
        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);

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
        final var estimator = new MSACEuclideanTransformation2DRobustEstimator();

        // check default value
        assertEquals(MSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(10);

        // check correctness
        assertEquals(10, estimator.getMaxIterations());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setMaxIterations(0));
    }

    @Test
    void testGetSetPointsAndIsReady() throws LockedException {
        final var estimator = new MSACEuclideanTransformation2DRobustEstimator();

        // check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());

        // set new value
        final var inputPoints = new ArrayList<Point2D>();
        final var outputPoints = new ArrayList<Point2D>();
        for (var i = 0; i < MSACEuclideanTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }

        estimator.setPoints(inputPoints, outputPoints);

        // check correctness
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());

        // Force IllegalArgumentException
        final var pointsEmpty = new ArrayList<Point2D>();
        // not enough points
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(pointsEmpty, pointsEmpty));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(pointsEmpty, pointsEmpty));
    }

    @Test
    void testGetSetListenerAndIsListenerAvailable() throws LockedException {
        final var estimator = new MSACEuclideanTransformation2DRobustEstimator();

        // check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
    }

    @Test
    void testIsSetWeakMinimumPointsAllowed() throws LockedException {
        final var estimator = new MSACEuclideanTransformation2DRobustEstimator();

        // check default value
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());

        // set new value
        estimator.setWeakMinimumSizeAllowed(true);

        // check correctness
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());
    }

    @Test
    void testGetSetProgressDelta() throws LockedException {
        final var estimator = new MSACEuclideanTransformation2DRobustEstimator();

        // check default value
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check correctness
        assertEquals(0.5f, estimator.getProgressDelta(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setProgressDelta(-1.0f));
        assertThrows(IllegalArgumentException.class, () -> estimator.setProgressDelta(2.0f));
    }

    @Test
    void testIsSetResultRefined() throws LockedException {
        final var estimator = new MSACEuclideanTransformation2DRobustEstimator();

        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check correctness
        assertFalse(estimator.isResultRefined());
    }

    @Test
    void testIsSetCovarianceKept() throws LockedException {
        final var estimator = new MSACEuclideanTransformation2DRobustEstimator();

        assertFalse(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(true);

        // check correctness
        assertTrue(estimator.isCovarianceKept());
    }

    @Test
    void testEstimateWithoutRefinement() throws LockedException, NotReadyException, RobustEstimatorException {
        for (var t = 0; t < TIMES; t++) {
            // create an euclidean transformation
            final var randomizer = new UniformRandomizer();

            final var theta = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation = new Rotation2D(theta);

            final var translation = new double[2];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

            final var transformation1 = new EuclideanTransformation2D(rotation, translation);

            // generate random points
            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var inputPoints = new ArrayList<Point2D>();
            final var outputPoints = new ArrayList<Point2D>();
            final var outputPointsWithError = new ArrayList<Point2D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            for (var i = 0; i < nPoints; i++) {
                final var inputPoint = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                final var outputPoint = transformation1.transformAndReturnNew(inputPoint);
                Point2D outputPointWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    outputPointWithError = new InhomogeneousPoint2D(
                            outputPoint.getInhomX() + errorX,
                            outputPoint.getInhomY() + errorY);
                } else {
                    // inlier point (without error)
                    outputPointWithError = outputPoint;
                }

                inputPoints.add(inputPoint);
                outputPoints.add(outputPoint);
                outputPointsWithError.add(outputPointWithError);
            }

            final var estimator = new MSACEuclideanTransformation2DRobustEstimator(this, inputPoints,
                    outputPointsWithError);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var transformation2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by transforming input points
            // using estimated transformation (transformation2) and checking
            // that output points are equal to the original output points without
            // error
            for (var i = 0; i < nPoints; i++) {
                final var p1 = outputPoints.get(i);
                final var p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
                assertEquals(0.0, p1.distanceTo(p2), ABSOLUTE_ERROR);
            }

            // check parameters of estimated transformation
            final var rotation2 = transformation2.getRotation();
            final var translation2 = transformation2.getTranslation();

            assertEquals(rotation.getTheta(), rotation2.getTheta(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
        }
    }

    @Test
    void testEstimateColinearWithoutRefinement() throws LockedException, NotReadyException, RobustEstimatorException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            // create an euclidean transformation
            final var randomizer = new UniformRandomizer();

            final var theta = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation = new Rotation2D(theta);

            final var translation = new double[2];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

            final var transformation1 = new EuclideanTransformation2D(rotation, translation);

            // generate random line
            final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var line = new Line2D(a, b, c);

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var inputPoints = new ArrayList<Point2D>();
            final var outputPoints = new ArrayList<Point2D>();
            final var outputPointsWithError = new ArrayList<Point2D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            for (var i = 0; i < nPoints; i++) {
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

                final var inputPoint = new HomogeneousPoint2D(homX, homY, homW);

                assertTrue(line.isLocus(inputPoint));

                final var outputPoint = transformation1.transformAndReturnNew(inputPoint);
                final Point2D outputPointWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    outputPointWithError = new InhomogeneousPoint2D(
                            outputPoint.getInhomX() + errorX,
                            outputPoint.getInhomY() + errorY);
                } else {
                    // inlier point (without error)
                    outputPointWithError = outputPoint;
                }

                inputPoints.add(inputPoint);
                outputPoints.add(outputPoint);
                outputPointsWithError.add(outputPointWithError);
            }

            final var estimator = new MSACEuclideanTransformation2DRobustEstimator(this, inputPoints,
                    outputPointsWithError, true);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var transformation2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by transforming input points
            // using estimated transformation (transformation2) and checking
            // that output points are equal to the original output points without
            // error
            var validPoints = true;
            for (var i = 0; i < nPoints; i++) {
                final var p1 = outputPoints.get(i);
                final var p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
                if (p1.distanceTo(p2) > ABSOLUTE_ERROR) {
                    validPoints = false;
                    break;
                }
                assertEquals(0.0, p1.distanceTo(p2), ABSOLUTE_ERROR);
            }

            if (!validPoints) {
                continue;
            }

            // check parameters of estimated transformation
            final var rotation2 = transformation2.getRotation();
            final var translation2 = transformation2.getTranslation();

            if (Math.abs(rotation2.getTheta() - rotation.getTheta()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation2.getTheta(), rotation.getTheta(), ABSOLUTE_ERROR);

            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);

            numValid++;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testEstimateWithRefinement() throws LockedException, NotReadyException, RobustEstimatorException {
        for (int t = 0; t < TIMES; t++) {
            // create an euclidean transformation
            final var randomizer = new UniformRandomizer();

            final var theta = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation = new Rotation2D(theta);

            final var translation = new double[2];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

            final var transformation1 = new EuclideanTransformation2D(rotation, translation);

            // generate random points
            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var inputPoints = new ArrayList<Point2D>();
            final var outputPoints = new ArrayList<Point2D>();
            final var outputPointsWithError = new ArrayList<Point2D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            for (var i = 0; i < nPoints; i++) {
                final var inputPoint = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                final var outputPoint = transformation1.transformAndReturnNew(inputPoint);
                final Point2D outputPointWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    outputPointWithError = new InhomogeneousPoint2D(
                            outputPoint.getInhomX() + errorX,
                            outputPoint.getInhomY() + errorY);
                } else {
                    // inlier point (without error)
                    outputPointWithError = outputPoint;
                }

                inputPoints.add(inputPoint);
                outputPoints.add(outputPoint);
                outputPointsWithError.add(outputPointWithError);
            }

            final var estimator = new MSACEuclideanTransformation2DRobustEstimator(this, inputPoints,
                    outputPointsWithError);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var transformation2 = estimator.estimate();

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                assertEquals(estimator.getCovariance().getRows(),
                        1 + EuclideanTransformation2D.NUM_TRANSLATION_COORDS);
                assertEquals(estimator.getCovariance().getColumns(),
                        1 + EuclideanTransformation2D.NUM_TRANSLATION_COORDS);
            }

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by transforming input points
            // using estimated transformation (transformation2) and checking
            // that output points are equal to the original output points without
            // error
            for (var i = 0; i < nPoints; i++) {
                final var p1 = outputPoints.get(i);
                final var p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
                assertEquals(0.0, p1.distanceTo(p2), ABSOLUTE_ERROR);
            }

            // check parameters of estimated transformation
            final var rotation2 = transformation2.getRotation();
            final var translation2 = transformation2.getTranslation();

            assertEquals(rotation.getTheta(), rotation2.getTheta(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
        }
    }

    @Test
    void testEstimateColinearWithRefinement() throws LockedException, NotReadyException, RobustEstimatorException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            // create an euclidean transformation
            final var randomizer = new UniformRandomizer();

            final var theta = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation = new Rotation2D(theta);

            final var translation = new double[2];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

            final var transformation1 = new EuclideanTransformation2D(rotation, translation);

            // generate random line
            final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var line = new Line2D(a, b, c);

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var inputPoints = new ArrayList<Point2D>();
            final var outputPoints = new ArrayList<Point2D>();
            final var outputPointsWithError = new ArrayList<Point2D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            for (var i = 0; i < nPoints; i++) {
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

                final var inputPoint = new HomogeneousPoint2D(homX, homY, homW);

                assertTrue(line.isLocus(inputPoint));

                final var outputPoint = transformation1.transformAndReturnNew(inputPoint);
                final Point2D outputPointWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    outputPointWithError = new InhomogeneousPoint2D(
                            outputPoint.getInhomX() + errorX,
                            outputPoint.getInhomY() + errorY);
                } else {
                    // inlier point (without error)
                    outputPointWithError = outputPoint;
                }

                inputPoints.add(inputPoint);
                outputPoints.add(outputPoint);
                outputPointsWithError.add(outputPointWithError);
            }

            final var estimator = new MSACEuclideanTransformation2DRobustEstimator(this, inputPoints,
                    outputPointsWithError, true);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var transformation2 = estimator.estimate();

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                assertEquals(estimator.getCovariance().getRows(),
                        1 + EuclideanTransformation2D.NUM_TRANSLATION_COORDS);
                assertEquals(estimator.getCovariance().getColumns(),
                        1 + EuclideanTransformation2D.NUM_TRANSLATION_COORDS);
            }

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by transforming input points
            // using estimated transformation (transformation2) and checking
            // that output points are equal to the original output points without
            // error
            var validPoints = true;
            for (var i = 0; i < nPoints; i++) {
                final var p1 = outputPoints.get(i);
                final var p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
                if (p1.distanceTo(p2) > ABSOLUTE_ERROR) {
                    validPoints = false;
                    break;
                }
                assertEquals(0.0, p1.distanceTo(p2), ABSOLUTE_ERROR);
            }

            if (!validPoints) {
                continue;
            }

            // check parameters of estimated transformation
            final var rotation2 = transformation2.getRotation();
            final var translation2 = transformation2.getTranslation();

            if (Math.abs(rotation2.getTheta() - rotation.getTheta()) >
                    ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation2.getTheta(), rotation.getTheta(), ABSOLUTE_ERROR);

            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);

            numValid++;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onEstimateStart(final EuclideanTransformation2DRobustEstimator estimator) {
        estimateStart++;
        checkLocked((MSACEuclideanTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final EuclideanTransformation2DRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((MSACEuclideanTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(
            final EuclideanTransformation2DRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        checkLocked((MSACEuclideanTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(
            final EuclideanTransformation2DRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        checkLocked((MSACEuclideanTransformation2DRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = estimateProgressChange = 0;
    }

    private static void checkLocked(final MSACEuclideanTransformation2DRobustEstimator estimator) {
        final var points = new ArrayList<Point2D>();
        assertThrows(LockedException.class, () -> estimator.setPoints(points, points));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setProgressDelta(0.01f));
        assertThrows(LockedException.class, () -> estimator.setThreshold(0.5));
        assertThrows(LockedException.class, () -> estimator.setConfidence(0.5));
        assertThrows(LockedException.class, () -> estimator.setMaxIterations(10));
        assertThrows(LockedException.class, estimator::estimate);
        assertThrows(LockedException.class, () -> estimator.setWeakMinimumSizeAllowed(true));
        assertTrue(estimator.isLocked());
    }
}
