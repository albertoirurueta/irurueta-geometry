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

import com.irurueta.algebra.DecomposerException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.AffineTransformation2D;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class LMedSPointCorrespondenceAffineTransformation2DRobustEstimatorTest
        implements AffineTransformation2DRobustEstimatorListener {

    private static final double MIN_RANDOM_VALUE = -1000.0;
    private static final double MAX_RANDOM_VALUE = 1000.0;

    private static final double ABSOLUTE_ERROR = 5e-6;

    private static final int MIN_POINTS = 500;
    private static final int MAX_POINTS = 1000;

    private static final double STOP_THRESHOLD = 1e-3;

    private static final double STD_ERROR = 100.0;

    private static final int PERCENTAGE_OUTLIER = 20;

    private static final int TIMES = 10;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    void testConstants() {
        assertEquals(1.0, LMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(0.0, LMedSPointCorrespondenceAffineTransformation2DRobustEstimator.MIN_STOP_THRESHOLD,
                0.0);
    }

    @Test
    void testConstructor() {
        // test constructor without arguments
        var estimator = new LMedSPointCorrespondenceAffineTransformation2DRobustEstimator();

        assertEquals(LMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(LMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(LMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test constructor with points
        final var inputPoints = new ArrayList<Point2D>();
        final var outputPoints = new ArrayList<Point2D>();
        for (var i = 0; i < PointCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }

        estimator = new LMedSPointCorrespondenceAffineTransformation2DRobustEstimator(inputPoints, outputPoints);

        assertEquals(LMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(LMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(LMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final var pointsEmpty = new ArrayList<Point2D>();
        // not enough points
        assertThrows(IllegalArgumentException.class,
                () -> new LMedSPointCorrespondenceAffineTransformation2DRobustEstimator(pointsEmpty, pointsEmpty));
        // different sizes
        assertThrows(IllegalArgumentException.class,
                () -> new LMedSPointCorrespondenceAffineTransformation2DRobustEstimator(inputPoints, pointsEmpty));

        // test constructor with listener
        estimator = new LMedSPointCorrespondenceAffineTransformation2DRobustEstimator(this);

        assertEquals(RANSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RANSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(RANSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test constructor with listener and points
        estimator = new LMedSPointCorrespondenceAffineTransformation2DRobustEstimator(this, inputPoints,
                outputPoints);

        assertEquals(RANSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RANSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(RANSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        // not enough points
        assertThrows(IllegalArgumentException.class,
                () -> new LMedSPointCorrespondenceAffineTransformation2DRobustEstimator(this, pointsEmpty,
                        pointsEmpty));
        // different sizes
        assertThrows(IllegalArgumentException.class,
                () -> new LMedSPointCorrespondenceAffineTransformation2DRobustEstimator(this, inputPoints,
                        pointsEmpty));
    }

    @Test
    void testGetSetStopThreshold() throws LockedException {
        final var estimator = new LMedSPointCorrespondenceAffineTransformation2DRobustEstimator();

        // check default value
        assertEquals(LMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);

        // set new value
        estimator.setStopThreshold(0.5);

        // check correctness
        assertEquals(0.5, estimator.getStopThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setStopThreshold(0.0));
    }

    @Test
    void testGetSetConfidence() throws LockedException {
        final var estimator = new LMedSPointCorrespondenceAffineTransformation2DRobustEstimator();

        // check default value
        assertEquals(LMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);

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
        final var estimator = new LMedSPointCorrespondenceAffineTransformation2DRobustEstimator();

        // check default value
        assertEquals(LMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(10);

        // check correctness
        assertEquals(10, estimator.getMaxIterations());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setMaxIterations(0));
    }

    @Test
    void testGetSetPointsAndIsReady() throws LockedException {
        final var estimator = new LMedSPointCorrespondenceAffineTransformation2DRobustEstimator();

        // check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());

        // set new value
        final var inputPoints = new ArrayList<Point2D>();
        final var outputPoints = new ArrayList<Point2D>();
        for (var i = 0; i < PointCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
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
        final var estimator = new LMedSPointCorrespondenceAffineTransformation2DRobustEstimator();

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
    void testGetSetProgressDelta() throws LockedException {
        final var estimator = new LMedSPointCorrespondenceAffineTransformation2DRobustEstimator();

        // check default value
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
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
    void testGetSetQualityScores() throws LockedException {
        final var estimator = new LMedSPointCorrespondenceAffineTransformation2DRobustEstimator();

        assertNull(estimator.getQualityScores());

        final var qualityScores = new double[PointCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertNull(estimator.getQualityScores());
    }

    @Test
    void testIsSetResultRefined() throws LockedException {
        final var estimator = new LMedSPointCorrespondenceAffineTransformation2DRobustEstimator();

        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check correctness
        assertFalse(estimator.isResultRefined());
    }

    @Test
    void testIsSetCovarianceKept() throws LockedException {
        final var estimator = new LMedSPointCorrespondenceAffineTransformation2DRobustEstimator();

        assertFalse(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(true);

        // check correctness
        assertTrue(estimator.isCovarianceKept());
    }

    @Test
    void testEstimateWithoutRefinement() throws WrongSizeException, DecomposerException, LockedException,
            NotReadyException, RobustEstimatorException {
        for (var t = 0; t < TIMES; t++) {
            // create an affine transformation
            Matrix a;
            do {
                // ensure A matrix is invertible
                a = Matrix.createWithUniformRandomValues(AffineTransformation2D.INHOM_COORDS,
                        AffineTransformation2D.INHOM_COORDS, -1.0, 1.0);
                final var norm = Utils.normF(a);
                // normalize T to increase accuracy
                a.multiplyByScalar(1.0 / norm);
            } while (Utils.rank(a) < AffineTransformation2D.INHOM_COORDS);

            final var translation = new double[AffineTransformation2D.INHOM_COORDS];
            final var randomizer = new UniformRandomizer();
            randomizer.fill(translation, -1.0, 1.0);

            final var transformation1 = new AffineTransformation2D(a, translation);

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

            final var estimator = new LMedSPointCorrespondenceAffineTransformation2DRobustEstimator(this,
                    inputPoints, outputPointsWithError);

            estimator.setStopThreshold(STOP_THRESHOLD);
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
        }
    }

    @Test
    void testEstimateWithRefinement() throws WrongSizeException, DecomposerException, LockedException,
            NotReadyException, RobustEstimatorException {
        for (var t = 0; t < TIMES; t++) {
            // create an affine transformation
            Matrix a;
            do {
                // ensure A matrix is invertible
                a = Matrix.createWithUniformRandomValues(AffineTransformation2D.INHOM_COORDS,
                        AffineTransformation2D.INHOM_COORDS, -1.0, 1.0);
                final var norm = Utils.normF(a);
                // normalize T to increase accuracy
                a.multiplyByScalar(1.0 / norm);
            } while (Utils.rank(a) < AffineTransformation2D.INHOM_COORDS);

            final var translation = new double[AffineTransformation2D.INHOM_COORDS];
            final var randomizer = new UniformRandomizer();
            randomizer.fill(translation, -1.0, 1.0);

            final var transformation1 = new AffineTransformation2D(a, translation);

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

            final var estimator = new LMedSPointCorrespondenceAffineTransformation2DRobustEstimator(this,
                    inputPoints, outputPointsWithError);

            estimator.setStopThreshold(STOP_THRESHOLD);
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
            assertNotNull(estimator.getCovariance());
            assertEquals(estimator.getCovariance().getRows(),
                    AffineTransformation2D.INHOM_COORDS * AffineTransformation2D.INHOM_COORDS
                            + AffineTransformation2D.NUM_TRANSLATION_COORDS);
            assertEquals(estimator.getCovariance().getColumns(),
                    AffineTransformation2D.INHOM_COORDS * AffineTransformation2D.INHOM_COORDS
                            + AffineTransformation2D.NUM_TRANSLATION_COORDS);

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
        }
    }

    @Override
    public void onEstimateStart(final AffineTransformation2DRobustEstimator estimator) {
        estimateStart++;
        checkLocked((LMedSPointCorrespondenceAffineTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final AffineTransformation2DRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((LMedSPointCorrespondenceAffineTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(final AffineTransformation2DRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        checkLocked((LMedSPointCorrespondenceAffineTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(final AffineTransformation2DRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        checkLocked((LMedSPointCorrespondenceAffineTransformation2DRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = estimateProgressChange = 0;
    }

    private static void checkLocked(final LMedSPointCorrespondenceAffineTransformation2DRobustEstimator estimator) {
        final var points = new ArrayList<Point2D>();
        assertThrows(LockedException.class, () -> estimator.setPoints(points, points));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setProgressDelta(0.01f));
        assertThrows(LockedException.class, () -> estimator.setStopThreshold(0.5));
        assertThrows(LockedException.class, () -> estimator.setConfidence(0.5));
        assertThrows(LockedException.class, () -> estimator.setMaxIterations(10));
        assertThrows(LockedException.class, estimator::estimate);
        assertTrue(estimator.isLocked());
    }
}
