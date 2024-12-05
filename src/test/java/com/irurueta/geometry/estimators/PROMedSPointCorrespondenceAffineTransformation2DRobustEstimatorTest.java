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

class PROMedSPointCorrespondenceAffineTransformation2DRobustEstimatorTest
        implements AffineTransformation2DRobustEstimatorListener {

    private static final double MIN_RANDOM_VALUE = -1000.0;
    private static final double MAX_RANDOM_VALUE = 1000.0;

    private static final double ABSOLUTE_ERROR = 5e-6;

    private static final int MIN_POINTS = 500;
    private static final int MAX_POINTS = 1000;

    private static final double THRESHOLD = 1.0;

    private static final double STD_ERROR = 100.0;

    private static final double MIN_SCORE_ERROR = -0.3;
    private static final double MAX_SCORE_ERROR = 0.3;

    private static final int PERCENTAGE_OUTLIER = 20;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    void testConstants() {
        assertEquals(3, AffineTransformation2DRobustEstimator.MINIMUM_SIZE);
        assertEquals(0.05f, AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0f);
        assertEquals(0.0f, AffineTransformation2DRobustEstimator.MIN_PROGRESS_DELTA, 0.0f);
        assertEquals(1.0f, AffineTransformation2DRobustEstimator.MAX_PROGRESS_DELTA, 0.0f);
        assertEquals(0.99, AffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(5000, AffineTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(0.0, AffineTransformation2DRobustEstimator.MIN_CONFIDENCE, 0.0);
        assertEquals(1.0, AffineTransformation2DRobustEstimator.MAX_CONFIDENCE, 0.0);
        assertEquals(1, AffineTransformation2DRobustEstimator.MIN_ITERATIONS);
        assertTrue(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(AffineTransformation2DRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertEquals(RobustEstimatorMethod.PROMEDS,
                PointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_ROBUST_METHOD);
        assertEquals(1.0,
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(0.0, PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.MIN_STOP_THRESHOLD,
                0.0);
    }

    @Test
    void testConstructor() {
        // test constructor without arguments
        var estimator = new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator();

        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
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

        estimator = new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator(inputPoints, outputPoints);

        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final var pointsEmpty = new ArrayList<Point2D>();
        // not enough points
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator(pointsEmpty, pointsEmpty));
        // different sizes
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator(inputPoints, pointsEmpty));

        // test constructor with listener
        estimator = new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator(this);

        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test constructor with listener and points
        estimator = new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator(this, inputPoints,
                outputPoints);

        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        // not enough points
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator(this, pointsEmpty,
                        pointsEmpty));
        // different sizes
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator(this, inputPoints,
                        pointsEmpty));

        // test constructor with quality scores
        final var qualityScores = new double[PointCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE];
        final var shortQualityScores = new double[1];

        estimator = new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator(qualityScores);

        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator(shortQualityScores));

        // test constructor with points and quality scores
        estimator = new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator(inputPoints, outputPoints,
                qualityScores);

        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        // not enough points
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator(pointsEmpty, pointsEmpty,
                        qualityScores));
        // different sizes
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator(inputPoints, pointsEmpty,
                        qualityScores));
        // not enough scores
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator(inputPoints, outputPoints,
                        shortQualityScores));

        // test constructor with listener and quality scores
        estimator = new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator(this, qualityScores);

        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator(this,
                        shortQualityScores));

        // test constructor with listener, points and quality scores
        estimator = new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator(this, inputPoints,
                outputPoints, qualityScores);

        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        // not enough points
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator(this, pointsEmpty,
                        pointsEmpty, qualityScores));
        // different sizes
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator(this, inputPoints,
                        pointsEmpty, qualityScores));
        // not enough scores
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator(this, inputPoints,
                        outputPoints, shortQualityScores));
    }

    @Test
    void testGetSetStopThreshold() throws LockedException {
        final var estimator = new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator();

        // check default value
        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);

        // set new value
        estimator.setStopThreshold(0.5);

        // check correctness
        assertEquals(0.5, estimator.getStopThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setStopThreshold(0.0));
    }

    @Test
    void testGetSetQualityScores() throws LockedException {
        final var estimator = new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator();

        // check default value
        assertNull(estimator.getQualityScores());

        // set new value
        final var qualityScores = new double[PointCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        final var qualityScores2 = new double[1];
        assertThrows(IllegalArgumentException.class, () -> estimator.setQualityScores(qualityScores2));
    }

    @Test
    void testGetSetConfidence() throws LockedException {
        final var estimator = new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator();

        // check default value
        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE,
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
        final var estimator = new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator();

        // check default value
        assertEquals(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
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
        final var estimator = new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator();

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
        assertFalse(estimator.isReady());

        // if we set quality scores, then estimator becomes ready
        final var qualityScores = new double[PointCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);

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
        final var estimator = new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator();

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
        final var estimator = new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator();

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
    void testIsSetResultRefined() throws LockedException {
        final var estimator = new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator();

        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check correctness
        assertFalse(estimator.isResultRefined());
    }

    @Test
    void testIsSetCovarianceKept() throws LockedException {
        final var estimator = new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator();

        assertFalse(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(true);

        // check correctness
        assertTrue(estimator.isCovarianceKept());
    }

    @Test
    void testEstimateWithoutRefinement() throws WrongSizeException, DecomposerException, LockedException,
            NotReadyException, RobustEstimatorException {
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
        final var qualityScores = new double[nPoints];
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
        for (var i = 0; i < nPoints; i++) {
            final var inputPoint = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var outputPoint = transformation1.transformAndReturnNew(inputPoint);
            Point2D outputPointWithError;
            final var scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, MAX_SCORE_ERROR);
            qualityScores[i] = 1.0 + scoreError;
            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                // point is outlier
                final var errorX = errorRandomizer.nextDouble();
                final var errorY = errorRandomizer.nextDouble();
                outputPointWithError = new InhomogeneousPoint2D(
                        outputPoint.getInhomX() + errorX,
                        outputPoint.getInhomY() + errorY);

                final var error = Math.sqrt(errorX * errorX + errorY * errorY);
                qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
            } else {
                // inlier point (without error)
                outputPointWithError = outputPoint;
            }

            inputPoints.add(inputPoint);
            outputPoints.add(outputPoint);
            outputPointsWithError.add(outputPointWithError);
        }

        final var estimator = new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator(this,
                inputPoints, outputPointsWithError, qualityScores);

        estimator.setStopThreshold(THRESHOLD);
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
        Point2D p1, p2;
        for (var i = 0; i < nPoints; i++) {
            p1 = outputPoints.get(i);
            p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
            assertEquals(0.0, p1.distanceTo(p2), ABSOLUTE_ERROR);
        }
    }

    @Test
    void testEstimateWithRefinement() throws WrongSizeException, DecomposerException, LockedException,
            NotReadyException, RobustEstimatorException {
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
        final var qualityScores = new double[nPoints];
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
        for (var i = 0; i < nPoints; i++) {
            final var inputPoint = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var outputPoint = transformation1.transformAndReturnNew(inputPoint);
            final var scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, MAX_SCORE_ERROR);
            qualityScores[i] = 1.0 + scoreError;
            final Point2D outputPointWithError;
            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                // point is outlier
                final var errorX = errorRandomizer.nextDouble();
                final var errorY = errorRandomizer.nextDouble();
                outputPointWithError = new InhomogeneousPoint2D(
                        outputPoint.getInhomX() + errorX,
                        outputPoint.getInhomY() + errorY);

                final var error = Math.sqrt(errorX * errorX + errorY * errorY);
                qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
            } else {
                // inlier point (without error)
                outputPointWithError = outputPoint;
            }

            inputPoints.add(inputPoint);
            outputPoints.add(outputPoint);
            outputPointsWithError.add(outputPointWithError);
        }

        final var estimator = new PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator(this,
                inputPoints, outputPointsWithError, qualityScores);

        estimator.setStopThreshold(THRESHOLD);
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
        assertEquals(AffineTransformation2D.INHOM_COORDS * AffineTransformation2D.INHOM_COORDS
                + AffineTransformation2D.NUM_TRANSLATION_COORDS, estimator.getCovariance().getRows());
        assertEquals(AffineTransformation2D.INHOM_COORDS * AffineTransformation2D.INHOM_COORDS
                + AffineTransformation2D.NUM_TRANSLATION_COORDS, estimator.getCovariance().getColumns());

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

    @Override
    public void onEstimateStart(final AffineTransformation2DRobustEstimator estimator) {
        estimateStart++;
        checkLocked((PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final AffineTransformation2DRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(final AffineTransformation2DRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        checkLocked((PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(final AffineTransformation2DRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        checkLocked((PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = estimateProgressChange = 0;
    }

    private static void checkLocked(final PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator estimator) {
        final var points = new ArrayList<Point2D>();
        assertThrows(LockedException.class, () -> estimator.setPoints(points, points));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setProgressDelta(0.01f));
        assertThrows(LockedException.class, () -> estimator.setStopThreshold(0.5));
        final var qualityScores = new double[PointCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE];
        assertThrows(LockedException.class, () -> estimator.setQualityScores(qualityScores));
        assertThrows(LockedException.class, () -> estimator.setConfidence(0.5));
        assertThrows(LockedException.class, () -> estimator.setMaxIterations(10));
        assertThrows(LockedException.class, estimator::estimate);
        assertTrue(estimator.isLocked());
    }
}
