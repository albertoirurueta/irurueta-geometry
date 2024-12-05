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

import com.irurueta.geometry.*;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class PROSACEuclideanTransformation2DRobustEstimatorTest implements EuclideanTransformation2DRobustEstimatorListener {

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_ANGLE_DEGREES = -90.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double MIN_TRANSLATION = -100.0;
    private static final double MAX_TRANSLATION = 100.0;

    private static final double THRESHOLD = 1.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final double STD_ERROR = 100.0;

    private static final double MIN_SCORE_ERROR = -0.3;
    private static final double MAX_SCORE_ERROR = 0.3;

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
        assertEquals(3, EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE);
        assertEquals(2, EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE);
        assertEquals(0.05f, EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0f);
        assertEquals(0.0f, EuclideanTransformation2DRobustEstimator.MIN_PROGRESS_DELTA, 0.0f);
        assertEquals(1.0f, EuclideanTransformation2DRobustEstimator.MAX_PROGRESS_DELTA, 0.0f);
        assertEquals(0.99, EuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(5000, EuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(0.0, EuclideanTransformation2DRobustEstimator.MIN_CONFIDENCE, 0.0);
        assertEquals(1.0, EuclideanTransformation2DRobustEstimator.MAX_CONFIDENCE, 0.0);
        assertEquals(1, EuclideanTransformation2DRobustEstimator.MIN_ITERATIONS);
        assertTrue(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(EuclideanTransformation2DRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertEquals(RobustEstimatorMethod.PROMEDS, EuclideanTransformation2DRobustEstimator.DEFAULT_ROBUST_METHOD);
        assertEquals(1.0, PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(0.0, PROSACEuclideanTransformation2DRobustEstimator.MIN_THRESHOLD, 0.0);
        assertFalse(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertFalse(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
    }

    @Test
    void testConstructor() {
        // test constructor without arguments
        var estimator = new PROSACEuclideanTransformation2DRobustEstimator();

        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // test constructor with points
        final var inputPoints = new ArrayList<Point2D>();
        final var outputPoints = new ArrayList<Point2D>();
        for (var i = 0; i < EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }

        estimator = new PROSACEuclideanTransformation2DRobustEstimator(inputPoints, outputPoints);

        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        final var pointsEmpty = new ArrayList<Point2D>();
        // not enough points
        assertThrows(IllegalArgumentException.class, () -> new PROSACEuclideanTransformation2DRobustEstimator(
                pointsEmpty, pointsEmpty));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> new PROSACEuclideanTransformation2DRobustEstimator(
                inputPoints, pointsEmpty));

        // test constructor with listener
        estimator = new PROSACEuclideanTransformation2DRobustEstimator(this);

        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // test constructor with listener and points
        estimator = new PROSACEuclideanTransformation2DRobustEstimator(this, inputPoints, outputPoints);

        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        // not enough points
        assertThrows(IllegalArgumentException.class, () -> new PROSACEuclideanTransformation2DRobustEstimator(
                this, pointsEmpty, pointsEmpty));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> new PROSACEuclideanTransformation2DRobustEstimator(
                this, inputPoints, pointsEmpty));

        // test constructor with quality scores
        final var qualityScores = new double[EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE];
        final var shortQualityScores = new double[1];

        estimator = new PROSACEuclideanTransformation2DRobustEstimator(qualityScores);

        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACEuclideanTransformation2DRobustEstimator(
                shortQualityScores));

        // test constructor with points and quality scores
        estimator = new PROSACEuclideanTransformation2DRobustEstimator(inputPoints, outputPoints, qualityScores);

        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        // not enough points
        assertThrows(IllegalArgumentException.class, () -> new PROSACEuclideanTransformation2DRobustEstimator(
                pointsEmpty, pointsEmpty, qualityScores));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> new PROSACEuclideanTransformation2DRobustEstimator(
                inputPoints, pointsEmpty, qualityScores));
        // not enough scores
        assertThrows(IllegalArgumentException.class, () -> new PROSACEuclideanTransformation2DRobustEstimator(
                inputPoints, outputPoints, shortQualityScores));

        // test constructor with listener and quality scores
        estimator = new PROSACEuclideanTransformation2DRobustEstimator(this, qualityScores);

        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACEuclideanTransformation2DRobustEstimator(
                this, shortQualityScores));

        // test constructor with listener, points and quality scores
        estimator = new PROSACEuclideanTransformation2DRobustEstimator(this, inputPoints, outputPoints,
                qualityScores);

        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        // not enough points
        assertThrows(IllegalArgumentException.class, () -> new PROSACEuclideanTransformation2DRobustEstimator(
                this, pointsEmpty, pointsEmpty, qualityScores));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> new PROSACEuclideanTransformation2DRobustEstimator(
                this, inputPoints, pointsEmpty, qualityScores));
        // not enough scores
        assertThrows(IllegalArgumentException.class, () -> new PROSACEuclideanTransformation2DRobustEstimator(
                this, inputPoints, outputPoints, shortQualityScores));

        // test constructor without arguments and weak min points
        estimator = new PROSACEuclideanTransformation2DRobustEstimator(true);

        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // test constructor with points
        final var inputPoints2 = new ArrayList<Point2D>();
        final var outputPoints2 = new ArrayList<Point2D>();
        for (var i = 0; i < EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE; i++) {
            inputPoints2.add(Point2D.create());
            outputPoints2.add(Point2D.create());
        }

        estimator = new PROSACEuclideanTransformation2DRobustEstimator(inputPoints2, outputPoints2,
                true);

        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertSame(inputPoints2, estimator.getInputPoints());
        assertSame(outputPoints2, estimator.getOutputPoints());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        // not enough points
        assertThrows(IllegalArgumentException.class, () -> new PROSACEuclideanTransformation2DRobustEstimator(
                pointsEmpty, pointsEmpty, true));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> new PROSACEuclideanTransformation2DRobustEstimator(
                inputPoints2, pointsEmpty, true));

        // test constructor with listener
        estimator = new PROSACEuclideanTransformation2DRobustEstimator(this, true);

        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // test constructor with listener and points
        estimator = new PROSACEuclideanTransformation2DRobustEstimator(this, inputPoints2, outputPoints2,
                true);

        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertSame(inputPoints2, estimator.getInputPoints());
        assertSame(outputPoints2, estimator.getOutputPoints());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        // not enough points
        assertThrows(IllegalArgumentException.class, () -> new PROSACEuclideanTransformation2DRobustEstimator(
                this, pointsEmpty, pointsEmpty, true));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> new PROSACEuclideanTransformation2DRobustEstimator(
                this, inputPoints, pointsEmpty, true));

        // test constructor with quality scores
        final var qualityScores2 = new double[EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE];

        estimator = new PROSACEuclideanTransformation2DRobustEstimator(qualityScores2, true);

        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(qualityScores2, estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACEuclideanTransformation2DRobustEstimator(
                shortQualityScores, true));

        // test constructor with points and quality scores
        estimator = new PROSACEuclideanTransformation2DRobustEstimator(inputPoints2, outputPoints2, qualityScores2,
                true);

        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertSame(inputPoints2, estimator.getInputPoints());
        assertSame(outputPoints2, estimator.getOutputPoints());
        assertSame(qualityScores2, estimator.getQualityScores());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        // not enough points
        assertThrows(IllegalArgumentException.class, () -> new PROSACEuclideanTransformation2DRobustEstimator(
                pointsEmpty, pointsEmpty, qualityScores, true));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> new PROSACEuclideanTransformation2DRobustEstimator(
                inputPoints, pointsEmpty, qualityScores, true));
        // not enough scores
        assertThrows(IllegalArgumentException.class, () -> new PROSACEuclideanTransformation2DRobustEstimator(
                inputPoints, outputPoints, shortQualityScores, true));

        // test constructor with listener and quality scores
        estimator = new PROSACEuclideanTransformation2DRobustEstimator(this, qualityScores,
                true);

        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACEuclideanTransformation2DRobustEstimator(
                this, shortQualityScores, true));

        // test constructor with listener, points and quality scores
        estimator = new PROSACEuclideanTransformation2DRobustEstimator(this, inputPoints, outputPoints,
                qualityScores, true);

        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        // not enough points
        assertThrows(IllegalArgumentException.class, () -> new PROSACEuclideanTransformation2DRobustEstimator(
                this, pointsEmpty, pointsEmpty, qualityScores, true));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> new PROSACEuclideanTransformation2DRobustEstimator(
                this, inputPoints, pointsEmpty, qualityScores, true));
        // not enough scores
        assertThrows(IllegalArgumentException.class, () -> new PROSACEuclideanTransformation2DRobustEstimator(
                this, inputPoints, outputPoints, shortQualityScores, true));
    }

    @Test
    void testGetSetStopThreshold() throws LockedException {
        final var estimator = new PROSACEuclideanTransformation2DRobustEstimator();

        // check default value
        assertEquals(PROMedSEuclideanTransformation2DRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getThreshold(),
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
        final var estimator = new PROSACEuclideanTransformation2DRobustEstimator();

        // check default value
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
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
        final var estimator = new PROSACEuclideanTransformation2DRobustEstimator();

        // check default value
        assertEquals(PROSACEuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
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
        final var estimator = new PROSACEuclideanTransformation2DRobustEstimator();

        // check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());

        // set new value
        final var inputPoints = new ArrayList<Point2D>();
        final var outputPoints = new ArrayList<Point2D>();
        for (var i = 0; i < EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }

        estimator.setPoints(inputPoints, outputPoints);

        // check correctness
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertFalse(estimator.isReady());

        // if we set quality scores, then estimator becomes ready
        final var qualityScores = new double[EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE];
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
        final var estimator = new PROSACEuclideanTransformation2DRobustEstimator();

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
        final var estimator = new PROSACEuclideanTransformation2DRobustEstimator();

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
        final var estimator = new PROSACEuclideanTransformation2DRobustEstimator();

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
        final var estimator = new PROSACEuclideanTransformation2DRobustEstimator();

        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check correctness
        assertFalse(estimator.isResultRefined());
    }

    @Test
    void testIsSetCovarianceKept() throws LockedException {
        final var estimator = new PROSACEuclideanTransformation2DRobustEstimator();

        assertFalse(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(true);

        // check correctness
        assertTrue(estimator.isCovarianceKept());
    }

    @Test
    void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        final var estimator = new PROSACEuclideanTransformation2DRobustEstimator();

        assertFalse(estimator.isComputeAndKeepInliersEnabled());

        // set new value
        estimator.setComputeAndKeepInliersEnabled(true);

        // check correctness
        assertTrue(estimator.isComputeAndKeepInliersEnabled());
    }

    @Test
    void testIsSetComputeAndKeepResidualsEnabled() throws LockedException {
        final var estimator = new PROSACEuclideanTransformation2DRobustEstimator();

        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // set new value
        estimator.setComputeAndKeepResidualsEnabled(true);

        // check correctness
        assertTrue(estimator.isComputeAndKeepResidualsEnabled());
    }

    @Test
    void testEstimateWithoutRefinement() throws LockedException, NotReadyException, RobustEstimatorException {
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

        final var estimator = new PROSACEuclideanTransformation2DRobustEstimator(this, inputPoints,
                outputPointsWithError, qualityScores);

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
            final var qualityScores = new double[nPoints];
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

            final var estimator = new PROSACEuclideanTransformation2DRobustEstimator(this, inputPoints,
                    outputPointsWithError, qualityScores, true);

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
                if (p1.distanceTo(p2) > ABSOLUTE_ERROR) {
                    break;
                }
                assertEquals(0.0, p1.distanceTo(p2), ABSOLUTE_ERROR);
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

            final var estimator = new PROSACEuclideanTransformation2DRobustEstimator(this, inputPoints,
                    outputPointsWithError, qualityScores);

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
                assertEquals(1 + EuclideanTransformation2D.NUM_TRANSLATION_COORDS,
                        estimator.getCovariance().getRows());
                assertEquals(1 + EuclideanTransformation2D.NUM_TRANSLATION_COORDS,
                        estimator.getCovariance().getColumns());
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
            final var qualityScores = new double[nPoints];
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

            final var estimator = new PROSACEuclideanTransformation2DRobustEstimator(this, inputPoints,
                    outputPointsWithError, qualityScores, true);

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
                assertEquals(1 + EuclideanTransformation2D.NUM_TRANSLATION_COORDS,
                        estimator.getCovariance().getRows());
                assertEquals(1 + EuclideanTransformation2D.NUM_TRANSLATION_COORDS,
                        estimator.getCovariance().getColumns());
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
                if (p1.distanceTo(p2) > ABSOLUTE_ERROR) {
                    break;
                }
                assertEquals(0.0, p1.distanceTo(p2), ABSOLUTE_ERROR);
            }

            // check parameters of estimated transformation
            final var rotation2 = transformation2.getRotation();
            final var translation2 = transformation2.getTranslation();

            if (Math.abs(rotation2.getTheta() - rotation.getTheta()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getTheta(), rotation2.getTheta(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onEstimateStart(final EuclideanTransformation2DRobustEstimator estimator) {
        estimateStart++;
        checkLocked((PROSACEuclideanTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final EuclideanTransformation2DRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((PROSACEuclideanTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(final EuclideanTransformation2DRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        checkLocked((PROSACEuclideanTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(
            final EuclideanTransformation2DRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        checkLocked((PROSACEuclideanTransformation2DRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = estimateProgressChange = 0;
    }

    private static void checkLocked(final PROSACEuclideanTransformation2DRobustEstimator estimator) {
        final var points = new ArrayList<Point2D>();
        assertThrows(LockedException.class, () -> estimator.setPoints(points, points));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setProgressDelta(0.01f));
        assertThrows(LockedException.class, () -> estimator.setThreshold(0.5));
        final var qualityScores = new double[EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE];
        assertThrows(LockedException.class, () -> estimator.setQualityScores(qualityScores));
        assertThrows(LockedException.class, () -> estimator.setConfidence(0.5));
        assertThrows(LockedException.class, () -> estimator.setMaxIterations(10));
        assertThrows(LockedException.class, () -> estimator.setCovarianceKept(true));
        assertThrows(LockedException.class, estimator::estimate);
        assertThrows(LockedException.class, () -> estimator.setWeakMinimumSizeAllowed(true));
        assertTrue(estimator.isLocked());
    }
}
