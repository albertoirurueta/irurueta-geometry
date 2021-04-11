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
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class PROMedSEuclideanTransformation2DRobustEstimatorTest implements
        EuclideanTransformation2DRobustEstimatorListener {

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
    public void testConstructor() {
        // test constructor without arguments
        PROMedSEuclideanTransformation2DRobustEstimator estimator =
                new PROMedSEuclideanTransformation2DRobustEstimator();

        assertEquals(estimator.getStopThreshold(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());


        // test constructor with points
        List<Point2D> inputPoints = new ArrayList<>();
        List<Point2D> outputPoints = new ArrayList<>();
        for (int i = 0; i < EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }

        estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                inputPoints, outputPoints);

        assertEquals(estimator.getStopThreshold(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        List<Point2D> pointsEmpty = new ArrayList<>();
        estimator = null;
        try {
            // not enough points
            estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                    pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                    inputPoints, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor with listener
        estimator = new PROMedSEuclideanTransformation2DRobustEstimator(this);

        assertEquals(estimator.getStopThreshold(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());


        // test constructor with listener and points
        estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                this, inputPoints, outputPoints);

        assertEquals(estimator.getStopThreshold(),
                LMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                LMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                LMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough points
            estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                    this, pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                    this, inputPoints, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor with quality scores
        double[] qualityScores = new double[
                EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE];
        final double[] shortQualityScores = new double[1];

        estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                qualityScores);

        assertEquals(estimator.getStopThreshold(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                    shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with points and quality scores
        estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                inputPoints, outputPoints, qualityScores);

        assertEquals(estimator.getStopThreshold(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough points
            estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                    pointsEmpty, pointsEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                    inputPoints, pointsEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // not enough scores
            estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                    inputPoints, outputPoints, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener and quality scores
        estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                this, qualityScores);

        assertEquals(estimator.getStopThreshold(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                    this, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor with listener, points and quality scores
        estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                this, inputPoints, outputPoints, qualityScores);

        assertEquals(estimator.getStopThreshold(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough points
            estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                    this, pointsEmpty, pointsEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                    this, inputPoints, pointsEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // not enough scores
            estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                    this, inputPoints, outputPoints, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor without arguments and with weak minimum points
        estimator = new PROMedSEuclideanTransformation2DRobustEstimator(true);

        assertEquals(estimator.getStopThreshold(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());


        // test constructor with points
        inputPoints = new ArrayList<>();
        outputPoints = new ArrayList<>();
        for (int i = 0; i < EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE; i++) {
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }

        estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                inputPoints, outputPoints, true);

        assertEquals(estimator.getStopThreshold(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough points
            estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                    pointsEmpty, pointsEmpty, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                    inputPoints, pointsEmpty, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener
        estimator = new PROMedSEuclideanTransformation2DRobustEstimator(this,
                true);

        assertEquals(estimator.getStopThreshold(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());


        // test constructor with listener and points
        estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                this, inputPoints, outputPoints, true);

        assertEquals(estimator.getStopThreshold(),
                LMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                LMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                LMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough points
            estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                    this, pointsEmpty, pointsEmpty, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                    this, inputPoints, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores
        qualityScores = new double[
                EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE];

        estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                qualityScores, true);

        assertEquals(estimator.getStopThreshold(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                    shortQualityScores, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with points and quality scores
        estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                inputPoints, outputPoints, qualityScores, true);

        assertEquals(estimator.getStopThreshold(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough points
            estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                    pointsEmpty, pointsEmpty, qualityScores, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                    inputPoints, pointsEmpty, qualityScores, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // not enough scores
            estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                    inputPoints, outputPoints, shortQualityScores, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener and quality scores
        estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                this, qualityScores, true);

        assertEquals(estimator.getStopThreshold(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                    this, shortQualityScores, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener, points and quality scores
        estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                this, inputPoints, outputPoints, qualityScores, true);

        assertEquals(estimator.getStopThreshold(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough points
            estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                    this, pointsEmpty, pointsEmpty, qualityScores, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                    this, inputPoints, pointsEmpty, qualityScores, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // not enough scores
            estimator = new PROMedSEuclideanTransformation2DRobustEstimator(
                    this, inputPoints, outputPoints, shortQualityScores, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetStopThreshold() throws LockedException {
        final PROMedSEuclideanTransformation2DRobustEstimator estimator =
                new PROMedSEuclideanTransformation2DRobustEstimator();

        // check default value
        assertEquals(estimator.getStopThreshold(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);

        // set new value
        estimator.setStopThreshold(0.5);

        // check correctness
        assertEquals(estimator.getStopThreshold(), 0.5, 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setStopThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetConfidence() throws LockedException {
        final PROMedSEuclideanTransformation2DRobustEstimator estimator =
                new PROMedSEuclideanTransformation2DRobustEstimator();

        // check default value
        assertEquals(estimator.getConfidence(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);

        // set new value
        estimator.setConfidence(0.5);

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
        final PROMedSEuclideanTransformation2DRobustEstimator estimator =
                new PROMedSEuclideanTransformation2DRobustEstimator();

        // check default value
        assertEquals(estimator.getMaxIterations(),
                PROMedSEuclideanTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);

        // set new value
        estimator.setMaxIterations(10);

        // check correctness
        assertEquals(estimator.getMaxIterations(), 10);

        // Force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetPointsAndIsReady() throws LockedException {
        final PROMedSEuclideanTransformation2DRobustEstimator estimator =
                new PROMedSEuclideanTransformation2DRobustEstimator();

        // check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());

        // set new value
        final List<Point2D> inputPoints = new ArrayList<>();
        final List<Point2D> outputPoints = new ArrayList<>();
        for (int i = 0; i < LMedSEuclideanTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }

        estimator.setPoints(inputPoints, outputPoints);

        // check correctness
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertFalse(estimator.isReady());

        // if we set quality scores, then estimator becomes ready
        final double[] qualityScores = new double[
                EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);

        assertTrue(estimator.isReady());

        // Force IllegalArgumentException
        final List<Point2D> pointsEmpty = new ArrayList<>();
        try {
            // not enough points
            estimator.setPoints(pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator.setPoints(pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListenerAndIsListenerAvailable()
            throws LockedException {
        final PROMedSEuclideanTransformation2DRobustEstimator estimator =
                new PROMedSEuclideanTransformation2DRobustEstimator();

        // check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
    }

    @Test
    public void testIsSetWeakMinimumPointsAllowed() throws LockedException {
        final PROMedSEuclideanTransformation2DRobustEstimator estimator =
                new PROMedSEuclideanTransformation2DRobustEstimator();

        // check default value
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE);

        // set new value
        estimator.setWeakMinimumSizeAllowed(true);

        // check correctness
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE);
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final PROMedSEuclideanTransformation2DRobustEstimator estimator =
                new PROMedSEuclideanTransformation2DRobustEstimator();

        // check default value
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);

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
    public void testIsSetResultRefined() throws LockedException {
        final PROMedSEuclideanTransformation2DRobustEstimator estimator =
                new PROMedSEuclideanTransformation2DRobustEstimator();

        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check correctness
        assertFalse(estimator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final PROMedSEuclideanTransformation2DRobustEstimator estimator =
                new PROMedSEuclideanTransformation2DRobustEstimator();

        assertFalse(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(true);

        // check correctness
        assertTrue(estimator.isCovarianceKept());
    }

    @Test
    public void testEstimateWithoutRefinement() throws LockedException,
            NotReadyException, RobustEstimatorException {
        for (int t = 0; t < TIMES; t++) {
            // create an euclidean transformation
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double theta = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final Rotation2D rotation = new Rotation2D(theta);

            final double[] translation = new double[2];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

            final EuclideanTransformation2D transformation1 =
                    new EuclideanTransformation2D(rotation, translation);


            // generate random points
            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final List<Point2D> inputPoints = new ArrayList<>();
            final List<Point2D> outputPoints = new ArrayList<>();
            final List<Point2D> outputPointsWithError = new ArrayList<>();
            final double[] qualityScores = new double[nPoints];
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            for (int i = 0; i < nPoints; i++) {
                final Point2D inputPoint = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE));
                final Point2D outputPoint = transformation1.transformAndReturnNew(
                        inputPoint);
                Point2D outputPointWithError;
                final double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR,
                        MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    outputPointWithError = new InhomogeneousPoint2D(
                            outputPoint.getInhomX() + errorX,
                            outputPoint.getInhomY() + errorY);
                    final double error = Math.sqrt(errorX * errorX + errorY * errorY);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    // inlier point (without error)
                    outputPointWithError = outputPoint;
                }

                inputPoints.add(inputPoint);
                outputPoints.add(outputPoint);
                outputPointsWithError.add(outputPointWithError);
            }

            final PROMedSEuclideanTransformation2DRobustEstimator estimator =
                    new PROMedSEuclideanTransformation2DRobustEstimator(this,
                            inputPoints, outputPointsWithError, qualityScores);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final EuclideanTransformation2D transformation2 = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by transforming input points
            // using estimated transformation (transformation2) and checking
            // that output points are equal to the original output points without
            // error
            Point2D p1, p2;
            for (int i = 0; i < nPoints; i++) {
                p1 = outputPoints.get(i);
                p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
                assertEquals(p1.distanceTo(p2), 0.0,
                        ABSOLUTE_ERROR);
            }

            // check parameters of estimated transformation
            final Rotation2D rotation2 = transformation2.getRotation();
            final double[] translation2 = transformation2.getTranslation();

            assertEquals(rotation2.getTheta(), rotation.getTheta(),
                    ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testEstimateColinearWithoutRefinement() throws LockedException,
            NotReadyException, RobustEstimatorException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            // create an euclidean transformation
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double theta = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final Rotation2D rotation = new Rotation2D(theta);

            final double[] translation = new double[2];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

            final EuclideanTransformation2D transformation1 =
                    new EuclideanTransformation2D(rotation, translation);

            // generate random line
            final double a = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final double b = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final double c = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final Line2D line = new Line2D(a, b, c);

            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final List<Point2D> inputPoints = new ArrayList<>();
            final List<Point2D> outputPoints = new ArrayList<>();
            final List<Point2D> outputPointsWithError = new ArrayList<>();
            final double[] qualityScores = new double[nPoints];
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            HomogeneousPoint2D inputPoint;
            for (int i = 0; i < nPoints; i++) {
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

                inputPoint = new HomogeneousPoint2D(homX, homY, homW);

                assertTrue(line.isLocus(inputPoint));

                final Point2D outputPoint = transformation1.transformAndReturnNew(
                        inputPoint);
                final Point2D outputPointWithError;
                final double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR,
                        MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    outputPointWithError = new InhomogeneousPoint2D(
                            outputPoint.getInhomX() + errorX,
                            outputPoint.getInhomY() + errorY);
                    final double error = Math.sqrt(errorX * errorX + errorY * errorY);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    // inlier point (without error)
                    outputPointWithError = outputPoint;
                }

                inputPoints.add(inputPoint);
                outputPoints.add(outputPoint);
                outputPointsWithError.add(outputPointWithError);
            }

            final PROMedSEuclideanTransformation2DRobustEstimator estimator =
                    new PROMedSEuclideanTransformation2DRobustEstimator(this,
                            inputPoints, outputPointsWithError, qualityScores,
                            true);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final EuclideanTransformation2D transformation2 = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by transforming input points
            // using estimated transformation (transformation2) and checking
            // that output points are equal to the original output points without
            // error
            Point2D p1, p2;
            boolean isValid = true;
            for (int i = 0; i < nPoints; i++) {
                p1 = outputPoints.get(i);
                p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
                if (p1.distanceTo(p2) > ABSOLUTE_ERROR) {
                    isValid = false;
                    break;
                }
                assertEquals(p1.distanceTo(p2), 0.0,
                        ABSOLUTE_ERROR);
            }

            if (!isValid) {
                continue;
            }

            // check parameters of estimated transformation
            final Rotation2D rotation2 = transformation2.getRotation();
            final double[] translation2 = transformation2.getTranslation();

            assertEquals(rotation2.getTheta(), rotation.getTheta(),
                    ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);

            numValid++;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateWithRefinement() throws LockedException,
            NotReadyException, RobustEstimatorException {
        for (int t = 0; t < TIMES; t++) {
            // create an euclidean transformation
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double theta = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final Rotation2D rotation = new Rotation2D(theta);

            final double[] translation = new double[2];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

            final EuclideanTransformation2D transformation1 =
                    new EuclideanTransformation2D(rotation, translation);


            // generate random points
            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final List<Point2D> inputPoints = new ArrayList<>();
            final List<Point2D> outputPoints = new ArrayList<>();
            final List<Point2D> outputPointsWithError = new ArrayList<>();
            final double[] qualityScores = new double[nPoints];
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            for (int i = 0; i < nPoints; i++) {
                final Point2D inputPoint = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE));
                final Point2D outputPoint = transformation1.transformAndReturnNew(
                        inputPoint);
                final Point2D outputPointWithError;
                final double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR,
                        MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    outputPointWithError = new InhomogeneousPoint2D(
                            outputPoint.getInhomX() + errorX,
                            outputPoint.getInhomY() + errorY);
                    final double error = Math.sqrt(errorX * errorX + errorY * errorY);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    // inlier point (without error)
                    outputPointWithError = outputPoint;
                }

                inputPoints.add(inputPoint);
                outputPoints.add(outputPoint);
                outputPointsWithError.add(outputPointWithError);
            }

            final PROMedSEuclideanTransformation2DRobustEstimator estimator =
                    new PROMedSEuclideanTransformation2DRobustEstimator(this,
                            inputPoints, outputPointsWithError, qualityScores);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final EuclideanTransformation2D transformation2 = estimator.estimate();

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

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by transforming input points
            // using estimated transformation (transformation2) and checking
            // that output points are equal to the original output points without
            // error
            Point2D p1, p2;
            for (int i = 0; i < nPoints; i++) {
                p1 = outputPoints.get(i);
                p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
                assertEquals(p1.distanceTo(p2), 0.0,
                        ABSOLUTE_ERROR);
            }

            // check parameters of estimated transformation
            final Rotation2D rotation2 = transformation2.getRotation();
            final double[] translation2 = transformation2.getTranslation();

            assertEquals(rotation2.getTheta(), rotation.getTheta(),
                    ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testEstimateColinearWithRefinement() throws LockedException,
            NotReadyException, RobustEstimatorException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            // create an euclidean transformation
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double theta = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final Rotation2D rotation = new Rotation2D(theta);

            final double[] translation = new double[2];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

            final EuclideanTransformation2D transformation1 =
                    new EuclideanTransformation2D(rotation, translation);

            // generate random line
            final double a = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final double b = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final double c = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final Line2D line = new Line2D(a, b, c);

            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final List<Point2D> inputPoints = new ArrayList<>();
            final List<Point2D> outputPoints = new ArrayList<>();
            final List<Point2D> outputPointsWithError = new ArrayList<>();
            final double[] qualityScores = new double[nPoints];
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            HomogeneousPoint2D inputPoint;
            for (int i = 0; i < nPoints; i++) {
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

                inputPoint = new HomogeneousPoint2D(homX, homY, homW);

                assertTrue(line.isLocus(inputPoint));

                final Point2D outputPoint = transformation1.transformAndReturnNew(
                        inputPoint);
                final Point2D outputPointWithError;
                final double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR,
                        MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    outputPointWithError = new InhomogeneousPoint2D(
                            outputPoint.getInhomX() + errorX,
                            outputPoint.getInhomY() + errorY);
                    final double error = Math.sqrt(errorX * errorX + errorY * errorY);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    // inlier point (without error)
                    outputPointWithError = outputPoint;
                }

                inputPoints.add(inputPoint);
                outputPoints.add(outputPoint);
                outputPointsWithError.add(outputPointWithError);
            }

            final PROMedSEuclideanTransformation2DRobustEstimator estimator =
                    new PROMedSEuclideanTransformation2DRobustEstimator(this,
                            inputPoints, outputPointsWithError, qualityScores,
                            true);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final EuclideanTransformation2D transformation2 = estimator.estimate();

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

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by transforming input points
            // using estimated transformation (transformation2) and checking
            // that output points are equal to the original output points without
            // error
            Point2D p1, p2;
            boolean isValid = true;
            for (int i = 0; i < nPoints; i++) {
                p1 = outputPoints.get(i);
                p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
                if (p1.distanceTo(p2) > ABSOLUTE_ERROR) {
                    isValid = false;
                    break;
                }
                assertEquals(p1.distanceTo(p2), 0.0,
                        ABSOLUTE_ERROR);
            }

            if (!isValid) {
                continue;
            }

            // check parameters of estimated transformation
            final Rotation2D rotation2 = transformation2.getRotation();
            final double[] translation2 = transformation2.getTranslation();

            assertEquals(rotation2.getTheta(), rotation.getTheta(),
                    ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);

            numValid++;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onEstimateStart(final EuclideanTransformation2DRobustEstimator estimator) {
        estimateStart++;
        checkLocked((PROMedSEuclideanTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final EuclideanTransformation2DRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((PROMedSEuclideanTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(
            final EuclideanTransformation2DRobustEstimator estimator, int iteration) {
        estimateNextIteration++;
        checkLocked((PROMedSEuclideanTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(
            final EuclideanTransformation2DRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        checkLocked((PROMedSEuclideanTransformation2DRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void checkLocked(
            final PROMedSEuclideanTransformation2DRobustEstimator estimator) {
        final List<Point2D> points = new ArrayList<>();
        try {
            estimator.setPoints(points, points);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setProgressDelta(0.01f);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setStopThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            final double[] qualityScores = new double[
                    EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE];
            estimator.setQualityScores(qualityScores);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setConfidence(0.5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setMaxIterations(10);
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
        try {
            estimator.setWeakMinimumSizeAllowed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        assertTrue(estimator.isLocked());
    }
}
