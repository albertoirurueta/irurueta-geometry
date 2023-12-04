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
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class PROSACPointCorrespondenceAffineTransformation2DRobustEstimatorTest
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
    public void testConstants() {
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
                PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_ROBUST_METHOD);
        assertEquals(1.0, PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(0.0, PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.MIN_THRESHOLD,
                0.0);
        assertFalse(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertFalse(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
    }

    @Test
    public void testConstructor() {
        // test constructor without arguments
        PROSACPointCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator();

        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // test constructor with points
        final List<Point2D> inputPoints = new ArrayList<>();
        final List<Point2D> outputPoints = new ArrayList<>();
        for (int i = 0; i < PointCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }

        estimator = new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator(
                inputPoints, outputPoints);

        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        final List<Point2D> pointsEmpty = new ArrayList<>();
        estimator = null;
        try {
            // not enough points
            estimator = new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator(
                    pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator(
                    inputPoints, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener
        estimator = new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator(
                this);

        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // test constructor with listener and points
        estimator = new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator(
                this, inputPoints, outputPoints);

        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough points
            estimator = new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator(
                    this, pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator(
                    this, inputPoints, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores
        final double[] qualityScores = new double[
                PointCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE];
        final double[] shortQualityScores = new double[1];

        estimator = new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator(
                qualityScores);

        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator(
                    shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with points and quality scores
        estimator = new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator(
                inputPoints, outputPoints, qualityScores);

        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough points
            estimator = new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator(
                    pointsEmpty, pointsEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            //different sizes
            estimator = new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator(
                    inputPoints, pointsEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            //not enough scores
            estimator = new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator(
                    inputPoints, outputPoints, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener and quality scores
        estimator = new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator(
                this, qualityScores);

        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator(
                    this, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener, points and quality scores
        estimator = new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator(
                this, inputPoints, outputPoints, qualityScores);

        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough points
            estimator = new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator(
                    this, pointsEmpty, pointsEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator(
                    this, inputPoints, pointsEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // not enough scores
            estimator = new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator(
                    this, inputPoints, outputPoints, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetThreshold() throws LockedException {
        final PROSACPointCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator();

        // check default value
        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);

        // set new value
        estimator.setThreshold(0.5);

        // check correctness
        assertEquals(0.5, estimator.getThreshold(), 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final PROSACPointCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator();

        // check default value
        assertNull(estimator.getQualityScores());

        // set new value
        double[] qualityScores = new double[
                PointCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        qualityScores = new double[1];
        try {
            estimator.setQualityScores(qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetConfidence() throws LockedException {
        final PROSACPointCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator();

        // check default value
        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);

        // set new value
        estimator.setConfidence(0.5);

        // check correctness
        assertEquals(0.5, estimator.getConfidence(), 0.0);

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
        final PROSACPointCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator();

        // check default value
        assertEquals(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(10);

        // check correctness
        assertEquals(10, estimator.getMaxIterations());

        // Force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetPointsAndIsReady() throws LockedException {
        final PROSACPointCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator();

        // check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());

        // set new value
        final List<Point2D> inputPoints = new ArrayList<>();
        final List<Point2D> outputPoints = new ArrayList<>();
        for (int i = 0; i < PointCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }

        estimator.setPoints(inputPoints, outputPoints);

        // check correctness
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertFalse(estimator.isReady());

        // if we set quality scores, then estimator becomes ready
        final double[] qualityScores = new double[
                PointCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE];
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
        final PROSACPointCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator();

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
    public void testGetSetProgressDelta() throws LockedException {
        final PROSACPointCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator();

        // check default value
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check correctness
        assertEquals(0.5f, estimator.getProgressDelta(), 0.0);

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
        final PROSACPointCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator();

        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check correctness
        assertFalse(estimator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final PROSACPointCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator();

        assertFalse(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(true);

        // check correctness
        assertTrue(estimator.isCovarianceKept());
    }

    @Test
    public void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        final PROSACPointCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator();

        assertFalse(estimator.isComputeAndKeepInliersEnabled());

        // set new value
        estimator.setComputeAndKeepInliersEnabled(true);

        // check correctness
        assertTrue(estimator.isComputeAndKeepInliersEnabled());
    }

    @Test
    public void testIsSetComputeAndKeepResidualsEnabled()
            throws LockedException {
        final PROSACPointCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator();

        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // set new value
        estimator.setComputeAndKeepResidualsEnabled(true);

        // check correctness
        assertTrue(estimator.isComputeAndKeepResidualsEnabled());
    }

    @Test
    public void testEstimateWithoutRefinement() throws WrongSizeException,
            DecomposerException, LockedException, NotReadyException,
            RobustEstimatorException {
        // create an affine transformation
        Matrix a;
        do {
            // ensure A matrix is invertible
            a = Matrix.createWithUniformRandomValues(
                    AffineTransformation2D.INHOM_COORDS,
                    AffineTransformation2D.INHOM_COORDS, -1.0, 1.0);
            final double norm = Utils.normF(a);
            // normalize T to increase accuracy
            a.multiplyByScalar(1.0 / norm);
        } while (Utils.rank(a) < AffineTransformation2D.INHOM_COORDS);

        final double[] translation = new double[
                AffineTransformation2D.INHOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(translation, -1.0, 1.0);

        final AffineTransformation2D transformation1 =
                new AffineTransformation2D(a, translation);

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
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final Point2D outputPoint = transformation1.transformAndReturnNew(inputPoint);
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

        final PROSACPointCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator(
                        this, inputPoints, outputPointsWithError, qualityScores);

        estimator.setThreshold(THRESHOLD);
        estimator.setResultRefined(false);
        estimator.setCovarianceKept(false);

        assertEquals(0, estimateStart);
        assertEquals(0, estimateEnd);
        assertEquals(0, estimateNextIteration);
        assertEquals(0, estimateProgressChange);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());

        final AffineTransformation2D transformation2 = estimator.estimate();

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
        for (int i = 0; i < nPoints; i++) {
            p1 = outputPoints.get(i);
            p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
            assertEquals(0.0, p1.distanceTo(p2), ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testEstimateWithRefinement() throws WrongSizeException,
            DecomposerException, LockedException, NotReadyException,
            RobustEstimatorException {
        // create an affine transformation
        Matrix a;
        do {
            // ensure A matrix is invertible
            a = Matrix.createWithUniformRandomValues(
                    AffineTransformation2D.INHOM_COORDS,
                    AffineTransformation2D.INHOM_COORDS, -1.0, 1.0);
            final double norm = Utils.normF(a);
            // normalize T to increase accuracy
            a.multiplyByScalar(1.0 / norm);
        } while (Utils.rank(a) < AffineTransformation2D.INHOM_COORDS);

        final double[] translation = new double[
                AffineTransformation2D.INHOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(translation, -1.0, 1.0);

        final AffineTransformation2D transformation1 =
                new AffineTransformation2D(a, translation);

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
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final Point2D outputPoint = transformation1.transformAndReturnNew(inputPoint);
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

        final PROSACPointCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceAffineTransformation2DRobustEstimator(
                        this, inputPoints, outputPointsWithError, qualityScores);

        estimator.setThreshold(THRESHOLD);
        estimator.setResultRefined(true);
        estimator.setCovarianceKept(true);
        estimator.setComputeAndKeepInliersEnabled(true);
        estimator.setComputeAndKeepResidualsEnabled(true);

        assertEquals(0, estimateStart);
        assertEquals(0, estimateEnd);
        assertEquals(0, estimateNextIteration);
        assertEquals(0, estimateProgressChange);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());

        final AffineTransformation2D transformation2 = estimator.estimate();

        assertNotNull(estimator.getInliersData());
        assertNotNull(estimator.getInliersData().getInliers());
        assertNotNull(estimator.getInliersData().getResiduals());
        assertTrue(estimator.getInliersData().getNumInliers() > 0);
        assertNotNull(estimator.getCovariance());
        assertEquals(estimator.getCovariance().getRows(),
                AffineTransformation2D.INHOM_COORDS *
                        AffineTransformation2D.INHOM_COORDS +
                        AffineTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(estimator.getCovariance().getColumns(),
                AffineTransformation2D.INHOM_COORDS *
                        AffineTransformation2D.INHOM_COORDS +
                        AffineTransformation2D.NUM_TRANSLATION_COORDS);

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
        for (int i = 0; i < nPoints; i++) {
            p1 = outputPoints.get(i);
            p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
            assertEquals(0.0, p1.distanceTo(p2), ABSOLUTE_ERROR);
        }
    }

    @Override
    public void onEstimateStart(
            final AffineTransformation2DRobustEstimator estimator) {
        estimateStart++;
        checkLocked((PROSACPointCorrespondenceAffineTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(
            final AffineTransformation2DRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((PROSACPointCorrespondenceAffineTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(
            final AffineTransformation2DRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        checkLocked((PROSACPointCorrespondenceAffineTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(
            final AffineTransformation2DRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        checkLocked((PROSACPointCorrespondenceAffineTransformation2DRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void checkLocked(
            final PROSACPointCorrespondenceAffineTransformation2DRobustEstimator estimator) {
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
            estimator.setThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            final double[] qualityScores = new double[
                    PointCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE];
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
        assertTrue(estimator.isLocked());
    }
}
