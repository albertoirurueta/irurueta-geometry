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

import com.irurueta.algebra.*;
import com.irurueta.geometry.AffineTransformation2D;
import com.irurueta.geometry.Line2D;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class PROMedSLineCorrespondenceAffineTransformation2DRobustEstimatorTest
        implements AffineTransformation2DRobustEstimatorListener {

    private static final double MIN_RANDOM_VALUE = -1000.0;
    private static final double MAX_RANDOM_VALUE = 1000.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int MIN_LINES = 500;
    private static final int MAX_LINES = 1000;

    private static final double THRESHOLD = 1e-6;

    private static final double STD_ERROR = 100.0;

    private static final double MIN_SCORE_ERROR = -0.3;
    private static final double MAX_SCORE_ERROR = 0.3;

    private static final int PERCENTAGE_OUTLIER = 20;

    private static final int TIMES = 100;

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
        assertEquals(RobustEstimatorMethod.PROMedS,
                LineCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_ROBUST_METHOD);
        assertEquals(1e-6,
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(0.0, PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.MIN_STOP_THRESHOLD,
                0.0);
    }

    @Test
    public void testConstructor() {
        // test constructor without arguments
        PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator();

        assertEquals(estimator.getStopThreshold(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());


        // test constructor with lines
        final List<Line2D> inputLines = new ArrayList<>();
        final List<Line2D> outputLines = new ArrayList<>();
        for (int i = 0; i < PointCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputLines.add(new Line2D());
            outputLines.add(new Line2D());
        }

        estimator = new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                inputLines, outputLines);

        assertEquals(estimator.getStopThreshold(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final List<Line2D> linesEmpty = new ArrayList<>();
        estimator = null;
        try {
            // not enough lines
            estimator = new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                    linesEmpty, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                    inputLines, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener
        estimator = new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                this);

        assertEquals(estimator.getStopThreshold(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());


        // test constructor with listener and points
        estimator = new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                this, inputLines, outputLines);

        assertEquals(estimator.getStopThreshold(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough points
            estimator = new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                    this, linesEmpty, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                    this, inputLines, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores
        final double[] qualityScores = new double[
                PointCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE];
        final double[] shortQualityScores = new double[1];

        estimator = new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                qualityScores);

        assertEquals(estimator.getStopThreshold(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                    shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with lines and quality scores
        estimator = new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                inputLines, outputLines, qualityScores);

        assertEquals(estimator.getStopThreshold(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough lines
            estimator = new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                    linesEmpty, linesEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                    inputLines, linesEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // not enough scores
            estimator = new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                    inputLines, outputLines, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener and quality scores
        estimator = new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                this, qualityScores);

        assertEquals(estimator.getStopThreshold(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                    this, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener and points
        estimator = new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                this, inputLines, outputLines, qualityScores);

        assertEquals(estimator.getStopThreshold(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough line
            estimator = new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                    this, linesEmpty, linesEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                    this, inputLines, linesEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // not enough scores
            estimator = new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                    this, inputLines, outputLines, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetStopThreshold() throws LockedException {
        final PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator();

        // check default value
        assertEquals(estimator.getStopThreshold(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_STOP_THRESHOLD, 0.0);

        // set new value
        estimator.setStopThreshold(0.5);

        // check correctness
        assertEquals(estimator.getStopThreshold(), 0.5, 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setStopThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator();

        // check default value
        assertNull(estimator.getQualityScores());

        // set new value
        double[] qualityScores = new double[
                LineCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertSame(estimator.getQualityScores(), qualityScores);

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
        final PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator();

        // check default value
        assertEquals(estimator.getConfidence(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
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
        final PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator();

        // check default value
        assertEquals(estimator.getMaxIterations(),
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.
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
    public void testGetSetLinesAndIsReady() throws LockedException {
        final PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator();

        // check default values
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());

        // set new value
        final List<Line2D> inputLines = new ArrayList<>();
        final List<Line2D> outputLines = new ArrayList<>();
        for (int i = 0; i < PointCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputLines.add(new Line2D());
            outputLines.add(new Line2D());
        }

        estimator.setLines(inputLines, outputLines);

        // check correctness
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertFalse(estimator.isReady());

        // if we set quality scores, then estimator becomes ready
        final double[] qualityScores = new double[
                LineCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);

        assertTrue(estimator.isReady());

        // Force IllegalArgumentException
        final List<Line2D> linesEmpty = new ArrayList<>();
        try {
            // not enough lines
            estimator.setLines(linesEmpty, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator.setLines(linesEmpty, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListenerAndIsListenerAvailable() throws LockedException {
        final PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator();

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
    public void testGetSetProgressDelta() throws LockedException {
        final PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator();

        // check default value
        assertEquals(estimator.getProgressDelta(),
                AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
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
        final PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator();

        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check correctness
        assertFalse(estimator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator();

        assertFalse(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(true);

        // check correctness
        assertTrue(estimator.isCovarianceKept());
    }

    @Test
    public void testEstimateWithoutRefinement() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        for (int t = 0; t < TIMES; t++) {
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

            // generate random lines
            final int nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);
            final List<Line2D> inputLines = new ArrayList<>();
            final List<Line2D> outputLines = new ArrayList<>();
            final List<Line2D> outputLinesWithError = new ArrayList<>();
            final double[] qualityScores = new double[nLines];
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            for (int i = 0; i < nLines; i++) {
                final Line2D inputLine = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                final Line2D outputLine = transformation1.transformAndReturnNew(inputLine);
                final Line2D outputLineWithError;
                final double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR,
                        MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // line is outlier
                    final double errorA = errorRandomizer.nextDouble();
                    final double errorB = errorRandomizer.nextDouble();
                    final double errorC = errorRandomizer.nextDouble();
                    outputLineWithError = new Line2D(outputLine.getA() + errorA,
                            outputLine.getB() + errorB,
                            outputLine.getC() + errorC);
                    final double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    // inlier line (without error)
                    outputLineWithError = outputLine;
                }

                inputLines.add(inputLine);
                outputLines.add(outputLine);
                outputLinesWithError.add(outputLineWithError);
            }

            final PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                    new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                            this, inputLines, outputLinesWithError, qualityScores);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final AffineTransformation2D transformation2 = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by transforming input lines
            // using estimated transformation (transformation2) and checking
            // that output lines are equal to the original output lines without
            // error
            Line2D l1, l2;
            for (int i = 0; i < nLines; i++) {
                l1 = outputLines.get(i);
                l2 = transformation2.transformAndReturnNew(inputLines.get(i));
                l1.normalize();
                l2.normalize();
                assertEquals(
                        LineCorrespondenceAffineTransformation2DRobustEstimator.
                                getResidual(l1, l2), 0.0, ABSOLUTE_ERROR);
                assertTrue(l1.equals(l2, ABSOLUTE_ERROR));
            }
        }
    }

    @Test
    public void testEstimateWithRefinement() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        for (int t = 0; t < TIMES; t++) {
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

            // generate random lines
            final int nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);
            final List<Line2D> inputLines = new ArrayList<>();
            final List<Line2D> outputLines = new ArrayList<>();
            final List<Line2D> outputLinesWithError = new ArrayList<>();
            final double[] qualityScores = new double[nLines];
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            for (int i = 0; i < nLines; i++) {
                final Line2D inputLine = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                final Line2D outputLine = transformation1.transformAndReturnNew(inputLine);
                final Line2D outputLineWithError;
                final double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR,
                        MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // line is outlier
                    final double errorA = errorRandomizer.nextDouble();
                    final double errorB = errorRandomizer.nextDouble();
                    final double errorC = errorRandomizer.nextDouble();
                    outputLineWithError = new Line2D(outputLine.getA() + errorA,
                            outputLine.getB() + errorB,
                            outputLine.getC() + errorC);
                    final double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    // inlier line (without error)
                    outputLineWithError = outputLine;
                }

                inputLines.add(inputLine);
                outputLines.add(outputLine);
                outputLinesWithError.add(outputLineWithError);
            }

            final PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                    new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                            this, inputLines, outputLinesWithError, qualityScores);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final AffineTransformation2D transformation2 = estimator.estimate();

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                assertEquals(estimator.getCovariance().getRows(),
                        AffineTransformation2D.INHOM_COORDS *
                                AffineTransformation2D.INHOM_COORDS +
                                AffineTransformation2D.NUM_TRANSLATION_COORDS);
                assertEquals(estimator.getCovariance().getColumns(),
                        AffineTransformation2D.INHOM_COORDS *
                                AffineTransformation2D.INHOM_COORDS +
                                AffineTransformation2D.NUM_TRANSLATION_COORDS);
            }

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by transforming input lines
            // using estimated transformation (transformation2) and checking
            // that output lines are equal to the original output lines without
            // error
            Line2D l1, l2;
            for (int i = 0; i < nLines; i++) {
                l1 = outputLines.get(i);
                l2 = transformation2.transformAndReturnNew(inputLines.get(i));
                l1.normalize();
                l2.normalize();
                assertEquals(
                        LineCorrespondenceAffineTransformation2DRobustEstimator.
                                getResidual(l1, l2), 0.0, ABSOLUTE_ERROR);
                assertTrue(l1.equals(l2, ABSOLUTE_ERROR));
            }
        }
    }

    @Override
    public void onEstimateStart(final AffineTransformation2DRobustEstimator estimator) {
        estimateStart++;
        checkLocked((PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(
            final AffineTransformation2DRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(
            final AffineTransformation2DRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        checkLocked((PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(
            final AffineTransformation2DRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        checkLocked((PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void checkLocked(
            final PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator estimator) {
        final List<Line2D> lines = new ArrayList<>();
        try {
            estimator.setLines(lines, lines);
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
