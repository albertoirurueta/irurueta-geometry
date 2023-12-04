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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.AffineTransformation3D;
import com.irurueta.geometry.Plane;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimatorTest
        implements AffineTransformation3DRobustEstimatorListener {

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
        assertEquals(4, AffineTransformation3DRobustEstimator.MINIMUM_SIZE);
        assertEquals(0.05f, AffineTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0f);
        assertEquals(0.0f, AffineTransformation3DRobustEstimator.MIN_PROGRESS_DELTA, 0.0f);
        assertEquals(1.0f, AffineTransformation3DRobustEstimator.MAX_PROGRESS_DELTA, 0.0f);
        assertEquals(0.99, AffineTransformation3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(5000, AffineTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(0.0, AffineTransformation3DRobustEstimator.MIN_CONFIDENCE, 0.0);
        assertEquals(1.0, AffineTransformation3DRobustEstimator.MAX_CONFIDENCE, 0.0);
        assertEquals(1, AffineTransformation3DRobustEstimator.MIN_ITERATIONS);
        assertTrue(AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(AffineTransformation3DRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertEquals(RobustEstimatorMethod.PROMEDS,
                PlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_ROBUST_METHOD);
        assertEquals(1e-6,
                PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(0.0, PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.MIN_STOP_THRESHOLD,
                0.0);
    }

    @Test
    public void testConstructor() {
        // test constructor without arguments
        PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator =
                new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator();

        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test constructor with planes
        final List<Plane> inputPlanes = new ArrayList<>();
        final List<Plane> outputPlanes = new ArrayList<>();
        for (int i = 0; i < PointCorrespondenceAffineTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPlanes.add(new Plane());
            outputPlanes.add(new Plane());
        }

        estimator = new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                inputPlanes, outputPlanes);

        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final List<Plane> planesEmpty = new ArrayList<>();
        estimator = null;
        try {
            // not enough planes
            estimator = new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                    planesEmpty, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                    inputPlanes, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener
        estimator = new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                this);

        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());


        // test constructor with listener and points
        estimator = new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                this, inputPlanes, outputPlanes);

        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough points
            estimator = new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                    this, planesEmpty, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                    this, inputPlanes, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores
        final double[] qualityScores = new double[
                PlaneCorrespondenceAffineTransformation3DRobustEstimator.MINIMUM_SIZE];
        final double[] shortQualityScores = new double[1];

        estimator = new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                qualityScores);

        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                    shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with lines and quality scores
        estimator = new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                inputPlanes, outputPlanes, qualityScores);

        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough planes
            estimator = new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                    planesEmpty, planesEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                    inputPlanes, planesEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // not enough scores
            estimator = new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                    inputPlanes, outputPlanes, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener and quality scores
        estimator = new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                this, qualityScores);

        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                    this, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener and points
        estimator = new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                this, inputPlanes, outputPlanes, qualityScores);

        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(AffineTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough planes
            estimator = new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                    this, planesEmpty, planesEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                    this, inputPlanes, planesEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) {
        }
        try {
            // not enough scores
            estimator = new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                    this, inputPlanes, outputPlanes, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetStopThreshold() throws LockedException {
        final PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator =
                new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator();

        // check default value
        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);

        // set new value
        estimator.setStopThreshold(0.5);

        // check correctness
        assertEquals(0.5, estimator.getStopThreshold(), 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setStopThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator =
                new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator();

        // check default value
        assertNull(estimator.getQualityScores());

        // set new value
        double[] qualityScores = new double[
                PlaneCorrespondenceAffineTransformation3DRobustEstimator.MINIMUM_SIZE];
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
        final PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator =
                new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator();

        // check default value
        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_CONFIDENCE,
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
        final PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator =
                new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator();

        // check default value
        assertEquals(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
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
    public void testGetSetLinesAndIsReady() throws LockedException {
        final PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator =
                new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator();

        // check default values
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());

        // set new value
        final List<Plane> inputPlanes = new ArrayList<>();
        final List<Plane> outputPlanes = new ArrayList<>();
        for (int i = 0; i < PointCorrespondenceAffineTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPlanes.add(new Plane());
            outputPlanes.add(new Plane());
        }

        estimator.setPlanes(inputPlanes, outputPlanes);

        // check correctness
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertFalse(estimator.isReady());

        // if we set quality scores, then estimator becomes ready
        final double[] qualityScores = new double[
                PlaneCorrespondenceAffineTransformation3DRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);

        assertTrue(estimator.isReady());

        // Force IllegalArgumentException
        final List<Plane> planesEmpty = new ArrayList<>();
        try {
            // not enough planes
            estimator.setPlanes(planesEmpty, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator.setPlanes(planesEmpty, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListenerAndIsListenerAvailable() throws LockedException {
        final PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator =
                new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator();

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
        final PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator =
                new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator();

        // check default value
        assertEquals(AffineTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
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
        final PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator =
                new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator();

        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check correctness
        assertFalse(estimator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator =
                new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator();

        assertFalse(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(true);

        // check correctness
        assertTrue(estimator.isCovarianceKept());
    }

    @Test
    public void testEstimateWithoutRefinement() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            reset();

            // create an affine transformation
            Matrix a;
            do {
                // ensure A matrix is invertible
                a = Matrix.createWithUniformRandomValues(
                        AffineTransformation3D.INHOM_COORDS,
                        AffineTransformation3D.INHOM_COORDS, -1.0, 1.0);
                final double norm = Utils.normF(a);
                // normalize T to increase accuracy
                a.multiplyByScalar(1.0 / norm);
            } while (Utils.rank(a) < AffineTransformation3D.INHOM_COORDS);

            final double[] translation = new double[
                    AffineTransformation3D.INHOM_COORDS];
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(translation, -1.0, 1.0);

            final AffineTransformation3D transformation1 =
                    new AffineTransformation3D(a, translation);

            // generate random planes
            final int nPlanes = randomizer.nextInt(MIN_LINES, MAX_LINES);
            final List<Plane> inputPlanes = new ArrayList<>();
            final List<Plane> outputPlanes = new ArrayList<>();
            final List<Plane> outputPlanesWithError = new ArrayList<>();
            final double[] qualityScores = new double[nPlanes];
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            for (int i = 0; i < nPlanes; i++) {
                final Plane inputPlane = new Plane(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                final Plane outputPlane = transformation1.transformAndReturnNew(inputPlane);
                final Plane outputPlaneWithError;
                final double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR,
                        MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // line is outlier
                    final double errorA = errorRandomizer.nextDouble();
                    final double errorB = errorRandomizer.nextDouble();
                    final double errorC = errorRandomizer.nextDouble();
                    final double errorD = errorRandomizer.nextDouble();
                    outputPlaneWithError = new Plane(outputPlane.getA() + errorA,
                            outputPlane.getB() + errorB,
                            outputPlane.getC() + errorC,
                            outputPlane.getD() + errorD);
                    final double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    // inlier line (without error)
                    outputPlaneWithError = outputPlane;
                }

                inputPlanes.add(inputPlane);
                outputPlanes.add(outputPlane);
                outputPlanesWithError.add(outputPlaneWithError);
            }

            final PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator =
                    new PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                            this, inputPlanes, outputPlanesWithError, qualityScores);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final AffineTransformation3D transformation2 = estimator.estimate();

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() == null) {
                continue;
            }
            assertNotNull(estimator.getCovariance());
            assertEquals(estimator.getCovariance().getRows(),
                    AffineTransformation3D.INHOM_COORDS *
                            AffineTransformation3D.INHOM_COORDS +
                            AffineTransformation3D.NUM_TRANSLATION_COORDS);
            assertEquals(estimator.getCovariance().getColumns(),
                    AffineTransformation3D.INHOM_COORDS *
                            AffineTransformation3D.INHOM_COORDS +
                            AffineTransformation3D.NUM_TRANSLATION_COORDS);


            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by transforming input planes
            // using estimated transformation (transformation2) and checking
            // that output planes are equal to the original output planes without
            // error
            Plane p1, p2;
            for (int i = 0; i < nPlanes; i++) {
                p1 = outputPlanes.get(i);
                p2 = transformation2.transformAndReturnNew(inputPlanes.get(i));
                p1.normalize();
                p2.normalize();
                assertEquals(0.0,
                        PlaneCorrespondenceAffineTransformation3DRobustEstimator.getResidual(p1, p2),
                        ABSOLUTE_ERROR);
                assertTrue(p1.equals(p2, ABSOLUTE_ERROR));
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    @Override
    public void onEstimateStart(final AffineTransformation3DRobustEstimator estimator) {
        estimateStart++;
        testLocked((PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(
            final AffineTransformation3DRobustEstimator estimator) {
        estimateEnd++;
        testLocked((PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(
            final AffineTransformation3DRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        testLocked((PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(
            final AffineTransformation3DRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        testLocked((PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator) estimator);
    }

    private void testLocked(
            final PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator) {
        final List<Plane> planes = new ArrayList<>();
        try {
            estimator.setPlanes(planes, planes);
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
        } catch (final Exception ignore) {
            fail("LockedException expected but not thrown");
        }
        assertTrue(estimator.isLocked());
    }
}
