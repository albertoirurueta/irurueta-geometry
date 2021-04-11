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
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.ProjectiveTransformation3D;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimatorTest
        implements ProjectiveTransformation3DRobustEstimatorListener {

    private static final double MIN_RANDOM_VALUE = -1000.0;
    private static final double MAX_RANDOM_VALUE = 1000.0;

    private static final double ABSOLUTE_ERROR = 5e-6;

    private static final int MIN_LINES = 500;
    private static final int MAX_LINES = 1000;

    private static final double THRESHOLD = 1e-6;

    private static final double STD_ERROR = 100.0;

    private static final double MIN_SCORE_ERROR = -0.3;
    private static final double MAX_SCORE_ERROR = 0.3;

    private static final int PERCENTAGE_OUTLIER = 20;

    private static final int TIMES = 10;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    public void testConstructor() {
        // test constructor without arguments
        PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        assertEquals(estimator.getThreshold(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());


        // test constructor with points
        final List<Plane> inputPlanes = new ArrayList<>();
        final List<Plane> outputPlanes = new ArrayList<>();
        for (int i = 0; i < PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPlanes.add(new Plane());
            outputPlanes.add(new Plane());
        }

        estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                inputPlanes, outputPlanes);

        assertEquals(estimator.getThreshold(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        final List<Plane> planesEmpty = new ArrayList<>();
        estimator = null;
        try {
            // not enough points
            estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    planesEmpty, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    inputPlanes, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor with listener
        estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                this);

        assertEquals(estimator.getThreshold(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());


        // test constructor with listener and points
        estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                this, inputPlanes, outputPlanes);

        assertEquals(estimator.getThreshold(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough points
            estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    this, planesEmpty, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    this, inputPlanes, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor with quality scores
        final double[] qualityScores = new double[
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE];
        final double[] shortQualityScores = new double[1];

        estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                qualityScores);

        assertEquals(estimator.getThreshold(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with planes and quality scores
        estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                inputPlanes, outputPlanes, qualityScores);

        assertEquals(estimator.getThreshold(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough planes
            estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    planesEmpty, planesEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    inputPlanes, planesEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // not enough scores
            estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    inputPlanes, outputPlanes, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener and quality scores
        estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                this, qualityScores);

        assertEquals(estimator.getThreshold(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    this, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor with listener and planes
        estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                this, inputPlanes, outputPlanes, qualityScores);

        assertEquals(estimator.getThreshold(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough points
            estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    this, planesEmpty, planesEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    this, inputPlanes, planesEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // not enough scores
            estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    this, inputPlanes, outputPlanes, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetThreshold() throws LockedException {
        final PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        // check default value
        assertEquals(estimator.getThreshold(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);

        // set new value
        estimator.setThreshold(0.5);

        // check correctness
        assertEquals(estimator.getThreshold(), 0.5, 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        // check default value
        assertNull(estimator.getQualityScores());

        // set new value
        double[] qualityScores = new double[
                PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE];
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
        final PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        // check default value
        assertEquals(estimator.getConfidence(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
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
        final PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        // check default value
        assertEquals(estimator.getMaxIterations(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
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
    public void testGetSetPlanesAndIsReady() throws LockedException {
        final PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        // check default values
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());

        // set new value
        final List<Plane> inputPlanes = new ArrayList<>();
        final List<Plane> outputPlanes = new ArrayList<>();
        for (int i = 0; i < PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPlanes.add(new Plane());
            outputPlanes.add(new Plane());
        }

        estimator.setPlanes(inputPlanes, outputPlanes);

        // check correctness
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertFalse(estimator.isReady());

        // if we set quality scores, then estimator becomes ready
        final double[] qualityScores = new double[
                PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);

        assertTrue(estimator.isReady());

        // Force IllegalArgumentException
        final List<Plane> pointsEmpty = new ArrayList<>();
        try {
            // not enough lines
            estimator.setPlanes(pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator.setPlanes(pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListenerAndIsListenerAvailable() throws LockedException {
        final PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

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
        final PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        // check default value
        assertEquals(estimator.getProgressDelta(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
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
        final PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check correctness
        assertFalse(estimator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        assertFalse(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(true);

        // check correctness
        assertTrue(estimator.isCovarianceKept());
    }

    @Test
    public void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        final PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        assertFalse(estimator.isComputeAndKeepInliersEnabled());

        // set new value
        estimator.setComputeAndKeepInliersEnabled(true);

        // check correctness
        assertTrue(estimator.isComputeAndKeepInliersEnabled());
    }

    @Test
    public void testIsSetComputeAndKeepResidualsEnabled()
            throws LockedException {
        final PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // set new value
        estimator.setComputeAndKeepResidualsEnabled(true);

        // check correctness
        assertTrue(estimator.isComputeAndKeepResidualsEnabled());
    }

    @Test
    public void testEstimateWithoutRefinement() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            // create an affine transformation
            Matrix a;
            do {
                // ensure A matrix is invertible
                a = Matrix.createWithUniformRandomValues(
                        ProjectiveTransformation3D.INHOM_COORDS,
                        ProjectiveTransformation3D.INHOM_COORDS, -1.0, 1.0);
                final double norm = Utils.normF(a);
                // normalize T to increase accuracy
                a.multiplyByScalar(1.0 / norm);
            } while (Utils.rank(a) < ProjectiveTransformation3D.INHOM_COORDS);

            final double[] translation = new double[
                    ProjectiveTransformation3D.INHOM_COORDS];
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(translation, -1.0, 1.0);

            final ProjectiveTransformation3D transformation1 =
                    new ProjectiveTransformation3D(a, translation);

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
                    // plane is outlier
                    final double errorA = errorRandomizer.nextDouble();
                    final double errorB = errorRandomizer.nextDouble();
                    final double errorC = errorRandomizer.nextDouble();
                    final double errorD = errorRandomizer.nextDouble();
                    outputPlaneWithError = new Plane(
                            outputPlane.getA() + errorA,
                            outputPlane.getB() + errorB,
                            outputPlane.getC() + errorC,
                            outputPlane.getD() + errorD);
                    final double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    // inlier plane (without error)
                    outputPlaneWithError = outputPlane;
                }

                inputPlanes.add(inputPlane);
                outputPlanes.add(outputPlane);
                outputPlanesWithError.add(outputPlaneWithError);
            }

            final PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                    new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                            this, inputPlanes, outputPlanesWithError, qualityScores);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final ProjectiveTransformation3D transformation2 = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by transforming input planes
            // using estimated transformation (transformation2) and checking
            // that output planes are equal to the original output planes without
            // error
            boolean failed = false;
            Plane plane1, plane2;
            for (int i = 0; i < nPlanes; i++) {
                plane1 = outputPlanes.get(i);
                plane2 = transformation2.transformAndReturnNew(inputPlanes.get(i));
                plane1.normalize();
                plane2.normalize();
                if (PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        getResidual(plane1, plane2) > ABSOLUTE_ERROR) {
                    failed = true;
                    break;
                }
                assertEquals(
                        PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                                getResidual(plane1, plane2), 0.0, ABSOLUTE_ERROR);
                assertTrue(plane1.equals(plane2, ABSOLUTE_ERROR));
            }

            if (failed) {
                continue;
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateWithRefinement() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            // create an affine transformation
            Matrix a;
            do {
                // ensure A matrix is invertible
                a = Matrix.createWithUniformRandomValues(
                        ProjectiveTransformation3D.INHOM_COORDS,
                        ProjectiveTransformation3D.INHOM_COORDS, -1.0, 1.0);
                final double norm = Utils.normF(a);
                // normalize T to increase accuracy
                a.multiplyByScalar(1.0 / norm);
            } while (Utils.rank(a) < ProjectiveTransformation3D.INHOM_COORDS);

            final double[] translation = new double[
                    ProjectiveTransformation3D.INHOM_COORDS];
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(translation, -1.0, 1.0);

            final ProjectiveTransformation3D transformation1 =
                    new ProjectiveTransformation3D(a, translation);

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
                    // plane is outlier
                    final double errorA = errorRandomizer.nextDouble();
                    final double errorB = errorRandomizer.nextDouble();
                    final double errorC = errorRandomizer.nextDouble();
                    final double errorD = errorRandomizer.nextDouble();
                    outputPlaneWithError = new Plane(
                            outputPlane.getA() + errorA,
                            outputPlane.getB() + errorB,
                            outputPlane.getC() + errorC,
                            outputPlane.getD() + errorD);
                    final double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    // inlier plane (without error)
                    outputPlaneWithError = outputPlane;
                }

                inputPlanes.add(inputPlane);
                outputPlanes.add(outputPlane);
                outputPlanesWithError.add(outputPlaneWithError);
            }

            final PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                    new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                            this, inputPlanes, outputPlanesWithError, qualityScores);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final ProjectiveTransformation3D transformation2 = estimator.estimate();

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                assertEquals(estimator.getCovariance().getRows(),
                        ProjectiveTransformation3D.HOM_COORDS *
                                ProjectiveTransformation3D.HOM_COORDS);
                assertEquals(estimator.getCovariance().getColumns(),
                        ProjectiveTransformation3D.HOM_COORDS *
                                ProjectiveTransformation3D.HOM_COORDS);
            }

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by transforming input planes
            // using estimated transformation (transformation2) and checking
            // that output planes are equal to the original output planes without
            // error
            Plane plane1;
            Plane plane2;
            boolean failed = false;
            for (int i = 0; i < nPlanes; i++) {
                plane1 = outputPlanes.get(i);
                plane2 = transformation2.transformAndReturnNew(inputPlanes.get(i));
                plane1.normalize();
                plane2.normalize();
                if (Math.abs(PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        getResidual(plane1, plane2)) > ABSOLUTE_ERROR) {
                    failed = true;
                    break;
                }
                assertEquals(
                        PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                                getResidual(plane1, plane2), 0.0, ABSOLUTE_ERROR);
                assertTrue(plane1.equals(plane2, ABSOLUTE_ERROR));
            }

            if (failed) {
                continue;
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onEstimateStart(final ProjectiveTransformation3DRobustEstimator estimator) {
        estimateStart++;
        checkLocked((PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(
            final ProjectiveTransformation3DRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(
            final ProjectiveTransformation3DRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        checkLocked((PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(
            final ProjectiveTransformation3DRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        checkLocked((PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void checkLocked(
            final PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator) {
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
            estimator.setThreshold(0.5);
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
