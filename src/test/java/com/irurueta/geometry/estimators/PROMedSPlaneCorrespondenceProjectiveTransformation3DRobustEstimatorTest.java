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
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimatorTest
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
    void testConstants() {
        assertEquals(5, ProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE);
        assertEquals(0.05f, ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0f);
        assertEquals(0.0f, ProjectiveTransformation3DRobustEstimator.MIN_PROGRESS_DELTA, 0.0f);
        assertEquals(1.0f, ProjectiveTransformation3DRobustEstimator.MAX_PROGRESS_DELTA, 0.0f);
        assertEquals(0.99, ProjectiveTransformation3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(5000, ProjectiveTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(0.0, ProjectiveTransformation3DRobustEstimator.MIN_CONFIDENCE, 0.0);
        assertEquals(1.0, ProjectiveTransformation3DRobustEstimator.MAX_CONFIDENCE, 0.0);
        assertEquals(1, ProjectiveTransformation3DRobustEstimator.MIN_ITERATIONS);
        assertTrue(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(ProjectiveTransformation3DRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertEquals(RobustEstimatorMethod.PROMEDS,
                PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_ROBUST_METHOD);
        assertEquals(1e-6,
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(0.0,
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.MIN_STOP_THRESHOLD, 0.0);
    }

    @Test
    void testConstructor() {
        // test constructor without arguments
        var estimator = new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test constructor with points
        final var inputPlanes = new ArrayList<Plane>();
        final var outputPlanes = new ArrayList<Plane>();
        for (var i = 0; i < PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPlanes.add(new Plane());
            outputPlanes.add(new Plane());
        }

        estimator = new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(inputPlanes, outputPlanes);

        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final var planesEmpty = new ArrayList<Plane>();
        // not enough points
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(planesEmpty,
                        planesEmpty));
        // different sizes
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(inputPlanes,
                        planesEmpty));

        // test constructor with listener
        estimator = new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(this);

        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test constructor with listener and points
        estimator = new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(this, inputPlanes,
                outputPlanes);

        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        // not enough points
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(this, planesEmpty,
                        planesEmpty));
        // different sizes
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(this, inputPlanes,
                        planesEmpty));

        // test constructor with quality scores
        final var qualityScores = new double[
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE];
        final var shortQualityScores = new double[1];

        estimator = new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(qualityScores);

        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(shortQualityScores));

        // test constructor with planes and quality scores
        estimator = new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(inputPlanes, outputPlanes,
                qualityScores);

        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        // not enough planes
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(planesEmpty, planesEmpty,
                        qualityScores));
        // different sizes
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(inputPlanes, planesEmpty,
                        qualityScores));
        // not enough scores
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(inputPlanes, outputPlanes,
                        shortQualityScores));

        // test constructor with listener and quality scores
        estimator = new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(this,
                qualityScores);

        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(this,
                        shortQualityScores));

        // test constructor with listener and planes
        estimator = new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(this, inputPlanes,
                outputPlanes, qualityScores);

        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        // not enough points
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(this, planesEmpty,
                        planesEmpty, qualityScores));
        // different sizes
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(this, inputPlanes,
                        planesEmpty, qualityScores));
        // not enough scores
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(this, inputPlanes,
                        outputPlanes, shortQualityScores));
    }

    @Test
    void testGetSetThreshold() throws LockedException {
        final var estimator = new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        // check default value
        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_STOP_THRESHOLD,
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
        final var estimator = new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        // check default value
        assertNull(estimator.getQualityScores());

        // set new value
        final var qualityScores = new double[PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        final var qualityScores2 = new double[1];
        assertThrows(IllegalArgumentException.class, () -> estimator.setQualityScores(qualityScores2));
    }

    @Test
    void testGetSetConfidence() throws LockedException {
        final var estimator = new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        // check default value
        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_CONFIDENCE,
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
        final var estimator = new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        // check default value
        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(10);

        // check correctness
        assertEquals(10, estimator.getMaxIterations());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setMaxIterations(0));
    }

    @Test
    void testGetSetPlanesAndIsReady() throws LockedException {
        final var estimator = new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        // check default values
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());

        // set new value
        final var inputPlanes = new ArrayList<Plane>();
        final var outputPlanes = new ArrayList<Plane>();
        for (var i = 0; i < PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPlanes.add(new Plane());
            outputPlanes.add(new Plane());
        }

        estimator.setPlanes(inputPlanes, outputPlanes);

        // check correctness
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertFalse(estimator.isReady());

        // if we set quality scores, then estimator becomes ready
        final var qualityScores = new double[PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);

        assertTrue(estimator.isReady());

        // Force IllegalArgumentException
        final var pointsEmpty = new ArrayList<Plane>();
        // not enough lines
        assertThrows(IllegalArgumentException.class, () -> estimator.setPlanes(pointsEmpty, pointsEmpty));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> estimator.setPlanes(pointsEmpty, pointsEmpty));
    }

    @Test
    void testGetSetListenerAndIsListenerAvailable() throws LockedException {
        final var estimator = new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

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
        final var estimator = new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        // check default value
        assertEquals(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);

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
        final var estimator = new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check correctness
        assertFalse(estimator.isResultRefined());
    }

    @Test
    void testIsSetCovarianceKept() throws LockedException {
        final var estimator = new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        assertFalse(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(true);

        // check correctness
        assertTrue(estimator.isCovarianceKept());
    }

    @Test
    void testEstimateWithoutRefinement() throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        for (var t = 0; t < TIMES; t++) {
            // create an affine transformation
            Matrix a;
            do {
                // ensure A matrix is invertible
                a = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.INHOM_COORDS,
                        ProjectiveTransformation3D.INHOM_COORDS, -1.0, 1.0);
                final var norm = Utils.normF(a);
                // normalize T to increase accuracy
                a.multiplyByScalar(1.0 / norm);
            } while (Utils.rank(a) < ProjectiveTransformation3D.INHOM_COORDS);

            final var translation = new double[ProjectiveTransformation3D.INHOM_COORDS];
            final var randomizer = new UniformRandomizer();
            randomizer.fill(translation, -1.0, 1.0);

            final var transformation1 = new ProjectiveTransformation3D(a, translation);

            // generate random planes
            final var nPlanes = randomizer.nextInt(MIN_LINES, MAX_LINES);
            final var inputPlanes = new ArrayList<Plane>();
            final var outputPlanes = new ArrayList<Plane>();
            final var outputPlanesWithError = new ArrayList<Plane>();
            final var qualityScores = new double[nPlanes];
            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            for (var i = 0; i < nPlanes; i++) {
                final var inputPlane = new Plane(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                final var outputPlane = transformation1.transformAndReturnNew(inputPlane);
                final var scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;
                final Plane outputPlaneWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // plane is outlier
                    final var errorA = errorRandomizer.nextDouble();
                    final var errorB = errorRandomizer.nextDouble();
                    final var errorC = errorRandomizer.nextDouble();
                    final var errorD = errorRandomizer.nextDouble();
                    outputPlaneWithError = new Plane(
                            outputPlane.getA() + errorA,
                            outputPlane.getB() + errorB,
                            outputPlane.getC() + errorC,
                            outputPlane.getD() + errorD);
                    final var error = Math.sqrt(errorA * errorA + errorB * errorB + errorC * errorC + errorD * errorD);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    // inlier plane (without error)
                    outputPlaneWithError = outputPlane;
                }

                inputPlanes.add(inputPlane);
                outputPlanes.add(outputPlane);
                outputPlanesWithError.add(outputPlaneWithError);
            }

            final var estimator = new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(this,
                    inputPlanes, outputPlanesWithError, qualityScores);

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

            // check correctness of estimation by transforming input planes
            // using estimated transformation (transformation2) and checking
            // that output planes are equal to the original output planes without
            // error
            for (var i = 0; i < nPlanes; i++) {
                final var plane1 = outputPlanes.get(i);
                final var plane2 = transformation2.transformAndReturnNew(inputPlanes.get(i));
                plane1.normalize();
                plane2.normalize();
                assertEquals(0.0,
                        PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.getResidual(plane1, plane2),
                        2.0 * ABSOLUTE_ERROR);
                assertTrue(plane1.equals(plane2, ABSOLUTE_ERROR));
            }
        }
    }

    @Test
    void testEstimateWithRefinement() throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        for (var t = 0; t < TIMES; t++) {
            // create an affine transformation
            Matrix a;
            do {
                // ensure A matrix is invertible
                a = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.INHOM_COORDS,
                        ProjectiveTransformation3D.INHOM_COORDS, -1.0, 1.0);
                final var norm = Utils.normF(a);
                // normalize T to increase accuracy
                a.multiplyByScalar(1.0 / norm);
            } while (Utils.rank(a) < ProjectiveTransformation3D.INHOM_COORDS);

            final var translation = new double[ProjectiveTransformation3D.INHOM_COORDS];
            final var randomizer = new UniformRandomizer();
            randomizer.fill(translation, -1.0, 1.0);

            final var transformation1 = new ProjectiveTransformation3D(a, translation);

            // generate random planes
            final var nPlanes = randomizer.nextInt(MIN_LINES, MAX_LINES);
            final var inputPlanes = new ArrayList<Plane>();
            final var outputPlanes = new ArrayList<Plane>();
            final var outputPlanesWithError = new ArrayList<Plane>();
            final var qualityScores = new double[nPlanes];
            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            for (var i = 0; i < nPlanes; i++) {
                final var inputPlane = new Plane(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                final var outputPlane = transformation1.transformAndReturnNew(inputPlane);
                final var scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;
                final Plane outputPlaneWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // plane is outlier
                    final var errorA = errorRandomizer.nextDouble();
                    final var errorB = errorRandomizer.nextDouble();
                    final var errorC = errorRandomizer.nextDouble();
                    final var errorD = errorRandomizer.nextDouble();
                    outputPlaneWithError = new Plane(
                            outputPlane.getA() + errorA,
                            outputPlane.getB() + errorB,
                            outputPlane.getC() + errorC,
                            outputPlane.getD() + errorD);
                    final var error = Math.sqrt(errorA * errorA + errorB * errorB + errorC * errorC + errorD * errorD);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    // inlier plane (without error)
                    outputPlaneWithError = outputPlane;
                }

                inputPlanes.add(inputPlane);
                outputPlanes.add(outputPlane);
                outputPlanesWithError.add(outputPlaneWithError);
            }

            final var estimator = new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(this,
                    inputPlanes, outputPlanesWithError, qualityScores);

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
            if (estimator.getCovariance() != null) {
                assertEquals(estimator.getCovariance().getRows(),
                        ProjectiveTransformation3D.HOM_COORDS * ProjectiveTransformation3D.HOM_COORDS);
                assertEquals(estimator.getCovariance().getColumns(),
                        ProjectiveTransformation3D.HOM_COORDS * ProjectiveTransformation3D.HOM_COORDS);
            }

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by transforming input planes
            // using estimated transformation (transformation2) and checking
            // that output planes are equal to the original output planes without
            // error
            for (var i = 0; i < nPlanes; i++) {
                final var plane1 = outputPlanes.get(i);
                final var plane2 = transformation2.transformAndReturnNew(inputPlanes.get(i));
                plane1.normalize();
                plane2.normalize();
                assertEquals(0.0,
                        PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.getResidual(plane1, plane2),
                        2.0 * ABSOLUTE_ERROR);
                assertTrue(plane1.equals(plane2, ABSOLUTE_ERROR));
            }
        }
    }

    @Override
    public void onEstimateStart(final ProjectiveTransformation3DRobustEstimator estimator) {
        estimateStart++;
        checkLocked((PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final ProjectiveTransformation3DRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(
            final ProjectiveTransformation3DRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        checkLocked((PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(
            final ProjectiveTransformation3DRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        checkLocked((PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = estimateProgressChange = 0;
    }

    private static void checkLocked(
            final PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator) {
        final var planes = new ArrayList<Plane>();
        assertThrows(LockedException.class, () -> estimator.setPlanes(planes, planes));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setProgressDelta(0.01f));
        assertThrows(LockedException.class, () -> estimator.setStopThreshold(0.5));
        assertThrows(LockedException.class, () -> estimator.setConfidence(0.5));
        assertThrows(LockedException.class, () -> estimator.setMaxIterations(10));
        assertThrows(LockedException.class, estimator::estimate);
        assertTrue(estimator.isLocked());
    }
}
