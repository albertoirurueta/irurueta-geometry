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
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.ProjectiveTransformation3D;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimatorTest
        implements ProjectiveTransformation3DRobustEstimatorListener {

    private static final double MIN_RANDOM_VALUE = -1000.0;
    private static final double MAX_RANDOM_VALUE = 1000.0;

    private static final double ABSOLUTE_ERROR = 5e-6;

    private static final int MIN_LINES = 500;
    private static final int MAX_LINES = 1000;

    private static final double THRESHOLD = 1e-6;

    private static final double STD_ERROR = 100.0;

    private static final int PERCENTAGE_OUTLIER = 20;

    private static final int TIMES = 10;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    void testConstants() {
        assertEquals(1e-6, MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(0.0, MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.MIN_THRESHOLD,
                0.0);
    }

    @Test
    void testConstructor() {
        // test constructor without arguments
        var estimator = new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        assertEquals(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
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

        // test constructor with planes
        final var inputPlanes = new ArrayList<Plane>();
        final var outputPlanes = new ArrayList<Plane>();
        for (var i = 0; i < PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPlanes.add(new Plane());
            outputPlanes.add(new Plane());
        }

        estimator = new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(inputPlanes, outputPlanes);

        assertEquals(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
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
        final var planesEmpty = new ArrayList<Plane>();
        // not enough planes
        assertThrows(IllegalArgumentException.class,
                () -> new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(planesEmpty, planesEmpty));
        // different sizes
        assertThrows(IllegalArgumentException.class,
                () -> new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(inputPlanes, planesEmpty));

        // test constructor with listener
        estimator = new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(this);

        assertEquals(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test constructor with listener and lines
        estimator = new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(this, inputPlanes,
                outputPlanes);

        assertEquals(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
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
                () -> new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(this, planesEmpty,
                        planesEmpty));
        // different sizes
        assertThrows(IllegalArgumentException.class,
                () -> new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(this, inputPlanes,
                        planesEmpty));
    }

    @Test
    void testGetSetThreshold() throws LockedException {
        final var estimator = new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        // check default value
        assertEquals(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);

        // set new value
        estimator.setThreshold(0.5);

        // check correctness
        assertEquals(0.5, estimator.getThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setThreshold(0.0));
    }

    @Test
    void testGetSetConfidence() throws LockedException {
        final var estimator = new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        // check default value
        assertEquals(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_CONFIDENCE,
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
        final var estimator = new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        // check default value
        assertEquals(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
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
        final var estimator = new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

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
        assertTrue(estimator.isReady());

        // Force IllegalArgumentException
        final var planesEmpty = new ArrayList<Plane>();
        // not enough planes
        assertThrows(IllegalArgumentException.class, () -> estimator.setPlanes(planesEmpty, planesEmpty));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> estimator.setPlanes(planesEmpty, planesEmpty));
    }

    @Test
    void testGetSetListenerAndIsListenerAvailable() throws LockedException {
        final var estimator = new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

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
        final var estimator = new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        // check default value
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
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
        final var estimator = new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check correctness
        assertFalse(estimator.isResultRefined());
    }

    @Test
    void testIsSetCovarianceKept() throws LockedException {
        final var estimator = new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        assertFalse(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(true);

        // check correctness
        assertTrue(estimator.isCovarianceKept());
    }

    @Test
    void testEstimateWithoutRefinement() throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        // create an affine transformation
        Matrix a;
        do {
            // ensure A matrix is invertible
            a = Matrix.createWithUniformRandomValues(
                    ProjectiveTransformation3D.INHOM_COORDS,
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
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
        for (var i = 0; i < nPlanes; i++) {
            final var inputPlane = new Plane(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var outputPlane = transformation1.transformAndReturnNew(inputPlane);
            final Plane outputPlaneWithError;
            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                // line is outlier
                final var errorA = errorRandomizer.nextDouble();
                final var errorB = errorRandomizer.nextDouble();
                final var errorC = errorRandomizer.nextDouble();
                final var errorD = errorRandomizer.nextDouble();
                outputPlaneWithError = new Plane(
                        outputPlane.getA() + errorA,
                        outputPlane.getB() + errorB,
                        outputPlane.getC() + errorC,
                        outputPlane.getD() + errorD);
            } else {
                // inlier line (without error)
                outputPlaneWithError = outputPlane;
            }

            inputPlanes.add(inputPlane);
            outputPlanes.add(outputPlane);
            outputPlanesWithError.add(outputPlaneWithError);
        }

        final var estimator = new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(this,
                inputPlanes, outputPlanesWithError);

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
                    ABSOLUTE_ERROR);
            assertTrue(plane1.equals(plane2, ABSOLUTE_ERROR));
        }
    }

    @Test
    void testEstimateWithRefinement() throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        var numValid = 0;
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
            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            for (var i = 0; i < nPlanes; i++) {
                final var inputPlane = new Plane(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                final var outputPlane = transformation1.transformAndReturnNew(inputPlane);
                final Plane outputPlaneWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // line is outlier
                    final var errorA = errorRandomizer.nextDouble();
                    final var errorB = errorRandomizer.nextDouble();
                    final var errorC = errorRandomizer.nextDouble();
                    final var errorD = errorRandomizer.nextDouble();
                    outputPlaneWithError = new Plane(
                            outputPlane.getA() + errorA,
                            outputPlane.getB() + errorB,
                            outputPlane.getC() + errorC,
                            outputPlane.getD() + errorD);
                } else {
                    // inlier line (without error)
                    outputPlaneWithError = outputPlane;
                }

                inputPlanes.add(inputPlane);
                outputPlanes.add(outputPlane);
                outputPlanesWithError.add(outputPlaneWithError);
            }

            final var estimator = new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(this,
                    inputPlanes, outputPlanesWithError);

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
            var failed = false;
            for (var i = 0; i < nPlanes; i++) {
                final var plane1 = outputPlanes.get(i);
                final var plane2 = transformation2.transformAndReturnNew(inputPlanes.get(i));
                plane1.normalize();
                plane2.normalize();
                if (Math.abs(PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.getResidual(plane1, plane2))
                        > ABSOLUTE_ERROR) {
                    failed = true;
                    break;
                }
                assertEquals(0.0,
                        PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.getResidual(plane1, plane2),
                        ABSOLUTE_ERROR);

                if (!plane1.equals(plane2, ABSOLUTE_ERROR)) {
                    failed = true;
                    break;
                }
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
        checkLocked((MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final ProjectiveTransformation3DRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(
            final ProjectiveTransformation3DRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        checkLocked((MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(
            final ProjectiveTransformation3DRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        checkLocked((MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = estimateProgressChange = 0;
    }

    private static void checkLocked(final MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator) {
        final var planes = new ArrayList<Plane>();
        assertThrows(LockedException.class, () -> estimator.setPlanes(planes, planes));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setProgressDelta(0.01f));
        assertThrows(LockedException.class, () -> estimator.setThreshold(0.5));
        assertThrows(LockedException.class, () -> estimator.setConfidence(0.5));
        assertThrows(LockedException.class, () -> estimator.setMaxIterations(10));
        assertThrows(LockedException.class, estimator::estimate);
        assertTrue(estimator.isLocked());
    }
}
