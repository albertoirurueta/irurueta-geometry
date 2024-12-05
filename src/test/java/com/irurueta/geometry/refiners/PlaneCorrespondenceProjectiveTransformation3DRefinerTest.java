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
package com.irurueta.geometry.refiners;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.ProjectiveTransformation3D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class PlaneCorrespondenceProjectiveTransformation3DRefinerTest implements RefinerListener<ProjectiveTransformation3D> {

    private static final double MIN_RANDOM_VALUE = -1000.0;
    private static final double MAX_RANDOM_VALUE = 1000.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int MIN_LINES = 50;
    private static final int MAX_LINES = 100;

    private static final int PERCENTAGE_OUTLIER = 20;

    private static final double STD_ERROR = 100.0;
    private static final double THRESHOLD = 1e-6;

    private static final int TIMES = 1000;

    private int refineStart;
    private int refineEnd;

    @Test
    void testConstructor() throws AlgebraException, LockedException, NotReadyException, RobustEstimatorException {
        final var estimator = createRobustEstimator();
        final var transformation = estimator.estimate();
        final var inliersData = estimator.getInliersData();
        final var inliers = inliersData.getInliers();
        final var residuals = inliersData.getResiduals();
        final var numInliers = inliersData.getNumInliers();
        final var refinementStandardDeviation = estimator.getThreshold();
        final var samples1 = estimator.getInputPlanes();
        final var samples2 = estimator.getOutputPlanes();

        assertNotNull(transformation);
        assertNotNull(inliersData);

        // test empty constructor
        var refiner = new PlaneCorrespondenceProjectiveTransformation3DRefiner();

        // check default values
        assertEquals(0.0, refiner.getRefinementStandardDeviation(), 0.0);
        assertNull(refiner.getSamples1());
        assertNull(refiner.getSamples2());
        assertFalse(refiner.isReady());
        assertNull(refiner.getInliers());
        assertEquals(0, refiner.getNumInliers());
        assertEquals(0, refiner.getTotalSamples());
        assertNull(refiner.getInitialEstimation());
        assertFalse(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());

        // test non-empty constructor
        refiner = new PlaneCorrespondenceProjectiveTransformation3DRefiner(transformation, true, inliers,
                residuals, numInliers, samples1, samples2, refinementStandardDeviation);

        // check default values
        assertEquals(refiner.getRefinementStandardDeviation(), refinementStandardDeviation, 0.0);
        assertSame(samples1, refiner.getSamples1());
        assertSame(samples2, refiner.getSamples2());
        assertTrue(refiner.isReady());
        assertSame(inliers, refiner.getInliers());
        assertSame(residuals, refiner.getResiduals());
        assertEquals(numInliers, refiner.getNumInliers());
        assertEquals(samples1.size(), refiner.getTotalSamples());
        assertSame(transformation, refiner.getInitialEstimation());
        assertTrue(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());

        refiner = new PlaneCorrespondenceProjectiveTransformation3DRefiner(transformation, true,
                inliersData, samples1, samples2, refinementStandardDeviation);

        // check default values
        assertEquals(refiner.getRefinementStandardDeviation(), refinementStandardDeviation, 0.0);
        assertSame(samples1, refiner.getSamples1());
        assertSame(samples2, refiner.getSamples2());
        assertTrue(refiner.isReady());
        assertSame(inliers, refiner.getInliers());
        assertSame(residuals, refiner.getResiduals());
        assertEquals(numInliers, refiner.getNumInliers());
        assertEquals(samples1.size(), refiner.getTotalSamples());
        assertSame(transformation, refiner.getInitialEstimation());
        assertTrue(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());
    }

    @Test
    void testGetSetListener() {
        final var refiner = new PlaneCorrespondenceProjectiveTransformation3DRefiner();

        // check default value
        assertNull(refiner.getListener());

        // set new value
        refiner.setListener(this);

        // check correctness
        assertSame(this, refiner.getListener());
    }

    @Test
    void testGetSetRefinementStandardDeviation() throws LockedException {
        final var refiner = new PlaneCorrespondenceProjectiveTransformation3DRefiner();

        // check default value
        assertEquals(0.0, refiner.getRefinementStandardDeviation(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var refinementStandardDeviation = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        refiner.setRefinementStandardDeviation(refinementStandardDeviation);

        // check correctness
        assertEquals(refiner.getRefinementStandardDeviation(), refinementStandardDeviation, 0.0);
    }

    @Test
    void testGetSetSamples1() throws LockedException {
        final var refiner = new PlaneCorrespondenceProjectiveTransformation3DRefiner();

        // check default value
        assertNull(refiner.getSamples1());

        // set new value
        final var samples1 = new ArrayList<Plane>();
        refiner.setSamples1(samples1);

        // check correctness
        assertSame(samples1, refiner.getSamples1());
    }

    @Test
    void testGetSetSamples2() throws LockedException {
        final var refiner = new PlaneCorrespondenceProjectiveTransformation3DRefiner();

        // check default value
        assertNull(refiner.getSamples2());

        // set new value
        final var samples2 = new ArrayList<Plane>();
        refiner.setSamples2(samples2);

        // check correctness
        assertSame(samples2, refiner.getSamples2());
    }

    @Test
    void testGetSetInliers() throws LockedException, AlgebraException, NotReadyException, RobustEstimatorException {
        final var estimator = createRobustEstimator();

        assertNotNull(estimator.estimate());
        final var inliersData = estimator.getInliersData();
        final var inliers = inliersData.getInliers();

        final var refiner = new PlaneCorrespondenceProjectiveTransformation3DRefiner();

        // check default value
        assertNull(refiner.getInliers());

        // set new value
        refiner.setInliers(inliers);

        // check correctness
        assertSame(inliers, refiner.getInliers());
    }

    @Test
    void testGetSetResiduals() throws LockedException, AlgebraException, NotReadyException, RobustEstimatorException {
        final var estimator = createRobustEstimator();

        assertNotNull(estimator.estimate());
        final var inliersData = estimator.getInliersData();
        final var residuals = inliersData.getResiduals();

        final var refiner = new PlaneCorrespondenceProjectiveTransformation3DRefiner();

        // check default value
        assertNull(refiner.getResiduals());

        // set new value
        refiner.setResiduals(residuals);

        // check correctness
        assertSame(residuals, refiner.getResiduals());
    }

    @Test
    void testGetSetNumInliers() throws LockedException, AlgebraException, NotReadyException, RobustEstimatorException {
        final var estimator = createRobustEstimator();

        assertNotNull(estimator.estimate());
        final var inliersData = estimator.getInliersData();
        final var numInliers = inliersData.getNumInliers();

        final var refiner = new PlaneCorrespondenceProjectiveTransformation3DRefiner();

        // check default value
        assertEquals(0, refiner.getNumInliers());

        // set new value
        refiner.setNumInliers(numInliers);

        // check correctness
        assertEquals(numInliers, refiner.getNumInliers());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> refiner.setNumInliers(0));
    }

    @Test
    void testSetInliersData() throws LockedException, AlgebraException, NotReadyException, RobustEstimatorException {
        final var estimator = createRobustEstimator();

        assertNotNull(estimator.estimate());
        final var inliersData = estimator.getInliersData();

        final var refiner = new PlaneCorrespondenceProjectiveTransformation3DRefiner();

        // check default values
        assertNull(refiner.getInliers());
        assertNull(refiner.getResiduals());
        assertEquals(0, refiner.getNumInliers());

        // set new value
        refiner.setInliersData(inliersData);

        // check correctness
        assertSame(inliersData.getInliers(), refiner.getInliers());
        assertSame(inliersData.getResiduals(), refiner.getResiduals());
        assertEquals(refiner.getNumInliers(), inliersData.getNumInliers());
    }

    @Test
    void testGetSetInitialEstimation() throws LockedException {
        final var refiner = new PlaneCorrespondenceProjectiveTransformation3DRefiner();

        // check default value
        assertNull(refiner.getInitialEstimation());

        // set new value
        final var transformation = new ProjectiveTransformation3D();
        refiner.setInitialEstimation(transformation);

        // check correctness
        assertSame(transformation, refiner.getInitialEstimation());
    }

    @Test
    void testIsSetCovarianceKept() throws LockedException {
        final var refiner = new PlaneCorrespondenceProjectiveTransformation3DRefiner();

        // check default value
        assertFalse(refiner.isCovarianceKept());

        // set new value
        refiner.setCovarianceKept(true);

        // check correctness
        assertTrue(refiner.isCovarianceKept());
    }

    @Test
    void testRefine() throws AlgebraException, LockedException, NotReadyException, RobustEstimatorException,
            RefinerException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var estimator = createRobustEstimator();

            final var transformation = estimator.estimate();
            final var inliersData = estimator.getInliersData();
            final var refineStandardDeviation = estimator.getThreshold();
            final var samples1 = estimator.getInputPlanes();
            final var samples2 = estimator.getOutputPlanes();

            final var refiner = new PlaneCorrespondenceProjectiveTransformation3DRefiner(transformation,
                    true, inliersData, samples1, samples2, refineStandardDeviation);
            refiner.setListener(this);

            final var result1 = new ProjectiveTransformation3D();

            reset();
            assertEquals(0, refineStart);
            assertEquals(0, refineEnd);

            if (!refiner.refine(result1)) {
                continue;
            }

            final var result2 = refiner.refine();

            assertEquals(2, refineStart);
            assertEquals(2, refineEnd);

            assertTrue(result1.asMatrix().equals(result2.asMatrix(), ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    private static RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator createRobustEstimator()
            throws AlgebraException, LockedException {

        final var transformation = createTransformation();

        final var randomizer = new UniformRandomizer();

        // generate random planes
        final var nPlanes = randomizer.nextInt(MIN_LINES, MAX_LINES);

        final var inputPlanes = new ArrayList<Plane>();
        final var outputPlanesWithError = new ArrayList<Plane>();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
        for (var i = 0; i < nPlanes; i++) {
            // generate input point
            final var inputPlane = new Plane(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var outputPlane = transformation.transformAndReturnNew(inputPlane);
            Plane outputPlaneWithError;
            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                // line is outlier
                final var errorA = errorRandomizer.nextDouble();
                final var errorB = errorRandomizer.nextDouble();
                final var errorC = errorRandomizer.nextDouble();
                final var errorD = errorRandomizer.nextDouble();
                outputPlaneWithError = new Plane(outputPlane.getA() + errorA,
                        outputPlane.getB() + errorB,
                        outputPlane.getC() + errorC,
                        outputPlane.getD() + errorD);
            } else {
                // inlier line (without error)
                outputPlaneWithError = outputPlane;
            }

            inputPlanes.add(inputPlane);
            outputPlanesWithError.add(outputPlaneWithError);
        }

        final var estimator = new RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(inputPlanes,
                outputPlanesWithError);

        estimator.setThreshold(THRESHOLD);
        estimator.setComputeAndKeepInliersEnabled(true);
        estimator.setComputeAndKeepResidualsEnabled(true);
        estimator.setResultRefined(false);
        estimator.setCovarianceKept(false);

        return estimator;
    }

    private static ProjectiveTransformation3D createTransformation() throws AlgebraException {

        Matrix t;
        do {
            // ensure T matrix is invertible
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS, -1.0, 1.0);
            final var norm = Utils.normF(t);
            // normalize T to increase accuracy
            t.multiplyByScalar(1.0 / norm);
        } while (Utils.rank(t) < ProjectiveTransformation3D.HOM_COORDS);

        return new ProjectiveTransformation3D(t);
    }

    @Override
    public void onRefineStart(final Refiner<ProjectiveTransformation3D> refiner,
                              final ProjectiveTransformation3D initialEstimation) {
        refineStart++;
        checkLocked((PlaneCorrespondenceProjectiveTransformation3DRefiner) refiner);
    }

    @Override
    public void onRefineEnd(final Refiner<ProjectiveTransformation3D> refiner,
                            final ProjectiveTransformation3D initialEstimation,
                            final ProjectiveTransformation3D result,
                            final boolean errorDecreased) {
        refineEnd++;
        checkLocked((PlaneCorrespondenceProjectiveTransformation3DRefiner) refiner);
    }

    private void reset() {
        refineStart = refineEnd = 0;
    }

    private static void checkLocked(final PlaneCorrespondenceProjectiveTransformation3DRefiner refiner) {
        assertTrue(refiner.isLocked());
        assertThrows(LockedException.class, () -> refiner.setInitialEstimation(null));
        assertThrows(LockedException.class, () -> refiner.setCovarianceKept(true));
        assertThrows(LockedException.class, () -> refiner.refine(null));
        assertThrows(LockedException.class, refiner::refine);
        assertThrows(LockedException.class, () -> refiner.setInliers(null));
        assertThrows(LockedException.class, () -> refiner.setResiduals(null));
        assertThrows(LockedException.class, () -> refiner.setNumInliers(0));
        assertThrows(LockedException.class, () -> refiner.setInliersData(null));
        assertThrows(LockedException.class, () -> refiner.setSamples1(null));
        assertThrows(LockedException.class, () -> refiner.setSamples2(null));
        assertThrows(LockedException.class, () -> refiner.setRefinementStandardDeviation(0.0));
    }
}
