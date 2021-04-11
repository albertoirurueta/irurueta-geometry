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
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class PlaneCorrespondenceProjectiveTransformation3DRefinerTest implements
        RefinerListener<ProjectiveTransformation3D> {

    private static final double MIN_RANDOM_VALUE = -1000.0;
    private static final double MAX_RANDOM_VALUE = 1000.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int MIN_LINES = 50;
    private static final int MAX_LINES = 100;

    private static final int PERCENTAGE_OUTLIER = 20;

    private static final double STD_ERROR = 100.0;
    private static final double THRESHOLD = 1e-6;

    private static final int TIMES = 1000;

    private int mRefineStart;
    private int mRefineEnd;

    @Test
    public void testConstructor() throws AlgebraException, LockedException,
            NotReadyException, RobustEstimatorException {
        final RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                createRobustEstimator();
        final ProjectiveTransformation3D transformation = estimator.estimate();
        final InliersData inliersData = estimator.getInliersData();
        final BitSet inliers = inliersData.getInliers();
        final double[] residuals = inliersData.getResiduals();
        final int numInliers = inliersData.getNumInliers();
        final double refinementStandardDeviation = estimator.getThreshold();
        final List<Plane> samples1 = estimator.getInputPlanes();
        final List<Plane> samples2 = estimator.getOutputPlanes();

        assertNotNull(transformation);
        assertNotNull(inliersData);

        // test empty constructor
        PlaneCorrespondenceProjectiveTransformation3DRefiner refiner =
                new PlaneCorrespondenceProjectiveTransformation3DRefiner();

        // check default values
        assertEquals(refiner.getRefinementStandardDeviation(), 0.0, 0.0);
        assertNull(refiner.getSamples1());
        assertNull(refiner.getSamples2());
        assertFalse(refiner.isReady());
        assertNull(refiner.getInliers());
        assertEquals(refiner.getNumInliers(), 0);
        assertEquals(refiner.getTotalSamples(), 0);
        assertNull(refiner.getInitialEstimation());
        assertFalse(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());

        // test non-empty constructor
        refiner = new PlaneCorrespondenceProjectiveTransformation3DRefiner(
                transformation, true, inliers, residuals, numInliers, samples1,
                samples2, refinementStandardDeviation);

        // check default values
        assertEquals(refiner.getRefinementStandardDeviation(),
                refinementStandardDeviation, 0.0);
        assertSame(refiner.getSamples1(), samples1);
        assertSame(refiner.getSamples2(), samples2);
        assertTrue(refiner.isReady());
        assertSame(refiner.getInliers(), inliers);
        assertSame(refiner.getResiduals(), residuals);
        assertEquals(refiner.getNumInliers(), numInliers);
        assertEquals(refiner.getTotalSamples(), samples1.size());
        assertSame(refiner.getInitialEstimation(), transformation);
        assertTrue(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());

        refiner = new PlaneCorrespondenceProjectiveTransformation3DRefiner(
                transformation, true, inliersData, samples1, samples2,
                refinementStandardDeviation);

        // check default values
        assertEquals(refiner.getRefinementStandardDeviation(),
                refinementStandardDeviation, 0.0);
        assertSame(refiner.getSamples1(), samples1);
        assertSame(refiner.getSamples2(), samples2);
        assertTrue(refiner.isReady());
        assertSame(refiner.getInliers(), inliers);
        assertSame(refiner.getResiduals(), residuals);
        assertEquals(refiner.getNumInliers(), numInliers);
        assertEquals(refiner.getTotalSamples(), samples1.size());
        assertSame(refiner.getInitialEstimation(), transformation);
        assertTrue(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());
    }

    @Test
    public void testGetSetListener() {
        final PlaneCorrespondenceProjectiveTransformation3DRefiner refiner =
                new PlaneCorrespondenceProjectiveTransformation3DRefiner();

        // check default value
        assertNull(refiner.getListener());

        // set new value
        refiner.setListener(this);

        // check correctness
        assertSame(refiner.getListener(), this);
    }

    @Test
    public void testGetSetRefinementStandardDeviation() throws LockedException {
        final PlaneCorrespondenceProjectiveTransformation3DRefiner refiner =
                new PlaneCorrespondenceProjectiveTransformation3DRefiner();

        // check default value
        assertEquals(refiner.getRefinementStandardDeviation(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double refinementStandardDeviation = randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        refiner.setRefinementStandardDeviation(refinementStandardDeviation);

        // check correctness
        assertEquals(refiner.getRefinementStandardDeviation(),
                refinementStandardDeviation, 0.0);
    }

    @Test
    public void testGetSetSamples1() throws LockedException {
        final PlaneCorrespondenceProjectiveTransformation3DRefiner refiner =
                new PlaneCorrespondenceProjectiveTransformation3DRefiner();

        // check default value
        assertNull(refiner.getSamples1());

        // set new value
        final List<Plane> samples1 = new ArrayList<>();
        refiner.setSamples1(samples1);

        // check correctness
        assertSame(refiner.getSamples1(), samples1);
    }

    @Test
    public void testGetSetSamples2() throws LockedException {
        final PlaneCorrespondenceProjectiveTransformation3DRefiner refiner =
                new PlaneCorrespondenceProjectiveTransformation3DRefiner();

        // check default value
        assertNull(refiner.getSamples2());

        // set new value
        final List<Plane> samples2 = new ArrayList<>();
        refiner.setSamples2(samples2);

        // check correctness
        assertSame(refiner.getSamples2(), samples2);
    }

    @Test
    public void testGetSetInliers() throws LockedException, AlgebraException,
            NotReadyException, RobustEstimatorException {
        final RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                createRobustEstimator();

        assertNotNull(estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();
        final BitSet inliers = inliersData.getInliers();

        final PlaneCorrespondenceProjectiveTransformation3DRefiner refiner =
                new PlaneCorrespondenceProjectiveTransformation3DRefiner();

        // check default value
        assertNull(refiner.getInliers());

        // set new value
        refiner.setInliers(inliers);

        // check correctness
        assertSame(refiner.getInliers(), inliers);
    }

    @Test
    public void testGetSetResiduals() throws LockedException, AlgebraException,
            NotReadyException, RobustEstimatorException {
        final RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                createRobustEstimator();

        assertNotNull(estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();
        final double[] residuals = inliersData.getResiduals();

        final PlaneCorrespondenceProjectiveTransformation3DRefiner refiner =
                new PlaneCorrespondenceProjectiveTransformation3DRefiner();

        // check default value
        assertNull(refiner.getResiduals());

        // set new value
        refiner.setResiduals(residuals);

        // check correctness
        assertSame(refiner.getResiduals(), residuals);
    }

    @Test
    public void testGetSetNumInliers() throws LockedException, AlgebraException,
            NotReadyException, RobustEstimatorException {
        final RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                createRobustEstimator();

        assertNotNull(estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();
        final int numInliers = inliersData.getNumInliers();

        final PlaneCorrespondenceProjectiveTransformation3DRefiner refiner =
                new PlaneCorrespondenceProjectiveTransformation3DRefiner();

        // check default value
        assertEquals(refiner.getNumInliers(), 0);

        // set new value
        refiner.setNumInliers(numInliers);

        // check correctness
        assertEquals(refiner.getNumInliers(), numInliers);

        // Force IllegalArgumentException
        try {
            refiner.setNumInliers(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testSetInliersData() throws LockedException, AlgebraException,
            NotReadyException, RobustEstimatorException {
        final RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                createRobustEstimator();

        assertNotNull(estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();

        final PlaneCorrespondenceProjectiveTransformation3DRefiner refiner =
                new PlaneCorrespondenceProjectiveTransformation3DRefiner();

        // check default values
        assertNull(refiner.getInliers());
        assertNull(refiner.getResiduals());
        assertEquals(refiner.getNumInliers(), 0);

        // set new value
        refiner.setInliersData(inliersData);

        // check correctness
        assertSame(refiner.getInliers(), inliersData.getInliers());
        assertSame(refiner.getResiduals(), inliersData.getResiduals());
        assertEquals(refiner.getNumInliers(), inliersData.getNumInliers());
    }

    @Test
    public void testGetSetInitialEstimation() throws LockedException {
        final PlaneCorrespondenceProjectiveTransformation3DRefiner refiner =
                new PlaneCorrespondenceProjectiveTransformation3DRefiner();

        // check default value
        assertNull(refiner.getInitialEstimation());

        // set new value
        final ProjectiveTransformation3D transformation =
                new ProjectiveTransformation3D();
        refiner.setInitialEstimation(transformation);

        // check correctness
        assertSame(refiner.getInitialEstimation(), transformation);
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final PlaneCorrespondenceProjectiveTransformation3DRefiner refiner =
                new PlaneCorrespondenceProjectiveTransformation3DRefiner();

        // check default value
        assertFalse(refiner.isCovarianceKept());

        // set new value
        refiner.setCovarianceKept(true);

        // check correctness
        assertTrue(refiner.isCovarianceKept());
    }

    @Test
    public void testRefine() throws AlgebraException, LockedException,
            NotReadyException, RobustEstimatorException, RefinerException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                    createRobustEstimator();

            final ProjectiveTransformation3D transformation = estimator.estimate();
            final InliersData inliersData = estimator.getInliersData();
            final double refineStandardDeviation = estimator.getThreshold();
            final List<Plane> samples1 = estimator.getInputPlanes();
            final List<Plane> samples2 = estimator.getOutputPlanes();

            final PlaneCorrespondenceProjectiveTransformation3DRefiner refiner =
                    new PlaneCorrespondenceProjectiveTransformation3DRefiner(
                            transformation, true, inliersData, samples1, samples2,
                            refineStandardDeviation);
            refiner.setListener(this);

            final ProjectiveTransformation3D result1 =
                    new ProjectiveTransformation3D();

            reset();
            assertEquals(mRefineStart, 0);
            assertEquals(mRefineEnd, 0);

            if (!refiner.refine(result1)) {
                continue;
            }

            final ProjectiveTransformation3D result2 = refiner.refine();

            assertEquals(mRefineStart, 2);
            assertEquals(mRefineEnd, 2);

            assertTrue(result1.asMatrix().equals(result2.asMatrix(),
                    ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    private RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator createRobustEstimator()
            throws AlgebraException, LockedException {

        final ProjectiveTransformation3D transformation = createTransformation();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // generate random planes
        final int nPlanes = randomizer.nextInt(MIN_LINES, MAX_LINES);

        final List<Plane> inputPlanes = new ArrayList<>();
        final List<Plane> outputPlanesWithError = new ArrayList<>();
        Plane inputPlane;
        Plane outputPlane;
        Plane outputPlaneWithError;
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_ERROR);
        for (int i = 0; i < nPlanes; i++) {
            // generate input point
            inputPlane = new Plane(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            outputPlane = transformation.transformAndReturnNew(inputPlane);

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
            } else {
                // inlier line (without error)
                outputPlaneWithError = outputPlane;
            }

            inputPlanes.add(inputPlane);
            outputPlanesWithError.add(outputPlaneWithError);
        }

        final RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        inputPlanes, outputPlanesWithError);

        estimator.setThreshold(THRESHOLD);
        estimator.setComputeAndKeepInliersEnabled(true);
        estimator.setComputeAndKeepResidualsEnabled(true);
        estimator.setResultRefined(false);
        estimator.setCovarianceKept(false);

        return estimator;
    }

    private ProjectiveTransformation3D createTransformation()
            throws AlgebraException {

        Matrix t;
        do {
            // ensure T matrix is invertible
            t = Matrix.createWithUniformRandomValues(
                    ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS, -1.0, 1.0);
            final double norm = Utils.normF(t);
            // normalize T to increase accuracy
            t.multiplyByScalar(1.0 / norm);
        } while (Utils.rank(t) < ProjectiveTransformation3D.HOM_COORDS);

        return new ProjectiveTransformation3D(t);
    }

    @Override
    public void onRefineStart(final Refiner<ProjectiveTransformation3D> refiner,
                              final ProjectiveTransformation3D initialEstimation) {
        mRefineStart++;
        checkLocked((PlaneCorrespondenceProjectiveTransformation3DRefiner) refiner);
    }

    @Override
    public void onRefineEnd(final Refiner<ProjectiveTransformation3D> refiner,
                            final ProjectiveTransformation3D initialEstimation,
                            final ProjectiveTransformation3D result,
                            final boolean errorDecreased) {
        mRefineEnd++;
        checkLocked((PlaneCorrespondenceProjectiveTransformation3DRefiner) refiner);
    }

    private void reset() {
        mRefineStart = mRefineEnd = 0;
    }

    private void checkLocked(
            final PlaneCorrespondenceProjectiveTransformation3DRefiner refiner) {
        assertTrue(refiner.isLocked());
        try {
            refiner.setInitialEstimation(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setCovarianceKept(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.refine(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
        try {
            refiner.refine();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
        try {
            refiner.setInliers(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setResiduals(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setNumInliers(0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setInliersData(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setSamples1(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setSamples2(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setRefinementStandardDeviation(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
    }
}
