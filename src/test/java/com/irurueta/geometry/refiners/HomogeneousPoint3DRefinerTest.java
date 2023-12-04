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

import com.irurueta.geometry.ColinearPointsException;
import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.RANSACPoint3DRobustEstimator;
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

public class HomogeneousPoint3DRefinerTest implements
        RefinerListener<HomogeneousPoint3D> {

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double ABSOLUTE_ERROR = 5e-6;

    private static final int MIN_LINES = 500;
    private static final int MAX_LINES = 1000;

    private static final double THRESHOLD = 1e-6;

    private static final double STD_ERROR = 100.0;

    private static final int PERCENTAGE_OUTLIER = 20;

    private static final int TIMES = 100;

    private int mRefineStart;
    private int mRefineEnd;

    @Test
    public void testConstructor() throws LockedException, NotReadyException,
            ColinearPointsException, RobustEstimatorException {
        final RANSACPoint3DRobustEstimator estimator = createRobustEstimator();
        final HomogeneousPoint3D point = new HomogeneousPoint3D(
                estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();
        final BitSet inliers = inliersData.getInliers();
        final double[] residuals = inliersData.getResiduals();
        final int numInliers = inliersData.getNumInliers();
        final double refinementStandardDeviation = estimator.getThreshold();
        final List<Plane> samples = estimator.getPlanes();

        assertNotNull(point);
        assertNotNull(inliersData);

        // test empty constructor
        HomogeneousPoint3DRefiner refiner = new HomogeneousPoint3DRefiner();

        // check default values
        assertEquals(0.0, refiner.getRefinementStandardDeviation(), 0.0);
        assertNull(refiner.getSamples());
        assertFalse(refiner.isReady());
        assertNull(refiner.getInliers());
        assertNull(refiner.getResiduals());
        assertEquals(0, refiner.getNumInliers());
        assertEquals(0, refiner.getTotalSamples());
        assertNull(refiner.getInitialEstimation());
        assertFalse(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());

        // test non-empty constructor
        refiner = new HomogeneousPoint3DRefiner(point, true, inliers,
                residuals, numInliers, samples, refinementStandardDeviation);

        // check default values
        assertEquals(refinementStandardDeviation, refiner.getRefinementStandardDeviation(), 0.0);
        assertSame(samples, refiner.getSamples());
        assertTrue(refiner.isReady());
        assertSame(inliers, refiner.getInliers());
        assertSame(residuals, refiner.getResiduals());
        assertEquals(numInliers, refiner.getNumInliers());
        assertEquals(samples.size(), refiner.getTotalSamples());
        assertSame(point, refiner.getInitialEstimation());
        assertTrue(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());

        // test non-empty constructor with InliersData
        refiner = new HomogeneousPoint3DRefiner(point, true, inliersData,
                samples, refinementStandardDeviation);

        // check default values
        assertEquals(refinementStandardDeviation, refiner.getRefinementStandardDeviation(), 0.0);
        assertSame(samples, refiner.getSamples());
        assertTrue(refiner.isReady());
        assertSame(inliers, refiner.getInliers());
        assertSame(residuals, refiner.getResiduals());
        assertEquals(numInliers, refiner.getNumInliers());
        assertEquals(samples.size(), refiner.getTotalSamples());
        assertSame(point, refiner.getInitialEstimation());
        assertTrue(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());
    }

    @Test
    public void testGetSetListener() {
        final HomogeneousPoint3DRefiner refiner = new HomogeneousPoint3DRefiner();

        // check default value
        assertNull(refiner.getListener());

        // set new value
        refiner.setListener(this);

        // check correctness
        assertSame(this, refiner.getListener());
    }

    @Test
    public void testGetSetRefinementStandardDeviation() throws LockedException {
        final HomogeneousPoint3DRefiner refiner = new HomogeneousPoint3DRefiner();

        // check default value
        assertEquals(0.0, refiner.getRefinementStandardDeviation(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double refinementStandardDeviation = randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        refiner.setRefinementStandardDeviation(refinementStandardDeviation);

        // check correctness
        assertEquals(refinementStandardDeviation, refiner.getRefinementStandardDeviation(), 0.0);
    }

    @Test
    public void testGetSetSamples() throws LockedException,
            ColinearPointsException {
        final RANSACPoint3DRobustEstimator estimator = createRobustEstimator();
        final List<Plane> samples = estimator.getPlanes();

        final HomogeneousPoint3DRefiner refiner = new HomogeneousPoint3DRefiner();

        // check default value
        assertNull(refiner.getSamples());

        // set new value
        refiner.setSamples(samples);

        // check correctness
        assertSame(samples, refiner.getSamples());
    }

    @Test
    public void testGetSetInliers() throws LockedException, NotReadyException,
            RobustEstimatorException, ColinearPointsException {
        final RANSACPoint3DRobustEstimator estimator = createRobustEstimator();

        assertNotNull(estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();
        final BitSet inliers = inliersData.getInliers();

        final HomogeneousPoint3DRefiner refiner = new HomogeneousPoint3DRefiner();

        // check default value
        assertNull(refiner.getInliers());

        // set new value
        refiner.setInliers(inliers);

        // check correctness
        assertSame(inliers, refiner.getInliers());
    }

    @Test
    public void testGetSetResiduals() throws LockedException, NotReadyException,
            RobustEstimatorException, ColinearPointsException {
        final RANSACPoint3DRobustEstimator estimator = createRobustEstimator();

        assertNotNull(estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();
        final double[] residuals = inliersData.getResiduals();

        final HomogeneousPoint3DRefiner refiner = new HomogeneousPoint3DRefiner();

        // check default value
        assertNull(refiner.getResiduals());

        // set new value
        refiner.setResiduals(residuals);

        // check correctness
        assertSame(residuals, refiner.getResiduals());
    }

    @Test
    public void testGetSetNumInliers() throws LockedException, NotReadyException,
            RobustEstimatorException, ColinearPointsException {
        final RANSACPoint3DRobustEstimator estimator = createRobustEstimator();

        assertNotNull(estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();
        final int numInliers = inliersData.getNumInliers();

        final HomogeneousPoint3DRefiner refiner = new HomogeneousPoint3DRefiner();

        // check default value
        assertEquals(0, refiner.getNumInliers());

        // set new value
        refiner.setNumInliers(numInliers);

        // check correctness
        assertEquals(numInliers, refiner.getNumInliers());

        // Force IllegalArgumentException
        try {
            refiner.setNumInliers(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testSetInliersData() throws LockedException, NotReadyException,
            RobustEstimatorException, ColinearPointsException {
        final RANSACPoint3DRobustEstimator estimator = createRobustEstimator();

        assertNotNull(estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();

        final HomogeneousPoint3DRefiner refiner = new HomogeneousPoint3DRefiner();

        // check default values
        assertNull(refiner.getInliers());
        assertNull(refiner.getResiduals());
        assertEquals(0, refiner.getNumInliers());

        // set new value
        refiner.setInliersData(inliersData);

        // check correctness
        assertSame(inliersData.getInliers(), refiner.getInliers());
        assertSame(inliersData.getResiduals(), refiner.getResiduals());
        assertEquals(inliersData.getNumInliers(), refiner.getNumInliers());
    }

    @Test
    public void testGetSetInitialEstimation() throws LockedException {
        final HomogeneousPoint3DRefiner refiner = new HomogeneousPoint3DRefiner();

        // check default value
        assertNull(refiner.getInitialEstimation());

        // set new value
        final HomogeneousPoint3D point = new HomogeneousPoint3D();
        refiner.setInitialEstimation(point);

        // check correctness
        assertSame(point, refiner.getInitialEstimation());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final HomogeneousPoint3DRefiner refiner = new HomogeneousPoint3DRefiner();

        // check default value
        assertFalse(refiner.isCovarianceKept());

        // set new value
        refiner.setCovarianceKept(true);

        // check correctness
        assertTrue(refiner.isCovarianceKept());
    }

    @Test
    public void testRefine() throws ColinearPointsException, LockedException,
            NotReadyException, RobustEstimatorException, RefinerException {
        int numValid = 0;
        for (int t = 0; t < 5 * TIMES; t++) {
            final RANSACPoint3DRobustEstimator estimator = createRobustEstimator();

            final HomogeneousPoint3D point = new HomogeneousPoint3D(
                    estimator.estimate());
            final InliersData inliersData = estimator.getInliersData();
            final double refineStandardDeviation = estimator.getThreshold();
            final List<Plane> samples = estimator.getPlanes();

            final HomogeneousPoint3DRefiner refiner =
                    new HomogeneousPoint3DRefiner(point, true, inliersData,
                            samples, refineStandardDeviation);
            refiner.setListener(this);

            final HomogeneousPoint3D result1 = new HomogeneousPoint3D();

            reset();
            assertEquals(0, mRefineStart);
            assertEquals(0, mRefineEnd);

            if (!refiner.refine(result1)) {
                continue;
            }

            final HomogeneousPoint3D result2 = refiner.refine();

            assertEquals(2, mRefineStart);
            assertEquals(2, mRefineEnd);

            assertEquals(result1, result2);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    private RANSACPoint3DRobustEstimator createRobustEstimator()
            throws ColinearPointsException, LockedException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final Point3D point = new HomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                1.0);

        // compute random planes passing through the point
        final int nPlanes = randomizer.nextInt(MIN_LINES, MAX_LINES);
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_ERROR);
        final List<Plane> planesWithError = new ArrayList<>();
        Plane plane;
        Plane planeWithError;
        for (int i = 0; i < nPlanes; i++) {
            // get two more points(far enough to compute a plane)
            Point3D point2;
            Point3D point3;
            do {
                point2 = new HomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE), 1.0);
            } while (point2.distanceTo(point) < STD_ERROR);
            do {
                point3 = new HomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE), 1.0);
            } while (point3.distanceTo(point) < STD_ERROR ||
                    point3.distanceTo(point2) < STD_ERROR);

            plane = new Plane(point, point2, point3);

            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                // line is outlier
                final double errorA = errorRandomizer.nextDouble();
                final double errorB = errorRandomizer.nextDouble();
                final double errorC = errorRandomizer.nextDouble();
                final double errorD = errorRandomizer.nextDouble();
                planeWithError = new Plane(plane.getA() + errorA,
                        plane.getB() + errorB, plane.getC() + errorC,
                        plane.getD() + errorD);
            } else {
                // inlier plane
                planeWithError = plane;
            }

            planesWithError.add(planeWithError);

            // check that point is locus of plane without error
            assertTrue(plane.isLocus(point, ABSOLUTE_ERROR));
        }

        final RANSACPoint3DRobustEstimator estimator =
                new RANSACPoint3DRobustEstimator(planesWithError);

        estimator.setThreshold(THRESHOLD);
        estimator.setComputeAndKeepInliersEnabled(true);
        estimator.setComputeAndKeepResidualsEnabled(true);
        estimator.setResultRefined(false);
        estimator.setCovarianceKept(false);

        return estimator;
    }

    private void reset() {
        mRefineStart = mRefineEnd = 0;
    }

    @Override
    public void onRefineStart(final Refiner<HomogeneousPoint3D> refiner,
                              final HomogeneousPoint3D initialEstimation) {
        mRefineStart++;
        checkLocked((HomogeneousPoint3DRefiner) refiner);
    }

    @Override
    public void onRefineEnd(final Refiner<HomogeneousPoint3D> refiner,
                            final HomogeneousPoint3D initialEstimation,
                            final HomogeneousPoint3D result,
                            final boolean errorDecreased) {
        mRefineEnd++;
        checkLocked((HomogeneousPoint3DRefiner) refiner);
    }

    private void checkLocked(final HomogeneousPoint3DRefiner refiner) {
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
            refiner.setSamples(null);
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
