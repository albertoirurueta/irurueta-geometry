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
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.ProjectiveTransformation2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator;
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

public class PointCorrespondenceProjectiveTransformation2DRefinerTest implements
        RefinerListener<ProjectiveTransformation2D> {

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
        final RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                createRobustEstimator();
        final ProjectiveTransformation2D transformation = estimator.estimate();
        final InliersData inliersData = estimator.getInliersData();
        final BitSet inliers = inliersData.getInliers();
        final double[] residuals = inliersData.getResiduals();
        final int numInliers = inliersData.getNumInliers();
        final double refinementStandardDeviation = estimator.getThreshold();
        final List<Point2D> samples1 = estimator.getInputPoints();
        final List<Point2D> samples2 = estimator.getOutputPoints();

        assertNotNull(transformation);
        assertNotNull(inliersData);

        // test empty constructor
        PointCorrespondenceProjectiveTransformation2DRefiner refiner =
                new PointCorrespondenceProjectiveTransformation2DRefiner();

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
        refiner = new PointCorrespondenceProjectiveTransformation2DRefiner(
                transformation, true, inliers, residuals, numInliers, samples1,
                samples2, refinementStandardDeviation);

        // check default values
        assertEquals(refinementStandardDeviation, refiner.getRefinementStandardDeviation(), 0.0);
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

        refiner = new PointCorrespondenceProjectiveTransformation2DRefiner(
                transformation, true, inliersData, samples1, samples2,
                refinementStandardDeviation);

        // check default values
        assertEquals(refinementStandardDeviation, refiner.getRefinementStandardDeviation(), 0.0);
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
    public void testGetSetListener() {
        final PointCorrespondenceProjectiveTransformation2DRefiner refiner =
                new PointCorrespondenceProjectiveTransformation2DRefiner();

        // check default value
        assertNull(refiner.getListener());

        // set new value
        refiner.setListener(this);

        // check correctness
        assertSame(this, refiner.getListener());
    }

    @Test
    public void testGetSetRefinementStandardDeviation() throws LockedException {
        final PointCorrespondenceProjectiveTransformation2DRefiner refiner =
                new PointCorrespondenceProjectiveTransformation2DRefiner();

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
    public void testGetSetSamples1() throws LockedException {
        final PointCorrespondenceProjectiveTransformation2DRefiner refiner =
                new PointCorrespondenceProjectiveTransformation2DRefiner();

        // check default value
        assertNull(refiner.getSamples1());

        // set new value
        final List<Point2D> samples1 = new ArrayList<>();
        refiner.setSamples1(samples1);

        // check correctness
        assertSame(samples1, refiner.getSamples1());
    }

    @Test
    public void testGetSetSamples2() throws LockedException {
        final PointCorrespondenceProjectiveTransformation2DRefiner refiner =
                new PointCorrespondenceProjectiveTransformation2DRefiner();

        // check default value
        assertNull(refiner.getSamples2());

        // set new value
        final List<Point2D> samples2 = new ArrayList<>();
        refiner.setSamples2(samples2);

        // check correctness
        assertSame(samples2, refiner.getSamples2());
    }

    @Test
    public void testGetSetInliers() throws LockedException, AlgebraException,
            NotReadyException, RobustEstimatorException {
        final RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                createRobustEstimator();

        assertNotNull(estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();
        final BitSet inliers = inliersData.getInliers();

        final PointCorrespondenceProjectiveTransformation2DRefiner refiner =
                new PointCorrespondenceProjectiveTransformation2DRefiner();

        // check default value
        assertNull(refiner.getInliers());

        // set new value
        refiner.setInliers(inliers);

        // check correctness
        assertSame(inliers, refiner.getInliers());
    }

    @Test
    public void testGetSetResiduals() throws LockedException, AlgebraException,
            NotReadyException, RobustEstimatorException {
        final RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                createRobustEstimator();

        assertNotNull(estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();
        final double[] residuals = inliersData.getResiduals();

        final PointCorrespondenceProjectiveTransformation2DRefiner refiner =
                new PointCorrespondenceProjectiveTransformation2DRefiner();

        // check default value
        assertNull(refiner.getResiduals());

        // set new value
        refiner.setResiduals(residuals);

        // check correctness
        assertSame(residuals, refiner.getResiduals());
    }

    @Test
    public void testGetSetNumInliers() throws LockedException, AlgebraException,
            NotReadyException, RobustEstimatorException {
        final RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                createRobustEstimator();

        assertNotNull(estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();
        final int numInliers = inliersData.getNumInliers();

        final PointCorrespondenceProjectiveTransformation2DRefiner refiner =
                new PointCorrespondenceProjectiveTransformation2DRefiner();

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
    public void testSetInliersData() throws LockedException, AlgebraException,
            NotReadyException, RobustEstimatorException {
        final RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                createRobustEstimator();

        assertNotNull(estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();

        final PointCorrespondenceProjectiveTransformation2DRefiner refiner =
                new PointCorrespondenceProjectiveTransformation2DRefiner();

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
        final PointCorrespondenceProjectiveTransformation2DRefiner refiner =
                new PointCorrespondenceProjectiveTransformation2DRefiner();

        // check default value
        assertNull(refiner.getInitialEstimation());

        // set new value
        final ProjectiveTransformation2D transformation =
                new ProjectiveTransformation2D();
        refiner.setInitialEstimation(transformation);

        // check correctness
        assertSame(transformation, refiner.getInitialEstimation());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final PointCorrespondenceProjectiveTransformation2DRefiner refiner =
                new PointCorrespondenceProjectiveTransformation2DRefiner();

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
            final RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                    createRobustEstimator();

            final ProjectiveTransformation2D transformation = estimator.estimate();
            final InliersData inliersData = estimator.getInliersData();
            final double refineStandardDeviation = estimator.getThreshold();
            final List<Point2D> samples1 = estimator.getInputPoints();
            final List<Point2D> samples2 = estimator.getOutputPoints();

            final PointCorrespondenceProjectiveTransformation2DRefiner refiner =
                    new PointCorrespondenceProjectiveTransformation2DRefiner(
                            transformation, true, inliersData, samples1, samples2,
                            refineStandardDeviation);
            refiner.setListener(this);

            final ProjectiveTransformation2D result1 =
                    new ProjectiveTransformation2D();

            reset();
            assertEquals(0, mRefineStart);
            assertEquals(0, mRefineEnd);

            if (!refiner.refine(result1)) {
                continue;
            }

            final ProjectiveTransformation2D result2 = refiner.refine();

            assertEquals(2, mRefineStart);
            assertEquals(2, mRefineEnd);

            assertTrue(result1.asMatrix().equals(result2.asMatrix(), ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    private RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator createRobustEstimator()
            throws AlgebraException, LockedException {

        final ProjectiveTransformation2D transformation = createTransformation();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // generate random points
        final int nPoints = randomizer.nextInt(MIN_LINES, MAX_LINES);

        final List<Point2D> inputPoints = new ArrayList<>();
        final List<Point2D> outputLinesWithError = new ArrayList<>();
        Point2D inputPoint;
        Point2D outputPoint;
        Point2D outputPointWithError;
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_ERROR);
        for (int i = 0; i < nPoints; i++) {
            // generate input point
            inputPoint = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            outputPoint = transformation.transformAndReturnNew(inputPoint);

            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                // point is outlier
                final double errorX = errorRandomizer.nextDouble();
                final double errorY = errorRandomizer.nextDouble();
                outputPointWithError = new InhomogeneousPoint2D(
                        outputPoint.getInhomX() + errorX,
                        outputPoint.getInhomY() + errorY);
            } else {
                // inlier line (without error)
                outputPointWithError = outputPoint;
            }

            inputPoints.add(inputPoint);
            outputLinesWithError.add(outputPointWithError);
        }

        final RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                        inputPoints, outputLinesWithError);

        estimator.setThreshold(THRESHOLD);
        estimator.setComputeAndKeepInliersEnabled(true);
        estimator.setComputeAndKeepResidualsEnabled(true);
        estimator.setResultRefined(false);
        estimator.setCovarianceKept(false);

        return estimator;
    }

    private ProjectiveTransformation2D createTransformation()
            throws AlgebraException {

        Matrix t;
        do {
            // ensure T matrix is invertible
            t = Matrix.createWithUniformRandomValues(
                    ProjectiveTransformation2D.HOM_COORDS,
                    ProjectiveTransformation2D.HOM_COORDS, -1.0, 1.0);
            final double norm = Utils.normF(t);
            // normalize T to increase accuracy
            t.multiplyByScalar(1.0 / norm);
        } while (Utils.rank(t) < ProjectiveTransformation2D.HOM_COORDS);

        return new ProjectiveTransformation2D(t);
    }

    @Override
    public void onRefineStart(final Refiner<ProjectiveTransformation2D> refiner,
                              final ProjectiveTransformation2D initialEstimation) {
        mRefineStart++;
        checkLocked((PointCorrespondenceProjectiveTransformation2DRefiner) refiner);
    }

    @Override
    public void onRefineEnd(final Refiner<ProjectiveTransformation2D> refiner,
                            final ProjectiveTransformation2D initialEstimation,
                            final ProjectiveTransformation2D result,
                            final boolean errorDecreased) {
        mRefineEnd++;
        checkLocked((PointCorrespondenceProjectiveTransformation2DRefiner) refiner);
    }

    private void reset() {
        mRefineStart = mRefineEnd = 0;
    }

    private void checkLocked(
            final PointCorrespondenceProjectiveTransformation2DRefiner refiner) {
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
