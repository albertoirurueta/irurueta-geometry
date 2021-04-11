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
import com.irurueta.geometry.AffineTransformation2D;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.RANSACLineCorrespondenceAffineTransformation2DRobustEstimator;
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

public class LineCorrespondenceAffineTransformation2DRefinerTest implements
        RefinerListener<AffineTransformation2D> {

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
        final RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                createRobustEstimator();
        final AffineTransformation2D transformation = estimator.estimate();
        final InliersData inliersData = estimator.getInliersData();
        final BitSet inliers = inliersData.getInliers();
        final double[] residuals = inliersData.getResiduals();
        final int numInliers = inliersData.getNumInliers();
        final double refinementStandardDeviation = estimator.getThreshold();
        final List<Line2D> samples1 = estimator.getInputLines();
        final List<Line2D> samples2 = estimator.getOutputLines();

        assertNotNull(transformation);
        assertNotNull(inliersData);

        // test empty constructor
        LineCorrespondenceAffineTransformation2DRefiner refiner =
                new LineCorrespondenceAffineTransformation2DRefiner();

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
        refiner = new LineCorrespondenceAffineTransformation2DRefiner(
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


        refiner = new LineCorrespondenceAffineTransformation2DRefiner(
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
        final LineCorrespondenceAffineTransformation2DRefiner refiner =
                new LineCorrespondenceAffineTransformation2DRefiner();

        // check default value
        assertNull(refiner.getListener());

        // set new value
        refiner.setListener(this);

        // check correctness
        assertSame(refiner.getListener(), this);
    }

    @Test
    public void testGetSetRefinementStandardDeviation() throws LockedException {
        final LineCorrespondenceAffineTransformation2DRefiner refiner =
                new LineCorrespondenceAffineTransformation2DRefiner();

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
        final LineCorrespondenceAffineTransformation2DRefiner refiner =
                new LineCorrespondenceAffineTransformation2DRefiner();

        // check default value
        assertNull(refiner.getSamples1());

        // set new value
        final List<Line2D> samples1 = new ArrayList<>();
        refiner.setSamples1(samples1);

        // check correctness
        assertSame(refiner.getSamples1(), samples1);
    }

    @Test
    public void testGetSetSamples2() throws LockedException {
        final LineCorrespondenceAffineTransformation2DRefiner refiner =
                new LineCorrespondenceAffineTransformation2DRefiner();

        // check default value
        assertNull(refiner.getSamples2());

        // set new value
        final List<Line2D> samples2 = new ArrayList<>();
        refiner.setSamples2(samples2);

        // check correctness
        assertSame(refiner.getSamples2(), samples2);
    }

    @Test
    public void testGetSetInliers() throws LockedException, AlgebraException,
            NotReadyException, RobustEstimatorException {
        final RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                createRobustEstimator();

        assertNotNull(estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();
        final BitSet inliers = inliersData.getInliers();

        final LineCorrespondenceAffineTransformation2DRefiner refiner =
                new LineCorrespondenceAffineTransformation2DRefiner();

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
        final RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                createRobustEstimator();

        assertNotNull(estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();
        final double[] residuals = inliersData.getResiduals();

        final LineCorrespondenceAffineTransformation2DRefiner refiner =
                new LineCorrespondenceAffineTransformation2DRefiner();

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
        final RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                createRobustEstimator();

        assertNotNull(estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();
        final int numInliers = inliersData.getNumInliers();

        final LineCorrespondenceAffineTransformation2DRefiner refiner =
                new LineCorrespondenceAffineTransformation2DRefiner();

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
        final RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                createRobustEstimator();

        assertNotNull(estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();

        final LineCorrespondenceAffineTransformation2DRefiner refiner =
                new LineCorrespondenceAffineTransformation2DRefiner();

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
        final LineCorrespondenceAffineTransformation2DRefiner refiner =
                new LineCorrespondenceAffineTransformation2DRefiner();

        // check default value
        assertNull(refiner.getInitialEstimation());

        // set new value
        final AffineTransformation2D transformation = new AffineTransformation2D();
        refiner.setInitialEstimation(transformation);

        // check correctness
        assertSame(refiner.getInitialEstimation(), transformation);
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final LineCorrespondenceAffineTransformation2DRefiner refiner =
                new LineCorrespondenceAffineTransformation2DRefiner();

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
            final RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                    createRobustEstimator();

            final AffineTransformation2D transformation = estimator.estimate();
            final InliersData inliersData = estimator.getInliersData();
            final double refineStandardDeviation = estimator.getThreshold();
            final List<Line2D> samples1 = estimator.getInputLines();
            final List<Line2D> samples2 = estimator.getOutputLines();

            final LineCorrespondenceAffineTransformation2DRefiner refiner =
                    new LineCorrespondenceAffineTransformation2DRefiner(
                            transformation, true, inliersData, samples1, samples2,
                            refineStandardDeviation);
            refiner.setListener(this);

            final AffineTransformation2D result1 = new AffineTransformation2D();

            reset();
            assertEquals(mRefineStart, 0);
            assertEquals(mRefineEnd, 0);

            if (!refiner.refine(result1)) {
                continue;
            }

            final AffineTransformation2D result2 = refiner.refine();

            assertEquals(mRefineStart, 2);
            assertEquals(mRefineEnd, 2);

            assertTrue(result1.asMatrix().equals(result2.asMatrix(),
                    ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }


    private RANSACLineCorrespondenceAffineTransformation2DRobustEstimator createRobustEstimator()
            throws AlgebraException, LockedException {

        final AffineTransformation2D transformation = createTransformation();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // generate random lines
        final int nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);

        final List<Line2D> inputLines = new ArrayList<>();
        final List<Line2D> outputLinesWithError = new ArrayList<>();
        Line2D inputLine;
        Line2D outputLine;
        Line2D outputLineWithError;
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_ERROR);
        for (int i = 0; i < nLines; i++) {
            // generate input point
            inputLine = new Line2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            outputLine = transformation.transformAndReturnNew(inputLine);

            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                // line is outlier
                final double errorA = errorRandomizer.nextDouble();
                final double errorB = errorRandomizer.nextDouble();
                final double errorC = errorRandomizer.nextDouble();
                outputLineWithError = new Line2D(outputLine.getA() + errorA,
                        outputLine.getB() + errorB,
                        outputLine.getC() + errorC);
            } else {
                // inlier line (without error)
                outputLineWithError = outputLine;
            }

            inputLines.add(inputLine);
            outputLinesWithError.add(outputLineWithError);
        }

        final RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                        inputLines, outputLinesWithError);

        estimator.setThreshold(THRESHOLD);
        estimator.setComputeAndKeepInliersEnabled(true);
        estimator.setComputeAndKeepResidualsEnabled(true);
        estimator.setResultRefined(false);
        estimator.setCovarianceKept(false);

        return estimator;
    }

    private AffineTransformation2D createTransformation()
            throws AlgebraException {

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

        final double[] translation = new double[AffineTransformation2D.INHOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(translation, -1.0, 1.0);

        return new AffineTransformation2D(a, translation);
    }

    @Override
    public void onRefineStart(final Refiner<AffineTransformation2D> refiner,
                              final AffineTransformation2D initialEstimation) {
        mRefineStart++;
        checkLocked((LineCorrespondenceAffineTransformation2DRefiner) refiner);
    }

    @Override
    public void onRefineEnd(final Refiner<AffineTransformation2D> refiner,
                            final AffineTransformation2D initialEstimation,
                            final AffineTransformation2D result,
                            final boolean errorDecreased) {
        mRefineEnd++;
        checkLocked((LineCorrespondenceAffineTransformation2DRefiner) refiner);
    }

    private void reset() {
        mRefineStart = mRefineEnd = 0;
    }

    private void checkLocked(
            final LineCorrespondenceAffineTransformation2DRefiner refiner) {
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
