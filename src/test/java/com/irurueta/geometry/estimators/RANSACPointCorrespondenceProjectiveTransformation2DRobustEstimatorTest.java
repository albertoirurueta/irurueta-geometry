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

import com.irurueta.algebra.DecomposerException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.ProjectiveTransformation2D;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimatorTest
        implements ProjectiveTransformation2DRobustEstimatorListener {

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double ABSOLUTE_ERROR = 5e-6;

    private static final int MIN_POINTS = 500;
    private static final int MAX_POINTS = 1000;

    private static final double THRESHOLD = 1e-6;

    private static final double STD_ERROR = 100.0;

    private static final int PERCENTAGE_OUTLIER = 20;

    private static final int TIMES = 10;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    public void testConstants() {
        assertEquals(4, ProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE);
        assertEquals(0.05f, ProjectiveTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0f);
        assertEquals(0.0f, ProjectiveTransformation2DRobustEstimator.MIN_PROGRESS_DELTA, 0.0f);
        assertEquals(1.0f, ProjectiveTransformation2DRobustEstimator.MAX_PROGRESS_DELTA, 0.0f);
        assertEquals(0.99, ProjectiveTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(5000, ProjectiveTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(0.0, ProjectiveTransformation2DRobustEstimator.MIN_CONFIDENCE, 0.0);
        assertEquals(1.0, ProjectiveTransformation2DRobustEstimator.MAX_CONFIDENCE, 0.0);
        assertEquals(1, ProjectiveTransformation2DRobustEstimator.MIN_ITERATIONS);
        assertTrue(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(ProjectiveTransformation2DRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertEquals(RobustEstimatorMethod.PROMedS,
                PointCorrespondenceProjectiveTransformation2DRobustEstimator.DEFAULT_ROBUST_METHOD);
        assertEquals(1.0, RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(0.0, RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.MIN_THRESHOLD,
                0.0);
        assertFalse(RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator
                .DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertFalse(RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator
                .DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
    }

    @Test
    public void testConstructor() {
        // test constructor without arguments
        RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator();

        assertEquals(estimator.getThreshold(),
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // test constructor with points
        final List<Point2D> inputPoints = new ArrayList<>();
        final List<Point2D> outputPoints = new ArrayList<>();
        for (int i = 0; i < PointCorrespondenceProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }

        estimator = new RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                inputPoints, outputPoints);

        assertEquals(estimator.getThreshold(),
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        final List<Point2D> pointsEmpty = new ArrayList<>();
        estimator = null;
        try {
            // not enough points
            estimator = new RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                    pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                    inputPoints, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor with listener
        estimator = new RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                this);

        assertEquals(estimator.getThreshold(),
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // test constructor with listener and points
        estimator = new RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                this, inputPoints, outputPoints);

        assertEquals(estimator.getThreshold(),
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough points
            estimator = new RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                    this, pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                    this, inputPoints, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetThreshold() throws LockedException {
        final RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator();

        // check default value
        assertEquals(estimator.getThreshold(),
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
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
    public void testGetSetConfidence() throws LockedException {
        final RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator();

        // check default value
        assertEquals(estimator.getConfidence(),
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
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
        final RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator();

        // check default value
        assertEquals(estimator.getMaxIterations(),
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
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
    public void testGetSetPointsAndIsReady() throws LockedException {
        final RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator();

        // check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());

        // set new value
        final List<Point2D> inputPoints = new ArrayList<>();
        final List<Point2D> outputPoints = new ArrayList<>();
        for (int i = 0; i < PointCorrespondenceProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }

        estimator.setPoints(inputPoints, outputPoints);

        // check correctness
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());

        // Force IllegalArgumentException
        final List<Point2D> pointsEmpty = new ArrayList<>();
        try {
            // not enough points
            estimator.setPoints(pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator.setPoints(pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListenerAndIsListenerAvailable() throws LockedException {
        final RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator();

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
        final RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator();

        // check default value
        assertEquals(estimator.getProgressDelta(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
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
        final RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator();

        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check correctness
        assertFalse(estimator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator();

        assertFalse(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(true);

        // check correctness
        assertTrue(estimator.isCovarianceKept());
    }

    @Test
    public void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        final RANSACPointCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new RANSACPointCorrespondenceAffineTransformation2DRobustEstimator();

        assertFalse(estimator.isComputeAndKeepInliersEnabled());

        // set new value
        estimator.setComputeAndKeepInliersEnabled(true);

        // check correctness
        assertTrue(estimator.isComputeAndKeepInliersEnabled());
    }

    @Test
    public void testIsSetComputeAndKeepResidualsEnabled()
            throws LockedException {
        final RANSACPointCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new RANSACPointCorrespondenceAffineTransformation2DRobustEstimator();

        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // set new value
        estimator.setComputeAndKeepResidualsEnabled(true);

        // check correctness
        assertTrue(estimator.isComputeAndKeepResidualsEnabled());
    }

    @Test
    public void testEstimateWithoutRefinement() throws WrongSizeException,
            DecomposerException, LockedException, NotReadyException,
            RobustEstimatorException {
        // create an affine transformation
        Matrix a;
        do {
            // ensure A matrix is invertible
            a = Matrix.createWithUniformRandomValues(
                    ProjectiveTransformation2D.INHOM_COORDS,
                    ProjectiveTransformation2D.INHOM_COORDS, -1.0, 1.0);
            final double norm = Utils.normF(a);
            // normalize T to increase accuracy
            a.multiplyByScalar(1.0 / norm);
        } while (Utils.rank(a) < ProjectiveTransformation2D.INHOM_COORDS);

        final double[] translation = new double[
                ProjectiveTransformation2D.INHOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(translation, -1.0, 1.0);

        final ProjectiveTransformation2D transformation1 =
                new ProjectiveTransformation2D(a, translation);

        // generate random points
        final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        final List<Point2D> inputPoints = new ArrayList<>();
        final List<Point2D> outputPoints = new ArrayList<>();
        final List<Point2D> outputPointsWithError = new ArrayList<>();
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_ERROR);
        for (int i = 0; i < nPoints; i++) {
            final Point2D inputPoint = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final Point2D outputPoint = transformation1.transformAndReturnNew(inputPoint);
            final Point2D outputPointWithError;
            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                // point is outlier
                final double errorX = errorRandomizer.nextDouble();
                final double errorY = errorRandomizer.nextDouble();
                outputPointWithError = new InhomogeneousPoint2D(
                        outputPoint.getInhomX() + errorX,
                        outputPoint.getInhomY() + errorY);
            } else {
                // inlier point (without error)
                outputPointWithError = outputPoint;
            }

            inputPoints.add(inputPoint);
            outputPoints.add(outputPoint);
            outputPointsWithError.add(outputPointWithError);
        }

        final RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                        this, inputPoints, outputPointsWithError);

        estimator.setThreshold(THRESHOLD);
        estimator.setResultRefined(false);
        estimator.setCovarianceKept(false);

        assertEquals(estimateStart, 0);
        assertEquals(estimateEnd, 0);
        assertEquals(estimateNextIteration, 0);
        assertEquals(estimateProgressChange, 0);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());

        final ProjectiveTransformation2D transformation2 = estimator.estimate();

        assertEquals(estimateStart, 1);
        assertEquals(estimateEnd, 1);
        assertTrue(estimateNextIteration > 0);
        assertTrue(estimateProgressChange >= 0);
        reset();

        // check correctness of estimation by transforming input points
        // using estimated transformation (transformation2) and checking
        // that output points are equal to the original output points without
        // error
        Point2D p1;
        Point2D p2;
        for (int i = 0; i < nPoints; i++) {
            p1 = outputPoints.get(i);
            p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
            assertEquals(p1.distanceTo(p2), 0.0,
                    ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testEstimateWithRefinement() throws WrongSizeException,
            DecomposerException, LockedException, NotReadyException,
            RobustEstimatorException {
        for (int t = 0; t < TIMES; t++) {
            // create an affine transformation
            Matrix a;
            do {
                // ensure A matrix is invertible
                a = Matrix.createWithUniformRandomValues(
                        ProjectiveTransformation2D.INHOM_COORDS,
                        ProjectiveTransformation2D.INHOM_COORDS, -1.0, 1.0);
                final double norm = Utils.normF(a);
                // normalize T to increase accuracy
                a.multiplyByScalar(1.0 / norm);
            } while (Utils.rank(a) < ProjectiveTransformation2D.INHOM_COORDS);

            final double[] translation = new double[
                    ProjectiveTransformation2D.INHOM_COORDS];
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(translation, -1.0, 1.0);

            final ProjectiveTransformation2D transformation1 =
                    new ProjectiveTransformation2D(a, translation);

            // generate random points
            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final List<Point2D> inputPoints = new ArrayList<>();
            final List<Point2D> outputPoints = new ArrayList<>();
            final List<Point2D> outputPointsWithError = new ArrayList<>();
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            for (int i = 0; i < nPoints; i++) {
                final Point2D inputPoint = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                final Point2D outputPoint = transformation1.transformAndReturnNew(inputPoint);
                final Point2D outputPointWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    outputPointWithError = new InhomogeneousPoint2D(
                            outputPoint.getInhomX() + errorX,
                            outputPoint.getInhomY() + errorY);
                } else {
                    // inlier point (without error)
                    outputPointWithError = outputPoint;
                }

                inputPoints.add(inputPoint);
                outputPoints.add(outputPoint);
                outputPointsWithError.add(outputPointWithError);
            }

            final RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                    new RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                            this, inputPoints, outputPointsWithError);

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

            final ProjectiveTransformation2D transformation2 = estimator.estimate();

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                assertEquals(estimator.getCovariance().getRows(),
                        ProjectiveTransformation2D.HOM_COORDS *
                                ProjectiveTransformation2D.HOM_COORDS);
                assertEquals(estimator.getCovariance().getColumns(),
                        ProjectiveTransformation2D.HOM_COORDS *
                                ProjectiveTransformation2D.HOM_COORDS);
            }

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by transforming input points
            // using estimated transformation (transformation2) and checking
            // that output points are equal to the original output points without
            // error
            Point2D p1;
            Point2D p2;
            for (int i = 0; i < nPoints; i++) {
                p1 = outputPoints.get(i);
                p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
                assertEquals(p1.distanceTo(p2), 0.0,
                        ABSOLUTE_ERROR);
            }
        }
    }

    @Override
    public void onEstimateStart(final ProjectiveTransformation2DRobustEstimator estimator) {
        estimateStart++;
        checkLocked((RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(
            final ProjectiveTransformation2DRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(
            final ProjectiveTransformation2DRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        checkLocked((RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(
            final ProjectiveTransformation2DRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        checkLocked((RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void checkLocked(
            final RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator) {
        final List<Point2D> points = new ArrayList<>();
        try {
            estimator.setPoints(points, points);
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
