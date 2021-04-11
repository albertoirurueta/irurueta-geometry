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
import com.irurueta.geometry.AffineTransformation2D;
import com.irurueta.geometry.Line2D;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class MSACLineCorrespondenceAffineTransformation2DRobustEstimatorTest
        implements AffineTransformation2DRobustEstimatorListener {

    private static final double MIN_RANDOM_VALUE = -1000.0;
    private static final double MAX_RANDOM_VALUE = 1000.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int MIN_LINES = 500;
    private static final int MAX_LINES = 1000;

    private static final double THRESHOLD = 1e-6;

    private static final double STD_ERROR = 100.0;

    private static final int PERCENTAGE_OUTLIER = 20;

    private static final int TIMES = 100;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    public void testConstructor() {
        // test constructor without arguments
        MSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new MSACLineCorrespondenceAffineTransformation2DRobustEstimator();

        assertEquals(estimator.getThreshold(),
                MSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                MSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                MSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());


        // test constructor with lines
        final List<Line2D> inputLines = new ArrayList<>();
        final List<Line2D> outputLines = new ArrayList<>();
        for (int i = 0; i < LineCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputLines.add(new Line2D());
            outputLines.add(new Line2D());
        }

        estimator = new MSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                inputLines, outputLines);

        assertEquals(estimator.getThreshold(),
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final List<Line2D> linesEmpty = new ArrayList<>();
        estimator = null;
        try {
            // not enough lines
            estimator = new MSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                    linesEmpty, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new MSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                    inputLines, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener
        estimator = new MSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                this);

        assertEquals(estimator.getThreshold(),
                MSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                MSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                MSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());


        // test constructor with listener and lines
        estimator = new MSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                this, inputLines, outputLines);

        assertEquals(estimator.getThreshold(),
                MSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                MSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                MSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough points
            estimator = new MSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                    this, linesEmpty, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new MSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                    this, inputLines, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetThreshold() throws LockedException {
        final MSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new MSACLineCorrespondenceAffineTransformation2DRobustEstimator();

        // check default value
        assertEquals(estimator.getThreshold(),
                MSACLineCorrespondenceAffineTransformation2DRobustEstimator.
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
        final MSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new MSACLineCorrespondenceAffineTransformation2DRobustEstimator();

        // check default value
        assertEquals(estimator.getConfidence(),
                MSACLineCorrespondenceAffineTransformation2DRobustEstimator.
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
        final MSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new MSACLineCorrespondenceAffineTransformation2DRobustEstimator();

        // check default value
        assertEquals(estimator.getMaxIterations(),
                MSACLineCorrespondenceAffineTransformation2DRobustEstimator.
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
    public void testGetSetLinesAndIsReady() throws LockedException {
        final MSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new MSACLineCorrespondenceAffineTransformation2DRobustEstimator();

        // check default values
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());

        // set new value
        final List<Line2D> inputLines = new ArrayList<>();
        final List<Line2D> outputLines = new ArrayList<>();
        for (int i = 0; i < LineCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputLines.add(new Line2D());
            outputLines.add(new Line2D());
        }

        estimator.setLines(inputLines, outputLines);

        // check correctness
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertTrue(estimator.isReady());

        // Force IllegalArgumentException
        final List<Line2D> linesEmpty = new ArrayList<>();
        try {
            // not enough lines
            estimator.setLines(linesEmpty, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator.setLines(linesEmpty, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListenerAndIsListenerAvailable() throws LockedException {
        final MSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new MSACLineCorrespondenceAffineTransformation2DRobustEstimator();

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
        final MSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new MSACLineCorrespondenceAffineTransformation2DRobustEstimator();

        // check default value
        assertEquals(estimator.getProgressDelta(),
                AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
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
        final MSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new MSACLineCorrespondenceAffineTransformation2DRobustEstimator();

        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check correctness
        assertFalse(estimator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final MSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new MSACLineCorrespondenceAffineTransformation2DRobustEstimator();

        assertFalse(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(true);

        // check correctness
        assertTrue(estimator.isCovarianceKept());
    }

    @Test
    public void testEstimateWithoutRefinement() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        for (int t = 0; t < TIMES; t++) {
            // create an affine transformation
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

            final double[] translation = new double[
                    AffineTransformation2D.INHOM_COORDS];
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(translation, -1.0, 1.0);

            final AffineTransformation2D transformation1 =
                    new AffineTransformation2D(a, translation);

            // generate random lines
            final int nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);
            final List<Line2D> inputLines = new ArrayList<>();
            final List<Line2D> outputLines = new ArrayList<>();
            final List<Line2D> outputLinesWithError = new ArrayList<>();
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            for (int i = 0; i < nLines; i++) {
                final Line2D inputLine = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                final Line2D outputLine = transformation1.transformAndReturnNew(inputLine);
                final Line2D outputLineWithError;
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
                outputLines.add(outputLine);
                outputLinesWithError.add(outputLineWithError);
            }

            final MSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                    new MSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                            this, inputLines, outputLinesWithError);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final AffineTransformation2D transformation2 = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by transforming input lines
            // using estimated transformation (transformation2) and checking
            // that output lines are equal to the original output lines without
            // error
            Line2D l1, l2;
            for (int i = 0; i < nLines; i++) {
                l1 = outputLines.get(i);
                l2 = transformation2.transformAndReturnNew(inputLines.get(i));
                l1.normalize();
                l2.normalize();
                assertEquals(
                        LineCorrespondenceAffineTransformation2DRobustEstimator.
                                getResidual(l1, l2), 0.0, ABSOLUTE_ERROR);
                assertTrue(l1.equals(l2, ABSOLUTE_ERROR));
            }
        }
    }

    @Test
    public void testEstimateWithRefinement() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        for (int t = 0; t < TIMES; t++) {
            // create an affine transformation
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

            final double[] translation = new double[
                    AffineTransformation2D.INHOM_COORDS];
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(translation, -1.0, 1.0);

            final AffineTransformation2D transformation1 =
                    new AffineTransformation2D(a, translation);

            // generate random lines
            final int nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);
            final List<Line2D> inputLines = new ArrayList<>();
            final List<Line2D> outputLines = new ArrayList<>();
            final List<Line2D> outputLinesWithError = new ArrayList<>();
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            for (int i = 0; i < nLines; i++) {
                final Line2D inputLine = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                final Line2D outputLine = transformation1.transformAndReturnNew(inputLine);
                final Line2D outputLineWithError;
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
                outputLines.add(outputLine);
                outputLinesWithError.add(outputLineWithError);
            }

            final MSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                    new MSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                            this, inputLines, outputLinesWithError);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final AffineTransformation2D transformation2 = estimator.estimate();

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNotNull(estimator.getCovariance());
            assertEquals(estimator.getCovariance().getRows(),
                    AffineTransformation2D.INHOM_COORDS *
                            AffineTransformation2D.INHOM_COORDS +
                            AffineTransformation2D.NUM_TRANSLATION_COORDS);
            assertEquals(estimator.getCovariance().getColumns(),
                    AffineTransformation2D.INHOM_COORDS *
                            AffineTransformation2D.INHOM_COORDS +
                            AffineTransformation2D.NUM_TRANSLATION_COORDS);

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by transforming input lines
            // using estimated transformation (transformation2) and checking
            // that output lines are equal to the original output lines without
            // error
            Line2D l1, l2;
            for (int i = 0; i < nLines; i++) {
                l1 = outputLines.get(i);
                l2 = transformation2.transformAndReturnNew(inputLines.get(i));
                l1.normalize();
                l2.normalize();
                assertEquals(
                        LineCorrespondenceAffineTransformation2DRobustEstimator.
                                getResidual(l1, l2), 0.0, ABSOLUTE_ERROR);
                assertTrue(l1.equals(l2, ABSOLUTE_ERROR));
            }
        }
    }

    @Override
    public void onEstimateStart(final AffineTransformation2DRobustEstimator estimator) {
        estimateStart++;
        checkLocked((MSACLineCorrespondenceAffineTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(
            final AffineTransformation2DRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((MSACLineCorrespondenceAffineTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(
            final AffineTransformation2DRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        checkLocked((MSACLineCorrespondenceAffineTransformation2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(
            final AffineTransformation2DRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        checkLocked((MSACLineCorrespondenceAffineTransformation2DRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void checkLocked(
            final MSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator) {
        final List<Line2D> lines = new ArrayList<>();
        try {
            estimator.setLines(lines, lines);
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
