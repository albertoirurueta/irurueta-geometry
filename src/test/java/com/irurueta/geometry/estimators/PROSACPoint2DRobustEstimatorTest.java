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

import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class PROSACPoint2DRobustEstimatorTest implements
        Point2DRobustEstimatorListener {

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int MIN_LINES = 500;
    private static final int MAX_LINES = 1000;

    private static final double THRESHOLD = 1e-3;

    private static final double MIN_SCORE_ERROR = -0.3;
    private static final double MAX_SCORE_ERROR = 0.3;

    private static final double STD_ERROR = 100.0;

    private static final int PERCENTAGE_OUTLIER = 20;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    public void testConstants() {
        assertEquals(2, Point2DRobustEstimator.MINIMUM_SIZE);
        assertEquals(0.05f, Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0f);
        assertEquals(0.0f, Point2DRobustEstimator.MIN_PROGRESS_DELTA, 0.0f);
        assertEquals(1.0f, Point2DRobustEstimator.MAX_PROGRESS_DELTA, 0.0f);
        assertEquals(0.99, Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(5000, Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(0.0, Point2DRobustEstimator.MIN_CONFIDENCE, 0.0);
        assertEquals(1.0, Point2DRobustEstimator.MAX_CONFIDENCE, 0.0);
        assertEquals(1, Point2DRobustEstimator.MIN_ITERATIONS);
        assertEquals(RobustEstimatorMethod.PROMEDS, Point2DRobustEstimator.DEFAULT_ROBUST_METHOD);
        assertTrue(Point2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(Point2DRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertEquals(1.0, PROSACPoint2DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(0.0, PROSACPoint2DRobustEstimator.MIN_THRESHOLD, 0.0);
        assertFalse(PROSACPoint2DRobustEstimator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertFalse(PROSACPoint2DRobustEstimator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
    }

    @Test
    public void testConstructor() {
        // test constructor without arguments
        PROSACPoint2DRobustEstimator estimator = new PROSACPoint2DRobustEstimator();

        // check correctness
        assertEquals(PROSACPoint2DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(Point2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(Point3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES,
                estimator.getRefinementCoordinatesType());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // test constructor with lines
        final List<Line2D> lines = new ArrayList<>();
        for (int i = 0; i < Point2DRobustEstimator.MINIMUM_SIZE; i++) {
            lines.add(new Line2D());
        }

        estimator = new PROSACPoint2DRobustEstimator(lines);

        // check correctness
        assertEquals(PROSACPoint2DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(Point2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(lines, estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(Point3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES,
                estimator.getRefinementCoordinatesType());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        final List<Line2D> emptyLines = new ArrayList<>();
        estimator = null;
        try {
            estimator = new PROSACPoint2DRobustEstimator(emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener
        final Point2DRobustEstimatorListener listener =
                new Point2DRobustEstimatorListener() {

                    @Override
                    public void onEstimateStart(final Point2DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateEnd(final Point2DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final Point2DRobustEstimator estimator, final int iteration) {
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final Point2DRobustEstimator estimator, final float progress) {
                    }
                };

        estimator = new PROSACPoint2DRobustEstimator(listener);

        // check correctness
        assertEquals(PROSACPoint2DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(Point2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(Point3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES,
                estimator.getRefinementCoordinatesType());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // test constructor with listener and lines
        estimator = new PROSACPoint2DRobustEstimator(listener, lines);

        // check correctness
        assertEquals(PROSACPoint2DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(Point2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(lines, estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(Point3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES,
                estimator.getRefinementCoordinatesType());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACPoint2DRobustEstimator(listener, emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores
        final double[] qualityScores = new double[
                Point2DRobustEstimator.MINIMUM_SIZE];
        estimator = new PROSACPoint2DRobustEstimator(qualityScores);

        // check correctness
        assertEquals(PROSACPoint2DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(Point2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(Point3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES,
                estimator.getRefinementCoordinatesType());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        final double[] emptyScores = new double[0];
        estimator = null;
        try {
            estimator = new PROSACPoint2DRobustEstimator(emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with lines and scores
        estimator = new PROSACPoint2DRobustEstimator(lines, qualityScores);

        // check correctness
        assertEquals(PROSACPoint2DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(Point2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(lines, estimator.getLines());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(Point3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES,
                estimator.getRefinementCoordinatesType());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACPoint2DRobustEstimator(emptyLines,
                    qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACPoint2DRobustEstimator(lines, emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener and quality scores
        estimator = new PROSACPoint2DRobustEstimator(listener, qualityScores);

        // check correctness
        assertEquals(PROSACPoint2DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(Point2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(Point3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES,
                estimator.getRefinementCoordinatesType());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACPoint2DRobustEstimator(listener, emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener, lines and quality scores
        estimator = new PROSACPoint2DRobustEstimator(listener, lines,
                qualityScores);

        // check correctness
        assertEquals(PROSACPoint2DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(Point2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(lines, estimator.getLines());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(Point3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES,
                estimator.getRefinementCoordinatesType());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACPoint2DRobustEstimator(listener, emptyLines,
                    qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACPoint2DRobustEstimator(listener, lines,
                    emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetThreshold() throws LockedException {
        final PROSACPoint2DRobustEstimator estimator =
                new PROSACPoint2DRobustEstimator();

        // check default value
        assertEquals(PROSACPoint2DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);

        // set new value
        estimator.setThreshold(0.5);

        assertEquals(0.5, estimator.getThreshold(), 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final PROSACPoint2DRobustEstimator estimator =
                new PROSACPoint2DRobustEstimator();

        // check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());

        // set listener
        estimator.setListener(this);

        // check correctness
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final PROSACPoint2DRobustEstimator estimator =
                new PROSACPoint2DRobustEstimator();

        // check default value
        assertEquals(Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check correctness
        assertEquals(0.5f, estimator.getProgressDelta(), 0.0);

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
    public void testGetSetConfidence() throws LockedException {
        final PROSACPoint2DRobustEstimator estimator =
                new PROSACPoint2DRobustEstimator();

        // check default value
        assertEquals(Point2DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);

        // set new value
        estimator.setConfidence(0.5f);

        // check correctness
        assertEquals(0.5, estimator.getConfidence(), 0.0);

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
        final PROSACPoint2DRobustEstimator estimator =
                new PROSACPoint2DRobustEstimator();

        // check default value
        assertEquals(Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(1);

        // check correctness
        assertEquals(1, estimator.getMaxIterations());

        // Fail IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetLines() throws LockedException {
        final PROSACPoint2DRobustEstimator estimator =
                new PROSACPoint2DRobustEstimator();

        // check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());

        // set new value
        final List<Line2D> lines = new ArrayList<>();
        for (int i = 0; i < Point2DRobustEstimator.MINIMUM_SIZE; i++) {
            lines.add(new Line2D());
        }
        estimator.setLines(lines);

        // check correctness
        assertSame(lines, estimator.getLines());
        assertFalse(estimator.isReady());

        // if we set quality scores, then estimator becomes ready
        final double[] qualityScores = new double[lines.size()];
        estimator.setQualityScores(qualityScores);

        assertTrue(estimator.isReady());

        // clearing list makes instance not ready
        lines.clear();

        assertFalse(estimator.isReady());

        // Force IllegalArgumentException
        final List<Line2D> emptyLines = new ArrayList<>();
        try {
            estimator.setLines(emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final PROSACPoint2DRobustEstimator estimator =
                new PROSACPoint2DRobustEstimator();

        assertNull(estimator.getQualityScores());

        double[] qualityScores = new double[
                Point2DRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        qualityScores = new double[1];
        try {
            estimator.setQualityScores(qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetResultRefined() throws LockedException {
        final PROSACPoint2DRobustEstimator estimator =
                new PROSACPoint2DRobustEstimator();

        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check correctness
        assertFalse(estimator.isResultRefined());
    }

    @Test
    public void testGetSetRefinementCoordinatesType() throws LockedException {
        final PROSACPoint2DRobustEstimator estimator =
                new PROSACPoint2DRobustEstimator();

        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES,
                estimator.getRefinementCoordinatesType());

        // set new value
        estimator.setRefinementCoordinatesType(
                CoordinatesType.HOMOGENEOUS_COORDINATES);

        // check correctness
        assertEquals(CoordinatesType.HOMOGENEOUS_COORDINATES,
                estimator.getRefinementCoordinatesType());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final PROSACPoint2DRobustEstimator estimator =
                new PROSACPoint2DRobustEstimator();

        assertFalse(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(true);

        // check correctness
        assertTrue(estimator.isCovarianceKept());
    }

    @Test
    public void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        final PROSACPoint2DRobustEstimator estimator =
                new PROSACPoint2DRobustEstimator();

        assertFalse(estimator.isComputeAndKeepInliersEnabled());

        // set new value
        estimator.setComputeAndKeepInliersEnabled(true);

        // check correctness
        assertTrue(estimator.isComputeAndKeepInliersEnabled());
    }

    @Test
    public void testIsSetComputeAndKeepResidualsEnabled()
            throws LockedException {
        final PROSACPoint2DRobustEstimator estimator =
                new PROSACPoint2DRobustEstimator();

        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // set new value
        estimator.setComputeAndKeepResidualsEnabled(true);

        // check correctness
        assertTrue(estimator.isComputeAndKeepResidualsEnabled());
    }

    @Test
    public void testEstimateWithoutRefinement() throws LockedException,
            NotReadyException, RobustEstimatorException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final Point2D point = new HomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                1.0);

        // compute random lines passing through the point
        final int nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);
        final double[] qualityScores = new double[nLines];
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_ERROR);
        final List<Line2D> lines = new ArrayList<>();
        final List<Line2D> linesWithError = new ArrayList<>();
        Line2D line;
        Line2D lineWithError;
        for (int i = 0; i < nLines; i++) {
            final double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR,
                    MAX_SCORE_ERROR);
            qualityScores[i] = 1.0 + scoreError;

            // get another point (far enough to compute a line)
            Point2D anotherPoint;
            do {
                anotherPoint = new HomogeneousPoint2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE), 1.0);
            } while (anotherPoint.distanceTo(point) < STD_ERROR);

            line = new Line2D(point, anotherPoint);

            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                // line is outlier
                final double errorA = errorRandomizer.nextDouble();
                final double errorB = errorRandomizer.nextDouble();
                final double errorC = errorRandomizer.nextDouble();
                lineWithError = new Line2D(line.getA() + errorA,
                        line.getB() + errorB, line.getC() + errorC);

                final double error = Math.sqrt(errorA * errorA + errorB * errorB +
                        errorC * errorC);
                qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
            } else {
                // inlier line
                lineWithError = line;
            }

            lines.add(line);
            linesWithError.add(lineWithError);

            // check that point is locus of line without error
            assertTrue(line.isLocus(point, ABSOLUTE_ERROR));
        }

        final PROSACPoint2DRobustEstimator estimator =
                new PROSACPoint2DRobustEstimator(this, linesWithError,
                        qualityScores);

        estimator.setThreshold(THRESHOLD);
        estimator.setResultRefined(false);
        estimator.setCovarianceKept(false);

        assertEquals(0, estimateStart);
        assertEquals(0, estimateEnd);
        assertEquals(0, estimateNextIteration);
        assertEquals(0, estimateProgressChange);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());

        final Point2D point2 = estimator.estimate();

        assertEquals(1, estimateStart);
        assertEquals(1, estimateEnd);
        assertTrue(estimateNextIteration > 0);
        assertTrue(estimateProgressChange >= 0);
        reset();

        // check correctness of estimation by checking that all lines without
        // error have estimated point as locus
        for (final Line2D l : lines) {
            assertTrue(l.isLocus(point2, ABSOLUTE_ERROR));
        }

        // check that both points are equal
        assertEquals(0.0, point.distanceTo(point2), ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateWithInhomogeneousRefinement() throws LockedException,
            NotReadyException, RobustEstimatorException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final Point2D point = new HomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                1.0);

        // compute random lines passing through the point
        final int nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);
        final double[] qualityScores = new double[nLines];
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_ERROR);
        final List<Line2D> lines = new ArrayList<>();
        final List<Line2D> linesWithError = new ArrayList<>();
        Line2D line;
        Line2D lineWithError;
        for (int i = 0; i < nLines; i++) {
            final double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR,
                    MAX_SCORE_ERROR);
            qualityScores[i] = 1.0 + scoreError;

            // get another point (far enough to compute a line)
            Point2D anotherPoint;
            do {
                anotherPoint = new HomogeneousPoint2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE), 1.0);
            } while (anotherPoint.distanceTo(point) < STD_ERROR);

            line = new Line2D(point, anotherPoint);

            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                // line is outlier
                final double errorA = errorRandomizer.nextDouble();
                final double errorB = errorRandomizer.nextDouble();
                final double errorC = errorRandomizer.nextDouble();
                lineWithError = new Line2D(line.getA() + errorA,
                        line.getB() + errorB, line.getC() + errorC);

                final double error = Math.sqrt(errorA * errorA + errorB * errorB +
                        errorC * errorC);
                qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
            } else {
                // inlier line
                lineWithError = line;
            }

            lines.add(line);
            linesWithError.add(lineWithError);

            // check that point is locus of line without error
            assertTrue(line.isLocus(point, ABSOLUTE_ERROR));
        }

        final PROSACPoint2DRobustEstimator estimator =
                new PROSACPoint2DRobustEstimator(this, linesWithError,
                        qualityScores);

        estimator.setThreshold(THRESHOLD);
        estimator.setResultRefined(true);
        estimator.setCovarianceKept(true);
        estimator.setRefinementCoordinatesType(
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        estimator.setComputeAndKeepInliersEnabled(true);
        estimator.setComputeAndKeepResidualsEnabled(true);

        assertEquals(0, estimateStart);
        assertEquals(0, estimateEnd);
        assertEquals(0, estimateNextIteration);
        assertEquals(0, estimateProgressChange);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());

        final Point2D point2 = estimator.estimate();

        assertNotNull(estimator.getInliersData());
        assertNotNull(estimator.getInliersData().getInliers());
        assertNotNull(estimator.getInliersData().getResiduals());
        assertTrue(estimator.getInliersData().getNumInliers() > 0);
        assertNotNull(estimator.getCovariance());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH,
                estimator.getCovariance().getRows());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH,
                estimator.getCovariance().getColumns());

        assertEquals(1, estimateStart);
        assertEquals(1, estimateEnd);
        assertTrue(estimateNextIteration > 0);
        assertTrue(estimateProgressChange >= 0);
        reset();

        // check correctness of estimation by checking that all lines without
        // error have estimated point as locus
        for (final Line2D l : lines) {
            assertTrue(l.isLocus(point2, ABSOLUTE_ERROR));
        }

        // check that both points are equal
        assertEquals(0.0, point.distanceTo(point2), ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateWithHomogeneousRefinement() throws LockedException,
            NotReadyException, RobustEstimatorException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final Point2D point = new HomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                1.0);

        // compute random lines passing through the point
        final int nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);
        final double[] qualityScores = new double[nLines];
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_ERROR);
        final List<Line2D> lines = new ArrayList<>();
        final List<Line2D> linesWithError = new ArrayList<>();
        Line2D line;
        Line2D lineWithError;
        for (int i = 0; i < nLines; i++) {
            final double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR,
                    MAX_SCORE_ERROR);
            qualityScores[i] = 1.0 + scoreError;

            // get another point (far enough to compute a line)
            Point2D anotherPoint;
            do {
                anotherPoint = new HomogeneousPoint2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE), 1.0);
            } while (anotherPoint.distanceTo(point) < STD_ERROR);

            line = new Line2D(point, anotherPoint);

            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                // line is outlier
                final double errorA = errorRandomizer.nextDouble();
                final double errorB = errorRandomizer.nextDouble();
                final double errorC = errorRandomizer.nextDouble();
                lineWithError = new Line2D(line.getA() + errorA,
                        line.getB() + errorB, line.getC() + errorC);

                final double error = Math.sqrt(errorA * errorA + errorB * errorB +
                        errorC * errorC);
                qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
            } else {
                // inlier line
                lineWithError = line;
            }

            lines.add(line);
            linesWithError.add(lineWithError);

            // check that point is locus of line without error
            assertTrue(line.isLocus(point, ABSOLUTE_ERROR));
        }

        final PROSACPoint2DRobustEstimator estimator =
                new PROSACPoint2DRobustEstimator(this, linesWithError,
                        qualityScores);

        estimator.setThreshold(THRESHOLD);
        estimator.setResultRefined(true);
        estimator.setCovarianceKept(true);
        estimator.setRefinementCoordinatesType(
                CoordinatesType.HOMOGENEOUS_COORDINATES);
        estimator.setComputeAndKeepInliersEnabled(true);
        estimator.setComputeAndKeepResidualsEnabled(true);

        assertEquals(0, estimateStart);
        assertEquals(0, estimateEnd);
        assertEquals(0, estimateNextIteration);
        assertEquals(0, estimateProgressChange);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());

        final Point2D point2 = estimator.estimate();

        assertNotNull(estimator.getInliersData());
        assertNotNull(estimator.getInliersData().getInliers());
        assertNotNull(estimator.getInliersData().getResiduals());
        assertTrue(estimator.getInliersData().getNumInliers() > 0);
        assertNotNull(estimator.getCovariance());
        assertEquals(Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH,
                estimator.getCovariance().getRows());
        assertEquals(Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH,
                estimator.getCovariance().getColumns());

        assertEquals(1, estimateStart);
        assertEquals(1, estimateEnd);
        assertTrue(estimateNextIteration > 0);
        assertTrue(estimateProgressChange >= 0);
        reset();

        // check correctness of estimation by checking that all lines without
        // error have estimated point as locus
        for (final Line2D l : lines) {
            assertTrue(l.isLocus(point2, ABSOLUTE_ERROR));
        }

        // check that both points are equal
        assertEquals(0.0, point.distanceTo(point2), ABSOLUTE_ERROR);
    }

    @Override
    public void onEstimateStart(final Point2DRobustEstimator estimator) {
        estimateStart++;
        checkLocked((PROSACPoint2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final Point2DRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((PROSACPoint2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(final Point2DRobustEstimator estimator,
                                        final int iteration) {
        estimateNextIteration++;
        checkLocked((PROSACPoint2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(final Point2DRobustEstimator estimator,
                                         final float progress) {
        estimateProgressChange++;
        checkLocked((PROSACPoint2DRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void checkLocked(final PROSACPoint2DRobustEstimator estimator) {
        try {
            estimator.setThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setConfidence(0.5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setMaxIterations(5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setLines(null);
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
