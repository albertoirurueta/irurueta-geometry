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

public class LMedSPoint2DRobustEstimatorTest implements
        Point2DRobustEstimatorListener {

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double ABSOLUTE_ERROR = 5e-6;

    private static final int MIN_LINES = 500;
    private static final int MAX_LINES = 1000;

    private static final double STOP_THRESHOLD = 1e-6;

    private static final double STD_ERROR = 100.0;

    private static final int PERCENTAGE_OUTLIER = 20;

    private static final int TIMES = 10;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    public void testConstructor() {
        // test constructor without arguments
        LMedSPoint2DRobustEstimator estimator = new LMedSPoint2DRobustEstimator();

        // check correctness
        assertEquals(estimator.getStopThreshold(),
                LMedSPoint2DRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                Point2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.getRefinementCoordinatesType(),
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test constructor with lines
        final List<Line2D> lines = new ArrayList<>();
        for (int i = 0; i < Point2DRobustEstimator.MINIMUM_SIZE; i++) {
            lines.add(new Line2D());
        }

        estimator = new LMedSPoint2DRobustEstimator(lines);

        // check correctness
        assertEquals(estimator.getStopThreshold(),
                LMedSPoint2DRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                Point3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.getRefinementCoordinatesType(),
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final List<Line2D> emptyLines = new ArrayList<>();
        estimator = null;
        try {
            estimator = new LMedSPoint2DRobustEstimator(emptyLines);
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

        estimator = new LMedSPoint2DRobustEstimator(listener);

        // check correctness
        assertEquals(estimator.getStopThreshold(),
                LMedSPoint2DRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                Point3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.getRefinementCoordinatesType(),
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test constructor with listener and lines
        estimator = new LMedSPoint2DRobustEstimator(listener, lines);

        // check correctness
        assertEquals(estimator.getStopThreshold(),
                LMedSPoint2DRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                Point3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.getRefinementCoordinatesType(),
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LMedSPoint2DRobustEstimator(listener, emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetThreshold() throws LockedException {
        final LMedSPoint2DRobustEstimator estimator =
                new LMedSPoint2DRobustEstimator();

        // check default value
        assertEquals(estimator.getStopThreshold(),
                LMedSPoint2DRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);

        // set new value
        estimator.setStopThreshold(0.5);

        assertEquals(estimator.getStopThreshold(), 0.5, 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setStopThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final LMedSPoint2DRobustEstimator estimator =
                new LMedSPoint2DRobustEstimator();

        // check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());

        // set listener
        estimator.setListener(this);

        // check correctness
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final LMedSPoint2DRobustEstimator estimator =
                new LMedSPoint2DRobustEstimator();

        // check default value
        assertEquals(estimator.getProgressDelta(),
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);

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
    public void testGetSetConfidence() throws LockedException {
        final LMedSPoint2DRobustEstimator estimator =
                new LMedSPoint2DRobustEstimator();

        // check default value
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);

        // set new value
        estimator.setConfidence(0.5f);

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
        final LMedSPoint2DRobustEstimator estimator =
                new LMedSPoint2DRobustEstimator();

        // check default value
        assertEquals(estimator.getMaxIterations(),
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);

        // set new value
        estimator.setMaxIterations(1);

        // check correctness
        assertEquals(estimator.getMaxIterations(), 1);

        // Fail IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetLines() throws LockedException {
        final LMedSPoint2DRobustEstimator estimator =
                new LMedSPoint2DRobustEstimator();

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
        assertSame(estimator.getLines(), lines);
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
        final LMedSPoint2DRobustEstimator estimator =
                new LMedSPoint2DRobustEstimator();

        assertNull(estimator.getQualityScores());

        final double[] qualityScores = new double[
                Point2DRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertNull(estimator.getQualityScores());
    }

    @Test
    public void testIsSetResultRefined() throws LockedException {
        final LMedSPoint2DRobustEstimator estimator =
                new LMedSPoint2DRobustEstimator();

        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check correctness
        assertFalse(estimator.isResultRefined());
    }

    @Test
    public void testGetSetRefinementCoordinatesType() throws LockedException {
        final LMedSPoint2DRobustEstimator estimator =
                new LMedSPoint2DRobustEstimator();

        assertEquals(estimator.getRefinementCoordinatesType(),
                CoordinatesType.INHOMOGENEOUS_COORDINATES);

        // set new value
        estimator.setRefinementCoordinatesType(
                CoordinatesType.HOMOGENEOUS_COORDINATES);

        // check correctness
        assertEquals(estimator.getRefinementCoordinatesType(),
                CoordinatesType.HOMOGENEOUS_COORDINATES);
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final LMedSPoint2DRobustEstimator estimator =
                new LMedSPoint2DRobustEstimator();

        assertFalse(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(true);

        // check correctness
        assertTrue(estimator.isCovarianceKept());
    }

    @Test
    public void testEstimateWithoutRefinement() throws LockedException,
            NotReadyException, RobustEstimatorException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final Point2D point = new HomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    1.0);

            // compute random lines passing through the point
            final int nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            final List<Line2D> lines = new ArrayList<>();
            final List<Line2D> linesWithError = new ArrayList<>();
            Line2D line, lineWithError;
            for (int i = 0; i < nLines; i++) {
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
                } else {
                    // inlier line
                    lineWithError = line;
                }

                lines.add(line);
                linesWithError.add(lineWithError);

                // check that point is locus of line without error
                assertTrue(line.isLocus(point, ABSOLUTE_ERROR));
            }

            final LMedSPoint2DRobustEstimator estimator =
                    new LMedSPoint2DRobustEstimator(this, linesWithError);

            estimator.setStopThreshold(STOP_THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final Point2D point2 = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by checking that all lines without
            // error have estimated point as locus
            for (final Line2D l : lines) {
                assertTrue(l.isLocus(point2, ABSOLUTE_ERROR));
            }

            // check that both points are equal
            assertEquals(point.distanceTo(point2), 0.0, ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testEstimateWithInhomogeneousRefinement() throws LockedException,
            NotReadyException, RobustEstimatorException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final Point2D point = new HomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    1.0);

            // compute random lines passing through the point
            final int nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            final List<Line2D> lines = new ArrayList<>();
            final List<Line2D> linesWithError = new ArrayList<>();
            Line2D line, lineWithError;
            for (int i = 0; i < nLines; i++) {
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
                } else {
                    // inlier line
                    lineWithError = line;
                }

                lines.add(line);
                linesWithError.add(lineWithError);

                // check that point is locus of line without error
                assertTrue(line.isLocus(point, ABSOLUTE_ERROR));
            }

            final LMedSPoint2DRobustEstimator estimator =
                    new LMedSPoint2DRobustEstimator(this, linesWithError);

            estimator.setStopThreshold(STOP_THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            estimator.setRefinementCoordinatesType(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES);

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final Point2D point2 = estimator.estimate();

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNotNull(estimator.getCovariance());
            assertEquals(estimator.getCovariance().getRows(),
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH);
            assertEquals(estimator.getCovariance().getColumns(),
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH);

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by checking that all lines without
            // error have estimated point as locus
            for (final Line2D l : lines) {
                assertTrue(l.isLocus(point2, ABSOLUTE_ERROR));
            }

            // check that both points are equal
            assertEquals(point.distanceTo(point2), 0.0, ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testEstimateWithHomogeneousRefinement() throws LockedException,
            NotReadyException, RobustEstimatorException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final Point2D point = new HomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    1.0);

            // compute random lines passing through the point
            final int nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            final List<Line2D> lines = new ArrayList<>();
            final List<Line2D> linesWithError = new ArrayList<>();
            Line2D line, lineWithError;
            for (int i = 0; i < nLines; i++) {
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
                } else {
                    // inlier line
                    lineWithError = line;
                }

                lines.add(line);
                linesWithError.add(lineWithError);

                // check that point is locus of line without error
                assertTrue(line.isLocus(point, ABSOLUTE_ERROR));
            }

            final LMedSPoint2DRobustEstimator estimator =
                    new LMedSPoint2DRobustEstimator(this, linesWithError);

            estimator.setStopThreshold(STOP_THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            estimator.setRefinementCoordinatesType(
                    CoordinatesType.HOMOGENEOUS_COORDINATES);

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final Point2D point2 = estimator.estimate();

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNotNull(estimator.getCovariance());
            assertEquals(estimator.getCovariance().getRows(),
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH);
            assertEquals(estimator.getCovariance().getColumns(),
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH);

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by checking that all lines without
            // error have estimated point as locus
            for (final Line2D l : lines) {
                assertTrue(l.isLocus(point2, ABSOLUTE_ERROR));
            }

            // check that both points are equal
            assertEquals(point.distanceTo(point2), 0.0, ABSOLUTE_ERROR);
        }
    }

    @Override
    public void onEstimateStart(final Point2DRobustEstimator estimator) {
        estimateStart++;
        checkLocked((LMedSPoint2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final Point2DRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((LMedSPoint2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(final Point2DRobustEstimator estimator,
                                        final int iteration) {
        estimateNextIteration++;
        checkLocked((LMedSPoint2DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(final Point2DRobustEstimator estimator,
                                         final float progress) {
        estimateProgressChange++;
        checkLocked((LMedSPoint2DRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void checkLocked(final LMedSPoint2DRobustEstimator estimator) {
        try {
            estimator.setStopThreshold(0.5);
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
    }
}
