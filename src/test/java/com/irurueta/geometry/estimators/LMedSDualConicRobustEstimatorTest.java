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

import com.irurueta.geometry.Circle;
import com.irurueta.geometry.Conic;
import com.irurueta.geometry.DualConic;
import com.irurueta.geometry.DualConicNotAvailableException;
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

public class LMedSDualConicRobustEstimatorTest implements
        DualConicRobustEstimatorListener {

    private static final int MIN_LINES = 500;
    private static final int MAX_LINES = 1000;

    private static final double MIN_RANDOM_POINT_VALUE = -1.0;
    private static final double MAX_RANDOM_POINT_VALUE = 1.0;

    private static final double STD_ERROR = 1.0;
    private static final int PERCENTAGE_OUTLIER = 20;

    private static final double STOP_THRESHOLD = 1e-9;
    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int TIMES = 100;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    public void testConstants() {
        assertEquals(1e-9, LMedSDualConicRobustEstimator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(0.0, LMedSDualConicRobustEstimator.MIN_STOP_THRESHOLD,
                0.0);
    }

    @Test
    public void testConstructor() {
        LMedSDualConicRobustEstimator estimator;

        // test constructor without arguments
        estimator = new LMedSDualConicRobustEstimator();

        // check correctness
        assertEquals(LMedSDualConicRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualConicRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualConicRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualConicRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // test constructor with lines
        final List<Line2D> lines = new ArrayList<>();
        for (int i = 0; i < DualConicRobustEstimator.MINIMUM_SIZE; i++) {
            lines.add(new Line2D());
        }

        estimator = new LMedSDualConicRobustEstimator(lines);

        // check correctness
        assertEquals(LMedSDualConicRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualConicRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualConicRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualConicRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(lines, estimator.getLines());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        final List<Line2D> emptyLines = new ArrayList<>();
        estimator = null;
        try {
            estimator = new LMedSDualConicRobustEstimator(emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener
        final DualConicRobustEstimatorListener listener =
                new DualConicRobustEstimatorListener() {

                    @Override
                    public void onEstimateStart(final DualConicRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateEnd(final DualConicRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final DualConicRobustEstimator estimator, final int iteration) {
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final DualConicRobustEstimator estimator, final float progress) {
                    }
                };

        estimator = new LMedSDualConicRobustEstimator(listener);

        // check correctness
        assertEquals(LMedSDualConicRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualConicRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualConicRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualConicRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // test constructor with listener and points
        estimator = new LMedSDualConicRobustEstimator(listener, lines);

        // check correctness
        assertEquals(LMedSDualConicRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualConicRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualConicRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualConicRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(lines, estimator.getLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LMedSDualConicRobustEstimator(listener,
                    emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetStopThreshold() throws LockedException {
        final LMedSDualConicRobustEstimator estimator =
                new LMedSDualConicRobustEstimator();

        // check default value
        assertEquals(LMedSDualConicRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);

        // set new value
        estimator.setStopThreshold(0.5);

        // check correctness
        assertEquals(0.5, estimator.getStopThreshold(), 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setStopThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final LMedSDualConicRobustEstimator estimator =
                new LMedSDualConicRobustEstimator();

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
        final LMedSDualConicRobustEstimator estimator =
                new LMedSDualConicRobustEstimator();

        // check default value
        assertEquals(DualConicRobustEstimator.DEFAULT_PROGRESS_DELTA,
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
        final LMedSDualConicRobustEstimator estimator =
                new LMedSDualConicRobustEstimator();

        // check default value
        assertEquals(DualConicRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);

        // set new value
        estimator.setConfidence(0.5);

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
        final LMedSDualConicRobustEstimator estimator =
                new LMedSDualConicRobustEstimator();

        // check default value
        assertEquals(DualConicRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(1);

        // check correctness
        assertEquals(1, estimator.getMaxIterations());

        // Force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetLines() throws LockedException {
        final LMedSDualConicRobustEstimator estimator =
                new LMedSDualConicRobustEstimator();

        // check default value
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());

        // set new value
        final List<Line2D> lines = new ArrayList<>();
        for (int i = 0; i < DualConicRobustEstimator.MINIMUM_SIZE; i++) {
            lines.add(new Line2D());
        }
        estimator.setLines(lines);

        // check correctness
        assertSame(lines, estimator.getLines());
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
        final LMedSDualConicRobustEstimator estimator =
                new LMedSDualConicRobustEstimator();

        assertNull(estimator.getQualityScores());

        final double[] qualityScores =
                new double[DualConicRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertNull(estimator.getQualityScores());
    }

    @Test
    public void testEstimate() throws LockedException, NotReadyException,
            RobustEstimatorException, DualConicNotAvailableException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            // instantiate a random circle
            final Point2D center = new HomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE), 1.0);
            final double radius = Math.abs(randomizer.nextDouble(
                    MAX_RANDOM_POINT_VALUE / 2.0,
                    MAX_RANDOM_POINT_VALUE));

            final Circle circle = new Circle(center, radius);
            final Conic conic = circle.toConic();
            final DualConic dualConic = conic.getDualConic();

            // compute lines in the dual conic locus
            final int nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);
            final double theta = (double) nLines / 360.0 * Math.PI / 180.0;
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            final List<Line2D> lines = new ArrayList<>();
            final List<Line2D> linesWithError = new ArrayList<>();
            Point2D point;
            Line2D line, lineWithError;
            final double[] directorVector = new double[
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            for (int i = 0; i < nLines; i++) {
                final double angle = theta * (double) i;
                point = new HomogeneousPoint2D(
                        center.getInhomX() + radius * Math.cos(angle),
                        center.getInhomY() + radius * Math.sin(angle), 1.0);
                directorVector[0] = point.getInhomX() - center.getInhomX();
                directorVector[1] = point.getInhomY() - center.getInhomY();

                line = new Line2D(point, directorVector);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final double errorA = errorRandomizer.nextDouble();
                    final double errorB = errorRandomizer.nextDouble();
                    lineWithError = new Line2D(line.getA() + errorA,
                            line.getB() + errorB, line.getC());
                } else {
                    // inlier point
                    lineWithError = line;
                }

                lines.add(line);
                linesWithError.add(lineWithError);

                // check that point without error is within conic locus
                assertTrue(circle.isLocus(point, ABSOLUTE_ERROR));
                assertTrue(conic.isLocus(point, ABSOLUTE_ERROR));
                assertTrue(dualConic.isLocus(line, ABSOLUTE_ERROR));
            }

            final LMedSDualConicRobustEstimator estimator =
                    new LMedSDualConicRobustEstimator(this, linesWithError);

            estimator.setStopThreshold(STOP_THRESHOLD);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final DualConic dualConic2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by checking that all points
            // are within the estimated conic locus
            for (final Line2D p : lines) {
                assertTrue(dualConic2.isLocus(p, ABSOLUTE_ERROR));
            }
        }
    }

    @Override
    public void onEstimateStart(final DualConicRobustEstimator estimator) {
        estimateStart++;
        checkLocked((LMedSDualConicRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final DualConicRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((LMedSDualConicRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(final DualConicRobustEstimator estimator,
                                        final int iteration) {
        estimateNextIteration++;
        checkLocked((LMedSDualConicRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(final DualConicRobustEstimator estimator,
                                         final float progress) {
        estimateProgressChange++;
        checkLocked((LMedSDualConicRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void checkLocked(final LMedSDualConicRobustEstimator estimator) {
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
