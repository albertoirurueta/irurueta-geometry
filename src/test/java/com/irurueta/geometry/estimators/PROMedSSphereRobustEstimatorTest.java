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

import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Sphere;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class PROMedSSphereRobustEstimatorTest implements
        SphereRobustEstimatorListener {

    private static final double MIN_RANDOM_VALUE = -1000.0;
    private static final double MAX_RANDOM_VALUE = 1000.0;

    private static final double ABSOLUTE_ERROR = 1e-5;

    private static final int MIN_POINTS = 500;
    private static final int MAX_POINTS = 1000;

    private static final double STOP_THRESHOLD = 1e-6;

    private static final double STD_ERROR = 100.0;

    private static final int PERCENTAGE_OUTLIER = 20;

    private static final int TIMES = 10;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    public void testConstants() {
        assertEquals(4, SphereRobustEstimator.MINIMUM_SIZE);
        assertEquals(0.05f, SphereRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0f);
        assertEquals(0.0f, SphereRobustEstimator.MIN_PROGRESS_DELTA, 0.0f);
        assertEquals(1.0f, SphereRobustEstimator.MAX_PROGRESS_DELTA, 0.0f);
        assertEquals(0.99, SphereRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(5000, SphereRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(0.0, SphereRobustEstimator.MIN_CONFIDENCE, 0.0);
        assertEquals(1.0, SphereRobustEstimator.MAX_CONFIDENCE, 0.0);
        assertEquals(1, SphereRobustEstimator.MIN_ITERATIONS);
        assertEquals(RobustEstimatorMethod.PROMEDS, SphereRobustEstimator.DEFAULT_ROBUST_METHOD);
        assertEquals(1e-3, PROMedSSphereRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(0.0, PROMedSSphereRobustEstimator.MIN_STOP_THRESHOLD, 0.0);
    }

    @Test
    public void testConstructor() {
        // test constructor without arguments
        PROMedSSphereRobustEstimator estimator = new PROMedSSphereRobustEstimator();

        // check correctness
        assertEquals(PROMedSSphereRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(SphereRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SphereRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(SphereRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // test constructor with points
        final List<Point3D> points = new ArrayList<>();
        for (int i = 0; i < SphereRobustEstimator.MINIMUM_SIZE; i++) {
            points.add(Point3D.create());
        }

        estimator = new PROMedSSphereRobustEstimator(points);

        // check correctness
        assertEquals(PROMedSSphereRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(SphereRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SphereRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(SphereRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(points, estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        final List<Point3D> emptyPoints = new ArrayList<>();
        estimator = null;
        try {
            estimator = new PROMedSSphereRobustEstimator(emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener
        final SphereRobustEstimatorListener listener =
                new SphereRobustEstimatorListener() {

                    @Override
                    public void onEstimateStart(final SphereRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateEnd(final SphereRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateNextIteration(final SphereRobustEstimator estimator,
                                                        final int iteration) {
                    }

                    @Override
                    public void onEstimateProgressChange(final SphereRobustEstimator estimator,
                                                         final float progress) {
                    }
                };

        estimator = new PROMedSSphereRobustEstimator(listener);

        // check correctness
        assertEquals(PROMedSSphereRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(SphereRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SphereRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(SphereRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // test constructor with listener and points
        estimator = new PROMedSSphereRobustEstimator(listener, points);

        // check correctness
        assertEquals(PROMedSSphereRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(SphereRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SphereRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(SphereRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(points, estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSSphereRobustEstimator(listener, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores
        final double[] qualityScores = new double[SphereRobustEstimator.MINIMUM_SIZE];
        estimator = new PROMedSSphereRobustEstimator(qualityScores);

        // check correctness
        assertEquals(PROMedSSphereRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(SphereRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SphereRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(SphereRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        final double[] emptyScores = new double[0];
        estimator = null;
        try {
            estimator = new PROMedSSphereRobustEstimator(emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with points and scores
        estimator = new PROMedSSphereRobustEstimator(points, qualityScores);

        // check correctness
        assertEquals(PROMedSSphereRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(SphereRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SphereRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(SphereRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(points, estimator.getPoints());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSSphereRobustEstimator(emptyPoints,
                    qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSSphereRobustEstimator(points, emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener and quality scores
        estimator = new PROMedSSphereRobustEstimator(listener, qualityScores);

        // check correctness
        assertEquals(PROMedSSphereRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(SphereRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SphereRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(SphereRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSSphereRobustEstimator(listener, emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener, points and quality scores
        estimator = new PROMedSSphereRobustEstimator(listener, points,
                qualityScores);

        // check correctness
        assertEquals(PROMedSSphereRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(SphereRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SphereRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(SphereRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(points, estimator.getPoints());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSSphereRobustEstimator(listener, emptyPoints,
                    qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSSphereRobustEstimator(listener, points,
                    emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetThreshold() throws LockedException {
        final PROMedSSphereRobustEstimator estimator =
                new PROMedSSphereRobustEstimator();

        // check default value
        assertEquals(PROMedSSphereRobustEstimator.DEFAULT_STOP_THRESHOLD,
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
        final PROMedSSphereRobustEstimator estimator =
                new PROMedSSphereRobustEstimator();

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
        final PROMedSSphereRobustEstimator estimator =
                new PROMedSSphereRobustEstimator();

        // check default value
        assertEquals(SphereRobustEstimator.DEFAULT_PROGRESS_DELTA,
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
        final PROMedSSphereRobustEstimator estimator =
                new PROMedSSphereRobustEstimator();

        // check default value
        assertEquals(SphereRobustEstimator.DEFAULT_CONFIDENCE,
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
        final PROMedSSphereRobustEstimator estimator =
                new PROMedSSphereRobustEstimator();

        // check default value
        assertEquals(SphereRobustEstimator.DEFAULT_MAX_ITERATIONS,
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
    public void testGetSetPoints() throws LockedException {
        final PROMedSSphereRobustEstimator estimator =
                new PROMedSSphereRobustEstimator();

        // check default value
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());

        // set new value
        final List<Point3D> points = new ArrayList<>();
        for (int i = 0; i < SphereRobustEstimator.MINIMUM_SIZE; i++) {
            points.add(Point3D.create());
        }
        estimator.setPoints(points);

        // check correctness
        assertSame(points, estimator.getPoints());
        assertFalse(estimator.isReady());

        // if we set quality scores, then estimator becomes ready
        final double[] qualityScores = new double[points.size()];
        estimator.setQualityScores(qualityScores);

        assertTrue(estimator.isReady());

        // clearing list makes instance not ready
        points.clear();

        assertFalse(estimator.isReady());

        // Force IllegalArgumentException
        final List<Point3D> emptyPoints = new ArrayList<>();
        try {
            estimator.setPoints(emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final PROMedSSphereRobustEstimator estimator =
                new PROMedSSphereRobustEstimator();

        assertNull(estimator.getQualityScores());

        double[] qualityScores = new double[SphereRobustEstimator.MINIMUM_SIZE];
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
    public void testEstimate() throws LockedException, NotReadyException,
            RobustEstimatorException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            // instantiate a random circle
            final Point3D center = new HomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    1.0);
            final double radius = Math.abs(randomizer.nextDouble(
                    MAX_RANDOM_VALUE / 2.0,
                    MAX_RANDOM_VALUE));

            final Sphere sphere = new Sphere(center, radius);

            // compute points in the circle locus
            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final int halfPoints = (int) Math.ceil((double) nPoints / 2.0);
            final double theta = (double) nPoints / 360.0 * Math.PI / 180.0;
            final double[] qualityScores = new double[nPoints];
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            final List<Point3D> points = new ArrayList<>();
            final List<Point3D> pointsWithError = new ArrayList<>();
            Point3D point, pointWithError;
            for (int i = 0; i < nPoints; i++) {
                double angle1 = 0.0, angle2 = 0.0;
                if (i < halfPoints) {
                    angle1 = theta * (double) i;
                } else {
                    angle2 = theta * (double) (i - halfPoints);
                }
                point = new HomogeneousPoint3D(
                        center.getInhomX() + radius * Math.cos(angle1) *
                                Math.cos(angle2),
                        center.getInhomY() + radius * Math.sin(angle1) *
                                Math.cos(angle2),
                        center.getInhomZ() + radius * Math.sin(angle2),
                        1.0);
                qualityScores[i] = 1.0;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double errorZ = errorRandomizer.nextDouble();
                    pointWithError = new HomogeneousPoint3D(
                            point.getInhomX() + errorX,
                            point.getInhomY() + errorY,
                            point.getInhomZ() + errorZ, 1.0);

                    final double error = Math.sqrt(errorX * errorX + errorY * errorY +
                            errorZ * errorZ);
                    qualityScores[i] = 1.0 / (1.0 + error);
                } else {
                    // inlier point
                    pointWithError = point;
                }

                points.add(point);
                pointsWithError.add(pointWithError);

                // check that point without error is within circle locus
                assertTrue(sphere.isLocus(point, ABSOLUTE_ERROR));
            }

            final PROMedSSphereRobustEstimator estimator =
                    new PROMedSSphereRobustEstimator(this, pointsWithError,
                            qualityScores);

            estimator.setStopThreshold(STOP_THRESHOLD);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final Sphere sphere2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by checking that all points
            // are within the estimated circle locus
            boolean valid = true;
            for (final Point3D p : points) {
                if (!sphere2.isLocus(p, ABSOLUTE_ERROR)) {
                    valid = false;
                    break;
                }
                assertTrue(sphere2.isLocus(p, ABSOLUTE_ERROR));
            }

            if (!valid) {
                continue;
            }

            // check that both spheres are equal
            assertEquals(0.0, sphere.getCenter().distanceTo(sphere2.getCenter()),
                    ABSOLUTE_ERROR);
            assertEquals(sphere.getRadius(), sphere2.getRadius(), ABSOLUTE_ERROR);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onEstimateStart(final SphereRobustEstimator estimator) {
        estimateStart++;
        checkLocked((PROMedSSphereRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final SphereRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((PROMedSSphereRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(final SphereRobustEstimator estimator,
                                        final int iteration) {
        estimateNextIteration++;
        checkLocked((PROMedSSphereRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(final SphereRobustEstimator estimator,
                                         final float progress) {
        estimateProgressChange++;
        checkLocked((PROMedSSphereRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void checkLocked(final PROMedSSphereRobustEstimator estimator) {
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
            estimator.setPoints(null);
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
