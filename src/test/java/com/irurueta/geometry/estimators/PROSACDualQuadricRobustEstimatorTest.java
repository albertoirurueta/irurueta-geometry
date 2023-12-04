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

import com.irurueta.geometry.DualQuadric;
import com.irurueta.geometry.DualQuadricNotAvailableException;
import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quadric;
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

public class PROSACDualQuadricRobustEstimatorTest implements
        DualQuadricRobustEstimatorListener {

    private static final int MIN_PLANES = 500;
    private static final int MAX_PLANES = 1000;

    private static final double MIN_RANDOM_POINT_VALUE = -1.0;
    private static final double MAX_RANDOM_POINT_VALUE = 1.0;

    private static final double STD_ERROR = 1.0;

    private static final double MIN_SCORE_ERROR = -0.3;
    private static final double MAX_SCORE_ERROR = 0.3;

    private static final int PERCENTAGE_OUTLIER = 20;

    private static final double THRESHOLD = 1e-7;
    private static final double ABSOLUTE_ERROR = 5e-6;

    private static final int TIMES = 10;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    public void testConstants() {
        assertEquals(9, DualQuadricRobustEstimator.MINIMUM_SIZE);
        assertEquals(0.05f, DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0f);
        assertEquals(0.0f, DualQuadricRobustEstimator.MIN_PROGRESS_DELTA, 0.0f);
        assertEquals(1.0f, DualQuadricRobustEstimator.MAX_PROGRESS_DELTA, 0.0f);
        assertEquals(0.99, DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(5000, DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(0.0, DualQuadricRobustEstimator.MIN_CONFIDENCE, 0.0);
        assertEquals(1.0, DualQuadricRobustEstimator.MAX_CONFIDENCE, 0.0);
        assertEquals(1, DualQuadricRobustEstimator.MIN_ITERATIONS);
        assertEquals(RobustEstimatorMethod.PROMEDS, DualQuadricRobustEstimator.DEFAULT_ROBUST_METHOD);
        assertEquals(1e-7, PROSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(0.0, PROSACDualQuadricRobustEstimator.MIN_THRESHOLD, 0.0);
    }

    @Test
    public void testConstructor() {
        // test constructor without arguments
        PROSACDualQuadricRobustEstimator estimator = new PROSACDualQuadricRobustEstimator();

        // check correctness
        assertEquals(PROSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // test constructor with planes
        final List<Plane> planes = new ArrayList<>();
        for (int i = 0; i < DualQuadricRobustEstimator.MINIMUM_SIZE; i++) {
            planes.add(new Plane());
        }

        estimator = new PROSACDualQuadricRobustEstimator(planes);

        // check correctness
        assertEquals(PROSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(planes, estimator.getPlanes());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        final List<Plane> emptyPlanes = new ArrayList<>();
        estimator = null;
        try {
            estimator = new PROSACDualQuadricRobustEstimator(emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener
        final DualQuadricRobustEstimatorListener listener =
                new DualQuadricRobustEstimatorListener() {

                    @Override
                    public void onEstimateStart(final DualQuadricRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateEnd(final DualQuadricRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final DualQuadricRobustEstimator estimator, final int iteration) {
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final DualQuadricRobustEstimator estimator, final float progress) {
                    }
                };

        estimator = new PROSACDualQuadricRobustEstimator(listener);

        // check correctness
        assertEquals(PROSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // test constructor with listener and points
        estimator = new PROSACDualQuadricRobustEstimator(listener, planes);

        // check correctness
        assertEquals(PROSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(planes, estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACDualQuadricRobustEstimator(listener,
                    emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores
        final double[] qualityScores = new double[
                DualQuadricRobustEstimator.MINIMUM_SIZE];
        estimator = new PROSACDualQuadricRobustEstimator(qualityScores);

        // check correctness
        assertEquals(PROSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        final double[] emptyScores = new double[0];
        estimator = null;
        try {
            estimator = new PROSACDualQuadricRobustEstimator(emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with planes and quality scores
        estimator = new PROSACDualQuadricRobustEstimator(planes, qualityScores);

        // check correctness
        assertEquals(PROSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROSACDualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PROSACDualQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROSACDualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(planes, estimator.getPlanes());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACDualQuadricRobustEstimator(emptyPlanes,
                    qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACDualQuadricRobustEstimator(planes,
                    emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener and quality scores
        estimator = new PROSACDualQuadricRobustEstimator(listener,
                qualityScores);

        // check correctness
        assertEquals(PROSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACDualQuadricRobustEstimator(listener,
                    emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener, points and quality scores
        estimator = new PROSACDualQuadricRobustEstimator(listener, planes,
                qualityScores);

        // check correctness
        assertEquals(PROSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(planes, estimator.getPlanes());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACDualQuadricRobustEstimator(listener,
                    emptyPlanes, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACDualQuadricRobustEstimator(listener, planes,
                    emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetThreshold() throws LockedException {
        final PROSACDualQuadricRobustEstimator estimator =
                new PROSACDualQuadricRobustEstimator();

        // check default value
        assertEquals(PROSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);

        // set new value
        estimator.setThreshold(0.5);

        // check correctness
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
        final PROSACDualQuadricRobustEstimator estimator =
                new PROSACDualQuadricRobustEstimator();

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
        final PROSACDualQuadricRobustEstimator estimator =
                new PROSACDualQuadricRobustEstimator();

        // check default value
        assertEquals(DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
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
        final PROSACDualQuadricRobustEstimator estimator =
                new PROSACDualQuadricRobustEstimator();

        // check default value
        assertEquals(DualQuadricRobustEstimator.DEFAULT_CONFIDENCE,
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
        final PROSACDualQuadricRobustEstimator estimator =
                new PROSACDualQuadricRobustEstimator();

        // check default value
        assertEquals(DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
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
    public void testGetSetPlanes() throws LockedException {
        final PROSACDualQuadricRobustEstimator estimator =
                new PROSACDualQuadricRobustEstimator();

        // check default value
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());

        // set new value
        final List<Plane> planes = new ArrayList<>();
        for (int i = 0; i < DualQuadricRobustEstimator.MINIMUM_SIZE; i++) {
            planes.add(new Plane());
        }
        estimator.setPlanes(planes);

        // check correctness
        assertSame(planes, estimator.getPlanes());
        assertFalse(estimator.isReady());

        // if we set quality scores, then estimator becomes ready
        final double[] qualityScores = new double[planes.size()];
        estimator.setQualityScores(qualityScores);

        assertTrue(estimator.isReady());

        // clearing list makes instance not ready
        planes.clear();

        assertFalse(estimator.isReady());

        // Force IllegalArgumentException
        final List<Plane> emptyPlanes = new ArrayList<>();
        try {
            estimator.setPlanes(emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final PROSACDualQuadricRobustEstimator estimator =
                new PROSACDualQuadricRobustEstimator();

        assertNull(estimator.getQualityScores());

        double[] qualityScores =
                new double[DualQuadricRobustEstimator.MINIMUM_SIZE];
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
            RobustEstimatorException, DualQuadricNotAvailableException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            // instantiate a random circle
            final Point3D center = new HomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE), 1.0);
            final double radius = Math.abs(randomizer.nextDouble(
                    MAX_RANDOM_POINT_VALUE / 2.0,
                    MAX_RANDOM_POINT_VALUE));

            final Sphere sphere = new Sphere(center, radius);
            final Quadric quadric = sphere.toQuadric();
            final DualQuadric dualQuadric = quadric.getDualQuadric();

            // compute planes in the dual quadric locus
            final int nPlanes = randomizer.nextInt(MIN_PLANES, MAX_PLANES);
            final int halfPoints = (int) Math.ceil((double) nPlanes / 2.0);
            final double theta = (double) nPlanes / 360.0 * Math.PI / 180.0;
            final double[] qualityScores = new double[nPlanes];
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            final List<Plane> planes = new ArrayList<>();
            final List<Plane> planesWithError = new ArrayList<>();
            Point3D point;
            Plane plane;
            Plane planeWithError;
            final double[] directorVector = new double[
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            for (int i = 0; i < nPlanes; i++) {
                double angle1 = 0.0;
                double angle2 = 0.0;
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
                directorVector[0] = point.getInhomX() - center.getInhomX();
                directorVector[1] = point.getInhomY() - center.getInhomY();
                directorVector[2] = point.getInhomZ() - center.getInhomZ();

                plane = new Plane(point, directorVector);
                final double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR,
                        MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final double errorA = errorRandomizer.nextDouble();
                    final double errorB = errorRandomizer.nextDouble();
                    final double errorC = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD());

                    final double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    // inlier plane
                    planeWithError = plane;
                }

                planes.add(plane);
                planesWithError.add(planeWithError);

                // check that point without error is within quadric locus
                assertTrue(sphere.isLocus(point, ABSOLUTE_ERROR));
                assertTrue(quadric.isLocus(point, ABSOLUTE_ERROR));
                assertTrue(dualQuadric.isLocus(plane, ABSOLUTE_ERROR));
            }

            final PROSACDualQuadricRobustEstimator estimator =
                    new PROSACDualQuadricRobustEstimator(this, planesWithError,
                            qualityScores);

            estimator.setThreshold(THRESHOLD);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final DualQuadric dualQuadric2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by checking that all planes
            // are within the estimated conic locus
            for (final Plane p : planes) {
                assertTrue(dualQuadric2.isLocus(p, ABSOLUTE_ERROR));
            }
        }
    }

    @Override
    public void onEstimateStart(final DualQuadricRobustEstimator estimator) {
        estimateStart++;
        checkLocked((PROSACDualQuadricRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final DualQuadricRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((PROSACDualQuadricRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(final DualQuadricRobustEstimator estimator,
                                        final int iteration) {
        estimateNextIteration++;
        checkLocked((PROSACDualQuadricRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(final DualQuadricRobustEstimator estimator,
                                         final float progress) {
        estimateProgressChange++;
        checkLocked((PROSACDualQuadricRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void checkLocked(final PROSACDualQuadricRobustEstimator estimator) {
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
            estimator.setPlanes(null);
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
