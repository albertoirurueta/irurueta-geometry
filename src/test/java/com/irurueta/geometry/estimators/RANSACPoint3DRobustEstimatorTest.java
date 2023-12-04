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

import com.irurueta.geometry.*;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class RANSACPoint3DRobustEstimatorTest implements
        Point3DRobustEstimatorListener {

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double ABSOLUTE_ERROR = 1e-5;

    private static final int MIN_LINES = 500;
    private static final int MAX_LINES = 1000;

    private static final double THRESHOLD = 1e-6;

    private static final double STD_ERROR = 100.0;

    private static final int PERCENTAGE_OUTLIER = 10;

    private static final int TIMES = 10;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    public void testConstants() {
        assertEquals(3, Point3DRobustEstimator.MINIMUM_SIZE);
        assertEquals(0.05f, Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0f);
        assertEquals(0.0f, Point3DRobustEstimator.MIN_PROGRESS_DELTA, 0.0f);
        assertEquals(1.0f, Point3DRobustEstimator.MAX_PROGRESS_DELTA, 0.0f);
        assertEquals(0.99, Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(5000, Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(0.0, Point3DRobustEstimator.MIN_CONFIDENCE, 0.0);
        assertEquals(1.0, Point3DRobustEstimator.MAX_CONFIDENCE, 0.0);
        assertEquals(1, Point3DRobustEstimator.MIN_ITERATIONS);
        assertEquals(RobustEstimatorMethod.PROMEDS, Point3DRobustEstimator.DEFAULT_ROBUST_METHOD);
        assertTrue(Point3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(Point3DRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertEquals(1.0, RANSACPoint3DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(0.0, RANSACPoint3DRobustEstimator.MIN_THRESHOLD, 0.0);
        assertFalse(RANSACPoint3DRobustEstimator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertFalse(RANSACPoint3DRobustEstimator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
    }

    @Test
    public void testConstructor() {
        // test constructor without arguments
        RANSACPoint3DRobustEstimator estimator = new RANSACPoint3DRobustEstimator();

        // check correctness
        assertEquals(RANSACPoint3DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(Point3DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getPlanes());
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

        // test constructor with planes
        final List<Plane> planes = new ArrayList<>();
        for (int i = 0; i < Point3DRobustEstimator.MINIMUM_SIZE; i++) {
            planes.add(new Plane());
        }

        estimator = new RANSACPoint3DRobustEstimator(planes);

        // check correctness
        assertEquals(RANSACPoint3DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(Point3DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(planes, estimator.getPlanes());
        assertTrue(estimator.isReady());
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
        final List<Plane> emptyPlanes = new ArrayList<>();
        estimator = null;
        try {
            estimator = new RANSACPoint3DRobustEstimator(emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener
        final Point3DRobustEstimatorListener listener =
                new Point3DRobustEstimatorListener() {

                    @Override
                    public void onEstimateStart(final Point3DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateEnd(final Point3DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final Point3DRobustEstimator estimator, final int iteration) {
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final Point3DRobustEstimator estimator, final float progress) {
                    }
                };

        estimator = new RANSACPoint3DRobustEstimator(listener);

        // check correctness
        assertEquals(RANSACPoint3DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(Point3DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getPlanes());
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
        estimator = new RANSACPoint3DRobustEstimator(listener, planes);

        // check correctness
        assertEquals(RANSACPoint3DRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(Point3DRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(planes, estimator.getPlanes());
        assertTrue(estimator.isReady());
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
            estimator = new RANSACPoint3DRobustEstimator(listener, emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetThreshold() throws LockedException {
        final RANSACPoint3DRobustEstimator estimator =
                new RANSACPoint3DRobustEstimator();

        // check default value
        assertEquals(RANSACPoint3DRobustEstimator.DEFAULT_THRESHOLD,
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
        final RANSACPoint3DRobustEstimator estimator =
                new RANSACPoint3DRobustEstimator();

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
        final RANSACPoint3DRobustEstimator estimator =
                new RANSACPoint3DRobustEstimator();

        // check default value
        assertEquals(Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
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
        final RANSACPoint3DRobustEstimator estimator =
                new RANSACPoint3DRobustEstimator();

        // check default value
        assertEquals(Point3DRobustEstimator.DEFAULT_CONFIDENCE,
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
        final RANSACPoint3DRobustEstimator estimator =
                new RANSACPoint3DRobustEstimator();

        // check default value
        assertEquals(Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS,
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
    public void testGetSetPlanes() throws LockedException {
        final RANSACPoint3DRobustEstimator estimator =
                new RANSACPoint3DRobustEstimator();

        // check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());

        // set new value
        final List<Plane> planes = new ArrayList<>();
        for (int i = 0; i < Point3DRobustEstimator.MINIMUM_SIZE; i++) {
            planes.add(new Plane());
        }
        estimator.setPlanes(planes);

        // check correctness
        assertSame(planes, estimator.getPlanes());
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
        final RANSACPoint3DRobustEstimator estimator =
                new RANSACPoint3DRobustEstimator();

        assertNull(estimator.getQualityScores());

        final double[] qualityScores = new double[Point3DRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertNull(estimator.getQualityScores());
    }

    @Test
    public void testIsSetResultRefined() throws LockedException {
        final RANSACPoint3DRobustEstimator estimator =
                new RANSACPoint3DRobustEstimator();

        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check correctness
        assertFalse(estimator.isResultRefined());
    }

    @Test
    public void testGetSetRefinementCoordinatesType() throws LockedException {
        final RANSACPoint3DRobustEstimator estimator =
                new RANSACPoint3DRobustEstimator();

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
        final RANSACPoint3DRobustEstimator estimator =
                new RANSACPoint3DRobustEstimator();

        assertFalse(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(true);

        // check correctness
        assertTrue(estimator.isCovarianceKept());
    }

    @Test
    public void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        final RANSACPoint3DRobustEstimator estimator =
                new RANSACPoint3DRobustEstimator();

        assertFalse(estimator.isComputeAndKeepInliersEnabled());

        // set new value
        estimator.setComputeAndKeepInliersEnabled(true);

        // check correctness
        assertTrue(estimator.isComputeAndKeepInliersEnabled());
    }

    @Test
    public void testIsSetComputeAndKeepResidualsEnabled()
            throws LockedException {
        final RANSACPoint3DRobustEstimator estimator =
                new RANSACPoint3DRobustEstimator();

        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // set new value
        estimator.setComputeAndKeepResidualsEnabled(true);

        // check correctness
        assertTrue(estimator.isComputeAndKeepResidualsEnabled());
    }

    @Test
    public void testEstimateWithoutRefinement() throws LockedException,
            NotReadyException, RobustEstimatorException,
            ColinearPointsException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final Point3D point = new HomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    1.0);

            // compute random lines passing through the point
            final int nPlanes = randomizer.nextInt(MIN_LINES, MAX_LINES);
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            final List<Plane> planes = new ArrayList<>();
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
                    // inlier line
                    planeWithError = plane;
                }

                planes.add(plane);
                planesWithError.add(planeWithError);

                // check that point is locus of plane without error
                assertTrue(plane.isLocus(point, ABSOLUTE_ERROR));
            }

            final RANSACPoint3DRobustEstimator estimator =
                    new RANSACPoint3DRobustEstimator(this, planesWithError);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final Point3D point2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by checking that all planes
            // without error have estimated point as locus
            for (final Plane p : planes) {
                assertTrue(p.isLocus(point2, ABSOLUTE_ERROR));
            }

            // check that both points are equal
            assertEquals(0.0, point.distanceTo(point2), ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testEstimateWithInhomogeneousRefinement()
            throws LockedException, NotReadyException, RobustEstimatorException,
            ColinearPointsException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final Point3D point = new HomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    1.0);

            // compute random lines passing through the point
            final int nPlanes = randomizer.nextInt(MIN_LINES, MAX_LINES);
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            final List<Plane> planes = new ArrayList<>();
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
                    // inlier line
                    planeWithError = plane;
                }

                planes.add(plane);
                planesWithError.add(planeWithError);

                // check that point is locus of plane without error
                assertTrue(plane.isLocus(point, ABSOLUTE_ERROR));
            }

            final RANSACPoint3DRobustEstimator estimator =
                    new RANSACPoint3DRobustEstimator(this, planesWithError);

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

            final Point3D point2 = estimator.estimate();

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNotNull(estimator.getCovariance());
            assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                    estimator.getCovariance().getRows());
            assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                    estimator.getCovariance().getColumns());

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by checking that all planes
            // without error have estimated point as locus
            for (final Plane p : planes) {
                assertTrue(p.isLocus(point2, ABSOLUTE_ERROR));
            }

            // check that both points are equal
            assertEquals(0.0, point.distanceTo(point2), ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testEstimateWithHomogeneousRefinement()
            throws LockedException, NotReadyException, RobustEstimatorException,
            ColinearPointsException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final Point3D point = new HomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                1.0);

        // compute random lines passing through the point
        final int nPlanes = randomizer.nextInt(MIN_LINES, MAX_LINES);
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_ERROR);
        final List<Plane> planes = new ArrayList<>();
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
                // inlier line
                planeWithError = plane;
            }

            planes.add(plane);
            planesWithError.add(planeWithError);

            // check that point is locus of plane without error
            assertTrue(plane.isLocus(point, ABSOLUTE_ERROR));
        }

        final RANSACPoint3DRobustEstimator estimator =
                new RANSACPoint3DRobustEstimator(this, planesWithError);

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

        final Point3D point2 = estimator.estimate();

        assertNotNull(estimator.getInliersData());
        assertNotNull(estimator.getInliersData().getInliers());
        assertNotNull(estimator.getInliersData().getResiduals());
        assertTrue(estimator.getInliersData().getNumInliers() > 0);
        assertNotNull(estimator.getCovariance());
        assertEquals(Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH,
                estimator.getCovariance().getRows());
        assertEquals(Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH,
                estimator.getCovariance().getColumns());

        assertEquals(1, estimateStart);
        assertEquals(1, estimateEnd);
        assertTrue(estimateNextIteration > 0);
        assertTrue(estimateProgressChange >= 0);
        reset();

        // check correctness of estimation by checking that all planes
        // without error have estimated point as locus
        for (final Plane p : planes) {
            assertTrue(p.isLocus(point2, ABSOLUTE_ERROR));
        }

        // check that both points are equal
        assertEquals(0.0, point.distanceTo(point2), ABSOLUTE_ERROR);
    }

    @Override
    public void onEstimateStart(final Point3DRobustEstimator estimator) {
        estimateStart++;
        checkLocked((RANSACPoint3DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final Point3DRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((RANSACPoint3DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(final Point3DRobustEstimator estimator,
                                        final int iteration) {
        estimateNextIteration++;
        checkLocked((RANSACPoint3DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(final Point3DRobustEstimator estimator,
                                         final float progress) {
        estimateProgressChange++;
        checkLocked((RANSACPoint3DRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void checkLocked(final RANSACPoint3DRobustEstimator estimator) {
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
        try {
            estimator.setRefinementCoordinatesType(
                    CoordinatesType.HOMOGENEOUS_COORDINATES);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setCovarianceKept(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setComputeAndKeepInliersEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setComputeAndKeepResidualsEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        assertTrue(estimator.isLocked());
    }
}
