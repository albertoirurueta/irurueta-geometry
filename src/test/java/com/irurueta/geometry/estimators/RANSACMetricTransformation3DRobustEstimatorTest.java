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

package com.irurueta.geometry.estimators;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.MetricTransformation3D;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Utils;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.Test;

public class RANSACMetricTransformation3DRobustEstimatorTest implements
        MetricTransformation3DRobustEstimatorListener {

    private static final double MIN_RANDOM_VALUE = -100.0;

    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_ANGLE_DEGREES = -90.0;

    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double MIN_TRANSLATION = -100.0;

    private static final double MAX_TRANSLATION = 100.0;

    private static final double MIN_SCALE = 0.5;

    private static final double MAX_SCALE = 2.0;

    private static final double THRESHOLD = 1.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final double STD_ERROR = 100.0;

    private static final int TIMES = 10;

    private static final int MIN_POINTS = 500;

    private static final int MAX_POINTS = 1000;

    private static final int PERCENTAGE_OUTLIER = 20;

    private int estimateStart;

    private int estimateEnd;

    private int estimateNextIteration;

    private int estimateProgressChange;

    @Test
    public void testConstructor() {
        // test constructor without arguments
        RANSACMetricTransformation3DRobustEstimator estimator =
                new RANSACMetricTransformation3DRobustEstimator();

        assertEquals(estimator.getThreshold(),
                RANSACMetricTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACMetricTransformation3DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACMetricTransformation3DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                MetricTransformation3DRobustEstimator.MINIMUM_SIZE);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // test constructor with points
        List<Point3D> inputPoints = new ArrayList<>();
        List<Point3D> outputPoints = new ArrayList<>();
        for (int i = 0; i < MetricTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }

        estimator = new RANSACMetricTransformation3DRobustEstimator(
                inputPoints, outputPoints);

        assertEquals(estimator.getThreshold(),
                RANSACMetricTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACMetricTransformation3DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACMetricTransformation3DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                MetricTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                MetricTransformation3DRobustEstimator.MINIMUM_SIZE);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        final List<Point3D> pointsEmpty = new ArrayList<>();
        estimator = null;
        try {
            // not enough points
            estimator = new RANSACMetricTransformation3DRobustEstimator(
                    pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new RANSACMetricTransformation3DRobustEstimator(
                    inputPoints, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener
        estimator = new RANSACMetricTransformation3DRobustEstimator(this);

        assertEquals(estimator.getThreshold(),
                RANSACMetricTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACMetricTransformation3DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACMetricTransformation3DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                MetricTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                MetricTransformation3DRobustEstimator.MINIMUM_SIZE);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // test constructor with listener and points
        estimator = new RANSACMetricTransformation3DRobustEstimator(
                this, inputPoints, outputPoints);

        assertEquals(estimator.getThreshold(),
                RANSACMetricTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACMetricTransformation3DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACMetricTransformation3DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                MetricTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                MetricTransformation3DRobustEstimator.MINIMUM_SIZE);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough points
            estimator = new RANSACMetricTransformation3DRobustEstimator(
                    this, pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new RANSACMetricTransformation3DRobustEstimator(
                    this, inputPoints, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor without arguments
        estimator = new RANSACMetricTransformation3DRobustEstimator(true);

        assertEquals(estimator.getThreshold(),
                RANSACMetricTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACMetricTransformation3DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACMetricTransformation3DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                MetricTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // test constructor with points
        inputPoints = new ArrayList<>();
        outputPoints = new ArrayList<>();
        for (int i = 0; i < MetricTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE; i++) {
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }

        estimator = new RANSACMetricTransformation3DRobustEstimator(
                inputPoints, outputPoints, true);

        assertEquals(estimator.getThreshold(),
                RANSACMetricTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACMetricTransformation3DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACMetricTransformation3DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                MetricTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                MetricTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough points
            estimator = new RANSACMetricTransformation3DRobustEstimator(
                    pointsEmpty, pointsEmpty, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new RANSACMetricTransformation3DRobustEstimator(
                    inputPoints, pointsEmpty, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener
        estimator = new RANSACMetricTransformation3DRobustEstimator(this, true);

        assertEquals(estimator.getThreshold(),
                RANSACMetricTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACMetricTransformation3DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACMetricTransformation3DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                MetricTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                MetricTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // test constructor with listener and points
        estimator = new RANSACMetricTransformation3DRobustEstimator(
                this, inputPoints, outputPoints, true);

        assertEquals(estimator.getThreshold(),
                RANSACMetricTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACMetricTransformation3DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACMetricTransformation3DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                MetricTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                MetricTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough points
            estimator = new RANSACMetricTransformation3DRobustEstimator(
                    this, pointsEmpty, pointsEmpty, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new RANSACMetricTransformation3DRobustEstimator(
                    this, inputPoints, pointsEmpty, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetStopThreshold() throws LockedException {
        final RANSACMetricTransformation3DRobustEstimator estimator =
                new RANSACMetricTransformation3DRobustEstimator();

        // check default value
        assertEquals(estimator.getThreshold(),
                RANSACMetricTransformation3DRobustEstimator.
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
        final RANSACMetricTransformation3DRobustEstimator estimator =
                new RANSACMetricTransformation3DRobustEstimator();

        // check default value
        assertEquals(estimator.getConfidence(),
                RANSACMetricTransformation3DRobustEstimator.
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
        final RANSACMetricTransformation3DRobustEstimator estimator =
                new RANSACMetricTransformation3DRobustEstimator();

        // check default value
        assertEquals(estimator.getMaxIterations(),
                RANSACMetricTransformation3DRobustEstimator.
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
        final RANSACMetricTransformation3DRobustEstimator estimator =
                new RANSACMetricTransformation3DRobustEstimator();

        // check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());

        // set new value
        final List<Point3D> inputPoints = new ArrayList<>();
        final List<Point3D> outputPoints = new ArrayList<>();
        for (int i = 0; i < RANSACMetricTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }

        estimator.setPoints(inputPoints, outputPoints);

        // check correctness
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());

        // Force IllegalArgumentException
        final List<Point3D> pointsEmpty = new ArrayList<>();
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
    public void testGetSetListenerAndIsListenerAvailable()
            throws LockedException {
        final RANSACMetricTransformation3DRobustEstimator estimator =
                new RANSACMetricTransformation3DRobustEstimator();

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
    public void testIsSetWeakMinimumPointsAllowed() throws LockedException {
        final RANSACMetricTransformation3DRobustEstimator estimator =
                new RANSACMetricTransformation3DRobustEstimator();

        // check default value
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                MetricTransformation3DRobustEstimator.MINIMUM_SIZE);

        // set new value
        estimator.setWeakMinimumSizeAllowed(true);

        // check correctness
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                MetricTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE);
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final RANSACMetricTransformation3DRobustEstimator estimator =
                new RANSACMetricTransformation3DRobustEstimator();

        // check default value
        assertEquals(estimator.getProgressDelta(),
                MetricTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA,
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
        final RANSACMetricTransformation3DRobustEstimator estimator =
                new RANSACMetricTransformation3DRobustEstimator();

        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check correctness
        assertFalse(estimator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final RANSACMetricTransformation3DRobustEstimator estimator =
                new RANSACMetricTransformation3DRobustEstimator();

        assertFalse(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(true);

        // check correctness
        assertTrue(estimator.isCovarianceKept());
    }

    @Test
    public void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        final RANSACMetricTransformation3DRobustEstimator estimator =
                new RANSACMetricTransformation3DRobustEstimator();

        assertFalse(estimator.isComputeAndKeepInliersEnabled());

        // set new value
        estimator.setComputeAndKeepInliersEnabled(true);

        // check correctness
        assertTrue(estimator.isComputeAndKeepInliersEnabled());
    }

    @Test
    public void testIsSetComputeAndKeepResidualsEnabled()
            throws LockedException {
        final RANSACMetricTransformation3DRobustEstimator estimator =
                new RANSACMetricTransformation3DRobustEstimator();

        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // set new value
        estimator.setComputeAndKeepResidualsEnabled(true);

        // check correctness
        assertTrue(estimator.isComputeAndKeepResidualsEnabled());
    }

    @Test
    public void testEstimateWithoutRefinement() throws LockedException,
            NotReadyException, RobustEstimatorException {
        // create an euclidean transformation
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double roll = Utils.convertToRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Utils.convertToRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Utils.convertToRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final Quaternion q = new Quaternion(roll, pitch, yaw);
        q.normalize();

        final double scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);

        final double[] translation = new double[3];
        randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

        final MetricTransformation3D transformation1 =
                new MetricTransformation3D(q, translation, scale);

        // generate random points
        final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        final List<Point3D> inputPoints = new ArrayList<>();
        final List<Point3D> outputPoints = new ArrayList<>();
        final List<Point3D> outputPointsWithError = new ArrayList<>();
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_ERROR);
        for (int i = 0; i < nPoints; i++) {
            final Point3D inputPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE));
            final Point3D outputPoint = transformation1.transformAndReturnNew(
                    inputPoint);
            final Point3D outputPointWithError;
            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                // point is outlier
                final double errorX = errorRandomizer.nextDouble();
                final double errorY = errorRandomizer.nextDouble();
                final double errorZ = errorRandomizer.nextDouble();
                outputPointWithError = new InhomogeneousPoint3D(
                        outputPoint.getInhomX() + errorX,
                        outputPoint.getInhomY() + errorY,
                        outputPoint.getInhomZ() + errorZ);
            } else {
                // inlier point (without error)
                outputPointWithError = outputPoint;
            }

            inputPoints.add(inputPoint);
            outputPoints.add(outputPoint);
            outputPointsWithError.add(outputPointWithError);
        }

        final RANSACMetricTransformation3DRobustEstimator estimator =
                new RANSACMetricTransformation3DRobustEstimator(this,
                        inputPoints, outputPointsWithError);

        estimator.setThreshold(THRESHOLD);
        estimator.setResultRefined(false);
        estimator.setCovarianceKept(false);

        assertEquals(estimateStart, 0);
        assertEquals(estimateEnd, 0);
        assertEquals(estimateNextIteration, 0);
        assertEquals(estimateProgressChange, 0);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());

        final MetricTransformation3D transformation2 = estimator.estimate();

        assertEquals(estimateStart, 1);
        assertEquals(estimateEnd, 1);
        assertTrue(estimateNextIteration > 0);
        assertTrue(estimateProgressChange >= 0);
        reset();

        // check correctness of estimation by transforming input points
        // using estimated transformation (transformation2) and checking
        // that output points are equal to the original output points without
        // error
        Point3D p1;
        Point3D p2;
        for (int i = 0; i < nPoints; i++) {
            p1 = outputPoints.get(i);
            p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
            assertEquals(p1.distanceTo(p2), 0.0,
                    ABSOLUTE_ERROR);
        }

        // check parameters of estimated transformation
        final Quaternion q2 = transformation2.getRotation().toQuaternion();
        q2.normalize();
        final double[] translation2 = transformation2.getTranslation();
        final double scale2 = transformation2.getScale();

        assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
        assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
        assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
        assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
        assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
        assertEquals(scale, scale2, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateCoplanarWithoutRefinement() throws LockedException,
            NotReadyException, RobustEstimatorException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            // create an euclidean transformation
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double roll = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final Quaternion q = new Quaternion(roll, pitch, yaw);
            q.normalize();

            final double scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);

            final double[] translation = new double[3];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

            final MetricTransformation3D transformation1 =
                    new MetricTransformation3D(q, translation, scale);

            // generate random plane
            final double a = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final double b = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final double c = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final double d = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final Plane plane = new Plane(a, b, c, d);

            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final List<Point3D> inputPoints = new ArrayList<>();
            final List<Point3D> outputPoints = new ArrayList<>();
            final List<Point3D> outputPointsWithError = new ArrayList<>();
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            HomogeneousPoint3D inputPoint;
            for (int i = 0; i < nPoints; i++) {
                final double homX;
                final double homY;
                final double homW = randomizer.nextDouble(MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                final double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                if (Math.abs(b) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }
                inputPoint = new HomogeneousPoint3D(homX, homY, homZ, homW);

                assertTrue(plane.isLocus(inputPoint));

                final Point3D outputPoint = transformation1.transformAndReturnNew(
                        inputPoint);
                final Point3D outputPointWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double errorZ = errorRandomizer.nextDouble();
                    outputPointWithError = new InhomogeneousPoint3D(
                            outputPoint.getInhomX() + errorX,
                            outputPoint.getInhomY() + errorY,
                            outputPoint.getInhomZ() + errorZ);
                } else {
                    // inlier point (without error)
                    outputPointWithError = outputPoint;
                }

                inputPoints.add(inputPoint);
                outputPoints.add(outputPoint);
                outputPointsWithError.add(outputPointWithError);
            }

            final RANSACMetricTransformation3DRobustEstimator estimator =
                    new RANSACMetricTransformation3DRobustEstimator(this,
                            inputPoints, outputPointsWithError, true);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final MetricTransformation3D transformation2 = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by transforming input points
            // using estimated transformation (transformation2) and checking
            // that output points are equal to the original output points without
            // error
            Point3D p1;
            Point3D p2;
            boolean isValid = true;
            for (int i = 0; i < nPoints; i++) {
                p1 = outputPoints.get(i);
                p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
                if (p1.distanceTo(p2) > ABSOLUTE_ERROR) {
                    isValid = false;
                    break;
                }
                assertEquals(p1.distanceTo(p2), 0.0,
                        ABSOLUTE_ERROR);
            }

            if (!isValid) {
                continue;
            }

            // check parameters of estimated transformation
            final Quaternion q2 = transformation2.getRotation().toQuaternion();
            q2.normalize();
            final double[] translation2 = transformation2.getTranslation();
            final double scale2 = transformation2.getScale();

            assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
            assertEquals(scale, scale2, ABSOLUTE_ERROR);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateWithRefinement() throws LockedException,
            NotReadyException, RobustEstimatorException {
        // create an euclidean transformation
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double roll = Utils.convertToRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Utils.convertToRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Utils.convertToRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final Quaternion q = new Quaternion(roll, pitch, yaw);
        q.normalize();

        final double scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);

        final double[] translation = new double[3];
        randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

        final MetricTransformation3D transformation1 =
                new MetricTransformation3D(q, translation, scale);

        // generate random points
        final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        final List<Point3D> inputPoints = new ArrayList<>();
        final List<Point3D> outputPoints = new ArrayList<>();
        final List<Point3D> outputPointsWithError = new ArrayList<>();
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_ERROR);
        for (int i = 0; i < nPoints; i++) {
            final Point3D inputPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE));
            final Point3D outputPoint = transformation1.transformAndReturnNew(
                    inputPoint);
            final Point3D outputPointWithError;
            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                // point is outlier
                final double errorX = errorRandomizer.nextDouble();
                final double errorY = errorRandomizer.nextDouble();
                final double errorZ = errorRandomizer.nextDouble();
                outputPointWithError = new InhomogeneousPoint3D(
                        outputPoint.getInhomX() + errorX,
                        outputPoint.getInhomY() + errorY,
                        outputPoint.getInhomZ() + errorZ);
            } else {
                // inlier point (without error)
                outputPointWithError = outputPoint;
            }

            inputPoints.add(inputPoint);
            outputPoints.add(outputPoint);
            outputPointsWithError.add(outputPointWithError);
        }

        final RANSACMetricTransformation3DRobustEstimator estimator =
                new RANSACMetricTransformation3DRobustEstimator(this,
                        inputPoints, outputPointsWithError);

        estimator.setThreshold(THRESHOLD);
        estimator.setResultRefined(true);
        estimator.setCovarianceKept(true);

        assertEquals(estimateStart, 0);
        assertEquals(estimateEnd, 0);
        assertEquals(estimateNextIteration, 0);
        assertEquals(estimateProgressChange, 0);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());

        final MetricTransformation3D transformation2 = estimator.estimate();

        assertNotNull(estimator.getInliersData());
        assertNotNull(estimator.getInliersData().getInliers());
        assertNotNull(estimator.getInliersData().getResiduals());
        assertTrue(estimator.getInliersData().getNumInliers() > 0);
        if (estimator.getCovariance() != null) {
            assertEquals(estimator.getCovariance().getRows(),
                    1 + Quaternion.N_PARAMS +
                            MetricTransformation3D.NUM_TRANSLATION_COORDS);
            assertEquals(estimator.getCovariance().getColumns(),
                    1 + Quaternion.N_PARAMS +
                            MetricTransformation3D.NUM_TRANSLATION_COORDS);
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
        Point3D p1;
        Point3D p2;
        for (int i = 0; i < nPoints; i++) {
            p1 = outputPoints.get(i);
            p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
            assertEquals(p1.distanceTo(p2), 0.0,
                    ABSOLUTE_ERROR);
        }

        // check parameters of estimated transformation
        final Quaternion q2 = transformation2.getRotation().toQuaternion();
        q2.normalize();
        final double[] translation2 = transformation2.getTranslation();
        final double scale2 = transformation2.getScale();

        assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
        assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
        assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
        assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
        assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
        assertEquals(scale, scale2, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateCoplanarWithRefinement() throws LockedException,
            NotReadyException, RobustEstimatorException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            // create an euclidean transformation
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double roll = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final Quaternion q = new Quaternion(roll, pitch, yaw);
            q.normalize();

            final double scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);

            final double[] translation = new double[3];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

            final MetricTransformation3D transformation1 =
                    new MetricTransformation3D(q, translation, scale);

            // generate random plane
            final double a = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final double b = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final double c = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final double d = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final Plane plane = new Plane(a, b, c, d);

            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final List<Point3D> inputPoints = new ArrayList<>();
            final List<Point3D> outputPoints = new ArrayList<>();
            final List<Point3D> outputPointsWithError = new ArrayList<>();
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            HomogeneousPoint3D inputPoint;
            for (int i = 0; i < nPoints; i++) {
                final double homX;
                final double homY;
                final double homW = randomizer.nextDouble(MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                final double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                if (Math.abs(b) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }
                inputPoint = new HomogeneousPoint3D(homX, homY, homZ, homW);

                assertTrue(plane.isLocus(inputPoint));

                final Point3D outputPoint = transformation1.transformAndReturnNew(
                        inputPoint);
                final Point3D outputPointWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double errorZ = errorRandomizer.nextDouble();
                    outputPointWithError = new InhomogeneousPoint3D(
                            outputPoint.getInhomX() + errorX,
                            outputPoint.getInhomY() + errorY,
                            outputPoint.getInhomZ() + errorZ);
                } else {
                    // inlier point (without error)
                    outputPointWithError = outputPoint;
                }

                inputPoints.add(inputPoint);
                outputPoints.add(outputPoint);
                outputPointsWithError.add(outputPointWithError);
            }

            final RANSACMetricTransformation3DRobustEstimator estimator =
                    new RANSACMetricTransformation3DRobustEstimator(this,
                            inputPoints, outputPointsWithError, true);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final MetricTransformation3D transformation2 = estimator.estimate();

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                assertEquals(estimator.getCovariance().getRows(),
                        1 + Quaternion.N_PARAMS +
                                MetricTransformation3D.NUM_TRANSLATION_COORDS);
                assertEquals(estimator.getCovariance().getColumns(),
                        1 + Quaternion.N_PARAMS +
                                MetricTransformation3D.NUM_TRANSLATION_COORDS);
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
            Point3D p1;
            Point3D p2;
            boolean isValid = true;
            for (int i = 0; i < nPoints; i++) {
                p1 = outputPoints.get(i);
                p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
                if (p1.distanceTo(p2) > ABSOLUTE_ERROR) {
                    isValid = false;
                    break;
                }
                assertEquals(p1.distanceTo(p2), 0.0,
                        ABSOLUTE_ERROR);
            }

            if (!isValid) {
                continue;
            }

            // check parameters of estimated transformation
            final Quaternion q2 = transformation2.getRotation().toQuaternion();
            q2.normalize();
            final double[] translation2 = transformation2.getTranslation();
            final double scale2 = transformation2.getScale();

            assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
            assertEquals(scale, scale2, ABSOLUTE_ERROR);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onEstimateStart(final MetricTransformation3DRobustEstimator estimator) {
        estimateStart++;
        checkLocked((RANSACMetricTransformation3DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final MetricTransformation3DRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((RANSACMetricTransformation3DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(
            final MetricTransformation3DRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        checkLocked((RANSACMetricTransformation3DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(
            final MetricTransformation3DRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        checkLocked((RANSACMetricTransformation3DRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void checkLocked(
            final RANSACMetricTransformation3DRobustEstimator estimator) {
        final List<Point3D> points = new ArrayList<>();
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
        try {
            estimator.setWeakMinimumSizeAllowed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        assertTrue(estimator.isLocked());
    }
}
