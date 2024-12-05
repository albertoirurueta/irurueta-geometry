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

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class RANSACMetricTransformation3DRobustEstimatorTest implements
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
    void testConstants() {
        assertEquals(4, MetricTransformation3DRobustEstimator.MINIMUM_SIZE);
        assertEquals(3, MetricTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE);
        assertEquals(0.05f, MetricTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0f);
        assertEquals(0.0f, MetricTransformation3DRobustEstimator.MIN_PROGRESS_DELTA, 0.0f);
        assertEquals(1.0f, MetricTransformation3DRobustEstimator.MAX_PROGRESS_DELTA, 0.0f);
        assertEquals(0.99, MetricTransformation3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(5000, MetricTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(0.0, MetricTransformation3DRobustEstimator.MIN_CONFIDENCE, 0.0);
        assertEquals(1.0, MetricTransformation3DRobustEstimator.MAX_CONFIDENCE, 0.0);
        assertEquals(1, MetricTransformation3DRobustEstimator.MIN_ITERATIONS);
        assertTrue(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(MetricTransformation3DRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertEquals(RobustEstimatorMethod.PROMEDS, MetricTransformation3DRobustEstimator.DEFAULT_ROBUST_METHOD);
        assertEquals(1.0, RANSACMetricTransformation3DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(0.0, RANSACMetricTransformation3DRobustEstimator.MIN_THRESHOLD, 0.0);
        assertFalse(RANSACMetricTransformation3DRobustEstimator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertFalse(RANSACMetricTransformation3DRobustEstimator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
    }

    @Test
    void testConstructor() {
        // test constructor without arguments
        var estimator = new RANSACMetricTransformation3DRobustEstimator();

        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation3DRobustEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // test constructor with points
        final var inputPoints = new ArrayList<Point3D>();
        final var outputPoints = new ArrayList<Point3D>();
        for (var i = 0; i < MetricTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }

        estimator = new RANSACMetricTransformation3DRobustEstimator(inputPoints, outputPoints);

        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation3DRobustEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        final var pointsEmpty = new ArrayList<Point3D>();
        // not enough points
        assertThrows(IllegalArgumentException.class, () -> new RANSACMetricTransformation3DRobustEstimator(
                pointsEmpty, pointsEmpty));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> new RANSACMetricTransformation3DRobustEstimator(
                inputPoints, pointsEmpty));

        // test constructor with listener
        estimator = new RANSACMetricTransformation3DRobustEstimator(this);

        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation3DRobustEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // test constructor with listener and points
        estimator = new RANSACMetricTransformation3DRobustEstimator(this, inputPoints, outputPoints);

        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation3DRobustEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        // not enough points
        assertThrows(IllegalArgumentException.class, () -> new RANSACMetricTransformation3DRobustEstimator(
                this, pointsEmpty, pointsEmpty));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> new RANSACMetricTransformation3DRobustEstimator(
                this, inputPoints, pointsEmpty));

        // test constructor without arguments
        estimator = new RANSACMetricTransformation3DRobustEstimator(true);

        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(EuclideanTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // test constructor with points
        final var inputPoints2 = new ArrayList<Point3D>();
        final var outputPoints2 = new ArrayList<Point3D>();
        for (var i = 0; i < MetricTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE; i++) {
            inputPoints2.add(Point3D.create());
            outputPoints2.add(Point3D.create());
        }

        estimator = new RANSACMetricTransformation3DRobustEstimator(inputPoints2, outputPoints2,
                true);

        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertSame(inputPoints2, estimator.getInputPoints());
        assertSame(outputPoints2, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        // not enough points
        assertThrows(IllegalArgumentException.class, () -> new RANSACMetricTransformation3DRobustEstimator(pointsEmpty,
                pointsEmpty, true));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> new RANSACMetricTransformation3DRobustEstimator(inputPoints2,
                pointsEmpty, true));

        // test constructor with listener
        estimator = new RANSACMetricTransformation3DRobustEstimator(this, true);

        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // test constructor with listener and points
        estimator = new RANSACMetricTransformation3DRobustEstimator(this, inputPoints2, outputPoints2,
                true);

        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);
        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertSame(inputPoints2, estimator.getInputPoints());
        assertSame(outputPoints2, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // Force IllegalArgumentException
        // not enough points
        assertThrows(IllegalArgumentException.class, () -> new RANSACMetricTransformation3DRobustEstimator(
                this, pointsEmpty, pointsEmpty, true));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> new RANSACMetricTransformation3DRobustEstimator(
                this, inputPoints, pointsEmpty, true));
    }

    @Test
    void testGetSetStopThreshold() throws LockedException {
        final var estimator = new RANSACMetricTransformation3DRobustEstimator();

        // check default value
        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);

        // set new value
        estimator.setThreshold(0.5);

        // check correctness
        assertEquals(0.5, estimator.getThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setThreshold(0.0));
    }

    @Test
    void testGetSetConfidence() throws LockedException {
        final var estimator = new RANSACMetricTransformation3DRobustEstimator();

        // check default value
        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);

        // set new value
        estimator.setConfidence(0.5);

        // check correctness
        assertEquals(0.5, estimator.getConfidence(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setConfidence(-1.0));
        assertThrows(IllegalArgumentException.class, () -> estimator.setConfidence(2.0));
    }

    @Test
    void testGetSetMaxIterations() throws LockedException {
        final var estimator = new RANSACMetricTransformation3DRobustEstimator();

        // check default value
        assertEquals(RANSACMetricTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(10);

        // check correctness
        assertEquals(10, estimator.getMaxIterations());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setMaxIterations(0));
    }

    @Test
    void testGetSetPointsAndIsReady() throws LockedException {
        final var estimator = new RANSACMetricTransformation3DRobustEstimator();

        // check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());

        // set new value
        final var inputPoints = new ArrayList<Point3D>();
        final var outputPoints = new ArrayList<Point3D>();
        for (var i = 0; i < RANSACMetricTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }

        estimator.setPoints(inputPoints, outputPoints);

        // check correctness
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());

        // Force IllegalArgumentException
        final var pointsEmpty = new ArrayList<Point3D>();
        // not enough points
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(pointsEmpty, pointsEmpty));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(pointsEmpty, pointsEmpty));
    }

    @Test
    void testGetSetListenerAndIsListenerAvailable() throws LockedException {
        final var estimator = new RANSACMetricTransformation3DRobustEstimator();

        // check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
    }

    @Test
    void testIsSetWeakMinimumPointsAllowed() throws LockedException {
        final var estimator = new RANSACMetricTransformation3DRobustEstimator();

        // check default value
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation3DRobustEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());

        // set new value
        estimator.setWeakMinimumSizeAllowed(true);

        // check correctness
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());
    }

    @Test
    void testGetSetProgressDelta() throws LockedException {
        final var estimator = new RANSACMetricTransformation3DRobustEstimator();

        // check default value
        assertEquals(MetricTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check correctness
        assertEquals(0.5f, estimator.getProgressDelta(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setProgressDelta(-1.0f));
        assertThrows(IllegalArgumentException.class, () -> estimator.setProgressDelta(2.0f));
    }

    @Test
    void testIsSetResultRefined() throws LockedException {
        final var estimator = new RANSACMetricTransformation3DRobustEstimator();

        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check correctness
        assertFalse(estimator.isResultRefined());
    }

    @Test
    void testIsSetCovarianceKept() throws LockedException {
        final var estimator = new RANSACMetricTransformation3DRobustEstimator();

        assertFalse(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(true);

        // check correctness
        assertTrue(estimator.isCovarianceKept());
    }

    @Test
    void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        final var estimator = new RANSACMetricTransformation3DRobustEstimator();

        assertFalse(estimator.isComputeAndKeepInliersEnabled());

        // set new value
        estimator.setComputeAndKeepInliersEnabled(true);

        // check correctness
        assertTrue(estimator.isComputeAndKeepInliersEnabled());
    }

    @Test
    void testIsSetComputeAndKeepResidualsEnabled() throws LockedException {
        final var estimator = new RANSACMetricTransformation3DRobustEstimator();

        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // set new value
        estimator.setComputeAndKeepResidualsEnabled(true);

        // check correctness
        assertTrue(estimator.isComputeAndKeepResidualsEnabled());
    }

    @Test
    void testEstimateWithoutRefinement() throws LockedException, NotReadyException, RobustEstimatorException {
        // create an euclidean transformation
        final var randomizer = new UniformRandomizer();

        final var roll = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var q = new Quaternion(roll, pitch, yaw);
        q.normalize();

        final var scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);

        final var translation = new double[3];
        randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

        final var transformation1 = new MetricTransformation3D(q, translation, scale);

        // generate random points
        final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        final var inputPoints = new ArrayList<Point3D>();
        final var outputPoints = new ArrayList<Point3D>();
        final var outputPointsWithError = new ArrayList<Point3D>();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
        for (var i = 0; i < nPoints; i++) {
            final var inputPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var outputPoint = transformation1.transformAndReturnNew(inputPoint);
            final Point3D outputPointWithError;
            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                // point is outlier
                final var errorX = errorRandomizer.nextDouble();
                final var errorY = errorRandomizer.nextDouble();
                final var errorZ = errorRandomizer.nextDouble();
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

        final var estimator = new RANSACMetricTransformation3DRobustEstimator(this, inputPoints,
                outputPointsWithError);

        estimator.setThreshold(THRESHOLD);
        estimator.setResultRefined(false);
        estimator.setCovarianceKept(false);

        assertEquals(0, estimateStart);
        assertEquals(0, estimateEnd);
        assertEquals(0, estimateNextIteration);
        assertEquals(0, estimateProgressChange);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());

        final var transformation2 = estimator.estimate();

        assertEquals(1, estimateStart);
        assertEquals(1, estimateEnd);
        assertTrue(estimateNextIteration > 0);
        assertTrue(estimateProgressChange >= 0);
        reset();

        // check correctness of estimation by transforming input points
        // using estimated transformation (transformation2) and checking
        // that output points are equal to the original output points without
        // error
        for (var i = 0; i < nPoints; i++) {
            final var p1 = outputPoints.get(i);
            final var p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
            assertEquals(0.0, p1.distanceTo(p2), ABSOLUTE_ERROR);
        }

        // check parameters of estimated transformation
        final var q2 = transformation2.getRotation().toQuaternion();
        q2.normalize();
        final var translation2 = transformation2.getTranslation();
        final var scale2 = transformation2.getScale();

        assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
        assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
        assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
        assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
        assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
        assertEquals(scale, scale2, ABSOLUTE_ERROR);
    }

    @Test
    void testEstimateCoplanarWithoutRefinement() throws LockedException, NotReadyException, RobustEstimatorException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            // create an euclidean transformation
            final var randomizer = new UniformRandomizer();

            final var roll = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var q = new Quaternion(roll, pitch, yaw);
            q.normalize();

            final var scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);

            final var translation = new double[3];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

            final var transformation1 = new MetricTransformation3D(q, translation, scale);

            // generate random plane
            final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var plane = new Plane(a, b, c, d);

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var inputPoints = new ArrayList<Point3D>();
            final var outputPoints = new ArrayList<Point3D>();
            final var outputPointsWithError = new ArrayList<Point3D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            for (var i = 0; i < nPoints; i++) {
                final double homX;
                final double homY;
                final var homW = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                if (Math.abs(b) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }
                final var inputPoint = new HomogeneousPoint3D(homX, homY, homZ, homW);

                assertTrue(plane.isLocus(inputPoint));

                final var outputPoint = transformation1.transformAndReturnNew(inputPoint);
                final Point3D outputPointWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var errorZ = errorRandomizer.nextDouble();
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

            final var estimator = new RANSACMetricTransformation3DRobustEstimator(this, inputPoints,
                    outputPointsWithError, true);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var transformation2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by transforming input points
            // using estimated transformation (transformation2) and checking
            // that output points are equal to the original output points without
            // error
            var isValid = true;
            for (var i = 0; i < nPoints; i++) {
                final var p1 = outputPoints.get(i);
                final var p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
                if (p1.distanceTo(p2) > ABSOLUTE_ERROR) {
                    isValid = false;
                    break;
                }
                assertEquals(0.0, p1.distanceTo(p2), ABSOLUTE_ERROR);
            }

            if (!isValid) {
                continue;
            }

            // check parameters of estimated transformation
            final var q2 = transformation2.getRotation().toQuaternion();
            q2.normalize();
            final var translation2 = transformation2.getTranslation();
            final var scale2 = transformation2.getScale();

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
    void testEstimateWithRefinement() throws LockedException, NotReadyException, RobustEstimatorException {
        // create an euclidean transformation
        final var randomizer = new UniformRandomizer();

        final var roll = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var q = new Quaternion(roll, pitch, yaw);
        q.normalize();

        final var scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);

        final var translation = new double[3];
        randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

        final var transformation1 = new MetricTransformation3D(q, translation, scale);

        // generate random points
        final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        final var inputPoints = new ArrayList<Point3D>();
        final var outputPoints = new ArrayList<Point3D>();
        final var outputPointsWithError = new ArrayList<Point3D>();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
        for (var i = 0; i < nPoints; i++) {
            final var inputPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var outputPoint = transformation1.transformAndReturnNew(inputPoint);
            final Point3D outputPointWithError;
            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                // point is outlier
                final var errorX = errorRandomizer.nextDouble();
                final var errorY = errorRandomizer.nextDouble();
                final var errorZ = errorRandomizer.nextDouble();
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

        final var estimator = new RANSACMetricTransformation3DRobustEstimator(this, inputPoints,
                outputPointsWithError);

        estimator.setThreshold(THRESHOLD);
        estimator.setResultRefined(true);
        estimator.setCovarianceKept(true);

        assertEquals(0, estimateStart);
        assertEquals(0, estimateEnd);
        assertEquals(0, estimateNextIteration);
        assertEquals(0, estimateProgressChange);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());

        final var transformation2 = estimator.estimate();

        assertNotNull(estimator.getInliersData());
        assertNotNull(estimator.getInliersData().getInliers());
        assertNotNull(estimator.getInliersData().getResiduals());
        assertTrue(estimator.getInliersData().getNumInliers() > 0);
        if (estimator.getCovariance() != null) {
            assertEquals(1 + Quaternion.N_PARAMS + MetricTransformation3D.NUM_TRANSLATION_COORDS,
                    estimator.getCovariance().getRows());
            assertEquals(1 + Quaternion.N_PARAMS + MetricTransformation3D.NUM_TRANSLATION_COORDS,
                    estimator.getCovariance().getColumns());
        }

        assertEquals(1, estimateStart);
        assertEquals(1, estimateEnd);
        assertTrue(estimateNextIteration > 0);
        assertTrue(estimateProgressChange >= 0);
        reset();

        // check correctness of estimation by transforming input points
        // using estimated transformation (transformation2) and checking
        // that output points are equal to the original output points without
        // error
        for (var i = 0; i < nPoints; i++) {
            final var p1 = outputPoints.get(i);
            final var p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
            assertEquals(0.0, p1.distanceTo(p2), ABSOLUTE_ERROR);
        }

        // check parameters of estimated transformation
        final var q2 = transformation2.getRotation().toQuaternion();
        q2.normalize();
        final var translation2 = transformation2.getTranslation();
        final var scale2 = transformation2.getScale();

        assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
        assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
        assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
        assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
        assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
        assertEquals(scale, scale2, ABSOLUTE_ERROR);
    }

    @Test
    void testEstimateCoplanarWithRefinement() throws LockedException, NotReadyException, RobustEstimatorException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            // create an euclidean transformation
            final var randomizer = new UniformRandomizer();

            final var roll = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var q = new Quaternion(roll, pitch, yaw);
            q.normalize();

            final var scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);

            final var translation = new double[3];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

            final var transformation1 = new MetricTransformation3D(q, translation, scale);

            // generate random plane
            final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var plane = new Plane(a, b, c, d);

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var inputPoints = new ArrayList<Point3D>();
            final var outputPoints = new ArrayList<Point3D>();
            final var outputPointsWithError = new ArrayList<Point3D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            for (var i = 0; i < nPoints; i++) {
                final double homX;
                final double homY;
                final var homW = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                if (Math.abs(b) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }
                final var inputPoint = new HomogeneousPoint3D(homX, homY, homZ, homW);

                assertTrue(plane.isLocus(inputPoint));

                final var outputPoint = transformation1.transformAndReturnNew(inputPoint);
                final Point3D outputPointWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var errorZ = errorRandomizer.nextDouble();
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

            final var estimator = new RANSACMetricTransformation3DRobustEstimator(this, inputPoints,
                    outputPointsWithError, true);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var transformation2 = estimator.estimate();

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                assertEquals(1 + Quaternion.N_PARAMS + MetricTransformation3D.NUM_TRANSLATION_COORDS,
                        estimator.getCovariance().getRows());
                assertEquals(1 + Quaternion.N_PARAMS + MetricTransformation3D.NUM_TRANSLATION_COORDS,
                        estimator.getCovariance().getColumns());
            }

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation by transforming input points
            // using estimated transformation (transformation2) and checking
            // that output points are equal to the original output points without
            // error
            var isValid = true;
            for (var i = 0; i < nPoints; i++) {
                final var p1 = outputPoints.get(i);
                final var p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
                if (p1.distanceTo(p2) > ABSOLUTE_ERROR) {
                    isValid = false;
                    break;
                }
                assertEquals(0.0, p1.distanceTo(p2), ABSOLUTE_ERROR);
            }

            if (!isValid) {
                continue;
            }

            // check parameters of estimated transformation
            final var q2 = transformation2.getRotation().toQuaternion();
            q2.normalize();
            final var translation2 = transformation2.getTranslation();
            final var scale2 = transformation2.getScale();

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
    public void onEstimateNextIteration(final MetricTransformation3DRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        checkLocked((RANSACMetricTransformation3DRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(final MetricTransformation3DRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        checkLocked((RANSACMetricTransformation3DRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = estimateProgressChange = 0;
    }

    private static void checkLocked(final RANSACMetricTransformation3DRobustEstimator estimator) {
        final var points = new ArrayList<Point3D>();
        assertThrows(LockedException.class, () -> estimator.setPoints(points, points));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setProgressDelta(0.01f));
        assertThrows(LockedException.class, () -> estimator.setThreshold(0.5));
        assertThrows(LockedException.class, () -> estimator.setConfidence(0.5));
        assertThrows(LockedException.class, () -> estimator.setMaxIterations(10));
        assertThrows(LockedException.class, estimator::estimate);
        assertThrows(LockedException.class, () -> estimator.setWeakMinimumSizeAllowed(true));
        assertTrue(estimator.isLocked());
    }
}
