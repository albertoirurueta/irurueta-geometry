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

import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class MetricTransformation3DRobustEstimatorTest {

    @Test
    public void testConstants() {
        assertEquals(4,
                MetricTransformation3DRobustEstimator.MINIMUM_SIZE);
        assertEquals(3,
                MetricTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE);
        assertEquals(0.05f,
                MetricTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(0.0f,
                MetricTransformation3DRobustEstimator.MIN_PROGRESS_DELTA,
                0.0f);
        assertEquals(1.0f,
                MetricTransformation3DRobustEstimator.MAX_PROGRESS_DELTA,
                0.0f);
        assertEquals(0.99,
                MetricTransformation3DRobustEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(5000,
                MetricTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(0.0,
                MetricTransformation3DRobustEstimator.MIN_CONFIDENCE, 0.0);
        assertEquals(1.0,
                MetricTransformation3DRobustEstimator.MAX_CONFIDENCE, 0.0);
        assertEquals(1,
                MetricTransformation3DRobustEstimator.MIN_ITERATIONS);
        assertTrue(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(MetricTransformation3DRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertEquals(RobustEstimatorMethod.PROMEDS,
                MetricTransformation3DRobustEstimator.DEFAULT_ROBUST_METHOD);
    }

    @Test
    public void testCreate() {
        // create with method
        MetricTransformation3DRobustEstimator estimator = MetricTransformation3DRobustEstimator.create(
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation3DRobustEstimator);

        estimator = MetricTransformation3DRobustEstimator.create(
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation3DRobustEstimator);

        estimator = MetricTransformation3DRobustEstimator.create(
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation3DRobustEstimator);

        estimator = MetricTransformation3DRobustEstimator.create(
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation3DRobustEstimator);

        estimator = MetricTransformation3DRobustEstimator.create(
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);

        // create with points and method
        List<Point3D> inputPoints = new ArrayList<>();
        List<Point3D> outputPoints = new ArrayList<>();
        for (int i = 0; i < MetricTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final List<Point3D> emptyPoints = new ArrayList<>();

        estimator = null;
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    emptyPoints, outputPoints, RobustEstimatorMethod.LMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    inputPoints, emptyPoints, RobustEstimatorMethod.LMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test create with listener and method
        final MetricTransformation3DRobustEstimatorListener listener =
                new MetricTransformation3DRobustEstimatorListener() {

                    @Override
                    public void onEstimateStart(
                            final MetricTransformation3DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateEnd(
                            final MetricTransformation3DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final MetricTransformation3DRobustEstimator estimator,
                            final int iteration) {
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final MetricTransformation3DRobustEstimator estimator,
                            final float progress) {
                    }
                };
        estimator = MetricTransformation3DRobustEstimator.create(listener,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());

        estimator = MetricTransformation3DRobustEstimator.create(listener,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());

        estimator = MetricTransformation3DRobustEstimator.create(listener,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());

        estimator = MetricTransformation3DRobustEstimator.create(listener,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());

        estimator = MetricTransformation3DRobustEstimator.create(listener,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());

        // test with listener and points
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with quality scores and method
        double[] qualityScores = new double[
                MetricTransformation3DRobustEstimator.MINIMUM_SIZE];
        estimator = MetricTransformation3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getQualityScores());

        estimator = MetricTransformation3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation3DRobustEstimator);
        assertNull(estimator.getQualityScores());

        estimator = MetricTransformation3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getQualityScores());

        estimator = MetricTransformation3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation3DRobustEstimator);
        assertSame(qualityScores, estimator.getQualityScores());

        estimator = MetricTransformation3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertSame(qualityScores, estimator.getQualityScores());

        // test with points, quality scores and method
        final double[] wrongQualityScores = new double[1];

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    emptyPoints, outputPoints, qualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    inputPoints, emptyPoints, qualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    inputPoints, outputPoints, wrongQualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, quality scores and method
        estimator = MetricTransformation3DRobustEstimator.create(listener,
                qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertNull(estimator.getQualityScores());

        estimator = MetricTransformation3DRobustEstimator.create(listener,
                qualityScores, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertNull(estimator.getQualityScores());

        estimator = MetricTransformation3DRobustEstimator.create(listener,
                qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertNull(estimator.getQualityScores());

        estimator = MetricTransformation3DRobustEstimator.create(listener,
                qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());

        estimator = MetricTransformation3DRobustEstimator.create(listener,
                qualityScores, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());

        // test with listener, points, quality scores and method
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    listener, emptyPoints, outputPoints, qualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    listener, inputPoints, emptyPoints, qualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    listener, inputPoints, outputPoints, wrongQualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with weak minimum and method
        estimator = MetricTransformation3DRobustEstimator.create(
                true, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation3DRobustEstimator);
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(
                true, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation3DRobustEstimator);
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(
                true, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation3DRobustEstimator);
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(
                true, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation3DRobustEstimator);
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(
                true, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        // test with listener, weak minimum and method
        estimator = MetricTransformation3DRobustEstimator.create(listener,
                true, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(listener,
                true, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(listener,
                true, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(listener,
                true, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(listener,
                true, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        // test with quality scores, weak minimum and method
        estimator = MetricTransformation3DRobustEstimator.create(qualityScores,
                true, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(qualityScores,
                true, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation3DRobustEstimator);
        assertNull(estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(qualityScores,
                true, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(qualityScores,
                true, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation3DRobustEstimator);
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(qualityScores,
                true, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        // test with listener, quality scores, weak minimum and method
        estimator = MetricTransformation3DRobustEstimator.create(listener,
                qualityScores, true,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(listener,
                qualityScores, true,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(listener,
                qualityScores, true,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(listener,
                qualityScores, true,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(listener,
                qualityScores, true,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        // create with points and method and weak points
        inputPoints = new ArrayList<>();
        outputPoints = new ArrayList<>();
        for (int i = 0; i < MetricTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE; i++) {
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    emptyPoints, outputPoints, true,
                    RobustEstimatorMethod.LMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    inputPoints, emptyPoints, true,
                    RobustEstimatorMethod.LMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener and points
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, true,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, true,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, true,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, true,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, true,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points, quality scores and method
        qualityScores = new double[
                MetricTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE];

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    emptyPoints, outputPoints, qualityScores, true,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    inputPoints, emptyPoints, qualityScores, true,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    inputPoints, outputPoints, wrongQualityScores, true,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, points, quality scores and method
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    listener, emptyPoints, outputPoints, qualityScores, true,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    listener, inputPoints, emptyPoints, qualityScores, true,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    listener, inputPoints, outputPoints, wrongQualityScores,
                    true, RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testCreate2() {
        // create
        MetricTransformation3DRobustEstimator estimator = MetricTransformation3DRobustEstimator.create();
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);

        // create with points
        List<Point3D> inputPoints = new ArrayList<>();
        List<Point3D> outputPoints = new ArrayList<>();
        for (int i = 0; i < MetricTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final List<Point3D> emptyPoints = new ArrayList<>();

        estimator = null;
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    emptyPoints, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    inputPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test create with listener and method
        final MetricTransformation3DRobustEstimatorListener listener =
                new MetricTransformation3DRobustEstimatorListener() {

                    @Override
                    public void onEstimateStart(
                            final MetricTransformation3DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateEnd(
                            final MetricTransformation3DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final MetricTransformation3DRobustEstimator estimator,
                            final int iteration) {
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final MetricTransformation3DRobustEstimator estimator,
                            final float progress) {
                    }
                };

        estimator = MetricTransformation3DRobustEstimator.create(listener);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());


        // test with listener and points
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with quality scores
        double[] qualityScores = new double[
                MetricTransformation3DRobustEstimator.MINIMUM_SIZE];

        estimator = MetricTransformation3DRobustEstimator.create(qualityScores);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertSame(qualityScores, estimator.getQualityScores());

        // test with points and quality scores
        final double[] wrongQualityScores = new double[1];

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    emptyPoints, outputPoints, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    inputPoints, emptyPoints, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    inputPoints, outputPoints, wrongQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener and quality scores
        estimator = MetricTransformation3DRobustEstimator.create(listener,
                qualityScores);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());

        // test with listener, points and quality scores
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    listener, emptyPoints, outputPoints, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    listener, inputPoints, emptyPoints, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    listener, inputPoints, outputPoints, wrongQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with weak minimum
        estimator = MetricTransformation3DRobustEstimator.create(
                true);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        // create with points and weak minimum
        inputPoints = new ArrayList<>();
        outputPoints = new ArrayList<>();
        for (int i = 0; i < MetricTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE; i++) {
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, true);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    emptyPoints, outputPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    inputPoints, emptyPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener and weak minimum
        estimator = MetricTransformation3DRobustEstimator.create(listener,
                true);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        // test with listener, points and weak minimum
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, true);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());


        // test with quality scores and weak minimum and method
        estimator = MetricTransformation3DRobustEstimator.create(qualityScores,
                true);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        // test with points, quality scores and weak minimum
        qualityScores = new double[
                MetricTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE];

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    emptyPoints, outputPoints, qualityScores, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    inputPoints, emptyPoints, qualityScores, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    inputPoints, outputPoints, wrongQualityScores, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, quality scores and weak minimum
        estimator = MetricTransformation3DRobustEstimator.create(listener,
                qualityScores, true);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        // test with listener, points, quality scores and method
        estimator = MetricTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true);
        assertTrue(estimator instanceof
                PROMedSMetricTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    listener, emptyPoints, outputPoints, qualityScores, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    listener, inputPoints, emptyPoints, qualityScores, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = MetricTransformation3DRobustEstimator.create(
                    listener, inputPoints, outputPoints, wrongQualityScores,
                    true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }
}
