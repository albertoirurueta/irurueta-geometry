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
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class MetricTransformation3DRobustEstimatorTest {

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
    }

    @Test
    void testCreate() {
        // create with method
        var estimator = MetricTransformation3DRobustEstimator.create(RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACMetricTransformation3DRobustEstimator.class, estimator);

        estimator = MetricTransformation3DRobustEstimator.create(RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSMetricTransformation3DRobustEstimator.class, estimator);

        estimator = MetricTransformation3DRobustEstimator.create(RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACMetricTransformation3DRobustEstimator.class, estimator);

        estimator = MetricTransformation3DRobustEstimator.create(RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACMetricTransformation3DRobustEstimator.class, estimator);

        estimator = MetricTransformation3DRobustEstimator.create(RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);

        // create with points and method
        final var inputPoints = new ArrayList<Point3D>();
        final var outputPoints = new ArrayList<Point3D>();
        for (var i = 0; i < MetricTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }

        estimator = MetricTransformation3DRobustEstimator.create(inputPoints, outputPoints,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(inputPoints, outputPoints,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(inputPoints, outputPoints, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(inputPoints, outputPoints,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(inputPoints, outputPoints,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point3D>();
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(emptyPoints,
                outputPoints, RobustEstimatorMethod.LMEDS));
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(inputPoints,
                emptyPoints, RobustEstimatorMethod.LMEDS));

        // test create with listener and method
        final var listener = new MetricTransformation3DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(final MetricTransformation3DRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateEnd(final MetricTransformation3DRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateNextIteration(
                    final MetricTransformation3DRobustEstimator estimator, final int iteration) {
                // no action needed
            }

            @Override
            public void onEstimateProgressChange(
                    final MetricTransformation3DRobustEstimator estimator, final float progress) {
                // no action needed
            }
        };
        estimator = MetricTransformation3DRobustEstimator.create(listener, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());

        estimator = MetricTransformation3DRobustEstimator.create(listener, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());

        estimator = MetricTransformation3DRobustEstimator.create(listener, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());

        estimator = MetricTransformation3DRobustEstimator.create(listener, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());

        estimator = MetricTransformation3DRobustEstimator.create(listener, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());

        // test with listener and points
        estimator = MetricTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with quality scores and method
        final var qualityScores = new double[MetricTransformation3DRobustEstimator.MINIMUM_SIZE];
        estimator = MetricTransformation3DRobustEstimator.create(qualityScores, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getQualityScores());

        estimator = MetricTransformation3DRobustEstimator.create(qualityScores, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getQualityScores());

        estimator = MetricTransformation3DRobustEstimator.create(qualityScores, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getQualityScores());

        estimator = MetricTransformation3DRobustEstimator.create(qualityScores, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(qualityScores, estimator.getQualityScores());

        estimator = MetricTransformation3DRobustEstimator.create(qualityScores, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(qualityScores, estimator.getQualityScores());

        // test with points, quality scores and method
        final var wrongQualityScores = new double[1];

        estimator = MetricTransformation3DRobustEstimator.create(inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(emptyPoints,
                outputPoints, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(inputPoints,
                emptyPoints, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(inputPoints,
                outputPoints, wrongQualityScores, RobustEstimatorMethod.PROMEDS));

        // test with listener, quality scores and method
        estimator = MetricTransformation3DRobustEstimator.create(listener, qualityScores, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertNull(estimator.getQualityScores());

        estimator = MetricTransformation3DRobustEstimator.create(listener, qualityScores, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertNull(estimator.getQualityScores());

        estimator = MetricTransformation3DRobustEstimator.create(listener, qualityScores, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertNull(estimator.getQualityScores());

        estimator = MetricTransformation3DRobustEstimator.create(listener, qualityScores, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());

        estimator = MetricTransformation3DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());

        // test with listener, points, quality scores and method
        estimator = MetricTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(listener,
                emptyPoints, outputPoints, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(listener,
                inputPoints, emptyPoints, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(listener,
                inputPoints, outputPoints, wrongQualityScores, RobustEstimatorMethod.PROMEDS));

        // test with weak minimum and method
        estimator = MetricTransformation3DRobustEstimator.create(true,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACMetricTransformation3DRobustEstimator.class, estimator);
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(true,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(true,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACMetricTransformation3DRobustEstimator.class, estimator);
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(true,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACMetricTransformation3DRobustEstimator.class, estimator);
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(true,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        // test with listener, weak minimum and method
        estimator = MetricTransformation3DRobustEstimator.create(listener, true,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(listener, true,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(listener, true,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(listener, true,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(listener, true,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        // test with quality scores, weak minimum and method
        estimator = MetricTransformation3DRobustEstimator.create(qualityScores, true,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(qualityScores, true,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(qualityScores, true,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(qualityScores, true,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(qualityScores, true,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        // test with listener, quality scores, weak minimum and method
        estimator = MetricTransformation3DRobustEstimator.create(listener, qualityScores, true,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(listener, qualityScores, true,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(listener, qualityScores, true,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(listener, qualityScores, true,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = MetricTransformation3DRobustEstimator.create(listener, qualityScores, true,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        // create with points and method and weak points
        final var inputPoints2 = new ArrayList<Point3D>();
        final var outputPoints2 = new ArrayList<Point3D>();
        for (var i = 0; i < MetricTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE; i++) {
            inputPoints2.add(Point3D.create());
            outputPoints2.add(Point3D.create());
        }

        estimator = MetricTransformation3DRobustEstimator.create(inputPoints2, outputPoints2,
                true, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(inputPoints2, outputPoints2,
                true, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(inputPoints2, outputPoints2,
                true, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(inputPoints2, outputPoints2,
                true, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(inputPoints2, outputPoints2,
                true, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(emptyPoints,
                outputPoints2, true, RobustEstimatorMethod.LMEDS));
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(inputPoints2,
                emptyPoints, true, RobustEstimatorMethod.LMEDS));

        // test with listener and points
        estimator = MetricTransformation3DRobustEstimator.create(listener, inputPoints2, outputPoints2,
                true, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(listener, inputPoints2, outputPoints2,
                true, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(listener, inputPoints2, outputPoints2,
                true, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(listener, inputPoints2, outputPoints2,
                true, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(listener, inputPoints2, outputPoints2,
                true, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points, quality scores and method
        final var qualityScores2 = new double[MetricTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE];

        estimator = MetricTransformation3DRobustEstimator.create(
                inputPoints2, outputPoints2, qualityScores2, true, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(inputPoints2, outputPoints2, qualityScores2,
                true, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(inputPoints, outputPoints, qualityScores,
                true, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(inputPoints, outputPoints, qualityScores,
                true, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(inputPoints, outputPoints, qualityScores,
                true, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(emptyPoints,
                outputPoints, qualityScores, true, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(inputPoints,
                emptyPoints, qualityScores, true, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(inputPoints,
                outputPoints, wrongQualityScores, true, RobustEstimatorMethod.PROMEDS));

        // test with listener, points, quality scores and method
        estimator = MetricTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints, qualityScores,
                true, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints, qualityScores,
                true, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints, qualityScores,
                true, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints, qualityScores,
                true, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = MetricTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints, qualityScores,
                true, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(listener,
                emptyPoints, outputPoints, qualityScores, true, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(listener,
                inputPoints, emptyPoints, qualityScores, true, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(listener,
                inputPoints, outputPoints, wrongQualityScores, true,
                RobustEstimatorMethod.PROMEDS));
    }

    @Test
    void testCreate2() {
        // create
        var estimator = MetricTransformation3DRobustEstimator.create();
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);

        // create with points
        final var inputPoints = new ArrayList<Point3D>();
        final var outputPoints = new ArrayList<Point3D>();
        for (var i = 0; i < MetricTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }

        estimator = MetricTransformation3DRobustEstimator.create(inputPoints, outputPoints);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point3D>();
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(emptyPoints,
                outputPoints));
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(inputPoints,
                emptyPoints));

        // test create with listener and method
        final var listener = new MetricTransformation3DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(final MetricTransformation3DRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateEnd(final MetricTransformation3DRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateNextIteration(
                    final MetricTransformation3DRobustEstimator estimator, final int iteration) {
                // no action needed
            }

            @Override
            public void onEstimateProgressChange(
                    final MetricTransformation3DRobustEstimator estimator, final float progress) {
                // no action needed
            }
        };

        estimator = MetricTransformation3DRobustEstimator.create(listener);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());

        // test with listener and points
        estimator = MetricTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with quality scores
        final var qualityScores = new double[MetricTransformation3DRobustEstimator.MINIMUM_SIZE];

        estimator = MetricTransformation3DRobustEstimator.create(qualityScores);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(qualityScores, estimator.getQualityScores());

        // test with points and quality scores
        final var wrongQualityScores = new double[1];

        estimator = MetricTransformation3DRobustEstimator.create(inputPoints, outputPoints, qualityScores);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(emptyPoints,
                outputPoints, qualityScores));
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(inputPoints,
                emptyPoints, qualityScores));
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(inputPoints,
                outputPoints, wrongQualityScores));

        // test with listener and quality scores
        estimator = MetricTransformation3DRobustEstimator.create(listener, qualityScores);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());

        // test with listener, points and quality scores
        estimator = MetricTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints, qualityScores);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(listener,
                emptyPoints, outputPoints, qualityScores));
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(listener,
                inputPoints, emptyPoints, qualityScores));
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(listener,
                inputPoints, outputPoints, wrongQualityScores));

        // test with weak minimum
        estimator = MetricTransformation3DRobustEstimator.create(true);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        // create with points and weak minimum
        final var inputPoints2 = new ArrayList<Point3D>();
        final var outputPoints2 = new ArrayList<Point3D>();
        for (var i = 0; i < MetricTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE; i++) {
            inputPoints2.add(Point3D.create());
            outputPoints2.add(Point3D.create());
        }

        estimator = MetricTransformation3DRobustEstimator.create(inputPoints2, outputPoints2,
                true);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(emptyPoints,
                outputPoints2, true));
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(
                inputPoints2, emptyPoints, true));

        // test with listener and weak minimum
        estimator = MetricTransformation3DRobustEstimator.create(listener, true);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        // test with listener, points and weak minimum
        estimator = MetricTransformation3DRobustEstimator.create(listener, inputPoints2, outputPoints2,
                true);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with quality scores and weak minimum and method
        estimator = MetricTransformation3DRobustEstimator.create(qualityScores, true);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        // test with points, quality scores and weak minimum
        final var qualityScores2 = new double[MetricTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE];

        estimator = MetricTransformation3DRobustEstimator.create(inputPoints2, outputPoints2, qualityScores2,
                true);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(emptyPoints,
                outputPoints, qualityScores, true));
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(inputPoints,
                emptyPoints, qualityScores, true));
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(inputPoints,
                outputPoints, wrongQualityScores, true));

        // test with listener, quality scores and weak minimum
        estimator = MetricTransformation3DRobustEstimator.create(listener, qualityScores2, true);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertSame(qualityScores2, estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        // test with listener, points, quality scores and method
        estimator = MetricTransformation3DRobustEstimator.create(listener, inputPoints2, outputPoints2, qualityScores2,
                true);
        assertInstanceOf(PROMedSMetricTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(MetricTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(listener,
                emptyPoints, outputPoints2, qualityScores2, true));
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(listener,
                inputPoints2, emptyPoints, qualityScores2, true));
        assertThrows(IllegalArgumentException.class, () -> MetricTransformation3DRobustEstimator.create(
                listener, inputPoints2, outputPoints2, wrongQualityScores, true));
    }
}
