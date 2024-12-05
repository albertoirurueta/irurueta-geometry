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

import com.irurueta.geometry.Point2D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class PointCorrespondenceProjectiveTransformation2DRobustEstimatorTest {

    @Test
    void testConstants() {
        assertEquals(4, ProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE);
        assertEquals(0.05f, ProjectiveTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0f);
        assertEquals(0.0f, ProjectiveTransformation2DRobustEstimator.MIN_PROGRESS_DELTA, 0.0f);
        assertEquals(1.0f, ProjectiveTransformation2DRobustEstimator.MAX_PROGRESS_DELTA, 0.0f);
        assertEquals(0.99, ProjectiveTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(5000, ProjectiveTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(0.0, ProjectiveTransformation2DRobustEstimator.MIN_CONFIDENCE, 0.0);
        assertEquals(1.0, ProjectiveTransformation2DRobustEstimator.MAX_CONFIDENCE, 0.0);
        assertEquals(1, ProjectiveTransformation2DRobustEstimator.MIN_ITERATIONS);
        assertTrue(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(ProjectiveTransformation2DRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertEquals(RobustEstimatorMethod.PROMEDS,
                PointCorrespondenceProjectiveTransformation2DRobustEstimator.DEFAULT_ROBUST_METHOD);
    }

    @Test
    void testCreate() {
        // create with robust estimator method
        var estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // create with points and method
        final var inputPoints = new ArrayList<Point2D>();
        final var outputPoints = new ArrayList<Point2D>();
        for (var i = 0; i < PointCorrespondenceProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputPoints, outputPoints,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputPoints, outputPoints,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputPoints, outputPoints,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputPoints, outputPoints,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputPoints, outputPoints,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point2D>();
        assertThrows(IllegalArgumentException.class, () -> PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(emptyPoints, outputPoints, RobustEstimatorMethod.LMEDS));
        assertThrows(IllegalArgumentException.class, () -> PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputPoints, emptyPoints, RobustEstimatorMethod.LMEDS));

        // create with listener and method
        final var listener = new ProjectiveTransformation2DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(final ProjectiveTransformation2DRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateEnd(final ProjectiveTransformation2DRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateNextIteration(
                    final ProjectiveTransformation2DRobustEstimator estimator, final int iteration) {
                // no action needed
            }

            @Override
            public void onEstimateProgressChange(
                    final ProjectiveTransformation2DRobustEstimator estimator, final float progress) {
                // no action needed
            }
        };


        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and points
        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputPoints,
                outputPoints, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputPoints,
                outputPoints, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputPoints,
                outputPoints, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputPoints,
                outputPoints, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputPoints,
                outputPoints, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with quality scores and method
        final var qualityScores = new double[PointCorrespondenceProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE];
        final var wrongQualityScores = new double[1];

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points, quality scores and method
        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(emptyPoints, outputPoints, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputPoints, emptyPoints, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputPoints, outputPoints, wrongQualityScores, RobustEstimatorMethod.PROMEDS));

        // test with listener, quality scores and method
        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points, quality scores and method
        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputPoints,
                outputPoints, qualityScores, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputPoints,
                outputPoints, qualityScores, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputPoints,
                outputPoints, qualityScores, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputPoints,
                outputPoints, qualityScores, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputPoints,
                outputPoints, qualityScores, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, emptyPoints, outputPoints, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputPoints, emptyPoints, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints, wrongQualityScores, RobustEstimatorMethod.PROMEDS));

        // test no arguments
        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create();
        assertInstanceOf(PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points
        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputPoints, outputPoints);
        assertInstanceOf(PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(emptyPoints, outputPoints));
        assertThrows(IllegalArgumentException.class, () -> PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputPoints, emptyPoints));

        // test with listener
        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener);
        assertInstanceOf(PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and points
        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputPoints,
                outputPoints);
        assertInstanceOf(PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, emptyPoints, outputPoints));
        assertThrows(IllegalArgumentException.class, () -> PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputPoints, emptyPoints));

        // test with quality scores
        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(qualityScores);
        assertInstanceOf(PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points and quality scores
        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputPoints, outputPoints,
                qualityScores);
        assertInstanceOf(PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and quality scores
        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, qualityScores);
        assertInstanceOf(PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points and quality scores
        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputPoints,
                outputPoints, qualityScores);
        assertInstanceOf(PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
    }
}
