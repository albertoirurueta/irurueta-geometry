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

import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class PointCorrespondenceAffineTransformation3DRobustEstimatorTest {

    @Test
    void testConstants() {
        assertEquals(RobustEstimatorMethod.PROMEDS,
                PointCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_ROBUST_METHOD);
    }

    @Test
    void testCreate() {
        // create with robust estimator method
        var estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // create with points and method
        final var inputPoints = new ArrayList<Point3D>();
        final var outputPoints = new ArrayList<Point3D>();
        for (var i = 0; i < PointCorrespondenceAffineTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(inputPoints, outputPoints,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(inputPoints, outputPoints,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(inputPoints, outputPoints,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(inputPoints, outputPoints,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(inputPoints, outputPoints,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point3D>();
        assertThrows(IllegalArgumentException.class,
                () -> PointCorrespondenceAffineTransformation3DRobustEstimator.create(emptyPoints, outputPoints,
                        RobustEstimatorMethod.LMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> PointCorrespondenceAffineTransformation3DRobustEstimator.create(inputPoints, emptyPoints,
                        RobustEstimatorMethod.LMEDS));

        // create with listener and method
        final var listener = new AffineTransformation3DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(final AffineTransformation3DRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateEnd(final AffineTransformation3DRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateNextIteration(final AffineTransformation3DRobustEstimator estimator,
                                                final int iteration) {
                // no action needed
            }

            @Override
            public void onEstimateProgressChange(final AffineTransformation3DRobustEstimator estimator,
                                                 final float progress) {
                // no action needed
            }
        };


        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and points
        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with quality scores and method
        final var qualityScores = new double[PointCorrespondenceAffineTransformation3DRobustEstimator.MINIMUM_SIZE];
        final var wrongQualityScores = new double[1];

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points, quality scores and method
        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> PointCorrespondenceAffineTransformation3DRobustEstimator.create(emptyPoints, outputPoints,
                        qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> PointCorrespondenceAffineTransformation3DRobustEstimator.create(inputPoints, emptyPoints,
                        qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> PointCorrespondenceAffineTransformation3DRobustEstimator.create(inputPoints, outputPoints,
                        wrongQualityScores, RobustEstimatorMethod.PROMEDS));

        // test with listener, quality scores and method
        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points, quality scores and method
        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener, emptyPoints,
                        outputPoints, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPoints,
                        emptyPoints, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPoints,
                        outputPoints, wrongQualityScores, RobustEstimatorMethod.PROMEDS));

        // test no arguments
        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create();
        assertInstanceOf(PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points
        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(inputPoints, outputPoints);
        assertInstanceOf(PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> PointCorrespondenceAffineTransformation3DRobustEstimator.create(emptyPoints, outputPoints));
        assertThrows(IllegalArgumentException.class,
                () -> PointCorrespondenceAffineTransformation3DRobustEstimator.create(inputPoints, emptyPoints));

        // test with listener
        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener);
        assertInstanceOf(PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and points
        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPoints,
                outputPoints);
        assertInstanceOf(PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener, emptyPoints,
                        outputPoints));
        assertThrows(IllegalArgumentException.class,
                () -> PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPoints,
                        emptyPoints));

        // test with quality scores
        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(qualityScores);
        assertInstanceOf(PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points and quality scores
        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(inputPoints, outputPoints,
                qualityScores);
        assertInstanceOf(PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and quality scores
        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener, qualityScores);
        assertInstanceOf(PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points and quality scores
        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPoints, outputPoints,
                qualityScores);
        assertInstanceOf(PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
    }
}
