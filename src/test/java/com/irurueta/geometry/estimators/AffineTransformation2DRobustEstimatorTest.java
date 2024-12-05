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

import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class AffineTransformation2DRobustEstimatorTest {

    @Test
    void testConstants() {
        assertEquals(3, AffineTransformation2DRobustEstimator.MINIMUM_SIZE);
        assertEquals(0.05f, AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0f);
        assertEquals(0.0f, AffineTransformation2DRobustEstimator.MIN_PROGRESS_DELTA, 0.0f);
        assertEquals(1.0f, AffineTransformation2DRobustEstimator.MAX_PROGRESS_DELTA, 0.0f);
        assertEquals(0.99, AffineTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(5000, AffineTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(0.0, AffineTransformation2DRobustEstimator.MIN_CONFIDENCE, 0.0);
        assertEquals(1.0, AffineTransformation2DRobustEstimator.MAX_CONFIDENCE, 0.0);
        assertEquals(1, AffineTransformation2DRobustEstimator.MIN_ITERATIONS);
        assertTrue(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(AffineTransformation2DRobustEstimator.DEFAULT_KEEP_COVARIANCE);
    }

    @Test
    void testCreateFromPoints() {
        // create with points and method
        final var inputPoints = new ArrayList<Point2D>();
        final var outputPoints = new ArrayList<Point2D>();
        for (var i = 0; i < AffineTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }

        var estimator = AffineTransformation2DRobustEstimator.createFromPoints(inputPoints, outputPoints,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPointCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(inputPoints, outputPoints,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPointCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(inputPoints, outputPoints,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPointCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(inputPoints, outputPoints,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(inputPoints, outputPoints,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point2D>();
        assertThrows(IllegalArgumentException.class, () -> AffineTransformation2DRobustEstimator.createFromPoints(
                emptyPoints, outputPoints, RobustEstimatorMethod.LMEDS));
        assertThrows(IllegalArgumentException.class, () -> AffineTransformation2DRobustEstimator.createFromPoints(
                inputPoints, emptyPoints, RobustEstimatorMethod.LMEDS));

        // test with listener and points
        final var listener = new AffineTransformation2DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(final AffineTransformation2DRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateEnd(final AffineTransformation2DRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateNextIteration(
                    final AffineTransformation2DRobustEstimator estimator, final int iteration) {
                // no action needed
            }

            @Override
            public void onEstimateProgressChange(
                    final AffineTransformation2DRobustEstimator estimator, final float progress) {
                // no action needed
            }
        };

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(listener, inputPoints, outputPoints,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPointCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(listener, inputPoints, outputPoints,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPointCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(listener, inputPoints, outputPoints,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPointCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(listener, inputPoints, outputPoints,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(listener, inputPoints, outputPoints,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points, quality scores and method
        final var qualityScores = new double[AffineTransformation2DRobustEstimator.MINIMUM_SIZE];
        final var wrongQualityScores = new double[1];

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPointCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPointCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPointCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> AffineTransformation2DRobustEstimator.createFromPoints(
                emptyPoints, outputPoints, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> AffineTransformation2DRobustEstimator.createFromPoints(
                inputPoints, emptyPoints, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> AffineTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, wrongQualityScores, RobustEstimatorMethod.PROMEDS));

        // test with listener, points, quality scores and method
        estimator = AffineTransformation2DRobustEstimator.createFromPoints(listener, inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPointCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(listener, inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPointCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(listener, inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPointCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(listener, inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPointCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(listener, inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> AffineTransformation2DRobustEstimator.createFromPoints(
                listener, emptyPoints, outputPoints, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> AffineTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, emptyPoints, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> AffineTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, wrongQualityScores, RobustEstimatorMethod.PROMEDS));

        // test with points
        estimator = AffineTransformation2DRobustEstimator.createFromPoints(inputPoints, outputPoints);
        assertInstanceOf(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> AffineTransformation2DRobustEstimator.createFromPoints(
                emptyPoints, outputPoints));
        assertThrows(IllegalArgumentException.class, () -> AffineTransformation2DRobustEstimator.createFromPoints(
                inputPoints, emptyPoints));

        // test with listener and points
        estimator = AffineTransformation2DRobustEstimator.createFromPoints(listener, inputPoints, outputPoints);
        assertInstanceOf(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> AffineTransformation2DRobustEstimator.createFromPoints(
                listener, emptyPoints, outputPoints));
        assertThrows(IllegalArgumentException.class, () -> AffineTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, emptyPoints));

        // test with points and quality scores
        estimator = AffineTransformation2DRobustEstimator.createFromPoints(inputPoints, outputPoints, qualityScores);
        assertInstanceOf(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points and quality scores
        estimator = AffineTransformation2DRobustEstimator.createFromPoints(listener, inputPoints, outputPoints,
                qualityScores);
        assertInstanceOf(PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
    }

    @Test
    void testFromLines() {
        // create with lines and method
        final var inputLines = new ArrayList<Line2D>();
        final var outputLines = new ArrayList<Line2D>();
        for (var i = 0; i < AffineTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputLines.add(new Line2D());
            outputLines.add(new Line2D());
        }

        var estimator = AffineTransformation2DRobustEstimator.createFromLines(inputLines, outputLines,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(inputLines, outputLines,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(inputLines, outputLines,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(inputLines, outputLines,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(inputLines, outputLines,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final var emptyLines = new ArrayList<Line2D>();
        assertThrows(IllegalArgumentException.class, () -> AffineTransformation2DRobustEstimator.createFromLines(
                emptyLines, outputLines, RobustEstimatorMethod.LMEDS));
        assertThrows(IllegalArgumentException.class, () -> AffineTransformation2DRobustEstimator.createFromLines(
                inputLines, emptyLines, RobustEstimatorMethod.LMEDS));

        // test with listener and points
        final var listener = new AffineTransformation2DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(final AffineTransformation2DRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateEnd(final AffineTransformation2DRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateNextIteration(
                    final AffineTransformation2DRobustEstimator estimator, final int iteration) {
                // no action needed
            }

            @Override
            public void onEstimateProgressChange(
                    final AffineTransformation2DRobustEstimator estimator, final float progress) {
                // no action needed
            }
        };

        estimator = AffineTransformation2DRobustEstimator.createFromLines(listener, inputLines, outputLines,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(listener, inputLines, outputLines,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(listener, inputLines, outputLines,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(listener, inputLines, outputLines,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(listener, inputLines, outputLines,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points, quality scores and method
        final var qualityScores = new double[PointCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE];
        final var wrongQualityScores = new double[1];

        estimator = AffineTransformation2DRobustEstimator.createFromLines(inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> AffineTransformation2DRobustEstimator.createFromLines(
                emptyLines, outputLines, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> AffineTransformation2DRobustEstimator.createFromLines(
                inputLines, emptyLines, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> AffineTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines, wrongQualityScores, RobustEstimatorMethod.PROMEDS));

        // test with listener, points, quality scores and method
        estimator = AffineTransformation2DRobustEstimator.createFromLines(listener, inputLines, outputLines,
                qualityScores, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(listener, inputLines, outputLines,
                qualityScores, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(listener, inputLines, outputLines,
                qualityScores, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(listener, inputLines, outputLines,
                qualityScores, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(listener, inputLines, outputLines,
                qualityScores, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> AffineTransformation2DRobustEstimator.createFromLines(
                listener, emptyLines, outputLines, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> AffineTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, emptyLines, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> AffineTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, outputLines, wrongQualityScores, RobustEstimatorMethod.PROMEDS));

        // test with points
        estimator = AffineTransformation2DRobustEstimator.createFromLines(inputLines, outputLines);
        assertInstanceOf(PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> AffineTransformation2DRobustEstimator.createFromLines(
                emptyLines, outputLines));
        assertThrows(IllegalArgumentException.class, () -> AffineTransformation2DRobustEstimator.createFromLines(
                inputLines, emptyLines));

        // test with listener and points
        estimator = AffineTransformation2DRobustEstimator.createFromLines(listener, inputLines, outputLines);
        assertInstanceOf(PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> AffineTransformation2DRobustEstimator.createFromLines(
                listener, emptyLines, outputLines));
        assertThrows(IllegalArgumentException.class, () -> AffineTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, emptyLines));

        // test with points and quality scores
        estimator = AffineTransformation2DRobustEstimator.createFromLines(inputLines, outputLines, qualityScores);
        assertInstanceOf(PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points and quality scores
        estimator = AffineTransformation2DRobustEstimator.createFromLines(listener, inputLines, outputLines,
                qualityScores);
        assertInstanceOf(PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
    }
}
