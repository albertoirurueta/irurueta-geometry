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
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class LineCorrespondenceAffineTransformation2DRobustEstimatorTest {

    @Test
    void testConstants() {
        assertEquals(RobustEstimatorMethod.PROMEDS,
                LineCorrespondenceAffineTransformation2DRobustEstimator.DEFAULT_ROBUST_METHOD);
    }

    @Test
    void testCreate() {
        // create with robust estimator method
        var estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // create with lines and method
        final var inputLines = new ArrayList<Line2D>();
        final var outputLines = new ArrayList<Line2D>();
        for (var i = 0; i < LineCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputLines.add(new Line2D());
            outputLines.add(new Line2D());
        }

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(inputLines, outputLines,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(inputLines, outputLines,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(inputLines, outputLines,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(inputLines, outputLines,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(inputLines, outputLines,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final var emptyLines = new ArrayList<Line2D>();

        assertThrows(IllegalArgumentException.class,
                () -> LineCorrespondenceAffineTransformation2DRobustEstimator.create(emptyLines, outputLines,
                        RobustEstimatorMethod.LMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> LineCorrespondenceAffineTransformation2DRobustEstimator.create(inputLines, emptyLines,
                        RobustEstimatorMethod.LMEDS));

        // create with listener and method
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


        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and points
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener, inputLines, outputLines,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener, inputLines, outputLines,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener, inputLines, outputLines,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener, inputLines, outputLines,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener, inputLines, outputLines,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with quality scores and method
        final var qualityScores = new double[LineCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE];
        final var wrongQualityScores = new double[1];

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points, quality scores and method
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(inputLines, outputLines,
                qualityScores, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(inputLines, outputLines,
                qualityScores, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(inputLines, outputLines,
                qualityScores, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(inputLines, outputLines,
                qualityScores, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(inputLines, outputLines,
                qualityScores, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> LineCorrespondenceAffineTransformation2DRobustEstimator.create(emptyLines, outputLines,
                        qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> LineCorrespondenceAffineTransformation2DRobustEstimator.create(inputLines, emptyLines,
                        qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> LineCorrespondenceAffineTransformation2DRobustEstimator.create(inputLines, outputLines,
                        wrongQualityScores, RobustEstimatorMethod.PROMEDS));

        // test with listener, quality scores and method
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points, quality scores and method
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener, inputLines, outputLines,
                qualityScores, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener, inputLines, outputLines,
                qualityScores, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener, inputLines, outputLines,
                qualityScores, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener, inputLines, outputLines,
                qualityScores, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener, inputLines, outputLines,
                qualityScores, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener, emptyLines, outputLines,
                        qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener, inputLines, emptyLines,
                        qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener, inputLines, outputLines,
                        wrongQualityScores, RobustEstimatorMethod.PROMEDS));

        // test no arguments
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create();
        assertInstanceOf(PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(inputLines, outputLines);
        assertInstanceOf(PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> LineCorrespondenceAffineTransformation2DRobustEstimator.create(emptyLines, outputLines));
        assertThrows(IllegalArgumentException.class,
                () -> LineCorrespondenceAffineTransformation2DRobustEstimator.create(inputLines, emptyLines));

        // test with listener
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener);
        assertInstanceOf(PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and points
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener, inputLines, outputLines);
        assertInstanceOf(PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener, emptyLines,
                        outputLines));
        assertThrows(IllegalArgumentException.class,
                () -> LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener, inputLines, emptyLines));

        // test with quality scores
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(qualityScores);
        assertInstanceOf(PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points and quality scores
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(inputLines, outputLines,
                qualityScores);
        assertInstanceOf(PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and quality scores
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener, qualityScores);
        assertInstanceOf(PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points and quality scores
        estimator = LineCorrespondenceAffineTransformation2DRobustEstimator.create(listener, inputLines, outputLines,
                qualityScores);
        assertInstanceOf(PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
    }
}
