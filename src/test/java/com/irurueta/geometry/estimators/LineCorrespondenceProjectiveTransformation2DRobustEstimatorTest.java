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

class LineCorrespondenceProjectiveTransformation2DRobustEstimatorTest {

    @Test
    void testConstants() {
        assertEquals(RobustEstimatorMethod.PROMEDS,
                LineCorrespondenceProjectiveTransformation2DRobustEstimator.DEFAULT_ROBUST_METHOD);
    }

    @Test
    void testCreate() {
        // create with robust estimator method
        var estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // create with lines and method
        final var inputLines = new ArrayList<Line2D>();
        final var outputLines = new ArrayList<Line2D>();
        for (var i = 0; i < LineCorrespondenceProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputLines.add(new Line2D());
            outputLines.add(new Line2D());
        }

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputLines, outputLines,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputLines, outputLines,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputLines, outputLines,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputLines, outputLines,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputLines, outputLines,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final var emptyLines = new ArrayList<Line2D>();
        assertThrows(IllegalArgumentException.class,
                () -> LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(emptyLines, outputLines,
                        RobustEstimatorMethod.LMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputLines, emptyLines,
                        RobustEstimatorMethod.LMEDS));

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


        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and points
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputLines,
                outputLines, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputLines,
                outputLines, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputLines,
                outputLines, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputLines,
                outputLines, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputLines,
                outputLines, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with quality scores and method
        final var qualityScores = new double[LineCorrespondenceProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE];
        final var wrongQualityScores = new double[1];

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points, quality scores and method
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputLines, outputLines,
                qualityScores, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputLines, outputLines,
                qualityScores, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputLines, outputLines,
                qualityScores, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputLines, outputLines,
                qualityScores, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputLines, outputLines,
                qualityScores, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(emptyLines, outputLines,
                        qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputLines, emptyLines,
                        qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputLines, outputLines,
                        wrongQualityScores, RobustEstimatorMethod.PROMEDS));

        // test with listener, quality scores and method
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points, quality scores and method
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputLines,
                outputLines, qualityScores, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputLines,
                outputLines, qualityScores, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputLines,
                outputLines, qualityScores, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputLines,
                outputLines, qualityScores, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputLines,
                outputLines, qualityScores, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, emptyLines,
                        outputLines, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputLines,
                        emptyLines, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputLines,
                        outputLines, wrongQualityScores, RobustEstimatorMethod.PROMEDS));

        // test no arguments
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create();
        assertInstanceOf(PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputLines, outputLines);
        assertInstanceOf(PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(emptyLines, outputLines));
        assertThrows(IllegalArgumentException.class,
                () -> LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputLines, emptyLines));

        // test with listener
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener);
        assertInstanceOf(PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and points
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputLines,
                outputLines);
        assertInstanceOf(PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, emptyLines,
                        outputLines));
        assertThrows(IllegalArgumentException.class,
                () -> LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputLines,
                        emptyLines));

        // test with quality scores
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(qualityScores);
        assertInstanceOf(PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points and quality scores
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(inputLines, outputLines,
                qualityScores);
        assertInstanceOf(PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and quality scores
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, qualityScores);
        assertInstanceOf(PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points and quality scores
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.create(listener, inputLines,
                outputLines, qualityScores);
        assertInstanceOf(PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.class, estimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
    }
}
