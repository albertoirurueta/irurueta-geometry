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
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class LineCorrespondenceProjectiveTransformation2DRobustEstimatorTest {

    @Test
    public void testConstants() {
        assertEquals(RobustEstimatorMethod.PROMEDS,
                LineCorrespondenceProjectiveTransformation2DRobustEstimator.DEFAULT_ROBUST_METHOD);
    }

    @Test
    public void testCreate() {
        LineCorrespondenceProjectiveTransformation2DRobustEstimator estimator;

        // create with robust estimator method
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // create with lines and method
        final List<Line2D> inputLines = new ArrayList<>();
        final List<Line2D> outputLines = new ArrayList<>();
        for (int i = 0; i < LineCorrespondenceProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputLines.add(new Line2D());
            outputLines.add(new Line2D());
        }

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputLines, outputLines, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputLines, outputLines, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputLines, outputLines, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputLines, outputLines, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputLines, outputLines, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final List<Line2D> emptyLines = new ArrayList<>();

        estimator = null;
        try {
            estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create(emptyLines, outputLines,
                            RobustEstimatorMethod.LMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create(inputLines, emptyLines,
                            RobustEstimatorMethod.LMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // create with listener and method
        final ProjectiveTransformation2DRobustEstimatorListener listener =
                new ProjectiveTransformation2DRobustEstimatorListener() {

                    @Override
                    public void onEstimateStart(final ProjectiveTransformation2DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateEnd(final ProjectiveTransformation2DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final ProjectiveTransformation2DRobustEstimator estimator,
                            final int iteration) {
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final ProjectiveTransformation2DRobustEstimator estimator,
                            final float progress) {
                    }
                };


        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and points
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines,
                        RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines,
                        RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines,
                        RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines,
                        RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines,
                        RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with quality scores and method
        final double[] qualityScores = new double[
                LineCorrespondenceProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE];
        final double[] wrongQualityScores = new double[1];

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points, quality scores and method
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputLines, outputLines, qualityScores,
                        RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputLines, outputLines, qualityScores,
                        RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputLines, outputLines, qualityScores,
                        RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputLines, outputLines, qualityScores,
                        RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputLines, outputLines, qualityScores,
                        RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create(emptyLines, outputLines, qualityScores,
                            RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create(inputLines, emptyLines, qualityScores,
                            RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create(inputLines, outputLines, wrongQualityScores,
                            RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, quality scores and method
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points, quality scores and method
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines, qualityScores,
                        RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines, qualityScores,
                        RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines, qualityScores,
                        RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines, qualityScores,
                        RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines, qualityScores,
                        RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create(listener, emptyLines, outputLines, qualityScores,
                            RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create(listener, inputLines, emptyLines, qualityScores,
                            RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create(listener, inputLines, outputLines, wrongQualityScores,
                            RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test no arguments
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create();
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputLines, outputLines);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create(emptyLines, outputLines);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create(inputLines, emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and points
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create(listener, emptyLines, outputLines);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create(listener, inputLines, emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with quality scores
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(qualityScores);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points and quality scores
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputLines, outputLines, qualityScores);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and quality scores
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, qualityScores);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points and quality scores
        estimator = LineCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputLines, outputLines, qualityScores);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(inputLines, estimator.getInputLines());
        assertSame(outputLines, estimator.getOutputLines());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
    }
}
