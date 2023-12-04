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
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class AffineTransformation2DRobustEstimatorTest {

    @Test
    public void testConstants() {
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
    public void testCreateFromPoints() {
        AffineTransformation2DRobustEstimator estimator;

        // create with points and method
        final List<Point2D> inputPoints = new ArrayList<>();
        final List<Point2D> outputPoints = new ArrayList<>();
        for (int i = 0; i < AffineTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final List<Point2D> emptyPoints = new ArrayList<>();

        estimator = null;
        try {
            estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                    emptyPoints, outputPoints, RobustEstimatorMethod.LMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                    inputPoints, emptyPoints, RobustEstimatorMethod.LMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener and points
        final AffineTransformation2DRobustEstimatorListener listener =
                new AffineTransformation2DRobustEstimatorListener() {

                    @Override
                    public void onEstimateStart(
                            final AffineTransformation2DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateEnd(
                            final AffineTransformation2DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final AffineTransformation2DRobustEstimator estimator,
                            final int iteration) {
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final AffineTransformation2DRobustEstimator estimator,
                            final float progress) {
                    }
                };

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points, quality scores and method
        final double[] qualityScores = new double[
                AffineTransformation2DRobustEstimator.MINIMUM_SIZE];
        final double[] wrongQualityScores = new double[1];

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                    emptyPoints, outputPoints, qualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                    inputPoints, emptyPoints, qualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                    inputPoints, outputPoints, wrongQualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, points, quality scores and method
        estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                    listener, emptyPoints, outputPoints, qualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                    listener, inputPoints, emptyPoints, qualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                    listener, inputPoints, outputPoints, wrongQualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with points
        estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                    emptyPoints, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                    inputPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener and points
        estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                    listener, emptyPoints, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                    listener, inputPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with points and quality scores
        estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points and quality scores
        estimator = AffineTransformation2DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
    }

    @Test
    public void testFromLines() {
        AffineTransformation2DRobustEstimator estimator;

        // create with lines and method
        final List<Line2D> inputLines = new ArrayList<>();
        final List<Line2D> outputLines = new ArrayList<>();
        for (int i = 0; i < AffineTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputLines.add(new Line2D());
            outputLines.add(new Line2D());
        }

        estimator = AffineTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final List<Line2D> emptyLines = new ArrayList<>();

        estimator = null;
        try {
            estimator = AffineTransformation2DRobustEstimator.createFromLines(
                    emptyLines, outputLines, RobustEstimatorMethod.LMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = AffineTransformation2DRobustEstimator.createFromLines(
                    inputLines, emptyLines, RobustEstimatorMethod.LMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener and points
        final AffineTransformation2DRobustEstimatorListener listener =
                new AffineTransformation2DRobustEstimatorListener() {

                    @Override
                    public void onEstimateStart(
                            final AffineTransformation2DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateEnd(
                            final AffineTransformation2DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final AffineTransformation2DRobustEstimator estimator,
                            final int iteration) {
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final AffineTransformation2DRobustEstimator estimator,
                            final float progress) {
                    }
                };

        estimator = AffineTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, outputLines,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, outputLines,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, outputLines,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, outputLines,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, outputLines,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points, quality scores and method
        final double[] qualityScores = new double[
                PointCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE];
        final double[] wrongQualityScores = new double[1];

        estimator = AffineTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = AffineTransformation2DRobustEstimator.createFromLines(
                    emptyLines, outputLines, qualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = AffineTransformation2DRobustEstimator.createFromLines(
                    inputLines, emptyLines, qualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = AffineTransformation2DRobustEstimator.createFromLines(
                    inputLines, outputLines, wrongQualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, points, quality scores and method
        estimator = AffineTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = AffineTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, outputLines, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = AffineTransformation2DRobustEstimator.createFromLines(
                    listener, emptyLines, outputLines, qualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = AffineTransformation2DRobustEstimator.createFromLines(
                    listener, inputLines, emptyLines, qualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = AffineTransformation2DRobustEstimator.createFromLines(
                    listener, inputLines, outputLines, wrongQualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with points
        estimator = AffineTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = AffineTransformation2DRobustEstimator.createFromLines(
                    emptyLines, outputLines);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = AffineTransformation2DRobustEstimator.createFromLines(
                    inputLines, emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener and points
        estimator = AffineTransformation2DRobustEstimator.
                createFromLines(listener, inputLines, outputLines);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = AffineTransformation2DRobustEstimator.createFromLines(
                    listener, emptyLines, outputLines);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = AffineTransformation2DRobustEstimator.createFromLines(
                    listener, inputLines, emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with points and quality scores
        estimator = AffineTransformation2DRobustEstimator.createFromLines(
                inputLines, outputLines, qualityScores);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points and quality scores
        estimator = AffineTransformation2DRobustEstimator.createFromLines(
                listener, inputLines, outputLines, qualityScores);
        assertTrue(estimator instanceof
                PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
    }
}
