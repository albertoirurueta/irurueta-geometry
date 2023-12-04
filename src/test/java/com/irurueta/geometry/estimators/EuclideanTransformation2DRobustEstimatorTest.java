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

import com.irurueta.geometry.Point2D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class EuclideanTransformation2DRobustEstimatorTest {

    @Test
    public void testConstants() {
        assertEquals(3, EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE);
        assertEquals(2, EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE);
        assertEquals(0.05f, EuclideanTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0f);
        assertEquals(0.0f, EuclideanTransformation2DRobustEstimator.MIN_PROGRESS_DELTA, 0.0f);
        assertEquals(1.0f, EuclideanTransformation2DRobustEstimator.MAX_PROGRESS_DELTA, 0.0f);
        assertEquals(0.99, EuclideanTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(5000, EuclideanTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(0.0, EuclideanTransformation2DRobustEstimator.MIN_CONFIDENCE, 0.0);
        assertEquals(1.0, EuclideanTransformation2DRobustEstimator.MAX_CONFIDENCE, 0.0);
        assertEquals(1, EuclideanTransformation2DRobustEstimator.MIN_ITERATIONS);
        assertTrue(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(EuclideanTransformation2DRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertEquals(RobustEstimatorMethod.PROMEDS, EuclideanTransformation2DRobustEstimator.DEFAULT_ROBUST_METHOD);
    }

    @Test
    public void testCreate() {
        EuclideanTransformation2DRobustEstimator estimator;

        // create with method
        estimator = EuclideanTransformation2DRobustEstimator.create(RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACEuclideanTransformation2DRobustEstimator);

        estimator = EuclideanTransformation2DRobustEstimator.create(RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEuclideanTransformation2DRobustEstimator);

        estimator = EuclideanTransformation2DRobustEstimator.create(RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEuclideanTransformation2DRobustEstimator);

        estimator = EuclideanTransformation2DRobustEstimator.create(RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEuclideanTransformation2DRobustEstimator);

        estimator = EuclideanTransformation2DRobustEstimator.create(RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);

        // create with points and method
        List<Point2D> inputPoints = new ArrayList<>();
        List<Point2D> outputPoints = new ArrayList<>();
        for (int i = 0; i < EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }

        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final List<Point2D> emptyPoints = new ArrayList<>();

        estimator = null;
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    emptyPoints, outputPoints, RobustEstimatorMethod.LMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    inputPoints, emptyPoints, RobustEstimatorMethod.LMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener
        final EuclideanTransformation2DRobustEstimatorListener listener =
                new EuclideanTransformation2DRobustEstimatorListener() {

                    @Override
                    public void onEstimateStart(
                            final EuclideanTransformation2DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateEnd(
                            final EuclideanTransformation2DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final EuclideanTransformation2DRobustEstimator estimator,
                            final int iteration) {
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final EuclideanTransformation2DRobustEstimator estimator,
                            final float progress) {
                    }
                };

        estimator = EuclideanTransformation2DRobustEstimator.create(listener, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());

        estimator = EuclideanTransformation2DRobustEstimator.create(listener, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());

        estimator = EuclideanTransformation2DRobustEstimator.create(listener, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());

        estimator = EuclideanTransformation2DRobustEstimator.create(listener, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());

        estimator = EuclideanTransformation2DRobustEstimator.create(listener, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());

        // test with quality scores and method
        double[] qualityScores = new double[
                EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE];
        estimator = EuclideanTransformation2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getQualityScores());
        assertTrue(estimator instanceof
                RANSACEuclideanTransformation2DRobustEstimator);

        estimator = EuclideanTransformation2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertNull(estimator.getQualityScores());
        assertTrue(estimator instanceof
                LMedSEuclideanTransformation2DRobustEstimator);

        estimator = EuclideanTransformation2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.MSAC);
        assertNull(estimator.getQualityScores());
        assertTrue(estimator instanceof
                MSACEuclideanTransformation2DRobustEstimator);

        estimator = EuclideanTransformation2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator instanceof
                PROSACEuclideanTransformation2DRobustEstimator);

        estimator = EuclideanTransformation2DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);

        // test with listener and points
        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points, quality scores and method
        final double[] wrongQualityScores = new double[1];

        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    emptyPoints, outputPoints, qualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    inputPoints, emptyPoints, qualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    inputPoints, outputPoints, wrongQualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, quality scores and method
        estimator = EuclideanTransformation2DRobustEstimator.create(listener,
                qualityScores, RobustEstimatorMethod.RANSAC);
        assertSame(listener, estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertTrue(estimator instanceof
                RANSACEuclideanTransformation2DRobustEstimator);

        estimator = EuclideanTransformation2DRobustEstimator.create(listener,
                qualityScores, RobustEstimatorMethod.LMEDS);
        assertSame(listener, estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertTrue(estimator instanceof
                LMedSEuclideanTransformation2DRobustEstimator);

        estimator = EuclideanTransformation2DRobustEstimator.create(listener,
                qualityScores, RobustEstimatorMethod.MSAC);
        assertSame(listener, estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertTrue(estimator instanceof
                MSACEuclideanTransformation2DRobustEstimator);

        estimator = EuclideanTransformation2DRobustEstimator.create(listener,
                qualityScores, RobustEstimatorMethod.PROSAC);
        assertSame(listener, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator instanceof
                PROSACEuclideanTransformation2DRobustEstimator);

        estimator = EuclideanTransformation2DRobustEstimator.create(listener,
                qualityScores, RobustEstimatorMethod.PROMEDS);
        assertSame(listener, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);

        // test with listener, points, quality scores and method
        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    listener, emptyPoints, outputPoints, qualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    listener, inputPoints, emptyPoints, qualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    listener, inputPoints, outputPoints, wrongQualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with weak minimum and method
        estimator = EuclideanTransformation2DRobustEstimator.create(
                true, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACEuclideanTransformation2DRobustEstimator);
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                true, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEuclideanTransformation2DRobustEstimator);
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                true, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEuclideanTransformation2DRobustEstimator);
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                true, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEuclideanTransformation2DRobustEstimator);
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                true, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        // test with listener, weak minimum and method
        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, true, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, true, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, true, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, true, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, true, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        // test with quality scores, weak minimum and method
        estimator = EuclideanTransformation2DRobustEstimator.create(
                qualityScores, true,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                qualityScores, true,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                qualityScores, true,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                qualityScores, true,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEuclideanTransformation2DRobustEstimator);
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                qualityScores, true,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        // test with listener, quality scores, weak minimum and method
        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, qualityScores, true,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, qualityScores, true,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, qualityScores, true,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                qualityScores, true,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEuclideanTransformation2DRobustEstimator);
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                qualityScores, true,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        // create with points and method and weak points
        inputPoints = new ArrayList<>();
        outputPoints = new ArrayList<>();
        for (int i = 0; i < EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE; i++) {
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }

        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, true, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    emptyPoints, outputPoints, true,
                    RobustEstimatorMethod.LMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    inputPoints, emptyPoints, true,
                    RobustEstimatorMethod.LMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener and points
        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, true,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, true,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, true,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, true,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, true,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points, quality scores and method
        qualityScores = new double[
                EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE];

        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    emptyPoints, outputPoints, qualityScores, true,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    inputPoints, emptyPoints, qualityScores, true,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    inputPoints, outputPoints, wrongQualityScores, true,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, points, quality scores and method
        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    listener, emptyPoints, outputPoints, qualityScores, true,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    listener, inputPoints, emptyPoints, qualityScores, true,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    listener, inputPoints, outputPoints, wrongQualityScores,
                    true, RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with points
        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, true);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    emptyPoints, outputPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    inputPoints, emptyPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener and points
        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, true);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    listener, emptyPoints, outputPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    listener, inputPoints, emptyPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with points and quality scores
        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points and quality scores
        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, true);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
    }

    @Test
    public void testCreate2() {
        EuclideanTransformation2DRobustEstimator estimator;

        // create
        estimator = EuclideanTransformation2DRobustEstimator.create();
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);

        // create with points and method
        List<Point2D> inputPoints = new ArrayList<>();
        List<Point2D> outputPoints = new ArrayList<>();
        for (int i = 0; i < EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }

        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final List<Point2D> emptyPoints = new ArrayList<>();

        estimator = null;
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    emptyPoints, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    inputPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // create with listener
        final EuclideanTransformation2DRobustEstimatorListener listener =
                new EuclideanTransformation2DRobustEstimatorListener() {

                    @Override
                    public void onEstimateStart(
                            final EuclideanTransformation2DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateEnd(
                            final EuclideanTransformation2DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final EuclideanTransformation2DRobustEstimator estimator,
                            final int iteration) {
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final EuclideanTransformation2DRobustEstimator estimator,
                            final float progress) {
                    }
                };

        estimator = EuclideanTransformation2DRobustEstimator.create(listener);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());

        // test with listener and points
        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with quality scores
        double[] qualityScores = new double[
                EuclideanTransformation2DRobustEstimator.MINIMUM_SIZE];
        estimator = EuclideanTransformation2DRobustEstimator.create(qualityScores);
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);

        // test with points and quality scores
        final double[] wrongQualityScores = new double[1];

        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    emptyPoints, outputPoints, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    inputPoints, emptyPoints, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    inputPoints, outputPoints, wrongQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener and quality scores
        estimator = EuclideanTransformation2DRobustEstimator.create(listener,
                qualityScores);
        assertSame(listener, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);

        // test with listener, points and quality scores
        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    listener, emptyPoints, outputPoints, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    listener, inputPoints, emptyPoints, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    listener, inputPoints, outputPoints, wrongQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with weak minimum
        estimator = EuclideanTransformation2DRobustEstimator.create(true);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        // create with points and method and weak points
        inputPoints = new ArrayList<>();
        outputPoints = new ArrayList<>();
        for (int i = 0; i < EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE; i++) {
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }
        qualityScores = new double[EuclideanTransformation2DRobustEstimator.WEAK_MINIMUM_SIZE];

        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, true);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    emptyPoints, outputPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    inputPoints, emptyPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener and weak minimum
        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, true);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        // test with listener, points and weak minimum
        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, true);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    listener, emptyPoints, outputPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    listener, inputPoints, emptyPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with quality scores and weak minimum
        estimator = EuclideanTransformation2DRobustEstimator.create(
                qualityScores, true);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        // test with points, quality scores and weak minimum
        estimator = EuclideanTransformation2DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, true);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    emptyPoints, outputPoints, qualityScores, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    inputPoints, emptyPoints, qualityScores, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = EuclideanTransformation2DRobustEstimator.create(
                    inputPoints, outputPoints, wrongQualityScores,
                    true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, quality scores and weak minimum
        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, qualityScores, true);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator.isWeakMinimumSizeAllowed());

        // test with listener, points, quality scores and weak minimum
        estimator = EuclideanTransformation2DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores,
                true);
        assertTrue(estimator instanceof
                PROMedSEuclideanTransformation2DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertNull(estimator.getInliersData());
        assertEquals(EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
    }
}
