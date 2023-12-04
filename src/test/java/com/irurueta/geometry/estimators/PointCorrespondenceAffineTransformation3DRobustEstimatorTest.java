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
import org.junit.*;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class PointCorrespondenceAffineTransformation3DRobustEstimatorTest {

    @Test
    public void testConstants() {
        assertEquals(RobustEstimatorMethod.PROMEDS,
                PointCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_ROBUST_METHOD);
    }

    @Test
    public void testCreate() {
        // create with robust estimator method
        PointCorrespondenceAffineTransformation3DRobustEstimator estimator =
                PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // create with points and method
        final List<Point3D> inputPoints = new ArrayList<>();
        final List<Point3D> outputPoints = new ArrayList<>();
        for (int i = 0; i < PointCorrespondenceAffineTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPoints, outputPoints, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPoints, outputPoints, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPoints, outputPoints, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPoints, outputPoints, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPoints, outputPoints, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final List<Point3D> emptyPoints = new ArrayList<>();

        estimator = null;
        try {
            estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                    create(emptyPoints, outputPoints,
                            RobustEstimatorMethod.LMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                    create(inputPoints, emptyPoints,
                            RobustEstimatorMethod.LMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // create with listener and method
        final AffineTransformation3DRobustEstimatorListener listener =
                new AffineTransformation3DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(final AffineTransformation3DRobustEstimator estimator) {
            }

            @Override
            public void onEstimateEnd(final AffineTransformation3DRobustEstimator estimator) {
            }

            @Override
            public void onEstimateNextIteration(
                    final AffineTransformation3DRobustEstimator estimator, final int iteration) {
            }

            @Override
            public void onEstimateProgressChange(
                    final AffineTransformation3DRobustEstimator estimator, final float progress) {
            }
        };


        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and points
        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPoints, outputPoints,
                        RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPoints, outputPoints,
                        RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPoints, outputPoints,
                        RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPoints, outputPoints,
                        RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPoints, outputPoints,
                        RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with quality scores and method
        final double[] qualityScores = new double[
                PointCorrespondenceAffineTransformation3DRobustEstimator.MINIMUM_SIZE];
        final double[] wrongQualityScores = new double[1];

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points, quality scores and method
        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPoints, outputPoints, qualityScores,
                        RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPoints, outputPoints, qualityScores,
                        RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPoints, outputPoints, qualityScores,
                        RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPoints, outputPoints, qualityScores,
                        RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPoints, outputPoints, qualityScores,
                        RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
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
            estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                    create(emptyPoints, outputPoints, qualityScores,
                            RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                    create(inputPoints, emptyPoints, qualityScores,
                            RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                    create(inputPoints, outputPoints, wrongQualityScores,
                            RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, quality scores and method
        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points, quality scores and method
        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPoints, outputPoints, qualityScores,
                        RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPoints, outputPoints, qualityScores,
                        RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPoints, outputPoints, qualityScores,
                        RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPoints, outputPoints, qualityScores,
                        RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPoints, outputPoints, qualityScores,
                        RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
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
            estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                    create(listener, emptyPoints, outputPoints, qualityScores,
                            RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                    create(listener, inputPoints, emptyPoints, qualityScores,
                            RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                    create(listener, inputPoints, outputPoints, wrongQualityScores,
                            RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test no arguments
        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create();
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points
        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertFalse(estimator.isReady());
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
            estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                    create(emptyPoints, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                    create(inputPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener
        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and points
        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertFalse(estimator.isReady());
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
            estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                    create(listener, emptyPoints, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) {
        }
        try {
            estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                    create(listener, inputPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with quality scores
        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(qualityScores);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points and quality scores
        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and quality scores
        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points and quality scores
        estimator = PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(listener, inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceAffineTransformation3DRobustEstimator);
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
    }
}
