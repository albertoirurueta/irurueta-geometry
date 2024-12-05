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

import com.irurueta.geometry.Plane;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class PlaneCorrespondenceAffineTransformation3DRobustEstimatorTest {

    @Test
    void testConstants() {
        assertEquals(RobustEstimatorMethod.PROMEDS,
                PlaneCorrespondenceAffineTransformation3DRobustEstimator.DEFAULT_ROBUST_METHOD);
    }

    @Test
    void testCreate() {
        // create with robust estimator method
        var estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // create with planes and method
        final var inputPlanes = new ArrayList<Plane>();
        final var outputPlanes = new ArrayList<Plane>();
        for (var i = 0; i < PlaneCorrespondenceAffineTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPlanes.add(new Plane());
            outputPlanes.add(new Plane());
        }

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(inputPlanes, outputPlanes,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(inputPlanes, outputPlanes,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(inputPlanes, outputPlanes,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(inputPlanes, outputPlanes,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(inputPlanes, outputPlanes,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final var emptyPlanes = new ArrayList<Plane>();
        assertThrows(IllegalArgumentException.class,
                () -> PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(emptyPlanes, outputPlanes,
                        RobustEstimatorMethod.LMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(inputPlanes, emptyPlanes,
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
            public void onEstimateNextIteration(
                    final AffineTransformation3DRobustEstimator estimator, final int iteration) {
                // no action needed
            }

            @Override
            public void onEstimateProgressChange(
                    final AffineTransformation3DRobustEstimator estimator, final float progress) {
                // no action needed
            }
        };


        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and points
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPlanes, outputPlanes,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPlanes, outputPlanes,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPlanes, outputPlanes,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPlanes, outputPlanes,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPlanes, outputPlanes,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with quality scores and method
        final var qualityScores = new double[PlaneCorrespondenceAffineTransformation3DRobustEstimator.MINIMUM_SIZE];
        final var wrongQualityScores = new double[1];

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points, quality scores and method
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(inputPlanes, outputPlanes,
                qualityScores, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(inputPlanes, outputPlanes,
                qualityScores, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(inputPlanes, outputPlanes,
                qualityScores, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(inputPlanes, outputPlanes,
                qualityScores, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(inputPlanes, outputPlanes,
                qualityScores, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(emptyPlanes, outputPlanes,
                        qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(inputPlanes, emptyPlanes,
                        qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(inputPlanes, outputPlanes,
                        wrongQualityScores, RobustEstimatorMethod.PROMEDS));

        // test with listener, quality scores and method
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points, quality scores and method
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPlanes, outputPlanes,
                qualityScores, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPlanes, outputPlanes,
                qualityScores, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPlanes, outputPlanes,
                qualityScores, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPlanes, outputPlanes,
                qualityScores, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPlanes, outputPlanes,
                qualityScores, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener, emptyPlanes,
                        outputPlanes, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPlanes,
                        emptyPlanes, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPlanes,
                        outputPlanes, wrongQualityScores, RobustEstimatorMethod.PROMEDS));

        // test no arguments
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create();
        assertInstanceOf(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(inputPlanes, outputPlanes);
        assertInstanceOf(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(emptyPlanes, outputPlanes));
        assertThrows(IllegalArgumentException.class,
                () -> PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(inputPlanes, emptyPlanes));

        // test with listener
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener);
        assertInstanceOf(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and points
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPlanes,
                outputPlanes);
        assertInstanceOf(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener, emptyPlanes,
                        outputPlanes));
        assertThrows(IllegalArgumentException.class,
                () -> PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPlanes,
                        emptyPlanes));

        // test with quality scores
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(qualityScores);
        assertInstanceOf(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with planes and quality scores
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(inputPlanes, outputPlanes,
                qualityScores);
        assertInstanceOf(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and quality scores
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener, qualityScores);
        assertInstanceOf(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points and quality scores
        estimator = PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(listener, inputPlanes, outputPlanes,
                qualityScores);
        assertInstanceOf(PROMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
    }
}
