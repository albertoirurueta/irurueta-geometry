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

class PlaneCorrespondenceProjectiveTransformation3DRobustEstimatorTest {

    @Test
    void testConstants() {
        assertEquals(RobustEstimatorMethod.PROMEDS,
                PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.DEFAULT_ROBUST_METHOD);
    }

    @Test
    void testCreate() {
        // create with robust estimator method
        var estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // create with lines and method
        final var inputPlanes = new ArrayList<Plane>();
        final var outputPlanes = new ArrayList<Plane>();
        for (var i = 0; i < PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPlanes.add(new Plane());
            outputPlanes.add(new Plane());
        }

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(inputPlanes, outputPlanes,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(inputPlanes, outputPlanes,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(inputPlanes, outputPlanes,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(inputPlanes, outputPlanes,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(inputPlanes, outputPlanes,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final var emptyLines = new ArrayList<Plane>();
        assertThrows(IllegalArgumentException.class,
                () -> PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(emptyLines, outputPlanes,
                        RobustEstimatorMethod.LMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(inputPlanes, emptyLines,
                        RobustEstimatorMethod.LMEDS));

        // create with listener and method
        final var listener = new ProjectiveTransformation3DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(final ProjectiveTransformation3DRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateEnd(final ProjectiveTransformation3DRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateNextIteration(
                    final ProjectiveTransformation3DRobustEstimator estimator, final int iteration) {
                // no action needed
            }

            @Override
            public void onEstimateProgressChange(
                    final ProjectiveTransformation3DRobustEstimator estimator, final float progress) {
                // no action needed
            }
        };

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and points
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener, inputPlanes,
                outputPlanes, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener, inputPlanes,
                outputPlanes, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener, inputPlanes,
                outputPlanes, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener, inputPlanes,
                outputPlanes, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener, inputPlanes,
                outputPlanes, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with quality scores and method
        final var qualityScores = new double[PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE];
        final var wrongQualityScores = new double[1];

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points, quality scores and method
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(inputPlanes, outputPlanes,
                qualityScores, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(inputPlanes, outputPlanes,
                qualityScores, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(inputPlanes, outputPlanes,
                qualityScores, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(inputPlanes, outputPlanes,
                qualityScores, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(inputPlanes, outputPlanes,
                qualityScores, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(emptyLines, outputPlanes,
                        qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(inputPlanes, emptyLines,
                        qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(inputPlanes, outputPlanes,
                        wrongQualityScores, RobustEstimatorMethod.PROMEDS));

        // test with listener, quality scores and method
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points, quality scores and method
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener, inputPlanes,
                outputPlanes, qualityScores, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener, inputPlanes,
                outputPlanes, qualityScores, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener, inputPlanes,
                outputPlanes, qualityScores, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener, inputPlanes,
                outputPlanes, qualityScores, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener, inputPlanes,
                outputPlanes, qualityScores, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener, emptyLines,
                        outputPlanes, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener, inputPlanes,
                        emptyLines, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class,
                () -> PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener, inputPlanes,
                        outputPlanes, wrongQualityScores, RobustEstimatorMethod.PROMEDS));

        // test no arguments
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create();
        assertInstanceOf(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(inputPlanes, outputPlanes);
        assertInstanceOf(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(emptyLines, outputPlanes));
        assertThrows(IllegalArgumentException.class,
                () -> PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(inputPlanes, emptyLines));

        // test with listener
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener);
        assertInstanceOf(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and points
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener, inputPlanes,
                outputPlanes);
        assertInstanceOf(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener, emptyLines,
                        outputPlanes));
        assertThrows(IllegalArgumentException.class,
                () -> PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener, inputPlanes,
                        emptyLines));

        // test with quality scores
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(qualityScores);
        assertInstanceOf(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points and quality scores
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(inputPlanes, outputPlanes,
                qualityScores);
        assertInstanceOf(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and quality scores
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener, qualityScores);
        assertInstanceOf(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points and quality scores
        estimator = PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.create(listener, inputPlanes,
                outputPlanes, qualityScores);
        assertInstanceOf(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(inputPlanes, estimator.getInputPlanes());
        assertSame(outputPlanes, estimator.getOutputPlanes());
        assertTrue(estimator.isReady());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
    }
}
