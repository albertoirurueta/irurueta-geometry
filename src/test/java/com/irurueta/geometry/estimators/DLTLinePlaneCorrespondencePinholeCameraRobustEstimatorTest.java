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

import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.Plane;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class DLTLinePlaneCorrespondencePinholeCameraRobustEstimatorTest {

    @Test
    void testCreate() {
        // create with robust estimator method
        var estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // create with robust estimator method and lines and planes
        final var planes = new ArrayList<Plane>();
        final var lines = new ArrayList<Line2D>();
        for (var i = 0; i < LinePlaneCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES; i++) {
            planes.add(new Plane());
            lines.add(new Line2D());
        }

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(planes, lines,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(estimator.getLines(), lines);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(planes, lines,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(lines, estimator.getLines());
        assertSame(planes, estimator.getPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(planes, lines,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(lines, estimator.getLines());
        assertSame(planes, estimator.getPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(planes, lines,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(lines, estimator.getLines());
        assertSame(planes, estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(planes, lines,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(lines, estimator.getLines());
        assertSame(planes, estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final var emptyPlanes = new ArrayList<Plane>();
        final var emptyLines = new ArrayList<Line2D>();
        assertThrows(IllegalArgumentException.class,
                () -> DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(emptyPlanes, emptyLines,
                        RobustEstimatorMethod.RANSAC));
        assertThrows(IllegalArgumentException.class,
                () -> DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(planes, emptyLines,
                        RobustEstimatorMethod.RANSAC));

        // create with listener and robust estimator method
        final var listener = new PinholeCameraRobustEstimatorListener() {

            @Override
            public void onEstimateStart(final PinholeCameraRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateEnd(final PinholeCameraRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateNextIteration(final PinholeCameraRobustEstimator estimator, final int iteration) {
                // no action needed
            }

            @Override
            public void onEstimateProgressChange(final PinholeCameraRobustEstimator estimator, final float progress) {
                // no action needed
            }
        };

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points and robust estimator method
        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, planes, lines,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(lines, estimator.getLines());
        assertSame(planes, estimator.getPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, planes, lines,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(lines, estimator.getLines());
        assertSame(planes, estimator.getPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, planes, lines,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(lines, estimator.getLines());
        assertSame(planes, estimator.getPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, planes, lines,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(lines, estimator.getLines());
        assertSame(planes, estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, planes, lines,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(lines, estimator.getLines());
        assertSame(planes, estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with quality scores and robust estimator method
        final var qualityScores = new double[
                LinePlaneCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES];
        final var shortScores = new double[1];

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                create(shortScores, RobustEstimatorMethod.PROSAC));

        // test with points, quality scores and robust estimator method
        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(planes, lines, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(lines, estimator.getLines());
        assertSame(planes, estimator.getPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(planes, lines, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(lines, estimator.getLines());
        assertSame(planes, estimator.getPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(planes, lines, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(lines, estimator.getLines());
        assertSame(planes, estimator.getPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(planes, lines, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(lines, estimator.getLines());
        assertSame(planes, estimator.getPlanes());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(planes, lines, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(lines, estimator.getLines());
        assertSame(planes, estimator.getPlanes());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(emptyPlanes, emptyLines,
                        qualityScores, RobustEstimatorMethod.PROSAC));
        assertThrows(IllegalArgumentException.class,
                () -> DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(planes, emptyLines, qualityScores,
                        RobustEstimatorMethod.PROSAC));
        assertThrows(IllegalArgumentException.class,
                () -> DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(planes, lines, shortScores,
                        RobustEstimatorMethod.PROSAC));

        // test with listener, quality scores and method
        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                create(listener, shortScores, RobustEstimatorMethod.PROSAC));

        // test with listener, points, quality scores and method
        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, planes, lines,
                qualityScores, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(lines, estimator.getLines());
        assertSame(planes, estimator.getPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, planes, lines,
                qualityScores, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(lines, estimator.getLines());
        assertSame(planes, estimator.getPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, planes, lines,
                qualityScores, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(lines, estimator.getLines());
        assertSame(planes, estimator.getPlanes());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, planes, lines,
                qualityScores, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(lines, estimator.getLines());
        assertSame(planes, estimator.getPlanes());
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, planes, lines,
                qualityScores, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(lines, estimator.getLines());
        assertSame(planes, estimator.getPlanes());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, emptyPlanes, emptyLines,
                        qualityScores, RobustEstimatorMethod.PROSAC));
        assertThrows(IllegalArgumentException.class,
                () -> DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, planes, emptyLines,
                        qualityScores, RobustEstimatorMethod.PROSAC));
        assertThrows(IllegalArgumentException.class,
                () -> DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, planes, lines,
                        shortScores, RobustEstimatorMethod.PROSAC));

        // test with no arguments
        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create();
        assertInstanceOf(PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points
        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(planes, lines);
        assertInstanceOf(PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(lines, estimator.getLines());
        assertSame(planes, estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(emptyPlanes, emptyLines));
        assertThrows(IllegalArgumentException.class,
                () -> DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(planes, emptyLines));

        // test with listener
        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener);
        assertInstanceOf(PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and points
        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, planes, lines);
        assertInstanceOf(PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(lines, estimator.getLines());
        assertSame(planes, estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, emptyPlanes, emptyLines));
        assertThrows(IllegalArgumentException.class,
                () -> DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, planes, emptyLines));

        // test with quality scores
        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(qualityScores);
        assertInstanceOf(PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(shortScores));

        // test with points and quality scores
        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(planes, lines, qualityScores);
        assertInstanceOf(PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(lines, estimator.getLines());
        assertSame(planes, estimator.getPlanes());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(emptyPlanes, emptyLines,
                        qualityScores));
        assertThrows(IllegalArgumentException.class,
                () -> DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(planes, emptyLines, qualityScores));
        assertThrows(IllegalArgumentException.class,
                () -> DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(planes, lines, shortScores));

        // test with listener and scores
        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, qualityScores);
        assertInstanceOf(PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, shortScores));

        // test with listener, points and quality scores
        estimator = DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, planes, lines,
                qualityScores);
        assertInstanceOf(PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(lines, estimator.getLines());
        assertSame(planes, estimator.getPlanes());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(emptyPlanes, emptyLines,
                        qualityScores));
        assertThrows(IllegalArgumentException.class,
                () -> DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(planes, emptyLines, qualityScores));
        assertThrows(IllegalArgumentException.class,
                () -> DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(planes, lines, shortScores));
    }
}
