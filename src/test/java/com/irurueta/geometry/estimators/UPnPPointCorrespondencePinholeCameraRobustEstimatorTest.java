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
import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class UPnPPointCorrespondencePinholeCameraRobustEstimatorTest {

    @Test
    void testCreate() {
        // create with robust estimator method
        var estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // create with robust estimator method and points
        final var points3D = new ArrayList<Point3D>();
        final var points2D = new ArrayList<Point2D>();
        for (var i = 0; i < PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES; i++) {
            points3D.add(Point3D.create());
            points2D.add(Point2D.create());
        }

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(points3D, points2D,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(points3D, points2D,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(points3D, points2D,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(points3D, points2D,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(points3D, points2D,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // Force IllegalArgumentException
        final var empty3D = new ArrayList<Point3D>();
        final var empty2D = new ArrayList<Point2D>();
        // empty points
        assertThrows(IllegalArgumentException.class, () -> UPnPPointCorrespondencePinholeCameraRobustEstimator.
                create(empty3D, empty2D, RobustEstimatorMethod.RANSAC));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> UPnPPointCorrespondencePinholeCameraRobustEstimator.
                create(points3D, empty2D, RobustEstimatorMethod.RANSAC));

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

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // test with listener, points and robust estimator method
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, points3D, points2D,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, points3D, points2D,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, points3D, points2D,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, points3D, points2D,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, points3D, points2D,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // test with quality scores and robust estimator method
        final var qualityScores = new double[
                PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES];
        final var shortScores = new double[1];

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                shortScores, RobustEstimatorMethod.PROSAC));

        // test with points, quality scores and robust estimator method
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(points3D, points2D, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(points3D, points2D, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(points3D, points2D, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(points3D, points2D, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(points3D, points2D, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // Force IllegalArgumentException
        // empty points
        assertThrows(IllegalArgumentException.class, () -> UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                empty3D, empty2D, qualityScores, RobustEstimatorMethod.PROSAC));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, empty2D, qualityScores, RobustEstimatorMethod.PROSAC));
        // short scores
        assertThrows(IllegalArgumentException.class, () -> UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, shortScores, RobustEstimatorMethod.PROSAC));

        // test with listener, quality scores and method
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, shortScores, RobustEstimatorMethod.PROSAC));

        // test with listener, points, quality scores and method
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, points3D, points2D,
                qualityScores, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, points3D, points2D,
                qualityScores, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, points3D, points2D,
                qualityScores, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, points3D, points2D,
                qualityScores, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, points3D, points2D,
                qualityScores, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // Force IllegalArgumentException
        // empty points
        assertThrows(IllegalArgumentException.class, () -> UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, empty3D, empty2D, qualityScores, RobustEstimatorMethod.PROSAC));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, empty2D, qualityScores, RobustEstimatorMethod.PROSAC));
        // short scores
        assertThrows(IllegalArgumentException.class, () -> UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, shortScores, RobustEstimatorMethod.PROSAC));

        // test with no arguments
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create();
        assertInstanceOf(PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // test with points
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(points3D, points2D);
        assertInstanceOf(PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // Force IllegalArgumentException
        // empty points
        assertThrows(IllegalArgumentException.class, () -> UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                empty3D, empty2D));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, empty2D));

        // test with listener
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener);
        assertInstanceOf(PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // test with listener and points
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, points3D, points2D);
        assertInstanceOf(PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // Force IllegalArgumentException
        // empty points
        assertThrows(IllegalArgumentException.class, () -> UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, empty3D, empty2D));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, empty2D));

        // test with quality scores
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(qualityScores);
        assertInstanceOf(PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                shortScores));

        // test with points and quality scores
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(points3D, points2D, qualityScores);
        assertInstanceOf(PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // Force IllegalArgumentException
        // empty points
        assertThrows(IllegalArgumentException.class, () -> UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                empty3D, empty2D, qualityScores));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, empty2D, qualityScores));
        // short scores
        assertThrows(IllegalArgumentException.class, () -> UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, shortScores));

        // test with listener and scores
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, qualityScores);
        assertInstanceOf(PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, shortScores));

        // test with listener, points and quality scores
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, points3D, points2D,
                qualityScores);
        assertInstanceOf(PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.class, estimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // Force IllegalArgumentException
        // empty points
        assertThrows(IllegalArgumentException.class, () -> UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                empty3D, empty2D, qualityScores));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, empty2D, qualityScores));
        // short scores
        assertThrows(IllegalArgumentException.class, () -> UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, shortScores));
    }
}
