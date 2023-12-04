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
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class UPnPPointCorrespondencePinholeCameraRobustEstimatorTest {

    @Test
    public void testCreate() {
        // create with robust estimator method
        UPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                        RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
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

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
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
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
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
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
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
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
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
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // create with robust estimator method and points
        final List<Point3D> points3D = new ArrayList<>();
        final List<Point2D> points2D = new ArrayList<>();
        for (int i = 0; i < PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES; i++) {
            points3D.add(Point3D.create());
            points2D.add(Point2D.create());
        }

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
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

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertFalse(estimator.isReady());
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
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertFalse(estimator.isReady());
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
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // Force IllegalArgumentException
        final List<Point3D> empty3D = new ArrayList<>();
        final List<Point2D> empty2D = new ArrayList<>();
        estimator = null;
        try {
            // empty points
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(empty3D, empty2D, RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, empty2D, RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // create with listener and robust estimator method
        final PinholeCameraRobustEstimatorListener listener =
                new PinholeCameraRobustEstimatorListener() {

                    @Override
                    public void onEstimateStart(final PinholeCameraRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateEnd(final PinholeCameraRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final PinholeCameraRobustEstimator estimator, final int iteration) {
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final PinholeCameraRobustEstimator estimator, final float progress) {
                    }
                };

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
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

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // test with listener, points and robust estimator method
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
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

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // test with quality scores and robust estimator method
        final double[] qualityScores = new double[
                PointCorrespondencePinholeCameraRobustEstimator.
                        MIN_NUMBER_OF_POINT_CORRESPONDENCES];
        final double[] shortScores = new double[1];

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
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

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
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
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
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
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
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
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
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
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(shortScores, RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with points, quality scores and robust estimator method
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
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

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
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
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
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
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(empty3D, empty2D, qualityScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, empty2D, qualityScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // short scores
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, points2D, shortScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, quality scores and method
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
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
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, shortScores, RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, points, quality scores and method
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
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

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, empty3D, empty2D, qualityScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, points3D, empty2D, qualityScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // short scores
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, points3D, points2D, shortScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with no arguments
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                create();
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
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
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // test with points
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D);
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertFalse(estimator.isReady());
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
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(empty3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener);
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // test with listener and points
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D);
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, empty3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, points3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with quality scores
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores);
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
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
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(shortScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with points and quality scores
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, qualityScores);
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
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
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(empty3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // short scores
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, points2D, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener and scores
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores);
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, points and quality scores
        estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores);
        assertTrue(estimator instanceof
                PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(0.0, estimator.getSkewness(), 0.0);
        assertEquals(0.0, estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getVerticalPrincipalPoint(), 0.0);

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(empty3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // short scores
            estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, points2D, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }
}
