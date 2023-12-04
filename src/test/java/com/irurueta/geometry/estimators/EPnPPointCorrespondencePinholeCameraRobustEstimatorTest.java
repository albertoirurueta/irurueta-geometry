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

import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class EPnPPointCorrespondencePinholeCameraRobustEstimatorTest {

    @Test
    public void testCreate() {
        EPnPPointCorrespondencePinholeCameraRobustEstimator estimator;

        // create with robust estimator method
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        // create with robust estimator method and points
        final List<Point3D> points3D = new ArrayList<>();
        final List<Point2D> points2D = new ArrayList<>();
        for (int i = 0; i < PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES; i++) {
            points3D.add(Point3D.create());
            points2D.add(Point2D.create());
        }

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
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
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        // Force IllegalArgumentException
        final List<Point3D> empty3D = new ArrayList<>();
        final List<Point2D> empty2D = new ArrayList<>();
        estimator = null;
        try {
            // empty points
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(empty3D, empty2D, RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
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

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getIntrinsic());

        // test with listener, points and robust estimator method
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
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
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        // test with quality scores and robust estimator method
        final double[] qualityScores = new double[
                PointCorrespondencePinholeCameraRobustEstimator.
                        MIN_NUMBER_OF_POINT_CORRESPONDENCES];
        final double[] shortScores = new double[1];

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
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
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(shortScores, RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with points, quality scores and robust estimator method
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
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
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
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
        assertNull(estimator.getIntrinsic());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(empty3D, empty2D, qualityScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, empty2D, qualityScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // short scores
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, points2D, shortScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, quality scores and method
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, shortScores, RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, points, quality scores and method
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
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
        assertNull(estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
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
        assertNull(estimator.getIntrinsic());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, empty3D, empty2D, qualityScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, points3D, empty2D, qualityScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // short scores
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, points3D, points2D, shortScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // create with robust estimator method and intrinsic
        final PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters();
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        // create with robust estimator method and points
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, points3D, points2D, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, points3D, points2D, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
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
        assertSame(estimator.getIntrinsic(), intrinsic);

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, points3D, points2D, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, points3D, points2D, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, points3D, points2D, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(estimator.getIntrinsic(), intrinsic);

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(intrinsic, empty3D, empty2D,
                            RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(intrinsic, points3D, empty2D,
                            RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // create with listener and robust estimator method
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        // test with listener, points and robust estimator method
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, points3D, points2D,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, points3D, points2D,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, points3D, points2D,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, points3D, points2D,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, points3D, points2D,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(estimator.getIntrinsic(), intrinsic);

        // test with quality scores and robust estimator method
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, qualityScores, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, qualityScores, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(intrinsic, shortScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with points, quality scores and robust estimator method
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, points3D, points2D, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, points3D, points2D, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, points3D, points2D, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, points3D, points2D, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, points3D, points2D, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(intrinsic, empty3D, empty2D, qualityScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(intrinsic, points3D, empty2D, qualityScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // short scores
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(intrinsic, points3D, points2D, shortScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, quality scores and method
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(estimator.getIntrinsic(), intrinsic);

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(estimator.getIntrinsic(), intrinsic);

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, intrinsic, shortScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, points, quality scores and method
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, points3D, points2D, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, points3D, points2D, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, points3D, points2D, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, points3D, points2D, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, points3D, points2D, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(estimator.getIntrinsic(), intrinsic);

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, empty3D, empty2D, qualityScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, points3D, empty2D, qualityScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // short scores
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, points3D, points2D, shortScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test with no arguments
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                create();
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        // test with points
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(empty3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        // test with listener and points
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, empty3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, points3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with quality scores
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(shortScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with points and quality scores
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, qualityScores);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
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
        assertNull(estimator.getIntrinsic());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(empty3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // short scores
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, points2D, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener and scores
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertNull(estimator.getIntrinsic());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, points and quality scores
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
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
        assertNull(estimator.getIntrinsic());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(empty3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // short scores
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, points2D, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test with no arguments and intrinsic
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                create(intrinsic);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        // test with points
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, points3D, points2D);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(intrinsic, empty3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(intrinsic, points3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        // test with listener and points
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, points3D, points2D);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, intrinsic, empty3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, intrinsic, points3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with quality scores
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, qualityScores);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(intrinsic, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with points and quality scores
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                intrinsic, points3D, points2D, qualityScores);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(intrinsic, empty3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(intrinsic, points3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // short scores
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(intrinsic, points3D, points2D, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener and scores
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, qualityScores);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, intrinsic, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, points and quality scores
        estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, intrinsic, points3D, points2D, qualityScores);
        assertTrue(estimator instanceof
                PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator);
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
        assertSame(intrinsic, estimator.getIntrinsic());

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(intrinsic, empty3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(intrinsic, points3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // short scores
            estimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.
                    create(intrinsic, points3D, points2D, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }
}
