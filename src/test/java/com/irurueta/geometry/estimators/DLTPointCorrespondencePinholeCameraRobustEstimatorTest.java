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
import org.junit.*;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class DLTPointCorrespondencePinholeCameraRobustEstimatorTest {

    @Test
    public void testCreate() {
        DLTPointCorrespondencePinholeCameraRobustEstimator estimator;

        // create with robust estimator method
        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);

        // create with robust estimator method and points
        final List<Point3D> points3D = new ArrayList<>();
        final List<Point2D> points2D = new ArrayList<>();
        for (int i = 0; i < PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES; i++) {
            points3D.add(Point3D.create());
            points2D.add(Point2D.create());
        }

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);

        // Force IllegalArgumentException
        final List<Point3D> empty3D = new ArrayList<>();
        final List<Point2D> empty2D = new ArrayList<>();
        estimator = null;
        try {
            // empty points
            estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.
                    create(empty3D, empty2D, RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.
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

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);

        // test with listener, points and robust estimator method
        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);

        // test with quality scores and robust estimator method
        final double[] qualityScores = new double[
                PointCorrespondencePinholeCameraRobustEstimator.
                        MIN_NUMBER_OF_POINT_CORRESPONDENCES];
        final double[] shortScores = new double[1];

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.
                    create(shortScores, RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with points, quality scores and robust estimator method
        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.
                    create(empty3D, empty2D, qualityScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, empty2D, qualityScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // short scores
            estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, points2D, shortScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, quality scores and method
        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, shortScores, RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, points, quality scores and method
        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores,
                RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);

        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores,
                RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, empty3D, empty2D, qualityScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, points3D, empty2D, qualityScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // short scores
            estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, points3D, points2D, shortScores,
                            RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with no arguments
        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create();
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);

        // test with points
        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D);
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.
                    create(empty3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener
        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                listener);
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);

        // test with listener and points
        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D);
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, empty3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, points3D, empty2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with quality scores
        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores);
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.
                    create(shortScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with points and quality scores
        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                points3D, points2D, qualityScores);
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.
                    create(empty3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // short scores
            estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, points2D, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener and scores
        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores);
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.
                    create(listener, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, points and quality scores
        estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(
                listener, points3D, points2D, qualityScores);
        assertTrue(estimator instanceof
                PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.isNormalizeSubsetPointCorrespondences(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);

        // Force IllegalArgumentException
        estimator = null;
        try {
            // empty points
            estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.
                    create(empty3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, empty2D, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // short scores
            estimator = DLTPointCorrespondencePinholeCameraRobustEstimator.
                    create(points3D, points2D, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

    }
}
